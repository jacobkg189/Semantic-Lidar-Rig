import ARKit
import Foundation
import UIKit
import simd

enum DeviceCapabilities {
    static func current() -> WireFormat.Capabilities {
        var caps: WireFormat.Capabilities = [.cameraFrames]
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            caps.insert(.sceneReconstruction)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            caps.insert(.sceneDepth)
        }
        return caps
    }

    /// e.g. "iPhone17,2" — more useful than the marketing name for knowing
    /// exactly which sensor suite produced a recording.
    static func modelIdentifier() -> String {
        var sysinfo = utsname()
        uname(&sysinfo)
        let raw = withUnsafeBytes(of: &sysinfo.machine) { buf in
            buf.compactMap { $0 == 0 ? nil : Character(UnicodeScalar(UInt8($0))) }
        }
        return String(raw)
    }
}

/// Runs the ARKit session and pushes poses (and optionally JPEG frames) at the
/// PhoneServer.
final class ARStreamer: NSObject, ObservableObject {
    private let session = ARSession()
    private let server: PhoneServer

    @Published private(set) var poseCount: Int = 0
    @Published private(set) var frameCount: Int = 0
    @Published private(set) var trackingLabel: String = "starting"

    /// Camera frames are the fat stream — full rate would swamp the tunnel and
    /// stall poses behind it. Poses stay at the session's native 60 Hz.
    var cameraFrameHz: Double = 0      // 0 disables frame streaming
    private var lastFrameSent: TimeInterval = 0

    /// ARKit's LiDAR depth — the dense geometry the C1 cannot supply, since a
    /// planar scanner only sweeps 3D as the rig moves and stays sparse.
    /// ~144 KB per frame, so it is rate-limited like camera frames.
    var sceneDepthHz: Double = 0
    private var lastDepthSent: TimeInterval = 0
    private let depthQueue = DispatchQueue(label: "depth-pack", qos: .utility)
    @Published private(set) var depthCount: Int = 0

    /// ARKit's scene mesh, with per-face classification — the free semantic
    /// layer. Anchors are revised constantly, so each is re-sent at most every
    /// `meshResendSeconds` and only a few per update, which keeps this far
    /// cheaper than the depth stream despite there being many anchors.
    var streamMesh: Bool = false
    private var lastMeshSent: [UUID: TimeInterval] = [:]
    private let meshResendSeconds: TimeInterval = 3.0
    private let meshBudgetPerUpdate = 3
    @Published private(set) var meshCount: Int = 0
    @Published private(set) var meshAnchorCount: Int = 0

    private let jpegQueue = DispatchQueue(label: "jpeg-encode", qos: .utility)
    private let ciContext = CIContext()

    init(server: PhoneServer) {
        self.server = server
        super.init()
        session.delegate = self
    }

    func start() {
        guard ARWorldTrackingConfiguration.isSupported else {
            trackingLabel = "ARKit unsupported on this device"
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            // Not consumed until Phase 5, but enabling it now means recordings
            // made today already carry ARKit's free semantic labels.
            config.sceneReconstruction = .meshWithClassification
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        // A locked screen suspends the session and kills the stream mid-capture.
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func stop() {
        session.pause()
        UIApplication.shared.isIdleTimerDisabled = false
    }
}

extension ARStreamer: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let camera = frame.camera

        let transform = camera.transform
        let position = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let rotation = simd_quatf(simd_float3x3(
            SIMD3<Float>(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z),
            SIMD3<Float>(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z),
            SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
        ))

        // simd stores column-major: intrinsics are [fx 0 cx; 0 fy cy; 0 0 1],
        // so cx/cy live in column 2, not row 2.
        let k = camera.intrinsics
        let intrinsics = (fx: k.columns.0.x, fy: k.columns.1.y, cx: k.columns.2.x, cy: k.columns.2.y)

        let state: UInt8
        switch camera.trackingState {
        case .normal:        state = 2
        case .limited:       state = 1
        case .notAvailable:  state = 0
        }

        server.send(WireFormat.pose(
            timestampUs: UInt64(frame.timestamp * 1_000_000),
            position: position,
            quaternion: rotation,
            intrinsics: intrinsics,
            trackingState: state
        ), droppable: false)

        DispatchQueue.main.async {
            self.poseCount += 1
            self.trackingLabel = Self.describe(camera.trackingState)
        }

        maybeSendCameraFrame(frame)
        maybeSendSceneDepth(frame)
    }

    private func maybeSendSceneDepth(_ frame: ARFrame) {
        guard sceneDepthHz > 0, let depth = frame.sceneDepth else { return }
        guard frame.timestamp - lastDepthSent >= 1.0 / sceneDepthHz else { return }
        lastDepthSent = frame.timestamp

        let timestampUs = UInt64(frame.timestamp * 1_000_000)
        let depthMap = depth.depthMap
        let confMap = depth.confidenceMap
        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)

        // Scale the camera intrinsics from capture resolution to depth
        // resolution. Getting this wrong yields a cloud that looks reasonable
        // but is geometrically incorrect, which is hard to spot later.
        let k = frame.camera.intrinsics
        let imageRes = frame.camera.imageResolution
        let sx = Float(w) / Float(imageRes.width)
        let sy = Float(h) / Float(imageRes.height)
        let intr = (fx: k.columns.0.x * sx, fy: k.columns.1.y * sy,
                    cx: k.columns.2.x * sx, cy: k.columns.2.y * sy)

        depthQueue.async { [weak self] in
            guard let self else { return }
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
            guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return }

            let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
            var mm = [UInt16](repeating: 0, count: w * h)
            let f = base.assumingMemoryBound(to: Float32.self)
            for y in 0..<h {
                for x in 0..<w {
                    let v = f[y * stride + x]
                    // Non-finite and out-of-range become 0 = "no return", which
                    // the Mac treats as missing rather than as a point at origin.
                    mm[y * w + x] = (v.isFinite && v > 0 && v < 65.0)
                        ? UInt16(v * 1000.0) : 0
                }
            }

            var conf = [UInt8](repeating: 0, count: w * h)
            if let c = confMap {
                CVPixelBufferLockBaseAddress(c, .readOnly)
                if let cb = CVPixelBufferGetBaseAddress(c) {
                    let cstride = CVPixelBufferGetBytesPerRow(c)
                    let u = cb.assumingMemoryBound(to: UInt8.self)
                    for y in 0..<h {
                        for x in 0..<w { conf[y * w + x] = u[y * cstride + x] }
                    }
                }
                CVPixelBufferUnlockBaseAddress(c, .readOnly)
            }

            self.server.send(WireFormat.sceneDepth(
                timestampUs: timestampUs,
                width: UInt16(w), height: UInt16(h),
                intrinsics: intr,
                depthMillimetres: mm.withUnsafeBufferPointer { Data(buffer: $0) },
                confidence: conf.withUnsafeBufferPointer { Data(buffer: $0) }
            ), droppable: true)

            DispatchQueue.main.async { self.depthCount += 1 }
        }
    }

    private func maybeSendCameraFrame(_ frame: ARFrame) {
        guard cameraFrameHz > 0 else { return }
        let interval = 1.0 / cameraFrameHz
        guard frame.timestamp - lastFrameSent >= interval else { return }
        lastFrameSent = frame.timestamp

        // capturedImage is only valid for the lifetime of the frame, so retain
        // the frame across the hop onto the encode queue.
        let timestampUs = UInt64(frame.timestamp * 1_000_000)
        let pixelBuffer = frame.capturedImage

        jpegQueue.async { [weak self] in
            guard let self else { return }
            let image = CIImage(cvPixelBuffer: pixelBuffer)
            guard let jpeg = self.ciContext.jpegRepresentation(
                of: image,
                colorSpace: CGColorSpaceCreateDeviceRGB(),
                options: [kCGImageDestinationLossyCompressionQuality as CIImageRepresentationOption: 0.6]
            ) else { return }

            self.server.send(WireFormat.cameraFrame(
                timestampUs: timestampUs,
                width: UInt16(CVPixelBufferGetWidth(pixelBuffer)),
                height: UInt16(CVPixelBufferGetHeight(pixelBuffer)),
                jpeg: jpeg
            ), droppable: true)

            DispatchQueue.main.async { self.frameCount += 1 }
        }
    }

    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) { sendMeshes(anchors) }
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) { sendMeshes(anchors) }

    private func sendMeshes(_ anchors: [ARAnchor]) {
        guard streamMesh else { return }
        let now = CACurrentMediaTime()
        var budget = meshBudgetPerUpdate

        for case let anchor as ARMeshAnchor in anchors {
            guard budget > 0 else { break }
            if let last = lastMeshSent[anchor.identifier], now - last < meshResendSeconds {
                continue
            }
            lastMeshSent[anchor.identifier] = now
            budget -= 1

            let g = anchor.geometry
            guard let cls = g.classification else { continue }

            let verts = Self.rawVectors(g.vertices)
            let faceCount = g.faces.count
            let idxBytes = g.faces.bytesPerIndex * faceCount * g.faces.indexCountPerPrimitive
            let faces = Data(bytes: g.faces.buffer.contents(), count: idxBytes)
            let labels = Data(bytes: cls.buffer.contents().advanced(by: cls.offset),
                              count: cls.count)

            server.send(WireFormat.meshChunk(
                timestampUs: UInt64(now * 1_000_000),
                anchorId: anchor.identifier,
                transform: anchor.transform,
                vertices: verts,
                faces: faces,
                classification: labels
            ), droppable: true)

            DispatchQueue.main.async {
                self.meshCount += 1
                self.meshAnchorCount = self.lastMeshSent.count
            }
        }
    }

    /// Copy a geometry source out of its Metal buffer. The source is strided and
    /// may be interleaved, so it cannot be memcpy'd wholesale.
    private static func rawVectors(_ src: ARGeometrySource) -> Data {
        var out = Data(capacity: src.count * 12)
        let base = src.buffer.contents().advanced(by: src.offset)
        for i in 0..<src.count {
            let p = base.advanced(by: i * src.stride).assumingMemoryBound(to: Float.self)
            var v = (p[0], p[1], p[2])
            withUnsafeBytes(of: &v) { out.append(contentsOf: $0) }
        }
        return out
    }

    private static func describe(_ state: ARCamera.TrackingState) -> String {
        switch state {
        case .normal: return "normal"
        case .notAvailable: return "unavailable"
        case .limited(let reason):
            switch reason {
            case .initializing: return "limited: initializing"
            case .excessiveMotion: return "limited: too fast"
            case .insufficientFeatures: return "limited: featureless scene"
            case .relocalizing: return "limited: relocalizing"
            @unknown default: return "limited"
            }
        }
    }
}
