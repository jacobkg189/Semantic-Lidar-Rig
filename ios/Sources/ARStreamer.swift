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
