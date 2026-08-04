import Foundation
import simd

/// Mirrors mac/protocol.py — change both together. Spec: docs/WIRE_FORMAT.md
enum WireFormat {
    static let protocolVersion: UInt16 = 1

    enum MsgType: UInt8 {
        case hello = 0x01
        case pose = 0x02
        case cameraFrame = 0x03
        case sceneDepth = 0x04
    }

    struct Capabilities: OptionSet {
        let rawValue: UInt32
        static let sceneReconstruction = Capabilities(rawValue: 1 << 0)
        static let sceneDepth          = Capabilities(rawValue: 1 << 1)
        static let cameraFrames        = Capabilities(rawValue: 1 << 2)
    }

    /// Length-prefixed frame: [length u32][type u8][payload]. Length excludes
    /// the 5-byte header.
    static func frame(_ type: MsgType, _ payload: Data) -> Data {
        var out = Data(capacity: 5 + payload.count)
        out.appendLE(UInt32(payload.count))
        out.append(type.rawValue)
        out.append(payload)
        return out
    }

    static func hello(deviceName: String, osVersion: String, capabilities: Capabilities) -> Data {
        var p = Data()
        p.appendLE(protocolVersion)
        p.appendPString(deviceName)
        p.appendPString(osVersion)
        p.appendLE(capabilities.rawValue)
        return frame(.hello, p)
    }

    static func pose(
        timestampUs: UInt64,
        position: SIMD3<Float>,
        quaternion: simd_quatf,
        intrinsics: (fx: Float, fy: Float, cx: Float, cy: Float),
        trackingState: UInt8
    ) -> Data {
        var p = Data(capacity: 53)
        p.appendLE(timestampUs)
        p.appendLE(position.x); p.appendLE(position.y); p.appendLE(position.z)
        // Quaternion order is x,y,z,w to match the Python side. simd's `vector`
        // is already (x,y,z,w) but spell it out — silently transposing w is the
        // kind of bug that shows up as a plausible-looking wrong rotation.
        p.appendLE(quaternion.vector.x)
        p.appendLE(quaternion.vector.y)
        p.appendLE(quaternion.vector.z)
        p.appendLE(quaternion.vector.w)
        p.appendLE(intrinsics.fx); p.appendLE(intrinsics.fy)
        p.appendLE(intrinsics.cx); p.appendLE(intrinsics.cy)
        p.append(trackingState)
        return frame(.pose, p)
    }

    static func cameraFrame(timestampUs: UInt64, width: UInt16, height: UInt16, jpeg: Data) -> Data {
        var p = Data(capacity: 16 + jpeg.count)
        p.appendLE(timestampUs)
        p.appendLE(width)
        p.appendLE(height)
        p.appendLE(UInt32(jpeg.count))
        p.append(jpeg)
        return frame(.cameraFrame, p)
    }
}

private extension Data {
    /// Little-endian on both ends — native for ARM64, so no byte swapping.
    mutating func appendLE<T: FixedWidthInteger>(_ value: T) {
        var le = value.littleEndian
        Swift.withUnsafeBytes(of: &le) { append(contentsOf: $0) }
    }

    mutating func appendLE(_ value: Float) {
        appendLE(value.bitPattern)
    }

    mutating func appendPString(_ s: String) {
        let bytes = Data(s.utf8)
        appendLE(UInt16(bytes.count))
        append(bytes)
    }
}
