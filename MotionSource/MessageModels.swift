//
//  MessageModels.swift
//  MotionSource
//
//  Encodable models matching the WebSocket message protocol.
//  Use MessageEncoder.encode() to serialize.
//

import Foundation
import QuartzCore

// MARK: - Shared Encoder

nonisolated enum MessageEncoder {
    static let encoder: JSONEncoder = {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [] // compact
        return encoder
    }()

    static func encode<T: Encodable>(_ value: T) -> String? {
        guard let data = try? encoder.encode(value) else { return nil }
        return String(data: data, encoding: .utf8)
    }
}

// MARK: - Hand Frame Message

/// hand-wire v2 (see docs/hand-wire-v2.md). v1 fields (`hands[].landmarks`, `scene`) are kept.
nonisolated struct HandFrameMessage: Encodable {
    let type = "handFrame"
    let v = 2
    let timestamp: TimeInterval      // ARFrame capture time, device monotonic clock (s)
    let sentAt: TimeInterval = CACurrentMediaTime()  // same clock, stamped as the frame is built for sending
    let mount: String                // faceDown | faceUp | upright | unknown, from gravity
    let device: DevicePayload?       // nil until CoreMotion has a sample
    let ceiling: Float?              // meters along PHONE +z; nil = unknown
    let hands: [HandObservation]
    let cloud: CloudBlock?           // whole depth map (roi = full), ~1 Hz; absent otherwise
    let scene: ScenePayload

    private enum CodingKeys: String, CodingKey { case type, v, timestamp, sentAt, mount, device, ceiling, hands, cloud, scene }

    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(type, forKey: .type)
        try c.encode(v, forKey: .v)
        try c.encode(timestamp, forKey: .timestamp)
        try c.encode(sentAt, forKey: .sentAt)
        try c.encode(mount, forKey: .mount)
        // Explicit nulls: the contract says "null when unknown", not "absent".
        if let device { try c.encode(device, forKey: .device) } else { try c.encodeNil(forKey: .device) }
        if let ceiling { try c.encode((ceiling * 1000).rounded() / 1000, forKey: .ceiling) }
        else { try c.encodeNil(forKey: .ceiling) }
        try c.encode(hands, forKey: .hands)
        try c.encodeIfPresent(cloud, forKey: .cloud)
        try c.encode(scene, forKey: .scene)
    }
}

nonisolated struct DevicePayload: Encodable {
    let gravity: [Float]    // unit vector, PHONE frame
    let attitude: [Float]   // [qx, qy, qz, qw], PHONE frame vs CoreMotion xArbitraryZVertical
    let userAcceleration: [Float]  // m/s², gravity removed, PHONE frame (latest 100 Hz sample)

    init(_ sample: MotionSample) {
        let r: (Float) -> Float = { ($0 * 10000).rounded() / 10000 }
        gravity = [r(sample.gravity.x), r(sample.gravity.y), r(sample.gravity.z)]
        attitude = [r(sample.attitude.x), r(sample.attitude.y), r(sample.attitude.z), r(sample.attitude.w)]
        userAcceleration = [r(sample.userAcceleration.x), r(sample.userAcceleration.y), r(sample.userAcceleration.z)]
    }
}

nonisolated struct ScenePayload: Encodable {
    let intrinsics: IntrinsicsPayload
    let imageResolution: [Int]
    let cameraTransform: [[Float]]
    let trackingState: String
    let lightIntensity: Float
    let viewport: ViewportPayload

    // High-water-mark depth range — only expands, never shrinks.
    // Converges to the camera's effective hand tracking volume over time.
    nonisolated(unsafe) private static var hwmMinZ: Float = .greatestFiniteMagnitude
    nonisolated(unsafe) private static var hwmMaxZ: Float = 0

    init(from ctx: SceneContext, observations: [HandObservation]) {
        let fx = ctx.intrinsicsFx
        let fy = ctx.intrinsicsFy
        let w = Float(ctx.imageWidth)
        let h = Float(ctx.imageHeight)

        intrinsics = IntrinsicsPayload(fx: fx, fy: fy, cx: ctx.intrinsicsCx, cy: ctx.intrinsicsCy)
        imageResolution = [ctx.imageWidth, ctx.imageHeight]
        cameraTransform = stride(from: 0, to: 16, by: 4).map {
            Array(ctx.cameraTransform[$0 ..< $0 + 4])
        }
        trackingState = ctx.trackingState
        lightIntensity = ctx.lightIntensity

        // Derive viewport from intrinsics
        let fovH = 2.0 * atan(w / (2.0 * fx))
        let fovV = 2.0 * atan(h / (2.0 * fy))

        // Expand high-water-mark from observed hand depths (only grows, never shrinks)
        for obs in observations {
            for lm in obs.landmarks {
                let z = lm[2]
                guard z > 0 else { continue }
                Self.hwmMinZ = min(Self.hwmMinZ, z)
                Self.hwmMaxZ = max(Self.hwmMaxZ, z)
            }
        }
        let hasDepth = Self.hwmMinZ <= Self.hwmMaxZ

        viewport = ViewportPayload(
            fovHorizontal: fovH,
            fovVertical: fovV,
            aspectRatio: w / h,
            physicalWidth: w / fx,   // meters visible at 1m depth
            physicalHeight: h / fy,
            depthRange: hasDepth ? [Self.hwmMinZ, Self.hwmMaxZ] : [0, 0]
        )
    }
}

nonisolated struct IntrinsicsPayload: Encodable {
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
}

nonisolated struct ViewportPayload: Encodable {
    let fovHorizontal: Float   // radians
    let fovVertical: Float     // radians
    let aspectRatio: Float
    let physicalWidth: Float   // meters at 1m depth
    let physicalHeight: Float  // meters at 1m depth
    let depthRange: [Float]    // [minZ, maxZ] high-water-mark of observed depths, meters
}

// MARK: - Camera Frame Message

struct CameraFrameMessage: Encodable {
    let type = "cameraFrame"
    let timestamp: TimeInterval
    let image: String
    let width: Int
    let height: Int
    let orientation: String
}
