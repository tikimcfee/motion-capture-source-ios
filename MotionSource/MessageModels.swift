//
//  MessageModels.swift
//  MotionSource
//
//  Encodable models matching the WebSocket message protocol.
//  Use MessageEncoder.encode() to serialize.
//

import Foundation

// MARK: - Shared Encoder

enum MessageEncoder {
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

struct HandFrameMessage: Encodable {
    let type = "handFrame"
    let hands: [HandObservation]
    let timestamp: TimeInterval
    let scene: ScenePayload
}

struct ScenePayload: Encodable {
    let intrinsics: IntrinsicsPayload
    let imageResolution: [Int]
    let cameraTransform: [[Float]]
    let trackingState: String
    let lightIntensity: Float
    let viewport: ViewportPayload

    // High-water-mark depth range — only expands, never shrinks.
    // Converges to the camera's effective hand tracking volume over time.
    private static var hwmMinZ: Float = .greatestFiniteMagnitude
    private static var hwmMaxZ: Float = 0

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

struct IntrinsicsPayload: Encodable {
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
}

struct ViewportPayload: Encodable {
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
