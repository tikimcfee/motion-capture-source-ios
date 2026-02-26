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

    init(from ctx: SceneContext) {
        intrinsics = IntrinsicsPayload(
            fx: ctx.intrinsicsFx,
            fy: ctx.intrinsicsFy,
            cx: ctx.intrinsicsCx,
            cy: ctx.intrinsicsCy
        )
        imageResolution = [ctx.imageWidth, ctx.imageHeight]
        cameraTransform = stride(from: 0, to: 16, by: 4).map {
            Array(ctx.cameraTransform[$0 ..< $0 + 4])
        }
        trackingState = ctx.trackingState
        lightIntensity = ctx.lightIntensity
    }
}

struct IntrinsicsPayload: Encodable {
    let fx: Float
    let fy: Float
    let cx: Float
    let cy: Float
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
