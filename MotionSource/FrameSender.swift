//
//  FrameSender.swift
//  MotionSource
//
//  Builds and sends handFrame v2 messages on the hand-tracking queue. Sending straight from
//  there, rather than hopping to the main thread (busy with the Metal renderer and UI), keeps
//  the main run loop out of the capture → send path.
//

import Foundation
import QuartzCore

nonisolated final class FrameSender: @unchecked Sendable {
    /// Empty-hand frames keep the consumer informed (phone present, mount, ceiling) without
    /// spending full frame rate on them: at most this often.
    static let emptyFrameInterval: TimeInterval = 0.1   // 10 Hz

    private let client: WebSocketClient
    private let motion: MotionSensor
    private var lastEmptyFrameTime: TimeInterval = 0   // touched only on the send queue

    init(client: WebSocketClient, motion: MotionSensor) {
        self.client = client
        self.motion = motion
    }

    /// Call on one serial queue (frames arrive in order). Returns whether a frame was sent.
    @discardableResult
    func send(_ tracked: TrackedFrame) -> Bool {
        let hands = tracked.hands
        // An empty frame carrying the ~1 Hz scene cloud is never throttled away.
        if hands.isEmpty && tracked.sceneCloud == nil {
            guard tracked.timestamp - lastEmptyFrameTime >= Self.emptyFrameInterval else { return false }
            lastEmptyFrameTime = tracked.timestamp
        }

        let encodeStart = CACurrentMediaTime()
        let sample = motion.latest
        let message = HandFrameMessage(
            timestamp: tracked.timestamp,
            mount: (sample?.mount ?? .unknown).rawValue,
            device: sample.map(DevicePayload.init),
            ceiling: tracked.ceiling,
            hands: hands,
            cloud: tracked.sceneCloud,
            scene: ScenePayload(from: tracked.scene, observations: hands)
        )
        guard let json = MessageEncoder.encode(message) else { return false }
        PipelineStats.shared.encoded(seconds: CACurrentMediaTime() - encodeStart)
        client.send(json, captureTimestamp: tracked.timestamp)
        PipelineStats.shared.sent(captureTimestamp: tracked.timestamp,
                                  bytes: json.utf8.count, hasHands: !hands.isEmpty)
        return true
    }
}
