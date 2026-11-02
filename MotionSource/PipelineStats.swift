//
//  PipelineStats.swift
//  MotionSource
//
//  Rolling pipeline measurements, logged to the console every couple of seconds:
//  camera rate, processed rate, Vision time, capture→send latency, joints dropped for depth,
//  the ceiling's frame-to-frame spread and, with a hand held still, per-joint noise.
//

import Foundation
import QuartzCore
import simd

nonisolated final class PipelineStats: @unchecked Sendable {
    static let shared = PipelineStats()

    private let lock = NSLock()
    private let window: TimeInterval = 2.0
    private var windowStart = CACurrentMediaTime()

    private var arFrames = 0
    private var processedFrames = 0
    private var visionSum = 0.0
    private var sentHandFrames = 0
    private var sentEmptyFrames = 0
    private var latencySum = 0.0
    private var latencyMax = 0.0
    private var bytesSum = 0
    private var seenJoints = 0
    private var nullPointJoints = 0
    private var ceilings: [Float] = []
    private var limitedReason: String?
    private var stillJoints: [Int: [SIMD3<Float>]] = [:]   // joint index → points, hand 0
    private var handFramesInWindow = 0
    /// "vision/geometry" chirality pairs, e.g. "right/right". Geometry assumes palm-down over the
    /// phone: n = (p5 − p0) × (p17 − p0) in the PHONE frame; right hand ⇔ n.z < 0.
    private var chirality: [String: Int] = [:]

    /// Joints whose stillness noise is reported: wrist, index MCP, index tip, thumb tip.
    private static let noiseJoints = [0, 5, 8, 4]

    private var deliverySum = 0.0      // capture → frame handed to the app
    private var deliveryCount = 0
    private var visionQueueSum = 0.0   // capture → Vision starts on it
    private var palmMissing = 0
    private var handsSeen = 0
    private var tipSeen = 0, tipNull = 0, otherSeen = 0, otherNull = 0

    func cameraFrame(timestamp: TimeInterval) {
        let delay = CACurrentMediaTime() - timestamp
        lock.lock(); arFrames += 1; deliverySum += delay; deliveryCount += 1; lock.unlock()
    }

    private var depthAgeSum = 0.0, depthAgeCount = 0

    /// AVFoundation only: how old the depth map paired with a video frame was.
    func depthAge(_ age: TimeInterval) {
        lock.lock(); depthAgeSum += abs(age); depthAgeCount += 1; lock.unlock()
    }

    private var wireOK = 0, wireFailed = 0, wireLatencySum = 0.0

    func sendCompleted(captureTimestamp: TimeInterval, ok: Bool) {
        let latency = CACurrentMediaTime() - captureTimestamp
        lock.lock()
        if ok { wireOK += 1; wireLatencySum += latency } else { wireFailed += 1 }
        lock.unlock()
    }

    private var cloudCount = 0, cloudBytes = 0, cloudCells = 0, cloudNear = 0, cloudBuild = 0.0

    func cloudBuilt(cells: Int, bytes: Int, seconds: Double) {
        lock.lock()
        cloudCount += 1; cloudBytes += bytes; cloudCells += cells; cloudBuild += seconds
        lock.unlock()
    }

    private var accelMax: Float = 0

    /// |userAcceleration| in m/s², from every CoreMotion sample (peak per window is logged).
    func userAcceleration(_ magnitude: Float) {
        lock.lock(); accelMax = max(accelMax, magnitude); lock.unlock()
    }

    private var edgeHoles = 0, edgeHand = 0, edgeFlying = 0, edgeTotal = 0

    /// Per hand cloud: pixel counts for holes / hand / flying (between hand and 1.2 m) / total.
    func cloudEdges(holes: Int, hand: Int, flying: Int, total: Int) {
        lock.lock()
        edgeHoles += holes; edgeHand += hand; edgeFlying += flying; edgeTotal += total
        lock.unlock()
    }

    func visionStart(captureTimestamp: TimeInterval) {
        let delay = CACurrentMediaTime() - captureTimestamp
        lock.lock(); visionQueueSum += delay; lock.unlock()
    }

    func depthCoverage(hasPalm: Bool, tips: (seen: Int, null: Int), others: (seen: Int, null: Int)) {
        lock.lock(); defer { lock.unlock() }
        handsSeen += 1
        if !hasPalm { palmMissing += 1 }
        tipSeen += tips.seen; tipNull += tips.null
        otherSeen += others.seen; otherNull += others.null
    }

    func trackingLimited(reason: String) {
        lock.lock(); limitedReason = reason; lock.unlock()
    }

    func processed(visionSeconds: Double, hands: [HandObservation], ceiling: Float?) {
        lock.lock(); defer { lock.unlock() }
        processedFrames += 1
        visionSum += visionSeconds
        if let ceiling { ceilings.append(ceiling) }
        for hand in hands {
            for i in 0..<21 where hand.confidence[i] > HandTracker.minJointConfidence {
                seenJoints += 1
                if hand.points[i] == nil { nullPointJoints += 1 }
            }
        }
        for hand in hands {
            var geo = "?"
            if let p0 = hand.points[0], let p5 = hand.points[5], let p17 = hand.points[17] {
                geo = simd_cross(p5 - p0, p17 - p0).z < 0 ? "right" : "left"
            }
            chirality["\(hand.handedness)/\(geo)", default: 0] += 1
        }
        if let hand = hands.first {
            handFramesInWindow += 1
            for j in Self.noiseJoints {
                if let p = hand.points[j] { stillJoints[j, default: []].append(p) }
            }
        }
    }

    /// Called just after a handFrame is handed to the socket.
    private var encodeSum = 0.0, encodeCount = 0

    func encoded(seconds: Double) {
        lock.lock(); encodeSum += seconds; encodeCount += 1; lock.unlock()
    }

    func sent(captureTimestamp: TimeInterval, bytes: Int, hasHands: Bool) {
        let latency = CACurrentMediaTime() - captureTimestamp   // same systemUptime clock
        lock.lock()
        if hasHands { sentHandFrames += 1 } else { sentEmptyFrames += 1 }
        latencySum += latency
        latencyMax = max(latencyMax, latency)
        bytesSum += bytes
        lock.unlock()
        reportIfDue()
    }

    private func reportIfDue() {
        lock.lock(); defer { lock.unlock() }
        let now = CACurrentMediaTime()
        let dt = now - windowStart
        guard dt >= window else { return }

        let sent = sentHandFrames + sentEmptyFrames
        var line = String(format: "[Stats] cam %.0f fps | processed %.0f fps (vision %.1f ms) | sent %.0f fps (%d hand, %d empty, %.1f KB/s) | capture→send avg %.1f ms max %.1f ms",
                          Double(arFrames) / dt,
                          Double(processedFrames) / dt,
                          processedFrames > 0 ? visionSum / Double(processedFrames) * 1000 : 0,
                          Double(sent) / dt, sentHandFrames, sentEmptyFrames,
                          Double(bytesSum) / dt / 1024,
                          sent > 0 ? latencySum / Double(sent) * 1000 : 0,
                          latencyMax * 1000)
        line += String(format: " | wire %.0f fps ok, %d failed, capture→wire %.1f ms",
                       Double(wireOK) / dt, wireFailed, wireOK > 0 ? wireLatencySum / Double(wireOK) * 1000 : 0)
        if cloudCount > 0 {
            line += String(format: " | cloud %.1f/s, %.1f KB b64 each (%.0f KB/s), %d px each, build %.2f ms",
                           Double(cloudCount) / dt, Double(cloudBytes) / Double(cloudCount) / 1024,
                           Double(cloudBytes) / dt / 1024, cloudCells / cloudCount,
                           cloudBuild / Double(cloudCount) * 1000)
        }
        if edgeTotal > 0, edgeHand > 0 {
            line += String(format: " | cloud px: holes %.1f%%, hand %.1f%%, flying %.1f%% (flying/hand %.3f)",
                           Double(edgeHoles) / Double(edgeTotal) * 100, Double(edgeHand) / Double(edgeTotal) * 100,
                           Double(edgeFlying) / Double(edgeTotal) * 100, Double(edgeFlying) / Double(edgeHand))
        }
        if encodeCount > 0 {
            line += String(format: " | encode %.2f ms", encodeSum / Double(encodeCount) * 1000)
        }
        if deliveryCount > 0 {
            line += String(format: " | delivery %.1f ms, to-vision %.1f ms",
                           deliverySum / Double(deliveryCount) * 1000,
                           processedFrames > 0 ? visionQueueSum / Double(processedFrames) * 1000 : 0)
        }
        if depthAgeCount > 0 {
            line += String(format: " | depth age %.1f ms (%d/%d frames)", depthAgeSum / Double(depthAgeCount) * 1000,
                           depthAgeCount, arFrames)
        }
        if seenJoints > 0 {
            line += String(format: " | depth-null %.1f%% (tips %.0f%%, others %.0f%%, no-palm %d/%d)",
                           Double(nullPointJoints) / Double(seenJoints) * 100,
                           tipSeen > 0 ? Double(tipNull) / Double(tipSeen) * 100 : 0,
                           otherSeen > 0 ? Double(otherNull) / Double(otherSeen) * 100 : 0,
                           palmMissing, handsSeen)
        }
        if ceilings.count > 1 {
            let (mean, sd) = Self.meanStd(ceilings)
            line += String(format: " | ceiling %.3f m ±%.1f mm", mean, sd * 1000)
        }
        line += String(format: " | userAccel peak %.2f m/s²", accelMax)
        if let limitedReason { line += " | tracking limited(\(limitedReason))" }
        if !chirality.isEmpty {
            line += " | chirality vision/geom " + chirality.sorted { $0.key < $1.key }
                .map { "\($0.key)=\($0.value)" }.joined(separator: " ")
        }
        print(line)

        // Stillness noise: only meaningful when one hand was present for the whole window.
        if handFramesInWindow >= processedFrames * 9 / 10, handFramesInWindow >= 20 {
            var parts: [String] = []
            for j in Self.noiseJoints {
                guard let pts = stillJoints[j], pts.count >= 10 else { continue }
                let sd = SIMD3<Float>(Self.meanStd(pts.map(\.x)).1,
                                      Self.meanStd(pts.map(\.y)).1,
                                      Self.meanStd(pts.map(\.z)).1) * 1000
                let mean = pts.reduce(SIMD3<Float>.zero, +) / Float(pts.count)
                parts.append(String(format: "j%d sd(x,y,z)=(%.1f,%.1f,%.1f)mm @(%.3f,%.3f,%.3f) n=%d",
                                    j, sd.x, sd.y, sd.z, mean.x, mean.y, mean.z, pts.count))
            }
            if !parts.isEmpty { print("[Stats] hand noise: " + parts.joined(separator: " | ")) }
        }

        windowStart = now
        wireOK = 0; wireFailed = 0; wireLatencySum = 0; accelMax = 0
        edgeHoles = 0; edgeHand = 0; edgeFlying = 0; edgeTotal = 0; encodeSum = 0; encodeCount = 0
        cloudCount = 0; cloudBytes = 0; cloudCells = 0; cloudNear = 0; cloudBuild = 0
        deliverySum = 0; deliveryCount = 0; visionQueueSum = 0; depthAgeSum = 0; depthAgeCount = 0
        palmMissing = 0; handsSeen = 0; tipSeen = 0; tipNull = 0; otherSeen = 0; otherNull = 0
        arFrames = 0; processedFrames = 0; visionSum = 0
        sentHandFrames = 0; sentEmptyFrames = 0; latencySum = 0; latencyMax = 0; bytesSum = 0
        seenJoints = 0; nullPointJoints = 0; ceilings.removeAll(keepingCapacity: true)
        limitedReason = nil; chirality.removeAll(); stillJoints.removeAll(keepingCapacity: true); handFramesInWindow = 0
    }

    private static func meanStd(_ v: [Float]) -> (Float, Float) {
        guard !v.isEmpty else { return (0, 0) }
        let mean = v.reduce(0, +) / Float(v.count)
        let variance = v.reduce(0) { $0 + ($1 - mean) * ($1 - mean) } / Float(v.count)
        return (mean, variance.squareRoot())
    }
}
