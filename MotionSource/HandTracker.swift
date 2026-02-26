//
//  HandTracker.swift
//  MotionSource
//

import Vision
import ARKit

struct HandObservation: Sendable, Encodable {
    let handedness: String    // "left" or "right"
    let landmarks: [[Float]]  // 21 entries, each [x, y, z]
}

struct SceneContext: Sendable {
    let intrinsicsFx: Float
    let intrinsicsFy: Float
    let intrinsicsCx: Float
    let intrinsicsCy: Float
    let imageWidth: Int
    let imageHeight: Int
    let cameraTransform: [Float]  // 16 floats, column-major
    let trackingState: String     // "normal", "limited", "notAvailable"
    let lightIntensity: Float     // lux, 0 if unavailable
}

extension simd_float4x4 {
    var columnMajorArray: [Float] {
        [columns.0.x, columns.0.y, columns.0.z, columns.0.w,
         columns.1.x, columns.1.y, columns.1.z, columns.1.w,
         columns.2.x, columns.2.y, columns.2.z, columns.2.w,
         columns.3.x, columns.3.y, columns.3.z, columns.3.w]
    }
}

// Opt out of default MainActor isolation — this class manages its own threading
nonisolated class HandTracker: @unchecked Sendable {

    // 21 joints ordered by MediaPipe index (matching HandData.js Joint enum)
    static let jointOrder: [VNHumanHandPoseObservation.JointName] = [
        .wrist,                                              // 0  WRIST
        .thumbCMC, .thumbMP, .thumbIP, .thumbTip,           // 1-4  THUMB
        .indexMCP, .indexPIP, .indexDIP, .indexTip,          // 5-8  INDEX
        .middleMCP, .middlePIP, .middleDIP, .middleTip,    // 9-12  MIDDLE
        .ringMCP, .ringPIP, .ringDIP, .ringTip,             // 13-16 RING
        .littleMCP, .littlePIP, .littleDIP, .littleTip      // 17-20 PINKY
    ]

    private let visionQueue = DispatchQueue(label: "com.motionsource.vision", qos: .userInitiated)
    private var isProcessing = false
    private let handPoseRequest: VNDetectHumanHandPoseRequest
    var depthMode: DepthMode

    // Per-joint EMA smoothing — alpha=1 means raw (no smoothing), lower = more smoothing.
    // Tips get heavier smoothing to tame jitter; wrist/MCPs stay responsive.
    private static let jointAlpha: [Float] = [
        0.7,                    // 0  WRIST
        0.7, 0.6, 0.5, 0.35,   // 1-4  THUMB  (CMC → TIP)
        0.7, 0.6, 0.5, 0.35,   // 5-8  INDEX
        0.7, 0.6, 0.5, 0.35,   // 9-12 MIDDLE
        0.7, 0.6, 0.5, 0.35,   // 13-16 RING
        0.7, 0.6, 0.5, 0.35,   // 17-20 PINKY
    ]
    private var previousLandmarks: [String: [[Float]]] = [:]  // keyed by handedness
    private var didLogDepthMapSize = false

    var onHandsDetected: ((@Sendable ([HandObservation], SceneContext, TimeInterval) -> Void))?
    
    enum DepthMode {
        case noDepth
        case smoothed
        case raw
        
        func depthData(for frame: ARFrame) -> ARDepthData? {
            switch self {
            case .smoothed:
                frame.smoothedSceneDepth
            case .raw:
                frame.sceneDepth
            case .noDepth:
                nil
            }
        }
        
        static var isDepthAvailable: Bool {
            ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth)
            || ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        }
    }

    init(
        depthMode: DepthMode
    ) {
        self.depthMode = depthMode
        handPoseRequest = VNDetectHumanHandPoseRequest()
        handPoseRequest.maximumHandCount = 2
    }

    func processFrame(_ frame: ARFrame) {
        guard !isProcessing else { return }
        isProcessing = true

        let pixelBuffer = frame.capturedImage
        let timestamp = frame.timestamp
        let depthData = depthMode.depthData(for: frame)

        // Extract scene context on the calling thread (where ARFrame is valid)
        let camera = frame.camera
        let intrinsics = camera.intrinsics
        let imageRes = camera.imageResolution
        let focalLengthX = intrinsics[0][0]
        let imgWidth = Float(imageRes.width)
        let imgHeight = Float(imageRes.height)
        let trackingState: String = {
            switch camera.trackingState {
            case .normal: return "normal"
            case .limited: return "limited"
            case .notAvailable: return "notAvailable"
            }
        }()
        let sceneContext = SceneContext(
            intrinsicsFx: intrinsics[0][0],
            intrinsicsFy: intrinsics[1][1],
            intrinsicsCx: intrinsics[2][0],
            intrinsicsCy: intrinsics[2][1],
            imageWidth: Int(imageRes.width),
            imageHeight: Int(imageRes.height),
            cameraTransform: camera.transform.columnMajorArray,
            trackingState: trackingState,
            lightIntensity: Float(frame.lightEstimate?.ambientIntensity ?? 0)
        )

        visionQueue.async { [weak self] in
            guard let self = self else { return }
            defer { self.isProcessing = false }

            do {
                let handler = VNImageRequestHandler(
                    cvPixelBuffer: pixelBuffer,
                    orientation: .up,  // Buffer is already upright in landscape-right
                    options: [:]
                )
                try handler.perform([self.handPoseRequest])

                guard let results = self.handPoseRequest.results, !results.isEmpty else { return }

                var observations: [HandObservation] = []

                for observation in results {
                    let handedness = observation.chirality == .left ? "left" : "right"
                    var landmarks: [[Float]] = []

                    for jointName in Self.jointOrder {
                        do {
                            let point = try observation.recognizedPoint(jointName)
                            guard point.confidence > 0.3 else {
                                landmarks.append([0, 0, 0])
                                continue
                            }

                            let x = Float(point.location.x)
                            // Flip Y: Vision origin is bottom-left, MediaPipe expects top-left
                            let y = Float(1.0 - point.location.y)
                            var z: Float = 0.0

                            // Sample LiDAR depth at this joint's position
                            if let depthData = depthData {
                                // Flip Y for depth sampling — Vision origin is bottom-left, depth map is top-left
                                z = self.sampleDepth(depthData, normalizedX: Float(point.location.x), normalizedY: Float(1.0 - point.location.y))
                            }

                            landmarks.append([x, y, z])
                        } catch {
                            landmarks.append([0, 0, 0])
                        }
                    }

                    // Depth coherence clamp — prevent tips from stretching into background.
                    // Use median depth of all valid joints as reference, clamp outliers.
                    let validDepths = landmarks.compactMap { $0[2] > 0 ? $0[2] : nil }
                    if validDepths.count >= 3 {
                        let sorted = validDepths.sorted()
                        let refDepth = sorted[sorted.count / 2]
                        let maxDeviation: Float = 0.10  // 10cm — covers wrist-to-tip range for most poses
                        for i in 0..<landmarks.count where landmarks[i][2] > 0 {
                            landmarks[i][2] = min(max(landmarks[i][2], refDepth - maxDeviation), refDepth + maxDeviation)
                        }
                    }

                    // Temporal EMA smoothing — blend with previous frame per joint
                    if let prev = self.previousLandmarks[handedness], prev.count == landmarks.count {
                        for i in 0..<landmarks.count {
                            let cur = landmarks[i]
                            let old = prev[i]
                            // Skip if either is zeroed (undetected) — snap, don't blend
                            let curDetected = cur[0] != 0 || cur[1] != 0
                            let oldDetected = old[0] != 0 || old[1] != 0
                            guard curDetected && oldDetected else { continue }
                            let a = Self.jointAlpha[i]
                            landmarks[i] = [
                                a * cur[0] + (1 - a) * old[0],
                                a * cur[1] + (1 - a) * old[1],
                                a * cur[2] + (1 - a) * old[2],
                            ]
                        }
                    }
                    self.previousLandmarks[handedness] = landmarks

                    observations.append(HandObservation(handedness: handedness, landmarks: landmarks))
                }

                let callback = self.onHandsDetected
                DispatchQueue.main.async {
                    callback?(observations, sceneContext, timestamp)
                }
            } catch {
                print("[HandTracker] Vision request failed: \(error)")
            }
        }
    }

    /// Sample depth using median of a small kernel around the target point.
    /// Median is robust to edge discontinuities (hand vs background) and
    /// reduces per-pixel LiDAR noise, giving stabler joint depths.
    private static let kernelRadius = 2  // 5x5 kernel

    private func sampleDepth(_ depthData: ARDepthData, normalizedX: Float, normalizedY: Float) -> Float {
        let depthMap = depthData.depthMap
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        if !didLogDepthMapSize {
            didLogDepthMapSize = true
            print("[HandTracker] Depth map size: \(width)x\(height)")
        }
        let r = Self.kernelRadius

        let centerX = min(max(Int(normalizedX * Float(width)), 0), width - 1)
        let centerY = min(max(Int(normalizedY * Float(height)), 0), height - 1)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return 0.0 }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let stride = MemoryLayout<Float32>.size

        var samples: [Float] = []
        let minY = max(centerY - r, 0)
        let maxY = min(centerY + r, height - 1)
        let minX = max(centerX - r, 0)
        let maxX = min(centerX + r, width - 1)

        for py in minY...maxY {
            let rowBase = baseAddress.advanced(by: py * bytesPerRow)
            for px in minX...maxX {
                let d = rowBase.advanced(by: px * stride)
                    .assumingMemoryBound(to: Float32.self).pointee
                if !d.isNaN && !d.isInfinite && d > 0 {
                    samples.append(d)
                }
            }
        }

        guard !samples.isEmpty else { return 0.0 }
        samples.sort()
        // Use near-minimum (2nd smallest) — the closest surface is the hand,
        // not background bleed. Skipping the absolute min avoids single-pixel noise.
        let index = min(1, samples.count - 1)
        return samples[index]
    }
}
