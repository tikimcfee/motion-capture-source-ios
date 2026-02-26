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
    let supportsDepth: Bool

    var onHandsDetected: ((@Sendable ([HandObservation], SceneContext, TimeInterval) -> Void))?

    init() {
        handPoseRequest = VNDetectHumanHandPoseRequest()
        handPoseRequest.maximumHandCount = 2
        supportsDepth = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
    }

    func processFrame(_ frame: ARFrame) {
        guard !isProcessing else { return }
        isProcessing = true

        let pixelBuffer = frame.capturedImage
        let timestamp = frame.timestamp
        let depthData = supportsDepth ? frame.smoothedSceneDepth : nil

        // Extract scene context on the calling thread (where ARFrame is valid)
        let camera = frame.camera
        let intrinsics = camera.intrinsics
        let imageRes = camera.imageResolution
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
                                // Use un-flipped Y for depth sampling (depth map shares camera orientation)
                                z = self.sampleDepth(depthData, normalizedX: Float(point.location.x), normalizedY: Float(point.location.y))
                            }

                            landmarks.append([x, y, z])
                        } catch {
                            landmarks.append([0, 0, 0])
                        }
                    }

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

    private func sampleDepth(_ depthData: ARDepthData, normalizedX: Float, normalizedY: Float) -> Float {
        let depthMap = depthData.depthMap
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)

        let pixelX = min(max(Int(normalizedX * Float(width)), 0), width - 1)
        let pixelY = min(max(Int(normalizedY * Float(height)), 0), height - 1)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return 0.0 }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let pointer = baseAddress.advanced(by: pixelY * bytesPerRow + pixelX * MemoryLayout<Float32>.size)
        let depth = pointer.assumingMemoryBound(to: Float32.self).pointee

        return depth.isNaN || depth.isInfinite ? 0.0 : depth
    }
}
