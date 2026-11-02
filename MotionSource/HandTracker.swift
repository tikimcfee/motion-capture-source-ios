//
//  HandTracker.swift
//  MotionSource
//

import Vision
import ARKit

nonisolated struct HandObservation: Sendable, Encodable {
    let handedness: String          // "left" or "right", as Vision reports it
    let confidence: [Float]         // 21, Vision's per-joint confidence (0 when not recognized)
    let points: [SIMD3<Float>?]     // 21, meters in the PHONE frame; nil = not seen / no depth
    let rays: [SIMD3<Float>?]       // 21, unit direction optical center → joint, PHONE frame;
                                    // depth-independent, nil only when the joint isn't detected
    let landmarks: [[Float]]        // v1: 21 × [u, v, depth] — normalized top-left image coords
    var cloud: CloudBlock? = nil    // raw depth crop around this hand, only on frames with NEW depth

    private enum CodingKeys: String, CodingKey { case handedness, confidence, points, rays, landmarks, cloud }

    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(handedness, forKey: .handedness)
        try c.encode(confidence.map(HandObservation.round3), forKey: .confidence)
        // [[x,y,z] | null] — nil must encode as an explicit null so indices stay joint-aligned.
        try c.encode(points.map { p in p.map { [Self.round4($0.x), Self.round4($0.y), Self.round4($0.z)] } },
                     forKey: .points)
        try c.encode(rays.map { r in r.map { [Self.round4($0.x), Self.round4($0.y), Self.round4($0.z)] } },
                     forKey: .rays)
        try c.encode(landmarks, forKey: .landmarks)
        try c.encodeIfPresent(cloud, forKey: .cloud)
    }

    // Trim JSON: 0.1 mm and 0.001 confidence are far below the sensor's noise.
    private static func round4(_ v: Float) -> Float { (v * 10000).rounded() / 10000 }
    private static func round3(_ v: Float) -> Float { (v * 1000).rounded() / 1000 }
}

nonisolated struct SceneContext: Sendable {
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

/// One processed camera frame: everything the v2 handFrame needs.
nonisolated struct TrackedFrame: Sendable {
    let hands: [HandObservation]
    let scene: SceneContext
    let timestamp: TimeInterval   // ARFrame capture time (systemUptime clock)
    let ceiling: Float?           // meters along PHONE +z, median depth outside the hands
    var sceneCloud: CloudBlock? = nil  // the whole depth map, ~1 Hz
}

nonisolated extension simd_float4x4 {
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

    /// Below this Vision confidence a joint is reported as unseen (null point, zero landmark).
    static let minJointConfidence: Float = 0.3

    /// "Hand band": depth samples further than this from the palm's depth are not the hand
    /// (ceiling, or flying pixels on a silhouette edge) and are ignored. 12 cm covers a pitched
    /// hand's wrist-to-fingertip spread; the ceiling is ≥1 m further away.
    static let handBand: Float = 0.12

    /// Interior joints whose depth defines the palm: wrist and the four finger MCPs.
    static let palmJoints = [0, 5, 9, 13, 17]
    static let tipJoints: Set<Int> = [4, 8, 12, 16, 20]

    private let visionQueue = DispatchQueue(label: "com.motionsource.vision", qos: .userInitiated)
    private let processingLock = NSLock()
    private var isProcessing = false
    private let handPoseRequest: VNDetectHumanHandPoseRequest
    var depthMode: DepthMode
    private var didLogDepthMapSize = false
    /// The whole depth map (ceiling + scene) rides along at most this often.
    static let sceneCloudInterval: TimeInterval = 1.0
    private var lastSceneCloud: TimeInterval = 0   // tracking queue only

    var onFrameTracked: ((@Sendable (TrackedFrame) -> Void))?

    enum DepthMode: String {
        case noDepth
        case smoothed
        case raw

        func depthData(for frame: ARFrame) -> ARDepthData? {
            switch self {
            case .smoothed: frame.smoothedSceneDepth
            case .raw: frame.sceneDepth
            case .noDepth: nil
            }
        }

        var frameSemantic: ARConfiguration.FrameSemantics? {
            switch self {
            case .smoothed: .smoothedSceneDepth
            case .raw: .sceneDepth
            case .noDepth: nil
            }
        }

        /// Raw sceneDepth by default: the consumer filters for itself and wants near-raw motion,
        /// and smoothedSceneDepth blends depth across frames (lag + smear on a moving hand).
        /// Override with the launch argument `-depthMode smoothed|raw|noDepth`.
        static var preferred: DepthMode {
            let requested = UserDefaults.standard.string(forKey: "depthMode").flatMap(DepthMode.init(rawValue:))
            let wanted = requested ?? .raw
            guard let semantic = wanted.frameSemantic else { return .noDepth }
            if ARWorldTrackingConfiguration.supportsFrameSemantics(semantic) { return wanted }
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { return .raw }
            return .noDepth
        }
    }

    init(depthMode: DepthMode) {
        self.depthMode = depthMode
        handPoseRequest = VNDetectHumanHandPoseRequest()
        handPoseRequest.maximumHandCount = 2
    }

    /// ARKit entry point: unpack the frame on the calling thread (where ARFrame is valid).
    func processFrame(_ frame: ARFrame) {
        PipelineStats.shared.cameraFrame(timestamp: frame.timestamp)
        guard beginProcessing() else { return }

        let camera = frame.camera
        let intrinsics = camera.intrinsics
        let imageRes = camera.imageResolution
        let trackingState: String
        switch camera.trackingState {
        case .normal: trackingState = "normal"
        case .limited(let reason):
            trackingState = "limited"
            PipelineStats.shared.trackingLimited(reason: "\(reason)")
        case .notAvailable: trackingState = "notAvailable"
        }
        let scene = SceneContext(
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
        let depth = depthMode.depthData(for: frame).map { DepthSampler($0, filtered: depthMode == .smoothed) }
        track(pixelBuffer: frame.capturedImage, depth: depth, scene: scene, timestamp: frame.timestamp)
    }

    /// Drop the frame if the previous one is still in Vision (keeps latency from piling up).
    func beginProcessing() -> Bool {
        processingLock.lock(); defer { processingLock.unlock() }
        if isProcessing { return false }
        isProcessing = true
        return true
    }

    /// Capture-agnostic entry point (ARKit or AVFoundation). Call only after
    /// `beginProcessing()` returned true. `pixelBuffer` must be the native landscape-right
    /// camera buffer the intrinsics in `scene` describe.
    /// `depthTimestamp` is the depth map's own capture time; `depthIsNew` is false when the depth
    /// map was already used for an earlier frame (AVFoundation depth is 30 Hz under 60 Hz video),
    /// and then no cloud is attached.
    func track(pixelBuffer: CVPixelBuffer, depth: DepthSampler?, scene: SceneContext, timestamp: TimeInterval,
               depthTimestamp: TimeInterval? = nil, depthIsNew: Bool = true) {
        nonisolated(unsafe) let pixelBuffer = pixelBuffer
        let depthLabel = depth.map { "\($0.width)x\($0.height)" } ?? "none"
        visionQueue.async { [weak self] in
            guard let self = self else { return }
            defer {
                self.processingLock.lock()
                self.isProcessing = false
                self.processingLock.unlock()
            }
            if !self.didLogDepthMapSize {
                self.didLogDepthMapSize = true
                print("[HandTracker] depth \(depthLabel), image \(scene.imageWidth)x\(scene.imageHeight), fx \(scene.intrinsicsFx) fy \(scene.intrinsicsFy) cx \(scene.intrinsicsCx) cy \(scene.intrinsicsCy)")
            }
            PipelineStats.shared.visionStart(captureTimestamp: timestamp)

            let visionStart = CACurrentMediaTime()
            let results: [VNHumanHandPoseObservation]
            do {
                let handler = VNImageRequestHandler(
                    cvPixelBuffer: pixelBuffer,
                    orientation: .up,  // Normalized coords stay in the native landscape-right buffer
                    options: [:]
                )
                try handler.perform([self.handPoseRequest])
                results = self.handPoseRequest.results ?? []
            } catch {
                print("[HandTracker] Vision request failed: \(error)")
                return
            }

            var hands = results.map { self.makeObservation($0, scene: scene, depth: depth) }
            var sceneCloud: CloudBlock?
            if let depth, depthIsNew {
                let t = depthTimestamp ?? timestamp
                for i in hands.indices {
                    if let roi = depth.handROI(landmarks: hands[i].landmarks) {
                        hands[i].cloud = depth.cloud(roi: roi, scene: scene, depthTimestamp: t)
                    }
                }
                if t - self.lastSceneCloud >= Self.sceneCloudInterval {
                    self.lastSceneCloud = t
                    sceneCloud = depth.cloud(roi: (0, 0, depth.width, depth.height), scene: scene, depthTimestamp: t)
                }
            }
            let ceiling = depth?.ceiling(excluding: hands)
            PipelineStats.shared.processed(visionSeconds: CACurrentMediaTime() - visionStart,
                                           hands: hands, ceiling: ceiling)

            var tracked = TrackedFrame(hands: hands, scene: scene, timestamp: timestamp, ceiling: ceiling)
            tracked.sceneCloud = sceneCloud
            self.onFrameTracked?(tracked)
        }
    }

    private func makeObservation(_ observation: VNHumanHandPoseObservation,
                                 scene: SceneContext, depth: DepthSampler?) -> HandObservation {
        let handedness = observation.chirality == .left ? "left" : "right"
        var confidence = [Float](repeating: 0, count: 21)
        var uv = [SIMD2<Float>?](repeating: nil, count: 21)   // normalized, top-left origin
        var depths = [Float?](repeating: nil, count: 21)

        for (i, jointName) in Self.jointOrder.enumerated() {
            guard let point = try? observation.recognizedPoint(jointName) else { continue }
            confidence[i] = point.confidence
            guard point.confidence > Self.minJointConfidence else { continue }
            // Vision's origin is bottom-left; image and depth buffers are top-left.
            uv[i] = SIMD2<Float>(Float(point.location.x), Float(1.0 - point.location.y))
        }

        let palm = depth?.palmDepth(at: Self.palmJoints.compactMap { uv[$0] })
        if let depth, let palm {
            for i in 0..<21 {
                guard let p = uv[i] else { continue }
                depths[i] = depth.jointDepth(normalizedX: p.x, normalizedY: p.y, palm: palm,
                                             band: Self.handBand,
                                             radius: Self.tipJoints.contains(i) ? 3 : 2)
            }
        }
        PipelineStats.shared.depthCoverage(hasPalm: palm != nil,
                                           tips: (seen: Self.tipJoints.filter { uv[$0] != nil }.count,
                                                  null: Self.tipJoints.filter { uv[$0] != nil && depths[$0] == nil }.count),
                                           others: (seen: (0..<21).filter { !Self.tipJoints.contains($0) && uv[$0] != nil }.count,
                                                    null: (0..<21).filter { !Self.tipJoints.contains($0) && uv[$0] != nil && depths[$0] == nil }.count))

        let w = Float(scene.imageWidth), h = Float(scene.imageHeight)
        var points = [SIMD3<Float>?](repeating: nil, count: 21)
        var rays = [SIMD3<Float>?](repeating: nil, count: 21)
        var landmarks = [[Float]](repeating: [0, 0, 0], count: 21)
        for i in 0..<21 {
            guard let p = uv[i] else { continue }
            landmarks[i] = [p.x, p.y, depths[i] ?? 0]
            rays[i] = simd_normalize(PhoneFrame.unproject(u: p.x * w, v: p.y * h, depth: 1,
                                                          fx: scene.intrinsicsFx, fy: scene.intrinsicsFy,
                                                          cx: scene.intrinsicsCx, cy: scene.intrinsicsCy))
            guard let d = depths[i] else { continue }
            // Intrinsics are in captured-image pixels, so unproject at image resolution; the
            // depth map was sampled at the same normalized position (same 4:3 field of view).
            points[i] = PhoneFrame.unproject(u: p.x * w, v: p.y * h, depth: d,
                                             fx: scene.intrinsicsFx, fy: scene.intrinsicsFy,
                                             cx: scene.intrinsicsCx, cy: scene.intrinsicsCy)
        }
        return HandObservation(handedness: handedness, confidence: confidence,
                               points: points, rays: rays, landmarks: landmarks)
    }
}

/// Reads an ARKit depth map (+ confidence map) at normalized image coordinates.
/// ARKit z-depth is meters along the camera's optical axis, which is PHONE +z.
nonisolated struct DepthSampler: @unchecked Sendable {
    let width: Int
    let height: Int
    private let depthMap: CVPixelBuffer
    private let confidenceMap: CVPixelBuffer?

    /// Lowest ARConfidenceLevel accepted (0 low, 1 medium, 2 high).
    static let minConfidence: UInt8 = 1

    /// Whether the depth source already smooths/fills depth (AVFoundation isFilteringEnabled,
    /// ARKit smoothedSceneDepth). Reported in every cloud block.
    let filtered: Bool

    init(_ data: ARDepthData, filtered: Bool) {
        self.init(depthMap: data.depthMap, confidenceMap: data.confidenceMap, filtered: filtered)
    }

    /// `depthMap` must be kCVPixelFormatType_DepthFloat32 (meters); `confidenceMap`, if given,
    /// ARConfidenceLevel bytes of the same size (AVFoundation depth has none).
    init(depthMap: CVPixelBuffer, confidenceMap: CVPixelBuffer?, filtered: Bool) {
        self.filtered = filtered
        self.depthMap = depthMap
        self.confidenceMap = confidenceMap
        width = CVPixelBufferGetWidth(depthMap)
        height = CVPixelBufferGetHeight(depthMap)
    }

    /// ROI around a hand: its detected keypoints' bounding box in depth pixels, padded by 25 % of
    /// the box's larger side, clamped to the map. Nil if no keypoint was detected.
    func handROI(landmarks: [[Float]]) -> (x: Int, y: Int, w: Int, h: Int)? {
        let seen = landmarks.filter { $0[0] != 0 || $0[1] != 0 }
        guard !seen.isEmpty else { return nil }
        let xs = seen.map { $0[0] * Float(width) }, ys = seen.map { $0[1] * Float(height) }
        let pad = 0.25 * max(xs.max()! - xs.min()!, ys.max()! - ys.min()!, 8)
        let x0 = max(Int((xs.min()! - pad).rounded(.down)), 0)
        let y0 = max(Int((ys.min()! - pad).rounded(.down)), 0)
        let x1 = min(Int((xs.max()! + pad).rounded(.up)), width - 1)
        let y1 = min(Int((ys.max()! + pad).rounded(.up)), height - 1)
        guard x1 >= x0, y1 >= y0 else { return nil }
        return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)
    }

    /// Raw depth (no filtering, no segmentation) inside `roi`, as a CloudBlock.
    func cloud(roi: (x: Int, y: Int, w: Int, h: Int), scene: SceneContext, depthTimestamp: TimeInterval) -> CloudBlock {
        let start = CACurrentMediaTime()
        var depth = [UInt16](repeating: 0, count: roi.w * roi.h)
        var conf: [UInt8]? = confidenceMap != nil ? [UInt8](repeating: 0, count: roi.w * roi.h) : nil
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        if let confidenceMap { CVPixelBufferLockBaseAddress(confidenceMap, .readOnly) }
        if let base = CVPixelBufferGetBaseAddress(depthMap) {
            let rowBytes = CVPixelBufferGetBytesPerRow(depthMap)
            let confBase = confidenceMap.flatMap { CVPixelBufferGetBaseAddress($0) }
            let confRowBytes = confidenceMap.map { CVPixelBufferGetBytesPerRow($0) } ?? 0
            for j in 0..<roi.h {
                let row = base.advanced(by: (roi.y + j) * rowBytes).assumingMemoryBound(to: Float32.self)
                let confRow = confBase?.advanced(by: (roi.y + j) * confRowBytes).assumingMemoryBound(to: UInt8.self)
                for i in 0..<roi.w {
                    let d = row[roi.x + i]
                    if d.isFinite && d > 0 { depth[j * roi.w + i] = UInt16(min(d * 10000, 65535).rounded()) }
                    if let confRow { conf?[j * roi.w + i] = confRow[roi.x + i] }
                }
            }
        }
        if let confidenceMap { CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly) }
        CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)

        // Edge quality (stats only): holes = no depth; "hand" = within 15 cm of the nearest-decile
        // depth; flying = between the hand and 1.2 m (nothing real is there from below: those are
        // mixed hand/ceiling pixels on silhouette edges, or fill); the rest is background.
        let valid = depth.filter { $0 > 0 }
        if valid.count >= 20 {
            let sorted = valid.sorted()
            let handMax = UInt32(sorted[sorted.count / 10]) + 1500   // +15 cm in 0.1 mm units
            var hand = 0, flying = 0
            for d in valid {
                if UInt32(d) <= handMax { hand += 1 } else if d < 12000 { flying += 1 }
            }
            PipelineStats.shared.cloudEdges(holes: depth.count - valid.count, hand: hand,
                                            flying: flying, total: depth.count)
        }

        // Image intrinsics scaled to depth resolution (same FOV; pixel-center convention).
        let sx = Float(width) / Float(scene.imageWidth), sy = Float(height) / Float(scene.imageHeight)
        let r4: (Float) -> Float = { ($0 * 10000).rounded() / 10000 }
        let block = CloudBlock(
            t: depthTimestamp,
            filtered: filtered,
            w: width, h: height,
            roi: [roi.x, roi.y, roi.w, roi.h],
            intr: [r4(scene.intrinsicsFx * sx), r4(scene.intrinsicsFy * sy),
                   r4((scene.intrinsicsCx + 0.5) * sx - 0.5), r4((scene.intrinsicsCy + 0.5) * sy - 0.5)],
            depth: depth.withUnsafeBytes { Data($0) }.base64EncodedString(),
            conf: conf.map { Data($0).base64EncodedString() }
        )
        PipelineStats.shared.cloudBuilt(cells: roi.w * roi.h, bytes: block.depth.utf8.count + (block.conf?.utf8.count ?? 0),
                                        seconds: CACurrentMediaTime() - start)
        return block
    }

    /// Visit valid depths in a pixel rectangle (inclusive), stepping by `step`.
    private func forEachDepth(x0: Int, x1: Int, y0: Int, y1: Int, step: Int = 1,
                              _ body: (Int, Int, Float) -> Void) {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        if let confidenceMap { CVPixelBufferLockBaseAddress(confidenceMap, .readOnly) }
        defer { if let confidenceMap { CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly) } }

        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return }
        let rowBytes = CVPixelBufferGetBytesPerRow(depthMap)
        let confBase = confidenceMap.flatMap { CVPixelBufferGetBaseAddress($0) }
        let confRowBytes = confidenceMap.map { CVPixelBufferGetBytesPerRow($0) } ?? 0

        var y = max(y0, 0)
        while y <= min(y1, height - 1) {
            let row = base.advanced(by: y * rowBytes).assumingMemoryBound(to: Float32.self)
            let confRow = confBase?.advanced(by: y * confRowBytes).assumingMemoryBound(to: UInt8.self)
            var x = max(x0, 0)
            while x <= min(x1, width - 1) {
                let d = row[x]
                if d.isFinite && d > 0 && (confRow.map { $0[x] >= Self.minConfidence } ?? true) {
                    body(x, y, d)
                }
                x += step
            }
            y += step
        }
    }

    private func window(normalizedX: Float, normalizedY: Float, radius r: Int) -> [Float] {
        let cx = min(max(Int(normalizedX * Float(width)), 0), width - 1)
        let cy = min(max(Int(normalizedY * Float(height)), 0), height - 1)
        var samples: [Float] = []
        samples.reserveCapacity((2 * r + 1) * (2 * r + 1))
        forEachDepth(x0: cx - r, x1: cx + r, y0: cy - r, y1: cy + r) { _, _, d in samples.append(d) }
        return samples
    }

    private static func median(_ v: [Float]) -> Float {
        let s = v.sorted()
        return s[s.count / 2]
    }

    /// The palm's depth: from below, the hand is the nearest surface and the ceiling is far
    /// behind it, so take the nearest-quartile depth of each interior joint's 5×5 window (robust
    /// to a window that straddles the palm's edge), then the median across those joints.
    func palmDepth(at joints: [SIMD2<Float>]) -> Float? {
        let perJoint: [Float] = joints.compactMap { p in
            let w = window(normalizedX: p.x, normalizedY: p.y, radius: 2).sorted()
            return w.count >= 3 ? w[w.count / 4] : nil
        }
        guard !perJoint.isEmpty else { return nil }
        return Self.median(perJoint)
    }

    /// Depth at a joint: the median of the confident samples in its window that fall within
    /// `band` of the palm depth. Tips sit on the silhouette edge, so they get a wider window.
    /// Nil when fewer than 3 samples survive (the joint has no trustworthy depth this frame).
    func jointDepth(normalizedX: Float, normalizedY: Float, palm: Float, band: Float, radius: Int) -> Float? {
        let kept = window(normalizedX: normalizedX, normalizedY: normalizedY, radius: radius)
            .filter { abs($0 - palm) < band }
        guard kept.count >= 3 else { return nil }
        return Self.median(kept)
    }

    /// The static reference: median depth over a coarse grid of the frame, skipping each hand's
    /// joint bounding box (padded). Nil when too little confident depth remains.
    func ceiling(excluding hands: [HandObservation]) -> Float? {
        let pad: Float = 0.08
        let boxes: [(Float, Float, Float, Float)] = hands.compactMap { hand in
            let seen = hand.landmarks.filter { $0[0] != 0 || $0[1] != 0 }
            guard !seen.isEmpty else { return nil }
            let xs = seen.map { $0[0] }, ys = seen.map { $0[1] }
            return (xs.min()! - pad, xs.max()! + pad, ys.min()! - pad, ys.max()! + pad)
        }
        var samples: [Float] = []
        samples.reserveCapacity((width / 4) * (height / 4))
        let fw = Float(width), fh = Float(height)
        forEachDepth(x0: 0, x1: width - 1, y0: 0, y1: height - 1, step: 4) { x, y, d in
            let nx = Float(x) / fw, ny = Float(y) / fh
            for b in boxes where nx >= b.0 && nx <= b.1 && ny >= b.2 && ny <= b.3 { return }
            samples.append(d)
        }
        guard samples.count >= 50 else { return nil }
        samples.sort()
        return samples[samples.count / 2]
    }
}

/// A raw depth crop (hand-wire v2 `cloud`). Pixel (u, v) = (roi.x + i, roi.y + j) of the full
/// w×h depth map; with d = depth·1e-4 m: X = (u−cx)/fx·d, Y = (v−cy)/fy·d, phone = toPhone·(X,Y,d).
nonisolated struct CloudBlock: Sendable, Encodable {
    let v = 1                  // cloud block version
    let t: TimeInterval        // depth capture time (same clock as handFrame.timestamp)
    let filtered: Bool         // true = source-side smoothing/hole filling (AVF filtering, ARKit smoothed)
    let w: Int                 // full depth map width
    let h: Int                 // full depth map height
    let roi: [Int]             // [x0, y0, w, h] in depth pixels
    let intr: [Float]          // [fx, fy, cx, cy] at depth resolution
    let depth: String          // base64 u16 LE, row-major roi.w×roi.h, 0.1 mm units, 0 = no depth
    let conf: String?          // base64 u8 ARConfidenceLevel per pixel; absent on AVFoundation
    /// OpenCV camera (x right, y down, z forward) → PHONE, row-major. Constant for the native
    /// landscape-right back-camera buffer.
    let toPhone: [Float] = [0, 1, 0, -1, 0, 0, 0, 0, 1]

    private enum CodingKeys: String, CodingKey { case v, t, filtered, w, h, roi, intr, depth, conf, toPhone }
    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(v, forKey: .v)
        try c.encode(t, forKey: .t); try c.encode(filtered, forKey: .filtered); try c.encode(w, forKey: .w); try c.encode(h, forKey: .h)
        try c.encode(roi, forKey: .roi); try c.encode(intr, forKey: .intr)
        try c.encode(depth, forKey: .depth); try c.encodeIfPresent(conf, forKey: .conf)
        try c.encode(toPhone, forKey: .toPhone)
    }
}
