//
//  LiDARCapture.swift
//  MotionSource
//
//  AVFoundation alternative to ARKit for a phone that doesn't move: the back wide camera plus
//  the LiDAR depth camera, synchronized, with no world tracking (no SLAM) in the loop.
//  Selected with the launch argument `-pipeline avf`.
//

import AVFoundation
import simd

nonisolated final class LiDARCapture: NSObject, AVCaptureDataOutputSynchronizerDelegate,
    AVCaptureVideoDataOutputSampleBufferDelegate, AVCaptureDepthDataOutputDelegate, @unchecked Sendable {
    let session = AVCaptureSession()
    private let videoOutput = AVCaptureVideoDataOutput()
    private let depthOutput = AVCaptureDepthDataOutput()
    private var synchronizer: AVCaptureDataOutputSynchronizer?
    private let queue = DispatchQueue(label: "com.motionsource.lidar", qos: .userInitiated)

    /// Depth runs slower than video on some formats; a video frame without its own depth reuses
    /// the latest depth map if it is at most this old.
    static let maxDepthAge: TimeInterval = 0.045

    /// Synchronized (default): AVCaptureDataOutputSynchronizer holds each video frame until its
    /// depth arrives. Unsynchronized (`-avfSync NO`): video goes to Vision as soon as it lands and
    /// pairs with the latest depth map — lower latency, slightly staler depth.
    let synchronized: Bool = UserDefaults.standard.object(forKey: "avfSync") == nil
        || UserDefaults.standard.bool(forKey: "avfSync")
    private var lastDepth: (sampler: DepthSampler, timestamp: TimeInterval)?
    private var fallbackIntrinsics: (fx: Float, fy: Float, cx: Float, cy: Float)?

    /// Called on the capture queue for every video frame: pixel buffer (native landscape-right),
    /// depth (may be nil), scene context (intrinsics in video-buffer pixels), capture timestamp.
    /// Also: the paired depth map's capture time, and whether that depth map is new (not yet
    /// handed out with an earlier video frame).
    var onFrame: ((CVPixelBuffer, DepthSampler?, SceneContext, TimeInterval, TimeInterval?, Bool) -> Void)?
    private var lastHandedDepthTimestamp: TimeInterval = -1

    static var isAvailable: Bool {
        AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) != nil
    }

    /// Configure the session. Logs every depth-capable format so the choice can be checked.
    func configure() -> Bool {
        guard let device = AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) else {
            print("[LiDARCapture] no builtInLiDARDepthCamera")
            return false
        }

        func dims(_ f: AVCaptureDevice.Format) -> CMVideoDimensions {
            CMVideoFormatDescriptionGetDimensions(f.formatDescription)
        }
        func maxFPS(_ f: AVCaptureDevice.Format) -> Double {
            f.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0
        }
        func depthType(_ f: AVCaptureDevice.Format) -> FourCharCode {
            CMFormatDescriptionGetMediaSubType(f.formatDescription)
        }
        let isFloatDepth: (AVCaptureDevice.Format) -> Bool = {
            let t = depthType($0)
            return t == kCVPixelFormatType_DepthFloat32 || t == kCVPixelFormatType_DepthFloat16
        }

        let candidates = device.formats.filter { !$0.supportedDepthDataFormats.isEmpty }
        for f in candidates {
            let d = dims(f)
            let depths = f.supportedDepthDataFormats.map { df -> String in
                let dd = dims(df)
                let rate = df.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0
                return "\(dd.width)x\(dd.height) \(Self.fourCC(depthType(df))) \(Int(rate))fps"
            }
            print("[LiDARCapture] format \(d.width)x\(d.height) \(Self.fourCC(CMFormatDescriptionGetMediaSubType(f.formatDescription))) ≤\(Int(maxFPS(f)))fps depth: \(depths.joined(separator: ", "))")
        }

        // Prefer the ARKit-equivalent 1920×1440 (4:3, full FOV), full-range 4:2:0, highest rate.
        let fullRange = kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
        let ranked = candidates
            .filter { f in f.supportedDepthDataFormats.contains(where: isFloatDepth) }
            .sorted { a, b in
                func score(_ f: AVCaptureDevice.Format) -> (Int, Int, Double, Int) {
                    let d = dims(f)
                    return (d.width * 3 == d.height * 4 ? 1 : 0,
                            d.width == 1920 ? 1 : 0,
                            maxFPS(f),
                            CMFormatDescriptionGetMediaSubType(f.formatDescription) == fullRange ? 1 : 0)
                }
                return score(a) > score(b)
            }
        guard let format = ranked.first else {
            print("[LiDARCapture] no format with float depth")
            return false
        }
        // Largest float depth map, 32-bit preferred.
        guard let depthFormat = format.supportedDepthDataFormats.filter(isFloatDepth).max(by: { a, b in
            let da = dims(a), db = dims(b)
            if da.width != db.width { return da.width < db.width }
            return depthType(a) == kCVPixelFormatType_DepthFloat16 && depthType(b) == kCVPixelFormatType_DepthFloat32
        }) else { return false }

        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .inputPriority

        guard let input = try? AVCaptureDeviceInput(device: device), session.canAddInput(input) else {
            print("[LiDARCapture] cannot add input"); return false
        }
        session.addInput(input)

        videoOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: fullRange]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        guard session.canAddOutput(videoOutput) else { print("[LiDARCapture] cannot add video output"); return false }
        session.addOutput(videoOutput)

        // Unfiltered by default (near-raw, like ARKit sceneDepth); `-depthFilter YES` to compare.
        depthOutput.isFilteringEnabled = UserDefaults.standard.bool(forKey: "depthFilter")
        depthOutput.alwaysDiscardsLateDepthData = true
        guard session.canAddOutput(depthOutput) else { print("[LiDARCapture] cannot add depth output"); return false }
        session.addOutput(depthOutput)

        if let connection = videoOutput.connection(with: .video),
           connection.isCameraIntrinsicMatrixDeliverySupported {
            connection.isCameraIntrinsicMatrixDeliveryEnabled = true
        }

        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            device.activeDepthDataFormat = depthFormat
            if let range = format.videoSupportedFrameRateRanges.max(by: { $0.maxFrameRate < $1.maxFrameRate }) {
                device.activeVideoMinFrameDuration = range.minFrameDuration
                device.activeVideoMaxFrameDuration = range.minFrameDuration
            }
            if let depthRange = depthFormat.videoSupportedFrameRateRanges.max(by: { $0.maxFrameRate < $1.maxFrameRate }) {
                device.activeDepthDataMinFrameDuration = depthRange.minFrameDuration
            }
            device.unlockForConfiguration()
        } catch {
            print("[LiDARCapture] lockForConfiguration failed: \(error)")
            return false
        }

        let vd = dims(format), dd = dims(depthFormat)
        print("[LiDARCapture] using video \(vd.width)x\(vd.height) @ \(maxFPS(format)) fps, depth \(dd.width)x\(dd.height) \(Self.fourCC(depthType(depthFormat))) @ \(depthFormat.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0) fps, filtering \(depthOutput.isFilteringEnabled)")

        if synchronized {
            let sync = AVCaptureDataOutputSynchronizer(dataOutputs: [videoOutput, depthOutput])
            sync.setDelegate(self, queue: queue)
            synchronizer = sync
        } else {
            videoOutput.setSampleBufferDelegate(self, queue: queue)
            depthOutput.setDelegate(self, callbackQueue: queue)
        }
        print("[LiDARCapture] synchronized \(synchronized)")
        return true
    }

    func start() { queue.async { self.session.startRunning() } }
    func stop() { queue.async { self.session.stopRunning() } }

    // MARK: - AVCaptureDataOutputSynchronizerDelegate

    func dataOutputSynchronizer(_ synchronizer: AVCaptureDataOutputSynchronizer,
                                didOutput collection: AVCaptureSynchronizedDataCollection) {
        if let synced = collection.synchronizedData(for: depthOutput) as? AVCaptureSynchronizedDepthData,
           !synced.depthDataWasDropped {
            handleDepth(synced.depthData, timestamp: synced.timestamp)
        }
        if let synced = collection.synchronizedData(for: videoOutput) as? AVCaptureSynchronizedSampleBufferData,
           !synced.sampleBufferWasDropped {
            handleVideo(synced.sampleBuffer)
        }
    }

    // Unsynchronized delegates.
    func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData,
                         timestamp: CMTime, connection: AVCaptureConnection) {
        handleDepth(depthData, timestamp: timestamp)
    }

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        handleVideo(sampleBuffer)
    }

    private func handleDepth(_ raw: AVDepthData, timestamp: CMTime) {
        let depthData = raw.depthDataType == kCVPixelFormatType_DepthFloat32
            ? raw : raw.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
        lastDepth = (DepthSampler(depthMap: depthData.depthDataMap, confidenceMap: nil,
                                  filtered: depthOutput.isFilteringEnabled),
                     CMTimeGetSeconds(timestamp))

        if fallbackIntrinsics == nil, let cal = depthData.cameraCalibrationData {
            let k = cal.intrinsicMatrix, ref = cal.intrinsicMatrixReferenceDimensions
            fallbackIntrinsics = (k[0][0], k[1][1], k[2][0], k[2][1])
            print("[LiDARCapture] calibration intrinsics fx \(k[0][0]) ref \(ref)")
        }
    }

    private func handleVideo(_ sampleBuffer: CMSampleBuffer) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        // Capture-session timestamps are on the host clock, the same clock as
        // CACurrentMediaTime() and ARFrame.timestamp.
        let timestamp = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer))
        PipelineStats.shared.cameraFrame(timestamp: timestamp)

        let width = CVPixelBufferGetWidth(pixelBuffer), height = CVPixelBufferGetHeight(pixelBuffer)
        var k: (fx: Float, fy: Float, cx: Float, cy: Float)
        if let data = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix,
                                      attachmentModeOut: nil) as? Data,
           data.count >= MemoryLayout<matrix_float3x3>.size {
            let m = data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float3x3.self) }
            k = (m[0][0], m[1][1], m[2][0], m[2][1])
        } else if let f = fallbackIntrinsics {
            k = f   // reference dimensions may differ; logged once above for checking
        } else {
            return
        }

        var depth: DepthSampler?
        var depthTimestamp: TimeInterval?
        var depthIsNew = false
        if let last = lastDepth, abs(timestamp - last.timestamp) <= Self.maxDepthAge {
            depth = last.sampler
            depthTimestamp = last.timestamp
            depthIsNew = last.timestamp != lastHandedDepthTimestamp
            lastHandedDepthTimestamp = last.timestamp
            PipelineStats.shared.depthAge(timestamp - last.timestamp)
        }

        let scene = SceneContext(
            intrinsicsFx: k.fx, intrinsicsFy: k.fy, intrinsicsCx: k.cx, intrinsicsCy: k.cy,
            imageWidth: width, imageHeight: height,
            cameraTransform: matrix_identity_float4x4.columnMajorArray,
            trackingState: "notAvailable",   // no world tracking in this pipeline
            lightIntensity: 0
        )
        onFrame?(pixelBuffer, depth, scene, timestamp, depthTimestamp, depthIsNew)
    }

    private static func fourCC(_ code: FourCharCode) -> String {
        let bytes = [24, 16, 8, 0].map { UInt8((code >> $0) & 0xFF) }
        return String(bytes: bytes, encoding: .ascii) ?? "\(code)"
    }
}
