//
//  CameraFrameStreamer.swift
//  MotionSource
//

import ARKit
import CoreImage

/// Encodes ARFrame camera images as low-quality JPEG snapshots for WebSocket streaming.
/// Throttles output to a configurable interval and uses a GPU-backed CIContext for encoding.
nonisolated class CameraFrameStreamer: @unchecked Sendable {

    private let ciContext: CIContext
    private let encodeQueue = DispatchQueue(label: "com.motionsource.cameraframe", qos: .utility)
    private var lastSendTime: TimeInterval = 0
    private var isEncoding = false

    /// Minimum seconds between camera frame sends
    var sendInterval: TimeInterval = 0.5

    /// Target image width for downsampling (height computed from aspect ratio)
    var targetWidth: Int = 320

    /// JPEG compression quality (0.0 – 1.0)
    var jpegQuality: CGFloat = 0.3

    /// Called with the JSON string ready to broadcast
    var onFrameEncoded: ((@Sendable (String) -> Void))?

    init() {
        ciContext = CIContext(options: [.useSoftwareRenderer: false])
    }

    /// Feed every ARFrame here — internally throttles and drops if still encoding.
    func processFrame(_ frame: ARFrame) {
        let now = frame.timestamp
        guard now - lastSendTime >= sendInterval else { return }
        guard !isEncoding else { return }

        isEncoding = true
        lastSendTime = now

        let pixelBuffer = frame.capturedImage
        let timestamp = frame.timestamp

        encodeQueue.async { [weak self] in
            guard let self = self else { return }
            defer { self.isEncoding = false }

            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)

            // Downsample
            let originalWidth = CGFloat(CVPixelBufferGetWidth(pixelBuffer))
            let originalHeight = CGFloat(CVPixelBufferGetHeight(pixelBuffer))
            let scaleFactor = CGFloat(self.targetWidth) / originalWidth
            let targetHeight = Int(originalHeight * scaleFactor)

            let scaled = ciImage.transformed(by: CGAffineTransform(scaleX: scaleFactor, y: scaleFactor))

            // Encode to JPEG
            let colorSpace = CGColorSpace(name: CGColorSpace.sRGB)!
            guard let jpegData = self.ciContext.jpegRepresentation(
                of: scaled,
                colorSpace: colorSpace,
                options: [kCGImageDestinationLossyCompressionQuality as CIImageRepresentationOption: self.jpegQuality]
            ) else { return }

            let message = CameraFrameMessage(
                timestamp: timestamp,
                image: jpegData.base64EncodedString(),
                width: self.targetWidth,
                height: targetHeight,
                orientation: "landscapeRight"
            )

            guard let jsonString = MessageEncoder.encode(message) else { return }

            let callback = self.onFrameEncoded
            callback?(jsonString)
        }
    }
}
