//
//  ViewController.swift
//  MotionSource
//
//  Created by Ivan Lugo on 2/25/26.
//

import UIKit
import SwiftUI
import Metal
import MetalKit
import ARKit

extension MTKView : RenderDestinationProvider {
}

class ViewController: UIViewController, MTKViewDelegate, ARSessionDelegate {

    var session: ARSession!
    var renderer: Renderer!
    var handTracker: HandTracker!
    var cameraFrameStreamer: CameraFrameStreamer!
    var connectionViewModel: ConnectionViewModel!
    let motionSensor = MotionSensor()

    /// Capture pipeline, chosen at launch with `-pipeline avf|arkit`:
    /// AVFoundation wide camera + LiDAR depth camera (default: the phone is static, so no world
    /// tracking is needed, and it measured ~30 ms lower capture→send latency and far fewer
    /// depthless joints than ARKit), or ARKit world tracking + sceneDepth.
    enum Pipeline: String { case arkit, avf }
    let pipeline: Pipeline = {
        let requested = UserDefaults.standard.string(forKey: "pipeline").flatMap(Pipeline.init(rawValue:))
        let wanted = requested ?? .avf
        if wanted == .avf && !LiDARCapture.isAvailable { return .arkit }
        return wanted
    }()
    private var lidarCapture: LiDARCapture?

    /// The sourceId the first-hand cue last fired for: one cue per registered connection.
    private var firstHandCuedFor: String?

    override func viewDidLoad() {
        super.viewDidLoad()

        // Face-down, nobody touches the screen: never let it auto-lock and suspend the stream.
        UIApplication.shared.isIdleTimerDisabled = true
        motionSensor.start()

        // Set the view's delegate
        session = ARSession()
        session.delegate = self

        // Set the view to use the default device
        if let view = self.view as? MTKView {
            view.device = MTLCreateSystemDefaultDevice()
            view.backgroundColor = UIColor.clear
            view.delegate = self

            guard view.device != nil else {
                print("Metal is not supported on this device")
                return
            }

            // Configure the renderer to draw to the view
            renderer = Renderer(session: session, metalDevice: view.device!, renderDestination: view)

            renderer.drawRectResized(size: view.bounds.size)
        }

        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(ViewController.handleTap(gestureRecognize:)))
        view.addGestureRecognizer(tapGesture)

        // Hand tracking
        let computedMode = HandTracker.DepthMode.preferred
        print("Starting hand tracking with mode: \(computedMode)")
        handTracker = HandTracker(
            depthMode: computedMode
        )

        // Camera frame streamer
        cameraFrameStreamer = CameraFrameStreamer()
        cameraFrameStreamer.onFrameEncoded = { [weak self] jsonString in
            Task { @MainActor [weak self] in
                self?.connectionViewModel.send(jsonString)
            }
        }

        print("[Pipeline] \(pipeline.rawValue)")
        if pipeline == .avf {
            let capture = LiDARCapture()
            if capture.configure() {
                let tracker = handTracker!, streamer = cameraFrameStreamer!
                capture.onFrame = { pixelBuffer, depth, scene, timestamp, depthTimestamp, depthIsNew in
                    streamer.processFrame(pixelBuffer: pixelBuffer, timestamp: timestamp)
                    // A frame dropped here while Vision is busy may carry the only use of a new
                    // depth map; the cloud then skips that depth (rate is reported in [Stats]).
                    guard tracker.beginProcessing() else { return }
                    tracker.track(pixelBuffer: pixelBuffer, depth: depth, scene: scene, timestamp: timestamp,
                                  depthTimestamp: depthTimestamp, depthIsNew: depthIsNew)
                }
                lidarCapture = capture
            } else {
                print("[Pipeline] AVFoundation LiDAR setup failed; falling back to ARKit")
            }
        }

        // Connection view model + SwiftUI overlay
        connectionViewModel = ConnectionViewModel()

        // Send from the tracking queue; only on-screen state goes through main.
        let sender = FrameSender(client: connectionViewModel.webSocketClient, motion: motionSensor)
        // Encoding a frame with its depth crop (~130 KB of JSON) takes milliseconds; doing it on
        // its own serial queue lets the tracker take the next camera frame meanwhile.
        let sendQueue = DispatchQueue(label: "com.motionsource.send", qos: .userInitiated)
        handTracker.onFrameTracked = { [weak self] tracked in
            sendQueue.async { sender.send(tracked) }
            Task { @MainActor [weak self] in
                self?.showTrackedFrame(tracked)
            }
        }

        let overlay = ConnectionOverlayView(viewModel: connectionViewModel)
        let hostingController = UIHostingController(rootView: overlay)
        hostingController.view.backgroundColor = .clear
        addChild(hostingController)
        view.addSubview(hostingController.view)
        hostingController.view.translatesAutoresizingMaskIntoConstraints = false
        NSLayoutConstraint.activate([
            hostingController.view.topAnchor.constraint(equalTo: view.topAnchor),
            hostingController.view.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            hostingController.view.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            hostingController.view.trailingAnchor.constraint(equalTo: view.trailingAnchor),
        ])
        hostingController.didMove(toParent: self)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)

        if let lidarCapture {
            lidarCapture.start()
            return
        }
        let configuration = ARWorldTrackingConfiguration()
        if let semantic = handTracker.depthMode.frameSemantic {
            configuration.frameSemantics.insert(semantic)
        }
        print("[ARSession] video format \(configuration.videoFormat.imageResolution) @ \(configuration.videoFormat.framesPerSecond) fps")

        session.run(configuration)
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)

        session.pause()
        lidarCapture?.stop()
        motionSensor.stop()
        connectionViewModel.webSocketClient.disconnect()
    }

    @objc
    func handleTap(gestureRecognize: UITapGestureRecognizer) {
        // Create anchor using the camera's current position
        if let currentFrame = session.currentFrame {

            // Create a transform with a translation of 0.2 meters in front of the camera
            var translation = matrix_identity_float4x4
            translation.columns.3.z = -0.2
            let transform = simd_mul(currentFrame.camera.transform, translation)

            // Add a new anchor to the session
            let anchor = ARAnchor(transform: transform)
            session.add(anchor: anchor)
        }
    }

    // MARK: - MTKViewDelegate

    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        renderer.drawRectResized(size: size)
    }

    // Called whenever the view needs to render
    func draw(in view: MTKView) {
        renderer.update()
    }

    // MARK: - ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        handTracker.processFrame(frame)
        cameraFrameStreamer.processFrame(frame)

    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        print("[ARSession] Error: \(error)")
    }

    func sessionWasInterrupted(_ session: ARSession) {
    }

    func sessionInterruptionEnded(_ session: ARSession) {
    }

    // MARK: - Hand Frame Streaming

    /// Main-thread side of a tracked frame: on-screen state and the first-hand cue. The frame
    /// itself was already sent from the tracking queue by `FrameSender`.
    private func showTrackedFrame(_ tracked: TrackedFrame) {
        let observations = tracked.hands
        connectionViewModel.handsDetected = observations.count
        renderer.handObservations = observations
        if !observations.isEmpty, let id = connectionViewModel.sourceId, firstHandCuedFor != id {
            firstHandCuedFor = id
            Cues.firstHand()
        }
    }
}
