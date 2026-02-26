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

    override func viewDidLoad() {
        super.viewDidLoad()

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
        handTracker = HandTracker()
        handTracker.onHandsDetected = { [weak self] observations, scene, timestamp in
            Task { @MainActor [weak self] in
                self?.sendHandFrame(observations: observations, scene: scene, timestamp: timestamp)
            }
            
        }

        // Camera frame streamer
        cameraFrameStreamer = CameraFrameStreamer()
        cameraFrameStreamer.onFrameEncoded = { [weak self] jsonString in
            Task { @MainActor [weak self] in
                self?.connectionViewModel.send(jsonString)
            }
        }

        // Connection view model + SwiftUI overlay
        connectionViewModel = ConnectionViewModel()

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

        let configuration = ARWorldTrackingConfiguration()

        // Enable LiDAR scene depth if available
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
        }

        session.run(configuration)
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)

        session.pause()
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

    private func sendHandFrame(observations: [HandObservation], scene: SceneContext, timestamp: TimeInterval) {
        connectionViewModel.handsDetected = observations.count
        renderer.handObservations = observations

        let message = HandFrameMessage(
            hands: observations,
            timestamp: timestamp,
            scene: ScenePayload(from: scene)
        )

        guard let jsonString = MessageEncoder.encode(message) else { return }
        connectionViewModel.send(jsonString)
    }
}
