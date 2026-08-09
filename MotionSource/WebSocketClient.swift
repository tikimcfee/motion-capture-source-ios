//
//  WebSocketClient.swift
//  MotionSource
//

import Foundation

enum ConnectionState: Sendable, Equatable {
    case disconnected
    case connecting
    case connected
    case failed(String)
}

/// WebSocket client that connects to a relay endpoint and pushes data upstream.
/// Uses URLSessionWebSocketTask for proper HTTP upgrade handshake with ws:// URLs.
nonisolated class WebSocketClient: NSObject, URLSessionWebSocketDelegate, @unchecked Sendable {
    private var webSocketTask: URLSessionWebSocketTask?
    private var urlSession: URLSession!

    var onStateChanged: ((@Sendable (ConnectionState) -> Void))?

    /// Called with the relay's assigned source id once the handshake is accepted.
    var onRegistered: ((@Sendable (String) -> Void))?

    override init() {
        super.init()
        urlSession = URLSession(configuration: .default, delegate: self, delegateQueue: nil)
    }

    /// Open the connection, optionally announcing a role first.
    ///
    /// The relay classifies a client by its FIRST message: `DISPLAY` for the
    /// browser, `SOURCE <kind>` for a sensor device, anything else for a CLI
    /// controller. Send a hand frame first and this app gets classed a controller
    /// and its frames parsed as command strings.
    ///
    /// The greeting is enqueued here, immediately after `resume()`, rather than in
    /// the didOpen delegate callback. ARKit's hand-tracking callback pushes frames
    /// without waiting for `.connected`, so a frame can be queued before the socket
    /// finishes opening. URLSessionWebSocketTask preserves send order, so enqueuing
    /// the greeting first makes it first on the wire by construction — no ordering
    /// assumption about when the app starts streaming.
    func connect(to urlString: String, greeting: String? = nil) {
        disconnect()

        guard let url = URL(string: urlString) else {
            notifyState(.failed("Invalid URL"))
            return
        }

        notifyState(.connecting)

        webSocketTask = urlSession.webSocketTask(with: url)
        webSocketTask?.resume()
        if let greeting {
            send(greeting)
        }
        listenForDisconnect()
    }

    func disconnect() {
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
    }

    func send(_ string: String) {
        webSocketTask?.send(.string(string)) { error in
            if let error = error {
                print("[WebSocketClient] Send error: \(error)")
            }
        }
    }

    // MARK: - URLSessionWebSocketDelegate

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didOpenWithProtocol protocol: String?) {
        print("[WebSocketClient] Connected")
        notifyState(.connected)
    }

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask, didCloseWith closeCode: URLSessionWebSocketTask.CloseCode, reason: Data?) {
        print("[WebSocketClient] Closed (code: \(closeCode))")
        notifyState(.disconnected)
    }

    func urlSession(_ session: URLSession, task: URLSessionTask, didCompleteWithError error: (any Error)?) {
        if let error = error {
            print("[WebSocketClient] Connection error: \(error)")
            notifyState(.failed(error.localizedDescription))
        }
    }

    // MARK: - Private

    /// Keep a receive loop alive to detect server-side disconnects, and pick the
    /// relay's registration ack out of the stream.
    ///
    /// The ack is the only on-device proof the greeting was accepted — a device
    /// that got misclassified still shows a healthy socket, so "connected" alone
    /// says nothing about whether frames are being understood.
    private func listenForDisconnect() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .success(let message):
                if case .string(let text) = message {
                    self?.handleServerMessage(text)
                }
                self?.listenForDisconnect()
            case .failure:
                break // delegate methods handle state
            }
        }
    }

    /// Parse the relay ack: {"ok":true,"role":"source","sourceId":"src-hand-0",…}
    private func handleServerMessage(_ text: String) {
        guard let data = text.data(using: .utf8),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let role = json["role"] as? String,
              let sourceId = json["sourceId"] as? String
        else { return }

        guard role == "source" else {
            print("[WebSocketClient] Relay classified this client as '\(role)', not a source")
            return
        }

        print("[WebSocketClient] Registered as source '\(sourceId)'")
        let callback = onRegistered
        DispatchQueue.main.async {
            callback?(sourceId)
        }
    }

    private func notifyState(_ state: ConnectionState) {
        let callback = onStateChanged
        DispatchQueue.main.async {
            callback?(state)
        }
    }
}
