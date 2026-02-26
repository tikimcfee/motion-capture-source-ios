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

    override init() {
        super.init()
        urlSession = URLSession(configuration: .default, delegate: self, delegateQueue: nil)
    }

    func connect(to urlString: String) {
        disconnect()

        guard let url = URL(string: urlString) else {
            notifyState(.failed("Invalid URL"))
            return
        }

        notifyState(.connecting)

        webSocketTask = urlSession.webSocketTask(with: url)
        webSocketTask?.resume()
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

    /// Keep a receive loop alive to detect server-side disconnects.
    private func listenForDisconnect() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .success:
                self?.listenForDisconnect()
            case .failure:
                break // delegate methods handle state
            }
        }
    }

    private func notifyState(_ state: ConnectionState) {
        let callback = onStateChanged
        DispatchQueue.main.async {
            callback?(state)
        }
    }
}
