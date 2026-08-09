//
//  ConnectionViewModel.swift
//  MotionSource
//

import Foundation
import Observation
import Synchronization

actor DoOnce {
    let didRun = Mutex(false)
    
    nonisolated func execute(_ action: () -> Void) {
        let didRun = self.didRun.withLock { $0 }
        guard !didRun else { return }
        self.didRun.withLock { $0 = true }
        action()
    }
    
    nonisolated func execute(_ action: () async -> Void) async {
        let didRun = self.didRun.withLock { $0 }
        guard !didRun else { return }
        self.didRun.withLock { $0 = true }
        await action()
    }
    
    nonisolated func execute(_ action: () async throws -> Void) async throws {
        let didRun = self.didRun.withLock { $0 }
        guard !didRun else { return }
        self.didRun.withLock { $0 = true }
        try await action()
    }
}

@Observable
class ConnectionViewModel {
    /// The relay's sensor handshake. `SOURCE <kind>` must be the first message on
    /// the wire — the relay classifies a client by it, and a device that skips it
    /// is treated as a CLI controller with its frames parsed as commands.
    static let sourceGreeting = "SOURCE hand"

    /// Default relay endpoint. Port matches `glyph3d-cli serve` (8080); the host
    /// is the Mac running the relay and is expected to be edited in the UI, since
    /// it moves with DHCP.
    var address: String = "ws://192.168.50.249:8080"
    var state: ConnectionState = .disconnected
    var handsDetected: Int = 0
    var deviceIP: String = "unknown"

    /// Relay-assigned id for this device (e.g. "src-hand-0"), once registered.
    /// Nil until the ack lands — which is the signal the handshake was accepted,
    /// distinct from the socket merely being open.
    var sourceId: String?

    let webSocketClient = WebSocketClient()
    let onAppearOnce = DoOnce()

    init() {
        webSocketClient.onStateChanged = { [weak self] newState in
            Task { @MainActor [weak self] in
                self?.state = newState
                // A dropped connection invalidates the assigned id; the relay
                // issues a fresh one on reconnect.
                if newState != .connected { self?.sourceId = nil }
            }
        }
        webSocketClient.onRegistered = { [weak self] id in
            Task { @MainActor [weak self] in
                self?.sourceId = id
            }
        }
        refreshDeviceIP()
    }

    func onAppear() {
        onAppearOnce.execute {
            guard state == .disconnected else { return }
            openConnection()
        }
    }

    func toggleConnection() {
        switch state {
        case .disconnected, .failed:
            openConnection()
        case .connecting, .connected:
            webSocketClient.disconnect()
        }
    }

    /// Single funnel for every connect, so no call site can open a socket without
    /// announcing what this device is.
    private func openConnection() {
        webSocketClient.connect(to: address, greeting: Self.sourceGreeting)
    }

    func send(_ string: String) {
        webSocketClient.send(string)
    }

    func refreshDeviceIP() {
        deviceIP = Self.getWiFiAddress()
    }

    var isConnected: Bool {
        state == .connected
    }

    var stateLabel: String {
        switch state {
        case .disconnected: return "Disconnected"
        case .connecting: return "Connecting..."
        case .connected: return "Connected"
        case .failed(let msg): return "Failed: \(msg)"
        }
    }

    // MARK: - WiFi Address

    static func getWiFiAddress() -> String {
        var address = "unknown"
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&ifaddr) == 0, let firstAddr = ifaddr else { return address }
        defer { freeifaddrs(ifaddr) }

        for ptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
            let interface = ptr.pointee
            let addrFamily = interface.ifa_addr.pointee.sa_family
            if addrFamily == UInt8(AF_INET) {
                let name = String(cString: interface.ifa_name)
                if name == "en0" {
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(
                        interface.ifa_addr,
                        socklen_t(interface.ifa_addr.pointee.sa_len),
                        &hostname,
                        socklen_t(hostname.count),
                        nil,
                        socklen_t(0),
                        NI_NUMERICHOST
                    )
                    address = String(cString: hostname)
                }
            }
        }
        return address
    }
}
