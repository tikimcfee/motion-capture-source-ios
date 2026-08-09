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

    /// Fallback relay endpoint, used only until a connection succeeds once. Port
    /// matches `glyph3d-cli serve` (8080); the host moves with DHCP and is
    /// expected to be edited in the UI.
    static let defaultAddress = "ws://192.168.50.249:8080"

    /// Where the last *successful* connection was made. Deliberately not written
    /// on every edit: the address field is a live-bound TextField, so persisting
    /// each keystroke would overwrite a known-good endpoint with whatever partial
    /// text was on screen mid-typing. Written when the socket actually opens.
    private static let addressKey = "relay.address"

    var address: String = ConnectionViewModel.defaultAddress
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
        // Restore the last endpoint that worked. An empty stored value is treated
        // as absent so a cleared field can't brick the next launch.
        if let saved = UserDefaults.standard.string(forKey: Self.addressKey),
           !saved.trimmingCharacters(in: .whitespaces).isEmpty {
            address = saved
        }

        webSocketClient.onStateChanged = { [weak self] newState in
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.state = newState
                if newState == .connected {
                    // Only a real open proves the endpoint is reachable — that is
                    // what's worth remembering for next launch.
                    UserDefaults.standard.set(self.address, forKey: Self.addressKey)
                } else {
                    // A dropped connection invalidates the assigned id; the relay
                    // issues a fresh one on reconnect.
                    self.sourceId = nil
                }
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
