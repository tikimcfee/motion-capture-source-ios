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

    /// Placeholder relay endpoint, shown only until a connection succeeds once (the last working
    /// address is remembered) or `-relayAddress` is passed at launch. Port matches
    /// `glyph3d-cli serve` (8080); edit the host in the UI.
    static let defaultAddress = "ws://relay-host:8080"

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

    /// Hands-free: once the phone is face-down nobody can tap Connect, so a dropped or failed
    /// connection retries on its own until the user explicitly disconnects.
    private var userDisconnected = false
    private var retryTask: Task<Void, Never>?
    static let retryDelay: Duration = .seconds(2)
    /// A connect to an unreachable host can sit in `.connecting` for a minute; give up sooner.
    static let connectTimeout: Duration = .seconds(6)

    /// Launch-argument override (`-relayAddress ws://host:port`, e.g. from
    /// `xcrun devicectl device process launch … -relayAddress ws://…`), so the endpoint can be set
    /// without touching the screen. Read from the argument domain only, never persisted here; a
    /// successful connection persists it like any other address.
    private static var launchArgumentAddress: String? {
        let args = UserDefaults.standard.volatileDomain(forName: UserDefaults.argumentDomain)
        guard let value = args["relayAddress"] as? String,
              !value.trimmingCharacters(in: .whitespaces).isEmpty else { return nil }
        return value
    }

    init() {
        // Restore the last endpoint that worked. An empty stored value is treated
        // as absent so a cleared field can't brick the next launch.
        if let saved = UserDefaults.standard.string(forKey: Self.addressKey),
           !saved.trimmingCharacters(in: .whitespaces).isEmpty {
            address = saved
        }
        if let override = Self.launchArgumentAddress {
            address = override
        }

        webSocketClient.onStateChanged = { [weak self] newState in
            Task { @MainActor [weak self] in
                guard let self else { return }
                self.state = newState
                switch newState {
                case .connected:
                    // Only a real open proves the endpoint is reachable — that is
                    // what's worth remembering for next launch.
                    UserDefaults.standard.set(self.address, forKey: Self.addressKey)
                case .connecting:
                    self.sourceId = nil
                    self.scheduleRetry(after: Self.connectTimeout, ifStillIn: .connecting)
                case .disconnected, .failed:
                    // A dropped connection invalidates the assigned id; the relay
                    // issues a fresh one on reconnect.
                    if self.sourceId != nil { Cues.connectionLost() }
                    self.sourceId = nil
                    if !self.userDisconnected { self.scheduleRetry(after: Self.retryDelay, ifStillIn: nil) }
                }
            }
        }
        webSocketClient.onRegistered = { [weak self] id in
            Task { @MainActor [weak self] in
                self?.sourceId = id
                Cues.registered()
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
            userDisconnected = true
            retryTask?.cancel()
            webSocketClient.disconnect()
        }
    }

    /// Retry the connection after `delay`, unless the user disconnected meanwhile or (when
    /// `ifStillIn` is given) the state moved on. A newer schedule replaces an older one.
    private func scheduleRetry(after delay: Duration, ifStillIn expected: ConnectionState?) {
        retryTask?.cancel()
        retryTask = Task { @MainActor [weak self] in
            try? await Task.sleep(for: delay)
            guard let self, !Task.isCancelled, !self.userDisconnected else { return }
            if let expected, self.state != expected { return }
            if expected == nil, self.state == .connected || self.state == .connecting { return }
            print("[Connection] retrying \(self.address)")
            self.openConnection()
        }
    }

    /// Single funnel for every connect, so no call site can open a socket without
    /// announcing what this device is.
    private func openConnection() {
        userDisconnected = false
        webSocketClient.connect(to: address, greeting: Self.sourceGreeting)
    }

    func send(_ string: String, captureTimestamp: TimeInterval? = nil) {
        webSocketClient.send(string, captureTimestamp: captureTimestamp)
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
