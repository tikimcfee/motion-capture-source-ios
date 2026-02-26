//
//  ConnectionViewModel.swift
//  MotionSource
//

import Foundation
import Observation

@Observable
class ConnectionViewModel {
    var address: String = "ws://192.168.50.249:8765"
    var state: ConnectionState = .disconnected
    var handsDetected: Int = 0
    var deviceIP: String = "unknown"

    let webSocketClient = WebSocketClient()

    init() {
        webSocketClient.onStateChanged = { [weak self] newState in
            Task { @MainActor [weak self] in
                self?.state = newState
            }
        }
        refreshDeviceIP()
    }

    func toggleConnection() {
        switch state {
        case .disconnected, .failed:
            webSocketClient.connect(to: address)
        case .connecting, .connected:
            webSocketClient.disconnect()
        }
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
