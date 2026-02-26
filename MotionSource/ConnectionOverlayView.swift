//
//  ConnectionOverlayView.swift
//  MotionSource
//

import SwiftUI

struct ConnectionOverlayView: View {
    @Bindable var viewModel: ConnectionViewModel

    var body: some View {
        VStack {
            Spacer()

            VStack(spacing: 10) {
                // Status row
                HStack(spacing: 12) {
                    statusDot
                    Text(viewModel.stateLabel)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.white)

                    Spacer()

                    Text("Hands: \(viewModel.handsDetected)")
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.white)

                    Text(viewModel.deviceIP)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.white.opacity(0.6))
                }

                // Address + connect row
                HStack(spacing: 8) {
                    TextField("ws://host:port", text: $viewModel.address)
                        .font(.system(.body, design: .monospaced))
                        .textFieldStyle(.plain)
                        .foregroundStyle(.white)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(.black.opacity(0.5))
                        .clipShape(RoundedRectangle(cornerRadius: 8))
                        .autocorrectionDisabled()
                        .textInputAutocapitalization(.never)
                        .disabled(viewModel.isConnected)

                    Button(action: { viewModel.toggleConnection() }) {
                        Text(buttonLabel)
                            .font(.system(.body, design: .monospaced, weight: .semibold))
                            .foregroundStyle(.white)
                            .padding(.horizontal, 16)
                            .padding(.vertical, 6)
                            .background(buttonColor)
                            .clipShape(RoundedRectangle(cornerRadius: 8))
                    }
                }
            }
            .padding(12)
            .background(.black.opacity(0.6))
            .clipShape(RoundedRectangle(cornerRadius: 12))
            .padding(.horizontal, 16)
            .padding(.bottom, 8)
        }
    }

    private var statusDot: some View {
        Circle()
            .fill(dotColor)
            .frame(width: 8, height: 8)
    }

    private var dotColor: Color {
        switch viewModel.state {
        case .connected: return .green
        case .connecting: return .yellow
        case .disconnected: return .gray
        case .failed: return .red
        }
    }

    private var buttonLabel: String {
        switch viewModel.state {
        case .disconnected, .failed: return "Connect"
        case .connecting: return "Cancel"
        case .connected: return "Disconnect"
        }
    }

    private var buttonColor: Color {
        switch viewModel.state {
        case .disconnected, .failed: return .blue
        case .connecting: return .orange
        case .connected: return .red.opacity(0.8)
        }
    }
}
