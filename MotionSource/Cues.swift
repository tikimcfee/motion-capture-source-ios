//
//  Cues.swift
//  MotionSource
//
//  Haptic + sound cues for face-down use, where the screen can't be seen.
//

import UIKit
import AudioToolbox

enum Cues {
    // System UI sounds (they follow the ringer switch; the haptic fires regardless).
    private static let registeredSound: SystemSoundID = 1113   // "begin recording"
    private static let firstHandSound: SystemSoundID = 1057    // "tink"
    private static let lostSound: SystemSoundID = 1114         // "end recording"

    /// The relay acked `SOURCE hand`: frames are now reaching the display.
    static func registered() {
        UINotificationFeedbackGenerator().notificationOccurred(.success)
        AudioServicesPlaySystemSound(registeredSound)
    }

    /// The first hand seen since this connection registered.
    static func firstHand() {
        UIImpactFeedbackGenerator(style: .medium).impactOccurred()
        AudioServicesPlaySystemSound(firstHandSound)
    }

    /// A registered connection dropped (it will retry on its own).
    static func connectionLost() {
        UINotificationFeedbackGenerator().notificationOccurred(.error)
        AudioServicesPlaySystemSound(lostSound)
    }
}
