//
//  MotionSensor.swift
//  MotionSource
//
//  CoreMotion gravity + attitude, expressed in the wire contract's PHONE frame.
//

import CoreMotion
import simd

/// The PHONE frame (hand-wire v2): right-handed, meters, origin at the wide camera.
///   +y toward the phone's TOP edge
///   +z out of the phone's BACK (up toward the ceiling when the phone lies face-down)
///   +x = y × z (the person's RIGHT when the phone lies face-down, top edge away)
///
/// CoreMotion's device frame shares +y but has +z out of the SCREEN, so the PHONE frame is the
/// CoreMotion frame turned 180° about y: (x, y, z)_phone = (-x, y, -z)_coremotion.
nonisolated enum PhoneFrame {
    static func fromCoreMotion(_ v: SIMD3<Double>) -> SIMD3<Float> {
        SIMD3<Float>(Float(-v.x), Float(v.y), Float(-v.z))
    }

    /// CoreMotion attitude q maps device coords to its reference frame (v_ref = q·v_dev).
    /// With v_dev = R_y(180°)·v_phone, the PHONE frame's attitude is q·R_y(180°), and for
    /// q = (x, y, z, w) that product is (-z, w, x, -y).
    static func attitudeFromCoreMotion(_ q: CMQuaternion) -> SIMD4<Float> {
        SIMD4<Float>(Float(-q.z), Float(q.w), Float(q.x), Float(-q.y))
    }

    /// A pixel on the ARKit captured image (native landscape-right buffer, top-left origin) plus
    /// its z-depth, unprojected into the PHONE frame.
    ///
    /// In the landscape-right buffer, pixel +u (right) runs toward the phone's BOTTOM edge and
    /// pixel +v (down) runs toward the screen's left side, i.e. PHONE +x. The camera looks along
    /// PHONE +z. So with the OpenCV-style ray X = (u-cx)/fx·d, Y = (v-cy)/fy·d, Z = d:
    ///   phone = (Y, -X, d)
    static func unproject(u: Float, v: Float, depth d: Float,
                          fx: Float, fy: Float, cx: Float, cy: Float) -> SIMD3<Float> {
        let X = (u - cx) / fx * d
        let Y = (v - cy) / fy * d
        return SIMD3<Float>(Y, -X, d)
    }
}

/// How the phone lies, from gravity in the PHONE frame. Gravity points DOWN, so a face-down
/// phone (back up) reads gravity ≈ (0, 0, -1).
nonisolated enum Mount: String, Sendable {
    case faceDown, faceUp, upright, unknown

    static func from(gravity g: SIMD3<Float>) -> Mount {
        let t: Float = 0.8
        if g.z < -t { return .faceDown }
        if g.z > t { return .faceUp }
        if abs(g.y) > t || abs(g.x) > t { return .upright }
        return .unknown
    }
}

nonisolated struct MotionSample: Sendable {
    let gravity: SIMD3<Float>    // unit, PHONE frame
    let attitude: SIMD4<Float>   // qx, qy, qz, qw — PHONE frame vs xArbitraryZVertical reference
    let userAcceleration: SIMD3<Float>  // m/s², gravity removed, PHONE frame
    var mount: Mount { Mount.from(gravity: gravity) }
}

/// Polls CoreMotion device motion; `latest` is safe to read from any thread.
nonisolated final class MotionSensor: @unchecked Sendable {
    private let manager = CMMotionManager()
    private let lock = NSLock()
    private var _latest: MotionSample?

    var latest: MotionSample? {
        lock.lock(); defer { lock.unlock() }
        return _latest
    }

    func start() {
        guard manager.isDeviceMotionAvailable else {
            print("[MotionSensor] Device motion unavailable")
            return
        }
        manager.deviceMotionUpdateInterval = 1.0 / 100.0  // faster than frames: the sample sent is <= 10 ms old
        let queue = OperationQueue()
        queue.name = "com.motionsource.motion"
        queue.maxConcurrentOperationCount = 1
        manager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { [weak self] motion, _ in
            guard let self, let motion else { return }
            let g = motion.gravity
            var gravity = PhoneFrame.fromCoreMotion(SIMD3(g.x, g.y, g.z))
            let len = simd_length(gravity)
            if len > 0 { gravity /= len }
            // CoreMotion reports userAcceleration in g; convert to m/s².
            let a = motion.userAcceleration
            let sample = MotionSample(
                gravity: gravity,
                attitude: PhoneFrame.attitudeFromCoreMotion(motion.attitude.quaternion),
                userAcceleration: PhoneFrame.fromCoreMotion(SIMD3(a.x, a.y, a.z)) * 9.80665
            )
            PipelineStats.shared.userAcceleration(simd_length(sample.userAcceleration))
            self.lock.lock()
            self._latest = sample
            self.lock.unlock()
        }
    }

    func stop() {
        manager.stopDeviceMotionUpdates()
    }
}
