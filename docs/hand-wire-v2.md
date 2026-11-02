# Hand wire contract v2 — face-down phone

Status: implemented by MotionSource on branch `face-down`. Owned jointly by the app side
(MotionSource) and the consumer side (glyph3d-js calibration). Change it by agreement between
the two, then update both repos' docs.

## Setup
The iPhone lies FACE-DOWN on the desk, top edge pointing away from the person. The back camera
and LiDAR look UP at the hand hovering above it; the ceiling is the static background. The
screen can't be seen or touched, so the phone just streams and all guidance lives in glyph3d-js.

Hand height: the camera's field of view is ~70°, so a hand ~0.2 m above the phone clips at
the edges. At ~0.3 m the whole hand is in view.

On the phone, nothing is needed after launch:
- the screen never auto-locks while the app runs (`isIdleTimerDisabled`);
- it connects on its own to the last relay address that worked, and keeps retrying (2 s, and a
  6 s connect timeout) after any drop until the user disconnects in the UI;
- the endpoint can be set from the Mac at launch with no typing:
  `xcrun devicectl device process launch --device <phone> lugonium.works.MotionSource -- -relayAddress ws://<relay-host>:8080`
  (persisted once it connects; no host address lives in the repo);
- cues (haptic + system sound; the sound follows the ringer switch): **success + "begin
  recording"** when the relay acks `SOURCE hand`; **tap + "tink"** on the first hand seen on
  that connection; **error + "end recording"** when a registered connection drops.

## Transport (unchanged from v1)
WebSocket to the glyph3d relay (`glyph3d-cli serve`, port 8080). First message `SOURCE hand`;
the relay acks `{"ok":true,"role":"source","sourceId":"src-hand-0",…}` and forwards each later
text frame to the display as `{event:'source.frame', source, kind, data}` (drop-oldest).
A low-rate `cameraFrame` JPEG (2 Hz, 320 px) is also sent, as in v1.

## handFrame
```
{
  "type": "handFrame",
  "v": 2,
  "timestamp": 1234.567,             // s, device monotonic clock (systemUptime), CAPTURE time
  "sentAt": 1234.601,                // s, same clock, stamped as the frame is serialized for
                                     // sending: sentAt − timestamp = on-phone latency
  "mount": "faceDown",               // faceDown | faceUp | upright | unknown — from gravity
  "device": {                        // null until CoreMotion has delivered a sample
    "gravity": [x, y, z],            // unit vector, PHONE frame; face-down ≈ [0, 0, -1]
    "attitude": [qx, qy, qz, qw],    // PHONE frame vs CoreMotion's xArbitraryZVertical reference
    "userAcceleration": [x, y, z]    // m/s², gravity removed, PHONE frame: the latest 100 Hz
                                     // CoreMotion sample (≤ 10 ms old). A slide shows here; it is
                                     // invisible to gravity/attitude. At rest: peaks ~0.15 m/s²
  },
  "ceiling": 2.31,                   // m along PHONE +z, median depth outside the hands; null if unknown
  "hands": [{
    "handedness": "left" | "right",  // as Vision reports it — see "Handedness" below
    "confidence": [c0 … c20],        // Vision per-joint confidence 0..1 (0 = not recognized)
    "points": [[x,y,z] | null × 21], // METERS, PHONE frame; SKIN-SURFACE points (see below);
                                     // null = not seen or no valid depth
    "rays": [[x,y,z] | null × 21],   // UNIT direction optical center → joint, PHONE frame, from
                                     // the 2D keypoint + image intrinsics, depth-independent;
                                     // null only when not detected (same conf gate as points).
                                     // points[j] null + rays[j] set = seen in 2D, no usable depth
    "landmarks": [[u,v,d] × 21]      // v1: normalized image coords (top-left origin, landscape-
                                     // right buffer) + depth in m; [0,0,0] when not seen
  }],
  "scene": { … v1 scene payload (intrinsics, imageResolution, cameraTransform, trackingState,
             lightIntensity, viewport) … }
}
```
- Joint order is MediaPipe's 21: wrist 0; thumb 1-4; index 5-8; middle 9-12; ring 13-16;
  pinky 17-20 (glyph3d-core `HandData.js` `Joint`).
- Frames with `"hands": []` are sent at 10 Hz while no hand is seen, so presence, mount and
  ceiling are known before a hand shows up. With hands, every processed frame is sent.
- Numbers are rounded: points 0.1 mm, rays 1e-4, gravity/attitude 1e-4, confidence 1e-3, ceiling 1 mm.

## cloud (raw depth crops, for cloud-based tracking)
The relay forwards source frames as TEXT only, so depth rides inside handFrame as base64.
```
"cloud": {
  "v": 1,                            // cloud block version
  "t": 1234.542,                     // DEPTH capture time, same clock as handFrame.timestamp
  "filtered": false,                 // true = source-side temporal smoothing / hole filling
                                     // (AVF isFilteringEnabled, ARKit smoothedSceneDepth)
  "w": 320, "h": 240,                // FULL depth map size
  "roi": [x0, y0, w, h],             // the crop, in depth pixels
  "intr": [fx, fy, cx, cy],          // at DEPTH resolution (image intrinsics scaled, pixel centers)
  "depth": "<base64>",               // u16 little-endian, row-major roi.w×roi.h,
                                     // 0.1 mm units along PHONE +z; 0 = no depth FROM THE
                                     // SENSOR (non-finite/≤0); the app applies no near/far cut
  "conf": "<base64>",                // u8 ARConfidenceLevel 0/1/2 per pixel — ABSENT on AVFoundation
  "toPhone": [0,1,0, -1,0,0, 0,0,1]  // OpenCV camera (x right, y down, z fwd) → PHONE, row-major
}
```
- `hands[i].cloud`: around that hand. The ROI is its detected keypoints' bounding box in depth
  pixels, padded by 25 % of the box's larger side and clamped. Depth is RAW: no filtering and
  no segmentation, ceiling included. Segmenting is the consumer's job.
- Top-level `cloud`: the whole depth map (roi = full), about once a second, also on empty
  frames (never throttled away). This is for the ceiling and the scene.
- Rate: AVFoundation depth is 30 Hz under 60 Hz video, so a cloud is attached only to the frame
  that first uses a new depth map, about every other hand frame. A frame dropped while Vision
  is busy can skip a depth map. ARKit attaches one to every frame.
- Unproject pixel (u, v) = (roi.x + i, roi.y + j) with `d = depth·1e-4`:
  `p_cam = ((u−cx)/fx·d, (v−cy)/fy·d, d)`, `phone = toPhone · p_cam` (= the joints' mapping).

## The PHONE frame (right-handed, meters)
- origin: the wide camera's optical center (ARKit registers LiDAR depth to it).
- +y: toward the phone's TOP edge (away from the person, the way the fingers point).
- +z: out of the phone's BACK (straight up toward the ceiling when face-down).
- +x = y × z: the person's RIGHT when the phone lies face-down, top edge away.

Relation to other frames:
- CoreMotion device frame: PHONE = CoreMotion turned 180° about y, `(x,y,z)_phone = (-x, y, -z)_cm`.
  Attitude: `q_phone = q_cm ⊗ (0,1,0,0)`, i.e. `(qx,qy,qz,qw)_phone = (-qz, qw, qx, -qy)_cm`.
- Camera image (ARKit `capturedImage`, native landscape-right, top-left origin): pixel +u runs
  toward the phone's BOTTOM edge, pixel +v toward PHONE +x. A joint at pixel (u, v) with
  z-depth d unprojects as `X = (u-cx)/fx·d`, `Y = (v-cy)/fy·d`, then `phone = (Y, -X, d)`.
  The intrinsics are those of the captured image (1920×1440); the depth map (256×192, same 4:3
  field of view) is sampled at the same normalized position, so no intrinsics rescaling is
  needed for the unprojection itself.

## Points are skin-surface points, not joint centers
From below, LiDAR measures the palmar skin, so each point is where the camera ray through the
2D joint meets the hand's surface (nearest to the camera). True joint centers sit about
0.6–0.9 cm further along +z for fingers and 1.2–1.8 cm for palm and wrist. The app does NOT
add an offset; calibration absorbs it. Bone lengths measured between surface points come out
slightly short for bones tilted toward the camera.

## How the numbers are made (app-side processing)
- Hand pose: Apple Vision `VNDetectHumanHandPoseRequest`, up to 2 hands, on the full captured
  image. A joint with confidence ≤ 0.3 is unseen (`null` point, zero landmark), but its
  confidence is still reported.
- Capture pipeline (launch argument `-pipeline avf|arkit`):
  - **`avf` (default)**: AVFoundation `builtInLiDARDepthCamera`, 1920×1440 video at 60 fps plus
    320×240 Float32 depth at 30 fps (the only LiDAR depth rate on iPhone 17 Pro), unfiltered
    (`-depthFilter YES` to compare), paired by `AVCaptureDataOutputSynchronizer` (`-avfSync NO`
    pairs each video frame with the latest depth instead). A video frame reuses a depth map
    ≤ 45 ms old. Intrinsics come per frame from the video buffer. There is no world tracking:
    `scene.trackingState` is `"notAvailable"`, `scene.cameraTransform` is identity, and
    `lightIntensity` is 0. There is no per-pixel depth confidence, so all finite depths are used.
  - `arkit`: ARKit world tracking + `sceneDepth` (raw, not `smoothedSceneDepth`; switchable with
    `-depthMode smoothed|raw|noDepth`), using only pixels with ARKit confidence ≥ medium.
- Depth at a joint ("hand band"): first the palm depth = median over wrist + 4 MCPs of the
  nearest-quartile depth in each one's 5×5 window. Then each joint's depth is the median of the
  pixels in its window (5×5; 7×7 for fingertips, which sit on the silhouette edge) that lie
  within ±12 cm of the palm depth, which drops the ceiling and the flying pixels between it and
  the hand. Fewer than 3 such pixels → `null`. No bone-length reconstruction of missing joints
  is done in the app.
- Smoothing: **none**. No temporal filter on points or depth (the v1 per-joint EMA and depth
  clamp were removed); the consumer filters for itself.
- Ceiling: the median of confident depth pixels on a 4-px grid of the whole depth map, skipping
  each hand's joint bounding box padded by 8 % of the image; per frame, unfiltered; `null` with
  fewer than 50 samples.
- Mount: gravity in the PHONE frame; `faceDown` if g.z < -0.8, `faceUp` if g.z > 0.8,
  `upright` if |g.x| or |g.y| > 0.8, else `unknown`.

## Handedness
`handedness` is Vision's chirality, passed through unchanged. Measured from below (back camera,
palm toward the lens, phone face-down): **not mirrored**, a back-camera image is not a mirror
image. Vision's label is noisy from this view, though. Over live use, a right hand was labelled
right in ~98 % of frames, but a left hand was labelled left in only ~64 %. A geometric test
stayed consistent within each session (1 disagreeing frame in ~2,000) and matched Vision's
majority label: for a palm-down hand in the PHONE
frame, `n = (p5 − p0) × (p17 − p0)`; right hand ⇔ `n.z < 0`. Consumers should prefer the
geometric test (with a vote over frames) to the per-frame label.

## Measured (iPhone 17 Pro, Release build, 2026-11-01, phone face-down, natural hand use)
Medians of 2 s windows. A "still" window = wrist sd < 3 mm on every axis, detected
automatically. Noise is the per-axis sd of the PHONE-frame point.

| | AVFoundation (default) | ARKit |
|---|---|---|
| camera rate | 60 fps | 60 fps |
| processed, hand present | 57 fps | ~42 fps |
| processed, no hand | 60 fps | 60 fps |
| Vision time, hand / none | 14 / 8 ms | 15 / 9 ms |
| frame delivered to app after capture | ~28 ms (synchronizer waits for depth) | ~60 ms |
| capture → send, hand / none | 43 / 37 ms | 76 / 59–66 ms |
| joints seen but depthless | 5.7 % (tips 18 %, others 2 %) | 22–35 % |
| still noise, wrist (x/y/z) | 0.7 / 0.6 / 0.6 mm @ 0.17 m | 0.7 / 1.1 / 1.2 mm @ 0.27 m |
| still noise, index MCP | 0.8 / 0.8 / 1.3 mm @ 0.20 m | 1.1 / 1.0 / 1.7 mm @ 0.30 m |
| still noise, index tip | 2.7 / 2.7 / 5.5 mm @ 0.22 m | 1.9 / 2.9 / 7.4 mm @ 0.32 m |
| ceiling, frame-to-frame spread | ±0–1 mm (2.04 m) | ±0.5 mm (2.04 m) |
| world tracking on the ceiling | n/a | `normal` once static (no lasting `limited`) |

The hand heights differed between runs (noise grows with distance), so the noise rows compare
only roughly. Every frame handed to the socket completed on the wire (0 failures). The relay
saw ~43 hand frames/s.

### Depth filtering A/B (AVFoundation `isFilteringEnabled`, off = default)
Four alternating runs, 10–12 full-hand windows each, hand at ~0.17–0.25 m. Values are off vs on.
- capture → send: 46–48 ms vs 47–48 ms. No visible latency cost.
- Depthless joints: 0.9–4.6 % vs 0.7–3.9 %. No real difference.
- Still noise: wrist and MCP the same (≈0.4–1.2 mm per axis). Fingertip z about halved with
  filtering on (index tip z 5.8–11.4 mm off, 2.1–6.6 mm on).
- Hand-crop holes: ~9 % of pixels off, 0 % on (hole filling). Neither mode showed intermediate
  "flying" depths between hand and ceiling: unfiltered edges drop out as holes.
Off stays the default. The gains with filtering on come from temporal smoothing and filled-in
edges, and this contract wants near-raw depth that the consumer filters itself. `filtered` in
each cloud block says which was used.

