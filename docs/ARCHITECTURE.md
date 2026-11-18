# Golf Simulator — Architecture & Interface Contract

This file is the single source of truth shared by every sub-project. If your
component touches an interface defined here, implement it exactly. Do not
change this file; report needed changes instead.

## 1. System overview

```
┌──────────────── Sensor unit (launch monitor) ────────────────┐
│  2x USB3 mono global-shutter cameras (IMX273 class,          │
│     1440x1080, ROI-cropped for several hundred fps,          │
│     hardware trigger input, GenICam)                         │
│  IR strobe (850 nm LED array, 20-50 us pulses)               │
│  24 GHz CW Doppler radar module with I/Q outputs             │
│  RP2040 (Pico) "timing MCU": camera trigger, strobe timing,  │
│     radar ADC sampling, shot detection                       │
│  x86 mini PC (Intel N100 class, Ubuntu 24.04), USB to all    │
│     of the above. Runs: gs_core, gs_service, OTA agent       │
└───────────────┬──────────────────────────────────────────────┘
                │ Wi-Fi / Ethernet: WebSocket + REST (JSON), mDNS
     ┌──────────┴───────────┐
     │                      │
 PC simulator (Godot 4)   Phone (PWA served by the sensor unit)
 course play + range      live shot data, history, settings, OTA
```

The sensor unit only produces shot data. All 3D simulation runs on the PC.

Placement: the unit sits on the floor behind the ball on the target line
extension, about 1.5–2.5 m back, looking down-range (radar needs radial
velocity). All geometry comes from calibration/config, never hard-coded, so a
side-on placement must also work for the camera pipeline.

## 2. Repository layout (one owner per directory)

| Path | Content |
|---|---|
| `common/gs_common/` | Shared Python types and interfaces (already written — read it) |
| `firmware/` | RP2040 timing-MCU firmware (C, pico-sdk) + `firmware/host/gs_mcu/` Python driver |
| `sensor/gs_core/` | Camera abstraction, calibration, ball detection, stereo, spin, radar DSP, fusion |
| `service/gs_service/` | Device API server (REST + WebSocket), shot storage, mDNS, GSPro adapter |
| `service/web/` | Phone PWA (static files served by gs_service) |
| `ota/` | Update bundle format, signing tools, device update agent, update server |
| `sim/` | Godot 4 PC simulator (course play, driving range, self-update) |
| `hardware/electronics/` | Schematics, netlists, BOM |
| `hardware/enclosure/` | Parametric 3D-printable models, STL/STEP exports |
| `docs/` | Cross-cutting docs |

Python: 3.11+ compatible, type-hinted, `pytest` tests in each project's
`tests/`. Each Python project has its own `pyproject.toml` and depends on
`gs_common` by path. No project may import another project's internals — only
`gs_common` and the public interfaces listed here.

## 3. Coordinate system and units

Right-handed, origin at the ball's rest position:
- `+X` down the target line (toward the target)
- `+Y` to the left of the target line (seen from behind the ball)
- `+Z` up

SI units internally (m, m/s, s). Angles in degrees and spin in rpm at API
boundaries.

Launch parameters:
- `speed_mps` — ball speed magnitude
- `vla_deg` — vertical launch angle above horizontal
- `hla_deg` — horizontal launch angle, **positive = right of target** (toward −Y)
- `total_spin_rpm` — spin rate magnitude
- `spin_axis_deg` — tilt of the spin axis, 0 = pure backspin,
  **positive = ball curves right** (fade/slice for a right-hander)
- `back_spin_rpm = total * cos(axis)`, `side_spin_rpm = total * sin(axis)`

## 4. Shot message (schema version 1)

Canonical definition: `common/gs_common/shot.py` and
`common/schema/shot.schema.json`.

```json
{
  "type": "shot",
  "schema": 1,
  "shot_id": "3f2b6c1e-...-uuid4",
  "ts": "2026-11-16T08:30:12.345Z",
  "ball": {
    "speed_mps": 67.1,
    "vla_deg": 12.4,
    "hla_deg": -1.8,
    "total_spin_rpm": 2850.0,
    "spin_axis_deg": 4.5,
    "back_spin_rpm": 2841.2,
    "side_spin_rpm": 223.6
  },
  "club": { "speed_mps": 45.2 },
  "quality": {
    "confidence": 0.93,
    "sources": ["stereo", "radar"],
    "spin_measured": true
  }
}
```

`club` may be `null`. When `spin_measured` is false the spin values are
model estimates.

## 5. Device API (gs_service, default port 8080)

Discovery: mDNS service `_golfsim._tcp.local.`, TXT `id=<device id>`,
`ver=<software version>`, `api=1`.

WebSocket `ws://<host>:8080/ws/v1` — server pushes JSON messages, each with a
`type` field:
- `hello` `{type, device_id, sw_version, fw_version, api: 1}` (first message)
- `shot` (section 4)
- `status` `{type, state: "idle"|"ready"|"processing"|"error"|"updating", ball_detected: bool, message: str}` on change and every 2 s
- `update` `{type, phase, progress: 0..1, message}` during OTA

Client → server over the same socket: `{"type":"ping"}` → `{"type":"pong"}`,
`{"type":"arm"}`, `{"type":"disarm"}`.

REST (JSON):
- `GET  /api/v1/status` — device info, state, versions, sensor health
- `GET  /api/v1/shots?limit=&before=` — history (newest first)
- `GET  /api/v1/shots/{shot_id}`; `DELETE /api/v1/shots/{shot_id}`
- `GET/PUT /api/v1/config` — user settings (units, handedness, altitude, placement distance…)
- `POST /api/v1/sim/shot` — inject a shot (body = `ball` object, optional `club`); for testing without hardware
- `GET  /api/v1/update` — current version, available version, state
- `POST /api/v1/update/check`, `POST /api/v1/update/apply`
- `GET  /` — phone PWA

GSPro compatibility: optional adapter that forwards shots as GSPro Open
Connect v1 JSON over TCP (default `127.0.0.1:921`, configurable host).

## 6. Internal Python interfaces (`gs_common.interfaces`)

- `ShotSource` — async source of `Shot` objects plus status; implemented by
  `gs_core` (real pipeline and a synthetic one), consumed by `gs_service`.
- `McuLink` — driver for the timing MCU; implemented by `gs_mcu`, consumed by
  `gs_core`.
- `Updater` — OTA control surface; implemented by `ota`, consumed by
  `gs_service`.

Every implementer also ships a mock/synthetic implementation so each project
runs and tests without hardware and without the other projects.

## 7. Timing MCU (RP2040) responsibilities

- Generate camera trigger pulses (both cameras, same edge) at a configured
  frame rate, and IR strobe pulses aligned inside each exposure (configurable
  delay, width 10–100 us, 1–4 pulses per frame for multi-exposure). Hardware
  timed (PIO), jitter < 1 us. Hard firmware limit on strobe duty cycle.
- Count frames (32-bit frame counter, incremented per trigger pulse).
- Sample radar I and Q with the ADC continuously into a ring buffer
  (>= 40 kS/s per channel; 24.125 GHz gives 160.8 Hz per m/s, ball up to
  ~90 m/s => ~14.5 kHz).
- Detect a shot from radar band energy, report `t_us` and the frame counter at
  detection, then send the buffered raw I/Q window (pre- and post-trigger) to
  the host.
- USB CDC link to the host, COBS-framed binary packets with CRC16. The
  firmware project owns the wire protocol and documents it in
  `firmware/PROTOCOL.md`; the host only uses it through `gs_mcu`.
- Enter the USB bootloader on command for firmware OTA.

## 8. OTA

Three updatable artifacts, one signed manifest format (Ed25519):
1. Sensor software (gs_common, gs_core, gs_service, gs_mcu, PWA) — A/B slot
   directories on the mini PC, atomic symlink switch, health check, automatic
   rollback.
2. MCU firmware (`.uf2`) — flashed by the device agent through the bootloader,
   version verified afterwards.
3. PC simulator — the Godot app checks the same kind of manifest and downloads
   a new build (`.pck` or full package).

Manifest (`manifest.json`, with detached signature `manifest.json.sig`):

```json
{
  "schema": 1,
  "channel": "stable",
  "released": "2026-11-16T00:00:00Z",
  "artifacts": {
    "sensor":   {"version": "1.2.0", "url": "sensor-1.2.0.tar.zst", "sha256": "…", "size": 123, "min_version": "1.0.0"},
    "firmware": {"version": "1.1.0", "url": "timing-mcu-1.1.0.uf2", "sha256": "…", "size": 123},
    "sim":      {"version": "1.3.0", "url": "sim-1.3.0.pck", "sha256": "…", "size": 123, "platform": "windows-x86_64"}
  },
  "notes": "Release notes text"
}
```

URLs are relative to the manifest URL. Versions are semver. The signature
covers the exact bytes of `manifest.json`. Clients must verify the signature
before trusting any field and the sha256 before installing any artifact.

## 9. Revision 3 — side-on four-camera unit (supersedes conflicting text above)

The agreed hardware concept is in `docs/SPEC.md` (Traditional Chinese). Where
sections 1 and 7 above disagree with it, SPEC.md wins. Summary of what changes
for the software contract:

- Four measurement cameras in one side-on unit (ball stereo pair + club stereo
  pair), one colour replay camera on a mast, built-in host and touchscreen.
  Radar is an optional add-on; the shot trigger is image-based on the host.
- The IR strobe pulse train is the master clock; cameras are triggered on
  every Nth pulse (see SPEC.md section 4). MCU pin map and trigger config will
  be revised when work resumes.
- No markers on the ball. Two stickers per club. `ClubData` has optional
  fields for path, attack angle, face angle, face-to-path, dynamic loft,
  impact x/y and an `estimated` list (`common/gs_common/shot.py`).
- New API surface to add: swing replay clips per shot, Wi-Fi/Bluetooth setup,
  club registry, battery/tilt status.
