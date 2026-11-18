"""
Simplified phase-1 bench rig for the golf launch monitor -- ALL dimensions live here.

One 2020 aluminium beam carries the two cameras, the two LED boards, the electronics box, the
shield clamps and the tripod adapter.  A photo tripod with a lockable head provides height,
pitch and level.  Camera toe-in is built into the printed camera mounts (no adjustment).

Units: mm, degrees, grams.  Every value tagged  VERIFY  is a nominal figure for a bought part:
MEASURE the real part and edit it before printing, then run `python3 build.py`.

Frames:
BF  beam frame: +X along the beam, +Y = optical axis direction (towards the ball), +Z up,
    origin on the beam centre line at the camera-pair centre (mean lens pupil x = 0).
K   camera frame: origin on the optical axis in the plane of the board's front face, +Y forward.
WF  world frame: origin on the mat below the ball centre, +X = target line, rig at -Y, +Z up.
"""
import math

# ---------------------------------------------------------------------------
# 0. Printer / process
# ---------------------------------------------------------------------------
BUILD_VOLUME = (220.0, 220.0, 250.0)   # printer X, Y, Z                          VERIFY
NOZZLE = 0.4
MIN_WALL = 2.4            # design rule: >= 6 perimeters of 0.4
WALL = 3.0
CLR = 0.25                # sliding clearance per side                tune to printer
CLR_TIGHT = 0.15
EMBOSS_H = 0.6
EMBOSS_SIZE = 5.0
INFILL_EFF = 0.5          # effective solid fraction (mass estimate only)
PETG_DENSITY = 1.27       # g/cm3

# ---------------------------------------------------------------------------
# 1. Fasteners
# ---------------------------------------------------------------------------
INSERT_M25_D = 3.6        # M2.5 heat-set insert hole                    VERIFY
INSERT_M25_L = 4.5
M3_CLEAR = 3.4
M3_BTN_D = 6.0            # ISO 7380 M3 button head pocket
M3_HEAD_H = 3.4
INSERT_M3_D = 4.2         # M3 heat-set insert hole                      VERIFY
INSERT_M3_L = 5.7                                                      # VERIFY
INSERT_M3_SHORT_L = 4.4                                                # VERIFY
M5_CLEAR = 5.6
M5_HEAD_D = 9.8           # ISO 7380 button head M5
M5_HEAD_H = 3.2
M6_CLEAR = 6.6
UNC14_CLEAR = 6.6         # 1/4"-20 tripod stud clearance
NUT14_AF = 11.11 + 0.4    # 1/4"-20 hex nut across flats (7/16")         VERIFY
NUT14_H = 5.56 + 0.4                                                   # VERIFY
NUT38_AF = 14.29 + 0.4    # optional 3/8"-16 hex nut (9/16")             VERIFY
NUT38_H = 7.5 + 0.4                                                    # VERIFY
UNC38_CLEAR = 9.8
ZIP_W = 3.6               # cable tie width                              VERIFY
ZIP_T = 1.6

# ---------------------------------------------------------------------------
# 2. 2020 extrusion                                                    VERIFY
# ---------------------------------------------------------------------------
EXT = 20.0
EXT_SLOT_OPEN = 6.2
EXT_LIP = 1.8
EXT_CAV_W = 11.0
EXT_CAV_D = 6.1
EXT_BORE = 4.2
EXT_MASS_PER_M = 520.0    # g/m

# ---------------------------------------------------------------------------
# 3. Set-up geometry
# ---------------------------------------------------------------------------
BALL_D = 42.67
BALL_R = BALL_D / 2.0
BALL_DIST = 475.0         # horizontal distance mean lens pupil -> ball centre (450-500)
LENS_HEIGHT = 255.0       # lens axis height above the mat (set by the tripod)
PITCH_NOM = 17.0          # nominal pitch down (tripod head tilt, 15-25)
BASELINE = 160.0          # stereo baseline (120-220)
AIM_U = 110.0             # ball -> mean pupil, down-range (+X world)
TOE_IN_R = 6.0            # built-in toe-in, camera at +X turns towards -X (deg); optimize_fov.py
TOE_IN_L = 10.5           # camera at -X turns towards +X (deg)
FLIGHT_COVER = 250.0
FOV_MARGIN_DEG = 1.0

# ---------------------------------------------------------------------------
# 4. IMX296 board, lens, filter                                         ALL VERIFY
# ---------------------------------------------------------------------------
CAM_PCB = 38.0
CAM_PCB_T = 1.6
CAM_HOLE_SQ = 34.0        # 4 x M2.5
CAM_HOLDER_W = 27.0       # CS holder body (square), must stay < ~28
CAM_HOLDER_H = 11.0       # PCB front face -> lens flange face
CAM_REAR_H = 5.0          # tallest part behind the PCB (FPC connector)
CAM_CONN_W = 17.0
CAM_CONN_D = 5.0
CAM_FPC_LEN = 200.0
LENS_D = 35.0             # max barrel diameter
LENS_L = 45.0             # max length from the flange
LENS_PUPIL = 28.0         # entrance pupil ahead of the flange
LENS_MASS = 90.0
CAM_MASS = 35.0
FOCAL = 8.0
SENSOR_W = 5.02
SENSOR_H = 3.75
FILTER_D = 25.0           # 850 nm band-pass
FILTER_T = 2.0
FILTER_APERTURE = 23.0
FILTER_GAP = 1.0          # lens front -> first aperture wall

# ---------------------------------------------------------------------------
# 5. LED side boards (two) and strobe modules                           VERIFY
# ---------------------------------------------------------------------------
LED_PLATE = (110.0, 90.0, 3.0)    # aluminium plate X x Z x T
LED_PLATE_HOLE_INSET = 5.0
LED_STAR_D = 20.0
LED_STAR_T = 1.6
LED_LENS_H = 15.0                 # TIR lens height (8-15)
LED_GRID = (5, 4)
LED_SKIP = ((0, 0), (4, 0))       # 18 of 20 positions used
LED_HALF_ANGLE = 100.0            # check emission half angle (conservative)
LED_X = 140.0                     # board centre from the centre line (140 nominal; 200-250 by sliding)
LED_ZC = -42.0                    # board centre height in BF (below the camera hoods)
LED_MASS = 210.0                  # plate + stars + lenses
DIFFUSER_T = 1.5
MOD = (90.0, 70.0, 1.6)           # strobe module board X x Z x T
MOD_HOLE_INSET = 4.0
MOD_PARTS_H = 25.0
MOD_MASS = 120.0
LED_WIRE_MAX = 150.0

# ---------------------------------------------------------------------------
# 6. Electronics box                                                   VERIFY
# ---------------------------------------------------------------------------
PI_PCB = (85.0, 56.0, 1.6)
PI_HOLES = (58.0, 49.0)
PI_HOLE_EDGE = 3.5
PI_COOLER_H = 17.0
PI_COOLER = (38.0, 30.0)
PI_CSI = ((54.0, 0.0), (70.0, 0.0))   # CSI connector centres (u, v from left / bottom edge)
PI_MASS = 70.0
CTRL = (100.0, 160.0, 1.6)        # control board X x Z (standing) x T
CTRL_HOLE_INSET = 4.0
CTRL_COMP_H = 20.0
CTRL_MASS = 110.0
PICO_PCB = (51.0, 21.0, 1.0)
PICO_HEADER_H = 9.0
PICO_MASS = 6.0
JACK_D = 11.0
SWITCH_D = 12.0
ARMED_LED_D = 5.1
BOARD_STANDOFF = 6.0
BOX_WALL = 3.0
BOX_DEPTH_IN = 38.0               # interior depth behind the front wall
PI_X = 18.0                       # Pi centre (BF x): near the centre so both ribbons stay short
PI_Z = 44.0
CTRL_GAP = 12.0                   # Pi left edge -> control board right edge
CTRL_Z0 = 16.0                    # control board bottom (BF z)

# ---------------------------------------------------------------------------
# 7. Shield, tripod, alignment card
# ---------------------------------------------------------------------------
SHIELD_T = 3.0                    # polycarbonate thickness               VERIFY
SHIELD_CLEAR = 8.0                # hood front -> sheet back face
SHIELD_X_ARM = 210.0              # clamp centre
SHIELD_Z = (-94.0, 72.0)          # sheet bottom / top height (BF)
TA_X = 15.2                       # tripod stud position under the beam (BF), set from the head COM
TA_Y = 52.1
TA_T = 12.0                       # adapter plate thickness
TA_W = 60.0
TRIPOD_HEAD = (55.0, 55.0, 12.0)  # head platform proxy                   VERIFY
CARD = (200.0, 200.0, 4.0)        # alignment card X x Y x T

# ---------------------------------------------------------------------------
# camera mount geometry (K frame)
# ---------------------------------------------------------------------------
HOOD_HALF = 25.0
BOSS_H = 1.5
WALL_T = 9.0
CAM_Y0 = 34.0                     # K origin y in BF (board front face plane)
CAM_AXIS_Z = 10.0 + HOOD_HALF     # lens axis height in BF (hood floor on the beam top plane)
BASE_T = 6.0
CBASE_HALF_X = 40.0

# ---------------------------------------------------------------------------
# overrides (sweeps)
# ---------------------------------------------------------------------------
import json as _json
import os as _os
for _k, _v in _json.loads(_os.environ.get("RIG_OVERRIDES", "{}")).items():
    globals()[_k] = _v

# ===========================================================================
# derived
# ===========================================================================
Z_BEAM_TOP = EXT / 2
Z_BEAM_BOT = -EXT / 2
Y_BEAM_FRONT = EXT / 2
Y_BEAM_REAR = -EXT / 2
HOLDER_RECESS = -0.5
PCB_FRONT_K = 0.0
WALL_BACK_K = BOSS_H
WALL_FRONT_K = BOSS_H + WALL_T
HOLDER_FRONT_K = CAM_HOLDER_H
LENS_FRONT_K = HOLDER_FRONT_K + LENS_L
PUPIL_K = HOLDER_FRONT_K + LENS_PUPIL
BORE_R = max(LENS_D / 2 + 0.8, CAM_HOLDER_W * math.sqrt(2) / 2 + 0.6)
BORE_END_K = LENS_FRONT_K + FILTER_GAP
APW1_K = (BORE_END_K, BORE_END_K + MIN_WALL)                  # rear aperture wall
POCKET_K = (APW1_K[1], APW1_K[1] + FILTER_T + 0.4)            # filter pocket
APW2_K = (POCKET_K[1], POCKET_K[1] + MIN_WALL)                # front aperture wall
HOOD_FRONT_K = APW2_K[1]
ZIP_Y = (-22.0, -3.0)

BEAM_HALF = max(240.0, LED_X + 60.0 + 12.0, SHIELD_X_ARM + 17.0 + 3.0)
BEAM_X0, BEAM_X1 = -BEAM_HALF, BEAM_HALF
BEAM_LEN = BEAM_X1 - BEAM_X0
SHIELD_W = 2.0 * (SHIELD_X_ARM + 17.0 - 6.0)                   # edge sits 8 mm inside the post
