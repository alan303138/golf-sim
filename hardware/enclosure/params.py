"""
Phase-1 bench test rig for the golf launch monitor -- ALL dimensions live here.

Units: millimetres, degrees, grams.  Every value tagged  VERIFY  is a nominal
or catalogue figure for a bought part: MEASURE the real part and edit the value
before printing, then run `python3 build.py` to regenerate everything.

Coordinate frames (see README for pictures)
-------------------------------------------
BF  beam frame  (rotates with the pitch):  +X along the 2020 beam, +Y = optical
    axis direction (forward, towards the ball), +Z up.  Origin: centre of the
    camera pair, centre line of the beam extrusion.
KP  camera-stage frame: origin on the yaw pivot, on the top face of the camera
    clamp base; +Y along the (toed-in) optical axis; parts of one camera stage
    are modelled in this frame and rotated into BF.
PF  pivot frame: BF axes at pitch 0, origin on the pitch axis.
HF  hub frame: origin on the hub bottom, directly below the mast.
WF  world frame: origin on the mat directly below the ball centre,
    +X = target line (flight direction), +Y = from ball towards the ball's
    far side, i.e. the rig stands at negative Y and looks along +Y, +Z up.
"""
import math

# ---------------------------------------------------------------------------
# 0. Printer / process
# ---------------------------------------------------------------------------
BUILD_VOLUME = (220.0, 220.0, 250.0)   # X, Y, Z of the printer          VERIFY
NOZZLE = 0.4
MIN_WALL = 2.4            # design rule: >= 6 perimeters of 0.4
WALL = 3.0                # default wall
CLR = 0.25                # sliding clearance, per side                  tune to printer
CLR_TIGHT = 0.15          # location fits (tongue, datum pins)
EMBOSS_H = 0.6            # raised part-number height
EMBOSS_SIZE = 5.0         # part-number character height
INFILL_EFF = 0.5          # effective solid fraction of a printed part (mass estimate only)
PETG_DENSITY = 1.27       # g/cm3

# ---------------------------------------------------------------------------
# 1. Fasteners (clearance holes, head pockets, heat-set insert holes)
# ---------------------------------------------------------------------------
M25_CLEAR = 2.8
INSERT_M25_D = 3.6        # hole for M2.5 heat-set insert (OD ~3.5)      VERIFY
INSERT_M25_L = 4.5
M3_CLEAR = 3.4
M3_HEAD_D = 6.4
M3_HEAD_H = 3.4
M3_CSK_D = 6.6            # countersunk flat head
M3_BTN_D = 6.0            # pocket for an ISO 7380 button head M3 (5.7)
INSERT_M3_D = 4.2         # hole for M3 heat-set insert (OD ~4.6)        VERIFY
INSERT_M3_L = 5.7         # hole depth (insert length + ~0.3)            VERIFY
INSERT_M3_SHORT_L = 4.4   # where the wall is thin (M3x4 short insert)   VERIFY
PICO_PILOT = 1.8          # pilot hole for M2 self-tapping into PETG
M4_CLEAR = 4.5
INSERT_M4_D = 5.6         # hole for M4 heat-set insert                  VERIFY
INSERT_M4_L = 5.5                                                      # VERIFY (short M4 insert)
M5_CLEAR = 5.6
M5_HEAD_D = 9.8           # ISO 7380 button head M5 (9.5) + clearance
M5_HEAD_H = 3.2
M5_TOOL_D = 10.5          # hex-key / socket access bore
M6_CLEAR = 6.6
INSERT_M6_D = 8.0         # hole for M6 heat-set insert                  VERIFY
INSERT_M6_L = 10.0                                                     # VERIFY
NUT_M6_AF = 10.0 + 0.4    # M6 hex nut across flats + clearance          VERIFY
NUT_M6_H = 5.2 + 0.3
ZIP_W = 3.6               # cable tie slot width (tie <= 3.6 wide)       VERIFY
ZIP_T = 1.6

# ---------------------------------------------------------------------------
# 2. Aluminium extrusion 2020 (T-slot, slot 6)                         VERIFY
# ---------------------------------------------------------------------------
EXT = 20.0
EXT_SLOT_OPEN = 6.2       # slot opening (6.0-6.3)                       VERIFY
EXT_LIP = 1.8             # lip thickness at the slot opening            VERIFY
EXT_CAV_W = 11.0          # T-cavity width                               VERIFY
EXT_CAV_D = 6.1           # T-cavity bottom, depth from the outer face   VERIFY
EXT_BORE = 4.2            # centre bore (M5 tap)
TONGUE_W = EXT_SLOT_OPEN - 0.5   # printed key that sits in the slot opening
TONGUE_D = EXT_LIP - 0.2         # key depth (stays inside the lip)
EXT_MASS_PER_M = 520.0    # g/m (typical 2020, 20x20 with centre bore)   VERIFY

# Extrusion lengths to buy / cut
BEAM_HALF_MIN = 240.0     # beam is symmetric about the camera centre; longer if LED_X needs it
LEG_LEN = 230.0           # three legs, 2020

# ---------------------------------------------------------------------------
# 3. Geometry of the measurement set-up (nominal pose)
# ---------------------------------------------------------------------------
BALL_D = 42.67
BALL_R = BALL_D / 2.0
BALL_DIST = 475.0         # horizontal distance lens pupil -> ball centre (450-500)
LENS_HEIGHT = 255.0       # optical axis height above the mat (200-300)
PITCH_NOM = 17.0          # pitch down (15-25)  nominal chosen by optimize_fov.py
PITCH_MIN = 15.0
PITCH_MAX = 25.0
BASELINE = 160.0          # stereo baseline, camera centre to centre (120-220)
AIM_U = 118.0             # down-range distance (+X world) between the ball and the beam
                          # centre (mean lens pupil); the mast stands directly in front of the ball
# Nominal toe-in of the two cameras (towards the beam centre), found with
# optimize_fov.py so that BOTH cameras keep the ball at rest and as much flight
# as possible in view.  The yaw stages allow +-YAW_RANGE around these values.
TOE_IN_R = 6.0            # camera at +X turns towards -X by this angle (deg)
TOE_IN_L = 9.0            # camera at -X turns towards +X by this angle (deg)
FLIGHT_COVER = 250.0      # down-range flight length that should be visible (target for the checks)
LAUNCH_MAX = 45.0         # launch elevation up to which coverage is requested

# ---------------------------------------------------------------------------
# 4. IMX296 camera board (third-party MIPI module)                     ALL VERIFY
# ---------------------------------------------------------------------------
CAM_PCB = 38.0            # square board, mm
CAM_PCB_T = 1.6
CAM_HOLE_SQ = 34.0        # M2.5 hole pattern (square)
CAM_HOLDER_W = 27.0       # CS-mount holder body, square side (must stay < ~28 for the 34 mm hole square)
CAM_HOLDER_H = 11.0       # PCB front face -> lens flange face (CS: 12.5 from sensor)
CAM_REAR_H = 5.0          # tallest component behind the PCB (FPC connector)
CAM_CONN_W = 17.0         # FPC connector length along the board edge
CAM_CONN_D = 5.0          # connector depth into the board
CAM_FPC_LEN = 200.0       # ribbon length available                      VERIFY
CAM_FPC_EDGE = "top"      # modelled: ribbon leaves the TOP edge; if your board
                          # has it at the bottom, mount the board upside down
                          # (image rotated 180 deg in software)
LENS_D = 35.0             # max lens barrel diameter                      VERIFY
LENS_L = 45.0             # max lens length from the flange face          VERIFY
LENS_PUPIL = 28.0         # entrance pupil position ahead of the flange   VERIFY
LENS_MASS = 90.0          # g
CAM_MASS = 35.0           # g (board + holder)
FOCAL = 8.0               # mm lens focal length
SENSOR_W = 5.02           # IMX296: 1456 x 3.45 um
SENSOR_H = 3.75
FOV_MARGIN_DEG = 1.0      # extra clear angle around the nominal field of view

FILTER_D = 25.0           # 850 nm band-pass filter                       VERIFY
FILTER_T = 2.0                                                          # VERIFY
FILTER_APERTURE = 23.0    # clear aperture left by the holder lip
FILTER_GAP = 1.0          # lens front -> filter holder roof

# ---------------------------------------------------------------------------
# 5. IR LED board                                                     VERIFY
# ---------------------------------------------------------------------------
# Two LED side boards (electronics README): each = 18 star MCPCBs (20 mm) with 60 deg TIR lenses
# on an aluminium plate, placed at +-LED_X from the camera-pair centre line, axes parallel to the cameras.
LED_PLATE = (110.0, 90.0, 3.0)    # aluminium plate X x Z x thickness                  VERIFY
LED_PLATE_HOLE_INSET = 5.0        # M3 holes at the plate corners, from the edges      VERIFY
LED_STAR_D = 20.0                 # star MCPCB diameter (= TIR lens footprint)         VERIFY
LED_STAR_T = 1.6                  # star board thickness                               VERIFY
LED_LENS_H = 15.0                 # TIR lens height above the star (8-15 mm)           VERIFY
LED_GRID = (5, 4)                 # star positions (x, z), pitch = LED_STAR_D
LED_SKIP = ((0, 0), (4, 0))       # the two unused positions (18 of 20 used)
LED_HALF_ANGLE = 100.0            # emission half angle assumed by the line-of-sight check (conservative: lens skirt leakage beyond 90 deg)
LED_X = 140.0                     # board centre distance from the centre line (140 nominal, 250 = wing)
LED_ZC = -26.0                    # board centre height in BF (below the lens axis: the hoods use the space beside the lenses)
LED_MASS = 210.0                  # g: plate + 18 stars + 18 lenses
DIFFUSER_T = 1.5                  # IR-passing diffuser sheet thickness                VERIFY
LED_FRAME_FRONT_CLEAR = 4.0       # frame front face to the shield posts
BAFFLE_WING_T = 3.0
MOD = (90.0, 70.0, 1.6)           # strobe module board X x Z x thickness              VERIFY
MOD_HOLE_INSET = 4.0                                                                  # VERIFY
MOD_PARTS_H = 25.0                # tallest part (2200 uF capacitor), parts face the beam   VERIFY
MOD_MASS = 120.0
MOD_STANDOFF = 6.0
LED_WIRE_MAX = 150.0              # max wire length module -> LED board (electronics README)

# ---------------------------------------------------------------------------
# 6. Computer boards                                                   VERIFY
# ---------------------------------------------------------------------------
PI_PCB = (85.0, 56.0, 1.6)
PI_HOLES = (58.0, 49.0)          # M2.5 pattern
PI_HOLE_EDGE = 3.5               # hole centre from the left / bottom edge
PI_COOLER_H = 17.0               # active cooler above the PCB top face
PI_COOLER = (38.0, 30.0)         # footprint (x, z) of the cooler
PI_CSI = ((54.0, 0.0), (70.0, 0.0))   # CSI connector centres (x from left edge, y from bottom edge) VERIFY
PI_MASS = 70.0
CTRL = (160.0, 100.0, 1.6)       # control board X x Z x thickness (Pico socket at its +X end, USB facing the Pi)  VERIFY
CTRL_HOLE_INSET = 4.0                                                                  # VERIFY
CTRL_COMP_H = 20.0               # tallest component on the control board             VERIFY
CTRL_MASS = 110.0
PICO_PCB = (51.0, 21.0, 1.0)
PICO_HEADER_H = 9.0              # pin headers + USB above the control board          VERIFY
PICO_MASS = 6.0
JACK_D = 11.0                    # 12 V barrel-jack panel hole                        VERIFY
SWITCH_D = 12.0                  # interlock switch panel hole                        VERIFY
ARMED_LED_D = 5.1                # front-visible ARMED indicator (5 mm LED)            VERIFY
MIC_BOARD = (20.0, 20.0, 1.6)    # small microphone board                              VERIFY
MIC_FROM_BALL = 150.0            # microphone distance from the ball (<= 200)
SENSOR_HEAD = (12.0, 12.0, 20.0) # optical sensor head body (width, height, length)    VERIFY
SENSOR_GAP = 15.0                # sensor face to ball surface (10-20)
BOARD_STANDOFF = 8.0             # printed standoff height under every board
REAR_PLATE_Y = -40.0             # plane of the rear plates in BF (front face)
PLATE_T = 4.0
# positions of the rear plates along the beam (BF x) and height of their centre (BF z)
PI_PLATE_X, PI_PLATE_Z = 12.0, 14.0       # Pi 5 (near the camera centre: short ribbons)
CTRL_PLATE_X, CTRL_PLATE_Z = -144.5, 0.0  # control board, left of the Pi (Pico USB faces the Pi)
RJ_X = 130.0                              # rear jack / interlock panel arm on the rear slot
AL_X = 70.0                               # ARMED LED holder on the front slot
CLIP_X = (-70.0, 110.0, -150.0, 150.0)     # cable clips on the bottom slot

# ---------------------------------------------------------------------------
# 7. Camera stage (clamp base -> yaw stage -> camera plate)
# ---------------------------------------------------------------------------
CB_T = 10.0                      # clamp base thickness above the extrusion
CB_HALF_L = 52.0                 # clamp base half length along the beam
CB_HALF_W = 16.0                 # clamp base half width (Y)
CB_SCREW_X = 44.0                # M5 clamp screws, from the pivot
YS_T = 6.0                       # yaw plate thickness
YS_LOCK_R = 18.0                 # yaw lock screws (M3) at +-R from the pivot
YAW_RANGE = 5.0                  # +- yaw adjustment about the toed-in nominal
TILT_RANGE = 4.0                 # +- tilt adjustment
TILT_LOCK_Z = 12.0               # tilt lock screws at +-Z from the hinge axis
CP_W = 54.0                      # camera plate size
CP_H = 54.0
BOSS_H = 1.5                     # PCB seating bosses behind the camera plate
HOLDER_RECESS = -0.5             # holder flange protrudes this far ahead of the plate front (lens barrel is wider than the window)
CHEEK_T = 6.0
CHEEK_GAP = 0.4                  # between plate and cheek
L_ARM = 38.0                     # pivot -> camera plate front face (along the axis)
CP_GAP_Z = 6.0                   # yaw plate top -> camera plate bottom (tilt swing)
FH_FLANGE = 53.0                 # filter housing mounting flange (square)
FH_FLANGE_T = 6.0
FH_SCREW = 21.0                  # filter housing screw pattern (+-)
FH_BORE_CLR = 0.8                # radial clearance bore vs lens barrel
FH_RING_SCREW_R = 22.8           # retaining ring screws (45 deg)
FILTER_SEAT_CLR = 0.2            # radial clearance filter in seat

# ---------------------------------------------------------------------------
# 8. Pitch pivot, mast, hub, feet
# ---------------------------------------------------------------------------
DRUM_R = 32.0                    # pitch drum radius
DRUM_FLAT_HALF = 14.0            # half width of the flat on top
DRUM_L = 50.0                    # drum length along X
EAR_T = 8.0                      # ear thickness
EAR_GAP = 0.6                    # ear <-> drum
EAR_R = 27.0                     # ear disc radius (< distance pivot -> beam bottom)
EAR_STEM = 22.0                  # half width of the ear below the disc
PIV_LOCK_R = 16.0                # pitch lock bolts (M4) radius
PIV_RANGE = (-3.0, 30.0)         # pitch angle range the arc slots allow
PIV_PINS = ((15.0, 60.0), (20.0, 90.0), (25.0, 120.0))   # (pitch deg, ear angle deg) of the M4 detent-pin holes
PIV_PIN_R = 12.0                 # radius of the detent-pin holes
BUBBLE_D = 15.0                  # round bubble level glued on the hub lug   VERIFY
PV_T = 6.0                       # ear base plate thickness
MC_FLANGE_T = 8.0                # collar flange thickness
MC_COLLAR_H = 26.0               # mast collar height
MC_WALL = 3.5
TOWER_H = 44.0                   # hub tower height (mast clamp)
TOWER_WALL = 6.0
HUB_ARM_H = 30.0                 # hub arm height (leg 20 + 2 x 7)
HUB_ARM_W = 34.0
HUB_ARM_L = 70.0                 # arm length from the hub centre
LEG_SOCKET_DEPTH = 44.0          # leg inserted into the hub
FB_SOCKET_DEPTH = 26.0           # leg inserted into the foot bracket
FB_BOLT_FROM_BRACKET_END = 15.0  # foot bolt axis from the far end of the bracket
LEG_ANGLES = (25.0, 155.0, 270.0)  # leg directions in HF (from +X, CCW)
FOOT_BOLT_L = 60.0               # M6 x 60 hex-head bolts
FOOT_PAD_D = 36.0
FOOT_PAD_T = 10.0
RUBBER_D = 20.0                  # adhesive rubber foot                  VERIFY
RUBBER_T = 3.0
RUBBER_RECESS = 1.5
CUP_CLR = 0.1                    # ball cup clearance (ball centre ends ~0.15 mm low)
FOOT_GAP_NOM = 14.0              # nominal gap bracket <-> pad (adjusts 0-30)
LUG_Y = 72.0                     # hub lug (template docking) from the mast axis
STABILITY_MARGIN = 60.0          # required COM distance inside the foot triangle

# ---------------------------------------------------------------------------
# 9. Shield, sight, template
# ---------------------------------------------------------------------------
SHIELD_H = 110.0                 # polycarbonate sheet height (Z)            VERIFY
SHIELD_T = 3.0                   # sheet thickness                           VERIFY
# sheet width is derived from the arm spacing (see below)
SHIELD_CLEAR = 8.0               # sheet back face -> filter retaining ring front
SHIELD_ARM_OUT = 70.0            # shield arm clamp centre = BASELINE/2 + this
SIGHT_H = 42.0
CARD = (3.0, 120.0)              # calibration card holder: thickness x width  VERIFY

# ---------------------------------------------------------------------------
# 10. Proxy masses (g) for the centre-of-mass check
# ---------------------------------------------------------------------------
MASS = dict(shield_per_cm3=1.2, ball=45.9)

# ---------------------------------------------------------------------------
# Optional overrides for parameter sweeps:  RIG_OVERRIDES='{"BASELINE": 120}'
# ---------------------------------------------------------------------------
import json as _json
import os as _os
for _k, _v in _json.loads(_os.environ.get("RIG_OVERRIDES", "{}")).items():
    globals()[_k] = _v

# ===========================================================================
# Derived values -- do not edit below this line
# ===========================================================================
Z_BEAM_TOP = EXT / 2.0
Z_BEAM_BOT = -EXT / 2.0
Y_BEAM_FRONT = EXT / 2.0
Y_BEAM_REAR = -EXT / 2.0

# camera stage (KP frame: z = 0 on top face of clamp base)
CP_T = max(9.0, CAM_HOLDER_H - BOSS_H + HOLDER_RECESS)
Z_AX = YS_T + CP_GAP_Z + CP_H / 2.0          # optical axis height above clamp base top
Z_CB_TOP = Z_BEAM_TOP + CB_T                 # clamp base top face (BF z)
CAM_Z_AXIS = Z_CB_TOP + Z_AX                 # optical axis height in BF
CP_FRONT_Y = L_ARM
CP_BACK_Y = L_ARM - CP_T
PCB_FRONT_Y = CP_BACK_Y - BOSS_H
PCB_BACK_Y = PCB_FRONT_Y - CAM_PCB_T
HOLDER_FRONT_Y = PCB_FRONT_Y + CAM_HOLDER_H
LENS_FRONT_Y = HOLDER_FRONT_Y + LENS_L
PUPIL_Y = HOLDER_FRONT_Y + LENS_PUPIL
FH_BORE_R = max(LENS_D / 2.0 + FH_BORE_CLR, CAM_HOLDER_W * math.sqrt(2.0) / 2.0 + 0.6)   # also clears the holder corners
FH_TUBE_R = FH_BORE_R + WALL
FH_ROOF_Y = LENS_FRONT_Y + FILTER_GAP          # inside face of the roof (KP y)
SEAT_DEPTH = FILTER_T + 0.1
FH_FRONT_Y = FH_ROOF_Y + MIN_WALL + SEAT_DEPTH   # front face of the filter housing
FR_T = 4.0
FR_FRONT_Y = FH_FRONT_Y + FR_T
FH_DISC_R = FILTER_D / 2.0 + 16.0               # ring / front disc radius


# pitch pivot (BF)
Z_PIVOT = Z_BEAM_BOT - math.sqrt(DRUM_R ** 2 - DRUM_FLAT_HALF ** 2)
PV_PLATE_TOP_PF = -(DRUM_R + 1.0)                # top face of the ear base plate (PF z)
PV_BOTTOM_PF = PV_PLATE_TOP_PF - PV_T
MC_FLANGE_BOTTOM_PF = PV_BOTTOM_PF - MC_FLANGE_T # collar interior ceiling = mast top
SHIELD_X_ARM = BASELINE / 2.0 + SHIELD_ARM_OUT   # shield arm clamp centre
SHIELD_W = 2.0 * (SHIELD_X_ARM + 11.0)            # sheet width follows the arm spacing
LE_ARM_X = 58.0                                  # LED arm clamp, outboard of the board centre
BEAM_HALF = max(BEAM_HALF_MIN, SHIELD_X_ARM + 70.0, LED_X + LE_ARM_X + 14.0 + 12.0)
BEAM_X0, BEAM_X1 = -BEAM_HALF, BEAM_HALF
BEAM_LEN = BEAM_X1 - BEAM_X0
SIGHT_X = (-(BEAM_HALF - 20.0), BEAM_HALF - 20.0)  # sight blades (rear / front) along the beam

# hub / feet
FB_LEN = FB_SOCKET_DEPTH + 30.0
FOOT_R = LEG_LEN + FB_LEN - FB_BOLT_FROM_BRACKET_END   # foot bolt axis radius from the mast axis
Z_PAD_TOP = FOOT_PAD_T + (RUBBER_T - RUBBER_RECESS)   # rubber foot protrudes below the pad
