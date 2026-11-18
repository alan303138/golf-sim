"""Single source of truth for the phase-1 golf-sim electronics.

Everything else (netlist.yaml / netlist.csv, BOM.csv, the schematic labels, the
checker) is derived from or verified against this file.

Board partitioning
  * CONTROL board (perfboard, refs 1..99): Pico socket, power input and
    protection, camera-trigger level shifter, strobe gating / interlock,
    shot-trigger front ends (acoustic + optical), connectors, test points.
  * STROBE module A and B (refs 101.. and 201..): capacitor bank, MOSFET,
    gate driver, ballast resistors and the LED strings of one LED side board.
    The two modules are identical; B = A + 100 in the reference number.

Cables are modelled as ideal wires: a connector on each side of a cable is put
on the same net name (e.g. J2 on the control board and J102 on the module are
both on V12_A).  Cable inductance is handled in the simulations.

Every value that appears in a drawing is read from here.
"""
from __future__ import annotations

NC = "NC"

# --------------------------------------------------------------------------
# Catalogue: key -> (value, description, package, part number or 'generic',
#                    unit cost USD (rough, 2026 hobby quantities), notes)
# Part numbers are given only where the author is confident of the family;
# the suffix / package variant must still be confirmed at order time.
# --------------------------------------------------------------------------
CAT = {
    "pico": ("Raspberry Pi Pico", "RP2040 timing MCU board, 3.3 V logic",
             "2x20 DIP module on 2x 1x20 sockets", "SC0915 (verify)", 4.0,
             "Use female sockets so the Pico can be removed; USB powered"),
    "screw2": ("2P screw terminal 5.08", "PCB screw terminal block, 2 pos, 5.08 mm, >=8 A",
               "5.08 mm pitch", "generic", 0.5, ""),
    "screw6": ("6P screw terminal 5.08", "PCB screw terminal block, 6 pos, 5.08 mm (2x 3P stacked)",
               "5.08 mm pitch", "generic", 1.4, "Rated >=3 A per pole"),
    "hdr2": ("2P header 2.54", "pin header 1x2, 2.54 mm", "2.54 mm", "generic", 0.1, ""),
    "hdr3": ("3P header 2.54", "pin header 1x3, 2.54 mm (jumper select)", "2.54 mm", "generic", 0.1, ""),
    "hdr5": ("5P header 2.54", "pin header 1x5, 2.54 mm", "2.54 mm", "generic", 0.15, ""),
    "fuse": ("2 A time-lag", "fuse 5x20 mm, 2 A slow-blow, in PCB holder", "5x20 mm", "generic", 0.8,
             "Holder + fuse; 250 V AC rated is fine for 12 V DC"),
    "d_1n5822": ("1N5822", "Schottky 3 A 40 V", "DO-201AD axial", "1N5822", 0.25, ""),
    "d_1n5819": ("1N5819", "Schottky 1 A 40 V", "DO-41 axial", "1N5819", 0.1, ""),
    "tvs": ("1.5KE15A", "TVS 15 V (Vwm 12.8 V) 1500 W unidirectional", "DO-201 axial", "1.5KE15A", 0.5, "Vc ~21 V max"),
    "zener15": ("1N4744A", "Zener 15 V 1 W", "DO-41 axial", "1N4744A", 0.1, "Protects gate driver supply"),
    "bat85": ("BAT85", "small-signal Schottky 30 V", "DO-34 axial", "BAT85", 0.1, "BAT43/1N5711 are equivalents"),
    "bat54s": ("BAT54S", "dual series Schottky 30 V", "SOT-23 on adapter", "BAT54S", 0.15, "Needs SOT-23 breakout board"),
    "led_red": ("LED red 5 mm", "red LED 5 mm, ~10 mA", "5 mm", "generic", 0.1, ""),
    "led_grn": ("LED green 3 mm", "green LED 3 mm, ~2 mA", "3 mm", "generic", 0.1, ""),
    "led_ir850": ("850 nm IR LED on 20 mm star", "850 nm power IR LED, ~1 mm2 chip, 3535-class, 1 A rated, "
                  "pulse capability >=3 A at tp=20 us D=2 % MUST be verified, on 20 mm aluminium star MCPCB",
                  "3535 SMD on 20 mm star", "generic (OSRAM Oslon-Black 850 nm family, e.g. SFH 4715AS - verify)",
                  2.8, "Verify datasheet pulse-handling graph at 20 us / 2 %; verify wavelength suffix"),
    "q_irlb8721": ("IRLB8721", "N-MOSFET 30 V 62 A logic-level, Qg~8-17 nC", "TO-220", "IRLB8721PBF", 1.2, "Verify Qg, Vgs(th) on datasheet"),
    "q_2n3904": ("2N3904", "NPN small-signal 200 mA", "TO-92", "2N3904", 0.08, "Pin order EBC for most makers - verify"),
    "tc4427": ("TC4427A", "dual 1.5 A MOSFET driver (paralleled), 4.5-18 V, TTL/CMOS input",
               "DIP-8", "TC4427AEPA (verify suffix)", 1.6, "Verify Vih <= 2.4 V for 3.3 V drive"),
    "hc08": ("74HC08", "quad 2-input AND, 3.3 V", "DIP-14", "SN74HC08N", 0.5, ""),
    "hc86": ("74HC86", "quad XOR, 3.3 V", "DIP-14", "SN74HC86N", 0.5, ""),
    "lvc2g17": ("SN74LVC2G17", "dual Schmitt buffer, 1.65-5.5 V, 5 V tolerant inputs",
                "SOT-23-6 on adapter", "SN74LVC2G17DBVR", 0.5, "Verify pin-out vs adapter"),
    "ldo18": ("MCP1700-1802", "1.8 V LDO 250 mA, 1 uF caps", "TO-92", "MCP1700-1802E/TO", 0.4,
              "TO-92 pin order GND/VIN/VOUT - verify"),
    "mcp6022": ("MCP6022", "dual rail-to-rail op-amp 10 MHz, 2.5-5.5 V", "DIP-8", "MCP6022-I/P", 0.9, ""),
    "lm339": ("LM339", "quad comparator, open-collector", "DIP-14", "LM339N", 0.5, ""),
    "tlc555": ("TLC555", "CMOS 555 timer", "DIP-8", "TLC555CP", 0.9, "Any CMOS 555 (LMC555, TLC555) works"),
    "ptc": ("pot 10k", "trimmer potentiometer 10 kOhm, multi-turn", "3296W style", "generic", 0.8, ""),
    "pot500": ("trimmer 500k", "trimmer potentiometer 500 kOhm", "3296W style", "generic", 0.8, ""),
    "mic": ("electret capsule", "electret microphone capsule 6 mm, 2-wire, >=-44 dBV/Pa, response to >=15 kHz",
            "6 mm capsule", "generic (verify bandwidth)", 0.8, "Alternative: analogue MEMS mic on breakout (verify)"),
    "pd_ir": ("IR photodiode", "5 mm photodiode with daylight/IR filter, 850-950 nm, Cj<=15 pF, e.g. SFH 213 FA class",
              "5 mm T-1 3/4", "generic (SFH 213 FA class - verify)", 0.8, ""),
    "led_ir940": ("940 nm IR LED 5 mm", "5 mm 940 nm IR emitter, 100 mA, e.g. TSAL6100 class", "5 mm",
                  "generic (TSAL6100 class - verify)", 0.4,
                  "940 nm so the 850 nm camera filter and the 850 nm strobe do not see it"),
    "tp": ("test point", "test point loop / pin", "1 pin", "generic", 0.05, ""),
}


def _passives():
    # value, description, package, cost
    return {}


def cat_entry(key):
    return CAT[key]


PARTS: dict = {}   # ref -> dict
ORDER: list = []


def add(ref, key, sheet, pins, value=None, desc=None, package=None, mpn=None, cost=None, notes=None):
    """pins: dict pin -> net | (net, type) | (name, net, type)"""
    assert ref not in PARTS, f"duplicate ref {ref}"
    if key in CAT:
        v, d, p, m, c, n = CAT[key]
    else:
        v, d, p, m, c, n = (None, None, None, "generic", 0.03, "")
    pp = {}
    for pin, spec in pins.items():
        if isinstance(spec, str):
            name, net, typ = "", spec, "passive"
        elif len(spec) == 2:
            name, net, typ = "", spec[0], spec[1]
        else:
            name, net, typ = spec
        if net == NC:
            typ = "nc"
        pp[str(pin)] = {"name": name, "net": net, "type": typ}
    PARTS[ref] = dict(ref=ref, key=key, sheet=sheet,
                      value=value or v, description=desc or d, package=package or p,
                      mpn=mpn or m, cost=cost if cost is not None else c,
                      notes=notes if notes is not None else n, pins=pp)
    ORDER.append(ref)


def r(ref, sheet, n1, n2, value, pkg="axial 1/4 W", cost=0.03, desc=None, notes=""):
    add(ref, "R", sheet, {1: n1, 2: n2}, value=value, desc=desc or f"resistor {value}",
        package=pkg, cost=cost, notes=notes)


def c(ref, sheet, n1, n2, value, pkg="radial MLCC / film 5 mm", cost=0.08, desc=None, notes="", polar=False):
    add(ref, "C", sheet, {1: n1, 2: n2}, value=value, desc=desc or f"capacitor {value}", package=pkg, cost=cost, notes=notes)


def tp(ref, sheet, net):
    add(ref, "tp", sheet, {1: net})


# ==========================================================================
# CONTROL BOARD
# ==========================================================================
# ---- power input, protection, grounding, supplies ('power') --------------
S = "power"
add("J1", "screw2", S, {1: ("+", "VIN_RAW", "pwr_out"), 2: ("-", "PGND", "passive")}, notes="12 V DC input, 2 A")
add("F1", "fuse", S, {1: "VIN_RAW", 2: "VIN_F"})
add("D1", "d_1n5822", S, {"A": "VIN_F", "K": ("K", "V12", "pwr_out")}, notes="reverse-polarity block")
add("D2", "tvs", S, {"K": "V12", "A": "PGND"})
c("C1", S, "V12", "PGND", "470u 25V", "radial 8x12", 0.3, "bulk electrolytic, low ESR")
c("C2", S, "V12", "PGND", "100n", cost=0.05)
add("JP5", "hdr2", S, {1: "PGND", 2: "GND_L"}, notes="STAR link: the only connection between strobe return (PGND) and logic ground (GND_L)")
# charge-limiting resistors: 2 x 11 R 5 W in parallel = 5.5 R per module
r("R1", S, "V12", "V12_A", "11R 5W", "cement 5 W", 0.5, "charge-limit resistor (half of 5.5 R)")
r("R2", S, "V12", "V12_A", "11R 5W", "cement 5 W", 0.5, "charge-limit resistor (half of 5.5 R)")
r("R3", S, "V12", "V12_B", "11R 5W", "cement 5 W", 0.5, "charge-limit resistor (half of 5.5 R)")
r("R4", S, "V12", "V12_B", "11R 5W", "cement 5 W", 0.5, "charge-limit resistor (half of 5.5 R)")
# analogue and emitter supplies from USB 5 V
r("R5", S, "V5", "V5A", "22R", notes="analogue 5 V filter")
c("C4", S, "V5A", "GND_L", "100u 10V", "radial 5x11", 0.1)
c("C5", S, "V5A", "GND_L", "100n", cost=0.05)
r("R6", S, "V5", "V5E", "4R7", notes="emitter 5 V filter")
c("C6", S, "V5E", "GND_L", "100u 10V", "radial 5x11", 0.1)
c("C7", S, "V5E", "GND_L", "100n", cost=0.05)
c("C8", S, "3V3", "GND_L", "10u 16V", "radial 5x11", 0.1)
c("C9", S, "3V3", "GND_L", "100n", cost=0.05)
c("C10", S, "V5", "GND_L", "10u 16V", "radial 5x11", 0.1)
# rail sense for GP28
add("JP6", "hdr3", S, {1: "SENSE_A", 2: "RAIL_SEL", 3: "SENSE_B"}, notes="1-2 senses module A bank (default), 2-3 module B")
r("R7", S, "RAIL_SEL", "RAIL_SENSE", "33k 1%", cost=0.03)
r("R8", S, "RAIL_SENSE", "GND_L", "10k 1%", cost=0.03)
c("C11", S, "RAIL_SENSE", "GND_L", "100n", cost=0.05)
add("D4", "bat85", S, {"A": "RAIL_SENSE", "K": "3V3"}, notes="clamps ADC pin")
tp("TP1", S, "V12"); tp("TP2", S, "PGND"); tp("TP3", S, "3V3"); tp("TP4", S, "V5"); tp("TP5", S, "GND_L")
tp("TP6", S, "RAIL_SENSE")

# ---- Pico socket ----------------------------------------------------------
S = "pico"
_pico = {
    1: ("GP0", NC, "bidir"), 2: ("GP1", NC, "bidir"), 3: ("GND", "GND_L", "passive"),
    4: ("GP2", "CAM_TRIG", "out"), 5: ("GP3", "STROBE_GATE", "out"), 6: ("GP4", "STROBE_EN", "in"),
    7: ("GP5", "TRIG_IN", "in"), 8: ("GND", "GND_L", "passive"), 9: ("GP6", "STATUS", "out"),
    10: ("GP7", NC, "bidir"), 11: ("GP8", NC, "bidir"), 12: ("GP9", NC, "bidir"), 13: ("GND", "GND_L", "passive"),
    14: ("GP10", NC, "bidir"), 15: ("GP11", NC, "bidir"), 16: ("GP12", NC, "bidir"), 17: ("GP13", NC, "bidir"),
    18: ("GND", "GND_L", "passive"), 19: ("GP14", NC, "bidir"), 20: ("GP15", NC, "bidir"),
    21: ("GP16", NC, "bidir"), 22: ("GP17", NC, "bidir"), 23: ("GND", "GND_L", "passive"),
    24: ("GP18", NC, "bidir"), 25: ("GP19", NC, "bidir"), 26: ("GP20", NC, "bidir"), 27: ("GP21", NC, "bidir"),
    28: ("GND", "GND_L", "passive"), 29: ("GP22", NC, "bidir"), 30: ("RUN", NC, "in"),
    31: ("GP26/ADC0", NC, "bidir"), 32: ("GP27/ADC1", NC, "bidir"), 33: ("AGND", "GND_L", "passive"),
    34: ("GP28/ADC2", "RAIL_SENSE", "in"), 35: ("ADC_VREF", NC, "passive"), 36: ("3V3(OUT)", "3V3", "pwr_out"),
    37: ("3V3_EN", NC, "in"), 38: ("GND", "GND_L", "passive"), 39: ("VSYS", NC, "passive"),
    40: ("VBUS", "V5", "pwr_out"),
}
add("U1", "pico", S, _pico)

# ---- camera trigger interface ('camtrig') ------------------------------------
S = "camtrig"
add("U3", "lvc2g17", S, {1: ("1A", "CAMTRIG_IN", "in"), 2: ("GND", "GND_L", "passive"),
                          3: ("2A", "CAMTRIG_IN", "in"), 4: ("2Y", "CAM2_DRV", "out"),
                          5: ("VCC", "VTRIG", "pwr_in"), 6: ("1Y", "CAM1_DRV", "out")})
add("U4", "ldo18", S, {1: ("GND", "GND_L", "passive"), 2: ("VIN", "3V3", "pwr_in"), 3: ("VOUT", "V1P8", "pwr_out")})
c("C12", S, "3V3", "GND_L", "1u X7R", "radial MLCC", 0.1)
c("C13", S, "V1P8", "GND_L", "1u X7R", "radial MLCC", 0.1)
c("C14", S, "VTRIG", "GND_L", "100n", cost=0.05)
c("C15", S, "VTRIG", "GND_L", "10u 16V", "radial 5x11", 0.1)
add("JP1", "hdr3", S, {1: "V1P8", 2: "VTRIG", 3: "3V3"},
    notes="1-2 = 1.8 V trigger level (DEFAULT/SAFE); 2-3 = 3.3 V; NO jumper = buffer unpowered")
r("R10", S, "CAM_TRIG", "CAMTRIG_IN", "100R", notes="series damping from Pico GP2")
r("R11", S, "CAMTRIG_IN", "3V3", "10k", notes="idle-high pull-up (camera trigger is active low)")
r("R12", S, "CAM1_DRV", "CAM1_OUT", "47R", notes="series protection / edge damping")
r("R13", S, "CAM2_DRV", "CAM2_OUT", "47R", notes="series protection / edge damping")
add("D5", "bat54s", S, {1: "GND_L", 2: "VTRIG", 3: "CAM1_OUT"}, notes="clamps CAM1 line to -0.3 V .. VTRIG+0.3 V")
add("D6", "bat54s", S, {1: "GND_L", 2: "VTRIG", 3: "CAM2_OUT"}, notes="clamps CAM2 line to -0.3 V .. VTRIG+0.3 V")
add("J6", "hdr2", S, {1: "CAM1_OUT", 2: "GND_L"}, notes="to camera 1 external-trigger pad + its GND (verify module manual)")
add("J7", "hdr2", S, {1: "CAM2_OUT", 2: "GND_L"}, notes="to camera 2 external-trigger pad + its GND (verify module manual)")
tp("TP7", S, "CAMTRIG_IN"); tp("TP8", S, "CAM1_OUT"); tp("TP9", S, "VTRIG")

# ---- strobe gating, interlock, indicators ('gating') -----------------------------
S = "gating"
add("U2", "hc08", S, {1: ("1A", "GATE_RAW", "in"), 2: ("1B", "EN_SW", "in"), 3: ("1Y", "GATE_BUS", "out"),
                       4: ("2A", "GND_L", "in"), 5: ("2B", "GND_L", "in"), 6: ("2Y", NC, "out"),
                       7: ("GND", "GND_L", "passive"),
                       8: ("3Y", NC, "out"), 9: ("3A", "GND_L", "in"), 10: ("3B", "GND_L", "in"),
                       11: ("4Y", NC, "out"), 12: ("4A", "GND_L", "in"), 13: ("4B", "GND_L", "in"),
                       14: ("VCC", "3V3", "pwr_in")})
r("R14", S, "STROBE_GATE", "GATE_RAW", "100R", notes="series damping from Pico GP3")
r("R15", S, "GATE_RAW", "GND_L", "10k", notes="keeps gate LOW while Pico is in reset/boot")
r("R16", S, "3V3", "ILK_SRC", "100R")
add("J8", "hdr2", S, {1: "ILK_SRC", 2: "EN_SW"}, notes="interlock loop: switch closed = strobe enabled; open wire = disabled")
r("R17", S, "EN_SW", "GND_L", "100k", notes="open loop = LOW = disabled")
c("C16", S, "EN_SW", "GND_L", "10n", cost=0.05, notes="RC ~1 ms debounce")
r("R18", S, "EN_SW", "STROBE_EN", "1k", notes="series protection to Pico GP4")
r("R19", S, "EN_SW", "ARM_B", "4k7")
add("Q2", "q_2n3904", S, {1: ("E", "GND_L", "passive"), 2: ("B", "ARM_B", "in"), 3: ("C", "ARM_K", "passive")})
r("R20", S, "V12", "ARM_A", "1k", notes="~9 mA through ARMED LED")
add("D7", "led_red", S, {"A": "ARM_A", "K": "ARM_K"}, notes="ARMED indicator: lit only when interlock closed AND 12 V present (hardware, not firmware)")
add("J9", "hdr2", S, {1: "ARM_A", 2: "ARM_K"}, notes="optional second ARMED LED (panel mount) in parallel with D7")
r("R21", S, "STATUS", "STATUS_LED_A", "470R")
add("D8", "led_grn", S, {"A": "STATUS_LED_A", "K": "GND_L"}, notes="Pico GP6 status")
r("R22", S, "GATE_BUS", "GATE_A", "33R", notes="line damping to module A")
r("R23", S, "GATE_BUS", "GATE_B", "33R", notes="line damping to module B")
add("J2", "screw2", S, {1: ("V12_A", "V12_A", "passive"), 2: ("PGND", "PGND", "passive")}, notes="to module A J102")
add("J3", "hdr3", S, {1: "GATE_A", 2: "PGND", 3: "SENSE_A"}, notes="to module A J103; twist GATE_A with PGND")
add("J4", "screw2", S, {1: ("V12_B", "V12_B", "passive"), 2: ("PGND", "PGND", "passive")}, notes="to module B J202")
add("J5", "hdr3", S, {1: "GATE_B", 2: "PGND", 3: "SENSE_B"}, notes="to module B J203; twist GATE_B with PGND")
tp("TP10", S, "GATE_BUS"); tp("TP11", S, "EN_SW"); tp("TP12", S, "STROBE_GATE")

# ---- shot trigger front ends --------------------------------------------------------
# common: reference + selection ('trigsel')
S = "trigsel"
r("R30", S, "V5A", "VREF_RAW", "10k 1%")
r("R31", S, "VREF_RAW", "GND_L", "10k 1%")
c("C17", S, "VREF_RAW", "GND_L", "10u 16V", "radial 5x11", 0.1)
def opamp8(ref, sheet, outa, ina_n, ina_p, inb_p, inb_n, outb, vdd="V5A", vss="GND_L"):
    add(ref, "mcp6022", sheet, {
        1: ("OUTA", outa, "out"), 2: ("INA-", ina_n, "in"), 3: ("INA+", ina_p, "in"), 4: ("VSS", vss, "passive"),
        5: ("INB+", inb_p, "in"), 6: ("INB-", inb_n, "in"), 7: ("OUTB", outb, "out"), 8: ("VDD", vdd, "pwr_in")})
# U7: A = VREF buffer, B = inverted copy of acoustic signal
opamp8("U7", S, "VREF", "VREF", "VREF_RAW", "VREF", "INV_FB", "ACOU_INV")
c("C18", S, "VREF", "GND_L", "1u X7R", "radial MLCC", 0.1)
c("C19", S, "V5A", "GND_L", "100n", cost=0.05, notes="U7 decoupling")
add("U8", "lm339", S, {
    1: ("OUT2", "ACOU_N", "oc"), 2: ("OUT1", "ACOU_N", "oc"), 3: ("VCC", "V5A", "pwr_in"),
    4: ("IN1-", "ACOU_SIG", "in"), 5: ("IN1+", "THR_P", "in"), 6: ("IN2-", "ACOU_INV", "in"),
    7: ("IN2+", "THR_P", "in"), 8: ("IN3-", "THO", "in"), 9: ("IN3+", "ENV", "in"),
    10: ("IN4-", "VREF", "in"), 11: ("IN4+", "GND_L", "in"), 12: ("GND", "GND_L", "passive"),
    13: ("OUT4", NC, "oc"), 14: ("OUT3", "OPT_N", "oc")})
c("C20", S, "V5A", "GND_L", "100n", cost=0.05, notes="U8 decoupling")
r("R44", S, "3V3", "ACOU_N", "10k")
r("R45", S, "3V3", "OPT_N", "10k")
add("JP2", "hdr3", S, {1: "ACOU_N", 2: "TRIG_RAW", 3: "OPT_N"}, notes="1-2 = acoustic, 2-3 = optical (no jumper: TRIG_IN idle)")
r("R60", S, "TRIG_RAW", "3V3", "100k", notes="idle-high when no jumper")
add("U10", "hc86", S, {1: ("1A", "TRIG_RAW", "in"), 2: ("1B", "POL", "in"), 3: ("1Y", "TRIG_X", "out"),
                        4: ("2A", "GND_L", "in"), 5: ("2B", "GND_L", "in"), 6: ("2Y", NC, "out"),
                        7: ("GND", "GND_L", "passive"),
                        8: ("3Y", NC, "out"), 9: ("3A", "GND_L", "in"), 10: ("3B", "GND_L", "in"),
                        11: ("4Y", NC, "out"), 12: ("4A", "GND_L", "in"), 13: ("4B", "GND_L", "in"),
                        14: ("VCC", "3V3", "pwr_in")})
c("C21", S, "3V3", "GND_L", "100n", cost=0.05, notes="U10 decoupling")
c("C22", S, "3V3", "GND_L", "100n", cost=0.05, notes="U2 decoupling")
c("C23", S, "3V3", "GND_L", "100n", cost=0.05, notes="3V3 local decoupling near U4 / pull-ups")
add("JP3", "hdr3", S, {1: "3V3", 2: "POL", 3: "GND_L"}, notes="1-2: TRIG_IN idle LOW, rising edge = shot (DEFAULT); 2-3: idle HIGH, falling edge = shot")
r("R61", S, "TRIG_X", "TRIG_IN", "100R", notes="series damping to Pico GP5")
r("R62", S, "TRIG_IN", "GND_L", "100k")
tp("TP13", S, "VREF"); tp("TP14", S, "TRIG_IN"); tp("TP15", S, "ACOU_N"); tp("TP16", S, "OPT_N")

# acoustic channel ('acoustic')
S = "acoustic"
r("R32", S, "MIC_B", "MIC_P", "4k7", notes="electret bias")
r("R33", S, "V5A", "MIC_B", "1k")
c("C24", S, "MIC_B", "GND_L", "10u 16V", "radial 5x11", 0.1)
add("M1", "mic", S, {1: ("OUT+", "MIC_P", "passive"), 2: ("-", "GND_L", "passive")})
add("J10", "hdr2", S, {1: "MIC_P", 2: "GND_L"}, notes="optional remote capsule (use shielded cable, <1 m)")
c("C25", S, "MIC_P", "MIC_AC", "100n", cost=0.05)
r("R34", S, "MIC_AC", "VREF", "100k")
opamp8("U5", S, "AMP1", "FB1", "MIC_AC", "VREF", "FB2", "ACOU_SIG")
r("R35", S, "AMP1", "FB1", "47k 1%")
c("C26", S, "AMP1", "FB1", "220p C0G", "radial", 0.1)
r("R36", S, "FB1", "FB1G", "1k 1%")
c("C27", S, "FB1G", "VREF", "100n", cost=0.05)
c("C28", S, "AMP1", "S2IN", "10n", cost=0.06)
r("R37", S, "S2IN", "FB2", "10k 1%")
r("R38", S, "FB2", "ACOU_SIG", "20k 1%")
c("C29", S, "FB2", "ACOU_SIG", "470p C0G", "radial", 0.1)
c("C30", S, "V5A", "GND_L", "100n", cost=0.05, notes="U5 decoupling")
r("R39", S, "ACOU_SIG", "INV_FB", "10k 1%")
r("R40", S, "INV_FB", "ACOU_INV", "10k 1%")
r("R41", S, "V5A", "THR_TOP", "16k 1%")
add("RV1", "ptc", S, {1: "THR_TOP", 2: "THR_P", 3: "THR_BOT"}, notes="acoustic threshold: ~2.55..3.49 V (Vref+0.05..+0.99 V)")
r("R42", S, "THR_BOT", "GND_L", "27k 1%")
c("C31", S, "THR_P", "GND_L", "100n", cost=0.05)
r("R43", S, "ACOU_N", "HYS_M", "100k", notes="hysteresis feedback (min)")
add("RV2", "pot500", S, {1: "HYS_M", 2: "THR_P", 3: "THR_P"}, notes="hysteresis trim, used as rheostat (wiper tied to pin 3)")
tp("TP17", S, "ACOU_SIG"); tp("TP18", S, "THR_P"); tp("TP19", S, "MIC_P")

# optical channel ('optical')
S = "optical"
add("J11", "hdr5", S, {1: ("LED_A", "EMIT_A", "passive"), 2: ("LED_K", "EMIT_K", "passive"),
                        3: ("PD_K", "PD_K", "passive"), 4: ("PD_A", "TIA_IN", "passive"), 5: ("SHIELD", "GND_L", "passive")},
    notes="remote sensor head: 940 nm LED + IR photodiode, twisted/shielded cable <1 m")
add("D10", "led_ir940", S, {"A": "EMIT_A", "K": "EMIT_K"}, notes="on sensor head (wired via J11)")
add("D11", "pd_ir", S, {"A": "TIA_IN", "K": "PD_K"}, notes="on sensor head (wired via J11)")
r("R47", S, "V5A", "PD_K", "100R")
c("C32", S, "PD_K", "GND_L", "4u7 16V", "radial 5x11", 0.1)
opamp8("U6", S, "TIA_OUT", "TIA_IN", "VREF", "VREF", "MFB_INV", "OPT_BP")
r("R48", S, "TIA_IN", "TIA_OUT", "33k 1%")
c("C33", S, "TIA_IN", "TIA_OUT", "4p7 C0G", "radial", 0.1)
r("R49", S, "TIA_OUT", "MFB_A", "1k58 1%")
r("R50", S, "MFB_A", "VREF", "332R 1%")
c("C34", S, "MFB_A", "MFB_INV", "1n C0G", "radial", 0.1)
c("C35", S, "MFB_A", "OPT_BP", "1n C0G", "radial", 0.1)
r("R51", S, "MFB_INV", "OPT_BP", "9k53 1%")
c("C36", S, "V5A", "GND_L", "100n", cost=0.05, notes="U6 decoupling")
add("D9", "bat85", S, {"A": "OPT_BP", "K": "ENV"}, notes="envelope detector")
c("C37", S, "ENV", "VREF", "2n2 C0G", "radial", 0.1)
r("R52", S, "ENV", "VREF", "10k")
r("R46", S, "OPT_N", "ENV", "680k", notes="comparator hysteresis")
r("R53", S, "V5A", "THO_TOP", "16k 1%")
add("RV3", "ptc", S, {1: "THO_TOP", 2: "THO", 3: "THO_BOT"}, notes="optical threshold: ~2.55..3.49 V")
r("R54", S, "THO_BOT", "GND_L", "27k 1%")
c("C38", S, "THO", "GND_L", "100n", cost=0.05)
add("U9", "tlc555", S, {1: ("GND", "GND_L", "passive"), 2: ("TRIG", "OSC_C", "in"), 3: ("OUT", "OSC_OUT", "out"),
                         4: ("RESET", "OSC_EN", "in"), 5: ("CTRL", "OSC_CT", "passive"), 6: ("THR", "OSC_C", "in"),
                         7: ("DIS", "OSC_D", "passive"), 8: ("VCC", "V5E", "pwr_in")})
r("R55", S, "V5E", "OSC_D", "1k")
r("R56", S, "OSC_D", "OSC_C", "6k8")
c("C39", S, "OSC_C", "GND_L", "1n C0G", "radial", 0.1)
c("C40", S, "OSC_CT", "GND_L", "10n", cost=0.05)
r("R57", S, "OSC_EN", "GND_L", "100k")
add("JP4", "hdr2", S, {1: "OSC_EN", 2: "V5E"}, notes="install to enable the optical emitter (default open: emitter off)")
r("R58", S, "OSC_OUT", "EMIT_B", "270R")
add("Q3", "q_2n3904", S, {1: ("E", "GND_L", "passive"), 2: ("B", "EMIT_B", "in"), 3: ("C", "EMIT_K", "passive")})
r("R59", S, "V5E", "EMIT_A", "33R", notes="~95 mA emitter current")
c("C41", S, "V5E", "GND_L", "100n", cost=0.05, notes="U9 decoupling")
tp("TP20", S, "TIA_OUT"); tp("TP21", S, "OPT_BP"); tp("TP22", S, "ENV"); tp("TP23", S, "THO"); tp("TP24", S, "OSC_OUT")


# ==========================================================================
# STROBE MODULES  (A: refs 101.., B: refs 201..)
# ==========================================================================
def strobe_module(k, x):
    """k = 100 or 200; x = 'A' or 'B'."""
    S = "strobe"
    n = lambda s: f"{s}_{x}"
    V12, DRN, SRC = n("V12"), n("DRN"), n("SRC")
    add(f"J{k+2}", "screw2", S, {1: (V12, V12, "pwr_in"), 2: ("PGND", "PGND", "passive")}, notes=f"from control board J{2 if x=='A' else 4}")
    add(f"J{k+3}", "hdr3", S, {1: (n("GATE"), n("GATE"), "in"), 2: ("PGND", "PGND", "passive"),
                                3: (n("SENSE"), n("SENSE"), "passive")}, notes=f"from control board J{3 if x=='A' else 5}")
    add(f"J{k+4}", "screw6", S, {i: (f"STR_{x}{i}", f"STR_{x}{i}", "passive") for i in range(1, 7)},
        notes="LED string anode feeds (string 1..6; build strings 1..4 first = 24 LEDs total, add 5..6 = 36 LEDs)")
    add(f"J{k+5}", "screw6", S, {i: (DRN, DRN, "passive") for i in range(1, 7)},
        notes="LED string returns (all tied to MOSFET drain)")
    c(f"C{k+1}", S, V12, "PGND", "2200u 16V", "radial 12.5x25", 1.2, "low-ESR bank electrolytic (ESR <= 30 mohm, verify)")
    c(f"C{k+2}", S, V12, "PGND", "2200u 16V", "radial 12.5x25", 1.2, "low-ESR bank electrolytic (ESR <= 30 mohm, verify)")
    for i in range(3, 7):
        c(f"C{k+i}", S, V12, "PGND", "10u 25V X7R", "1210 SMD on adapter / radial", 0.35, "HF bank, as close to MOSFET/ballast as possible")
    c(f"C{k+7}", S, V12, "PGND", "100n", cost=0.05)
    for i in range(1, 7):
        r(f"RB{k+i}", S, V12, f"STR_{x}{i}", "1R8 3W", "metal-oxide 3 W axial", 0.35,
          "ballast / current-set; 3 A pulse -> 16 W for 20 us (verify pulse rating)")
    led = k
    for s in range(1, 7):
        a = f"STR_{x}{s}"
        n1, n2 = f"L{x}{s}_1", f"L{x}{s}_2"
        add(f"LD{led+1}", "led_ir850", S, {"A": a, "K": n1})
        add(f"LD{led+2}", "led_ir850", S, {"A": n1, "K": n2})
        add(f"LD{led+3}", "led_ir850", S, {"A": n2, "K": DRN})
        led += 3
    add(f"D{k+1}", "d_1n5822", S, {"A": DRN, "K": V12}, notes="flyback / drain clamp to rail")
    add(f"D{k+2}", "zener15", S, {"K": n("VDRV"), "A": SRC})
    add(f"D{k+3}", "d_1n5819", S, {"A": SRC, "K": n("GFET")}, notes="gate DC-restore clamp (AC-coupled gate)")
    add(f"D{k+4}", "led_red", S, {"A": n("CHGLED"), "K": "PGND"}, notes="BANK CHARGED indicator (lit whenever bank > ~3 V)")
    add(f"Q{k+1}", "q_irlb8721", S, {1: ("G", n("GFET"), "in"), 2: ("D", DRN, "passive"), 3: ("S", SRC, "passive")})
    r(f"RS{k+1}", S, SRC, "PGND", "10m 1W", "2512 SMD metal strip / axial 3 W", 0.6, "current sense, 12-18 A pulses (Kelvin: scope across TP)")
    add(f"U{k+1}", "tc4427", S, {1: ("NC", NC, "passive"), 2: ("INA", n("DRV_IN"), "in"), 3: ("GND", SRC, "passive"),
                                  4: ("INB", n("DRV_IN"), "in"), 5: ("OUTB", n("GOUT"), "out"), 6: ("VDD", n("VDRV"), "pwr_in"),
                                  7: ("OUTA", n("GOUT"), "out"), 8: ("NC", NC, "passive")})
    r(f"R{k+1}", S, V12, n("VDRV"), "10R", notes="driver supply filter")
    c(f"C{k+8}", S, n("VDRV"), SRC, "10u 25V X7R", "1210 / radial", 0.3)
    c(f"C{k+9}", S, n("VDRV"), SRC, "100n", cost=0.05)
    r(f"R{k+2}", S, n("GATE"), n("DRV_IN"), "100R", notes="input damping")
    r(f"R{k+3}", S, n("DRV_IN"), SRC, "10k", notes="pull-down: unplugged cable = off")
    r(f"R{k+4}", S, n("GOUT"), n("GACI"), "2R2", notes="gate resistor")
    c(f"C{k+10}", S, n("GACI"), n("GFET"), "10n C0G/film 50V", "radial 5 mm", 0.15,
      "gate AC-coupling: sets maximum on-time ~55 us")
    r(f"R{k+5}", S, n("GFET"), SRC, "3k3", notes="gate bias: discharges AC-coupling capacitor (tau ~ 40 us)")
    r(f"R{k+6}", S, V12, n("SENSE"), "100R", notes="rail sense feed to control board divider")
    r(f"R{k+7}", S, V12, n("CHGLED"), "2k2", notes="charged-LED resistor (~4.5 mA)")
    r(f"R{k+8}", S, V12, "PGND", "1k 0.5W", "axial 1/2 W", 0.05, "bleeder: tau = 4.4 s with 4.4 mF")
    tp(f"TP{k+1}", S, V12); tp(f"TP{k+2}", S, DRN); tp(f"TP{k+3}", S, SRC); tp(f"TP{k+4}", S, n("GFET")); tp(f"TP{k+5}", S, "PGND")
    tp(f"TP{k+6}", S, n("DRV_IN"))


strobe_module(100, "A")
strobe_module(200, "B")

# --------------------------------------------------------------------------
# Mechanical / non-netlist BOM lines (ref '-'): (desc, qty, value, package, mpn, unit cost, notes)
# --------------------------------------------------------------------------
EXTRAS = [
    ("lens_tir", 36, "TIR lens ~60 deg FWHM for 3535", "generic: 20 mm round TIR lens, 3535 holder for 20 mm star",
     "20 mm lens + holder", "generic (verify fit)", 0.9, "Start without lenses (Lambertian) for the first test; add lenses to gain ~3x on-axis"),
    ("heatsink", 2, "Al plate 110x90x3 mm", "LED side board heatsink; 18 stars (5x4 grid at 20 mm pitch, 2 free) + thermal pad/paste",
     "plate", "generic", 3.0, "or 2x 100x80 finned extrusion; average heat only ~1.5 W per side"),
    ("perf_ctrl", 1, "perfboard 100x160 mm", "control board, 2.54 mm, plated through holes", "perfboard", "generic", 3.0, ""),
    ("perf_mod", 2, "perfboard 90x70 mm", "strobe module board, 2.54 mm, plated through holes", "perfboard", "generic", 2.0, ""),
    ("socket_dip8", 6, "DIP-8 socket", "U5-U7 (3x MCP6022), U9, U101, U201", "DIP-8", "generic", 0.1, ""),
    ("socket_dip14", 4, "DIP-14 socket", "U2, U8, U10", "DIP-14", "generic", 0.12, ""),
    ("socket_pico", 2, "1x20 female header", "Pico socket", "2.54 mm", "generic", 0.5, ""),
    ("adapter_sot23", 4, "SOT-23 breakout board", "for D5, D6 (BAT54S) and U3 (SOT-23-6)", "adapter", "generic", 0.3, ""),
    ("jumpers", 8, "2.54 mm jumper", "shunts for JP1-JP6", "2.54 mm", "generic", 0.05, ""),
    ("wire_power", 1, "18 AWG silicone wire 2 m", "module power pairs, LED chains", "wire", "generic", 3.0, "keep each LED string wire <= 15 cm"),
    ("wire_sig", 1, "26 AWG wire 2 m + shielded pair 1 m", "gate/sense/trigger/sensor cables", "wire", "generic", 2.0, ""),
    ("switch_ilk", 1, "interlock switch (lid reed or key)", "normally open when lid removed -> closes when closed", "switch", "generic", 2.0, "wired to J8"),
    ("diffuser", 1, "IR-transmitting window / diffuser", "acrylic or PC, 2 mm, matte-sanded; keeps >= 5 cm eye distance", "plate", "generic (verify IR transmission)", 4.0, "optional but strongly recommended"),
    ("misc", 1, "screws, standoffs, solder, heat shrink", "", "-", "generic", 6.0, ""),
]

SHEETS = ["power", "pico", "camtrig", "gating", "trigsel", "acoustic", "optical", "strobe"]


def all_parts():
    return [PARTS[r_] for r_ in ORDER]


def nets():
    out = {}
    for p in all_parts():
        for pin, d in p["pins"].items():
            if d["net"] != NC:
                out.setdefault(d["net"], []).append((p["ref"], pin))
    return out


def val(ref):
    return PARTS[ref]["value"]


if __name__ == "__main__":
    print(len(PARTS), "parts,", len(nets()), "nets")
