#!/usr/bin/env python3
"""Perfboard placement sketches (not to scale for lead spacing; board outline and 2.54 mm grid are).
Strobe module 90 x 70 mm, control board 100 x 160 mm.  The red path is the high-current loop."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch

OUT = os.path.dirname(os.path.abspath(__file__))


def board(ax, w, h):
    ax.add_patch(Rectangle((0, 0), w, h, fill=True, fc="#f3ecd2", ec="k", lw=1.5))
    for x in range(2, int(w), 3):
        for y in range(2, int(h), 3):
            ax.plot(x * 1.0, y * 1.0, ".", color="#c8b87a", ms=1.2)
    ax.set_xlim(-4, w + 4); ax.set_ylim(-6, h + 8); ax.set_aspect("equal"); ax.axis("off")


def part(ax, x, y, w, h, label, fc="#dfe8f5", fs=6.5, ec="#333"):
    ax.add_patch(Rectangle((x, y), w, h, fc=fc, ec=ec, lw=0.9))
    ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=fs)


def strobe():
    fig, ax = plt.subplots(figsize=(11, 8.5))
    W, H = 90, 70
    board(ax, W, H)
    ax.set_title("Strobe module perfboard 90 x 70 mm (top view, components side).  RED = high-current loop: keep it < 2 cm, wide copper / doubled wire", fontsize=9)
    # top: ballast row and J104
    part(ax, 4, 56, 54, 9, "RB101..RB106  1R8 3W  (6x, axial, standing or lying in a row)", fc="#f7d9c4")
    part(ax, 4, 66, 54, 3.5, "J104  6P screw (string anodes)  - wires to LED boards", fc="#d7efd7")
    part(ax, 62, 66, 26, 3.5, "J105  6P screw (returns)", fc="#d7efd7")
    # caps
    part(ax, 2, 12, 12, 30, "C101\nC102\n2200u\n(electrolytic,\nlying down)", fc="#e6e6fa")
    part(ax, 17, 36, 28, 7, "C103..C106  4x 10u X7R\nrow directly under ballast rail", fc="#ffe9a8")
    part(ax, 17, 46, 28, 5, "C107 100n   R108 bleeder", fc="#eee")
    # drain / fet
    part(ax, 50, 28, 12, 22, "Q101\nTO-220\n(heatsink tab\nup, bolted to\nplate if desired)", fc="#f5b7b1")
    part(ax, 64, 38, 10, 5, "D101\n1N5822", fc="#f5b7b1")
    part(ax, 64, 28, 12, 5, "RS101 10m\n(SRC -> PGND)", fc="#f5b7b1")
    # driver
    part(ax, 22, 14, 14, 10, "U101 TC4427\n(DIP-8 socket)", fc="#dfe8f5")
    part(ax, 38, 14, 10, 5, "R104  C110\nR105  D103", fc="#dfe8f5")
    part(ax, 38, 21, 10, 4, "C108 C109 D102 R101", fc="#dfe8f5")
    part(ax, 22, 4, 14, 5, "R102 R103\n(input)", fc="#dfe8f5")
    # connectors
    part(ax, 2, 2, 14, 7, "J102 2P screw\n12 V in", fc="#d7efd7")
    part(ax, 50, 2, 16, 7, "J103 3P header\nGATE / PGND / SENSE", fc="#d7efd7")
    part(ax, 68, 14, 20, 7, "D104 + R107 (charged LED)\nR106 (sense)", fc="#eee")
    part(ax, 68, 2, 20, 8, "TP101 V12  TP102 DRN\nTP103 SRC  TP104 GATE\nTP105 PGND  TP106 DRV_IN", fc="#fff3cd", fs=6)
    # loop arrows
    def arr(p, q, **kw):
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=12, color="red", lw=2.2, **kw))
    arr((31, 45), (31, 55))               # HF caps -> ballast
    arr((58, 59), (60, 52))               # ballast -> LED wires (leaves board) -> drain
    arr((62, 45), (58, 40))               # DRN into Q
    arr((56, 28), (70, 30.5))             # Q source -> RS
    arr((72, 28), (40, 36.5), connectionstyle="arc3,rad=-0.25")   # RS -> PGND plane back to HF caps
    ax.text(45, 52.5, "loop: C103-106 -> RB -> (LED board) -> J105/DRN -> Q101 -> RS101 -> PGND -> C103-106", fontsize=7, color="red", ha="center")
    ax.text(0, -4, "Rules: 1) C103-C106, Q101, RS101, D101 within ~2 cm.  2) Bus the 12 V and PGND nets with doubled 1 mm wire / solder bridges on the copper side, not thin jumpers.\n"
            "3) Driver ground and Kelvin return go to the FET SOURCE pin (SRC_A), not to PGND.  4) Bolt the LED side board near the module; LED wires <= 15 cm, twisted per string with its return.", fontsize=6.5)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "strobe_module_perfboard.png"), dpi=140); fig.savefig(os.path.join(OUT, "strobe_module_perfboard.svg")); plt.close(fig)


def control():
    fig, ax = plt.subplots(figsize=(9, 12))
    W, H = 100, 160
    board(ax, W, H)
    ax.set_title("Control board perfboard 100 x 160 mm (top view).  Zones separated so strobe power stays away from the analogue input", fontsize=9)
    part(ax, 3, 140, 52, 16, "POWER ZONE\nJ1  F1  D1  D2 TVS  C1 C2\nR1..R4 (cement 5 W, 15 mm clearance)", fc="#f7d9c4")
    part(ax, 60, 140, 36, 16, "J2 J3 -> module A\nJ4 J5 -> module B\nJP5 STAR (next to J1 return)", fc="#d7efd7")
    part(ax, 3, 82, 22, 52, "U1  Pico\n(2x 1x20 sockets)\nUSB connector\nover the board edge", fc="#dfe8f5")
    part(ax, 30, 112, 30, 22, "GATING\nU2 74HC08\nR14-R23  J8  Q2\nD7 ARMED (front edge)\nD8 status", fc="#ffe9a8")
    part(ax, 64, 112, 32, 22, "CAMERA TRIGGER\nU3 (SOT-23 adapter)\nU4 LDO  JP1\nR10-R13  D5 D6 adapters\nJ6 J7 (edge)", fc="#e6e6fa")
    part(ax, 30, 84, 66, 24, "SHOT-TRIGGER ANALOGUE  (quiet corner, far from R1-R4 and the J2-J5 cables)\nU5 U6 U7 (DIP sockets)  U8 LM339  U9 TLC555  Q3\nRV1 RV2 RV3 (trimmers on the edge)  JP2 JP3 JP4  J10 J11 (edge)\nU10 74HC86", fc="#dfe8f5")
    part(ax, 3, 60, 40, 18, "V5A / V5E filters\nR5 C4 C5   R6 C6 C7\nR7 R8 C11 D4  JP6", fc="#eee")
    part(ax, 48, 60, 48, 18, "Test point row along one edge:\nTP1..TP24 labelled with a marker pen", fc="#fff3cd")
    ax.text(0, -4, "Ground: run PGND (power zone) and GND_L (everything else) as two separate nets; join ONLY at JP5.  Put JP5 beside J1.\n"
            "Keep GATE_A/GATE_B and the camera-trigger wires away from the electret / photodiode input nodes (MIC_P, TIA_IN).", fontsize=6.5)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "control_board_perfboard.png"), dpi=130); fig.savefig(os.path.join(OUT, "control_board_perfboard.svg")); plt.close(fig)


if __name__ == "__main__":
    strobe(); control()
