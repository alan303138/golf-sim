class_name Surface
extends RefCounted
## Ground surface types and their physical parameters.
##
## Bounce parameters follow the crater model of A. R. Penner, "The run of a
## golf ball" (Can. J. Phys. 2002): on impact the effective surface is tilted
## back towards the incoming ball by theta_c (the pitch-mark wall) and a
## rigid-body impulse with Coulomb friction and spin is applied. Per-surface
## multipliers make greens receptive, fairways firmer, rough/sand dead.

enum { ROUGH = 0, FAIRWAY = 1, GREEN = 2, SAND = 3, WATER = 4, OB = 5, TEE = 6 }

const COUNT := 7

## restitution_mult: scales Penner's e(v_n)
## friction: sliding friction coefficient during impact
## crater: scales Penner's pitch-mark angle (softer = larger)
## roll_mu: constant rolling-resistance deceleration factor (a = mu * g)
## roll_drag: additional speed-proportional deceleration (1/s), grass drag
const PARAMS := {
	ROUGH: {"restitution_mult": 0.70, "friction": 0.60, "crater": 1.25, "roll_mu": 0.26, "roll_drag": 0.55},
	FAIRWAY: {"restitution_mult": 1.10, "friction": 0.45, "crater": 0.80, "roll_mu": 0.10, "roll_drag": 0.20},
	GREEN: {"restitution_mult": 0.95, "friction": 0.48, "crater": 1.00, "roll_mu": 0.056, "roll_drag": 0.0},
	SAND: {"restitution_mult": 0.15, "friction": 0.80, "crater": 1.90, "roll_mu": 0.90, "roll_drag": 2.0},
	WATER: {"restitution_mult": 0.0, "friction": 1.0, "crater": 1.0, "roll_mu": 5.0, "roll_drag": 10.0},
	OB: {"restitution_mult": 0.70, "friction": 0.60, "crater": 1.25, "roll_mu": 0.30, "roll_drag": 0.6},
	TEE: {"restitution_mult": 1.10, "friction": 0.45, "crater": 0.80, "roll_mu": 0.10, "roll_drag": 0.20},
}

const NAME_KEYS := {
	ROUGH: "SURF_ROUGH", FAIRWAY: "SURF_FAIRWAY", GREEN: "SURF_GREEN", SAND: "SURF_SAND",
	WATER: "SURF_WATER", OB: "SURF_OB", TEE: "SURF_TEE",
}

## Base colours used for terrain texture and mini-map.
const COLORS := {
	ROUGH: Color(0.24, 0.42, 0.16),
	FAIRWAY: Color(0.36, 0.62, 0.22),
	GREEN: Color(0.42, 0.72, 0.30),
	SAND: Color(0.86, 0.79, 0.58),
	WATER: Color(0.13, 0.35, 0.55),
	OB: Color(0.20, 0.32, 0.14),
	TEE: Color(0.40, 0.66, 0.26),
}


static func params(id: int) -> Dictionary:
	return PARAMS.get(id, PARAMS[ROUGH])


static func name_key(id: int) -> String:
	return NAME_KEYS.get(id, "SURF_ROUGH")


static func color(id: int) -> Color:
	return COLORS.get(id, COLORS[ROUGH])


## Rolling-resistance factor on the green for a given Stimpmeter reading
## (feet). A ball released at 1.83 m/s rolls stimp_ft feet on a flat green.
static func green_mu_for_stimp(stimp_ft: float) -> float:
	var d := maxf(stimp_ft, 4.0) * 0.3048
	return 1.83 * 1.83 / (2.0 * d * 9.80665)
