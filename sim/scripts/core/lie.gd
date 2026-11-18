class_name Lie
extends RefCounted
## Lie-dependent shot penalties for course play. A measured shot from the
## sensor is hit off a mat; when the virtual ball sits in rough or sand the
## launch is degraded with some randomness (speed loss, inconsistent spin,
## "fliers"). Deterministic for a given RNG seed.

## Per-lie ranges: speed factor [min,max], spin factor [min,max], extra
## launch-direction noise (deg standard deviation).
const EFFECTS := {
	Surface.ROUGH: {"speed": [0.86, 0.97], "spin": [0.45, 0.90], "hla_sd": 1.0},
	Surface.SAND: {"speed": [0.72, 0.88], "spin": [0.55, 0.85], "hla_sd": 1.5},
	Surface.OB: {"speed": [0.86, 0.97], "spin": [0.45, 0.90], "hla_sd": 1.0},
}


static func has_penalty(surface: int) -> bool:
	return EFFECTS.has(surface)


## Returns a new ball dictionary (contract fields) adjusted for the lie.
## Putts (is_putt) are not affected.
static func apply(ball: Dictionary, surface: int, rng: RandomNumberGenerator, is_putt: bool = false) -> Dictionary:
	var out := ball.duplicate()
	if is_putt or not EFFECTS.has(surface):
		return out
	var e: Dictionary = EFFECTS[surface]
	var sf := rng.randf_range(e["speed"][0], e["speed"][1])
	var pf := rng.randf_range(e["spin"][0], e["spin"][1])
	out["speed_mps"] = float(ball["speed_mps"]) * sf
	out["total_spin_rpm"] = float(ball["total_spin_rpm"]) * pf
	out["hla_deg"] = float(ball["hla_deg"]) + rng.randfn(0.0, e["hla_sd"])
	var axis := float(ball["spin_axis_deg"])
	out["back_spin_rpm"] = float(out["total_spin_rpm"]) * cos(deg_to_rad(axis))
	out["side_spin_rpm"] = float(out["total_spin_rpm"]) * sin(deg_to_rad(axis))
	return out
