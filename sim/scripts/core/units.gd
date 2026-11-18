class_name Units
extends RefCounted
## Display unit conversion and formatting. Internal values are always SI.

const M_TO_YD := 1.0936133
const M_TO_FT := 3.2808399
const MPS_TO_MPH := 2.2369363
const MPS_TO_KMH := 3.6

## distance: "m" | "yd"; speed: "mps" | "kmh" | "mph"
static var distance_unit := "yd"
static var speed_unit := "mph"


static func dist_value(m: float, unit: String = "") -> float:
	var u := unit if unit != "" else distance_unit
	return m * M_TO_YD if u == "yd" else m


static func dist_suffix(unit: String = "") -> String:
	var u := unit if unit != "" else distance_unit
	return "yd" if u == "yd" else "m"


static func dist(m: float, decimals: int = 1, unit: String = "") -> String:
	return ("%." + str(decimals) + "f %s") % [dist_value(m, unit), dist_suffix(unit)]


## Short distances (putts): feet in imperial mode, metres otherwise.
static func short_dist(m: float, unit: String = "") -> String:
	var u := unit if unit != "" else distance_unit
	if u == "yd":
		return "%.1f ft" % (m * M_TO_FT)
	return "%.1f m" % m


static func speed_value(mps: float, unit: String = "") -> float:
	var u := unit if unit != "" else speed_unit
	match u:
		"mph":
			return mps * MPS_TO_MPH
		"kmh":
			return mps * MPS_TO_KMH
	return mps


static func speed_suffix(unit: String = "") -> String:
	var u := unit if unit != "" else speed_unit
	match u:
		"mph":
			return "mph"
		"kmh":
			return "km/h"
	return "m/s"


static func speed(mps: float, decimals: int = 1, unit: String = "") -> String:
	return ("%." + str(decimals) + "f %s") % [speed_value(mps, unit), speed_suffix(unit)]


## Offline: positive = right. Returns e.g. "R 4.2 m" / "L 3.0 yd" using
## translated direction letters supplied by the caller.
static func offline(m: float, right_label: String, left_label: String, decimals: int = 1) -> String:
	if absf(dist_value(m)) < 0.05:
		return dist(0.0, decimals)
	var lbl := right_label if m > 0.0 else left_label
	return "%s %s" % [lbl, dist(absf(m), decimals)]
