class_name Clubs
extends RefCounted
## Club presets for the test-shot panel and auto-aim club suggestion.
## Values approximate a solid amateur / low-handicap player (not tour).
## speed = ball speed m/s, vla deg, spin rpm, club = club head speed m/s,
## sd_* = one standard deviation used for random dispersion.

const LIST := [
	{"id": "DR", "key": "CLUB_DR", "speed": 69.0, "vla": 12.5, "spin": 2700.0, "club": 47.0, "sd_speed": 1.8, "sd_vla": 1.6, "sd_hla": 2.2, "sd_spin": 350.0, "sd_axis": 7.0},
	{"id": "3W", "key": "CLUB_3W", "speed": 64.0, "vla": 11.0, "spin": 3500.0, "club": 44.0, "sd_speed": 1.7, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 380.0, "sd_axis": 6.5},
	{"id": "5W", "key": "CLUB_5W", "speed": 61.0, "vla": 12.5, "spin": 4300.0, "club": 42.5, "sd_speed": 1.6, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 420.0, "sd_axis": 6.0},
	{"id": "4H", "key": "CLUB_4H", "speed": 58.0, "vla": 13.0, "spin": 4600.0, "club": 41.0, "sd_speed": 1.5, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 450.0, "sd_axis": 6.0},
	{"id": "5I", "key": "CLUB_5I", "speed": 55.0, "vla": 13.5, "spin": 5300.0, "club": 40.0, "sd_speed": 1.4, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 500.0, "sd_axis": 5.5},
	{"id": "6I", "key": "CLUB_6I", "speed": 52.5, "vla": 15.0, "spin": 6100.0, "club": 39.0, "sd_speed": 1.3, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 520.0, "sd_axis": 5.5},
	{"id": "7I", "key": "CLUB_7I", "speed": 50.0, "vla": 16.5, "spin": 6900.0, "club": 37.5, "sd_speed": 1.2, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 550.0, "sd_axis": 5.0},
	{"id": "8I", "key": "CLUB_8I", "speed": 47.0, "vla": 18.5, "spin": 7700.0, "club": 36.0, "sd_speed": 1.1, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 600.0, "sd_axis": 5.0},
	{"id": "9I", "key": "CLUB_9I", "speed": 44.0, "vla": 21.0, "spin": 8400.0, "club": 34.5, "sd_speed": 1.0, "sd_vla": 1.5, "sd_hla": 2.0, "sd_spin": 650.0, "sd_axis": 5.0},
	{"id": "PW", "key": "CLUB_PW", "speed": 41.0, "vla": 24.0, "spin": 9000.0, "club": 33.5, "sd_speed": 1.0, "sd_vla": 1.6, "sd_hla": 2.0, "sd_spin": 700.0, "sd_axis": 5.0},
	{"id": "GW", "key": "CLUB_GW", "speed": 37.0, "vla": 27.0, "spin": 9400.0, "club": 32.0, "sd_speed": 0.9, "sd_vla": 1.7, "sd_hla": 2.0, "sd_spin": 700.0, "sd_axis": 5.0},
	{"id": "SW", "key": "CLUB_SW", "speed": 33.0, "vla": 30.0, "spin": 9800.0, "club": 30.0, "sd_speed": 0.9, "sd_vla": 1.8, "sd_hla": 2.0, "sd_spin": 750.0, "sd_axis": 5.0},
	{"id": "LW", "key": "CLUB_LW", "speed": 27.0, "vla": 34.0, "spin": 9500.0, "club": 27.0, "sd_speed": 0.8, "sd_vla": 2.0, "sd_hla": 2.0, "sd_spin": 750.0, "sd_axis": 5.0},
	{"id": "PT", "key": "CLUB_PT", "speed": 3.0, "vla": 1.0, "spin": 0.0, "club": 0.0, "sd_speed": 0.08, "sd_vla": 0.2, "sd_hla": 0.6, "sd_spin": 0.0, "sd_axis": 0.0},
]


static func index_of(id: String) -> int:
	for i in LIST.size():
		if LIST[i]["id"] == id:
			return i
	return 0


static func get_club(id: String) -> Dictionary:
	return LIST[index_of(id)]


## Contract "ball" values for a preset, optionally with random dispersion.
## power scales ball speed (used for partial shots / putts).
static func preset_ball(id: String, rng: RandomNumberGenerator = null, power: float = 1.0) -> Dictionary:
	var c := get_club(id)
	var speed: float = c["speed"] * power
	var vla: float = c["vla"]
	var hla := 0.0
	var spin: float = c["spin"] * lerpf(0.6, 1.0, clampf(power, 0.0, 1.0))
	var axis := 0.0
	var club_speed: float = c["club"] * power
	if rng != null:
		speed += rng.randfn(0.0, c["sd_speed"] * power)
		vla += rng.randfn(0.0, c["sd_vla"])
		hla += rng.randfn(0.0, c["sd_hla"])
		spin += rng.randfn(0.0, c["sd_spin"])
		axis += rng.randfn(0.0, c["sd_axis"])
	return {
		"speed_mps": maxf(speed, 0.1), "vla_deg": vla, "hla_deg": hla,
		"total_spin_rpm": maxf(spin, 0.0), "spin_axis_deg": clampf(axis, -45.0, 45.0),
		"club_speed_mps": club_speed,
	}


## Rough full-swing carry (m) per club, computed once with BallPhysics in
## calm sea-level conditions; used to suggest a club for a distance.
static var _carry_cache: Dictionary = {}

static func nominal_carry(id: String) -> float:
	if _carry_cache.has(id):
		return _carry_cache[id]
	var b := preset_ball(id)
	var r := BallPhysics.simulate_flat(b)
	_carry_cache[id] = r["carry"]
	return r["carry"]


## Suggest the club whose nominal carry best covers the distance.
static func suggest(distance_m: float, on_green: bool) -> String:
	if on_green:
		return "PT"
	var best := "DR"
	for c in LIST:
		if c["id"] == "PT":
			continue
		if nominal_carry(c["id"]) >= distance_m * 0.97:
			best = c["id"]
	return best
