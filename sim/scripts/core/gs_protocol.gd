class_name GsProtocol
extends RefCounted
## Device API WebSocket message parsing/building (ARCHITECTURE.md sections 4-5).
## parse() never throws: it returns {ok: bool, type: String, msg: Dictionary,
## error: String}. A parsed "shot" message's msg is normalised (see
## normalize_shot) so the game can rely on every field being present.

const SCHEMA_VERSION := 1
const API_VERSION := 1
const STATES := ["idle", "ready", "processing", "error", "updating"]


static func parse(text: String) -> Dictionary:
	var json := JSON.new()
	if json.parse(text) != OK:
		return _err("invalid json: %s" % json.get_error_message())
	var data = json.data
	if typeof(data) != TYPE_DICTIONARY:
		return _err("message is not an object")
	var d: Dictionary = data
	if not d.has("type") or typeof(d["type"]) != TYPE_STRING:
		return _err("missing type")
	var t: String = d["type"]
	match t:
		"shot":
			var s := normalize_shot(d)
			if s.has("error"):
				return _err(s["error"], t)
			return {"ok": true, "type": t, "msg": s, "error": ""}
		"hello":
			return {"ok": true, "type": t, "error": "", "msg": {
				"device_id": str(d.get("device_id", "")),
				"sw_version": str(d.get("sw_version", "")),
				"fw_version": str(d.get("fw_version", "")),
				"api": int(_num(d.get("api", 0), 0.0)),
			}}
		"status":
			var st := str(d.get("state", "idle"))
			if not STATES.has(st):
				st = "error"
			return {"ok": true, "type": t, "error": "", "msg": {
				"state": st,
				"ball_detected": bool(d.get("ball_detected", false)),
				"message": str(d.get("message", "")),
			}}
		"update":
			return {"ok": true, "type": t, "error": "", "msg": {
				"phase": str(d.get("phase", "")),
				"progress": clampf(_num(d.get("progress", 0.0), 0.0), 0.0, 1.0),
				"message": str(d.get("message", "")),
			}}
		"pong":
			return {"ok": true, "type": t, "error": "", "msg": {}}
		_:
			# Unknown types are tolerated (forward compatibility).
			return {"ok": true, "type": t, "error": "", "msg": d}


static func _err(e: String, t: String = "") -> Dictionary:
	return {"ok": false, "type": t, "msg": {}, "error": e}


static func _num(v, fallback: float) -> float:
	if typeof(v) == TYPE_FLOAT or typeof(v) == TYPE_INT:
		var f := float(v)
		if is_finite(f):
			return f
	return fallback


static func _is_num(v) -> bool:
	return (typeof(v) == TYPE_FLOAT or typeof(v) == TYPE_INT) and is_finite(float(v))


## Validate and normalise a shot message. Returns the normalised dictionary
## or {error: "..."}.
static func normalize_shot(d: Dictionary) -> Dictionary:
	var schema = d.get("schema", SCHEMA_VERSION)
	if not _is_num(schema) or int(schema) != SCHEMA_VERSION:
		return {"error": "unsupported schema %s" % str(schema)}
	if typeof(d.get("ball")) != TYPE_DICTIONARY:
		return {"error": "missing ball"}
	var b: Dictionary = d["ball"]
	for k in ["speed_mps", "vla_deg", "hla_deg", "total_spin_rpm", "spin_axis_deg"]:
		if not _is_num(b.get(k)):
			return {"error": "ball.%s missing or not a number" % k}
	var speed := float(b["speed_mps"])
	var vla := float(b["vla_deg"])
	var hla := float(b["hla_deg"])
	var spin := float(b["total_spin_rpm"])
	var axis := float(b["spin_axis_deg"])
	if speed < 0.0 or speed > 120.0:
		return {"error": "ball speed out of range"}
	if vla < -20.0 or vla > 89.0 or absf(hla) > 60.0:
		return {"error": "launch angle out of range"}
	if spin < 0.0 or spin > 20000.0 or absf(axis) > 90.0:
		return {"error": "spin out of range"}
	var back := spin * cos(deg_to_rad(axis))
	var side := spin * sin(deg_to_rad(axis))
	if _is_num(b.get("back_spin_rpm")):
		back = float(b["back_spin_rpm"])
	if _is_num(b.get("side_spin_rpm")):
		side = float(b["side_spin_rpm"])
	var club = null
	if typeof(d.get("club")) == TYPE_DICTIONARY and _is_num(d["club"].get("speed_mps")):
		var cs := float(d["club"]["speed_mps"])
		if cs > 0.0 and cs < 90.0:
			club = {"speed_mps": cs}
	var q := {"confidence": 1.0, "sources": [], "spin_measured": false}
	if typeof(d.get("quality")) == TYPE_DICTIONARY:
		var qd: Dictionary = d["quality"]
		q["confidence"] = clampf(_num(qd.get("confidence", 1.0), 1.0), 0.0, 1.0)
		if typeof(qd.get("sources")) == TYPE_ARRAY:
			q["sources"] = qd["sources"]
		q["spin_measured"] = bool(qd.get("spin_measured", false))
	return {
		"type": "shot",
		"schema": SCHEMA_VERSION,
		"shot_id": str(d.get("shot_id", "")),
		"ts": str(d.get("ts", "")),
		"ball": {
			"speed_mps": speed, "vla_deg": vla, "hla_deg": hla,
			"total_spin_rpm": spin, "spin_axis_deg": axis,
			"back_spin_rpm": back, "side_spin_rpm": side,
		},
		"club": club,
		"quality": q,
	}


## Build a locally generated shot (test panel / auto putt) in the same shape.
static func make_shot(speed: float, vla: float, hla: float, spin: float, axis: float, club_speed: float = 0.0, source: String = "manual") -> Dictionary:
	var d := {
		"type": "shot", "schema": SCHEMA_VERSION,
		"shot_id": "local-%d-%d" % [Time.get_ticks_msec(), randi() % 100000],
		"ts": Time.get_datetime_string_from_system(true) + "Z",
		"ball": {"speed_mps": speed, "vla_deg": vla, "hla_deg": hla, "total_spin_rpm": spin, "spin_axis_deg": axis},
		"club": {"speed_mps": club_speed} if club_speed > 0.0 else null,
		"quality": {"confidence": 1.0, "sources": [source], "spin_measured": false},
	}
	return normalize_shot(d)


static func msg_ping() -> String:
	return JSON.stringify({"type": "ping"})


static func msg_arm() -> String:
	return JSON.stringify({"type": "arm"})


static func msg_disarm() -> String:
	return JSON.stringify({"type": "disarm"})


## "192.168.1.20", "host:9000", "ws://host:8080/ws/v1" -> full ws URL.
static func ws_url(host: String) -> String:
	var h := host.strip_edges()
	if h.begins_with("ws://") or h.begins_with("wss://"):
		return h
	if h.begins_with("http://"):
		h = h.substr(7)
	h = h.trim_suffix("/")
	if h.is_empty():
		return ""
	var has_port := false
	if h.begins_with("["):
		has_port = h.contains("]:")
	else:
		has_port = h.count(":") == 1
	if not has_port:
		h += ":8080"
	return "ws://%s/ws/v1" % h
