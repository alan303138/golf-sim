class_name HoleData
extends RefCounted
## A golf hole (or the driving range) loaded from a JSON description.
## See README.md ("新增球洞") for the file format. All authoring is in the 2D
## map frame [east, north] metres (see Coords). This class rasterises the
## surface polygons and evaluates terrain height, and implements the ground
## interface for BallPhysics in the physics frame:
##   height_at(xy) / normal_at(xy) / surface_at(xy)

const FORMAT := 1
const RES := 0.5             # surface raster, metres per pixel
const DEP_CELL := 1.0        # depression grid (bunkers, water) cell size
const MARGIN := 60.0         # raster margin around the in-bounds polygon
const SAND_DEPTH := 0.35
const WATER_DEPTH := 0.9

const SMOOTH_DEFAULT := {
	"fairway": true, "green": true, "bunkers": true, "water": true, "tee_boxes": false, "bounds": false,
}
const LAYERS := [
	["fairway", Surface.FAIRWAY],
	["tee_boxes", Surface.TEE],
	["green", Surface.GREEN],
	["bunkers", Surface.SAND],
	["water", Surface.WATER],
]

var id := ""
var names: Dictionary = {}
var par := 4
var handicap := 0
var tees: Dictionary = {}           # name -> Vector2 (map)
var tee_names: Array[String] = []
var pins: Array[Vector2] = []
var pin_index := 0
var bounds := PackedVector2Array()
var polys: Dictionary = {}          # Surface id -> Array[PackedVector2Array]
var trees: Array = []               # {at: Vector2, height: float, radius: float}
var aim_points: Array[Vector2] = []
var targets: Array = []             # range targets {at: Vector2, radius: float}
var markers: Array = []             # range distance markers (metres)
var features: Array = []
var base_height := 0.0
var heightmap: Dictionary = {}
var rect := Rect2()                 # raster area (map frame)
var img_w := 0
var img_h := 0
var surf_img: Image                 # R8, value = surface id
var color_img: Image                # RGB8 base colours
var dep_w := 0
var dep_h := 0
var depress := PackedFloat32Array()
var water_levels: Array[float] = []
var errors: Array[String] = []
var built := false


static func load_file(path: String) -> HoleData:
	var h := HoleData.new()
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		h.errors.append("cannot open %s" % path)
		return h
	var json := JSON.new()
	if json.parse(f.get_as_text()) != OK:
		h.errors.append("%s: JSON error line %d: %s" % [path, json.get_error_line(), json.get_error_message()])
		return h
	if typeof(json.data) != TYPE_DICTIONARY:
		h.errors.append("%s: root must be an object" % path)
		return h
	h.parse(json.data)
	return h


static func from_dict(d: Dictionary) -> HoleData:
	var h := HoleData.new()
	h.parse(d)
	return h


func is_valid() -> bool:
	return errors.is_empty()


func display_name(locale: String = "zh_TW") -> String:
	if names.has(locale):
		return str(names[locale])
	if names.has("zh_TW"):
		return str(names["zh_TW"])
	return id


static func _v2(a) -> Vector2:
	if typeof(a) == TYPE_ARRAY and a.size() >= 2:
		return Vector2(float(a[0]), float(a[1]))
	return Vector2.ZERO


static func _is_pt(a) -> bool:
	return typeof(a) == TYPE_ARRAY and a.size() >= 2 and (typeof(a[0]) == TYPE_FLOAT or typeof(a[0]) == TYPE_INT)


func _poly(spec, smooth_default: bool) -> PackedVector2Array:
	var pts = spec
	var smooth := smooth_default
	if typeof(spec) == TYPE_DICTIONARY:
		pts = spec.get("points", [])
		smooth = bool(spec.get("smooth", smooth_default))
	var out := PackedVector2Array()
	if typeof(pts) != TYPE_ARRAY:
		return out
	for p in pts:
		if _is_pt(p):
			out.append(_v2(p))
	if smooth and out.size() >= 3:
		out = catmull_rom_closed(out, 6)
	return out


## Closed Catmull-Rom spline through the control points.
static func catmull_rom_closed(pts: PackedVector2Array, sub: int) -> PackedVector2Array:
	var out := PackedVector2Array()
	var n := pts.size()
	for i in n:
		var p0 := pts[(i - 1 + n) % n]
		var p1 := pts[i]
		var p2 := pts[(i + 1) % n]
		var p3 := pts[(i + 2) % n]
		for s in sub:
			var t := float(s) / sub
			var t2 := t * t
			var t3 := t2 * t
			out.append(0.5 * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3))
	return out


## List of polygons: accepts a single polygon or a list of polygons.
func _poly_list(spec, smooth_default: bool) -> Array:
	var out: Array = []
	if spec == null:
		return out
	if typeof(spec) == TYPE_DICTIONARY:
		var p := _poly(spec, smooth_default)
		if p.size() >= 3:
			out.append(p)
		return out
	if typeof(spec) != TYPE_ARRAY or spec.is_empty():
		return out
	if _is_pt(spec[0]):
		var p := _poly(spec, smooth_default)
		if p.size() >= 3:
			out.append(p)
		return out
	for item in spec:
		var p := _poly(item, smooth_default)
		if p.size() >= 3:
			out.append(p)
	return out


func parse(d: Dictionary) -> void:
	if int(d.get("format", FORMAT)) != FORMAT:
		errors.append("unsupported format %s" % str(d.get("format")))
		return
	id = str(d.get("id", "hole"))
	if typeof(d.get("name")) == TYPE_DICTIONARY:
		names = d["name"]
	elif d.has("name"):
		names = {"zh_TW": str(d["name"])}
	par = int(d.get("par", 4))
	handicap = int(d.get("handicap", 0))
	if par < 3 or par > 6:
		errors.append("par must be 3..6")
	if typeof(d.get("tees")) == TYPE_DICTIONARY:
		for k in d["tees"]:
			if _is_pt(d["tees"][k]):
				tees[str(k)] = _v2(d["tees"][k])
				tee_names.append(str(k))
	if tees.is_empty():
		errors.append("at least one tee is required")
	if typeof(d.get("pins")) == TYPE_ARRAY:
		for p in d["pins"]:
			if _is_pt(p):
				pins.append(_v2(p))
	if pins.is_empty():
		errors.append("at least one pin is required")
	bounds = _poly(d.get("bounds", []), false)
	if bounds.size() < 3:
		errors.append("bounds polygon needs >= 3 points")
	for layer in LAYERS:
		polys[layer[1]] = _poly_list(d.get(layer[0]), SMOOTH_DEFAULT[layer[0]])
	if typeof(d.get("aim_points")) == TYPE_ARRAY:
		for p in d["aim_points"]:
			if _is_pt(p):
				aim_points.append(_v2(p))
	if typeof(d.get("targets")) == TYPE_ARRAY:
		for t in d["targets"]:
			if typeof(t) == TYPE_DICTIONARY and _is_pt(t.get("at")):
				targets.append({"at": _v2(t["at"]), "radius": float(t.get("radius", 8.0))})
	if typeof(d.get("markers")) == TYPE_ARRAY:
		for m in d["markers"]:
			markers.append(float(m))
	var terrain: Dictionary = d.get("terrain", {}) if typeof(d.get("terrain")) == TYPE_DICTIONARY else {}
	base_height = float(terrain.get("base", 0.0))
	if typeof(terrain.get("features")) == TYPE_ARRAY:
		features = terrain["features"]
	if typeof(terrain.get("heightmap")) == TYPE_DICTIONARY:
		var hm: Dictionary = terrain["heightmap"]
		var cols := int(hm.get("cols", 0))
		var rows := int(hm.get("rows", 0))
		var data: Array = hm.get("data", [])
		if cols >= 2 and rows >= 2 and data.size() == cols * rows:
			heightmap = {"origin": _v2(hm.get("origin", [0, 0])), "cell": float(hm.get("cell", 10.0)),
				"cols": cols, "rows": rows, "data": PackedFloat32Array(data)}
		else:
			errors.append("heightmap needs cols*rows data values")
	_parse_trees(d)
	if errors.is_empty():
		build()


func _parse_trees(d: Dictionary) -> void:
	if typeof(d.get("trees")) == TYPE_ARRAY:
		for t in d["trees"]:
			if typeof(t) == TYPE_DICTIONARY and _is_pt(t.get("at")):
				trees.append({"at": _v2(t["at"]), "height": float(t.get("height", 12.0)), "radius": float(t.get("radius", 3.5))})
	if typeof(d.get("tree_lines")) == TYPE_ARRAY:
		for line in d["tree_lines"]:
			if typeof(line) != TYPE_DICTIONARY:
				continue
			var a := _v2(line.get("from"))
			var b := _v2(line.get("to"))
			var spacing := maxf(float(line.get("spacing", 10.0)), 2.0)
			var jitter := float(line.get("jitter", 2.0))
			var hr: Array = line.get("height", [10.0, 15.0])
			var rr: Array = line.get("radius", [3.0, 4.5])
			var rng := RandomNumberGenerator.new()
			rng.seed = int(line.get("seed", 7)) + int(a.x * 13.0 + b.y * 7.0)
			var n := int(a.distance_to(b) / spacing) + 1
			for i in n:
				var t := float(i) / maxf(n - 1, 1)
				var pos := a.lerp(b, t) + Vector2(rng.randf_range(-jitter, jitter), rng.randf_range(-jitter, jitter))
				trees.append({"at": pos, "height": rng.randf_range(float(hr[0]), float(hr[1])), "radius": rng.randf_range(float(rr[0]), float(rr[1]))})


# ----------------------------------------------------------------- raster

func build() -> void:
	var r := Rect2(bounds[0], Vector2.ZERO)
	for p in bounds:
		r = r.expand(p)
	rect = r.grow(MARGIN)
	img_w = int(ceil(rect.size.x / RES))
	img_h = int(ceil(rect.size.y / RES))
	surf_img = Image.create(img_w, img_h, false, Image.FORMAT_R8)
	color_img = Image.create(img_w, img_h, false, Image.FORMAT_RGB8)
	surf_img.fill(Color(Surface.OB / 255.0, 0, 0))
	color_img.fill(Surface.color(Surface.OB))
	_fill_poly(bounds, Surface.ROUGH)
	for layer in LAYERS:
		for poly in polys[layer[1]]:
			_fill_poly(poly, layer[1])
	# Trees must not stand on greens, bunkers or water.
	var kept: Array = []
	for t in trees:
		var s := surface_map(t["at"])
		if s != Surface.GREEN and s != Surface.SAND and s != Surface.WATER and s != Surface.TEE:
			kept.append(t)
	trees = kept
	_build_depressions()
	water_levels.clear()
	for poly in polys[Surface.WATER]:
		var lvl := INF
		for p in poly:
			lvl = minf(lvl, analytic_height(p))
		water_levels.append(lvl - 0.15)
	built = true


func _fill_poly(poly: PackedVector2Array, surf: int) -> void:
	var col := Surface.color(surf)
	var sc := Color(surf / 255.0, 0, 0)
	var n := poly.size()
	var ymin := INF
	var ymax := -INF
	for p in poly:
		ymin = minf(ymin, p.y)
		ymax = maxf(ymax, p.y)
	var row0 := clampi(int(floor((rect.end.y - ymax) / RES)), 0, img_h - 1)
	var row1 := clampi(int(ceil((rect.end.y - ymin) / RES)), 0, img_h - 1)
	var xs: Array[float] = []
	for row in range(row0, row1 + 1):
		var y := rect.end.y - (row + 0.5) * RES
		xs.clear()
		for i in n:
			var a := poly[i]
			var b := poly[(i + 1) % n]
			if (a.y <= y and b.y > y) or (b.y <= y and a.y > y):
				xs.append(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x))
		xs.sort()
		var k := 0
		while k + 1 < xs.size():
			# Pixels whose centres lie inside [xs[k], xs[k+1]].
			var c0 := clampi(int(ceil((xs[k] - rect.position.x) / RES - 0.5)), 0, img_w)
			var c1 := clampi(int(floor((xs[k + 1] - rect.position.x) / RES - 0.5)) + 1, 0, img_w)
			if c1 > c0:
				surf_img.fill_rect(Rect2i(c0, row, c1 - c0, 1), sc)
				color_img.fill_rect(Rect2i(c0, row, c1 - c0, 1), col)
			k += 2


func _build_depressions() -> void:
	dep_w = int(ceil(rect.size.x / DEP_CELL)) + 1
	dep_h = int(ceil(rect.size.y / DEP_CELL)) + 1
	var raw := PackedFloat32Array()
	raw.resize(dep_w * dep_h)
	for j in dep_h:
		for i in dep_w:
			var m := Vector2(rect.position.x + i * DEP_CELL, rect.position.y + j * DEP_CELL)
			var s := surface_map(m)
			var v := 0.0
			if s == Surface.SAND:
				v = SAND_DEPTH
			elif s == Surface.WATER:
				v = WATER_DEPTH
			raw[j * dep_w + i] = v
	# Two separable box-blur passes (radius 1) soften the bunker lips.
	for _pass in 2:
		raw = _blur(raw, dep_w, dep_h)
	depress = raw


static func _blur(src: PackedFloat32Array, w: int, h: int) -> PackedFloat32Array:
	var tmp := PackedFloat32Array()
	tmp.resize(w * h)
	for j in h:
		for i in w:
			var s := 0.0
			var c := 0
			for di in range(-1, 2):
				var ii := i + di
				if ii >= 0 and ii < w:
					s += src[j * w + ii]
					c += 1
			tmp[j * w + i] = s / c
	var out := PackedFloat32Array()
	out.resize(w * h)
	for j in h:
		for i in w:
			var s := 0.0
			var c := 0
			for dj in range(-1, 2):
				var jj := j + dj
				if jj >= 0 and jj < h:
					s += tmp[jj * w + i]
					c += 1
			out[j * w + i] = s / c
	return out


# ----------------------------------------------------------- map queries

func surface_map(m: Vector2) -> int:
	if not built:
		return Surface.ROUGH
	var c := int(floor((m.x - rect.position.x) / RES))
	var r := int(floor((rect.end.y - m.y) / RES))
	if c < 0 or r < 0 or c >= img_w or r >= img_h:
		return Surface.OB
	return int(round(surf_img.get_pixel(c, r).r * 255.0))


## Terrain height without bunker/water depressions.
func analytic_height(m: Vector2) -> float:
	var h := base_height
	for f in features:
		if typeof(f) != TYPE_DICTIONARY:
			continue
		match str(f.get("type", "")):
			"hill":
				var r := maxf(float(f.get("radius", 20.0)), 0.1)
				var d := m.distance_to(_v2(f.get("at")))
				h += float(f.get("height", 1.0)) * exp(-(d * d) / (r * r))
			"ramp":
				var a := _v2(f.get("from"))
				var b := _v2(f.get("to"))
				var ab := b - a
				var l2 := ab.length_squared()
				if l2 > 1e-6:
					var t := clampf((m - a).dot(ab) / l2, 0.0, 1.0)
					h += float(f.get("rise", 0.0)) * (t * t * (3.0 - 2.0 * t))
			"tilt":
				var c := _v2(f.get("at"))
				var r := maxf(float(f.get("radius", 15.0)), 0.1)
				var d := m.distance_to(c)
				var w := 1.0 - smoothstep(r * 0.7, r, d)
				var sl := _v2(f.get("slope"))
				h += (m - c).dot(sl) * w
	if not heightmap.is_empty():
		h += _heightmap_at(m)
	return h


func _heightmap_at(m: Vector2) -> float:
	var o: Vector2 = heightmap["origin"]
	var cell: float = heightmap["cell"]
	var cols: int = heightmap["cols"]
	var rows: int = heightmap["rows"]
	var data: PackedFloat32Array = heightmap["data"]
	var fx := clampf((m.x - o.x) / cell, 0.0, cols - 1.001)
	var fy := clampf((m.y - o.y) / cell, 0.0, rows - 1.001)
	var ix := int(fx)
	var iy := int(fy)
	var tx := fx - ix
	var ty := fy - iy
	var h00 := data[iy * cols + ix]
	var h10 := data[iy * cols + ix + 1]
	var h01 := data[(iy + 1) * cols + ix]
	var h11 := data[(iy + 1) * cols + ix + 1]
	return lerpf(lerpf(h00, h10, tx), lerpf(h01, h11, tx), ty)


func _depression_at(m: Vector2) -> float:
	if depress.is_empty():
		return 0.0
	var fx := clampf((m.x - rect.position.x) / DEP_CELL, 0.0, dep_w - 1.001)
	var fy := clampf((m.y - rect.position.y) / DEP_CELL, 0.0, dep_h - 1.001)
	var ix := int(fx)
	var iy := int(fy)
	var tx := fx - ix
	var ty := fy - iy
	var a := depress[iy * dep_w + ix]
	var b := depress[iy * dep_w + ix + 1]
	var c := depress[(iy + 1) * dep_w + ix]
	var d := depress[(iy + 1) * dep_w + ix + 1]
	return lerpf(lerpf(a, b, tx), lerpf(c, d, tx), ty)


func height_map(m: Vector2) -> float:
	return analytic_height(m) - _depression_at(m)


func tee_map(tee_name: String = "") -> Vector2:
	if tee_name != "" and tees.has(tee_name):
		return tees[tee_name]
	return tees[tee_names[0]]


func pin_map() -> Vector2:
	return pins[clampi(pin_index, 0, pins.size() - 1)]


func map_to_phys3(m: Vector2) -> Vector3:
	var p := Coords.map_to_phys(m)
	return Vector3(p.x, p.y, height_map(m))


func pin_phys() -> Vector3:
	return map_to_phys3(pin_map())


## Tree crowns for BallPhysics (physics frame).
func physics_trees() -> Array:
	var out: Array = []
	for t in trees:
		var m: Vector2 = t["at"]
		var p := Coords.map_to_phys(m)
		var h: float = t["height"]
		var r: float = t["radius"]
		out.append({"c": Vector3(p.x, p.y, height_map(m) + h - r * 1.1), "r": r * 0.9})
	return out


## Auto-aim target (map frame): the pin when reachable, otherwise the
## farthest-forward aim point that is reachable from `from`.
func auto_target(from: Vector2, reach_m: float) -> Vector2:
	var pin := pin_map()
	if from.distance_to(pin) <= reach_m * 1.05 or aim_points.is_empty():
		return pin
	var best := pin
	var best_d := INF
	for a in aim_points:
		var da := from.distance_to(a)
		if da <= reach_m * 1.05 and a.distance_to(pin) < from.distance_to(pin) - 5.0:
			var dp := a.distance_to(pin)
			if dp < best_d:
				best_d = dp
				best = a
	return best


# ------------------------------------------------- ground interface (phys)

func height_at(xy: Vector2) -> float:
	return height_map(Coords.phys_to_map(xy))


func normal_at(xy: Vector2) -> Vector3:
	var e := 0.3
	var hx1 := height_at(xy + Vector2(e, 0))
	var hx0 := height_at(xy - Vector2(e, 0))
	var hy1 := height_at(xy + Vector2(0, e))
	var hy0 := height_at(xy - Vector2(0, e))
	return Vector3(-(hx1 - hx0) / (2.0 * e), -(hy1 - hy0) / (2.0 * e), 1.0).normalized()


func surface_at(xy: Vector2) -> int:
	return surface_map(Coords.phys_to_map(xy))


## Nearest point (map) outside water along the segment from `entry` back to
## `origin` - used for the water-hazard penalty drop.
func water_drop_point(entry: Vector2, origin: Vector2) -> Vector2:
	var d := entry.distance_to(origin)
	var steps := int(d / 0.5) + 1
	for i in range(1, steps + 1):
		var p := entry.lerp(origin, float(i) / steps)
		var s := surface_map(p)
		if s != Surface.WATER and s != Surface.OB:
			# Step a little further from the water edge.
			var back := p.lerp(origin, minf(2.0 / maxf(p.distance_to(origin), 0.01), 1.0))
			return back if surface_map(back) != Surface.WATER else p
	return origin
