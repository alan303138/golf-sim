class_name BallPhysics
extends RefCounted
## Golf ball flight, bounce and roll. Pure logic, no scene dependencies, so it
## runs in headless unit tests. Everything here is in the physics frame "P"
## (see Coords): +X forward/north, +Y left/west, +Z up, SI units.
##
## Flight: 3-DOF point mass, fixed-step RK4, gravity + drag + Magnus lift.
##   Cd(Re, S) = Cd0(Re) + CD_SPIN * S         (drag rises with spin ratio and
##               at low Reynolds number - the late-flight "drag crisis" region)
##   Cl(S)     = CL_MAX * (1 - exp(-S / CL_S0)) (saturating lift)
##   S = r * |omega| / |v_rel|  (spin ratio), Re = rho * |v_rel| * d / mu
##   Spin decays exponentially (time constant SPIN_TAU).
## Coefficients were fitted to published tour launch-monitor averages
## (driver, 3-wood, 5-iron, 7-iron, PW); see tests/test_physics.gd.
##
## Ground: crater/impulse bounce model (Penner 2002) with surface-specific
## parameters (Surface.PARAMS), then rolling with slope gravity and
## rolling resistance; cup capture on greens.

const MASS := 0.04593
const RADIUS := 0.021335
const DIAMETER := 2.0 * RADIUS
const AREA := PI * RADIUS * RADIUS
const INERTIA_FACTOR := 0.4          # solid-ish sphere I = 2/5 m r^2
const GRAVITY := 9.80665

const FLIGHT_DT := 0.002
const ROLL_DT := 0.004
const SAMPLE_DT := 1.0 / 60.0
const MAX_TIME := 40.0

const CD_LOW := 0.213        # supercritical drag (high Re)
const CD_HIGH := 0.52        # subcritical drag (low Re)
const RE_LOW := 5.0e4
const RE_HIGH := 1.06e5
const CD_SPIN := 0.13
const CL_MAX := 0.295
const CL_S0 := 0.13
const SPIN_TAU := 25.0       # s

const CUP_RADIUS := 0.054    # 108 mm cup
const CUP_CAPTURE_SPEED := 1.6
const MIN_BOUNCE_VN := 0.6   # m/s, below this the ball starts rolling
const MAX_BOUNCES := 20
const STOP_SPEED := 0.03


# ---------------------------------------------------------------- atmosphere

## Moist air density (kg/m^3) from altitude (m), temperature (C), RH (0..1).
static func air_density(altitude_m: float, temp_c: float, rh: float = 0.5) -> float:
	var t_k := temp_c + 273.15
	var p := 101325.0 * pow(maxf(1.0 - 2.25577e-5 * altitude_m, 0.1), 5.25588)
	var es := 610.94 * exp(17.625 * temp_c / (temp_c + 243.04))
	var pv := clampf(rh, 0.0, 1.0) * es
	var pd := p - pv
	return pd / (287.058 * t_k) + pv / (461.495 * t_k)


## Dynamic viscosity of air (Pa s), Sutherland's law.
static func air_viscosity(temp_c: float) -> float:
	var t_k := temp_c + 273.15
	return 1.458e-6 * pow(t_k, 1.5) / (t_k + 110.4)


# --------------------------------------------------------------- aerodynamics

static func drag_coefficient(re: float, spin_ratio: float) -> float:
	var base: float
	if re >= RE_HIGH:
		base = CD_LOW
	elif re <= RE_LOW:
		base = CD_HIGH
	else:
		var t := (re - RE_LOW) / (RE_HIGH - RE_LOW)
		base = CD_HIGH + (CD_LOW - CD_HIGH) * (3.0 * t * t - 2.0 * t * t * t)
	return base + CD_SPIN * spin_ratio


static func lift_coefficient(spin_ratio: float) -> float:
	return CL_MAX * (1.0 - exp(-maxf(spin_ratio, 0.0) / CL_S0))


## Acceleration in flight. v = ball velocity, om = angular velocity (rad/s),
## wind = air velocity at the ball.
static func flight_accel(v: Vector3, om: Vector3, wind: Vector3, rho: float, mu: float) -> Vector3:
	var vr := v - wind
	var sp := vr.length()
	var a := Vector3(0.0, 0.0, -GRAVITY)
	if sp < 1e-6:
		return a
	var k := 0.5 * rho * AREA / MASS
	var s := RADIUS * om.length() / sp
	var re := rho * sp * DIAMETER / mu
	a -= vr * (k * drag_coefficient(re, s) * sp)
	var c := om.cross(vr)
	var cl_len := c.length()
	if cl_len > 1e-9:
		a += c * (k * lift_coefficient(s) * sp * sp / cl_len)
	return a


static func _rk4(p: Vector3, v: Vector3, om: Vector3, dt: float, env: FlightEnv, rho: float, mu: float, ground_h: float) -> Array:
	var w1 := env.wind_at(p.z - ground_h)
	var k1v := flight_accel(v, om, w1, rho, mu)
	var k1p := v
	var v2 := v + k1v * (0.5 * dt)
	var p2 := p + k1p * (0.5 * dt)
	var w2 := env.wind_at(p2.z - ground_h)
	var k2v := flight_accel(v2, om, w2, rho, mu)
	var k2p := v2
	var v3 := v + k2v * (0.5 * dt)
	var p3 := p + k2p * (0.5 * dt)
	var k3v := flight_accel(v3, om, w2, rho, mu)
	var k3p := v3
	var v4 := v + k3v * dt
	var p4 := p + k3p * dt
	var w4 := env.wind_at(p4.z - ground_h)
	var k4v := flight_accel(v4, om, w4, rho, mu)
	var k4p := v4
	var np := p + (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0)
	var nv := v + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0)
	return [np, nv]


# ------------------------------------------------------------------- bounce

## Penner's normal coefficient of restitution as a function of the normal
## impact speed (m/s).
static func restitution(vn: float) -> float:
	if vn > 20.0:
		return 0.12
	return clampf(0.510 - 0.0375 * vn + 0.000903 * vn * vn, 0.12, 0.6)


## Impact with the ground. Returns [v_out, omega_out].
static func bounce(v: Vector3, om: Vector3, n: Vector3, surface: int) -> Array:
	var prm := Surface.params(surface)
	var speed := v.length()
	if speed < 1e-6:
		return [Vector3.ZERO, Vector3.ZERO]
	var vn_g := -v.dot(n)
	if vn_g <= 0.0:
		return [v, om]
	var theta := asin(clampf(vn_g / speed, 0.0, 1.0))
	var vt_g := v + n * vn_g
	var n2 := n
	if vt_g.length() > 1e-4:
		var t_dir := vt_g.normalized()
		# Pitch-mark (crater) angle, Penner 2002, scaled by surface softness.
		var theta_c := float(prm["crater"]) * deg_to_rad(15.4) * (speed / 18.6) * (rad_to_deg(theta) / 44.4)
		theta_c = clampf(theta_c, 0.0, deg_to_rad(32.0))
		n2 = (n * cos(theta_c) - t_dir * sin(theta_c)).normalized()
	var vn := -v.dot(n2)
	if vn <= 0.0:
		n2 = n
		vn = vn_g
	var e := restitution(vn) * float(prm["restitution_mult"])
	var vt := v + n2 * vn
	var rc := -n2 * RADIUS
	var u := vt + om.cross(rc)            # slip velocity of the contact point
	var u_len := u.length()
	var v_out := vt + n2 * (e * vn)
	var om_out := om
	if u_len > 1e-6:
		var jn := (1.0 + e) * vn          # normal impulse per unit mass
		var j_roll := u_len / (1.0 + 1.0 / INERTIA_FACTOR)  # = 2/7 |u| for I = 2/5 m r^2
		var jt := minf(float(prm["friction"]) * jn, j_roll)
		var udir := u / u_len
		v_out -= udir * jt
		om_out = om + rc.cross(-udir * jt) / (INERTIA_FACTOR * RADIUS * RADIUS)
	# Spin about the surface normal is mostly scrubbed off by the turf.
	var spin_n := om_out.dot(n)
	om_out -= n * (spin_n * 0.5)
	return [v_out, om_out]


# --------------------------------------------------------------- simulate

static func _hxy(p: Vector3) -> Vector2:
	return Vector2(p.x, p.y)


## launch: {speed_mps, vla_deg, hla_deg, total_spin_rpm, spin_axis_deg}
## ground: object with height_at/normal_at/surface_at (see FlatGround)
## opts:
##   start: Vector3 physics position of the ball's ground contact point
##          (default (0,0,ground height))
##   yaw: float, target-line yaw (radians), default 0
##   putt: bool, start rolling on the surface (no flight)
##   cup: Vector3 hole position (enables capture), optional
##   trees: Array of {c: Vector3 crown centre, r: float}
##   seed: int, deterministic randomness for tree deflections
static func simulate(launch: Dictionary, env: FlightEnv, ground, opts: Dictionary = {}) -> Dictionary:
	var rho := env.air_density()
	var mu := env.air_viscosity()
	var yaw: float = opts.get("yaw", 0.0)
	var start: Vector3
	if opts.has("start"):
		start = opts["start"]
	else:
		start = Vector3(0.0, 0.0, ground.height_at(Vector2.ZERO))
	var p := start + Vector3(0.0, 0.0, RADIUS)
	var speed: float = launch.get("speed_mps", 0.0)
	var vla: float = launch.get("vla_deg", 0.0)
	var hla: float = launch.get("hla_deg", 0.0)
	var spin: float = launch.get("total_spin_rpm", 0.0)
	var axis: float = launch.get("spin_axis_deg", 0.0)
	var v := Coords.launch_velocity(speed, vla, hla, yaw)
	var om := Coords.spin_vector(spin, axis, hla, yaw)
	var has_cup := opts.has("cup") and opts["cup"] != null
	var cup: Vector3 = opts["cup"] if has_cup else Vector3.ZERO
	var trees: Array = opts.get("trees", [])
	var tree_used := {}
	var rng := RandomNumberGenerator.new()
	rng.seed = int(opts.get("seed", 12345))
	var green_mu := Surface.green_mu_for_stimp(env.green_stimp_ft)

	var rolling: bool = bool(opts.get("putt", false))
	if rolling:
		var n0: Vector3 = ground.normal_at(_hxy(p))
		v = v - n0 * v.dot(n0)
		om = Vector3.ZERO

	var points := PackedVector3Array()
	var times := PackedFloat32Array()
	points.append(p)
	times.append(0.0)
	var next_sample := SAMPLE_DT

	var t := 0.0
	var apex_z := p.z
	var landed := rolling
	var carry_pos := p
	var land_time := 0.0
	var land_angle := 0.0
	var land_index := 0
	var land_surface: int = ground.surface_at(_hxy(p))
	var land_spin := spin
	var bounces := 0
	var holed := false
	var hazard := ""
	var water_entry := Vector3.ZERO
	var tree_hit := false
	var events: Array = []

	while t < MAX_TIME:
		if not rolling:
			var gh: float = ground.height_at(_hxy(p))
			var r := _rk4(p, v, om, FLIGHT_DT, env, rho, mu, gh)
			var np: Vector3 = r[0]
			var nv: Vector3 = r[1]
			om *= exp(-FLIGHT_DT / SPIN_TAU)
			# Tree crowns.
			for i in trees.size():
				if tree_used.has(i):
					continue
				var tr: Dictionary = trees[i]
				var c: Vector3 = tr["c"]
				var rr: float = tr["r"]
				if np.distance_squared_to(c) < rr * rr:
					tree_used[i] = true
					tree_hit = true
					var rnd := Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-0.5, 0.3))
					nv = nv * rng.randf_range(0.05, 0.25) * (1.0 if rng.randf() < 0.5 else -0.4) + rnd * 2.0
					om *= 0.2
					events.append({"t": t, "kind": "tree", "pos": np})
			var ngh: float = ground.height_at(_hxy(np))
			var clear1 := np.z - RADIUS - ngh
			if clear1 <= 0.0 and (t > 0.0 or nv.z < 0.0):
				var clear0 := p.z - RADIUS - gh
				var f := 1.0
				if clear0 - clear1 > 1e-9:
					f = clampf(clear0 / (clear0 - clear1), 0.0, 1.0)
				var contact := p.lerp(np, f)
				var vc := v.lerp(nv, f)
				t += FLIGHT_DT * f
				var cxy := _hxy(contact)
				contact.z = ground.height_at(cxy) + RADIUS
				var surf: int = ground.surface_at(cxy)
				if not landed:
					landed = true
					carry_pos = contact
					land_time = t
					land_surface = surf
					land_spin = om.length() / Coords.RPM_TO_RADS
					var hs := Vector2(vc.x, vc.y).length()
					land_angle = rad_to_deg(atan2(-vc.z, hs))
					points.append(contact)
					times.append(t)
					land_index = points.size() - 1
					next_sample = t + SAMPLE_DT
					events.append({"t": t, "kind": "land", "pos": contact})
				else:
					events.append({"t": t, "kind": "bounce", "pos": contact})
				p = contact
				if has_cup and Vector2(p.x - cup.x, p.y - cup.y).length() < CUP_RADIUS and vc.length() < 12.0:
					holed = true
					p = Vector3(cup.x, cup.y, cup.z)
					break
				if surf == Surface.WATER:
					hazard = "water"
					water_entry = contact
					events.append({"t": t, "kind": "water", "pos": contact})
					break
				var n: Vector3 = ground.normal_at(cxy)
				var b := bounce(vc, om, n, surf)
				v = b[0]
				om = b[1]
				bounces += 1
				if v.dot(n) < MIN_BOUNCE_VN or bounces >= MAX_BOUNCES:
					v = v - n * v.dot(n)
					rolling = true
					events.append({"t": t, "kind": "roll", "pos": p})
			else:
				p = np
				v = nv
				t += FLIGHT_DT
				if not landed:
					apex_z = maxf(apex_z, p.z)
		else:
			var xy := _hxy(p)
			var n: Vector3 = ground.normal_at(xy)
			var surf: int = ground.surface_at(xy)
			if surf == Surface.WATER:
				hazard = "water"
				water_entry = p
				events.append({"t": t, "kind": "water", "pos": p})
				break
			var prm := Surface.params(surf)
			var mu_r: float = green_mu if surf == Surface.GREEN else float(prm["roll_mu"])
			var drag: float = prm["roll_drag"]
			var g_t := Vector3(0.0, 0.0, -GRAVITY) + n * (GRAVITY * n.z)
			var a_slope := g_t * (5.0 / 7.0)
			var resist := mu_r * GRAVITY * n.z
			var sp := v.length()
			if sp < STOP_SPEED and a_slope.length() <= resist:
				break
			var nv := v + a_slope * ROLL_DT
			var s1 := nv.length()
			if s1 > 1e-9:
				var s2 := maxf(0.0, s1 - (resist + drag * s1) * ROLL_DT)
				nv *= s2 / s1
			nv -= n * nv.dot(n)
			var np := p + nv * ROLL_DT
			np.z = ground.height_at(_hxy(np)) + RADIUS
			if has_cup:
				var d := _seg_point_dist(_hxy(p), _hxy(np), Vector2(cup.x, cup.y))
				if d < CUP_RADIUS:
					var allowed := CUP_CAPTURE_SPEED * sqrt(maxf(0.0, 1.0 - pow(d / CUP_RADIUS, 2.0)))
					if sp <= allowed:
						holed = true
						p = Vector3(cup.x, cup.y, cup.z)
						events.append({"t": t, "kind": "holed", "pos": p})
						break
			p = np
			v = nv
			t += ROLL_DT
		if t >= next_sample:
			points.append(p)
			times.append(t)
			next_sample += SAMPLE_DT

	if points[points.size() - 1] != p:
		points.append(p)
		times.append(t)
	if not landed:
		carry_pos = p
		land_time = t
		land_index = points.size() - 1
	var final_ground := p - Vector3(0.0, 0.0, 0.0 if holed else RADIUS)
	var carry_ground := carry_pos - Vector3(0.0, 0.0, RADIUS)
	var result := {
		"points": points,
		"times": times,
		"events": events,
		"start": start,
		"yaw": yaw,
		"land_index": land_index,
		"carry_pos": carry_ground,
		"final_pos": final_ground,
		"carry": Vector2(carry_ground.x - start.x, carry_ground.y - start.y).length(),
		"total": Vector2(final_ground.x - start.x, final_ground.y - start.y).length(),
		"carry_along": Coords.along_line(start, yaw, carry_ground),
		"total_along": Coords.along_line(start, yaw, final_ground),
		"offline_carry": Coords.offline_right(start, yaw, carry_ground),
		"offline": Coords.offline_right(start, yaw, final_ground),
		"apex": apex_z - RADIUS - start.z,
		"flight_time": land_time,
		"total_time": t,
		"land_angle_deg": land_angle,
		"landing_surface": land_surface,
		"final_surface": ground.surface_at(_hxy(final_ground)) if hazard == "" else Surface.WATER,
		"landing_spin_rpm": land_spin,
		"holed": holed,
		"hazard": hazard,
		"water_entry": water_entry,
		"tree_hit": tree_hit,
		"bounces": bounces,
	}
	return result


static func _seg_point_dist(a: Vector2, b: Vector2, c: Vector2) -> float:
	var ab := b - a
	var l2 := ab.length_squared()
	if l2 < 1e-12:
		return a.distance_to(c)
	var tt := clampf((c - a).dot(ab) / l2, 0.0, 1.0)
	return (a + ab * tt).distance_to(c)


## Convenience: simulate a contract "ball" dictionary on flat fairway.
static func simulate_flat(ball: Dictionary, env: FlightEnv = null, surf: int = Surface.FAIRWAY) -> Dictionary:
	if env == null:
		env = FlightEnv.new()
	return simulate(ball, env, FlatGround.new(surf), {})
