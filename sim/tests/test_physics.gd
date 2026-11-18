extends "res://tests/test_case.gd"
## Ball flight validation against published tour launch-monitor averages
## (widely published PGA Tour averages; carry in metres, apex in metres).

const M_TO_YD := 1.0936133

# name, speed m/s, vla, spin rpm, expected carry m, carry tolerance (fraction), apex m
const REFERENCE := [
	["driver", 74.7, 10.9, 2686.0, 251.5, 0.06, 29.0],
	["3-wood", 70.6, 9.2, 3655.0, 222.0, 0.08, 27.4],
	["5-iron", 59.0, 12.1, 5361.0, 177.4, 0.08, 28.3],
	["7-iron", 53.6, 16.3, 7097.0, 157.3, 0.06, 29.3],
	["PW", 45.6, 24.2, 9304.0, 124.4, 0.08, 26.5],
]


func _ball(speed: float, vla: float, spin: float, hla := 0.0, axis := 0.0) -> Dictionary:
	return {"speed_mps": speed, "vla_deg": vla, "hla_deg": hla, "total_spin_rpm": spin, "spin_axis_deg": axis}


func _calm_env() -> FlightEnv:
	var env := FlightEnv.new()
	env.temperature_c = 20.0
	env.altitude_m = 0.0
	env.humidity = 0.5
	return env


func test_air_density() -> void:
	check_near(BallPhysics.air_density(0.0, 15.0, 0.0), 1.225, 0.005, "ISA sea level density")
	check(BallPhysics.air_density(1600.0, 15.0, 0.0) < 1.05, "density at 1600 m is lower")
	check(BallPhysics.air_density(0.0, 35.0, 0.5) < BallPhysics.air_density(0.0, 5.0, 0.5), "hot air is thinner")


func test_reference_carries() -> void:
	for r in REFERENCE:
		var res := BallPhysics.simulate(_ball(r[1], r[2], r[3]), _calm_env(), FlatGround.new(Surface.FAIRWAY))
		var carry: float = res["carry"]
		var expected: float = r[4]
		note("%-7s %5.1f m/s %4.1f deg %5.0f rpm -> carry %.1f m (%.0f yd) [ref %.1f m, %+.1f%%], apex %.1f m [ref %.1f], land %.1f deg, total %.1f m, flight %.2f s" % [
			r[0], r[1], r[2], r[3], carry, carry * M_TO_YD, expected, 100.0 * (carry - expected) / expected,
			res["apex"], r[6], res["land_angle_deg"], res["total"], res["flight_time"]])
		check(absf(carry - expected) <= expected * r[5], "%s carry %.1f vs %.1f" % [r[0], carry, expected])
		check(absf(float(res["apex"]) - r[6]) <= 6.0, "%s apex %.1f vs %.1f" % [r[0], res["apex"], r[6]])
		check(absf(float(res["offline"])) < 0.5, "%s straight shot stays online" % r[0])


func test_task_reference_values() -> void:
	# The two reference points named in the project brief.
	var d := BallPhysics.simulate(_ball(75.0, 11.0, 2700.0), _calm_env(), FlatGround.new())
	var i7 := BallPhysics.simulate(_ball(54.0, 16.0, 7000.0), _calm_env(), FlatGround.new())
	note("brief driver 75 m/s 11 deg 2700 rpm: carry %.1f m (%.0f yd), total %.1f m" % [d["carry"], d["carry"] * M_TO_YD, d["total"]])
	note("brief 7-iron 54 m/s 16 deg 7000 rpm: carry %.1f m (%.0f yd), total %.1f m" % [i7["carry"], i7["carry"] * M_TO_YD, i7["total"]])
	check_near(d["carry"], 250.0, 15.0, "driver carry ~250 m")
	check_near(i7["carry"], 157.0, 9.5, "7-iron carry ~157 m")


func test_curvature_signs() -> void:
	var env := _calm_env()
	var fade := BallPhysics.simulate(_ball(70.0, 12.0, 3000.0, 0.0, 15.0), env, FlatGround.new())
	var draw := BallPhysics.simulate(_ball(70.0, 12.0, 3000.0, 0.0, -15.0), env, FlatGround.new())
	var push := BallPhysics.simulate(_ball(70.0, 12.0, 3000.0, 4.0, 0.0), env, FlatGround.new())
	note("axis +15 offline %.1f m, axis -15 offline %.1f m, hla +4 offline %.1f m" % [fade["offline_carry"], draw["offline_carry"], push["offline_carry"]])
	check(float(fade["offline_carry"]) > 8.0, "positive spin axis curves right")
	check(float(draw["offline_carry"]) < -8.0, "negative spin axis curves left")
	check(float(push["offline_carry"]) > 8.0, "positive hla starts right")
	check_near(fade["offline_carry"], -float(draw["offline_carry"]), 0.05, "symmetry")


func test_yaw_rotation() -> void:
	var env := _calm_env()
	var ball := _ball(60.0, 14.0, 5000.0, 2.0, 5.0)
	var a := BallPhysics.simulate(ball, env, FlatGround.new(), {"yaw": 0.0})
	var b := BallPhysics.simulate(ball, env, FlatGround.new(), {"yaw": deg_to_rad(73.0), "start": Vector3(10, -5, 0)})
	check_near(a["carry"], b["carry"], 0.05, "carry invariant under yaw")
	check_near(a["offline"], b["offline"], 0.05, "offline invariant under yaw")
	check_near(a["total_along"], b["total_along"], 0.05, "along invariant under yaw")


func test_wind_and_altitude() -> void:
	var ball := _ball(60.0, 14.0, 5500.0)
	var calm := BallPhysics.simulate(ball, _calm_env(), FlatGround.new())
	var head_env := _calm_env()
	head_env.wind = FlightEnv.wind_vector(5.0, 0.0)
	var tail_env := _calm_env()
	tail_env.wind = FlightEnv.wind_vector(5.0, 180.0)
	var cross_env := _calm_env()
	cross_env.wind = FlightEnv.wind_vector(5.0, 90.0)   # from the left
	var high_env := _calm_env()
	high_env.altitude_m = 1600.0
	var head := BallPhysics.simulate(ball, head_env, FlatGround.new())
	var tail := BallPhysics.simulate(ball, tail_env, FlatGround.new())
	var cross := BallPhysics.simulate(ball, cross_env, FlatGround.new())
	var high := BallPhysics.simulate(ball, high_env, FlatGround.new())
	note("6-iron-ish carry: calm %.1f, 5 m/s head %.1f, tail %.1f, cross-from-left offline %.1f, 1600 m altitude %.1f" % [calm["carry"], head["carry"], tail["carry"], cross["offline_carry"], high["carry"]])
	check(float(head["carry"]) < float(calm["carry"]) - 5.0, "head wind shortens")
	check(float(tail["carry"]) > float(calm["carry"]) + 3.0, "tail wind lengthens")
	check(float(cross["offline_carry"]) > 3.0, "wind from the left pushes ball right")
	check(float(high["carry"]) > float(calm["carry"]) * 1.02, "altitude adds distance")


func test_bounce_and_roll() -> void:
	var env := _calm_env()
	var drv := BallPhysics.simulate(_ball(74.7, 10.9, 2686.0), env, FlatGround.new(Surface.FAIRWAY))
	var drv_rough := BallPhysics.simulate(_ball(74.7, 10.9, 2686.0), env, FlatGround.new(Surface.ROUGH))
	var i7 := BallPhysics.simulate(_ball(53.6, 16.3, 7097.0), env, FlatGround.new(Surface.GREEN))
	var pw := BallPhysics.simulate(_ball(45.6, 24.2, 9304.0), env, FlatGround.new(Surface.GREEN))
	var lw := BallPhysics.simulate(_ball(32.0, 30.0, 10500.0), env, FlatGround.new(Surface.GREEN))
	var sand := BallPhysics.simulate(_ball(53.6, 16.3, 7097.0), env, FlatGround.new(Surface.SAND))
	var drv_roll := float(drv["total"]) - float(drv["carry"])
	var i7_roll := float(i7["total_along"]) - float(i7["carry_along"])
	var pw_roll := float(pw["total_along"]) - float(pw["carry_along"])
	var lw_roll := float(lw["total_along"]) - float(lw["carry_along"])
	note("roll: driver fairway %.1f m (rough %.1f m, %d bounces), 7-iron green %.1f m, PW green %.1f m, 60deg-wedge green %.1f m, 7-iron sand %.1f m" % [
		drv_roll, float(drv_rough["total"]) - float(drv_rough["carry"]), drv["bounces"], i7_roll, pw_roll, lw_roll, float(sand["total"]) - float(sand["carry"])])
	check(drv_roll > 8.0 and drv_roll < 40.0, "driver rolls 8-40 m on fairway")
	check(float(drv_rough["total"]) - float(drv_rough["carry"]) < drv_roll, "rough rolls less than fairway")
	check(i7_roll > -2.0 and i7_roll < 9.0, "7-iron holds the green")
	check(pw_roll < i7_roll, "PW stops quicker than 7-iron")
	check(lw_roll < 2.0, "high-spin wedge checks or spins back")
	check(float(sand["total"]) - float(sand["carry"]) < 1.5, "ball plugs/stops in sand")


func test_green_speed_putt() -> void:
	var env := _calm_env()
	env.green_stimp_ft = 10.0
	var res := BallPhysics.simulate(_ball(1.83, 0.0, 0.0), env, FlatGround.new(Surface.GREEN), {"putt": true})
	note("stimp 10 putt at 1.83 m/s rolls %.2f m (expect 3.05)" % res["total"])
	check_near(res["total"], 3.05, 0.15, "stimpmeter calibration")


func test_cup_capture() -> void:
	var env := _calm_env()
	var g := FlatGround.new(Surface.GREEN)
	var cup := Vector3(3.0, 0.0, 0.0)
	var good := BallPhysics.simulate(_ball(2.3, 0.0, 0.0), env, g, {"putt": true, "cup": cup})
	var fast := BallPhysics.simulate(_ball(6.0, 0.0, 0.0), env, g, {"putt": true, "cup": cup})
	var wide := BallPhysics.simulate(_ball(2.3, 0.0, 0.0, 3.0), env, g, {"putt": true, "cup": cup})
	check(good["holed"], "firm centred putt drops")
	check(not fast["holed"], "very fast putt lips over")
	check(not wide["holed"], "3 degree miss at 3 m misses")


func test_slope_roll() -> void:
	var env := _calm_env()
	var g := FlatGround.new(Surface.GREEN)
	g.slope = Vector2(0.0, 0.03)   # rises to the left => breaks right
	var res := BallPhysics.simulate(_ball(2.0, 0.0, 0.0), env, g, {"putt": true})
	note("putt on 3%% side slope breaks %.2f m right" % res["offline"])
	check(float(res["offline"]) > 0.1, "ball breaks downhill (right)")


func test_water_stops() -> void:
	var res := BallPhysics.simulate(_ball(50.0, 15.0, 6000.0), _calm_env(), FlatGround.new(Surface.WATER))
	check_eq(res["hazard"], "water", "lands in water")
	check(float(res["total"]) - float(res["carry"]) < 0.01, "no roll in water")


func test_deterministic() -> void:
	var a := BallPhysics.simulate(_ball(66.0, 12.0, 3100.0, -1.0, 3.0), _calm_env(), FlatGround.new())
	var b := BallPhysics.simulate(_ball(66.0, 12.0, 3100.0, -1.0, 3.0), _calm_env(), FlatGround.new())
	check_eq(a["final_pos"], b["final_pos"], "same input, same result")
