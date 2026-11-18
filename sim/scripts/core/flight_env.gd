class_name FlightEnv
extends RefCounted
## Atmospheric and course conditions for a shot.

var altitude_m: float = 0.0
var temperature_c: float = 20.0
var humidity: float = 0.5           # relative, 0..1
## Wind velocity (physics frame, m/s) at the reference height. This is the
## air velocity, i.e. a wind blowing towards +X is a tail wind for yaw 0.
var wind: Vector3 = Vector3.ZERO
var wind_ref_height: float = 10.0
var roughness_length: float = 0.03  # m, short grass
var green_stimp_ft: float = 10.0


func air_density() -> float:
	return BallPhysics.air_density(altitude_m, temperature_c, humidity)


func air_viscosity() -> float:
	return BallPhysics.air_viscosity(temperature_c)


## Wind at height z (above ground) using a logarithmic boundary layer.
func wind_at(z: float) -> Vector3:
	if wind == Vector3.ZERO:
		return wind
	var zz := clampf(z, roughness_length * 2.0, 200.0)
	var f := log(zz / roughness_length) / log(wind_ref_height / roughness_length)
	return wind * clampf(f, 0.0, 1.6)


## Wind from compass-style values: speed and the direction the wind is
## blowing FROM, given as an angle relative to the physics +X axis
## (0 = from ahead i.e. head wind for yaw 0, 90 = from the left (+Y)).
static func wind_vector(speed_mps: float, from_deg: float) -> Vector3:
	var a := deg_to_rad(from_deg)
	return -Vector3(cos(a), sin(a), 0.0) * speed_mps
