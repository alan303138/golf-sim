class_name Coords
extends RefCounted
## THE single place where coordinate frames are defined and converted.
##
## 1. Contract / physics frame "P" (docs/ARCHITECTURE.md section 3):
##    right-handed, metres, +X = forward (down the target line), +Y = LEFT,
##    +Z = up. All ball physics (BallPhysics) runs in this frame. On the
##    course the frame is "world-anchored": +X = map north, +Y = map west,
##    and a shot's own target line is obtained by a yaw rotation about +Z.
##
## 2. Hole map frame "M" (hole JSON files, mini-map): 2D [east, north] in
##    metres, i.e. a top-down drawing with north up the page.
##       P.x = M.north,  P.y = -M.east
##
## 3. Godot frame "G": right-handed, +Y up, -Z forward, +X right.
##       G = (-P.y, P.z, -P.x)      P = (-G.z, -G.x, G.y)
##    so G.x = M.east and G.z = -M.north. This is a proper rotation
##    (det = +1): cross products (spin, Magnus force) map consistently.
##
## Launch conventions (contract):
##    hla_deg  > 0  => ball starts RIGHT of target line (towards -Y in P).
##    spin_axis_deg > 0 => ball curves RIGHT. Back spin about -Y(P) for a
##    ball travelling +X, i.e. omega = w * (0, -cos(axis), -sin(axis)) before
##    rotating by the start direction.
##    yaw (radians) rotates the contract target line (+X) counter-clockwise
##    seen from above (towards +Y / left). yaw = 0 => target line = P +X.

const RPM_TO_RADS := TAU / 60.0


static func to_godot(p: Vector3) -> Vector3:
	return Vector3(-p.y, p.z, -p.x)


static func from_godot(g: Vector3) -> Vector3:
	return Vector3(-g.z, -g.x, g.y)


## Map [east, north] -> physics horizontal (x, y).
static func map_to_phys(m: Vector2) -> Vector2:
	return Vector2(m.y, -m.x)


## Physics horizontal (x, y) -> map [east, north].
static func phys_to_map(p: Vector2) -> Vector2:
	return Vector2(-p.y, p.x)


static func map_to_godot(m: Vector2, height: float) -> Vector3:
	return Vector3(m.x, height, -m.y)


static func godot_to_map(g: Vector3) -> Vector2:
	return Vector2(g.x, -g.z)


## Rotate a physics-frame vector about +Z by yaw radians.
static func rotate_yaw(v: Vector3, yaw: float) -> Vector3:
	var c := cos(yaw)
	var s := sin(yaw)
	return Vector3(c * v.x - s * v.y, s * v.x + c * v.y, v.z)


## Yaw (radians) of a physics-frame horizontal direction.
static func yaw_of(dir: Vector2) -> float:
	return atan2(dir.y, dir.x)


## Initial velocity (physics frame, m/s) from contract launch values.
static func launch_velocity(speed_mps: float, vla_deg: float, hla_deg: float, yaw: float = 0.0) -> Vector3:
	var vla := deg_to_rad(vla_deg)
	var hla := deg_to_rad(hla_deg)
	var v := Vector3(cos(vla) * cos(hla), -cos(vla) * sin(hla), sin(vla)) * speed_mps
	return rotate_yaw(v, yaw)


## Initial angular velocity (physics frame, rad/s) from contract spin values.
## The spin axis lies horizontal, perpendicular to the start direction,
## tilted by spin_axis_deg (positive = curves right).
static func spin_vector(total_rpm: float, spin_axis_deg: float, hla_deg: float, yaw: float = 0.0) -> Vector3:
	var w := total_rpm * RPM_TO_RADS
	var a := deg_to_rad(spin_axis_deg)
	var om := Vector3(0.0, -cos(a), -sin(a)) * w
	# Start direction is rotated by -hla (right is negative yaw), then by yaw.
	return rotate_yaw(om, yaw - deg_to_rad(hla_deg))


## Signed lateral offset of point p from the line through origin with yaw.
## Positive = RIGHT of the line (contract convention).
static func offline_right(origin: Vector3, yaw: float, p: Vector3) -> float:
	var d := Vector2(p.x - origin.x, p.y - origin.y)
	var left := Vector2(-sin(yaw), cos(yaw))
	return -d.dot(left)


## Distance along the line through origin with yaw.
static func along_line(origin: Vector3, yaw: float, p: Vector3) -> float:
	var d := Vector2(p.x - origin.x, p.y - origin.y)
	return d.dot(Vector2(cos(yaw), sin(yaw)))
