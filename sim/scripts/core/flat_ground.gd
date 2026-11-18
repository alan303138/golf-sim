class_name FlatGround
extends RefCounted
## Simple ground model for tests and the driving range fallback.
## Implements the ground interface used by BallPhysics (physics frame):
##   height_at(xy: Vector2) -> float
##   normal_at(xy: Vector2) -> Vector3
##   surface_at(xy: Vector2) -> int   (Surface.*)

var height: float = 0.0
var surface: int = Surface.FAIRWAY
## Optional uniform slope: dz/dx and dz/dy.
var slope: Vector2 = Vector2.ZERO


func _init(surf: int = Surface.FAIRWAY, h: float = 0.0) -> void:
	surface = surf
	height = h


func height_at(xy: Vector2) -> float:
	return height + slope.x * xy.x + slope.y * xy.y


func normal_at(_xy: Vector2) -> Vector3:
	return Vector3(-slope.x, -slope.y, 1.0).normalized()


func surface_at(_xy: Vector2) -> int:
	return surface
