extends RefCounted
## Minimal assertion helpers for the headless test runner.

var failures: Array[String] = []
var notes: Array[String] = []
var tree: SceneTree


func check(cond: bool, msg: String) -> void:
	if not cond:
		failures.append(msg)


func check_near(actual: float, expected: float, tol: float, msg: String) -> void:
	if absf(actual - expected) > tol:
		failures.append("%s: got %.4f expected %.4f +/- %.4f" % [msg, actual, expected, tol])


func check_eq(actual, expected, msg: String) -> void:
	if typeof(actual) != typeof(expected) or actual != expected:
		failures.append("%s: got %s expected %s" % [msg, str(actual), str(expected)])


func note(msg: String) -> void:
	notes.append(msg)
