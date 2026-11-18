extends SceneTree
## Headless test runner.
##   godot --headless --path sim --script res://tests/run_tests.gd
## Optional: -- --only=physics   (substring filter on test file names)
## Exit code 0 = all passed, 1 = failures.

const TEST_FILES := [
	"res://tests/test_coords.gd",
	"res://tests/test_physics.gd",
	"res://tests/test_protocol.gd",
	"res://tests/test_scoring.gd",
	"res://tests/test_holes.gd",
	"res://tests/test_update.gd",
	"res://tests/test_scenes.gd",
]


func _initialize() -> void:
	_run()


func _run() -> void:
	var only := ""
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--only="):
			only = a.substr(7)
	var total := 0
	var failed := 0
	for path in TEST_FILES:
		if only != "" and not path.contains(only):
			continue
		var script: GDScript = load(path)
		if script == null:
			print("[ERROR] cannot load ", path)
			failed += 1
			continue
		var methods: Array = []
		for m in script.get_script_method_list():
			var mname: String = m["name"]
			if mname.begins_with("test_") and not methods.has(mname):
				methods.append(mname)
		for mname in methods:
			var inst = script.new()
			inst.tree = self
			total += 1
			var t0 := Time.get_ticks_msec()
			await inst.call(mname)
			var dt := Time.get_ticks_msec() - t0
			for n in inst.notes:
				print("    note: ", n)
			if inst.failures.is_empty():
				print("[PASS] %s::%s (%d ms)" % [path.get_file(), mname, dt])
			else:
				failed += 1
				print("[FAIL] %s::%s" % [path.get_file(), mname])
				for f in inst.failures:
					print("    - ", f)
	print("")
	print("%d tests, %d failed" % [total, failed])
	quit(1 if failed > 0 else 0)
