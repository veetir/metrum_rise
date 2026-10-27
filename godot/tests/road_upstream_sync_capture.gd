# SPDX-License-Identifier: GPL-2.0-only

## Read-only imported-city frame measurements for the upstream integration.
## Writes PNGs and frame samples only to METRUM_SYNC_CAPTURE_OUTPUT.
extends "res://tests/road_junction_geometry_capture.gd"

var measurements: Array[Dictionary] = []

func run() -> void:
	await setup_capture_scene()
	output = OS.get_environment("METRUM_SYNC_CAPTURE_OUTPUT")
	DirAccess.make_dir_recursive_absolute(output)
	var source := OS.get_environment("METRUM_JUNCTION_SAVE")
	if not simulation.load_game(source):
		expect(false, "Load imported city for upstream comparison")
		main.queue_free()
		await process_frame
		quit(failures)
		return
	main.get_node("InputManager")._refresh_after_world_load()
	simulation.set_simulation_speed(0)
	main.get_node("ZoningOverlay").set_tool_active(true)
	Engine.max_fps = 0
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	var prisma := Vector3(-1002.86615, 100.44034, 1817.88599)
	var kys := Vector3(-73.19017, 98.29294, 922.70374)
	await native_capture("prisma_overhead", prisma, 110, -1.1, -3.446)
	await sample_frames("prisma_paused", 180)
	simulation.set_simulation_speed(1)
	await sample_frames("prisma_1x", 180)
	await native_capture("prisma_street", prisma, 35, -0.12, -3.446)
	await native_capture("kys_overhead", kys, 152.47514, -0.596, -1.94)
	await sample_frames("kys_1x", 180)
	await native_capture("kys_street", kys, 40, -0.12, -1.94)
	var intervals := PackedFloat64Array()
	var previous := Time.get_ticks_usec()
	for i in 360:
		var t := float(i) / 359.0
		var pivot := prisma.lerp(kys, t)
		var offset := Vector3(sin(t * TAU) * 300, 350, cos(t * TAU) * 300)
		camera.global_position = pivot + offset
		camera.look_at(pivot, Vector3.UP)
		await process_frame
		var now := Time.get_ticks_usec()
		intervals.append(float(now - previous) / 1000.0)
		previous = now
		if i % 90 == 0:
			await RenderingServer.frame_post_draw
			expect(root.get_texture().get_image().save_png(output.path_join("sweep_%03d.png" % i)) == OK, "Sweep capture")
	record_samples("city_sweep_1x", intervals)
	var file := FileAccess.open(output.path_join("frames.json"), FileAccess.WRITE)
	file.store_string(JSON.stringify({"source": source.get_file(), "viewport": root.size, "zoning": true, "measurements": measurements}, "\t"))
	main.queue_free()
	await process_frame
	quit(failures)

func sample_frames(label: String, count: int) -> void:
	for i in 30:
		await process_frame
	var samples := PackedFloat64Array()
	var previous := Time.get_ticks_usec()
	for i in count:
		await process_frame
		var now := Time.get_ticks_usec()
		samples.append(float(now - previous) / 1000.0)
		previous = now
	record_samples(label, samples)

func record_samples(label: String, samples: PackedFloat64Array) -> void:
	var ordered := samples.duplicate()
	ordered.sort()
	var total := 0.0
	for sample in samples:
		total += sample
	measurements.append({"label": label, "frames": samples.size(), "mean_ms": total / samples.size(), "fps": 1000.0 * samples.size() / total, "p50_ms": ordered[ordered.size() / 2], "p95_ms": ordered[int(ordered.size() * 0.95)], "samples_ms": samples})
