# SPDX-License-Identifier: GPL-2.0-only

## Real-scene trajectory inspection for Prisma, ordinary junctions and lane transitions.
## METRUM_ROADS_B_CAPTURE=1 captures rebuilt connector paths and routed cars.
extends "res://tests/road_pockets_test.gd"

func settle() -> bool:
	# These dry fixtures exercise roads; off-camera water prewarm is not part of capture readiness.
	var stable := 0
	var deadline := Time.get_ticks_msec() + 120000
	while stable < 24 and Time.get_ticks_msec() < deadline:
		await process_frame
		if not main.get_node("Terrain").has_pending_render_work(false) and not main.get_node("Vegetation").has_pending_work() and not simulation.is_network_dirty():
			stable += 1
		else:
			stable = 0
	expect(stable >= 24, "Trajectory road and terrain must settle")
	return stable >= 24

func junction() -> void:
	await build(PackedVector3Array([ground(-230, 0), ground(230, 0)]), 2, 2, 2, 5.0)
	await build(PackedVector3Array([ground(0, -150), ground(0, 150)]), 1, 1, 0, 0.0)
	var edge: int = simulation.get_hovered_edge(-60, 0)
	var result := await finish_request(simulation.change_road_pocket(edge, false, true, true, 80.0, 35.0))
	expect(result.get("committed", false), "Trajectory Prisma pocket fixture")
	# Separate fixtures keep every approach visible in one windowed run.
	await build(stroke(Vector2(-210, -300), [Vector2(-95, 0), Vector2(95, 0)]), 1, 1, 0, 0.0)
	await build(stroke(Vector2(-210, -300), [Vector2(0, 0), Vector2(0, 95)]), 1, 1, 0, 0.0)
	await build(stroke(Vector2(210, -300), [Vector2(-95, 0), Vector2(95, 0)]), 2, 2, 2, 5.0)
	await build(stroke(Vector2(210, -300), [Vector2(0, -95), Vector2(0, 95)]), 2, 2, 2, 5.0)
	await build(PackedVector3Array([ground(-230, 280), ground(-80, 280)]), 1, 1, 0, 0.0)
	await build(PackedVector3Array([ground(-80, 280), ground(100, 280)]), 2, 2, 0, 0.0)
	await build(PackedVector3Array([ground(160, 280), ground(260, 280)]), 2, 2, 2, 5.0)
	await build(PackedVector3Array([ground(260, 280), ground(260, 400)]), 2, 2, 2, 5.0)
	if main != null:
		await settle()
		var console: Node = root.get_node_or_null("DebugConsole")
		if console != null:
			console.set_visual_mode("junction_connectors", "lanes")
		await capture("plain_t_paths", ground(-210, -300), 95.0, -1.5, -1.57)
		await capture("divided_cross_paths", ground(210, -300), 100.0, -1.5, -1.57)
		await capture("lane_transition_paths", ground(-80, 280), 75.0, -1.5, -1.57)
		await capture("divided_bend_paths", ground(260, 280), 80.0, -1.5, -1.57)
		await capture("prisma_paths", ground(0, 0), 100.0, -1.5, -1.57)
		if console != null:
			console.set_visual_mode("junction_connectors", "off")

func capture(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float = -0.65) -> void:
	var tag := OS.get_environment("METRUM_TRAJECTORY_TAG")
	if tag.is_empty():
		tag = "after"
	# Bypass the parent stage5 prefix so the accepted slice B captures remain intact.
	camera.set_view_state(PackedFloat32Array([pivot.x, pivot.y, pivot.z, yaw, pitch, distance, 0.0]))
	if pitch > -0.2:
		var forward := Vector3(-sin(yaw), 0.0, -cos(yaw))
		var eye := pivot - forward * distance
		eye.y = simulation.get_world_surface_height(Vector2(eye.x, eye.z)) + 1.7
		var target := pivot + forward * 60.0
		target.y = simulation.get_world_surface_height(Vector2(target.x, target.z)) + 1.3
		camera.global_position = eye
		camera.look_at(target, Vector3.UP)
	await settle()
	await process_frame
	await RenderingServer.frame_post_draw
	var image := root.get_texture().get_image()
	expect(image.save_png(output.path_join("trajectory_%s_%s.png" % [tag, label])) == OK, "Save trajectory capture")
