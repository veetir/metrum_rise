# SPDX-License-Identifier: GPL-2.0-only

## Split-pocket bridge regression; opt-in captures reuse the lane-choice harness.
## Final slice B names preserve historical stages 1–2 screenshots.
extends "res://tests/road_turn_lanes_test.gd"

func junction() -> void:
	await build(PackedVector3Array([ground(-230, 0), ground(230, 0)]), 2, 2, 2, 5.0)
	if main != null:
		await settle()
	await build(PackedVector3Array([ground(0, -150), ground(0, 150)]), 1, 1, 0, 0.0)
	if main != null:
		await settle()
	var edge: int = simulation.get_hovered_edge(-60.0, 0.0)
	var result := await finish_request(simulation.change_road_pocket(edge, false, true, true, 80.0, 35.0))
	expect(result.get("committed", false), "Pocket transaction: %s" % result)
	if main != null:
		await settle()
		await capture("pocket_taper_above", ground(-112, 0), 95.0, -1.3, -1.57)
		await capture("pocket_taper_street", ground(-130, 5), 24.0, -0.10, -1.57)
		await capture_tool_preview()
		# A continuous drawing stroke can change count at its next node.
		await build(PackedVector3Array([ground(-230, -260), ground(-80, -260)]), 1, 1, 0, 0.0)
		await build(PackedVector3Array([ground(-80, -260), ground(100, -260)]), 2, 2, 0, 0.0)
		await settle()
		await capture("midroad_taper_above", ground(-65, -260), 85.0, -1.3, -1.57)
		await capture("midroad_taper_street", ground(-110, -259), 22.0, -0.12, -1.57)
	var pocket: Dictionary = simulation.get_road_cross_section(simulation.get_hovered_edge(-40.0, 0.0))
	var upstream: Dictionary = simulation.get_road_cross_section(simulation.get_hovered_edge(-200.0, 0.0))
	expect(pocket.get("fwd_lanes", 0) == 4 and pocket.get("bkw_lanes", 0) == 2, "Four approach lanes and unchanged opposite carriageway")
	expect(upstream.get("fwd_lanes", 0) == 2, "Two upstream through lanes")

func capture_tool_preview() -> void:
	var tool: Node = main.get_node("RoadTool")
	tool.set_process(false)
	tool.active = true
	tool.toggle_pocket_edit()
	# The drawing selection deliberately differs from the edited divided approach.
	tool.fwd_lanes = 1
	tool.bkw_lanes = 1
	tool.set_median(0, 0.0)
	tool._last_world_mouse_pos = ground(-40, 0)
	tool._update_pocket_hover()
	tool.adjust_pocket_length(2.0, false)
	tool._update_pocket_hover()
	expect(tool._pocket_preview_valid and tool._junction_preview.generation >= 0, "H installs exact replacement preview")
	expect(tool._terrain_preview.request_id == tool._junction_preview.request_id, "Road and terrain previews install together")
	expect(not tool._preview_prop_keys.is_empty(), "Endpoint preview replaces lamp and sign chunks")
	await capture("tool_preview_above", ground(-112, 0), 95.0, -1.3, -1.57)
	await capture("tool_preview_street", ground(-130, 5), 24.0, -0.10, -1.57)
	# A previously absent affected patch invalidates the paired display when it streams in.
	var pending_patch := Vector2i(999, 999)
	tool._pocket_pending_patches[pending_patch] = true
	tool._pocket_patch_will_change(pending_patch)
	expect(tool._pocket_preview_key.is_empty() and tool._junction_preview.generation < 0, "Streaming invalidates the endpoint preview")
	tool._update_pocket_hover()
	expect(tool._junction_preview.generation >= 0, "Endpoint preview can reinstall after streaming")
	var edge: int = simulation.get_hovered_edge(-40, 0)
	var result := await finish_request(tool.apply_pocket_to_end(edge, false))
	expect(result.get("committed", false), "Commit previewed resize")
	tool._edit_request_id = 0
	tool.update_main_mesh()
	await capture("tool_committed_above", ground(-112, 0), 95.0, -1.3, -1.57)
	await capture("tool_committed_street", ground(-130, 5), 24.0, -0.10, -1.57)
	tool.pocket_left = false
	tool.pocket_right = false
	tool._pocket_preview_key = []
	tool._update_pocket_hover()
	expect(tool._pocket_preview_valid and tool._junction_preview.generation >= 0, "H previews pocket removal")
	await capture("tool_remove_preview_above", ground(-112, 0), 95.0, -1.3, -1.57)
	result = await finish_request(tool.apply_pocket_to_end(edge, false))
	expect(result.get("committed", false), "Commit previewed removal")
	tool._edit_request_id = 0
	tool.update_main_mesh()
	await capture("tool_remove_committed_above", ground(-112, 0), 95.0, -1.3, -1.57)
	edge = simulation.get_hovered_edge(-40, 0)
	result = await finish_request(simulation.change_road_pocket(edge, false, true, true, 80.0, 35.0))
	expect(result.get("committed", false), "Restore Prisma pockets after preview checks")
	tool.toggle_pocket_edit()
	tool.active = false
	tool.set_process(true)
	await settle()

func check_right_lane() -> void:
	var start := ground(-230, 0)
	var id: int = simulation.spawn_debug_car(start, ground(0, 150))
	expect(id >= 0, "Pocket route must exist")
	simulation.set_simulation_speed(4.0)
	var deadline := Time.get_ticks_msec() + 10000
	while simulation.pick_debug_agent(start, 35.0) != id and Time.get_ticks_msec() < deadline:
		await process_frame
	deadline = Time.get_ticks_msec() + 30000
	var prepared := false
	var turned := false
	while Time.get_ticks_msec() < deadline and not turned:
		await process_frame
		var details: Dictionary = simulation.get_agent_inspection(id)
		if not details.get("active", false):
			continue
		var at: Vector3 = details.position
		if at.x < -25.0 and at.x > -90.0 and at.z > 9.8:
			prepared = true
		if at.z > 35.0:
			turned = true
			expect(details.get("last_failure", "").is_empty(), "Pocket turn must retain its route")
	expect(prepared, "Car must enter the kerb-side pocket before the junction")
	expect(turned, "Car must finish its turn from the pocket")
	simulation.set_simulation_speed(0.0)
	if main == null:
		await check_tool_edits()

func check_tool_edits() -> void:
	var road: Node = load("res://scripts/tools/road_tool.gd").new()
	# This isolated controller fixture owns the preview node normally created by _ready.
	road.blueprint_mesh = MeshInstance3D.new()
	road.add_child(road.blueprint_mesh)
	road.simulation_node = simulation
	road.toggle_pocket_edit()
	expect(road.edit_pocket and not road.edit_cross_section, "H enters pocket mode")
	road.pocket_left = true
	road.pocket_right = false
	road.toggle_pocket_side(false)
	expect(road.pocket_right, "K toggles the kerb-side lane")
	road.adjust_pocket_length(1.0, false)
	road.adjust_pocket_length(-1.0, true)
	var edge: int = simulation.get_hovered_edge(-40, 0)
	var preview: Dictionary = simulation.preview_road_pocket(edge, false, true, true, road.pocket_length, road.pocket_taper)
	expect(preview.get("is_valid", false) and not preview.get("junction_preview", {}).get("chunks", []).is_empty(), "Endpoint preview compiles")
	expect(not preview.get("terrain_preview", {}).get("patches", []).is_empty(), "Endpoint preview restores terrain cutouts")
	var result := await finish_request(road.apply_pocket_to_end(edge, false))
	expect(result.get("committed", false), "Tool resize: %s" % result)
	var selected: Dictionary = simulation.get_road_pocket_selection(-40, 0)
	expect(absf(float(selected.get("pocket_m", 0)) - 90.0) < 0.01 and absf(float(selected.get("taper_m", 0)) - 30.0) < 0.01, "Tool commits its preview dimensions")
	road._edit_request_id = 0
	road.pocket_left = false
	road.pocket_right = false
	result = await finish_request(road.apply_pocket_to_end(edge, false))
	expect(result.get("committed", false), "Tool removes both pockets")
	var section: Dictionary = simulation.get_road_cross_section(simulation.get_hovered_edge(-40, 0))
	expect(section.get("fwd_lanes", 0) == 2, "Removal restores the base lane count")
	road.free()

func capture(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float = -0.65) -> void:
	await super.capture("stage5_" + label, pivot, distance, pitch, yaw)
