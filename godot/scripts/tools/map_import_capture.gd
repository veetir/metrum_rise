# SPDX-License-Identifier: GPL-2.0-only

## Windowed, read-only save inspection at fixed north-up orthographic map scales.
## Each image has a matching source-centreline PNG from tools.map_import.render.
extends SceneTree

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() != 3:
		push_error("Expected save path, output directory, before/after prefix")
		quit(2)
		return
	var main: Node = load("res://scenes/Main.tscn").instantiate()
	main.set_script(null)
	root.add_child(main)
	current_scene = main
	await process_frame
	var input: Node = main.get_node("InputManager")
	input.set_process(false)
	input.set_process_input(false)
	root.set_disable_input(true)
	if not input.menu_load_game_from_path(ProjectSettings.globalize_path(args[0])):
		quit(2)
		return
	input.set_simulation_speed(0.0)
	main.get_node("MainUI").hide()
	root.get_node("FpsOverlay").hide()
	var camera: Camera3D = main.get_node("CameraNode")
	camera.set_process(false)
	camera.set_process_input(false)
	camera.set_process_unhandled_input(false)
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.far = 10000.0
	main.get_node("SceneLighting").pin_hour_of_day(12.0)
	DisplayServer.window_set_size(Vector2i(1200, 900))
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	Engine.max_fps = 60
	var views: Variant = JSON.parse_string(FileAccess.get_file_as_string("res://../maps/processed/kuopio_sw/shots/views.json"))
	var output := ProjectSettings.globalize_path(args[1])
	DirAccess.make_dir_recursive_absolute(output)
	for view in views:
		var p := Vector3(view["x"], 1000.0, view["z"])
		camera.set_view_state(PackedFloat32Array([p.x, 100.0, p.z, 0.0, -1.5, 1000.0, 1.0]))
		camera.position = p
		camera.rotation = Vector3(-PI/2.0, 0.0, 0.0)
		camera.size = view["height_m"]
		var stable := 0
		var begin := Time.get_ticks_msec()
		while stable < 40 and Time.get_ticks_msec()-begin < 120000:
			await process_frame
			if not main.get_node("Terrain").has_pending_render_work(false) and not main.get_node("Water").has_pending_render_work(false) and not main.get_node("Vegetation").has_pending_work():
				stable += 1
			else:
				stable = 0
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(output.path_join(args[2]+"_"+view["name"]+".png"))
	quit(0)
