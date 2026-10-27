# SPDX-License-Identifier: GPL-2.0-only

## Per-lane bus access controls; Rust owns validation, routing, rendering and undo.
extends VBoxContainer

var simulation: Node
var select_tool: Node
var edge_id: int = -1
var busy := false
var status: Label

func setup(simulation_node: Node, selection: Node) -> void:
	simulation = simulation_node
	select_tool = selection

func show_edge(id: int) -> void:
	edge_id = id
	status = null
	for child in get_children():
		remove_child(child)
		child.queue_free()
	if id < 0 or simulation == null:
		return
	var section: Dictionary = simulation.get_road_cross_section(id)
	for forward in [true, false]:
		var count: int = section.get("fwd_lanes" if forward else "bkw_lanes", 0)
		var mask: int = section.get("bus_forward_mask" if forward else "bus_backward_mask", 0)
		for lane in count:
			var check := CheckBox.new()
			check.text = "%s lane %d: bus only" % ["Forward" if forward else "Backward", lane + 1]
			check.tooltip_text = "Lanes are numbered from the median. Cars can cross a bus lane through legal junction turns."
			check.set_pressed_no_signal(mask & (1 << lane) != 0)
			check.disabled = busy
			check.toggled.connect(_changed.bind(id, forward, lane))
			add_child(check)
	status = Label.new()
	status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	add_child(status)

func _changed(enabled: bool, id: int, forward: bool, lane: int) -> void:
	if busy:
		return
	busy = true
	for child in get_children():
		if child is CheckBox:
			child.disabled = true
	var section: Dictionary = simulation.get_road_cross_section(id)
	var masks := [int(section.get("bus_forward_mask", 0)), int(section.get("bus_backward_mask", 0))]
	var direction := 0 if forward else 1
	masks[direction] = (masks[direction] | (1 << lane)) if enabled else (masks[direction] & ~(1 << lane))
	var request: int = simulation.change_road_bus_lanes(id, masks[0], masks[1])
	var result: Dictionary = {}
	var deadline := Time.get_ticks_msec() + 60000
	while Time.get_ticks_msec() < deadline:
		var response: Variant = simulation.get_road_commit_result(request)
		if response is Dictionary:
			result = response
			break
		await get_tree().process_frame
	busy = false
	show_edge(edge_id)
	if not result.get("committed", false) and is_instance_valid(status):
		if result.get("reason", "") == "bus_lane_overlaps_taper":
			status.text = "Bus lanes cannot be combined with a taper yet."
		elif result.is_empty():
			status.text = "Bus lane edit timed out."
		else:
			status.text = "Could not change bus lanes. Select the road and try again."
	if select_tool != null:
		var roads := select_tool.get_node_or_null("../RoadTool")
		if roads != null:
			roads.update_main_mesh()
