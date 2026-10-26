# SPDX-License-Identifier: GPL-2.0-only

## Read-only full-import routing and movement probe through the production vehicle API.
## The source save is never written; debug cars exist only in this diagnostic session.
## METRUM_IMPORT_PROBE_TRIP_FILE optionally adds source-derived trips with XYZ arrays.
extends "res://tests/road_import_capture.gd"

func run() -> void:
	var source := OS.get_environment("METRUM_IMPORT_CAPTURE_SAVE")
	var destination := OS.get_environment("METRUM_IMPORT_PROBE_OUTPUT")
	if source.is_empty() or destination.is_empty():
		expect(false, "Provide the importer-owned save and probe JSON destination")
		quit(failures)
		return
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	expect(simulation.load_game(source), "Cold-load the complete road/path import")
	simulation.set_simulation_speed(0.0)
	var trips: Array[Dictionary] = [
		{"name": "university_motor_underpass", "from": Vector3(-632.051, 0, 1217.01), "to": Vector3(-447.079, 0, 1186.896), "via": Vector3(-548.65, 0, 1260.38)},
		{"name": "university_parking", "from": Vector3(-534.482, 0, 1186.639), "to": Vector3(-580, 0, 1305), "via": Vector3(-502.357, 0, 1206.397)},
		{"name": "luola_roundabout", "from": Vector3(-1440, 0, 1690), "to": Vector3(-1239, 0, 1852), "via": Vector3(-1342.151, 0, 1709.442)},
		{"name": "west_roundabout", "from": Vector3(-75.078, 0, 929.633), "to": Vector3(173.968, 0, 1007.229), "via": Vector3(51.128, 0, 973.101)},
		{"name": "east_roundabout", "from": Vector3(706.277, 0, 1057.233), "to": Vector3(891.704, 0, 1199.944), "via": Vector3(745.712, 0, 1079.876)},
		{"name": "prisma", "from": Vector3(-1097.711, 0, 1831.148), "to": Vector3(-983.664, 0, 1792.277), "via": Vector3(-999.179, 0, 1829.554)},
		{"name": "niirala", "from": Vector3(113.804, 0, 1503.513), "to": Vector3(231.267, 0, 1614.881), "via": Vector3(176.207, 0, 1548.486)}]
	var trip_file := OS.get_environment("METRUM_IMPORT_PROBE_TRIP_FILE")
	if not trip_file.is_empty():
		var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(trip_file))
		if not parsed is Array:
			expect(false, "Trip manifest must contain an array")
			simulation.queue_free()
			await process_frame
			quit(failures)
			return
		for entry: Dictionary in parsed:
			var trip: Dictionary = entry.duplicate(true)
			for key in ["from", "to", "via"]:
				var xyz: Array = trip[key]
				trip[key] = Vector3(xyz[0], xyz[1], xyz[2])
			trips.append(trip)
	for trip in trips:
		trip.id = simulation.spawn_debug_car(trip.from, trip.to)
		trip.position = trip.from
		trip.closest_via_m = INF
		trip.travelled_m = 0.0
		trip.observations = 0
		trip.last_failure = ""
		trip.passed_feature = false
		trip.exit_distance_m = float(trip.get("exit_distance_m", 12.0 if trip.name == "prisma" else 24.0))
		expect(trip.id >= 0, "Full-map car route: %s" % trip.name)
	simulation.set_simulation_speed(4.0)
	var deadline := Time.get_ticks_msec() + 45000
	while Time.get_ticks_msec() < deadline:
		await process_frame
		var complete := true
		for trip in trips:
			if trip.id < 0 or trip.passed_feature:
				continue
			complete = false
			if simulation.pick_debug_agent(trip.position, 50.0) != trip.id:
				continue
			var details: Dictionary = simulation.get_agent_inspection(trip.id)
			if not details.get("active", false):
				continue
			var position: Vector3 = details.position
			trip.travelled_m += Vector2(position.x - trip.position.x, position.z - trip.position.z).length()
			trip.position = position
			trip.closest_via_m = minf(trip.closest_via_m, Vector2(position.x - trip.via.x, position.z - trip.via.z).length())
			var axis := Vector2(trip.to.x - trip.from.x, trip.to.z - trip.from.z).normalized()
			trip.passed_feature = trip.closest_via_m < 24.0 and Vector2(position.x - trip.via.x, position.z - trip.via.z).dot(axis) > trip.exit_distance_m
			trip.observations += 1
			trip.last_failure = details.get("last_failure", "")
		if complete:
			break
	simulation.set_simulation_speed(0.0)
	for trip in trips:
		expect(trip.passed_feature, "Car crosses its intended imported feature: %s" % trip)
		expect(trip.last_failure.is_empty(), "Imported route remains valid: %s" % trip.name)
	var report := FileAccess.open(destination, FileAccess.WRITE)
	report.store_string(JSON.stringify({"save_sha256": FileAccess.get_sha256(source), "trips": trips, "failures": failures}, "\t") + "\n")
	simulation.queue_free()
	await process_frame
	quit(failures)
