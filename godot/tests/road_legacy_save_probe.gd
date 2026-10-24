# SPDX-License-Identifier: GPL-2.0-only

## Read isolated legacy copies and round-trip only into task-owned output files.
extends SceneTree

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	var simulation: Node = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	var folder := OS.get_environment("METRUM_MIGRATION_ARTIFACTS")
	var output := OS.get_environment("METRUM_MIGRATION_OUTPUT")
	DirAccess.make_dir_recursive_absolute(output)
	var results: Array[Dictionary] = []
	var failures := 0
	var names := ["kuopio-sw-pockets", "kys-pond", "bioteknia", "prisma", "uni-underpass"]
	if OS.get_environment("METRUM_MIGRATION_UPSTREAM") == "1":
		names.append("upstream")
	if OS.get_environment("METRUM_MIGRATION_EARLY") == "1":
		names.append("upstream-v59")
	for name in names:
		var source := folder.path_join("copies/roads-sync-" + name + ".sqlite")
		var row := {"name": name, "load": simulation.load_game(source)}
		if row.load:
			var destination := output.path_join("roads-sync-" + name + ".sqlite")
			row.save = simulation.save_game(destination)
			row.reload = row.save and simulation.load_game(destination)
			if row.reload:
				row.resave = simulation.save_game(output.path_join("roads-sync-" + name + "-roundtrip.sqlite"))
		if not row.get("resave", false):
			failures += 1
		results.append(row)
	var file := FileAccess.open(output.path_join("probe.json"), FileAccess.WRITE)
	file.store_string(JSON.stringify(results, "\t"))
	simulation.queue_free()
	await process_frame
	quit(failures)
