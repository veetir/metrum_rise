# SPDX-License-Identifier: GPL-2.0-only

## Checkpoint retry regression: acknowledged refusals resume once per sweep and stage.
extends "res://scripts/tools/map_import_apply.gd"

var calls: Array[String] = []

func _run() -> void:
	# A checkpoint after one accepted piece and refusal A, before B and C.
	_results = JSON.parse_string(JSON.stringify({"roads": [], "source_retry_sweeps": {
		"settled": {"sweep": 0, "recovered": true, "complete": false}}}))
	for id in ["A", "B", "C"]:
		var failure := {"points_xz": [[0.0,0.0],[10.0,0.0]], "reason": "synthetic"}
		if id == "A":
			failure["retry_pass"] = "settled/0"
		var record := {"id": id, "attempts": [], "pieces": [], "failed_pieces": [failure]}
		_results["roads"].append(record)
		_deferred.append({"record": record, "failure": failure, "stroke": {"id": id}})
	await _retry_deferred(root, "settled")
	assert(calls == ["B:deferred_exact/0", "C:deferred_exact/0", "A:deferred_exact/1", "C:deferred_exact/1"])
	assert(_results["roads"][1]["status"] == "accepted")
	assert(_deferred.size() == 2)
	assert(_results["source_retry_sweeps"]["settled"]["complete"])
	assert(_results["source_retry_sweeps"]["settled"]["sweep"] == 2)
	# A restart at the completed-stage checkpoint must not restart that sweep.
	await _retry_deferred(root, "settled")
	assert(calls.size() == 4)
	# Motor preparation is a distinct stage: a marker from settled cannot skip it.
	await _retry_deferred(root, "motor")
	assert(calls.slice(4) == ["A:deferred_exact/0", "C:deferred_exact/0"])
	assert(_deferred.size() == 2)
	quit(0)

func _attempt(_simulation: Node, stroke: Dictionary, points: Array, record: Dictionary, strategy: String) -> String:
	calls.append(String(stroke["id"])+":"+strategy)
	record["attempts"].append({"points_xz": points})
	return "" if stroke["id"] == "B" else "synthetic_refusal"

func _checkpoint(_simulation: Node) -> void:
	pass

func _write_results() -> bool:
	return true
