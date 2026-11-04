# SPDX-License-Identifier: GPL-2.0-only

## Top-down road tile art, using the builder's glyphs and Rust's per-part widths. Geometry is
## prepared once per type refresh; painting is O(parts), independent of world population.
extends RefCounted

const Glyph = preload("res://scripts/ui/road_builder/road_part_glyph.gd")

var _parts: Array = []
var _widths := PackedFloat32Array()
var _total := 0.0

## Uses a get_road_types entry, including the widths shared with check_road_layout.
func setup(type: Dictionary) -> void:
	_parts = Array(type.get("parts", [])).duplicate()
	_widths = type.get("part_widths_m", PackedFloat32Array())
	# As in RoadBuilderStrip, rural roads show Rust's shoulder allowance at both edges.
	var shoulder := 0.5 * (float(type.get("total_m", 0.0)) - float(type.get("carriageway_m", 0.0)))
	if not bool(type.get("sidewalks", true)) and shoulder > 0.0:
		_parts.push_front({"kind": "shoulder"})
		_parts.append({"kind": "shoulder"})
		var widths := PackedFloat32Array([shoulder])
		widths.append_array(_widths)
		widths.append(shoulder)
		_widths = widths
	_total = 0.0
	for width in _widths:
		_total += width

## Draws each surface and directional glyph inside the supplied square.
func paint(canvas: CanvasItem, rect: Rect2) -> void:
	if _total <= 0.0 or _parts.size() != _widths.size():
		return
	# Cap pixels/metre so the footpath does not look as wide as a four-lane road.
	var px_per_m := minf(rect.size.x / _total, 7.0)
	var x := rect.get_center().x - _total * px_per_m * 0.5
	for index in _parts.size():
		var band := Rect2(x, rect.position.y, _widths[index] * px_per_m, rect.size.y)
		Glyph.draw_surface(canvas, band, _parts[index])
		var kind := String(_parts[index].get("kind", ""))
		if kind == "car_lane" or kind == "sidewalk" or kind == "footpath":
			var side := minf(band.size.x - 2.0, 20.0)
			if side > 2.0:
				var box := Rect2(band.get_center() - Vector2.ONE * side * 0.5, Vector2.ONE * side)
				Glyph.draw_glyph(canvas, box, _parts[index])
		x = band.end.x
