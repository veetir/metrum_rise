# SPDX-License-Identifier: GPL-2.0-only

"""Offline, staged map import; source data and generated products must remain local.

Run ``python3.12 -m tools.map_import --help`` for the preparation CLI. The
GDScript runner is deliberately not executed by this package. It uses the game's
player validation/commit/zoning/save APIs; Python never authors a city database.

Extension points: add a SourceAdapter yielding SourceFeature records, extend the
single CAPABILITIES table, then extend application capabilities as game features
ship. neutral.json retains geometry, tags, topology and relation membership even
for features the current mapping rejects. No simulation decisions live here.
"""
