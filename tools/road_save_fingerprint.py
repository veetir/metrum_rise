#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only

"""Compare road authority and directed lane topology in read-only snapshot copies.

Run each build's load/save path first, then compare the resulting snapshots. The
retired congestion and frontage-delay counters are runtime prices, not topology.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sqlite3

TABLES = (
    "network_nodes", "network_edges", "network_edge_geometry",
    "lane_connections", "road_lane_tapers", "road_crossing_levels",
    "roundabout_edges", "road_bus_lanes", "road_crosswalks", "saved_lanes",
)
RUNTIME_COLUMNS = {
    "network_edges": {"current_congestion"},
    "saved_lanes": {"frontage_delay_penalty_s", "live_frontage_delay_s"},
}


def fingerprint(path: Path) -> dict:
    """Hash every authored road field and saved lane identity, without writing."""
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as conn:
        present = {row[0] for row in conn.execute(
            "SELECT name FROM sqlite_master WHERE type = 'table'"
        )}
        required = {"network_nodes", "network_edges", "network_edge_geometry", "lane_connections", "saved_lanes"}
        if required - present:
            raise ValueError(f"Missing road tables: {sorted(required - present)}")
        result = {}
        for table in TABLES:
            if table not in present:
                result[table] = {"columns": [], "rows": 0, "sha256": None}
                continue
            columns = sorted(row[1] for row in conn.execute(f'PRAGMA table_info("{table}")')
                             if row[1] not in RUNTIME_COLUMNS.get(table, set()))
            selection = ", ".join(f'"{column}"' for column in columns)
            rows = list(conn.execute(f'SELECT {selection} FROM "{table}"'))
            # JSON's float representation preserves binary64 values, including saved f32s.
            encoded = sorted(json.dumps(row, separators=(",", ":"), allow_nan=False)
                             for row in rows)
            digest = hashlib.sha256(json.dumps([columns, encoded], separators=(",", ":")).encode())
            result[table] = {"columns": columns, "rows": len(rows), "sha256": digest.hexdigest()}
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    args = parser.parse_args()
    before, after = fingerprint(args.before), fingerprint(args.after)
    differences = {table: {"before": before[table], "after": after[table]}
                   for table in TABLES if before[table] != after[table]}
    print(json.dumps({"matches": not differences, "fingerprint": before,
                      "differences": differences}, indent=2))
    return bool(differences)


if __name__ == "__main__":
    raise SystemExit(main())
