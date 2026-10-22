# SPDX-License-Identifier: GPL-2.0-only

"""Prepare local render-only marking fixtures from read-only player saves.

The full Prisma graph has existing surface failures outside this view. Keep all source
nodes, terrain and exact nearby road geometry; remap only retained edge IDs. The Prisma
candidate copy supplies empty explicit crossing authoring, as importer integration will.
Niirala retains its legacy zebras. Outputs live exclusively under ignored captures/.
"""

import hashlib
import math
from pathlib import Path
import sqlite3


def quoted(name):
    return '"' + name.replace('"', '""') + '"'


def crop_roads(connection, mapping):
    tables = connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table'"
    ).fetchall()
    for (table,) in tables:
        columns = [row[1] for row in connection.execute(f"PRAGMA table_info({quoted(table)})")]
        if "edge_id" not in columns or table == "saved_lanes":
            continue
        rows = connection.execute(f"SELECT * FROM {quoted(table)}").fetchall()
        index = columns.index("edge_id")
        connection.execute(f"DELETE FROM {quoted(table)}")
        for row in rows:
            if row[index] not in mapping:
                continue
            row = list(row)
            row[index] = mapping[row[index]]
            placeholders = ",".join("?" for _ in row)
            connection.execute(f"INSERT INTO {quoted(table)} VALUES ({placeholders})", row)

    rows = connection.execute("SELECT * FROM lane_connections").fetchall()
    connection.execute("DELETE FROM lane_connections")
    for node, source, source_lane, target, target_lane in rows:
        if source in mapping and target in mapping:
            connection.execute(
                "INSERT INTO lane_connections VALUES (?,?,?,?,?)",
                (node, mapping[source], source_lane, mapping[target], target_lane),
            )
    # These snapshots contain no agents or buildings; lane identities are rebuilt on load.
    connection.execute("DELETE FROM saved_lanes")
    connection.execute("DELETE FROM zoning_cell_road_alignments")


def main():
    output = Path(__file__).resolve().parents[1] / "captures/road_marking_rules"
    output.mkdir(parents=True, exist_ok=True)
    saves = Path.home() / "Library/Application Support/Godot/app_userdata/Metrum Rise/saves"
    cases = [
        ("prisma", "prisma.sqlite", -1002.87, 1817.89, 180),
        ("niirala", "niirala-import-2.sqlite", -77.09, 926.68, 260),
    ]
    for label, name, x, z, radius in cases:
        path = saves / name
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        source = sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)
        try:
            points = source.execute(
                "SELECT edge_id,x,z FROM network_edge_geometry WHERE physical=1"
            )
            selected = {edge for edge, px, pz in points if math.hypot(px - x, pz - z) < radius}
            mapping = {old: new for new, old in enumerate(sorted(selected))}
            for phase in ("before", "after"):
                destination = output / f"roads-markings-{label}-{phase}.sqlite"
                destination.unlink(missing_ok=True)
                connection = sqlite3.connect(destination)
                try:
                    source.backup(connection)
                    crop_roads(connection, mapping)
                    if phase == "after" and label == "prisma":
                        connection.execute(
                            "CREATE TABLE road_crosswalks(node_id INTEGER, edge_id INTEGER, "
                            "marked INTEGER, PRIMARY KEY(node_id,edge_id))"
                        )
                    connection.commit()
                finally:
                    connection.close()
        finally:
            source.close()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
        print(f"{label}: {len(selected)} road owners; unchanged source SHA-256 {digest}")


if __name__ == "__main__":
    main()
