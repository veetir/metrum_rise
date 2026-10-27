# SPDX-License-Identifier: GPL-2.0-only

"""Create read-only-source ROAD-73 local render excerpts under ignored captures/.

All retained road rows and terrain bytes come from the source unchanged. Only edge IDs
are compacted. Local excerpts preserve complete incident roads around the camera; the
separate two-edge KYS slip-bend excerpt isolates its crossfall defect.
"""

import hashlib
import json
import math
from pathlib import Path
import sqlite3

from road_marking_capture_fixtures import crop_roads


def main():
    output = Path(__file__).resolve().parents[1] / "captures/road_junction_geometry/fixtures"
    output.mkdir(parents=True, exist_ok=True)
    saves = Path.home() / "Library/Application Support/Godot/app_userdata/Metrum Rise/saves"
    cases = [
        ("kys", "kys-pond.sqlite", -73.19, 922.70, 220.0, None),
        ("kys-slip", "kys-pond.sqlite", -41.535, 920.851, 0.0, {123, 137}),
        ("bioteknia", "bioteknia.sqlite", -778.57, 1471.59, 220.0, None),
        ("south-curve", "kuopio-sw-pockets.sqlite", 1205.44, 3415.43, 160.0, None),
        ("west-curve", "kuopio-sw-pockets.sqlite", -1546.87, 2281.71, 160.0, None),
        ("east-curve", "kuopio-sw-pockets.sqlite", 803.42, 1060.13, 160.0, None),
    ]
    manifest = []
    for label, name, x, z, radius, explicit in cases:
        path = saves / name
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as source:
            selected = explicit
            if selected is None:
                selected = {
                    edge for edge, px, pz in source.execute(
                        "SELECT edge_id,x,z FROM network_edge_geometry WHERE physical=1"
                    ) if math.hypot(px - x, pz - z) < radius
                }
            mapping = {old: new for new, old in enumerate(sorted(selected))}
            destination = output / f"roads-junction-{label}.sqlite"
            destination.unlink(missing_ok=True)
            with sqlite3.connect(destination) as connection:
                source.backup(connection)
                crop_roads(connection, mapping)
            manifest.append({
                "case": label, "source": name, "sha256": digest,
                "x": x, "z": z, "radius": radius,
                "source_edges": sorted(selected), "mapping": mapping,
            })
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
