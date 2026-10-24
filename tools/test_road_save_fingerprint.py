# SPDX-License-Identifier: GPL-2.0-only

"""The save comparison must observe physical geometry and reject missing authority."""

from pathlib import Path
import sqlite3
import tempfile
import unittest

from road_save_fingerprint import TABLES, fingerprint


class RoadFingerprintTest(unittest.TestCase):
    def test_geometry_and_topology_are_authority_runtime_prices_are_not(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "roads-sync-fingerprint.sqlite"
            with sqlite3.connect(path) as conn:
                for table in TABLES:
                    conn.execute(f'CREATE TABLE "{table}" (identity INTEGER, value REAL)')
                    conn.execute(f'INSERT INTO "{table}" VALUES (1, 2.0)')
                conn.execute('ALTER TABLE network_edges ADD current_congestion REAL DEFAULT 0')
            before = fingerprint(path)
            with sqlite3.connect(path) as conn:
                conn.execute('UPDATE network_edges SET current_congestion = 0.75')
            self.assertEqual(fingerprint(path), before)
            with sqlite3.connect(path) as conn:
                conn.execute('UPDATE network_edge_geometry SET value = 2.01')
            self.assertNotEqual(fingerprint(path), before)
            with sqlite3.connect(path) as conn:
                conn.execute('DROP TABLE network_edge_geometry')
            with self.assertRaises(ValueError):
                fingerprint(path)


if __name__ == "__main__":
    unittest.main()
