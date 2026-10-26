// SPDX-License-Identifier: GPL-2.0-only

//! Section migration, compacted snapshot references and exact new-version resave regressions.

use super::*;

const DROP_SECTIONS: &str = "DROP TABLE road_edge_sections; DROP TABLE road_layout_strips; DROP TABLE road_section_details; DROP TABLE road_layouts;";

fn snapshot(graph: &RegionGraph) -> Connection {
    let maps = build_snapshot_maps(graph, &BuildingAllocator::new(), &AgentSystem::new()).unwrap();
    let mut conn = Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    let tx = conn.transaction().unwrap();
    network::save_network(&tx, graph, &maps).unwrap();
    tx.commit().unwrap();
    conn
}

fn section_rows(conn: &Connection) -> Vec<Vec<rusqlite::types::Value>> {
    let mut result = Vec::new();
    for table in [
        "road_layouts",
        "road_layout_strips",
        "road_section_details",
        "road_edge_sections",
        "network_nodes",
        "network_edges",
        "network_edge_geometry",
        "road_crossing_levels",
        "roundabout_edges",
        "road_lane_tapers",
        "road_bus_lanes",
        "road_crosswalks",
        "lane_connections",
    ] {
        let columns = conn
            .prepare(&format!("SELECT * FROM {table}"))
            .unwrap()
            .column_count();
        let order = (1..=columns)
            .map(|i| i.to_string())
            .collect::<Vec<_>>()
            .join(",");
        let mut query = conn
            .prepare(&format!("SELECT * FROM {table} ORDER BY {order}"))
            .unwrap();
        let rows = query
            .query_map([], |row| (0..columns).map(|i| row.get(i)).collect())
            .unwrap();
        result.extend(rows.map(Result::unwrap));
    }
    result
}

/// Historical decode followed by new-version reload/resave, used by the raw compiled proof.
pub(crate) fn migrate_compile_fixture(source: &RegionGraph) -> RegionGraph {
    let mut source = source.clone();
    source.resolve_legacy_road_details(0..source.edge_count());
    let old = snapshot(&source);
    old.execute_batch(DROP_SECTIONS).unwrap();
    old.execute(
        "INSERT INTO save_meta VALUES (109,0,'road-section-legacy-fixture')",
        [],
    )
    .unwrap();
    let migrated = network::load_graph(&old).unwrap();
    assert!(migrated.validate_road_sections().is_ok());
    let current = snapshot(&migrated);
    current
        .execute(
            "INSERT INTO save_meta VALUES (?1,0,'road-section-fixture')",
            [SAVE_VERSION],
        )
        .unwrap();
    let loaded = network::load_graph(&current).unwrap();
    let resaved = snapshot(&loaded);
    assert_eq!(
        section_rows(&current),
        section_rows(&resaved),
        "load -> 110 -> reload -> resave section rows"
    );
    loaded
}

#[test]
fn section_migration_compacts_tombstones_and_resaves_identically() {
    let (mut graph, keep) = snapshot_test_graph();
    graph.resolve_legacy_road_details(0..graph.edge_count());
    let expected_layout = graph.road_layout(keep).clone();
    let expected_detail = graph.road_section_detail(keep).cloned();
    let conn = snapshot(&graph);
    let loaded = network::load_graph(&conn).unwrap();
    assert_eq!(loaded.road_layout(0), &expected_layout);
    assert_eq!(loaded.road_section_detail(0), expected_detail.as_ref());
    assert_eq!(
        loaded.edge_count(),
        graph.edges().iter().filter(|e| !e.deleted).count()
    );
    assert_eq!(section_rows(&conn), section_rows(&snapshot(&loaded)));
    let migrated = migrate_compile_fixture(&graph);
    assert_eq!(migrated.road_layout(0), &expected_layout);
}

#[test]
fn section_load_rejects_corruption_before_publication() {
    let (mut graph, _) = snapshot_test_graph();
    graph.resolve_legacy_road_details(0..graph.edge_count());
    for mutation in [
        "UPDATE road_edge_sections SET layout_id=999",
        "UPDATE road_edge_sections SET detail_id=999",
        "UPDATE road_edge_sections SET access=0",
        "UPDATE road_edge_sections SET class=255",
        "UPDATE road_edge_sections SET overrides=32768",
        "DELETE FROM road_edge_sections",
        "DELETE FROM road_layout_strips WHERE ordinal=1",
        "UPDATE road_layouts SET format_version=999",
        "UPDATE road_layout_strips SET ordinal=7 WHERE ordinal=1",
        "UPDATE road_layouts SET layout_blob=x'00'",
        "DROP TABLE road_layouts",
    ] {
        let conn = snapshot(&graph);
        conn.execute(
            "INSERT INTO save_meta VALUES (?1,0,'section-corruption')",
            [SAVE_VERSION],
        )
        .unwrap();
        conn.execute_batch(mutation).unwrap();
        assert!(network::load_graph(&conn).is_err(), "accepted {mutation}");
    }
}

#[test]
fn section_save_validates_authority_instead_of_repairing_it() {
    let (mut graph, id) = snapshot_test_graph();
    graph.edge_mut(id).width += 0.125;
    let maps = build_snapshot_maps(&graph, &BuildingAllocator::new(), &AgentSystem::new()).unwrap();
    let mut conn = Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    let tx = conn.transaction().unwrap();
    assert!(network::save_network(&tx, &graph, &maps).is_err());
}

pub(super) fn legacy_fixture_schema(conn: &Connection) {
    conn.execute_batch(SCHEMA).unwrap();
    // Frozen geometry SQL predates v110 and has no profile tables or header of its own.
    conn.execute(
        "INSERT INTO save_meta VALUES (109,0,'pre-section-geometry-fixture')",
        [],
    )
    .unwrap();
}
