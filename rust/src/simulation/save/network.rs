// SPDX-License-Identifier: GPL-2.0-only

//! Road graph and lane connection serialization.

use crate::config::HIGH_SPEED_ROAD_THRESHOLD_MS;
use crate::simulation::network::TransitNetwork;
use crate::simulation::network::graph::{Edge, RegionGraph, RoadLaneTaper, RoadMedian};
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::Vector3;
use rusqlite::{Connection, Transaction, params};
use std::collections::HashMap;

use super::schema::*;
use super::{SaveLoadError, SaveLoadResult, SnapshotMaps};
use super::{i64_to_i8, i64_to_u8, i64_to_u32, i64_to_usize, usize_to_i64};

pub(super) fn save_network(
    tx: &Transaction,
    graph: &RegionGraph,
    maps: &SnapshotMaps,
) -> SaveLoadResult<()> {
    // 1. Nodes
    {
        let mut node_rows: Vec<(u32, u32)> = maps
            .node_old_to_new
            .iter()
            .map(|(&old, &new)| (new, old))
            .collect();
        node_rows.sort_by_key(|&(new, _)| new);
        let mut stmt = tx.prepare(
            "INSERT INTO network_nodes(node_id, x, y, z, node_type) VALUES (?1, ?2, ?3, ?4, ?5)",
        )?;
        for (saved_id, old_id) in node_rows {
            let node = graph.node(old_id);
            stmt.execute(params![
                i64::from(saved_id),
                node.pos.x,
                node.pos.y,
                node.pos.z,
                node_type_to_i64(node.node_type)
            ])?;
        }
    }

    {
        let mut stmt = tx.prepare("INSERT INTO road_crossing_levels VALUES (?1,?2)")?;
        for (id, edge) in graph.edges().iter().enumerate() {
            if edge.crossing_level != 0
                && let Some(saved) = maps.edge_old_to_new.get(&id)
            {
                stmt.execute(params![usize_to_i64(*saved)?, edge.crossing_level])?;
            }
        }
    }

    {
        let mut stmt = tx.prepare("INSERT INTO roundabout_edges VALUES (?1)")?;
        for (id, edge) in graph.edges().iter().enumerate() {
            if edge.roundabout
                && let Some(saved) = maps.edge_old_to_new.get(&id)
            {
                stmt.execute([usize_to_i64(*saved)?])?;
            }
        }
    }

    // 2. Edges
    {
        let mut stmt = tx.prepare("INSERT INTO network_edges(edge_id, start_node, end_node, primary_type, allowed_types, class, width, fwd_lanes, bkw_lanes, speed_limit, base_cost, physical_length, current_congestion, start_clip, end_clip, no_building_spawn, vehicle_frontage_access, median_style, median_width, median_start_setback, median_end_setback) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16, ?17, ?18, ?19, ?20, ?21)")?;
        for (old_id, edge) in graph.edges().iter().enumerate() {
            let Some(&saved_id) = maps.edge_old_to_new.get(&old_id) else {
                continue;
            };
            let start = maps
                .node_old_to_new
                .get(&canonical_existing_node(graph, edge.start_node)?)
                .copied()
                .ok_or_else(|| SaveLoadError::custom("missing saved start node"))?;
            let end = maps
                .node_old_to_new
                .get(&canonical_existing_node(graph, edge.end_node)?)
                .copied()
                .ok_or_else(|| SaveLoadError::custom("missing saved end node"))?;
            stmt.execute(params![
                usize_to_i64(saved_id)?,
                i64::from(start),
                i64::from(end),
                transit_type_to_i64(edge.primary_type),
                i64::from(edge.allowed_types),
                edge_class_to_i64(edge.class),
                edge.width,
                i64::from(edge.fwd_lanes),
                i64::from(edge.bkw_lanes),
                edge.speed_limit,
                edge.base_cost,
                edge.physical_length,
                edge.current_congestion,
                edge.start_clip,
                edge.end_clip,
                i64::from(edge.no_building_spawn),
                vehicle_frontage_access_to_i64(edge.vehicle_frontage_access),
                edge.median.map_or(0, |median| median.style as i64),
                edge.median.map_or(0.0, |median| median.width_m),
                edge.median.map_or(0.0, |median| median.start_setback_m),
                edge.median.map_or(0.0, |median| median.end_setback_m)
            ])?;
        }
    }

    {
        let mut stmt =
            tx.prepare("INSERT INTO road_lane_tapers VALUES (?1,?2,?3,?4,?5,?6,?7,?8)")?;
        for (old_id, edge) in graph.edges().iter().enumerate() {
            let Some(&saved_id) = maps.edge_old_to_new.get(&old_id) else {
                continue;
            };
            if let Some(ends) = &edge.lane_tapers {
                for (end, taper) in ends.iter().enumerate() {
                    if let Some(taper) = taper {
                        stmt.execute(params![
                            usize_to_i64(saved_id)?,
                            end as i64,
                            taper.length_m,
                            i64::from(taper.from_fwd),
                            i64::from(taper.from_bkw),
                            taper.from_median_m,
                            i64::from(taper.left_pocket),
                            i64::from(taper.right_pocket)
                        ])?;
                    }
                }
            }
        }
    }

    {
        let mut stmt = tx.prepare("INSERT INTO road_bus_lanes VALUES (?1,?2,?3)")?;
        for (old_id, edge) in graph.edges().iter().enumerate() {
            if edge.bus_lanes != [0; 2]
                && let Some(&saved_id) = maps.edge_old_to_new.get(&old_id)
            {
                stmt.execute(params![
                    usize_to_i64(saved_id)?,
                    edge.bus_lanes[0],
                    edge.bus_lanes[1]
                ])?;
            }
        }
    }

    {
        let mut stmt =
            tx.prepare("INSERT INTO road_crosswalks(node_id,edge_id,marked) VALUES (?1,?2,?3)")?;
        let mut rows = Vec::new();
        for (&old, &saved) in &maps.node_old_to_new {
            for (&edge, &marked) in &graph.node(old).crosswalk_overrides {
                if let Some(&edge) = maps.edge_old_to_new.get(&edge) {
                    rows.push((saved, edge, marked));
                }
            }
        }
        rows.sort_unstable();
        for (node, edge, marked) in rows {
            stmt.execute(params![node, usize_to_i64(edge)?, marked])?;
        }
    }

    // 3. Geometry
    {
        let mut stmt = tx.prepare("INSERT INTO network_edge_geometry(edge_id, point_index, x, y, z, physical) VALUES (?1, ?2, ?3, ?4, ?5, ?6)")?;
        for (old_id, edge) in graph.edges().iter().enumerate() {
            let Some(&saved_id) = maps.edge_old_to_new.get(&old_id) else {
                continue;
            };
            for (idx, p) in edge.geometry.iter().enumerate() {
                stmt.execute(params![
                    usize_to_i64(saved_id)?,
                    usize_to_i64(idx)?,
                    p.x,
                    p.y,
                    p.z,
                    false
                ])?;
            }
            for (idx, p) in edge.physical_geometry.iter().enumerate() {
                stmt.execute(params![
                    usize_to_i64(saved_id)?,
                    usize_to_i64(idx)?,
                    p.x,
                    p.y,
                    p.z,
                    true
                ])?;
            }
        }
    }

    // 4. Lane Connections
    {
        let mut stmt = tx.prepare("INSERT INTO lane_connections(node_id, from_edge, from_lane, to_edge, to_lane) VALUES (?1, ?2, ?3, ?4, ?5)")?;
        for (&old_node, &saved_node) in &maps.node_old_to_new {
            let node = graph.node(old_node);
            for (&(from_e, from_l), targets) in &node.lane_connections {
                let Some(&saved_from) = maps.edge_old_to_new.get(&from_e) else {
                    continue;
                };
                for &(to_e, to_l) in targets {
                    let Some(&saved_to) = maps.edge_old_to_new.get(&to_e) else {
                        continue;
                    };
                    stmt.execute(params![
                        i64::from(saved_node),
                        usize_to_i64(saved_from)?,
                        i64::from(from_l),
                        usize_to_i64(saved_to)?,
                        i64::from(to_l)
                    ])?;
                }
            }
        }
    }
    Ok(())
}

pub(super) fn load_graph(conn: &Connection) -> SaveLoadResult<RegionGraph> {
    let mut graph = RegionGraph::new();
    {
        let mut stmt =
            conn.prepare("SELECT node_id, x, y, z, node_type FROM network_nodes ORDER BY node_id")?;
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let node_id = i64_to_u32(row.get(0)?)?;
            if node_id != graph.node_count() as u32 {
                return Err(SaveLoadError::custom(format!(
                    "network node ids must be contiguous; found {} after {} nodes",
                    node_id,
                    graph.node_count()
                )));
            }
            graph.add_node(
                Vector3::new(row.get(1)?, row.get(2)?, row.get(3)?),
                node_type_from_i64(row.get(4)?)?,
            );
        }
    }

    let (mut geometry, mut physical_geometry) = (HashMap::new(), HashMap::new());
    {
        let mut stmt = conn.prepare("SELECT edge_id, point_index, x, y, z, physical FROM network_edge_geometry ORDER BY edge_id, physical, point_index")?;
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let eid = i64_to_usize(row.get(0)?)?;
            let p = Vector3::new(row.get(2)?, row.get(3)?, row.get(4)?);
            let points = if row.get::<_, bool>(5)? {
                physical_geometry.entry(eid).or_insert_with(Vec::new)
            } else {
                geometry.entry(eid).or_insert_with(Vec::new)
            };
            if i64_to_usize(row.get(1)?)? != points.len() {
                return Err(SaveLoadError::custom("non-contiguous edge geometry points"));
            }
            points.push(p);
        }
    }

    {
        // Old schemas have no separator columns. Inspect once, never per edge.
        let has_medians: bool = conn.query_row("SELECT count(*) != 0 FROM pragma_table_info('network_edges') WHERE name = 'median_style'", [], |row| row.get(0))?;
        let median_columns = if has_medians {
            "median_style, median_width, median_start_setback, median_end_setback"
        } else {
            "0, 0.0, 0.0, 0.0"
        };
        let sql = format!(
            "SELECT edge_id, start_node, end_node, primary_type, allowed_types, class, width, fwd_lanes, bkw_lanes, speed_limit, base_cost, physical_length, current_congestion, start_clip, end_clip, no_building_spawn, vehicle_frontage_access, {median_columns} FROM network_edges ORDER BY edge_id"
        );
        let mut stmt = conn.prepare(&sql)?;
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let eid = i64_to_usize(row.get(0)?)?;
            if eid != graph.edge_count() {
                return Err(SaveLoadError::custom(format!(
                    "network edge ids must be contiguous; found {} after {} edges",
                    eid,
                    graph.edge_count()
                )));
            }
            let speed_limit = row.get(9)?;
            let no_building_spawn =
                row.get::<_, i64>(15)? != 0 || speed_limit >= HIGH_SPEED_ROAD_THRESHOLD_MS;
            let start_node = i64_to_u32(row.get(1)?)?;
            let end_node = i64_to_u32(row.get(2)?)?;
            if start_node as usize >= graph.node_count() || end_node as usize >= graph.node_count()
            {
                return Err(SaveLoadError::custom("edge references out-of-range node"));
            }
            let mut median =
                RoadMedian::from_tool(row.get(17)?, row.get(18)?).map_err(SaveLoadError::custom)?;
            if let Some(median) = &mut median {
                median.start_setback_m = row.get(19)?;
                median.end_setback_m = row.get(20)?;
                median.validate().map_err(SaveLoadError::custom)?;
            }
            let edge = Edge {
                crossing_level: 0,
                roundabout: false,
                lane_tapers: None,
                bus_lanes: [0; 2],
                median,
                start_node,
                end_node,
                primary_type: transit_type_from_i64(row.get(3)?)?,
                allowed_types: i64_to_u8(row.get(4)?)?,
                class: edge_class_from_i64(row.get(5)?)?,
                width: row.get(6)?,
                fwd_lanes: i64_to_u8(row.get(7)?)?,
                bkw_lanes: i64_to_u8(row.get(8)?)?,
                speed_limit,
                base_cost: row.get(10)?,
                physical_length: row.get(11)?,
                current_congestion: row.get(12)?,
                start_clip: row.get(13)?,
                end_clip: row.get(14)?,
                geometry: geometry.remove(&eid).unwrap_or_default(),
                physical_geometry: physical_geometry.remove(&eid).unwrap_or_default(),
                deleted: false,
                no_building_spawn,
                vehicle_frontage_access: vehicle_frontage_access_from_i64(row.get(16)?)?,
            };
            edge.validate_cross_section(edge.fwd_lanes, edge.bkw_lanes, median)
                .map_err(SaveLoadError::custom)?;
            graph.add_edge(edge);
        }
    }

    {
        let exists: bool = conn.query_row("SELECT count(*) != 0 FROM sqlite_master WHERE type='table' AND name='roundabout_edges'", [], |row| row.get(0))?;
        if exists {
            let mut stmt = conn.prepare("SELECT edge_id FROM roundabout_edges ORDER BY edge_id")?;
            for row in stmt.query_map([], |row| row.get::<_, i64>(0))? {
                let id = i64_to_usize(row?)?;
                if id >= graph.edge_count() {
                    return Err(SaveLoadError::custom("invalid roundabout edge"));
                }
                let edge = graph.edge_mut(id);
                if edge.fwd_lanes != 1
                    || edge.bkw_lanes != 0
                    || edge.class != crate::simulation::network::types::EdgeClass::Standard
                {
                    return Err(SaveLoadError::custom("invalid roundabout carriageway"));
                }
                edge.roundabout = true;
            }
        }
    }

    {
        let exists: bool = conn.query_row("SELECT count(*) != 0 FROM sqlite_master WHERE type='table' AND name='road_crossing_levels'", [], |row| row.get(0))?;
        if exists {
            let mut stmt =
                conn.prepare("SELECT edge_id,level FROM road_crossing_levels ORDER BY edge_id")?;
            let mut rows = stmt.query([])?;
            while let Some(row) = rows.next()? {
                let id = i64_to_usize(row.get(0)?)?;
                if id >= graph.edge_count() {
                    return Err(SaveLoadError::custom(
                        "crossing level references missing road",
                    ));
                }
                graph.edge_mut(id).crossing_level = row.get(1)?;
            }
        }
    }
    {
        let exists: bool = conn.query_row("SELECT count(*) != 0 FROM sqlite_master WHERE type='table' AND name='road_lane_tapers'", [], |row| row.get(0))?;
        if exists {
            let mut stmt = conn.prepare("SELECT edge_id,end_index,length_m,from_fwd,from_bkw,from_median,left_pocket,right_pocket FROM road_lane_tapers ORDER BY edge_id,end_index")?;
            let mut rows = stmt.query([])?;
            while let Some(row) = rows.next()? {
                let id = i64_to_usize(row.get(0)?)?;
                let end = i64_to_usize(row.get(1)?)?;
                if id >= graph.edge_count() || end > 1 {
                    return Err(SaveLoadError::custom("invalid lane taper owner"));
                }
                let taper = RoadLaneTaper {
                    length_m: row.get(2)?,
                    from_fwd: i64_to_u8(row.get(3)?)?,
                    from_bkw: i64_to_u8(row.get(4)?)?,
                    from_median_m: row.get(5)?,
                    left_pocket: row.get(6)?,
                    right_pocket: row.get(7)?,
                };
                let edge = graph.edge(id);
                if !taper.length_m.is_finite()
                    || taper.length_m < 2.0
                    || taper.length_m > edge.physical_length
                    || taper.from_fwd > 4
                    || taper.from_bkw > 4
                    || taper.from_fwd + taper.from_bkw == 0
                    || !taper.from_median_m.is_finite()
                    || !(0.0..=12.0).contains(&taper.from_median_m)
                    || (taper.left_pocket
                        && edge
                            .median
                            .is_none_or(|m| m.width_m < crate::config::LANE_WIDTH + 0.3))
                {
                    return Err(SaveLoadError::custom("invalid lane taper dimensions"));
                }
                let node = if end == 0 {
                    edge.start_node
                } else {
                    edge.end_node
                };
                if graph.node(node).node_type
                    != crate::simulation::network::types::NodeType::LaneTransition
                {
                    return Err(SaveLoadError::custom("lane taper requires transition node"));
                }
                graph
                    .edge_mut(id)
                    .lane_tapers
                    .get_or_insert_with(|| Box::new([None, None]))[end] = Some(taper);
            }
        }
    }

    {
        let exists: bool = conn.query_row(
            "SELECT count(*) != 0 FROM sqlite_master WHERE type='table' AND name='road_bus_lanes'",
            [],
            |row| row.get(0),
        )?;
        if exists {
            let mut stmt = conn.prepare(
                "SELECT edge_id,forward_mask,backward_mask FROM road_bus_lanes ORDER BY edge_id",
            )?;
            let mut rows = stmt.query([])?;
            while let Some(row) = rows.next()? {
                let id = i64_to_usize(row.get(0)?)?;
                let masks = [i64_to_u8(row.get(1)?)?, i64_to_u8(row.get(2)?)?];
                graph
                    .validate_bus_lane_edit(id, masks)
                    .map_err(SaveLoadError::custom)?;
                graph.edge_mut(id).bus_lanes = masks;
            }
        }
    }

    {
        let mut stmt = conn.prepare("SELECT node_id, from_edge, from_lane, to_edge, to_lane FROM lane_connections ORDER BY node_id, from_edge, from_lane, to_edge, to_lane")?;
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let nid = i64_to_u32(row.get(0)?)?;
            let fe = i64_to_usize(row.get(1)?)?;
            let fl = i64_to_i8(row.get(2)?)?;
            let te = i64_to_usize(row.get(3)?)?;
            let tl = i64_to_i8(row.get(4)?)?;
            if nid as usize >= graph.node_count()
                || fe >= graph.edge_count()
                || te >= graph.edge_count()
            {
                return Err(SaveLoadError::custom(
                    "lane connection references out-of-range node or edge",
                ));
            }
            graph.add_lane_connection(nid, fe, fl, te, tl);
        }
    }
    let has_crosswalks: bool = conn.query_row(
        "SELECT count(*) != 0 FROM sqlite_master WHERE type='table' AND name='road_crosswalks'",
        [],
        |row| row.get(0),
    )?;
    if has_crosswalks {
        let mut stmt = conn.prepare(
            "SELECT node_id,edge_id,marked FROM road_crosswalks ORDER BY node_id,edge_id",
        )?;
        let mut rows = stmt.query([])?;
        while let Some(row) = rows.next()? {
            let node = i64_to_u32(row.get(0)?)?;
            let edge = i64_to_usize(row.get(1)?)?;
            let marked: i64 = row.get(2)?;
            if node as usize >= graph.node_count()
                || !graph
                    .get_edge(edge)
                    .is_some_and(|e| e.start_node == node || e.end_node == node)
                || !(0..=1).contains(&marked)
                || (marked == 1 && !graph.crosswalk_arm_eligible(node, edge))
            {
                return Err(SaveLoadError::custom("invalid crosswalk arm"));
            }
            graph.set_crosswalk_override(node, edge, marked != 0);
        }
    } else {
        graph.preserve_legacy_zebras();
    }
    if !geometry.is_empty() || !physical_geometry.is_empty() {
        return Err(SaveLoadError::custom("orphan edge geometry"));
    }
    // add_node/add_edge already maintained adjacency and both spatial indices.
    Ok(graph)
}

pub(super) fn rebuild_loaded_graph_runtime(
    graph: &mut RegionGraph,
    transit_network: &mut TransitNetwork,
    terrain: &mut TerrainSystem,
) -> SaveLoadResult<()> {
    // Node positions, grades, physical lengths and junction clips are saved authority. Terrain
    // synchronization is an edit: running it here moves roads away from their restored parcels
    // and changes the boundary geometry consumed by the renderer.
    transit_network.rebuild_all_terrain_earthworks(graph, terrain);
    if transit_network
        .road_surface
        .published_generation_matches_source()
    {
        return Ok(());
    }
    // Legacy saves can contain contradictory junction supports. Repair only rejected
    // grounded junctions using the edit planner's finalizer, before lane/agent restoration.
    // The source SQLite and terrain are not rewritten; valid saved profiles stay authoritative.
    let nodes = transit_network
        .road_surface
        .failed_node_ids()
        .iter()
        .copied()
        .filter(|&node| {
            graph.node_adjacency(node).iter().all(|&edge| {
                graph.edge(edge).deleted
                    || graph.edge(edge).class
                        == crate::simulation::network::types::EdgeClass::Standard
            })
        })
        .collect::<std::collections::HashSet<_>>();
    let edges = nodes
        .iter()
        .flat_map(|&node| graph.node_adjacency(node).iter().copied())
        .filter(|&edge| !graph.edge(edge).deleted)
        .collect::<std::collections::HashSet<_>>();
    if !edges.is_empty() {
        let changed = graph.finalize_junction_endpoint_profiles_for_edges(&nodes, &edges, &edges);
        transit_network.mark_surface_dirty_from_sets(graph, &changed, &nodes);
        transit_network.rebuild_all_terrain_earthworks(graph, terrain);
        crate::debug_log!(
            "save",
            "loaded_junction_profile_repair nodes={} changed_edges={} compiled={}",
            nodes.len(),
            changed.len(),
            transit_network
                .road_surface
                .published_generation_matches_source()
        );
    }
    if !transit_network
        .road_surface
        .published_generation_matches_source()
    {
        return Err(SaveLoadError::custom(format!(
            "road surface load failed: {}",
            transit_network
                .road_surface
                .last_compile_failure_label()
                .unwrap_or("incomplete surface")
        )));
    }
    Ok(())
}

pub(super) fn canonical_existing_node(graph: &RegionGraph, node_id: u32) -> SaveLoadResult<u32> {
    if (node_id as usize) >= graph.node_count() {
        return Err(SaveLoadError::custom(format!(
            "node {} out of bounds",
            node_id
        )));
    }
    let c = graph.get_valid_node(node_id);
    if (c as usize) >= graph.node_count() {
        return Err(SaveLoadError::custom(format!(
            "canonical node {} out of bounds",
            c
        )));
    }
    Ok(c)
}
