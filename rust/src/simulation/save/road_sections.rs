// SPDX-License-Identifier: GPL-2.0-only

//! Versioned cold section tables. Floats are serialized as exact IEEE bits, never SQL REALs.

use super::{SaveLoadError, SaveLoadResult, SnapshotMaps, i64_to_usize, usize_to_i64};
use crate::simulation::network::graph::{
    RegionGraph, RoadLayout, RoadSectionDetail, RoadSectionRef, RoadStrip,
};
use rusqlite::{Connection, Transaction, params};
use std::collections::BTreeMap;

pub(super) fn save(
    tx: &Transaction,
    graph: &RegionGraph,
    maps: &SnapshotMaps,
) -> SaveLoadResult<()> {
    graph
        .validate_road_sections()
        .map_err(SaveLoadError::custom)?;
    let mut layouts = BTreeMap::new();
    let mut details = BTreeMap::new();
    for &edge in maps.edge_old_to_new.keys() {
        let reference = graph.road_section_ref(edge);
        layouts.insert(reference.layout, edge);
        if reference.detail != u32::MAX {
            details.insert(reference.detail, edge);
        }
    }
    let mut layout_ids = BTreeMap::new();
    let mut insert = tx.prepare("INSERT INTO road_layouts VALUES (?1,1,?2)")?;
    let mut strip_insert = tx.prepare("INSERT INTO road_layout_strips VALUES (?1,?2,?3,?4)")?;
    for (saved, (&id, &edge)) in layouts.iter().enumerate() {
        layout_ids.insert(id, saved);
        let mut header = graph.road_layout(edge).clone();
        let strips = header.strips;
        header.strips = [[RoadStrip::default(); 8]; 2];
        insert.execute(params![
            usize_to_i64(saved)?,
            serde_json::to_vec(&header).map_err(|e| SaveLoadError::custom(e.to_string()))?
        ])?;
        for (side, strips) in strips.iter().enumerate() {
            for (ordinal, strip) in strips
                .iter()
                .take(usize::from(header.strip_counts[side]))
                .enumerate()
            {
                strip_insert.execute(params![
                    usize_to_i64(saved)?,
                    side as i64,
                    ordinal as i64,
                    serde_json::to_vec(strip).map_err(|e| SaveLoadError::custom(e.to_string()))?
                ])?;
            }
        }
    }
    let mut detail_ids = BTreeMap::new();
    let mut insert = tx.prepare("INSERT INTO road_section_details VALUES (?1,1,?2)")?;
    for (saved, (&id, &edge)) in details.iter().enumerate() {
        detail_ids.insert(id, saved);
        insert.execute(params![
            usize_to_i64(saved)?,
            serde_json::to_vec(graph.road_section_detail(edge).unwrap())
                .map_err(|e| SaveLoadError::custom(e.to_string()))?
        ])?;
    }
    let mut insert = tx.prepare("INSERT INTO road_edge_sections VALUES (?1,?2,?3,?4,?5,?6,?7)")?;
    // Source edge order and snapshot compaction order agree. HashMap iteration never publishes IDs.
    for edge in 0..graph.edge_count() {
        let Some(&saved) = maps.edge_old_to_new.get(&edge) else {
            continue;
        };
        let reference = graph.road_section_ref(edge);
        insert.execute(params![
            usize_to_i64(saved)?,
            usize_to_i64(layout_ids[&reference.layout])?,
            if reference.detail == u32::MAX {
                -1
            } else {
                usize_to_i64(detail_ids[&reference.detail])?
            },
            reference.revision,
            reference.class,
            reference.access,
            reference.overrides
        ])?;
    }
    Ok(())
}

pub(super) fn load(conn: &Connection, graph: &mut RegionGraph) -> SaveLoadResult<()> {
    let current = is_current(conn)?;
    if !current {
        // Historical decoder has finished all median, taper, mask, crossing and frontage work.
        graph.resolve_legacy_road_details(0..graph.edge_count());
        return graph
            .validate_road_sections()
            .map_err(SaveLoadError::custom);
    }
    let mut layouts: Vec<RoadLayout> = Vec::new();
    let mut query = conn.prepare(
        "SELECT layout_id,format_version,layout_blob FROM road_layouts ORDER BY layout_id",
    )?;
    let mut rows = query.query([])?;
    while let Some(row) = rows.next()? {
        if i64_to_usize(row.get(0)?)? != layouts.len() || row.get::<_, i64>(1)? != 1 {
            return Err(SaveLoadError::custom(
                "invalid road layout catalog version or ID",
            ));
        }
        let layout: RoadLayout = serde_json::from_slice(&row.get::<_, Vec<u8>>(2)?)
            .map_err(|e| SaveLoadError::custom(e.to_string()))?;
        if layout.strip_counts.iter().any(|&n| n > 8)
            || layout.strips != [[RoadStrip::default(); 8]; 2]
        {
            return Err(SaveLoadError::custom("invalid road layout header"));
        }
        layouts.push(layout);
    }
    let mut counts = vec![[0usize; 2]; layouts.len()];
    let mut query = conn.prepare("SELECT layout_id,side,ordinal,strip_blob FROM road_layout_strips ORDER BY layout_id,side,ordinal")?;
    let mut rows = query.query([])?;
    while let Some(row) = rows.next()? {
        let id = i64_to_usize(row.get(0)?)?;
        let side = i64_to_usize(row.get(1)?)?;
        let ordinal = i64_to_usize(row.get(2)?)?;
        if id >= layouts.len() || side > 1 || ordinal >= 8 || counts[id][side] != ordinal {
            return Err(SaveLoadError::custom(
                "invalid road strip owner or correspondence",
            ));
        }
        layouts[id].strips[side][ordinal] = serde_json::from_slice(&row.get::<_, Vec<u8>>(3)?)
            .map_err(|e| SaveLoadError::custom(e.to_string()))?;
        counts[id][side] += 1;
    }
    for (layout, counts) in layouts.iter().zip(counts) {
        if counts != layout.strip_counts.map(usize::from) {
            return Err(SaveLoadError::custom("missing road layout strips"));
        }
        layout.validate(true).map_err(SaveLoadError::custom)?;
    }
    let mut details: Vec<RoadSectionDetail> = Vec::new();
    let mut query = conn.prepare(
        "SELECT detail_id,format_version,detail_blob FROM road_section_details ORDER BY detail_id",
    )?;
    let mut rows = query.query([])?;
    while let Some(row) = rows.next()? {
        if i64_to_usize(row.get(0)?)? != details.len() || row.get::<_, i64>(1)? != 1 {
            return Err(SaveLoadError::custom(
                "invalid road detail catalog version or ID",
            ));
        }
        let detail: RoadSectionDetail = serde_json::from_slice(&row.get::<_, Vec<u8>>(2)?)
            .map_err(|e| SaveLoadError::custom(e.to_string()))?;
        detail.validate().map_err(SaveLoadError::custom)?;
        details.push(detail);
    }
    graph
        .load_road_section_catalog(layouts.clone(), details.clone())
        .map_err(SaveLoadError::custom)?;
    let mut query = conn.prepare("SELECT edge_id,layout_id,detail_id,revision,class,access,overrides FROM road_edge_sections ORDER BY edge_id")?;
    let mut rows = query.query([])?;
    let mut count = 0;
    while let Some(row) = rows.next()? {
        let id = i64_to_usize(row.get(0)?)?;
        if id != count || id >= graph.edge_count() {
            return Err(SaveLoadError::custom("invalid road section owner"));
        }
        let layout = layouts
            .get(i64_to_usize(row.get(1)?)?)
            .ok_or_else(|| SaveLoadError::custom("invalid layout reference"))?;
        let detail_id: i64 = row.get(2)?;
        let detail = if detail_id == -1 {
            None
        } else {
            Some(
                details
                    .get(i64_to_usize(detail_id)?)
                    .ok_or_else(|| SaveLoadError::custom("invalid detail reference"))?
                    .clone(),
            )
        };
        let reference = RoadSectionRef {
            layout: 0,
            detail: u32::MAX,
            revision: row.get(3)?,
            class: row.get(4)?,
            access: row.get(5)?,
            overrides: row.get(6)?,
        };
        graph
            .install_road_section(id, layout.clone(), detail, reference)
            .map_err(SaveLoadError::custom)?;
        count += 1;
    }
    if count != graph.edge_count() {
        return Err(SaveLoadError::custom("missing road section references"));
    }
    graph
        .validate_road_sections()
        .map_err(SaveLoadError::custom)
}

// A full snapshot's version is authoritative. Isolated network tests intentionally omit
// metadata rows; their section-table presence identifies the current network schema.
pub(super) fn is_current(conn: &Connection) -> SaveLoadResult<bool> {
    let has_header = super::sqlite_table_exists(conn, "save_meta")?
        && conn.query_row("SELECT count(*) != 0 FROM save_meta", [], |row| {
            row.get::<_, bool>(0)
        })?;
    if has_header {
        Ok(super::format::SaveFormat::read(conn)?.since(110))
    } else {
        super::sqlite_table_exists(conn, "road_layouts")
    }
}
