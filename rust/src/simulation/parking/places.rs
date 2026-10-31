// SPDX-License-Identifier: GPL-2.0-only

//! Six-metre kerb bays from authoritative span sections; no road profile mutations.

use super::geometry::{covered, rectangle, structure_collision};
use super::{Bay, ParkingRule, SideRule};
use crate::assets::AnchorType;
use crate::simulation::buildings::allocator::{BuildingAllocator, building_local_xz_basis};
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::roadbed::{ArmKey, BandKind, EdgeEnd, RoadSection, RoadbedSystem};
use crate::simulation::network::types::{EdgeClass, NodeType, TransitType};
use godot::prelude::Vector2;

/// Local geometry inputs reused for both side choice and exclusions.
pub(crate) struct KerbContext {
    pub mouths: [usize; 2],
    pub driveways: [Vec<(f32, f32)>; 2],
}

/// O(local site candidates + incident degree); uses the existing site index, never all buildings.
pub(crate) fn kerb_context(
    graph: &RegionGraph,
    allocator: &BuildingAllocator,
    edge_idx: usize,
) -> KerbContext {
    let edge = graph.edge(edge_idx);
    let mut result = KerbContext {
        mouths: [0; 2],
        driveways: [Vec::new(), Vec::new()],
    };
    for (node, at_start) in [(edge.start_node, true), (edge.end_node, false)] {
        let node = graph.get_valid_node(node);
        let points = &edge.physical_geometry;
        if points.len() < 2 {
            continue;
        }
        let along = if at_start {
            points[1] - points[0]
        } else {
            points[points.len() - 1] - points[points.len() - 2]
        };
        for &other in graph.node_adjacency(node) {
            if other == edge_idx || graph.edge(other).deleted {
                continue;
            }
            let other = graph.edge(other);
            // Count the actual mouth direction. A curved arm's far endpoint can lie
            // on the opposite side of this street from its junction approach.
            let points = &other.physical_geometry;
            if points.len() < 2 {
                continue;
            }
            let toward = if graph.get_valid_node(other.start_node) == node {
                points[1] - points[0]
            } else {
                points[points.len() - 2] - points[points.len() - 1]
            };
            let cross = along.x * toward.z - along.z * toward.x;
            if cross != 0.0 {
                result.mouths[usize::from(cross < 0.0)] += 1;
            }
        }
    }
    let mut min = Vector2::new(f32::INFINITY, f32::INFINITY);
    let mut max = -min;
    for p in &edge.physical_geometry {
        min = Vector2::new(min.x.min(p.x), min.y.min(p.z));
        max = Vector2::new(max.x.max(p.x), max.y.max(p.z));
    }
    let margin = edge.width + 4.0;
    let mut buildings = Vec::new();
    allocator.site_cells.visit(
        [
            min.x - margin,
            min.y - margin,
            max.x + margin,
            max.y + margin,
        ],
        |i| buildings.push(i),
    );
    buildings.sort_unstable();
    buildings.dedup();
    for i in buildings {
        let building = &allocator.buildings[i];
        if building.edge_idx != edge_idx {
            continue;
        }
        let Some(asset) = allocator.registry.get(&building.asset_id) else {
            continue;
        };
        let (bx, bz) = building_local_xz_basis(
            building.facing_dir,
            asset.manifest.building_frontage_forward(),
        );
        let center = Vector2::new(building.center_x, building.center_y);
        for anchor in &asset.manifest.anchors {
            if anchor.anchor_type != AnchorType::Driveway {
                continue;
            }
            let pos = center + bx * anchor.position[0] + bz * anchor.position[2];
            let station =
                BuildingAllocator::project_point_to_polyline_s(&edge.physical_geometry, pos);
            let half = anchor.width_m.unwrap_or(3.0) * 0.5 + 1.0;
            let side = usize::from(building.side > 0);
            result.mouths[side] += 1;
            result.driveways[side].push((station - half, station + half));
        }
    }
    result
}

fn carriageway(section: &RoadSection) -> Option<(f32, f32)> {
    let mut bands = section
        .bands
        .iter()
        .filter(|b| b.kind == BandKind::Carriageway);
    let first = bands.next()?;
    let mut bounds = (first.lateral_start_m, first.lateral_end_m);
    for band in bands {
        bounds.0 = bounds.0.min(band.lateral_start_m);
        bounds.1 = bounds.1.max(band.lateral_end_m);
    }
    Some(bounds)
}

fn sample(sections: &[RoadSection], station: f32) -> Option<(Vector2, Vector2, f32, (f32, f32))> {
    let hi = sections
        .partition_point(|s| s.s_m < station)
        .clamp(1, sections.len() - 1);
    let (a, b) = (&sections[hi - 1], &sections[hi]);
    let t = ((f64::from(station) - f64::from(a.s_m)) / (f64::from(b.s_m) - f64::from(a.s_m)))
        .clamp(0.0, 1.0);
    let pos = a.center_xz.lerp(b.center_xz, t);
    let lateral = a.lateral_xz.lerp(b.lateral_xz, t).normalize();
    let (aw, bw) = (carriageway(a)?, carriageway(b)?);
    Some((
        Vector2::new(pos.x as f32, pos.y as f32),
        Vector2::new(lateral.x as f32, lateral.y as f32),
        a.center_height_m + (b.center_height_m - a.center_height_m) * t as f32,
        (
            aw.0 + (bw.0 - aw.0) * t as f32,
            aw.1 + (bw.1 - aw.1) * t as f32,
        ),
    ))
}

/// O(sections + bays × (log sections + local mesh/structure checks)); edit-boundary only.
/// Full bay polygons must be contained in this span's carriageway, so no junction/sidewalk
/// footprint can become supply. Crossings read the roadbed's authoritative strip/arm data.
pub(crate) fn kerb_bays(
    graph: &RegionGraph,
    roadbed: &RoadbedSystem,
    allocator: &BuildingAllocator,
    edge_idx: usize,
    side: usize,
    rule: ParkingRule,
    context: &KerbContext,
) -> Vec<Bay> {
    let edge = graph.edge(edge_idx);
    if edge.deleted
        || edge.class != EdgeClass::Standard
        || edge.primary_type != TransitType::Road
        || rule.sides[side] == SideRule::None
        || [edge.start_node, edge.end_node]
            .into_iter()
            .any(|n| graph.node(graph.get_valid_node(n)).node_type == NodeType::Border)
    {
        return Vec::new();
    }
    let Some(span) = roadbed.span_piece(edge_idx) else {
        return Vec::new();
    };
    let sections = span.sections();
    if sections.len() < 2 {
        return Vec::new();
    }
    let both = rule.sides.iter().all(|s| *s != SideRule::None);
    let min_width = if both { 7.5 } else { 5.5 };
    let sign = if side == 0 { 1.0 } else { -1.0 };
    let mut exclusions = context.driveways[side].clone();
    for (end, node) in [
        (EdgeEnd::Start, edge.start_node),
        (EdgeEnd::End, edge.end_node),
    ] {
        if let Some((lo, hi)) = span.crossing_strip_m(end) {
            exclusions.push((lo - 5.0, hi + 5.0));
        }
        if roadbed
            .node_piece(graph.get_valid_node(node))
            .is_some_and(|n| {
                n.crosswalk_kerbs(ArmKey {
                    edge: edge_idx,
                    end,
                })
                .is_some()
            })
        {
            let mouth = if end == EdgeEnd::Start {
                sections[0].s_m
            } else {
                sections.last().unwrap().s_m
            };
            exclusions.push((mouth - 5.0, mouth + 5.0));
        }
    }
    // Carriageway triangles are already indexed by band; retain only the local span, not nodes.
    let mesh = span.mesh();
    let surfaces: Vec<_> = mesh
        .band_triangles(BandKind::Carriageway)
        .iter()
        .map(|triangle| {
            triangle
                .iter()
                .map(|&i| {
                    let p = mesh.positions()[i as usize];
                    [p.x, p.z]
                })
                .collect()
        })
        .collect();
    let start = sections[0].s_m;
    let finish = sections.last().unwrap().s_m;
    let mut station = ((start + rule.origin_m) / 6.0).ceil() * 6.0 - rule.origin_m + 3.0;
    let mut result = Vec::new();
    while station + 3.0 <= finish {
        let lo = station - 3.0;
        let hi = station + 3.0;
        if !exclusions.iter().any(|&(a, b)| lo < b && hi > a) {
            let Some((center, lateral, _, bounds)) = sample(sections, station) else {
                break;
            };
            let varying = sections
                .iter()
                .filter(|s| s.s_m >= lo && s.s_m <= hi)
                .any(|s| carriageway(s) != Some(bounds));
            let end_widths = [lo, hi].map(|s| sample(sections, s).map(|v| v.3));
            if bounds.1 - bounds.0 >= min_width
                && !varying
                && end_widths.iter().all(|b| *b == Some(bounds))
            {
                let outer = if sign < 0.0 { bounds.0 } else { bounds.1 };
                let position = center + lateral * (outer - sign * (0.25 + 0.9));
                let forward = Vector2::new(-lateral.y, lateral.x) * -sign;
                let footprint = rectangle(position, forward, 1.8, 6.0);
                if covered(&footprint, &surfaces)
                    && !structure_collision(allocator, &footprint)
                    && let Some(height) = roadbed.carriageway_height_at(position.x, position.y)
                {
                    result.push(Bay {
                        identity: [
                            rule.lineage,
                            ((station + rule.origin_m - 3.0) / 6.0).round() as u64,
                        ],
                        position: [position.x, height + 0.02, position.y],
                        heading: (-forward.x).atan2(-forward.y),
                        width: 1.8,
                        length: 6.0,
                    });
                }
            }
        }
        station += 6.0;
    }
    result
}
