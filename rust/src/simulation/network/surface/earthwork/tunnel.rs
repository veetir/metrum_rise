// SPDX-License-Identifier: GPL-2.0-only

//! Short concrete tunnel shells and the shared boundary between open cuts and covered bores.

use super::super::{
    RoadSurfaceEarthworkFaceSource, RoadSurfaceSection, RoadSurfaceSystem, RoadVec3,
};
use crate::config::HEIGHT_SCALE;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::types::EdgeClass;
use crate::simulation::terrain::TerrainSystem;

const CLEARANCE_M: f32 = 4.8;
const WALL_M: f32 = 0.45;
const ROOF_M: f32 = 0.6;
const WALL_SETBACK_M: f32 = 0.25;
const WING_LENGTH_M: f64 = 6.0;
const WING_FLARE_M: f64 = 2.0;

/// Derived cold shell; pavement, terrain support and routing retain their existing owners.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct ShortTunnel {
    /// Concrete lining, roof, portal faces and retaining wings, facing their visible volumes.
    pub(crate) concrete: Vec<[RoadVec3; 4]>,
    // Canonical section boundaries where generic earthworks must leave an open mouth.
    portals: Vec<TunnelPortal>,
}

#[derive(Clone, Debug, PartialEq)]
struct TunnelPortal {
    section: usize,
    direction: f64,
    wing_length_m: f64,
}

pub(super) fn roof_height(section: &RoadSurfaceSection) -> f32 {
    section
        .bands
        .iter()
        .fold(section.center_height_m, |top, band| {
            top.max(band.height_start_m).max(band.height_end_m)
        })
        + CLEARANCE_M
}

pub(super) fn roof_is_covered(section: &RoadSurfaceSection, terrain: &TerrainSystem) -> bool {
    let roof = roof_height(section) + ROOF_M;
    let covered = |lateral| {
        let p = RoadSurfaceSystem::section_boundary_world_point_static(section, lateral, roof);
        terrain.sample_height_world(p.x as f32, p.z as f32) * HEIGHT_SCALE >= roof
    };
    covered(0.0)
        && section
            .bands
            .iter()
            .all(|band| covered(band.lateral_start_m) && covered(band.lateral_end_m))
}

impl RoadSurfaceSystem {
    // O(S B) at edit time, with bounded section-band count B. Each covered run owns its
    // roof; its complement owns terrain stamps. No terrain authority enters the bore.
    pub(in crate::simulation::network::surface) fn tunnel_bore_ranges(
        sections: &[RoadSurfaceSection],
        terrain: &TerrainSystem,
    ) -> Vec<(usize, usize)> {
        let mut ranges = Vec::new();
        let mut start = None;
        for (i, section) in sections.iter().enumerate() {
            if roof_is_covered(section, terrain) {
                start.get_or_insert(i);
            } else if let Some(a) = start.take()
                && i > a + 1
            {
                ranges.push((a, i - 1));
            }
        }
        if let Some(a) = start
            && sections.len() > a + 1
        {
            ranges.push((a, sections.len() - 1));
        }
        ranges
    }

    pub(in crate::simulation::network::surface) fn compile_short_tunnel(
        graph: &RegionGraph,
        terrain: &TerrainSystem,
        edge: usize,
        sections: &[RoadSurfaceSection],
    ) -> Option<Box<ShortTunnel>> {
        if graph.edge(edge).class != EdgeClass::Tunnel {
            return None;
        }
        let mut shell = ShortTunnel::default();
        for (a, b) in Self::tunnel_bore_ranges(sections, terrain) {
            for pair in sections[a..=b].windows(2) {
                let bounds_a = graph.paved_bounds(edge, pair[0].s_m);
                let bounds_b = graph.paved_bounds(edge, pair[1].s_m);
                let bottom = |s: &RoadSurfaceSection, bounds: [f32; 2], side: usize| {
                    let sign = if side == 0 { -1.0 } else { 1.0 };
                    Self::section_boundary_world_point_static(
                        s,
                        bounds[side] + sign * WALL_SETBACK_M,
                        s.center_height_m - 0.1,
                    )
                };
                let left = [bottom(&pair[0], bounds_a, 0), bottom(&pair[1], bounds_b, 0)];
                let right = [bottom(&pair[0], bounds_a, 1), bottom(&pair[1], bounds_b, 1)];
                let top = |mut p: RoadVec3, s: &RoadSurfaceSection| {
                    p.y = f64::from(roof_height(s));
                    p
                };
                let lt = [top(left[0], &pair[0]), top(left[1], &pair[1])];
                let rt = [top(right[0], &pair[0]), top(right[1], &pair[1])];
                let lateral = RoadVec3::new(pair[0].lateral_xz.x, 0.0, pair[0].lateral_xz.y);
                shell.face([left[0], left[1], lt[1], lt[0]], lateral);
                shell.face([right[0], right[1], rt[1], rt[0]], -lateral);
                // The existing pavement stops at its queried edge. Concrete floor
                // strips close the small clearance to the lining, without widening lanes.
                for (side, wall) in [left, right].into_iter().enumerate() {
                    let paved = std::array::from_fn::<_, 2, _>(|i| {
                        Self::section_boundary_world_point_static(
                            &pair[i],
                            [bounds_a, bounds_b][i][side],
                            paved_edge_height(&pair[i], side),
                        )
                    });
                    shell.face(
                        [
                            paved[0],
                            paved[1],
                            RoadVec3::new(wall[1].x, paved[1].y, wall[1].z),
                            RoadVec3::new(wall[0].x, paved[0].y, wall[0].z),
                        ],
                        RoadVec3::Y,
                    );
                }
                shell.face([lt[0], lt[1], rt[1], rt[0]], -RoadVec3::Y);
                shell.face(
                    [lt[0], lt[1], rt[1], rt[0]].map(|p| p + RoadVec3::Y * f64::from(ROOF_M)),
                    RoadVec3::Y,
                );
            }
            for (i, direction) in [(a, -1.0), (b, 1.0)] {
                // Fully buried endpoints are continuations, not manufactured surface portals.
                if i == 0 || i + 1 == sections.len() {
                    continue;
                }
                let neighbor = if direction < 0.0 { i - 1 } else { i + 1 };
                // Keep wings inside the adjacent open section interval, including dense
                // authored curves and cuts shared by two consecutive covered runs.
                let wing_length_m = WING_LENGTH_M
                    .min(f64::from((sections[neighbor].s_m - sections[i].s_m).abs()) * 0.45);
                let portal = TunnelPortal {
                    section: i,
                    direction,
                    wing_length_m,
                };
                shell.portal(graph, terrain, edge, sections, &portal);
                shell.portals.push(portal);
            }
        }
        shell.portals.sort_unstable_by_key(|portal| portal.section);
        (!shell.concrete.is_empty()).then(|| Box::new(shell))
    }
}

impl ShortTunnel {
    // The terrain cut follows the outer concrete wings and ends on the roof.
    // Pavement and the clear opening never supply the height of that roof rail.
    // O(E log P), local to this stroke's E boundary edges and P portals.
    pub(in crate::simulation::network::surface) fn lift_portal_clip_boundaries(
        &self,
        boundary: &mut super::super::RoadSurfaceTerrainClipLoop,
        sections: &[RoadSurfaceSection],
        graph: &RegionGraph,
        terrain: &TerrainSystem,
        edge_idx: usize,
    ) {
        use super::super::keys::SurfaceXzKey;
        use std::collections::BTreeMap;
        let mut roof_points = BTreeMap::new();
        for edge in &boundary.source_edges {
            let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
                start_section_index: a,
                end_section_index: b,
                ..
            } = edge.source
            else {
                continue;
            };
            for i in [a, b] {
                let Ok(index) = self
                    .portals
                    .binary_search_by_key(&i, |portal| portal.section)
                else {
                    continue;
                };
                let portal = &self.portals[index];
                let section = &sections[i];
                let bounds = graph.paved_bounds(edge_idx, section.s_m);
                for point in [edge.start, edge.end] {
                    let delta =
                        point - RoadVec3::new(section.center_xz.x, 0.0, section.center_xz.y);
                    let station = delta.x * section.tangent_xz.x + delta.z * section.tangent_xz.y;
                    if station.abs() >= 0.002 {
                        continue;
                    }
                    let lateral = delta.x * section.lateral_xz.x + delta.z * section.lateral_xz.y;
                    let mut lifted = point;
                    lifted.y = portal_top_height(graph, terrain, edge_idx, section);
                    let mut toe = None;
                    for (side, bound) in bounds.into_iter().enumerate() {
                        if (lateral - f64::from(bound)).abs() >= 0.002 {
                            continue;
                        }
                        let sign = if side == 0 { -1.0 } else { 1.0 };
                        let outside = bound + sign * (WALL_SETBACK_M + WALL_M);
                        lifted = RoadSurfaceSystem::section_boundary_world_point_static(
                            section,
                            outside,
                            lifted.y as f32,
                        );
                        let mut end = lifted
                            + RoadVec3::new(section.tangent_xz.x, 0.0, section.tangent_xz.y)
                                * portal.direction
                                * portal.wing_length_m
                            + RoadVec3::new(section.lateral_xz.x, 0.0, section.lateral_xz.y)
                                * f64::from(sign)
                                * (WING_FLARE_M + f64::from(WALL_M));
                        end.y = ground_height(terrain, end)
                            .max(f64::from(section.center_height_m - 0.1))
                            + 0.15;
                        toe = Some(end);
                    }
                    roof_points.insert(SurfaceXzKey::from_world_xz(point), (lifted, toe));
                }
            }
        }
        let split = |start: RoadVec3, end: RoadVec3| {
            let start_roof = roof_points.get(&SurfaceXzKey::from_world_xz(start));
            let end_roof = roof_points.get(&SurfaceXzKey::from_world_xz(end));
            let mut points = Vec::with_capacity(4);
            points.push(start_roof.map_or(start, |entry| entry.0));
            for roof in [start_roof, end_roof].into_iter().flatten() {
                if let Some(toe) = roof.1 {
                    let direction = end - start;
                    let length2 = direction.x * direction.x + direction.z * direction.z;
                    let t = ((toe.x - start.x) * direction.x + (toe.z - start.z) * direction.z)
                        / length2;
                    if t > 0.001 && t < 0.999 {
                        points.push(toe)
                    }
                }
            }
            points.push(end_roof.map_or(end, |entry| entry.0));
            (points, start_roof.is_some() || end_roof.is_some())
        };
        let mut points = Vec::new();
        for i in 0..boundary.points_world.len() {
            let (mut segment, _) = split(
                boundary.points_world[i],
                boundary.points_world[(i + 1) % boundary.points_world.len()],
            );
            segment.pop();
            points.extend(segment);
        }
        boundary.points_world = points;
        let mut edges = Vec::new();
        for edge in &boundary.source_edges {
            let (points, roof) = split(edge.start, edge.end);
            let mut source = edge.source;
            if let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary { support_policy, .. } =
                &mut source
            {
                *support_policy = if roof {
                    super::super::RoadSurfaceEarthworkSupportPolicy::TunnelPortalRoof
                } else {
                    super::super::RoadSurfaceEarthworkSupportPolicy::TunnelOpenCut
                };
            }
            edges.extend(points.windows(2).map(|pair| {
                super::super::RoadSurfaceTerrainClipSourceEdge {
                    start: pair[0],
                    end: pair[1],
                    source,
                    kind: edge.kind,
                }
            }));
        }
        boundary.source_edges = edges;
    }

    fn face(&mut self, mut points: [RoadVec3; 4], normal: RoadVec3) {
        if (points[2] - points[0])
            .cross(points[1] - points[0])
            .dot(normal)
            < 0.0
        {
            points.reverse();
        }
        self.concrete.push(points);
    }

    fn portal(
        &mut self,
        graph: &RegionGraph,
        terrain: &TerrainSystem,
        edge: usize,
        sections: &[RoadSurfaceSection],
        portal: &TunnelPortal,
    ) {
        let s = &sections[portal.section];
        let neighbor = &sections[if portal.direction < 0.0 {
            portal.section - 1
        } else {
            portal.section + 1
        }];
        let bounds = graph.paved_bounds(edge, s.s_m);
        let tangent = RoadVec3::new(s.tangent_xz.x, 0.0, s.tangent_xz.y) * portal.direction;
        let lateral = RoadVec3::new(s.lateral_xz.x, 0.0, s.lateral_xz.y);
        let point = |offset, height| {
            RoadSurfaceSystem::section_boundary_world_point_static(s, offset, height)
        };
        let roof = roof_height(s);
        let height = portal_top_height(graph, terrain, edge, s) as f32;
        self.face(
            [
                point(bounds[0] - WALL_SETBACK_M - WALL_M, roof),
                point(bounds[1] + WALL_SETBACK_M + WALL_M, roof),
                point(bounds[1] + WALL_SETBACK_M + WALL_M, height),
                point(bounds[0] - WALL_SETBACK_M - WALL_M, height),
            ],
            tangent,
        );
        for (side, &bound) in bounds.iter().enumerate() {
            let sign = if side == 0 { -1.0 } else { 1.0 };
            let inside = bound + sign * WALL_SETBACK_M;
            let outside = inside + sign * WALL_M;
            self.face(
                [
                    point(inside, s.center_height_m - 0.1),
                    point(outside, s.center_height_m - 0.1),
                    point(outside, height),
                    point(inside, roof),
                ],
                tangent,
            );
            // Finnish concrete retaining wings flare into the approach slope, ending at DEM.
            let base = point(outside, s.center_height_m - 0.1);
            let toe =
                base + tangent * portal.wing_length_m + lateral * f64::from(sign) * WING_FLARE_M;
            let end_top = toe.y.max(ground_height(terrain, toe) + 0.15);
            let face = [
                base,
                toe,
                RoadVec3::new(toe.x, end_top, toe.z),
                RoadVec3::new(base.x, f64::from(height), base.z),
            ];
            // Close the widened approach footprint to its concrete wing. The next
            // canonical section is also the terrain cut's return to the paved edge.
            let paved = point(bound, paved_edge_height(s, side));
            let neighbor_paved = RoadSurfaceSystem::section_boundary_world_point_static(
                neighbor,
                graph.paved_bounds(edge, neighbor.s_m)[side],
                paved_edge_height(neighbor, side),
            );
            let floor_height = paved.y
                + (neighbor_paved.y - paved.y) * portal.wing_length_m
                    / f64::from((neighbor.s_m - s.s_m).abs());
            let mut outer_toe = toe + lateral * f64::from(sign * WALL_M);
            outer_toe.y = floor_height;
            self.face(
                [
                    paved,
                    neighbor_paved,
                    outer_toe,
                    RoadVec3::new(base.x, paved.y, base.z),
                ],
                RoadVec3::Y,
            );
            self.face(
                [
                    face[1],
                    face[1] + lateral * f64::from(sign * WALL_M),
                    face[2] + lateral * f64::from(sign * WALL_M),
                    face[2],
                ],
                tangent,
            );
            self.face(face, -lateral * f64::from(sign));
            self.face(
                face.map(|p| p + lateral * f64::from(sign * WALL_M)),
                lateral * f64::from(sign),
            );
            self.face(
                [
                    face[3],
                    face[2],
                    face[2] + lateral * f64::from(sign * WALL_M),
                    face[3] + lateral * f64::from(sign * WALL_M),
                ],
                RoadVec3::Y,
            );
        }
    }
}

fn ground_height(terrain: &TerrainSystem, point: RoadVec3) -> f64 {
    f64::from(terrain.sample_height_world(point.x as f32, point.z as f32) * HEIGHT_SCALE)
}

fn portal_top_height(
    graph: &RegionGraph,
    terrain: &TerrainSystem,
    edge: usize,
    section: &RoadSurfaceSection,
) -> f64 {
    let bounds = graph.paved_bounds(edge, section.s_m);
    [
        bounds[0] - WALL_SETBACK_M - WALL_M,
        bounds[1] + WALL_SETBACK_M + WALL_M,
    ]
    .into_iter()
    .fold(
        f64::from(roof_height(section) + ROOF_M),
        |height, offset| {
            let point =
                RoadSurfaceSystem::section_boundary_world_point_static(section, offset, 0.0);
            height.max(ground_height(terrain, point) + 0.15)
        },
    )
}

fn paved_edge_height(section: &RoadSurfaceSection, side: usize) -> f32 {
    if side == 0 {
        section
            .bands
            .first()
            .map_or(section.center_height_m, |band| band.height_start_m)
    } else {
        section
            .bands
            .last()
            .map_or(section.center_height_m, |band| band.height_end_m)
    }
}
