// SPDX-License-Identifier: GPL-2.0-only

//! Multi-height embankment shells and box openings owned by a layered bridge span.

use super::super::{RoadSurfaceSection, RoadSurfaceSystem, RoadVec2, RoadVec3};
use crate::config::HEIGHT_SCALE;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::surface::grade_separation::{section_pair_gap, triangle_overlap};
use crate::simulation::network::types::EdgeClass;
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::Vector3;

const SHOULDER_MARGIN_M: f64 = 18.0;
const OPENING_MARGIN_M: f32 = 0.75;
const DECK_DEPTH_M: f64 = 0.6;

/// Compiled multi-height support. Only its solid fill slopes enter the terrain stamper.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct FilledCrossing {
    /// Lower owners used to invalidate the upper span when a crossing is removed or changed.
    pub(crate) lower_edges: Vec<usize>,
    /// Grass-covered sides between pavement and source ground, excluding every opening.
    pub(crate) fill: Vec<[RoadVec3; 4]>,
    /// Soffits, abutments, wing walls and parapets, with winding facing the visible volume.
    pub(crate) concrete: Vec<[RoadVec3; 4]>,
    // Open longitudinal intervals: a single-height terrain stamp must never cover these.
    openings: Vec<(f32, f32)>,
}

#[derive(Clone, Copy, Debug)]
struct Opening {
    start: f32,
    end: f32,
    floor: f64,
}

impl RoadSurfaceSystem {
    /// Discovers structural dependencies through the existing centreline R-tree.
    pub(crate) fn crossing_neighbors(&self, graph: &RegionGraph, edge_idx: usize) -> Vec<usize> {
        self.crossing_neighbors_in_source(graph, graph.edge(edge_idx))
    }

    // A broad-phase halo is not a structural dependency. A rejected span frame may be
    // repaired beside a bridge, provided no span-owned pavement actually overlaps it.
    // Only used on geometry repair: O(log E + K log K + P sum(Q B²)), no tick work.
    pub(in crate::simulation::network::surface) fn has_layered_span_overlap(
        &self,
        graph: &RegionGraph,
        edge_idx: usize,
    ) -> bool {
        let sections = self.compile_edge_sections(graph, edge_idx);
        self.sections_have_layered_span_overlap(graph, edge_idx, &sections)
    }

    pub(in crate::simulation::network::surface) fn sections_have_layered_span_overlap(
        &self,
        graph: &RegionGraph,
        edge_idx: usize,
        sections: &[RoadSurfaceSection],
    ) -> bool {
        let neighbors = self.crossing_neighbors(graph, edge_idx);
        if neighbors.is_empty() {
            return false;
        }
        let Some((start, end)) =
            self.visible_corridor_index_range_for_edge(graph, edge_idx, sections)
        else {
            return false;
        };
        neighbors.into_iter().any(|other| {
            let lower = self.compile_edge_sections(graph, other);
            let Some((a, b)) = self.visible_corridor_index_range_for_edge(graph, other, &lower)
            else {
                return false;
            };
            sections[start..=end].windows(2).any(|pair| {
                lower[a..=b]
                    .windows(2)
                    .any(|other| section_pair_gap(pair, other).is_some())
            })
        })
    }

    /// Queries the same structural halo for an edge in a preview excerpt against live owners.
    pub(crate) fn crossing_neighbors_in_source(
        &self,
        graph: &RegionGraph,
        edge: &crate::simulation::network::graph::Edge,
    ) -> Vec<usize> {
        let points = self.edge_points(edge);
        let mut min = Vector3::new(f32::INFINITY, 0.0, f32::INFINITY);
        let mut max = Vector3::new(f32::NEG_INFINITY, 0.0, f32::NEG_INFINITY);
        for point in points {
            min.x = min.x.min(point.x);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.z = max.z.max(point.z);
        }
        let halo = Self::visual_roadbed_half_width_m(edge) + 24.0 + SHOULDER_MARGIN_M as f32;
        let offset = Vector3::new(halo, 0.0, halo);
        let mut neighbors = graph.get_edges_near_aabb(min - offset, max + offset);
        neighbors.retain(|&id| {
            let other = graph.edge(id);
            Self::is_surface_edge(other) && other.crossing_level != edge.crossing_level
        });
        neighbors.sort_unstable();
        neighbors
    }

    /// O(log E + K log K + P * sum(Q B²) + C log C) edit-time clipping for local
    /// sections P/Q, bounded band count B and C overlaps; no simulation-tick work.
    pub(in crate::simulation::network::surface) fn compile_filled_crossing(
        &self,
        graph: &RegionGraph,
        terrain: &TerrainSystem,
        edge_idx: usize,
        sections: &[RoadSurfaceSection],
    ) -> Option<Box<FilledCrossing>> {
        let edge = graph.edge(edge_idx);
        if edge.class == EdgeClass::Tunnel || sections.len() < 2 {
            return None;
        }
        // Junction pavement and ground have their own owner. Extending the span's
        // grass through that footprint creates competing height carriers at the mouth.
        let (start, end) = self.visible_corridor_index_range_for_edge(graph, edge_idx, sections)?;
        if start >= end {
            return None;
        }
        let sections = &sections[start..=end];
        let mut result = FilledCrossing::default();
        let mut openings = Vec::new();
        for lower_id in self.crossing_neighbors(graph, edge_idx) {
            if graph.edge(lower_id).crossing_level >= edge.crossing_level {
                continue;
            }
            let lower = self.compile_edge_sections(graph, lower_id);
            // A genuinely connected approach joint is not another underpass. At an
            // independent crossing, the lower node-owned pavement still needs the same
            // opening as its span; clipping it away leaves upper fill beside a lower kerb.
            let lower_range = if super::super::grade_separation::edges_share_endpoint(
                graph, edge_idx, graph, lower_id,
            ) {
                self.visible_corridor_index_range_for_edge(graph, lower_id, &lower)
            } else {
                lower.len().checked_sub(1).map(|end| (0, end))
            };
            let Some((lower_start, lower_end)) = lower_range else {
                continue;
            };
            let lower = &lower[lower_start..=lower_end];
            let before = openings.len();
            let mut crosses_deck = false;
            for upper_pair in sections.windows(2) {
                for lower_pair in lower.windows(2) {
                    crosses_deck |= section_pair_gap(upper_pair, lower_pair)
                        .is_some_and(|gap| gap > DECK_DEPTH_M);
                    if let Some(opening) = opening_extent(upper_pair, lower_pair, terrain) {
                        openings.push(opening);
                    }
                }
            }
            if !crosses_deck {
                openings.truncate(before);
            } else if openings.len() != before {
                result.lower_edges.push(lower_id);
            }
        }
        if openings.is_empty() && (edge.crossing_level == 0 || edge.class != EdgeClass::Bridge) {
            return None;
        }
        // Explicit raised crossing approaches stay filled even when a split leaves the
        // actual opening on the next owner. Ordinary level-zero valley bridges keep piers.
        openings.sort_by(|a, b| a.start.total_cmp(&b.start).then(a.end.total_cmp(&b.end)));
        let mut merged: Vec<Opening> = Vec::new();
        for opening in openings {
            if let Some(last) = merged.last_mut()
                && opening.start <= last.end
            {
                last.end = last.end.max(opening.end);
                last.floor = last.floor.min(opening.floor);
            } else {
                merged.push(opening);
            }
        }
        // Both partitions are station-ordered. Sweep them once rather than visiting every
        // opening for every section on a long road with several underpasses.
        let mut opening_index = 0;
        for pair in sections.windows(2) {
            let mut cursor = pair[0].s_m;
            while opening_index < merged.len() && merged[opening_index].end <= cursor {
                opening_index += 1;
            }
            for opening in merged[opening_index..]
                .iter()
                .take_while(|opening| opening.start < pair[1].s_m)
            {
                if opening.start > cursor {
                    result.fill_sides(pair, cursor, opening.start.min(pair[1].s_m), terrain);
                }
                let end = opening.end.min(pair[1].s_m);
                result.deck(pair, cursor.max(opening.start), end);
                cursor = end;
            }
            if cursor < pair[1].s_m {
                result.fill_sides(pair, cursor, pair[1].s_m, terrain);
            }
        }
        let mut section_index = 0;
        for opening in merged {
            result.openings.push((opening.start, opening.end));
            for (station, direction) in [(opening.start, -1.0), (opening.end, 1.0)] {
                while section_index + 2 < sections.len()
                    && sections[section_index + 1].s_m < station
                {
                    section_index += 1;
                }
                let pair = &sections[section_index..section_index + 2];
                result.abutment(pair, station, opening.floor, direction, terrain);
            }
        }
        // Close elevated fill terminals. Grounded ends make these faces degenerate.
        for (pair, station, reverse) in [
            (&sections[..2], sections[0].s_m, false),
            (
                &sections[sections.len() - 2..],
                sections.last().unwrap().s_m,
                true,
            ),
        ] {
            let left = boundary(pair, station, false);
            let right = boundary(pair, station, true);
            let mut cap = [
                left,
                right,
                toe(right, pair[0].lateral_xz, terrain),
                toe(left, -pair[0].lateral_xz, terrain),
            ];
            if reverse {
                cap.reverse();
            }
            result.fill.push(cap);
        }
        Some(Box::new(result))
    }
}

// Project the exact clipped overlap across the entire embankment envelope, rather than only
// its asphalt. This also keeps the grass slope and wing walls outside a skew lower approach.
fn opening_extent(
    upper: &[RoadSurfaceSection],
    lower: &[RoadSurfaceSection],
    terrain: &TerrainSystem,
) -> Option<Opening> {
    let corners = |pair: &[RoadSurfaceSection], margin: f64| {
        let side = |section: &RoadSurfaceSection, right: bool| {
            let lateral = if right {
                f64::from(section.bands.last().unwrap().lateral_end_m) + margin
            } else {
                f64::from(section.bands.first().unwrap().lateral_start_m) - margin
            };
            let point = section.center_xz + section.lateral_xz * lateral;
            RoadVec3::new(point.x, f64::from(section.center_height_m), point.y)
        };
        [
            side(&pair[0], false),
            side(&pair[1], false),
            side(&pair[1], true),
            side(&pair[0], true),
        ]
    };
    if upper
        .iter()
        .chain(lower)
        .any(|section| section.bands.is_empty())
    {
        return None;
    }
    // Clip against the actual grass toes, not the maximum possible setback. A low
    // embankment must not acquire a long opening just because the crossing is skew.
    let a = [
        toe(
            boundary(upper, upper[0].s_m, false),
            -upper[0].lateral_xz,
            terrain,
        ),
        toe(
            boundary(upper, upper[1].s_m, false),
            -upper[1].lateral_xz,
            terrain,
        ),
        toe(
            boundary(upper, upper[1].s_m, true),
            upper[1].lateral_xz,
            terrain,
        ),
        toe(
            boundary(upper, upper[0].s_m, true),
            upper[0].lateral_xz,
            terrain,
        ),
    ];
    let b = corners(lower, OPENING_MARGIN_M.into());
    let bounds = |quad: &[RoadVec3; 4]| {
        quad.iter().fold(
            (
                RoadVec2::splat(f64::INFINITY),
                RoadVec2::splat(f64::NEG_INFINITY),
            ),
            |(min, max), p| {
                let p = RoadVec2::new(p.x, p.z);
                (min.min(p), max.max(p))
            },
        )
    };
    let (amin, amax) = bounds(&a);
    let (bmin, bmax) = bounds(&b);
    if amin.x > bmax.x || bmin.x > amax.x || amin.y > bmax.y || bmin.y > amax.y {
        return None;
    }
    let delta = upper[1].center_xz - upper[0].center_xz;
    let length_squared = delta.length_squared();
    if length_squared < 1e-10 {
        return None;
    }
    let mut min_t = f64::INFINITY;
    let mut max_t = f64::NEG_INFINITY;
    for ai in [[0, 1, 2], [0, 2, 3]] {
        for bi in [[0, 1, 2], [0, 2, 3]] {
            if let Some((points, count)) = triangle_overlap(ai.map(|i| a[i]), bi.map(|i| b[i])) {
                for point in &points[..count] {
                    let t = (*point - upper[0].center_xz).dot(delta) / length_squared;
                    min_t = min_t.min(t);
                    max_t = max_t.max(t);
                }
            }
        }
    }
    min_t.is_finite().then(|| Opening {
        start: upper[0].s_m + (upper[1].s_m - upper[0].s_m) * min_t.clamp(0.0, 1.0) as f32,
        end: upper[0].s_m + (upper[1].s_m - upper[0].s_m) * max_t.clamp(0.0, 1.0) as f32,
        floor: lower
            .iter()
            .flat_map(|s| &s.bands)
            .fold(f64::INFINITY, |height, b| {
                height.min(f64::from(b.height_start_m.min(b.height_end_m)))
            })
            - 0.08,
    })
}

fn boundary(pair: &[RoadSurfaceSection], station: f32, right: bool) -> RoadVec3 {
    let point = |section: &RoadSurfaceSection| {
        let band = if right {
            section.bands.last().unwrap()
        } else {
            &section.bands[0]
        };
        let (lateral, height) = if right {
            (band.lateral_end_m, band.height_end_m)
        } else {
            (band.lateral_start_m, band.height_start_m)
        };
        RoadSurfaceSystem::section_boundary_world_point_static(section, lateral, height)
    };
    let t = f64::from((station - pair[0].s_m) / (pair[1].s_m - pair[0].s_m));
    point(&pair[0]).lerp(point(&pair[1]), t)
}

fn toe(point: RoadVec3, lateral: RoadVec2, terrain: &TerrainSystem) -> RoadVec3 {
    let ground =
        f64::from(terrain.sample_height_world(point.x as f32, point.z as f32) * HEIGHT_SCALE);
    let margin = ((point.y - ground).abs() * 2.0).min(SHOULDER_MARGIN_M);
    let xz = RoadVec2::new(point.x, point.z) + lateral * margin;
    RoadVec3::new(
        xz.x,
        f64::from(terrain.sample_height_world(xz.x as f32, xz.y as f32) * HEIGHT_SCALE) - 0.04,
        xz.y,
    )
}

impl FilledCrossing {
    /// Shared slope triangles for the existing terrain stamper and owner-indexed ground queries.
    pub(crate) fn fill_triangles(&self) -> impl Iterator<Item = [RoadVec3; 3]> + '_ {
        self.fill
            .iter()
            .flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]])
    }

    /// Removes complete sampled terrain-support strips which touch an opening. Pavement
    /// remains continuous; only the single-valued ground carrier is interrupted here.
    pub(in crate::simulation::network::surface) fn support_ranges(
        &self,
        sections: &[RoadSurfaceSection],
        ranges: &[(usize, usize)],
    ) -> Vec<(usize, usize)> {
        let mut result = Vec::new();
        let mut opening_index = 0;
        for &(start, end) in ranges {
            let mut run = None;
            for i in start..end {
                while opening_index < self.openings.len()
                    && self.openings[opening_index].1 <= sections[i].s_m
                {
                    opening_index += 1;
                }
                let open = self
                    .openings
                    .get(opening_index)
                    .is_some_and(|&(a, _)| sections[i + 1].s_m > a);
                if open {
                    if let Some(begin) = run.take() {
                        result.push((begin, i));
                    }
                } else {
                    run.get_or_insert(i);
                }
            }
            if let Some(begin) = run {
                result.push((begin, end));
            }
        }
        result
    }

    fn fill_sides(
        &mut self,
        pair: &[RoadSurfaceSection],
        start: f32,
        end: f32,
        terrain: &TerrainSystem,
    ) {
        for right in [false, true] {
            let a = boundary(pair, start, right);
            let b = boundary(pair, end, right);
            let side = if right { 1.0 } else { -1.0 };
            let mut face = [
                a,
                b,
                toe(b, pair[1].lateral_xz * side, terrain),
                toe(a, pair[0].lateral_xz * side, terrain),
            ];
            if !right {
                face.reverse();
            }
            self.fill.push(face);
        }
    }

    fn deck(&mut self, pair: &[RoadSurfaceSection], start: f32, end: f32) {
        let top = [
            boundary(pair, start, false),
            boundary(pair, end, false),
            boundary(pair, end, true),
            boundary(pair, start, true),
        ];
        let bottom = top.map(|p| p - RoadVec3::Y * DECK_DEPTH_M);
        let mut soffit = bottom;
        soffit.reverse();
        self.concrete.push(soffit);
        for [a, b] in [[0, 1], [2, 3]] {
            self.concrete.push([top[a], bottom[a], bottom[b], top[b]]);
            // Low concrete upstand and open grey metal rails, as in the Finnish references.
            let outward = pair[0].lateral_xz * if a == 0 { -0.25 } else { 0.25 };
            let offset = RoadVec3::new(outward.x, 0.0, outward.y);
            let base = [top[a], top[b], top[b] + offset, top[a] + offset];
            self.box_faces(base, 0.22);
            let rail_offset = offset * 0.35;
            for height in [0.58, 0.88, 1.16] {
                let lift = RoadVec3::Y * height;
                self.box_faces(
                    [
                        top[a] + lift,
                        top[b] + lift,
                        top[b] + lift + rail_offset,
                        top[a] + lift + rail_offset,
                    ],
                    0.06,
                );
            }
            // A global station lattice keeps post density independent of section sampling.
            let first = (f64::from(start) / 2.0).ceil() as i64;
            let last = (f64::from(end) / 2.0).ceil() as i64;
            for post in first..last {
                let p = boundary(pair, (post * 2) as f32, a != 0) + offset * 0.5;
                let along = pair[0].tangent_xz * 0.05;
                let across = pair[0].lateral_xz * 0.05;
                let v = |d: RoadVec2| p + RoadVec3::new(d.x, 0.22, d.y);
                self.box_faces(
                    [
                        v(-along - across),
                        v(along - across),
                        v(along + across),
                        v(-along + across),
                    ],
                    1.0,
                );
            }
        }
    }

    fn abutment(
        &mut self,
        pair: &[RoadSurfaceSection],
        station: f32,
        floor: f64,
        direction: f64,
        terrain: &TerrainSystem,
    ) {
        let left = boundary(pair, station, false);
        let right = boundary(pair, station, true);
        let bottom = |p: RoadVec3| RoadVec3::new(p.x, floor.min(p.y - DECK_DEPTH_M), p.z);
        let mut wall = [left, bottom(left), bottom(right), right];
        if direction > 0.0 {
            wall.reverse();
        }
        self.concrete.push(wall);
        for (p, side) in [(left, -1.0), (right, 1.0)] {
            // Taper to the slope within 6 m; a high fill must not create an 18 m
            // concrete triangle beside a narrow path.
            let raw_end = toe(p, pair[0].lateral_xz * side, terrain);
            let setback = (RoadVec2::new(raw_end.x - p.x, raw_end.z - p.z)).length();
            let end = p.lerp(raw_end, (6.0 / setback.max(6.0)).min(1.0));
            let mut wing = [p, bottom(p), end, end];
            let source_height =
                f64::from(terrain.sample_height_world(p.x as f32, p.z as f32) * HEIGHT_SCALE);
            if source_height - floor > DECK_DEPTH_M {
                // A cut portal has no fill-slope setback. Its retaining wing extends 6 m
                // along the approach and flares 3 m away from the lower corridor.
                let offset =
                    pair[0].lateral_xz * (side * 6.0) + pair[0].tangent_xz * (direction * 3.0);
                let mut outer = p + RoadVec3::new(offset.x, 0.0, offset.y);
                outer.y = f64::from(
                    terrain.sample_height_world(outer.x as f32, outer.z as f32) * HEIGHT_SCALE,
                );
                // Taper the exposed wall into the grass rather than ending in a full-height slab.
                wing = [p, bottom(p), outer, outer];
            }
            if side * direction > 0.0 {
                wing.reverse();
            }
            self.concrete.push(wing);
        }
    }

    fn box_faces(&mut self, mut base: [RoadVec3; 4], height: f64) {
        base.reverse();
        let top = base.map(|p| p + RoadVec3::Y * height);
        self.concrete.push(top);
        for i in 0..4 {
            let j = (i + 1) % 4;
            self.concrete.push([base[i], base[j], top[j], top[i]]);
        }
    }
}
