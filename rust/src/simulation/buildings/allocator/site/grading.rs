// SPDX-License-Identifier: GPL-3.0-or-later

//! Building-site apron grading and road/terrain tie-in validation.

use super::geometry::{SITE_POINT_EPS_M, point_in_polygon_slice, signed_polygon_area};
use crate::simulation::buildings::allocator::BuildingAllocator;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::roadbed::{RoadbedSystem, side_band_width_m};
use crate::simulation::network::types::EdgeClass;
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::{Vector2, Vector3};

const BUILDING_SITE_GRADING_RING_MULTIPLIERS: [f32; 5] = [0.5, 1.0, 2.0, 4.0, 8.0];
/// Boundary station spacing for grading rays and frontage bank constraints.
pub(crate) const BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M: f32 = 2.0;
const BUILDING_SITE_SUPPORT_TIE_IN_EPS_M: f32 = 0.05;
pub(super) const BUILDING_SITE_NEAREST_ROAD_SURFACE_MIN_RADIUS_M: f32 = 3.0;
/// Existing local road-surface probe reach used by adjacent-frontage bank checks.
pub(crate) const BUILDING_SITE_NEAREST_ROAD_SURFACE_MAX_RADIUS_M: f32 = 8.0;
const BUILDING_SITE_ROAD_SURFACE_PROBE_INSET_M: f32 = 0.05;
// How far a pad's tie-in rays reach (`docs/terrain.md` T12 (a)): the graded terrain's cap, the
// farthest a pad's cone grades the ground.
const BUILDING_SITE_SUPPORT_TIE_IN_REACH_M: f32 = crate::simulation::terrain::graded::CAP_M as f32;

/// A measured placement failure, retained by cached verdicts and the build preview.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct StreetContactIssue {
    /// Contact or grading constraint that bound the feasible height interval.
    pub(crate) constraint: &'static str,
    /// Measured value in hundredths of `unit`, rounded for presentation only.
    pub(crate) measured: u32,
    /// Allowed value in hundredths of `unit`.
    pub(crate) limit: u32,
    /// Display unit (metres or percent).
    pub(crate) unit: &'static str,
}

impl StreetContactIssue {
    /// Player-facing measured constraint, shared by preview and commit errors.
    pub(crate) fn message(self) -> String {
        format!(
            "{} requires {:.2} {}; at most {:.2} {}",
            self.constraint,
            self.measured as f32 / 100.0,
            self.unit,
            self.limit as f32 / 100.0,
            self.unit
        )
    }
}

/// Roads seen by site grading: the graph for nearest-edge probes, the roadbed for heights.
#[derive(Clone, Copy)]
pub(crate) struct SiteRoads<'a> {
    graph: &'a RegionGraph,
    roadbed: &'a RoadbedSystem,
}

impl<'a> SiteRoads<'a> {
    /// Binds the live graph and the roadbed built from it.
    pub(crate) fn new(graph: &'a RegionGraph, roadbed: &'a RoadbedSystem) -> Self {
        Self { graph, roadbed }
    }

    // The visible road height at a point: a tunnel's buried deck lies in the hill under it
    // (`ROAD-61` round R8h), so a pad over a bore reads the ground as if no tunnel ran there.
    fn sample_visible_height(self, x: f32, z: f32) -> Option<f32> {
        self.roadbed.exposed_height_at(x, z)
    }

    // Visits R-tree candidates in ascending edge order for deterministic nearest-edge ties.
    // Cost is O(log(resident edges) + local candidates log(local candidates)).
    fn visit_edges_near_point(
        self,
        pos: Vector3,
        radius_m: f32,
        mut visit: impl FnMut(&RegionGraph, usize),
    ) {
        let mut edges = self.graph.get_edges_near_point(pos, radius_m);
        edges.sort_unstable();
        edges.dedup();
        for edge in edges {
            visit(self.graph, edge);
        }
    }
}
pub(super) struct SiteGradingContext<'a> {
    terrain: &'a TerrainSystem,
    roads: SiteRoads<'a>,
    safe_step_m: f32,
    max_distance_m: f32,
}

impl<'a> SiteGradingContext<'a> {
    pub(super) fn new(
        terrain: &'a TerrainSystem,
        graph: &'a RegionGraph,
        road_surface: &'a RoadbedSystem,
        safe_step_m: f32,
        max_distance_m: f32,
    ) -> Self {
        Self {
            terrain,
            roads: SiteRoads::new(graph, road_surface),
            safe_step_m,
            max_distance_m,
        }
    }
}

#[derive(Clone, Copy)]
struct FootprintEdge {
    start: Vector2,
    end: Vector2,
    outward: Vector2,
    sample_count: u32,
}

pub(crate) fn building_site_support_tie_in_is_valid(
    footprint_world: &[Vector2],
    support_height_m: f32,
    terrain: &TerrainSystem,
    graph: &RegionGraph,
    road_surface: &RoadbedSystem,
) -> bool {
    if !support_height_m.is_finite() {
        return false;
    }
    let safe_step_m = BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M.max(f32::EPSILON);
    let context = SiteGradingContext::new(
        terrain,
        graph,
        road_surface,
        safe_step_m,
        BUILDING_SITE_SUPPORT_TIE_IN_REACH_M,
    );
    visit_footprint_grading_rays(footprint_world, safe_step_m, |seam, outward| {
        building_site_support_tie_in_ray_is_valid(support_height_m, seam, outward, &context)
    })
}

/// Chooses the closest feasible level pad to the preferred road-connection height.
/// Intersects exact unions of the existing ray-height intervals, plus driveway grade intervals;
/// it does not raise the slope limit or move the road. Ties choose the lower height. The rays
/// read the base terrain and the roads out to the graded terrain's cap (`docs/terrain.md` T12
/// (a)).
pub(crate) fn solve_building_site_support_height(
    footprint: &[Vector2],
    preferred: f32,
    connections: &[(Vector2, f32)],
    terrain: &TerrainSystem,
    graph: &RegionGraph,
    surface: &RoadbedSystem,
) -> Option<f32> {
    if footprint.len() < 3 || !preferred.is_finite() || connections.is_empty() {
        return None;
    }
    let mut low = f32::NEG_INFINITY;
    let mut high = f32::INFINITY;
    for &(point, height) in connections {
        let distance = if point_in_polygon_slice(point, footprint) {
            0.0
        } else {
            (0..footprint.len())
                .map(|i| {
                    let a = footprint[i];
                    let delta = footprint[(i + 1) % footprint.len()] - a;
                    let t = ((point - a).dot(delta) / delta.length_squared()).clamp(0.0, 1.0);
                    point.distance_to(a + delta * t)
                })
                .fold(f32::INFINITY, f32::min)
        };
        let budget = distance * crate::simulation::terrain::graded::TIE_IN_SLOPE as f32;
        low = low.max(height - budget);
        high = high.min(height + budget);
    }
    if low > high {
        return None;
    }
    let intervals = site_support_intervals(footprint, terrain, graph, surface)?;
    intervals
        .into_iter()
        .filter_map(|(lo, hi)| {
            let lo = lo.max(low);
            let hi = hi.min(high);
            (lo <= hi).then(|| preferred.clamp(lo, hi))
        })
        .min_by(|a, b| {
            (a - preferred)
                .abs()
                .total_cmp(&(b - preferred).abs())
                .then(a.total_cmp(b))
        })
}

// An undrawn bank near a frontage may rise only one metre. Larger level differences need
// explicit retaining geometry (Part C); the 1:2 cone still bounds its horizontal reach.
/// Maximum undrawn near-frontage bank height before retaining geometry is required.
pub(crate) const FRONTAGE_BANK_MAX_M: f32 = 1.0;

/// Feasible floor intervals: intersect fixed roads, union regradeable terrain per ray.
pub(crate) fn site_support_intervals(
    footprint: &[Vector2],
    terrain: &TerrainSystem,
    graph: &RegionGraph,
    surface: &RoadbedSystem,
) -> Option<Vec<(f32, f32)>> {
    let context = SiteGradingContext::new(
        terrain,
        graph,
        surface,
        BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M,
        BUILDING_SITE_SUPPORT_TIE_IN_REACH_M,
    );
    let mut intervals = vec![(f32::NEG_INFINITY, f32::INFINITY)];
    let mut next = Vec::new();
    let valid = visit_footprint_grading_rays(footprint, context.safe_step_m, |seam, outward| {
        let mut fixed_low = f32::NEG_INFINITY;
        let mut fixed_high = f32::INFINITY;
        next.clear();
        let mut soft = false;
        for d in grading_ring_distances(context.safe_step_m, context.max_distance_m) {
            let pos = seam + outward * d;
            if let Some(target) = surface.exposed_height_at(pos.x, pos.y) {
                let budget = road_budget(d);
                fixed_low = fixed_low.max(target - budget);
                fixed_high = fixed_high.min(target + budget);
            } else {
                soft = true;
                let target = building_site_raw_tie_in_target_height(pos, d, terrain, context.roads);
                let budget = d * crate::simulation::terrain::graded::TIE_IN_SLOPE as f32;
                for &(lo, hi) in &intervals {
                    let a = lo.max(target - budget);
                    let b = hi.min(target + budget);
                    if a <= b {
                        next.push((a, b));
                    }
                }
            }
        }
        if !soft {
            next.extend_from_slice(&intervals);
        }
        next.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        intervals.clear();
        for &(lo, hi) in &next {
            let lo = lo.max(fixed_low);
            let hi = hi.min(fixed_high);
            if lo > hi {
                continue;
            }
            if let Some(last) = intervals.last_mut().filter(|last| lo <= last.1) {
                last.1 = last.1.max(hi);
            } else {
                intervals.push((lo, hi));
            }
        }
        !intervals.is_empty()
    });
    valid.then_some(intervals)
}

/// Reports incompatible immutable road samples using their actual ray distances.
pub(crate) fn fixed_road_issue(
    footprint: &[Vector2],
    surface: &RoadbedSystem,
) -> Option<StreetContactIssue> {
    let mut lower = (f32::NEG_INFINITY, 0.0, 0.0);
    let mut upper = (f32::INFINITY, 0.0, 0.0);
    visit_footprint_grading_rays(
        footprint,
        BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M,
        |seam, outward| {
            for d in grading_ring_distances(
                BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M,
                BUILDING_SITE_SUPPORT_TIE_IN_REACH_M,
            ) {
                let p = seam + outward * d;
                if let Some(h) = surface.exposed_height_at(p.x, p.y) {
                    if h - road_budget(d) > lower.0 {
                        lower = (h - road_budget(d), h, d);
                    }
                    if h + road_budget(d) < upper.0 {
                        upper = (h + road_budget(d), h, d);
                    }
                }
            }
            true
        },
    );
    (lower.0 > upper.0).then(|| StreetContactIssue {
        constraint: "fixed-road tie-in grade",
        measured: (((lower.1 - upper.1) / (lower.2 + upper.2)) * 10_000.0).round() as u32,
        limit: (crate::simulation::terrain::graded::TIE_IN_SLOPE * 10_000.0).round() as u32,
        unit: "%",
    })
}

fn road_budget(distance: f32) -> f32 {
    distance * crate::simulation::terrain::graded::TIE_IN_SLOPE as f32
}

/// Padding needed to capture every terrain and nearest-road dependency of site feasibility:
/// the tie-in reach (the graded terrain's cap), the nearest-road probe radius and one terrain
/// cell of bilinear support.
pub(crate) fn site_feasibility_dependency_margin_m(terrain: &TerrainSystem) -> f32 {
    BUILDING_SITE_SUPPORT_TIE_IN_REACH_M
        + BUILDING_SITE_NEAREST_ROAD_SURFACE_MAX_RADIUS_M
        + terrain.cell_size_m()
}

/// Visits boundary stations and corner bisectors at the shared grading spacing.
pub(crate) fn visit_footprint_grading_rays(
    footprint_world: &[Vector2],
    safe_step_m: f32,
    mut visit: impl FnMut(Vector2, Vector2) -> bool,
) -> bool {
    let signed_area = signed_polygon_area(footprint_world);
    if signed_area.abs() <= f32::EPSILON {
        return false;
    }
    let loop_is_ccw = signed_area > 0.0;
    let mut edges = Vec::with_capacity(footprint_world.len());

    for edge_idx in 0..footprint_world.len() {
        let start = footprint_world[edge_idx];
        let end = footprint_world[(edge_idx + 1) % footprint_world.len()];
        let delta = end - start;
        let length_m = delta.length();
        if length_m <= f32::EPSILON {
            return false;
        }
        let outward = if loop_is_ccw {
            Vector2::new(delta.y, -delta.x)
        } else {
            Vector2::new(-delta.y, delta.x)
        } / length_m;
        edges.push(FootprintEdge {
            start,
            end,
            outward: corrected_footprint_outward(footprint_world, (start + end) * 0.5, outward),
            sample_count: ((length_m / safe_step_m).ceil() as u32).max(1),
        });
    }

    for edge in &edges {
        for sample_idx in 0..=edge.sample_count {
            let t = sample_idx as f32 / edge.sample_count as f32;
            if !visit(edge.start.lerp(edge.end, t), edge.outward) {
                return false;
            }
        }
    }

    for vertex_idx in 0..footprint_world.len() {
        let previous = edges[(vertex_idx + edges.len() - 1) % edges.len()].outward;
        let next = edges[vertex_idx].outward;
        let bisector = previous + next;
        if bisector.length_squared() <= f32::EPSILON {
            continue;
        }
        let vertex = footprint_world[vertex_idx];
        let outward = corrected_footprint_outward(footprint_world, vertex, bisector.normalized());
        if !visit(vertex, outward) {
            return false;
        }
    }

    true
}

fn building_site_support_tie_in_ray_is_valid(
    seam_height_m: f32,
    seam: Vector2,
    outward: Vector2,
    context: &SiteGradingContext<'_>,
) -> bool {
    let mut soft_valid = false;
    let mut soft = false;
    for distance_m in grading_ring_distances(context.safe_step_m, context.max_distance_m) {
        let pos = seam + outward * distance_m;
        if let Some(target) = context.roads.sample_visible_height(pos.x, pos.y) {
            if (target - seam_height_m).abs()
                > road_budget(distance_m) + BUILDING_SITE_SUPPORT_TIE_IN_EPS_M
            {
                return false;
            }
        } else {
            soft = true;
            let target = building_site_raw_tie_in_target_height(
                pos,
                distance_m,
                context.terrain,
                context.roads,
            );
            soft_valid |= (target - seam_height_m).abs()
                <= distance_m * crate::simulation::terrain::graded::TIE_IN_SLOPE as f32
                    + BUILDING_SITE_SUPPORT_TIE_IN_EPS_M;
        }
    }
    !soft || soft_valid
}

fn grading_ring_distances(safe_step_m: f32, max_distance_m: f32) -> impl Iterator<Item = f32> {
    let mut previous_distance_m = 0.0_f32;
    BUILDING_SITE_GRADING_RING_MULTIPLIERS
        .into_iter()
        .map(move |multiplier| (safe_step_m * multiplier).min(max_distance_m))
        // The envelope need not be a power-of-two multiple of the render step (Kuopio: 20 m).
        // Validation and guide generation must both reach its actual outer boundary.
        .chain(std::iter::once(max_distance_m))
        .filter(move |&distance_m| {
            if distance_m <= previous_distance_m + f32::EPSILON {
                return false;
            }
            previous_distance_m = distance_m;
            true
        })
}

/// Samples the road or terrain target used to validate a proposed pad's support height: the
/// visible road, else the base terrain (`docs/terrain.md` T12 (a)), which the graded terrain
/// grades from.
pub(super) fn building_site_raw_tie_in_target_height(
    pos: Vector2,
    distance_m: f32,
    terrain: &TerrainSystem,
    roads: SiteRoads<'_>,
) -> f32 {
    let nearest_radius_m = distance_m.clamp(
        BUILDING_SITE_NEAREST_ROAD_SURFACE_MIN_RADIUS_M,
        BUILDING_SITE_NEAREST_ROAD_SURFACE_MAX_RADIUS_M,
    );
    if let Some(road_height_m) = building_site_visible_road_height(roads, pos, nearest_radius_m) {
        return road_height_m;
    }
    terrain.sample_height_world(pos.x, pos.y) * crate::config::HEIGHT_SCALE
}

fn building_site_visible_road_height(
    roads: SiteRoads<'_>,
    pos: Vector2,
    nearest_radius_m: f32,
) -> Option<f32> {
    if let Some(height_m) = roads.sample_visible_height(pos.x, pos.y) {
        return Some(height_m);
    }
    nearest_building_site_road_surface_sample(roads, pos, nearest_radius_m)
        .map(|(_, height_m)| height_m)
}

/// Nearest exposed road-side point, using the existing deterministic road-edge R-tree query.
pub(crate) fn nearest_building_site_road_surface_sample(
    roads: SiteRoads<'_>,
    pos: Vector2,
    nearest_radius_m: f32,
) -> Option<(Vector2, f32)> {
    nearest_road_surface_sample(roads, pos, nearest_radius_m, None)
}

/// The nearest road-side point in front of a secondary entrance, rather than behind its door.
pub(crate) fn entrance_road_surface_sample(
    roads: SiteRoads<'_>,
    pos: Vector2,
    direction: Vector2,
) -> Option<(Vector2, f32)> {
    nearest_road_surface_sample(
        roads,
        pos,
        BUILDING_SITE_SUPPORT_TIE_IN_REACH_M,
        Some(direction),
    )
}

/// Nearest road in a lot edge's outward half-plane, so a closer road behind it cannot mask
/// a corner or rear frontage. Uses the existing adjacent-road probe reach.
pub(crate) fn frontage_road_surface_sample(
    roads: SiteRoads<'_>,
    pos: Vector2,
    outward: Vector2,
) -> Option<(Vector2, f32)> {
    nearest_road_surface_sample(
        roads,
        pos,
        BUILDING_SITE_NEAREST_ROAD_SURFACE_MAX_RADIUS_M,
        Some(outward),
    )
}

fn nearest_road_surface_sample(
    roads: SiteRoads<'_>,
    pos: Vector2,
    nearest_radius_m: f32,
    direction: Option<Vector2>,
) -> Option<(Vector2, f32)> {
    let radius_m = nearest_radius_m.max(0.0);
    if radius_m <= f32::EPSILON {
        return None;
    }
    let mut best: Option<(f32, usize, Vector2, f32)> = None;
    roads.visit_edges_near_point(
        Vector3::new(pos.x, 0.0, pos.y),
        radius_m,
        |graph, edge_idx| {
            let Some(edge) = graph.get_edge(edge_idx) else {
                return;
            };
            // A tunnel is no road a site meets at the surface (`ROAD-61` round R8h).
            if edge.deleted
                || edge.class == EdgeClass::Tunnel
                || edge.physical_geometry.len() < 2
                || edge.physical_length <= 1e-6
            {
                return;
            }
            let Some(projection) =
                BuildingAllocator::project_point_to_edge_centerline(edge_idx, edge, pos)
            else {
                return;
            };
            let center = BuildingAllocator::sample_pos_on_edge(graph, edge_idx, projection.t);
            let tangent = BuildingAllocator::sample_tangent_on_edge(graph, edge_idx, projection.t);
            if tangent.length_squared() <= 1e-12 {
                return;
            }
            let normal = Vector2::new(tangent.y, -tangent.x) * projection.side as f32;
            let probe = center
                + normal * building_site_road_connection_lateral_offset_m(edge, projection.side);
            let delta = probe - pos;
            if direction.is_some_and(|forward| delta.dot(forward) <= SITE_POINT_EPS_M)
                && delta.length_squared() > SITE_POINT_EPS_M * SITE_POINT_EPS_M
            {
                return;
            }
            let dist_sq = probe.distance_squared_to(pos);
            if dist_sq > radius_m * radius_m {
                return;
            }
            let Some(height_m) = roads.sample_visible_height(probe.x, probe.y) else {
                return;
            };
            let replace = best
                .as_ref()
                .is_none_or(|(best_dist_sq, best_edge_idx, _, _)| {
                    dist_sq
                        .total_cmp(best_dist_sq)
                        .then(edge_idx.cmp(best_edge_idx))
                        .is_lt()
                });
            if replace {
                best = Some((dist_sq, edge_idx, probe, height_m));
            }
        },
    );
    best.map(|(_, _, probe, height_m)| (probe, height_m))
}

// The road's outer edge on `side` (sidewalk, or shoulder on a road without sidewalks), just
// inside it; a pocket edge's kerbs lie off its centreline (`ROAD-95`, D17).
pub(super) fn building_site_road_connection_lateral_offset_m(
    edge: &crate::simulation::network::graph::Edge,
    side: i8,
) -> f32 {
    (edge.frontage_half_width_m(side) + side_band_width_m(edge, side)
        - BUILDING_SITE_ROAD_SURFACE_PROBE_INSET_M)
        .max(0.0)
}

fn corrected_footprint_outward(
    footprint_world: &[Vector2],
    seam: Vector2,
    outward: Vector2,
) -> Vector2 {
    if point_in_polygon_slice(seam + outward * SITE_POINT_EPS_M * 8.0, footprint_world) {
        -outward
    } else {
        outward
    }
}
