// SPDX-License-Identifier: GPL-2.0-only

//! Deterministic bounded-grade crossing profiles for the player's over/under selector.

use super::{band_quad, triangle_overlap, xz};
use crate::simulation::network::build_surface_edge;
use crate::simulation::network::graph::{RegionGraph, RoadMedian};
use crate::simulation::network::surface::{RoadSurfaceSystem, RoadVec2};
use crate::simulation::network::types::{EdgeClass, NodeType, TransitFlags};
use godot::prelude::Vector3;

const APPROACH_GRADE: f64 = 0.12;
const PROFILE_STEP_M: f64 = 4.0;

/// Solved input consumed unchanged by the existing exact road preview and commit pipeline.
pub(crate) struct CrossingProfile {
    /// Physical profile, including grounded endpoints and the clearance plateau.
    pub(crate) points: Vec<Vector3>,
    /// Explicit level above or below every crossed edge.
    pub(crate) level: i16,
    /// Crossed source owners for the tool's highlight.
    pub(crate) edges: Vec<usize>,
}

#[derive(Clone, Copy)]
struct Constraint {
    start: f64,
    end: f64,
    height: f64,
}

impl RoadSurfaceSystem {
    /// Solves a 12% Lipschitz envelope around exact roadbed overlap intervals. Endpoints
    /// stay authored; inadequate approach length is refused. Editing work is
    /// O(P log E + K log K + P sum(Q B) + C log C + S C), with
    /// O(S + C + K + (P + max(Q)) B) scratch and no per-agent work or persistent index.
    /// K counts the local R-tree query hits before deduplication.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn prepare_crossing_profile(
        &self,
        source: &RegionGraph,
        raw: &[Vector3],
        lanes: (u8, u8),
        median: Option<RoadMedian>,
        over: bool,
        source_level: Option<i16>,
    ) -> Result<CrossingProfile, &'static str> {
        if raw.len() < 2 || raw.iter().any(|p| !p.is_finite()) {
            return Err("too_short");
        }
        let mut candidate = RegionGraph::new();
        let start = candidate.add_node(raw[0], NodeType::Junction);
        let end = candidate.add_node(*raw.last().unwrap(), NodeType::Junction);
        // Footprint stations use horizontal arc length, matching the grade envelope below.
        let alignment = raw.iter().map(|p| Vector3::new(p.x, 0.0, p.z)).collect();
        let mut edge =
            build_surface_edge(start, end, alignment, lanes.0, lanes.1, EdgeClass::Standard);
        edge.median = median;
        edge.width += median.map_or(0.0, |m| m.width_m);
        let candidate_id = candidate.add_edge(edge);
        let sections = self.compile_edge_sections(&candidate, candidate_id);
        let length = f64::from(sections.last().ok_or("too_short")?.s_m);
        let halo = Self::visual_roadbed_half_width_m(candidate.edge(candidate_id)) + 24.0;
        let mut neighbors = Vec::new();
        for pair in raw.windows(2) {
            let min = Vector3::new(
                pair[0].x.min(pair[1].x) - halo,
                0.0,
                pair[0].z.min(pair[1].z) - halo,
            );
            let max = Vector3::new(
                pair[0].x.max(pair[1].x) + halo,
                0.0,
                pair[0].z.max(pair[1].z) + halo,
            );
            neighbors.extend(source.get_edges_near_aabb(min, max));
        }
        neighbors.sort_unstable();
        neighbors.dedup();
        let mut constraints = Vec::new();
        let mut edges = Vec::new();
        let mut level = None::<i16>;
        for id in neighbors {
            let edge = source.edge(id);
            if !Self::is_surface_edge(edge) {
                continue;
            }
            // Imported levels already declare topology. Same-level crossings are
            // ordinary junctions, never constraints that manufacture another tunnel.
            if source_level.is_some_and(|level| {
                if over {
                    edge.crossing_level >= level
                } else {
                    edge.crossing_level <= level
                }
            }) {
                continue;
            }
            let lower_is_vehicle = if over {
                edge.allowed_types & TransitFlags::CAR != 0
            } else {
                lanes != (0, 0)
            };
            let clearance = super::DECK_DEPTH_M
                + if lower_is_vehicle {
                    super::VEHICLE_CLEARANCE_M
                } else {
                    super::FOOT_CYCLE_CLEARANCE_M
                };
            let other = self.compile_edge_sections(source, id);
            let shares_endpoint = |point: Vector3| {
                edge.physical_geometry
                    .first()
                    .into_iter()
                    .chain(edge.physical_geometry.last())
                    .any(|&end| end.distance_squared_to(point) <= 1e-6)
            };
            let shares_start = shares_endpoint(raw[0]);
            let shares_end = shares_endpoint(*raw.last().unwrap());
            let before = constraints.len();
            for a in sections.windows(2) {
                // Clearance belongs to the whole band footprint. A lower sidewalk/kerb is
                // above its centreline; an upper road's lowest band may be below it.
                let offset = a
                    .iter()
                    .flat_map(|s| {
                        s.bands.iter().flat_map(move |band| {
                            [
                                f64::from(band.height_start_m - s.center_height_m),
                                f64::from(band.height_end_m - s.center_height_m),
                            ]
                        })
                    })
                    .fold(
                        if over {
                            f64::INFINITY
                        } else {
                            f64::NEG_INFINITY
                        },
                        |old, value| if over { old.min(value) } else { old.max(value) },
                    );
                let left = |section: &crate::simulation::network::surface::RoadSurfaceSection,
                            right: bool| {
                    let band = if right {
                        section.bands.last().unwrap()
                    } else {
                        &section.bands[0]
                    };
                    Self::section_boundary_world_point_static(
                        section,
                        if right {
                            band.lateral_end_m
                        } else {
                            band.lateral_start_m
                        },
                        section.center_height_m,
                    )
                };
                let quad = [
                    left(&a[0], false),
                    left(&a[1], false),
                    left(&a[1], true),
                    left(&a[0], true),
                ];
                let delta = a[1].center_xz - a[0].center_xz;
                if delta.length_squared() < 1e-10 {
                    continue;
                }
                for b in other.windows(2) {
                    for (first, second) in b[0].bands.iter().zip(&b[1].bands) {
                        let Some(surface) = band_quad(b, first, second) else {
                            continue;
                        };
                        for ai in [[0, 1, 2], [0, 2, 3]] {
                            for &triangle in surface.triangles() {
                                let Some((points, count)) =
                                    triangle_overlap(ai.map(|i| quad[i]), triangle)
                                else {
                                    continue;
                                };
                                let mut begin = f64::INFINITY;
                                let mut end = f64::NEG_INFINITY;
                                let mut height = if over {
                                    f64::NEG_INFINITY
                                } else {
                                    f64::INFINITY
                                };
                                for &point in &points[..count] {
                                    let t = (point - a[0].center_xz).dot(delta)
                                        / delta.length_squared();
                                    let station = f64::from(a[0].s_m)
                                        + f64::from(a[1].s_m - a[0].s_m) * t.clamp(0.0, 1.0);
                                    begin = begin.min(station);
                                    end = end.max(station);
                                    let y = triangle_height(triangle, point);
                                    // Two millimetres cover the shared 1 mm boundary grid and
                                    // conversion to the Godot f32 profile at city coordinates.
                                    height = if over {
                                        height.max(y + clearance - offset + 0.002)
                                    } else {
                                        height.min(y - clearance - offset - 0.002)
                                    };
                                }
                                constraints.push(Constraint {
                                    start: begin,
                                    end,
                                    height,
                                });
                            }
                        }
                    }
                }
            }
            if constraints.len() > before {
                // One continuous roadbed overlap owns one clearance plateau. Taking each
                // kerb strip separately would put shallow kerb-shaped dips in the upper road.
                constraints[before..].sort_by(|a, b| a.start.total_cmp(&b.start));
                let mut write = before;
                for read in before..constraints.len() {
                    let next = constraints[read];
                    if write > before && next.start <= constraints[write - 1].end + 0.001 {
                        let prior = &mut constraints[write - 1];
                        prior.end = prior.end.max(next.end);
                        prior.height = if over {
                            prior.height.max(next.height)
                        } else {
                            prior.height.min(next.height)
                        };
                    } else {
                        constraints[write] = next;
                        write += 1;
                    }
                }
                constraints.truncate(write);
                // The endpoint-connected overlap is the retained approach joint, whose exact
                // clips are solved by ordinary topology. Other crossings of that same curved
                // source road remain constraints. Never move the shared endpoint to clear itself.
                let mut write = before;
                for read in before..constraints.len() {
                    let constraint = constraints[read];
                    if (shares_start && constraint.start <= 0.001)
                        || (shares_end && constraint.end >= length - 0.001)
                    {
                        continue;
                    }
                    constraints[write] = constraint;
                    write += 1;
                }
                constraints.truncate(write);
            }
            if constraints.len() > before {
                edges.push(id);
                level = Some(level.map_or(edge.crossing_level, |old| {
                    if over {
                        old.max(edge.crossing_level)
                    } else {
                        old.min(edge.crossing_level)
                    }
                }));
            }
        }
        let level = level.ok_or("crossing_not_found")?;
        let level = source_level
            .or_else(|| {
                if over {
                    level.checked_add(1)
                } else {
                    level.checked_sub(1)
                }
            })
            .ok_or("crossing_level_out_of_range")?;
        let mut cumulative = vec![0.0];
        for pair in raw.windows(2) {
            let delta = RoadVec2::new(
                f64::from(pair[1].x - pair[0].x),
                f64::from(pair[1].z - pair[0].z),
            );
            cumulative.push(cumulative.last().unwrap() + delta.length());
        }
        let length = *cumulative.last().unwrap();
        if length < 1.0 {
            return Err("too_short");
        }
        // Subdivide each authored segment evenly, preserving its vertices without combining
        // two independent station lattices. Near-coincident constraint/raw knots destabilise
        // offset rails at city-coordinate f32 precision. Instead expand each clearance plateau
        // to the enclosing construction stations, conservatively keeping the opening clear.
        let mut stations = vec![0.0];
        for pair in cumulative.windows(2) {
            let run = pair[1] - pair[0];
            let steps = (run / PROFILE_STEP_M).ceil() as usize;
            stations.extend((1..=steps).map(|i| pair[0] + run * i as f64 / steps as f64));
        }
        stations.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        for constraint in &mut constraints {
            let start = stations
                .partition_point(|&s| s <= constraint.start)
                .saturating_sub(1);
            let end = stations
                .partition_point(|&s| s < constraint.end)
                .min(stations.len() - 1);
            constraint.start = stations[start];
            constraint.end = stations[end];
        }
        // Between these aligned boundaries the Over envelope is a maximum of affine terms
        // (convex), whose endpoint chord stays above it. Under is the corresponding minimum
        // (concave), so its chord stays below. No sub-station clearance peak is omitted.
        let mut points = Vec::with_capacity(stations.len());
        let mut segment = 0;
        for station in stations {
            while segment + 2 < raw.len() && cumulative[segment + 1] < station {
                segment += 1;
            }
            let run = cumulative[segment + 1] - cumulative[segment];
            if run < 1e-8 {
                continue;
            }
            let t = ((station - cumulative[segment]) / run).clamp(0.0, 1.0) as f32;
            let mut point = raw[segment].lerp(raw[segment + 1], t);
            let mut height = f64::from(point.y);
            for constraint in &constraints {
                let distance = (constraint.start - station)
                    .max(station - constraint.end)
                    .max(0.0);
                let bound = constraint.height
                    + if over {
                        -APPROACH_GRADE * distance
                    } else {
                        APPROACH_GRADE * distance
                    };
                height = if over {
                    height.max(bound)
                } else {
                    height.min(bound)
                };
            }
            if (station < 1e-5 || length - station < 1e-5)
                && (height - f64::from(point.y)).abs() > 0.01
            {
                return Err("crossing_approach_too_short");
            }
            point.y = height as f32;
            points.push(point);
        }
        // Extend the same grade envelope over the raw terrain profile as well. Otherwise
        // a steep sampled hill outside the opening survives and fails exact placement.
        for reverse in [false, true] {
            for step in 1..points.len() {
                let (i, previous) = if reverse {
                    (points.len() - 1 - step, points.len() - step)
                } else {
                    (step, step - 1)
                };
                let dx = f64::from(points[i].x) - f64::from(points[previous].x);
                let dz = f64::from(points[i].z) - f64::from(points[previous].z);
                let run = dx.hypot(dz);
                let bound = f64::from(points[previous].y)
                    + if over {
                        -APPROACH_GRADE * run
                    } else {
                        APPROACH_GRADE * run
                    };
                points[i].y = if over {
                    f64::from(points[i].y).max(bound)
                } else {
                    f64::from(points[i].y).min(bound)
                } as f32;
            }
        }
        for (index, original) in [(0, raw[0]), (points.len() - 1, *raw.last().unwrap())] {
            if (points[index].y - original.y).abs() > 0.01 {
                return Err("crossing_approach_too_short");
            }
            points[index] = original;
        }
        // f64 stations may become identical after conversion to Godot's f32 points.
        points.dedup();
        Ok(CrossingProfile {
            points,
            level,
            edges,
        })
    }
}

fn triangle_height(triangle: [super::DVec3; 3], point: RoadVec2) -> f64 {
    let origin = xz(triangle[0]);
    let ab = xz(triangle[1]) - origin;
    let ac = xz(triangle[2]) - origin;
    let offset = point - origin;
    triangle[0].y
        + (offset.perp_dot(ac) * (triangle[1].y - triangle[0].y)
            + ab.perp_dot(offset) * (triangle[2].y - triangle[0].y))
            / ab.perp_dot(ac)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_approach_joint_does_not_erase_another_crossing_of_the_same_road() {
        let mut graph = RegionGraph::new();
        let raw = [
            Vector3::new(0.0, 0.0, -140.0),
            Vector3::new(0.0, 0.0, 140.0),
        ];
        let points = vec![
            raw[0],
            Vector3::new(-80.0, 0.0, -120.0),
            Vector3::new(-80.0, 0.0, 0.0),
            Vector3::new(80.0, 0.0, 0.0),
        ];
        let start = graph.add_node(points[0], NodeType::Junction);
        let end = graph.add_node(*points.last().unwrap(), NodeType::Junction);
        graph.add_edge(build_surface_edge(
            start,
            end,
            points,
            1,
            1,
            EdgeClass::Standard,
        ));
        let surface = RoadSurfaceSystem::new(128.0);
        let profile = surface
            .prepare_crossing_profile(&graph, &raw, (1, 1), None, true, None)
            .unwrap();
        assert_eq!(profile.points.first(), Some(&raw[0]));
        assert_eq!(profile.points.last(), Some(&raw[1]));
        assert_eq!(profile.edges, [0]);
        assert!(
            profile.points.iter().all(|p| {
                let station = (p.z + 140.0) / 4.0;
                (station - station.round()).abs() < 1e-4
            }),
            "Clearance boundaries use the enclosing construction stations"
        );
        assert!(
            profile
                .points
                .iter()
                .filter(|p| p.z.abs() < 4.0)
                .all(|p| p.y > 5.4)
        );
    }

    #[test]
    #[ignore = "manual release locality timing of automatic crossing profiles"]
    fn benchmark_crossing_profile_locality() {
        use std::{hint::black_box, time::Instant};
        let surface = RoadSurfaceSystem::new(128.0);
        let raw = [
            Vector3::new(0.0, 0.0, -140.0),
            Vector3::new(0.0, 0.0, 140.0),
        ];
        let mut expected = None;
        for background in [0, 1_000, 10_000, 100_000] {
            let mut graph = RegionGraph::new();
            for index in 0..=background {
                let (a, b) = if index == 0 {
                    (
                        Vector3::new(-160.0, 0.0, 0.0),
                        Vector3::new(160.0, 0.0, 0.0),
                    )
                } else {
                    let a = Vector3::new(
                        1000.0 + (index % 512) as f32 * 32.0,
                        0.0,
                        1000.0 + (index / 512) as f32 * 32.0,
                    );
                    (a, a + Vector3::new(16.0, 0.0, 0.0))
                };
                let start = graph.add_node(a, NodeType::Junction);
                let end = graph.add_node(b, NodeType::Junction);
                graph.add_edge(build_surface_edge(
                    start,
                    end,
                    vec![a, b],
                    2,
                    2,
                    EdgeClass::Standard,
                ));
            }
            // All construction/index publication is outside repeated profile planning.
            let profile = surface
                .prepare_crossing_profile(&graph, &raw, (1, 1), None, true, None)
                .unwrap();
            let product = (profile.points, profile.level, profile.edges);
            if let Some(expected) = &expected {
                assert_eq!(&product, expected);
            } else {
                expected = Some(product);
            }
            let mut samples = Vec::with_capacity(200);
            for _ in 0..10 {
                black_box(
                    surface
                        .prepare_crossing_profile(&graph, &raw, (1, 1), None, true, None)
                        .unwrap(),
                );
            }
            for _ in 0..200 {
                let started = Instant::now();
                let profile = surface
                    .prepare_crossing_profile(black_box(&graph), &raw, (1, 1), None, true, None)
                    .unwrap();
                samples.push(started.elapsed().as_secs_f64() * 1000.0);
                assert_eq!(
                    &(profile.points, profile.level, profile.edges),
                    expected.as_ref().unwrap()
                );
            }
            samples.sort_by(f64::total_cmp);
            eprintln!(
                "CROSSING_PROFILE background={background} local_edges=1 samples=200 median_ms={:.6} p95_ms={:.6}",
                samples[100], samples[190]
            );
        }
    }
}
