// SPDX-License-Identifier: GPL-2.0-only

//! Local ramp stubs with an outer auxiliary lane; preview and commit share this layout.

use super::graph::{Edge, RegionGraph, RoadLaneTaper};
use super::types::{EdgeClass, NodeType, TransitFlags, TransitType};
use crate::config::LANE_WIDTH;
use godot::prelude::Vector3;

/// Stations follow the selected edge's stored direction; points run from its centreline outward.
#[derive(Clone)]
pub(crate) struct RampRequest {
    pub(crate) edge: usize,
    pub(crate) station: f32,
    pub(crate) forward: bool,
    pub(crate) exit: bool,
    pub(crate) points: Vec<Vector3>,
    pub(crate) length: f32,
    pub(crate) taper: f32,
}

/// A bounded replacement for one straight divided road. No live state is mutated during planning.
pub(crate) struct RampLayout {
    pub(crate) originals: Vec<(usize, Edge)>,
    pieces: Vec<Edge>,
    ramp: Edge,
    transition: Vector3,
    fork: Vector3,
    fork_first: bool,
    forward: bool,
    exit: bool,
}

impl RampLayout {
    /// O(P + K log K + local segment pairs), using the existing edge R-tree for clearance.
    pub(crate) fn prepare(graph: &RegionGraph, request: RampRequest) -> Result<Self, &'static str> {
        // The local fork-mouth prediction rebuilds clips through Rayon, including on hover.
        crate::thread_pools::install(|| Self::prepare_in_pool(graph, request))
    }

    fn prepare_in_pool(graph: &RegionGraph, request: RampRequest) -> Result<Self, &'static str> {
        let RampRequest {
            edge: id,
            station,
            forward,
            exit,
            mut points,
            length,
            taper,
        } = request;
        let edge = graph
            .get_edge(id)
            .filter(|e| !e.deleted)
            .ok_or("invalid_road")?;
        if edge.primary_type != TransitType::Road
            || edge.class != EdgeClass::Standard
            || edge.median.is_none()
            || edge.bus_lanes != [0; 2]
            || edge.roundabout
            || edge.lane_tapers.is_some()
            || edge.fwd_lanes == 0
            || edge.bkw_lanes == 0
        {
            return Err("ramp_requires_plain_divided_road");
        }
        if (if forward {
            edge.fwd_lanes
        } else {
            edge.bkw_lanes
        }) >= 4
        {
            return Err("ramp_lane_limit");
        }
        if !station.is_finite()
            || !length.is_finite()
            || !taper.is_finite()
            || !(60.0..=300.0).contains(&length)
            || !(12.0..=100.0).contains(&taper)
            || points.len() != 2
            || points.iter().any(|p| !p.is_finite())
        {
            return Err("invalid_ramp_dimensions");
        }
        let start = graph.node(edge.start_node).pos;
        let end = graph.node(edge.end_node).pos;
        let total = start.distance_to(end);
        if total < 1.0 || edge.physical_geometry.len() < 2 {
            return Err("ramp_requires_straight_road");
        }
        let axis = (end - start) / total;
        if edge
            .physical_geometry
            .iter()
            .chain(&edge.geometry)
            .any(|&p| {
                let d = p - start;
                (d - axis * d.dot(axis)).length() > 0.001
            })
        {
            return Err("ramp_requires_straight_road");
        }
        let travel = axis * if forward { 1.0 } else { -1.0 };
        let right = Vector3::new(-travel.z, 0.0, travel.x).normalized();
        let fork = start + axis * station;
        let direction = points[1] - fork;
        let along = direction.dot(travel) * if exit { 1.0 } else { -1.0 };
        let across = direction.dot(right);
        // A real outward branch is required; a crossing arm must remain an ordinary junction.
        if points[0].distance_to(fork) > 0.01
            || along <= 0.0
            || across / along < 0.47
            || across / along > 2.75
            || points.last().unwrap().distance_to(fork) < 80.0
            || points.iter().skip(1).any(|p| (*p - fork).dot(right) <= 0.0)
        {
            return Err("ramp_requires_outer_branch");
        }
        if points.windows(2).any(|p| {
            let d = p[1] - p[0];
            let horizontal = (d.x * d.x + d.z * d.z).sqrt();
            horizontal < 0.1 || d.y.abs() > horizontal * 0.16
        }) {
            return Err("ramp_grade_out_of_range");
        }
        let fork_first = forward != exit;
        let mut auxiliary = edge.clone();
        auxiliary.fwd_lanes += u8::from(forward);
        auxiliary.bkw_lanes += u8::from(!forward);
        auxiliary.width += LANE_WIDTH;
        let mut tapers = [None, None];
        tapers[usize::from(fork_first)] = Some(RoadLaneTaper {
            length_m: taper,
            from_fwd: edge.fwd_lanes,
            from_bkw: edge.bkw_lanes,
            from_median_m: edge.median.unwrap().width_m,
            left_pocket: false,
            right_pocket: true,
        });
        auxiliary.lane_tapers = Some(Box::new(tapers));
        // Predict the actual fork clip with the existing solver on three local arms.
        // The auxiliary's usable full-width length excludes both this mouth and its taper.
        let mut mouth_graph = RegionGraph::new();
        for p in [start, fork, end, *points.last().unwrap()] {
            mouth_graph.add_node(p, NodeType::Junction);
        }
        for (i, (a, b)) in [(start, fork), (fork, end)].into_iter().enumerate() {
            let mut arm = if (i == 1) == fork_first {
                auxiliary.clone()
            } else {
                edge.clone()
            };
            arm.start_node = i as u32;
            arm.end_node = i as u32 + 1;
            arm.geometry = vec![a, b];
            arm.physical_geometry = arm.geometry.clone();
            arm.physical_length = a.distance_to(b);
            mouth_graph.add_edge(arm);
        }
        mouth_graph.add_edge(Edge {
            start_node: 1,
            end_node: 3,
            width: LANE_WIDTH,
            geometry: points.clone(),
            physical_geometry: points.clone(),
            physical_length: fork.distance_to(*points.last().unwrap()),
            primary_type: TransitType::Road,
            allowed_types: TransitFlags::CAR,
            fwd_lanes: 1,
            ..Default::default()
        });
        mouth_graph.rebuild_intersection_clips_for_nodes(&std::collections::HashSet::from([1]));
        let arm = mouth_graph.edge(usize::from(fork_first));
        let mouth = if fork_first {
            arm.start_clip
        } else {
            arm.end_clip
        };
        let span = length + taper + mouth;
        let other = station + if fork_first { span } else { -span };
        let lo = station.min(other);
        let hi = station.max(other);
        if lo < edge.start_clip + 30.0 || hi > total - edge.end_clip - 30.0 {
            return Err("ramp_road_too_short");
        }
        let transition = start + axis * other;
        let cuts = [0.0, lo, hi, total];
        let mut pieces = Vec::with_capacity(3);
        for (i, range) in cuts.windows(2).enumerate() {
            let mut piece = if i == 1 {
                auxiliary.clone()
            } else {
                edge.clone()
            };
            piece.geometry = vec![start + axis * range[0], start + axis * range[1]];
            piece.physical_geometry = piece.geometry.clone();
            piece.physical_length = range[1] - range[0];
            piece.start_clip = 0.0;
            piece.end_clip = 0.0;
            if let Some(m) = &mut piece.median {
                m.start_setback_m = 0.0;
                m.end_setback_m = 0.0;
            }
            pieces.push(piece);
        }
        pieces[0].median.as_mut().unwrap().start_setback_m = edge.median.unwrap().start_setback_m;
        pieces[2].median.as_mut().unwrap().end_setback_m = edge.median.unwrap().end_setback_m;
        if !exit {
            points.reverse();
        }
        let ramp = Edge {
            geometry: points.clone(),
            physical_geometry: points,
            fwd_lanes: 1,
            bkw_lanes: 0,
            width: LANE_WIDTH,
            allowed_types: TransitFlags::CAR,
            primary_type: TransitType::Road,
            speed_limit: edge.speed_limit.min(60.0 / 3.6),
            crossing_level: edge.crossing_level,
            no_building_spawn: true,
            ..Default::default()
        };
        let layout = Self {
            originals: vec![(id, edge.clone())],
            pieces,
            ramp,
            transition,
            fork,
            fork_first,
            forward,
            exit,
        };
        layout.check_clearance(graph)?;
        Ok(layout)
    }

    fn check_clearance(&self, graph: &RegionGraph) -> Result<(), &'static str> {
        use super::lanes::geometry::segment_distance_squared_xz;
        use super::surface::RoadSurfaceSystem;
        use godot::prelude::Vector2;
        let flat = |p: Vector3| Vector2::new(p.x, p.z);
        for corridor in [&self.pieces[1], &self.ramp] {
            let mut min = Vector3::new(f32::INFINITY, 0.0, f32::INFINITY);
            let mut max = Vector3::new(f32::NEG_INFINITY, 0.0, f32::NEG_INFINITY);
            for p in &corridor.geometry {
                min.x = min.x.min(p.x);
                min.z = min.z.min(p.z);
                max.x = max.x.max(p.x);
                max.z = max.z.max(p.z);
            }
            let half = RoadSurfaceSystem::visual_roadbed_half_width_m(corridor);
            for id in graph.get_edges_near_aabb(
                min - Vector3::new(half + 40.0, 0.0, half + 40.0),
                max + Vector3::new(half + 40.0, 0.0, half + 40.0),
            ) {
                if id == self.originals[0].0 {
                    continue;
                }
                let other = graph.edge(id);
                if other.deleted {
                    continue;
                }
                // The first slice refuses crossings instead of guessing grade/support ownership.
                let gap = half + RoadSurfaceSystem::visual_roadbed_half_width_m(other);
                if corridor.geometry.windows(2).any(|a| {
                    other.physical_geometry.windows(2).any(|b| {
                        segment_distance_squared_xz(flat(a[0]), flat(a[1]), flat(b[0]), flat(b[1]))
                            < gap * gap
                    })
                }) {
                    return Err("ramp_overlaps_road");
                }
            }
        }
        Ok(())
    }

    /// New/widened corridors for existing water, field, parcel and zoning guards.
    pub(crate) fn corridors(&self) -> impl Iterator<Item = &Edge> {
        self.pieces.iter().chain(std::iter::once(&self.ramp))
    }

    /// Publishes graph indices and all lane whitelists together; caller owns dependent publication.
    pub(crate) fn apply(mut self, graph: &mut RegionGraph) -> Vec<usize> {
        let old = &self.originals[0];
        let transition = graph.add_node(self.transition, NodeType::LaneTransition);
        let fork = graph.add_node(self.fork, NodeType::Ramp);
        let terminal_point = if self.exit {
            *self.ramp.geometry.last().unwrap()
        } else {
            self.ramp.geometry[0]
        };
        let terminal = graph.add_node(terminal_point, NodeType::Junction);
        let middle = if self.fork_first {
            [fork, transition]
        } else {
            [transition, fork]
        };
        let nodes = [old.1.start_node, middle[0], middle[1], old.1.end_node];
        let ids = [old.0, graph.edge_count(), graph.edge_count() + 1];
        for (i, piece) in self.pieces.iter_mut().enumerate() {
            piece.start_node = nodes[i];
            piece.end_node = nodes[i + 1];
            if i == 0 {
                super::pockets::replace_edge(graph, ids[i], piece.clone());
            } else {
                graph.add_edge(piece.clone());
            }
        }
        self.ramp.start_node = if self.exit { fork } else { terminal };
        self.ramp.end_node = if self.exit { terminal } else { fork };
        self.ramp.physical_length = graph.calculate_length(&self.ramp.physical_geometry);
        let ramp = graph.add_edge(self.ramp);
        // References at the old stored end now belong to the last main-road piece.
        let end = &mut graph.nodes[old.1.end_node as usize];
        if let Some(value) = end.crosswalk_overrides.remove(&old.0) {
            end.crosswalk_overrides.insert(ids[2], value);
        }
        let previous = std::mem::take(&mut end.lane_connections);
        for ((from, lane), mut targets) in previous {
            for (to, _) in &mut targets {
                if *to == old.0 {
                    *to = ids[2];
                }
            }
            end.lane_connections
                .insert((if from == old.0 { ids[2] } else { from }, lane), targets);
        }
        let (before, after) = if self.fork_first {
            (ids[0], ids[1])
        } else {
            (ids[1], ids[2])
        };
        let index = |rank: u8, fwd: bool| if fwd { rank as i8 } else { -(rank as i8) - 1 };
        for (from, to, fwd) in [(before, after, true), (after, before, false)] {
            let a = graph.edge(from);
            let b = graph.edge(to);
            let count = if fwd {
                a.fwd_lanes.min(b.fwd_lanes)
            } else {
                a.bkw_lanes.min(b.bkw_lanes)
            };
            for rank in 0..count {
                graph.add_lane_connection(fork, from, index(rank, fwd), to, index(rank, fwd));
            }
        }
        let aux = ids[1];
        let extra = if self.forward {
            graph.edge(aux).fwd_lanes - 1
        } else {
            graph.edge(aux).bkw_lanes - 1
        };
        if self.exit {
            graph.add_lane_connection(fork, aux, index(extra, self.forward), ramp, 0);
        } else {
            graph.add_lane_connection(fork, ramp, 0, aux, index(extra, self.forward));
        }
        for id in [before, after, ramp] {
            graph.set_crosswalk_override(fork, id, false);
        }
        vec![old.0, ids[1], ids[2], ramp]
    }
}
