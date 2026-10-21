// SPDX-License-Identifier: GPL-2.0-only

//! Deterministic local pocket layout shared by endpoint preview and commit.

use super::graph::{Edge, RegionGraph, RoadLaneTaper};
use super::types::{EdgeClass, NodeType, TransitFlags, TransitType};
use crate::config::{LANE_WIDTH, SIDEWALK_WIDTH};
use godot::prelude::Vector3;

/// Immutable endpoint edit; dimensions exclude the junction mouth and the taper respectively.
#[derive(Clone, Copy)]
pub(crate) struct PocketRequest {
    pub(crate) edge: usize,
    pub(crate) at_start: bool,
    pub(crate) left: bool,
    pub(crate) right: bool,
    pub(crate) length: f32,
    pub(crate) taper: f32,
}

/// Bounded topology product. No agent or frontage state is consulted while planning.
pub(crate) struct PocketLayout {
    pub(crate) originals: Vec<(usize, Edge)>,
    pieces: Vec<Edge>,
    transition: Option<(u32, Vector3)>,
    junction: u32,
    at_start: bool,
    old_left: bool,
    new_left: bool,
    old_pocket: usize,
}

impl PocketLayout {
    /// Plans addition, resizing, role changes or removal using only the selected pair and node arms.
    pub(crate) fn prepare(
        graph: &RegionGraph,
        request: PocketRequest,
    ) -> Result<Self, &'static str> {
        let PocketRequest {
            edge: id,
            mut at_start,
            left,
            right,
            length,
            taper,
        } = request;
        let selected = graph
            .get_edge(id)
            .filter(|e| !e.deleted)
            .ok_or("invalid_road")?;
        let existing =
            selected.pocket_taper().map(|_| id).or_else(|| {
                let node = if at_start {
                    selected.start_node
                } else {
                    selected.end_node
                };
                (graph.node(node).node_type == NodeType::LaneTransition)
                    .then(|| {
                        graph.node_adjacency(node).iter().copied().find(|&other| {
                            other != id && graph.edge(other).pocket_taper().is_some()
                        })
                    })
                    .flatten()
            });
        let (originals, mut base, transition, old_left, old_pocket) =
            if let Some(pocket_id) = existing {
                let pocket = graph.edge(pocket_id);
                let (end, profile) = pocket.pocket_taper().unwrap();
                at_start = end == 1;
                let node = if end == 0 {
                    pocket.start_node
                } else {
                    pocket.end_node
                };
                if graph.node(node).node_type != NodeType::LaneTransition
                    || graph.live_node_connection_count(node) != 2
                {
                    return Err("pocket_transition_connected");
                }
                let other = *graph
                    .node_adjacency(node)
                    .iter()
                    .find(|&&other| other != pocket_id && !graph.edge(other).deleted)
                    .ok_or("pocket_transition_missing")?;
                let (first, second) = if graph.edge(other).end_node == node {
                    (other, pocket_id)
                } else {
                    (pocket_id, other)
                };
                if graph.edge(first).end_node != node || graph.edge(second).start_node != node {
                    return Err("pocket_alignment_changed");
                }
                let mut joined = graph.edge(first).clone();
                joined.end_node = graph.edge(second).end_node;
                joined
                    .geometry
                    .extend_from_slice(&graph.edge(second).geometry[1..]);
                let mut physical =
                    RegionGraph::physical_profile_on_control_alignment(graph.edge(first));
                physical.extend_from_slice(
                    &RegionGraph::physical_profile_on_control_alignment(graph.edge(second))[1..],
                );
                joined.physical_geometry = physical;
                joined.fwd_lanes = profile.from_fwd;
                joined.bkw_lanes = profile.from_bkw;
                joined.median = pocket.median;
                joined.lane_tapers = None;
                joined.width = f32::from(profile.from_fwd + profile.from_bkw) * LANE_WIDTH
                    + profile.from_median_m;
                joined.end_clip = graph.edge(second).end_clip;
                (
                    vec![
                        (first, graph.edge(first).clone()),
                        (second, graph.edge(second).clone()),
                    ],
                    joined,
                    Some(node),
                    profile.left_pocket,
                    pocket_id,
                )
            } else {
                if selected.lane_tapers.is_some() {
                    return Err("pocket_overlaps_transition");
                }
                (
                    vec![(id, selected.clone())],
                    selected.clone(),
                    None,
                    false,
                    id,
                )
            };
        if base.primary_type != TransitType::Road || base.class != EdgeClass::Standard {
            return Err("pocket_requires_surface_road");
        }
        let junction = if at_start {
            base.start_node
        } else {
            base.end_node
        };
        let total = RegionGraph::new().calculate_length(&base.physical_geometry);
        base.physical_length = total;
        if !left && !right {
            return Ok(Self {
                originals,
                pieces: vec![base],
                transition: transition.map(|n| (n, graph.node(n).pos)),
                junction,
                at_start,
                old_left,
                new_left: false,
                old_pocket,
            });
        }
        if !length.is_finite()
            || !taper.is_finite()
            || !(60.0..=300.0).contains(&length)
            || !(12.0..=100.0).contains(&taper)
        {
            return Err("invalid_pocket_dimensions");
        }
        let incoming = if at_start {
            base.bkw_lanes
        } else {
            base.fwd_lanes
        };
        let extra = u8::from(left) + u8::from(right);
        if incoming == 0 || incoming + extra > 4 {
            return Err("pocket_lane_limit");
        }
        if left && base.median.is_none_or(|m| m.width_m < LANE_WIDTH + 0.3) {
            return Err("pocket_requires_wide_median");
        }
        if !super::lanes::LaneSystem::approach_supports_pockets(
            graph, old_pocket, junction, left, right,
        ) {
            return Err("pocket_movement_missing");
        }
        let mut pocket = base.clone();
        if at_start {
            pocket.bkw_lanes += extra;
        } else {
            pocket.fwd_lanes += extra;
        }
        pocket.width = base.width + f32::from(u8::from(right)) * LANE_WIDTH;
        let half = pocket.width * 0.5
            + if base.allowed_types & TransitFlags::FOOT != 0 {
                SIDEWALK_WIDTH
            } else {
                0.0
            }
            + if right { LANE_WIDTH * 0.5 } else { 0.0 };
        let clip = graph.endpoint_clip_for_width(old_pocket, junction, pocket.width, half);
        let span = length + taper + clip;
        let far_clip = if at_start {
            base.end_clip
        } else {
            base.start_clip
        };
        if span + far_clip + 12.0 >= total {
            return Err("pocket_road_too_short");
        }
        let split = if at_start { span } else { total - span };
        // Geometry and physical supports share the XZ alignment, including existing profile knots.
        let physical = RegionGraph::physical_profile_on_control_alignment(&base);
        let cumulative = super::lanes::geometry::build_cum_dist(&physical);
        let segment = cumulative
            .partition_point(|&s| s <= split)
            .saturating_sub(1)
            .min(physical.len() - 2);
        let fraction =
            (split - cumulative[segment]) / (cumulative[segment + 1] - cumulative[segment]);
        let point = physical[segment].lerp(physical[segment + 1], fraction);
        let cut = |points: &[Vector3]| {
            let mut a = points[..=segment].to_vec();
            if a.last().is_some_and(|p| p.distance_to(point) < 0.001) {
                *a.last_mut().unwrap() = point;
            } else {
                a.push(point);
            }
            let mut b = vec![point];
            b.extend(
                points[segment + 1..]
                    .iter()
                    .copied()
                    .filter(|p| p.distance_to(point) >= 0.001),
            );
            (a, b)
        };
        let (a, b) = cut(&base.geometry);
        let (pa, pb) = cut(&physical);
        let profile = RoadLaneTaper {
            length_m: taper,
            from_fwd: base.fwd_lanes,
            from_bkw: base.bkw_lanes,
            from_median_m: base.median.map_or(0.0, |m| m.width_m),
            left_pocket: left,
            right_pocket: right,
        };
        let mut ends = [None, None];
        ends[usize::from(at_start)] = Some(profile);
        pocket.lane_tapers = Some(Box::new(ends));
        let (mut first, mut second) = if at_start {
            (pocket, base)
        } else {
            (base, pocket)
        };
        first.geometry = a;
        first.physical_geometry = pa;
        second.geometry = b;
        second.physical_geometry = pb;
        if let Some(m) = &mut first.median {
            m.end_setback_m = 0.0;
        }
        if let Some(m) = &mut second.median {
            m.start_setback_m = 0.0;
        }
        Ok(Self {
            originals,
            pieces: vec![first, second],
            transition: Some((transition.unwrap_or(u32::MAX), point)),
            junction,
            at_start,
            old_left,
            new_left: left,
            old_pocket,
        })
    }

    /// Samples conservative two-metre corridor strips around the independently moving boundaries.
    /// The opposite kerb stays fixed for a pocket; clearance does not expand symmetrically.
    pub(crate) fn corridor_strips(&self) -> Vec<([Vector3; 2], f32)> {
        use crate::simulation::buildings::allocator::BuildingAllocator;
        let mut result = Vec::new();
        for edge in &self.pieces {
            let length: f32 = edge
                .physical_geometry
                .windows(2)
                .map(|p| p[0].distance_to(p[1]))
                .sum();
            let count = (length / 2.0).ceil().max(1.0) as usize;
            let margin = super::surface::RoadSurfaceSystem::visual_roadbed_half_width_m(edge)
                - edge.width * 0.5
                - edge.pocket_surface_shift().abs();
            for index in 0..count {
                let a = length * index as f32 / count as f32;
                let b = length * (index + 1) as f32 / count as f32;
                let section_a = edge.lane_section_at(a);
                let section_b = edge.lane_section_at(b);
                let left = section_a.left.min(section_b.left) - margin;
                let right = section_a.right.max(section_b.right) + margin;
                let shift = (left + right) * 0.5;
                let pa =
                    BuildingAllocator::sample_pos_on_polyline(&edge.physical_geometry, length, a);
                let pb =
                    BuildingAllocator::sample_pos_on_polyline(&edge.physical_geometry, length, b);
                let d = pb - pa;
                if d.length_squared() < 1e-6 {
                    continue;
                }
                let normal = godot::prelude::Vector2::new(-d.y, d.x).normalized() * shift;
                result.push((
                    [
                        Vector3::new(pa.x + normal.x, 0.0, pa.y + normal.y),
                        Vector3::new(pb.x + normal.x, 0.0, pb.y + normal.y),
                    ],
                    (right - left) * 0.5,
                ));
            }
        }
        result
    }

    /// Installs the local topology product; runtime owners are published by the caller afterwards.
    pub(crate) fn apply(mut self, graph: &mut RegionGraph) -> Vec<usize> {
        let first_id = self.originals[0].0;
        let second_id = self.originals.get(1).map_or(graph.edge_count(), |e| e.0);
        let removes = self.pieces.len() == 1;
        let node = self.transition.map(|(node, point)| {
            if removes {
                graph.remove_node_from_spatial_index(node, graph.node(node).pos);
                node
            } else if node == u32::MAX {
                graph.add_node(point, NodeType::LaneTransition)
            } else {
                graph.remove_node_from_spatial_index(node, graph.node(node).pos);
                graph.set_node_pos(node, point);
                graph.add_node_to_spatial_index(node);
                node
            }
        });
        if !removes {
            self.pieces[0].end_node = node.unwrap();
            self.pieces[1].start_node = node.unwrap();
        }
        let new_pocket = if removes || self.at_start {
            first_id
        } else {
            second_id
        };
        let endpoint = &mut graph.nodes[self.junction as usize];
        let mut previous: Vec<_> = std::mem::take(&mut endpoint.lane_connections)
            .into_iter()
            .collect();
        previous.sort_by_key(|(key, _)| *key);
        let count = if self.at_start {
            self.pieces[0].bkw_lanes
        } else {
            self.pieces.last().unwrap().fwd_lanes
        };
        for ((from, lane), mut targets) in previous {
            for (edge, _) in &mut targets {
                if *edge == self.old_pocket {
                    *edge = new_pocket;
                }
            }
            let key = if from == self.old_pocket && lane > -100 && lane < 100 {
                let rank = if lane >= 0 {
                    lane as usize
                } else {
                    (-i16::from(lane) - 1) as usize
                };
                let rank =
                    rank.saturating_sub(usize::from(self.old_left)) + usize::from(self.new_left);
                let rank = rank.min(usize::from(count.saturating_sub(1))) as i8;
                (new_pocket, if lane >= 0 { rank } else { -rank - 1 })
            } else {
                (
                    if from == self.old_pocket {
                        new_pocket
                    } else {
                        from
                    },
                    lane,
                )
            };
            let merged = endpoint.lane_connections.entry(key).or_default();
            merged.extend(targets);
            merged.sort_unstable();
            merged.dedup();
        }
        if let Some(enabled) = endpoint.crosswalk_overrides.remove(&self.old_pocket) {
            endpoint.crosswalk_overrides.insert(new_pocket, enabled);
        }
        // A split's far endpoint changes owner even when the pocket is at the stored start.
        if self.originals.len() == 1 && !removes {
            let far = self.pieces[1].end_node;
            if far != self.junction {
                let endpoint = &mut graph.nodes[far as usize];
                let previous = std::mem::take(&mut endpoint.lane_connections);
                for ((from, lane), mut targets) in previous {
                    for (edge, _) in &mut targets {
                        if *edge == first_id {
                            *edge = second_id;
                        }
                    }
                    endpoint.lane_connections.insert(
                        (if from == first_id { second_id } else { from }, lane),
                        targets,
                    );
                }
                if let Some(enabled) = endpoint.crosswalk_overrides.remove(&first_id) {
                    endpoint.crosswalk_overrides.insert(second_id, enabled);
                }
            }
        }
        if removes && self.originals.len() == 2 && self.at_start {
            let far = self.pieces[0].end_node;
            let endpoint = &mut graph.nodes[far as usize];
            let previous = std::mem::take(&mut endpoint.lane_connections);
            for ((from, lane), mut targets) in previous {
                for (edge, _) in &mut targets {
                    if *edge == second_id {
                        *edge = first_id;
                    }
                }
                endpoint.lane_connections.insert(
                    (if from == second_id { first_id } else { from }, lane),
                    targets,
                );
            }
            if let Some(enabled) = endpoint.crosswalk_overrides.remove(&second_id) {
                endpoint.crosswalk_overrides.insert(first_id, enabled);
            }
        }
        for (index, mut edge) in self.pieces.into_iter().enumerate() {
            let id = if index == 0 { first_id } else { second_id };
            let (cost, length) =
                crate::simulation::pathing::cost::CostCalculator::calculate_costs(&edge);
            edge.base_cost = cost;
            edge.physical_length = length;
            if id == graph.edge_count() {
                graph.add_edge(edge);
            } else {
                replace_edge(graph, id, edge);
            }
        }
        if removes && self.originals.len() == 2 {
            let mut retired = graph.edge(second_id).clone();
            retired.deleted = true;
            replace_edge(graph, second_id, retired);
            if let Some(node) = node {
                graph.set_node_type(node, NodeType::Junction);
            }
        }
        if removes {
            vec![first_id, second_id]
                .into_iter()
                .filter(|&id| id < graph.edge_count())
                .collect()
        } else {
            vec![first_id, second_id]
        }
    }
}

pub(super) fn replace_edge(graph: &mut RegionGraph, id: usize, edge: Edge) {
    graph.remove_from_spatial_index(id);
    let old = graph.edge(id);
    let (a, b) = (old.start_node, old.end_node);
    graph.adjacency[a as usize].retain(|&e| e != id);
    graph.adjacency[b as usize].retain(|&e| e != id);
    let (a, b) = (edge.start_node, edge.end_node);
    let live = !edge.deleted;
    graph.edges[id] = edge;
    if live {
        graph.adjacency[a as usize].push(id);
        graph.adjacency[b as usize].push(id);
        graph.add_to_spatial_index(id);
    }
}

/// Authors straight degree-two count transitions only during explicit road edits.
/// Work is bounded by the supplied local nodes and their incident edges.
pub(crate) fn author_lane_count_transitions(
    graph: &mut RegionGraph,
    nodes: &std::collections::HashSet<u32>,
    dirty: &mut std::collections::HashSet<usize>,
) {
    let mut ordered: Vec<_> = nodes.iter().copied().collect();
    ordered.sort_unstable();
    for node in ordered {
        if !matches!(
            graph.node(node).node_type,
            NodeType::Junction | NodeType::LaneTransition
        ) || graph
            .node(node)
            .crosswalk_overrides
            .values()
            .any(|&enabled| enabled)
            || graph
                .node(node)
                .lane_connections
                .keys()
                .any(|&(edge, lane)| graph.is_vehicle_turn_rule(edge, lane))
        {
            continue;
        }
        let mut ids: Vec<_> = graph
            .node_adjacency(node)
            .iter()
            .copied()
            .filter(|&id| {
                !graph.edge(id).deleted && graph.edge(id).primary_type == TransitType::Road
            })
            .collect();
        ids.sort_unstable();
        if ids.len() != 2
            || graph
                .node_adjacency(node)
                .iter()
                .filter(|&&id| !graph.edge(id).deleted)
                .count()
                != 2
        {
            continue;
        }
        if ids
            .iter()
            .any(|&id| graph.edge(id).pocket_taper().is_some())
        {
            continue;
        }
        for &id in &ids {
            let end = usize::from(graph.edge(id).end_node == node);
            let edge = graph.edge_mut(id);
            if let Some(ends) = &mut edge.lane_tapers {
                ends[end] = None;
                if ends.iter().all(Option::is_none) {
                    edge.lane_tapers = None;
                }
            }
        }
        if graph.node(node).node_type == NodeType::LaneTransition {
            graph.set_node_type(node, NodeType::Junction);
        }
        let a = graph.edge(ids[0]);
        let b = graph.edge(ids[1]);
        let outward = |edge: &crate::simulation::network::graph::Edge| {
            let p = &edge.physical_geometry;
            let mut axis = if edge.start_node == node {
                p[1] - p[0]
            } else {
                p[p.len() - 2] - p[p.len() - 1]
            };
            axis.y = 0.0;
            axis
        };
        let axis_a = outward(a);
        let axis_b = outward(b);
        if axis_a.dot(axis_b) / (axis_a.length() * axis_b.length()).max(1e-5) > -0.98 {
            continue;
        }
        let (wide_id, narrow_id) = if a.width > b.width {
            (ids[0], ids[1])
        } else {
            (ids[1], ids[0])
        };
        let wide = graph.edge(wide_id);
        let narrow = graph.edge(narrow_id);
        if wide.physical_length < 5.0
            || wide.pocket_taper().is_some()
            || wide.class != EdgeClass::Standard
            || wide.median.map(|m| m.style) != narrow.median.map(|m| m.style)
        {
            continue;
        }
        let same = wide.start_node == narrow.end_node || wide.end_node == narrow.start_node;
        let (from_fwd, from_bkw) = if same {
            (narrow.fwd_lanes, narrow.bkw_lanes)
        } else {
            (narrow.bkw_lanes, narrow.fwd_lanes)
        };
        if (from_fwd == 0 && wide.fwd_lanes != 0) || (from_bkw == 0 && wide.bkw_lanes != 0) {
            continue;
        }
        if from_fwd == wide.fwd_lanes
            && from_bkw == wide.bkw_lanes
            && narrow.median.map_or(0.0, |m| m.width_m) == wide.median.map_or(0.0, |m| m.width_m)
        {
            continue;
        }
        let end = usize::from(wide.end_node == node);
        let taper = RoadLaneTaper {
            length_m: 28.0_f32.min(wide.physical_length * 0.4),
            from_fwd,
            from_bkw,
            from_median_m: narrow.median.map_or(0.0, |m| m.width_m),
            left_pocket: false,
            right_pocket: false,
        };
        graph
            .edge_mut(wide_id)
            .lane_tapers
            .get_or_insert_with(|| Box::new([None, None]))[end] = Some(taper);
        graph.set_node_type(node, NodeType::LaneTransition);
        dirty.extend([wide_id, narrow_id]);
    }
}

/// Refuses topology strokes through an authored pocket until it is removed or resized.
/// Uses the existing edge R-tree and segment intersections; no whole-map search.
pub(crate) fn stroke_splits_pocket(graph: &RegionGraph, points: &[Vector3], level: i16) -> bool {
    if points.len() < 2 {
        return false;
    }
    let mut min = godot::prelude::Vector2::new(f32::MAX, f32::MAX);
    let mut max = godot::prelude::Vector2::new(f32::MIN, f32::MIN);
    for point in points {
        min.x = min.x.min(point.x);
        min.y = min.y.min(point.z);
        max.x = max.x.max(point.x);
        max.y = max.y.max(point.z);
    }
    let mut conflict = false;
    graph.visit_edges_near_aabb(
        Vector3::new(min.x, 0.0, min.y),
        Vector3::new(max.x, 0.0, max.y),
        |id| {
            let edge = graph.edge(id);
            if conflict || edge.deleted || edge.lane_tapers.is_none() {
                return;
            }
            // A separated interior crossing is safe, but an authored same-point endpoint
            // must not attach a third arm to a protected pocket transition on either layer.
            for point in [points[0], *points.last().unwrap()] {
                for node in [edge.start_node, edge.end_node] {
                    if graph.node(node).node_type == NodeType::LaneTransition
                        && point.distance_to(graph.node(node).pos) < 0.2
                    {
                        conflict = true;
                    }
                }
            }
            if conflict || edge.crossing_level != level {
                return;
            }
            for a in points.windows(2) {
                for b in edge.geometry.windows(2) {
                    if let Some((t, _u)) =
                        super::interaction::find_intersection_2d(a[0], a[1], b[0], b[1])
                    {
                        let pa = a[0].lerp(a[1], t);
                        if pa.distance_to(edge.geometry[0]) > 0.2
                            && pa.distance_to(*edge.geometry.last().unwrap()) > 0.2
                        {
                            conflict = true;
                            return;
                        }
                    }
                }
            }
        },
    );
    conflict
}
