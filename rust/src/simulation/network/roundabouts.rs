// SPDX-License-Identifier: GPL-2.0-only

//! Bounded single-lane roundabout layout, shared by preview and transactional authoring.

use super::graph::{Edge, RegionGraph};
use super::types::{EdgeClass, NodeType, TransitFlags, TransitType};
use godot::prelude::Vector3;

/// Diameter is measured between the outside asphalt kerbs, excluding sidewalks.
#[derive(Clone)]
pub(crate) struct RoundaboutRequest {
    pub(crate) center: Vector3,
    pub(crate) diameter: f32,
    /// Outward arm angles in radians about +Y, measured from +X toward +Z.
    pub(crate) angles: Vec<f32>,
    pub(crate) replace: Option<u32>,
}

/// Only incident arms and newly generated geometry are retained, never a city snapshot.
pub(crate) struct RoundaboutLayout {
    pub(crate) originals: Vec<(usize, Edge)>,
    pub(crate) points: Vec<Vector3>,
    pub(crate) arms: Vec<Edge>,
    ring: Vec<Edge>,
    old_node: Option<u32>,
}

impl RoundaboutLayout {
    /// O(P + A log A), where P is selected arm samples and 3 <= A <= 6.
    pub(crate) fn prepare(
        graph: &RegionGraph,
        request: RoundaboutRequest,
    ) -> Result<Self, &'static str> {
        let RoundaboutRequest {
            mut center,
            diameter,
            angles,
            replace,
        } = request;
        if !center.is_finite() || !diameter.is_finite() || !(12.0..=120.0).contains(&diameter) {
            return Err("invalid_roundabout_dimensions");
        }
        // Keep the removed central footprint inside the existing 30 m lane-reattachment
        // reach. Larger empty-ground presets do not displace live road occupants.
        if replace.is_some() && diameter > 60.0 {
            return Err("roundabout_replacement_too_large");
        }
        let radius = (diameter - crate::config::LANE_WIDTH) * 0.5;
        if replace.is_none() && !(3..=6).contains(&angles.len()) {
            return Err("roundabout_requires_three_to_six_arms");
        }
        let mut entries = Vec::new();
        if let Some(node) = replace {
            if node as usize >= graph.node_count() || graph.get_valid_node(node) != node {
                return Err("invalid_roundabout_junction");
            }
            if !(3..=6).contains(&graph.live_node_connection_count(node)) {
                return Err("roundabout_requires_three_to_six_arms");
            }
            center = graph.node(node).pos;
            if super::lanes::vehicle_junctions::node_whitelists_vehicle_turns(graph, node as usize)
            {
                return Err("roundabout_has_lane_restrictions");
            }
            let mut ids = graph.node_adjacency(node).to_vec();
            ids.sort_unstable();
            for id in ids {
                let edge = graph.edge(id);
                if edge.deleted {
                    continue;
                }
                if edge.roundabout
                    || edge.class != EdgeClass::Standard
                    || edge.primary_type != TransitType::Road
                    || edge.fwd_lanes > 2
                    || edge.bkw_lanes > 2
                    || edge.fwd_lanes + edge.bkw_lanes == 0
                    || edge.lane_tapers.is_some()
                    || edge.start_node == edge.end_node
                    || edge.crossing_level != 0
                {
                    return Err("roundabout_requires_ordinary_surface_arms");
                }
                let at_start = edge.start_node == node;
                let mut points = RegionGraph::physical_profile_on_control_alignment(edge);
                if !at_start {
                    points.reverse();
                }
                let (cut, mut point) =
                    circle_cut(&points, center, radius).ok_or("roundabout_arm_too_short")?;
                // A level ring retains the existing junction height. The normal finalizer owns the arm profile.
                point.y = center.y;
                let mut trimmed = vec![point];
                trimmed.extend_from_slice(&points[cut + 1..]);
                // A short retained approach is certified by the same full span/node
                // compiler as any other edit; 30 m was a preset-stub restriction.
                if trimmed.len() < 2 || graph.calculate_length(&trimmed) <= f32::EPSILON {
                    return Err("roundabout_arm_too_short");
                }
                let angle = (point.z - center.z)
                    .atan2(point.x - center.x)
                    .rem_euclid(std::f32::consts::TAU);
                let mut arm = edge.clone();
                if !at_start {
                    trimmed.reverse();
                }
                arm.geometry = trimmed.clone();
                arm.physical_geometry = trimmed;
                arm.physical_length = graph.calculate_length(&arm.physical_geometry);
                entries.push((angle, point, arm, Some((id, edge.clone()))));
            }
        } else {
            for angle in angles {
                if !angle.is_finite() {
                    return Err("invalid_roundabout_angles");
                }
                let angle = angle.rem_euclid(std::f32::consts::TAU);
                let direction = Vector3::new(angle.cos(), 0.0, angle.sin());
                let point = center + direction * radius;
                entries.push((
                    angle,
                    point,
                    road(vec![point, center + direction * (radius + 60.0)], 1, 1),
                    None,
                ));
            }
        }
        if !(3..=6).contains(&entries.len()) {
            return Err("roundabout_requires_three_to_six_arms");
        }
        entries.sort_by(|a, b| a.0.total_cmp(&b.0));
        // Leave room for two ordinary junction mouths and a supported circulating lane.
        // Small diameters remain valid inputs, but crowded layouts are refused explicitly.
        for i in 0..entries.len() {
            let gap = (entries[(i + 1) % entries.len()].0 - entries[i].0)
                .rem_euclid(std::f32::consts::TAU);
            if gap * radius < 12.0 {
                return Err("roundabout_arms_too_close");
            }
        }
        let originals = entries
            .iter_mut()
            .filter_map(|entry| entry.3.take())
            .collect();
        let mut ring = Vec::new();
        for i in 0..entries.len() {
            let start = entries[i].0;
            let previous = (i + entries.len() - 1) % entries.len();
            let sweep = (start - entries[previous].0).rem_euclid(std::f32::consts::TAU);
            // Decreasing XZ angle keeps the central island on the driver's left (RHT).
            let count = ((sweep * radius / 1.0).ceil() as usize).max(2);
            // Circumscribed supports intersect consecutive circle tangents. Unlike sampled
            // chords, the first/last segments share exactly the same tangent at each arm;
            // this avoids manufacturing a tiny extra corner inside the T-junction kerbs.
            let delta = sweep / count as f32;
            let support_radius = radius / (delta * 0.5).cos();
            let mut points = Vec::with_capacity(count + 2);
            points.push(entries[i].1);
            points.extend((0..count).map(|step| {
                let angle = start - delta * (step as f32 + 0.5);
                center + Vector3::new(angle.cos(), 0.0, angle.sin()) * support_radius
            }));
            points.push(entries[previous].1);
            let mut edge = road(points, 1, 0);
            edge.roundabout = true;
            edge.no_building_spawn = true;
            ring.push(edge);
        }
        let layout = Self {
            originals,
            points: entries.iter().map(|e| e.1).collect(),
            arms: entries.into_iter().map(|e| e.2).collect(),
            ring,
            old_node: replace,
        };
        layout.validate_road_clearance(graph)?;
        Ok(layout)
    }

    // The existing R-tree bounds discovery. Only new corridors need checking; unchanged
    // outer arm sections may already meet other junctions. Circle/arm mouths are intentional.
    fn validate_road_clearance(&self, graph: &RegionGraph) -> Result<(), &'static str> {
        use super::lanes::geometry::segment_distance_squared_xz;
        use super::surface::RoadSurfaceSystem;
        use godot::prelude::Vector2;
        let flat = |p: Vector3| Vector2::new(p.x, p.z);
        for edge in self
            .ring
            .iter()
            .chain(self.arms.iter().filter(|_| self.old_node.is_none()))
        {
            let half = RoadSurfaceSystem::visual_roadbed_half_width_m(edge);
            let mut min = Vector3::new(f32::INFINITY, 0.0, f32::INFINITY);
            let mut max = Vector3::new(f32::NEG_INFINITY, 0.0, f32::NEG_INFINITY);
            for p in &edge.physical_geometry {
                min.x = min.x.min(p.x);
                min.z = min.z.min(p.z);
                max.x = max.x.max(p.x);
                max.z = max.z.max(p.z);
            }
            let halo = Vector3::new(half + 24.0, 0.0, half + 24.0);
            for id in graph.get_edges_near_aabb(min - halo, max + halo) {
                let other = graph.edge(id);
                if other.deleted || self.originals.iter().any(|(original, _)| *original == id) {
                    continue;
                }
                let gap = half + RoadSurfaceSystem::visual_roadbed_half_width_m(other);
                if edge.physical_geometry.windows(2).any(|a| {
                    other.physical_geometry.windows(2).any(|b| {
                        segment_distance_squared_xz(flat(a[0]), flat(a[1]), flat(b[0]), flat(b[1]))
                            < gap * gap
                    })
                }) {
                    return Err("roundabout_overlaps_road");
                }
            }
        }
        Ok(())
    }

    /// New corridors used by the existing water/parcel/field guards.
    pub(crate) fn corridors(&self) -> impl Iterator<Item = &Edge> {
        self.ring.iter().chain(&self.arms)
    }

    /// Installs graph/index state together; caller owns lane, frontage, terrain and undo publication.
    pub(crate) fn apply(mut self, graph: &mut RegionGraph) -> Vec<usize> {
        let nodes: Vec<_> = self
            .points
            .iter()
            .map(|&p| graph.add_node(p, NodeType::Junction))
            .collect();
        let mut changed = Vec::new();
        for (i, mut arm) in self.arms.drain(..).enumerate() {
            if let Some(old) = self.old_node {
                if arm.start_node == old {
                    arm.start_node = nodes[i];
                } else {
                    arm.end_node = nodes[i];
                }
                let id = self.originals[i].0;
                super::pockets::replace_edge(graph, id, arm);
                graph.nodes[nodes[i] as usize]
                    .crosswalk_overrides
                    .insert(id, true);
                changed.push(id);
            } else {
                arm.start_node = nodes[i];
                arm.end_node = graph.add_node(*arm.geometry.last().unwrap(), NodeType::Junction);
                let id = graph.add_edge(arm);
                graph.nodes[nodes[i] as usize]
                    .crosswalk_overrides
                    .insert(id, true);
                changed.push(id);
            }
        }
        for (i, mut edge) in self.ring.drain(..).enumerate() {
            edge.start_node = nodes[i];
            edge.end_node = nodes[(i + nodes.len() - 1) % nodes.len()];
            let id = graph.add_edge(edge);
            for node in [nodes[i], nodes[(i + nodes.len() - 1) % nodes.len()]] {
                graph.nodes[node as usize]
                    .crosswalk_overrides
                    .insert(id, false);
            }
            changed.push(id);
        }
        if let Some(old) = self.old_node {
            graph.remove_node_from_spatial_index(old, graph.node(old).pos);
        }
        changed
    }
}

fn road(points: Vec<Vector3>, fwd: u8, bkw: u8) -> Edge {
    Edge {
        primary_type: TransitType::Road,
        allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
        width: f32::from(fwd + bkw) * crate::config::LANE_WIDTH,
        fwd_lanes: fwd,
        bkw_lanes: bkw,
        speed_limit: 20.0 / 3.6,
        physical_length: RegionGraph::new().calculate_length(&points),
        geometry: points.clone(),
        physical_geometry: points,
        ..Default::default()
    }
}

fn circle_cut(points: &[Vector3], center: Vector3, radius: f32) -> Option<(usize, Vector3)> {
    for (i, pair) in points.windows(2).enumerate() {
        let a = godot::prelude::Vector2::new(pair[0].x - center.x, pair[0].z - center.z);
        let b = godot::prelude::Vector2::new(pair[1].x - center.x, pair[1].z - center.z);
        if b.length_squared() < radius * radius {
            continue;
        }
        let d = b - a;
        let det = a.dot(d).powi(2) - d.length_squared() * (a.length_squared() - radius * radius);
        if det < 0.0 || d.length_squared() < 1e-8 {
            continue;
        }
        let t = (-a.dot(d) + det.sqrt()) / d.length_squared();
        if (0.0..=1.0).contains(&t) {
            return Some((i, pair[0].lerp(pair[1], t)));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::lanes::control::{ApproachControl, approach_control};
    use crate::simulation::network::lanes::{LaneSystem, LaneType};

    pub(super) fn request(arms: usize, diameter: f32) -> RoundaboutRequest {
        RoundaboutRequest {
            center: Vector3::ZERO,
            diameter,
            replace: None,
            angles: (0..arms)
                .map(|i| i as f32 * std::f32::consts::TAU / arms as f32)
                .collect(),
        }
    }

    #[test]
    fn roundabout_ring_is_counterclockwise_and_entries_yield_to_it() {
        for count in 3..=6 {
            let mut graph = RegionGraph::new();
            let layout = RoundaboutLayout::prepare(&graph, request(count, 60.0)).unwrap();
            layout.apply(&mut graph);
            graph.rebuild_intersection_clips();
            let mut lanes = LaneSystem::new();
            lanes.rebuild(&mut graph);
            for edge in graph.edges().iter().filter(|e| e.roundabout) {
                let a = edge.geometry[0];
                let b = edge.geometry[1];
                assert!(a.x * b.z - a.z * b.x < 0.0);
                assert_eq!(
                    approach_control(&graph, edge.end_node, count),
                    ApproachControl::Uncontrolled
                );
            }
            for id in 0..count {
                graph.edge_mut(id).speed_limit = 80.0 / 3.6;
                assert_eq!(
                    approach_control(&graph, graph.edge(id).start_node, id),
                    ApproachControl::Yield
                );
                let node = graph.edge(id).start_node;
                let controlled: Vec<_> = lanes.node_lanes[&(node as usize)]
                    .iter()
                    .filter_map(|&id| lanes.controlled_movement(id))
                    .collect();
                let enter = controlled
                    .iter()
                    .find(|c| lanes.lanes[c.incoming].edge_id == id)
                    .unwrap();
                assert!(!enter.yields_to.is_empty());
                for movement in controlled
                    .iter()
                    .filter(|c| graph.edge(lanes.lanes[c.incoming].edge_id).roundabout)
                {
                    assert!(movement.yields_to.iter().all(|&other| {
                        graph
                            .edge(
                                lanes.lanes[lanes.controlled_movement(other).unwrap().incoming]
                                    .edge_id,
                            )
                            .roundabout
                    }));
                }
                assert!(
                    lanes.edge_lanes[&id]
                        .iter()
                        .any(|&lane| lanes.lanes[lane].lane_type == LaneType::Vehicle)
                );
            }
        }
    }

    #[test]
    fn replacement_preserves_one_way_entry_and_exit_arms() {
        for reverse in [false, true] {
            let mut graph = RegionGraph::new();
            let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
            for (i, (fwd, bkw)) in [(1, 0), (0, 1), (1, 1)].into_iter().enumerate() {
                let angle = i as f32 * std::f32::consts::TAU / 3.0;
                let end = Vector3::new(angle.cos(), 0.0, angle.sin()) * 120.0;
                let node = graph.add_node(end, NodeType::Junction);
                let mut edge = road(vec![Vector3::ZERO, end], fwd, bkw);
                edge.start_node = center;
                edge.end_node = node;
                if reverse {
                    std::mem::swap(&mut edge.start_node, &mut edge.end_node);
                    edge.geometry.reverse();
                    edge.physical_geometry.reverse();
                }
                graph.add_edge(edge);
            }
            let mut replacement = request(3, 40.0);
            replacement.replace = Some(center);
            let layout = RoundaboutLayout::prepare(&graph, replacement).unwrap();
            layout.apply(&mut graph);
            graph.rebuild_intersection_clips();
            let mut lanes = LaneSystem::new();
            lanes.rebuild(&mut graph);
            for (id, counts) in [(1, 0), (0, 1), (1, 1)].into_iter().enumerate() {
                let edge = graph.edge(id);
                assert_eq!((edge.fwd_lanes, edge.bkw_lanes), counts);
                let vehicle = lanes.edge_lanes[&id]
                    .iter()
                    .filter(|&&id| lanes.lanes[id].lane_type == LaneType::Vehicle)
                    .count();
                assert_eq!(vehicle, usize::from(counts.0 + counts.1));
            }
        }
    }

    #[test]
    fn roundabout_heavy_uneven_arrivals_drain_with_identical_worker_results() {
        fn run(workers: usize) -> Vec<(usize, u32, u32)> {
            rayon::ThreadPoolBuilder::new()
                .num_threads(workers)
                .build()
                .unwrap()
                .install(|| {
                    use crate::simulation::economy::agents::AgentSystem;
                    use crate::simulation::network::TransitNetwork;
                    let mut graph = RegionGraph::new();
                    let mut request = request(6, 70.0);
                    request.angles = vec![0.0, 0.9, 2.0, 3.2, 4.2, 5.3];
                    RoundaboutLayout::prepare(&graph, request)
                        .unwrap()
                        .apply(&mut graph);
                    graph.rebuild_intersection_clips();
                    let mut network = TransitNetwork::new();
                    network.lane_system.rebuild(&mut graph);
                    let mut agents = AgentSystem::new();
                    let mut targets = Vec::new();
                    for from in 0..6 {
                        let to = (from + 1) % 6;
                        let lane = *network.lane_system.edge_lanes[&from]
                            .iter()
                            .find(|&&id| {
                                let lane = &network.lane_system.lanes[id];
                                lane.lane_type == LaneType::Vehicle && !lane.is_fwd
                            })
                            .unwrap();
                        for queue in 0..6 {
                            let mut path = vec![(6 + from) as u32, from as u32];
                            let mut at = from;
                            while at != to {
                                at = (at + 5) % 6;
                                path.push(at as u32);
                            }
                            path.push((6 + to) as u32);
                            let id = agents.spawn_debug_car(
                                path,
                                &network.lane_system,
                                lane,
                                godot::prelude::Vector2::ZERO,
                            );
                            agents.current_node[id] = (6 + from) as u32;
                            agents.current_path_index[id] = 1;
                            agents.lane_distance[id] =
                                (network.lane_system.lanes[lane].length - 2.0 - queue as f32 * 8.0)
                                    .max(0.0);
                            targets.push(to);
                        }
                    }
                    let allocator =
                        crate::simulation::buildings::allocator::BuildingAllocator::new();
                    let time = crate::simulation::core::time::TimeSystem::new();
                    let mut exited = vec![false; targets.len()];
                    for _ in 0..2400 {
                        agents.tick(&allocator, &mut network, &mut graph, 0.1, &time);
                        for (id, &target) in targets.iter().enumerate() {
                            exited[id] |= agents.current_edge[id] == target
                                && agents.current_path_index[id] >= 7;
                            assert!(agents.speed[id].is_finite());
                        }
                        if exited.iter().all(|&exited| exited) {
                            break;
                        }
                    }
                    assert!(
                        exited.iter().all(|&exited| exited),
                        "roundabout queues must drain: {exited:?}"
                    );
                    (0..targets.len())
                        .map(|id| {
                            (
                                agents.current_edge[id],
                                agents.lane_distance[id].to_bits(),
                                agents.speed[id].to_bits(),
                            )
                        })
                        .collect()
                })
        }
        assert_eq!(run(1), run(4));
    }
}
