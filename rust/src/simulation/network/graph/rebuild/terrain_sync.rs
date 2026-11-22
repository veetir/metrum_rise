// SPDX-License-Identifier: GPL-3.0-or-later

//! Terrain-height synchronization for standard road source geometry.

use super::super::data::RegionGraph;
use crate::simulation::network::grade::{
    GradeScratch, SourcePoint, construct_within_bounds, source_bounds, source_ground_m,
};
use crate::simulation::network::types::EdgeClass;
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::Vector3;
use std::collections::HashSet;

const TERRAIN_SYNC_SMOOTH_ITERATIONS: usize = 50;
const TERRAIN_SYNC_SMOOTH_LAMBDA: f32 = 0.5;
const TERRAIN_SYNC_SMOOTH_MU: f32 = -0.53;

impl RegionGraph {
    /// Synchronizes all road nodes and intermediate geometries to the terrain heightmap.
    ///
    /// Applies Laplacian smoothing to road grades to ensure smooth vertical transitions, then
    /// builds each re-graded control within the road grade rule inside the source band, as the
    /// stroke solve does (`ROAD-68` open question 37): where the ground leaves no profile within
    /// 1:3, the control is built at the least grade it allows, steeper than the maximum. Refusing
    /// such a terrain edit is open (`ROAD-68-K15` handoff).
    pub fn sync_to_terrain(&mut self, terrain: &TerrainSystem) {
        // 0. Pre-calculate which nodes are snappable (Standard only)
        let mut node_snappable = vec![true; self.nodes.len()];
        for edge in &self.edges {
            if edge.deleted {
                continue;
            }
            if edge.class != EdgeClass::Standard {
                node_snappable[edge.start_node as usize] = false;
                node_snappable[edge.end_node as usize] = false;
            }
        }

        // 1. Sync Nodes Only if snappable
        for (i, node) in self.nodes.iter_mut().enumerate() {
            if !node_snappable[i] {
                continue;
            }
            node.pos.y =
                terrain.sample_height_world(node.pos.x, node.pos.z) * crate::config::HEIGHT_SCALE;
        }

        // 2. Re-interpolate edge geometry and smooth grades.
        // One construction's buffers for the whole sync.
        let mut scratch = GradeScratch::default();
        let mut temporary_heights = Vec::new();
        for edge in &mut self.edges {
            if edge.deleted {
                continue;
            }
            if edge.class != EdgeClass::Standard {
                continue;
            }

            let count = edge.geometry.len();
            if count < 2 {
                continue;
            }

            // Snap endpoints to nodes
            edge.geometry[0] = self.nodes[edge.start_node as usize].pos;
            edge.geometry[count - 1] = self.nodes[edge.end_node as usize].pos;

            for point in &mut edge.geometry[1..count - 1] {
                point.y =
                    terrain.sample_height_world(point.x, point.z) * crate::config::HEIGHT_SCALE;
            }

            if count > 2 {
                temporary_heights.resize(count, 0.0);
                for _ in 0..TERRAIN_SYNC_SMOOTH_ITERATIONS {
                    for (target, window) in temporary_heights[1..count - 1]
                        .iter_mut()
                        .zip(edge.geometry.windows(3))
                    {
                        let laplacian = 0.5 * (window[0].y + window[2].y) - window[1].y;
                        *target = window[1].y + TERRAIN_SYNC_SMOOTH_LAMBDA * laplacian;
                    }
                    for (point, &height) in edge.geometry[1..count - 1]
                        .iter_mut()
                        .zip(&temporary_heights[1..count - 1])
                    {
                        point.y = height;
                    }
                    for (target, window) in temporary_heights[1..count - 1]
                        .iter_mut()
                        .zip(edge.geometry.windows(3))
                    {
                        let laplacian = 0.5 * (window[0].y + window[2].y) - window[1].y;
                        *target = window[1].y + TERRAIN_SYNC_SMOOTH_MU * laplacian;
                    }
                    for (point, &height) in edge.geometry[1..count - 1]
                        .iter_mut()
                        .zip(&temporary_heights[1..count - 1])
                    {
                        point.y = height;
                    }
                }
            }
            // The grade rule, with the two node ends held where the sync put them.
            construct_in_source_band(
                &mut edge.geometry,
                terrain,
                |index| index == 0 || index + 1 == count,
                &mut scratch,
            );
            // Terrain authoring changes both source and physical heights explicitly. Clip
            // rebuilds preserve physical profiles and no longer perform an implicit copy.
            edge.physical_geometry.clone_from(&edge.geometry);
        }
        self.rebuild_intersection_clips();
        // The copies above drop every junction's pins: each node's arms re-derive their profiles
        // from the re-graded control polylines, as an edit's solve does (`ROAD-68` invariant 6,
        // open question 11 (c)). O(nodes + edges), like the sync.
        let nodes = (0..self.nodes.len() as u32).collect();
        self.finalize_junction_endpoint_profiles(&nodes, Some(terrain), &mut HashSet::new());
    }

    /// Moves the standard roads among `candidates` (edge ids, ascending) with a local change of
    /// the source ground: every control point and every node whose roads are all standard rises
    /// by `ground_delta_m` at its XZ, the change a gameplay brush stroke made there (zero where
    /// it changed nothing), so each road keeps its cut or fill over the new ground. Points the
    /// change missed stay where they are, so a road moves only where the ground under it did;
    /// the moved profile is then built within the grade rule inside the source band with those
    /// points and the node ends held, as the editor's sync builds it. An edge with no moved
    /// point is left alone. The moved edges go into `changed_edges` and their end nodes into
    /// `changed_nodes`; their clips and the junction profiles of those nodes are re-derived, and
    /// the neighbouring arms whose physical profiles that changes join `changed_edges` too.
    /// `O(candidate points + local degree)`, independent of the city.
    pub(crate) fn follow_local_ground_change(
        &mut self,
        terrain: &TerrainSystem,
        candidates: &[usize],
        ground_delta_m: impl Fn(f32, f32) -> f32,
        changed_edges: &mut HashSet<usize>,
        changed_nodes: &mut HashSet<u32>,
    ) {
        let standard = |edge: &crate::simulation::network::graph::Edge| {
            !edge.deleted && edge.class == EdgeClass::Standard && edge.geometry.len() >= 2
        };
        // Nodes first: a node moves only when every live road at it is standard, as in the sync.
        let mut nodes: Vec<u32> = candidates
            .iter()
            .filter(|&&edge| standard(&self.edges[edge]))
            .flat_map(|&edge| [self.edges[edge].start_node, self.edges[edge].end_node])
            .collect();
        nodes.sort_unstable();
        nodes.dedup();
        for node in nodes {
            let snappable = self.adjacency[node as usize].iter().all(|&edge| {
                let edge = &self.edges[edge];
                edge.deleted || edge.class == EdgeClass::Standard
            });
            let pos = self.nodes[node as usize].pos;
            let delta = ground_delta_m(pos.x, pos.z);
            if snappable && delta != 0.0 {
                self.nodes[node as usize].pos.y = pos.y + delta;
            }
        }

        let mut scratch = GradeScratch::default();
        let mut moved = Vec::new();
        for &edge_idx in candidates {
            if !standard(&self.edges[edge_idx]) {
                continue;
            }
            let (start, end) = (
                self.edges[edge_idx].start_node,
                self.edges[edge_idx].end_node,
            );
            let (start_pos, end_pos) =
                (self.nodes[start as usize].pos, self.nodes[end as usize].pos);
            let geometry = &self.edges[edge_idx].geometry;
            let count = geometry.len();
            moved.clear();
            moved.extend(geometry.iter().enumerate().map(|(index, point)| {
                if index == 0 || index + 1 == count {
                    false
                } else {
                    ground_delta_m(point.x, point.z) != 0.0
                }
            }));
            let ends_moved = geometry[0].y.to_bits() != start_pos.y.to_bits()
                || geometry[count - 1].y.to_bits() != end_pos.y.to_bits();
            if !ends_moved && !moved.contains(&true) {
                continue;
            }
            self.remove_from_spatial_index(edge_idx);
            let edge = &mut self.edges[edge_idx];
            edge.geometry[0].y = start_pos.y;
            edge.geometry[count - 1].y = end_pos.y;
            for (point, _) in edge
                .geometry
                .iter_mut()
                .zip(&moved)
                .filter(|(_, moved)| **moved)
            {
                point.y += ground_delta_m(point.x, point.z);
            }
            construct_in_source_band(
                &mut edge.geometry,
                terrain,
                |index| !moved[index],
                &mut scratch,
            );
            // The physical profile re-derives from the moved control below, as the sync's does.
            edge.physical_geometry.clone_from(&edge.geometry);
            self.add_to_spatial_index(edge_idx);
            changed_edges.insert(edge_idx);
            changed_nodes.extend([start, end]);
        }
        if changed_nodes.is_empty() {
            return;
        }
        self.rebuild_intersection_clips_for_nodes(changed_nodes);
        self.finalize_junction_endpoint_profiles(changed_nodes, Some(terrain), changed_edges);
        for &edge_idx in changed_edges.iter() {
            let (cost, length) = crate::simulation::pathing::cost::CostCalculator::calculate_costs(
                &self.edges[edge_idx],
            );
            self.edges[edge_idx].base_cost = cost;
            self.edges[edge_idx].physical_length = length;
        }
    }
}

// Builds a road control profile within the grade rule inside the source band of `terrain`,
// holding the points `held` names at their heights and taking the others nearest their own
// (`ROAD-68` open question 37). O(points); `scratch` is reused between profiles.
fn construct_in_source_band(
    geometry: &mut [Vector3],
    terrain: &TerrainSystem,
    held: impl Fn(usize) -> bool,
    scratch: &mut GradeScratch,
) {
    let GradeScratch {
        runs,
        sources,
        bounds,
        envelope,
        heights,
        backward,
    } = scratch;
    runs.clear();
    runs.extend(
        geometry
            .windows(2)
            .map(|pair| RegionGraph::edge_profile_point_distance_m(pair[0], pair[1])),
    );
    sources.clear();
    sources.extend(
        geometry
            .iter()
            .enumerate()
            .map(|(index, point)| SourcePoint {
                ground_m: source_ground_m(terrain, point.x, point.z),
                held_m: held(index).then_some(point.y),
            }),
    );
    source_bounds(sources, runs, bounds);
    heights.clear();
    heights.extend(geometry.iter().map(|point| point.y));
    construct_within_bounds(heights, bounds, runs, envelope, backward);
    for (point, &height) in geometry.iter_mut().zip(&*heights) {
        point.y = height;
    }
}
