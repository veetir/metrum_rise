// SPDX-License-Identifier: GPL-2.0-only

//! Compile paired swept-path overlap intervals; no geometry queries run during traffic.

use super::super::{Lane, LaneSystem};
use godot::prelude::{Vector2, Vector3};

// Same swept lateral clearance as the existing connector conflict compiler.
const CLEARANCE_M: f32 = 2.2;

/// One connector's longitudinal side of a paired conflict-zone resource.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct MovementZone {
    /// Stable resource index until the next full lane rebuild.
    pub(crate) id: usize,
    /// Other connector sharing this resource.
    pub(crate) other: usize,
    /// First potentially overlapping centre position on this connector.
    pub(crate) enter: f32,
    /// Last potentially overlapping centre position on this connector.
    pub(crate) exit: f32,
    /// Corresponding beginning on the other connector.
    pub(crate) other_enter: f32,
    /// Corresponding end on the other connector.
    pub(crate) other_exit: f32,
    // Segment-component vertex bounds survive terrain lifting without geometry recompilation.
    vertices: [usize; 4],
}

/// Junction-owned paired zone; both directed sides share one reservation resource.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ConflictZone {
    /// First connector, ordered by lane ID.
    pub(crate) a: usize,
    /// Other connector.
    pub(crate) b: usize,
    /// First connector's view of the resource.
    pub(crate) movement: MovementZone,
}

fn flat(p: Vector3) -> Vector2 {
    Vector2::new(p.x, p.z)
}

// A connected component of intersecting sampled segment pairs is one physical overlap.
// Adjacent segment cells are conservatively joined; neither a curve nor a long segment
// can leave an untested gap. O(P*Q) time/storage for a connector pair, at rebuild only.
fn pair_vertices(a: &Lane, b: &Lane) -> Vec<[usize; 4]> {
    let rows = a.geometry.len().saturating_sub(1);
    let cols = b.geometry.len().saturating_sub(1);
    let mut cells = vec![false; rows * cols];
    for i in 0..rows {
        for j in 0..cols {
            cells[i * cols + j] = super::super::geometry::segment_distance_squared_xz(
                flat(a.geometry[i]),
                flat(a.geometry[i + 1]),
                flat(b.geometry[j]),
                flat(b.geometry[j + 1]),
            ) < CLEARANCE_M * CLEARANCE_M;
        }
    }
    let mut stack = Vec::new();
    let mut intervals = Vec::new();
    for seed in 0..cells.len() {
        if !cells[seed] {
            continue;
        }
        cells[seed] = false;
        stack.push(seed);
        let mut range = [usize::MAX, 0, usize::MAX, 0];
        while let Some(cell) = stack.pop() {
            let (i, j) = (cell / cols, cell % cols);
            range[0] = range[0].min(i);
            range[1] = range[1].max(i + 1);
            range[2] = range[2].min(j);
            range[3] = range[3].max(j + 1);
            for ni in i.saturating_sub(1)..=(i + 1).min(rows - 1) {
                for nj in j.saturating_sub(1)..=(j + 1).min(cols - 1) {
                    let next = ni * cols + nj;
                    if cells[next] {
                        cells[next] = false;
                        stack.push(next);
                    }
                }
            }
        }
        intervals.push(range);
    }
    intervals
}

impl LaneSystem {
    pub(super) fn rebuild_node_zones(&mut self, node: usize) {
        let mut ids = self.node_lanes.get(&node).cloned().unwrap_or_default();
        ids.retain(|&id| self.controlled_movement(id).is_some());
        ids.sort_unstable();
        let mut zones = Vec::new();
        for (index, &a) in ids.iter().enumerate() {
            for &b in &ids[index + 1..] {
                for vertices in pair_vertices(&self.lanes[a], &self.lanes[b]) {
                    let movement = MovementZone {
                        id: self.conflict_zone_count(),
                        other: b,
                        enter: self.lanes[a].cum_dist[vertices[0]],
                        exit: self.lanes[a].cum_dist[vertices[1]],
                        other_enter: self.lanes[b].cum_dist[vertices[2]],
                        other_exit: self.lanes[b].cum_dist[vertices[3]],
                        vertices,
                    };
                    self.conflict_zone_count += 1;
                    zones.push(ConflictZone { a, b, movement });
                    self.controlled[a].as_mut().unwrap().zones.push(movement);
                    self.controlled[b]
                        .as_mut()
                        .unwrap()
                        .zones
                        .push(MovementZone {
                            other: a,
                            enter: movement.other_enter,
                            exit: movement.other_exit,
                            other_enter: movement.enter,
                            other_exit: movement.exit,
                            vertices: [vertices[2], vertices[3], vertices[0], vertices[1]],
                            ..movement
                        });
                }
            }
        }
        for id in ids {
            self.controlled[id].as_mut().unwrap().zones.sort_by(|a, b| {
                a.enter
                    .total_cmp(&b.enter)
                    .then(a.exit.total_cmp(&b.exit))
                    .then(a.id.cmp(&b.id))
            });
        }
        self.node_conflict_zones.insert(node, zones);
        debug_assert!(
            self.junction_zones(node)
                .iter()
                .all(|z| z.movement.id < self.conflict_zone_count())
        );
    }

    /// Refreshes only cached stations after the owning junction's lane heights change.
    pub(crate) fn refresh_zone_stations(&mut self, node: usize) {
        fn refresh(zone: &mut MovementZone, own: &Lane, other: &Lane) {
            zone.enter = own.cum_dist[zone.vertices[0]];
            zone.exit = own.cum_dist[zone.vertices[1]];
            zone.other_enter = other.cum_dist[zone.vertices[2]];
            zone.other_exit = other.cum_dist[zone.vertices[3]];
        }
        if let Some(ids) = self.node_lanes.get(&node) {
            for &id in ids {
                if let Some(control) = self.controlled.get_mut(id).and_then(Option::as_mut) {
                    for zone in &mut control.zones {
                        refresh(zone, &self.lanes[id], &self.lanes[zone.other]);
                    }
                }
            }
        }
        if let Some(zones) = self.node_conflict_zones.get_mut(&node) {
            for zone in zones {
                refresh(&mut zone.movement, &self.lanes[zone.a], &self.lanes[zone.b]);
            }
        }
    }

    /// Reservation scratch upper bound, including retired incremental lane products.
    pub(crate) fn conflict_zone_count(&self) -> usize {
        self.conflict_zone_count
    }

    /// Compiled paired zones owned by this junction, in stable connector-pair order.
    pub(crate) fn junction_zones(&self, node: usize) -> &[ConflictZone] {
        self.node_conflict_zones
            .get(&node)
            .map_or(&[], Vec::as_slice)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::lanes::geometry::build_cum_dist;

    fn lane(points: &[(f32, f32)]) -> Lane {
        let geometry: Vec<_> = points
            .iter()
            .map(|&(x, z)| Vector3::new(x, 0.0, z))
            .collect();
        let cum_dist = build_cum_dist(&geometry);
        Lane {
            length: *cum_dist.last().unwrap(),
            geometry,
            cum_dist,
            ..Default::default()
        }
    }
    #[test]
    fn overlap_intervals_are_local_reciprocal_and_keep_disconnected_crossings() {
        let a = lane(&[
            (-10., 0.),
            (-5., 0.),
            (-3., 0.),
            (-2., 0.),
            (0., 0.),
            (2., 0.),
            (3., 0.),
            (5., 0.),
            (10., 0.),
        ]);
        let b = lane(&[
            (0., -10.),
            (0., -5.),
            (0., -3.),
            (0., -2.),
            (0., 0.),
            (0., 2.),
            (0., 3.),
            (0., 5.),
            (0., 10.),
        ]);
        let zones = pair_vertices(&a, &b);
        assert_eq!(zones.len(), 1);
        assert!(zones[0][0] > 0 && zones[0][1] < a.geometry.len() - 1);
        let reverse = pair_vertices(&b, &a);
        assert_eq!(
            zones[0],
            [reverse[0][2], reverse[0][3], reverse[0][0], reverse[0][1]]
        );
        let remote = lane(&[(-10., 20.), (10., 20.)]);
        assert!(pair_vertices(&a, &remote).is_empty());
        let zig = lane(&[
            (-8., -5.),
            (-8., 0.),
            (-8., 5.),
            (8., 5.),
            (8., 0.),
            (8., -5.),
        ]);
        assert_eq!(pair_vertices(&a, &zig).len(), 2);
    }
}
