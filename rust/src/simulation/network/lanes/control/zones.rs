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
    // Fractional segment stations survive terrain lifting without geometry recompilation.
    stations: [f32; 4],
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

// Clip a segment against the other segment's swept capsule. A capsule is convex,
// so the union of its two endpoint discs and projected strip is one interval.
// Keeping fractional stations avoids reserving an entire long straight sample.
fn capsule_interval(a: Vector2, b: Vector2, c: Vector2, d: Vector2) -> Option<(f32, f32)> {
    fn clip(value: f32, step: f32, lo: f32, hi: f32, range: &mut (f32, f32)) -> bool {
        if step == 0.0 {
            return value >= lo && value <= hi;
        }
        let x = (lo - value) / step;
        let y = (hi - value) / step;
        range.0 = range.0.max(x.min(y));
        range.1 = range.1.min(x.max(y));
        range.0 <= range.1
    }
    let v = b - a;
    let w = d - c;
    let vv = v.length_squared();
    let mut result = (f32::INFINITY, f32::NEG_INFINITY);
    let mut include = |lo: f32, hi: f32| {
        if lo <= hi {
            result.0 = result.0.min(lo.max(0.0));
            result.1 = result.1.max(hi.min(1.0));
        }
    };
    for end in [c, d] {
        let q = a - end;
        let dot = q.dot(v);
        let offset = q.length_squared() - CLEARANCE_M * CLEARANCE_M;
        if vv == 0.0 {
            if offset <= 0.0 {
                include(0.0, 1.0);
            }
        } else {
            let discriminant = dot * dot - vv * offset;
            if discriminant >= 0.0 {
                let root = discriminant.sqrt();
                include(((-dot - root) / vv).max(0.0), ((-dot + root) / vv).min(1.0));
            }
        }
    }
    let ww = w.length_squared();
    if ww > 0.0 {
        let q = a - c;
        let cross = |x: Vector2, y: Vector2| x.x * y.y - x.y * y.x;
        let radius = CLEARANCE_M * ww.sqrt();
        let mut strip = (0.0, 1.0);
        if clip(q.dot(w), v.dot(w), 0.0, ww, &mut strip)
            && clip(cross(q, w), cross(v, w), -radius, radius, &mut strip)
        {
            include(strip.0, strip.1);
        }
    }
    (result.0 <= result.1).then_some((result.0.next_down().max(0.0), result.1.next_up().min(1.0)))
}

// Connected components of overlapping sampled segment pairs are physical zones.
// O(P*Q) time/storage for a pair, exclusively at junction compilation.
fn pair_vertices(a: &Lane, b: &Lane) -> Vec<[f32; 4]> {
    let rows = a.geometry.len().saturating_sub(1);
    let cols = b.geometry.len().saturating_sub(1);
    let mut cells = vec![None; rows * cols];
    for i in 0..rows {
        for j in 0..cols {
            let (a0, a1) = (flat(a.geometry[i]), flat(a.geometry[i + 1]));
            let (b0, b1) = (flat(b.geometry[j]), flat(b.geometry[j + 1]));
            if super::super::geometry::segment_distance_squared_xz(a0, a1, b0, b1)
                < CLEARANCE_M * CLEARANCE_M
            {
                let (lo, hi) = capsule_interval(a0, a1, b0, b1).unwrap_or((0.0, 1.0));
                let (other_lo, other_hi) = capsule_interval(b0, b1, a0, a1).unwrap_or((0.0, 1.0));
                cells[i * cols + j] = Some([
                    i as f32 + lo,
                    i as f32 + hi,
                    j as f32 + other_lo,
                    j as f32 + other_hi,
                ]);
            }
        }
    }
    let mut stack = Vec::new();
    let mut intervals = Vec::new();
    for seed in 0..cells.len() {
        let Some(bounds) = cells[seed].take() else {
            continue;
        };
        stack.push((seed, bounds));
        let mut range = [f32::INFINITY, 0.0, f32::INFINITY, 0.0];
        while let Some((cell, bounds)) = stack.pop() {
            let (i, j) = (cell / cols, cell % cols);
            range[0] = range[0].min(bounds[0]);
            range[1] = range[1].max(bounds[1]);
            range[2] = range[2].min(bounds[2]);
            range[3] = range[3].max(bounds[3]);
            for ni in i.saturating_sub(1)..=(i + 1).min(rows - 1) {
                for nj in j.saturating_sub(1)..=(j + 1).min(cols - 1) {
                    let next = ni * cols + nj;
                    if let Some(bounds) = cells[next].take() {
                        stack.push((next, bounds));
                    }
                }
            }
        }
        intervals.push(range);
    }
    intervals
}

fn station(lane: &Lane, sample: f32) -> f32 {
    let i = sample.floor() as usize;
    let j = (i + 1).min(lane.cum_dist.len() - 1);
    lane.cum_dist[i] + (lane.cum_dist[j] - lane.cum_dist[i]) * sample.fract()
}

impl LaneSystem {
    /// Compiles stable paired resources for this junction only.
    pub(super) fn rebuild_node_zones(&mut self, node: usize) {
        let mut ids = self.node_lanes.get(&node).cloned().unwrap_or_default();
        ids.retain(|&id| self.controlled_movement(id).is_some());
        ids.sort_unstable();
        let mut zones = Vec::new();
        for (index, &a) in ids.iter().enumerate() {
            for &b in &ids[index + 1..] {
                for stations in pair_vertices(&self.lanes[a], &self.lanes[b]) {
                    let movement = MovementZone {
                        id: self.conflict_zone_count(),
                        other: b,
                        enter: station(&self.lanes[a], stations[0]),
                        exit: station(&self.lanes[a], stations[1]),
                        other_enter: station(&self.lanes[b], stations[2]),
                        other_exit: station(&self.lanes[b], stations[3]),
                        stations,
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
                            stations: [stations[2], stations[3], stations[0], stations[1]],
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
            zone.enter = station(own, zone.stations[0]);
            zone.exit = station(own, zone.stations[1]);
            zone.other_enter = station(other, zone.stations[2]);
            zone.other_exit = station(other, zone.stations[3]);
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
        assert!(zones[0][0] > 0.0 && zones[0][1] < (a.geometry.len() - 1) as f32);
        let reverse = pair_vertices(&b, &a);
        assert_eq!(
            zones[0],
            [reverse[0][2], reverse[0][3], reverse[0][0], reverse[0][1]]
        );
        // A single long segment must yield the same local interval as a sampled line.
        let long = lane(&[(-10.0, 0.0), (10.0, 0.0)]);
        let cross = lane(&[(0.0, -10.0), (0.0, 10.0)]);
        let interval = pair_vertices(&long, &cross)[0];
        assert!((station(&long, interval[0]) - (10.0 - CLEARANCE_M)).abs() < 0.0001);
        assert!((station(&long, interval[1]) - (10.0 + CLEARANCE_M)).abs() < 0.0001);
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
