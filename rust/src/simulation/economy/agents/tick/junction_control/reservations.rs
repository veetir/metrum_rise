// SPDX-License-Identifier: GPL-2.0-only

//! Ordered conflict-zone claims, acyclic local waits and reserved downstream storage.

use super::super::super::data::AgentVec;
use crate::config::{CAR_JUNCTION_SPEED_MS, CAR_LENGTH, IDM_S_MIN};
use crate::simulation::network::TransitNetwork;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

const FREE: usize = usize::MAX;
const SPACE: f32 = CAR_LENGTH + IDM_S_MIN;

// Longitudinal reference stations use the existing front-to-front IDM convention.
// An occupied boundary needs a following gap; an empty road ends at its stop line.
fn downstream_slots(length: f32, first: Option<f32>) -> usize {
    let boundary = first.map_or(length, |d| (d - SPACE).min(length));
    if boundary < CAR_LENGTH {
        0
    } else {
        1 + ((boundary - CAR_LENGTH) / SPACE).floor() as usize
    }
}

/// Retained local reservation scratch; no persisted state or unstable agent-ID ownership.
#[derive(Default)]
pub(super) struct Reservations {
    zones: Vec<AtomicUsize>,
    exits: Vec<AtomicUsize>,
    capacity: Vec<usize>,
    touched_exits: Vec<usize>,
    active: Vec<AtomicBool>,
    frontier: Vec<AtomicU32>,
    visited: Vec<AtomicUsize>,
    stack: Vec<AtomicUsize>,
    visit_epoch: AtomicUsize,
    step_distance: f32,
}

impl Reservations {
    /// Reconstructs physical zone occupancy and books the current step in stable lane order.
    pub(super) fn prepare(
        &mut self,
        agents: &AgentVec,
        buckets: &[Vec<(f32, usize)>],
        nodes: &[usize],
        network: &TransitNetwork,
        delta: f32,
    ) {
        let lanes = &network.lane_system;
        self.step_distance = CAR_JUNCTION_SPEED_MS * delta;
        self.zones
            .resize_with(lanes.conflict_zone_count().max(self.zones.len()), || {
                AtomicUsize::new(FREE)
            });
        let count = lanes.lanes.len().max(self.exits.len());
        self.exits.resize_with(count, || AtomicUsize::new(0));
        self.capacity.resize(count, FREE);
        self.active.resize_with(count, || AtomicBool::new(false));
        self.frontier
            .resize_with(count, || AtomicU32::new(f32::MAX.to_bits()));
        self.visited.resize_with(count, || AtomicUsize::new(0));
        self.stack.resize_with(count, || AtomicUsize::new(0));
        for &out in &self.touched_exits {
            self.capacity[out] = FREE;
            self.exits[out].store(0, Ordering::Relaxed);
        }
        self.touched_exits.clear();
        self.touched_exits.reserve(count);
        for &node in nodes {
            for zone in lanes.junction_zones(node) {
                self.zones[zone.movement.id].store(FREE, Ordering::Relaxed);
            }
            let Some(ids) = lanes.node_lanes.get(&node) else {
                continue;
            };
            for &id in ids {
                if lanes.reservation_movement(id).is_none() {
                    continue;
                }
                self.active[id].store(false, Ordering::Relaxed);
                self.frontier[id].store(f32::MAX.to_bits(), Ordering::Relaxed);
                let Some(&out) = lanes.lanes[id].next_lanes.first() else {
                    continue;
                };
                if self.capacity[out] == FREE {
                    self.capacity[out] = downstream_slots(
                        lanes.lanes[out].length,
                        buckets.get(out).and_then(|b| b.first()).map(|&(d, _)| d),
                    );
                    self.touched_exits.push(out);
                }
                if let Some(bucket) = buckets.get(id) {
                    for &(d, a) in bucket {
                        if agents.current_lane_id[a] == id
                            && agents.transit_mode[a] == super::super::super::MODE_CAR
                        {
                            self.active[id].store(true, Ordering::Relaxed);
                            self.frontier[id].fetch_min(d.max(0.0).to_bits(), Ordering::Relaxed);
                            self.exits[out].fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        }
        // Restore bodies before planning any movement. A tail remains in its last zone
        // after its reference point has entered the outgoing road.
        for &node in nodes {
            let Some(ids) = lanes.node_lanes.get(&node) else {
                continue;
            };
            for &id in ids {
                let Some(control) = lanes.reservation_movement(id) else {
                    continue;
                };
                for zone in &control.zones {
                    let occupied = buckets.get(id).is_some_and(|b| {
                        let start = b.partition_point(|&(d, _)| d < zone.enter);
                        let end = b.partition_point(|&(d, _)| d <= zone.exit + CAR_LENGTH);
                        b[start..end].iter().any(|&(_, a)| {
                            agents.current_lane_id[a] == id
                                && agents.transit_mode[a] == super::super::super::MODE_CAR
                        })
                    });
                    let tail = lanes.lanes[id]
                        .next_lanes
                        .first()
                        .and_then(|&out| buckets.get(out))
                        .and_then(|b| b.first())
                        .is_some_and(|&(d, _)| {
                            d + lanes.lanes[id].length <= zone.exit + CAR_LENGTH
                        });
                    if occupied || tail {
                        self.zones[zone.id].fetch_min(id, Ordering::Relaxed);
                    }
                }
            }
        }
        for &node in nodes {
            let Some(ids) = lanes.node_lanes.get(&node) else {
                continue;
            };
            for &id in ids {
                if lanes.reservation_movement(id).is_none() {
                    continue;
                }
                if let Some(bucket) = buckets.get(id) {
                    for &(d, a) in bucket {
                        if agents.current_lane_id[a] == id
                            && agents.transit_mode[a] == super::super::super::MODE_CAR
                        {
                            self.claim_step(id, d, network);
                        }
                    }
                }
            }
        }
    }

    // An edge A -> B means A will need a zone currently owned by B. Every resource
    // has exactly two compiled paths, so acquiring A's side adds at most B -> A.
    // Reject that acquisition iff A already reaches B. This preserves acyclicity
    // without speculative travel-time estimates or a global traffic graph.
    fn reachable(&self, start: usize, target: usize, network: &TransitNetwork) -> bool {
        let mut epoch = self
            .visit_epoch
            .fetch_add(1, Ordering::Relaxed)
            .wrapping_add(1);
        if epoch == 0 {
            for stamp in &self.visited {
                stamp.store(0, Ordering::Relaxed);
            }
            epoch = self.visit_epoch.fetch_add(1, Ordering::Relaxed) + 1;
        }
        let mut top = 1;
        self.stack[0].store(start, Ordering::Relaxed);
        self.visited[start].store(epoch, Ordering::Relaxed);
        while top > 0 {
            top -= 1;
            let id = self.stack[top].load(Ordering::Relaxed);
            if id == target {
                return true;
            }
            if !self.active[id].load(Ordering::Relaxed) {
                continue;
            }
            let front = f32::from_bits(self.frontier[id].load(Ordering::Relaxed));
            let Some(control) = network.lane_system.reservation_movement(id) else {
                continue;
            };
            for zone in &control.zones {
                if front > zone.exit + CAR_LENGTH {
                    continue;
                }
                let owner = self.zones[zone.id].load(Ordering::Relaxed);
                if owner == FREE
                    || owner == id
                    || self.visited[owner].load(Ordering::Relaxed) == epoch
                {
                    continue;
                }
                self.visited[owner].store(epoch, Ordering::Relaxed);
                self.stack[top].store(owner, Ordering::Relaxed);
                top += 1;
            }
        }
        false
    }

    fn claim_step(&self, connector: usize, distance: f32, network: &TransitNetwork) {
        let Some(control) = network.lane_system.reservation_movement(connector) else {
            return;
        };
        for zone in &control.zones {
            if distance > zone.exit + CAR_LENGTH {
                continue;
            }
            if zone.enter > distance + self.step_distance {
                break;
            }
            let owner = self.zones[zone.id].load(Ordering::Relaxed);
            if owner == connector {
                continue;
            }
            if owner != FREE {
                break;
            }
            let other_needs = self.active[zone.other].load(Ordering::Relaxed)
                && f32::from_bits(self.frontier[zone.other].load(Ordering::Relaxed))
                    <= zone.other_exit + CAR_LENGTH;
            if other_needs && self.reachable(connector, zone.other, network) {
                break;
            }
            self.zones[zone.id].store(connector, Ordering::Relaxed);
        }
    }

    /// Checks current occupancy/step bookings, without reserving the rest of a traversal.
    pub(super) fn clear(&self, connector: usize, network: &TransitNetwork) -> bool {
        network
            .lane_system
            .reservation_movement(connector)
            .is_none_or(|control| {
                control.zones.iter().all(|z| {
                    let owner = self.zones[z.id].load(Ordering::Relaxed);
                    owner == FREE || owner == connector
                })
            })
    }

    /// Tests physical downstream storage not already promised to an in-flight car.
    pub(super) fn exit_available(&self, connector: usize, network: &TransitNetwork) -> bool {
        network
            .lane_system
            .lanes
            .get(connector)
            .and_then(|l| l.next_lanes.first())
            .is_some_and(|&out| {
                self.capacity.get(out).is_none_or(|&capacity| {
                    capacity == FREE || self.exits[out].load(Ordering::Relaxed) < capacity
                })
            })
    }

    /// Activates the admitted path and claims only zones it can reach in this movement step.
    pub(super) fn reserve(&self, connector: usize, network: &TransitNetwork) {
        if network
            .lane_system
            .reservation_movement(connector)
            .is_some()
        {
            // Entry was checked against every currently held zone. Extending this group's
            // frontier therefore adds no wait edge; subsequent claims each pass the cycle test.
            self.active[connector].store(true, Ordering::Relaxed);
            self.frontier[connector].store(0.0f32.to_bits(), Ordering::Relaxed);
            self.claim_step(connector, 0.0, network);
            if let Some(&out) = network.lane_system.lanes[connector].next_lanes.first() {
                self.exits[out].fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Converts booked downstream space into actual outgoing-lane occupancy.
    pub(super) fn release_exit(&self, connector: usize, out: usize, network: &TransitNetwork) {
        if network
            .lane_system
            .reservation_movement(connector)
            .is_some()
        {
            let old = self.exits[out].fetch_sub(1, Ordering::Relaxed);
            debug_assert!(
                old > 0,
                "connector exit must consume its reserved downstream storage"
            );
        }
    }

    /// Protects space in which every booked vehicle's tail must clear the junction.
    pub(super) fn external_entry_clear(&self, lane: usize, distance: f32) -> bool {
        self.exits.get(lane).is_none_or(|reserved| {
            let count = reserved.load(Ordering::Relaxed);
            count == 0 || distance >= count as f32 * SPACE + CAR_LENGTH
        })
    }

    /// Stops before the first unclaimed zone reachable during this exact movement step.
    pub(super) fn travel_limit(
        &self,
        connector: usize,
        distance: f32,
        network: &TransitNetwork,
    ) -> f32 {
        network
            .lane_system
            .reservation_movement(connector)
            .map_or(f32::MAX, |control| {
                control
                    .zones
                    .iter()
                    .filter(|z| {
                        z.exit + CAR_LENGTH >= distance && z.enter <= distance + self.step_distance
                    })
                    .filter(|z| self.zones[z.id].load(Ordering::Relaxed) != connector)
                    .map(|z| (z.enter.next_down() - distance).max(0.0))
                    .fold(f32::MAX, f32::min)
            })
    }
}
