// SPDX-License-Identifier: GPL-2.0-only

//! Per-vehicle zone claims, acyclic local waits and promised downstream body storage.

use super::super::super::data::AgentVec;
use crate::config::{CAR_JUNCTION_SPEED_MS, CAR_LENGTH, IDM_S_MIN};
use crate::simulation::network::TransitNetwork;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

const FREE: usize = usize::MAX;
const OUTGOING_TAIL: usize = usize::MAX - 1;
const ENTERED: usize = 1 << (usize::BITS - 1);
const SPACE: f32 = CAR_LENGTH + IDM_S_MIN;

fn tail_before(front: f32, exit: f32) -> bool {
    f64::from(front) - f64::from(CAR_LENGTH) < f64::from(exit)
}

fn downstream_slots(length: f32, first: Option<f32>) -> usize {
    let boundary = first.map_or(length, |d| (d - SPACE).min(length));
    let clear = CAR_LENGTH.next_up();
    if boundary < clear {
        0
    } else {
        1 + ((boundary - clear) / SPACE).floor() as usize
    }
}

// Slots are local to this tick; dense agent swap-removes cannot leave persistent owners.
// A zone shared by a platoon names its rearmost claimant: that particular tail releases last.
#[derive(Default)]
struct Vehicle {
    agent: AtomicUsize,
    connector: AtomicUsize,
    front: AtomicU32,
    leader: AtomicUsize,
    inside: AtomicBool,
}

/// Retained sparse traffic scratch. All growth precedes vehicle visitation.
#[derive(Default)]
pub(super) struct Reservations {
    zones: Vec<AtomicUsize>,
    physical: Vec<AtomicUsize>,
    exits: Vec<AtomicUsize>,
    capacity: Vec<usize>,
    touched_exits: Vec<usize>,
    path_state: Vec<AtomicUsize>,
    entry_leader: Vec<AtomicUsize>,
    agent_slot: Vec<AtomicUsize>,
    vehicles: Vec<Vehicle>,
    vehicle_count: AtomicUsize,
    visited: Vec<AtomicUsize>,
    stack: Vec<AtomicUsize>,
    visit_epoch: AtomicUsize,
    step_distance: f32,
}

impl Reservations {
    /// Restores physical bodies/tails, then claims reachable intervals in stable lane/agent order.
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
        for slot in 0..self.vehicle_count.swap(0, Ordering::Relaxed) {
            let agent = self.vehicles[slot].agent.load(Ordering::Relaxed);
            self.agent_slot[agent].store(FREE, Ordering::Relaxed);
        }
        self.zones
            .resize_with(lanes.conflict_zone_count().max(self.zones.len()), || {
                AtomicUsize::new(FREE)
            });
        self.physical
            .resize_with(self.zones.len(), || AtomicUsize::new(FREE));
        let count = lanes.lanes.len().max(self.exits.len());
        self.exits.resize_with(count, || AtomicUsize::new(0));
        self.capacity.resize(count, FREE);
        self.path_state.resize_with(count, || AtomicUsize::new(0));
        self.entry_leader
            .resize_with(count, || AtomicUsize::new(FREE));
        for &out in &self.touched_exits {
            self.capacity[out] = FREE;
            self.exits[out].store(0, Ordering::Relaxed);
        }
        self.touched_exits.clear();
        self.touched_exits.reserve(count);
        let mut paths = 0;
        let mut occupants = 0;
        for &node in nodes {
            if let Some(ids) = lanes.node_lanes.get(&node) {
                for &id in ids {
                    if lanes.reservation_movement(id).is_some() {
                        paths += 1;
                        occupants += buckets.get(id).map_or(0, Vec::len);
                    }
                }
            }
        }
        // At most one tail per outgoing lane and one new handoff per connector per tick.
        // One car can finish an old connector and start another, hence no min(agent count).
        let bound = occupants + 2 * paths;
        if bound > 0 {
            self.agent_slot
                .resize_with(agents.transit.len().max(self.agent_slot.len()), || {
                    AtomicUsize::new(FREE)
                });
            self.vehicles
                .resize_with(bound.max(self.vehicles.len()), Vehicle::default);
            self.visited
                .resize_with(self.vehicles.len(), || AtomicUsize::new(0));
            self.stack
                .resize_with(self.vehicles.len(), || AtomicUsize::new(0));
        }
        for &node in nodes {
            for zone in lanes.junction_zones(node) {
                self.zones[zone.movement.id].store(FREE, Ordering::Relaxed);
                self.physical[zone.movement.id].store(FREE, Ordering::Relaxed);
            }
            let Some(ids) = lanes.node_lanes.get(&node) else {
                continue;
            };
            for &id in ids {
                if lanes.reservation_movement(id).is_none() {
                    continue;
                }
                self.path_state[id].store(0, Ordering::Relaxed);
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
                // Register front to back so each vehicle's physical leader already has a slot.
                let mut leader = FREE;
                if let Some(bucket) = buckets.get(id) {
                    for &(d, a) in bucket.iter().rev() {
                        if agents.current_lane_id[a] == id
                            && agents.transit_mode[a] == super::super::super::MODE_CAR
                        {
                            leader = self.register(a, id, d, leader, true);
                            self.exits[out].fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                self.entry_leader[id].store(leader, Ordering::Relaxed);
            }
        }
        for &node in nodes {
            let Some(ids) = lanes.node_lanes.get(&node) else {
                continue;
            };
            for &id in ids {
                let Some(control) = lanes.reservation_movement(id) else {
                    continue;
                };
                for zone in &control.zones {
                    if let Some(bucket) = buckets.get(id) {
                        let start = bucket.partition_point(|&(d, _)| d <= zone.enter);
                        let end = bucket.partition_point(|&(d, _)| tail_before(d, zone.exit));
                        for &(_, a) in &bucket[start..end] {
                            if agents.current_lane_id[a] == id
                                && agents.transit_mode[a] == super::super::super::MODE_CAR
                            {
                                self.seed(zone.id, self.agent_slot[a].load(Ordering::Relaxed));
                            }
                        }
                    }
                    if let Some(&(d, a)) = lanes.lanes[id]
                        .next_lanes
                        .first()
                        .and_then(|&out| buckets.get(out))
                        .and_then(|b| b.first())
                        && f64::from(d) - f64::from(CAR_LENGTH)
                            < f64::from(zone.exit) - f64::from(lanes.lanes[id].length)
                    {
                        let source = agents.tail_connector_lane_id[a];
                        if source == u32::MAX {
                            self.seed(zone.id, OUTGOING_TAIL);
                        } else if source as usize == id {
                            let slot = self.slot(a, id).unwrap_or_else(|| {
                                self.register(a, id, lanes.lanes[id].length + d, FREE, false)
                            });
                            self.seed(zone.id, slot);
                        }
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
                    for &(_, a) in bucket {
                        if agents.current_lane_id[a] == id
                            && agents.transit_mode[a] == super::super::super::MODE_CAR
                        {
                            self.claim_step(self.agent_slot[a].load(Ordering::Relaxed), network);
                        }
                    }
                }
            }
        }
    }

    fn slot(&self, agent: usize, connector: usize) -> Option<usize> {
        let slot = self.agent_slot.get(agent)?.load(Ordering::Relaxed);
        (slot != FREE && self.vehicles[slot].connector.load(Ordering::Relaxed) == connector)
            .then_some(slot)
    }

    fn register(
        &self,
        agent: usize,
        connector: usize,
        front: f32,
        leader: usize,
        inside: bool,
    ) -> usize {
        let slot = self.vehicle_count.fetch_add(1, Ordering::Relaxed);
        let vehicle = &self.vehicles[slot];
        vehicle.agent.store(agent, Ordering::Relaxed);
        vehicle.connector.store(connector, Ordering::Relaxed);
        vehicle.front.store(front.to_bits(), Ordering::Relaxed);
        vehicle.leader.store(leader, Ordering::Relaxed);
        vehicle.inside.store(inside, Ordering::Relaxed);
        self.agent_slot[agent].store(slot, Ordering::Relaxed);
        if inside {
            self.path_state[connector].fetch_add(1, Ordering::Relaxed);
        }
        slot
    }

    fn seed(&self, zone: usize, slot: usize) {
        let old = self.zones[zone].load(Ordering::Relaxed);
        let owner = if old == OUTGOING_TAIL || slot == OUTGOING_TAIL {
            OUTGOING_TAIL
        } else if old == FREE {
            slot
        } else if self.vehicles[old].connector.load(Ordering::Relaxed)
            != self.vehicles[slot].connector.load(Ordering::Relaxed)
        {
            OUTGOING_TAIL
        } else if self
            .front(slot)
            .total_cmp(&self.front(old))
            .then(slot.cmp(&old))
            .is_lt()
        {
            slot
        } else {
            old
        };
        self.zones[zone].store(owner, Ordering::Relaxed);
        self.physical[zone].store(owner, Ordering::Relaxed);
    }

    fn front(&self, slot: usize) -> f32 {
        f32::from_bits(self.vehicles[slot].front.load(Ordering::Relaxed))
    }

    fn permits(&self, slot: usize, owner: usize) -> bool {
        slot != FREE
            && (slot == owner
                || (owner != FREE
                    && owner != OUTGOING_TAIL
                    && self.vehicles[slot].connector.load(Ordering::Relaxed)
                        == self.vehicles[owner].connector.load(Ordering::Relaxed)
                    && self.front(slot) >= self.front(owner)))
    }

    fn enqueue(&self, slot: usize, epoch: usize, top: &mut usize) {
        if slot != FREE && self.visited[slot].swap(epoch, Ordering::Relaxed) != epoch {
            self.stack[*top].store(slot, Ordering::Relaxed);
            *top += 1;
        }
    }

    // Owning a zone adds waits from the opposite path to this exact vehicle, not its platoon.
    // Following edges are necessary: a rear claimant cannot clear through a blocked leader.
    // A transfer to a rearmost same-path claimant preserves existing following reachability.
    fn reaches_waiter(
        &self,
        start: usize,
        other: usize,
        exit: f32,
        network: &TransitNetwork,
    ) -> bool {
        if self.path_state[other].load(Ordering::Relaxed) & !ENTERED == 0 {
            return false;
        }
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
        let mut top = 0;
        self.enqueue(start, epoch, &mut top);
        while top > 0 {
            top -= 1;
            let slot = self.stack[top].load(Ordering::Relaxed);
            let vehicle = &self.vehicles[slot];
            if !vehicle.inside.load(Ordering::Relaxed) {
                continue;
            }
            let connector = vehicle.connector.load(Ordering::Relaxed);
            let front = self.front(slot);
            if connector == other && tail_before(front, exit) {
                return true;
            }
            if let Some(control) = network.lane_system.reservation_movement(connector) {
                for zone in &control.zones {
                    if !tail_before(front, zone.exit) {
                        continue;
                    }
                    let owner = self.zones[zone.id].load(Ordering::Relaxed);
                    if owner != FREE && owner != OUTGOING_TAIL && !self.permits(slot, owner) {
                        self.enqueue(owner, epoch, &mut top);
                    }
                }
            }
            self.enqueue(vehicle.leader.load(Ordering::Relaxed), epoch, &mut top);
        }
        false
    }

    fn claim_step(&self, slot: usize, network: &TransitNetwork) {
        let connector = self.vehicles[slot].connector.load(Ordering::Relaxed);
        let front = self.front(slot);
        let Some(control) = network.lane_system.reservation_movement(connector) else {
            return;
        };
        for zone in &control.zones {
            if !tail_before(front, zone.exit) {
                continue;
            }
            if zone.enter > front + self.step_distance {
                break;
            }
            let owner = self.zones[zone.id].load(Ordering::Relaxed);
            if self.permits(slot, owner) || (owner == OUTGOING_TAIL && front > zone.enter) {
                continue;
            }
            if owner == OUTGOING_TAIL
                || (owner != FREE
                    && self.vehicles[owner].connector.load(Ordering::Relaxed) != connector)
            {
                break;
            }
            if self.reaches_waiter(slot, zone.other, zone.other_exit, network) {
                break;
            }
            self.zones[zone.id].store(slot, Ordering::Relaxed);
        }
    }

    /// Full crossing readiness for approach braking; precise claims still bound each vehicle.
    pub(super) fn clear(&self, connector: usize, network: &TransitNetwork) -> bool {
        network
            .lane_system
            .reservation_movement(connector)
            .is_none_or(|control| {
                control.zones.iter().all(|z| {
                    let owner = self.physical[z.id].load(Ordering::Relaxed);
                    owner == FREE
                        || (owner != OUTGOING_TAIL
                            && self.vehicles[owner].connector.load(Ordering::Relaxed) == connector)
                })
            })
    }

    /// A real ordered handoff registered this path's zone and downstream storage bookings.
    pub(super) fn has_entry_claim(&self, connector: usize) -> bool {
        self.path_state
            .get(connector)
            .is_some_and(|state| state.load(Ordering::Relaxed) & ENTERED != 0)
    }

    /// Tests tail-clearing storage not already promised to an in-flight vehicle.
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

    /// Starts an individual vehicle; its first ordered claims cannot close a local wait cycle.
    pub(super) fn reserve(&self, agent: usize, connector: usize, network: &TransitNetwork) {
        if network
            .lane_system
            .reservation_movement(connector)
            .is_none()
        {
            return;
        }
        let previous = self.entry_leader[connector].load(Ordering::Relaxed);
        let leader = if previous != FREE && self.vehicles[previous].inside.load(Ordering::Relaxed) {
            previous
        } else {
            FREE
        };
        let slot = self.register(agent, connector, 0.0, leader, true);
        self.entry_leader[connector].store(slot, Ordering::Relaxed);
        self.path_state[connector].fetch_or(ENTERED, Ordering::Relaxed);
        self.claim_step(slot, network);
        let out = network.lane_system.lanes[connector].next_lanes[0];
        self.exits[out].fetch_add(1, Ordering::Relaxed);
    }

    /// An actual exit consumes promised space and makes its outgoing tail a wait-graph sink.
    pub(super) fn release_exit(
        &self,
        agent: usize,
        connector: usize,
        out: usize,
        network: &TransitNetwork,
    ) {
        if network
            .lane_system
            .reservation_movement(connector)
            .is_none()
        {
            return;
        }
        if let Some(slot) = self.slot(agent, connector)
            && self.vehicles[slot].inside.swap(false, Ordering::Relaxed)
        {
            self.path_state[connector].fetch_sub(1, Ordering::Relaxed);
        }
        let old = self.exits[out].fetch_sub(1, Ordering::Relaxed);
        debug_assert!(
            old > 0,
            "connector exit must consume reserved downstream storage"
        );
    }

    /// Outside entry cannot consume a booked prefix or leave its own body across the mouth.
    pub(super) fn external_entry_clear(&self, lane: usize, distance: f32) -> bool {
        self.exits.get(lane).is_none_or(|reserved| {
            let count = reserved.load(Ordering::Relaxed);
            (distance >= CAR_LENGTH || self.capacity[lane] == FREE || count < self.capacity[lane])
                && (count == 0 || distance >= count as f32 * SPACE + CAR_LENGTH)
        })
    }

    /// Stops this particular vehicle before the first zone it has not claimed this step.
    pub(super) fn travel_limit(
        &self,
        agent: usize,
        connector: usize,
        distance: f32,
        network: &TransitNetwork,
    ) -> f32 {
        let slot = self.slot(agent, connector).unwrap_or(FREE);
        network
            .lane_system
            .reservation_movement(connector)
            .map_or(f32::MAX, |control| {
                control
                    .zones
                    .iter()
                    .filter(|z| {
                        tail_before(distance, z.exit) && z.enter <= distance + self.step_distance
                    })
                    .filter(|z| {
                        let owner = self.zones[z.id].load(Ordering::Relaxed);
                        !self.permits(slot, owner)
                            && !(owner == OUTGOING_TAIL && slot != FREE && distance > z.enter)
                    })
                    .map(|z| (z.enter.next_down() - distance).max(0.0))
                    .fold(f32::MAX, f32::min)
            })
    }

    /// Fixture-only diagnostics outside movement and timing loops.
    #[cfg(test)]
    pub(super) fn diagnostic(
        &self,
        connector: usize,
        network: &TransitNetwork,
    ) -> serde_json::Value {
        let lanes = &network.lane_system;
        let out = lanes.lanes[connector].next_lanes[0];
        let held:Vec<_>=lanes.reservation_movement(connector).unwrap().zones.iter().filter_map(|z| {
            let owner=self.zones[z.id].load(Ordering::Relaxed);
            (owner!=FREE).then(||serde_json::json!({"zone":z.id,"enter":z.enter,"exit":z.exit,"owner_agent":self.vehicles.get(owner).map(|v|v.agent.load(Ordering::Relaxed)),"owner_connector":self.vehicles.get(owner).map(|v|v.connector.load(Ordering::Relaxed)),"owner_front":self.vehicles.get(owner).map(|_|self.front(owner))}))
        }).collect();
        serde_json::json!({"out":out,"capacity":self.capacity[out],"booked":self.exits[out].load(Ordering::Relaxed),"held":held})
    }
}
