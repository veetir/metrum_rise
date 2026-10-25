// SPDX-License-Identifier: GPL-2.0-only

//! Allocation-free admission over occupied junction neighborhoods and immutable lane buckets.

mod reservations;

use super::super::data::{AgentSystem, AgentVec};
use super::lane_nav::{
    collect_connector_lanes_to_edge, collect_connector_lanes_to_lane, planned_next_connector,
};
use super::traffic::{connector_turn_speed, lane_entry_slot_clear};
use crate::config::{CAR_JUNCTION_MIN_SPEED_MS, IDM_B};
use crate::simulation::network::{TransitNetwork, graph::RegionGraph};
use std::sync::atomic::{AtomicBool, Ordering};

#[derive(Clone)]
struct Request {
    agent: usize,
    eligible: bool,
    clear: bool,
    granted: bool,
    start: bool,
    batchable: bool,
    winner: usize,
    index: usize,
    low: usize,
    component: usize,
    on_stack: bool,
}
impl Default for Request {
    fn default() -> Self {
        Self {
            agent: usize::MAX,
            eligible: false,
            clear: false,
            granted: false,
            start: false,
            batchable: false,
            winner: usize::MAX,
            index: usize::MAX,
            low: usize::MAX,
            component: usize::MAX,
            on_stack: false,
        }
    }
}

/// Reusable connector admission scratch; all growth is prepared before visiting traffic.
#[derive(Default)]
pub(crate) struct JunctionAdmission {
    requests: Vec<Request>,
    touched: Vec<usize>,
    nodes: Vec<usize>,
    node_seen: Vec<bool>,
    stack: Vec<usize>,
    candidates: Vec<usize>,
    next_index: usize,
    reservations: reservations::Reservations,
    component_open: Vec<AtomicBool>,
}

impl JunctionAdmission {
    /// Full-traversal readiness, used for approach speed; staged entry has its own zone guard.
    pub(crate) fn allows(&self, agent: usize, connector: usize, network: &TransitNetwork) -> bool {
        network.lane_system.controlled_movement(connector).is_none()
            || self
                .requests
                .get(connector)
                .is_some_and(|request| request.granted && request.agent == agent)
    }

    /// Right of way to start, with stepwise conflict claims enforcing actual zone entry.
    pub(super) fn entry_allows(
        &self,
        agent: usize,
        connector: usize,
        network: &TransitNetwork,
    ) -> bool {
        network.lane_system.controlled_movement(connector).is_none()
            || self
                .requests
                .get(connector)
                .is_some_and(|r| r.start && r.agent == agent)
    }

    /// Brakes against the granted routing candidates, including a shorter fan connector
    /// that differs from the planner's first representative. O(local successors), no allocation.
    pub(crate) fn approach_turn_speed(
        &self,
        agent: usize,
        connector: usize,
        network: &TransitNetwork,
        buckets: &[Vec<(f32, usize)>],
    ) -> f32 {
        let lanes = &network.lane_system;
        let Some(control) = lanes.controlled_movement(connector) else {
            return if lane_entry_slot_clear(connector, buckets) {
                connector_turn_speed(&lanes.lanes[connector])
            } else {
                0.0
            };
        };
        lanes.lanes[control.incoming]
            .next_lanes
            .iter()
            .copied()
            .filter(|&id| {
                self.requests
                    .get(id)
                    .is_some_and(|r| (r.start || r.batchable) && r.agent == agent)
            })
            .map(|id| {
                let speed = connector_turn_speed(&lanes.lanes[id]);
                if self.allows(agent, id, network) {
                    speed
                } else {
                    speed.min(CAR_JUNCTION_MIN_SPEED_MS)
                }
            })
            .reduce(f32::min)
            .unwrap_or(0.0)
    }

    /// Tests zone leases and downstream storage immediately before a serial entry.
    pub(super) fn storage_and_zones_clear(
        &self,
        connector: usize,
        network: &TransitNetwork,
    ) -> bool {
        let lane = &network.lane_system.lanes[connector];
        (network
            .lane_system
            .reservation_movement(connector)
            .is_none()
            || self.node_seen.get(lane.node_id).copied().unwrap_or(false))
            && self.reservations.exit_available(connector, network)
    }

    /// Books downstream storage and the first ordered movement-step zones after admission.
    pub(super) fn reserve_path(&self, agent: usize, connector: usize, network: &TransitNetwork) {
        self.reservations.reserve(agent, connector, network);
        if let Some(request) = self.requests.get(connector)
            && let Some(open) = self.component_open.get(request.component)
        {
            open.store(true, Ordering::Relaxed);
        }
    }

    /// After the cycle winner entered, admits compatible movements from that same closed cycle.
    pub(super) fn batch_allows(&self, agent: usize, connector: usize) -> bool {
        self.requests.get(connector).is_some_and(|r| {
            r.agent == agent
                && r.batchable
                && self.component_open[r.component].load(Ordering::Relaxed)
        })
    }

    /// Distinguishes brokered zone claims from an unregistered connector-lane owner.
    pub(super) fn has_entry_claim(&self, connector: usize) -> bool {
        self.reservations.has_entry_claim(connector)
    }

    /// Protects the downstream prefix against frontage, border and lateral entry.
    pub(super) fn external_entry_clear(&self, lane: usize, distance: f32) -> bool {
        self.reservations.external_entry_clear(lane, distance)
    }

    /// Converts a connector's reserved downstream space into actual lane occupancy.
    pub(super) fn release_exit(
        &self,
        agent: usize,
        connector: usize,
        out: usize,
        network: &TransitNetwork,
    ) {
        self.reservations
            .release_exit(agent, connector, out, network);
    }

    /// Distance to the first unowned conflict interval, including body clearance.
    pub(crate) fn connector_travel_limit(
        &self,
        agent: usize,
        connector: usize,
        distance: f32,
        network: &TransitNetwork,
    ) -> f32 {
        self.reservations
            .travel_limit(agent, connector, distance, network)
    }

    /// Returns one connector's reservations for a failed safety assertion, outside traffic loops.
    #[cfg(test)]
    pub(crate) fn zone_diagnostic(
        &self,
        connector: usize,
        network: &TransitNetwork,
    ) -> serde_json::Value {
        self.reservations.diagnostic(connector, network)
    }

    /// Emits manual fixture diagnostics outside traffic loops; absent from gameplay builds.
    #[cfg(test)]
    pub(crate) fn diagnostic(&self, network: &TransitNetwork) -> serde_json::Value {
        let rows: Vec<_> = self.touched.iter().filter_map(|&id| {
            let r=&self.requests[id];
            (r.agent != usize::MAX).then(|| serde_json::json!({"connector":id,"agent":r.agent,"eligible":r.eligible,"clear":r.clear,"granted":r.granted,"component":r.component,"reservation":self.reservations.diagnostic(id,network)}))
        }).collect();
        serde_json::json!(rows)
    }

    /// Reports clear granted heads after movement for the manual starvation observer.
    #[cfg(test)]
    pub(crate) fn audit_ready_heads(&self, network: &TransitNetwork) -> Vec<(usize, usize, usize)> {
        self.touched
            .iter()
            .filter_map(|&id| {
                let r = &self.requests[id];
                (r.granted && self.reservations.exit_available(id, network)).then(|| {
                    (
                        id,
                        network
                            .lane_system
                            .controlled_movement(id)
                            .unwrap()
                            .incoming,
                        r.agent,
                    )
                })
            })
            .collect()
    }

    fn prepare_buffers(&mut self, lanes: usize, nodes: usize) {
        for &id in &self.touched {
            self.requests[id] = Request::default();
            self.component_open[id].store(false, Ordering::Relaxed);
        }
        for &id in &self.nodes {
            self.node_seen[id] = false;
        }
        self.touched.clear();
        self.nodes.clear();
        self.requests
            .resize_with(lanes.max(self.requests.len()), Request::default);
        self.node_seen
            .resize(nodes.max(self.node_seen.len()), false);
        self.component_open
            .resize_with(lanes.max(self.component_open.len()), || {
                AtomicBool::new(false)
            });
        self.touched.reserve(lanes);
        self.nodes.reserve(nodes);
        self.stack.reserve(lanes);
        self.candidates.reserve(lanes);
    }

    // Tarjan's strongly connected components resolve a closed right-hand waiting cycle only
    // inside that cycle. A minor-road request outside it never jumps a priority movement.
    fn visit(&mut self, id: usize, network: &TransitNetwork) {
        let index = self.next_index;
        self.next_index += 1;
        let request = &mut self.requests[id];
        request.index = index;
        request.low = index;
        request.on_stack = true;
        self.stack.push(id);
        for &other in &network
            .lane_system
            .controlled_movement(id)
            .unwrap()
            .yields_to
        {
            if !self.requests[other].eligible
                || self.requests[other].agent == self.requests[id].agent
            {
                continue;
            }
            if self.requests[other].index == usize::MAX {
                self.visit(other, network);
                self.requests[id].low = self.requests[id].low.min(self.requests[other].low);
            } else if self.requests[other].on_stack {
                self.requests[id].low = self.requests[id].low.min(self.requests[other].index);
            }
        }
        if self.requests[id].low == index {
            while let Some(other) = self.stack.pop() {
                self.requests[other].on_stack = false;
                self.requests[other].component = id;
                if other == id {
                    break;
                }
            }
        }
    }

    fn prepare(
        &mut self,
        agents: &AgentVec,
        buckets: &[Vec<(f32, usize)>],
        dirty: &[usize],
        network: &TransitNetwork,
        graph: &RegionGraph,
        delta: f32,
    ) {
        self.prepare_buffers(network.lane_system.lanes.len(), graph.node_count());
        let lanes = &network.lane_system;
        for &id in dirty {
            let lane = &lanes.lanes[id];
            let endpoints = if lane.edge_id == usize::MAX {
                [lane.node_id, lane.node_id]
            } else {
                let edge = graph.edge(lane.edge_id);
                [edge.start_node as usize, edge.end_node as usize]
            };
            // Outgoing bodies and near-mouth external entry need the source node too.
            for node in endpoints {
                if node < self.node_seen.len() && !self.node_seen[node] {
                    self.node_seen[node] = true;
                    self.nodes.push(node);
                }
            }
        }

        self.nodes.sort_unstable();
        self.reservations
            .prepare(agents, buckets, &self.nodes, network, delta);
        // Admission shares sparse global lane IDs and one Tarjan stack. Keep this bounded
        // arbitration ordered; per-agent speed and ordinary movement remain parallel.
        for n in 0..self.nodes.len() {
            let Some(ids) = lanes.node_lanes.get(&self.nodes[n]) else {
                continue;
            };
            for &id in ids {
                let Some(control) = lanes.controlled_movement(id) else {
                    continue;
                };
                self.touched.push(id);
                let source = control.incoming;
                let Some(&(distance, agent)) = buckets.get(source).and_then(|b| b.last()) else {
                    continue;
                };
                if agents.current_lane_id[agent] != source {
                    continue;
                }
                let remaining = (lanes.lanes[source].length - distance).max(0.0);
                let lookahead = agents.speed[agent].powi(2) / (2.0 * IDM_B)
                    + 20.0
                    + agents.speed[agent] * delta;
                if remaining > lookahead {
                    continue;
                }
                let Some(wanted) = planned_next_connector(
                    source,
                    agents.current_node[agent],
                    &agents.current_path[agent],
                    agents.current_path_index[agent],
                    agents.access_flags[agent],
                    agents.planned_detach_node[agent],
                    agents.planned_detach_lane_id[agent] as usize,
                    network,
                    graph,
                )
                .or_else(|| {
                    // A missed turn uses the existing first legal onward movement, then
                    // replans on its outgoing road. It must still obey local control.
                    lanes.lanes[source].next_lanes.first().copied()
                }) else {
                    continue;
                };
                let Some(&out) = lanes.lanes[wanted].next_lanes.first() else {
                    continue;
                };
                let exact_detach = agents.planned_detach_node[agent] as usize
                    == lanes.lanes[id].node_id
                    && agents.planned_detach_lane_id[agent] as usize == out
                    && agents.access_flags[agent] & super::super::ACCESS_PLAN_VALID != 0;
                if exact_detach {
                    collect_connector_lanes_to_lane(source, out, network, &mut self.candidates);
                } else {
                    collect_connector_lanes_to_edge(
                        source,
                        lanes.lanes[out].edge_id,
                        network,
                        &mut self.candidates,
                    );
                }
                if !self.candidates.contains(&id) {
                    continue;
                }
                self.requests[id].agent = agent;
                let stopped = !control.stop || (remaining <= 0.05 && agents.speed[agent] <= 0.1);
                let exit_clear = lanes.lanes[id]
                    .next_lanes
                    .first()
                    .is_some_and(|&out| lane_entry_slot_clear(out, buckets))
                    && self.reservations.exit_available(id, network);
                let clear = self.reservations.clear(id, network);
                // An occupied crossing prevents traversal without cancelling a waiting car's
                // precedence. Otherwise a minor stream can keep following itself across
                // the junction while the major approach is repeatedly marked ineligible.
                self.requests[id].eligible = stopped && exit_clear;
                self.requests[id].clear = clear && lane_entry_slot_clear(id, buckets);
            }
            self.next_index = 0;
            for &id in ids {
                if self
                    .requests
                    .get(id)
                    .is_some_and(|r| r.eligible && r.index == usize::MAX)
                {
                    self.visit(id, network);
                }
            }
            // Reuse each component root's low field for its minimum agent ID.
            for &id in ids {
                if self.requests.get(id).is_some_and(|r| r.eligible) {
                    let root = self.requests[id].component;
                    if root == id {
                        self.requests[root].low = usize::MAX;
                        self.requests[root].winner = usize::MAX;
                    }
                }
            }
            for &id in ids {
                if self.requests.get(id).is_some_and(|r| r.eligible) {
                    let root = self.requests[id].component;
                    if self.requests[id].agent < self.requests[root].low {
                        self.requests[root].low = self.requests[id].agent;
                        self.requests[root].winner = id;
                    }
                }
            }
            for &id in ids {
                let Some(control) = lanes.controlled_movement(id) else {
                    continue;
                };
                if !self.requests[id].eligible {
                    continue;
                }
                let root = self.requests[id].component;
                self.requests[id].start = lane_entry_slot_clear(id, buckets)
                    && self.requests[id].agent == self.requests[root].low
                    && control.yields_to.iter().all(|&other| {
                        !self.requests[other].eligible
                            || self.requests[other].agent == self.requests[id].agent
                            || self.requests[other].component == root
                    });
                self.requests[id].granted = self.requests[id].start && self.requests[id].clear;
            }
            for &id in ids {
                if !self
                    .requests
                    .get(id)
                    .is_some_and(|r| r.eligible && !r.start)
                {
                    continue;
                }
                let root = self.requests[id].component;
                let winner = self.requests[root].winner;
                let control = lanes.controlled_movement(id).unwrap();
                self.requests[id].batchable = self.requests[winner].start
                    && lane_entry_slot_clear(id, buckets)
                    && !control.conflicts.contains(&winner)
                    && control.yields_to.iter().all(|&other| {
                        !self.requests[other].eligible
                            || self.requests[other].agent == self.requests[id].agent
                            || self.requests[other].component == root
                    });
            }
        }
    }
}

impl AgentSystem {
    pub(super) fn prepare_junction_admission(
        &mut self,
        network: &TransitNetwork,
        graph: &RegionGraph,
        delta: f32,
    ) {
        self.junction_admission.prepare(
            &self.agents,
            &self.lane_buckets,
            &self.dirty_lanes,
            network,
            graph,
            delta,
        );
    }
}
