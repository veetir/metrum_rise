// SPDX-License-Identifier: GPL-2.0-only

//! Allocation-free candidate ranking with cheap same-frontage walking quotes.

use super::*;
use crate::simulation::buildings::allocator::BuildingAllocator;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::types::TransitFlags;
use crate::simulation::pathing::cost::CostCalculator;
use godot::prelude::Vector2;

impl ParkingSystem {
    /// O(9 + local candidates × frontage knots), no allocations/candidate cap. Same-edge,
    /// same-side candidates use existing slope-aware walking cost and projected walking metres;
    /// other candidates use the foundation's Euclidean estimate. No CCH search is started.
    /// Round 2 provides full connected itinerary quotes through nearest_with at departure.
    pub(crate) fn nearest_frontage(
        &self,
        point: [f32; 2],
        frontage: usize,
        graph: &RegionGraph,
    ) -> Option<Handle<Place>> {
        let edge = graph
            .get_edge(frontage)
            .filter(|e| !e.deleted && e.allowed_types & TransitFlags::FOOT != 0)?;
        let target = Vector2::from_array(point);
        let target_s =
            BuildingAllocator::project_point_to_polyline_s(&edge.physical_geometry, target);
        let projection = BuildingAllocator::sample_pos_on_polyline(
            &edge.physical_geometry,
            edge.physical_length,
            target_s,
        );
        let tangent = BuildingAllocator::sample_tangent_on_edge(
            graph,
            frontage,
            target_s / edge.physical_length,
        );
        let normal = Vector2::new(tangent.y, -tangent.x);
        let side = (target - projection).dot(normal).signum() as i8;
        let walk_per_m = if edge.physical_length > 0.0 {
            CostCalculator::walk_cost_s(edge) / edge.physical_length
        } else {
            return None;
        };
        self.nearest_with(point, |handle, pos| {
            let p = self.place_handles.resolve(handle)?;
            let straight = ((pos[0] - point[0]).powi(2) + (pos[2] - point[1]).powi(2)).sqrt();
            let mut cost = ParkingCost {
                walk_m: straight,
                walk_s: straight / crate::config::AGENT_WALK_SPEED_MS,
                drive_s: 0.0,
            };
            if self.places.source[p]
                == (PlaceSource::Edge {
                    edge: frontage,
                    side,
                })
            {
                let car = Vector2::new(pos[0], pos[2]);
                let car_s =
                    BuildingAllocator::project_point_to_polyline_s(&edge.physical_geometry, car);
                let car_projection = BuildingAllocator::sample_pos_on_polyline(
                    &edge.physical_geometry,
                    edge.physical_length,
                    car_s,
                );
                let lateral =
                    ((target - projection).length() - (car - car_projection).length()).abs();
                let along = (target_s - car_s).abs();
                cost.walk_m = along + lateral;
                cost.walk_s = along * walk_per_m + lateral / crate::config::AGENT_WALK_SPEED_MS;
            }
            Some(cost)
        })
    }

    /// O(1) geometry inspection for bridge/tests; stale handles have no position.
    pub(crate) fn position(&self, place: Handle<Place>) -> Option<[f32; 3]> {
        self.place_handles
            .resolve(place)
            .map(|p| self.places.position[p])
    }
}

#[cfg(test)]
impl ParkingSystem {
    /// Ordered source products and stable identities for topology/locality acceptance only.
    pub(crate) fn source_bays(&self, source: PlaceSource) -> Vec<(u64, Bay)> {
        let mut result: Vec<_> = self
            .sources
            .get(&source)
            .into_iter()
            .flatten()
            .map(|&h| {
                let p = self.place_handles.resolve(h).expect("source bay");
                (
                    h.to_bits(),
                    Bay {
                        identity: self.places.identity[p],
                        position: self.places.position[p],
                        heading: self.places.heading[p],
                        width: self.places.width[p],
                        length: self.places.length[p],
                    },
                )
            })
            .collect();
        result.sort_unstable_by_key(|(_, bay)| bay.identity);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cheap_same_frontage_quote_keeps_the_walk_bound() {
        let mut graph = RegionGraph::new();
        let a = graph.add_node(
            godot::prelude::Vector3::ZERO,
            crate::simulation::network::types::NodeType::Junction,
        );
        let b = graph.add_node(
            godot::prelude::Vector3::new(500.0, 0.0, 0.0),
            crate::simulation::network::types::NodeType::Junction,
        );
        let mut edge = crate::simulation::network::roadbed::fixtures::road_edge(
            a,
            b,
            vec![graph.node(a).pos, graph.node(b).pos],
            7.0,
        );
        let (cost, length) = CostCalculator::calculate_costs(&edge);
        edge.base_cost = cost;
        edge.physical_length = length;
        graph.add_edge(edge);
        let mut system = ParkingSystem::default();
        let place = system.add_place(
            PlaceSource::Edge { edge: 0, side: 1 },
            Bay {
                identity: [1, 0],
                position: [399.0, 0.0, -2.35],
                heading: 0.0,
                width: 1.8,
                length: 6.0,
            },
        );
        assert!(system.nearest([0.0, -5.0]).is_some());
        assert!(system.nearest_frontage([0.0, -5.0], 0, &graph).is_none());
        assert_eq!(
            system.nearest_frontage([1.65, -5.0], 0, &graph),
            Some(place)
        );
    }
}
