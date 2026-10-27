// SPDX-License-Identifier: GPL-2.0-only

//! Unsigned ramp merges and diverges retain collision and downstream-storage reservations.

use super::*;
use crate::simulation::network::graph::RoadMedian;
use crate::simulation::network::ramps::{RampLayout, RampRequest};

#[test]
fn ramp_merge_and_diverge_fleets_drain_safely_and_match_workers() {
    for exit in [false, true] {
        let run = |workers| {
            let mut graph = RegionGraph::new();
            let a = Vector3::new(-600.0, 0.0, 0.0);
            let b = -a;
            graph.add_node(a, NodeType::Junction);
            graph.add_node(b, NodeType::Junction);
            graph.add_edge(Edge {
                start_node: 0,
                end_node: 1,
                geometry: vec![a, b],
                physical_geometry: vec![a, b],
                physical_length: 1200.0,
                fwd_lanes: 2,
                bkw_lanes: 2,
                width: 19.0,
                median: RoadMedian::from_tool(2, 5.0).unwrap(),
                speed_limit: 100.0 / 3.6,
                allowed_types: TransitFlags::CAR,
                ..Default::default()
            });
            RampLayout::prepare(
                &graph,
                RampRequest {
                    edge: 0,
                    station: 600.0,
                    forward: true,
                    exit,
                    points: vec![
                        Vector3::ZERO,
                        Vector3::new(if exit { 200.0 } else { -200.0 }, 0.0, 100.0),
                    ],
                    length: 80.0,
                    taper: 35.0,
                },
            )
            .unwrap()
            .apply(&mut graph);
            let routes = if exit {
                vec![(0, true, 2, true), (0, true, 3, true)]
            } else {
                vec![(0, true, 2, true), (3, true, 2, true)]
            };
            super::zone_support::routed_fleet(graph, &routes, workers)
        };
        assert_eq!(run(1), run(8));
    }
}
