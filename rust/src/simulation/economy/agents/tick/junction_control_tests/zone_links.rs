// SPDX-License-Identifier: GPL-2.0-only

//! Two nearby junctions coordinate finite short-link storage without blocking each other.

use super::*;

#[test]
fn opposing_short_link_queues_drain_safely_and_match_workers() {
    let run = |workers| {
        let mut graph = RegionGraph::new();
        for p in [
            (-15.0, 0.0),
            (15.0, 0.0),
            (-115.0, 0.0),
            (-15.0, -100.0),
            (-15.0, 100.0),
            (115.0, 0.0),
            (15.0, -100.0),
            (15.0, 100.0),
        ] {
            graph.add_node(Vector3::new(p.0, 0.0, p.1), NodeType::Junction);
        }
        for (a, b) in [(0, 1), (2, 0), (3, 0), (4, 0), (5, 1), (6, 1), (7, 1)] {
            let p = graph.node(a).pos;
            let q = graph.node(b).pos;
            graph.add_edge(Edge {
                start_node: a,
                end_node: b,
                geometry: vec![p, q],
                physical_geometry: vec![p, q],
                physical_length: p.distance_to(q),
                fwd_lanes: 1,
                bkw_lanes: 1,
                width: 7.0,
                speed_limit: 30.0 / 3.6,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                ..Default::default()
            });
        }
        super::zone_support::routed_fleet(
            graph,
            &[
                (1, true, 4, false),
                (4, true, 1, false),
                (2, true, 6, false),
                (5, true, 3, false),
            ],
            workers,
        )
    };
    assert_eq!(run(1), run(8));
}
