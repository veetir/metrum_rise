// SPDX-License-Identifier: GPL-2.0-only

//! Player over/under profiles; topology, preview, commit and undo use the ordinary road tool.

use super::*;

#[godot_api(secondary)]
impl SimulationNode {
    /// Solves a crossing's grounded approaches and clearance in Rust. The returned points
    /// and layer must be supplied unchanged to the existing road preview/commit entry points.
    /// Refuses strokes with insufficient approach length instead of moving their endpoints.
    #[func]
    pub fn prepare_road_crossing(
        &self,
        points: PackedVector3Array,
        forward: i32,
        backward: i32,
        over: bool,
    ) -> VarDictionary {
        self.crossing_profile(points, forward, backward, over, None)
    }

    /// Solves clearance only against roads beyond the tool's explicitly selected level.
    /// Same-level intersections retain ordinary junction topology and the selected level.
    #[func]
    pub fn prepare_layered_road_crossing(
        &self,
        points: PackedVector3Array,
        forward: i32,
        backward: i32,
        over: bool,
    ) -> VarDictionary {
        self.crossing_profile(
            points,
            forward,
            backward,
            over,
            Some(self.road_tool_crossing_level),
        )
    }

    fn crossing_profile(
        &self,
        points: PackedVector3Array,
        forward: i32,
        backward: i32,
        over: bool,
        source_level: Option<i16>,
    ) -> VarDictionary {
        if !(0..=4).contains(&forward) || !(0..=4).contains(&backward) {
            return vdict! { "is_valid": false, "invalid_reason": "invalid_cross_section" };
        }
        let core = self.lock_core();
        match core.transit_network.road_surface.prepare_crossing_profile(
            &core.region_graph,
            points.as_slice(),
            (forward as u8, backward as u8),
            self.road_tool_median,
            over,
            source_level,
        ) {
            Ok(profile) => vdict! {
                "is_valid": true,
                "points": PackedVector3Array::from(profile.points.as_slice()),
                "crossing_level": i32::from(profile.level),
                "crossed_edges": PackedInt64Array::from(profile.edges.iter().map(|&id| id as i64).collect::<Vec<_>>().as_slice()),
            },
            Err(reason) => vdict! { "is_valid": false, "invalid_reason": reason },
        }
    }
}
