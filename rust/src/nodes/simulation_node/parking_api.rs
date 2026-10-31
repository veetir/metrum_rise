// SPDX-License-Identifier: GPL-2.0-only

//! Thin parking inspector, debug fleet and versioned parked-render bridge.

use super::*;
use crate::simulation::parking::SideRule;

#[godot_api(secondary)]
impl SimulationNode {
    /// Sets one edge side: side 0 = right, 1 = left; value 0/1/2 = none/unmarked/marked.
    /// O(local geometry); edits tow, never reject because of vehicle custody.
    #[func]
    pub fn set_parking_side(&mut self, edge: i32, side: i32, value: i32) {
        if edge < 0 || !(0..=1).contains(&side) {
            return;
        }
        let rule = match value {
            0 => SideRule::None,
            1 => SideRule::Unmarked,
            2 => SideRule::Marked,
            _ => return,
        };
        self.lock_core()
            .set_parking_side(edge as usize, side as usize, rule);
    }

    /// Saved permission in O(1), using the same side/value codes as set_parking_side.
    #[func]
    pub fn get_parking_side(&self, edge: i32, side: i32) -> i32 {
        if edge < 0 || !(0..=1).contains(&side) {
            return 0;
        }
        self.lock_core()
            .region_graph
            .parking_rule(edge as usize)
            .sides[side as usize] as i32
    }

    /// Explicit debug/test command, O(places), never part of ordinary gameplay initialization.
    #[func]
    pub fn debug_fill_parking(&mut self) -> i32 {
        let mut core = self.lock_core();
        core.publish_pending_building_site_changes();
        let count = core.parking.fill_test_fleet();
        core.build_parked_chunks();
        count as i32
    }

    /// Local inspection counts and rejected asset-layout diagnostics; no simulation decisions.
    #[func]
    pub fn get_parking_info(&self) -> VarDictionary {
        let core = self.lock_core();
        let (places, vehicles, stored) = core.parking.counts();
        let mut result = VarDictionary::new();
        result.set("places", places as i64);
        result.set("vehicles", vehicles as i64);
        result.set("stored", stored as i64);
        result.set(
            "diagnostics",
            PackedStringArray::from_iter(
                core.parking
                    .diagnostics
                    .values()
                    .flatten()
                    .map(GString::from),
            ),
        );
        result
    }

    /// Quotes foundation supply near a known frontage, without reserving or changing agent trips.
    #[func]
    pub fn find_parking_near_frontage(&self, point: Vector2, edge: i32) -> Variant {
        if edge < 0 {
            return Variant::nil();
        }
        let core = self.lock_core();
        core.parking
            .nearest_frontage(point.to_array(), edge as usize, &core.region_graph)
            .and_then(|place| core.parking.position(place))
            .map(|pos| Vector3::from_array(pos).to_variant())
            .unwrap_or_else(Variant::nil)
    }

    /// Nonblocking O(pending dirty payload); unchanged chunks send nothing. Full reconnects
    /// explicitly request all chunks. Payloads survive skipped frames until exact-version ack.
    #[func]
    pub fn get_parked_car_chunks(&self, full: bool) -> VarDictionary {
        let mut result = VarDictionary::new();
        result.set("busy", true);
        let Some(mut core) = self.try_lock_core() else {
            return result;
        };
        core.build_parked_chunks();
        let epoch = core.world_epoch;
        let chunks = core.parking.publications(full);
        drop(core);
        result.set("busy", false);
        result.set("epoch", epoch as i64);
        let mut updates = VarArray::new();
        for (key, chunk) in chunks {
            let mut update = VarDictionary::new();
            update.set("chunk", Vector2i::new(key.0, key.1));
            update.set("version", chunk.version as i64);
            let mut buckets = VarDictionary::new();
            let mut identities = VarDictionary::new();
            for (&bucket, buffer) in &chunk.buckets {
                buckets.set(bucket as i32, PackedFloat32Array::from(buffer.as_slice()));
            }
            for (&bucket, ids) in &chunk.identities {
                identities.set(
                    bucket as i32,
                    PackedInt64Array::from_iter(ids.iter().map(|&id| id as i64)),
                );
            }
            update.set("buckets", buckets);
            update.set("identities", identities);
            updates.push(&update.to_variant());
        }
        result.set("chunks", updates);
        result
    }

    /// O(log pending chunks); stale world/version acknowledgements cannot discard a newer pose.
    #[func]
    pub fn acknowledge_parked_car_chunk(&mut self, epoch: i64, key: Vector2i, version: i64) {
        let Some(mut core) = self.try_lock_core() else {
            return;
        };
        if core.world_epoch == epoch as u64 {
            core.parking.acknowledge((key.x, key.y), version as u64);
        }
    }
}
