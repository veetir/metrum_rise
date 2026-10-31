// SPDX-License-Identifier: GPL-2.0-only

//! Thin parking inspector, debug fleet and versioned parked-render bridge.

use super::*;

#[godot_api(secondary)]
impl SimulationNode {
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
        pack_parked(epoch, chunks)
    }

    /// Returns moving and parked payloads from the same custody publication. If a newer
    /// parking transition has overtaken the moving snapshot, both renderers retain their
    /// previous frame. O(visible cars + dirty parked chunks), nonblocking on the simulation.
    #[func]
    pub fn get_vehicle_frame(&self, full: bool) -> VarDictionary {
        let mut result = VarDictionary::new();
        result.set("busy", true);
        let snapshot = self.snapshot.read().unwrap();
        let Some(mut core) = self.try_lock_core() else {
            return result;
        };
        if snapshot.parking_epoch != core.parking.publication_epoch()
            || snapshot.vehicle_world_epoch != core.world_epoch
        {
            return result;
        }
        core.build_parked_chunks();
        let chunks = core.parking.publications(full);
        let epoch = core.world_epoch;
        drop(core);
        result.set(
            "cars",
            crate::nodes::sim::bridge::agents::get_car_render_data(&snapshot),
        );
        result.set("parked", pack_parked(epoch, chunks));
        result.set("busy", false);
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

fn pack_parked(
    epoch: u64,
    chunks: Vec<(
        (i32, i32),
        std::sync::Arc<crate::simulation::parking::ParkedChunk>,
    )>,
) -> VarDictionary {
    let mut result = VarDictionary::new();
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
