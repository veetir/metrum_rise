// SPDX-License-Identifier: GPL-2.0-only

//! One-time conversion of pre-scheduler references, deadlines and accounting windows.

use super::{SaveLoadError, SaveLoadResult};
use rusqlite::Connection;

// Dense ids become generation-zero handles in the same appended order as SIM-02-B.
// The old shopper seed guard prevents a reused dense slot from binding the wrong agent.
pub(super) fn restore_dense_agent_references(
    conn: &Connection,
    households: &mut crate::simulation::economy::households::HouseholdSystem,
    logistics: &mut crate::simulation::economy::logistics::ShipmentSystem,
    agents: &crate::simulation::economy::agents::AgentSystem,
) -> SaveLoadResult<()> {
    use crate::simulation::core::handles::Handle;
    let mut stmt = conn.prepare("SELECT household_id, shopping_agent_id, shopping_agent_schedule_seed FROM households ORDER BY household_id")?;
    let mut rows = stmt.query([])?;
    while let Some(row) = rows.next()? {
        let household = super::i64_to_usize(row.get(0)?)?;
        let dense = row.get::<_, i64>(1)?;
        let seed = super::i64_to_u32(row.get(2)?)?;
        households.households[household].shopping_agent = if dense >= 0
            && (dense as usize) < agents.len()
            && agents.schedule_seed[dense as usize] == seed
        {
            agents.handles.handle(dense as usize)
        } else {
            Handle::NONE
        };
    }
    for shipment in &mut logistics.shipments {
        if !shipment.carrier.is_none() {
            let dense = shipment.carrier.index() as usize;
            if dense >= agents.len() || agents.freight_shipment_id[dense] != shipment.id {
                return Err(SaveLoadError::custom(
                    "legacy shipment carrier does not name its shipment",
                ));
            }
            shipment.carrier = agents.handles.handle(dense);
        }
    }
    Ok(())
}

fn event(scheduler: &mut super::SavedScheduler, slot: u64, kind: u16, a: u64) {
    scheduler.events.push((
        slot.max(scheduler.next_slot),
        scheduler.next_event_seq,
        kind,
        a,
    ));
    scheduler.next_event_seq += 1;
}

// SIM-02-C's construction queue pacing and SIM-02-D's generation-keyed completions.
// This is load-only O(queued spawns + buildings), not work in the frame loop.
pub(super) fn restore_construction_events(
    scheduler: &mut super::SavedScheduler,
    pending: &mut std::collections::VecDeque<crate::nodes::sim::core::PendingDemandSpawnAction>,
    allocator: &crate::simulation::buildings::allocator::BuildingAllocator,
) {
    use crate::simulation::core::scheduler::SLOTS_PER_MINUTE;
    let mut minute = (scheduler.next_slot - 1) / SLOTS_PER_MINUTE;
    for spawn in pending {
        spawn.due_minute = spawn.due_minute.max(minute + 1);
        minute = spawn.due_minute;
        scheduler.next_spawn_id = scheduler.next_spawn_id.max(spawn.id + 1);
        event(scheduler, minute * SLOTS_PER_MINUTE, 0, spawn.id);
    }
    restore_completion_events(scheduler, allocator);
}

pub(super) fn restore_completion_events(
    scheduler: &mut super::SavedScheduler,
    allocator: &crate::simulation::buildings::allocator::BuildingAllocator,
) {
    for (dense, building) in allocator.buildings.iter().enumerate() {
        if building.is_under_construction() {
            event(
                scheduler,
                building.construction_complete_slot,
                1,
                allocator.building_handles.handle(dense).to_bits(),
            );
        }
    }
}

// Before SIM-02-E, the operational hour polled these trips. Continue each live trip at
// the next slot, using exactly the event kinds SIM-02-E introduced, rather than dropping it.
pub(super) fn restore_trip_events(
    scheduler: &mut super::SavedScheduler,
    households: &mut crate::simulation::economy::households::HouseholdSystem,
    logistics: &mut crate::simulation::economy::logistics::ShipmentSystem,
    agents: &crate::simulation::economy::agents::AgentSystem,
) -> SaveLoadResult<()> {
    use crate::simulation::economy::agents::ACTIVITY_SHOPPING;
    for shipment in &mut logistics.shipments {
        if shipment.status.is_open()
            && (shipment.status == crate::simulation::economy::logistics::ShipmentStatus::Returning
                || !shipment.carrier.is_none())
        {
            shipment.cargo_loaded =
                shipment.status == crate::simulation::economy::logistics::ShipmentStatus::InTransit;
            shipment.trip_event_slot = scheduler.next_slot;
            event(scheduler, scheduler.next_slot, 3, shipment.id);
        }
    }
    for (dense, household) in households.households.iter().enumerate() {
        if !household.shopping_agent.is_none() {
            event(
                scheduler,
                scheduler.next_slot,
                5,
                households.handles.handle(dense).to_bits(),
            );
        }
    }
    for dense in 0..agents.len() {
        if agents.pending_household_size[dense] > 0 {
            event(
                scheduler,
                scheduler.next_slot,
                9,
                agents.handles.handle(dense).to_bits(),
            );
        } else if agents.activity[dense] == ACTIVITY_SHOPPING
            && !households
                .households
                .get(agents.household_id[dense])
                .is_some_and(|household| household.shopping_agent == agents.handles.handle(dense))
        {
            event(
                scheduler,
                scheduler.next_slot,
                7,
                agents.handles.handle(dense).to_bits(),
            );
        }
    }
    Ok(())
}

// v73–92 and road v88–95 retained already collected demolition bills and input purchases
// in one record. SIM-02-G moved input purchases into the city-day journal. Decode the full
// original record (including its validation), not a permissive subset of its JSON.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct LegacyRemovedElectricity {
    settled: bool,
    pub(super) settlement: crate::simulation::economy::households::DailyPowerSettlementSummary,
    city_input_cost: f64,
    power_input_cost: f64,
    coal_consumed_kg: f64,
}

pub(super) fn load_removed_electricity(json: &str) -> SaveLoadResult<LegacyRemovedElectricity> {
    let record: LegacyRemovedElectricity =
        serde_json::from_str(json).map_err(|e| SaveLoadError::custom(e.to_string()))?;
    if !record.city_input_cost.is_finite()
        || !record.power_input_cost.is_finite()
        || !record.coal_consumed_kg.is_finite()
        || record.coal_consumed_kg < 0.0
    {
        return Err(SaveLoadError::custom(
            "invalid legacy electricity input account",
        ));
    }
    Ok(record)
}

pub(super) fn restore_accounting(
    conn: &Connection,
    households: &mut crate::simulation::economy::households::HouseholdSystem,
    allocator: &mut crate::simulation::buildings::allocator::BuildingAllocator,
) -> SaveLoadResult<()> {
    let json: String = conn.query_row(
        "SELECT removed_buildings FROM electricity_day_state WHERE singleton = 1",
        [],
        |r| r.get(0),
    )?;
    let removed = load_removed_electricity(&json)?;
    if !households.removed_electricity.is_valid() {
        return Err(SaveLoadError::custom(
            "invalid legacy electricity settlement",
        ));
    }
    // A save after midnight settlement must not report the collected demolition bill twice.
    if removed.settled {
        households.removed_electricity = Default::default();
    }
    allocator.city_input_day.total = removed.city_input_cost;
    allocator.city_input_day.power = removed.power_input_cost;
    households.day_totals.coal_consumed_kg = removed.coal_consumed_kg;
    let catalog = crate::simulation::economy::definitions::load_runtime_economy_catalog()
        .map_err(SaveLoadError::custom)?;
    let coal = catalog.resource_runtime_id_for_id("coal");
    households.coal_inventory_kg = 0.0;
    // Old fiscal reporting used this same dense-order fold, including the removed providers.
    for building in &allocator.buildings {
        let input = f64::from(building.daily_city_funded_input_cost);
        allocator.city_input_day.total += input;
        let Some(profile) = catalog.profile_by_runtime_id(building.economy_profile_runtime_id)
        else {
            continue;
        };
        if profile.utility_service.as_deref() != Some("power") {
            continue;
        }
        allocator.city_input_day.power += input;
        if let Some(coal) = coal {
            households.coal_inventory_kg += f64::from(building.inventory_units(coal).max(0.0));
        }
        for port in &profile.inputs {
            if Some(port.resource_runtime_id) == coal
                && profile.base_rate_units_per_day > f32::EPSILON
            {
                households.day_totals.coal_consumed_kg += f64::from(
                    port.units_per_day.max(0.0) * building.daily_power_service_units.max(0.0)
                        / profile.base_rate_units_per_day,
                );
            }
        }
    }
    Ok(())
}

// The pre-event hourly pass also dispatched jobs with no carrier yet. Reuse the same
// authoritative cargo loader as new request queues; do not fabricate cargo or leave jobs idle.
pub(super) fn restore_unstarted_freight(
    scheduler: &mut super::SavedScheduler,
    logistics: &mut crate::simulation::economy::logistics::ShipmentSystem,
    allocator: &mut crate::simulation::buildings::allocator::BuildingAllocator,
    treasury: &mut f64,
) -> SaveLoadResult<()> {
    let catalog = crate::simulation::economy::definitions::load_runtime_economy_catalog()
        .map_err(SaveLoadError::custom)?;
    let tuning = crate::simulation::economy::definitions::load_runtime_economy_tuning()
        .map_err(SaveLoadError::custom)?;
    allocator.now_slot = scheduler.next_slot - 1;
    logistics.load_new_shipments(
        0,
        allocator,
        treasury,
        tuning.operational_clock.shipment_retry_cooldown_hours,
        &catalog,
    );
    for id in std::mem::take(&mut logistics.held_dispatches) {
        if let Some(index) = logistics.shipment_index(id) {
            logistics.shipments[index].trip_event_slot = scheduler.next_slot;
            event(scheduler, scheduler.next_slot, 2, id);
        }
    }
    Ok(())
}
