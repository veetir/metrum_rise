// SPDX-License-Identifier: GPL-2.0-only

//! Explicit saved rows, handle generations, reciprocal validation and index reconstruction.

use super::*;
use crate::simulation::core::handles::SavedHandleTable;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct SavedPlace {
    source: PlaceSource,
    bay: SavedBay,
    occupant: u64,
    reservation: u64,
}
#[derive(Serialize, Deserialize)]
struct SavedBay {
    identity: [u64; 2],
    position: [f32; 3],
    heading: f32,
    width: f32,
    length: f32,
}
#[derive(Serialize, Deserialize)]
struct SavedVehicle {
    home: [f32; 2],
    place: u64,
    reserved: u64,
    model: u8,
    colour: u8,
}
#[derive(Serialize, Deserialize)]
struct SavedTable {
    slots: Vec<(u32, Option<u32>)>,
    free: Vec<u32>,
}
#[derive(Serialize, Deserialize)]
struct SavedParking {
    places: Vec<SavedPlace>,
    vehicles: Vec<SavedVehicle>,
    place_handles: SavedTable,
    vehicle_handles: SavedTable,
}

impl SavedTable {
    fn capture<K>(table: &HandleTable<K>) -> Self {
        let SavedHandleTable { slots, free } = table.save();
        Self { slots, free }
    }
    fn restore<K>(self, len: usize) -> Result<HandleTable<K>, String> {
        HandleTable::restore(
            &SavedHandleTable {
                slots: self.slots,
                free: self.free,
            },
            len,
        )
    }
}

impl ParkingSystem {
    /// O(places + vehicles), save boundary only. Source remapping belongs to the save graph map.
    pub(crate) fn save_json(
        &self,
        map: impl Fn(PlaceSource) -> PlaceSource + Sync,
    ) -> Result<String, String> {
        let places = crate::thread_pools::install(|| {
            (0..self.places.len())
                .into_par_iter()
                .map(|p| SavedPlace {
                    source: map(self.places.source[p]),
                    bay: SavedBay {
                        identity: self.places.identity[p],
                        position: self.places.position[p],
                        heading: self.places.heading[p],
                        width: self.places.width[p],
                        length: self.places.length[p],
                    },
                    occupant: self.places.occupant[p].to_bits(),
                    reservation: self.places.reservation[p].to_bits(),
                })
                .collect()
        });
        let vehicles = crate::thread_pools::install(|| {
            (0..self.vehicles.len())
                .into_par_iter()
                .map(|v| SavedVehicle {
                    home: self.vehicles.home[v],
                    place: self.vehicles.place[v].to_bits(),
                    reserved: self.vehicles.reserved[v].to_bits(),
                    model: self.vehicles.model[v],
                    colour: self.vehicles.colour[v],
                })
                .collect()
        });
        // JSON emission and handle-table capture retain their single-writer saved order.
        serde_json::to_string(&SavedParking {
            places,
            vehicles,
            place_handles: SavedTable::capture(&self.place_handles),
            vehicle_handles: SavedTable::capture(&self.vehicle_handles),
        })
        .map_err(|e| e.to_string())
    }

    /// O(places + vehicles), rejects corrupt reciprocal custody before resuming. Rebuilds
    /// source/chunk indices and dirty buffers, without reallocating on later occupy/release.
    pub(crate) fn load_json(json: &str) -> Result<Self, String> {
        let saved: SavedParking = serde_json::from_str(json).map_err(|e| e.to_string())?;
        let mut system = Self {
            place_handles: saved.place_handles.restore(saved.places.len())?,
            vehicle_handles: saved.vehicle_handles.restore(saved.vehicles.len())?,
            ..Self::default()
        };
        for place in saved.places {
            let PlaceSource::Building { building, .. } = place.source;
            {
                let sources = system.building_sources.entry(building).or_default();
                if !sources.contains(&place.source) {
                    sources.push(place.source);
                }
            }
            let p = system.places.len();
            let handle = system.place_handles.handle(p);
            let bay = place.bay;
            if !bay.position.into_iter().all(f32::is_finite)
                || !bay.heading.is_finite()
                || !bay.width.is_finite()
                || !bay.length.is_finite()
                || bay.width <= 0.0
                || bay.length <= 0.0
            {
                return Err("invalid parking bay geometry".into());
            }
            let key = chunk_at([bay.position[0], bay.position[2]]);
            let chunk = system.chunks.entry(key).or_default();
            let list = system.sources.entry(place.source).or_default();
            system.places.push(Place {
                source: place.source,
                identity: bay.identity,
                position: bay.position,
                heading: bay.heading,
                width: bay.width,
                length: bay.length,
                occupant: Handle::from_bits(place.occupant),
                reservation: Handle::from_bits(place.reservation),
                chunk_slot: chunk.places.len(),
                source_slot: list.len(),
            });
            chunk.places.push(handle);
            list.push(handle);
        }
        for car in saved.vehicles {
            if car.model >= 5 || car.colour >= 5 || !car.home.into_iter().all(f32::is_finite) {
                return Err("invalid parked vehicle".into());
            }
            system.vehicles.push(Vehicle {
                home: car.home,
                place: Handle::from_bits(car.place),
                reserved: Handle::from_bits(car.reserved),
                model: car.model,
                colour: car.colour,
            });
        }
        for p in 0..system.places.len() {
            let handle = system.place_handles.handle(p);
            for (car, occupied) in [
                (system.places.occupant[p], true),
                (system.places.reservation[p], false),
            ] {
                if car.is_none() {
                    continue;
                }
                let v = system
                    .vehicle_handles
                    .resolve(car)
                    .ok_or("stale parking vehicle")?;
                if (if occupied {
                    system.vehicles.place[v]
                } else {
                    system.vehicles.reserved[v]
                }) != handle
                {
                    return Err("nonreciprocal parking place".into());
                }
            }
            if !system.places.occupant[p].is_none() && !system.places.reservation[p].is_none() {
                return Err("occupied place also reserved".into());
            }
        }
        for v in 0..system.vehicles.len() {
            let car = system.vehicle_handles.handle(v);
            for (handle, occupied) in [
                (system.vehicles.place[v], true),
                (system.vehicles.reserved[v], false),
            ] {
                if handle.is_none() {
                    continue;
                }
                let p = system
                    .place_handles
                    .resolve(handle)
                    .ok_or("stale vehicle place")?;
                if (if occupied {
                    system.places.occupant[p]
                } else {
                    system.places.reservation[p]
                }) != car
                {
                    return Err("nonreciprocal parked vehicle".into());
                }
            }
            if system.vehicles.place[v].is_none() && system.vehicles.reserved[v].is_none() {
                system
                    .stored
                    .entry(chunk_at(system.vehicles.home[v]))
                    .or_default()
                    .push(car);
            }
        }
        system.dirty.reserve(system.chunks.len());
        let mut keys: Vec<_> = system.chunks.keys().copied().collect();
        keys.sort_unstable();
        for key in keys {
            system.mark_dirty(key);
        }

        Ok(system)
    }
}

impl ParkingSystem {
    /// Load validation is O(places), never a tick path; dangling geometry owners reject the save.
    pub(crate) fn validate_sources(
        &self,
        allocator: &crate::simulation::buildings::allocator::BuildingAllocator,
    ) -> Result<(), String> {
        for source in &self.places.source {
            let valid = match *source {
                PlaceSource::Building { building, .. } => allocator
                    .building_handles
                    .resolve(Handle::from_bits(building))
                    .is_some(),
            };
            if !valid {
                return Err(format!("parking place has a missing source: {source:?}"));
            }
        }
        Ok(())
    }
}
