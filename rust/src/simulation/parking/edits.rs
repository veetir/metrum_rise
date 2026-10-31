// SPDX-License-Identifier: GPL-2.0-only

//! Ordered supply reconciliation and towing; undo never copies vehicle state.

use super::*;

impl ParkingSystem {
    /// Removes one bay and its claims; O(1) index work, plus a bounded local tow query.
    /// Never refuses an edit. Off-map cars remain persistent and are indexed by home chunk.
    pub(crate) fn remove_place(&mut self, handle: Handle<Place>) {
        let Some(p) = self.place_handles.resolve(handle) else {
            return;
        };
        let car = self.places.occupant[p];
        let reservation = self.places.reservation[p];
        if !reservation.is_none() {
            self.cancel_reservation(reservation);
        }
        if !car.is_none() {
            self.cancel_reservation(car);
            self.release(car);
        }
        let pos = self.places.position[p];
        let key = chunk_at([pos[0], pos[2]]);
        let slot = self.places.chunk_slot[p];
        let chunk = self.chunks.get_mut(&key).expect("indexed bay");
        chunk.places.swap_remove(slot);
        if let Some(&moved) = chunk.places.get(slot) {
            self.places.chunk_slot[self.place_handles.resolve(moved).expect("live bay")] = slot;
        }
        let source = self.places.source[p];
        let slot = self.places.source_slot[p];
        let list = self.sources.get_mut(&source).expect("indexed source");
        list.swap_remove(slot);
        if let Some(&moved) = list.get(slot) {
            self.places.source_slot[self.place_handles.resolve(moved).expect("live source bay")] =
                slot;
        }
        if list.is_empty() {
            self.sources.remove(&source);
        }
        self.place_handles.swap_remove(p, self.places.len());
        self.places.swap_remove(p);
        self.mark_dirty(key);
        if let Some(v) = self.vehicle_handles.resolve(car) {
            let home = self.vehicles.home[v];
            if let Some(destination) = self.home_place(v) {
                self.occupy(car, destination);
            } else {
                self.stored.entry(chunk_at(home)).or_default().push(car);
            }
        }
    }

    /// Reconciles one owner's geometry, keeping identical poses and live custody. O(old log old + new)
    /// expected map/sort work plus local towing. Called only at topology/asset boundaries.
    pub(crate) fn replace_source(&mut self, source: PlaceSource, bays: &[Bay]) {
        let mut old = self.sources.get(&source).cloned().unwrap_or_default();
        old.sort_unstable();
        let pose_key = |pos: [f32; 3], heading: f32| (pos.map(f32::to_bits), heading.to_bits());
        let mut retained: HashMap<_, _> = old
            .iter()
            .map(|&h| {
                let p = self.place_handles.resolve(h).expect("source bay");
                (pose_key(self.places.position[p], self.places.heading[p]), h)
            })
            .collect();
        for bay in bays {
            if let Some(h) = retained.remove(&pose_key(bay.position, bay.heading)) {
                let p = self.place_handles.resolve(h).expect("retained bay");
                self.places.width[p] = bay.width;
                self.places.length[p] = bay.length;
                self.places.path[p] = bay.path;
            } else {
                self.add_place(source, *bay);
            }
        }
        // Stable handles fix tow order even when load rebuilt different swap-remove lists.
        for h in old {
            let Some(p) = self.place_handles.resolve(h) else {
                continue;
            };
            if retained.contains_key(&pose_key(self.places.position[p], self.places.heading[p])) {
                self.remove_place(h);
            }
        }
        for bay in bays {
            self.rehouse_near([bay.position[0], bay.position[2]]);
        }
    }

    /// Wakes off-map cars only in the nine home chunks near newly freed supply, in handle
    /// order; O(W log W + W × local places), W local waiting cars. Allocates only at this
    /// edit/release boundary; never called from the round-1 simulation tick.
    pub(crate) fn rehouse_near(&mut self, point: [f32; 2]) {
        let (cx, cz) = chunk_at(point);
        let mut waiting = Vec::new();
        for z in cz - 1..=cz + 1 {
            for x in cx - 1..=cx + 1 {
                if let Some(cars) = self.stored.remove(&(x, z)) {
                    waiting.extend(cars);
                }
            }
        }
        waiting.sort_unstable();
        for car in waiting {
            let Some(v) = self.vehicle_handles.resolve(car) else {
                continue;
            };
            let home = self.vehicles.home[v];
            if let Some(place) = self.home_place(v) {
                self.occupy(car, place);
            } else {
                self.stored.entry(chunk_at(home)).or_default().push(car);
            }
        }
    }
}

impl ParkingSystem {
    /// Replaces only this building: O(old log old + old × new anchors + local towing),
    /// bounded by its asset-local swept-path validation, never city size.
    pub(crate) fn replace_building(&mut self, building: u64, bays: &[(u32, Bay)]) {
        let mut old = self.building_sources.remove(&building).unwrap_or_default();
        old.sort_unstable();
        for source in old {
            let PlaceSource::Building { anchor, .. } = source;
            let bay = bays.iter().find(|(i, _)| *i == anchor).map(|(_, b)| *b);
            self.replace_source(source, bay.as_slice());
        }
        for &(anchor, bay) in bays {
            let source = PlaceSource::Building { building, anchor };
            if !self.sources.contains_key(&source) {
                self.add_place(source, bay);
            }
        }
        let sources: Vec<_> = bays
            .iter()
            .map(|&(anchor, _)| PlaceSource::Building { building, anchor })
            .collect();
        if !sources.is_empty() {
            self.building_sources.insert(building, sources);
        }
        // Newly restored owners have no old source list to run replace_source's wake-up.
        // Reconcile waiting cars only after all of this building's supply is available.
        for (_, bay) in bays {
            self.rehouse_near([bay.position[0], bay.position[2]]);
        }
    }
}
