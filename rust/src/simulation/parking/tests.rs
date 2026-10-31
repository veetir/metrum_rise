// SPDX-License-Identifier: GPL-2.0-only

//! Local store, query, towing and publication regressions.

use super::*;
fn bay(x: f32, z: f32) -> Bay {
    Bay {
        identity: [0, 0],
        position: [x, 0.0, z],
        heading: 0.0,
        width: 2.5,
        length: 6.0,
    }
}
fn source(edge: usize) -> PlaceSource {
    PlaceSource::Edge { edge, side: 1 }
}

#[test]
fn query_ranks_walk_then_drive_then_handle_with_400m_bound() {
    let mut s = ParkingSystem::default();
    let a = s.add_place(source(0), bay(-1.0, 0.0));
    let b = s.add_place(source(0), bay(1.0, 0.0));
    s.add_place(source(1), bay(400.01, 0.0));
    assert_eq!(s.nearest([0.0, 0.0]), Some(a));
    assert_eq!(
        s.nearest_with([0.0, 0.0], |h, _| Some(ParkingCost {
            walk_m: 1.0,
            walk_s: 1.0,
            drive_s: if h == b { 0.0 } else { 1.0 }
        })),
        Some(b)
    );
    assert!(
        s.nearest_with([0.0, 0.0], |_, _| Some(ParkingCost {
            walk_m: 400.1,
            walk_s: 1.0,
            drive_s: 0.0
        }))
        .is_none()
    );
    let c = s.add_place(source(2), bay(0.0, 400.0));
    assert_eq!(s.nearest([0.0, 800.0]), Some(c));
    assert!(s.nearest([0.0, 800.01]).is_none());
}

#[test]
fn towing_undo_and_reservations_conserve_cars() {
    let mut s = ParkingSystem::default();
    let a = s.add_place(source(0), bay(0.0, 0.0));
    s.fill_test_fleet();
    let car = s.vehicle_handles.handle(0);
    let b = s.add_place(source(1), bay(12.0, 0.0));
    s.remove_place(a);
    assert_eq!(s.vehicles.place[0], b);
    s.remove_place(b);
    assert!(s.vehicles.place[0].is_none());
    s.replace_source(source(0), &[bay(0.0, 0.0)]);
    assert!(!s.vehicles.place[0].is_none());
    assert_eq!(s.vehicles.len(), 1);
    let free = s.add_place(source(2), bay(20.0, 0.0));
    assert!(s.reserve(car, free));
    assert!(!s.reserve(car, free));
    s.remove_place(free);
    assert!(s.vehicles.reserved[0].is_none());
    assert!(!s.occupy(car, s.vehicles.place[0]));
    assert_eq!(s.place_handles.resolve(a), None);
}

#[test]
fn save_round_trip_and_chunk_acknowledgements() {
    let mut s = ParkingSystem::default();
    s.add_place(source(0), bay(-512.0, 4.0));
    s.fill_test_fleet();
    s.build_dirty_chunks();
    let key = (-1, 0);
    let first = s.pending[&key].clone();
    s.build_dirty_chunks();
    assert!(Arc::ptr_eq(&first, &s.pending[&key]));
    let json = s.save_json(|s| s).unwrap();
    let mut loaded = ParkingSystem::load_json(&json).unwrap();
    loaded.build_dirty_chunks();
    assert_eq!(loaded.save_json(|s| s).unwrap(), json);
    assert_eq!(loaded.pending[&key].buckets, first.buckets);
    let pos = s.release(s.vehicle_handles.handle(0)).unwrap();
    s.rehouse_near(pos);
    s.build_dirty_chunks();
    s.acknowledge(key, first.version);
    assert!(!s.pending.is_empty());
    let version = s.pending[&key].version;
    s.acknowledge(key, version);
    assert!(s.pending.is_empty());
    s.build_dirty_chunks();
    assert!(s.pending.is_empty());
    let broken = json.replace("\"occupant\":0", "\"occupant\":4294967296");
    assert!(ParkingSystem::load_json(&broken).is_err());
}
