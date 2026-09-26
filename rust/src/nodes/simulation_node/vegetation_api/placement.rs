// SPDX-License-Identifier: GPL-2.0-only

//! Bounded brush proposals and class-local occupancy over the existing edit cells.

use super::*;
use brush::PlantClass;

const ATTEMPTS: usize = 2;

// Candidate identity is (class, signed cell x/z, attempt). All properties have distinct salts.
fn dart(x: i32, z: i32, attempt: usize, class: PlantClass, seed: u32) -> (Plant, u32) {
    let salt = layer_base(128 + class as u32 * 64 + attempt as u32 * 16, seed);
    let step = class.spacing() * std::f32::consts::FRAC_1_SQRT_2;
    (
        Plant {
            x: (x as f32 + unit(x, z, salt.wrapping_add(1))) * step,
            z: (z as f32 + unit(x, z, salt.wrapping_add(2))) * step,
            yaw: unit(x, z, salt.wrapping_add(3)) * std::f32::consts::TAU,
            scale: 0.75 + unit(x, z, salt.wrapping_add(4)) * 0.6,
            species: 0,
            variant: VARIANT_FROM_SEED,
        },
        salt,
    )
}

// The outer fifth fades with smoothstep. The threshold is fixed per dart, so stamps do not
// reroll density. A later stamp can expose a previous edge to its full interior influence.
fn influence(plant: &Plant, pos: Vector2, radius: f32) -> f32 {
    if radius == 0.0 {
        return if in_disc(plant, pos, radius) {
            1.0
        } else {
            0.0
        };
    }
    let distance = ((plant.x - pos.x).powi(2) + (plant.z - pos.y).powi(2)).sqrt();
    let t = ((radius - distance) / (radius * 0.2)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Uses species clearance while keeping the authored canopy owner cell.
pub(super) fn authored_clear(core: &SimCore, plant: &Plant) -> bool {
    placement_clear(
        core,
        plant.x,
        plant.z,
        PlantClass::of(plant.species).clearance_layer(),
    )
}

// Both layers can own authored plants. Generated neighbors are evaluated locally and honor
// tombstones; hidden authored entries reserve their space because they can become visible again.
fn occupied(core: &SimCore, plant: &Plant) -> bool {
    let class = PlantClass::of(plant.species);
    let radius = class.spacing();
    let pos = Vector2::new(plant.x, plant.z);
    let conflicts = |other: &Plant| {
        PlantClass::of(other.species) == class
            && (other.x - plant.x).powi(2) + (other.z - plant.z).powi(2) < radius * radius
    };
    for layer in [VegetationLayer::Canopy, VegetationLayer::Understory] {
        let (step, salt) = grid(core, layer);
        let first = cell_at(pos - Vector2::splat(radius), layer, step);
        let last = cell_at(pos + Vector2::splat(radius), layer, step);
        for z in first.z..=last.z {
            for x in first.x..=last.x {
                let cell = VegetationCell { layer, x, z };
                let (removed, added) = core.vegetation_edits.cell(cell);
                if added.iter().any(conflicts) {
                    return true;
                }
                if !removed
                    && layer == class.clearance_layer()
                    && generated_candidate(core, cell, step, salt)
                        .filter(conflicts)
                        .is_some_and(|p| placement_clear(core, p.x, p.z, layer))
                {
                    return true;
                }
            }
        }
    }
    false
}

/// Places one explicit plant subject to the same occupancy as a brush.
pub(super) fn add_at(core: &mut SimCore, pos: Vector2, option: i64) -> bool {
    let Some(preset) = preset(option) else {
        return false;
    };
    if !valid_disc(core, pos, 0.0) {
        return false;
    }
    prepare_sites(core);
    let (cell_m, salt) = grid(core, VegetationLayer::Canopy);
    let cell = cell_at(pos, VegetationLayer::Canopy, cell_m);
    let [_, _, yaw, scale] = candidate(cell.x, cell.z, cell_m, salt);
    let (species, variant) = preset.plant(cell.x, cell.z, salt);
    let plant = Plant {
        x: pos.x,
        z: pos.y,
        yaw,
        scale: preset.size(scale),
        species,
        variant,
    };
    if !authored_clear(core, &plant) || occupied(core, &plant) {
        return false;
    }
    let key = PatchLayout::new(core).key(pos.x, pos.y);
    let mut undo = VegetationEditUndo::for_stroke(0);
    undo.record_cell(cell, core.vegetation_edits.snapshot_cell(cell));
    undo.record_patch(key);
    core.vegetation_edits.add(cell, plant);
    core.vegetation_edits.bump_patch(key);
    core.push_vegetation_undo(undo);
    true
}

struct Proposal {
    plant: Plant,
    owner: VegetationCell,
    restore: Option<VegetationCell>,
    // Restoration/replacement sites precede new darts, then hash and integer tie-breakers.
    order: (u8, u32, u8, i32, i32, usize),
}

/// Evaluates bounded proposals in parallel, then accepts in canonical priority order.
pub(super) fn paint_at(
    core: &mut SimCore,
    pos: Vector2,
    radius: f32,
    option: i64,
    stroke: i64,
) -> usize {
    let Some(preset) = preset(option) else {
        return 0;
    };
    let class = preset.class();
    if !valid_disc(core, pos, radius) || radius > class.max_radius() {
        return 0;
    }
    prepare_sites(core);
    let seed = core.vegetation.config.seed;
    let owner_step = core.vegetation.canopy_cell_m;
    let step = class.spacing() * std::f32::consts::FRAC_1_SQRT_2;
    let cells = disc_cells(pos, radius, step, VegetationLayer::Canopy);
    let first = cell_at(pos - Vector2::splat(radius), VegetationLayer::Canopy, step);
    let last = cell_at(pos + Vector2::splat(radius), VegetationLayer::Canopy, step);
    let nx = last.x - first.x + 1;
    let mut plans: Vec<_> = (0..cells.len() * ATTEMPTS)
        .into_par_iter()
        .filter_map(|i| {
            let x = first.x + (i / ATTEMPTS) as i32 % nx;
            let z = first.z + (i / ATTEMPTS) as i32 / nx;
            let attempt = i % ATTEMPTS;
            let (mut plant, salt) = dart(x, z, attempt, class, seed);
            let clump = 0.25
                + 1.5
                    * crate::simulation::vegetation::value_noise(
                        plant.x / 40.0,
                        plant.z / 40.0,
                        layer_base(0x51a7_32b9, seed),
                    );
            if !preset.keeps(x, z, salt, clump * influence(&plant, pos, radius)) {
                return None;
            }
            (plant.species, plant.variant) = preset.plant(x, z, salt);
            plant.scale = preset.size(plant.scale);
            if !authored_clear(core, &plant) {
                return None;
            }
            Some(Proposal {
                owner: cell_at(
                    Vector2::new(plant.x, plant.z),
                    VegetationLayer::Canopy,
                    owner_step,
                ),
                plant,
                restore: None,
                order: (
                    1,
                    hash(x, z, salt.wrapping_add(10)),
                    class as u8,
                    z,
                    x,
                    attempt,
                ),
            })
        })
        .collect();
    // Only tombstones propose generator positions. Empty generator slots no longer add a
    // second lattice to the brush, and restore/replacement retains the original exact site.
    for layer in [VegetationLayer::Canopy, VegetationLayer::Understory] {
        if layer == VegetationLayer::Understory && class == PlantClass::Tree {
            continue;
        }
        let (cell_m, salt) = grid(core, layer);
        let restores: Vec<_> = disc_cells(pos, radius, cell_m, layer)
            .filter_map(|cell| {
                if !core.vegetation_edits.cell(cell).0 {
                    return None;
                }
                let [x, z, yaw, scale] = candidate(cell.x, cell.z, cell_m, salt);
                let (species, variant) = preset.plant(cell.x, cell.z, salt);
                let authored = Plant {
                    x,
                    z,
                    yaw,
                    scale: preset.size(scale),
                    species,
                    variant,
                };
                if !in_disc(&authored, pos, radius) {
                    return None;
                }
                let generated = evaluate_cell(core, cell, cell_m, salt);
                if layer == VegetationLayer::Understory
                    && generated.is_none_or(|p| PlantClass::of(p.species) != class)
                {
                    return None;
                }
                let restore =
                    generated.filter(|p| p.species == species && variant == VARIANT_FROM_SEED);
                let plant = restore.unwrap_or(authored);
                if !authored_clear(core, &plant) {
                    return None;
                }
                Some(Proposal {
                    plant,
                    owner: cell_at(Vector2::new(x, z), VegetationLayer::Canopy, owner_step),
                    restore: restore.map(|_| cell),
                    order: (
                        0,
                        hash(cell.x, cell.z, salt.wrapping_add(10)),
                        layer as u8,
                        cell.z,
                        cell.x,
                        0,
                    ),
                })
            })
            .collect();
        plans.extend(restores);
    }
    if plans.is_empty() {
        return 0;
    }
    // Reserve once per owner cell, outside acceptance, avoiding vector growth per candidate.
    plans.sort_unstable_by_key(|p| (p.owner.z, p.owner.x));
    let layout = PatchLayout::new(core);
    core.vegetation_edits
        .reserve(plans.len(), layout.reserve_count(radius));
    let mut start = 0;
    while start < plans.len() {
        let cell = plans[start].owner;
        let end = start + plans[start..].partition_point(|p| p.owner == cell);
        core.vegetation_edits.reserve_added(cell, end - start);
        start = end;
    }
    plans.sort_unstable_by_key(|p| p.order);
    let mut undo = VegetationEditUndo::for_stroke(stroke);
    let mut count = 0;
    // Acceptance is deliberately serial: each accepted plant constrains later proposals.
    for plan in &plans {
        if occupied(core, &plan.plant) {
            continue;
        }
        let cell = plan.restore.unwrap_or(plan.owner);
        undo.record_cell_with(cell, || core.vegetation_edits.snapshot_cell(cell));
        if plan.restore.is_some() {
            core.vegetation_edits.restore_reserved(cell);
        } else {
            core.vegetation_edits.add(cell, plan.plant);
        }
        let key = layout.key(plan.plant.x, plan.plant.z);
        core.vegetation_edits.bump_patch(key);
        undo.record_patch(key);
        count += 1;
    }
    // Empty reservation entries are not persistent edits, including in the undo snapshots.
    for plan in &plans {
        core.vegetation_edits.prune_empty(plan.owner);
        if let Some(cell) = plan.restore {
            core.vegetation_edits.prune_empty(cell);
        }
    }
    core.push_vegetation_undo(undo);
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn darts_fill_their_cells_with_stable_independent_properties() {
        for class in [PlantClass::Tree, PlantClass::Ground, PlantClass::Rock] {
            let step = class.spacing() * std::f32::consts::FRAC_1_SQRT_2;
            let mut range = (1.0_f32, 0.0_f32);
            for x in -100..100 {
                for attempt in 0..ATTEMPTS {
                    let (p, _) = dart(x, -7, attempt, class, 123);
                    assert_eq!(dart(x, -7, attempt, class, 123).0, p);
                    assert_ne!(dart(x, -7, attempt, class, 124).0, p);
                    assert!(p.x >= x as f32 * step && p.x < (x + 1) as f32 * step);
                    assert!(p.z >= -7.0 * step && p.z < -6.0 * step);
                    let fraction = p.x / step - x as f32;
                    range = (range.0.min(fraction), range.1.max(fraction));
                    assert!((0.75..1.35).contains(&p.scale));
                    assert!((0.0..std::f32::consts::TAU).contains(&p.yaw));
                }
            }
            assert!(range.0 < 0.05 && range.1 > 0.95);
        }
    }
}
