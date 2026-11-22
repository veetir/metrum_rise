// SPDX-License-Identifier: GPL-3.0-or-later

//! Gameplay brush path sampling, local terrain protection and stroke history.

use super::{SlopeAnchors, config};
use crate::nodes::sim::core::SimCore;
use crate::simulation::terrain::{TerrainSystem, rect_clip::clip_triangle_to_rect};
use godot::prelude::{Vector2, Vector3};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

const MAX_DAB_INTERVAL_SEC: f64 = 1.0 / 240.0;
const RAISE_PER_DIAMETER: [f32; 3] = [0.03, 0.10, 0.30];
const TARGET_SECONDS: [f32; 3] = [2.0, 0.7, 0.2];
const ROAD: u8 = 1;
const BUILDING: u8 = 2;
const LOT: u8 = 4;
const FIELD: u8 = 8;
const OUTSIDE: i32 = 16;
const WATER: i32 = 32;
const ALL_PROTECTED: i32 = 64;

type GridRect = [usize; 4];
type PatchKey = (usize, usize);

/// The five gameplay brush modes, in palette order.
#[derive(Clone, Copy)]
pub(crate) enum TerrainBrushMode {
    /// Increase the source height.
    Raise,
    /// Decrease the source height.
    Lower,
    /// Approach the height captured at press.
    Flatten,
    /// Approach the local source-height average.
    Smooth,
    /// Approach the clamped linear grade between two captured anchors.
    Ramp,
}

impl TerrainBrushMode {
    /// Decodes the gameplay palette index at the Godot boundary.
    pub(crate) fn from_index(index: i32) -> Option<Self> {
        Some(match index {
            0 => Self::Raise,
            1 => Self::Lower,
            2 => Self::Flatten,
            3 => Self::Smooth,
            4 => Self::Ramp,
            _ => return None,
        })
    }
}

/// The source samples of the changed rectangle in both history directions.
pub(crate) struct TerrainStrokeHeights {
    bounds: GridRect,
    before: Arc<[f32]>,
    after: Arc<[f32]>,
}

impl TerrainStrokeHeights {
    /// Shares the recorded samples with the inverse action on the other history stack.
    pub(crate) fn inverse(&self) -> Self {
        Self {
            bounds: self.bounds,
            before: Arc::clone(&self.after),
            after: Arc::clone(&self.before),
        }
    }
    /// Bytes of persisted sample data, excluding the history record and slice headers.
    #[cfg(test)]
    pub(in crate::nodes::sim) fn sample_bytes(&self) -> usize {
        (self.before.len() + self.after.len()) * size_of::<f32>()
    }
}

/// One reusable texture-sized window of protected source samples, in row-major order.
#[derive(Default)]
pub(crate) struct TerrainProtection {
    key: Option<([i32; 2], usize, [u64; 6])>,
    /// Location of the first sample in grid coordinates, including off-world padding.
    pub(crate) origin: [i32; 2],
    /// Width and height of the square mask.
    pub(crate) side: usize,
    /// Protection kinds at each sample, with zero for free ground.
    pub(crate) samples: Vec<u8>,
    /// Changes when the mask is regenerated.
    pub(crate) revision: u64,
}

impl TerrainProtection {
    fn at(&self, x: usize, z: usize) -> u8 {
        let i = x as i32 - self.origin[0];
        let j = z as i32 - self.origin[1];
        self.samples[j as usize * self.side + i as usize]
    }
}

#[derive(Clone, Copy)]
struct TerrainDab {
    mode: TerrainBrushMode,
    radius: f32,
    speed: f32,
    target: f32,
    anchors: SlopeAnchors,
}

/// The frozen settings, cursor and local journal of an open gameplay stroke.
pub(crate) struct GameTerrainStroke {
    dab: TerrainDab,
    cursor: Vector2,
    /// The source revision published to asynchronous terrain readers until release.
    pub(crate) published_source_generation: u64,
    original: HashMap<(usize, usize), f32>,
    bounds: Option<GridRect>,
    patch_scratch: Vec<PatchKey>,
    visual_patches: HashSet<PatchKey>,
    affected_patches: HashMap<PatchKey, GridRect>,
}

fn union([x0, z0, x1, z1]: GridRect, [a0, b0, a1, b1]: GridRect) -> GridRect {
    [x0.min(a0), z0.min(b0), x1.max(a1), z1.max(b1)]
}

fn include(bounds: &mut Option<GridRect>, rect: GridRect) {
    *bounds = Some(bounds.map_or(rect, |old| union(old, rect)));
}

impl GameTerrainStroke {
    fn sweep(&mut self, core: &mut SimCore, end: Vector2, elapsed: f64) -> i32 {
        // Integrate this input segment in full. Limit both the travel and the elapsed time
        // represented by each midpoint dab, even when one frame delivers a long segment.
        let start = self.cursor;
        let travel = f64::from(start.distance_to(end) / (self.dab.radius * 0.25));
        let count = (elapsed / MAX_DAB_INTERVAL_SEC).max(travel).ceil() as u32;
        let strength = self.dab.speed * (elapsed / f64::from(count)) as f32;
        let mut status = 0;
        for part in 0..count {
            let fraction = (part as f32 + 0.5) / count as f32;
            status |= self.apply(core, start.lerp(end, fraction), strength);
        }
        self.cursor = end;
        status
    }

    fn apply(&mut self, core: &mut SimCore, pos: Vector2, amount: f32) -> i32 {
        let Some((center, radius)) = core.game_terrain_brush_grid(pos, self.dab.radius) else {
            return OUTSIDE;
        };
        core.update_game_terrain_protection(center, radius);
        let mut changed = None;
        let mut protected = 0;
        let mut free = false;
        let terrain = &mut core.heightmap;
        for (x, z, distance) in brush_samples(terrain, center, radius) {
            let kind = core.terrain_brush_protection.at(x, z);
            if kind != 0 {
                protected |= i32::from(kind);
                continue;
            }
            free = true;
            if distance >= 1.0 {
                continue;
            }
            let falloff = 0.5 + 0.5 * (distance * std::f32::consts::PI).cos();
            let current = terrain.source_sample(x, z);
            let strength = amount * falloff;
            let next = match self.dab.mode {
                TerrainBrushMode::Raise => current + strength,
                TerrainBrushMode::Lower => current - strength,
                mode => {
                    let target = match mode {
                        TerrainBrushMode::Flatten => self.dab.target,
                        TerrainBrushMode::Ramp => self.dab.ramp_height(terrain, x, z),
                        TerrainBrushMode::Smooth => self.original_average(terrain, x, z),
                        _ => unreachable!(),
                    };
                    let original = self.original.get(&(x, z)).copied().unwrap_or(current);
                    let delta = target - current;
                    let step = strength * (target - original).abs();
                    if delta.abs() <= step {
                        target
                    } else {
                        current + delta.signum() * step
                    }
                }
            };
            if next.to_bits() != current.to_bits() {
                self.original.entry((x, z)).or_insert(current);
                terrain.set_source_sample(x, z, next);
                include(&mut changed, [x, z, x, z]);
            }
        }
        if let Some(rect) = changed {
            self.record_changes(terrain, rect);
        }
        if !free && protected != 0 {
            protected | ALL_PROTECTED
        } else {
            0
        }
    }

    fn original_average(&self, terrain: &TerrainSystem, x: usize, z: usize) -> f32 {
        let mut sum = 0.0;
        let mut count = 0;
        for j in z.saturating_sub(1)..=(z + 1).min(terrain.height - 1) {
            for i in x.saturating_sub(1)..=(x + 1).min(terrain.width - 1) {
                sum += self
                    .original
                    .get(&(i, j))
                    .copied()
                    .unwrap_or_else(|| terrain.source_sample(i, j));
                count += 1;
            }
        }
        sum / count as f32
    }

    fn record_changes(&mut self, terrain: &mut TerrainSystem, rect: GridRect) {
        include(&mut self.bounds, rect);
        terrain.record_live_source_edit(rect);
        let [min_x, min_z, max_x, max_z] = world_bounds(terrain, rect);
        let margin = terrain.render_patch_border_margin_m();
        self.patch_scratch.clear();
        terrain.append_render_patch_keys_for_world_bounds(
            min_x - margin,
            min_z - margin,
            max_x + margin,
            max_z + margin,
            &mut self.patch_scratch,
        );
        for &key in &self.patch_scratch {
            self.visual_patches.insert(key);
            self.affected_patches
                .entry(key)
                .and_modify(|old| *old = union(*old, rect))
                .or_insert(rect);
        }
    }
}

impl TerrainDab {
    fn ramp_height(&self, terrain: &TerrainSystem, x: usize, z: usize) -> f32 {
        let (x, z) = terrain.grid_to_world_coords(x, z);
        let start = self.anchors.start_world;
        let ab = self.anchors.end_world - start;
        let t =
            ((Vector2::new(x, z) - start).dot(ab) / ab.length_squared().max(1.0)).clamp(0.0, 1.0);
        let a = self.anchors.start_height_m;
        let b = self.anchors.end_height_m;
        (a + (b - a) * t) / config::HEIGHT_SCALE
    }
}

fn world_bounds(terrain: &TerrainSystem, [x0, z0, x1, z1]: GridRect) -> [f32; 4] {
    let (min_x, min_z) = terrain.grid_to_world_coords(x0, z0);
    let (max_x, max_z) = terrain.grid_to_world_coords(x1, z1);
    [min_x, min_z, max_x, max_z]
}

fn brush_samples(
    terrain: &TerrainSystem,
    center: Vector2,
    radius: f32,
) -> impl Iterator<Item = (usize, usize, f32)> + use<> {
    let min_x = (center.x - radius).ceil().max(0.0) as usize;
    let min_z = (center.y - radius).ceil().max(0.0) as usize;
    let max_x = ((center.x + radius).floor() as usize).min(terrain.width - 1);
    let max_z = ((center.y + radius).floor() as usize).min(terrain.height - 1);
    (min_z..=max_z).flat_map(move |z| {
        (min_x..=max_x).filter_map(move |x| {
            let distance_sq = center.distance_squared_to(Vector2::new(x as f32, z as f32));
            (distance_sq <= radius * radius).then(|| (x, z, distance_sq.sqrt() / radius))
        })
    })
}

impl SimCore {
    fn game_terrain_brush_grid(&self, pos: Vector2, radius: f32) -> Option<(Vector2, f32)> {
        let (half_x, half_z) = self.heightmap.half_world_extents();
        if !pos.is_finite()
            || pos.x.abs() > half_x
            || pos.y.abs() > half_z
            || !(10.0..=100.0).contains(&radius)
        {
            return None;
        }
        let (x, z) = self.heightmap.world_to_grid_coords(pos.x, pos.y);
        Some((Vector2::new(x, z), radius / self.heightmap.cell_size_m()))
    }

    /// Computes protected kinds strictly inside the footprint and whether any sample is free.
    pub(crate) fn game_terrain_brush_status(
        &mut self,
        pos: Vector2,
        radius: f32,
        include_water: bool,
    ) -> i32 {
        let Some((center, radius)) = self.game_terrain_brush_grid(pos, radius) else {
            return OUTSIDE;
        };
        self.update_game_terrain_protection(center, radius);
        let mut status = 0;
        let mut free = false;
        let mut wet = false;
        for (x, z, _) in brush_samples(&self.heightmap, center, radius) {
            let kind = self.terrain_brush_protection.at(x, z);
            status |= i32::from(kind);
            free |= kind == 0;
            if include_water && !wet {
                let (wx, wz) = self.heightmap.grid_to_world_coords(x, z);
                wet = self.watermap.visible_depth_world(wx, wz) > 0.001;
            }
        }
        if status != 0 && !free {
            status | ALL_PROTECTED
        } else if status == 0 && wet {
            WATER
        } else {
            status
        }
    }

    fn update_game_terrain_protection(&mut self, center: Vector2, radius: f32) {
        let reach = radius.ceil() as i32;
        let anchor = [center.x.floor() as i32, center.y.floor() as i32];
        let side = (2 * reach + 2) as usize;
        let revisions = [
            self.world_epoch,
            self.transit_network.roadbed.generation(),
            self.road_terrain_inputs.site_revision(),
            self.zoning.parcels.overlay_revision(),
            self.agriculture.visual_revision(),
            self.get_coal_pit_overlay_revision_internal(),
        ];
        let key = (anchor, side, revisions);
        if self.terrain_brush_protection.key == Some(key) {
            return;
        }
        let mut mask = std::mem::take(&mut self.terrain_brush_protection);
        mask.key = Some(key);
        mask.origin = [anchor[0] - reach, anchor[1] - reach];
        mask.side = side;
        mask.revision = mask.revision.wrapping_add(1);
        mask.samples.resize(side * side, 0);
        mask.samples.fill(0);
        let margin = self.heightmap.cell_size_m();
        let (half_x, half_z) = self.heightmap.half_world_extents();
        let origin = Vector2::new(
            mask.origin[0] as f32 * margin - half_x,
            mask.origin[1] as f32 * margin - half_z,
        );
        let padding = Vector2::new(margin, margin);
        let broad =
            self.protection_in_rect(origin - padding, origin + padding * side as f32, u8::MAX);
        if broad != 0 {
            for (j, row) in mask.samples.chunks_exact_mut(side).enumerate() {
                for (i, sample) in row.iter_mut().enumerate() {
                    let at = origin + Vector2::new(i as f32, j as f32) * margin;
                    if at.x.abs() <= half_x && at.y.abs() <= half_z {
                        *sample = self.protection_in_rect(at - padding, at + padding, broad);
                    }
                }
            }
        }
        self.terrain_brush_protection = mask;
    }

    fn protection_in_rect(&self, min: Vector2, max: Vector2, candidates: u8) -> u8 {
        let rect = [min.x, min.y, max.x, max.y].map(f64::from);
        let mut status = 0;
        if candidates & ROAD != 0
            && self
                .transit_network
                .roadbed
                .any_top_triangle_points_in_bounds(rect, |triangle| {
                    let mut intersects = false;
                    clip_triangle_to_rect(triangle, rect, |_| intersects = true);
                    intersects
                })
        {
            status |= ROAD;
        }
        if candidates & BUILDING != 0 && self.allocator.any_graded_site_in_bounds(rect) {
            status |= BUILDING;
        }
        if candidates & (BUILDING | LOT) != 0 {
            self.zoning.parcels.visit_in_bounds(min, max, |parcel| {
                status |= if parcel.occupied_building().is_some() {
                    BUILDING
                } else {
                    LOT
                };
            });
        }
        if candidates & FIELD != 0 {
            let corners = [
                min,
                Vector2::new(max.x, min.y),
                max,
                Vector2::new(min.x, max.y),
            ];
            if self.allocator.field_clearance.overlaps_polygon(&corners)
                || self
                    .allocator
                    .extraction_clearance
                    .overlaps_polygon(&corners)
            {
                status |= FIELD;
            }
        }
        status
    }

    /// Starts a gameplay stroke, freezing the settings and initial flatten height.
    pub(crate) fn begin_game_terrain_stroke(
        &mut self,
        mode: TerrainBrushMode,
        pos: Vector2,
        radius: f32,
        strength: f32,
        anchor_a: Vector3,
        anchor_b: Vector3,
    ) {
        let Some((_, radius_cells)) = self.game_terrain_brush_grid(pos, radius) else {
            return;
        };
        if self.terrain_stroke_active
            || ![0.0, 1.0, 2.0].contains(&strength)
            || !anchor_a.is_finite()
            || !anchor_b.is_finite()
        {
            return;
        }
        self.start_terrain_stroke_internal();
        // Asynchronous render readers retain the committed surface while the live textures
        // are installed directly. They only receive a new payload revision at release.
        self.shared_terrain_copy();
        let index = strength as usize;
        let speed = match mode {
            TerrainBrushMode::Raise | TerrainBrushMode::Lower => {
                RAISE_PER_DIAMETER[index] * radius * 2.0 / config::HEIGHT_SCALE
            }
            _ => TARGET_SECONDS[index].recip(),
        };
        let side = (2.0 * radius_cells.ceil() + 2.0) as usize;
        self.game_terrain_stroke = Some(GameTerrainStroke {
            dab: TerrainDab {
                mode,
                radius,
                speed,
                target: self.get_height_at_internal(pos) / config::HEIGHT_SCALE,
                anchors: SlopeAnchors {
                    start_world: Vector2::new(anchor_a.x, anchor_a.z),
                    start_height_m: anchor_a.y,
                    end_world: Vector2::new(anchor_b.x, anchor_b.z),
                    end_height_m: anchor_b.y,
                },
            },
            cursor: pos,
            published_source_generation: self.heightmap.source_generation(),
            original: HashMap::with_capacity(side * side),
            bounds: None,
            patch_scratch: Vec::with_capacity(4),
            visual_patches: HashSet::with_capacity(4),
            affected_patches: HashMap::with_capacity(4),
        });
    }

    /// Integrates the elapsed pointer segment and writes its changes during the current frame.
    /// O(dabs × brush samples), plus local indexed protection when the mask moves.
    pub(crate) fn game_terrain_stroke_step(&mut self, pos: Vector2, elapsed: f64) -> i32 {
        if !elapsed.is_finite() || elapsed <= 0.0 || !pos.is_finite() {
            return 0;
        }
        let Some(mut stroke) = self.game_terrain_stroke.take() else {
            return 0;
        };
        self.adopt_pending_cell_lots();
        let status = stroke.sweep(self, pos, elapsed);
        self.game_terrain_stroke = Some(stroke);
        status
    }

    /// Visits the render patches that need new live heights since the last refresh.
    pub(crate) fn visit_game_terrain_visual_patches(
        &mut self,
        mut visit: impl FnMut(&TerrainSystem, PatchKey),
    ) -> bool {
        let Some(stroke) = &mut self.game_terrain_stroke else {
            return false;
        };
        let changed = !stroke.visual_patches.is_empty();
        for key in stroke.visual_patches.drain() {
            visit(&self.heightmap, key);
        }
        changed
    }

    /// Closes a stroke and rebuilds the dependent terrain without regrading the graph.
    pub(crate) fn finish_game_terrain_stroke(&mut self) -> bool {
        let Some(stroke) = self.game_terrain_stroke.take() else {
            return false;
        };
        self.terrain_stroke_active = false;
        let Some(bounds @ [x0, z0, x1, z1]) = stroke.bounds else {
            return false;
        };
        let mut before = Vec::with_capacity((x1 - x0 + 1) * (z1 - z0 + 1));
        let mut after = Vec::with_capacity(before.capacity());
        for z in z0..=z1 {
            for x in x0..=x1 {
                let current = self.heightmap.source_sample(x, z);
                before.push(stroke.original.get(&(x, z)).copied().unwrap_or(current));
                after.push(current);
            }
        }
        let heights = TerrainStrokeHeights {
            bounds,
            before: before.into(),
            after: after.into(),
        };
        self.push_undo_snapshot(Self::terrain_history_record(Box::new(heights)));

        let mut patches: Vec<_> = stroke.affected_patches.into_iter().collect();
        patches.sort_unstable_by_key(|&(key, _)| key);
        for (key, rect) in patches {
            let [x0, z0, x1, z1] = world_bounds(&self.heightmap, rect);
            let margin = self.heightmap.render_patch_border_margin_m();
            let Some((a, b, c, d)) = self.heightmap.render_patch_world_bounds(key.0, key.1) else {
                continue;
            };
            self.mark_terrain_authoring_payload_bounds([
                (x0 - margin).max(a),
                (z0 - margin).max(b),
                (x1 + margin).min(c),
                (z1 + margin).min(d),
            ]);
        }
        self.finish_game_terrain_dependents();
        true
    }

    /// Restores the rectangle captured by gameplay terrain history.
    pub(in crate::nodes::sim) fn restore_game_terrain_heights(
        &mut self,
        heights: &TerrainStrokeHeights,
    ) {
        let [x0, z0, x1, _] = heights.bounds;
        let width = x1 - x0 + 1;
        for (j, row) in heights.before.chunks_exact(width).enumerate() {
            for (i, &height) in row.iter().enumerate() {
                let (x, z) = (x0 + i, z0 + j);
                if height.to_bits() != heights.after[j * width + i].to_bits() {
                    self.heightmap.set_source_sample(x, z, height);
                }
            }
        }
        self.heightmap.record_live_source_edit(heights.bounds);
        let [x0, z0, x1, z1] = world_bounds(&self.heightmap, heights.bounds);
        let margin = self.heightmap.render_patch_border_margin_m();
        self.mark_terrain_authoring_payload_bounds([
            x0 - margin,
            z0 - margin,
            x1 + margin,
            z1 + margin,
        ]);
        self.finish_game_terrain_dependents();
    }

    fn finish_game_terrain_dependents(&mut self) {
        // The editor re-grades the RegionGraph explicitly with sync_to_terrain.
        // Gameplay preserves the source samples beneath roads and their authored profiles.
        self.rebuild_building_entrances_internal();
        if self.has_authored_water_internal()
            && let Err(error) = self.rebuild_authored_water_preview_internal()
        {
            crate::debug_log!(
                "terrain",
                "gameplay terrain water rebuild failed: {}",
                error
            );
        }
    }
}
