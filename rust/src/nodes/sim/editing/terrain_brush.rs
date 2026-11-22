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
    /// Whether the roads moved with the ground ("Roads follow"), so a redo moves them again.
    pub(crate) roads_follow: bool,
}

impl TerrainStrokeHeights {
    /// Shares the recorded samples with the inverse action on the other history stack.
    pub(crate) fn inverse(&self) -> Self {
        Self {
            bounds: self.bounds,
            before: Arc::clone(&self.after),
            after: Arc::clone(&self.before),
            roads_follow: self.roads_follow,
        }
    }

    /// The source height change from `before` to `after` at world `(x, z)`, in metres:
    /// bilinear over the recorded samples as the terrain samples heights, zero where none of
    /// the four samples around the point changed. O(1).
    pub(crate) fn delta_m(&self, terrain: &TerrainSystem, x: f32, z: f32) -> f32 {
        let [x0, z0, x1, z1] = self.bounds;
        let width = x1 - x0 + 1;
        let sample = |i: i64, j: i64| -> f32 {
            if i < x0 as i64 || i > x1 as i64 || j < z0 as i64 || j > z1 as i64 {
                return 0.0;
            }
            let k = (j as usize - z0) * width + (i as usize - x0);
            self.after[k] - self.before[k]
        };
        let (gx, gz) = terrain.world_to_grid_coords(x, z);
        let (fx, fz) = (gx.floor(), gz.floor());
        let (i, j) = (fx as i64, fz as i64);
        let (tx, tz) = (gx - fx, gz - fz);
        let row = |j| sample(i, j) * (1.0 - tx) + sample(i + 1, j) * tx;
        (row(j) * (1.0 - tz) + row(j + 1) * tz) * config::HEIGHT_SCALE
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
    // "Roads follow" as it stood at press.
    roads_follow: bool,
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
            self.agriculture.visual_revision(),
            self.get_coal_pit_overlay_revision_internal(),
            u64::from(self.game_terrain_roads_follow),
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
        // "Roads follow" frees the road beds: the roads move with the ground at release.
        let candidates = if self.game_terrain_roads_follow {
            !ROAD
        } else {
            u8::MAX
        };
        let broad =
            self.protection_in_rect(origin - padding, origin + padding * side as f32, candidates);
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

    // A sample is reserved when its interpolation cell (`min..max`, one sample of margin) meets
    // a road bed, a work area or a building's flat pad. The yard paving and lot ground around
    // the pad follow the source, so the brush may reshape them; the margin keeps the source
    // under the pad, and so its support height, unchanged.
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
        if candidates & BUILDING != 0 && self.allocator.any_site_pad_in_bounds(rect) {
            status |= BUILDING;
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

    /// Flatten's target under `pos`, in metres: a building's floor height on its pad or paved
    /// yard, so one stroke levels the yard flush with the building, else the source height.
    pub(crate) fn game_terrain_flatten_height_m(&self, pos: Vector2) -> f32 {
        self.allocator
            .site_support_height_at(pos)
            .unwrap_or_else(|| self.get_height_at_internal(pos))
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
                target: self.game_terrain_flatten_height_m(pos) / config::HEIGHT_SCALE,
                anchors: SlopeAnchors {
                    start_world: Vector2::new(anchor_a.x, anchor_a.z),
                    start_height_m: anchor_a.y,
                    end_world: Vector2::new(anchor_b.x, anchor_b.z),
                    end_height_m: anchor_b.y,
                },
            },
            roads_follow: self.game_terrain_roads_follow,
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
            roads_follow: stroke.roads_follow,
        };

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
        // An undone stroke's restore rebuilds its own dependents.
        if self.record_game_terrain_stroke(Box::new(heights)) {
            self.finish_game_terrain_dependents();
        }
        true
    }

    /// Pushes a stroke's history, moving the roads with the ground first when the stroke had
    /// "Roads follow" on: the roads' undo record and the terrain's form one group, the terrain's
    /// on top, so undo restores both and redo finds the stroke. A move that leaves a road
    /// steeper than the 1:3 maximum, or a junction without a layout, undoes the whole stroke and
    /// leaves its reason in `game_terrain_rejection`. Returns whether the stroke stands.
    pub(in crate::nodes::sim) fn record_game_terrain_stroke(
        &mut self,
        heights: Box<TerrainStrokeHeights>,
    ) -> bool {
        let group = self.next_undo_group();
        self.undo_batch_group = Some(group);
        let rejection = if heights.roads_follow {
            self.follow_roads_to_game_terrain(&heights)
        } else {
            None
        };
        self.push_undo_snapshot(Self::terrain_history_record(heights));
        self.undo_batch_group = None;
        let Some(reason) = rejection else {
            return true;
        };
        crate::debug_log!("terrain", "gameplay stroke undone: {}", reason);
        self.game_terrain_rejection = Some(reason);
        self.undo_action_in_pool();
        false
    }

    // Moves the standard roads under a stroke with the ground (`follow_local_ground_change`)
    // and rebuilds what stands on them locally, as the border connection's in-place edit does
    // (`ROAD-122`): the graph's undo record first, then the routing graph, lots, roadbed,
    // render chunks, lanes with their agents, and the doors. Returns the reason to undo the
    // stroke, if any. O(roads under the stroke and their lane closure), independent of the city.
    fn follow_roads_to_game_terrain(
        &mut self,
        heights: &TerrainStrokeHeights,
    ) -> Option<&'static str> {
        let cell = self.heightmap.cell_size_m();
        let [x0, z0, x1, z1] = world_bounds(&self.heightmap, heights.bounds);
        // Bilinear reads reach one cell past the changed samples.
        let (min, max) = (
            Vector3::new(x0 - cell, 0.0, z0 - cell),
            Vector3::new(x1 + cell, 0.0, z1 + cell),
        );
        let mut candidates = Vec::new();
        self.region_graph
            .visit_edges_near_aabb(min, max, |edge| candidates.push(edge));
        candidates.sort_unstable();
        candidates.dedup();
        if candidates.is_empty() {
            return None;
        }
        let roads: HashSet<usize> = candidates.iter().copied().collect();
        let grades_before: HashMap<usize, f32> = candidates
            .iter()
            .map(|&edge| {
                (
                    edge,
                    crate::nodes::sim::core::edge_grade(&self.region_graph, edge),
                )
            })
            .collect();
        let checkpoint = self
            .region_graph
            .capture_undo_for_local_topology(roads, HashSet::new());
        let (mut edges, mut nodes) = (HashSet::new(), HashSet::new());
        let terrain = &self.heightmap;
        self.region_graph.follow_local_ground_change(
            terrain,
            &candidates,
            |x, z| heights.delta_m(terrain, x, z),
            &mut edges,
            &mut nodes,
        );
        if edges.is_empty() {
            return None;
        }
        self.push_network_undo_delta(checkpoint, None, None);
        self.schedule_cch_rebuild_of_live_graph();
        self.zoning
            .mark_cell_lots_for_roads(&self.region_graph, edges.iter().copied());
        self.transit_network
            .mark_surface_dirty_from_sets(&edges, &nodes);
        self.mark_local_network_render_dirty();
        // Godot's main thread runs the release; the lane passes are parallel (CODE-17).
        crate::thread_pools::install(|| self.rebuild_in_place_edit_locally(&edges, &nodes));
        self.transit_network
            .roadbed
            .compile_dirty(&self.region_graph);
        if !self.transit_network.roadbed.failed_nodes().is_empty() {
            return Some("A junction can't follow this ground");
        }
        crate::nodes::sim::core::steepened_profile(&self.region_graph, &edges, &grades_before)
            .map(|_| "A road would be steeper than 1:3")
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
        // The editor re-grades the whole RegionGraph with sync_to_terrain. Gameplay keeps the
        // source under road beds unless "Roads follow" moved the roads locally at release.
        self.rebuild_building_entrances_internal();
        // The graded base-edit journal queued nearby lots, including already refused ones.
        // Release and undo publish only once, after all stroke samples/dependents are current.
        self.publish_parking_sites();
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
