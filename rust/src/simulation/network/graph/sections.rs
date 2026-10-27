// SPDX-License-Identifier: GPL-2.0-only

//! Cold, immutable cross-section authority. No simulation tick resolves or interns layouts.

use super::{Edge, RegionGraph};
use crate::simulation::network::types::{TransitFlags, TransitType};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

mod queries;
#[cfg(test)]
mod tests;

/// An exact finite dimension, ordered and interned by canonical IEEE-754 bits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SectionScalar(u32);

impl SectionScalar {
    /// Rejects nonfinite input; new negative zero has the same identity as positive zero.
    pub fn new(value: f32) -> Result<Self, &'static str> {
        if !value.is_finite() {
            return Err("nonfinite_section_dimension");
        }
        Ok(Self(if value == 0.0 { 0 } else { value.to_bits() }))
    }

    /// Returns the stored dimension without quantization.
    pub fn get(self) -> f32 {
        f32::from_bits(self.0)
    }

    fn retained(value: f32) -> Self {
        Self(value.to_bits())
    }
}

/// Physical authoring role; mode permissions remain independent of the material.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(u8)]
pub enum StripRole {
    /// Unused bounded array slot, never a published strip.
    #[default]
    Absent,
    /// Raised or flush kerb outside the core.
    Kerb,
    /// Clear walking pavement.
    Sidewalk,
    /// Cycle facility with explicit direction.
    Cycle,
    /// Shared walking/cycling pavement.
    Combined,
    /// Nontraversable planted or furniture space.
    Verge,
    /// Authored parking width, independent of through lanes.
    Parking,
    /// Support or traversable shoulder as explicitly permitted.
    Shoulder,
    /// Depressed drainage profile.
    Ditch,
}

/// One stable side-local strip, ordered from the core outward.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RoadStrip {
    /// Exact plan width, excluding adjacent strips.
    pub width: SectionScalar,
    /// Inner and outer heights relative to the section plane; also express crossfall.
    pub heights: [SectionScalar; 2],
    /// Stable identity across endpoint correspondence, unique within a side.
    pub identity: u16,
    /// Furniture/clearance policy bits; zero disallows furniture.
    pub policy: u16,
    /// Physical role independent of surface and permissions.
    pub role: StripRole,
    /// Surface code: zero asphalt, one paving, two grass, three gravel.
    pub surface: u8,
    /// Explicit traversal permissions; a shoulder does not imply walking.
    pub modes: u8,
    /// Zero bidirectional, +1 stored direction, -1 reverse direction.
    pub direction: i8,
}

/// Bounded immutable base layout; median/taper/bus authority stays in its existing owner.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RoadLayout {
    /// Individual motor widths, innermost first, forward then backward (four slots each).
    pub lane_widths: [[SectionScalar; 4]; 2],
    /// Ordered outer strips, negative lateral side then positive lateral side.
    pub strips: [[RoadStrip; 8]; 2],
    /// Exact saved core width, including exceptional retained margins.
    pub core_width: SectionScalar,
    /// Exact full-section walking axes in the stored tangent frame; zero when absent or path-centred.
    pub walk_axes: [SectionScalar; 2],
    /// Retained single-path compiled width; zero for a motor profile.
    pub path_width: SectionScalar,
    /// Active lane counts; unused width slots are canonical zero.
    pub lane_counts: [u8; 2],
    /// Active strip counts; unused slots are canonical empty.
    pub strip_counts: [u8; 2],
}

impl RoadLayout {
    /// Captures the current compiler's exact dimensions, including constrained old profiles.
    pub fn legacy(edge: &Edge) -> Self {
        let scalar = SectionScalar::retained;
        // The current compiler treats non-CAR authority as one Footpath too. Retain that
        // historical interpretation here; future bus/cycle authoring must not use this rule.
        let path =
            edge.primary_type == TransitType::Foot || edge.allowed_types & TransitFlags::CAR == 0;
        let mut layout = Self {
            lane_widths: [[SectionScalar::default(); 4]; 2],
            strips: [[RoadStrip::default(); 8]; 2],
            core_width: scalar(edge.width),
            walk_axes: [SectionScalar::default(); 2],
            path_width: scalar(if path { edge.width.max(2.0) } else { 0.0 }),
            lane_counts: [edge.fwd_lanes, edge.bkw_lanes],
            strip_counts: [0; 2],
        };
        for side in 0..2 {
            for lane in layout.lane_widths[side]
                .iter_mut()
                .take(usize::from(layout.lane_counts[side]))
            {
                *lane = scalar(3.5);
            }
            if path {
                continue;
            }
            let foot = edge.allowed_types & TransitFlags::FOOT != 0;
            layout.strips[side][0] = RoadStrip {
                width: scalar(if foot { 0.15 } else { 0.75 }),
                heights: [scalar(0.12); 2],
                identity: 1,
                role: if foot {
                    StripRole::Kerb
                } else {
                    StripRole::Shoulder
                },
                ..Default::default()
            };
            layout.strip_counts[side] = 1;
            if foot {
                layout.strips[side][1] = RoadStrip {
                    // Preserve the compiler's subtraction, rather than a rounded decimal.
                    width: scalar(1.5 - 0.15),
                    heights: [scalar(0.12); 2],
                    identity: 2,
                    role: StripRole::Sidewalk,
                    surface: 1,
                    modes: TransitFlags::FOOT,
                    policy: RoadStrip::FURNITURE_ALLOWED,
                    ..Default::default()
                };
                layout.strip_counts[side] = 2;
                let asphalt_width = f32::from(edge.fwd_lanes + edge.bkw_lanes) * 3.5
                    + edge.median_half_width() * 2.0;
                let sign = if side == 0 { -1.0 } else { 1.0 };
                layout.walk_axes[side] =
                    scalar(edge.pocket_surface_shift() + (asphalt_width * 0.5 + 0.75) * sign);
            }
        }
        layout
    }

    /// Validates retained geometry separately from new-authoring walking recommendations.
    pub fn validate(&self, retained: bool) -> Result<(), &'static str> {
        if self.lane_counts.iter().any(|&n| n > 4) || self.strip_counts.iter().any(|&n| n > 8) {
            return Err("section_component_limit");
        }
        let finite = |v: SectionScalar| v.get().is_finite();
        if !finite(self.core_width)
            || self.core_width.get() <= 0.0
            || !finite(self.path_width)
            || self.path_width.get() < 0.0
            || self.walk_axes.iter().any(|&v| !finite(v))
        {
            return Err("invalid_section_envelope");
        }
        for side in 0..2 {
            for (rank, &width) in self.lane_widths[side].iter().enumerate() {
                if !finite(width)
                    || if rank < usize::from(self.lane_counts[side]) {
                        !(2.75..=4.5).contains(&width.get())
                    } else {
                        width != SectionScalar::default()
                    }
                {
                    return Err("invalid_section_lane_width");
                }
            }
            for (index, strip) in self.strips[side].iter().enumerate() {
                if index >= usize::from(self.strip_counts[side]) {
                    if *strip != RoadStrip::default() {
                        return Err("nonempty_unused_strip");
                    }
                    continue;
                }
                if strip.role == StripRole::Absent
                    || strip.identity == 0
                    || self.strips[side][..index]
                        .iter()
                        .any(|s| s.identity == strip.identity)
                    || !finite(strip.width)
                    || strip.width.get() <= 0.0
                    || strip.heights.iter().any(|&v| !finite(v))
                    || !(-1..=1).contains(&strip.direction)
                    || strip.surface > 3
                {
                    return Err("invalid_section_strip");
                }
                if strip.role == StripRole::Sidewalk
                    && (strip.modes & TransitFlags::FOOT == 0
                        || (!retained && strip.width.get() < 1.5))
                {
                    return Err("invalid_clear_walking_space");
                }
                if matches!(
                    strip.role,
                    StripRole::Kerb | StripRole::Verge | StripRole::Ditch
                ) && strip.modes != 0
                {
                    return Err("nontraversable_strip_modes");
                }
            }
        }
        Ok(())
    }

    /// Reverses orientation, preserving side-local identity and explicit permissions.
    pub fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.lane_widths.swap(0, 1);
        result.lane_counts.swap(0, 1);
        result.strips.swap(0, 1);
        result.strip_counts.swap(0, 1);
        result.walk_axes.swap(0, 1);
        for axis in &mut result.walk_axes {
            *axis = SectionScalar::new(-axis.get()).expect("validated finite walking axis");
        }
        for strip in result.strips.iter_mut().flatten() {
            strip.direction = -strip.direction;
        }
        result
    }
}

/// Compact reference in a separate edge-indexed table; no layout is copied into Edge or agents.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RoadSectionRef {
    /// Stable immutable layout ID within this graph's catalog.
    pub layout: u32,
    /// Optional sparse detail record; `u32::MAX` means absent.
    pub detail: u32,
    /// Authored section revision for preview/certificate invalidation.
    pub revision: u32,
    /// Functional class code; zero is unclassified/custom until slice B.
    pub class: u8,
    /// Resolved access permissions, independent of functional class defaults.
    pub access: u8,
    /// Explicit override bits, preserving authored policy when presets change.
    pub overrides: u16,
}

/// Broad-phase extents in the stored tangent frame, maintained beside section references.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RoadSideExtents {
    /// Outer paved boundary on each side, signed lateral metres.
    pub paved: [f32; 2],
    /// Road-owned ground boundary on each side, signed lateral metres.
    pub owned: [f32; 2],
}

#[derive(Clone, Default)]
pub(in crate::simulation::network) struct RoadSections {
    pub(in crate::simulation::network) references: Vec<RoadSectionRef>,
    pub(in crate::simulation::network) extents: Vec<RoadSideExtents>,
    pub(in crate::simulation::network) layouts: Vec<Arc<RoadLayout>>,
    // Arc keys share the immutable allocation rather than doubling every layout's memory.
    intern: BTreeMap<Arc<RoadLayout>, u32>,
    pub(in crate::simulation::network) details: Vec<Arc<RoadSectionDetail>>,
    detail_intern: BTreeMap<Arc<RoadSectionDetail>, u32>,
}

impl RoadSections {
    pub(in crate::simulation::network) fn intern(&mut self, layout: RoadLayout) -> u32 {
        if let Some(&id) = self.intern.get(&layout) {
            return id;
        }
        let id = u32::try_from(self.layouts.len()).expect("road layout ID space exhausted");
        let layout = Arc::new(layout);
        self.intern.insert(layout.clone(), id);
        self.layouts.push(layout);
        id
    }
}

impl RegionGraph {
    /// Immutable resolved section authority, with two array reads and no allocation.
    pub fn road_layout(&self, id: usize) -> &RoadLayout {
        &self.road_sections.layouts[self.road_sections.references[id].layout as usize]
    }

    /// Authored identity/revision and policy bytes for a road slot, including tombstones.
    pub fn road_section_ref(&self, id: usize) -> RoadSectionRef {
        self.road_sections.references[id]
    }

    /// Cached side-specific envelopes; read-only broad-phase queries are O(1).
    pub fn road_side_extents(&self, id: usize) -> RoadSideExtents {
        self.road_sections.extents[id]
    }

    // Existing tools still author legacy sections. Called only at graph mutation boundaries;
    // O(B log U) for a changed section, O(B) otherwise; B is bounded by sixteen strips.
    pub(in crate::simulation::network) fn sync_legacy_section(&mut self, id: usize) {
        let edge = &self.edges[id];
        let layout = RoadLayout::legacy(edge);
        let changed = self
            .road_sections
            .references
            .get(id)
            .and_then(|r| {
                self.road_sections
                    .layouts
                    .get(r.layout as usize)
                    .map(|existing| **existing != layout || r.access != edge.allowed_types)
            })
            .unwrap_or(true);
        if changed {
            let access = edge.allowed_types;
            let layout_id = self.road_sections.intern(layout);
            let default_detail = self
                .road_sections
                .intern_detail(RoadSectionDetail::default());
            if let Some(reference) = self.road_sections.references.get_mut(id) {
                reference.layout = layout_id;
                if reference.detail == u32::MAX {
                    reference.detail = default_detail;
                }
                reference.access = access;
                reference.revision = reference.revision.wrapping_add(1);
            } else {
                debug_assert_eq!(id, self.road_sections.references.len());
                let detail = self
                    .road_sections
                    .intern_detail(RoadSectionDetail::default());
                self.road_sections.references.push(RoadSectionRef {
                    layout: layout_id,
                    detail,
                    revision: 0,
                    class: 0,
                    access,
                    overrides: RoadSectionRef::LEGACY_OVERRIDES,
                });
                self.road_sections.extents.push(RoadSideExtents::default());
            }
        }
        self.road_sections.extents[id] = self.section_extents(id);
    }

    /// Validates all references and mode/lane compatibility before snapshot publication.
    pub fn validate_road_sections(&self) -> Result<(), &'static str> {
        if self.road_sections.references.len() != self.edges.len()
            || self.road_sections.extents.len() != self.edges.len()
        {
            return Err("section_slot_count_mismatch");
        }
        for (id, reference) in self.road_sections.references.iter().enumerate() {
            if self.edges[id].deleted {
                continue;
            }
            let layout = self
                .road_sections
                .layouts
                .get(reference.layout as usize)
                .ok_or("invalid_section_layout_reference")?;
            layout.validate(true)?;
            if reference.overrides & !RoadSectionRef::LEGACY_OVERRIDES != 0 {
                return Err("invalid_section_override_bits");
            }
            if reference.class != 0 {
                return Err("unsupported_section_class");
            }
            if reference.detail == u32::MAX {
                return Err("missing_authored_section_detail");
            }
            self.road_sections
                .details
                .get(reference.detail as usize)
                .ok_or("invalid_section_detail_reference")?
                .validate_supported()?;
            let edge = &self.edges[id];
            if reference.access != edge.allowed_types
                || layout.lane_counts != [edge.fwd_lanes, edge.bkw_lanes]
                || layout.core_width.get().to_bits() != edge.width.to_bits()
            {
                return Err("section_edge_authority_mismatch");
            }
            if layout.as_ref() != &RoadLayout::legacy(edge) {
                return Err("unsupported_section_authority");
            }
            if self.road_sections.extents[id] != self.section_extents(id) {
                return Err("stale_section_side_extents");
            }
            for (count, mask) in layout.lane_counts.iter().zip(edge.bus_lanes) {
                if u16::from(mask) >= (1u16 << count) {
                    return Err("invalid_section_bus_mask");
                }
            }
        }
        Ok(())
    }
}

/// Shared authored legacy placement/control policy, kept cold and outside Edge.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct RoadSectionDetail {
    /// Stored start/end approach control: 0 equal, 1 priority, 2 yield, 3 stop.
    pub approaches: [u8; 2],
    /// Crossing inset, stripe width, stripe gap and stripe length, in metres.
    pub crossing: [SectionScalar; 4],
    /// Lamp axis inset from the outer sidewalk edge, in metres.
    pub light_sidewalk_back_inset: SectionScalar,
    /// Lamp axis beyond a road shoulder when no sidewalk is present.
    pub light_verge_offset: SectionScalar,
    /// Lamp axis beyond an independent path's pavement.
    pub light_path_offset: SectionScalar,
    /// Minimum median interior width for a central lamp.
    pub median_light_clear_width: SectionScalar,
    /// Sign axis offset from the carriageway kerb face.
    pub sign_kerb_offset: SectionScalar,
    /// Sign axis beyond the shoulder without a sidewalk.
    pub sign_verge_offset: SectionScalar,
    /// Longitudinal sign setback from a crossing's centre.
    pub crossing_sign_setback: SectionScalar,
}

impl Default for RoadSectionDetail {
    fn default() -> Self {
        Self {
            approaches: [0; 2],
            crossing: [4.0, 0.5, 0.5, 2.5].map(SectionScalar::retained),
            light_sidewalk_back_inset: SectionScalar::retained(0.35),
            light_verge_offset: SectionScalar::retained(0.75),
            light_path_offset: SectionScalar::retained(0.5),
            median_light_clear_width: SectionScalar::retained(1.4),
            sign_kerb_offset: SectionScalar::retained(0.5),
            sign_verge_offset: SectionScalar::retained(0.75),
            crossing_sign_setback: SectionScalar::retained(1.75),
        }
    }
}

impl RoadSectionDetail {
    /// Checks finite positive anchor dimensions and recognized approach control codes.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.approaches.iter().any(|&v| v > 3)
            || [
                self.light_sidewalk_back_inset,
                self.light_verge_offset,
                self.light_path_offset,
                self.median_light_clear_width,
                self.sign_kerb_offset,
                self.sign_verge_offset,
                self.crossing_sign_setback,
            ]
            .iter()
            .chain(self.crossing.iter())
            .any(|v| !v.get().is_finite() || v.get() <= 0.0)
        {
            return Err("invalid_section_detail");
        }
        Ok(())
    }
}

impl RoadSectionDetail {
    fn validate_supported(&self) -> Result<(), &'static str> {
        self.validate()?;
        let expected = Self {
            approaches: self.approaches,
            ..Self::default()
        };
        // These dimensions become editable only when D/E convert their consumers.
        if *self != expected {
            return Err("unsupported_section_anchor_dimensions");
        }
        Ok(())
    }
}

impl RoadSections {
    pub(in crate::simulation::network) fn intern_detail(
        &mut self,
        detail: RoadSectionDetail,
    ) -> u32 {
        if let Some(&id) = self.detail_intern.get(&detail) {
            return id;
        }
        let id = u32::try_from(self.details.len()).expect("road detail ID space exhausted");
        let detail = Arc::new(detail);
        self.detail_intern.insert(detail.clone(), id);
        self.details.push(detail);
        id
    }
}

impl RegionGraph {
    /// Optional authored crossing, approach and furniture policy for this road.
    pub fn road_section_detail(&self, id: usize) -> Option<&RoadSectionDetail> {
        let reference = self.road_section_ref(id);
        (reference.detail != u32::MAX)
            .then(|| self.road_sections.details[reference.detail as usize].as_ref())
    }

    /// Installs a validated persisted record, interning in deterministic caller order.
    pub(crate) fn install_road_section(
        &mut self,
        id: usize,
        layout: RoadLayout,
        detail: Option<RoadSectionDetail>,
        mut reference: RoadSectionRef,
    ) -> Result<(), &'static str> {
        layout.validate(true)?;
        let detail = detail.ok_or("missing_authored_section_detail")?;
        detail.validate_supported()?;
        let edge = self.get_edge(id).ok_or("invalid_section_edge")?;
        // Until C–E convert consumers, accepting an unsupported shape would save authority
        // the game cannot render or route. Reject it before any record is published.
        if layout != RoadLayout::legacy(edge)
            || reference.class != 0
            || reference.access != edge.allowed_types
        {
            return Err("unsupported_section_authority");
        }
        reference.layout = self.road_sections.intern(layout);
        reference.detail = self.road_sections.intern_detail(detail);
        self.road_sections.references[id] = reference;
        self.road_sections.extents[id] = self.section_extents(id);
        Ok(())
    }

    pub(in crate::simulation::network) fn copy_road_section(
        &mut self,
        id: usize,
        source: &RegionGraph,
        source_id: usize,
    ) {
        let reference = source.road_section_ref(source_id);
        let layout = source.road_layout(source_id).clone();
        let detail = source.road_section_detail(source_id).cloned();
        self.install_road_section(id, layout, detail, reference)
            .expect("copied road section matches edge");
    }

    pub(crate) fn resolve_legacy_road_details(&mut self, ids: impl IntoIterator<Item = usize>) {
        use crate::simulation::network::lanes::control::{ApproachControl, approach_control};
        for id in ids {
            self.sync_legacy_section(id);
            let edge = self.edge(id);
            if edge.deleted {
                continue;
            }
            let encode = |control| match control {
                ApproachControl::Uncontrolled => 0,
                ApproachControl::Priority => 1,
                ApproachControl::Yield => 2,
                ApproachControl::Stop => 3,
            };
            let mut detail = self.road_section_detail(id).cloned().unwrap_or_default();
            detail.approaches = [edge.start_node, edge.end_node]
                .map(|node| encode(approach_control(self, self.get_valid_node(node), id)));
            let detail = self.road_sections.intern_detail(detail);
            let reference = &mut self.road_sections.references[id];
            if reference.detail != detail {
                reference.detail = detail;
                reference.revision = reference.revision.wrapping_add(1);
            }
        }
    }
}

impl RoadSectionRef {
    /// Retains individual lane widths when applying a preset.
    pub const LANES_OVERRIDE: u16 = 1;
    /// Retains the saved core envelope and exceptional margins.
    pub const CORE_OVERRIDE: u16 = 1 << 1;
    /// Retains the ordered outer strips and explicit walking axes.
    pub const STRIPS_OVERRIDE: u16 = 1 << 2;
    /// Retains the explicit speed independently of class defaults.
    pub const SPEED_OVERRIDE: u16 = 1 << 3;
    /// Retains access and frontage policy independently of class defaults.
    pub const ACCESS_OVERRIDE: u16 = 1 << 4;
    /// Retains authored approach controls independently of class proposals.
    pub const CONTROLS_OVERRIDE: u16 = 1 << 5;
    /// Retains authored crossing and furniture anchor dimensions.
    pub const FURNITURE_OVERRIDE: u16 = 1 << 6;
    /// Pins all currently authored legacy properties.
    pub const LEGACY_OVERRIDES: u16 = Self::LANES_OVERRIDE
        | Self::CORE_OVERRIDE
        | Self::STRIPS_OVERRIDE
        | Self::SPEED_OVERRIDE
        | Self::ACCESS_OVERRIDE
        | Self::CONTROLS_OVERRIDE
        | Self::FURNITURE_OVERRIDE;
}

impl RoadStrip {
    /// Allows explicitly retained furniture inside this strip's clearance policy.
    pub const FURNITURE_ALLOWED: u16 = 1;
}

impl Default for RoadSectionRef {
    fn default() -> Self {
        Self {
            layout: 0,
            detail: u32::MAX,
            revision: 0,
            class: 0,
            access: 0,
            overrides: 0,
        }
    }
}

/// Local checkpoint shares immutable allocations without copying the whole city catalog.
#[derive(Clone)]
pub(crate) struct RoadSectionSnapshot {
    reference: RoadSectionRef,
    layout: Arc<RoadLayout>,
    detail: Option<Arc<RoadSectionDetail>>,
    extents: RoadSideExtents,
}

impl RegionGraph {
    pub(crate) fn capture_road_section(&self, id: usize) -> RoadSectionSnapshot {
        let reference = self.road_section_ref(id);
        RoadSectionSnapshot {
            reference,
            layout: self.road_sections.layouts[reference.layout as usize].clone(),
            detail: (reference.detail != u32::MAX)
                .then(|| self.road_sections.details[reference.detail as usize].clone()),
            extents: self.road_sections.extents[id],
        }
    }

    pub(crate) fn restore_road_section(&mut self, id: usize, snapshot: RoadSectionSnapshot) {
        let mut reference = snapshot.reference;
        reference.layout = self.road_sections.intern((*snapshot.layout).clone());
        reference.detail = snapshot.detail.map_or(u32::MAX, |detail| {
            self.road_sections.intern_detail((*detail).clone())
        });
        self.road_sections.references[id] = reference;
        self.road_sections.extents[id] = snapshot.extents;
    }

    pub(crate) fn load_road_section_catalog(
        &mut self,
        layouts: Vec<RoadLayout>,
        details: Vec<RoadSectionDetail>,
    ) -> Result<(), &'static str> {
        let mut catalog = RoadSections::default();
        for (id, layout) in layouts.into_iter().enumerate() {
            if catalog.intern(layout) as usize != id {
                return Err("duplicate_section_layout");
            }
        }
        for (id, detail) in details.into_iter().enumerate() {
            if catalog.intern_detail(detail) as usize != id {
                return Err("duplicate_section_detail");
            }
        }
        catalog
            .references
            .resize(self.edges.len(), RoadSectionRef::default());
        catalog
            .extents
            .resize(self.edges.len(), RoadSideExtents::default());
        self.road_sections = catalog;
        Ok(())
    }

    pub(in crate::simulation::network) fn inherit_split_road_section(
        &mut self,
        first: usize,
        second: usize,
    ) {
        // An endpoint pocket can remain only on the appended half. Its already-resolved
        // layout must survive policy inheritance; the two halves need not share their axes.
        let second_layout = self.road_sections.references[second].layout;
        self.road_sections.references[second] = self.road_sections.references[first];
        self.road_sections.references[second].layout = second_layout;
        self.road_sections.references[first].revision = self.road_sections.references[first]
            .revision
            .wrapping_add(1);
        self.road_sections.references[second].revision =
            self.road_sections.references[first].revision;
        let detail = self.road_section_detail(first).cloned();
        if let Some(mut first_detail) = detail {
            let mut second_detail = first_detail.clone();
            first_detail.approaches[1] = 0;
            second_detail.approaches[0] = 0;
            self.road_sections.references[first].detail =
                self.road_sections.intern_detail(first_detail);
            self.road_sections.references[second].detail =
                self.road_sections.intern_detail(second_detail);
        }
        self.road_sections.extents[first] = self.section_extents(first);
        self.road_sections.extents[second] = self.section_extents(second);
    }
}

impl RoadSectionSnapshot {
    // IDs are graph-local; compare immutable authority across excerpt catalog remaps.
    pub(crate) fn matches(&self, graph: &RegionGraph, id: usize) -> bool {
        let current = graph.road_section_ref(id);
        self.reference.revision == current.revision
            && self.reference.class == current.class
            && self.reference.access == current.access
            && self.reference.overrides == current.overrides
            && self.layout.as_ref() == graph.road_layout(id)
            && self.detail.as_deref() == graph.road_section_detail(id)
    }
}
