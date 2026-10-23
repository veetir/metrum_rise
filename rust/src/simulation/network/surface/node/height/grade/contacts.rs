// SPDX-License-Identifier: GPL-2.0-only

//! Simultaneous node material-contact grading from declared source cross-section levels.

use super::super::super::input::NodeArrangementInput;
use super::*;

/// Resolves all material contacts at each canonical point together, before arrangement noding.
pub(in crate::simulation::network::surface::node::height) fn apply_source_material_contact_grades(
    input: &NodeArrangementInput,
    regions: &mut [NodeHeightedRegion],
    grade: Option<&super::common_field::NodeMaterialGradeField>,
) -> bool {
    let Some(mut levels) = declared_levels(input) else {
        return false;
    };
    for region in regions.iter() {
        if levels.contains_key(&region.height_field_id) {
            continue;
        }
        // A terminal cap has generated band identities, but inherits its material
        // level from the one source cross-section. It shares the approach grade
        // even where a tight end curve overlaps the cap in plan.
        if input.piece_kind != super::super::super::RoadSurfaceVisualNodePieceKind::Terminal {
            return false;
        }
        let mut candidates = levels
            .iter()
            .filter_map(|(id, &level)| (id.kind() == region.kind).then_some(level));
        let Some(level) = candidates.next() else {
            return false;
        };
        if candidates.any(|other| other != level) {
            return false;
        }
        levels.insert(region.height_field_id, level);
    }
    let built = grade
        .is_none()
        .then(|| super::common_field::NodeMaterialGradeField::from_input(input, &levels))
        .flatten();
    let Some(field) = grade.or(built.as_ref()) else {
        return false;
    };

    let owner_levels = regions
        .iter()
        .map(|region| (region.owner, levels[&region.height_field_id]))
        .collect::<BTreeMap<_, _>>();
    for region in regions.iter_mut() {
        for constraint in &mut region.seam_constraints {
            if let (Some(a), Some(b)) = (constraint.owner, constraint.opposite_owner)
                && let (Some(a), Some(b)) = (owner_levels.get(&a), owner_levels.get(&b))
                && a != b
            {
                // A generated contact cannot erase a cross-section's declared kerb.
                // Source-declared flat contacts still have equal levels and stay flat.
                constraint.constrains_shared_height = false;
            }
        }
    }
    let indices = regions
        .iter()
        .map(NodeGradeRegionConstraintIndex::from_region)
        .collect::<Vec<_>>();
    let mut points = BTreeMap::<SurfaceXzKey, Vec<(usize, usize, usize)>>::new();
    for (r, region) in regions.iter().enumerate() {
        for (c, contour) in region.shape.iter().enumerate() {
            for (v, vertex) in contour.iter().enumerate() {
                points
                    .entry(SurfaceXzKey::from_road_xz(vertex.point_xz))
                    .or_default()
                    .push((r, c, v));
            }
        }
    }
    for occurrences in points.values() {
        let mut owners = occurrences
            .iter()
            .map(|&(r, _, _)| regions[r].owner)
            .collect::<Vec<_>>();
        owners.sort_unstable();
        owners.dedup();
        let mut components = (0..owners.len()).collect::<Vec<_>>();
        for a in 0..owners.len() {
            for b in a + 1..owners.len() {
                if owners[a].kind() == owners[b].kind() {
                    merge(&mut components, a, b);
                }
            }
        }
        for &(r, c, v) in occurrences {
            let region = &regions[r];
            let vertex = &region.shape[c][v];
            for constraint in indexed_material_height_constraints_for_vertex(
                vertex.point_xz,
                &region.seam_constraints,
                &indices[r],
            ) {
                if let (Some(a), Some(b)) = (constraint.owner, constraint.opposite_owner)
                    && let (Ok(a), Ok(b)) = (owners.binary_search(&a), owners.binary_search(&b))
                {
                    merge(&mut components, a, b);
                }
            }
        }
        let mut offsets = vec![f64::INFINITY; owners.len()];
        for &(r, _, _) in occurrences {
            let owner = owners.binary_search(&regions[r].owner).unwrap();
            let component = components[owner];
            offsets[component] = offsets[component].min(levels[&regions[r].height_field_id]);
        }
        let (r, c, v) = occurrences[0];
        let base = field.height_at(regions[r].shape[c][v].point_xz);
        for &(r, c, v) in occurrences {
            let region = &mut regions[r];
            let owner = owners.binary_search(&region.owner).unwrap();
            set_vertex_grade_height(
                region.owner,
                &mut region.shape[c][v],
                base + offsets[components[owner]],
                NodeGradeCarrierDecision::DeclaredMaterialContact {
                    base_height_mm: SurfaceHeightMmKey::from_m_f64(base).as_i64(),
                    level_mm: SurfaceHeightMmKey::from_m_f64(offsets[components[owner]]).as_i64(),
                },
            );
        }
    }
    true
}

fn merge(components: &mut [usize], a: usize, b: usize) {
    let from = components[b];
    let to = components[a];
    if from != to {
        for value in components {
            if *value == from {
                *value = to;
            }
        }
    }
}

pub(super) fn declared_levels(
    input: &NodeArrangementInput,
) -> Option<BTreeMap<NodeBandHeightFieldId, f64>> {
    let step_mm = (f64::from(crate::simulation::network::surface::CURB_STEP_HEIGHT_M) * 1000.0)
        .round() as i64;
    let mut result = BTreeMap::new();
    for mouth in &input.mouths {
        let bands = &mouth.band_intervals;
        let road = bands
            .iter()
            .position(|band| band.band_kind == RoadSurfaceBandKind::Carriageway)?;
        let mut levels = vec![0_i64; bands.len()];
        let delta = |index: usize| {
            let a = &bands[index];
            let b = &bands[index + 1];
            let mut accepted = None;
            for (left, right) in [
                (a.mouth_end_world, b.mouth_start_world),
                (a.endpoint_end_world, b.endpoint_start_world),
            ] {
                if SurfaceXzKey::from_road_xz(RoadVec2::new(left.x, left.z))
                    != SurfaceXzKey::from_road_xz(RoadVec2::new(right.x, right.z))
                {
                    return None;
                }
                let difference = ((right.y - left.y) * 1000.0).round() as i64;
                if difference != 0 && difference.abs() != step_mm {
                    return None;
                }
                if accepted.is_some_and(|value| value != difference) {
                    return None;
                }
                accepted = Some(difference);
            }
            accepted
        };
        for index in road..bands.len().saturating_sub(1) {
            levels[index + 1] = levels[index] + delta(index)?;
        }
        for index in (0..road).rev() {
            levels[index] = levels[index + 1] - delta(index)?;
        }
        for (band, level) in bands.iter().zip(levels) {
            if match band.band_kind {
                RoadSurfaceBandKind::Carriageway => level != 0,
                RoadSurfaceBandKind::CurbOrShoulder
                | RoadSurfaceBandKind::Sidewalk
                | RoadSurfaceBandKind::Median => level != 0 && level != step_mm,
                _ => true,
            } {
                return None;
            }
            result.insert(
                NodeBandHeightFieldId::new(mouth.order_index, band.band_index, band.band_kind),
                level as f64 / 1000.0,
            );
        }
    }
    Some(result)
}
