// SPDX-License-Identifier: GPL-2.0-only

//! Local shared-grade compilation for connected junction kernels that overlap in plan.

use super::*;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::surface::material_partition::{node_grade_overlap, owner_level};
use crate::simulation::network::surface::{NodeVisualCompileResult, RoadMaterialOwner};
use crate::simulation::network::types::{EdgeClass, TransitType};
use crate::simulation::terrain::TerrainSystem;
use std::collections::HashSet;
use std::sync::Arc;

/// Recompiled products and their bounded span dependency set.
pub(in crate::simulation::network::surface) struct CohortCompileProducts {
    /// Every span whose mouth or internal grade was rebuilt.
    pub(in crate::simulation::network::surface) edges: Vec<usize>,
    /// Complete validated node products; publication remains atomic in the caller.
    pub(in crate::simulation::network::surface) nodes: Vec<(
        u32,
        crate::simulation::network::surface::RoadSurfaceVisualNodeCompileInput,
        NodeVisualCompileResult,
    )>,
}

impl RoadSurfaceSystem {
    /// Extends an edit only through previously certified shared-grade dependencies.
    pub(in crate::simulation::network::surface) fn extend_grade_cohort_nodes(
        &self,
        nodes: &mut HashSet<u32>,
    ) {
        let mut pending = nodes.iter().copied().collect::<Vec<_>>();
        let mut expanded_components = BTreeSet::new();
        while let Some(node) = pending.pop() {
            if let Some(members) = self
                .node_material_grade_fields
                .get(&node)
                .and_then(|f| f.cohort_nodes())
                && members
                    .first()
                    .is_some_and(|node| expanded_components.insert(*node))
            {
                for &member in members {
                    if nodes.insert(member) {
                        pending.push(member);
                    }
                }
            }
        }
    }

    /// Traverses positive overlaps along source road adjacency, never a city-wide dirty scan.
    /// Work is O(local incidence + intersecting triangle-grid candidates). Saved centres and
    /// exterior mouths constrain the solve; interior mouths are products, not new authorities.
    pub(in crate::simulation::network::surface) fn compile_junction_grade_cohorts(
        &mut self,
        graph: &RegionGraph,
        terrain: &TerrainSystem,
        seeds: &[u32],
        resident: Option<&Self>,
    ) -> Result<CohortCompileProducts, String> {
        let mut visited = BTreeSet::new();
        let mut groups = Vec::new();
        for &seed in seeds {
            if !visited.insert(seed) {
                continue;
            }
            let mut members = vec![seed];
            let mut cursor = 0;
            let mut conflict = false;
            while cursor < members.len() {
                let node = members[cursor];
                cursor += 1;
                let Some(a) = self
                    .compiled_visual_node_pieces
                    .get(&node)
                    .or_else(|| resident.and_then(|r| r.compiled_visual_node_pieces.get(&node)))
                else {
                    continue;
                };
                let Some(level) = owner_level(graph, RoadMaterialOwner::Node(node)) else {
                    continue;
                };
                for &id in graph.node_adjacency(node) {
                    let edge = graph.edge(id);
                    if edge.deleted
                        || edge.class != EdgeClass::Standard
                        || edge.primary_type != TransitType::Road
                        || edge.crossing_level != level
                    {
                        continue;
                    }
                    let start = graph.get_valid_node(edge.start_node);
                    let other = if start == node {
                        graph.get_valid_node(edge.end_node)
                    } else {
                        start
                    };
                    if owner_level(graph, RoadMaterialOwner::Node(other)) != Some(level) {
                        continue;
                    }
                    let Some(b) = self.compiled_visual_node_pieces.get(&other).or_else(|| {
                        resident.and_then(|r| r.compiled_visual_node_pieces.get(&other))
                    }) else {
                        continue;
                    };
                    if let Some(incompatible) = node_grade_overlap(a, b, true) {
                        conflict |= incompatible;
                        if visited.insert(other) {
                            members.push(other);
                        }
                    }
                }
            }
            if conflict && members.len() > 1 {
                members.sort_unstable();
                groups.push(members);
            }
        }
        let mut products = CohortCompileProducts {
            edges: Vec::new(),
            nodes: Vec::new(),
        };
        let mut prepared = Vec::new();
        for members in groups {
            let member_set = members.iter().copied().collect::<BTreeSet<_>>();
            let edges = members
                .iter()
                .flat_map(|&node| graph.node_adjacency(node).iter().copied())
                .filter(|&edge| Self::is_surface_edge(graph.edge(edge)))
                .collect::<BTreeSet<_>>();
            let internal = edges
                .iter()
                .copied()
                .filter(|&id| {
                    let e = graph.edge(id);
                    e.class == EdgeClass::Standard
                        && member_set.contains(&graph.get_valid_node(e.start_node))
                        && member_set.contains(&graph.get_valid_node(e.end_node))
                })
                .collect::<BTreeSet<_>>();
            // Import unchanged local dependencies by Arc; the staged maps stay neighborhood-sized.
            if let Some(resident) = resident {
                for &edge in &edges {
                    if let Some(sections) = resident.compiled_sections.get(&edge) {
                        self.compiled_sections
                            .entry(edge)
                            .or_insert_with(|| Arc::clone(sections));
                    }
                    if let Some(piece) = resident.compiled_visual_span_pieces.get(&edge) {
                        self.compiled_visual_span_pieces
                            .entry(edge)
                            .or_insert_with(|| Arc::clone(piece));
                    }
                }
            }
            let inputs = members
                .iter()
                .map(|&node| {
                    let input = self.visual_node_compile_input(graph, node)?;
                    Some((
                        node,
                        Self::build_node_arrangement_input_from_mouths(
                            node,
                            input.kind,
                            &input.mouths,
                        )
                        .ok()?,
                    ))
                })
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| format!("cohort_grade_input nodes={members:?}"))?;
            let mut bounds = [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for member in &members {
                let piece = self
                    .compiled_visual_node_pieces
                    .get(member)
                    .or_else(|| resident.and_then(|r| r.compiled_visual_node_pieces.get(member)))
                    .ok_or_else(|| format!("cohort_grade_missing_node node={member}"))?;
                let b = piece.surface_query.bounds_xz;
                bounds = [
                    bounds[0].min(b[0]),
                    bounds[1].min(b[1]),
                    bounds[2].max(b[2]),
                    bounds[3].max(b[3]),
                ];
            }
            for edge in &internal {
                if let Some(piece) = self.compiled_visual_span_pieces.get(edge) {
                    let b = piece.surface_query.bounds_xz;
                    bounds = [
                        bounds[0].min(b[0]),
                        bounds[1].min(b[1]),
                        bounds[2].max(b[2]),
                        bounds[3].max(b[3]),
                    ];
                }
            }
            // Ownership after band coalescing can include domains omitted by the
            // initial top query. Enclose the complete source geometry before solving
            // any shared grade, so later node compilation cannot leave its facets.
            for (node, input) in &inputs {
                let mut input = input.clone();
                input.coalesce_grade_subdivisions();
                let (rails, _) =
                    Self::build_node_rail_contours_from_input_with_profile(&input, false)
                        .map_err(|_| format!("cohort_grade_domain_rails node={node}"))?;
                for p in rails.contours.iter().flat_map(|c| c.points_xz.iter()) {
                    bounds = [
                        bounds[0].min(p.x),
                        bounds[1].min(p.y),
                        bounds[2].max(p.x),
                        bounds[3].max(p.y),
                    ];
                }
            }
            let field = Arc::new(
                NodeMaterialGradeField::for_cohort(graph, &inputs, &internal, bounds)
                    .ok_or_else(|| format!("cohort_grade_source_support nodes={members:?}"))?,
            );
            prepared.push((members, edges, internal, field));
        }
        // Solve every field from the same source-mouth snapshot. Adjacent cohorts can
        // share an exterior span: one cohort's generated subdivisions are not source
        // supports for the next cohort, and both endpoint fields must exist before
        // their common span is conformed.
        for (members, _, _, field) in &prepared {
            for &node in members {
                self.node_material_grade_fields
                    .insert(node, Arc::clone(field));
            }
        }
        for (members, edges, internal, field) in &prepared {
            products.edges.extend(edges.iter().copied());
            for &edge in internal {
                let mut sections = self
                    .compiled_sections
                    .get(&edge)
                    .ok_or_else(|| format!("cohort_grade_missing_sections edge={edge}"))?
                    .as_ref()
                    .clone();
                for section in &mut sections {
                    let bands = &section.bands;
                    let road = bands
                        .iter()
                        .position(|b| b.kind == RoadSurfaceBandKind::Carriageway)
                        .ok_or_else(|| format!("cohort_grade_missing_carriageway edge={edge}"))?;
                    let mut levels = vec![0.0; bands.len()];
                    for i in road..bands.len().saturating_sub(1) {
                        levels[i + 1] = levels[i]
                            + f64::from(bands[i + 1].height_start_m - bands[i].height_end_m);
                    }
                    for i in (0..road).rev() {
                        levels[i] = levels[i + 1]
                            - f64::from(bands[i + 1].height_start_m - bands[i].height_end_m);
                    }
                    for (band, level) in section.bands.iter_mut().zip(levels) {
                        let p = |l| section.center_xz + section.lateral_xz * f64::from(l);
                        band.height_start_m =
                            (field.height_at(p(band.lateral_start_m)) + level) as f32;
                        band.height_end_m = (field.height_at(p(band.lateral_end_m)) + level) as f32;
                    }
                    section.center_height_m = field.height_at(section.center_xz) as f32;
                }
                self.compiled_sections.insert(edge, Arc::new(sections));
                let piece = self
                    .compile_visual_span_piece(graph, terrain, edge)
                    .ok_or_else(|| format!("cohort_grade_span edge={edge} nodes={members:?}"))?;
                self.apply_span_compile_result(edge, Some(piece));
            }
        }
        products.edges.sort_unstable();
        products.edges.dedup();
        let failed = self.conform_junction_mouth_grades(graph, terrain, &products.edges, Some(&[]));
        if !failed.is_empty() {
            return Err(format!("cohort_grade_mouth_spans edges={failed:?}"));
        }
        #[cfg(test)]
        let collect_failures = self.retain_partial_validation_artifacts;
        #[cfg(not(test))]
        let collect_failures = false;
        let mut failures = Vec::new();
        for (members, _, internal, _) in prepared {
            let results = Self::collect_compile_work(
                &members,
                crate::simulation::network::surface::PARALLEL_NODE_COMPILE_MIN_ITEMS,
                |node| {
                    let input = self.visual_node_compile_input(graph, node)?;
                    let result = self.compile_visual_node_piece_with_earthwork_boundaries(
                        graph, terrain, node, &input, None,
                    )?;
                    Some((node, input, result))
                },
            );
            for (i, result) in results.into_iter().enumerate() {
                match result {
                    Some(result) => products.nodes.push(result),
                    None => {
                        let failure =
                            format!("cohort_grade_node node={} nodes={members:?}", members[i]);
                        if !collect_failures {
                            return Err(failure);
                        }
                        failures.push(failure);
                    }
                }
            }
            crate::debug_log!(
                "road",
                "junction_grade_cohort nodes={:?} internal_edges={:?}",
                members,
                internal
            );
        }
        products.edges.sort_unstable();
        products.edges.dedup();
        products.nodes.sort_unstable_by_key(|p| p.0);
        if !failures.is_empty() {
            return Err(failures.join("; "));
        }
        Ok(products)
    }
}
