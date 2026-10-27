// SPDX-License-Identifier: GPL-2.0-only

//! Node triangulation entry points.

use super::coverage::overlay_shape_from_arrangement_region;
use super::regions::triangulate_arrangement_region;
use super::*;
use crate::simulation::network::surface::node::height::NodeMaterialGradeField;
use rayon::prelude::*;

const PARALLEL_NODE_REGION_TRIANGULATION_MIN_ITEMS: usize = 8;

enum NodeTriangulatedRegionBuildResult {
    Region(NodeTriangulatedRegion),
    Discarded(NodeTriangulationError),
}

impl RoadSurfaceSystem {
    #[cfg(test)]
    pub(in crate::simulation::network::surface) fn build_node_triangulation_from_arrangement(
        arrangement: &NodeArrangement,
    ) -> Result<NodeTriangulationSolution, NodeTriangulationError> {
        NodeTriangulationSolution::from_arrangement(arrangement)
    }

    /// Triangulates with the resolved grade and any already extracted material steps.
    pub(in crate::simulation::network::surface) fn build_node_triangulation_from_arrangement_with_grade(
        arrangement: &NodeArrangement,
        material_grade: Option<&NodeMaterialGradeField>,
        explicit_steps: Option<&[NodeExplicitVerticalStepSegment]>,
    ) -> Result<NodeTriangulationSolution, NodeTriangulationError> {
        NodeTriangulationSolution::from_arrangement_with_optional_explicit_steps(
            arrangement,
            explicit_steps,
            material_grade,
        )
    }
}

impl NodeTriangulationSolution {
    #[cfg(test)]
    pub(crate) fn from_arrangement(
        arrangement: &NodeArrangement,
    ) -> Result<Self, NodeTriangulationError> {
        Self::from_arrangement_with_optional_explicit_steps(arrangement, None, None)
    }

    fn from_arrangement_with_optional_explicit_steps(
        arrangement: &NodeArrangement,
        explicit_vertical_step_segments: Option<&[NodeExplicitVerticalStepSegment]>,
        material_grade: Option<&NodeMaterialGradeField>,
    ) -> Result<Self, NodeTriangulationError> {
        if arrangement.regions().is_empty() {
            return Err(NodeTriangulationError::EmptyHeightSolution {
                node_id: arrangement.node_id(),
            });
        }

        let triangulated_regions =
            if arrangement.regions().len() >= PARALLEL_NODE_REGION_TRIANGULATION_MIN_ITEMS {
                // A node compile on Godot's main thread runs outside any pool (CODE-17).
                crate::thread_pools::install(|| {
                    arrangement
                        .regions()
                        .par_iter()
                        .enumerate()
                        .map(|(region_index, region)| {
                            triangulate_arrangement_region_for_solution(
                                arrangement,
                                region_index,
                                region,
                                material_grade,
                            )
                        })
                        .collect::<Vec<_>>()
                })
            } else {
                arrangement
                    .regions()
                    .iter()
                    .enumerate()
                    .map(|(region_index, region)| {
                        triangulate_arrangement_region_for_solution(
                            arrangement,
                            region_index,
                            region,
                            material_grade,
                        )
                    })
                    .collect::<Vec<_>>()
            };

        let mut regions = Vec::with_capacity(triangulated_regions.len());
        let mut first_discarded_error = None;
        for result in triangulated_regions {
            match result {
                Ok(NodeTriangulatedRegionBuildResult::Region(region)) => regions.push(region),
                Ok(NodeTriangulatedRegionBuildResult::Discarded(error)) => {
                    first_discarded_error.get_or_insert(error);
                }
                Err(error) => return Err(error),
            }
        }

        if regions.is_empty()
            && let Some(error) = first_discarded_error
        {
            return Err(error);
        }

        Ok(Self {
            node_id: arrangement.node_id(),
            piece_kind: arrangement.piece_kind(),
            regions,
            explicit_vertical_step_segments: explicit_vertical_step_segments
                .map(|segments| segments.to_vec())
                .unwrap_or_else(|| arrangement.explicit_vertical_step_segments()),
        })
    }
}

fn triangulate_arrangement_region_for_solution(
    arrangement: &NodeArrangement,
    region_index: usize,
    region: &NodeOwnedRegion,
    material_grade: Option<&NodeMaterialGradeField>,
) -> Result<NodeTriangulatedRegionBuildResult, NodeTriangulationError> {
    triangulate_arrangement_region(
        arrangement.node_id(),
        region_index,
        arrangement,
        region,
        material_grade,
    )
    .map(NodeTriangulatedRegionBuildResult::Region)
    .or_else(|error| {
        if triangulation_error_is_discardable_numeric_region(&error, arrangement, region) {
            Ok(NodeTriangulatedRegionBuildResult::Discarded(error))
        } else {
            Err(error)
        }
    })
}

fn triangulation_error_is_discardable_numeric_region(
    error: &NodeTriangulationError,
    arrangement: &NodeArrangement,
    region: &NodeOwnedRegion,
) -> bool {
    if !matches!(
        error,
        NodeTriangulationError::DegenerateRegionContour { .. }
            | NodeTriangulationError::EmptyTriangulation { .. }
    ) {
        return false;
    }
    arrangement_region_is_numeric(arrangement, region)
}

/// Certifies a discarded top region against the existing overlay numeric-area budget.
pub(in crate::simulation::network::surface) fn arrangement_region_is_numeric(
    arrangement: &NodeArrangement,
    region: &NodeOwnedRegion,
) -> bool {
    let shape = overlay_shape_from_arrangement_region(arrangement, region);
    let area_m2 = RoadSurfaceSystem::overlay_shape_area_m2(&shape);
    area_m2 <= RoadSurfaceSystem::overlay_numeric_area_budget_for_shape(&shape)
}
