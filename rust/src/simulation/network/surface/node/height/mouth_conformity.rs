// SPDX-License-Identifier: GPL-2.0-only

//! Conforms span cross-sections to the piecewise grade shared with their junction mouths.

use super::*;
use crate::simulation::network::surface::{IncidentEdgeSide, RoadSurfaceBand, RoadSurfaceSection};
use std::sync::Arc;

struct BandProjection {
    start: RoadVec3,
    end: RoadVec3,
    level: f64,
}

struct MouthProjection {
    station: f32,
    field: Arc<NodeMaterialGradeField>,
    bands: Vec<Option<BandProjection>>,
}

struct NodeProjection {
    field: Arc<NodeMaterialGradeField>,
    mouths: Vec<(usize, usize, MouthProjection)>,
}

impl BandProjection {
    fn correction(&self, field: &NodeMaterialGradeField, t: f64) -> f64 {
        let point = self.start.lerp(self.end, t);
        field.height_at(xz(point)) + self.level - point.y
    }
}

impl RoadSurfaceSystem {
    /// Splits mouth bands at actual grade-triangle crossings before either owner is published.
    pub(in crate::simulation::network::surface) fn conform_junction_mouth_grades(
        &mut self,
        graph: &crate::simulation::network::graph::RegionGraph,
        terrain: &crate::simulation::terrain::TerrainSystem,
        edges: &[usize],
        solve_nodes: Option<&[u32]>,
    ) -> Vec<usize> {
        let nodes = edges
            .iter()
            .filter(|&&id| id < graph.edge_count())
            .flat_map(|&id| {
                let edge = graph.edge(id);
                [
                    graph.get_valid_node(edge.start_node),
                    graph.get_valid_node(edge.end_node),
                ]
            })
            .collect::<BTreeSet<_>>();
        let nodes = nodes.into_iter().collect::<Vec<_>>();
        let solved = Self::collect_compile_work(
            &nodes,
            super::super::super::PARALLEL_NODE_COMPILE_MIN_ITEMS,
            |node| {
                let solve = solve_nodes.is_none_or(|nodes| nodes.binary_search(&node).is_ok());
                (node, solve, self.node_grade_projection(graph, node, solve))
            },
        );
        let mut projections = BTreeMap::<usize, [Option<MouthProjection>; 2]>::new();
        for (node, solve, projection) in solved {
            if solve {
                self.node_material_grade_fields.remove(&node);
            }
            if let Some(projection) = projection {
                if solve {
                    self.node_material_grade_fields
                        .insert(node, projection.field);
                }
                for (edge, side, mouth) in projection.mouths {
                    projections.entry(edge).or_insert_with(|| [None, None])[side] = Some(mouth);
                }
            }
        }
        let updates = Self::collect_compile_work(
            edges,
            super::super::super::PARALLEL_SPAN_COMPILE_MIN_ITEMS,
            |edge| {
                let sections = self.compiled_sections.get(&edge).and_then(|original| {
                    projections
                        .get(&edge)
                        .and_then(|ends| conformed_sections(original, ends))
                });
                (edge, sections)
            },
        );
        let mut changed = Vec::new();
        for (edge, sections) in updates {
            if let Some((sections, parents)) = sections {
                self.compiled_sections.insert(edge, Arc::new(sections));
                changed.push((edge, parents));
            }
        }
        let rebuilt = Self::collect_compile_work(
            &changed,
            super::super::super::PARALLEL_SPAN_COMPILE_MIN_ITEMS,
            |(edge, ref parents)| {
                let piece = self
                    .compiled_visual_span_pieces
                    .get(&edge)
                    .and_then(|source| {
                        self.compile_visual_span_subdivision(graph, terrain, edge, source, parents)
                    });
                (edge, piece)
            },
        );
        let mut failed = Vec::new();
        for (edge, piece) in rebuilt {
            if let Some(piece) = piece {
                self.retain_span_sections(&piece);
                self.compiled_visual_span_pieces
                    .insert(edge, Arc::new(piece));
            } else {
                failed.push(edge);
            }
        }
        failed
    }
    fn node_grade_projection(
        &self,
        graph: &crate::simulation::network::graph::RegionGraph,
        node: u32,
        solve: bool,
    ) -> Option<NodeProjection> {
        let input = self.visual_node_compile_input(graph, node)?;
        let field = if solve {
            if input
                .mouth_edge_classes
                .iter()
                .any(|&class| class != crate::simulation::network::types::EdgeClass::Standard)
            {
                return None;
            }
            let input =
                Self::build_node_arrangement_input_from_mouths(node, input.kind, &input.mouths)
                    .ok()?;
            let field = NodeMaterialGradeField::for_input(&input)?;
            if let Some(previous) = self.node_material_grade_fields.get(&node)
                && field.same_source(previous)
            {
                Arc::clone(previous)
            } else {
                Arc::new(field)
            }
        } else {
            Arc::clone(self.node_material_grade_fields.get(&node)?)
        };
        let mut mouths = Vec::with_capacity(input.mouths.len());
        for mouth in &input.mouths {
            let Some(sections) = self.compiled_sections.get(&mouth.edge_idx) else {
                continue;
            };
            let Some(section) = sections.iter().find(|section| {
                Self::build_mouth_profile_from_section(section, mouth.side).is_some_and(|profile| {
                    profile.boundary_points_world == mouth.profile.boundary_points_world
                })
            }) else {
                continue;
            };
            let bands = section
                .bands
                .iter()
                .map(|band| {
                    if band.lateral_end_m - band.lateral_start_m <= 0.001 {
                        return None;
                    }
                    let start = Self::section_boundary_world_point_static(
                        section,
                        band.lateral_start_m,
                        band.height_start_m,
                    );
                    let end = Self::section_boundary_world_point_static(
                        section,
                        band.lateral_end_m,
                        band.height_end_m,
                    );
                    let level = start.y - field.height_at(xz(start));
                    let level_key = SurfaceHeightMmKey::from_m_f64(level).as_i64();
                    let end_key =
                        SurfaceHeightMmKey::from_m_f64(end.y - field.height_at(xz(end))).as_i64();
                    (matches!(level_key, 0 | 120) && level_key.abs_diff(end_key) <= 1).then_some(
                        BandProjection {
                            start,
                            end,
                            level: level_key as f64 / 1000.0,
                        },
                    )
                })
                .collect();
            let side = usize::from(mouth.side == IncidentEdgeSide::End);
            mouths.push((
                mouth.edge_idx,
                side,
                MouthProjection {
                    station: section.s_m,
                    field: Arc::clone(&field),
                    bands,
                },
            ));
        }
        Some(NodeProjection { field, mouths })
    }
}

fn conformed_sections(
    original: &[RoadSurfaceSection],
    ends: &[Option<MouthProjection>; 2],
) -> Option<(Vec<RoadSurfaceSection>, Vec<usize>)> {
    let first = original.first()?;
    let mut splits = vec![vec![0.0, 1.0]; first.bands.len()];
    for (band_index, parameters) in splits.iter_mut().enumerate() {
        for projection in ends.iter().flatten() {
            let Some(band) = projection.bands.get(band_index).and_then(Option::as_ref) else {
                continue;
            };
            let breaks = projection.field.band_breaks(xz(band.start), xz(band.end));
            if breaks.iter().any(|&t| {
                SurfaceHeightMmKey::from_m_f64(band.correction(&projection.field, t))
                    .as_i64()
                    .abs()
                    > 1
            }) {
                parameters.extend(breaks);
            }
        }
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|a, b| *a == *b);
    }
    for (index, parameters) in splits.iter_mut().enumerate() {
        *parameters = simplify_grade_breaks(parameters, ends, index);
    }
    // A terminal cap pairs material layers from the inside out. Preserve that
    // exact pairing when a derived grade splits one side's material band.
    for left in 0..splits.len() / 2 {
        let right = splits.len() - 1 - left;
        if first.bands[left].kind != first.bands[right].kind {
            continue;
        }
        let mut paired = splits[left].clone();
        paired.extend(splits[right].iter().map(|t| 1.0 - t));
        paired.sort_by(f64::total_cmp);
        paired.dedup_by(|a, b| *a == *b);
        splits[left] = paired.clone();
        splits[right] = paired.into_iter().rev().map(|t| 1.0 - t).collect();
    }
    if splits.iter().all(|split| split.len() == 2) {
        return None;
    }
    let total = original.last().unwrap().s_m;
    let start = ends[0].as_ref().map_or(0.0, |end| end.station);
    let end = ends[1].as_ref().map_or(total, |end| end.station);
    let mut sections = original.to_vec();
    for section in &mut sections {
        let (start_weight, end_weight) = projection_weights(section.s_m, start, end, total);
        let mut bands = Vec::new();
        for (index, band) in section.bands.iter().enumerate() {
            let boundary = |t: f64| {
                let lateral = f64::from(band.lateral_start_m)
                    + f64::from(band.lateral_end_m - band.lateral_start_m) * t;
                let mut height = f64::from(band.height_start_m)
                    + f64::from(band.height_end_m - band.height_start_m) * t;
                if t != 0.0 && t != 1.0 {
                    for (side, weight) in [start_weight, end_weight].into_iter().enumerate() {
                        if let Some(projection) = &ends[side]
                            && let Some(source) =
                                projection.bands.get(index).and_then(Option::as_ref)
                        {
                            height += weight * source.correction(&projection.field, t);
                        }
                    }
                }
                (lateral as f32, height as f32)
            };
            for pair in splits[index].windows(2) {
                let a = boundary(pair[0]);
                let b = boundary(pair[1]);
                bands.push(RoadSurfaceBand {
                    kind: band.kind,
                    lateral_start_m: a.0,
                    lateral_end_m: b.0,
                    height_start_m: a.1,
                    height_end_m: b.1,
                });
            }
        }
        section.bands = bands;
    }
    let parents = splits
        .iter()
        .enumerate()
        .flat_map(|(i, split)| std::iter::repeat_n(i, split.len() - 1))
        .collect();
    Some((sections, parents))
}

fn projection_weights(station: f32, start: f32, end: f32, total: f32) -> (f64, f64) {
    if station < start {
        return (f64::from(station / start), 0.0);
    }
    if station > end {
        return (0.0, f64::from((total - station) / (total - end)));
    }
    let t = f64::from((station - start) / (end - start));
    (1.0 - t, t)
}

// Every omitted grade breakpoint is certified against both endpoint profiles in the
// existing millimetre height-tie budget. Between breakpoints each source is affine.
fn simplify_grade_breaks(
    parameters: &[f64],
    ends: &[Option<MouthProjection>; 2],
    band: usize,
) -> Vec<f64> {
    let values = parameters
        .iter()
        .map(|&t| {
            [0, 1].map(|side| {
                if t == 0.0 || t == 1.0 {
                    return 0_i64;
                }
                ends[side]
                    .as_ref()
                    .and_then(|end| {
                        end.bands.get(band).and_then(Option::as_ref).map(|source| {
                            SurfaceHeightMmKey::from_m_f64(source.correction(&end.field, t))
                                .as_i64()
                        })
                    })
                    .unwrap_or(0)
            })
        })
        .collect::<Vec<_>>();
    let mut retained = vec![false; parameters.len()];
    retained[0] = true;
    retained[parameters.len() - 1] = true;
    let mut work = vec![(0, parameters.len() - 1)];
    while let Some((start, end)) = work.pop() {
        let split = (start + 1..end).find(|&index| {
            let t = (parameters[index] - parameters[start]) / (parameters[end] - parameters[start]);
            (0..2).any(|side| {
                let interpolated = (values[start][side] as f64
                    + (values[end][side] - values[start][side]) as f64 * t)
                    .round() as i64;
                interpolated.abs_diff(values[index][side]) > 1
            })
        });
        if let Some(split) = split {
            retained[split] = true;
            work.push((split, end));
            work.push((start, split));
        }
    }
    parameters
        .iter()
        .zip(retained)
        .filter_map(|(&t, retained)| retained.then_some(t))
        .collect()
}
