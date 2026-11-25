// SPDX-License-Identifier: GPL-3.0-or-later

//! Local picking, validation and topology transactions for the Zebra road tool.

use std::collections::HashSet;

use godot::prelude::{Vector2, Vector3};

use super::core::{RoadEditSite, RoadLandSource, RoadRejection, SimCore, TriedGraphEdit};
use crate::simulation::network::graph::{Edge, RegionGraph, SidewalkSides};
use crate::simulation::network::pockets::{
    PocketApplied, join_zebra_pair, migrate_pocket_dependents,
};
use crate::simulation::network::roadbed::{
    ArmKey, CROSSWALK_DEPTH_M, EdgeEnd, MAX_ROADBED_HALF_WIDTH_M, crossing_arm_eligible,
    plain_road_joint, visual_roadbed_half_width_m,
};
use crate::simulation::network::topology::{SPLIT_KNOT_MERGE_M, split_edge_for_crossing};
use crate::simulation::network::types::{EdgeClass, NodeType, TransitType};
use crate::simulation::network::validate::centreline;

const HALF_STRIP: f32 = CROSSWALK_DEPTH_M as f32 * 0.5;
type TriedZebra<B> = (TriedGraphEdit<B>, Option<[Vector3; 2]>);

#[derive(Clone, Copy)]
enum Action {
    Split {
        edge: usize,
        segment: usize,
        cut: Vector3,
    },
    Override {
        node: u32,
        edge: usize,
        enabled: bool,
    },
    Join {
        node: u32,
        first: usize,
        second: usize,
    },
}

/// The pose and verdict Godot displays for a single Zebra click.
pub(crate) struct ZebraPreview {
    /// Kerbs across the carriageway, and the longitudinal slope at the crossing line.
    pub(crate) pose: Option<([Vector3; 2], f32)>,
    /// Whether the click would remove an existing zebra.
    pub(crate) removing: bool,
    /// The local edit's refusal, if any.
    pub(crate) rejection: Option<RoadRejection>,
}

impl ZebraPreview {
    /// The pointer did not intersect a road surface.
    pub(crate) fn no_road() -> Self {
        Self {
            pose: None,
            removing: false,
            rejection: Some(refusal("zebra_no_road")),
        }
    }
}

struct Projection {
    point: Vector3,
    axis: Vector3,
    station: f32,
    length: f32,
}

fn xz(p: Vector3) -> Vector2 {
    Vector2::new(p.x, p.z)
}

fn project(points: &[Vector3], pointer: Vector3) -> Option<Projection> {
    let mut length = 0.0;
    let mut best_distance = f32::INFINITY;
    let mut best = None;
    for pair in points.windows(2) {
        let axis = pair[1] - pair[0];
        let horizontal = xz(axis);
        let run = RegionGraph::edge_profile_point_distance_m(pair[0], pair[1]);
        if run == 0.0 {
            continue;
        }
        let fraction = (xz(pointer - pair[0]).dot(horizontal) / (run * run)).clamp(0.0, 1.0);
        let point = pair[0].lerp(pair[1], fraction);
        let distance = xz(pointer - point).length_squared();
        if distance < best_distance {
            best_distance = distance;
            best = Some(Projection {
                point,
                axis: axis / run,
                station: length + fraction * run,
                length: 0.0,
            });
        }
        length += run;
    }
    best.map(|mut hit| {
        hit.length = length;
        hit
    })
}

fn sample(points: &[Vector3], station: f32) -> Option<(usize, Vector3)> {
    let mut walked = 0.0;
    for (segment, pair) in points.windows(2).enumerate() {
        let run = RegionGraph::edge_profile_point_distance_m(pair[0], pair[1]);
        if run > 0.0 && station <= walked + run {
            return Some((segment, pair[0].lerp(pair[1], (station - walked) / run)));
        }
        walked += run;
    }
    None
}

// Use station distance to ignore nearby knots on a different part of a curved road. The solved
// profile supplies the height when both polylines have a knot at the same station.
fn crossing_cut(edge: &Edge, station: f32) -> Option<(f32, usize, Vector3)> {
    let physical = centreline(edge);
    let mut nearest = None;
    let mut distance = SPLIT_KNOT_MERGE_M;
    for points in [physical, edge.geometry.as_slice()] {
        let mut walked = 0.0;
        for pair in points.windows(2) {
            walked += RegionGraph::edge_profile_point_distance_m(pair[0], pair[1]);
            let gap = (station - walked).abs();
            if gap <= SPLIT_KNOT_MERGE_M && (nearest.is_none() || gap < distance) {
                nearest = Some((walked, pair[1]));
                distance = gap;
            }
            if walked > station + SPLIT_KNOT_MERGE_M {
                break;
            }
        }
    }
    let actual = nearest.map_or(station, |(station, _)| station);
    let (segment, mut cut) = sample(&edge.geometry, actual)?;
    if let Some((_, knot)) = nearest {
        cut = knot;
    } else {
        cut.y = RegionGraph::profile_height_at(physical, cut.x, cut.z);
    }
    Some((actual, segment, cut))
}

fn refusal(key: &'static str) -> RoadRejection {
    RoadRejection::new(key, key)
}

// Hit the published strip, including its full width. Resolve overlaps by distance to the
// crossing line, then by its stable owner, with no dependence on R-tree traversal order.
fn strip_hit(kerbs: [Vector3; 2], pointer: Vector3) -> Option<f32> {
    let across = xz(kerbs[1] - kerbs[0]);
    let offset = xz(pointer - kerbs[0]);
    let length = across.length();
    if length == 0.0 || !(0.0..=length).contains(&(offset.dot(across) / length)) {
        return None;
    }
    let along = (across.cross(offset) / length).abs();
    (along <= HALF_STRIP).then_some(along)
}

fn removable_pair(graph: &RegionGraph, node: u32, crossing: usize) -> Option<[usize; 2]> {
    let [a, b] = plain_road_joint(graph, node)?;
    let [first, second] = if a.end == EdgeEnd::End {
        [a, b]
    } else {
        [b, a]
    };
    let overrides = &graph.node(node).crosswalk_overrides;
    (first.end == EdgeEnd::End
        && second.end == EdgeEnd::Start
        && graph.edge(first.edge).start_node != graph.edge(second.edge).end_node
        && overrides.len() == 2
        && overrides.get(&crossing) == Some(&true)
        && overrides.get(&if crossing == first.edge {
            second.edge
        } else {
            first.edge
        }) == Some(&false))
    .then_some([first.edge, second.edge])
}

impl<L: RoadLandSource> RoadEditSite<'_, L> {
    /// Resolves the pointer and tries the same edit a click will try on the live graph.
    pub(crate) fn preview_zebra(&mut self, point: Vector3) -> ZebraPreview {
        let Some((action, mut preview)) = self.zebra_at(point) else {
            return ZebraPreview::no_road();
        };
        if let Ok(action) = action {
            match self.try_zebra(action) {
                Ok((_, Some(kerbs))) => {
                    let slope = preview.pose.map_or(0.0, |(_, slope)| slope);
                    preview.pose = Some((kerbs, slope));
                }
                Ok(_) if preview.removing => {}
                Ok(_) => preview.rejection = Some(refusal("crosswalk_arm_not_eligible")),
                Err(rejection) => preview.rejection = Some(rejection),
            }
        }
        preview
    }

    // Bounded by the R-tree candidates under the pointer, their profile points and the two
    // selected edge ends' incidence. No candidate lists or whole-network crossing scan.
    fn zebra_at(&self, point: Vector3) -> Option<(Result<Action, RoadRejection>, ZebraPreview)> {
        if !point.is_finite() {
            return None;
        }
        let graph = &*self.graph;
        let reach = Vector3::new(MAX_ROADBED_HALF_WIDTH_M, 0.0, MAX_ROADBED_HALF_WIDTH_M);
        let mut road = None;
        graph.visit_edges_near_aabb(point - reach, point + reach, |id| {
            let edge = graph.edge(id);
            let Some(hit) = project(centreline(edge), point) else {
                return;
            };
            if xz(hit.point - point).length() > visual_roadbed_half_width_m(edge) {
                return;
            }
            let distance = hit.point.distance_squared_to(point);
            if road.as_ref().is_none_or(|(old_id, _, old_distance)| {
                distance < *old_distance || (distance == *old_distance && id < *old_id)
            }) {
                road = Some((id, hit, distance));
            }
        });
        let (edge, hit, _) = road?;
        let record = graph.edge(edge);
        let ends = [record.start_node, record.end_node].map(|node| graph.get_valid_node(node));
        let mut crossing = None;
        for node in ends {
            self.roadbed.for_each_crossing(graph, node, |arm, kerbs| {
                let Some(distance) = strip_hit(kerbs, point) else {
                    return;
                };
                if crossing
                    .as_ref()
                    .is_none_or(|(best, old_node, old_arm, _)| {
                        (distance, node, arm) < (*best, *old_node, *old_arm)
                    })
                {
                    crossing = Some((distance, node, arm, kerbs));
                }
            });
        }
        if let Some((_, node, arm, kerbs)) = crossing {
            let centre = (kerbs[0] + kerbs[1]) * 0.5;
            let slope =
                project(centreline(graph.edge(arm.edge)), centre).map_or(0.0, |hit| hit.axis.y);
            let action = match removable_pair(graph, node, arm.edge) {
                Some([first, second]) => Action::Join {
                    node,
                    first,
                    second,
                },
                None => Action::Override {
                    node,
                    edge: arm.edge,
                    enabled: false,
                },
            };
            return Some((
                Ok(action),
                ZebraPreview {
                    pose: Some((kerbs, slope)),
                    removing: true,
                    rejection: None,
                },
            ));
        }

        let half_width = record.width * 0.5;
        let pose = |position: Vector3, axis: Vector3| {
            let side = Vector3::new(-axis.z, 0.0, axis.x) * half_width;
            Some(([position + side, position - side], axis.y))
        };
        let mut preview = ZebraPreview {
            pose: pose(hit.point, hit.axis),
            removing: false,
            rejection: None,
        };
        let clips = [record.start_clip, record.end_clip];
        let distances = [hit.station, hit.length - hit.station];
        let mut lower = clips[0];
        let mut upper = hit.length - clips[1];
        if let Some(span) = self.roadbed.span_piece(edge) {
            lower = span.start_mouth().section.s_m;
            upper = span.end_mouth().section.s_m;
        }
        let action = (|| {
            match record.class {
                EdgeClass::Bridge => return Err(refusal("zebra_bridge")),
                EdgeClass::Tunnel => return Err(refusal("zebra_tunnel")),
                EdgeClass::Standard => {}
            }
            if record.primary_type != TransitType::Road || record.fwd_lanes + record.bkw_lanes == 0
            {
                return Err(refusal("zebra_walkway"));
            }
            if record.sidewalk_sides() != SidewalkSides::BOTH {
                return Err(refusal("zebra_sidewalks"));
            }
            for (index, node) in ends.into_iter().enumerate() {
                if graph.node(node).crosswalk_overrides.get(&edge) == Some(&false)
                    && crossing_arm_eligible(graph, node, edge)
                    && (distances[index] - clips[index]).abs() <= CROSSWALK_DEPTH_M as f32
                {
                    return Ok(Action::Override {
                        node,
                        edge,
                        enabled: true,
                    });
                }
            }
            if record.pocket.is_some() || record.roundabout.is_some() {
                return Err(refusal("crosswalk_arm_not_eligible"));
            }
            // Store the node at the strip's leading edge so the crossing line follows the
            // pointer. Leave each resident mouth within its half of the shortened road.
            let (cut_station, segment, cut) = crossing_cut(record, hit.station - HALF_STRIP)
                .ok_or_else(|| refusal("zebra_near_junction"))?;
            let strip_end = cut_station + CROSSWALK_DEPTH_M as f32;
            if let Some(span) = self.roadbed.span_piece(edge)
                && [EdgeEnd::Start, EdgeEnd::End].into_iter().any(|end| {
                    span.crossing_strip_m(end)
                        .is_some_and(|(start, end)| strip_end > start && cut_station < end)
                })
            {
                return Err(refusal("zebra_near_crossing"));
            }
            if cut_station < 2.0 * lower
                || strip_end > upper
                || hit.length - cut_station
                    < 2.0 * (hit.length - upper).max(CROSSWALK_DEPTH_M as f32)
                || cut_station < HALF_STRIP
            {
                return Err(refusal("zebra_near_junction"));
            }
            if [record.geometry[0], *record.geometry.last().unwrap()]
                .iter()
                .any(|end| cut.distance_to(*end) < 0.2)
            {
                return Err(refusal("zebra_near_junction"));
            }
            Ok(Action::Split { edge, segment, cut })
        })();
        preview.rejection = action.as_ref().err().cloned();
        Some((action, preview))
    }

    fn try_zebra(&mut self, action: Action) -> Result<TriedZebra<L::Building>, RoadRejection> {
        let (edges, nodes) = action.scope(self.graph);
        let target = match action {
            Action::Split { .. } => Some((self.graph.node_count() as u32, self.graph.edge_count())),
            Action::Override {
                node,
                edge,
                enabled: true,
            } => Some((node, edge)),
            _ => None,
        };
        self.try_graph_edit_with(
            &edges,
            &nodes,
            &[],
            |site| site.apply_zebra(action).0,
            |graph, delta| {
                let (node, edge) = target?;
                let end = if graph.edge(edge).start_node == node {
                    EdgeEnd::Start
                } else {
                    EdgeEnd::End
                };
                delta.crossing_kerbs(node, ArmKey { edge, end })
            },
        )
    }

    fn apply_zebra(&mut self, action: Action) -> (HashSet<usize>, Option<PocketApplied>) {
        match action {
            Action::Override {
                node,
                edge,
                enabled,
            } => (self.apply_crosswalk_toggle(node, edge, enabled), None),
            Action::Join {
                node,
                first,
                second,
            } => {
                let edit = join_zebra_pair(self.graph, [first, second], node);
                (HashSet::from([first, second]), Some(edit))
            }
            Action::Split { edge, segment, cut } => {
                let end_clip = self.graph.edge(edge).end_clip;
                let node = self.graph.add_node(cut, NodeType::Junction);
                let split = split_edge_for_crossing(self.graph, edge, segment, node)
                    .expect("the Zebra cut is clear of both ends");
                let second = split.new_edge_id;
                self.graph.edge_mut(second).end_clip = end_clip;
                self.graph.set_crosswalk_override(node, edge, false);
                let mut changed = self.apply_crosswalk_toggle(node, second, true);
                changed.extend([edge, second]);
                let edit = PocketApplied {
                    first: edge,
                    joined: None,
                    split: Some(split),
                    transition: Some(node),
                    pocket_edge: None,
                    first_length_m: self.graph.edge(edge).physical_length,
                };
                (changed, Some(edit))
            }
        }
    }
}

impl Action {
    fn scope(self, graph: &RegionGraph) -> (HashSet<usize>, HashSet<u32>) {
        match self {
            Self::Split { edge, .. } => {
                let source = graph.edge(edge);
                (
                    HashSet::from([edge]),
                    HashSet::from([
                        source.start_node,
                        source.end_node,
                        graph.node_count() as u32,
                    ]),
                )
            }
            Self::Join {
                node,
                first,
                second,
            } => (
                HashSet::from([first, second]),
                HashSet::from([
                    node,
                    graph.edge(first).start_node,
                    graph.edge(second).end_node,
                ]),
            ),
            Self::Override { node, .. } => (
                graph.node_adjacency(node).iter().copied().collect(),
                HashSet::from([node]),
            ),
        }
    }
}

impl SimCore {
    /// Applies the click as one road edit, or returns the same refusal as a hover on this graph.
    pub(crate) fn edit_zebra(&mut self, point: Vector3) -> Result<(), RoadRejection> {
        self.adopt_pending_cell_lots();
        self.allocator
            .prepare_building_site_query_index(self.config.zone_cell_m);
        let mut site = self.road_edit_site();
        let (action, preview) = site
            .zebra_at(point)
            .ok_or_else(|| refusal("zebra_no_road"))?;
        let action = action?;
        let (TriedGraphEdit { checkpoint, take }, kerbs) = site.try_zebra(action)?;
        if !preview.removing && kerbs.is_none() {
            return Err(refusal("crosswalk_arm_not_eligible"));
        }
        let (mut edges, nodes) = action.scope(site.graph);
        let (changed, topology) = site.apply_zebra(action);
        edges.extend(changed);
        let alignments = self.zoning.cells.road_alignment_records();
        let regeneration = topology.as_ref().map(|_| {
            self.zoning
                .mark_cell_lots_for_roads(&self.region_graph, edges.iter().copied())
        });
        let erased = self.take_land_for_edit(take, Some(regeneration.as_deref().unwrap_or(&[])));
        let erased_paint = erased.is_some();
        let dependents = topology.map(|edit| {
            migrate_pocket_dependents(&self.region_graph, &edit, &self.zoning, &mut self.allocator)
        });
        self.push_zebra_undo(checkpoint, alignments, erased, dependents);
        self.transit_network
            .mark_surface_dirty_from_sets(&edges, &nodes);
        crate::thread_pools::install(|| {
            self.rebuild_lanes_locally(&edges, &nodes);
            self.transit_network
                .sync_new_lane_heights(&self.region_graph);
            let cache_current = self.entrance_cache_current();
            let lots = self.prepare_cell_lots_internal();
            self.repair_commit_entrances(&edges, cache_current, &lots, &[]);
            self.reconcile_parking_frontages(&edges.iter().copied().collect());
        });
        let graph = self.schedule_cch_rebuild_of_live_graph();
        self.road_tool_publisher.offer_graph(graph);
        self.transit_network.flow_fields.mark_all_dirty();
        self.mark_local_network_render_dirty();
        if erased_paint {
            self.plan_lots_after_erase();
        }
        Ok(())
    }
}
