// SPDX-License-Identifier: GPL-3.0-or-later

//! Placement-only street contacts and deterministic floor candidate selection.

use super::*;
use crate::simulation::buildings::allocator::entrance::building_local_xz_basis;
use crate::simulation::buildings::allocator::site::grading::{
    StreetContactIssue, site_support_intervals, fixed_road_issue, FRONTAGE_BANK_MAX_M,
    BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M, SiteRoads, frontage_road_surface_sample,
    entrance_road_surface_sample, visit_footprint_grading_rays,
};
use crate::simulation::network::roadbed::CURB_STEP_HEIGHT_M;
use crate::simulation::parking::ground::{
    DRIVE_GRADE, MAX_GRADE_CHANGE, WINDOW_M, spot_issue, way_issue, way_constraint,
};
use crate::simulation::parking::validation::ProofClass;
use crate::simulation::terrain::graded::{GradedSurface, QueryScratch, SiteBuffer, SiteSource};

// No authored ramp region exists yet: do not infer a walkable ramp through paving, walls or
// planting. Every street-facing landing therefore takes the kerb-step rule in this slice.

struct CandidateSite<'a> {
    allocator: &'a BuildingAllocator,
    footprint: &'a [Vector2],
    height: f32,
    replaced: Option<usize>,
    taken: &'a [usize],
}

impl SiteSource for CandidateSite<'_> {
    fn ready(&self) -> bool {
        self.allocator.ready()
    }
    fn sites_in_bounds(&self, rect: [f64; 4], out: &mut SiteBuffer) {
        self.allocator
            .push_graded_sites_except(rect, self.replaced, self.taken, out);
        out.push_site(self.footprint, self.height);
    }
}

fn issue(
    constraint: &'static str,
    measured: f32,
    limit: f32,
    unit: &'static str,
) -> DemandSpawnPlacementRejection {
    DemandSpawnPlacementRejection::StreetContactInvalid(StreetContactIssue {
        constraint,
        measured: (measured * 100.0).round() as u32,
        limit: (limit * 100.0).round() as u32,
        unit,
    })
}

fn height_interval_issue(
    intervals: &[(f32, f32)],
    constraints: &[(f32, f32, DemandSpawnPlacementRejection)],
) -> DemandSpawnPlacementRejection {
    for &(height, budget, reason) in constraints {
        let required = intervals
            .iter()
            .map(|&(a, b)| (height - height.clamp(a, b)).abs())
            .fold(f32::INFINITY, f32::min);
        if required > budget + 0.0001
            && let DemandSpawnPlacementRejection::StreetContactInvalid(mut issue) = reason
        {
            issue.measured = (required / budget.max(0.0001) * issue.limit as f32).round() as u32;
            return DemandSpawnPlacementRejection::StreetContactInvalid(issue);
        }
    }
    // Individually feasible contacts can jointly occupy a gap between soft-terrain intervals.
    // In that case no single contact limit is violated: name the grading envelope, not a
    // fictitious bank/entrance excess measured at the old preferred height.
    DemandSpawnPlacementRejection::SiteSupportTieInInvalid
}

fn pad_contact(point: Vector2, inward: Vector2, footprint: &[Vector2]) -> Option<(Vector2, f32)> {
    if super::super::site::site_footprint_contains(footprint, point) {
        return Some((point, 0.0));
    }
    let mut distance = f32::INFINITY;
    for (i, &a) in footprint.iter().enumerate() {
        let edge = footprint[(i + 1) % footprint.len()] - a;
        let cross = inward.cross(edge);
        if cross.abs() <= 1e-8 {
            continue;
        }
        let t = (a - point).cross(edge) / cross;
        let u = (a - point).cross(inward) / cross;
        if t >= 0.0 && (0.0..=1.0).contains(&u) {
            distance = distance.min(t);
        }
    }
    distance
        .is_finite()
        .then_some((point + inward * distance, distance))
}

fn driveway_profile(
    carriageway: Vector2,
    footway: Vector2,
    anchor: Vector2,
    forward: Vector2,
    footprint: &[Vector2],
) -> Option<([Vector2; 3], f32)> {
    // D drives from the footway connection to one car lead inside the authored gate.
    // Keep that actual direction, including a gate angled along a sloping street.
    let approach = anchor + forward * ProofClass::Car.proof().lead;
    let inward = (approach - footway).normalized();
    let (contact, run) = pad_contact(footway, inward, footprint)?;
    let crossing = (footway - carriageway).normalized();
    Some((
        [
            carriageway - crossing * WINDOW_M,
            footway,
            contact + inward * WINDOW_M,
        ],
        carriageway.distance_to(footway) + run,
    ))
}

impl BuildingAllocator {
    pub(super) fn solve_street_contacts(
        &self,
        placement: &ResolvedPlacement,
        footprint: &[Vector2],
        connections: &[(Vector2, f32)],
        taken: &[usize],
        graph: &RegionGraph,
        environment: BuildingSiteEnvironment<'_>,
    ) -> Result<f32, DemandSpawnPlacementRejection> {
        let roads = environment.road_surface;
        let terrain = environment.terrain;
        let entry = self
            .registry
            .get(&placement.asset_id)
            .ok_or(DemandSpawnPlacementRejection::AssetUnavailable)?;
        let edge = graph.edge(placement.edge_idx);
        let (bx, bz) = building_local_xz_basis(
            placement.facing_dir,
            entry.manifest.building_frontage_forward(),
        );
        let world = |p: [f32; 3]| placement.center_2d + bx * p[0] + bz * p[2];
        let mut candidates: Vec<f32> = connections.iter().map(|c| c.1).collect();
        if let Some(height) = self.frontage_connection_height(placement, graph, roads) {
            candidates.push(height);
        }
        let (mut low, mut high) = self.footprint_neighbor_height_interval(
            footprint,
            placement.zone_cell_m,
            placement.replaced_building,
            taken,
        )?;
        let mut binding = DemandSpawnPlacementRejection::SiteSupportTieInInvalid;
        let mut constraints = Vec::new();
        let mut driveways = Vec::new();
        let mut constrain =
            |height: f32, budget: f32, reason| -> Result<(), DemandSpawnPlacementRejection> {
                let a = low.max(height - budget);
                let b = high.min(height + budget);
                if a > b + 0.0001 {
                    let required = (height - height.clamp(low, high)).abs();
                    if let DemandSpawnPlacementRejection::StreetContactInvalid(mut issue) = reason {
                        issue.measured =
                            (required / budget.max(0.0001) * issue.limit as f32).round() as u32;
                        return Err(DemandSpawnPlacementRejection::StreetContactInvalid(issue));
                    }
                    return Err(reason);
                }
                constraints.push((height, budget, reason));
                low = a;
                high = b.max(a);
                binding = reason;
                Ok(())
            };
        for anchor in &entry.manifest.anchors {
            let driveway = anchor.anchor_type == AnchorType::Driveway;
            let entrance = anchor.anchor_type == AnchorType::Entrance;
            let forward = (bx * anchor.forward[0] + bz * anchor.forward[2]).normalized();
            if !driveway && !entrance {
                continue;
            }
            let pos = world(anchor.position);
            let t = Self::project_point_to_polyline_s(&edge.physical_geometry, pos)
                / edge.physical_length;
            let (foot, foot_height) = if entrance && forward.dot(placement.facing_dir) <= 0.5 {
                let Some(contact) =
                    entrance_road_surface_sample(SiteRoads::new(graph, roads), pos, forward)
                else {
                    continue;
                };
                contact
            } else {
                let foot = Self::claimed_road_side_connection_pos(
                    graph,
                    placement.edge_idx,
                    placement.side,
                    t,
                )
                .ok_or(DemandSpawnPlacementRejection::FrontageRoadSurfaceMissing)?;
                let height = roads
                    .exposed_height_at(foot.x, foot.y)
                    .ok_or(DemandSpawnPlacementRejection::FrontageRoadSurfaceMissing)?;
                (foot, height)
            };
            candidates.push(foot_height);
            if entrance {
                // The landing is on the building floor; its anchor's authored Y is the threshold.
                let landing = foot_height - anchor.position[1];
                constrain(
                    landing,
                    CURB_STEP_HEIGHT_M,
                    issue(
                        "entrance landing step",
                        (placement.support_height_m - landing).abs(),
                        CURB_STEP_HEIGHT_M,
                        "m",
                    ),
                )?;
            }
            if driveway {
                let center = Self::sample_pos_on_edge(graph, placement.edge_idx, t);
                let tangent = Self::sample_tangent_on_edge(graph, placement.edge_idx, t);
                let normal = Vector2::new(tangent.y, -tangent.x) * placement.side as f32;
                // Measure from the carriageway, not the outer footway sample used for candidates.
                let carriageway =
                    center + normal * (edge.frontage_half_width_m(placement.side) - 0.01).max(0.0);
                let road_height = roads
                    .carriageway_height_at(carriageway.x, carriageway.y)
                    .unwrap_or(foot_height - CURB_STEP_HEIGHT_M);
                let (profile, length) =
                    driveway_profile(carriageway, foot, pos, forward, footprint)
                        .ok_or(DemandSpawnPlacementRejection::DrivewayConnectionMissing)?;
                driveways.push(profile);
                let rise = (placement.support_height_m - road_height).abs();
                constrain(
                    road_height,
                    DRIVE_GRADE * length,
                    issue(
                        "Driveway grade",
                        rise / length.max(0.001) * 100.0,
                        DRIVE_GRADE * 100.0,
                        "%",
                    ),
                )?;
                // A linear apron between horizontal contacts: the largest change in wheelbase
                // mean grade is rise / max(length, wheelbase). Longer transitions need terraces.
                let transition = MAX_GRADE_CHANGE * length.max(WINDOW_M);
                constrain(
                    road_height,
                    transition,
                    issue(
                        "Driveway grade change per wheelbase",
                        rise / length.max(WINDOW_M) * 100.0,
                        MAX_GRADE_CHANGE * 100.0,
                        "percentage points",
                    ),
                )?;
            }
        }
        // Bound the whole frontage even when the pad is set back from the lot line.
        // The two foremost rectangle corners define that line; sample at the grading rays'
        // spacing, so curved road profiles between the ends cannot hide a higher bank.
        let mut corners = self.placement_site_corners(placement);
        corners.sort_unstable_by(|a, b| {
            b.dot(placement.facing_dir)
                .total_cmp(&a.dot(placement.facing_dir))
        });
        let count = (corners[0].distance_to(corners[1])
            / BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M)
            .ceil()
            .max(1.0) as usize;
        for station in 0..=count {
            let point = corners[0].lerp(corners[1], station as f32 / count as f32);
            let t = Self::project_point_to_polyline_s(&edge.physical_geometry, point)
                / edge.physical_length;
            let connection = Self::claimed_road_side_connection_pos(
                graph,
                placement.edge_idx,
                placement.side,
                t,
            )
            .ok_or(DemandSpawnPlacementRejection::FrontageRoadSurfaceMissing)?;
            let height = roads
                .exposed_height_at(connection.x, connection.y)
                .ok_or(DemandSpawnPlacementRejection::FrontageRoadSurfaceMissing)?;
            constrain(
                height,
                FRONTAGE_BANK_MAX_M,
                issue(
                    "frontage bank",
                    (placement.support_height_m - height).abs(),
                    FRONTAGE_BANK_MAX_M,
                    "m",
                ),
            )?;
        }
        // A corner/double-frontage lot also has banks on its other road-facing boundaries.
        // Reuse the indexed nearest-road query; a road behind the edge's outward half-plane
        // is not that edge's frontage. This adds O(boundary rays × local road query cost).
        let lot = self.placement_site_corners(placement);
        let mut bank_result = Ok(());
        visit_footprint_grading_rays(
            &lot,
            BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M,
            |seam, outward| {
                if let Some((_, height)) =
                    frontage_road_surface_sample(SiteRoads::new(graph, roads), seam, outward)
                {
                    bank_result = constrain(
                        height,
                        FRONTAGE_BANK_MAX_M,
                        issue(
                            "frontage bank",
                            (placement.support_height_m - height).abs(),
                            FRONTAGE_BANK_MAX_M,
                            "m",
                        ),
                    );
                    return bank_result.is_ok();
                }
                true
            },
        );
        bank_result?;
        let mut base: Vec<f32> = footprint
            .iter()
            .map(|p| terrain.sample_height_world(p.x, p.y) * crate::config::HEIGHT_SCALE)
            .collect();
        base.push(
            terrain.sample_height_world(placement.center_2d.x, placement.center_2d.y)
                * crate::config::HEIGHT_SCALE,
        );
        base.sort_unstable_by(f32::total_cmp);
        candidates.push(base[base.len() / 2]);
        let intervals =
            site_support_intervals(footprint, terrain, graph, roads).ok_or_else(|| {
                fixed_road_issue(footprint, roads).map_or(
                    DemandSpawnPlacementRejection::SiteSupportTieInInvalid,
                    DemandSpawnPlacementRejection::StreetContactInvalid,
                )
            })?;
        let feasible: Vec<_> = intervals
            .iter()
            .copied()
            .filter_map(|(a, b)| {
                let a = a.max(low);
                let b = b.min(high);
                (a <= b + 0.0001).then_some((a, b.max(a)))
            })
            .collect();
        if feasible.is_empty() {
            return Err(height_interval_issue(&intervals, &constraints));
        }
        if placement.replaced_building.is_some() {
            // A replacement validates all contacts on the stored floor, never selects a new one.
            if !feasible.iter().any(|&(a, b)| {
                placement.support_height_m >= a - 0.0001 && placement.support_height_m <= b + 0.0001
            }) {
                return Err(height_interval_issue(
                    &[(placement.support_height_m, placement.support_height_m)],
                    &constraints,
                ));
            }
            candidates.clear();
            candidates.push(placement.support_height_m);
        }
        let (structure_world, vehicle_structure_world) =
            super::super::site::site_structure_footprints(&entry.manifest.mesh_parts, world);
        let site = super::super::site::BuildingSiteClient {
            footprint_world: footprint.to_vec(),
            lot_footprint_world: self.placement_site_corners(placement),
            support_height_m: placement.support_height_m,
            surfaces: entry
                .manifest
                .site_surfaces
                .iter()
                .map(|s| super::super::site::BuildingSiteSurfaceClient {
                    name: s.name.clone(),
                    material: s.material,
                    vertices_world: s
                        .vertices
                        .iter()
                        .map(|p| world([p[0], 0.0, p[1]]))
                        .collect(),
                })
                .collect(),
            structure_world,
            vehicle_structure_world,
            planting_world: Vec::new(),
        };
        let geometry = crate::simulation::parking::lots::lot_proof_at(
            self,
            crate::simulation::parking::lots::LotPose {
                asset_id: &placement.asset_id,
                center: placement.center_2d,
                facing_dir: placement.facing_dir,
                edge_idx: placement.edge_idx,
                side: placement.side,
                width_cells: placement.width_cells as u16,
                depth_cells: placement.depth_cells as u16,
            },
            &site,
            graph,
            roads,
            false,
            |class, foot| {
                crate::simulation::parking::geometry::structure_collision_for_site(
                    self,
                    class,
                    foot,
                    placement.replaced_building,
                    taken,
                    Some(&site),
                )
            },
            |_| 0.0,
        );
        candidates.sort_unstable_by(f32::total_cmp);
        candidates.dedup_by(|a, b| a.to_bits() == b.to_bits());
        // C is the authored contacts plus midpoint and terrain median, independent of city size.
        // Additional selection is O(C × (R + P + S)): R grading intervals/rays, P wheel samples,
        // S half-metre way samples, plus existing local indexed surface gathers. Geometric bay
        // proofs are built once above. No continuous search, tick work or background-city scan.
        let mut best: Option<(usize, f32, f32)> = None;
        for candidate in candidates {
            let height = feasible
                .iter()
                .map(|&(a, b)| candidate.clamp(a, b))
                .min_by(|a, b| {
                    (a - candidate)
                        .abs()
                        .total_cmp(&(b - candidate).abs())
                        .then(a.total_cmp(b))
                })
                .unwrap();
            let sites = CandidateSite {
                allocator: self,
                footprint,
                height,
                replaced: placement.replaced_building,
                taken,
            };
            let surface = GradedSurface {
                terrain,
                roadbed: roads,
                sites: &sites,
                query_index: None,
            };
            let mut scratch = QueryScratch::default();
            let reach = geometry
                .bays
                .iter()
                .flat_map(|(_, b)| b.path)
                .chain(driveways.iter().flatten().map(|p| [p.x, p.y]))
                .map(|p| placement.center_2d.distance_to(Vector2::new(p[0], p[1])))
                .fold(0.0_f32, f32::max)
                + 4.0;
            if !geometry.bays.is_empty() || !driveways.is_empty() {
                surface.gather_near(
                    placement.center_2d.x as f64,
                    placement.center_2d.y as f64,
                    reach as f64,
                    &mut scratch,
                );
            }
            let mut drive_issue = None;
            for path in &driveways {
                let mut sample = |p: Vector2| {
                    surface.surface_height_near(p.x as f64, p.y as f64, &mut scratch) as f32
                };
                if let Some(measured) = way_constraint(path, roads, &mut sample) {
                    drive_issue = Some(issue(
                        measured.name,
                        measured.measured,
                        measured.limit,
                        measured.unit,
                    ));
                    break;
                }
            }
            if let Some(reason) = drive_issue {
                binding = reason;
                continue;
            }
            let refused = geometry
                .bays
                .iter()
                .filter(|(_, bay)| {
                    let pos = Vector2::new(bay.position[0], bay.position[2]);
                    let anchor = &entry.manifest.anchors[bay.identity[1] as usize];
                    let forward = (bx * anchor.forward[0] + bz * anchor.forward[2]).normalized();
                    let path = bay.path.map(|p| Vector2::new(p[0], p[1]));
                    let mut sample = |p: Vector2| {
                        surface.surface_height_near(p.x as f64, p.y as f64, &mut scratch) as f32
                    };
                    spot_issue(ProofClass::Car, pos, forward, &mut sample)
                        .or_else(|| way_issue(&path, roads, &mut sample))
                        .is_some()
                })
                .count();
            let cut_fill = base.iter().map(|b| (height - b).abs()).sum::<f32>();
            let score = (refused, cut_fill, height);
            if best.is_none_or(|old| {
                score
                    .0
                    .cmp(&old.0)
                    .then(score.1.total_cmp(&old.1))
                    .then(score.2.total_cmp(&old.2))
                    .is_lt()
            }) {
                best = Some(score);
            }
        }
        best.map(|s| s.2).ok_or(binding)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assets::AssetManifest;
    use crate::simulation::network::roadbed::fixtures::road_edge;
    use crate::simulation::network::types::NodeType;

    fn street(graph: &mut RegionGraph, grade: f32, z: f32, height: f32) {
        let points = vec![
            Vector3::new(-64.0, height - 64.0 * grade, z),
            Vector3::new(64.0, height + 64.0 * grade, z),
        ];
        let a = graph.add_node(points[0], NodeType::Junction);
        let b = graph.add_node(points[1], NodeType::Junction);
        graph.add_edge(road_edge(a, b, points, 6.0));
    }

    fn lot(
        width: usize,
        grade: f32,
        rear: Option<f32>,
    ) -> Result<f32, DemandSpawnPlacementRejection> {
        lot_with_rear_door(width, grade, rear, false)
    }

    fn lot_with_rear_door(
        width: usize,
        grade: f32,
        rear: Option<f32>,
        rear_door: bool,
    ) -> Result<f32, DemandSpawnPlacementRejection> {
        let mut graph = RegionGraph::new();
        street(&mut graph, grade, 0.0, 20.0);
        let outer = graph.edge(0).frontage_half_width_m(-1) + side_band_width_m(graph.edge(0), -1);
        if let Some(height) = rear {
            street(&mut graph, 0.0, outer * 2.0 + 30.0, height);
        }
        let mut roads = RoadbedSystem::new(64.0);
        roads.compile_dirty(&graph);
        let mut allocator = BuildingAllocator::new();
        let mut manifest: AssetManifest = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../benchmarks/fixtures/kuopio-terrain/building-site.toml"
        )))
        .unwrap();
        let half = width as f32 * 5.0;
        manifest.building.as_mut().unwrap().lot_width_cells = width as u16;
        manifest.building.as_mut().unwrap().lot_depth_cells = 3;
        manifest.anchors[0].position = [-half + 1.0, 0.0, -10.0];
        manifest.anchors[1].position = [-half + 1.0, 0.0, -15.0];
        manifest.site_surfaces[0].vertices =
            vec![[-half, -15.0], [half, -15.0], [half, 15.0], [-half, 15.0]];
        if rear_door {
            let mut door = manifest.anchors[0].clone();
            door.name = "rear".into();
            door.position = [0.0, 0.0, 15.0];
            door.forward = [0.0, 0.0, 1.0];
            manifest.anchors.push(door);
        }
        let asset_id = format!("test:{}", manifest.asset_id);
        allocator.registry.register("test", manifest, String::new());
        let mut placement = ResolvedPlacement {
            replaced_building: None,
            asset_id,
            build_generation: 0,
            zone_profile_runtime_id: 0,
            zone_type: ZoneType::Residential,
            initial_level: 1,
            parcel_id: 0,
            edge_idx: 0,
            side: -1,
            cell_x: 0,
            width_cells: width,
            depth_cells: 3,
            zone_cell_m: 10.0,
            center_2d: Vector2::new(0.0, outer + 15.0),
            support_height_m: 0.0,
            facing_dir: Vector2::new(0.0, -1.0),
            frontage_t: 0.5,
            side_offset_m: outer,
            front_gap_m: [0.0; 3],
        };
        let terrain = TerrainSystem::with_chunking(257, 257, 1.0, 64, 1.0);
        if rear.is_some_and(|h| (h - 20.0).abs() > 1.0) {
            // The opposed d=1 road samples allow 0.5+0.5=1 m combined rise. Check the
            // shared fixed-road solver independently of the stricter contact/bank refusal.
            let footprint = allocator.placement_required_flat_support_footprint(&placement);
            assert!(site_support_intervals(&footprint, &terrain, &graph, &roads).is_none());
            assert!(!building_site_support_tie_in_is_valid(
                &footprint, 20.12, &terrain, &graph, &roads
            ));
        }
        allocator.prepare_site_support(&mut placement, &[], &graph, &roads, &terrain)?;
        // The production validator runs on the selected floor, not merely the candidate prior.
        assert!(building_site_support_tie_in_is_valid(
            &allocator.placement_required_flat_support_footprint(&placement),
            placement.support_height_m,
            &terrain,
            &graph,
            &roads
        ));
        Ok(placement.support_height_m)
    }

    #[test]
    fn placement_disjoint_terrain_intervals_do_not_invent_a_contact_excess() {
        // The two doors each reach one terrain interval, but their intersection [-.02,.02]
        // occupies the gap between them. Neither door individually violates its .12 m limit.
        let limits = [
            (-0.1, 0.12, issue("entrance landing step", 0.1, 0.12, "m")),
            (0.1, 0.12, issue("entrance landing step", 0.1, 0.12, "m")),
        ];
        assert_eq!(
            height_interval_issue(&[(-1.1, -0.1), (0.1, 1.1)], &limits),
            DemandSpawnPlacementRejection::SiteSupportTieInInvalid
        );
        let reason = height_interval_issue(
            &[(1.0, 2.0)],
            &[(0.0, 0.12, issue("entrance landing step", 0.0, 0.12, "m"))],
        );
        let DemandSpawnPlacementRejection::StreetContactInvalid(issue) = reason else {
            panic!("{reason:?}");
        };
        assert_eq!(
            issue.message(),
            "entrance landing step requires 1.00 m; at most 0.12 m"
        );
    }

    #[test]
    fn placement_driveway_profile_uses_the_authored_angle_and_carriageway_run() {
        let pad = [
            Vector2::new(-10.0, 2.0),
            Vector2::new(10.0, 2.0),
            Vector2::new(10.0, 10.0),
            Vector2::new(-10.0, 10.0),
        ];
        let foot = Vector2::new(0.0, 1.5);
        let anchor = Vector2::new(0.0, 2.0);
        let forward = Vector2::new(1.0, 1.0).normalized();
        let (path, run) = driveway_profile(Vector2::ZERO, foot, anchor, forward, &pad).unwrap();
        let direction = (anchor + forward * ProofClass::Car.proof().lead - foot).normalized();
        // 1.5 m across the footway, then 0.5 m inward to the pad measured on the angled
        // approach: 1.5 + 0.5 / cos(angle), rather than the 2 m perpendicular shortcut.
        assert!((run - (1.5 + 0.5 / direction.y)).abs() < 0.00001);
        assert!(run > 2.0);
        assert!(path[2].x > 0.0);
        assert!((path[2] - foot).normalized().distance_to(direction) < 0.00001);
        assert_eq!(path[0], Vector2::new(0.0, -WINDOW_M));
    }

    #[test]
    fn placement_downhill_driveway_lots_on_five_ten_twenty_percent_streets() {
        // A 20/40 m frontage rises 1/2 m at 5%, 2/4 m at 10%, and 4/8 m at 20%.
        // The driveway sits 1 m from the downhill end. Its ~2 m footway apron permits
        // ~0.14 m rise after the wheelbase transition bound; the footway is already 0.12 m
        // above the carriageway. Even 20 m at 5% cannot connect: its front-corner rays
        // at d=1 see road points 22 m apart (side rays extend 1 m each), rising 1.10 m.
        // Their combined 1:2 allowance is 1.00 m, i.e. the required grade is 55%.
        // Wider/steeper cases fail that rule or the <=1 m bank with the downhill access.
        // These whole-frontage pads need terraces even though the street itself is only 5%.
        for (width, grade, accepted) in [
            (2, 0.0, true),
            (4, 0.0, true),
            (2, 0.05, false),
            (4, 0.05, false),
            (2, 0.10, false),
            (4, 0.10, false),
            (2, 0.20, false),
            (4, 0.20, false),
        ] {
            let result = lot(width, grade, None);
            assert_eq!(
                result.is_ok(),
                accepted,
                "width={width}, grade={grade}: {result:?}"
            );
        }
    }

    #[test]
    fn placement_store_on_twenty_one_percent_street_refuses_the_bank() {
        let reason = lot(4, 0.21, None).unwrap_err();
        let DemandSpawnPlacementRejection::StreetContactInvalid(issue) = reason else {
            panic!("{reason:?}");
        };
        // The first binding 2 m frontage station is x=-14: its footway is
        // 20 - 14*0.21 + 0.12 = 17.18 m. The downhill driveway allows at most
        // 20 - 19*0.21 + 0.05*2.8 = 16.15 m, so that bank already needs 1.03 m.
        assert_eq!(
            issue.message(),
            "frontage bank requires 1.03 m; at most 1.00 m"
        );
    }

    #[test]
    fn placement_fixed_rear_road_cannot_be_waived_by_distant_soft_ground() {
        assert!(lot(2, 0.0, Some(20.0)).is_ok());
        let reason = lot(2, 0.0, Some(24.4)).unwrap_err();
        let DemandSpawnPlacementRejection::StreetContactInvalid(issue) = reason else {
            panic!("{reason:?}");
        };
        assert_eq!(
            issue.message(),
            "frontage bank requires 4.38 m; at most 1.00 m"
        );
    }
    #[test]
    fn placement_secondary_street_entrance_meets_its_own_footway() {
        assert!(lot_with_rear_door(2, 0.0, Some(20.0), true).is_ok());
        let reason = lot_with_rear_door(2, 0.0, Some(24.4), true).unwrap_err();
        let DemandSpawnPlacementRejection::StreetContactInvalid(issue) = reason else {
            panic!("{reason:?}");
        };
        assert_eq!(
            issue.message(),
            "entrance landing step requires 4.38 m; at most 0.12 m"
        );
    }
}
