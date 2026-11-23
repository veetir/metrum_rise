// SPDX-License-Identifier: GPL-3.0-or-later

//! Placement-only street contacts and deterministic floor candidate selection.

use super::*;
use crate::simulation::buildings::allocator::entrance::building_local_xz_basis;
use crate::simulation::buildings::allocator::site::grading::{
    StreetContactIssue, site_support_intervals, fixed_road_issue, FRONTAGE_BANK_MAX_M,
    BUILDING_SITE_SUPPORT_TIE_IN_SAMPLE_STEP_M, SiteRoads, frontage_road_surface_sample,
    entrance_road_surface_sample, visit_footprint_grading_rays,
    building_site_surface_tie_in,
};
use crate::simulation::network::roadbed::CURB_STEP_HEIGHT_M;
use crate::simulation::parking::ground::{
    DRIVE_GRADE, MAX_GRADE_CHANGE, WINDOW_M, spot_issue, way_issue, way_constraint,
};
use crate::simulation::parking::validation::ProofClass;
use crate::simulation::terrain::graded::{
    GradedSurface, LatticeBlock, QueryScratch, SiteBuffer, SiteSource, SiteTerrace,
};

// No authored ramp region exists yet: do not infer a walkable ramp through paving, walls or
// planting. Every street-facing landing therefore takes the kerb-step rule in this slice.

struct CandidateSite<'a> {
    allocator: &'a BuildingAllocator,
    footprint: &'a [Vector2],
    height: f32,
    terrace: Option<&'a SiteTerrace>,
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
        if let Some(terrace) = self.terrace {
            out.push_terrace(terrace);
        }
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

// Adjacent supported yards must join continuously. Along a common boundary the bank envelope
// is piecewise affine, so the union of both sets of arm crossings proves the entire join;
// sampling every few metres would miss a short bank near a landing. Local indexed neighbours
// only; O(shared edges × (P² + Q² + P log P + Q log Q)), P/Q the two level hull corner counts.
fn yard_join_issue(
    terrace: &SiteTerrace,
    floor: f32,
    neighbor: &super::super::site::BuildingSiteClient,
    breaks: &mut Vec<f64>,
    other_breaks: &mut Vec<f64>,
    lines: &mut Vec<[f64; 3]>,
) -> Option<DemandSpawnPlacementRejection> {
    let other = neighbor.terraces.first();
    let polygon = other.map_or(neighbor.footprint_world.as_slice(), |t| {
        t.footprint_world.as_slice()
    });
    for (i, &a) in terrace.footprint_world.iter().enumerate() {
        let b = terrace.footprint_world[(i + 1) % terrace.footprint_world.len()];
        let edge = b - a;
        let length = edge.length();
        for (j, &c) in polygon.iter().enumerate() {
            let d = polygon[(j + 1) % polygon.len()];
            let across = d - c;
            if edge.cross(across).abs() > 0.000001 * length * across.length()
                || edge.cross(c - a).abs() > 0.0001 * length
            {
                continue;
            }
            let (t0, t1) = (
                (c - a).dot(edge) / edge.length_squared(),
                (d - a).dot(edge) / edge.length_squared(),
            );
            let (low, high) = (t0.min(t1).max(0.0), t0.max(t1).min(1.0));
            if low >= high {
                continue;
            }
            let (p, q) = (a.lerp(b, low), a.lerp(b, high));
            terrace.edge_breaks(p, q, floor, breaks, lines);
            if let Some(other) = other {
                other.edge_breaks(p, q, neighbor.support_height_m, other_breaks, lines);
                breaks.extend_from_slice(other_breaks);
                breaks.sort_unstable_by(f64::total_cmp);
                breaks.dedup();
            }
            for &t in breaks.iter() {
                let p = p.lerp(q, t as f32);
                let h = terrace.height_at(f64::from(p.x), f64::from(p.y), floor) as f32;
                let difference = (h - neighbor.ground_height(p)).abs();
                if difference > 0.0001 {
                    return Some(issue("neighbour yard join", difference, 0.0001, "m"));
                }
            }
        }
    }
    None
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
        let terrace = placement.terraces.first();
        let yard_footprint = footprint;
        let footprint = terrace.map_or(footprint, |t| t.level_footprint_world.as_slice());
        let mut candidates: Vec<f32> = connections.iter().map(|c| c.1).collect();
        if let Some(height) = self.frontage_connection_height(placement, graph, roads) {
            candidates.push(height);
        }
        let (mut low, mut high) = if footprint.is_empty() {
            (f32::NEG_INFINITY, f32::INFINITY)
        } else {
            self.footprint_neighbor_height_interval(
                footprint,
                placement.zone_cell_m,
                placement.replaced_building,
                taken,
            )?
        };
        let mut binding = DemandSpawnPlacementRejection::SiteSupportTieInInvalid;
        let mut constraints = Vec::new();
        let mut driveways = Vec::new();
        let mut yard_drives = Vec::new();
        let mut yard_banks = Vec::new();
        let neighbors = if terrace.is_some() {
            self.lot_candidate_indices_for_bounds(
                super::polygon_bounds(yard_footprint),
                placement.zone_cell_m,
                placement.replaced_building,
            )
        } else {
            Vec::new()
        };
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
                    driveway_profile(carriageway, foot, pos, forward, yard_footprint)
                        .ok_or(DemandSpawnPlacementRejection::DrivewayConnectionMissing)?;
                driveways.push(profile);
                if terrace.is_some() {
                    let direction = (profile[2] - foot).normalized();
                    yard_drives.push((profile[2] - direction * WINDOW_M, length, road_height));
                    continue;
                }
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
            if terrace.is_some() {
                yard_banks.push((point, height));
                continue;
            }
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
                    if terrace.is_some() {
                        yard_banks.push((seam, height));
                        return true;
                    }
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
        let mut base: Vec<f32> = yard_footprint
            .iter()
            .map(|p| terrain.sample_height_world(p.x, p.y) * crate::config::HEIGHT_SCALE)
            .collect();
        base.push(
            terrain.sample_height_world(placement.center_2d.x, placement.center_2d.y)
                * crate::config::HEIGHT_SCALE,
        );
        base.sort_unstable_by(f32::total_cmp);
        candidates.push(base[base.len() / 2]);
        let intervals = if footprint.is_empty() {
            vec![(f32::NEG_INFINITY, f32::INFINITY)]
        } else {
            site_support_intervals(footprint, terrain, graph, roads).ok_or_else(|| {
                fixed_road_issue(footprint, roads).map_or(
                    DemandSpawnPlacementRejection::SiteSupportTieInInvalid,
                    DemandSpawnPlacementRejection::StreetContactInvalid,
                )
            })?
        };
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
            terraces: terrace.cloned().into_iter().collect(),
            footprint_world: yard_footprint.to_vec(),
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
        let dock_paths: Vec<Vec<Vector2>> = geometry
            .docks
            .iter()
            .map(|(_, bay)| bay.path.iter().map(|p| Vector2::new(p[0], p[1])).collect())
            .chain(
                geometry
                    .dock_ways
                    .iter()
                    .flat_map(|(_, way)| [way.route_in, way.route_out])
                    .filter(|route| route.as_slice().len() >= 2)
                    .map(|route| {
                        route
                            .as_slice()
                            .iter()
                            .map(|p| Vector2::new(p[0], p[1]))
                            .collect()
                    }),
            )
            .collect();
        // C is the authored contacts plus midpoint and terrain median, independent of city size.
        // Each candidate checks local grading/contact rays, exact shared-bank joins and D's
        // wheel/half-metre way samples. The owning Part C cost contract includes the level hull
        // corners in those query bounds. Geometric bay proofs are built once above; no tick solve.
        let mut best: Option<(usize, f32, f32)> = None;
        let mut previous_height = None;
        let mut join_breaks = Vec::new();
        let mut other_join_breaks = Vec::new();
        let mut join_lines = Vec::new();
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
            if previous_height == Some(height.to_bits()) {
                continue;
            }
            previous_height = Some(height.to_bits());
            if let Some(terrace) = &terrace {
                let yard_height =
                    |p: Vector2| terrace.height_at(f64::from(p.x), f64::from(p.y), height) as f32;
                let join = neighbors
                    .iter()
                    .filter(|&&i| taken.binary_search_by(|probe| i.cmp(probe)).is_err())
                    .find_map(|&i| {
                        yard_join_issue(
                            terrace,
                            height,
                            &self.building_sites[i],
                            &mut join_breaks,
                            &mut other_join_breaks,
                            &mut join_lines,
                        )
                    });
                if let Some(reason) = join {
                    binding = reason;
                    continue;
                }
                if let Some(&(p, road)) = yard_banks.iter().find(|&&(p, road)| {
                    (yard_height(p) - road).abs() > FRONTAGE_BANK_MAX_M + 0.0001
                }) {
                    binding = issue(
                        "frontage bank",
                        (yard_height(p) - road).abs(),
                        FRONTAGE_BANK_MAX_M,
                        "m",
                    );
                    continue;
                }
                let mut refused = false;
                for &(p, length, road) in &yard_drives {
                    let rise = (yard_height(p) - road).abs();
                    if rise > DRIVE_GRADE * length + 0.0001 {
                        binding = issue(
                            "Driveway grade",
                            rise / length.max(0.001) * 100.0,
                            DRIVE_GRADE * 100.0,
                            "%",
                        );
                        refused = true;
                        break;
                    }
                    if rise > MAX_GRADE_CHANGE * length.max(WINDOW_M) + 0.0001 {
                        binding = issue(
                            "Driveway grade change per wheelbase",
                            rise / length.max(WINDOW_M) * 100.0,
                            MAX_GRADE_CHANGE * 100.0,
                            "percentage points",
                        );
                        refused = true;
                        break;
                    }
                }
                if refused {
                    continue;
                }
                if let Err(reason) = building_site_surface_tie_in(
                    &terrace.footprint_world,
                    yard_height,
                    terrain,
                    graph,
                    roads,
                ) {
                    binding = reason.map_or(
                        DemandSpawnPlacementRejection::SiteSupportTieInInvalid,
                        DemandSpawnPlacementRejection::StreetContactInvalid,
                    );
                    continue;
                }
            }
            let sites = CandidateSite {
                allocator: self,
                footprint: yard_footprint,
                height,
                terrace,
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
            let mut block = LatticeBlock::default();
            let mut drive_issue = None;
            for path in driveways
                .iter()
                .map(|path| path.as_slice())
                .chain(dock_paths.iter().map(Vec::as_slice))
            {
                let mut sample = |p: Vector2| {
                    surface.surface_height_in_block(
                        p.x as f64,
                        p.y as f64,
                        &mut block,
                        &mut scratch,
                    ) as f32
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
                        surface.surface_height_in_block(
                            p.x as f64,
                            p.y as f64,
                            &mut block,
                            &mut scratch,
                        ) as f32
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
            terraces: Vec::new(),
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
        assert_eq!(
            placement.terraces,
            allocator
                .placement_yard_terrace(&placement, &graph, &roads)
                .into_iter()
                .collect::<Vec<_>>(),
            "floor selection must not change the floor-independent yard proposal"
        );
        // Independently recheck the installed boundary heights, level landing, and visible
        // driveway. At 5/10% the yard rises 1/2 m (20 m) or 2/4 m (40 m); the floor only
        // supports the downhill door. Its contact is within .12 m; each pad bank stays 1:2.
        if let Some(terrace) = placement.terraces.first() {
            let h = placement.support_height_m;
            assert!(building_site_support_tie_in_is_valid(
                &terrace.level_footprint_world,
                h,
                &terrain,
                &graph,
                &roads
            ));
            building_site_surface_tie_in(
                &terrace.footprint_world,
                |p| terrace.height_at(p.x as f64, p.y as f64, h) as f32,
                &terrain,
                &graph,
                &roads,
            )
            .unwrap();
            assert!(
                (h - (20.0 + (-half + 1.0) * grade + CURB_STEP_HEIGHT_M)).abs()
                    <= CURB_STEP_HEIGHT_M + 0.0001
            );
            for station in 0..=(half * 4.0) as usize {
                let x = -half + station as f32 * 0.5;
                let yard = terrace.height_at(x as f64, outer as f64, h) as f32;
                assert!(
                    (yard - (20.0 + x * grade + CURB_STEP_HEIGHT_M)).abs()
                        <= FRONTAGE_BANK_MAX_M + 0.0001
                );
            }
            let sites = CandidateSite {
                allocator: &allocator,
                footprint: &terrace.footprint_world,
                height: h,
                terrace: Some(terrace),
                replaced: None,
                taken: &[],
            };
            let surface = GradedSurface {
                terrain: &terrain,
                roadbed: &roads,
                sites: &sites,
                query_index: None,
            };
            let mut scratch = QueryScratch::default();
            let x = -half + 1.0;
            let path = [Vector2::new(x, outer - 5.0), Vector2::new(x, outer + 5.0)];
            let reason = way_constraint(&path, &roads, &mut |p| {
                surface.surface_height_at(p.x as f64, p.y as f64, &mut scratch) as f32
            });
            assert!(reason.is_none(), "visible driveway lost");
        } else {
            assert!(building_site_support_tie_in_is_valid(
                &allocator.placement_required_flat_support_footprint(&placement),
                placement.support_height_m,
                &terrain,
                &graph,
                &roads
            ));
        }
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
    fn placement_neighbor_yard_joins_check_every_affine_bank_interval() {
        let rectangle = |x: f32| {
            vec![
                Vector2::new(x - 10.0, -10.0),
                Vector2::new(x + 10.0, -10.0),
                Vector2::new(x + 10.0, 10.0),
                Vector2::new(x - 10.0, 10.0),
            ]
        };
        let left = SiteTerrace {
            footprint_world: rectangle(0.0),
            plane: [0.05, 0.0, 0.0],
            level_footprint_world: Vec::new(),
        };
        let right = SiteTerrace {
            footprint_world: rectangle(20.0),
            plane: left.plane,
            level_footprint_world: Vec::new(),
        };
        let mut neighbor = super::super::super::site::BuildingSiteClient {
            footprint_world: right.footprint_world.clone(),
            lot_footprint_world: right.footprint_world.clone().try_into().unwrap(),
            support_height_m: 1.0,
            terraces: vec![right],
            surfaces: Vec::new(),
            structure_world: Vec::new(),
            vehicle_structure_world: None,
            planting_world: Vec::new(),
        };
        let (mut a, mut b, mut lines) = (Vec::new(), Vec::new(), Vec::new());
        // Both planes meet at 0.5 m on x=10, even though their stored floors differ.
        assert!(yard_join_issue(&left, 0.0, &neighbor, &mut a, &mut b, &mut lines).is_none());
        neighbor.terraces[0].plane[2] = 0.2;
        let DemandSpawnPlacementRejection::StreetContactInvalid(reason) =
            yard_join_issue(&left, 0.0, &neighbor, &mut a, &mut b, &mut lines).unwrap()
        else {
            panic!()
        };
        assert_eq!(
            reason.message(),
            "neighbour yard join requires 0.20 m; at most 0.00 m"
        );

        // A 1 cm bank dip over z=.2..0.4 lies between 2 m boundary stations. Its exact arm
        // crossings must still reject the discontinuity against the neighbour's .01 m plane.
        let mut narrow = left;
        narrow.plane = [0.0, 0.0, 0.01];
        narrow.level_footprint_world = vec![
            Vector2::new(9.0, 0.2),
            Vector2::new(10.0, 0.2),
            Vector2::new(10.0, 0.4),
            Vector2::new(9.0, 0.4),
        ];
        neighbor.terraces[0].plane = narrow.plane;
        assert!(yard_join_issue(&narrow, 0.0, &neighbor, &mut a, &mut b, &mut lines).is_some());
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
        // A 20/40 m yard rises 1/2 m at 5%, 2/4 m at 10%, following the road.
        // A single level door pad at x=-9/-19 does not flatten that yard. At 20% the
        // 10% plane leaves 1/2 m residual at the frontage ends and must still prove C1.
        for (width, grade, accepted) in [
            (2, 0.0, true),
            (4, 0.0, true),
            (2, 0.05, true),
            (4, 0.05, true),
            (2, 0.10, true),
            (4, 0.10, true),
            (2, 0.20, false),
            (4, 0.20, false),
        ] {
            let result = lot(width, grade, None);
            if grade == 0.20 {
                let DemandSpawnPlacementRejection::StreetContactInvalid(reason) =
                    result.as_ref().unwrap_err()
                else {
                    panic!("{result:?}");
                };
                // Width 2: yard 19.22 - carriageway 18.20 = 1.02 m over the 1.51 m
                // apron, i.e. 67.55%. Width 4: H=16.44, front bank 16.44+2.75/2
                // against footway 16.12 = 1.695 m, rounded to 1.70 m.
                assert_eq!(
                    reason.message(),
                    if width == 2 {
                        "Driveway grade requires 67.55 %; at most 10.00 %"
                    } else {
                        "frontage bank requires 1.70 m; at most 1.00 m"
                    }
                );
            }
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
        // At the last candidate the downhill landing permits H=20-19*.21+.24=16.25.
        // Its level hull ends 2.75 m behind the front edge, so the 1:2 bank reaches
        // at most 16.25+2.75/2=17.625 at x=-20. The footway there is
        // 20-20*.21+.12=15.92: a 1.705 m bank rounds to 1.71 m, over the 1 m cap.

        assert_eq!(
            issue.message(),
            "frontage bank requires 1.71 m; at most 1.00 m"
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
