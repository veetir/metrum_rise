// SPDX-License-Identifier: GPL-3.0-or-later

//! Edit-boundary drivability of the visible surface under bays and their proven ways.

use super::geometry::rectangle;
use super::validation::ProofClass;
use crate::simulation::network::roadbed::{CURB_STEP_HEIGHT_M, RoadbedSystem};
use godot::prelude::{Vector2, Vector3};

// Ordinary parking: Helsinki 1:20 / federal parking criteria. Driveways: Helsinki 1:10.
/// Maximum wheel-envelope grade of an ordinary parked car or loading bay.
pub(crate) const PARKED_GRADE: f32 = 0.05;
/// Maximum longitudinal driveway/aisle grade shared by placement and visible-ground proof.
pub(crate) const DRIVE_GRADE: f32 = 0.10;
/// Maximum algebraic grade change over a car wheelbase (TxDOT driveway transitions).
pub(crate) const MAX_GRADE_CHANGE: f32 = 0.05;
const SAMPLE_M: f32 = 0.5;
// The grade a car feels is the one between its axles: measured over its wheelbase, a dip or
// bump shorter than that is the step rule's, as a kerb is.
/// Car wheelbase used for rolling drive grades and their transitions.
pub(crate) const WINDOW_M: f32 = CAR_WHEELS_M[1];
// The stations back to two wheelbases before the current one.
const HISTORY: usize = (2.0 * WINDOW_M / SAMPLE_M) as usize + 2;
const MAX_STEP_M: f32 = CURB_STEP_HEIGHT_M;
const HEIGHT_EPS_M: f32 = 0.0001;

// Render profiles (vehicle_ground.rs) are populated by Godot mesh loading, not available as
// canonical simulation data at load/proof time. Use fixed wheel contacts of the class's proof
// body instead (track x wheelbase): 1.6 x 2.8 m inside the 1.8 x 4.5 m car, 2.1 x 5.0 m inside
// the 2.6 x 8 m truck. The body's corners overhang the wheels and touch no ground.
const CAR_WHEELS_M: [f32; 2] = [1.6, 2.8];
const TRUCK_WHEELS_M: [f32; 2] = [2.1, 5.0];

/// The four wheel contacts of the admitted class, independent of runtime mesh loading.
pub(super) fn wheel_area(class: ProofClass, pos: Vector2, forward: Vector2) -> [Vector2; 4] {
    let [track, wheelbase] = if class == ProofClass::Car {
        CAR_WHEELS_M
    } else {
        TRUCK_WHEELS_M
    };
    rectangle(pos, forward, track, wheelbase)
}

/// A bay's wheel-envelope grade on the visible ground, with its measured refusal. Both
/// wheel triangles also check the full gradient, so rotating a bay cannot hide a steep plane.
pub(crate) fn spot_issue(
    class: ProofClass,
    pos: Vector2,
    forward: Vector2,
    height: &mut impl FnMut(Vector2) -> f32,
) -> Option<String> {
    let wheels = wheel_area(class, pos, forward);
    let heights = wheels.map(&mut *height);
    if heights[0].is_finite() && heights[1..].iter().all(|&h| h == heights[0]) {
        return None;
    }
    let mut grade = 0.0_f32;
    for i in 0..4 {
        for j in 0..i {
            let rise = (heights[i] - heights[j]).abs();
            let run = wheels[i].distance_to(wheels[j]);
            if !rise.is_finite() {
                return Some(
                    "not drivable: the ground under this spot has no finite height".into(),
                );
            }
            if rise > PARKED_GRADE * run + HEIGHT_EPS_M {
                grade = grade.max(rise / run);
            }
        }
    }
    for [a, b, c] in [[0, 1, 2], [0, 2, 3]] {
        let (u, v) = (wheels[b] - wheels[a], wheels[c] - wheels[a]);
        let (hu, hv) = (heights[b] - heights[a], heights[c] - heights[a]);
        let determinant = u.cross(v);
        let gradient = Vector2::new(
            (hu * v.y - hv * u.y) / determinant,
            (u.x * hv - v.x * hu) / determinant,
        )
        .length();
        let run = u.length().min(v.length());
        if gradient * run > PARKED_GRADE * run + HEIGHT_EPS_M {
            grade = grade.max(gradient);
        }
    }
    (grade > 0.0).then(|| format!(
        "not drivable: the ground under this spot slopes {:.1} %, a parked {} needs at most {:.0} %",
        grade * 100.0,
        if class == ProofClass::Car { "car" } else { "truck" },
        PARKED_GRADE * 100.0,
    ))
}

#[derive(Clone, Copy)]
struct Sample {
    height: f32,
    road: Option<Vector3>,
    carriageway: bool,
}

/// Checks the actual proven polyline, including authored dock ways, at half-metre stations.
pub(crate) fn way_issue(
    path: &[Vector2],
    roads: &RoadbedSystem,
    height: &mut impl FnMut(Vector2) -> f32,
) -> Option<String> {
    check_way(path, |p| visible_sample(p, roads, height))
}

fn visible_sample(
    p: Vector2,
    roads: &RoadbedSystem,
    height: &mut impl FnMut(Vector2) -> f32,
) -> Sample {
    let road = roads.exposed_height_and_normal_at(p.x, p.y);
    Sample {
        height: road.map_or_else(|| height(p), |(h, _)| h),
        road: road.map(|(_, n)| n),
        carriageway: road.is_some() && roads.carriageway_height_at(p.x, p.y).is_some(),
    }
}

/// A measured way limit, before rounding its player-facing diagnostic.
pub(crate) struct WayConstraint {
    /// Physical constraint that failed.
    pub(crate) name: &'static str,
    /// Measured rise or grade.
    pub(crate) measured: f32,
    /// Allowed rise or grade in the same unit.
    pub(crate) limit: f32,
    /// Metres, percent, or percentage points.
    pub(crate) unit: &'static str,
}

/// Placement checks a driveway's actual rendered approach and retains the same measured
/// failure as the parking proof, without parsing a display string.
pub(crate) fn way_constraint(
    path: &[Vector2],
    roads: &RoadbedSystem,
    height: &mut impl FnMut(Vector2) -> f32,
) -> Option<WayConstraint> {
    let mut measured = None;
    check_way_report(
        path,
        |p| visible_sample(p, roads, height),
        |name, value, limit, unit| {
            measured = Some(WayConstraint {
                name,
                measured: value,
                limit,
                unit,
            });
        },
    );
    measured
}

// Global half-metre stations continue through corners (not restarted per leg). `HISTORY`
// previous samples hold every rolling wheelbase window, including an interpolated final partial
// station.
// O(legs + samples), constant scratch, no work on a simulation tick.
fn check_way(path: &[Vector2], sample: impl FnMut(Vector2) -> Sample) -> Option<String> {
    check_way_report(path, sample, |_, _, _, _| {})
}

fn check_way_report(
    path: &[Vector2],
    mut sample: impl FnMut(Vector2) -> Sample,
    mut report: impl FnMut(&'static str, f32, f32, &'static str),
) -> Option<String> {
    let (&start, _) = path.split_first()?;
    let total: f32 = path.windows(2).map(|p| p[0].distance_to(p[1])).sum();
    let mut previous = sample(start);
    if !previous.height.is_finite() {
        report("Driveway non-finite heights", 1.0, 0.0, "samples");
        return Some("not drivable: the way in from the street has no finite height".into());
    }
    let mut previous_point = start;
    let mut history = [(0.0, previous.height); HISTORY];
    let mut leg = 0;
    let mut leg_start = 0.0;
    let mut correction = 0.0;
    let mut road_edge_used = false;
    let mut kerb = None::<Vector2>;
    // The station before a footway crossing began, while the way is on that footway.
    let mut crossing = None::<f32>;
    let stations = (total / SAMPLE_M).ceil() as usize;
    for station in 1..=stations {
        let distance = (station as f32 * SAMPLE_M).min(total);
        while leg + 1 < path.len() - 1
            && leg_start + path[leg].distance_to(path[leg + 1]) < distance
        {
            leg_start += path[leg].distance_to(path[leg + 1]);
            leg += 1;
        }
        let length = path[leg].distance_to(path[leg + 1]);
        let point = path[leg].lerp(
            path[leg + 1],
            ((distance - leg_start) / length).clamp(0.0, 1.0),
        );
        let current = sample(point);
        if !current.height.is_finite() {
            report("Driveway non-finite heights", 1.0, 0.0, "samples");
            return Some("not drivable: the way in from the street has no finite height".into());
        }
        // A way leaving the carriageway crosses the footway at a lowered kerb, which the roadbed
        // does not model: it draws the full kerb up and the footway's fall to the yard after it.
        // So the footway's samples are passed over, and the yard must start within a kerb's
        // height (plus the drive grade over the band) of the carriageway's edge.
        let footway = current.road.is_some() && !current.carriageway;
        if footway && (previous.carriageway || crossing.is_some()) {
            crossing.get_or_insert(history[HISTORY - 1].0);
            continue;
        }
        let rise = current.height - previous.height;
        if let Some(from) = crossing.take() {
            let run = distance - from;
            if rise.abs() > MAX_STEP_M + DRIVE_GRADE * run + HEIGHT_EPS_M {
                report(
                    "Driveway footway rise",
                    rise.abs(),
                    MAX_STEP_M + DRIVE_GRADE * run,
                    "m",
                );
                return Some(format!(
                    "not drivable: the yard is {:.2} m {} the street across the footway (at most {:.2} m)",
                    rise.abs(),
                    if rise > 0.0 { "above" } else { "below" },
                    MAX_STEP_M,
                ));
            }
            // The kerb part of the rise is waived; its grade part stays in the wheelbase window,
            // which spans the crossing between the history's stations.
            let grade = DRIVE_GRADE * run;
            correction += rise - rise.clamp(-grade, grade);
            road_edge_used = true;
            let adjusted = current.height - correction;
            history.rotate_left(1);
            history[HISTORY - 1] = (distance, adjusted);
            previous = current;
            previous_point = point;
            continue;
        }
        // The raised road bands have a real 0.12 m vertical kerb (roadbed/profile.rs).
        // Remove only that modelled discontinuity, retaining the road plane's slope. A street
        // path may backtrack over the same kerb; that is one physical kerb, not a second waiver.
        if let (Some(normal), Some(_)) = (previous.road, current.road)
            && previous.carriageway != current.carriageway
        {
            let delta = point - previous_point;
            let plane_rise = -(normal.x * delta.x + normal.z * delta.y) / normal.y;
            let step = rise - plane_rise;
            let crossing = (point + previous_point) * 0.5;
            if !road_edge_used
                && (step.abs() - CURB_STEP_HEIGHT_M).abs() <= 0.002
                && kerb.is_none_or(|old| old.distance_to(crossing) <= SAMPLE_M + 0.01)
            {
                kerb.get_or_insert(crossing);
                correction += step;
            }
        }
        // If no modelled kerb used the one allowance, permit a subtle step at the outer road
        // edge. Never waive a second street step, a yard-interior step or one over `MAX_STEP_M`.
        if previous.road.is_some() != current.road.is_some()
            && !road_edge_used
            && kerb.is_none()
            && rise.abs() <= MAX_STEP_M + HEIGHT_EPS_M
        {
            road_edge_used = true;
            correction += rise;
        } else if rise.abs() > MAX_STEP_M + HEIGHT_EPS_M {
            report("Driveway step", rise.abs(), MAX_STEP_M, "m");
            return Some(format!(
                "not drivable: the way in from the street has a {:.2} m step in {:.2} m (at most {:.2} m)",
                rise.abs(),
                distance - history[HISTORY - 1].0,
                MAX_STEP_M,
            ));
        }
        let adjusted = current.height - correction;
        // Every whole wheelbase of way, or a whole way shorter than one.
        if distance >= WINDOW_M || station == stations {
            // One wheelbase back, interpolated between the stations around it.
            let target = (distance - WINDOW_M).max(0.0);
            let k = history.iter().rposition(|h| h.0 <= target).unwrap_or(0);
            let before = match history.get(k + 1) {
                Some(next) if next.0 > history[k].0 => {
                    history[k].1
                        + (next.1 - history[k].1)
                            * ((target - history[k].0) / (next.0 - history[k].0))
                }
                _ => history[k].1,
            };
            let climb = (adjusted - before).abs();
            let run = distance - target;
            if climb > DRIVE_GRADE * run + HEIGHT_EPS_M {
                report(
                    "Driveway grade",
                    climb / run * 100.0,
                    DRIVE_GRADE * 100.0,
                    "%",
                );
                return Some(format!(
                    "not drivable: the way in from the street climbs {:.2} m in {:.2} m ({:.1} %, at most {:.0} %)",
                    climb,
                    run,
                    climb / run * 100.0,
                    DRIVE_GRADE * 100.0,
                ));
            }
        }
        if distance >= 2.0 * WINDOW_M {
            let height_at = |target: f32| {
                let k = history.iter().rposition(|h| h.0 <= target).unwrap_or(0);
                match history.get(k + 1) {
                    Some(next) if next.0 > history[k].0 => {
                        history[k].1
                            + (next.1 - history[k].1)
                                * ((target - history[k].0) / (next.0 - history[k].0))
                    }
                    _ => history[k].1,
                }
            };
            let middle = height_at(distance - WINDOW_M);
            let before = height_at(distance - 2.0 * WINDOW_M);
            let change = ((adjusted - middle) - (middle - before)).abs() / WINDOW_M;
            if change > MAX_GRADE_CHANGE + HEIGHT_EPS_M / WINDOW_M {
                report(
                    "Driveway grade change per wheelbase",
                    change * 100.0,
                    MAX_GRADE_CHANGE * 100.0,
                    "percentage points",
                );
                return Some(format!(
                    "not drivable: the way in changes grade by {:.1} percentage points per {:.1} m wheelbase (at most {:.0})",
                    change * 100.0,
                    WINDOW_M,
                    MAX_GRADE_CHANGE * 100.0
                ));
            }
        }
        history.rotate_left(1);
        history[HISTORY - 1] = (distance, adjusted);
        previous = current;
        previous_point = point;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parking_wheels_check_full_plane_gradient_between_pair_directions() {
        // A 5.2% plane at 30 degrees has 4.503% crossfall and 2.6% longitudinal
        // grade. On the 1.6 x 2.8 m wheel rectangle even its steepest diagonal is <5%.
        // The two visible wheel triangles still have full gradient 5.2%.
        let angle = std::f32::consts::FRAC_PI_6;
        let reason = spot_issue(ProofClass::Car, Vector2::ZERO, Vector2::UP, &mut |p| {
            0.052 * (p.x * angle.cos() + p.y * angle.sin())
        })
        .unwrap();
        assert_eq!(
            reason,
            "not drivable: the ground under this spot slopes 5.2 %, a parked car needs at most 5 %"
        );
    }

    #[test]
    fn parking_wheels_accept_flat_ground_and_refuse_a_one_in_two_cut() {
        for class in ProofClass::ALL {
            assert!(spot_issue(class, Vector2::ZERO, Vector2::UP, &mut |_| 10.0).is_none());
            let reason =
                spot_issue(class, Vector2::ZERO, Vector2::UP, &mut |p| 10.0 + p.x * 0.5).unwrap();
            assert!(reason.contains("slopes 50.0 %"), "{reason}");
            assert!(reason.contains("at most 5 %"), "{reason}");
        }
    }

    fn street_way(road_height: f32) -> Option<String> {
        check_way(&[Vector2::ZERO, Vector2::new(0.0, 3.0)], |p| Sample {
            height: if p.y < 0.25 { road_height } else { 0.0 },
            road: (p.y < 0.25).then_some(Vector3::UP),
            carriageway: false,
        })
    }

    #[test]
    fn parking_street_way_refuses_four_metres_but_allows_a_subtle_edge_step() {
        let issue = street_way(4.0).unwrap();
        assert!(issue.contains("4.00 m step"), "{issue}");
        assert!(street_way(0.1).is_none());
        assert!(street_way(CURB_STEP_HEIGHT_M).is_none());
        assert!(street_way(CURB_STEP_HEIGHT_M + 0.001).is_some());
    }

    #[test]
    fn parking_way_allows_one_modelled_kerb_but_keeps_the_road_grade() {
        let path = [Vector2::ZERO, Vector2::new(0.0, 3.0)];
        // Carriageway, then its raised footway band, then (`yard`) the yard flush with the
        // carriageway's plane, or (not `yard`) the carriageway again.
        let road = |p: Vector2, grade: f32, yard: bool| {
            let raised = (0.75..2.25).contains(&p.y);
            Sample {
                height: p.y * grade + if raised { CURB_STEP_HEIGHT_M } else { 0.0 },
                road: (!yard || p.y < 2.25).then(|| Vector3::new(0.0, 1.0, -grade).normalized()),
                carriageway: !raised && (!yard || p.y < 2.25),
            }
        };
        assert!(check_way(&path, |p| road(p, 0.10, true)).is_none());
        assert!(check_way(&path, |p| road(p, 0.11, true)).is_some());
        assert!(check_way(&path, |p| road(p, 0.10, false)).is_none());
        assert!(check_way(&path, |p| road(p, 0.11, false)).is_some());
        let backtrack = [Vector2::ZERO, Vector2::new(0.0, 2.0), Vector2::ZERO];
        assert!(check_way(&backtrack, |p| road(p, 0.0, false)).is_none());
        // The native kerb and an extra road-to-yard discontinuity do not get two waivers.
        assert!(
            check_way(&path, |p| {
                if p.y < 1.75 {
                    road(p, 0.0, false)
                } else {
                    Sample {
                        height: 0.32,
                        road: None,
                        carriageway: false,
                    }
                }
            })
            .is_some()
        );
    }

    #[test]
    fn parking_way_checks_rolling_wheelbases_across_corners_and_its_short_end() {
        let path = [
            Vector2::ZERO,
            Vector2::new(0.0, 0.7),
            Vector2::new(0.0, 2.2),
        ];
        let ground = |p: Vector2| Sample {
            height: p.y * 0.11,
            road: None,
            carriageway: false,
        };
        assert!(check_way(&path, ground).unwrap().contains("11.0 %"));
        assert!(
            check_way(&path, |p| Sample {
                height: p.y * 0.10,
                ..ground(p)
            })
            .is_none()
        );
        assert!(
            check_way(&path, |p| Sample {
                height: (p.y - 1.9).max(0.0),
                ..ground(p)
            })
            .is_some()
        );
        // A 20 cm step away from the street, over a kerb's height, still fails.
        assert!(
            check_way(&path, |p| Sample {
                height: if p.y > 0.7 { 0.2 } else { 0.0 },
                ..ground(p)
            })
            .is_some()
        );
    }
}

#[cfg(test)]
mod transition_tests {
    use super::*;

    #[test]
    fn parking_way_grade_change_is_separate_from_its_ten_percent_limit() {
        let path = [Vector2::ZERO, Vector2::new(0.0, 12.0)];
        let check = |grade: f32| {
            check_way(&path, |p| Sample {
                height: (p.y - 5.6).max(0.0) * grade,
                road: None,
                carriageway: false,
            })
        };
        assert!(check(0.05).is_none());
        let reason = check(0.10).unwrap();
        assert!(reason.contains("grade by"), "{reason}");
        assert!(reason.contains("at most 5"), "{reason}");
    }
}
