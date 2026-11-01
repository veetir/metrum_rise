// SPDX-License-Identifier: GPL-2.0-only

//! Shared asset-local parking proofs; frontage connections are supplied only by the city.

use super::geometry::{covered, overlaps, rectangle};
use godot::prelude::Vector2;
use serde_json::{Value, json};

pub(super) struct AnchorBay {
    pub pos: Vector2,
    pub forward: Vector2,
    pub width: f32,
    pub length: f32,
}

const SUPPORT_TOLERANCE_M: f32 = 0.05;

pub(super) fn driveway_points_inward(forward: Vector2, outward: Vector2) -> bool {
    forward.dot(-outward) >= 0.9
}

// How far a bay's entry edge may sit from the lot's street edge and still count as reached
// straight from the street.
const STREET_EDGE_TOLERANCE_M: f32 = 0.6;
// Where a car stops on the street before turning straight into a street-edge bay.
const STREET_APPROACH_M: f32 = 2.5;

/// Whether a bay, in asset-local coordinates, opens straight onto the street: its entry edge
/// lies on the lot's frontage edge and it points into the lot. Such a bay needs no Driveway.
/// `half_extents` are the lot's half width (x) and half depth (z). O(1).
pub(super) fn opens_onto_street(
    entry: Vector2,
    forward: Vector2,
    outward: Vector2,
    half_extents: Vector2,
) -> bool {
    let edge = outward.x.abs() * half_extents.x + outward.y.abs() * half_extents.y;
    driveway_points_inward(forward, outward)
        && (entry.dot(outward) - edge).abs() <= STREET_EDGE_TOLERANCE_M
}

// O(bays + local polygon/overlay work) per tested sweep; only authoring/edit boundaries.
// `street` is the road connection for a bay that opens onto the street, if any.
#[allow(clippy::too_many_arguments)]
pub(super) fn validate_bay(
    slot: usize,
    bays: &[AnchorBay],
    footprints: &[[Vector2; 4]],
    support: &[Vec<[f64; 2]>],
    has_driveway: bool,
    paths: &[[Vector2; 2]],
    street: Option<Vector2>,
    collision: impl Fn(&[Vector2]) -> bool,
) -> Result<[Vector2; 4], &'static str> {
    let bay = &bays[slot];
    if !has_driveway && street.is_none() {
        return Err(
            "needs a Driveway point, or put the spot's front edge on the street edge pointing into the lot",
        );
    }
    if bay.width < 1.8 || bay.length < 4.5 || bay.forward.length_squared() < 0.99 {
        return Err("too small for a car: make it at least 1.8 m wide and 4.5 m long");
    }
    if collision(&footprints[slot]) {
        return Err("overlaps the building or another structure");
    }
    let other_bay = |foot: &[Vector2]| {
        footprints
            .iter()
            .enumerate()
            .any(|(other, b)| other != slot && overlaps(foot, b))
    };
    if other_bay(&footprints[slot]) {
        return Err("overlaps another parking spot");
    }
    // Authored surfaces stop a few centimetres short of the lot edge they were drawn to.
    let inset = rectangle(
        bay.pos,
        bay.forward,
        bay.width - SUPPORT_TOLERANCE_M * 2.0,
        bay.length - SUPPORT_TOLERANCE_M * 2.0,
    );
    if !covered(&inset, support) {
        return Err(
            "not fully on asphalt or concrete: move the whole spot onto a paved yard surface",
        );
    }
    if let Some(connection) = street {
        // The car turns in from the street and drives straight onto the bay.
        let entry = bay.pos - bay.forward * (bay.length * 0.5);
        return Ok([
            connection,
            entry - bay.forward * STREET_APPROACH_M,
            entry,
            bay.pos,
        ]);
    }
    for &[connection, approach] in paths {
        let aisle = bay.pos - bay.forward * (bay.length * 0.5 + 2.5);
        let path = [connection, approach, aisle, bay.pos];
        let clear =
            |foot: &[Vector2]| covered(foot, support) && !collision(foot) && !other_bay(foot);
        let segments_clear = path.windows(2).all(|pair| {
            let delta = pair[1] - pair[0];
            delta.length_squared() < 1e-8
                || clear(&rectangle(
                    (pair[0] + pair[1]) * 0.5,
                    delta.normalized(),
                    1.8,
                    delta.length() + 4.5,
                ))
        });
        // Full turning envelopes prove the orientation change at both aisle corners.
        if segments_clear
            && [approach, aisle]
                .iter()
                .all(|&corner| clear(&rectangle(corner, Vector2::new(0.0, 1.0), 4.85, 4.85)))
        {
            return Ok(path);
        }
    }
    if paths.is_empty() {
        return Err("the Driveway must sit at the street edge and point into the lot");
    }
    Err(
        "no room to drive in: the way from the Driveway to this spot must be paved, clear of buildings and other spots, and about 5 m wide where the car turns",
    )
}

/// Editor payload with original anchor indices and exact local rejection reasons.
/// O(A²D + AD × local polygon overlay work), independent of city size. The road-to-driveway
/// segment needs a placed frontage and is checked by `lot_bays`, never guessed in the editor.
pub(crate) fn editor_payload(params: &Value, structures: &[f32]) -> Value {
    let anchors = params["anchors"]
        .as_array()
        .map(Vec::as_slice)
        .unwrap_or_default();
    let vector = |value: &Value| {
        Vector2::new(
            value[0].as_f64().unwrap_or(0.0) as f32,
            value[2].as_f64().unwrap_or(0.0) as f32,
        )
    };
    let number = |a: &Value, key: &str, fallback| a[key].as_f64().unwrap_or(fallback) as f32;
    let parking: Vec<_> = anchors
        .iter()
        .enumerate()
        .filter(|(_, a)| a["anchor_type"] == "parking")
        .collect();
    let bays: Vec<_> = parking
        .iter()
        .map(|(_, a)| {
            // Entry edge to centre, matching `lot_bays`.
            let forward = vector(&a["forward"]).normalized();
            let length = number(a, "length_m", 5.0);
            AnchorBay {
                pos: vector(&a["position"]) + forward * (length * 0.5),
                forward,
                width: number(a, "width_m", 2.5),
                length,
            }
        })
        .collect();
    let footprints: Vec<_> = bays
        .iter()
        .map(|b| rectangle(b.pos, b.forward, b.width, b.length))
        .collect();
    let support: Vec<_> = params["site_surfaces"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|s| s["material"] == "asphalt" || s["material"] == "concrete")
        .filter_map(|s| s["vertices"].as_array())
        .map(|vertices| {
            vertices
                .iter()
                .map(|p| [p[0].as_f64().unwrap_or(0.0), p[1].as_f64().unwrap_or(0.0)])
                .collect()
        })
        .collect();
    let walls: Vec<_> = structures
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| {
            [
                Vector2::new(b[0], b[1]),
                Vector2::new(b[2], b[1]),
                Vector2::new(b[2], b[3]),
                Vector2::new(b[0], b[3]),
            ]
        })
        .collect();
    let has_driveway = anchors.iter().any(|a| a["anchor_type"] == "driveway");
    let paths: Vec<_> = anchors
        .iter()
        .filter(|a| a["anchor_type"] == "driveway" && number(a, "width_m", 3.0) >= 1.8)
        .filter(|a| {
            params.get("frontage_forward").is_none_or(|outward| {
                driveway_points_inward(
                    vector(&a["forward"]).normalized(),
                    vector(outward).normalized(),
                )
            })
        })
        .map(|a| {
            let approach = vector(&a["position"]) + vector(&a["forward"]).normalized() * 2.5;
            [approach, approach]
        })
        .collect();
    let outward = params
        .get("frontage_forward")
        .map(|outward| vector(outward).normalized());
    let half_extents = params.get("lot_half_extents").map(|h| {
        Vector2::new(
            h[0].as_f64().unwrap_or(0.0) as f32,
            h[1].as_f64().unwrap_or(0.0) as f32,
        )
    });
    let rejected: Vec<_> = parking
        .iter()
        .enumerate()
        .filter_map(|(slot, (index, _))| {
            let bay = &bays[slot];
            let entry = bay.pos - bay.forward * (bay.length * 0.5);
            // Without a city the street side of the edge is assumed open; the placed
            // building's inspector reports the real road connection.
            let street = outward
                .zip(half_extents)
                .filter(|&(outward, half)| opens_onto_street(entry, bay.forward, outward, half))
                .map(|_| entry - bay.forward * STREET_APPROACH_M);
            validate_bay(
                slot,
                &bays,
                &footprints,
                &support,
                has_driveway,
                &paths,
                street,
                |foot| walls.iter().any(|wall| overlaps(foot, wall)),
            )
            .err()
            .map(|reason| json!({"anchor":index, "reason":reason}))
        })
        .collect();
    json!({"rejected":rejected, "frontage_check":"Road connection checked in the placed building inspector"})
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn editor_payload_reports_missing_driveway_and_accepts_clear_asset() {
        let mut params = json!({"anchors":[{"anchor_type":"parking", "position":[0,0,8],
            "forward":[0,0,1], "width_m":2.5, "length_m":5.0}],
            "site_surfaces":[{"material":"asphalt", "vertices":[[-20,-20],[20,-20],[20,20],[-20,20]]}]});
        assert_eq!(
            editor_payload(&params, &[])["rejected"][0]["reason"],
            "needs a Driveway point, or put the spot's front edge on the street edge pointing into the lot"
        );
        params["anchors"]
            .as_array_mut()
            .unwrap()
            .push(json!({"anchor_type":"driveway",
            "position":[0,0,-10], "forward":[0,0,1]}));
        assert_eq!(editor_payload(&params, &[])["rejected"], json!([]));
    }

    #[test]
    fn a_bay_opening_onto_the_street_needs_no_driveway() {
        // A detached-house layout: the bay's front edge is the lot's street edge.
        let mut params = json!({"frontage_forward":[0,0,1], "lot_half_extents":[10,15],
            "anchors":[{"anchor_type":"parking", "position":[-4.2,0,15],
            "forward":[0,0,-1], "width_m":2.5, "length_m":4.6}],
            "site_surfaces":[{"material":"asphalt",
            "vertices":[[-6.9,8.07],[4.18,8],[4.2,14.98],[-6.86,14.98]]}]});
        assert_eq!(editor_payload(&params, &[])["rejected"], json!([]));
        // Set back from the street edge, the same bay needs a Driveway.
        params["anchors"][0]["position"] = json!([-4.2, 0, 13]);
        params["anchors"][0]["length_m"] = json!(4.5);
        assert_eq!(
            editor_payload(&params, &[])["rejected"][0]["reason"],
            "needs a Driveway point, or put the spot's front edge on the street edge pointing into the lot"
        );
    }
}
