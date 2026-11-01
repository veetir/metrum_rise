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

// O(bays + local polygon/overlay work) per tested sweep; only authoring/edit boundaries.
pub(super) fn validate_bay(
    slot: usize,
    bays: &[AnchorBay],
    footprints: &[[Vector2; 4]],
    support: &[Vec<[f64; 2]>],
    has_driveway: bool,
    paths: &[[Vector2; 2]],
    collision: impl Fn(&[Vector2]) -> bool,
) -> Result<[Vector2; 4], &'static str> {
    let bay = &bays[slot];
    if !has_driveway {
        return Err("missing driveway anchor");
    }
    if bay.width < 1.8 || bay.length < 4.5 || bay.forward.length_squared() < 0.99 {
        return Err("bay too small or invalid direction");
    }
    if collision(&footprints[slot]) {
        return Err("bay overlaps a structure");
    }
    let other_bay = |foot: &[Vector2]| {
        footprints
            .iter()
            .enumerate()
            .any(|(other, b)| other != slot && overlaps(foot, b))
    };
    if other_bay(&footprints[slot]) {
        return Err("bay overlaps another bay");
    }
    if !covered(&footprints[slot], support) {
        return Err("no paved support");
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
    Err("no clear driveway–aisle–bay path")
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
        .map(|(_, a)| AnchorBay {
            pos: vector(&a["position"]),
            forward: vector(&a["forward"]).normalized(),
            width: number(a, "width_m", 2.5),
            length: number(a, "length_m", 5.0),
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
        .chunks_exact(4)
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
        .map(|a| {
            let approach = vector(&a["position"]) + vector(&a["forward"]).normalized() * 2.5;
            [approach, approach]
        })
        .collect();
    let rejected: Vec<_> = parking
        .iter()
        .enumerate()
        .filter_map(|(slot, (index, _))| {
            validate_bay(
                slot,
                &bays,
                &footprints,
                &support,
                has_driveway,
                &paths,
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
            "missing driveway anchor"
        );
        params["anchors"]
            .as_array_mut()
            .unwrap()
            .push(json!({"anchor_type":"driveway",
            "position":[0,0,-10], "forward":[0,0,1]}));
        assert_eq!(editor_payload(&params, &[])["rejected"], json!([]));
    }
}
