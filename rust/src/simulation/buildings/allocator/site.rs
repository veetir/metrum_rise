// SPDX-License-Identifier: GPL-3.0-or-later

//! Building-site derivation, grading, terrain ownership, and spatial queries.

mod cells;
mod derive;
mod geometry;
pub(super) mod grading;
mod model;
mod query;

pub(crate) use cells::SiteCells;
pub(crate) use derive::{lot_support_inset_m, zoning_support_footprint, site_structure_footprints};
pub(crate) use grading::building_site_support_tie_in_is_valid;
#[cfg(test)]
pub(crate) use grading::SiteRoads;
pub(crate) use grading::{
    site_feasibility_dependency_margin_m, solve_building_site_support_height,
};
pub(crate) use model::BuildingSiteSurfaceClient;
pub(crate) use model::{BuildingSiteClient, BuildingSiteTerrainSnapshot};
pub use model::SiteChange;
pub(crate) use query::site_footprint_contains;
pub(crate) use query::SiteSnapshot;

#[cfg(test)]
mod tests;
