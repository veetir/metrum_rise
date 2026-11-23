// SPDX-License-Identifier: GPL-3.0-or-later

//! Building-site derivation, grading, terrain ownership, and spatial queries.

mod cells;
mod derive;
mod geometry;
pub(super) mod grading;
mod model;
mod query;
mod yard;

pub(crate) use cells::SiteCells;
pub(crate) use derive::{lot_support_inset_m, site_structure_footprints};
#[cfg(test)]
pub(crate) use grading::SiteRoads;
#[cfg(test)]
pub(crate) use grading::building_site_support_tie_in_is_valid;
pub(crate) use grading::site_feasibility_dependency_margin_m;
pub(crate) use model::BuildingSiteSurfaceClient;
pub(crate) use model::{BuildingSiteClient, BuildingSiteTerrainSnapshot};
pub use model::SiteChange;
pub(crate) use query::site_footprint_contains;
pub(crate) use query::SiteSnapshot;
pub(crate) use yard::derive_yard_terrace;

#[cfg(test)]
mod tests;
