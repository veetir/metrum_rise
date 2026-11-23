// SPDX-License-Identifier: GPL-3.0-or-later

//! Asset-independent parcel terrain checks using the shared grading solver and local cache.

use super::*;
impl BuildingAllocator {
    /// Zoning reserves land even on rough/impossible terrain; released paint evaluates allowed
    /// assets and displays the derived verdict. Physical overlaps still refuse the designation.
    pub(crate) fn zoning_site_feasibility(
        &self,
        geometry: &ParcelGeometry,
        profile_id: u16,
        zoning: &ZoningSystem,
        _graph: &RegionGraph,
        _env: BuildingSiteEnvironment<'_>,
    ) -> Result<(), &'static str> {
        if self.field_clearance.overlaps_polygon(&geometry.corners) {
            return Err("field_overlap");
        }
        // Zoned land an extraction area took does not grow back in the pit (`ZONE-11`).
        if self.extraction_clearance.overlaps_polygon(&geometry.corners) {
            return Err("extraction_area_overlap");
        }
        if self.parcel_geometry_overlaps_explicit_site(geometry, zoning.config.zone_cell_m) {
            return Err("parcel overlaps a placed building site");
        }
        // Clearing a zoning designation must remain possible even on unusable terrain.
        if profile_id == 0 {
            return Ok(());
        }
        let profile = zoning
            .profiles
            .profile_by_runtime_id(profile_id)
            .ok_or("unknown zoning profile")?;
        zone_type_to_zone_class(profile.zone_type).ok_or("unsupported zoning profile")?;
        Ok(())
    }
}
