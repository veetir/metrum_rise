// SPDX-License-Identifier: GPL-2.0-only

//! Road network graph data structures and spatial indexing.

mod cross_section;
mod crossings;
mod lane_access;
mod lane_taper;
pub(crate) mod sections;
pub use lane_access::LaneUse;
pub use lane_taper::RoadLaneTaper;
pub use sections::{
    RoadLayout, RoadSectionDetail, RoadSectionRef, RoadSideExtents, RoadStrip, SectionScalar,
    StripRole,
};
/// Core data structures for nodes and edges.
pub mod data;
pub use cross_section::{MedianStyle, RoadMedian};
/// Logic for rebuilding adjacency lists and connectivity.
pub mod rebuild;
/// Chunk-based spatial index for fast distance and AABB queries.
pub mod spatial;

pub(crate) use data::RegionGraphUndoDelta;
pub use data::{Edge, Node, RegionGraph};
