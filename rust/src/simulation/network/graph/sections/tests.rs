// SPDX-License-Identifier: GPL-2.0-only

//! Bounded layout identity, retained dimensions and memory contract regressions.

use super::*;

fn road() -> Edge {
    Edge {
        primary_type: TransitType::Road,
        allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
        width: 7.0,
        fwd_lanes: 1,
        bkw_lanes: 1,
        ..Default::default()
    }
}

#[test]
fn canonical_interning_validation_and_reversal() {
    let mut table = RoadSections::default();
    let mut layout = RoadLayout::legacy(&road());
    assert!(layout.validate(true).is_ok());
    assert_eq!(layout.validate(false), Err("invalid_clear_walking_space"));
    assert_eq!(table.intern(layout.clone()), table.intern(layout.clone()));
    assert_eq!(SectionScalar::new(-0.0), SectionScalar::new(0.0));
    assert!(SectionScalar::new(f32::NAN).is_err());
    assert!(SectionScalar::new(f32::INFINITY).is_err());
    layout.strips[0][1].direction = 1;
    layout.lane_counts = [2, 1];
    layout.lane_widths[0][1] = SectionScalar::new(3.25).unwrap();
    assert_eq!(layout.reversed().reversed(), layout);
    assert_eq!(layout.reversed().strips[1][1].direction, -1);
    assert_ne!(table.intern(layout.clone()), 0);
    layout.strip_counts[0] = 9;
    assert_eq!(layout.validate(true), Err("section_component_limit"));
}

#[test]
fn explicit_legacy_path_and_no_foot_dimensions() {
    let mut edge = road();
    let layout = RoadLayout::legacy(&edge);
    assert_eq!(
        layout.strips[0][1].width.get().to_bits(),
        (1.5_f32 - 0.15).to_bits()
    );
    edge.allowed_types = TransitFlags::CAR;
    let layout = RoadLayout::legacy(&edge);
    assert_eq!(layout.strips[0][0].role, StripRole::Shoulder);
    assert_eq!(layout.strips[0][0].width.get(), 0.75);
    assert_eq!(layout.strips[0][0].heights[0].get(), 0.12);
    edge.primary_type = TransitType::Foot;
    edge.width = 1.25;
    edge.fwd_lanes = 0;
    edge.bkw_lanes = 0;
    let layout = RoadLayout::legacy(&edge);
    assert_eq!(layout.core_width.get(), 1.25);
    assert_eq!(layout.path_width.get(), 2.0);
    assert_eq!(layout.strip_counts, [0, 0]);
}

#[test]
fn section_memory_budget() {
    assert_eq!(std::mem::size_of::<RoadSectionRef>(), 16);
    assert_eq!(std::mem::size_of::<RoadSideExtents>(), 16);
    assert_eq!(std::mem::size_of::<RoadStrip>(), 20);
    assert!(std::mem::size_of::<RoadLayout>() <= 384);
    let refs = vec![RoadSectionRef::default(); 100_000];
    let extents = vec![RoadSideExtents::default(); 100_000];
    assert_eq!((refs.capacity() + extents.capacity()) * 16, 3_200_000);
    let mut table = RoadSections::default();
    for index in 0..1_000 {
        let mut layout = RoadLayout::legacy(&road());
        layout.core_width = SectionScalar::new(7.0 + index as f32 * 0.125).unwrap();
        table.intern(layout);
    }
    if let Ok(path) = std::env::var("ROAD_SECTION_MEMORY") {
        std::fs::write(path,serde_json::to_vec_pretty(&serde_json::json!({
            "edge_size":std::mem::size_of::<Edge>(),
            "reference_size":std::mem::size_of::<RoadSectionRef>(),
            "extent_size":std::mem::size_of::<RoadSideExtents>(),
            "strip_size":std::mem::size_of::<RoadStrip>(),
            "layout_size":std::mem::size_of::<RoadLayout>(),
            "detail_size":std::mem::size_of::<RoadSectionDetail>(),
            "reference_capacity":refs.capacity(),"extent_capacity":extents.capacity(),
            "reference_extent_bytes":(refs.capacity()+extents.capacity())*16,
            "distinct_layouts":table.layouts.len(),"layout_capacity":table.layouts.capacity(),
            "layout_payload_bytes":table.layouts.len()*std::mem::size_of::<RoadLayout>(),
            "layout_pointer_capacity_bytes":table.layouts.capacity()*std::mem::size_of::<Arc<RoadLayout>>(),
            "intern_entries":table.intern.len(),
            "intern_entry_type_size":std::mem::size_of::<(Arc<RoadLayout>,u32)>(),
            "note":"Arc counters and allocator/BTree node overhead are additional; payload is shared, not duplicated by keys"
        })).unwrap()).unwrap();
    }
}

fn graph() -> RegionGraph {
    use crate::simulation::network::types::NodeType;
    use godot::prelude::Vector3;
    let mut graph = RegionGraph::new();
    let mut edge = road();
    edge.start_node = graph.add_node(Vector3::ZERO, NodeType::Junction);
    edge.end_node = graph.add_node(Vector3::new(100.0, 0.0, 0.0), NodeType::Junction);
    edge.geometry = vec![Vector3::ZERO, Vector3::new(100.0, 0.0, 0.0)];
    edge.physical_geometry = edge.geometry.clone();
    edge.physical_length = 100.0;
    graph.add_edge(edge);
    graph
}

#[test]
fn section_queries_clone_and_bounded_undo() {
    let mut graph = graph();
    let before = graph.road_layout(0).clone();
    let reference = graph.road_section_ref(0);
    assert_eq!(graph.lane_center(0, true, 0, 50.0), Some(1.75));
    assert_eq!(graph.lane_center(0, false, 0, 50.0), Some(-1.75));
    assert_eq!(graph.walk_axis(0, 0, 50.0), Some(-4.25));
    assert_eq!(graph.walk_axis(0, 1, 50.0), Some(4.25));
    assert_eq!(graph.paved_bounds(0, 50.0), [-5.0, 5.0]);
    assert_eq!(graph.owned_ground_bounds(0, 50.0), [-5.0, 5.0]);
    assert_eq!(graph.road_side_extents(0).owned, [-5.0, 5.0]);
    assert_eq!(graph.frontage_boundary(0, 0, 50.0), Some(-5.0));
    assert_eq!(graph.clearance_envelope(0, 50.0), [-5.0, 5.0]);
    let undo = graph.capture_full_undo();
    graph.set_edge_cross_section(0, 2, 1, None).unwrap();
    assert_ne!(*graph.road_layout(0), before);
    assert_ne!(graph.road_section_ref(0).revision, reference.revision);
    assert_eq!(*graph.clone().road_layout(0), *graph.road_layout(0));
    graph.restore_undo_delta(undo);
    assert_eq!(*graph.road_layout(0), before);
    assert_eq!(graph.road_section_ref(0), reference);
    assert!(graph.validate_road_sections().is_ok());
}

#[test]
fn split_keeps_explicit_authority_and_undo_removes_appended_slots() {
    use crate::simulation::buildings::allocator::BuildingAllocator;
    use crate::simulation::core::config::WorldConfig;
    use crate::simulation::network::{TransitNetwork, topology::split_edge, types::NodeType};
    use crate::simulation::zoning::ZoningSystem;
    use godot::prelude::Vector3;
    let mut graph = graph();
    let original = graph.road_layout(0).clone();
    let undo = graph.capture_full_undo();
    let junction = graph.add_node(Vector3::new(50.0, 0.0, 0.0), NodeType::Junction);
    let mut network = TransitNetwork::new();
    split_edge(
        &mut network,
        &mut graph,
        0,
        0,
        0.5,
        junction,
        &mut ZoningSystem::new(&WorldConfig::default()),
        &mut BuildingAllocator::new(),
    );
    assert_eq!(graph.edge_count(), 2);
    assert_eq!(*graph.road_layout(0), original);
    assert_eq!(*graph.road_layout(1), original);
    assert_eq!(
        graph.road_section_ref(0).layout,
        graph.road_section_ref(1).layout
    );
    assert!(graph.validate_road_sections().is_ok());
    let mut excerpt = RegionGraph::new();
    for node in graph.nodes() {
        excerpt.add_node(node.pos, node.node_type);
    }
    let id = excerpt.add_edge(graph.edge(1).clone());
    excerpt.copy_road_section(id, &graph, 1);
    assert_eq!(
        excerpt.road_section_ref(id).revision,
        graph.road_section_ref(1).revision
    );
    assert_eq!(excerpt.road_layout(id), graph.road_layout(1));
    graph.restore_undo_delta(undo);
    assert_eq!(graph.edge_count(), 1);
    assert_eq!(graph.road_sections.references.len(), 1);
    assert_eq!(graph.road_sections.extents.len(), 1);
    assert_eq!(*graph.road_layout(0), original);
}

#[test]
fn exceptional_core_width_preserves_exact_lane_envelope_walk_axes() {
    let mut edge = road();
    edge.width = 8.125;
    let layout = RoadLayout::legacy(&edge);
    assert_eq!(layout.walk_axes.map(SectionScalar::get), [-4.25, 4.25]);
    assert_ne!(layout.walk_axes[1].get(), edge.width * 0.5 + 0.75);
    assert_eq!(layout.reversed().walk_axes, layout.walk_axes);
}
