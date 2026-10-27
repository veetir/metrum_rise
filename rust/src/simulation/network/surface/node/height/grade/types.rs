// SPDX-License-Identifier: GPL-2.0-only

//! Grade-local authority and candidate data types.

use super::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(crate) enum NodeGradeCarrierDecision {
    SourceCarrier {
        authority: Option<NodeHeightAuthoritySource>,
    },
    SameOwnerCanonicalVertex,
    SameMaterialSharedEdge,
    SameMaterialVertex,
    SameMaterialSeam,
    ExplicitMaterialSeam,
    /// A shared node grade with the exact material offset proved from its source cross section.
    DeclaredMaterialContact {
        /// Canonical grade below the source-declared material level, in millimetres.
        base_height_mm: i64,
        /// Source-declared offset above that grade, in millimetres.
        level_mm: i64,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(crate) struct NodeGradeVertexAuthority {
    pub(crate) key: SurfaceXzKey,
    pub(crate) owner: NodeBandOwner,
    pub(crate) height_field_id: NodeBandHeightFieldId,
    pub(crate) height_key: SurfaceHeightMmKey,
    pub(crate) source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(crate) decision: NodeGradeCarrierDecision,
}

impl NodeGradeVertexAuthority {
    /// Proves a material step from two contacts on the same resolved node grade.
    pub(crate) fn shares_declared_material_base(self, other: Self) -> bool {
        let (
            NodeGradeCarrierDecision::DeclaredMaterialContact {
                base_height_mm: a,
                level_mm: al,
            },
            NodeGradeCarrierDecision::DeclaredMaterialContact {
                base_height_mm: b,
                level_mm: bl,
            },
        ) = (self.decision, other.decision)
        else {
            return false;
        };
        let step = (f64::from(crate::simulation::network::surface::CURB_STEP_HEIGHT_M) * 1000.0)
            .round() as i64;
        self.key == other.key
            && a == b
            && self.owner.kind() != other.owner.kind()
            && [al, bl].iter().all(|&level| level == 0 || level == step)
            && self.height_key.as_i64() == a + al
            && other.height_key.as_i64() == b + bl
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(crate) struct NodeGradeExplicitSeamHeightKey {
    pub(super) point: SurfaceXzKey,
    pub(super) constraint_index: usize,
    pub(super) owner: Option<NodeBandOwner>,
    pub(super) opposite_owner: Option<NodeBandOwner>,
    pub(super) start: SurfaceXzKey,
    pub(super) end: SurfaceXzKey,
}

#[derive(Clone, Copy)]
pub(super) struct SameMaterialVertexHeightCandidate {
    pub(super) owner: NodeBandOwner,
    pub(super) height_field_id: NodeBandHeightFieldId,
    pub(super) height_m: f64,
    pub(super) height_authority: Option<NodeHeightAuthoritySource>,
    pub(super) source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) has_explicit_shared_material_seam: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct SameMaterialSharedEdgeKey {
    pub(super) kind: RoadSurfaceBandKind,
    pub(super) start: SurfaceXzKey,
    pub(super) end: SurfaceXzKey,
    pub(super) start_source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) end_source_provenance: Option<NodeHeightCarrierProvenanceKey>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct SameMaterialSharedEdgeCandidate {
    pub(super) owner: NodeBandOwner,
    pub(super) height_field_id: NodeBandHeightFieldId,
    pub(super) start: SurfaceXzKey,
    pub(super) start_height_m: f64,
    pub(super) start_height_authority: Option<NodeHeightAuthoritySource>,
    pub(super) start_source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) start_has_explicit_shared_material_seam: bool,
    pub(super) start_has_explicit_height_split: bool,
    pub(super) end: SurfaceXzKey,
    pub(super) end_height_m: f64,
    pub(super) end_height_authority: Option<NodeHeightAuthoritySource>,
    pub(super) end_source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) end_has_explicit_shared_material_seam: bool,
    pub(super) end_has_explicit_height_split: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct SameMaterialSharedVertexKey {
    pub(super) kind: RoadSurfaceBandKind,
    pub(super) point: SurfaceXzKey,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct SameMaterialSharedVertexContext {
    pub(super) owner: NodeBandOwner,
    pub(super) height_field_id: NodeBandHeightFieldId,
    pub(super) source_provenance: Option<NodeHeightCarrierProvenanceKey>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct SameMaterialVertexHeightContext {
    pub(super) owner: NodeBandOwner,
    pub(super) height_field_id: NodeBandHeightFieldId,
    pub(super) source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) height_mm: i64,
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct SameMaterialVertexHeightSupportKey {
    pub(super) kind: RoadSurfaceBandKind,
    pub(super) point: SurfaceXzKey,
    pub(super) source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    pub(super) explicit_seams: Vec<NodeGradeExplicitSeamHeightKey>,
    pub(super) explicit_height_splits: Vec<(NodeBandOwner, NodeGradeExplicitSeamHeightKey)>,
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(super) struct NodeGradeVertexContextKey {
    pub(super) point: SurfaceXzKey,
    pub(super) owner: NodeBandOwner,
    pub(super) height_field_id: NodeBandHeightFieldId,
}

pub(super) struct SameMaterialSharedEdgeHeightAgreement {
    pub(super) selected_by_vertex:
        BTreeMap<SameMaterialSharedVertexKey, SameMaterialVertexHeightCandidate>,
    pub(super) affected_contexts_by_vertex:
        BTreeMap<SameMaterialSharedVertexKey, Vec<SameMaterialSharedVertexContext>>,
}

pub(super) struct SameMaterialVertexHeightGroups {
    pub(super) by_key: BTreeMap<SameMaterialVertexHeightSupportKey, SameMaterialVertexHeightGroup>,
}

pub(super) struct SameMaterialVertexHeightGroup {
    pub(super) contexts: Vec<SameMaterialVertexHeightContext>,
    pub(super) candidates: Vec<SameMaterialVertexHeightCandidate>,
    pub(super) selected: SameMaterialVertexHeightCandidate,
    pub(super) occurrences: Vec<(usize, usize, usize)>,
}

impl NodeGradeVertexAuthority {
    pub(crate) fn new(
        point_xz: RoadVec2,
        height_m: f64,
        owner: NodeBandOwner,
        height_field_id: NodeBandHeightFieldId,
        decision: NodeGradeCarrierDecision,
    ) -> Self {
        Self::new_with_source_provenance(point_xz, height_m, owner, height_field_id, decision, None)
    }

    pub(crate) fn new_with_source_provenance(
        point_xz: RoadVec2,
        height_m: f64,
        owner: NodeBandOwner,
        height_field_id: NodeBandHeightFieldId,
        decision: NodeGradeCarrierDecision,
        source_provenance: Option<NodeHeightCarrierProvenanceKey>,
    ) -> Self {
        Self {
            key: SurfaceXzKey::from_road_xz(point_xz),
            owner,
            height_field_id,
            height_key: SurfaceHeightMmKey::from_m_f64(height_m),
            source_provenance,
            decision,
        }
    }
}

impl SameMaterialVertexHeightContext {
    pub(super) fn from_candidate(candidate: SameMaterialVertexHeightCandidate) -> Self {
        Self {
            owner: candidate.owner,
            height_field_id: candidate.height_field_id,
            source_provenance: candidate.source_provenance,
            height_mm: SurfaceHeightMmKey::from_m_f64(candidate.height_m).as_i64(),
        }
    }
}
