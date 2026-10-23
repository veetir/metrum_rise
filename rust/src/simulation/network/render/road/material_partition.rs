// SPDX-License-Identifier: GPL-2.0-only

//! Attribute-preserving subtraction of foreign carriageways from nonroad materials.

use super::super::ownership::Vertex;
use super::*;
use crate::simulation::network::surface::{
    RoadMaterialClipTriangle, RoadMaterialOwner, RoadSurfaceMaterialMask,
};
use std::collections::HashMap;

/// Mesh-job-local material certificates and reusable clipping scratch.
#[derive(Debug, Default)]
pub(in crate::simulation::network::render) struct MaterialPartition {
    masks: HashMap<NetworkMeshOwner, RoadSurfaceMaterialMask>,
    items: Vec<u32>,
    clips: Vec<RoadMaterialClipTriangle>,
    /// Invalidates the payload if a local material union cannot be compiled.
    pub(in crate::simulation::network::render) failed: bool,
    pending: Vec<[ClipVertex; 3]>,
    remaining: Vec<[ClipVertex; 3]>,
}

impl MaterialPartition {
    /// Drops compile-only source references before a mesh enters the resident cache.
    pub(in crate::simulation::network::render) fn release_working_storage(&mut self) {
        *self = Self {
            failed: self.failed,
            ..Self::default()
        };
    }
}

// Positions stay f64 across all mask triangles. Rounding each fragment back to the
// upload grid between cuts can turn a skinny convex polygon into a zigzag.
#[derive(Clone, Copy, Debug, Default)]
struct ClipVertex {
    point: glam::DVec3,
    attributes: Vertex,
}

impl ClipVertex {
    fn new(attributes: Vertex) -> Self {
        Self {
            point: glam::DVec3::new(
                f64::from(attributes.p.x),
                f64::from(attributes.p.y),
                f64::from(attributes.p.z),
            ),
            attributes,
        }
    }
    fn lerp(self, other: Self, t: f64) -> Self {
        Self {
            point: self.point.lerp(other.point, t),
            attributes: self.attributes.lerp(other.attributes, t as f32),
        }
    }
    fn render(self) -> Vertex {
        Vertex {
            p: Vector3::new(
                self.point.x as f32,
                self.point.y as f32,
                self.point.z as f32,
            ),
            ..self.attributes
        }
    }
}

// Three halfplanes starting from a triangle can at most double the vertex count
// at each cut, even under roundoff: 3 * 2^3. This bound does not assume convexity.
struct ClipPolygon {
    vertices: [ClipVertex; 24],
    len: usize,
}

impl ClipPolygon {
    fn empty() -> Self {
        Self {
            vertices: [ClipVertex::default(); 24],
            len: 0,
        }
    }
    fn push(&mut self, vertex: ClipVertex) {
        if self.len == 0 || self.vertices[self.len - 1].point != vertex.point {
            self.vertices[self.len] = vertex;
            self.len += 1;
        }
    }
    fn split(&self, side: impl Fn(&ClipVertex) -> f64) -> (Self, Self) {
        let mut low = Self::empty();
        let mut high = Self::empty();
        for i in 0..self.len {
            let a = self.vertices[i];
            let b = self.vertices[(i + 1) % self.len];
            let av = side(&a);
            let bv = side(&b);
            if av <= 0.0 {
                low.push(a);
            }
            if av >= 0.0 {
                high.push(a);
            }
            if (av < 0.0 && bv > 0.0) || (av > 0.0 && bv < 0.0) {
                let v = a.lerp(b, av / (av - bv));
                low.push(v);
                high.push(v);
            }
        }
        (low, high)
    }
}

pub(super) fn prepare(
    mesh: &mut NetworkMeshData,
    graph: &RegionGraph,
    surface: &RoadSurfaceSystem,
    coverage: &standard_surface::CompiledSurfaceCoverage,
) {
    for owner in coverage
        .edge_indices
        .iter()
        .copied()
        .map(NetworkMeshOwner::Edge)
        .chain(
            coverage
                .node_ids
                .iter()
                .copied()
                .map(NetworkMeshOwner::Node),
        )
    {
        let source = match owner {
            NetworkMeshOwner::Edge(id) => RoadMaterialOwner::Span(id),
            NetworkMeshOwner::Node(id) => RoadMaterialOwner::Node(id),
        };
        if let Some(mask) = surface.material_partition_mask(graph, source) {
            mesh.material_partition.masks.insert(owner, mask);
        }
    }
}

pub(super) fn emit_triangle(
    mesh: &mut NetworkMeshData,
    layer: MeshLayer,
    vertices: [Vector3; 3],
    uvs: [Vector2; 3],
    color: Color,
    normal: Vector3,
) -> bool {
    let Some(owner) = mesh.current_owner else {
        return false;
    };
    if !mesh.material_partition.masks.contains_key(&owner) {
        return false;
    }
    let bounds = vertices.into_iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |b, p| {
            [
                b[0].min(f64::from(p.x)),
                b[1].min(f64::from(p.z)),
                b[2].max(f64::from(p.x)),
                b[3].max(f64::from(p.z)),
            ]
        },
    );
    // The sink owns its scratch while emitting, so source certificates stay immutable
    // and no per-triangle allocation or owner-map clone is necessary.
    let mut partition = std::mem::take(&mut mesh.material_partition);
    if partition
        .masks
        .get_mut(&owner)
        .and_then(|mask| mask.collect_clips(bounds, &mut partition.items, &mut partition.clips))
        .is_none()
    {
        partition.failed = true;
        mesh.material_partition = partition;
        return true;
    }
    if partition.clips.is_empty() {
        mesh.material_partition = partition;
        return false;
    }
    partition.pending.clear();
    partition.pending.push(std::array::from_fn(|i| {
        ClipVertex::new(Vertex {
            p: vertices[i],
            n: normal,
            uv: uvs[i],
            c: color,
            crossing: Vector4::ZERO,
        })
    }));
    for clip in &partition.clips {
        partition.remaining.clear();
        for triangle in partition.pending.drain(..) {
            subtract_triangle(triangle, *clip, &mut partition.remaining);
        }
        std::mem::swap(&mut partition.pending, &mut partition.remaining);
        if partition.pending.is_empty() {
            break;
        }
    }
    for triangle in partition.pending.drain(..) {
        let triangle = triangle.map(ClipVertex::render);
        if (triangle[1].p - triangle[0].p)
            .cross(triangle[2].p - triangle[0].p)
            .length()
            * 0.5
            < crate::simulation::network::surface::NODE_OVERLAY_MIN_AREA_M2
        {
            continue;
        }
        // The partition scratch is taken above, so this re-enters the regular
        // asphalt/crosswalk sink without recursively applying another material mask.
        push_triangle_to_layer(
            mesh,
            layer,
            triangle.map(|v| v.p),
            triangle.map(|v| v.uv),
            color,
            normal,
        );
    }
    mesh.material_partition = partition;
    true
}

fn subtract_triangle(
    triangle: [ClipVertex; 3],
    clip: RoadMaterialClipTriangle,
    out: &mut Vec<[ClipVertex; 3]>,
) {
    let exterior_edges = clip.exterior_edges;
    let clip = clip.points.map(|p| glam::DVec2::new(p.x, p.z));
    let side = |i: usize, v: &ClipVertex| {
        (clip[(i + 1) % 3] - clip[i]).perp_dot(glam::DVec2::new(v.point.x, v.point.z) - clip[i])
    };
    // An exterior edge retains its real kerb; internal union diagonals remove
    // vertical faces too, even when their entire XZ projection lies on that edge.
    if (0..3).any(|i| {
        triangle.iter().all(|v| side(i, v) < 0.0)
            || (exterior_edges[i] && triangle.iter().all(|v| side(i, v) <= 0.0))
    }) {
        out.push(triangle);
        return;
    }
    let mut inner = ClipPolygon::empty();
    inner.len = 3;
    inner.vertices[..3].copy_from_slice(&triangle);
    for i in 0..3 {
        if inner.len < 3 {
            break;
        }
        if inner.vertices[..inner.len]
            .iter()
            .all(|v| side(i, v) >= 0.0)
        {
            continue;
        }
        let (outer, kept) = inner.split(|v| side(i, v));
        inner = kept;
        for j in 1..outer.len.saturating_sub(1) {
            out.push([outer.vertices[0], outer.vertices[j], outer.vertices[j + 1]]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::surface::RoadVec3;

    #[test]
    fn material_subtraction_preserves_source_height_uvs_and_vertical_boundary_faces() {
        let vertex = |x, z| {
            ClipVertex::new(Vertex {
                p: Vector3::new(x, 4.0 + x * 0.1 + z * 0.05, z),
                uv: Vector2::new(x, z),
                ..Vertex::default()
            })
        };
        let source = [vertex(0.0, 0.0), vertex(4.0, 0.0), vertex(0.0, 4.0)];
        let clip = RoadMaterialClipTriangle {
            points: [
                RoadVec3::new(0.0, 0.0, 0.0),
                RoadVec3::new(2.0, 0.0, 0.0),
                RoadVec3::new(0.0, 0.0, 2.0),
            ],
            exterior_edges: [true; 3],
        };
        let mut out = Vec::new();
        subtract_triangle(source, clip, &mut out);
        let area: f64 = out
            .iter()
            .map(|t| {
                (t[1].point - t[0].point)
                    .cross(t[2].point - t[0].point)
                    .y
                    .abs()
                    * 0.5
            })
            .sum();
        assert!((area - 6.0).abs() < 1e-5);
        for v in out.into_iter().flatten().map(ClipVertex::render) {
            assert_eq!(v.uv, Vector2::new(v.p.x, v.p.z));
            assert!((v.p.y - (4.0 + v.p.x * 0.1 + v.p.z * 0.05)).abs() < 1e-5);
        }
        let vertical = [
            vertex(0.0, 0.0),
            vertex(0.0, 2.0),
            ClipVertex {
                point: glam::DVec3::new(0.0, 5.0, 2.0),
                ..vertex(0.0, 2.0)
            },
        ];
        let mut out = Vec::new();
        subtract_triangle(vertical, clip, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].map(|v| v.point), vertical.map(|v| v.point));
        out.clear();
        subtract_triangle(
            vertical,
            RoadMaterialClipTriangle {
                exterior_edges: [false; 3],
                ..clip
            },
            &mut out,
        );
        assert!(
            out.is_empty(),
            "internal asphalt edges cannot retain a vertical wall"
        );
    }
}
