//! Spatial representation (공간)

use super::auditory::Vec3; // Reuse Vec3
use serde::{Deserialize, Serialize};

/// Spatial representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialRep {
    /// 3D position
    pub position: Vec3,

    /// Rotation (quaternion)
    pub rotation: Quat,

    /// Scale
    pub scale: Vec3,

    /// Bounding box
    pub bounds: BoundingBox,

    /// Spatial relations to other nodes
    pub spatial_relations: Vec<SpatialRelation>,
}

impl Default for SpatialRep {
    fn default() -> Self {
        Self {
            position: Vec3::zero(),
            rotation: Quat::identity(),
            scale: Vec3::new(1.0, 1.0, 1.0),
            bounds: BoundingBox::default(),
            spatial_relations: Vec::new(),
        }
    }
}

/// Quaternion for rotation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }
}

/// Axis-aligned bounding box
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self {
            min: Vec3::zero(),
            max: Vec3::new(1.0, 1.0, 1.0),
        }
    }
}

/// Spatial relation to another node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpatialRelation {
    pub relation_type: SpatialRelationType,
    pub target_node_id: String,
}

/// Types of spatial relations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SpatialRelationType {
    Above,
    Below,
    LeftOf,
    RightOf,
    Near,
    Far,
}

impl SpatialRep {
    /// Create a spatial representation at a position
    pub fn at_position(position: Vec3) -> Self {
        Self {
            position,
            ..Default::default()
        }
    }

    /// Set position
    pub fn with_position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }

    /// Set rotation
    pub fn with_rotation(mut self, rotation: Quat) -> Self {
        self.rotation = rotation;
        self
    }

    /// Set scale
    pub fn with_scale(mut self, scale: Vec3) -> Self {
        self.scale = scale;
        self
    }

    /// Add spatial relation
    pub fn add_relation(mut self, relation: SpatialRelation) -> Self {
        self.spatial_relations.push(relation);
        self
    }
}
