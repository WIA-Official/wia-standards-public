//! RViz2 Marker exporter for robot visualization

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use serde::{Deserialize, Serialize};

/// RViz2 marker types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MarkerType {
    Arrow = 0,
    Cube = 1,
    Sphere = 2,
    Cylinder = 3,
    LineStrip = 4,
    LineList = 5,
    CubeList = 6,
    SphereList = 7,
    Points = 8,
    TextViewFacing = 9,
    MeshResource = 10,
    TriangleList = 11,
}

/// RViz2 marker action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MarkerAction {
    Add = 0,
    Modify = 1,
    Delete = 2,
    DeleteAll = 3,
}

/// 3D position
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Quaternion orientation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

/// Pose (position + orientation)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point3D,
    pub orientation: Quaternion,
}

/// Scale (3D dimensions)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scale3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Scale3D {
    fn default() -> Self {
        Self {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        }
    }
}

/// RGBA color
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Default for ColorRGBA {
    fn default() -> Self {
        Self {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        }
    }
}

impl ColorRGBA {
    pub fn green() -> Self {
        Self { r: 0.0, g: 1.0, b: 0.0, a: 1.0 }
    }

    pub fn red() -> Self {
        Self { r: 1.0, g: 0.0, b: 0.0, a: 1.0 }
    }

    pub fn blue() -> Self {
        Self { r: 0.0, g: 0.0, b: 1.0, a: 1.0 }
    }

    pub fn yellow() -> Self {
        Self { r: 1.0, g: 1.0, b: 0.0, a: 1.0 }
    }

    pub fn orange() -> Self {
        Self { r: 1.0, g: 0.5, b: 0.0, a: 1.0 }
    }
}

/// RViz2 marker header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub frame_id: String,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
}

impl Default for Header {
    fn default() -> Self {
        Self {
            frame_id: "base_link".to_string(),
            stamp_sec: 0,
            stamp_nanosec: 0,
        }
    }
}

/// RViz2 Marker message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Marker {
    pub header: Header,
    pub ns: String,
    pub id: i32,
    pub marker_type: u8,
    pub action: u8,
    pub pose: Pose,
    pub scale: Scale3D,
    pub color: ColorRGBA,
    pub lifetime_sec: i32,
    pub lifetime_nanosec: u32,
    pub frame_locked: bool,
    pub points: Vec<Point3D>,
    pub colors: Vec<ColorRGBA>,
    pub text: String,
    pub mesh_resource: String,
    pub mesh_use_embedded_materials: bool,
}

impl Default for Marker {
    fn default() -> Self {
        Self {
            header: Header::default(),
            ns: "wia_robot".to_string(),
            id: 0,
            marker_type: MarkerType::Sphere as u8,
            action: MarkerAction::Add as u8,
            pose: Pose::default(),
            scale: Scale3D::default(),
            color: ColorRGBA::green(),
            lifetime_sec: 0,
            lifetime_nanosec: 0,
            frame_locked: false,
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh_resource: String::new(),
            mesh_use_embedded_materials: false,
        }
    }
}

impl Marker {
    /// Create a new marker
    pub fn new(marker_type: MarkerType, id: i32) -> Self {
        Self {
            marker_type: marker_type as u8,
            id,
            ..Default::default()
        }
    }

    /// Set namespace
    pub fn with_ns(mut self, ns: &str) -> Self {
        self.ns = ns.to_string();
        self
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        self.header.frame_id = frame_id.to_string();
        self
    }

    /// Set position
    pub fn with_position(mut self, x: f64, y: f64, z: f64) -> Self {
        self.pose.position = Point3D::new(x, y, z);
        self
    }

    /// Set scale
    pub fn with_scale(mut self, x: f64, y: f64, z: f64) -> Self {
        self.scale = Scale3D { x, y, z };
        self
    }

    /// Set uniform scale
    pub fn with_uniform_scale(mut self, scale: f64) -> Self {
        self.scale = Scale3D { x: scale, y: scale, z: scale };
        self
    }

    /// Set color
    pub fn with_color(mut self, color: ColorRGBA) -> Self {
        self.color = color;
        self
    }

    /// Set text (for TEXT_VIEW_FACING markers)
    pub fn with_text(mut self, text: &str) -> Self {
        self.text = text.to_string();
        self
    }

    /// Set lifetime
    pub fn with_lifetime(mut self, sec: i32, nanosec: u32) -> Self {
        self.lifetime_sec = sec;
        self.lifetime_nanosec = nanosec;
        self
    }

    /// Add point (for line/point markers)
    pub fn add_point(&mut self, point: Point3D) {
        self.points.push(point);
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// RViz2 MarkerArray message
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MarkerArray {
    pub markers: Vec<Marker>,
}

impl MarkerArray {
    /// Create a new marker array
    pub fn new() -> Self {
        Self { markers: Vec::new() }
    }

    /// Add a marker
    pub fn add(&mut self, marker: Marker) {
        self.markers.push(marker);
    }

    /// Get marker count
    pub fn len(&self) -> usize {
        self.markers.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.markers.is_empty()
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// RViz2 marker exporter
pub struct RVizMarkerExporter {
    base: BaseAdapter,
    namespace: String,
    frame_id: String,
    next_id: i32,
}

impl Default for RVizMarkerExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl RVizMarkerExporter {
    /// Create a new RViz marker exporter
    pub fn new() -> Self {
        Self {
            base: BaseAdapter::new("rviz", OutputType::Visualization),
            namespace: "wia_robot".to_string(),
            frame_id: "base_link".to_string(),
            next_id: 0,
        }
    }

    /// Set namespace
    pub fn with_namespace(mut self, namespace: &str) -> Self {
        self.namespace = namespace.to_string();
        self
    }

    /// Set frame ID
    pub fn with_frame_id(mut self, frame_id: &str) -> Self {
        self.frame_id = frame_id.to_string();
        self
    }

    /// Get next unique marker ID
    pub fn next_marker_id(&mut self) -> i32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Reset marker ID counter
    pub fn reset_ids(&mut self) {
        self.next_id = 0;
    }

    /// Create a joint position marker (sphere)
    pub fn joint_marker(&mut self, x: f64, y: f64, z: f64, color: ColorRGBA) -> Marker {
        Marker::new(MarkerType::Sphere, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_position(x, y, z)
            .with_uniform_scale(0.05)
            .with_color(color)
    }

    /// Create a trajectory line marker
    pub fn trajectory_marker(&mut self, points: Vec<Point3D>, color: ColorRGBA) -> Marker {
        let mut marker = Marker::new(MarkerType::LineStrip, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_scale(0.01, 0.0, 0.0)
            .with_color(color);
        marker.points = points;
        marker
    }

    /// Create a force arrow marker
    pub fn force_marker(&mut self, origin: Point3D, direction: Point3D, color: ColorRGBA) -> Marker {
        let mut marker = Marker::new(MarkerType::Arrow, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_scale(0.02, 0.04, 0.04)
            .with_color(color);
        marker.points = vec![origin, direction];
        marker
    }

    /// Create a text status marker
    pub fn text_marker(&mut self, text: &str, x: f64, y: f64, z: f64) -> Marker {
        Marker::new(MarkerType::TextViewFacing, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_position(x, y, z)
            .with_scale(0.0, 0.0, 0.1)
            .with_text(text)
            .with_color(ColorRGBA::default())
    }

    /// Create a workspace boundary marker
    pub fn workspace_marker(&mut self, points: Vec<Point3D>, color: ColorRGBA) -> Marker {
        let mut marker = Marker::new(MarkerType::LineList, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_scale(0.005, 0.0, 0.0)
            .with_color(color);
        marker.points = points;
        marker
    }

    /// Create a safety zone cylinder marker
    pub fn safety_zone_marker(&mut self, x: f64, y: f64, radius: f64, height: f64) -> Marker {
        let mut color = ColorRGBA::yellow();
        color.a = 0.3; // Semi-transparent

        Marker::new(MarkerType::Cylinder, self.next_marker_id())
            .with_ns(&self.namespace)
            .with_frame_id(&self.frame_id)
            .with_position(x, y, height / 2.0)
            .with_scale(radius * 2.0, radius * 2.0, height)
            .with_color(color)
    }

    /// Convert robot data to markers
    pub fn robot_to_markers(&mut self, data: &OutputData) -> RobotResult<MarkerArray> {
        let mut array = MarkerArray::new();

        // Try to extract joint positions from data
        if let Some(joints) = data.data.get("joints").and_then(|j| j.as_array()) {
            for joint in joints {
                let x = joint.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let y = joint.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let z = joint.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0);

                array.add(self.joint_marker(x, y, z, ColorRGBA::green()));
            }
        }

        // Add status text
        let status = data.data.get("status")
            .and_then(|s| s.as_str())
            .unwrap_or("unknown");
        array.add(self.text_marker(&format!("Status: {}", status), 0.0, 0.0, 1.5));

        Ok(array)
    }
}

impl OutputAdapter for RVizMarkerExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Visualization
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(ns) = config.get_string_option("namespace") {
            self.namespace = ns.to_string();
        }
        if let Some(frame) = config.get_string_option("frame_id") {
            self.frame_id = frame.to_string();
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        // Clone self to call robot_to_markers (which needs &mut self)
        let mut exporter = Self {
            base: self.base.clone(),
            namespace: self.namespace.clone(),
            frame_id: self.frame_id.clone(),
            next_id: 0,
        };

        let markers = exporter.robot_to_markers(data)?;
        let json = markers.to_json()?;

        Ok(OutputResult::success(&format!("Generated {} markers", markers.len()))
            .with_metadata(serde_json::json!({
                "marker_count": markers.len(),
                "format": "rviz_marker_array",
                "output": json
            })))
    }

    fn is_available(&self) -> bool {
        self.base.available
    }

    fn dispose(&mut self) -> RobotResult<()> {
        self.reset_ids();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_marker_creation() {
        let marker = Marker::new(MarkerType::Sphere, 1)
            .with_ns("test")
            .with_frame_id("world")
            .with_position(1.0, 2.0, 3.0)
            .with_uniform_scale(0.1)
            .with_color(ColorRGBA::green());

        assert_eq!(marker.id, 1);
        assert_eq!(marker.ns, "test");
        assert_eq!(marker.header.frame_id, "world");
        assert_eq!(marker.pose.position.x, 1.0);
    }

    #[test]
    fn test_marker_array() {
        let mut array = MarkerArray::new();
        array.add(Marker::new(MarkerType::Sphere, 0));
        array.add(Marker::new(MarkerType::Cube, 1));

        assert_eq!(array.len(), 2);
        assert!(!array.is_empty());
    }

    #[test]
    fn test_rviz_exporter() {
        let mut exporter = RVizMarkerExporter::new()
            .with_namespace("test_robot")
            .with_frame_id("world");

        let marker = exporter.joint_marker(0.0, 0.0, 1.0, ColorRGBA::green());
        assert_eq!(marker.ns, "test_robot");
        assert_eq!(marker.header.frame_id, "world");

        let marker2 = exporter.joint_marker(0.0, 0.0, 2.0, ColorRGBA::blue());
        assert_eq!(marker2.id, 1); // Auto-incremented
    }

    #[test]
    fn test_trajectory_marker() {
        let mut exporter = RVizMarkerExporter::new();
        let points = vec![
            Point3D::new(0.0, 0.0, 0.0),
            Point3D::new(1.0, 0.0, 0.0),
            Point3D::new(1.0, 1.0, 0.0),
        ];
        let marker = exporter.trajectory_marker(points, ColorRGBA::blue());

        assert_eq!(marker.marker_type, MarkerType::LineStrip as u8);
        assert_eq!(marker.points.len(), 3);
    }

    #[test]
    fn test_rviz_output_adapter() {
        let exporter = RVizMarkerExporter::new();
        assert_eq!(exporter.output_type(), OutputType::Visualization);
        assert_eq!(exporter.name(), "rviz");
        assert!(exporter.is_available());

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active",
                "joints": [
                    {"x": 0.0, "y": 0.0, "z": 0.5},
                    {"x": 0.0, "y": 0.0, "z": 1.0}
                ]
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);
    }
}
