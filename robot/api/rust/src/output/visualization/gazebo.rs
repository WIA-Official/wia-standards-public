//! Gazebo SDF (Simulation Description Format) exporter

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;

/// Gazebo SDF exporter for robot simulation
pub struct GazeboSdfExporter {
    base: BaseAdapter,
    world_name: String,
    sdf_version: String,
}

impl Default for GazeboSdfExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl GazeboSdfExporter {
    /// Create a new Gazebo SDF exporter
    pub fn new() -> Self {
        Self {
            base: BaseAdapter::new("gazebo", OutputType::Visualization),
            world_name: "wia_robot_world".to_string(),
            sdf_version: "1.7".to_string(),
        }
    }

    /// Set world name
    pub fn with_world_name(mut self, name: &str) -> Self {
        self.world_name = name.to_string();
        self
    }

    /// Set SDF version
    pub fn with_sdf_version(mut self, version: &str) -> Self {
        self.sdf_version = version.to_string();
        self
    }

    /// Generate SDF link element
    fn generate_link(&self, name: &str, pose: &str, size: &str, color: &str) -> String {
        format!(
            r#"      <link name="{}">
        <pose>{}</pose>
        <collision name="{}_collision">
          <geometry>
            <box>
              <size>{}</size>
            </box>
          </geometry>
        </collision>
        <visual name="{}_visual">
          <geometry>
            <box>
              <size>{}</size>
            </box>
          </geometry>
          <material>
            <ambient>{}</ambient>
            <diffuse>{}</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>"#,
            name, pose, name, size, name, size, color, color
        )
    }

    /// Generate SDF joint element
    fn generate_joint(&self, name: &str, joint_type: &str, parent: &str, child: &str, axis: &str, limits: (f64, f64)) -> String {
        format!(
            r#"      <joint name="{}" type="{}">
        <parent>{}</parent>
        <child>{}</child>
        <axis>
          <xyz>{}</xyz>
          <limit>
            <lower>{}</lower>
            <upper>{}</upper>
            <effort>100</effort>
            <velocity>1.0</velocity>
          </limit>
        </axis>
      </joint>"#,
            name, joint_type, parent, child, axis, limits.0, limits.1
        )
    }

    /// Generate basic exoskeleton SDF
    pub fn generate_exoskeleton_sdf(&self, model_name: &str) -> String {
        let base_link = self.generate_link("base_link", "0 0 0.5 0 0 0", "0.3 0.2 0.1", "0 0 1 1");
        let thigh_left = self.generate_link("thigh_left", "0.1 0.1 0.3 0 0 0", "0.08 0.08 0.4", "0 0.5 1 1");
        let thigh_right = self.generate_link("thigh_right", "-0.1 0.1 0.3 0 0 0", "0.08 0.08 0.4", "0 0.5 1 1");
        let shin_left = self.generate_link("shin_left", "0.1 0.1 -0.1 0 0 0", "0.06 0.06 0.35", "0 0.7 1 1");
        let shin_right = self.generate_link("shin_right", "-0.1 0.1 -0.1 0 0 0", "0.06 0.06 0.35", "0 0.7 1 1");

        let hip_left = self.generate_joint("hip_left_joint", "revolute", "base_link", "thigh_left", "1 0 0", (-1.57, 1.57));
        let hip_right = self.generate_joint("hip_right_joint", "revolute", "base_link", "thigh_right", "1 0 0", (-1.57, 1.57));
        let knee_left = self.generate_joint("knee_left_joint", "revolute", "thigh_left", "shin_left", "1 0 0", (0.0, 2.5));
        let knee_right = self.generate_joint("knee_right_joint", "revolute", "thigh_right", "shin_right", "1 0 0", (0.0, 2.5));

        format!(
            r#"<?xml version="1.0" ?>
<sdf version="{}">
  <world name="{}">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="{}">
      <static>false</static>
{}
{}
{}
{}
{}
{}
{}
{}
{}
    </model>
  </world>
</sdf>"#,
            self.sdf_version,
            self.world_name,
            model_name,
            base_link,
            thigh_left,
            thigh_right,
            shin_left,
            shin_right,
            hip_left,
            hip_right,
            knee_left,
            knee_right
        )
    }

    /// Generate prosthetic hand SDF
    pub fn generate_prosthetic_hand_sdf(&self, model_name: &str, side: &str) -> String {
        let palm = self.generate_link("palm", "0 0 0 0 0 0", "0.08 0.1 0.02", "0.8 0.6 0.5 1");

        let finger_names = ["thumb", "index", "middle", "ring", "pinky"];
        let finger_positions = [
            ("-0.03 -0.04 0.02", "0.4 0.3 0.2 1"),
            ("0.02 -0.05 0.01", "0.8 0.6 0.5 1"),
            ("0.00 -0.05 0.01", "0.8 0.6 0.5 1"),
            ("-0.02 -0.05 0.01", "0.8 0.6 0.5 1"),
            ("-0.04 -0.04 0.01", "0.8 0.6 0.5 1"),
        ];

        let mut links = String::new();
        let mut joints = String::new();

        for (i, (name, (pose, color))) in finger_names.iter().zip(finger_positions.iter()).enumerate() {
            links.push_str(&self.generate_link(name, pose, "0.015 0.05 0.015", color));
            links.push('\n');

            let axis = if i == 0 { "0 1 0" } else { "1 0 0" };
            joints.push_str(&self.generate_joint(
                &format!("{}_joint", name),
                "revolute",
                "palm",
                name,
                axis,
                (0.0, 1.57),
            ));
            joints.push('\n');
        }

        format!(
            r#"<?xml version="1.0" ?>
<sdf version="{}">
  <world name="{}">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="{}_{}">
      <static>false</static>
{}
{}
{}
    </model>
  </world>
</sdf>"#,
            self.sdf_version,
            self.world_name,
            model_name,
            side,
            palm,
            links,
            joints
        )
    }

    /// Generate SDF from robot data
    pub fn generate_sdf(&self, data: &OutputData) -> RobotResult<String> {
        let model_name = data.device_id.replace('-', "_");

        let sdf = match data.robot_type.as_str() {
            "exoskeleton" => self.generate_exoskeleton_sdf(&model_name),
            "prosthetic" => {
                let side = data.data.get("side")
                    .and_then(|s| s.as_str())
                    .unwrap_or("right");
                self.generate_prosthetic_hand_sdf(&model_name, side)
            }
            _ => {
                // Generic robot
                let link = self.generate_link("base_link", "0 0 0.1 0 0 0", "0.2 0.2 0.2", "0 0 1 1");
                format!(
                    r#"<?xml version="1.0" ?>
<sdf version="{}">
  <world name="{}">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="{}">
      <static>false</static>
{}
    </model>
  </world>
</sdf>"#,
                    self.sdf_version,
                    self.world_name,
                    model_name,
                    link
                )
            }
        };

        Ok(sdf)
    }
}

impl OutputAdapter for GazeboSdfExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Visualization
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(world) = config.get_string_option("world_name") {
            self.world_name = world.to_string();
        }
        if let Some(version) = config.get_string_option("sdf_version") {
            self.sdf_version = version.to_string();
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let sdf = self.generate_sdf(data)?;

        Ok(OutputResult::success("Generated Gazebo SDF")
            .with_metadata(serde_json::json!({
                "format": "sdf",
                "version": self.sdf_version,
                "world_name": self.world_name,
                "output": sdf
            })))
    }

    fn is_available(&self) -> bool {
        self.base.available
    }

    fn dispose(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gazebo_sdf_exporter_creation() {
        let exporter = GazeboSdfExporter::new()
            .with_world_name("test_world")
            .with_sdf_version("1.8");

        assert_eq!(exporter.world_name, "test_world");
        assert_eq!(exporter.sdf_version, "1.8");
    }

    #[test]
    fn test_generate_exoskeleton_sdf() {
        let exporter = GazeboSdfExporter::new();
        let sdf = exporter.generate_exoskeleton_sdf("test_exo");

        assert!(sdf.contains("<?xml version=\"1.0\" ?>"));
        assert!(sdf.contains("<sdf version=\"1.7\">"));
        assert!(sdf.contains("test_exo"));
        assert!(sdf.contains("base_link"));
        assert!(sdf.contains("hip_left_joint"));
    }

    #[test]
    fn test_generate_prosthetic_hand_sdf() {
        let exporter = GazeboSdfExporter::new();
        let sdf = exporter.generate_prosthetic_hand_sdf("hand", "right");

        assert!(sdf.contains("palm"));
        assert!(sdf.contains("thumb"));
        assert!(sdf.contains("index"));
        assert!(sdf.contains("hand_right"));
    }

    #[test]
    fn test_sdf_output_adapter() {
        let exporter = GazeboSdfExporter::new();

        assert_eq!(exporter.output_type(), OutputType::Visualization);
        assert_eq!(exporter.name(), "gazebo");

        let data = OutputData::new("exo-001", "exoskeleton");
        let result = exporter.output(&data).unwrap();

        assert!(result.success);
        assert!(result.metadata.is_some());
    }
}
