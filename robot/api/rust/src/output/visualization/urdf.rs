//! URDF (Universal Robot Description Format) generator

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;

/// URDF generator for ROS robot descriptions
pub struct UrdfGenerator {
    base: BaseAdapter,
    robot_name: String,
}

impl Default for UrdfGenerator {
    fn default() -> Self {
        Self::new()
    }
}

impl UrdfGenerator {
    /// Create a new URDF generator
    pub fn new() -> Self {
        Self {
            base: BaseAdapter::new("urdf", OutputType::Export),
            robot_name: "wia_robot".to_string(),
        }
    }

    /// Set robot name
    pub fn with_robot_name(mut self, name: &str) -> Self {
        self.robot_name = name.to_string();
        self
    }

    /// Generate material element
    fn generate_material(&self, name: &str, rgba: &str) -> String {
        format!(
            r#"  <material name="{}">
    <color rgba="{}"/>
  </material>"#,
            name, rgba
        )
    }

    /// Generate link element
    fn generate_link(
        &self,
        name: &str,
        size: &str,
        material: &str,
        mass: f64,
        origin: Option<&str>,
    ) -> String {
        let origin_str = origin
            .map(|o| format!("\n      <origin xyz=\"{}\" rpy=\"0 0 0\"/>", o))
            .unwrap_or_default();

        format!(
            r#"  <link name="{}">
    <visual>{}
      <geometry>
        <box size="{}"/>
      </geometry>
      <material name="{}"/>
    </visual>
    <collision>{}
      <geometry>
        <box size="{}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="{}"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>"#,
            name, origin_str, size, material, origin_str, size, mass
        )
    }

    /// Generate joint element
    fn generate_joint(
        &self,
        name: &str,
        joint_type: &str,
        parent: &str,
        child: &str,
        origin: &str,
        axis: &str,
        limits: Option<(f64, f64, f64, f64)>,
    ) -> String {
        let limits_str = limits
            .map(|(lower, upper, effort, velocity)| {
                format!(
                    "\n    <limit lower=\"{}\" upper=\"{}\" effort=\"{}\" velocity=\"{}\"/>",
                    lower, upper, effort, velocity
                )
            })
            .unwrap_or_default();

        format!(
            r#"  <joint name="{}" type="{}">
    <parent link="{}"/>
    <child link="{}"/>
    <origin xyz="{}" rpy="0 0 0"/>
    <axis xyz="{}"/>{}
  </joint>"#,
            name, joint_type, parent, child, origin, axis, limits_str
        )
    }

    /// Generate exoskeleton URDF
    pub fn generate_exoskeleton_urdf(&self, name: &str) -> String {
        let materials = [
            self.generate_material("blue", "0 0 1 1"),
            self.generate_material("light_blue", "0.3 0.5 1 1"),
            self.generate_material("dark_blue", "0 0 0.7 1"),
        ]
        .join("\n");

        let links = [
            self.generate_link("base_link", "0.3 0.2 0.1", "blue", 2.0, None),
            self.generate_link("thigh_left", "0.08 0.08 0.4", "light_blue", 1.5, Some("0 0 -0.2")),
            self.generate_link("thigh_right", "0.08 0.08 0.4", "light_blue", 1.5, Some("0 0 -0.2")),
            self.generate_link("shin_left", "0.06 0.06 0.35", "dark_blue", 1.0, Some("0 0 -0.175")),
            self.generate_link("shin_right", "0.06 0.06 0.35", "dark_blue", 1.0, Some("0 0 -0.175")),
            self.generate_link("foot_left", "0.1 0.06 0.02", "blue", 0.5, None),
            self.generate_link("foot_right", "0.1 0.06 0.02", "blue", 0.5, None),
        ]
        .join("\n\n");

        let joints = [
            self.generate_joint(
                "hip_left_joint",
                "revolute",
                "base_link",
                "thigh_left",
                "0.1 0 -0.05",
                "1 0 0",
                Some((-1.57, 1.57, 100.0, 1.0)),
            ),
            self.generate_joint(
                "hip_right_joint",
                "revolute",
                "base_link",
                "thigh_right",
                "-0.1 0 -0.05",
                "1 0 0",
                Some((-1.57, 1.57, 100.0, 1.0)),
            ),
            self.generate_joint(
                "knee_left_joint",
                "revolute",
                "thigh_left",
                "shin_left",
                "0 0 -0.4",
                "1 0 0",
                Some((0.0, 2.5, 80.0, 1.0)),
            ),
            self.generate_joint(
                "knee_right_joint",
                "revolute",
                "thigh_right",
                "shin_right",
                "0 0 -0.4",
                "1 0 0",
                Some((0.0, 2.5, 80.0, 1.0)),
            ),
            self.generate_joint(
                "ankle_left_joint",
                "revolute",
                "shin_left",
                "foot_left",
                "0 0 -0.35",
                "1 0 0",
                Some((-0.5, 0.5, 50.0, 1.0)),
            ),
            self.generate_joint(
                "ankle_right_joint",
                "revolute",
                "shin_right",
                "foot_right",
                "0 0 -0.35",
                "1 0 0",
                Some((-0.5, 0.5, 50.0, 1.0)),
            ),
        ]
        .join("\n\n");

        format!(
            r#"<?xml version="1.0"?>
<robot name="{}">
  <!-- Materials -->
{}

  <!-- Links -->
{}

  <!-- Joints -->
{}
</robot>"#,
            name, materials, links, joints
        )
    }

    /// Generate prosthetic hand URDF
    pub fn generate_prosthetic_hand_urdf(&self, name: &str) -> String {
        let materials = [
            self.generate_material("skin", "0.9 0.75 0.65 1"),
            self.generate_material("metal", "0.7 0.7 0.7 1"),
        ]
        .join("\n");

        let palm = self.generate_link("palm", "0.08 0.1 0.02", "metal", 0.5, None);

        let finger_names = ["thumb", "index", "middle", "ring", "pinky"];
        let finger_offsets = [
            ("-0.03", "-0.04", "0.02"),
            ("0.02", "-0.055", "0.01"),
            ("0.00", "-0.055", "0.01"),
            ("-0.02", "-0.055", "0.01"),
            ("-0.04", "-0.04", "0.01"),
        ];

        let mut finger_links = String::new();
        let mut finger_joints = String::new();

        for (i, (name, (x, y, z))) in finger_names.iter().zip(finger_offsets.iter()).enumerate() {
            // Proximal phalanx
            finger_links.push_str(&self.generate_link(
                &format!("{}_proximal", name),
                "0.015 0.03 0.015",
                "skin",
                0.02,
                Some("0 -0.015 0"),
            ));
            finger_links.push('\n');

            // Distal phalanx
            finger_links.push_str(&self.generate_link(
                &format!("{}_distal", name),
                "0.012 0.02 0.012",
                "skin",
                0.01,
                Some("0 -0.01 0"),
            ));
            finger_links.push('\n');

            // MCP joint (palm to proximal)
            let axis = if i == 0 { "0 1 0" } else { "1 0 0" };
            finger_joints.push_str(&self.generate_joint(
                &format!("{}_mcp_joint", name),
                "revolute",
                "palm",
                &format!("{}_proximal", name),
                &format!("{} {} {}", x, y, z),
                axis,
                Some((0.0, 1.57, 10.0, 2.0)),
            ));
            finger_joints.push('\n');

            // PIP joint (proximal to distal)
            finger_joints.push_str(&self.generate_joint(
                &format!("{}_pip_joint", name),
                "revolute",
                &format!("{}_proximal", name),
                &format!("{}_distal", name),
                "0 -0.03 0",
                "1 0 0",
                Some((0.0, 1.57, 5.0, 2.0)),
            ));
            finger_joints.push('\n');
        }

        format!(
            r#"<?xml version="1.0"?>
<robot name="{}">
  <!-- Materials -->
{}

  <!-- Palm -->
{}

  <!-- Fingers -->
{}

  <!-- Finger Joints -->
{}
</robot>"#,
            name, materials, palm, finger_links, finger_joints
        )
    }

    /// Generate wheelchair URDF
    pub fn generate_wheelchair_urdf(&self, name: &str) -> String {
        let materials = [
            self.generate_material("black", "0.1 0.1 0.1 1"),
            self.generate_material("silver", "0.8 0.8 0.8 1"),
            self.generate_material("rubber", "0.2 0.2 0.2 1"),
        ]
        .join("\n");

        let links = [
            self.generate_link("base_link", "0.5 0.6 0.1", "black", 15.0, None),
            self.generate_link("seat", "0.45 0.45 0.05", "black", 2.0, None),
            self.generate_link("backrest", "0.45 0.05 0.5", "black", 3.0, None),
            self.generate_link("wheel_left", "0.02 0.5 0.5", "rubber", 2.0, None),
            self.generate_link("wheel_right", "0.02 0.5 0.5", "rubber", 2.0, None),
            self.generate_link("caster_left", "0.02 0.15 0.15", "rubber", 0.5, None),
            self.generate_link("caster_right", "0.02 0.15 0.15", "rubber", 0.5, None),
        ]
        .join("\n\n");

        let joints = [
            self.generate_joint(
                "seat_joint",
                "fixed",
                "base_link",
                "seat",
                "0 0 0.1",
                "0 0 1",
                None,
            ),
            self.generate_joint(
                "backrest_joint",
                "fixed",
                "seat",
                "backrest",
                "0 0.225 0.275",
                "0 0 1",
                None,
            ),
            self.generate_joint(
                "wheel_left_joint",
                "continuous",
                "base_link",
                "wheel_left",
                "0.35 0 -0.05",
                "0 1 0",
                None,
            ),
            self.generate_joint(
                "wheel_right_joint",
                "continuous",
                "base_link",
                "wheel_right",
                "-0.35 0 -0.05",
                "0 1 0",
                None,
            ),
            self.generate_joint(
                "caster_left_joint",
                "continuous",
                "base_link",
                "caster_left",
                "0.2 -0.2 -0.1",
                "0 0 1",
                None,
            ),
            self.generate_joint(
                "caster_right_joint",
                "continuous",
                "base_link",
                "caster_right",
                "-0.2 -0.2 -0.1",
                "0 0 1",
                None,
            ),
        ]
        .join("\n\n");

        format!(
            r#"<?xml version="1.0"?>
<robot name="{}">
  <!-- Materials -->
{}

  <!-- Links -->
{}

  <!-- Joints -->
{}
</robot>"#,
            name, materials, links, joints
        )
    }

    /// Generate URDF from robot data
    pub fn generate_urdf(&self, data: &OutputData) -> RobotResult<String> {
        let name = data.device_id.replace('-', "_");

        let urdf = match data.robot_type.as_str() {
            "exoskeleton" => self.generate_exoskeleton_urdf(&name),
            "prosthetic" => self.generate_prosthetic_hand_urdf(&name),
            "wheelchair" | "mobility" => self.generate_wheelchair_urdf(&name),
            _ => {
                // Generic robot with base link
                let material = self.generate_material("gray", "0.5 0.5 0.5 1");
                let link = self.generate_link("base_link", "0.2 0.2 0.2", "gray", 1.0, None);
                format!(
                    r#"<?xml version="1.0"?>
<robot name="{}">
{}
{}
</robot>"#,
                    name, material, link
                )
            }
        };

        Ok(urdf)
    }
}

impl OutputAdapter for UrdfGenerator {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(name) = config.get_string_option("robot_name") {
            self.robot_name = name.to_string();
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let urdf = self.generate_urdf(data)?;

        Ok(OutputResult::success("Generated URDF")
            .with_metadata(serde_json::json!({
                "format": "urdf",
                "robot_type": data.robot_type,
                "output": urdf
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
    fn test_urdf_generator_creation() {
        let gen = UrdfGenerator::new().with_robot_name("test_robot");
        assert_eq!(gen.robot_name, "test_robot");
    }

    #[test]
    fn test_generate_exoskeleton_urdf() {
        let gen = UrdfGenerator::new();
        let urdf = gen.generate_exoskeleton_urdf("test_exo");

        assert!(urdf.contains("<?xml version=\"1.0\"?>"));
        assert!(urdf.contains("<robot name=\"test_exo\">"));
        assert!(urdf.contains("base_link"));
        assert!(urdf.contains("hip_left_joint"));
        assert!(urdf.contains("revolute"));
    }

    #[test]
    fn test_generate_prosthetic_hand_urdf() {
        let gen = UrdfGenerator::new();
        let urdf = gen.generate_prosthetic_hand_urdf("hand_test");

        assert!(urdf.contains("palm"));
        assert!(urdf.contains("thumb_proximal"));
        assert!(urdf.contains("index_mcp_joint"));
    }

    #[test]
    fn test_generate_wheelchair_urdf() {
        let gen = UrdfGenerator::new();
        let urdf = gen.generate_wheelchair_urdf("wheelchair_test");

        assert!(urdf.contains("seat"));
        assert!(urdf.contains("wheel_left"));
        assert!(urdf.contains("continuous"));
    }

    #[test]
    fn test_urdf_output_adapter() {
        let gen = UrdfGenerator::new();

        assert_eq!(gen.output_type(), OutputType::Export);
        assert_eq!(gen.name(), "urdf");

        let data = OutputData::new("exo-001", "exoskeleton");
        let result = gen.output(&data).unwrap();

        assert!(result.success);
        assert!(result.metadata.is_some());
    }
}
