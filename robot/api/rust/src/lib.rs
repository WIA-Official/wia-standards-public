//! WIA Robot SDK
//!
//! A Rust library for the WIA Robot Accessibility Standard, providing
//! unified APIs for controlling assistive robots including exoskeletons,
//! prosthetics, rehabilitation robots, care robots, surgical assistants,
//! and mobility aids.
//!
//! # Features
//!
//! - **Type-safe robot control**: Strongly typed interfaces for each robot type
//! - **Safety first**: Built-in safety validation and emergency stop
//! - **Real-time ready**: Designed for low-latency control loops
//! - **ROS compatible**: Data structures compatible with ROS/ROS2
//!
//! # Quick Start
//!
//! ```rust
//! use wia_robot::prelude::*;
//!
//! // Create an exoskeleton
//! let mut exo = ExoskeletonSpec::new_lower_body();
//! exo.assist_level = 0.75;
//!
//! // Calculate walking speed
//! exo.gait.cadence_steps_min = 60.0;
//! exo.gait.stride_length_cm = 65.0;
//! let speed = exo.walking_speed_m_s();
//! println!("Walking speed: {:.2} m/s", speed);
//!
//! // Check for fall risk
//! if exo.detect_fall_risk() {
//!     println!("Warning: Fall risk detected!");
//! }
//! ```
//!
//! # Robot Types
//!
//! ## Exoskeleton
//! ```rust
//! use wia_robot::adapters::exoskeleton::*;
//!
//! let exo = ExoskeletonSpec::new_lower_body();
//! ```
//!
//! ## Prosthetics
//! ```rust
//! use wia_robot::adapters::prosthetics::*;
//! use wia_robot::types::Side;
//!
//! let hand = ProstheticSpec::new_hand(Side::Right);
//! ```
//!
//! ## Rehabilitation Robot
//! ```rust
//! use wia_robot::adapters::rehabilitation::*;
//!
//! let rehab = RehabilitationSpec::new_upper_limb();
//! ```
//!
//! ## Care Robot
//! ```rust
//! use wia_robot::adapters::care::*;
//!
//! let care = CareRobotSpec::new_elderly_companion();
//! ```
//!
//! ## Surgical Robot
//! ```rust
//! use wia_robot::adapters::surgical::*;
//!
//! let surgical = SurgicalSpec::new_minimally_invasive();
//! ```
//!
//! ## Mobility Aid
//! ```rust
//! use wia_robot::adapters::mobility::*;
//!
//! let wheelchair = MobilityAidSpec::new_wheelchair();
//! ```
//!
//! # Safety
//!
//! All robot operations should check safety status:
//!
//! ```rust
//! use wia_robot::safety::*;
//!
//! let mut safety = SafetyStatus::new_safe();
//!
//! // Validate before operation
//! if let Err(e) = safety.validate() {
//!     println!("Safety error: {}", e);
//! }
//!
//! // Trigger emergency stop if needed
//! safety.trigger_estop(EStopSource::UserButton);
//! ```
//!
//! # Protocol
//!
//! WIA Robot Protocol (WRP) for communication:
//!
//! ```rust
//! use wia_robot::protocol::*;
//!
//! // Create a telemetry message
//! let msg = MessageBuilder::telemetry()
//!     .from_device("exo-001", "exoskeleton")
//!     .to_device("server-001", "server")
//!     .payload(serde_json::json!({"status": "ok"}))
//!     .build();
//!
//! // Create emergency stop message
//! let estop = MessageBuilder::emergency_stop()
//!     .from_device("exo-001", "exoskeleton")
//!     .build();
//! ```
//!
//! # Output Integration
//!
//! Export robot data to external systems:
//!
//! ```rust
//! use wia_robot::output::*;
//!
//! // Create output manager
//! let mut manager = OutputManager::new();
//!
//! // Register exporters
//! manager.register("json", Box::new(JsonExporter::new("./data")));
//! manager.register("csv", Box::new(CsvExporter::new("./data")));
//!
//! // Create output data
//! let data = OutputData::new("exo-001", "exoskeleton")
//!     .with_data(serde_json::json!({"status": "active"}));
//!
//! // Export to specific adapter
//! let result = manager.output_to("json", &data).unwrap();
//! ```

pub mod error;
pub mod safety;
pub mod types;
pub mod adapters;
pub mod core;
pub mod protocol;
pub mod transport;
pub mod output;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::error::{RobotError, RobotResult};
    pub use crate::safety::{SafetyStatus, SafetyConstraints, EStopSource, WorkspaceLimits};
    pub use crate::types::*;
    pub use crate::adapters::*;
    pub use crate::core::*;
    pub use crate::protocol::{WrpMessage, MessageType, Priority, MessageBuilder, Endpoint};
    pub use crate::transport::{Transport, TransportConfig, MockTransport};
    pub use crate::output::{
        OutputAdapter, OutputType, OutputConfig, OutputData, OutputResult,
        OutputManager, JsonExporter, CsvExporter, RVizMarkerExporter,
        GazeboSdfExporter, UrdfGenerator, FhirExporter, DatasetExporter,
        DashboardAdapter,
    };
}

/// Crate version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::prelude::*;

    #[test]
    fn test_exoskeleton_workflow() {
        let mut exo = ExoskeletonSpec::new_lower_body();
        exo.assist_level = 0.75;
        exo.gait.cadence_steps_min = 60.0;
        exo.gait.stride_length_cm = 65.0;

        let speed = exo.walking_speed_m_s();
        assert!(speed > 0.0);
        assert!(!exo.detect_fall_risk());
    }

    #[test]
    fn test_prosthetic_workflow() {
        let mut hand = ProstheticSpec::new_hand(Side::Right);

        assert!(hand.control_finger("thumb", 0.5).is_ok());
        assert!(hand.set_grip_force(30.0).is_ok());

        let intent = hand.classify_intent().unwrap();
        assert_eq!(intent, GripIntent::Rest);
    }

    #[test]
    fn test_rehabilitation_workflow() {
        let mut rehab = RehabilitationSpec::new_upper_limb();
        rehab.exercise.repetition = 5;
        rehab.exercise.total_repetitions = 10;

        let progress = rehab.exercise_progress();
        assert!((progress - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_care_robot_workflow() {
        let mut care = CareRobotSpec::new_elderly_companion();

        care.vital_signs.heart_rate_bpm = 72;
        care.vital_signs.body_temp_c = 36.5;

        assert!(care.vital_signs.is_normal());
        assert!(!care.needs_emergency_response());
    }

    #[test]
    fn test_surgical_workflow() {
        let mut surgical = SurgicalSpec::new_minimally_invasive();

        assert!(surgical.connect_console("console-main").is_ok());
        assert!(surgical.teleoperation.connected);

        let pos = Position3D::new(50.0, 50.0, 50.0);
        assert!(surgical.move_instrument(1, pos).is_ok());
    }

    #[test]
    fn test_mobility_workflow() {
        let mut wheelchair = MobilityAidSpec::new_wheelchair();

        wheelchair.set_destination(10.0, 5.0, Some("Kitchen".to_string()));
        assert!(wheelchair.navigation.destination.is_some());

        assert!(wheelchair.set_velocity(0.5, 0.1).is_ok());
    }

    #[test]
    fn test_safety_integration() {
        let mut safety = SafetyStatus::new_safe();
        assert!(safety.is_safe());
        assert!(safety.validate().is_ok());

        safety.trigger_estop(EStopSource::UserButton);
        assert!(!safety.is_safe());
        assert!(safety.validate().is_err());
    }

    #[test]
    fn test_device_management() {
        let mut registry = DeviceRegistry::new();

        let exo = RobotDevice::new(
            "exo-001",
            RobotType::Exoskeleton,
            "Test Exo",
            "WIA",
            "Model 1",
        );
        registry.register(exo);

        assert_eq!(registry.count(), 1);
        assert!(registry.get("exo-001").is_some());
    }

    #[test]
    fn test_control_systems() {
        let mut pid = PIDController::new(1.0, 0.1, 0.05);
        let output = pid.compute(1.0, 0.01);
        assert!(output != 0.0);

        let traj = TrajectoryGenerator::new(0.0, 100.0, 1.0);
        assert!((traj.position_at(0.5) - 50.0).abs() < 1e-10);
    }

    #[test]
    fn test_output_manager() {
        let mut manager = OutputManager::new();
        manager.register("json", Box::new(JsonExporter::new("./test")));
        manager.register("csv", Box::new(CsvExporter::new("./test")));

        assert_eq!(manager.count(), 2);
        assert!(manager.contains("json"));
        assert!(manager.contains("csv"));

        let adapters = manager.get_by_type(OutputType::Export);
        assert_eq!(adapters.len(), 2);
    }

    #[test]
    fn test_output_data_flow() {
        let mut manager = OutputManager::new();
        manager.register("json", Box::new(JsonExporter::new("./test")));

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active",
                "assist_level": 0.75
            }));

        let result = manager.output_to("json", &data).unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_visualization_output() {
        let mut exporter = RVizMarkerExporter::new()
            .with_namespace("test_robot");

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active",
                "joints": [
                    {"x": 0.0, "y": 0.0, "z": 0.5}
                ]
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_medical_output() {
        let exporter = FhirExporter::new().with_patient("patient-001");

        let data = OutputData::new("rehab-001", "rehabilitation")
            .with_data(serde_json::json!({
                "rom_deg": 85.5,
                "joint": "knee"
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_aiml_output() {
        let exporter = DatasetExporter::new("./test_data");

        let data = OutputData::new("prosthetic-001", "prosthetic")
            .with_data(serde_json::json!({
                "label": "grip",
                "emg_sensors": [
                    {"signal_mv": 0.5, "activation_level": 0.8}
                ]
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_dashboard_output() {
        let adapter = DashboardAdapter::new();

        let data = OutputData::new("care-001", "care_robot")
            .with_data(serde_json::json!({
                "status": "monitoring",
                "battery_percent": 85
            }));

        let result = adapter.output(&data).unwrap();
        assert!(result.success);
    }
}
