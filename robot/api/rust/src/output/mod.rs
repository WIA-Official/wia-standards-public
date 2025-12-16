//! Output integration layer for WIA Robot SDK
//!
//! This module provides adapters for integrating WIA Robot data with external systems:
//!
//! - **Visualization**: RViz2, Gazebo, URDF generation
//! - **Medical**: HL7 FHIR integration
//! - **AI/ML**: Dataset export for TensorFlow/PyTorch
//! - **Dashboard**: Real-time WebSocket streaming
//! - **Export**: JSON, CSV data export
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │           OutputManager             │
//! │         (Central Hub)               │
//! ├─────────────────────────────────────┤
//! │  Visualization │ Medical │ AI/ML   │
//! │  Dashboard     │ Export  │ Logger  │
//! └─────────────────────────────────────┘
//!           │
//!           ▼
//!   External Systems (ROS2, FHIR, etc.)
//! ```
//!
//! # Quick Start
//!
//! ```rust
//! use wia_robot::output::*;
//!
//! // Create output manager
//! let mut manager = OutputManager::new();
//!
//! // Register adapters
//! manager.register("json", Box::new(JsonExporter::new("./data")));
//! manager.register("csv", Box::new(CsvExporter::new("./data")));
//!
//! // Create output data
//! let data = OutputData::new("exo-001", "exoskeleton")
//!     .with_data(serde_json::json!({"status": "ok"}));
//!
//! // Export to specific adapter
//! let result = manager.output_to("json", &data).unwrap();
//! assert!(result.success);
//! ```

pub mod adapter;
pub mod manager;
pub mod visualization;
pub mod medical;
pub mod aiml;
pub mod dashboard;
pub mod export;

pub use adapter::*;
pub use manager::*;
pub use visualization::{RVizMarkerExporter, GazeboSdfExporter, UrdfGenerator};
pub use medical::FhirExporter;
pub use aiml::{DatasetExporter, TrainingDatapoint, DatasetMetadata};
pub use dashboard::DashboardAdapter;
pub use export::{JsonExporter, CsvExporter};
