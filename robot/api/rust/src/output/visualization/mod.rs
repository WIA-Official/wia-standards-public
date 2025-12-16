//! Visualization adapters for ROS2 ecosystem
//!
//! This module provides exporters for robot visualization:
//!
//! - **RViz2**: Marker messages for 3D visualization
//! - **Gazebo**: SDF (Simulation Description Format) generation
//! - **URDF**: Universal Robot Description Format generation

pub mod rviz;
pub mod gazebo;
pub mod urdf;

pub use rviz::RVizMarkerExporter;
pub use gazebo::GazeboSdfExporter;
pub use urdf::UrdfGenerator;
