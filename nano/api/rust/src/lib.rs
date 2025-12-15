//! # WIA Nano SDK
//!
//! Official Rust implementation of the Web of Industrial Artifacts (WIA) Nano Standard.
//!
//! This SDK provides comprehensive APIs for nanoscale systems including:
//! - Molecular assemblers
//! - Nanorobots
//! - Nanosensors
//! - Nano machines
//! - Molecular memory
//! - Nanomedicine delivery systems
//!
//! ## Features
//!
//! - **Type-safe data structures** - Strongly typed nanoscale data formats
//! - **Async-first** - All I/O operations are async
//! - **Simulator included** - Built-in simulation engine for testing
//! - **IEEE 1906.1 compatible** - Based on nanoscale communication standards
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use wia_nano::prelude::*;
//! use wia_nano::simulator::NanoSimulator;
//!
//! #[tokio::main]
//! async fn main() -> NanoResult<()> {
//!     // Create a nanorobot simulation
//!     let mut sim = NanoSimulator::new();
//!
//!     let robot = sim.create_nanorobot("robot-001")
//!         .with_position(Position3D::new(0.0, 0.0, 0.0))
//!         .with_environment(Environment::physiological())
//!         .build()?;
//!
//!     // Move the robot
//!     robot.move_to(Position3D::new(100.0, 50.0, 25.0)).await?;
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Modules
//!
//! - [`types`] - Core data types and structures
//! - [`traits`] - Behavioral traits for nano systems
//! - [`systems`] - Concrete implementations
//! - [`simulator`] - Simulation engine
//! - [`error`] - Error types

#![warn(missing_docs)]
#![warn(rustdoc::missing_doc_code_examples)]

pub mod error;
pub mod types;
pub mod traits;
pub mod systems;
pub mod simulator;
pub mod protocol;
pub mod integration;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::error::{NanoError, NanoResult};
    pub use crate::types::*;
    pub use crate::traits::*;
}

/// WIA Nano SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA Nano specification version
pub const SPEC_VERSION: &str = "1.0.0";

/// IEEE 1906.1 compliance level
pub const IEEE_COMPLIANCE: &str = "IEEE 1906.1-2015";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_spec_version() {
        assert_eq!(SPEC_VERSION, "1.0.0");
    }
}
