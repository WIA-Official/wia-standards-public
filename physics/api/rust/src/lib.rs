//! # WIA Physics Standard - Rust SDK
//!
//! A comprehensive Rust library for physics and energy research data.
//!
//! ## Overview
//!
//! The WIA Physics Standard provides data types and utilities for:
//!
//! - **Nuclear Fusion**: Plasma parameters, magnetic confinement, energy balance
//! - **Time Crystals**: Oscillation dynamics, quantum properties
//! - **Particle Physics**: Collision events, cross-sections, particle properties
//! - **Dark Matter**: Detection events, exclusion limits, axion searches
//! - **Antimatter**: Antiparticle properties, traps, spectroscopy
//! - **Quantum Gravity**: Theoretical predictions, black hole physics
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_physics::prelude::*;
//!
//! // Create a measurement with uncertainty
//! let temperature = Measurement::new(150_000_000.0, 5_000_000.0, "K");
//!
//! // Build fusion data
//! let fusion_data = FusionDataBuilder::new()
//!     .experiment("ITER")
//!     .plasma_simple(150e6, "K", 1e20, "m^-3")
//!     .tokamak(5.3, 6.2, 2.0)
//!     .build()
//!     .unwrap();
//!
//! // Serialize to JSON
//! let json = serde_json::to_string_pretty(&fusion_data).unwrap();
//! ```
//!
//! ## Modules
//!
//! - [`types`] - Data type definitions matching JSON schemas
//! - [`core`] - Physics calculations and utilities
//! - [`adapters`] - External system integrations and simulators
//! - [`error`] - Error handling
//!
//! ## Features
//!
//! - `wasm` - WebAssembly support via wasm-bindgen
//! - `python` - Python bindings via PyO3
//!
//! ## Examples
//!
//! See the `examples/` directory for complete usage examples.
//!
//! ## 弘益人間 - Benefit All Humanity
//!
//! This standard belongs to humanity.

#![cfg_attr(docsrs, feature(doc_cfg))]
#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod adapters;
pub mod core;
pub mod error;
pub mod integration;
pub mod protocol;
pub mod transport;
pub mod types;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::adapters::*;
    pub use crate::core::*;
    pub use crate::error::*;
    pub use crate::types::*;
}

// Re-export commonly used items at crate root
pub use error::{PhysicsError, PhysicsResult};
pub use types::*;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA Physics Standard version
pub const STANDARD_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
        assert_eq!(STANDARD_VERSION, "1.0.0");
    }

    #[test]
    fn test_measurement_creation() {
        let m = Measurement::new(100.0, 5.0, "GeV");
        assert_eq!(m.value, 100.0);
        assert_eq!(m.unit, "GeV");
    }

    #[test]
    fn test_metadata_creation() {
        let meta = Metadata::new();
        assert!(!meta.id.is_nil());
    }

    #[test]
    fn test_json_roundtrip() {
        let measurement = Measurement::new(125.25, 0.18, "GeV");
        let json = serde_json::to_string(&measurement).unwrap();
        let parsed: Measurement = serde_json::from_str(&json).unwrap();
        assert_eq!(measurement.value, parsed.value);
    }
}
