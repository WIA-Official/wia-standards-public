//! # WIA Perception Clarity Standard SDK
//!
//! Rust implementation of the WIA Perception Clarity Standard.
//!
//! Physical AI agents (autonomous vehicles, robots, drones, AMRs) standardize how
//! their optical / perception sensors **measure**, **state**, and **report** how
//! well they can currently perceive the world.
//!
//! The standard covers three responsibilities:
//!
//! - **MEASURE** — the Perception Clarity Index (PCI), a 0–100 integer computed
//!   from three damage axes (occlusion, distance degradation, MTF/contrast
//!   reduction) weighted per sensor class.
//! - **STATE** — a discrete [`ClarityState`] derived from the PCI band
//!   (clear 90–100 / degraded 60–89 / obstructed 30–59 / blind 0–29).
//! - **REPORT** — a self-describing [`SensorClarityReport`] serialized to JSON
//!   that is wire-identical to the Phase 1 schema.
//!
//! ## Example
//!
//! ```rust
//! use wia_perception_clarity::prelude::*;
//!
//! // Measure: rainy RGB camera.
//! let axes = ClarityAxes { occlusion: 0.18, distance_degradation: 0.22, mtf_reduction: 0.40 };
//! let weights = PciWeights::default_for(SensorClass::RgbCamera);
//! let pci = compute_pci(&axes, &weights);
//!
//! // State: derived from the PCI band.
//! assert_eq!(pci.state(), ClarityState::Degraded);
//! ```
//!
//! ## 弘益人間 — Benefit All Humanity

pub mod types;
pub mod core;
pub mod adapters;
pub mod error;
pub mod protocol;
pub mod security;
pub mod integrations;

// Re-exports for convenience
pub use types::*;
pub use core::*;
pub use error::{Error, Result};

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::types::*;
    pub use crate::core::*;
    pub use crate::adapters::*;
    pub use crate::protocol::*;
    pub use crate::security::*;
    pub use crate::integrations::*;
    pub use crate::error::{Error, Result};
}
