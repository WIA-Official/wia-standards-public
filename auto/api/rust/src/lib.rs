//! # WIA Autonomous Vehicle Accessibility API
//!
//! Rust implementation of the WIA Autonomous Vehicle Accessibility Standard.
//!
//! This library provides types and APIs for building accessible autonomous
//! vehicle systems, including passenger profiles, vehicle capabilities,
//! trip management, and HMI configuration.
//!
//! ## Features
//!
//! - **Passenger Profiles**: Define accessibility requirements and preferences
//! - **Vehicle Capabilities**: Describe vehicle accessibility features
//! - **Trip Management**: Request and manage accessible trips
//! - **HMI Configuration**: Configure multi-modal human-machine interface
//! - **Securement Monitoring**: Track wheelchair securement status
//! - **Emergency Handling**: Handle emergency events
//!
//! ## Example
//!
//! ```rust
//! use wia_auto::prelude::*;
//!
//! // Create a passenger profile
//! let profile = PassengerProfile::builder()
//!     .name("Example User")
//!     .add_disability(DisabilityType::VisualBlind)
//!     .preferred_modalities(vec![
//!         InteractionModality::AudioTts,
//!         InteractionModality::HapticVibration,
//!         InteractionModality::Braille,
//!     ])
//!     .minimize_walking(true)
//!     .build()
//!     .expect("Failed to build profile");
//!
//! // Create HMI config for blind users
//! let hmi_config = HmiConfig::for_blind();
//!
//! // Build accessibility requirements
//! let requirements = AccessibilityRequirements::builder()
//!     .wheelchair_accessible()
//!     .requires_ramp()
//!     .build();
//! ```
//!
//! ## 弘益人間 - Benefit All Humanity

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
