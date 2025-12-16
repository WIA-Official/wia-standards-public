//! WIA XR Accessibility API
//!
//! A comprehensive Rust library for XR (VR/AR/MR) accessibility features.
//!
//! This library implements the WIA (World Inclusive Accessibility) XR accessibility
//! standards, providing tools for building inclusive XR experiences.
//!
//! # Features
//!
//! - **Profile Management**: Create, load, and manage accessibility profiles
//! - **Adaptations**: Apply accessibility adaptations like captions, audio descriptions, etc.
//! - **Session Management**: Track session time, breaks, and health monitoring
//! - **Platform Adapters**: Integrate with various XR platforms
//! - **WIA Integration**: Connect with other WIA systems (Exoskeleton, Bionic Eye, Voice-Sign)
//!
//! # Example
//!
//! ```rust,no_run
//! use wia_xr_accessibility::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create the XR accessibility engine
//!     let engine = XRAccessibilityEngine::new();
//!
//!     // Build a profile for a user
//!     let profile = ProfileBuilder::new("user-001")
//!         .visual_disability(VisualDisability {
//!             blind: false,
//!             low_vision: Some(LowVision {
//!                 acuity: 0.3,
//!                 field_of_view_degrees: Some(60.0),
//!                 contrast_sensitivity: Some(0.5),
//!                 light_sensitivity: Some(0.7),
//!             }),
//!             color_blind: None,
//!             photosensitive_epilepsy: false,
//!         })
//!         .build();
//!
//!     // Load the profile
//!     engine.load_profile(profile).await?;
//!
//!     // Get accessibility state
//!     let state = engine.get_accessibility_state().await;
//!     println!("Active adaptations: {}", state.active_adaptation_count);
//!
//!     Ok(())
//! }
//! ```
//!
//! # Modules
//!
//! - [`types`]: Core type definitions for accessibility profiles
//! - [`error`]: Error types for the library
//! - [`core`]: Core engine, profile management, and adaptation system
//! - [`adapters`]: Platform and integration adapters

#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod types;
pub mod error;
pub mod core;
pub mod adapters;

/// Re-exports of commonly used types
pub mod prelude {
    pub use crate::types::*;
    pub use crate::error::*;
    pub use crate::core::*;
    pub use crate::adapters::*;
}

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA protocol version supported
pub const WIA_PROTOCOL_VERSION: &str = "1.0.0";

/// Create a new XR accessibility engine with default configuration
pub fn create_engine() -> core::XRAccessibilityEngine {
    core::XRAccessibilityEngine::new()
}

/// Create a new profile manager with in-memory storage
pub fn create_profile_manager() -> core::ProfileManager {
    core::ProfileManager::in_memory()
}

/// Create a new XR simulator for testing
pub fn create_simulator() -> adapters::XRSimulator {
    adapters::XRSimulator::new()
}

/// Create a new WIA integration hub
pub fn create_wia_hub() -> adapters::WIAIntegrationHub {
    adapters::WIAIntegrationHub::new()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileBuilder;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
        assert!(!WIA_PROTOCOL_VERSION.is_empty());
    }

    #[test]
    fn test_create_engine() {
        let _engine = create_engine();
    }

    #[test]
    fn test_create_profile_manager() {
        let _manager = create_profile_manager();
    }

    #[test]
    fn test_create_simulator() {
        let _sim = create_simulator();
    }

    #[tokio::test]
    async fn test_full_workflow() {
        use crate::core::*;
        use crate::types::*;

        // Create engine
        let engine = create_engine();

        // Create profile
        let profile = ProfileBuilder::new("test-user")
            .user_id("user-123")
            .build();

        // Load profile
        engine.load_profile(profile).await.unwrap();

        // Check state
        let state = engine.get_accessibility_state().await;
        assert!(state.profile_active);
    }

    #[tokio::test]
    async fn test_simulator_workflow() {
        use crate::adapters::*;

        // Create simulator
        let sim = create_simulator();

        // Simulate interactions
        sim.speak("open menu").await;
        sim.toggle_safe_space(true).await;

        let state = sim.get_state().await;
        assert!(state.is_in_safe_space);
        assert_eq!(state.voice_input, Some("open menu".to_string()));

        // Check events
        let events = sim.get_events().await;
        assert!(!events.is_empty());
    }
}
