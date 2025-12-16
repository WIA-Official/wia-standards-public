//! # WIA Medical Device Accessibility
//!
//! Rust implementation of the WIA Medical Device Accessibility Standard.
//!
//! This library provides:
//! - Type definitions for medical device accessibility profiles
//! - User accessibility profile management
//! - Device-user compatibility matching
//! - Accessibility score calculation
//! - Multi-sensory alarm system configuration
//! - WIA ecosystem integration (Exoskeleton, Voice-Sign, Haptic)
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_medical::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create a simulator adapter with sample data
//!     let adapter = SimulatorAdapter::with_sample_data().await;
//!
//!     // Get a device profile
//!     let device = adapter.get_device_profile("cgm_dexcom_g7").await?;
//!     println!("Device: {}", device.device.device_name);
//!
//!     // Get a user profile
//!     let user = adapter.get_user_profile("user_blind_001").await?;
//!
//!     // Check compatibility
//!     let result = ProfileMatcher::is_compatible(&device, &user);
//!     println!("Compatible: {}, Score: {:.1}", result.compatible, result.score);
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Building Profiles
//!
//! ```rust,no_run
//! use wia_medical::prelude::*;
//!
//! // Build a device profile
//! let device = DeviceProfileBuilder::new()
//!     .manufacturer("WIA Medical")
//!     .model("AccessibleCGM")
//!     .device_name("WIA Accessible CGM System")
//!     .device_type(MedicalDeviceType::Monitoring)
//!     .device_category(DeviceCategory::Cgm)
//!     .with_voice_output(vec!["en".to_string(), "ko".to_string()])
//!     .with_haptic_feedback()
//!     .build()
//!     .expect("Failed to build profile");
//!
//! // Build a user profile
//! let user = UserProfileBuilder::new()
//!     .visual_needs(VisualLevel::TotallyBlind)
//!     .primary_input(InputMethod::Voice)
//!     .build();
//! ```

#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod adapters;
pub mod core;
pub mod error;
pub mod types;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::adapters::*;
    pub use crate::core::*;
    pub use crate::error::*;
    pub use crate::types::*;
}

// Re-export commonly used items at crate root
pub use adapters::SimulatorAdapter;
pub use core::{
    AccessibilityScoreCalculator, CompatibilityIssue, CompatibilityResult, DeviceProfileBuilder,
    IssueCategory, IssueSeverity, ProfileManager, ProfileMatcher, UserProfileBuilder,
};
pub use error::{MedicalError, Result};
pub use types::*;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA Medical Standard version
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
    fn test_default_device_profile() {
        let profile = MedicalDeviceAccessibilityProfile::default();
        assert!(!profile.profile_id.is_empty());
        assert_eq!(profile.profile_version, "1.0.0");
    }

    #[test]
    fn test_default_user_profile() {
        let profile = UserMedicalAccessibilityProfile::default();
        assert!(!profile.user_id.is_empty());
        assert_eq!(profile.profile_version, "1.0.0");
    }

    #[test]
    fn test_accessibility_score_calculation() {
        let features = DeviceAccessibilityFeatures::default();
        let score = AccessibilityScoreCalculator::calculate(&features);
        assert!(score.overall >= 0.0 && score.overall <= 100.0);
    }
}
