//! # WIA Fintech - World Inclusive Accessibility Financial Technology Standards
//!
//! This crate provides Rust implementations of WIA Fintech accessibility standards,
//! enabling developers to build accessible financial services including:
//!
//! - ATM accessibility management and discovery
//! - User financial accessibility profiles
//! - Multi-modal accessible notifications
//! - WIA device integration for financial services
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_fintech::{SimulatorAdapter, ProfileManager, ATMManager};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create adapter with sample data
//!     let adapter = SimulatorAdapter::with_sample_data().await;
//!
//!     // Get user profile
//!     let profile = adapter.get_profile("user_blind_001").await?;
//!     println!("User: {:?}", profile.personal_info.preferred_name);
//!
//!     // Find accessible ATMs nearby (San Francisco coordinates)
//!     let atms = adapter.find_atms_nearby(37.7749, -122.4194, 5.0).await?;
//!     for atm in atms {
//!         println!("ATM: {} - WIA Level: {:?}", atm.atm_id, atm.certification.as_ref().map(|c| &c.wia_level));
//!     }
//!
//!     Ok(())
//! }
//! ```
//!
//! ## Features
//!
//! - **Profile Management**: Create, update, and manage user accessibility profiles
//! - **ATM Discovery**: Find accessible ATMs based on user needs and location
//! - **Accessibility Scoring**: Calculate accessibility scores for ATMs and services
//! - **Compatibility Checking**: Match user needs with ATM capabilities
//! - **Notification Building**: Create multi-modal accessible notifications
//!
//! ## WIA Device Integration
//!
//! This crate supports integration with WIA assistive devices:
//!
//! - **Exoskeleton**: Haptic feedback and movement assistance
//! - **Bionic Eye**: Visual overlay and AR guidance
//! - **Voice-Sign**: Sign language translation
//! - **Smart Wheelchair**: Positioning and navigation assistance
//! - **Hearing Aid**: Audio enhancement and alerts

pub mod types;
pub mod error;
pub mod core;
pub mod adapters;

// Re-export main types for convenience
pub use types::*;
pub use error::{FintechError, FintechResult, ErrorCode, ErrorResponse};
pub use core::{
    ProfileManager,
    ATMManager,
    AccessibilityScoreCalculator,
    CompatibilityChecker,
    NotificationBuilder,
    UserProfileBuilder,
    AccessibilityScore,
    CompatibilityResult,
    CompatibilityIssue,
    IssueSeverity,
    ATMWithCompatibility,
    WIACertificationLevel,
};
pub use adapters::SimulatorAdapter;

/// WIA Fintech API version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA Fintech schema version compatibility
pub const SCHEMA_VERSION: &str = "1.0.0";

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::types::*;
    pub use crate::error::{FintechError, FintechResult};
    pub use crate::core::{
        ProfileManager,
        ATMManager,
        AccessibilityScoreCalculator,
        CompatibilityChecker,
        NotificationBuilder,
        UserProfileBuilder,
        IssueSeverity,
    };
    pub use crate::adapters::SimulatorAdapter;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_schema_version() {
        assert_eq!(SCHEMA_VERSION, "1.0.0");
    }

    #[tokio::test]
    async fn test_simulator_adapter_creation() {
        let adapter = SimulatorAdapter::new();
        assert!(adapter.list_profiles().await.unwrap().is_empty());
    }

    #[tokio::test]
    async fn test_simulator_with_sample_data() {
        let adapter = SimulatorAdapter::with_sample_data().await;
        let profiles = adapter.list_profiles().await.unwrap();
        assert!(!profiles.is_empty());

        let atms = adapter.list_atms().await.unwrap();
        assert!(!atms.is_empty());
    }

    #[tokio::test]
    async fn test_accessibility_score_calculator() {
        let adapter = SimulatorAdapter::with_sample_data().await;
        let atm = adapter.get_atm("atm_gold_001").await.unwrap();

        let calculator = AccessibilityScoreCalculator::new();
        let score = calculator.calculate_atm_score(&atm);

        assert!(score.overall >= 0.0 && score.overall <= 100.0);
        assert!(score.visual >= 0.0 && score.visual <= 100.0);
        assert!(score.auditory >= 0.0 && score.auditory <= 100.0);
        assert!(score.motor >= 0.0 && score.motor <= 100.0);
    }

    #[tokio::test]
    async fn test_compatibility_checker() {
        let adapter = SimulatorAdapter::with_sample_data().await;
        let profile = adapter.get_profile("user_blind_001").await.unwrap();
        let atm = adapter.get_atm("atm_gold_001").await.unwrap();

        let checker = CompatibilityChecker::new();
        let result = checker.check_compatibility(&profile, &atm);

        assert!(result.compatibility_score >= 0.0 && result.compatibility_score <= 100.0);
    }

    #[test]
    fn test_notification_builder() {
        let notification = NotificationBuilder::new()
            .notification_type(NotificationType::Transaction)
            .priority(NotificationPriority::Normal)
            .title("Transaction Complete")
            .body("Your transfer of $100 has been completed.")
            .simple_language("You sent $100. It's done.")
            .voice_script("Your transfer of one hundred dollars has been completed successfully.")
            .build()
            .unwrap();

        assert_eq!(notification.notification_type, NotificationType::Transaction);
        assert_eq!(notification.priority, NotificationPriority::Normal);
    }

    #[test]
    fn test_user_profile_builder() {
        let profile = UserProfileBuilder::new("test-profile-001")
            .preferred_language("en-US")
            .region("US")
            .visual_level(VisualLevel::TotallyBlind)
            .uses_screen_reader(true)
            .uses_braille_display(true)
            .auth_method(AuthenticationMethod::BiometricVoice)
            .auth_method(AuthenticationMethod::BiometricFingerprint)
            .build()
            .unwrap();

        assert_eq!(profile.profile_id, "test-profile-001");
        assert_eq!(profile.personal_info.preferred_language, "en-US");
    }
}
