//! # WIA Game Accessibility API
//!
//! A Rust library for gaming accessibility standards.
//!
//! ## Features
//!
//! - **Player Profiles**: Manage player accessibility preferences
//! - **Presets**: Built-in and custom accessibility presets
//! - **Game Metadata**: Define and query game accessibility features
//! - **Validation**: Validate settings and configurations
//! - **Storage**: File-based and in-memory profile storage
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_game::{GameController, types::*};
//!
//! #[tokio::main]
//! async fn main() {
//!     // Create controller
//!     let mut controller = GameController::new();
//!
//!     // Create a profile for a blind user
//!     let profile = controller.create_profile_for_disability(DisabilityType::Blind);
//!
//!     // Profile has recommended settings for blind players
//!     assert!(profile.visual_settings.screen_reader.enabled);
//!     assert!(profile.motor_settings.aim_assist.auto_aim);
//! }
//! ```
//!
//! ## 弘益人間 - Gaming for Everyone

pub mod adapters;
pub mod core;
pub mod ecosystem;
pub mod error;
pub mod protocol;
pub mod types;

pub use adapters::*;
pub use core::*;
pub use error::{GameError, Result};
pub use types::*;

use std::sync::Arc;

/// Main game accessibility controller
#[derive(Debug)]
pub struct GameController {
    profile_manager: ProfileManager,
    preset_manager: PresetManager,
    game_manager: GameManager,
    storage: Option<Arc<dyn ProfileStorage>>,
}

impl GameController {
    /// Create a new game controller
    pub fn new() -> Self {
        Self {
            profile_manager: ProfileManager::new(),
            preset_manager: PresetManager::new(),
            game_manager: GameManager::new(),
            storage: None,
        }
    }

    /// Set profile storage adapter
    pub fn with_storage(mut self, storage: Arc<dyn ProfileStorage>) -> Self {
        self.storage = Some(storage);
        self
    }

    // ========================================================================
    // Profile Management
    // ========================================================================

    /// Create a new player profile with default settings
    pub fn create_profile(&mut self) -> PlayerProfile {
        self.profile_manager.create_profile()
    }

    /// Create a profile optimized for a specific disability
    pub fn create_profile_for_disability(&mut self, disability: DisabilityType) -> PlayerProfile {
        self.profile_manager.create_profile_for_disability(disability)
    }

    /// Get a profile by ID
    pub fn get_profile(&self, profile_id: ProfileId) -> Result<&PlayerProfile> {
        self.profile_manager.get_profile(profile_id)
    }

    /// Update a profile
    pub fn update_profile(&mut self, profile: PlayerProfile) -> Result<()> {
        Validator::validate_profile(&profile)?;
        self.profile_manager.update_profile(profile)
    }

    /// Delete a profile
    pub fn delete_profile(&mut self, profile_id: ProfileId) -> Result<PlayerProfile> {
        self.profile_manager.delete_profile(profile_id)
    }

    /// List all profiles
    pub fn list_profiles(&self) -> Vec<&PlayerProfile> {
        self.profile_manager.list_profiles()
    }

    /// Export profile to JSON
    pub fn export_profile(&self, profile_id: ProfileId) -> Result<String> {
        self.profile_manager.export_profile(profile_id)
    }

    /// Import profile from JSON
    pub fn import_profile(&mut self, json: &str) -> Result<PlayerProfile> {
        self.profile_manager.import_profile(json)
    }

    /// Apply a preset to a profile
    pub fn apply_preset(&mut self, profile_id: ProfileId, preset_id: PresetId) -> Result<()> {
        let preset = self.preset_manager.get_preset(preset_id)?.clone();
        self.profile_manager.apply_preset(profile_id, &preset)
    }

    // ========================================================================
    // Async Storage Operations
    // ========================================================================

    /// Save profile to storage
    pub async fn save_profile(&self, profile: &PlayerProfile) -> Result<()> {
        match &self.storage {
            Some(storage) => storage.save_profile(profile).await,
            None => Err(GameError::InvalidConfiguration("No storage configured".to_string())),
        }
    }

    /// Load profile from storage
    pub async fn load_profile(&mut self, profile_id: ProfileId) -> Result<PlayerProfile> {
        match &self.storage {
            Some(storage) => {
                let profile = storage.load_profile(profile_id).await?;
                // Add to in-memory manager
                self.profile_manager.update_profile(profile.clone()).ok();
                Ok(profile)
            }
            None => Err(GameError::InvalidConfiguration("No storage configured".to_string())),
        }
    }

    // ========================================================================
    // Preset Management
    // ========================================================================

    /// Get all system presets
    pub fn get_system_presets(&self) -> Vec<&AccessibilityPreset> {
        self.preset_manager.get_system_presets()
    }

    /// Get all presets
    pub fn list_presets(&self) -> Vec<&AccessibilityPreset> {
        self.preset_manager.list_presets()
    }

    /// Get preset by ID
    pub fn get_preset(&self, preset_id: PresetId) -> Result<&AccessibilityPreset> {
        self.preset_manager.get_preset(preset_id)
    }

    /// Find presets for a disability type
    pub fn find_presets_for_disability(&self, disability: DisabilityType) -> Vec<&AccessibilityPreset> {
        self.preset_manager.find_by_disability(disability)
    }

    /// Find presets for a game genre
    pub fn find_presets_for_genre(&self, genre: GameGenre) -> Vec<&AccessibilityPreset> {
        self.preset_manager.find_by_genre(genre)
    }

    /// Create a preset from a profile
    pub fn create_preset_from_profile(
        &mut self,
        name: String,
        profile: &PlayerProfile,
    ) -> AccessibilityPreset {
        self.preset_manager.create_from_profile(name, profile)
    }

    // ========================================================================
    // Game Management
    // ========================================================================

    /// Register a game with accessibility metadata
    pub fn register_game(&mut self, metadata: GameMetadata) -> GameId {
        self.game_manager.register_game(metadata)
    }

    /// Get game metadata
    pub fn get_game(&self, game_id: GameId) -> Result<&GameMetadata> {
        self.game_manager.get_game(game_id)
    }

    /// Check if a game supports a feature
    pub fn game_supports_feature(
        &self,
        game_id: GameId,
        feature: AccessibilityFeature,
    ) -> Result<bool> {
        self.game_manager.supports_feature(game_id, feature)
    }

    /// Get compatibility report between profile and game
    pub fn get_compatibility(
        &self,
        profile: &PlayerProfile,
        game_id: GameId,
    ) -> Result<CompatibilityReport> {
        self.game_manager.get_compatibility(profile, game_id)
    }

    /// Find games suitable for a disability
    pub fn find_games_for_disability(&self, disability: DisabilityType) -> Vec<&GameMetadata> {
        self.game_manager.find_games_for_disability(disability)
    }

    // ========================================================================
    // Validation
    // ========================================================================

    /// Validate a profile
    pub fn validate_profile(profile: &PlayerProfile) -> Result<()> {
        Validator::validate_profile(profile)
    }

    /// Validate a preset
    pub fn validate_preset(preset: &AccessibilityPreset) -> Result<()> {
        Validator::validate_preset(preset)
    }

    /// Validate a controller config
    pub fn validate_controller_config(config: &ControllerConfig) -> Result<()> {
        Validator::validate_controller_config(config)
    }
}

impl Default for GameController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_controller() {
        let controller = GameController::new();
        assert!(controller.list_profiles().is_empty());
        assert!(!controller.get_system_presets().is_empty());
    }

    #[test]
    fn test_profile_workflow() {
        let mut controller = GameController::new();

        // Create profile
        let profile = controller.create_profile();
        let id = profile.profile_id;

        // Get profile
        let retrieved = controller.get_profile(id).unwrap();
        assert_eq!(retrieved.profile_id, id);

        // Delete profile
        controller.delete_profile(id).unwrap();
        assert!(controller.get_profile(id).is_err());
    }

    #[test]
    fn test_disability_profile() {
        let mut controller = GameController::new();

        let profile = controller.create_profile_for_disability(DisabilityType::Deaf);

        assert!(profile.audio_settings.subtitles.enabled);
        assert!(profile.audio_settings.visual_sound_cues.enabled);
    }

    #[test]
    fn test_preset_application() {
        let mut controller = GameController::new();

        let profile = controller.create_profile();
        let profile_id = profile.profile_id;

        let presets = controller.find_presets_for_disability(DisabilityType::Blind);
        let preset_id = presets[0].preset_id;

        controller.apply_preset(profile_id, preset_id).unwrap();

        let updated = controller.get_profile(profile_id).unwrap();
        assert!(updated.visual_settings.screen_reader.enabled);
    }

    #[test]
    fn test_export_import() {
        let mut controller = GameController::new();

        let original = controller.create_profile_for_disability(DisabilityType::LowVision);
        let json = controller.export_profile(original.profile_id).unwrap();

        let imported = controller.import_profile(&json).unwrap();
        assert_eq!(
            original.visual_settings.magnification.level,
            imported.visual_settings.magnification.level
        );
    }
}
