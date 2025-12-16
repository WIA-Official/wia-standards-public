//! Player Profile Management
//! 弘益人間 - Gaming for Everyone

use crate::error::{GameError, Result};
use crate::types::*;
use chrono::Utc;
use std::collections::HashMap;
use uuid::Uuid;

/// Profile manager for handling player accessibility profiles
#[derive(Debug, Default)]
pub struct ProfileManager {
    profiles: HashMap<ProfileId, PlayerProfile>,
}

impl ProfileManager {
    /// Create a new profile manager
    pub fn new() -> Self {
        Self {
            profiles: HashMap::new(),
        }
    }

    /// Create a new player profile with default settings
    pub fn create_profile(&mut self) -> PlayerProfile {
        let profile = PlayerProfile::default();
        self.profiles.insert(profile.profile_id, profile.clone());
        profile
    }

    /// Create a profile for a specific disability type
    pub fn create_profile_for_disability(&mut self, disability: DisabilityType) -> PlayerProfile {
        let mut profile = PlayerProfile::default();
        profile.disability_context.primary_disabilities.push(disability);

        // Apply recommended settings based on disability
        match disability {
            DisabilityType::Blind => {
                profile.visual_settings.screen_reader.enabled = true;
                profile.visual_settings.screen_reader.verbosity = Verbosity::High;
                profile.audio_settings.haptic_audio.enabled = true;
                profile.audio_settings.haptic_audio.directional = true;
                profile.motor_settings.aim_assist.enabled = true;
                profile.motor_settings.aim_assist.strength = AimAssistStrength::Maximum;
                profile.motor_settings.aim_assist.auto_aim = true;
            }
            DisabilityType::LowVision => {
                profile.visual_settings.magnification.enabled = true;
                profile.visual_settings.magnification.level = 2.0;
                profile.visual_settings.text_settings.size_multiplier = 1.5;
                profile.visual_settings.color_settings.high_contrast = true;
                profile.visual_settings.target_indicators.enemy_highlight = true;
                profile.visual_settings.target_indicators.item_highlight = true;
            }
            DisabilityType::Colorblind => {
                profile.visual_settings.color_settings.colorblind_mode = ColorblindMode::Deuteranopia;
                profile.visual_settings.target_indicators.enemy_highlight = true;
            }
            DisabilityType::Deaf | DisabilityType::HardOfHearing => {
                profile.audio_settings.subtitles.enabled = true;
                profile.audio_settings.subtitles.style = SubtitleStyle::FullCaptions;
                profile.audio_settings.subtitles.speaker_identification = true;
                profile.audio_settings.subtitles.sound_effects = true;
                profile.audio_settings.subtitles.directional_indicators = true;
                profile.audio_settings.visual_sound_cues.enabled = true;
                profile.audio_settings.haptic_audio.enabled = true;
            }
            DisabilityType::LimitedMobility | DisabilityType::FineMotorDifficulty => {
                profile.motor_settings.aim_assist.enabled = true;
                profile.motor_settings.aim_assist.strength = AimAssistStrength::Strong;
                profile.motor_settings.auto_actions.auto_run = true;
                profile.motor_settings.auto_actions.auto_reload = true;
                profile.motor_settings.auto_actions.auto_climb = true;
                profile.motor_settings.button_behavior.hold_to_toggle = vec![
                    "aim".to_string(),
                    "run".to_string(),
                    "crouch".to_string(),
                ];
                profile.motor_settings.qte_settings.auto_complete = true;
            }
            DisabilityType::OneHanded => {
                profile.motor_settings.one_handed_mode.enabled = true;
                profile.motor_settings.aim_assist.enabled = true;
                profile.motor_settings.aim_assist.strength = AimAssistStrength::Strong;
                profile.motor_settings.auto_actions.auto_run = true;
            }
            DisabilityType::Cognitive | DisabilityType::Adhd => {
                profile.cognitive_settings.difficulty.combat_difficulty = DifficultyLevel::Easy;
                profile.cognitive_settings.guidance.objective_reminders = true;
                profile.cognitive_settings.guidance.waypoint_guidance = true;
                profile.cognitive_settings.guidance.hint_system = HintSystem::Progressive;
                profile.cognitive_settings.simplification.simplified_controls = true;
                profile.cognitive_settings.memory_aids.story_recap = true;
                profile.cognitive_settings.memory_aids.quest_journal = true;
                profile.cognitive_settings.focus_aids.break_reminders = true;
            }
            DisabilityType::Dyslexia => {
                profile.visual_settings.text_settings.dyslexia_friendly_font = true;
                profile.visual_settings.text_settings.size_multiplier = 1.3;
                profile.visual_settings.text_settings.letter_spacing = 0.5;
                profile.cognitive_settings.reading_aids.text_to_speech_ui = true;
                profile.cognitive_settings.reading_aids.reading_speed_control = true;
            }
            DisabilityType::Epilepsy => {
                profile.visual_settings.ui_settings.reduce_motion = true;
                profile.visual_settings.ui_settings.disable_screen_shake = true;
                profile.cognitive_settings.content_warnings.enabled = true;
                profile.cognitive_settings.content_warnings.categories.push(ContentWarningCategory::Flashing);
            }
            _ => {}
        }

        self.profiles.insert(profile.profile_id, profile.clone());
        profile
    }

    /// Get a profile by ID
    pub fn get_profile(&self, profile_id: ProfileId) -> Result<&PlayerProfile> {
        self.profiles
            .get(&profile_id)
            .ok_or(GameError::ProfileNotFound(profile_id))
    }

    /// Get a mutable profile by ID
    pub fn get_profile_mut(&mut self, profile_id: ProfileId) -> Result<&mut PlayerProfile> {
        self.profiles
            .get_mut(&profile_id)
            .ok_or(GameError::ProfileNotFound(profile_id))
    }

    /// Update a profile
    pub fn update_profile(&mut self, profile: PlayerProfile) -> Result<()> {
        if !self.profiles.contains_key(&profile.profile_id) {
            return Err(GameError::ProfileNotFound(profile.profile_id));
        }

        let mut updated = profile;
        updated.updated_at = Some(Utc::now());
        self.profiles.insert(updated.profile_id, updated);
        Ok(())
    }

    /// Delete a profile
    pub fn delete_profile(&mut self, profile_id: ProfileId) -> Result<PlayerProfile> {
        self.profiles
            .remove(&profile_id)
            .ok_or(GameError::ProfileNotFound(profile_id))
    }

    /// List all profiles
    pub fn list_profiles(&self) -> Vec<&PlayerProfile> {
        self.profiles.values().collect()
    }

    /// Export profile to JSON
    pub fn export_profile(&self, profile_id: ProfileId) -> Result<String> {
        let profile = self.get_profile(profile_id)?;
        let export = ProfileExport {
            format_version: SCHEMA_VERSION.to_string(),
            export_date: Utc::now(),
            profile: profile.clone(),
        };
        serde_json::to_string_pretty(&export).map_err(GameError::from)
    }

    /// Import profile from JSON
    pub fn import_profile(&mut self, json: &str) -> Result<PlayerProfile> {
        let export: ProfileExport = serde_json::from_str(json)?;

        // Validate version compatibility
        if export.format_version != SCHEMA_VERSION {
            // For now, we'll accept any version but log a warning
            // In production, implement migration logic
        }

        let mut profile = export.profile;
        profile.profile_id = Uuid::new_v4(); // Assign new ID
        profile.created_at = Some(Utc::now());
        profile.updated_at = Some(Utc::now());

        self.profiles.insert(profile.profile_id, profile.clone());
        Ok(profile)
    }

    /// Apply a preset to a profile
    pub fn apply_preset(&mut self, profile_id: ProfileId, preset: &AccessibilityPreset) -> Result<()> {
        let profile = self.get_profile_mut(profile_id)?;

        if let Some(ref visual) = preset.settings_override.visual_settings {
            profile.visual_settings = visual.clone();
        }
        if let Some(ref audio) = preset.settings_override.audio_settings {
            profile.audio_settings = audio.clone();
        }
        if let Some(ref motor) = preset.settings_override.motor_settings {
            profile.motor_settings = motor.clone();
        }
        if let Some(ref cognitive) = preset.settings_override.cognitive_settings {
            profile.cognitive_settings = cognitive.clone();
        }

        profile.updated_at = Some(Utc::now());
        Ok(())
    }

    /// Merge settings from another profile
    pub fn merge_profiles(&mut self, target_id: ProfileId, source: &PlayerProfile) -> Result<()> {
        let target = self.get_profile_mut(target_id)?;

        // Merge disability context
        for disability in &source.disability_context.primary_disabilities {
            if !target.disability_context.primary_disabilities.contains(disability) {
                target.disability_context.primary_disabilities.push(*disability);
            }
        }

        for tech in &source.disability_context.assistive_technologies {
            if !target.disability_context.assistive_technologies.contains(tech) {
                target.disability_context.assistive_technologies.push(*tech);
            }
        }

        target.updated_at = Some(Utc::now());
        Ok(())
    }
}

/// Profile export format
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ProfileExport {
    pub format_version: String,
    pub export_date: chrono::DateTime<Utc>,
    pub profile: PlayerProfile,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_profile() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile();
        assert_eq!(profile.version, SCHEMA_VERSION);
        assert!(manager.get_profile(profile.profile_id).is_ok());
    }

    #[test]
    fn test_create_profile_for_blind() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Blind);

        assert!(profile.visual_settings.screen_reader.enabled);
        assert!(profile.motor_settings.aim_assist.auto_aim);
        assert_eq!(profile.motor_settings.aim_assist.strength, AimAssistStrength::Maximum);
    }

    #[test]
    fn test_create_profile_for_deaf() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Deaf);

        assert!(profile.audio_settings.subtitles.enabled);
        assert!(profile.audio_settings.visual_sound_cues.enabled);
        assert!(profile.audio_settings.subtitles.directional_indicators);
    }

    #[test]
    fn test_export_import_profile() {
        let mut manager = ProfileManager::new();
        let original = manager.create_profile_for_disability(DisabilityType::LowVision);

        let json = manager.export_profile(original.profile_id).unwrap();
        let imported = manager.import_profile(&json).unwrap();

        assert_ne!(original.profile_id, imported.profile_id);
        assert_eq!(
            original.visual_settings.magnification.level,
            imported.visual_settings.magnification.level
        );
    }

    #[test]
    fn test_delete_profile() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile();
        let id = profile.profile_id;

        assert!(manager.delete_profile(id).is_ok());
        assert!(manager.get_profile(id).is_err());
    }
}
