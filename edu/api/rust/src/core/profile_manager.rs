//! Learner Profile Manager
//! 弘益人間 - Education for Everyone

use std::collections::HashMap;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::*;

/// Manages learner accessibility profiles
#[derive(Debug, Default)]
pub struct ProfileManager {
    profiles: HashMap<ProfileId, LearnerProfile>,
}

impl ProfileManager {
    /// Create a new profile manager
    pub fn new() -> Self {
        Self {
            profiles: HashMap::new(),
        }
    }

    /// Create a new learner profile with default settings
    pub fn create_profile(&mut self) -> LearnerProfile {
        let profile = LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo::default(),
            disability_profile: DisabilityProfile::default(),
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences::default(),
            learning_style: LearningStyle::default(),
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        };

        self.profiles.insert(profile.profile_id, profile.clone());
        profile
    }

    /// Create a profile optimized for a specific disability type
    pub fn create_profile_for_disability(&mut self, disability: DisabilityType) -> LearnerProfile {
        let mut profile = self.create_profile();

        profile.disability_profile.disclosed = true;
        profile.disability_profile.disability_types.push(disability);

        // Apply recommended settings based on disability
        match disability {
            DisabilityType::Blind => {
                profile.display_preferences.screen_reader.enabled = true;
                profile.display_preferences.screen_reader.verbosity = Some(Verbosity::Verbose);
                profile.content_preferences.audio_description.required = true;
                profile.content_preferences.transcripts.required = true;
                profile.control_preferences.input_method.keyboard_only = true;
                profile.assessment_accommodations.assistance.reader = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::LowVision => {
                profile.display_preferences.magnification.enabled = true;
                profile.display_preferences.magnification.level = 2.0;
                profile.display_preferences.text_settings.font_size = FontSize::XLarge;
                profile.display_preferences.color_settings.high_contrast = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::ColorBlind => {
                profile.display_preferences.color_settings.color_blind_filter = Some(ColorBlindFilter::Deuteranopia);
            }
            DisabilityType::Deaf => {
                profile.content_preferences.captions.required = true;
                profile.content_preferences.transcripts.required = true;
                profile.content_preferences.sign_language.preferred = true;
            }
            DisabilityType::HardOfHearing => {
                profile.content_preferences.captions.required = true;
                profile.content_preferences.transcripts.required = true;
            }
            DisabilityType::Deafblind => {
                profile.display_preferences.screen_reader.enabled = true;
                profile.content_preferences.transcripts.required = true;
                profile.content_preferences.alternative_formats.push(AlternativeFormat::Braille);
                profile.assessment_accommodations.assistance.reader = true;
                profile.assessment_accommodations.timing.extended_time = true;
                profile.assessment_accommodations.timing.time_multiplier = 3.0;
            }
            DisabilityType::MotorImpairment | DisabilityType::LimitedDexterity => {
                profile.control_preferences.input_method.keyboard_only = true;
                profile.control_preferences.click_settings.large_targets = true;
                profile.control_preferences.click_settings.min_target_size_px = 64;
                profile.control_preferences.timing.extended_time = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::Tremor => {
                profile.control_preferences.click_settings.large_targets = true;
                profile.control_preferences.click_settings.sticky_keys = true;
                profile.control_preferences.timing.extended_time = true;
            }
            DisabilityType::Dyslexia => {
                profile.display_preferences.text_settings.font_family = Some(FontFamily::Opendyslexic);
                profile.display_preferences.text_settings.line_spacing = 2.0;
                profile.display_preferences.text_settings.letter_spacing = Spacing::Wide;
                profile.display_preferences.reading_guide.enabled = true;
                profile.content_preferences.text_to_speech.enabled = true;
                profile.content_preferences.simplification.reading_level = ReadingLevel::Simplified;
                profile.assessment_accommodations.timing.extended_time = true;
                profile.assessment_accommodations.assistance.spell_check = true;
            }
            DisabilityType::Dyscalculia => {
                profile.assessment_accommodations.assistance.calculator = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::Dysgraphia => {
                profile.assessment_accommodations.format.response_formats.push(ResponseFormat::Typed);
                profile.assessment_accommodations.assistance.spell_check = true;
                profile.assessment_accommodations.assistance.grammar_check = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::Adhd => {
                profile.display_preferences.reduce_motion = true;
                profile.content_preferences.simplification.chunked_content = true;
                profile.assessment_accommodations.environment.reduced_distractions = true;
                profile.assessment_accommodations.timing.extended_time = true;
                profile.assessment_accommodations.timing.breaks_allowed = true;
                profile.learning_style.engagement.self_regulation_support = true;
            }
            DisabilityType::Autism => {
                profile.display_preferences.reduce_motion = true;
                profile.content_preferences.simplification.definitions = true;
                profile.learning_style.representation.background_knowledge_support = true;
                profile.assessment_accommodations.environment.reduced_distractions = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            DisabilityType::Cognitive | DisabilityType::Intellectual => {
                profile.content_preferences.simplification.reading_level = ReadingLevel::EasyRead;
                profile.content_preferences.simplification.chunked_content = true;
                profile.content_preferences.simplification.definitions = true;
                profile.assessment_accommodations.timing.extended_time = true;
                profile.assessment_accommodations.timing.time_multiplier = 2.0;
                profile.learning_style.representation.background_knowledge_support = true;
                profile.learning_style.action_expression.planning_support = true;
            }
            DisabilityType::Memory => {
                profile.assessment_accommodations.assistance.notes = true;
                profile.assessment_accommodations.timing.extended_time = true;
                profile.learning_style.engagement.self_regulation_support = true;
            }
            DisabilityType::Anxiety => {
                profile.assessment_accommodations.environment.separate_room = true;
                profile.assessment_accommodations.timing.breaks_allowed = true;
                profile.assessment_accommodations.timing.extended_time = true;
            }
            _ => {}
        }

        self.profiles.insert(profile.profile_id, profile.clone());
        profile
    }

    /// Get a profile by ID
    pub fn get_profile(&self, profile_id: ProfileId) -> Result<&LearnerProfile> {
        self.profiles
            .get(&profile_id)
            .ok_or(EduError::ProfileNotFound(profile_id))
    }

    /// Get a mutable profile by ID
    pub fn get_profile_mut(&mut self, profile_id: ProfileId) -> Result<&mut LearnerProfile> {
        self.profiles
            .get_mut(&profile_id)
            .ok_or(EduError::ProfileNotFound(profile_id))
    }

    /// Update a profile
    pub fn update_profile(&mut self, mut profile: LearnerProfile) -> Result<()> {
        if !self.profiles.contains_key(&profile.profile_id) {
            return Err(EduError::ProfileNotFound(profile.profile_id));
        }
        profile.updated_at = Some(Utc::now());
        self.profiles.insert(profile.profile_id, profile);
        Ok(())
    }

    /// Delete a profile
    pub fn delete_profile(&mut self, profile_id: ProfileId) -> Result<LearnerProfile> {
        self.profiles
            .remove(&profile_id)
            .ok_or(EduError::ProfileNotFound(profile_id))
    }

    /// List all profiles
    pub fn list_profiles(&self) -> Vec<&LearnerProfile> {
        self.profiles.values().collect()
    }

    /// Export profile to JSON
    pub fn export_profile(&self, profile_id: ProfileId) -> Result<String> {
        let profile = self.get_profile(profile_id)?;
        serde_json::to_string_pretty(profile).map_err(EduError::from)
    }

    /// Import profile from JSON
    pub fn import_profile(&mut self, json: &str) -> Result<LearnerProfile> {
        let mut profile: LearnerProfile = serde_json::from_str(json)?;
        // Assign new ID to avoid conflicts
        profile.profile_id = Uuid::new_v4();
        profile.created_at = Utc::now();
        profile.updated_at = None;
        self.profiles.insert(profile.profile_id, profile.clone());
        Ok(profile)
    }

    /// Find profiles by disability type
    pub fn find_by_disability(&self, disability: DisabilityType) -> Vec<&LearnerProfile> {
        self.profiles
            .values()
            .filter(|p| p.disability_profile.disability_types.contains(&disability))
            .collect()
    }

    /// Apply accessibility preferences from another profile
    pub fn apply_preferences_from(
        &mut self,
        target_id: ProfileId,
        source_id: ProfileId,
    ) -> Result<()> {
        let source = self.get_profile(source_id)?.clone();
        let target = self.get_profile_mut(target_id)?;

        target.display_preferences = source.display_preferences;
        target.control_preferences = source.control_preferences;
        target.content_preferences = source.content_preferences;
        target.updated_at = Some(Utc::now());

        Ok(())
    }

    /// Link WIA device profiles
    pub fn link_wia_profile(
        &mut self,
        profile_id: ProfileId,
        device_type: &str,
        device_profile_id: Uuid,
    ) -> Result<()> {
        let profile = self.get_profile_mut(profile_id)?;

        match device_type {
            "aac" => profile.wia_integrations.aac_profile_id = Some(device_profile_id),
            "bci" => profile.wia_integrations.bci_profile_id = Some(device_profile_id),
            "eye_gaze" => profile.wia_integrations.eye_gaze_profile_id = Some(device_profile_id),
            "wheelchair" => profile.wia_integrations.wheelchair_profile_id = Some(device_profile_id),
            "smart_home" => profile.wia_integrations.smart_home_profile_id = Some(device_profile_id),
            _ => return Err(EduError::InvalidConfiguration(format!("Unknown device type: {}", device_type))),
        }

        profile.updated_at = Some(Utc::now());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_profile() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile();

        assert!(!profile.profile_id.is_nil());
        assert_eq!(profile.schema_version, "1.0.0");
    }

    #[test]
    fn test_create_profile_for_blind() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Blind);

        assert!(profile.display_preferences.screen_reader.enabled);
        assert!(profile.content_preferences.audio_description.required);
        assert!(profile.control_preferences.input_method.keyboard_only);
    }

    #[test]
    fn test_create_profile_for_deaf() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Deaf);

        assert!(profile.content_preferences.captions.required);
        assert!(profile.content_preferences.sign_language.preferred);
    }

    #[test]
    fn test_create_profile_for_dyslexia() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Dyslexia);

        assert_eq!(profile.display_preferences.text_settings.font_family, Some(FontFamily::Opendyslexic));
        assert!(profile.content_preferences.text_to_speech.enabled);
        assert!(profile.assessment_accommodations.assistance.spell_check);
    }

    #[test]
    fn test_export_import() {
        let mut manager = ProfileManager::new();
        let original = manager.create_profile_for_disability(DisabilityType::LowVision);
        let original_id = original.profile_id;

        let json = manager.export_profile(original_id).unwrap();
        let imported = manager.import_profile(&json).unwrap();

        // IDs should be different
        assert_ne!(original_id, imported.profile_id);
        // But settings should match
        assert_eq!(
            original.display_preferences.magnification.level,
            imported.display_preferences.magnification.level
        );
    }
}
