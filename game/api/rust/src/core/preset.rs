//! Accessibility Preset Management
//! 弘益人間 - Gaming for Everyone

use crate::error::{GameError, Result};
use crate::types::*;
use chrono::Utc;
use std::collections::HashMap;

/// Preset manager for handling accessibility presets
#[derive(Debug)]
pub struct PresetManager {
    presets: HashMap<PresetId, AccessibilityPreset>,
}

impl PresetManager {
    /// Create a new preset manager with system presets
    pub fn new() -> Self {
        let mut manager = Self {
            presets: HashMap::new(),
        };
        manager.load_system_presets();
        manager
    }

    /// Load built-in system presets
    fn load_system_presets(&mut self) {
        // Blind-friendly preset
        self.add_preset(AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name: "Blind Friendly".to_string(),
            description: Some("Full audio/haptic feedback for blind players".to_string()),
            preset_type: PresetType::System,
            game_genre: Some(GameGenre::General),
            target_disability: vec![DisabilityType::Blind],
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: Some("WIA".to_string()),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: Some(VisualSettings {
                    screen_reader: ScreenReaderSettings {
                        enabled: true,
                        verbosity: Verbosity::High,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                audio_settings: Some(AudioSettings {
                    haptic_audio: HapticAudioSettings {
                        enabled: true,
                        intensity: 1.0,
                        directional: true,
                        bass_to_haptic: true,
                    },
                    visual_sound_cues: VisualSoundCueSettings {
                        enabled: true,
                        style: VisualSoundCueStyle::Radar,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                motor_settings: Some(MotorSettings {
                    aim_assist: AimAssistSettings {
                        enabled: true,
                        strength: AimAssistStrength::Maximum,
                        auto_aim: true,
                        lock_on: true,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                cognitive_settings: None,
            },
            tags: vec!["blind".to_string(), "audio".to_string(), "haptic".to_string()],
        });

        // Deaf-friendly preset
        self.add_preset(AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name: "Deaf Friendly".to_string(),
            description: Some("Full visual cues and captions for deaf players".to_string()),
            preset_type: PresetType::System,
            game_genre: Some(GameGenre::General),
            target_disability: vec![DisabilityType::Deaf, DisabilityType::HardOfHearing],
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: Some("WIA".to_string()),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: None,
                audio_settings: Some(AudioSettings {
                    subtitles: SubtitleSettings {
                        enabled: true,
                        style: SubtitleStyle::FullCaptions,
                        speaker_identification: true,
                        sound_effects: true,
                        music_indicators: true,
                        directional_indicators: true,
                        ..Default::default()
                    },
                    visual_sound_cues: VisualSoundCueSettings {
                        enabled: true,
                        style: VisualSoundCueStyle::ScreenEdge,
                        ..Default::default()
                    },
                    haptic_audio: HapticAudioSettings {
                        enabled: true,
                        directional: true,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                motor_settings: None,
                cognitive_settings: None,
            },
            tags: vec!["deaf".to_string(), "captions".to_string(), "visual-cues".to_string()],
        });

        // One-handed preset
        self.add_preset(AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name: "One-Handed (Right)".to_string(),
            description: Some("Single right-hand operation".to_string()),
            preset_type: PresetType::System,
            game_genre: Some(GameGenre::General),
            target_disability: vec![DisabilityType::OneHanded, DisabilityType::LimitedMobility],
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: Some("WIA".to_string()),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: None,
                audio_settings: None,
                motor_settings: Some(MotorSettings {
                    one_handed_mode: OneHandedModeSettings {
                        enabled: true,
                        hand: Hand::Right,
                        layout: OneHandedLayout::Compact,
                        gyro_aim: true,
                    },
                    aim_assist: AimAssistSettings {
                        enabled: true,
                        strength: AimAssistStrength::Strong,
                        auto_aim: true,
                        ..Default::default()
                    },
                    auto_actions: AutoActionSettings {
                        auto_run: true,
                        auto_climb: true,
                        auto_reload: true,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                cognitive_settings: None,
            },
            tags: vec!["one-handed".to_string(), "mobility".to_string()],
        });

        // Cognitive support preset
        self.add_preset(AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name: "Cognitive Support".to_string(),
            description: Some("Maximum guidance and simplification".to_string()),
            preset_type: PresetType::System,
            game_genre: Some(GameGenre::General),
            target_disability: vec![DisabilityType::Cognitive, DisabilityType::Adhd, DisabilityType::Dyslexia],
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: Some("WIA".to_string()),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: Some(VisualSettings {
                    text_settings: TextSettings {
                        dyslexia_friendly_font: true,
                        size_multiplier: 1.3,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                audio_settings: None,
                motor_settings: None,
                cognitive_settings: Some(CognitiveSettings {
                    difficulty: DifficultySettings {
                        combat_difficulty: DifficultyLevel::Easy,
                        puzzle_difficulty: DifficultyLevel::Easy,
                        time_pressure: TimePressure::None,
                        auto_complete_qte: true,
                        ..Default::default()
                    },
                    guidance: GuidanceSettings {
                        objective_reminders: true,
                        reminder_frequency_sec: 30,
                        waypoint_guidance: true,
                        path_visualization: true,
                        hint_system: HintSystem::Automatic,
                        ..Default::default()
                    },
                    simplification: SimplificationSettings {
                        simplified_controls: true,
                        reduced_button_combos: true,
                        reduced_visual_effects: true,
                        ..Default::default()
                    },
                    reading_aids: ReadingAidSettings {
                        text_to_speech_ui: true,
                        reading_speed_control: true,
                        pause_during_text: true,
                        ..Default::default()
                    },
                    memory_aids: MemoryAidSettings {
                        story_recap: true,
                        quest_journal: true,
                        recent_actions_log: true,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
            },
            tags: vec!["cognitive".to_string(), "adhd".to_string(), "dyslexia".to_string()],
        });

        // Low mobility preset
        self.add_preset(AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name: "Low Mobility".to_string(),
            description: Some("Minimal precision required".to_string()),
            preset_type: PresetType::System,
            game_genre: Some(GameGenre::General),
            target_disability: vec![DisabilityType::LimitedMobility, DisabilityType::FineMotorDifficulty],
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: Some("WIA".to_string()),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: None,
                audio_settings: None,
                motor_settings: Some(MotorSettings {
                    button_behavior: ButtonBehaviorSettings {
                        hold_to_toggle: vec![
                            "aim".to_string(),
                            "run".to_string(),
                            "crouch".to_string(),
                            "sprint".to_string(),
                        ],
                        tap_timing_ms: 800,
                        hold_timing_ms: 1500,
                        ..Default::default()
                    },
                    stick_settings: StickSettings {
                        left_sensitivity: 0.7,
                        right_sensitivity: 0.7,
                        left_dead_zone: 0.25,
                        right_dead_zone: 0.25,
                        ..Default::default()
                    },
                    aim_assist: AimAssistSettings {
                        enabled: true,
                        strength: AimAssistStrength::Maximum,
                        auto_aim: true,
                        lock_on: true,
                        sticky_aim: true,
                        ..Default::default()
                    },
                    auto_actions: AutoActionSettings {
                        auto_run: true,
                        auto_climb: true,
                        auto_mantle: true,
                        auto_pickup: true,
                        auto_reload: true,
                        ..Default::default()
                    },
                    qte_settings: QteSettings {
                        auto_complete: true,
                        ..Default::default()
                    },
                    sequential_inputs: SequentialInputSettings {
                        enabled: true,
                        timeout_ms: 3000,
                        ..Default::default()
                    },
                    ..Default::default()
                }),
                cognitive_settings: None,
            },
            tags: vec!["mobility".to_string(), "motor".to_string(), "assist".to_string()],
        });
    }

    /// Add a preset
    pub fn add_preset(&mut self, preset: AccessibilityPreset) -> PresetId {
        let id = preset.preset_id;
        self.presets.insert(id, preset);
        id
    }

    /// Get a preset by ID
    pub fn get_preset(&self, preset_id: PresetId) -> Result<&AccessibilityPreset> {
        self.presets
            .get(&preset_id)
            .ok_or(GameError::PresetNotFound(preset_id))
    }

    /// Update a preset
    pub fn update_preset(&mut self, preset: AccessibilityPreset) -> Result<()> {
        if !self.presets.contains_key(&preset.preset_id) {
            return Err(GameError::PresetNotFound(preset.preset_id));
        }

        let mut updated = preset;
        updated.updated_at = Some(Utc::now());
        self.presets.insert(updated.preset_id, updated);
        Ok(())
    }

    /// Delete a preset
    pub fn delete_preset(&mut self, preset_id: PresetId) -> Result<AccessibilityPreset> {
        let preset = self.presets
            .get(&preset_id)
            .ok_or(GameError::PresetNotFound(preset_id))?;

        if preset.preset_type == PresetType::System {
            return Err(GameError::ValidationError(
                "Cannot delete system presets".to_string(),
            ));
        }

        self.presets
            .remove(&preset_id)
            .ok_or(GameError::PresetNotFound(preset_id))
    }

    /// List all presets
    pub fn list_presets(&self) -> Vec<&AccessibilityPreset> {
        self.presets.values().collect()
    }

    /// Find presets by disability type
    pub fn find_by_disability(&self, disability: DisabilityType) -> Vec<&AccessibilityPreset> {
        self.presets
            .values()
            .filter(|p| p.target_disability.contains(&disability))
            .collect()
    }

    /// Find presets by game genre
    pub fn find_by_genre(&self, genre: GameGenre) -> Vec<&AccessibilityPreset> {
        self.presets
            .values()
            .filter(|p| p.game_genre == Some(genre) || p.game_genre == Some(GameGenre::General))
            .collect()
    }

    /// Find presets by tag
    pub fn find_by_tag(&self, tag: &str) -> Vec<&AccessibilityPreset> {
        self.presets
            .values()
            .filter(|p| p.tags.iter().any(|t| t.to_lowercase() == tag.to_lowercase()))
            .collect()
    }

    /// Get system presets only
    pub fn get_system_presets(&self) -> Vec<&AccessibilityPreset> {
        self.presets
            .values()
            .filter(|p| p.preset_type == PresetType::System)
            .collect()
    }

    /// Create a custom preset from current profile settings
    pub fn create_from_profile(
        &mut self,
        name: String,
        profile: &PlayerProfile,
    ) -> AccessibilityPreset {
        let preset = AccessibilityPreset {
            preset_id: uuid::Uuid::new_v4(),
            name,
            description: None,
            preset_type: PresetType::User,
            game_genre: None,
            target_disability: profile.disability_context.primary_disabilities.clone(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: profile.player_info.display_name.clone(),
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride {
                visual_settings: Some(profile.visual_settings.clone()),
                audio_settings: Some(profile.audio_settings.clone()),
                motor_settings: Some(profile.motor_settings.clone()),
                cognitive_settings: Some(profile.cognitive_settings.clone()),
            },
            tags: Vec::new(),
        };

        self.presets.insert(preset.preset_id, preset.clone());
        preset
    }
}

impl Default for PresetManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_system_presets_loaded() {
        let manager = PresetManager::new();
        let system_presets = manager.get_system_presets();
        assert!(system_presets.len() >= 5);
    }

    #[test]
    fn test_find_by_disability() {
        let manager = PresetManager::new();
        let presets = manager.find_by_disability(DisabilityType::Blind);
        assert!(!presets.is_empty());
        assert!(presets[0].target_disability.contains(&DisabilityType::Blind));
    }

    #[test]
    fn test_find_by_tag() {
        let manager = PresetManager::new();
        let presets = manager.find_by_tag("deaf");
        assert!(!presets.is_empty());
    }

    #[test]
    fn test_cannot_delete_system_preset() {
        let mut manager = PresetManager::new();
        let system_presets = manager.get_system_presets();
        let preset_id = system_presets[0].preset_id;

        let result = manager.delete_preset(preset_id);
        assert!(result.is_err());
    }

    #[test]
    fn test_create_from_profile() {
        let mut manager = PresetManager::new();
        let profile = PlayerProfile::default();

        let preset = manager.create_from_profile("My Preset".to_string(), &profile);
        assert_eq!(preset.name, "My Preset");
        assert_eq!(preset.preset_type, PresetType::User);
    }
}
