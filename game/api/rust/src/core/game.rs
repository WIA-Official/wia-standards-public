//! Game Accessibility Features
//! 弘益人間 - Gaming for Everyone

use crate::error::{GameError, Result};
use crate::types::*;
use std::collections::HashMap;

/// Game accessibility manager
#[derive(Debug)]
pub struct GameManager {
    games: HashMap<GameId, GameMetadata>,
}

impl GameManager {
    /// Create a new game manager
    pub fn new() -> Self {
        Self {
            games: HashMap::new(),
        }
    }

    /// Register a game with its accessibility metadata
    pub fn register_game(&mut self, metadata: GameMetadata) -> GameId {
        let id = metadata.game_id;
        self.games.insert(id, metadata);
        id
    }

    /// Get game metadata by ID
    pub fn get_game(&self, game_id: GameId) -> Result<&GameMetadata> {
        self.games
            .get(&game_id)
            .ok_or(GameError::GameNotFound(game_id))
    }

    /// List all registered games
    pub fn list_games(&self) -> Vec<&GameMetadata> {
        self.games.values().collect()
    }

    /// Check if a game supports a specific feature
    pub fn supports_feature(&self, game_id: GameId, feature: AccessibilityFeature) -> Result<bool> {
        let game = self.get_game(game_id)?;

        Ok(match feature {
            // Visual features
            AccessibilityFeature::ScreenReader => game.visual_features.screen_reader_support,
            AccessibilityFeature::Colorblind => !game.visual_features.colorblind_modes.is_empty(),
            AccessibilityFeature::HighContrast => game.visual_features.high_contrast_mode,
            AccessibilityFeature::ReduceMotion => game.visual_features.reduce_motion_option,

            // Audio features
            AccessibilityFeature::Subtitles => game.audio_features.subtitle_support,
            AccessibilityFeature::ClosedCaptions => game.audio_features.closed_captions,
            AccessibilityFeature::VisualSoundCues => game.audio_features.visual_sound_indicators,
            AccessibilityFeature::MonoAudio => game.audio_features.mono_audio_option,
            AccessibilityFeature::TtsChat => game.audio_features.tts_chat,

            // Motor features
            AccessibilityFeature::ButtonRemapping => game.motor_features.full_button_remapping,
            AccessibilityFeature::HoldToToggle => game.motor_features.hold_to_toggle_options,
            AccessibilityFeature::AimAssist => !game.motor_features.aim_assist_levels.is_empty(),
            AccessibilityFeature::OneHandedMode => !game.motor_features.one_handed_modes.is_empty(),
            AccessibilityFeature::AdaptiveController => game.motor_features.adaptive_controller_support,
            AccessibilityFeature::SwitchAccess => game.motor_features.switch_access,
            AccessibilityFeature::CopilotMode => game.motor_features.copilot_mode,

            // Cognitive features
            AccessibilityFeature::DifficultyOptions => game.cognitive_features.multiple_difficulty_levels,
            AccessibilityFeature::SkipPuzzles => game.cognitive_features.skip_puzzles_option,
            AccessibilityFeature::ObjectiveMarkers => game.cognitive_features.objective_markers,
            AccessibilityFeature::HintSystem => game.cognitive_features.hint_system,
            AccessibilityFeature::ContentWarnings => game.cognitive_features.content_warnings,
            AccessibilityFeature::SaveAnywhere => game.cognitive_features.save_anywhere,

            // Input support
            AccessibilityFeature::EyeTracking => game.input_support.eye_tracking,
            AccessibilityFeature::VoiceCommands => game.input_support.voice_commands,
            AccessibilityFeature::HeadTracking => game.input_support.head_tracking,
        })
    }

    /// Get compatibility score between a profile and a game
    pub fn get_compatibility(&self, profile: &PlayerProfile, game_id: GameId) -> Result<CompatibilityReport> {
        let game = self.get_game(game_id)?;

        let mut supported_features = Vec::new();
        let mut missing_features = Vec::new();
        let mut partial_features = Vec::new();

        // Check visual needs
        if profile.visual_settings.screen_reader.enabled {
            if game.visual_features.screen_reader_support {
                supported_features.push(AccessibilityFeature::ScreenReader);
            } else {
                missing_features.push(AccessibilityFeature::ScreenReader);
            }
        }

        if profile.visual_settings.color_settings.colorblind_mode != ColorblindMode::None {
            if game.visual_features.colorblind_modes.contains(&profile.visual_settings.color_settings.colorblind_mode) {
                supported_features.push(AccessibilityFeature::Colorblind);
            } else if !game.visual_features.colorblind_modes.is_empty() {
                partial_features.push(AccessibilityFeature::Colorblind);
            } else {
                missing_features.push(AccessibilityFeature::Colorblind);
            }
        }

        // Check audio needs
        if profile.audio_settings.subtitles.enabled {
            if game.audio_features.subtitle_support {
                if game.audio_features.closed_captions {
                    supported_features.push(AccessibilityFeature::ClosedCaptions);
                } else {
                    supported_features.push(AccessibilityFeature::Subtitles);
                }
            } else {
                missing_features.push(AccessibilityFeature::Subtitles);
            }
        }

        if profile.audio_settings.visual_sound_cues.enabled {
            if game.audio_features.visual_sound_indicators {
                supported_features.push(AccessibilityFeature::VisualSoundCues);
            } else {
                missing_features.push(AccessibilityFeature::VisualSoundCues);
            }
        }

        // Check motor needs
        if profile.motor_settings.aim_assist.enabled {
            if game.motor_features.aim_assist_levels.contains(&profile.motor_settings.aim_assist.strength) {
                supported_features.push(AccessibilityFeature::AimAssist);
            } else if !game.motor_features.aim_assist_levels.is_empty() {
                partial_features.push(AccessibilityFeature::AimAssist);
            } else {
                missing_features.push(AccessibilityFeature::AimAssist);
            }
        }

        if profile.motor_settings.one_handed_mode.enabled {
            if game.motor_features.one_handed_modes.contains(&profile.motor_settings.one_handed_mode.hand) {
                supported_features.push(AccessibilityFeature::OneHandedMode);
            } else {
                missing_features.push(AccessibilityFeature::OneHandedMode);
            }
        }

        // Check controller support
        if let InputDeviceType::XboxAdaptiveController | InputDeviceType::PsAccessController =
            profile.motor_settings.input_device.device_type
        {
            if game.motor_features.adaptive_controller_support {
                supported_features.push(AccessibilityFeature::AdaptiveController);
            } else {
                missing_features.push(AccessibilityFeature::AdaptiveController);
            }
        }

        // Calculate score
        let total_needs = supported_features.len() + partial_features.len() + missing_features.len();
        let score = if total_needs > 0 {
            let supported_weight = supported_features.len() as f32;
            let partial_weight = partial_features.len() as f32 * 0.5;
            ((supported_weight + partial_weight) / total_needs as f32 * 100.0) as u8
        } else {
            100 // No specific needs = fully compatible
        };

        let recommendations = Self::generate_recommendations(&missing_features);

        Ok(CompatibilityReport {
            game_id,
            profile_id: profile.profile_id,
            score,
            supported_features,
            partial_features,
            missing_features,
            recommendations,
        })
    }

    fn generate_recommendations(missing: &[AccessibilityFeature]) -> Vec<String> {
        missing
            .iter()
            .map(|feature| match feature {
                AccessibilityFeature::ScreenReader => {
                    "Consider using external screen reader software".to_string()
                }
                AccessibilityFeature::Subtitles => {
                    "Check for community-made caption mods".to_string()
                }
                AccessibilityFeature::AimAssist => {
                    "Try using gyro aiming or mouse for better precision".to_string()
                }
                AccessibilityFeature::OneHandedMode => {
                    "Use Xbox Adaptive Controller or custom button mapping".to_string()
                }
                _ => format!("Feature {:?} not available", feature),
            })
            .collect()
    }

    /// Find games that support specific disabilities
    pub fn find_games_for_disability(&self, disability: DisabilityType) -> Vec<&GameMetadata> {
        self.games
            .values()
            .filter(|game| {
                match disability {
                    DisabilityType::Blind => game.visual_features.screen_reader_support,
                    DisabilityType::LowVision => {
                        game.visual_features.high_contrast_mode
                            || !game.visual_features.colorblind_modes.is_empty()
                    }
                    DisabilityType::Colorblind => !game.visual_features.colorblind_modes.is_empty(),
                    DisabilityType::Deaf | DisabilityType::HardOfHearing => {
                        game.audio_features.closed_captions
                            && game.audio_features.visual_sound_indicators
                    }
                    DisabilityType::LimitedMobility | DisabilityType::FineMotorDifficulty => {
                        game.motor_features.full_button_remapping
                            && !game.motor_features.aim_assist_levels.is_empty()
                    }
                    DisabilityType::OneHanded => !game.motor_features.one_handed_modes.is_empty(),
                    DisabilityType::Cognitive | DisabilityType::Adhd => {
                        game.cognitive_features.multiple_difficulty_levels
                            && game.cognitive_features.hint_system
                    }
                    _ => true,
                }
            })
            .collect()
    }
}

impl Default for GameManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Accessibility feature enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AccessibilityFeature {
    // Visual
    ScreenReader,
    Colorblind,
    HighContrast,
    ReduceMotion,

    // Audio
    Subtitles,
    ClosedCaptions,
    VisualSoundCues,
    MonoAudio,
    TtsChat,

    // Motor
    ButtonRemapping,
    HoldToToggle,
    AimAssist,
    OneHandedMode,
    AdaptiveController,
    SwitchAccess,
    CopilotMode,

    // Cognitive
    DifficultyOptions,
    SkipPuzzles,
    ObjectiveMarkers,
    HintSystem,
    ContentWarnings,
    SaveAnywhere,

    // Input
    EyeTracking,
    VoiceCommands,
    HeadTracking,
}

/// Compatibility report between profile and game
#[derive(Debug, Clone)]
pub struct CompatibilityReport {
    pub game_id: GameId,
    pub profile_id: ProfileId,
    pub score: u8,
    pub supported_features: Vec<AccessibilityFeature>,
    pub partial_features: Vec<AccessibilityFeature>,
    pub missing_features: Vec<AccessibilityFeature>,
    pub recommendations: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_game() -> GameMetadata {
        GameMetadata {
            game_id: uuid::Uuid::new_v4(),
            title: "Test Game".to_string(),
            version: "1.0.0".to_string(),
            accessibility_version: Some("1.0.0".to_string()),
            platforms: vec![Platform::Pc, Platform::Xbox, Platform::Playstation],
            visual_features: VisualFeatures {
                screen_reader_support: true,
                colorblind_modes: vec![ColorblindMode::Deuteranopia, ColorblindMode::Protanopia],
                high_contrast_mode: true,
                reduce_motion_option: true,
                disable_screen_shake: true,
                target_lock_indicator: true,
                customizable_crosshair: true,
                customizable_hud: true,
            },
            audio_features: AudioFeatures {
                subtitle_support: true,
                closed_captions: true,
                visual_sound_indicators: true,
                mono_audio_option: true,
                separate_volume_controls: vec![
                    "master".to_string(),
                    "music".to_string(),
                    "sfx".to_string(),
                    "voice".to_string(),
                ],
                tts_chat: true,
                tts_ui: true,
            },
            motor_features: MotorFeatures {
                full_button_remapping: true,
                hold_to_toggle_options: true,
                adjustable_sensitivity: true,
                adjustable_dead_zones: true,
                aim_assist_levels: vec![
                    AimAssistStrength::Low,
                    AimAssistStrength::Medium,
                    AimAssistStrength::High,
                    AimAssistStrength::Maximum,
                ],
                one_handed_modes: vec![Hand::Left, Hand::Right],
                adaptive_controller_support: true,
                switch_access: true,
                keyboard_mouse_on_console: true,
                touch_controls: false,
                gyro_aiming: true,
                copilot_mode: true,
            },
            cognitive_features: CognitiveFeatures {
                multiple_difficulty_levels: true,
                difficulty_options: vec![
                    DifficultyLevel::Story,
                    DifficultyLevel::Easy,
                    DifficultyLevel::Normal,
                    DifficultyLevel::Hard,
                ],
                granular_difficulty: true,
                skip_puzzles_option: true,
                skip_combat_option: false,
                objective_markers: true,
                navigation_assistance: true,
                tutorial_replay: true,
                hint_system: true,
                content_warnings: true,
                save_anywhere: true,
                auto_save: true,
                pause_cutscenes: true,
            },
            input_support: InputSupport {
                controllers: vec![
                    InputDeviceType::StandardController,
                    InputDeviceType::XboxAdaptiveController,
                    InputDeviceType::PsAccessController,
                ],
                eye_tracking: true,
                voice_commands: true,
                switch_access: true,
                head_tracking: false,
                mouth_controller: true,
            },
            accessibility_rating: Some(AccessibilityRating {
                overall_score: 95,
                visual_score: 95,
                audio_score: 98,
                motor_score: 92,
                cognitive_score: 90,
                certifications: vec![Certification::WiaPlatinum],
                reviewed_by: vec!["WIA".to_string()],
                last_review_date: Some("2025-01-15".to_string()),
            }),
        }
    }

    #[test]
    fn test_register_game() {
        let mut manager = GameManager::new();
        let game = create_test_game();
        let id = game.game_id;

        manager.register_game(game);
        assert!(manager.get_game(id).is_ok());
    }

    #[test]
    fn test_supports_feature() {
        let mut manager = GameManager::new();
        let game = create_test_game();
        let id = game.game_id;
        manager.register_game(game);

        assert!(manager.supports_feature(id, AccessibilityFeature::ScreenReader).unwrap());
        assert!(manager.supports_feature(id, AccessibilityFeature::ClosedCaptions).unwrap());
        assert!(manager.supports_feature(id, AccessibilityFeature::OneHandedMode).unwrap());
    }

    #[test]
    fn test_compatibility_report() {
        let mut manager = GameManager::new();
        let game = create_test_game();
        let game_id = game.game_id;
        manager.register_game(game);

        let mut profile = PlayerProfile::default();
        profile.visual_settings.screen_reader.enabled = true;
        profile.audio_settings.subtitles.enabled = true;

        let report = manager.get_compatibility(&profile, game_id).unwrap();
        assert!(report.score > 0);
        assert!(!report.supported_features.is_empty());
    }
}
