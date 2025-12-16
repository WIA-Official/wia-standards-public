//! Profile and Settings Validation
//! 弘益人間 - Gaming for Everyone

use crate::error::{GameError, Result};
use crate::types::*;

/// Validator for player profiles and settings
pub struct Validator;

impl Validator {
    /// Validate a complete player profile
    pub fn validate_profile(profile: &PlayerProfile) -> Result<()> {
        Self::validate_version(&profile.version)?;
        Self::validate_visual_settings(&profile.visual_settings)?;
        Self::validate_audio_settings(&profile.audio_settings)?;
        Self::validate_motor_settings(&profile.motor_settings)?;
        Self::validate_cognitive_settings(&profile.cognitive_settings)?;
        Ok(())
    }

    /// Validate version string
    pub fn validate_version(version: &str) -> Result<()> {
        let parts: Vec<&str> = version.split('.').collect();
        if parts.len() != 3 {
            return Err(GameError::ValidationError(format!(
                "Invalid version format: {}. Expected semver (e.g., 1.0.0)",
                version
            )));
        }

        for part in parts {
            if part.parse::<u32>().is_err() {
                return Err(GameError::ValidationError(format!(
                    "Invalid version number: {}",
                    part
                )));
            }
        }

        Ok(())
    }

    /// Validate visual settings
    pub fn validate_visual_settings(settings: &VisualSettings) -> Result<()> {
        // Screen reader speed
        if settings.screen_reader.speed < 0.5 || settings.screen_reader.speed > 3.0 {
            return Err(GameError::ValidationError(
                "Screen reader speed must be between 0.5 and 3.0".to_string(),
            ));
        }

        // Magnification level
        if settings.magnification.level < 1.0 || settings.magnification.level > 10.0 {
            return Err(GameError::ValidationError(
                "Magnification level must be between 1.0 and 10.0".to_string(),
            ));
        }

        // Contrast level
        if settings.color_settings.contrast_level < 1.0
            || settings.color_settings.contrast_level > 3.0
        {
            return Err(GameError::ValidationError(
                "Contrast level must be between 1.0 and 3.0".to_string(),
            ));
        }

        // Text size multiplier
        if settings.text_settings.size_multiplier < 0.5
            || settings.text_settings.size_multiplier > 3.0
        {
            return Err(GameError::ValidationError(
                "Text size multiplier must be between 0.5 and 3.0".to_string(),
            ));
        }

        // Validate color formats
        Self::validate_color(&settings.target_indicators.enemy_highlight_color)?;
        Self::validate_color(&settings.target_indicators.ally_highlight_color)?;
        Self::validate_color(&settings.target_indicators.item_highlight_color)?;

        Ok(())
    }

    /// Validate audio settings
    pub fn validate_audio_settings(settings: &AudioSettings) -> Result<()> {
        // Background opacity
        if settings.subtitles.background_opacity < 0.0
            || settings.subtitles.background_opacity > 1.0
        {
            return Err(GameError::ValidationError(
                "Subtitle background opacity must be between 0.0 and 1.0".to_string(),
            ));
        }

        // Haptic intensity
        if settings.haptic_audio.intensity < 0.0 || settings.haptic_audio.intensity > 1.0 {
            return Err(GameError::ValidationError(
                "Haptic intensity must be between 0.0 and 1.0".to_string(),
            ));
        }

        // Mono audio balance
        if settings.mono_audio.balance < -1.0 || settings.mono_audio.balance > 1.0 {
            return Err(GameError::ValidationError(
                "Mono audio balance must be between -1.0 and 1.0".to_string(),
            ));
        }

        // Volume controls
        Self::validate_volume(settings.volume_controls.master, "Master volume")?;
        Self::validate_volume(settings.volume_controls.music, "Music volume")?;
        Self::validate_volume(settings.volume_controls.sfx, "SFX volume")?;
        Self::validate_volume(settings.volume_controls.voice, "Voice volume")?;
        Self::validate_volume(settings.volume_controls.ambient, "Ambient volume")?;
        Self::validate_volume(settings.volume_controls.ui, "UI volume")?;

        // TTS speed
        if settings.tts_for_chat.speed < 0.5 || settings.tts_for_chat.speed > 2.0 {
            return Err(GameError::ValidationError(
                "TTS speed must be between 0.5 and 2.0".to_string(),
            ));
        }

        // Validate colors
        Self::validate_color(&settings.subtitles.text_color)?;
        Self::validate_color(&settings.subtitles.background_color)?;

        Ok(())
    }

    /// Validate motor settings
    pub fn validate_motor_settings(settings: &MotorSettings) -> Result<()> {
        // Button timing
        if settings.button_behavior.tap_timing_ms < 100
            || settings.button_behavior.tap_timing_ms > 2000
        {
            return Err(GameError::ValidationError(
                "Tap timing must be between 100ms and 2000ms".to_string(),
            ));
        }

        if settings.button_behavior.hold_timing_ms < 200
            || settings.button_behavior.hold_timing_ms > 5000
        {
            return Err(GameError::ValidationError(
                "Hold timing must be between 200ms and 5000ms".to_string(),
            ));
        }

        // Stick sensitivity
        Self::validate_sensitivity(settings.stick_settings.left_sensitivity, "Left stick")?;
        Self::validate_sensitivity(settings.stick_settings.right_sensitivity, "Right stick")?;

        // Dead zones
        Self::validate_dead_zone(settings.stick_settings.left_dead_zone, "Left stick")?;
        Self::validate_dead_zone(settings.stick_settings.right_dead_zone, "Right stick")?;

        // Sequential input timeout
        if settings.sequential_inputs.timeout_ms < 500
            || settings.sequential_inputs.timeout_ms > 10000
        {
            return Err(GameError::ValidationError(
                "Sequential input timeout must be between 500ms and 10000ms".to_string(),
            ));
        }

        // QTE time multiplier
        if settings.qte_settings.time_multiplier < 1.0
            || settings.qte_settings.time_multiplier > 10.0
        {
            return Err(GameError::ValidationError(
                "QTE time multiplier must be between 1.0 and 10.0".to_string(),
            ));
        }

        Ok(())
    }

    /// Validate cognitive settings
    pub fn validate_cognitive_settings(settings: &CognitiveSettings) -> Result<()> {
        // Reminder frequency
        if settings.guidance.reminder_frequency_sec < 10
            || settings.guidance.reminder_frequency_sec > 600
        {
            return Err(GameError::ValidationError(
                "Reminder frequency must be between 10 and 600 seconds".to_string(),
            ));
        }

        // Hint delay
        if settings.guidance.hint_delay_sec > 300 {
            return Err(GameError::ValidationError(
                "Hint delay must be at most 300 seconds".to_string(),
            ));
        }

        // Warning duration
        if settings.content_warnings.warning_duration_sec < 1
            || settings.content_warnings.warning_duration_sec > 30
        {
            return Err(GameError::ValidationError(
                "Warning duration must be between 1 and 30 seconds".to_string(),
            ));
        }

        // Break interval
        if settings.focus_aids.break_interval_min < 15
            || settings.focus_aids.break_interval_min > 180
        {
            return Err(GameError::ValidationError(
                "Break interval must be between 15 and 180 minutes".to_string(),
            ));
        }

        Ok(())
    }

    /// Validate a hex color string
    fn validate_color(color: &str) -> Result<()> {
        if !color.starts_with('#') || color.len() != 7 {
            return Err(GameError::ValidationError(format!(
                "Invalid color format: {}. Expected #RRGGBB",
                color
            )));
        }

        if !color[1..].chars().all(|c| c.is_ascii_hexdigit()) {
            return Err(GameError::ValidationError(format!(
                "Invalid hex color: {}",
                color
            )));
        }

        Ok(())
    }

    /// Validate volume value
    fn validate_volume(volume: f32, name: &str) -> Result<()> {
        if volume < 0.0 || volume > 1.0 {
            return Err(GameError::ValidationError(format!(
                "{} must be between 0.0 and 1.0",
                name
            )));
        }
        Ok(())
    }

    /// Validate sensitivity value
    fn validate_sensitivity(value: f32, name: &str) -> Result<()> {
        if value < 0.1 || value > 3.0 {
            return Err(GameError::ValidationError(format!(
                "{} sensitivity must be between 0.1 and 3.0",
                name
            )));
        }
        Ok(())
    }

    /// Validate dead zone value
    fn validate_dead_zone(value: f32, name: &str) -> Result<()> {
        if value < 0.0 || value > 0.5 {
            return Err(GameError::ValidationError(format!(
                "{} dead zone must be between 0.0 and 0.5",
                name
            )));
        }
        Ok(())
    }

    /// Validate controller config
    pub fn validate_controller_config(config: &ControllerConfig) -> Result<()> {
        // Validate button actions
        for (button, action) in &config.button_mapping {
            if action.action.is_empty() {
                return Err(GameError::ValidationError(format!(
                    "Button '{}' has empty action",
                    button
                )));
            }

            if action.sensitivity < 0.0 || action.sensitivity > 2.0 {
                return Err(GameError::ValidationError(format!(
                    "Button '{}' sensitivity must be between 0.0 and 2.0",
                    button
                )));
            }
        }

        // Validate combos
        for combo in &config.combo_mapping {
            if combo.buttons.len() < 2 {
                return Err(GameError::ValidationError(
                    "Combo must have at least 2 buttons".to_string(),
                ));
            }

            if combo.action.is_empty() {
                return Err(GameError::ValidationError(
                    "Combo has empty action".to_string(),
                ));
            }
        }

        // Validate macros
        for macro_action in &config.macro_mapping {
            if macro_action.name.is_empty() {
                return Err(GameError::ValidationError(
                    "Macro has empty name".to_string(),
                ));
            }

            if macro_action.sequence.is_empty() {
                return Err(GameError::ValidationError(format!(
                    "Macro '{}' has empty sequence",
                    macro_action.name
                )));
            }
        }

        Ok(())
    }

    /// Validate preset
    pub fn validate_preset(preset: &AccessibilityPreset) -> Result<()> {
        if preset.name.is_empty() {
            return Err(GameError::ValidationError(
                "Preset name cannot be empty".to_string(),
            ));
        }

        if preset.name.len() > 50 {
            return Err(GameError::ValidationError(
                "Preset name must be at most 50 characters".to_string(),
            ));
        }

        if let Some(ref visual) = preset.settings_override.visual_settings {
            Self::validate_visual_settings(visual)?;
        }

        if let Some(ref audio) = preset.settings_override.audio_settings {
            Self::validate_audio_settings(audio)?;
        }

        if let Some(ref motor) = preset.settings_override.motor_settings {
            Self::validate_motor_settings(motor)?;
        }

        if let Some(ref cognitive) = preset.settings_override.cognitive_settings {
            Self::validate_cognitive_settings(cognitive)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_default_profile() {
        let profile = PlayerProfile::default();
        assert!(Validator::validate_profile(&profile).is_ok());
    }

    #[test]
    fn test_validate_version() {
        assert!(Validator::validate_version("1.0.0").is_ok());
        assert!(Validator::validate_version("2.10.5").is_ok());
        assert!(Validator::validate_version("1.0").is_err());
        assert!(Validator::validate_version("a.b.c").is_err());
    }

    #[test]
    fn test_validate_color() {
        assert!(Validator::validate_color("#FF0000").is_ok());
        assert!(Validator::validate_color("#00ff00").is_ok());
        assert!(Validator::validate_color("FF0000").is_err());
        assert!(Validator::validate_color("#GG0000").is_err());
    }

    #[test]
    fn test_validate_invalid_screen_reader_speed() {
        let mut settings = VisualSettings::default();
        settings.screen_reader.speed = 5.0;
        assert!(Validator::validate_visual_settings(&settings).is_err());
    }

    #[test]
    fn test_validate_invalid_volume() {
        let mut settings = AudioSettings::default();
        settings.volume_controls.master = 1.5;
        assert!(Validator::validate_audio_settings(&settings).is_err());
    }

    #[test]
    fn test_validate_invalid_dead_zone() {
        let mut settings = MotorSettings::default();
        settings.stick_settings.left_dead_zone = 0.8;
        assert!(Validator::validate_motor_settings(&settings).is_err());
    }
}
