//! WIA Accessibility Cluster Extensions
//! 弘益人間 - Benefit All Humanity

use crate::protocol::matter::wia_cluster_ids;
use crate::types::{HapticPattern, InputModality, OutputModality};
use serde::{Deserialize, Serialize};

// ============================================================================
// Voice Command Cluster (0xWIA1)
// ============================================================================

/// Voice command definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommandDef {
    /// Primary command phrase
    pub command: String,
    /// Alternative phrases
    pub aliases: Vec<String>,
    /// Action to execute
    pub action: String,
    /// Confirmation phrase (TTS)
    pub confirmation: Option<String>,
    /// Required parameters
    pub parameters: Vec<String>,
}

/// Voice Command cluster state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommandCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// Supported commands
    pub commands: Vec<VoiceCommandDef>,
    /// Active language
    pub active_language: String,
    /// Wake word
    pub wake_word: Option<String>,
    /// Voice enabled
    pub enabled: bool,
}

impl Default for VoiceCommandCluster {
    fn default() -> Self {
        Self {
            cluster_id: wia_cluster_ids::VOICE_COMMAND,
            commands: Vec::new(),
            active_language: "ko-KR".to_string(),
            wake_word: None,
            enabled: true,
        }
    }
}

impl VoiceCommandCluster {
    /// Add a voice command
    pub fn add_command(&mut self, cmd: VoiceCommandDef) {
        self.commands.push(cmd);
    }

    /// Find command by text
    pub fn find_command(&self, text: &str) -> Option<&VoiceCommandDef> {
        let text_lower = text.to_lowercase();
        self.commands.iter().find(|cmd| {
            cmd.command.to_lowercase() == text_lower
                || cmd.aliases.iter().any(|a| a.to_lowercase() == text_lower)
        })
    }

    /// Get supported languages
    pub fn supported_languages() -> Vec<&'static str> {
        vec!["ko-KR", "en-US", "ja-JP", "zh-CN"]
    }
}

// ============================================================================
// Audio Feedback Cluster (0xWIA2)
// ============================================================================

/// Audio feedback cluster state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioFeedbackCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// Volume (0-100)
    pub volume: u8,
    /// TTS enabled
    pub tts_enabled: bool,
    /// TTS voice ID
    pub tts_voice: String,
    /// TTS rate (0.5-2.0)
    pub tts_rate: f32,
    /// TTS language
    pub tts_language: String,
    /// Muted
    pub muted: bool,
}

impl Default for AudioFeedbackCluster {
    fn default() -> Self {
        Self {
            cluster_id: wia_cluster_ids::AUDIO_FEEDBACK,
            volume: 70,
            tts_enabled: true,
            tts_voice: "ko-KR-Wavenet-A".to_string(),
            tts_rate: 1.0,
            tts_language: "ko-KR".to_string(),
            muted: false,
        }
    }
}

/// Audio feedback command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AudioFeedbackCommand {
    Speak { text: String, language: Option<String> },
    PlayTone { tone_id: String },
    SetVolume { level: u8 },
    Mute,
    Unmute,
    SetTTSRate { rate: f32 },
    SetTTSVoice { voice_id: String },
}

impl AudioFeedbackCluster {
    /// Process command
    pub fn process_command(&mut self, cmd: &AudioFeedbackCommand) -> AudioFeedbackResult {
        match cmd {
            AudioFeedbackCommand::Speak { text, language } => {
                if self.muted {
                    return AudioFeedbackResult::Muted;
                }
                AudioFeedbackResult::Speak {
                    text: text.clone(),
                    language: language.clone().unwrap_or_else(|| self.tts_language.clone()),
                    voice: self.tts_voice.clone(),
                    rate: self.tts_rate,
                    volume: self.volume,
                }
            }
            AudioFeedbackCommand::PlayTone { tone_id } => {
                if self.muted {
                    return AudioFeedbackResult::Muted;
                }
                AudioFeedbackResult::Tone {
                    tone_id: tone_id.clone(),
                    volume: self.volume,
                }
            }
            AudioFeedbackCommand::SetVolume { level } => {
                self.volume = (*level).min(100);
                AudioFeedbackResult::Success
            }
            AudioFeedbackCommand::Mute => {
                self.muted = true;
                AudioFeedbackResult::Success
            }
            AudioFeedbackCommand::Unmute => {
                self.muted = false;
                AudioFeedbackResult::Success
            }
            AudioFeedbackCommand::SetTTSRate { rate } => {
                self.tts_rate = rate.clamp(0.5, 2.0);
                AudioFeedbackResult::Success
            }
            AudioFeedbackCommand::SetTTSVoice { voice_id } => {
                self.tts_voice = voice_id.clone();
                AudioFeedbackResult::Success
            }
        }
    }
}

/// Audio feedback result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AudioFeedbackResult {
    Success,
    Muted,
    Speak {
        text: String,
        language: String,
        voice: String,
        rate: f32,
        volume: u8,
    },
    Tone {
        tone_id: String,
        volume: u8,
    },
}

// ============================================================================
// Visual Feedback Cluster (0xWIA3)
// ============================================================================

/// LED pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LEDPattern {
    Solid,
    Blink,
    Pulse,
    Rainbow,
    Breathing,
}

/// RGB color
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct RGB {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl RGB {
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }

    pub fn red() -> Self {
        Self::new(255, 0, 0)
    }

    pub fn green() -> Self {
        Self::new(0, 255, 0)
    }

    pub fn blue() -> Self {
        Self::new(0, 0, 255)
    }

    pub fn white() -> Self {
        Self::new(255, 255, 255)
    }
}

/// Visual feedback cluster state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualFeedbackCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// LED supported
    pub led_supported: bool,
    /// Screen supported
    pub screen_supported: bool,
    /// High contrast enabled
    pub high_contrast_enabled: bool,
    /// Large text enabled
    pub large_text_enabled: bool,
    /// Current LED color
    pub led_color: RGB,
    /// Current LED pattern
    pub led_pattern: LEDPattern,
    /// LED brightness (0-100)
    pub led_brightness: u8,
}

impl Default for VisualFeedbackCluster {
    fn default() -> Self {
        Self {
            cluster_id: wia_cluster_ids::VISUAL_FEEDBACK,
            led_supported: true,
            screen_supported: false,
            high_contrast_enabled: false,
            large_text_enabled: false,
            led_color: RGB::default(),
            led_pattern: LEDPattern::Solid,
            led_brightness: 80,
        }
    }
}

/// Visual feedback command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum VisualFeedbackCommand {
    SetLED { color: RGB, pattern: LEDPattern },
    SetLEDBrightness { brightness: u8 },
    DisplayMessage { title: String, body: String },
    Flash { pattern: FlashPattern, duration_ms: u32 },
    SetHighContrast { enabled: bool },
    SetLargeText { enabled: bool },
}

/// Flash pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlashPattern {
    None,
    Pulse,
    Blink,
    Strobe,
}

impl VisualFeedbackCluster {
    /// Process command
    pub fn process_command(&mut self, cmd: &VisualFeedbackCommand) {
        match cmd {
            VisualFeedbackCommand::SetLED { color, pattern } => {
                self.led_color = *color;
                self.led_pattern = *pattern;
            }
            VisualFeedbackCommand::SetLEDBrightness { brightness } => {
                self.led_brightness = (*brightness).min(100);
            }
            VisualFeedbackCommand::SetHighContrast { enabled } => {
                self.high_contrast_enabled = *enabled;
            }
            VisualFeedbackCommand::SetLargeText { enabled } => {
                self.large_text_enabled = *enabled;
            }
            _ => {}
        }
    }
}

// ============================================================================
// Haptic Feedback Cluster (0xWIA4)
// ============================================================================

/// Haptic feedback cluster state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticFeedbackCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// Haptic supported
    pub haptic_supported: bool,
    /// Default intensity (0-100)
    pub default_intensity: u8,
    /// Enabled
    pub enabled: bool,
}

impl Default for HapticFeedbackCluster {
    fn default() -> Self {
        Self {
            cluster_id: wia_cluster_ids::HAPTIC_FEEDBACK,
            haptic_supported: true,
            default_intensity: 50,
            enabled: true,
        }
    }
}

/// Haptic feedback command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HapticFeedbackCommand {
    Vibrate { pattern: HapticPattern, intensity: Option<u8> },
    SetDefaultIntensity { intensity: u8 },
    Enable,
    Disable,
}

impl HapticFeedbackCluster {
    /// Process command
    pub fn process_command(&mut self, cmd: &HapticFeedbackCommand) -> HapticFeedbackResult {
        match cmd {
            HapticFeedbackCommand::Vibrate { pattern, intensity } => {
                if !self.enabled {
                    return HapticFeedbackResult::Disabled;
                }
                HapticFeedbackResult::Vibrate {
                    pattern: *pattern,
                    intensity: intensity.unwrap_or(self.default_intensity),
                }
            }
            HapticFeedbackCommand::SetDefaultIntensity { intensity } => {
                self.default_intensity = (*intensity).min(100);
                HapticFeedbackResult::Success
            }
            HapticFeedbackCommand::Enable => {
                self.enabled = true;
                HapticFeedbackResult::Success
            }
            HapticFeedbackCommand::Disable => {
                self.enabled = false;
                HapticFeedbackResult::Success
            }
        }
    }
}

/// Haptic feedback result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HapticFeedbackResult {
    Success,
    Disabled,
    Vibrate {
        pattern: HapticPattern,
        intensity: u8,
    },
}

// ============================================================================
// Switch Access Cluster (0xWIA5)
// ============================================================================

/// Switch type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SwitchType {
    SinglePress,
    DoublePress,
    LongPress,
    Toggle,
}

/// Switch mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchMapping {
    /// Switch ID
    pub switch_id: String,
    /// Switch type
    pub switch_type: SwitchType,
    /// Mapped action
    pub action: String,
    /// Target device/zone ID
    pub target_id: Option<String>,
}

/// Switch Access cluster state
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SwitchAccessCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// Switch mappings
    pub mappings: Vec<SwitchMapping>,
    /// Enabled
    pub enabled: bool,
    /// Debounce time (ms)
    pub debounce_ms: u32,
}

impl SwitchAccessCluster {
    pub fn new() -> Self {
        Self {
            cluster_id: wia_cluster_ids::SWITCH_ACCESS,
            mappings: Vec::new(),
            enabled: true,
            debounce_ms: 50,
        }
    }

    /// Add switch mapping
    pub fn add_mapping(&mut self, mapping: SwitchMapping) {
        self.mappings.push(mapping);
    }

    /// Find mapping for switch event
    pub fn find_mapping(&self, switch_id: &str, switch_type: SwitchType) -> Option<&SwitchMapping> {
        self.mappings.iter().find(|m| {
            m.switch_id == switch_id && m.switch_type == switch_type
        })
    }
}

// ============================================================================
// Dwell Control Cluster (0xWIA6)
// ============================================================================

/// Dwell Control cluster state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellControlCluster {
    /// Cluster ID
    pub cluster_id: u16,
    /// Dwell time (ms)
    pub dwell_time_ms: u32,
    /// Visual feedback for dwell progress
    pub show_progress: bool,
    /// Audio feedback on activation
    pub audio_on_activate: bool,
    /// Enabled
    pub enabled: bool,
}

impl Default for DwellControlCluster {
    fn default() -> Self {
        Self {
            cluster_id: wia_cluster_ids::DWELL_CONTROL,
            dwell_time_ms: 1000,
            show_progress: true,
            audio_on_activate: true,
            enabled: true,
        }
    }
}

// ============================================================================
// Accessibility Profile
// ============================================================================

/// Complete accessibility profile for a device
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceAccessibilityProfile {
    /// Voice command cluster
    pub voice_command: Option<VoiceCommandCluster>,
    /// Audio feedback cluster
    pub audio_feedback: Option<AudioFeedbackCluster>,
    /// Visual feedback cluster
    pub visual_feedback: Option<VisualFeedbackCluster>,
    /// Haptic feedback cluster
    pub haptic_feedback: Option<HapticFeedbackCluster>,
    /// Switch access cluster
    pub switch_access: Option<SwitchAccessCluster>,
    /// Dwell control cluster
    pub dwell_control: Option<DwellControlCluster>,
}

impl DeviceAccessibilityProfile {
    /// Create with all features enabled
    pub fn full() -> Self {
        Self {
            voice_command: Some(VoiceCommandCluster::default()),
            audio_feedback: Some(AudioFeedbackCluster::default()),
            visual_feedback: Some(VisualFeedbackCluster::default()),
            haptic_feedback: Some(HapticFeedbackCluster::default()),
            switch_access: Some(SwitchAccessCluster::new()),
            dwell_control: Some(DwellControlCluster::default()),
        }
    }

    /// Create with basic features
    pub fn basic() -> Self {
        Self {
            voice_command: Some(VoiceCommandCluster::default()),
            audio_feedback: Some(AudioFeedbackCluster::default()),
            visual_feedback: Some(VisualFeedbackCluster::default()),
            ..Default::default()
        }
    }

    /// Get supported input modalities
    pub fn supported_inputs(&self) -> Vec<InputModality> {
        let mut inputs = vec![InputModality::Touch];

        if self.voice_command.is_some() {
            inputs.push(InputModality::Voice);
        }
        if self.switch_access.is_some() {
            inputs.push(InputModality::Switch);
        }
        if self.dwell_control.is_some() {
            inputs.push(InputModality::Gaze);
        }

        inputs
    }

    /// Get supported output modalities
    pub fn supported_outputs(&self) -> Vec<OutputModality> {
        let mut outputs = Vec::new();

        if self.audio_feedback.is_some() {
            outputs.push(OutputModality::AudioTts);
            outputs.push(OutputModality::AudioTone);
        }
        if self.visual_feedback.is_some() {
            outputs.push(OutputModality::VisualLed);
            outputs.push(OutputModality::VisualScreen);
        }
        if self.haptic_feedback.is_some() {
            outputs.push(OutputModality::Haptic);
        }

        outputs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voice_command_cluster() {
        let mut cluster = VoiceCommandCluster::default();

        cluster.add_command(VoiceCommandDef {
            command: "불 켜".to_string(),
            aliases: vec!["조명 켜".to_string()],
            action: "on".to_string(),
            confirmation: Some("조명을 켰습니다".to_string()),
            parameters: vec![],
        });

        assert!(cluster.find_command("불 켜").is_some());
        assert!(cluster.find_command("조명 켜").is_some());
        assert!(cluster.find_command("불 꺼").is_none());
    }

    #[test]
    fn test_audio_feedback_cluster() {
        let mut cluster = AudioFeedbackCluster::default();

        let result = cluster.process_command(&AudioFeedbackCommand::Speak {
            text: "안녕하세요".to_string(),
            language: None,
        });

        match result {
            AudioFeedbackResult::Speak { language, .. } => {
                assert_eq!(language, "ko-KR");
            }
            _ => panic!("Expected Speak result"),
        }

        // Test mute
        cluster.process_command(&AudioFeedbackCommand::Mute);
        let result = cluster.process_command(&AudioFeedbackCommand::Speak {
            text: "test".to_string(),
            language: None,
        });
        assert!(matches!(result, AudioFeedbackResult::Muted));
    }

    #[test]
    fn test_accessibility_profile() {
        let profile = DeviceAccessibilityProfile::full();

        let inputs = profile.supported_inputs();
        assert!(inputs.contains(&InputModality::Voice));
        assert!(inputs.contains(&InputModality::Switch));

        let outputs = profile.supported_outputs();
        assert!(outputs.contains(&OutputModality::AudioTts));
        assert!(outputs.contains(&OutputModality::Haptic));
    }
}
