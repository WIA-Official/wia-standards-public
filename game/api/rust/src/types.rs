//! WIA Game Type Definitions
//! 弘益人間 - Gaming for Everyone

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// Type aliases
pub type ProfileId = Uuid;
pub type PresetId = Uuid;
pub type GameId = Uuid;
pub type ConfigId = Uuid;

/// Schema version
pub const SCHEMA_VERSION: &str = "1.0.0";

// ============================================================================
// Player Profile
// ============================================================================

/// Complete player accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayerProfile {
    pub profile_id: ProfileId,
    pub version: String,
    #[serde(default)]
    pub created_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub player_info: PlayerInfo,
    #[serde(default)]
    pub disability_context: DisabilityContext,
    #[serde(default)]
    pub visual_settings: VisualSettings,
    #[serde(default)]
    pub audio_settings: AudioSettings,
    #[serde(default)]
    pub motor_settings: MotorSettings,
    #[serde(default)]
    pub cognitive_settings: CognitiveSettings,
    #[serde(default)]
    pub presets: Vec<AccessibilityPreset>,
}

impl Default for PlayerProfile {
    fn default() -> Self {
        Self {
            profile_id: Uuid::new_v4(),
            version: SCHEMA_VERSION.to_string(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            player_info: PlayerInfo::default(),
            disability_context: DisabilityContext::default(),
            visual_settings: VisualSettings::default(),
            audio_settings: AudioSettings::default(),
            motor_settings: MotorSettings::default(),
            cognitive_settings: CognitiveSettings::default(),
            presets: Vec::new(),
        }
    }
}

/// Player information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PlayerInfo {
    #[serde(default)]
    pub display_name: Option<String>,
    #[serde(default = "default_language")]
    pub language: String,
    #[serde(default)]
    pub region: Option<String>,
}

fn default_language() -> String {
    "en-US".to_string()
}

// ============================================================================
// Disability Context
// ============================================================================

/// Disability context information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DisabilityContext {
    #[serde(default)]
    pub primary_disabilities: Vec<DisabilityType>,
    #[serde(default)]
    pub assistive_technologies: Vec<AssistiveTechnology>,
    #[serde(default)]
    pub input_devices: Vec<InputDeviceType>,
    #[serde(default)]
    pub notes: Option<String>,
}

/// Disability types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisabilityType {
    Blind,
    LowVision,
    Colorblind,
    Deaf,
    HardOfHearing,
    LimitedMobility,
    FineMotorDifficulty,
    OneHanded,
    Cognitive,
    Adhd,
    Dyslexia,
    Epilepsy,
    ChronicPain,
    Fatigue,
    Anxiety,
    Other,
}

/// Assistive technologies
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssistiveTechnology {
    ScreenReader,
    Magnifier,
    EyeTracker,
    SwitchAccess,
    VoiceControl,
    HeadTracking,
    MouthController,
    FootPedals,
    BrailleDisplay,
    CochlearImplant,
    HearingAid,
    Wheelchair,
    Other,
}

/// Input device types
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputDeviceType {
    #[default]
    StandardController,
    XboxAdaptiveController,
    PsAccessController,
    HoriFlex,
    Quadstick,
    EyeTracker,
    SwitchArray,
    KeyboardMouse,
    TouchScreen,
    Custom,
}

// ============================================================================
// Visual Settings
// ============================================================================

/// Visual accessibility settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualSettings {
    #[serde(default)]
    pub screen_reader: ScreenReaderSettings,
    #[serde(default)]
    pub magnification: MagnificationSettings,
    #[serde(default)]
    pub color_settings: ColorSettings,
    #[serde(default)]
    pub text_settings: TextSettings,
    #[serde(default)]
    pub ui_settings: UiSettings,
    #[serde(default)]
    pub target_indicators: TargetIndicatorSettings,
}

/// Screen reader settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenReaderSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub voice: Option<String>,
    #[serde(default = "default_speed")]
    pub speed: f32,
    #[serde(default)]
    pub verbosity: Verbosity,
    #[serde(default = "default_true")]
    pub read_button_prompts: bool,
    #[serde(default = "default_true")]
    pub read_menu_items: bool,
}

impl Default for ScreenReaderSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            voice: None,
            speed: 1.0,
            verbosity: Verbosity::Normal,
            read_button_prompts: true,
            read_menu_items: true,
        }
    }
}

/// Verbosity levels
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verbosity {
    Minimal,
    #[default]
    Normal,
    High,
    Verbose,
}

/// Magnification settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MagnificationSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_magnification")]
    pub level: f32,
    #[serde(default = "default_true")]
    pub follow_focus: bool,
    #[serde(default = "default_true")]
    pub smooth_scrolling: bool,
}

impl Default for MagnificationSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            level: 2.0,
            follow_focus: true,
            smooth_scrolling: true,
        }
    }
}

/// Color settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorSettings {
    #[serde(default)]
    pub colorblind_mode: ColorblindMode,
    #[serde(default)]
    pub high_contrast: bool,
    #[serde(default = "default_speed")]
    pub contrast_level: f32,
    #[serde(default)]
    pub custom_color_mapping: std::collections::HashMap<String, String>,
}

impl Default for ColorSettings {
    fn default() -> Self {
        Self {
            colorblind_mode: ColorblindMode::None,
            high_contrast: false,
            contrast_level: 1.0,
            custom_color_mapping: std::collections::HashMap::new(),
        }
    }
}

/// Colorblind modes
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorblindMode {
    #[default]
    None,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
    Custom,
}

/// Text settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextSettings {
    #[serde(default = "default_speed")]
    pub size_multiplier: f32,
    #[serde(default)]
    pub font_weight: FontWeight,
    #[serde(default)]
    pub dyslexia_friendly_font: bool,
    #[serde(default)]
    pub letter_spacing: f32,
    #[serde(default = "default_line_height")]
    pub line_height: f32,
}

impl Default for TextSettings {
    fn default() -> Self {
        Self {
            size_multiplier: 1.0,
            font_weight: FontWeight::Normal,
            dyslexia_friendly_font: false,
            letter_spacing: 0.0,
            line_height: 1.4,
        }
    }
}

/// Font weight
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FontWeight {
    #[default]
    Normal,
    Bold,
    ExtraBold,
}

/// UI settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UiSettings {
    #[serde(default)]
    pub reduce_motion: bool,
    #[serde(default)]
    pub reduce_transparency: bool,
    #[serde(default)]
    pub highlight_interactive: bool,
    #[serde(default)]
    pub focus_indicator_size: ElementSize,
    #[serde(default)]
    pub cursor_size: ElementSize,
    #[serde(default)]
    pub disable_parallax: bool,
    #[serde(default)]
    pub disable_screen_shake: bool,
}

impl Default for UiSettings {
    fn default() -> Self {
        Self {
            reduce_motion: false,
            reduce_transparency: false,
            highlight_interactive: false,
            focus_indicator_size: ElementSize::Medium,
            cursor_size: ElementSize::Medium,
            disable_parallax: false,
            disable_screen_shake: false,
        }
    }
}

/// Element size
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ElementSize {
    Small,
    #[default]
    Medium,
    Large,
    ExtraLarge,
}

/// Target indicator settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TargetIndicatorSettings {
    #[serde(default)]
    pub enemy_highlight: bool,
    #[serde(default = "default_red")]
    pub enemy_highlight_color: String,
    #[serde(default)]
    pub ally_highlight: bool,
    #[serde(default = "default_green")]
    pub ally_highlight_color: String,
    #[serde(default)]
    pub item_highlight: bool,
    #[serde(default = "default_yellow")]
    pub item_highlight_color: String,
    #[serde(default)]
    pub path_highlight: bool,
    #[serde(default)]
    pub interactable_highlight: bool,
}

impl Default for TargetIndicatorSettings {
    fn default() -> Self {
        Self {
            enemy_highlight: false,
            enemy_highlight_color: "#FF0000".to_string(),
            ally_highlight: false,
            ally_highlight_color: "#00FF00".to_string(),
            item_highlight: false,
            item_highlight_color: "#FFFF00".to_string(),
            path_highlight: false,
            interactable_highlight: false,
        }
    }
}

// ============================================================================
// Audio Settings
// ============================================================================

/// Audio accessibility settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AudioSettings {
    #[serde(default)]
    pub subtitles: SubtitleSettings,
    #[serde(default)]
    pub visual_sound_cues: VisualSoundCueSettings,
    #[serde(default)]
    pub haptic_audio: HapticAudioSettings,
    #[serde(default)]
    pub mono_audio: MonoAudioSettings,
    #[serde(default)]
    pub volume_controls: VolumeControls,
    #[serde(default)]
    pub tts_for_chat: TtsChatSettings,
}

/// Subtitle settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubtitleSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub style: SubtitleStyle,
    #[serde(default = "default_true")]
    pub speaker_identification: bool,
    #[serde(default = "default_true")]
    pub speaker_colors: bool,
    #[serde(default = "default_true")]
    pub sound_effects: bool,
    #[serde(default = "default_true")]
    pub music_indicators: bool,
    #[serde(default = "default_true")]
    pub directional_indicators: bool,
    #[serde(default = "default_opacity")]
    pub background_opacity: f32,
    #[serde(default)]
    pub text_size: ElementSize,
    #[serde(default = "default_white")]
    pub text_color: String,
    #[serde(default = "default_black")]
    pub background_color: String,
    #[serde(default)]
    pub position: SubtitlePosition,
}

impl Default for SubtitleSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            style: SubtitleStyle::FullCaptions,
            speaker_identification: true,
            speaker_colors: true,
            sound_effects: true,
            music_indicators: true,
            directional_indicators: true,
            background_opacity: 0.8,
            text_size: ElementSize::Medium,
            text_color: "#FFFFFF".to_string(),
            background_color: "#000000".to_string(),
            position: SubtitlePosition::Bottom,
        }
    }
}

/// Subtitle styles
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubtitleStyle {
    DialogueOnly,
    #[default]
    FullCaptions,
    Minimal,
    Verbose,
}

/// Subtitle position
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubtitlePosition {
    #[default]
    Bottom,
    Top,
    Custom,
}

/// Visual sound cue settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualSoundCueSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub style: VisualSoundCueStyle,
    #[serde(default)]
    pub position: SoundCuePosition,
    #[serde(default)]
    pub size: ElementSize,
    #[serde(default = "default_opacity")]
    pub opacity: f32,
    #[serde(default)]
    pub categories: SoundCueCategories,
}

impl Default for VisualSoundCueSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            style: VisualSoundCueStyle::ScreenEdge,
            position: SoundCuePosition::ScreenEdge,
            size: ElementSize::Medium,
            opacity: 0.8,
            categories: SoundCueCategories::default(),
        }
    }
}

/// Visual sound cue styles
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualSoundCueStyle {
    Radar,
    #[default]
    ScreenEdge,
    Compass,
    Icons,
}

/// Sound cue position
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SoundCuePosition {
    #[default]
    ScreenEdge,
    Minimap,
    Center,
    Custom,
}

/// Sound cue categories
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoundCueCategories {
    #[serde(default = "default_true")]
    pub footsteps: bool,
    #[serde(default = "default_true")]
    pub gunshots: bool,
    #[serde(default = "default_true")]
    pub explosions: bool,
    #[serde(default = "default_true")]
    pub voices: bool,
    #[serde(default = "default_true")]
    pub vehicles: bool,
    #[serde(default = "default_true")]
    pub environmental: bool,
    #[serde(default = "default_true")]
    pub alerts: bool,
    #[serde(default)]
    pub music: bool,
}

impl Default for SoundCueCategories {
    fn default() -> Self {
        Self {
            footsteps: true,
            gunshots: true,
            explosions: true,
            voices: true,
            vehicles: true,
            environmental: true,
            alerts: true,
            music: false,
        }
    }
}

/// Haptic audio settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticAudioSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_half")]
    pub intensity: f32,
    #[serde(default = "default_true")]
    pub directional: bool,
    #[serde(default = "default_true")]
    pub bass_to_haptic: bool,
}

impl Default for HapticAudioSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            intensity: 0.5,
            directional: true,
            bass_to_haptic: true,
        }
    }
}

/// Mono audio settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MonoAudioSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub balance: f32,
}

/// Volume controls
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VolumeControls {
    #[serde(default = "default_speed")]
    pub master: f32,
    #[serde(default = "default_volume_music")]
    pub music: f32,
    #[serde(default = "default_speed")]
    pub sfx: f32,
    #[serde(default = "default_speed")]
    pub voice: f32,
    #[serde(default = "default_half")]
    pub ambient: f32,
    #[serde(default = "default_opacity")]
    pub ui: f32,
}

impl Default for VolumeControls {
    fn default() -> Self {
        Self {
            master: 1.0,
            music: 0.7,
            sfx: 1.0,
            voice: 1.0,
            ambient: 0.5,
            ui: 0.8,
        }
    }
}

/// TTS chat settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TtsChatSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub voice: Option<String>,
    #[serde(default = "default_speed")]
    pub speed: f32,
    #[serde(default = "default_true")]
    pub read_player_names: bool,
    #[serde(default = "default_true")]
    pub filter_spam: bool,
}

impl Default for TtsChatSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            voice: None,
            speed: 1.0,
            read_player_names: true,
            filter_spam: true,
        }
    }
}

// ============================================================================
// Motor Settings
// ============================================================================

/// Motor accessibility settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorSettings {
    #[serde(default)]
    pub input_device: InputDeviceSettings,
    #[serde(default)]
    pub button_behavior: ButtonBehaviorSettings,
    #[serde(default)]
    pub stick_settings: StickSettings,
    #[serde(default)]
    pub aim_assist: AimAssistSettings,
    #[serde(default)]
    pub auto_actions: AutoActionSettings,
    #[serde(default)]
    pub one_handed_mode: OneHandedModeSettings,
    #[serde(default)]
    pub sequential_inputs: SequentialInputSettings,
    #[serde(default)]
    pub qte_settings: QteSettings,
    #[serde(default)]
    pub button_remapping: Option<ControllerConfig>,
}

/// Input device settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputDeviceSettings {
    #[serde(default)]
    pub device_type: InputDeviceType,
    #[serde(default)]
    pub profile_name: Option<String>,
    #[serde(default)]
    pub copilot_enabled: bool,
}

impl Default for InputDeviceSettings {
    fn default() -> Self {
        Self {
            device_type: InputDeviceType::StandardController,
            profile_name: None,
            copilot_enabled: false,
        }
    }
}

/// Button behavior settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonBehaviorSettings {
    #[serde(default)]
    pub hold_to_toggle: Vec<String>,
    #[serde(default = "default_tap_timing")]
    pub tap_timing_ms: u32,
    #[serde(default = "default_hold_timing")]
    pub hold_timing_ms: u32,
    #[serde(default = "default_repeat_rate")]
    pub repeat_rate_ms: u32,
    #[serde(default = "default_debounce")]
    pub debounce_ms: u32,
}

impl Default for ButtonBehaviorSettings {
    fn default() -> Self {
        Self {
            hold_to_toggle: Vec::new(),
            tap_timing_ms: 500,
            hold_timing_ms: 1000,
            repeat_rate_ms: 100,
            debounce_ms: 50,
        }
    }
}

/// Stick settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StickSettings {
    #[serde(default = "default_speed")]
    pub left_sensitivity: f32,
    #[serde(default = "default_speed")]
    pub right_sensitivity: f32,
    #[serde(default = "default_dead_zone")]
    pub left_dead_zone: f32,
    #[serde(default = "default_dead_zone")]
    pub right_dead_zone: f32,
    #[serde(default)]
    pub invert_y_look: bool,
    #[serde(default)]
    pub invert_y_move: bool,
    #[serde(default)]
    pub swap_sticks: bool,
    #[serde(default)]
    pub response_curve: ResponseCurve,
}

impl Default for StickSettings {
    fn default() -> Self {
        Self {
            left_sensitivity: 1.0,
            right_sensitivity: 1.0,
            left_dead_zone: 0.15,
            right_dead_zone: 0.15,
            invert_y_look: false,
            invert_y_move: false,
            swap_sticks: false,
            response_curve: ResponseCurve::Smooth,
        }
    }
}

/// Response curve
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseCurve {
    Linear,
    #[default]
    Smooth,
    Aggressive,
    Custom,
}

/// Aim assist settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AimAssistSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub strength: AimAssistStrength,
    #[serde(default)]
    pub auto_aim: bool,
    #[serde(default)]
    pub lock_on: bool,
    #[serde(default = "default_true")]
    pub sticky_aim: bool,
    #[serde(default)]
    pub snap_to_target: bool,
}

impl Default for AimAssistSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            strength: AimAssistStrength::Medium,
            auto_aim: false,
            lock_on: false,
            sticky_aim: true,
            snap_to_target: false,
        }
    }
}

/// Aim assist strength
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AimAssistStrength {
    Off,
    Low,
    #[default]
    Medium,
    High,
    Strong,
    Maximum,
}

/// Auto action settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AutoActionSettings {
    #[serde(default)]
    pub auto_run: bool,
    #[serde(default)]
    pub auto_climb: bool,
    #[serde(default)]
    pub auto_mantle: bool,
    #[serde(default)]
    pub auto_pickup: bool,
    #[serde(default)]
    pub auto_reload: bool,
    #[serde(default)]
    pub auto_switch_weapon: bool,
    #[serde(default)]
    pub auto_heal: bool,
    #[serde(default)]
    pub auto_sprint: bool,
}

/// One-handed mode settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OneHandedModeSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub hand: Hand,
    #[serde(default)]
    pub layout: OneHandedLayout,
    #[serde(default)]
    pub gyro_aim: bool,
}

impl Default for OneHandedModeSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            hand: Hand::Right,
            layout: OneHandedLayout::Compact,
            gyro_aim: false,
        }
    }
}

/// Hand preference
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    Left,
    #[default]
    Right,
}

/// One-handed layout
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OneHandedLayout {
    #[default]
    Compact,
    Extended,
    Custom,
}

/// Sequential input settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequentialInputSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_sequential_timeout")]
    pub timeout_ms: u32,
    #[serde(default = "default_true")]
    pub visual_indicator: bool,
}

impl Default for SequentialInputSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            timeout_ms: 2000,
            visual_indicator: true,
        }
    }
}

/// QTE settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QteSettings {
    #[serde(default)]
    pub auto_complete: bool,
    #[serde(default)]
    pub extended_time: bool,
    #[serde(default = "default_speed")]
    pub time_multiplier: f32,
    #[serde(default)]
    pub hold_instead_of_mash: bool,
}

impl Default for QteSettings {
    fn default() -> Self {
        Self {
            auto_complete: false,
            extended_time: false,
            time_multiplier: 1.0,
            hold_instead_of_mash: false,
        }
    }
}

// ============================================================================
// Cognitive Settings
// ============================================================================

/// Cognitive accessibility settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CognitiveSettings {
    #[serde(default)]
    pub difficulty: DifficultySettings,
    #[serde(default)]
    pub guidance: GuidanceSettings,
    #[serde(default)]
    pub simplification: SimplificationSettings,
    #[serde(default)]
    pub content_warnings: ContentWarningSettings,
    #[serde(default)]
    pub reading_aids: ReadingAidSettings,
    #[serde(default)]
    pub memory_aids: MemoryAidSettings,
    #[serde(default)]
    pub focus_aids: FocusAidSettings,
}

/// Difficulty settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DifficultySettings {
    #[serde(default)]
    pub combat_difficulty: DifficultyLevel,
    #[serde(default)]
    pub puzzle_difficulty: DifficultyLevel,
    #[serde(default)]
    pub time_pressure: TimePressure,
    #[serde(default)]
    pub auto_complete_qte: bool,
    #[serde(default)]
    pub skip_minigames: bool,
    #[serde(default)]
    pub invincibility_mode: bool,
}

impl Default for DifficultySettings {
    fn default() -> Self {
        Self {
            combat_difficulty: DifficultyLevel::Normal,
            puzzle_difficulty: DifficultyLevel::Normal,
            time_pressure: TimePressure::Normal,
            auto_complete_qte: false,
            skip_minigames: false,
            invincibility_mode: false,
        }
    }
}

/// Difficulty level
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DifficultyLevel {
    Story,
    Easy,
    #[default]
    Normal,
    Hard,
    Custom,
}

/// Time pressure level
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimePressure {
    None,
    Relaxed,
    #[default]
    Normal,
    Intense,
}

/// Guidance settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GuidanceSettings {
    #[serde(default = "default_true")]
    pub objective_reminders: bool,
    #[serde(default = "default_reminder_frequency")]
    pub reminder_frequency_sec: u32,
    #[serde(default = "default_true")]
    pub waypoint_guidance: bool,
    #[serde(default)]
    pub path_visualization: bool,
    #[serde(default)]
    pub hint_system: HintSystem,
    #[serde(default = "default_hint_delay")]
    pub hint_delay_sec: u32,
    #[serde(default = "default_true")]
    pub tutorial_repeat: bool,
    #[serde(default = "default_true")]
    pub control_reminders: bool,
}

impl Default for GuidanceSettings {
    fn default() -> Self {
        Self {
            objective_reminders: true,
            reminder_frequency_sec: 60,
            waypoint_guidance: true,
            path_visualization: false,
            hint_system: HintSystem::OnRequest,
            hint_delay_sec: 30,
            tutorial_repeat: true,
            control_reminders: true,
        }
    }
}

/// Hint system
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HintSystem {
    Off,
    #[default]
    OnRequest,
    Progressive,
    Automatic,
}

/// Simplification settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimplificationSettings {
    #[serde(default)]
    pub simplified_controls: bool,
    #[serde(default)]
    pub reduced_button_combos: bool,
    #[serde(default)]
    pub reduced_hud: bool,
    #[serde(default)]
    pub minimal_ui: bool,
    #[serde(default)]
    pub reduced_visual_effects: bool,
    #[serde(default)]
    pub reduced_enemy_count: bool,
    #[serde(default)]
    pub simplified_inventory: bool,
    #[serde(default)]
    pub auto_sort_inventory: bool,
}

/// Content warning settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentWarningSettings {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub categories: Vec<ContentWarningCategory>,
    #[serde(default)]
    pub auto_skip: bool,
    #[serde(default = "default_warning_duration")]
    pub warning_duration_sec: u32,
}

impl Default for ContentWarningSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            categories: Vec::new(),
            auto_skip: false,
            warning_duration_sec: 5,
        }
    }
}

/// Content warning categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentWarningCategory {
    Violence,
    Gore,
    Blood,
    Flashing,
    Jumpscares,
    Spiders,
    Heights,
    Drowning,
    Fire,
    Needles,
    ConfinedSpaces,
    LoudSounds,
    BodyHorror,
    ChildHarm,
    SelfHarm,
    SubstanceAbuse,
}

/// Reading aid settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReadingAidSettings {
    #[serde(default)]
    pub text_to_speech_ui: bool,
    #[serde(default)]
    pub text_to_speech_lore: bool,
    #[serde(default = "default_true")]
    pub reading_speed_control: bool,
    #[serde(default = "default_true")]
    pub pause_during_text: bool,
    #[serde(default)]
    pub highlight_keywords: bool,
    #[serde(default)]
    pub simplified_language: bool,
}

impl Default for ReadingAidSettings {
    fn default() -> Self {
        Self {
            text_to_speech_ui: false,
            text_to_speech_lore: false,
            reading_speed_control: true,
            pause_during_text: true,
            highlight_keywords: false,
            simplified_language: false,
        }
    }
}

/// Memory aid settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemoryAidSettings {
    #[serde(default = "default_true")]
    pub story_recap: bool,
    #[serde(default = "default_true")]
    pub quest_journal: bool,
    #[serde(default = "default_true")]
    pub recent_actions_log: bool,
    #[serde(default = "default_true")]
    pub character_bios: bool,
    #[serde(default)]
    pub relationship_tracker: bool,
    #[serde(default = "default_true")]
    pub map_annotations: bool,
    #[serde(default = "default_true")]
    pub visited_location_markers: bool,
}

impl Default for MemoryAidSettings {
    fn default() -> Self {
        Self {
            story_recap: true,
            quest_journal: true,
            recent_actions_log: true,
            character_bios: true,
            relationship_tracker: false,
            map_annotations: true,
            visited_location_markers: true,
        }
    }
}

/// Focus aid settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FocusAidSettings {
    #[serde(default)]
    pub break_reminders: bool,
    #[serde(default = "default_break_interval")]
    pub break_interval_min: u32,
    #[serde(default = "default_true")]
    pub pause_on_focus_loss: bool,
    #[serde(default)]
    pub reduce_background_motion: bool,
    #[serde(default = "default_true")]
    pub mute_on_focus_loss: bool,
}

impl Default for FocusAidSettings {
    fn default() -> Self {
        Self {
            break_reminders: false,
            break_interval_min: 60,
            pause_on_focus_loss: true,
            reduce_background_motion: false,
            mute_on_focus_loss: true,
        }
    }
}

// ============================================================================
// Controller Config
// ============================================================================

/// Controller configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerConfig {
    #[serde(default)]
    pub config_id: Option<ConfigId>,
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub controller_type: InputDeviceType,
    #[serde(default)]
    pub button_mapping: std::collections::HashMap<String, ButtonAction>,
    #[serde(default)]
    pub combo_mapping: Vec<ComboAction>,
    #[serde(default)]
    pub macro_mapping: Vec<MacroAction>,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            config_id: Some(Uuid::new_v4()),
            name: None,
            controller_type: InputDeviceType::StandardController,
            button_mapping: std::collections::HashMap::new(),
            combo_mapping: Vec::new(),
            macro_mapping: Vec::new(),
        }
    }
}

/// Button action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonAction {
    pub action: String,
    #[serde(default)]
    pub modifiers: Vec<String>,
    #[serde(default)]
    pub behavior: ButtonBehavior,
    #[serde(default = "default_speed")]
    pub sensitivity: f32,
    #[serde(default = "default_true")]
    pub enabled: bool,
}

/// Button behavior
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ButtonBehavior {
    #[default]
    Press,
    Hold,
    Toggle,
    Tap,
    DoubleTap,
}

/// Combo action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComboAction {
    pub buttons: Vec<String>,
    pub action: String,
    #[serde(default = "default_true")]
    pub simultaneous: bool,
    #[serde(default = "default_tap_timing")]
    pub timeout_ms: u32,
}

/// Macro action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MacroAction {
    pub name: String,
    pub trigger: String,
    pub sequence: Vec<MacroStep>,
    #[serde(default)]
    pub loop_macro: bool,
    #[serde(default = "default_true")]
    pub enabled: bool,
}

/// Macro step
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MacroStep {
    pub action: String,
    #[serde(default)]
    pub step_type: MacroStepType,
    #[serde(default)]
    pub delay_ms: u32,
    #[serde(default)]
    pub duration_ms: Option<u32>,
}

/// Macro step type
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MacroStepType {
    #[default]
    Press,
    Release,
    Hold,
}

// ============================================================================
// Accessibility Preset
// ============================================================================

/// Accessibility preset
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityPreset {
    pub preset_id: PresetId,
    pub name: String,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub preset_type: PresetType,
    #[serde(default)]
    pub game_genre: Option<GameGenre>,
    #[serde(default)]
    pub target_disability: Vec<DisabilityType>,
    #[serde(default)]
    pub created_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub author: Option<String>,
    #[serde(default)]
    pub version: Option<String>,
    #[serde(default)]
    pub settings_override: SettingsOverride,
    #[serde(default)]
    pub tags: Vec<String>,
}

impl Default for AccessibilityPreset {
    fn default() -> Self {
        Self {
            preset_id: Uuid::new_v4(),
            name: "Custom Preset".to_string(),
            description: None,
            preset_type: PresetType::User,
            game_genre: None,
            target_disability: Vec::new(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            author: None,
            version: Some("1.0.0".to_string()),
            settings_override: SettingsOverride::default(),
            tags: Vec::new(),
        }
    }
}

/// Preset type
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PresetType {
    System,
    #[default]
    User,
    Game,
    Community,
}

/// Game genre
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GameGenre {
    Fps,
    Tps,
    Rpg,
    ActionRpg,
    Strategy,
    Rts,
    Puzzle,
    Platformer,
    Racing,
    Sports,
    Fighting,
    Simulation,
    Adventure,
    Horror,
    Mmo,
    BattleRoyale,
    Roguelike,
    Rhythm,
    Vr,
    General,
}

/// Settings override in presets
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SettingsOverride {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visual_settings: Option<VisualSettings>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub audio_settings: Option<AudioSettings>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motor_settings: Option<MotorSettings>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cognitive_settings: Option<CognitiveSettings>,
}

// ============================================================================
// Game Metadata
// ============================================================================

/// Game accessibility metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GameMetadata {
    pub game_id: GameId,
    pub title: String,
    pub version: String,
    #[serde(default)]
    pub accessibility_version: Option<String>,
    #[serde(default)]
    pub platforms: Vec<Platform>,
    #[serde(default)]
    pub visual_features: VisualFeatures,
    #[serde(default)]
    pub audio_features: AudioFeatures,
    #[serde(default)]
    pub motor_features: MotorFeatures,
    #[serde(default)]
    pub cognitive_features: CognitiveFeatures,
    #[serde(default)]
    pub input_support: InputSupport,
    #[serde(default)]
    pub accessibility_rating: Option<AccessibilityRating>,
}

/// Platform
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Platform {
    Pc,
    Xbox,
    Playstation,
    Switch,
    Mobile,
    Vr,
    Cloud,
}

/// Visual features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualFeatures {
    #[serde(default)]
    pub screen_reader_support: bool,
    #[serde(default)]
    pub colorblind_modes: Vec<ColorblindMode>,
    #[serde(default)]
    pub high_contrast_mode: bool,
    #[serde(default)]
    pub reduce_motion_option: bool,
    #[serde(default)]
    pub disable_screen_shake: bool,
    #[serde(default)]
    pub target_lock_indicator: bool,
    #[serde(default)]
    pub customizable_crosshair: bool,
    #[serde(default)]
    pub customizable_hud: bool,
}

/// Audio features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AudioFeatures {
    #[serde(default)]
    pub subtitle_support: bool,
    #[serde(default)]
    pub closed_captions: bool,
    #[serde(default)]
    pub visual_sound_indicators: bool,
    #[serde(default)]
    pub mono_audio_option: bool,
    #[serde(default)]
    pub separate_volume_controls: Vec<String>,
    #[serde(default)]
    pub tts_chat: bool,
    #[serde(default)]
    pub tts_ui: bool,
}

/// Motor features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorFeatures {
    #[serde(default)]
    pub full_button_remapping: bool,
    #[serde(default)]
    pub hold_to_toggle_options: bool,
    #[serde(default)]
    pub adjustable_sensitivity: bool,
    #[serde(default)]
    pub adjustable_dead_zones: bool,
    #[serde(default)]
    pub aim_assist_levels: Vec<AimAssistStrength>,
    #[serde(default)]
    pub one_handed_modes: Vec<Hand>,
    #[serde(default)]
    pub adaptive_controller_support: bool,
    #[serde(default)]
    pub switch_access: bool,
    #[serde(default)]
    pub keyboard_mouse_on_console: bool,
    #[serde(default)]
    pub touch_controls: bool,
    #[serde(default)]
    pub gyro_aiming: bool,
    #[serde(default)]
    pub copilot_mode: bool,
}

/// Cognitive features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CognitiveFeatures {
    #[serde(default)]
    pub multiple_difficulty_levels: bool,
    #[serde(default)]
    pub difficulty_options: Vec<DifficultyLevel>,
    #[serde(default)]
    pub granular_difficulty: bool,
    #[serde(default)]
    pub skip_puzzles_option: bool,
    #[serde(default)]
    pub skip_combat_option: bool,
    #[serde(default)]
    pub objective_markers: bool,
    #[serde(default)]
    pub navigation_assistance: bool,
    #[serde(default)]
    pub tutorial_replay: bool,
    #[serde(default)]
    pub hint_system: bool,
    #[serde(default)]
    pub content_warnings: bool,
    #[serde(default)]
    pub save_anywhere: bool,
    #[serde(default)]
    pub auto_save: bool,
    #[serde(default)]
    pub pause_cutscenes: bool,
}

/// Input support
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InputSupport {
    #[serde(default)]
    pub controllers: Vec<InputDeviceType>,
    #[serde(default)]
    pub eye_tracking: bool,
    #[serde(default)]
    pub voice_commands: bool,
    #[serde(default)]
    pub switch_access: bool,
    #[serde(default)]
    pub head_tracking: bool,
    #[serde(default)]
    pub mouth_controller: bool,
}

/// Accessibility rating
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityRating {
    #[serde(default)]
    pub overall_score: u8,
    #[serde(default)]
    pub visual_score: u8,
    #[serde(default)]
    pub audio_score: u8,
    #[serde(default)]
    pub motor_score: u8,
    #[serde(default)]
    pub cognitive_score: u8,
    #[serde(default)]
    pub certifications: Vec<Certification>,
    #[serde(default)]
    pub reviewed_by: Vec<String>,
    #[serde(default)]
    pub last_review_date: Option<String>,
}

/// Certification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Certification {
    WiaCertified,
    WiaGold,
    WiaPlatinum,
    AbleGamersApproved,
    CanIPlayThatReviewed,
    DagCertified,
}

// ============================================================================
// Helper Functions
// ============================================================================

fn default_true() -> bool {
    true
}

fn default_speed() -> f32 {
    1.0
}

fn default_half() -> f32 {
    0.5
}

fn default_opacity() -> f32 {
    0.8
}

fn default_magnification() -> f32 {
    2.0
}

fn default_line_height() -> f32 {
    1.4
}

fn default_volume_music() -> f32 {
    0.7
}

fn default_dead_zone() -> f32 {
    0.15
}

fn default_tap_timing() -> u32 {
    500
}

fn default_hold_timing() -> u32 {
    1000
}

fn default_repeat_rate() -> u32 {
    100
}

fn default_debounce() -> u32 {
    50
}

fn default_sequential_timeout() -> u32 {
    2000
}

fn default_reminder_frequency() -> u32 {
    60
}

fn default_hint_delay() -> u32 {
    30
}

fn default_warning_duration() -> u32 {
    5
}

fn default_break_interval() -> u32 {
    60
}

fn default_red() -> String {
    "#FF0000".to_string()
}

fn default_green() -> String {
    "#00FF00".to_string()
}

fn default_yellow() -> String {
    "#FFFF00".to_string()
}

fn default_white() -> String {
    "#FFFFFF".to_string()
}

fn default_black() -> String {
    "#000000".to_string()
}
