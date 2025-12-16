//! Type definitions for WIA XR Accessibility
//!
//! These types correspond to the JSON Schema definitions in Phase 1.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Core Profile Types
// ============================================================================

/// Complete XR Accessibility Profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XRAccessibilityProfile {
    /// Unique profile identifier
    pub profile_id: String,

    /// Schema version
    pub version: String,

    /// Creation timestamp
    pub created_at: DateTime<Utc>,

    /// Last update timestamp
    pub updated_at: DateTime<Utc>,

    /// Anonymized user identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user_hash: Option<String>,

    /// Disability profile
    pub disabilities: DisabilityProfile,

    /// Sensory preferences
    pub sensory: SensoryPreferences,

    /// Input preferences
    pub input: InputPreferences,

    /// Output preferences
    pub output: OutputPreferences,

    /// Comfort settings
    pub comfort: ComfortSettings,

    /// WIA device integrations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_integrations: Option<WIAIntegrations>,
}

impl Default for XRAccessibilityProfile {
    fn default() -> Self {
        Self {
            profile_id: format!("prof_{}", uuid::Uuid::new_v4().simple()),
            version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: Utc::now(),
            user_hash: None,
            disabilities: DisabilityProfile::default(),
            sensory: SensoryPreferences::default(),
            input: InputPreferences::default(),
            output: OutputPreferences::default(),
            comfort: ComfortSettings::default(),
            wia_integrations: None,
        }
    }
}

// ============================================================================
// Disability Types
// ============================================================================

/// Disability profile containing all disability categories
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DisabilityProfile {
    /// Visual impairments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual: Option<VisualDisability>,

    /// Auditory impairments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory: Option<AuditoryDisability>,

    /// Motor/physical impairments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorDisability>,

    /// Cognitive/neurological conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive: Option<CognitiveDisability>,

    /// Speech impairments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speech: Option<SpeechDisability>,

    /// Has multiple disabilities
    pub has_multiple: bool,
}

/// Visual disability severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualLevel {
    None,
    LowVision,
    LegallyBlind,
    TotallyBlind,
}

impl Default for VisualLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Color vision type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorVision {
    Normal,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
}

impl Default for ColorVision {
    fn default() -> Self {
        Self::Normal
    }
}

/// Photosensitivity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhotosensitivityLevel {
    None,
    Mild,
    Moderate,
    Severe,
}

impl Default for PhotosensitivityLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Visual disability configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualDisability {
    /// Severity level
    pub level: VisualLevel,

    /// Specific conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions: Option<Vec<VisualCondition>>,

    /// Color vision type
    pub color_vision: ColorVision,

    /// Field of vision
    #[serde(skip_serializing_if = "Option::is_none")]
    pub field_of_vision: Option<FieldOfVision>,

    /// Photosensitivity
    pub photosensitivity: PhotosensitivityLevel,
}

impl Default for VisualDisability {
    fn default() -> Self {
        Self {
            level: VisualLevel::None,
            conditions: None,
            color_vision: ColorVision::Normal,
            field_of_vision: None,
            photosensitivity: PhotosensitivityLevel::None,
        }
    }
}

/// Visual conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualCondition {
    MacularDegeneration,
    Glaucoma,
    DiabeticRetinopathy,
    RetinitisPigmentosa,
    Cataracts,
    Nystagmus,
    Other,
}

/// Field of vision configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldOfVision {
    /// Horizontal degrees (normal ~180)
    pub horizontal_degrees: f32,
    /// Vertical degrees (normal ~120)
    pub vertical_degrees: f32,
    /// Blind spots
    #[serde(skip_serializing_if = "Option::is_none")]
    pub blind_spots: Option<Vec<BlindSpotRegion>>,
}

/// Blind spot region
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlindSpotRegion {
    pub center_x: f32,
    pub center_y: f32,
    pub radius: f32,
}

/// Auditory disability level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditoryLevel {
    None,
    HardOfHearing,
    Deaf,
    Deafblind,
}

impl Default for AuditoryLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Sign language code
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignLanguageCode {
    ASL,
    BSL,
    KSL,
    JSL,
    DGS,
    LSF,
    Auslan,
    #[serde(rename = "other")]
    Other,
}

/// Auditory disability configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryDisability {
    /// Severity level
    pub level: AuditoryLevel,

    /// Hearing thresholds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hearing_threshold: Option<HearingThreshold>,

    /// Frequency response
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency_response: Option<FrequencyResponse>,

    /// Preferred communication methods
    pub preferred_communication: Vec<CommunicationMethod>,

    /// Sign language preference
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language: Option<SignLanguageCode>,

    /// Uses hearing device
    pub uses_hearing_device: bool,

    /// Hearing device type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hearing_device_type: Option<HearingDeviceType>,
}

impl Default for AuditoryDisability {
    fn default() -> Self {
        Self {
            level: AuditoryLevel::None,
            hearing_threshold: None,
            frequency_response: None,
            preferred_communication: vec![CommunicationMethod::Spoken],
            sign_language: None,
            uses_hearing_device: false,
            hearing_device_type: None,
        }
    }
}

/// Hearing threshold in dB HL
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HearingThreshold {
    pub left_ear: f32,
    pub right_ear: f32,
}

/// Frequency response loss
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyResponse {
    pub low_freq_loss: bool,
    pub mid_freq_loss: bool,
    pub high_freq_loss: bool,
}

/// Communication method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommunicationMethod {
    Spoken,
    SignLanguage,
    Written,
    CuedSpeech,
}

/// Hearing device type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HearingDeviceType {
    HearingAid,
    CochlearImplant,
    BoneAnchored,
}

/// Motor disability level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotorLevel {
    None,
    Mild,
    Moderate,
    Severe,
}

impl Default for MotorLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Mobility status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MobilityStatus {
    Ambulatory,
    AssistedWalking,
    Wheelchair,
    BedBound,
}

impl Default for MobilityStatus {
    fn default() -> Self {
        Self::Ambulatory
    }
}

/// Motor disability configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorDisability {
    /// Severity level
    pub level: MotorLevel,

    /// Affected body areas
    pub affected_areas: Vec<MotorAffectedArea>,

    /// Mobility status
    pub mobility: MobilityStatus,

    /// Fine motor control
    pub fine_motor: FineMotorControl,

    /// Range of motion
    #[serde(skip_serializing_if = "Option::is_none")]
    pub range_of_motion: Option<RangeOfMotion>,

    /// Fatigue threshold in minutes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fatigue_threshold_minutes: Option<u32>,

    /// Assistive devices used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistive_devices: Option<Vec<MotorAssistiveDevice>>,
}

impl Default for MotorDisability {
    fn default() -> Self {
        Self {
            level: MotorLevel::None,
            affected_areas: vec![],
            mobility: MobilityStatus::Ambulatory,
            fine_motor: FineMotorControl::default(),
            range_of_motion: None,
            fatigue_threshold_minutes: None,
            assistive_devices: None,
        }
    }
}

/// Motor affected body area
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotorAffectedArea {
    UpperLimbLeft,
    UpperLimbRight,
    LowerLimbLeft,
    LowerLimbRight,
    Trunk,
    Neck,
    Face,
}

/// Fine motor control settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FineMotorControl {
    pub dominant_hand: DominantHand,
    pub grip_strength: GripStrength,
    pub precision: Precision,
    pub tremor: TremorLevel,
}

impl Default for FineMotorControl {
    fn default() -> Self {
        Self {
            dominant_hand: DominantHand::Right,
            grip_strength: GripStrength::Normal,
            precision: Precision::Normal,
            tremor: TremorLevel::None,
        }
    }
}

/// Dominant hand
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DominantHand {
    Left,
    Right,
    Ambidextrous,
    Neither,
}

/// Grip strength level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripStrength {
    Normal,
    Reduced,
    Minimal,
    None,
}

/// Precision level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Precision {
    Normal,
    Reduced,
    Minimal,
}

/// Tremor level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TremorLevel {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Range of motion percentages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RangeOfMotion {
    pub neck_rotation: f32,
    pub arm_reach: f32,
    pub wrist_rotation: f32,
    pub finger_dexterity: f32,
}

/// Motor assistive device
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotorAssistiveDevice {
    WheelchairManual,
    WheelchairPowered,
    Walker,
    Cane,
    Prosthetic,
    WiaExoskeleton,
    AdaptiveController,
    MouthStick,
    HeadPointer,
    EyeTracker,
}

/// Cognitive disability level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveLevel {
    None,
    Mild,
    Moderate,
    Significant,
}

impl Default for CognitiveLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Cognitive disability configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveDisability {
    /// Severity level
    pub level: CognitiveLevel,

    /// Conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions: Option<Vec<CognitiveCondition>>,

    /// Processing characteristics
    pub processing: CognitiveProcessing,

    /// Sensory processing
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensory_processing: Option<SensoryProcessing>,

    /// Needs predictable environment
    pub needs_predictability: bool,

    /// Needs simplified interface
    pub needs_simplified_interface: bool,

    /// Needs extended time
    pub needs_extended_time: bool,

    /// Has photosensitive epilepsy
    pub photosensitive_epilepsy: bool,

    /// Motion sensitivity
    pub motion_sensitivity: MotionSensitivity,
}

impl Default for CognitiveDisability {
    fn default() -> Self {
        Self {
            level: CognitiveLevel::None,
            conditions: None,
            processing: CognitiveProcessing::default(),
            sensory_processing: None,
            needs_predictability: false,
            needs_simplified_interface: false,
            needs_extended_time: false,
            photosensitive_epilepsy: false,
            motion_sensitivity: MotionSensitivity::None,
        }
    }
}

/// Cognitive condition
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveCondition {
    AutismSpectrum,
    Adhd,
    Dyslexia,
    Dyscalculia,
    IntellectualDisability,
    TraumaticBrainInjury,
    Dementia,
    Epilepsy,
    Ptsd,
    AnxietyDisorder,
    Other,
}

/// Cognitive processing characteristics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveProcessing {
    pub speed: ProcessingSpeed,
    pub working_memory: WorkingMemory,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attention_span_minutes: Option<u32>,
    pub multitasking_ability: MultitaskingAbility,
}

impl Default for CognitiveProcessing {
    fn default() -> Self {
        Self {
            speed: ProcessingSpeed::Normal,
            working_memory: WorkingMemory::Normal,
            attention_span_minutes: None,
            multitasking_ability: MultitaskingAbility::Normal,
        }
    }
}

/// Processing speed
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcessingSpeed {
    Normal,
    Slower,
    Variable,
}

/// Working memory capacity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WorkingMemory {
    Normal,
    Reduced,
    SignificantlyReduced,
}

/// Multitasking ability
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MultitaskingAbility {
    Normal,
    Limited,
    SingleTaskOnly,
}

/// Sensory processing characteristics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensoryProcessing {
    pub visual_overload_threshold: OverloadThreshold,
    pub auditory_overload_threshold: OverloadThreshold,
    pub haptic_sensitivity: HapticSensitivity,
}

/// Overload threshold
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverloadThreshold {
    High,
    Normal,
    Low,
    VeryLow,
}

/// Haptic sensitivity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HapticSensitivity {
    Hypo,
    Normal,
    Hyper,
}

/// Motion sensitivity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionSensitivity {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Speech disability level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeechLevel {
    None,
    Mild,
    Moderate,
    Severe,
    Nonverbal,
}

impl Default for SpeechLevel {
    fn default() -> Self {
        Self::None
    }
}

/// Speech disability configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeechDisability {
    /// Severity level
    pub level: SpeechLevel,

    /// Conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions: Option<Vec<SpeechCondition>>,

    /// Voice characteristics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_characteristics: Option<VoiceCharacteristics>,

    /// Uses AAC (Augmentative and Alternative Communication)
    pub uses_aac: bool,

    /// AAC type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aac_type: Option<AacType>,
}

impl Default for SpeechDisability {
    fn default() -> Self {
        Self {
            level: SpeechLevel::None,
            conditions: None,
            voice_characteristics: None,
            uses_aac: false,
            aac_type: None,
        }
    }
}

/// Speech condition
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeechCondition {
    Stuttering,
    Dysarthria,
    Apraxia,
    Aphasia,
    VoiceDisorder,
    Laryngectomy,
    Other,
}

/// Voice characteristics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCharacteristics {
    pub intelligibility: Intelligibility,
    pub volume: VolumeLevel,
    pub rate: SpeechRate,
}

/// Speech intelligibility
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Intelligibility {
    Clear,
    Reduced,
    Limited,
    Unintelligible,
}

/// Volume level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VolumeLevel {
    Normal,
    Quiet,
    Loud,
    Variable,
}

/// Speech rate
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeechRate {
    Normal,
    Slow,
    Fast,
    Variable,
}

/// AAC type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AacType {
    TextBased,
    SymbolBased,
    SpeechGenerating,
}

// ============================================================================
// Sensory Preferences
// ============================================================================

/// Sensory preferences for all modalities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensoryPreferences {
    /// Visual preferences
    pub visual: VisualPreferences,

    /// Auditory preferences
    pub auditory: AuditoryPreferences,

    /// Haptic preferences
    pub haptic: HapticPreferences,
}

impl Default for SensoryPreferences {
    fn default() -> Self {
        Self {
            visual: VisualPreferences::default(),
            auditory: AuditoryPreferences::default(),
            haptic: HapticPreferences::default(),
        }
    }
}

/// Visual preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualPreferences {
    pub brightness_multiplier: f32,
    pub contrast_multiplier: f32,
    pub saturation_multiplier: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_filter: Option<ColorFilter>,
    pub text_size_multiplier: f32,
    pub font_preference: FontPreference,
    pub ui_scale: f32,
    pub high_contrast_mode: bool,
    pub reduce_transparency: bool,
    pub reduce_motion: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flash_threshold_hz: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_fov_degrees: Option<f32>,
}

impl Default for VisualPreferences {
    fn default() -> Self {
        Self {
            brightness_multiplier: 1.0,
            contrast_multiplier: 1.0,
            saturation_multiplier: 1.0,
            color_filter: None,
            text_size_multiplier: 1.0,
            font_preference: FontPreference::Default,
            ui_scale: 1.0,
            high_contrast_mode: false,
            reduce_transparency: false,
            reduce_motion: false,
            flash_threshold_hz: None,
            preferred_fov_degrees: None,
        }
    }
}

/// Color filter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorFilter {
    #[serde(rename = "type")]
    pub filter_type: ColorFilterType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom_matrix: Option<Vec<f32>>,
}

/// Color filter type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorFilterType {
    None,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Grayscale,
    Inverted,
    Custom,
}

/// Font preference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FontPreference {
    Default,
    SansSerif,
    DyslexiaFriendly,
    HighLegibility,
}

/// Auditory preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryPreferences {
    pub master_volume: f32,
    pub voice_volume: f32,
    pub effects_volume: f32,
    pub music_volume: f32,
    pub balance: f32,
    pub mono_audio: bool,
    pub background_noise_reduction: bool,
    pub voice_enhancement: bool,
    pub spatial_audio_enabled: bool,
    pub spatial_audio_intensity: f32,
    pub bass_boost: f32,
    pub treble_boost: f32,
}

impl Default for AuditoryPreferences {
    fn default() -> Self {
        Self {
            master_volume: 0.8,
            voice_volume: 1.0,
            effects_volume: 0.8,
            music_volume: 0.6,
            balance: 0.0,
            mono_audio: false,
            background_noise_reduction: false,
            voice_enhancement: false,
            spatial_audio_enabled: true,
            spatial_audio_intensity: 1.0,
            bass_boost: 0.0,
            treble_boost: 0.0,
        }
    }
}

/// Haptic preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPreferences {
    pub enabled: bool,
    pub intensity: f32,
    pub controller_haptics: bool,
    pub vest_haptics: bool,
    pub glove_haptics: bool,
    pub vibration_enabled: bool,
    pub force_feedback_enabled: bool,
    pub texture_feedback_enabled: bool,
    pub temperature_feedback_enabled: bool,
}

impl Default for HapticPreferences {
    fn default() -> Self {
        Self {
            enabled: true,
            intensity: 0.7,
            controller_haptics: true,
            vest_haptics: false,
            glove_haptics: false,
            vibration_enabled: true,
            force_feedback_enabled: true,
            texture_feedback_enabled: false,
            temperature_feedback_enabled: false,
        }
    }
}

// ============================================================================
// Input Preferences
// ============================================================================

/// Input method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputMethod {
    ControllerStandard,
    ControllerAdaptive,
    EyeTracking,
    VoiceControl,
    HeadTracking,
    HandTracking,
    BrainComputerInterface,
    SwitchAccess,
    MouthController,
    WiaExoskeleton,
}

/// Input preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputPreferences {
    pub primary_input: InputMethod,
    pub fallback_inputs: Vec<InputMethod>,
    pub controller: ControllerSettings,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eye_tracking: Option<EyeTrackingSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_control: Option<VoiceControlSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub head_tracking: Option<HeadTrackingSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hand_tracking: Option<HandTrackingSettings>,
}

impl Default for InputPreferences {
    fn default() -> Self {
        Self {
            primary_input: InputMethod::ControllerStandard,
            fallback_inputs: vec![],
            controller: ControllerSettings::default(),
            eye_tracking: None,
            voice_control: None,
            head_tracking: None,
            hand_tracking: None,
        }
    }
}

/// Controller settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerSettings {
    pub dominant_hand: ControllerHand,
    pub one_handed_mode: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub one_handed_controller: Option<ControllerHand>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub button_remapping: Option<Vec<ButtonRemapping>>,
    pub stick_sensitivity: f32,
    pub trigger_sensitivity: f32,
    pub stick_deadzone: f32,
    pub trigger_deadzone: f32,
    pub grip_toggle: bool,
    pub trigger_toggle: bool,
}

impl Default for ControllerSettings {
    fn default() -> Self {
        Self {
            dominant_hand: ControllerHand::Right,
            one_handed_mode: false,
            one_handed_controller: None,
            button_remapping: None,
            stick_sensitivity: 1.0,
            trigger_sensitivity: 1.0,
            stick_deadzone: 0.1,
            trigger_deadzone: 0.05,
            grip_toggle: false,
            trigger_toggle: false,
        }
    }
}

/// Controller hand
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControllerHand {
    Left,
    Right,
    Either,
}

/// Button remapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonRemapping {
    pub original_button: String,
    pub remapped_to: String,
}

/// Eye tracking settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeTrackingSettings {
    pub enabled: bool,
    pub dwell_time_ms: u32,
    pub gaze_smoothing: f32,
    pub blink_to_select: bool,
}

/// Voice control settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceControlSettings {
    pub enabled: bool,
    pub language: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_word: Option<String>,
    pub continuous_listening: bool,
    pub command_timeout_ms: u32,
}

/// Head tracking settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeadTrackingSettings {
    pub enabled: bool,
    pub sensitivity: f32,
    pub snap_turning: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snap_angle_degrees: Option<u32>,
}

/// Hand tracking settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandTrackingSettings {
    pub enabled: bool,
    pub gesture_recognition: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom_gestures: Option<Vec<CustomGesture>>,
}

/// Custom gesture definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomGesture {
    pub gesture_id: String,
    pub gesture_name: String,
    pub hand: GestureHand,
    pub action: String,
    pub gesture_data: GestureData,
}

/// Gesture hand
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GestureHand {
    Left,
    Right,
    Both,
}

/// Gesture data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GestureData {
    pub finger_states: FingerStates,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub palm_direction: Option<PalmDirection>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motion: Option<GestureMotion>,
}

/// Finger states
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FingerStates {
    pub thumb: FingerState,
    pub index: FingerState,
    pub middle: FingerState,
    pub ring: FingerState,
    pub pinky: FingerState,
}

/// Finger state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FingerState {
    Extended,
    Curled,
    Any,
}

/// Palm direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PalmDirection {
    Up,
    Down,
    Forward,
    Back,
    Any,
}

/// Gesture motion
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GestureMotion {
    Static,
    Swipe,
    Circle,
    Wave,
}

// ============================================================================
// Output Preferences
// ============================================================================

/// Output preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputPreferences {
    pub captions: CaptionSettings,
    pub audio_description: AudioDescriptionSettings,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language: Option<SignLanguageSettings>,
    pub screen_reader: ScreenReaderSettings,
    pub notifications: NotificationSettings,
}

impl Default for OutputPreferences {
    fn default() -> Self {
        Self {
            captions: CaptionSettings::default(),
            audio_description: AudioDescriptionSettings::default(),
            sign_language: None,
            screen_reader: ScreenReaderSettings::default(),
            notifications: NotificationSettings::default(),
        }
    }
}

/// Caption settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaptionSettings {
    pub enabled: bool,
    pub font_size: CaptionFontSize,
    pub font_family: String,
    pub text_color: String,
    pub background_color: String,
    pub background_opacity: f32,
    pub position: CaptionPosition,
    pub speaker_identification: bool,
    pub sound_descriptions: bool,
    pub music_descriptions: bool,
    pub language: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub translate_from: Option<Vec<String>>,
}

impl Default for CaptionSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            font_size: CaptionFontSize::Medium,
            font_family: "system".to_string(),
            text_color: "#FFFFFF".to_string(),
            background_color: "#000000AA".to_string(),
            background_opacity: 0.7,
            position: CaptionPosition::default(),
            speaker_identification: true,
            sound_descriptions: true,
            music_descriptions: true,
            language: "en-US".to_string(),
            translate_from: None,
        }
    }
}

/// Caption font size
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptionFontSize {
    Small,
    Medium,
    Large,
    ExtraLarge,
}

/// Caption position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaptionPosition {
    pub mode: CaptionPositionMode,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position_3d: Option<Position3D>,
    pub follow_head: bool,
    pub follow_speed: f32,
}

impl Default for CaptionPosition {
    fn default() -> Self {
        Self {
            mode: CaptionPositionMode::Floating,
            position_3d: None,
            follow_head: true,
            follow_speed: 0.3,
        }
    }
}

/// Caption position mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptionPositionMode {
    Floating,
    Fixed,
    AttachedToSpeaker,
}

/// 3D position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position3D {
    pub distance_meters: f32,
    pub vertical_angle: f32,
    pub horizontal_offset: f32,
}

/// Audio description settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioDescriptionSettings {
    pub enabled: bool,
    pub voice: VoiceType,
    pub speed: f32,
    pub detail_level: DetailLevel,
    pub describe_actions: bool,
    pub describe_scenes: bool,
    pub describe_ui: bool,
}

impl Default for AudioDescriptionSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            voice: VoiceType::Neutral,
            speed: 1.0,
            detail_level: DetailLevel::Standard,
            describe_actions: true,
            describe_scenes: true,
            describe_ui: true,
        }
    }
}

/// Voice type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VoiceType {
    Male,
    Female,
    Neutral,
}

/// Detail level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DetailLevel {
    Minimal,
    Standard,
    Detailed,
}

/// Sign language settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignLanguageSettings {
    pub enabled: bool,
    pub language: SignLanguageCode,
    pub avatar_style: AvatarStyle,
    pub avatar_position: AvatarPosition,
    pub avatar_size: f32,
    pub signing_speed: f32,
}

/// Avatar style
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AvatarStyle {
    Realistic,
    Stylized,
    Minimal,
}

/// Avatar position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AvatarPosition {
    pub mode: AvatarPositionMode,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub corner: Option<Corner>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom_position: Option<CustomPosition>,
}

/// Avatar position mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AvatarPositionMode {
    Corner,
    BesideSpeaker,
    Custom,
}

/// Corner position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Corner {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
}

/// Custom position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomPosition {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Screen reader settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenReaderSettings {
    pub enabled: bool,
    pub verbosity: Verbosity,
    pub speak_hints: bool,
    pub speak_notifications: bool,
    pub voice_rate: f32,
    pub voice_pitch: f32,
}

impl Default for ScreenReaderSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            verbosity: Verbosity::Medium,
            speak_hints: true,
            speak_notifications: true,
            voice_rate: 1.0,
            voice_pitch: 1.0,
        }
    }
}

/// Verbosity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verbosity {
    Low,
    Medium,
    High,
}

/// Notification settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationSettings {
    pub visual_alerts: bool,
    pub audio_alerts: bool,
    pub haptic_alerts: bool,
    pub flash_screen: bool,
    pub notification_position: NotificationPosition,
}

impl Default for NotificationSettings {
    fn default() -> Self {
        Self {
            visual_alerts: true,
            audio_alerts: true,
            haptic_alerts: true,
            flash_screen: false,
            notification_position: NotificationPosition::TopCenter,
        }
    }
}

/// Notification position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationPosition {
    TopCenter,
    BottomCenter,
    TopLeft,
    TopRight,
    Wrist,
    Floating,
}

// ============================================================================
// Comfort Settings
// ============================================================================

/// Comfort settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComfortSettings {
    pub motion: MotionComfort,
    pub physical: PhysicalComfort,
    pub cognitive: CognitiveComfort,
    pub session: SessionLimits,
}

impl Default for ComfortSettings {
    fn default() -> Self {
        Self {
            motion: MotionComfort::default(),
            physical: PhysicalComfort::default(),
            cognitive: CognitiveComfort::default(),
            session: SessionLimits::default(),
        }
    }
}

/// Motion comfort settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionComfort {
    pub movement_type: MovementType,
    pub vignette_enabled: bool,
    pub vignette_intensity: f32,
    pub vignette_on_movement: bool,
    pub vignette_on_rotation: bool,
    pub max_movement_speed: f32,
    pub max_rotation_speed: f32,
    pub acceleration_smoothing: f32,
    pub seated_mode: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seated_height_offset_meters: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snap_turn_angle_degrees: Option<u32>,
}

impl Default for MotionComfort {
    fn default() -> Self {
        Self {
            movement_type: MovementType::Smooth,
            vignette_enabled: false,
            vignette_intensity: 0.5,
            vignette_on_movement: true,
            vignette_on_rotation: true,
            max_movement_speed: 3.0,
            max_rotation_speed: 180.0,
            acceleration_smoothing: 0.3,
            seated_mode: false,
            seated_height_offset_meters: None,
            snap_turn_angle_degrees: Some(45),
        }
    }
}

/// Movement type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MovementType {
    Smooth,
    Teleport,
    SnapTurn,
    Hybrid,
}

/// Physical comfort settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalComfort {
    pub play_area_type: PlayAreaType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub play_area_bounds: Option<PlayAreaBounds>,
    pub rest_reminder_enabled: bool,
    pub rest_reminder_interval_minutes: u32,
    pub boundary_style: BoundaryStyle,
    pub boundary_sensitivity: BoundarySensitivity,
}

impl Default for PhysicalComfort {
    fn default() -> Self {
        Self {
            play_area_type: PlayAreaType::Standing,
            play_area_bounds: None,
            rest_reminder_enabled: true,
            rest_reminder_interval_minutes: 30,
            boundary_style: BoundaryStyle::Grid,
            boundary_sensitivity: BoundarySensitivity::Normal,
        }
    }
}

/// Play area type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlayAreaType {
    Seated,
    Standing,
    RoomScale,
}

/// Play area bounds
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlayAreaBounds {
    pub width_meters: f32,
    pub depth_meters: f32,
}

/// Boundary style
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryStyle {
    Grid,
    Wall,
    Outline,
    Passthrough,
}

/// Boundary sensitivity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundarySensitivity {
    Close,
    Normal,
    Far,
}

/// Cognitive comfort settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveComfort {
    pub allow_pause_anytime: bool,
    pub auto_pause_on_overwhelm: bool,
    pub simplified_ui: bool,
    pub reduce_simultaneous_stimuli: bool,
    pub max_simultaneous_sounds: u32,
    pub safe_space_enabled: bool,
    pub safe_space_trigger: SafeSpaceTrigger,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safe_space_environment: Option<String>,
    pub content_warnings_enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trigger_warnings: Option<Vec<String>>,
}

impl Default for CognitiveComfort {
    fn default() -> Self {
        Self {
            allow_pause_anytime: true,
            auto_pause_on_overwhelm: false,
            simplified_ui: false,
            reduce_simultaneous_stimuli: false,
            max_simultaneous_sounds: 8,
            safe_space_enabled: true,
            safe_space_trigger: SafeSpaceTrigger::Button,
            safe_space_environment: None,
            content_warnings_enabled: true,
            trigger_warnings: None,
        }
    }
}

/// Safe space trigger
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafeSpaceTrigger {
    Button,
    Gesture,
    Voice,
    Automatic,
}

/// Session limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionLimits {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_session_duration_minutes: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required_break_duration_minutes: Option<u32>,
    pub warning_before_limit_minutes: u32,
}

impl Default for SessionLimits {
    fn default() -> Self {
        Self {
            max_session_duration_minutes: None,
            required_break_duration_minutes: None,
            warning_before_limit_minutes: 5,
        }
    }
}

// ============================================================================
// WIA Integrations
// ============================================================================

/// WIA device integrations
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WIAIntegrations {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exoskeleton: Option<ExoskeletonIntegration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bionic_eye: Option<BionicEyeIntegration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_sign: Option<VoiceSignIntegration>,
}

/// Exoskeleton integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonIntegration {
    pub enabled: bool,
    pub device_id: String,
    pub haptic_mapping: ExoskeletonHapticMapping,
    pub movement_assistance: MovementAssistance,
}

/// Exoskeleton haptic mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonHapticMapping {
    pub environment_textures: bool,
    pub collision_feedback: bool,
    pub ui_feedback: bool,
    pub notification_feedback: bool,
}

/// Movement assistance settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementAssistance {
    pub gesture_assistance: bool,
    pub assistance_level: AssistanceLevel,
}

/// Assistance level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssistanceLevel {
    Full,
    Partial,
    Guide,
}

/// Bionic eye integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BionicEyeIntegration {
    pub enabled: bool,
    pub device_id: String,
    pub visual_mode: VisualMode,
    pub contrast_enhancement: bool,
    pub edge_detection: bool,
    pub object_highlighting: bool,
}

/// Visual mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualMode {
    VrOnly,
    PassthroughOnly,
    Mixed,
    PictureInPicture,
}

/// Voice-Sign integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSignIntegration {
    pub enabled: bool,
    pub display_mode: SignDisplayMode,
    pub avatar_in_vr: bool,
    pub translate_vr_audio: bool,
    pub translate_npc_speech: bool,
    pub translate_player_voice: bool,
}

/// Sign display mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignDisplayMode {
    Avatar,
    Notation,
    Both,
}

// ============================================================================
// Device Capabilities
// ============================================================================

/// XR device capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XRDeviceCapabilities {
    pub device_id: String,
    pub device_name: String,
    pub manufacturer: String,
    pub model: String,
    pub firmware_version: String,
    pub display: DisplayCapabilities,
    pub audio: AudioCapabilities,
    pub input: InputCapabilities,
    pub haptics: HapticCapabilities,
    pub built_in_accessibility: BuiltInAccessibility,
    pub wia_compatibility: WIACompatibility,
}

/// Display capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayCapabilities {
    pub display_type: DisplayType,
    pub resolution_per_eye: Resolution,
    pub refresh_rates: Vec<f32>,
    pub field_of_view: FieldOfViewCaps,
    pub supports_passthrough: bool,
    pub passthrough_color: bool,
    pub supports_foveated_rendering: bool,
}

/// Display type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplayType {
    Lcd,
    Oled,
    MicroOled,
    Waveguide,
}

/// Resolution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Resolution {
    pub width: u32,
    pub height: u32,
}

/// Field of view capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldOfViewCaps {
    pub horizontal: f32,
    pub vertical: f32,
}

/// Audio capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioCapabilities {
    pub has_speakers: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speaker_type: Option<SpeakerType>,
    pub has_microphone: bool,
    pub supports_spatial_audio: bool,
    pub supports_bone_conduction: bool,
}

/// Speaker type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpeakerType {
    Integrated,
    OverEar,
    OpenEar,
}

/// Input capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputCapabilities {
    pub controller_type: ControllerType,
    pub has_eye_tracking: bool,
    pub has_hand_tracking: bool,
    pub has_face_tracking: bool,
    pub has_body_tracking: bool,
    pub voice_control: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub controller_features: Option<ControllerFeatures>,
}

/// Controller type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControllerType {
    #[serde(rename = "6dof")]
    SixDof,
    #[serde(rename = "3dof")]
    ThreeDof,
    None,
}

/// Controller features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerFeatures {
    pub has_haptics: bool,
    pub has_triggers: bool,
    pub has_grip: bool,
    pub has_thumbstick: bool,
    pub has_touchpad: bool,
    pub button_count: u32,
}

/// Haptic capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticCapabilities {
    pub controller_haptics: bool,
    pub haptic_fidelity: HapticFidelity,
    pub supports_external_haptics: bool,
}

/// Haptic fidelity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HapticFidelity {
    Basic,
    Hd,
    Advanced,
}

/// Built-in accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuiltInAccessibility {
    pub screen_reader: bool,
    pub magnification: bool,
    pub color_correction: bool,
    pub caption_support: bool,
    pub voice_control: bool,
    pub one_handed_mode: bool,
    pub seated_mode: bool,
}

/// WIA compatibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WIACompatibility {
    pub exoskeleton_compatible: bool,
    pub bionic_eye_compatible: bool,
    pub voice_sign_compatible: bool,
    pub protocol_version: String,
}

// ============================================================================
// Environment Configuration
// ============================================================================

/// Environment accessibility rating
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccessibilityRatingLevel {
    NotAccessible,
    PartiallyAccessible,
    Accessible,
    FullyAccessible,
}

/// Content warning type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentWarningType {
    FlashingLights,
    LoudSounds,
    SuddenEvents,
    Heights,
    EnclosedSpaces,
    Crowds,
    Violence,
    Gore,
    BodyHorror,
    MotionIntense,
    Other,
}

/// Content warning severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentWarningSeverity {
    Mild,
    Moderate,
    Strong,
}

/// Content warning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentWarning {
    #[serde(rename = "type")]
    pub warning_type: ContentWarningType,
    pub severity: ContentWarningSeverity,
    pub can_be_disabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

// ============================================================================
// XR Environment Configuration
// ============================================================================

/// XR environment accessibility configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct XREnvironmentConfig {
    /// Unique environment identifier
    pub environment_id: String,
    /// Human-readable environment name
    pub environment_name: String,
    /// Type of XR environment
    pub environment_type: EnvironmentType,
    /// Accessibility information
    pub accessibility: EnvironmentAccessibility,
    /// Sensory characteristics
    pub sensory: EnvironmentSensory,
    /// Interaction requirements
    pub interaction: EnvironmentInteraction,
}

/// Environment type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnvironmentType {
    Game,
    Social,
    Educational,
    Therapeutic,
    Productivity,
    Entertainment,
}

/// Environment accessibility information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentAccessibility {
    pub overall_rating: AccessibilityRatingLevel,
    pub visual_score: u32,
    pub auditory_score: u32,
    pub motor_score: u32,
    pub cognitive_score: u32,
    pub features: EnvironmentAccessibilityFeatures,
    pub content_warnings: Vec<ContentWarning>,
}

/// Environment accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EnvironmentAccessibilityFeatures {
    pub has_captions: bool,
    pub has_audio_description: bool,
    pub has_sign_language: bool,
    pub has_colorblind_mode: bool,
    pub has_high_contrast: bool,
    pub has_simplified_mode: bool,
    pub has_one_handed_mode: bool,
    pub has_seated_mode: bool,
    pub has_teleport_locomotion: bool,
    pub has_comfort_settings: bool,
}

/// Environment sensory characteristics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentSensory {
    pub visual_intensity: IntensityLevel,
    pub has_flashing: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flash_frequency_max_hz: Option<f32>,
    pub has_bright_lights: bool,
    pub has_darkness: bool,
    pub audio_intensity: IntensityLevel,
    pub has_sudden_sounds: bool,
    pub has_continuous_background: bool,
    pub motion_intensity: IntensityLevel,
    pub has_forced_movement: bool,
    pub has_falling_sequences: bool,
    pub has_vehicle_motion: bool,
}

/// Intensity level for sensory elements
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntensityLevel {
    Calm,
    Minimal,
    Quiet,
    Moderate,
    Intense,
    Loud,
}

/// Environment interaction requirements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentInteraction {
    pub minimum_input_methods: Vec<InputMethod>,
    pub required_physical_actions: Vec<PhysicalAction>,
    pub requires_voice: bool,
    pub requires_precise_aim: bool,
    pub requires_quick_reactions: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reaction_time_required_ms: Option<u32>,
}

/// Physical action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicalAction {
    Standing,
    Walking,
    Crouching,
    ReachingHigh,
    ReachingLow,
    TwoHanded,
    Pointing,
    Grabbing,
    Throwing,
    HeadMovement,
}

// ============================================================================
// Type Aliases for Compatibility
// ============================================================================

/// Alias for session settings
pub type SessionSettings = SessionLimits;

/// Exoskeleton settings alias
pub type ExoskeletonSettings = ExoskeletonIntegration;

/// Bionic eye settings alias
pub type BionicEyeSettings = BionicEyeIntegration;

/// Voice-Sign settings alias
pub type VoiceSignSettings = VoiceSignIntegration;
