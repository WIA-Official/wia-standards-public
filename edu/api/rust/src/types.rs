//! WIA Education Type Definitions
//! Based on Phase 1 JSON Schemas
//! 弘益人間 - Education for Everyone

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// ============================================================================
// Type Aliases
// ============================================================================

pub type ProfileId = Uuid;
pub type CourseId = Uuid;
pub type ModuleId = Uuid;
pub type ContentId = Uuid;
pub type AssessmentId = Uuid;
pub type QuestionId = Uuid;
pub type OutcomeId = Uuid;

// ============================================================================
// Learner Profile Types
// ============================================================================

/// Main learner accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LearnerProfile {
    pub profile_id: ProfileId,
    pub schema_version: String,
    pub created_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub learner_info: LearnerInfo,
    #[serde(default)]
    pub disability_profile: DisabilityProfile,
    #[serde(default)]
    pub display_preferences: DisplayPreferences,
    #[serde(default)]
    pub control_preferences: ControlPreferences,
    #[serde(default)]
    pub content_preferences: ContentPreferences,
    #[serde(default)]
    pub learning_style: LearningStyle,
    #[serde(default)]
    pub assessment_accommodations: AssessmentAccommodations,
    #[serde(default)]
    pub assistive_technology: AssistiveTechnology,
    #[serde(default)]
    pub wia_integrations: WIAIntegrations,
}

/// Basic learner information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LearnerInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_language: Option<String>,
    #[serde(default)]
    pub secondary_languages: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub age_group: Option<AgeGroup>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub education_level: Option<EducationLevel>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AgeGroup {
    Child,
    Teen,
    Adult,
    Senior,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EducationLevel {
    Elementary,
    MiddleSchool,
    HighSchool,
    Undergraduate,
    Graduate,
    Professional,
    LifelongLearner,
}

/// Disability-related accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DisabilityProfile {
    #[serde(default)]
    pub disclosed: bool,
    #[serde(default)]
    pub disability_types: Vec<DisabilityType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisabilityType {
    Blind,
    LowVision,
    ColorBlind,
    Deaf,
    HardOfHearing,
    Deafblind,
    MotorImpairment,
    LimitedDexterity,
    Tremor,
    Dyslexia,
    Dyscalculia,
    Dysgraphia,
    Adhd,
    Autism,
    Cognitive,
    Intellectual,
    Memory,
    Anxiety,
    ChronicIllness,
    TemporaryDisability,
    Situational,
    Other,
}

// ============================================================================
// Display Preferences (AccessForAll Display)
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DisplayPreferences {
    #[serde(default)]
    pub screen_reader: ScreenReaderSettings,
    #[serde(default)]
    pub magnification: MagnificationSettings,
    #[serde(default)]
    pub text_settings: TextSettings,
    #[serde(default)]
    pub color_settings: ColorSettings,
    #[serde(default)]
    pub reduce_motion: bool,
    #[serde(default)]
    pub reading_guide: ReadingGuide,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenReaderSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_reader: Option<ScreenReaderType>,
    #[serde(default = "default_speech_rate")]
    pub speech_rate: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verbosity: Option<Verbosity>,
}

fn default_speech_rate() -> f32 {
    1.0
}

impl Default for ScreenReaderSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            preferred_reader: None,
            speech_rate: 1.0,
            verbosity: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScreenReaderType {
    Jaws,
    Nvda,
    Voiceover,
    Talkback,
    Narrator,
    Orca,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Verbosity {
    Minimal,
    Normal,
    Verbose,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MagnificationSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_magnification")]
    pub level: f32,
    #[serde(default = "default_true")]
    pub follow_focus: bool,
}

fn default_magnification() -> f32 {
    1.0
}

fn default_true() -> bool {
    true
}

impl Default for MagnificationSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            level: 1.0,
            follow_focus: true,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TextSettings {
    #[serde(default)]
    pub font_size: FontSize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub font_family: Option<FontFamily>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom_font: Option<String>,
    #[serde(default = "default_line_spacing")]
    pub line_spacing: f32,
    #[serde(default)]
    pub letter_spacing: Spacing,
    #[serde(default)]
    pub word_spacing: Spacing,
}

fn default_line_spacing() -> f32 {
    1.5
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FontSize {
    Small,
    #[default]
    Medium,
    Large,
    XLarge,
    XxLarge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FontFamily {
    System,
    Serif,
    SansSerif,
    Monospace,
    Dyslexie,
    Opendyslexic,
    Custom,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Spacing {
    #[default]
    Normal,
    Wide,
    Wider,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ColorSettings {
    #[serde(default)]
    pub high_contrast: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contrast_mode: Option<ContrastMode>,
    #[serde(default)]
    pub invert_colors: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub background_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub link_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub highlight_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_blind_filter: Option<ColorBlindFilter>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContrastMode {
    Normal,
    HighContrastLight,
    HighContrastDark,
    Custom,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorBlindFilter {
    None,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReadingGuide {
    #[serde(default)]
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub guide_type: Option<ReadingGuideType>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReadingGuideType {
    LineHighlight,
    Ruler,
    Mask,
    FocusBox,
}

// ============================================================================
// Control Preferences (AccessForAll Control)
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ControlPreferences {
    #[serde(default)]
    pub input_method: InputMethodSettings,
    #[serde(default)]
    pub timing: TimingPreferences,
    #[serde(default)]
    pub click_settings: ClickSettings,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InputMethodSettings {
    #[serde(default)]
    pub primary: InputMethod,
    #[serde(default)]
    pub keyboard_only: bool,
    #[serde(default)]
    pub voice_control: VoiceControlSettings,
    #[serde(default)]
    pub eye_gaze: EyeGazeSettings,
    #[serde(default)]
    pub switch_access: SwitchAccessSettings,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputMethod {
    #[default]
    Keyboard,
    Mouse,
    Touch,
    Voice,
    EyeGaze,
    Switch,
    HeadTracking,
    Bci,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VoiceControlSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_word: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeGazeSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_dwell_time")]
    pub dwell_time_ms: u32,
}

fn default_dwell_time() -> u32 {
    1000
}

impl Default for EyeGazeSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            dwell_time_ms: 1000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchAccessSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_switches: Option<u8>,
    #[serde(default = "default_scan_speed")]
    pub scan_speed_ms: u32,
}

fn default_scan_speed() -> u32 {
    2000
}

impl Default for SwitchAccessSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            num_switches: None,
            scan_speed_ms: 2000,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingPreferences {
    #[serde(default)]
    pub extended_time: bool,
    #[serde(default = "default_time_multiplier")]
    pub time_multiplier: f32,
    #[serde(default)]
    pub disable_auto_advance: bool,
    #[serde(default = "default_true")]
    pub pause_on_inactivity: bool,
}

fn default_time_multiplier() -> f32 {
    1.5
}

impl Default for TimingPreferences {
    fn default() -> Self {
        Self {
            extended_time: false,
            time_multiplier: 1.5,
            disable_auto_advance: false,
            pause_on_inactivity: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClickSettings {
    #[serde(default)]
    pub large_targets: bool,
    #[serde(default = "default_target_size")]
    pub min_target_size_px: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub double_click_delay_ms: Option<u32>,
    #[serde(default)]
    pub sticky_keys: bool,
}

fn default_target_size() -> u32 {
    44
}

impl Default for ClickSettings {
    fn default() -> Self {
        Self {
            large_targets: false,
            min_target_size_px: 44,
            double_click_delay_ms: None,
            sticky_keys: false,
        }
    }
}

// ============================================================================
// Content Preferences (AccessForAll Content)
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ContentPreferences {
    #[serde(default)]
    pub captions: CaptionSettings,
    #[serde(default)]
    pub audio_description: AudioDescriptionSettings,
    #[serde(default)]
    pub transcripts: TranscriptSettings,
    #[serde(default)]
    pub sign_language: SignLanguageSettings,
    #[serde(default)]
    pub text_to_speech: TextToSpeechSettings,
    #[serde(default)]
    pub simplification: SimplificationSettings,
    #[serde(default)]
    pub alternative_formats: Vec<AlternativeFormat>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CaptionSettings {
    #[serde(default)]
    pub required: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub style: Option<CaptionStyle>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptionStyle {
    Default,
    Large,
    HighContrast,
    Custom,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AudioDescriptionSettings {
    #[serde(default)]
    pub required: bool,
    #[serde(default)]
    pub extended: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TranscriptSettings {
    #[serde(default)]
    pub required: bool,
    #[serde(default = "default_true")]
    pub interactive: bool,
}

impl Default for TranscriptSettings {
    fn default() -> Self {
        Self {
            required: false,
            interactive: true,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SignLanguageSettings {
    #[serde(default)]
    pub preferred: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<SignLanguageType>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignLanguageType {
    Asl,  // American Sign Language
    Bsl,  // British Sign Language
    Auslan, // Australian Sign Language
    Ksl,  // Korean Sign Language
    Jsl,  // Japanese Sign Language
    Lsf,  // French Sign Language
    Dgs,  // German Sign Language
    Other,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextToSpeechSettings {
    #[serde(default)]
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice: Option<String>,
    #[serde(default = "default_speech_rate")]
    pub rate: f32,
    #[serde(default = "default_speech_rate")]
    pub pitch: f32,
}

impl Default for TextToSpeechSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            voice: None,
            rate: 1.0,
            pitch: 1.0,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimplificationSettings {
    #[serde(default)]
    pub reading_level: ReadingLevel,
    #[serde(default = "default_true")]
    pub definitions: bool,
    #[serde(default)]
    pub chunked_content: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReadingLevel {
    #[default]
    Standard,
    Simplified,
    EasyRead,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlternativeFormat {
    Braille,
    LargePrint,
    Audio,
    Tactile,
    SimplifiedGraphics,
}

// ============================================================================
// Learning Style (UDL Alignment)
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LearningStyle {
    #[serde(default)]
    pub engagement: EngagementPreferences,
    #[serde(default)]
    pub representation: RepresentationPreferences,
    #[serde(default)]
    pub action_expression: ActionExpressionPreferences,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EngagementPreferences {
    #[serde(default)]
    pub interest_triggers: Vec<InterestTrigger>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collaboration_preference: Option<CollaborationPreference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedback_style: Option<FeedbackStyle>,
    #[serde(default)]
    pub self_regulation_support: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InterestTrigger {
    Choice,
    Relevance,
    Authenticity,
    Gamification,
    Social,
    Challenge,
    Creativity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CollaborationPreference {
    Individual,
    Pairs,
    SmallGroups,
    LargeGroups,
    Flexible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FeedbackStyle {
    Immediate,
    Delayed,
    Peer,
    Instructor,
    Automated,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RepresentationPreferences {
    #[serde(default)]
    pub preferred_modalities: Vec<LearningModality>,
    #[serde(default)]
    pub background_knowledge_support: bool,
    #[serde(default)]
    pub pattern_highlighting: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LearningModality {
    Visual,
    Auditory,
    Reading,
    Kinesthetic,
    Interactive,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ActionExpressionPreferences {
    #[serde(default)]
    pub expression_preferences: Vec<ExpressionMethod>,
    #[serde(default)]
    pub planning_support: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub progress_tracking: Option<ProgressTrackingStyle>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExpressionMethod {
    Written,
    Verbal,
    Visual,
    Multimedia,
    Demonstration,
    Portfolio,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProgressTrackingStyle {
    Detailed,
    Summary,
    Visual,
    Minimal,
}

// ============================================================================
// Assessment Accommodations
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssessmentAccommodations {
    #[serde(default)]
    pub timing: AssessmentTimingAccommodations,
    #[serde(default)]
    pub format: FormatAccommodations,
    #[serde(default)]
    pub environment: EnvironmentAccommodations,
    #[serde(default)]
    pub assistance: AssistanceAccommodations,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentTimingAccommodations {
    #[serde(default)]
    pub extended_time: bool,
    #[serde(default = "default_time_multiplier")]
    pub time_multiplier: f32,
    #[serde(default)]
    pub unlimited_time: bool,
    #[serde(default)]
    pub breaks_allowed: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub break_frequency_minutes: Option<u32>,
}

impl Default for AssessmentTimingAccommodations {
    fn default() -> Self {
        Self {
            extended_time: false,
            time_multiplier: 1.5,
            unlimited_time: false,
            breaks_allowed: false,
            break_frequency_minutes: None,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FormatAccommodations {
    #[serde(default)]
    pub alternative_formats: Vec<AssessmentFormat>,
    #[serde(default)]
    pub response_formats: Vec<ResponseFormat>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentFormat {
    LargePrint,
    Braille,
    Audio,
    Digital,
    Paper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseFormat {
    Written,
    Typed,
    Verbal,
    Scribe,
    Recorded,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EnvironmentAccommodations {
    #[serde(default)]
    pub separate_room: bool,
    #[serde(default)]
    pub reduced_distractions: bool,
    #[serde(default)]
    pub preferential_seating: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssistanceAccommodations {
    #[serde(default)]
    pub reader: bool,
    #[serde(default)]
    pub scribe: bool,
    #[serde(default)]
    pub calculator: bool,
    #[serde(default)]
    pub spell_check: bool,
    #[serde(default)]
    pub grammar_check: bool,
    #[serde(default)]
    pub dictionary: bool,
    #[serde(default)]
    pub notes: bool,
}

// ============================================================================
// Assistive Technology
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssistiveTechnology {
    #[serde(default)]
    pub devices: Vec<ATDevice>,
    #[serde(default)]
    pub software: Vec<ATSoftware>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ATDevice {
    pub device_type: ATDeviceType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_device_id: Option<Uuid>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ATDeviceType {
    ScreenReader,
    Magnifier,
    BrailleDisplay,
    AlternativeKeyboard,
    SwitchDevice,
    EyeTracker,
    HeadTracker,
    MouthStick,
    VoiceRecognition,
    AacDevice,
    HearingAid,
    CochlearImplant,
    FmSystem,
    Wheelchair,
    Bci,
    Other,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ATSoftware {
    pub software_type: ATSoftwareType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ATSoftwareType {
    ScreenReader,
    Magnification,
    VoiceControl,
    TextToSpeech,
    SpeechToText,
    ReadingAssistance,
    WritingAssistance,
    Organization,
    Focus,
    Other,
}

// ============================================================================
// WIA Integrations
// ============================================================================

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WIAIntegrations {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aac_profile_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bci_profile_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eye_gaze_profile_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wheelchair_profile_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smart_home_profile_id: Option<Uuid>,
    #[serde(default = "default_true")]
    pub sync_enabled: bool,
    #[serde(default)]
    pub cloud_backup: bool,
}

// ============================================================================
// Course Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Course {
    pub course_id: CourseId,
    pub schema_version: String,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub created_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub course_info: CourseInfo,
    #[serde(default)]
    pub accessibility_statement: AccessibilityStatement,
    #[serde(default)]
    pub modules: Vec<Module>,
    #[serde(default)]
    pub learning_outcomes: Vec<LearningOutcome>,
    #[serde(default)]
    pub accessibility_features: CourseAccessibilityFeatures,
    #[serde(default)]
    pub accommodations_available: CourseAccommodations,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CourseInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub institution: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instructor: Option<InstructorInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<CourseDuration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub level: Option<CourseLevel>,
    #[serde(default)]
    pub prerequisites: Vec<String>,
    #[serde(default)]
    pub tags: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InstructorInfo {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub office_hours: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CourseDuration {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weeks: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hours_per_week: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CourseLevel {
    Beginner,
    Intermediate,
    Advanced,
    Expert,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessibilityStatement {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub commitment: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wcag_conformance: Option<WCAGLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conformance_date: Option<String>,
    #[serde(default)]
    pub known_issues: Vec<AccessibilityIssue>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contact: Option<ContactInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accommodation_request_process: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WCAGLevel {
    #[serde(rename = "none")]
    None,
    A,
    AA,
    AAA,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityIssue {
    pub description: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impact: Option<ImpactLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workaround: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub planned_fix_date: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImpactLevel {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phone: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Module {
    pub module_id: ModuleId,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub sequence: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_minutes: Option<u32>,
    #[serde(default)]
    pub learning_objectives: Vec<String>,
    #[serde(default)]
    pub content_items: Vec<ContentItem>,
    #[serde(default)]
    pub assessments: Vec<AssessmentId>,
    #[serde(default)]
    pub udl_options: UDLOptions,
    #[serde(default)]
    pub prerequisites_modules: Vec<ModuleId>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentItem {
    pub content_id: ContentId,
    pub content_type: ContentType,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<u32>,
    #[serde(default = "default_true")]
    pub required: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_minutes: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(default)]
    pub accessibility_metadata: ContentAccessibilityMetadata,
    #[serde(default)]
    pub alternatives: Vec<ContentAlternative>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentType {
    Text,
    Video,
    Audio,
    Image,
    Document,
    Interactive,
    Simulation,
    ExternalLink,
    Discussion,
    LiveSession,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ContentAccessibilityMetadata {
    #[serde(default)]
    pub has_captions: bool,
    #[serde(default)]
    pub has_transcript: bool,
    #[serde(default)]
    pub has_audio_description: bool,
    #[serde(default)]
    pub has_sign_language: bool,
    #[serde(default)]
    pub alt_text_provided: bool,
    #[serde(default)]
    pub keyboard_accessible: bool,
    #[serde(default)]
    pub screen_reader_compatible: bool,
    #[serde(default)]
    pub color_independent: bool,
    #[serde(default)]
    pub no_flashing: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reading_level: Option<ContentReadingLevel>,
    #[serde(default)]
    pub hazards: Vec<ContentHazard>,
    #[serde(default)]
    pub accessibility_features: Vec<AccessibilityFeature>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentReadingLevel {
    Easy,
    Standard,
    Advanced,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentHazard {
    Flashing,
    Motion,
    Sound,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccessibilityFeature {
    AlternativeText,
    AudioDescription,
    Captions,
    DescribedMath,
    LongDescription,
    SignLanguage,
    Transcript,
    TactileObject,
    HighContrast,
    LargePrint,
    ReadableFonts,
    StructuredNavigation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentAlternative {
    pub alternative_type: AlternativeType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlternativeType {
    Transcript,
    Captions,
    AudioDescription,
    SignLanguage,
    Simplified,
    Braille,
    LargePrint,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UDLOptions {
    #[serde(default)]
    pub engagement_options: UDLEngagementOptions,
    #[serde(default)]
    pub representation_options: UDLRepresentationOptions,
    #[serde(default)]
    pub action_expression_options: UDLActionExpressionOptions,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UDLEngagementOptions {
    #[serde(default)]
    pub choice_available: bool,
    #[serde(default)]
    pub relevance_connections: Vec<String>,
    #[serde(default)]
    pub collaboration_options: Vec<CollaborationOption>,
    #[serde(default)]
    pub self_regulation_tools: Vec<SelfRegulationTool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CollaborationOption {
    Individual,
    Pairs,
    SmallGroup,
    LargeGroup,
    PeerReview,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SelfRegulationTool {
    GoalSetting,
    ProgressTracking,
    ReflectionPrompts,
    SelfAssessment,
    Checklists,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UDLRepresentationOptions {
    #[serde(default)]
    pub modalities_available: Vec<RepresentationModality>,
    #[serde(default)]
    pub language_supports: Vec<LanguageSupport>,
    #[serde(default)]
    pub comprehension_supports: Vec<ComprehensionSupport>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RepresentationModality {
    Text,
    Audio,
    Video,
    Graphics,
    Interactive,
    HandsOn,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LanguageSupport {
    Glossary,
    Definitions,
    Translations,
    SimplifiedLanguage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComprehensionSupport {
    Summaries,
    Outlines,
    ConceptMaps,
    Examples,
    NonExamples,
    Analogies,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UDLActionExpressionOptions {
    #[serde(default)]
    pub response_formats: Vec<UDLResponseFormat>,
    #[serde(default)]
    pub tools_available: Vec<LearningTool>,
    #[serde(default)]
    pub scaffolding: Vec<ScaffoldingType>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UDLResponseFormat {
    Written,
    Verbal,
    Visual,
    Multimedia,
    Demonstration,
    Portfolio,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LearningTool {
    SpellCheck,
    GrammarCheck,
    Calculator,
    DrawingTools,
    Recording,
    AssistiveTech,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScaffoldingType {
    Templates,
    GraphicOrganizers,
    SentenceStarters,
    Rubrics,
    Exemplars,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LearningOutcome {
    pub outcome_id: OutcomeId,
    pub description: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bloom_level: Option<BloomLevel>,
    #[serde(default)]
    pub assessment_methods: Vec<String>,
    #[serde(default)]
    pub related_modules: Vec<ModuleId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BloomLevel {
    Remember,
    Understand,
    Apply,
    Analyze,
    Evaluate,
    Create,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CourseAccessibilityFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lms_platform: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lms_accessibility_vpat: Option<String>,
    #[serde(default)]
    pub features: Vec<CourseFeature>,
    #[serde(default)]
    pub third_party_tools: Vec<ThirdPartyTool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CourseFeature {
    KeyboardNavigation,
    ScreenReaderSupport,
    CaptionsAllVideos,
    TranscriptsAllAudio,
    AudioDescriptions,
    AdjustableTextSize,
    HighContrastMode,
    ReadingOrderLogical,
    FormsAccessible,
    TablesAccessible,
    MathAccessible,
    MobileResponsive,
    OfflineAccess,
    DownloadableContent,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThirdPartyTool {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub purpose: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_info: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub known_issues: Option<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CourseAccommodations {
    #[serde(default)]
    pub timing: CourseTimingAccommodations,
    #[serde(default)]
    pub format: CourseFormatAccommodations,
    #[serde(default)]
    pub support: CourseSupportAccommodations,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub request_deadline: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub request_url: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CourseTimingAccommodations {
    #[serde(default = "default_true")]
    pub extended_time: bool,
    #[serde(default = "default_true")]
    pub flexible_deadlines: bool,
    #[serde(default = "default_true")]
    pub breaks: bool,
}

impl Default for CourseTimingAccommodations {
    fn default() -> Self {
        Self {
            extended_time: true,
            flexible_deadlines: true,
            breaks: true,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CourseFormatAccommodations {
    #[serde(default)]
    pub alternative_formats: Vec<AssessmentFormat>,
    #[serde(default)]
    pub alternative_assessments: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CourseSupportAccommodations {
    #[serde(default)]
    pub note_taking: bool,
    #[serde(default)]
    pub captioning_live: bool,
    #[serde(default)]
    pub sign_language_interpreter: bool,
    #[serde(default)]
    pub assistive_technology: bool,
    #[serde(default)]
    pub one_on_one_support: bool,
}

// ============================================================================
// Assessment Types
// ============================================================================

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Assessment {
    pub assessment_id: AssessmentId,
    pub schema_version: String,
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub assessment_type: AssessmentType,
    pub created_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub assessment_info: AssessmentInfo,
    #[serde(default)]
    pub timing: AssessmentTiming,
    #[serde(default)]
    pub questions: Vec<Question>,
    #[serde(default)]
    pub accessibility_settings: AssessmentAccessibilitySettings,
    #[serde(default)]
    pub accommodations_available: AssessmentAccommodationsAvailable,
    #[serde(default)]
    pub grading: GradingSettings,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentType {
    Quiz,
    Exam,
    Assignment,
    Project,
    Discussion,
    PeerReview,
    SelfAssessment,
    Portfolio,
    Presentation,
    Practical,
    Survey,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssessmentInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub course_id: Option<CourseId>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub module_id: Option<ModuleId>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instructions: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub purpose: Option<AssessmentPurpose>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weight: Option<f32>,
    #[serde(default = "default_one")]
    pub attempts_allowed: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub due_date: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub available_from: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub available_until: Option<DateTime<Utc>>,
    #[serde(default)]
    pub learning_outcomes: Vec<OutcomeId>,
}

fn default_one() -> u32 {
    1
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentPurpose {
    Formative,
    Summative,
    Diagnostic,
    Placement,
    Practice,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentTiming {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time_limit_minutes: Option<u32>,
    #[serde(default)]
    pub late_submission: LateSubmissionSettings,
    #[serde(default = "default_true")]
    pub auto_submit: bool,
    #[serde(default = "default_true")]
    pub show_timer: bool,
    #[serde(default)]
    pub timer_warning_minutes: Vec<u32>,
}

impl Default for AssessmentTiming {
    fn default() -> Self {
        Self {
            time_limit_minutes: None,
            late_submission: LateSubmissionSettings::default(),
            auto_submit: true,
            show_timer: true,
            timer_warning_minutes: vec![10, 5, 1],
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LateSubmissionSettings {
    #[serde(default)]
    pub allowed: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub penalty_percent_per_day: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_late_days: Option<u32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Question {
    pub question_id: QuestionId,
    pub question_type: QuestionType,
    pub content: QuestionContent,
    #[serde(default)]
    pub points: f32,
    #[serde(default = "default_true")]
    pub required: bool,
    #[serde(default)]
    pub options: Vec<AnswerOption>,
    #[serde(default)]
    pub correct_answers: Vec<String>,
    #[serde(default)]
    pub feedback: QuestionFeedback,
    #[serde(default)]
    pub hints: Vec<String>,
    #[serde(default)]
    pub accessibility: QuestionAccessibility,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bloom_level: Option<BloomLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub difficulty: Option<Difficulty>,
    #[serde(default)]
    pub tags: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QuestionType {
    MultipleChoice,
    MultipleSelect,
    TrueFalse,
    ShortAnswer,
    Essay,
    FillBlank,
    Matching,
    Ordering,
    Hotspot,
    DragDrop,
    FileUpload,
    Recording,
    Code,
    Math,
    Drawing,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct QuestionContent {
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plain_text: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub video_url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image: Option<QuestionImage>,
    #[serde(default)]
    pub attachments: Vec<QuestionAttachment>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub math_notation: Option<MathNotation>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuestionImage {
    pub url: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_text: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub long_description: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuestionAttachment {
    pub url: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attachment_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alt_text: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MathNotation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latex: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mathml: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spoken: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnswerOption {
    pub option_id: String,
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plain_text: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image: Option<QuestionImage>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feedback: Option<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct QuestionFeedback {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correct: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub incorrect: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partial: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub general: Option<String>,
    #[serde(default)]
    pub show_when: FeedbackTiming,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FeedbackTiming {
    Immediately,
    #[default]
    AfterSubmission,
    AfterDueDate,
    AfterAllAttempts,
    Never,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct QuestionAccessibility {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_reader_hint: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keyboard_instructions: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive_load: Option<CognitiveLoad>,
    #[serde(default)]
    pub requires_vision: bool,
    #[serde(default)]
    pub requires_hearing: bool,
    #[serde(default)]
    pub requires_fine_motor: bool,
    #[serde(default)]
    pub alternative_available: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alternative_question_id: Option<QuestionId>,
    #[serde(default)]
    pub hazards: Vec<QuestionHazard>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveLoad {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QuestionHazard {
    Flashing,
    Motion,
    Sound,
    TimePressure,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Difficulty {
    Easy,
    Medium,
    Hard,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentAccessibilitySettings {
    #[serde(default = "default_true")]
    pub keyboard_navigable: bool,
    #[serde(default = "default_true")]
    pub screen_reader_compatible: bool,
    #[serde(default = "default_true")]
    pub supports_zoom: bool,
    #[serde(default = "default_true")]
    pub color_independent: bool,
    #[serde(default)]
    pub question_order: QuestionOrder,
    #[serde(default)]
    pub navigation: NavigationType,
    #[serde(default = "default_true")]
    pub review_answers: bool,
    #[serde(default = "default_true")]
    pub save_progress: bool,
    #[serde(default = "default_true")]
    pub resume_allowed: bool,
}

impl Default for AssessmentAccessibilitySettings {
    fn default() -> Self {
        Self {
            keyboard_navigable: true,
            screen_reader_compatible: true,
            supports_zoom: true,
            color_independent: true,
            question_order: QuestionOrder::Fixed,
            navigation: NavigationType::Nonlinear,
            review_answers: true,
            save_progress: true,
            resume_allowed: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QuestionOrder {
    #[default]
    Fixed,
    Random,
    StudentChoice,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationType {
    Linear,
    #[default]
    Nonlinear,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssessmentAccommodationsAvailable {
    #[serde(default)]
    pub timing: TimingAccommodationsAvailable,
    #[serde(default)]
    pub presentation: PresentationAccommodationsAvailable,
    #[serde(default)]
    pub response: ResponseAccommodationsAvailable,
    #[serde(default)]
    pub setting: SettingAccommodationsAvailable,
    #[serde(default)]
    pub assistance: AssistanceAccommodationsAvailable,
    #[serde(default)]
    pub alternative_formats: Vec<AlternativeAssessmentFormat>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TimingAccommodationsAvailable {
    #[serde(default)]
    pub extended_time: ExtendedTimeSettings,
    #[serde(default)]
    pub unlimited_time: bool,
    #[serde(default = "default_true")]
    pub stop_clock_for_breaks: bool,
    #[serde(default = "default_true")]
    pub flexible_scheduling: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExtendedTimeSettings {
    #[serde(default = "default_true")]
    pub available: bool,
    #[serde(default = "default_multipliers")]
    pub multipliers: Vec<f32>,
}

fn default_multipliers() -> Vec<f32> {
    vec![1.5, 2.0, 2.5, 3.0]
}

impl Default for ExtendedTimeSettings {
    fn default() -> Self {
        Self {
            available: true,
            multipliers: vec![1.5, 2.0, 2.5, 3.0],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PresentationAccommodationsAvailable {
    #[serde(default = "default_true")]
    pub large_print: bool,
    #[serde(default = "default_true")]
    pub high_contrast: bool,
    #[serde(default = "default_true")]
    pub screen_reader: bool,
    #[serde(default = "default_true")]
    pub text_to_speech: bool,
    #[serde(default)]
    pub braille: bool,
    #[serde(default)]
    pub sign_language_video: bool,
    #[serde(default)]
    pub simplified_language: bool,
    #[serde(default = "default_true")]
    pub color_overlay: bool,
    #[serde(default = "default_true")]
    pub line_reader: bool,
}

impl Default for PresentationAccommodationsAvailable {
    fn default() -> Self {
        Self {
            large_print: true,
            high_contrast: true,
            screen_reader: true,
            text_to_speech: true,
            braille: false,
            sign_language_video: false,
            simplified_language: false,
            color_overlay: true,
            line_reader: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseAccommodationsAvailable {
    #[serde(default = "default_true")]
    pub keyboard_only: bool,
    #[serde(default = "default_true")]
    pub speech_to_text: bool,
    #[serde(default)]
    pub scribe: bool,
    #[serde(default = "default_true")]
    pub word_processor: bool,
    #[serde(default = "default_true")]
    pub spell_check: bool,
    #[serde(default = "default_true")]
    pub grammar_check: bool,
    #[serde(default = "default_true")]
    pub audio_response: bool,
    #[serde(default)]
    pub video_response: bool,
}

impl Default for ResponseAccommodationsAvailable {
    fn default() -> Self {
        Self {
            keyboard_only: true,
            speech_to_text: true,
            scribe: false,
            word_processor: true,
            spell_check: true,
            grammar_check: true,
            audio_response: true,
            video_response: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SettingAccommodationsAvailable {
    #[serde(default = "default_true")]
    pub separate_location: bool,
    #[serde(default = "default_true")]
    pub small_group: bool,
    #[serde(default = "default_true")]
    pub reduced_distractions: bool,
    #[serde(default)]
    pub breaks: BreaksSettings,
}

impl Default for SettingAccommodationsAvailable {
    fn default() -> Self {
        Self {
            separate_location: true,
            small_group: true,
            reduced_distractions: true,
            breaks: BreaksSettings::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BreaksSettings {
    #[serde(default = "default_true")]
    pub allowed: bool,
    #[serde(default = "default_break_frequencies")]
    pub frequency_minutes: Vec<u32>,
    #[serde(default = "default_break_durations")]
    pub duration_minutes: Vec<u32>,
}

fn default_break_frequencies() -> Vec<u32> {
    vec![15, 30, 45, 60]
}

fn default_break_durations() -> Vec<u32> {
    vec![5, 10, 15]
}

impl Default for BreaksSettings {
    fn default() -> Self {
        Self {
            allowed: true,
            frequency_minutes: vec![15, 30, 45, 60],
            duration_minutes: vec![5, 10, 15],
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssistanceAccommodationsAvailable {
    #[serde(default)]
    pub calculator: CalculatorSettings,
    #[serde(default)]
    pub dictionary: DictionarySettings,
    #[serde(default)]
    pub formula_sheet: bool,
    #[serde(default)]
    pub notes: bool,
    #[serde(default = "default_true")]
    pub reader: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CalculatorSettings {
    #[serde(default)]
    pub basic: bool,
    #[serde(default)]
    pub scientific: bool,
    #[serde(default)]
    pub graphing: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DictionarySettings {
    #[serde(default)]
    pub standard: bool,
    #[serde(default)]
    pub bilingual: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlternativeAssessmentFormat {
    Paper,
    Oral,
    PracticalDemonstration,
    Portfolio,
    Project,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GradingSettings {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_points: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub passing_score: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub passing_percentage: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grading_type: Option<GradingType>,
    #[serde(default = "default_true")]
    pub partial_credit: bool,
    #[serde(default)]
    pub negative_scoring: bool,
    #[serde(default)]
    pub show_score: FeedbackTiming,
    #[serde(default)]
    pub show_correct_answers: FeedbackTiming,
    #[serde(default)]
    pub rubric: Vec<RubricCriterion>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GradingType {
    Automatic,
    Manual,
    Hybrid,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RubricCriterion {
    pub criterion: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub max_points: f32,
    #[serde(default)]
    pub levels: Vec<RubricLevel>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RubricLevel {
    pub level: String,
    pub points: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}
