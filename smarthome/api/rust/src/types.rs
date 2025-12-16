//! WIA Smart Home Type Definitions
//! 弘益人間 - Benefit All Humanity

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Type Aliases
// ============================================================================

/// Device identifier type alias
pub type DeviceId = Uuid;

/// Zone identifier type alias
pub type ZoneId = Uuid;

/// Home identifier type alias
pub type HomeId = Uuid;

/// User profile identifier type alias
pub type ProfileId = Uuid;

// ============================================================================
// Input/Output Modalities
// ============================================================================

/// Input modality types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputModality {
    Voice,
    Touch,
    Switch,
    Gaze,
    Gesture,
    Bci,
    SipPuff,
    Keyboard,
    Remote,
}

/// Output modality types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputModality {
    VisualScreen,
    VisualLed,
    AudioTts,
    AudioTone,
    Haptic,
    Braille,
}

// ============================================================================
// Disability Types
// ============================================================================

/// Disability type categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisabilityType {
    VisualBlind,
    VisualLowVision,
    VisualColorBlind,
    HearingDeaf,
    HearingHard,
    MotorLimited,
    MotorTremor,
    MotorParalysis,
    CognitiveLearning,
    CognitiveMemory,
    CognitiveAttention,
    Speech,
    Multiple,
}

/// Severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SeverityLevel {
    Mild,
    Moderate,
    Severe,
    Profound,
}

/// WCAG conformance level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum WcagLevel {
    A,
    AA,
    AAA,
}

impl Default for WcagLevel {
    fn default() -> Self {
        WcagLevel::AA
    }
}

// ============================================================================
// Assistive Technology
// ============================================================================

/// Assistive technology type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssistiveTechType {
    ScreenReader,
    Magnifier,
    BrailleDisplay,
    HearingAid,
    CochlearImplant,
    SwitchDevice,
    EyeTracker,
    HeadTracker,
    SipPuffDevice,
    BciDevice,
    VoiceAmplifier,
}

/// Platform type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Platform {
    Ios,
    Android,
    Windows,
    Macos,
    Linux,
    Universal,
}

/// Assistive technology configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssistiveTechnology {
    #[serde(rename = "type")]
    pub tech_type: AssistiveTechType,
    pub name: Option<String>,
    pub platform: Option<Platform>,
    pub version: Option<String>,
    #[serde(default)]
    pub configuration: HashMap<String, serde_json::Value>,
}

// ============================================================================
// Accessibility Requirements
// ============================================================================

/// Visual accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualNeeds {
    #[serde(default)]
    pub screen_reader_required: bool,
    #[serde(default)]
    pub magnification_required: bool,
    #[serde(default)]
    pub high_contrast_required: bool,
    #[serde(default)]
    pub audio_descriptions_required: bool,
}

/// Auditory accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AuditoryNeeds {
    #[serde(default)]
    pub visual_alerts_required: bool,
    #[serde(default)]
    pub captions_required: bool,
    #[serde(default)]
    pub sign_language_preferred: bool,
    #[serde(default)]
    pub hearing_aid_compatible: bool,
}

/// Motor accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorNeeds {
    #[serde(default)]
    pub voice_control_required: bool,
    #[serde(default)]
    pub switch_access_required: bool,
    #[serde(default)]
    pub dwell_selection_required: bool,
    #[serde(default)]
    pub large_targets_required: bool,
    #[serde(default)]
    pub reduced_motion_required: bool,
}

/// Cognitive accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CognitiveNeeds {
    #[serde(default)]
    pub simplified_interface_required: bool,
    #[serde(default)]
    pub consistent_navigation_required: bool,
    #[serde(default)]
    pub error_prevention_required: bool,
    #[serde(default)]
    pub reading_assistance_required: bool,
}

/// Specific accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SpecificNeeds {
    #[serde(default)]
    pub visual: VisualNeeds,
    #[serde(default)]
    pub auditory: AuditoryNeeds,
    #[serde(default)]
    pub motor: MotorNeeds,
    #[serde(default)]
    pub cognitive: CognitiveNeeds,
}

/// Required modalities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RequiredModalities {
    pub input: Vec<InputModality>,
    pub output: Vec<OutputModality>,
}

/// Accessibility requirements
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessibilityRequirements {
    #[serde(default)]
    pub primary_disabilities: Vec<DisabilityType>,
    #[serde(default)]
    pub severity_levels: HashMap<String, SeverityLevel>,
    #[serde(default)]
    pub assistive_technologies: Vec<AssistiveTechnology>,
    pub required_modalities: Option<RequiredModalities>,
    #[serde(default)]
    pub wcag_level: WcagLevel,
    #[serde(default)]
    pub specific_needs: SpecificNeeds,
}

// ============================================================================
// User Profile
// ============================================================================

/// Voice settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSettings {
    pub wake_word: Option<String>,
    #[serde(default = "default_rate")]
    pub speech_rate: f32,
    #[serde(default = "default_rate")]
    pub pitch: f32,
    pub voice_id: Option<String>,
}

fn default_rate() -> f32 {
    1.0
}

impl Default for VoiceSettings {
    fn default() -> Self {
        Self {
            wake_word: None,
            speech_rate: 1.0,
            pitch: 1.0,
            voice_id: None,
        }
    }
}

/// Visual settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualSettings {
    #[serde(default)]
    pub high_contrast: bool,
    #[serde(default)]
    pub large_text: bool,
    #[serde(default = "default_rate")]
    pub text_scale: f32,
    #[serde(default)]
    pub reduce_motion: bool,
}

/// Timing settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingSettings {
    #[serde(default = "default_response_timeout")]
    pub response_timeout_ms: u32,
    #[serde(default = "default_dwell_time")]
    pub dwell_time_ms: u32,
    #[serde(default)]
    pub confirmation_required: bool,
}

fn default_response_timeout() -> u32 {
    5000
}

fn default_dwell_time() -> u32 {
    1000
}

impl Default for TimingSettings {
    fn default() -> Self {
        Self {
            response_timeout_ms: 5000,
            dwell_time_ms: 1000,
            confirmation_required: false,
        }
    }
}

/// Interaction preferences
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct InteractionPreferences {
    #[serde(default)]
    pub preferred_input_modalities: Vec<InputModality>,
    #[serde(default)]
    pub preferred_output_modalities: Vec<OutputModality>,
    #[serde(default)]
    pub voice_settings: VoiceSettings,
    #[serde(default)]
    pub visual_settings: VisualSettings,
    #[serde(default)]
    pub timing_settings: TimingSettings,
}

/// Quiet hours configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuietHours {
    #[serde(default = "default_true")]
    pub enabled: bool,
    pub start_time: Option<String>,
    pub end_time: Option<String>,
}

fn default_true() -> bool {
    true
}

impl Default for QuietHours {
    fn default() -> Self {
        Self {
            enabled: true,
            start_time: Some("22:00".to_string()),
            end_time: Some("07:00".to_string()),
        }
    }
}

/// Emergency contact
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    pub name: String,
    pub phone: String,
    pub relationship: Option<String>,
    #[serde(default = "default_true")]
    pub notify_on_emergency: bool,
    pub email: Option<String>,
}

/// Priority overrides
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PriorityOverrides {
    #[serde(default = "default_true")]
    pub critical_always_notify: bool,
    #[serde(default)]
    pub emergency_contacts: Vec<EmergencyContact>,
}

/// Notification preferences
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NotificationPreferences {
    #[serde(default)]
    pub quiet_hours: QuietHours,
    #[serde(default)]
    pub priority_overrides: PriorityOverrides,
}

/// Data collection settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataCollectionSettings {
    #[serde(default)]
    pub usage_analytics: bool,
    #[serde(default)]
    pub voice_recording: bool,
    #[serde(default)]
    pub location_history: bool,
}

/// Data sharing settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataSharingSettings {
    #[serde(default)]
    pub with_caregivers: bool,
    #[serde(default)]
    pub with_healthcare: bool,
}

/// Privacy settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PrivacySettings {
    #[serde(default)]
    pub data_collection: DataCollectionSettings,
    #[serde(default)]
    pub data_sharing: DataSharingSettings,
}

/// Personal info
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PersonalInfo {
    pub name: Option<String>,
    #[serde(default = "default_language")]
    pub preferred_language: String,
    #[serde(default = "default_timezone")]
    pub timezone: String,
}

fn default_language() -> String {
    "ko-KR".to_string()
}

fn default_timezone() -> String {
    "Asia/Seoul".to_string()
}

/// User profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserProfile {
    pub profile_id: Uuid,
    pub version: String,
    pub created_at: Option<DateTime<Utc>>,
    pub updated_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub personal_info: PersonalInfo,
    pub accessibility_requirements: AccessibilityRequirements,
    #[serde(default)]
    pub interaction_preferences: InteractionPreferences,
    #[serde(default)]
    pub notification_preferences: NotificationPreferences,
    #[serde(default)]
    pub privacy_settings: PrivacySettings,
}

impl UserProfile {
    pub fn new(profile_id: Uuid) -> Self {
        Self {
            profile_id,
            version: "1.0.0".to_string(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            personal_info: PersonalInfo::default(),
            accessibility_requirements: AccessibilityRequirements::default(),
            interaction_preferences: InteractionPreferences::default(),
            notification_preferences: NotificationPreferences::default(),
            privacy_settings: PrivacySettings::default(),
        }
    }
}

// ============================================================================
// Device Types
// ============================================================================

/// Device type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceType {
    Light,
    LightDimmer,
    LightColor,
    Switch,
    Outlet,
    Thermostat,
    Fan,
    AirPurifier,
    Humidifier,
    Lock,
    Doorbell,
    Camera,
    Alarm,
    Blind,
    Curtain,
    GarageDoor,
    MotionSensor,
    ContactSensor,
    TemperatureSensor,
    HumiditySensor,
    LeakSensor,
    SmokeDetector,
    CoDetector,
    Speaker,
    Tv,
    MediaPlayer,
    Vacuum,
    Washer,
    Dryer,
    Refrigerator,
    Oven,
    Hub,
    Bridge,
    Other,
}

/// Device status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum DeviceStatus {
    Online,
    #[default]
    Offline,
    Error,
    Updating,
}

/// Device capabilities
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceCapabilities {
    #[serde(default)]
    pub on_off: bool,
    #[serde(default)]
    pub dimming: bool,
    #[serde(default)]
    pub color_control: bool,
    #[serde(default)]
    pub color_temperature: bool,
    #[serde(default)]
    pub temperature_sensing: bool,
    #[serde(default)]
    pub humidity_sensing: bool,
    #[serde(default)]
    pub motion_sensing: bool,
    #[serde(default)]
    pub contact_sensing: bool,
    #[serde(default)]
    pub lock_control: bool,
    #[serde(default)]
    pub thermostat: bool,
    #[serde(default)]
    pub fan_control: bool,
    #[serde(default)]
    pub window_covering: bool,
    #[serde(default)]
    pub media_playback: bool,
    #[serde(default)]
    pub camera: bool,
    #[serde(default)]
    pub doorbell: bool,
}

/// Voice command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommand {
    pub command: String,
    #[serde(default)]
    pub aliases: Vec<String>,
    pub action: String,
    #[serde(default)]
    pub parameters: HashMap<String, serde_json::Value>,
    pub confirmation_phrase: Option<String>,
}

/// Audio feedback settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioFeedback {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default = "default_volume")]
    pub volume: u8,
    pub tts_voice: Option<String>,
    #[serde(default)]
    pub tones: HashMap<String, String>,
}

fn default_volume() -> u8 {
    70
}

impl Default for AudioFeedback {
    fn default() -> Self {
        Self {
            enabled: true,
            volume: 70,
            tts_voice: None,
            tones: HashMap::new(),
        }
    }
}

/// Visual feedback settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualFeedback {
    #[serde(default)]
    pub led_indicators: bool,
    #[serde(default)]
    pub high_contrast: bool,
    #[serde(default)]
    pub large_icons: bool,
}

/// Haptic feedback settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HapticFeedback {
    #[serde(default)]
    pub enabled: bool,
    pub pattern: Option<String>,
    #[serde(default = "default_intensity")]
    pub intensity: u8,
}

fn default_intensity() -> u8 {
    50
}

/// Device timing settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceTimingSettings {
    #[serde(default = "default_response_timeout")]
    pub response_timeout_ms: u32,
    #[serde(default)]
    pub confirmation_required: bool,
    #[serde(default = "default_dwell_time")]
    pub dwell_time_ms: u32,
}

impl Default for DeviceTimingSettings {
    fn default() -> Self {
        Self {
            response_timeout_ms: 5000,
            confirmation_required: false,
            dwell_time_ms: 1000,
        }
    }
}

/// Device accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceAccessibilityFeatures {
    #[serde(default)]
    pub supported_inputs: Vec<InputModality>,
    #[serde(default)]
    pub supported_outputs: Vec<OutputModality>,
    #[serde(default)]
    pub voice_commands: Vec<VoiceCommand>,
    #[serde(default)]
    pub audio_feedback: AudioFeedback,
    #[serde(default)]
    pub visual_feedback: VisualFeedback,
    #[serde(default)]
    pub haptic_feedback: HapticFeedback,
    #[serde(default)]
    pub timing: DeviceTimingSettings,
}

/// Device metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceMetadata {
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    pub serial_number: Option<String>,
    pub installation_date: Option<String>,
    pub last_maintenance: Option<String>,
}

/// Smart home device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Device {
    pub device_id: Uuid,
    pub matter_node_id: Option<String>,
    pub vendor_id: Option<String>,
    pub product_id: Option<String>,
    pub device_type: DeviceType,
    pub name: String,
    pub zone_id: Option<Uuid>,
    pub home_id: Option<Uuid>,
    #[serde(default)]
    pub status: DeviceStatus,
    pub firmware_version: Option<String>,
    #[serde(default)]
    pub capabilities: DeviceCapabilities,
    #[serde(default)]
    pub current_state: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub accessibility_features: DeviceAccessibilityFeatures,
    #[serde(default)]
    pub metadata: DeviceMetadata,
}

impl Device {
    pub fn new(device_id: Uuid, device_type: DeviceType, name: String) -> Self {
        Self {
            device_id,
            matter_node_id: None,
            vendor_id: None,
            product_id: None,
            device_type,
            name,
            zone_id: None,
            home_id: None,
            status: DeviceStatus::Offline,
            firmware_version: None,
            capabilities: DeviceCapabilities::default(),
            current_state: HashMap::new(),
            accessibility_features: DeviceAccessibilityFeatures::default(),
            metadata: DeviceMetadata::default(),
        }
    }
}

// ============================================================================
// Home and Zone
// ============================================================================

/// Hub type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HubType {
    Matter,
    Zigbee,
    Zwave,
    Wifi,
    Hybrid,
}

/// Hub info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HubInfo {
    pub hub_type: HubType,
    pub hub_id: Option<String>,
    pub firmware_version: Option<String>,
    pub ip_address: Option<String>,
}

/// Address
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Address {
    pub street: Option<String>,
    pub city: Option<String>,
    pub state: Option<String>,
    pub postal_code: Option<String>,
    pub country: Option<String>,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
}

/// Default modalities
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DefaultModalities {
    #[serde(default)]
    pub input: Vec<String>,
    #[serde(default)]
    pub output: Vec<String>,
}

/// TTS settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TtsSettings {
    pub default_voice: Option<String>,
    #[serde(default = "default_language")]
    pub default_language: String,
    #[serde(default = "default_rate")]
    pub speech_rate: f32,
}

impl Default for TtsSettings {
    fn default() -> Self {
        Self {
            default_voice: None,
            default_language: "ko-KR".to_string(),
            speech_rate: 1.0,
        }
    }
}

/// Caregiver permission
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaregiverPermission {
    ViewStatus,
    ControlDevices,
    ModifySettings,
    ReceiveAlerts,
}

/// Caregiver access settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CaregiverAccess {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub caregiver_profiles: Vec<Uuid>,
    #[serde(default)]
    pub permissions: Vec<CaregiverPermission>,
}

/// Home accessibility settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HomeAccessibilitySettings {
    #[serde(default)]
    pub default_modalities: DefaultModalities,
    #[serde(default = "default_volume")]
    pub global_volume: u8,
    #[serde(default = "default_brightness")]
    pub global_brightness: u8,
    #[serde(default)]
    pub tts_settings: TtsSettings,
    #[serde(default)]
    pub emergency_contacts: Vec<EmergencyContact>,
    #[serde(default)]
    pub caregiver_access: CaregiverAccess,
}

fn default_brightness() -> u8 {
    80
}

/// Smart home
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Home {
    pub home_id: Uuid,
    pub name: String,
    #[serde(default)]
    pub address: Address,
    #[serde(default = "default_timezone")]
    pub timezone: String,
    pub owner_profile_id: Uuid,
    #[serde(default)]
    pub member_profiles: Vec<Uuid>,
    #[serde(default)]
    pub zones: Vec<Uuid>,
    #[serde(default)]
    pub devices: Vec<Uuid>,
    pub hub_info: Option<HubInfo>,
    #[serde(default)]
    pub accessibility_settings: HomeAccessibilitySettings,
    pub created_at: Option<DateTime<Utc>>,
    pub updated_at: Option<DateTime<Utc>>,
}

impl Home {
    pub fn new(home_id: Uuid, name: String, owner_profile_id: Uuid) -> Self {
        Self {
            home_id,
            name,
            address: Address::default(),
            timezone: "Asia/Seoul".to_string(),
            owner_profile_id,
            member_profiles: Vec::new(),
            zones: Vec::new(),
            devices: Vec::new(),
            hub_info: None,
            accessibility_settings: HomeAccessibilitySettings::default(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
        }
    }
}

/// Zone type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZoneType {
    LivingRoom,
    Bedroom,
    Kitchen,
    Bathroom,
    Office,
    Garage,
    Garden,
    Entrance,
    Hallway,
    Basement,
    Attic,
    Other,
}

/// Zone
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Zone {
    pub zone_id: Uuid,
    pub home_id: Uuid,
    pub name: String,
    pub zone_type: ZoneType,
    #[serde(default)]
    pub devices: Vec<Uuid>,
    #[serde(default)]
    pub accessibility_overrides: HashMap<String, serde_json::Value>,
    pub created_at: Option<DateTime<Utc>>,
    pub updated_at: Option<DateTime<Utc>>,
}

impl Zone {
    pub fn new(zone_id: Uuid, home_id: Uuid, name: String, zone_type: ZoneType) -> Self {
        Self {
            zone_id,
            home_id,
            name,
            zone_type,
            devices: Vec::new(),
            accessibility_overrides: HashMap::new(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
        }
    }
}

// ============================================================================
// Automation
// ============================================================================

/// Trigger type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TriggerType {
    Time,
    Sunrise,
    Sunset,
    DeviceState,
    SensorValue,
    Location,
    VoiceCommand,
    Manual,
    Schedule,
}

/// Trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Trigger {
    #[serde(rename = "type")]
    pub trigger_type: TriggerType,
    #[serde(default)]
    pub config: HashMap<String, serde_json::Value>,
}

/// Condition type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConditionType {
    TimeRange,
    DayOfWeek,
    DeviceState,
    SensorValue,
    ZoneOccupancy,
    UserPresence,
    SunPosition,
    Weather,
}

/// Condition operator
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ConditionOperator {
    #[default]
    And,
    Or,
    Not,
}

/// Condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Condition {
    #[serde(rename = "type")]
    pub condition_type: ConditionType,
    #[serde(default)]
    pub config: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub operator: ConditionOperator,
}

/// Action type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionType {
    DeviceControl,
    SceneActivate,
    Notification,
    Delay,
    ConditionCheck,
    Webhook,
    VoiceAnnouncement,
}

/// Action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Action {
    #[serde(rename = "type")]
    pub action_type: ActionType,
    pub device_id: Option<Uuid>,
    pub command: Option<String>,
    #[serde(default)]
    pub parameters: HashMap<String, serde_json::Value>,
    pub delay_ms: Option<u32>,
}

/// Override priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum OverridePriority {
    #[default]
    Low,
    Medium,
    High,
    Critical,
}

/// Safe mode behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum SafeModeBehavior {
    Execute,
    Skip,
    #[default]
    Confirm,
}

/// Automation accessibility settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationAccessibilitySettings {
    #[serde(default = "default_true")]
    pub announce_activation: bool,
    pub announcement_text: Option<String>,
    #[serde(default)]
    pub confirmation_required: bool,
    #[serde(default)]
    pub override_priority: OverridePriority,
    #[serde(default)]
    pub safe_mode_behavior: SafeModeBehavior,
}

impl Default for AutomationAccessibilitySettings {
    fn default() -> Self {
        Self {
            announce_activation: true,
            announcement_text: None,
            confirmation_required: false,
            override_priority: OverridePriority::Low,
            safe_mode_behavior: SafeModeBehavior::Confirm,
        }
    }
}

/// Day of week
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DayOfWeek {
    Mon,
    Tue,
    Wed,
    Thu,
    Fri,
    Sat,
    Sun,
}

/// Automation schedule
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AutomationSchedule {
    #[serde(default)]
    pub active_days: Vec<DayOfWeek>,
    pub active_start_time: Option<String>,
    pub active_end_time: Option<String>,
    #[serde(default = "default_true")]
    pub respect_quiet_hours: bool,
}

/// Automation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Automation {
    pub automation_id: Uuid,
    pub name: String,
    pub description: Option<String>,
    #[serde(default = "default_true")]
    pub enabled: bool,
    pub home_id: Uuid,
    #[serde(default)]
    pub zone_ids: Vec<Uuid>,
    pub trigger: Trigger,
    #[serde(default)]
    pub conditions: Vec<Condition>,
    pub actions: Vec<Action>,
    #[serde(default)]
    pub accessibility_settings: AutomationAccessibilitySettings,
    #[serde(default)]
    pub schedule: AutomationSchedule,
    pub created_at: Option<DateTime<Utc>>,
    pub updated_at: Option<DateTime<Utc>>,
    pub last_triggered_at: Option<DateTime<Utc>>,
    #[serde(default)]
    pub trigger_count: u64,
}

impl Automation {
    pub fn new(
        automation_id: Uuid,
        name: String,
        home_id: Uuid,
        trigger: Trigger,
        actions: Vec<Action>,
    ) -> Self {
        Self {
            automation_id,
            name,
            description: None,
            enabled: true,
            home_id,
            zone_ids: Vec::new(),
            trigger,
            conditions: Vec::new(),
            actions,
            accessibility_settings: AutomationAccessibilitySettings::default(),
            schedule: AutomationSchedule::default(),
            created_at: Some(Utc::now()),
            updated_at: Some(Utc::now()),
            last_triggered_at: None,
            trigger_count: 0,
        }
    }
}

// ============================================================================
// Notification
// ============================================================================

/// Notification type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationType {
    Alert,
    Warning,
    Info,
    Reminder,
    Emergency,
}

/// Notification priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationPriority {
    Critical,
    High,
    Medium,
    Low,
}

/// Notification source
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NotificationSource {
    pub device_id: Option<Uuid>,
    pub device_type: Option<String>,
    pub device_name: Option<String>,
    pub zone_id: Option<Uuid>,
    pub zone_name: Option<String>,
    pub automation_id: Option<Uuid>,
}

/// Flash pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum FlashPattern {
    #[default]
    None,
    Pulse,
    Blink,
    Strobe,
}

/// Audio delivery
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AudioDelivery {
    pub tts_text: Option<String>,
    #[serde(default = "default_language")]
    pub tts_language: String,
    pub tts_voice: Option<String>,
    pub tts_rate: Option<f32>,
    pub tone: Option<String>,
    pub volume: Option<u8>,
    #[serde(default = "default_one")]
    pub repeat_count: u8,
    pub repeat_interval_ms: Option<u32>,
}

fn default_one() -> u8 {
    1
}

/// Visual delivery
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualDelivery {
    pub title: Option<String>,
    pub body: Option<String>,
    pub icon: Option<String>,
    pub image_url: Option<String>,
    pub color: Option<String>,
    #[serde(default)]
    pub flash_pattern: FlashPattern,
    pub duration_ms: Option<u32>,
}

/// Haptic pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HapticPattern {
    ShortTap,
    DoubleTap,
    LongBuzz,
    Pulse,
    Escalating,
}

/// Haptic delivery
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticDelivery {
    pub pattern: HapticPattern,
    #[serde(default = "default_intensity")]
    pub intensity: u8,
    #[serde(default = "default_one")]
    pub repeat: u8,
}

/// Notification delivery
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationDelivery {
    pub modalities: Vec<OutputModality>,
    pub audio: Option<AudioDelivery>,
    pub visual: Option<VisualDelivery>,
    pub haptic: Option<HapticDelivery>,
}

/// Notification action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationAction {
    pub id: String,
    pub label: String,
    #[serde(default)]
    pub label_i18n: HashMap<String, String>,
    pub command: String,
    #[serde(default)]
    pub parameters: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub confirmation_required: bool,
    pub icon: Option<String>,
}

/// Notification targeting
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NotificationTargeting {
    #[serde(default)]
    pub user_ids: Vec<Uuid>,
    #[serde(default)]
    pub zone_ids: Vec<Uuid>,
    #[serde(default)]
    pub all_users: bool,
}

/// Notification scheduling
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NotificationScheduling {
    pub deliver_at: Option<DateTime<Utc>>,
    pub expires_at: Option<DateTime<Utc>>,
    #[serde(default = "default_true")]
    pub respect_quiet_hours: bool,
    #[serde(default)]
    pub snooze_options: Vec<u32>,
}

/// Notification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Notification {
    pub notification_id: Uuid,
    #[serde(rename = "type")]
    pub notification_type: NotificationType,
    pub priority: NotificationPriority,
    #[serde(default)]
    pub source: NotificationSource,
    pub message: HashMap<String, String>,
    pub delivery: NotificationDelivery,
    #[serde(default)]
    pub actions: Vec<NotificationAction>,
    #[serde(default)]
    pub targeting: NotificationTargeting,
    #[serde(default)]
    pub scheduling: NotificationScheduling,
    pub timestamp: Option<DateTime<Utc>>,
    pub read_at: Option<DateTime<Utc>>,
    pub dismissed_at: Option<DateTime<Utc>>,
}

impl Notification {
    pub fn new(
        notification_id: Uuid,
        notification_type: NotificationType,
        priority: NotificationPriority,
        message: String,
        delivery: NotificationDelivery,
    ) -> Self {
        let mut msg_map = HashMap::new();
        msg_map.insert("default".to_string(), message);

        Self {
            notification_id,
            notification_type,
            priority,
            source: NotificationSource::default(),
            message: msg_map,
            delivery,
            actions: Vec::new(),
            targeting: NotificationTargeting::default(),
            scheduling: NotificationScheduling::default(),
            timestamp: Some(Utc::now()),
            read_at: None,
            dismissed_at: None,
        }
    }
}

// ============================================================================
// Accessibility Event
// ============================================================================

/// Event type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventType {
    VoiceCommandReceived,
    VoiceCommandExecuted,
    VoiceCommandFailed,
    SwitchActivated,
    GestureDetected,
    EyeGazeSelection,
    BciCommand,
    DeviceStateChanged,
    AutomationTriggered,
    AutomationExecuted,
    AutomationFailed,
    SceneActivated,
    EmergencyTriggered,
    EmergencyResolved,
    FallDetected,
    InactivityAlert,
    MedicationReminder,
    CaregiverAlertSent,
    CaregiverResponded,
    SafeModeActivated,
    SafeModeDeactivated,
    QuietHoursStarted,
    QuietHoursEnded,
    UserEnteredZone,
    UserLeftZone,
    AccessibilitySettingsChanged,
    DeviceAccessibilityError,
    SystemAnnouncement,
}

/// Event source type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventSourceType {
    Device,
    User,
    Automation,
    System,
    Caregiver,
}

/// Event source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventSource {
    #[serde(rename = "type")]
    pub source_type: EventSourceType,
    pub id: Uuid,
    pub name: Option<String>,
}

/// Event target type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventTargetType {
    Device,
    Zone,
    Home,
    User,
}

/// Event target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventTarget {
    #[serde(rename = "type")]
    pub target_type: EventTargetType,
    pub id: Uuid,
}

/// Event severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum EventSeverity {
    #[default]
    Info,
    Warning,
    Alert,
    Emergency,
}

/// Accessibility context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessibilityContext {
    #[serde(default)]
    pub modalities_used: Vec<InputModality>,
    pub assistive_tech: Option<String>,
    pub response_time_ms: Option<u32>,
    #[serde(default)]
    pub accessibility_features_active: Vec<String>,
}

/// Notification sent record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationSent {
    pub notification_id: Uuid,
    pub recipient_id: Uuid,
    pub channel: OutputModality,
    pub delivered: bool,
}

/// Accessibility event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityEvent {
    pub event_id: Uuid,
    pub event_type: EventType,
    pub timestamp: DateTime<Utc>,
    pub source: EventSource,
    pub target: Option<EventTarget>,
    pub home_id: Option<Uuid>,
    pub zone_id: Option<Uuid>,
    pub user_id: Option<Uuid>,
    #[serde(default)]
    pub severity: EventSeverity,
    #[serde(default)]
    pub data: HashMap<String, serde_json::Value>,
    #[serde(default)]
    pub accessibility_context: AccessibilityContext,
    #[serde(default)]
    pub notifications_sent: Vec<NotificationSent>,
    #[serde(default)]
    pub requires_acknowledgment: bool,
    pub acknowledged_at: Option<DateTime<Utc>>,
    pub acknowledged_by: Option<Uuid>,
}

impl AccessibilityEvent {
    pub fn new(event_type: EventType, source: EventSource) -> Self {
        Self {
            event_id: Uuid::new_v4(),
            event_type,
            timestamp: Utc::now(),
            source,
            target: None,
            home_id: None,
            zone_id: None,
            user_id: None,
            severity: EventSeverity::Info,
            data: HashMap::new(),
            accessibility_context: AccessibilityContext::default(),
            notifications_sent: Vec::new(),
            requires_acknowledgment: false,
            acknowledged_at: None,
            acknowledged_by: None,
        }
    }
}
