//! Type definitions for WIA Medical Device Accessibility
//!
//! These types correspond to the JSON Schema definitions in Phase 1.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

// ============================================================================
// Medical Device Accessibility Profile
// ============================================================================

/// Complete Medical Device Accessibility Profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicalDeviceAccessibilityProfile {
    /// Unique profile identifier
    pub profile_id: String,

    /// Schema version
    pub profile_version: String,

    /// Creation timestamp
    pub created_at: DateTime<Utc>,

    /// Last update timestamp
    pub updated_at: DateTime<Utc>,

    /// Device information
    pub device: MedicalDeviceInfo,

    /// Accessibility features
    pub accessibility: DeviceAccessibilityFeatures,

    /// Regulatory compliance
    pub compliance: RegulatoryCompliance,

    /// WIA integration settings
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_integration: Option<WIAIntegration>,

    /// Profile metadata
    pub metadata: ProfileMetadata,
}

impl Default for MedicalDeviceAccessibilityProfile {
    fn default() -> Self {
        Self {
            profile_id: format!("mdap_{}", uuid::Uuid::new_v4().simple()),
            profile_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: Utc::now(),
            device: MedicalDeviceInfo::default(),
            accessibility: DeviceAccessibilityFeatures::default(),
            compliance: RegulatoryCompliance::default(),
            wia_integration: None,
            metadata: ProfileMetadata::default(),
        }
    }
}

// ============================================================================
// Medical Device Information
// ============================================================================

/// Medical device basic information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicalDeviceInfo {
    /// Device unique identifier
    pub device_id: String,

    /// Manufacturer name
    pub manufacturer: String,

    /// Model number/name
    pub model: String,

    /// Device display name
    pub device_name: String,

    /// FDA classification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fda_classification: Option<FDAClassification>,

    /// Device type
    pub device_type: MedicalDeviceType,

    /// Device category
    pub device_category: DeviceCategory,

    /// Use environment
    pub use_environment: UseEnvironment,

    /// Intended users
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intended_users: Option<Vec<IntendedUser>>,

    /// Device interface
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interface: Option<DeviceInterface>,

    /// Connectivity options
    #[serde(skip_serializing_if = "Option::is_none")]
    pub connectivity: Option<DeviceConnectivity>,
}

impl Default for MedicalDeviceInfo {
    fn default() -> Self {
        Self {
            device_id: format!("dev_{}", uuid::Uuid::new_v4().simple()),
            manufacturer: String::new(),
            model: String::new(),
            device_name: String::new(),
            fda_classification: None,
            device_type: MedicalDeviceType::Monitoring,
            device_category: DeviceCategory::Other,
            use_environment: UseEnvironment::Home,
            intended_users: None,
            interface: None,
            connectivity: None,
        }
    }
}

/// FDA device classification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FDAClassification {
    /// FDA class (I, II, or III)
    pub class: FDAClass,

    /// Product code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub product_code: Option<String>,

    /// Regulation number
    #[serde(skip_serializing_if = "Option::is_none")]
    pub regulation_number: Option<String>,

    /// Clearance type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clearance_type: Option<ClearanceType>,
}

/// FDA device class
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FDAClass {
    #[serde(rename = "I")]
    ClassI,
    #[serde(rename = "II")]
    ClassII,
    #[serde(rename = "III")]
    ClassIII,
}

/// FDA clearance type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClearanceType {
    #[serde(rename = "510k")]
    K510,
    PMA,
    DeNovo,
    Exempt,
}

/// Medical device type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MedicalDeviceType {
    Diagnostic,
    Monitoring,
    Therapeutic,
    Assistive,
    Wearable,
    Implantable,
    Imaging,
    Laboratory,
}

/// Device category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceCategory {
    BloodGlucoseMonitor,
    BloodPressureMonitor,
    PulseOximeter,
    Thermometer,
    WeightScale,
    EcgMonitor,
    Cgm,
    InsulinPump,
    CpapBipap,
    HearingAid,
    CochlearImplant,
    ExaminationTable,
    ImagingEquipment,
    InfusionPump,
    Ventilator,
    Defibrillator,
    Other,
}

/// Use environment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UseEnvironment {
    Home,
    Clinical,
    Both,
}

/// Intended user
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntendedUser {
    /// User type
    #[serde(rename = "type")]
    pub user_type: IntendedUserType,

    /// Training required
    pub training_required: bool,

    /// Supervision required
    pub supervision_required: bool,
}

/// Intended user type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntendedUserType {
    Patient,
    Caregiver,
    HealthcareProfessional,
}

// ============================================================================
// Device Interface
// ============================================================================

/// Device physical interface
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceInterface {
    /// Display configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<DisplayConfig>,

    /// Physical controls
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical_controls: Option<PhysicalControls>,

    /// Audio capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio: Option<AudioCapabilities>,

    /// Haptic capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic: Option<HapticCapabilities>,
}

/// Display configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayConfig {
    /// Display type
    #[serde(rename = "type")]
    pub display_type: DisplayType,

    /// Size in inches
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size_inches: Option<f32>,

    /// Resolution
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution: Option<Resolution>,

    /// Touch enabled
    pub touch_enabled: bool,

    /// Color display
    pub color: bool,

    /// Brightness adjustable
    pub brightness_adjustable: bool,

    /// Contrast adjustable
    pub contrast_adjustable: bool,
}

/// Display type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplayType {
    Lcd,
    Led,
    Oled,
    EInk,
    None,
}

/// Screen resolution
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Resolution {
    pub width: u32,
    pub height: u32,
}

/// Physical controls
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PhysicalControls {
    /// Buttons
    #[serde(skip_serializing_if = "Option::is_none")]
    pub buttons: Option<Vec<ButtonInfo>>,

    /// Dials
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dials: Option<Vec<DialInfo>>,

    /// Switches
    #[serde(skip_serializing_if = "Option::is_none")]
    pub switches: Option<Vec<SwitchInfo>>,
}

/// Button information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ButtonInfo {
    /// Button ID
    pub id: String,

    /// Button label
    pub label: String,

    /// Has tactile marking
    pub tactile_marking: bool,

    /// Size in mm
    pub size_mm: f32,

    /// Force required in grams
    pub force_required_grams: f32,

    /// Location description
    pub location: String,
}

/// Dial information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DialInfo {
    pub id: String,
    pub label: String,
    pub tactile_detents: bool,
}

/// Switch information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchInfo {
    pub id: String,
    pub label: String,
    pub tactile_marking: bool,
}

/// Audio capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioCapabilities {
    pub speaker: bool,
    pub speaker_volume_adjustable: bool,
    pub microphone: bool,
    pub audio_jack: bool,
    pub bluetooth_audio: bool,
}

/// Haptic capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticCapabilities {
    pub vibration: bool,
    pub intensity_levels: u8,
    pub patterns_supported: bool,
}

// ============================================================================
// Device Connectivity
// ============================================================================

/// Device connectivity options
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceConnectivity {
    /// Wireless connectivity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wireless: Option<WirelessConnectivity>,

    /// Wired connectivity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wired: Option<WiredConnectivity>,

    /// Companion app
    #[serde(skip_serializing_if = "Option::is_none")]
    pub companion_app: Option<CompanionApp>,

    /// Cloud integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cloud_integration: Option<CloudIntegration>,
}

/// Wireless connectivity
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WirelessConnectivity {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bluetooth: Option<BluetoothConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub wifi: Option<WifiConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub cellular: Option<CellularConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub nfc: Option<bool>,
}

/// Bluetooth configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BluetoothConfig {
    pub version: String,
    pub profiles: Vec<String>,
}

/// WiFi configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WifiConfig {
    pub standards: Vec<String>,
}

/// Cellular configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CellularConfig {
    pub technology: String,
}

/// Wired connectivity
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WiredConnectivity {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub usb: Option<UsbConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_jack: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub proprietary: Option<String>,
}

/// USB configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UsbConfig {
    #[serde(rename = "type")]
    pub usb_type: String,
    pub version: String,
}

/// Companion app
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompanionApp {
    pub platforms: Vec<Platform>,
    pub accessibility_features: Vec<String>,
}

/// Platform
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Platform {
    Ios,
    Android,
    Windows,
    Macos,
}

/// Cloud integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CloudIntegration {
    pub provider: String,
    pub data_sync: bool,
    pub remote_monitoring: bool,
}

// ============================================================================
// Accessibility Features
// ============================================================================

/// Device accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeviceAccessibilityFeatures {
    /// Visual accessibility
    pub visual: VisualAccessibility,

    /// Auditory accessibility
    pub auditory: AuditoryAccessibility,

    /// Motor accessibility
    pub motor: MotorAccessibility,

    /// Cognitive accessibility
    pub cognitive: CognitiveAccessibility,

    /// Physical accessibility (MDE Standards)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical: Option<PhysicalAccessibility>,

    /// Accessibility score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_score: Option<AccessibilityScore>,
}

/// Visual accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VisualAccessibility {
    /// Screen reader support
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_reader: Option<ScreenReaderSupport>,

    /// Voice output
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_output: Option<VoiceOutput>,

    /// Display accessibility
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display_accessibility: Option<DisplayAccessibility>,

    /// Non-visual alternatives
    #[serde(skip_serializing_if = "Option::is_none")]
    pub non_visual_alternatives: Option<NonVisualAlternatives>,
}

/// Screen reader support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenReaderSupport {
    pub supported: bool,
    pub level: SupportLevel,
    pub protocols: Vec<ScreenReaderProtocol>,
}

/// Support level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SupportLevel {
    None,
    Partial,
    Full,
}

/// Screen reader protocols
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScreenReaderProtocol {
    Talkback,
    Voiceover,
    Nvda,
    Jaws,
    Custom,
}

/// Voice output configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceOutput {
    pub supported: bool,
    pub readings: Vec<VoiceReading>,
    pub languages: Vec<String>,
    pub speed_adjustable: bool,
    pub volume_adjustable: bool,
}

/// Voice reading configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceReading {
    pub data_type: String,
    pub format: String,
    pub units_spoken: bool,
    pub range_indication: bool,
}

/// Display accessibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayAccessibility {
    pub high_contrast_mode: bool,
    pub large_text_mode: bool,
    pub text_size_adjustable: bool,
    pub color_inversion: bool,
    pub color_blind_modes: Vec<ColorBlindMode>,
}

/// Color blind mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorBlindMode {
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
}

/// Non-visual alternatives
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NonVisualAlternatives {
    pub audio_feedback: bool,
    pub haptic_feedback: bool,
    pub braille_output: bool,
}

/// Auditory accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AuditoryAccessibility {
    /// Visual alerts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual_alerts: Option<VisualAlerts>,

    /// Haptic alerts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic_alerts: Option<HapticAlerts>,

    /// Audio adjustments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_adjustments: Option<AudioAdjustments>,

    /// Text alternatives
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text_alternatives: Option<TextAlternatives>,
}

/// Visual alerts
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualAlerts {
    pub supported: bool,
    pub types: Vec<VisualAlertType>,
    pub customizable: bool,
}

/// Visual alert type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualAlertType {
    ScreenFlash,
    LedIndicator,
    IconDisplay,
    TextNotification,
    ColorChange,
}

/// Haptic alerts
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticAlerts {
    pub supported: bool,
    pub patterns: Vec<HapticPattern>,
    pub intensity_levels: u8,
}

/// Haptic pattern
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPattern {
    pub id: String,
    pub name: String,
    pub meaning: String,
    pub duration_ms: u32,
    pub intensity: f32,
}

/// Audio adjustments
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioAdjustments {
    pub volume_range: (f32, f32),
    pub frequency_adjustable: bool,
    pub mono_audio: bool,
    pub hearing_aid_compatible: bool,
    pub t_coil_compatible: bool,
}

/// Text alternatives
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextAlternatives {
    pub on_screen_text: bool,
    pub closed_captions: bool,
    pub real_time_transcription: bool,
}

/// Motor accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorAccessibility {
    /// Alternative input methods
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alternative_input: Option<AlternativeInput>,

    /// Physical controls accessibility
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical_controls: Option<PhysicalControlsAccessibility>,

    /// Touchscreen accessibility
    #[serde(skip_serializing_if = "Option::is_none")]
    pub touchscreen: Option<TouchscreenAccessibility>,

    /// Automation features
    #[serde(skip_serializing_if = "Option::is_none")]
    pub automation: Option<AutomationFeatures>,
}

/// Alternative input methods
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlternativeInput {
    pub voice_control: bool,
    pub switch_access: bool,
    pub eye_tracking: bool,
    pub head_tracking: bool,
    pub wia_exoskeleton: bool,
}

/// Physical controls accessibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalControlsAccessibility {
    pub large_buttons: bool,
    pub button_spacing_adequate: bool,
    pub low_force_buttons: bool,
    pub one_handed_operation: bool,
    pub no_fine_motor_required: bool,
}

/// Touchscreen accessibility
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchscreenAccessibility {
    pub gesture_alternatives: bool,
    pub touch_accommodation: bool,
    pub dwell_control: bool,
    pub haptic_feedback: bool,
}

/// Automation features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationFeatures {
    pub auto_measurement: bool,
    pub scheduled_operation: bool,
    pub remote_control: bool,
}

/// Cognitive accessibility features
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CognitiveAccessibility {
    /// Simplified interface
    #[serde(skip_serializing_if = "Option::is_none")]
    pub simplified_interface: Option<SimplifiedInterface>,

    /// Memory support
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_support: Option<MemorySupport>,

    /// Error prevention
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_prevention: Option<ErrorPrevention>,

    /// Language support
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language_support: Option<LanguageSupport>,
}

/// Simplified interface
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimplifiedInterface {
    pub available: bool,
    pub reduced_options: bool,
    pub step_by_step_guidance: bool,
    pub clear_icons: bool,
}

/// Memory support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemorySupport {
    pub reminders: bool,
    pub history_log: bool,
    pub caregiver_notifications: bool,
    pub auto_data_sync: bool,
}

/// Error prevention
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPrevention {
    pub confirmation_prompts: bool,
    pub undo_capability: bool,
    pub clear_error_messages: bool,
    pub recovery_guidance: bool,
}

/// Language support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageSupport {
    pub languages: Vec<String>,
    pub simple_language_mode: bool,
    pub icon_based_navigation: bool,
    pub pictogram_support: bool,
}

/// Physical accessibility (MDE Standards compliance)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalAccessibility {
    /// Transfer height (MDE Standards)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer_height: Option<TransferHeight>,

    /// Wheelchair access
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wheelchair_access: Option<WheelchairAccess>,

    /// Physical dimensions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<PhysicalDimensions>,

    /// Support features
    #[serde(skip_serializing_if = "Option::is_none")]
    pub supports: Option<SupportFeatures>,
}

/// Transfer height (MDE Standards)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransferHeight {
    pub adjustable: bool,
    pub minimum_height_inches: f32,
    pub maximum_height_inches: f32,
    /// MDE compliant (17 inches)
    pub mde_compliant: bool,
}

/// Wheelchair access
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairAccess {
    pub accessible: bool,
    pub clear_floor_space: bool,
    pub approach_type: ApproachType,
    pub knee_clearance: bool,
}

/// Approach type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ApproachType {
    Front,
    Side,
    Both,
}

/// Physical dimensions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalDimensions {
    pub weight_kg: f32,
    pub portable: bool,
    pub height_adjustable: bool,
    pub tilt_adjustable: bool,
}

/// Support features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupportFeatures {
    pub armrests: bool,
    pub grab_bars: bool,
    pub head_support: bool,
    pub leg_support: bool,
}

/// Accessibility score
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityScore {
    pub overall: f32,
    pub visual: f32,
    pub auditory: f32,
    pub motor: f32,
    pub cognitive: f32,
}

// ============================================================================
// Regulatory Compliance
// ============================================================================

/// Regulatory compliance information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RegulatoryCompliance {
    /// FDA compliance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fda: Option<FDACompliance>,

    /// MDE Standards compliance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mde_standards: Option<MDECompliance>,

    /// ADA compliance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ada: Option<ADACompliance>,

    /// International standards compliance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub international: Option<InternationalCompliance>,

    /// WIA certification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_certification: Option<WIACertification>,

    /// Accessibility conformance
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_conformance: Option<AccessibilityConformance>,
}

/// FDA compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FDACompliance {
    pub registered: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clearance_number: Option<String>,
    pub human_factors_validated: bool,
    pub accessibility_tested: bool,
}

/// MDE Standards compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MDECompliance {
    pub compliant: bool,
    pub transfer_height_compliant: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_assessment_date: Option<String>,
}

/// ADA compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ADACompliance {
    pub title_ii_compliant: bool,
    pub title_iii_compliant: bool,
}

/// International standards compliance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InternationalCompliance {
    /// IEC 62366 (Usability engineering)
    pub iec_62366: bool,
    /// ISO 14971 (Risk management)
    pub iso_14971: bool,
    /// EN 301 549 (EU ICT accessibility)
    pub en_301_549: bool,
    /// EU MDR compliant
    pub mdr_compliant: bool,
}

/// WIA certification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WIACertification {
    pub level: CertificationLevel,
    pub certificate_id: String,
    pub valid_until: String,
}

/// Certification level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CertificationLevel {
    Bronze,
    Silver,
    Gold,
    Platinum,
}

/// Accessibility conformance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityConformance {
    pub vpat_available: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vpat_url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wcag_level: Option<WCAGLevel>,
    pub section_508_compliant: bool,
}

/// WCAG level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WCAGLevel {
    A,
    AA,
    AAA,
}

// ============================================================================
// WIA Integration
// ============================================================================

/// WIA ecosystem integration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WIAIntegration {
    /// Supported protocols
    pub supported_protocols: WIASupportedProtocols,

    /// Exoskeleton integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exoskeleton: Option<ExoskeletonIntegration>,

    /// Bionic eye integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bionic_eye: Option<BionicEyeIntegration>,

    /// Voice-Sign integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_sign: Option<VoiceSignIntegration>,

    /// Haptic integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic: Option<HapticIntegration>,

    /// Smart wheelchair integration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smart_wheelchair: Option<SmartWheelchairIntegration>,
}

/// WIA supported protocols
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WIASupportedProtocols {
    pub exoskeleton: bool,
    pub bionic_eye: bool,
    pub voice_sign: bool,
    pub haptic: bool,
    pub smart_wheelchair: bool,
}

/// Exoskeleton integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonIntegration {
    pub haptic_feedback_mapping: Vec<HapticMapping>,
    pub motion_assistance: bool,
    pub rehabilitation_mode: bool,
}

/// Haptic mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticMapping {
    pub device_event: String,
    pub haptic_pattern: String,
    pub target: HapticTarget,
    pub intensity: f32,
}

/// Haptic target
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HapticTarget {
    LeftHand,
    RightHand,
    Torso,
    Custom,
}

/// Bionic eye integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BionicEyeIntegration {
    pub display_optimization: bool,
    pub contrast_enhancement: bool,
    pub pattern_simplification: bool,
}

/// Voice-Sign integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSignIntegration {
    pub medical_terminology_support: bool,
    pub real_time_translation: bool,
    pub emergency_phrases: Vec<String>,
}

/// Haptic integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticIntegration {
    pub alarm_mapping: Vec<HapticAlarmMapping>,
    pub data_haptic_encoding: bool,
}

/// Haptic alarm mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticAlarmMapping {
    pub alarm_priority: String,
    pub haptic_pattern: String,
    pub body_location: String,
}

/// Smart wheelchair integration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SmartWheelchairIntegration {
    pub device_positioning: bool,
    pub height_adjustment_sync: bool,
}

// ============================================================================
// Profile Metadata
// ============================================================================

/// Profile metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileMetadata {
    pub profile_type: ProfileType,
    pub schema_version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_by: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validation: Option<ValidationInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

impl Default for ProfileMetadata {
    fn default() -> Self {
        Self {
            profile_type: ProfileType::Device,
            schema_version: "1.0.0".to_string(),
            language: Some("en".to_string()),
            created_by: None,
            validation: None,
            tags: None,
            notes: None,
        }
    }
}

/// Profile type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfileType {
    Device,
    User,
    Combination,
}

/// Validation info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationInfo {
    pub validated: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validator: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validation_date: Option<String>,
}

// ============================================================================
// User Medical Accessibility Profile
// ============================================================================

/// User medical accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserMedicalAccessibilityProfile {
    /// Anonymized user identifier
    pub user_id: String,

    /// Profile version
    pub profile_version: String,

    /// Accessibility needs
    pub accessibility_needs: UserAccessibilityNeeds,

    /// Sensory preferences
    pub sensory_preferences: UserSensoryPreferences,

    /// Input preferences
    pub input_preferences: UserInputPreferences,

    /// Cognitive support
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive_support: Option<UserCognitiveSupport>,

    /// Medical context
    #[serde(skip_serializing_if = "Option::is_none")]
    pub medical_context: Option<MedicalContext>,

    /// WIA device settings
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_devices: Option<WIADeviceSettings>,
}

impl Default for UserMedicalAccessibilityProfile {
    fn default() -> Self {
        Self {
            user_id: format!("user_{}", uuid::Uuid::new_v4().simple()),
            profile_version: "1.0.0".to_string(),
            accessibility_needs: UserAccessibilityNeeds::default(),
            sensory_preferences: UserSensoryPreferences::default(),
            input_preferences: UserInputPreferences::default(),
            cognitive_support: None,
            medical_context: None,
            wia_devices: None,
        }
    }
}

/// User accessibility needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UserAccessibilityNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensory: Option<SensoryNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive: Option<CognitiveNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub medical_conditions: Option<Vec<MedicalCondition>>,
}

/// Sensory needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SensoryNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual: Option<VisualNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory: Option<AuditoryNeeds>,
}

/// Visual needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualNeeds {
    pub level: VisualLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_blind: Option<ColorBlindMode>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub light_sensitivity: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub field_of_vision: Option<FieldOfVisionType>,
}

/// Visual level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualLevel {
    None,
    LowVision,
    LegallyBlind,
    TotallyBlind,
}

/// Field of vision type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FieldOfVisionType {
    Full,
    Tunnel,
    PeripheralLoss,
    CentralLoss,
}

/// Auditory needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryNeeds {
    pub level: AuditoryLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_hearing_aid: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_cochlear_implant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency_range_affected: Option<FrequencyRange>,
}

/// Auditory level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditoryLevel {
    None,
    HardOfHearing,
    Deaf,
}

/// Frequency range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrequencyRange {
    None,
    Low,
    Mid,
    High,
    All,
}

/// Motor needs
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upper_limb: Option<UpperLimbNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub fine_motor: Option<FineMotorNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mobility: Option<MobilityNeeds>,
}

/// Upper limb needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpperLimbNeeds {
    pub level: ImpairmentLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub affected_side: Option<AffectedSide>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tremor: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grip_strength: Option<GripStrength>,
}

/// Impairment level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImpairmentLevel {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Affected side
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AffectedSide {
    None,
    Left,
    Right,
    Both,
}

/// Grip strength
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripStrength {
    Normal,
    Reduced,
    Minimal,
    None,
}

/// Fine motor needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FineMotorNeeds {
    pub level: ImpairmentLevel,
}

/// Mobility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilityNeeds {
    pub level: MobilityLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_assistive_device: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistive_device_type: Option<String>,
}

/// Mobility level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MobilityLevel {
    Ambulatory,
    Assisted,
    Wheelchair,
    BedBound,
}

/// Cognitive needs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory: Option<MemoryLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attention: Option<AttentionLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub processing_speed: Option<ProcessingSpeed>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reading_level: Option<ReadingLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub primary_language: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub additional_languages: Option<Vec<String>>,
}

/// Memory level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MemoryLevel {
    Normal,
    MildImpairment,
    ModerateImpairment,
    SevereImpairment,
}

/// Attention level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttentionLevel {
    Normal,
    Limited,
    VeryLimited,
}

/// Processing speed
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcessingSpeed {
    Normal,
    Slower,
    VerySlow,
}

/// Reading level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReadingLevel {
    Basic,
    Intermediate,
    Advanced,
}

/// Medical condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicalCondition {
    pub condition: String,
    pub relevance_to_device_use: String,
}

/// User sensory preferences
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct UserSensoryPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual: Option<VisualPreferences>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory: Option<AuditoryPreferences>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic: Option<HapticPreferences>,
}

/// Visual preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualPreferences {
    pub text_size: TextSize,
    pub high_contrast: bool,
    pub dark_mode: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_scheme: Option<String>,
    pub reduce_motion: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reduce_transparency: Option<bool>,
}

/// Text size
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TextSize {
    Small,
    Medium,
    Large,
    ExtraLarge,
}

/// Auditory preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryPreferences {
    pub volume_level: u8,
    pub prefer_voice: bool,
    pub voice_speed: f32,
    pub voice_pitch: VoicePitch,
    pub prefer_tones: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tone_frequency_preference: Option<f32>,
}

/// Voice pitch
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VoicePitch {
    Low,
    Medium,
    High,
}

/// Haptic preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPreferences {
    pub enabled: bool,
    pub intensity: u8,
    pub prefer_haptic_over_audio: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_haptic_device: Option<String>,
}

/// User input preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserInputPreferences {
    pub primary_input: InputMethod,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fallback_inputs: Option<Vec<InputMethod>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub touch_settings: Option<TouchSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_settings: Option<VoiceSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub switch_settings: Option<SwitchSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dwell_settings: Option<DwellSettings>,
}

impl Default for UserInputPreferences {
    fn default() -> Self {
        Self {
            primary_input: InputMethod::Touch,
            fallback_inputs: None,
            touch_settings: None,
            voice_settings: None,
            switch_settings: None,
            dwell_settings: None,
        }
    }
}

/// Input method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputMethod {
    Touch,
    Buttons,
    Voice,
    Switch,
    EyeTracking,
    HeadTracking,
    WiaExoskeleton,
}

/// Touch settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TouchSettings {
    pub touch_duration_ms: u32,
    pub ignore_repeated_touches: bool,
    pub touch_accommodation: bool,
}

/// Voice settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSettings {
    pub voice_language: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wake_word: Option<String>,
    pub confirmation_required: bool,
}

/// Switch settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchSettings {
    pub switch_type: String,
    pub scanning_speed: ScanningSpeed,
    pub auto_scanning: bool,
}

/// Scanning speed
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScanningSpeed {
    Slow,
    Medium,
    Fast,
}

/// Dwell settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellSettings {
    pub enabled: bool,
    pub dwell_time_ms: u32,
}

/// User cognitive support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserCognitiveSupport {
    pub simplified_mode: bool,
    pub step_by_step_guidance: bool,
    pub reminder_frequency: ReminderFrequency,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub caregiver_mode: Option<CaregiverMode>,
    pub confirmation_level: ConfirmationLevel,
}

/// Reminder frequency
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReminderFrequency {
    None,
    Low,
    Medium,
    High,
}

/// Caregiver mode
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CaregiverMode {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub caregiver_contact: Option<String>,
    pub notification_types: Vec<NotificationType>,
}

/// Notification type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationType {
    MissedReading,
    AbnormalValue,
    DeviceError,
    LowBattery,
}

/// Confirmation level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfirmationLevel {
    None,
    ImportantOnly,
    AllActions,
}

/// Medical context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicalContext {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions_monitored: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub devices_used: Option<Vec<DeviceUsage>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub healthcare_provider: Option<HealthcareProviderInfo>,
}

/// Device usage
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceUsage {
    pub device_id: String,
    pub device_type: String,
    pub usage_frequency: UsageFrequency,
}

/// Usage frequency
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UsageFrequency {
    Continuous,
    Daily,
    Weekly,
    AsNeeded,
}

/// Healthcare provider info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthcareProviderInfo {
    pub data_sharing_enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider_id: Option<String>,
}

/// WIA device settings
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WIADeviceSettings {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exoskeleton: Option<ExoskeletonSettings>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub bionic_eye: Option<BionicEyeSettings>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_sign: Option<VoiceSignSettings>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub smart_wheelchair: Option<SmartWheelchairSettings>,
}

/// Exoskeleton settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonSettings {
    pub device_id: String,
    pub enabled: bool,
    pub haptic_intensity: u8,
    pub assistance_level: u8,
}

/// Bionic eye settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BionicEyeSettings {
    pub device_id: String,
    pub enabled: bool,
    pub brightness: u8,
    pub contrast: u8,
}

/// Voice-Sign settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSignSettings {
    pub enabled: bool,
    pub sign_language: SignLanguage,
    pub medical_terms_mode: bool,
}

/// Sign language
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum SignLanguage {
    #[serde(rename = "ASL")]
    Asl,
    #[serde(rename = "BSL")]
    Bsl,
    #[serde(rename = "KSL")]
    Ksl,
    #[serde(rename = "JSL")]
    Jsl,
    #[serde(rename = "DGS")]
    Dgs,
    #[serde(rename = "LSF")]
    Lsf,
    Auslan,
    Other,
}

/// Smart wheelchair settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SmartWheelchairSettings {
    pub device_id: String,
    pub enabled: bool,
    pub auto_positioning: bool,
}

// ============================================================================
// Alarm System Types
// ============================================================================

/// Medical alarm system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MedicalAlarmSystem {
    pub alarm_system_id: String,
    pub alarm_system_version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_id: Option<String>,
    pub alarm_categories: Vec<AlarmCategory>,
    pub output_modalities: OutputModalities,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_settings: Option<AlarmAccessibilitySettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub escalation: Option<EscalationConfig>,
}

/// Alarm category
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlarmCategory {
    pub id: String,
    pub name: String,
    pub priority: AlarmPriority,
    pub meaning: String,
    pub recommended_action: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub medical_urgency: Option<MedicalUrgency>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual_config: Option<VisualAlarmConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory_config: Option<AuditoryAlarmConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic_config: Option<HapticAlarmConfig>,
}

/// Alarm priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlarmPriority {
    Low,
    Medium,
    High,
    Critical,
}

/// Medical urgency
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MedicalUrgency {
    Informational,
    AttentionRequired,
    Urgent,
    Emergency,
}

/// Visual alarm config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualAlarmConfig {
    pub enabled: bool,
    pub color: String,
    pub pattern: VisualPattern,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flash_rate_hz: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icon: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

/// Visual pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VisualPattern {
    Solid,
    Flashing,
    Pulsing,
    Scrolling,
}

/// Auditory alarm config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditoryAlarmConfig {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tone_frequency_hz: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pattern: Option<AuditoryPattern>,
    pub volume_db: f32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_announcement: Option<VoiceAnnouncement>,
}

/// Auditory pattern
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuditoryPattern {
    Continuous,
    Intermittent,
    Escalating,
    Pulsing,
}

/// Voice announcement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceAnnouncement {
    pub enabled: bool,
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text_simple: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
}

/// Haptic alarm config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticAlarmConfig {
    pub enabled: bool,
    pub pattern_id: String,
    pub intensity: f32,
    pub duration_ms: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_haptic_device: Option<WIAHapticDeviceConfig>,
}

/// WIA haptic device config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WIAHapticDeviceConfig {
    pub enabled: bool,
    pub target_body_location: String,
}

/// Output modalities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputModalities {
    pub visual: ModalitySupport,
    pub auditory: ModalitySupport,
    pub haptic: ModalitySupport,
}

/// Modality support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalitySupport {
    pub supported: bool,
    pub required_for_critical: bool,
    pub default_enabled: bool,
}

/// Alarm accessibility settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlarmAccessibilitySettings {
    pub allow_single_modality: bool,
    pub minimum_modalities_critical: u8,
    pub minimum_modalities_other: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive_support: Option<AlarmCognitiveSupport>,
}

/// Alarm cognitive support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlarmCognitiveSupport {
    pub simplified_messages: bool,
    pub step_by_step_guidance: bool,
    pub confirmation_required: bool,
    pub caregiver_notification: bool,
}

/// Escalation config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EscalationConfig {
    pub enabled: bool,
    pub timeout_seconds: u32,
    pub steps: Vec<EscalationStep>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub emergency_contact: Option<EmergencyContact>,
}

/// Escalation step
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EscalationStep {
    pub delay_seconds: u32,
    pub action: EscalationAction,
}

/// Escalation action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EscalationAction {
    Repeat,
    IncreaseVolume,
    IncreaseIntensity,
    AddModality,
    NotifyCaregiver,
    NotifyProvider,
    EmergencyCall,
}

/// Emergency contact
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    pub enabled: bool,
    pub contacts: Vec<ContactInfo>,
}

/// Contact info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactInfo {
    pub name: String,
    pub phone: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub relationship: Option<String>,
    pub notification_methods: Vec<ContactMethod>,
}

/// Contact method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactMethod {
    Sms,
    Call,
    AppNotification,
    Email,
}
