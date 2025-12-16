//! Type definitions for WIA Autonomous Vehicle Accessibility Standard
//!
//! These types correspond to the JSON schemas defined in Phase 1.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use validator::Validate;

// ============================================================================
// Enums
// ============================================================================

/// Types of disabilities supported by the system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisabilityType {
    VisualBlind,
    VisualLowVision,
    HearingDeaf,
    HearingHard,
    MobilityWheelchairManual,
    MobilityWheelchairPower,
    MobilityWalker,
    MobilityCane,
    MobilityCrutches,
    CognitiveIntellectual,
    CognitiveAutism,
    CognitiveDementia,
    Speech,
    Multiple,
}

/// SAE J3016 automation levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum SAELevel {
    #[serde(rename = "0")]
    Level0 = 0, // No Automation
    #[serde(rename = "1")]
    Level1 = 1, // Driver Assistance
    #[serde(rename = "2")]
    Level2 = 2, // Partial Automation
    #[serde(rename = "3")]
    Level3 = 3, // Conditional Automation
    #[serde(rename = "4")]
    Level4 = 4, // High Automation
    #[serde(rename = "5")]
    Level5 = 5, // Full Automation
}

/// Interaction modalities for HMI
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InteractionModality {
    VisualScreen,
    VisualLed,
    AudioTts,
    AudioChime,
    AudioSpeechRec,
    HapticVibration,
    HapticForce,
    Braille,
    PhysicalButton,
}

/// Mobility aid types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MobilityAidType {
    None,
    ManualWheelchair,
    PowerWheelchair,
    Scooter,
    Walker,
    Cane,
    Crutches,
}

/// Service animal sizes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AnimalSize {
    Small,
    Medium,
    Large,
}

/// Door types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DoorType {
    Swing,
    Slide,
    Gullwing,
}

/// Ramp types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RampType {
    Fold,
    Slide,
    Deploy,
}

/// Lift types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LiftType {
    Platform,
    Rotary,
}

/// Entry types for vehicle boarding
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EntryType {
    Ramp,
    Lift,
    LevelBoarding,
}

/// Securement types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SecurementType {
    Manual,
    SemiAuto,
    FullAuto,
}

/// Securement point status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SecurementPointStatus {
    Disengaged,
    Engaging,
    Engaged,
    Error,
}

/// Securement point position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SecurementPosition {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

/// Overall securement status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OverallSecurementStatus {
    Unsecured,
    Securing,
    Secured,
    Error,
}

/// Wheelchair type for securement
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WheelchairType {
    Manual,
    Power,
    Scooter,
}

/// Emergency event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmergencyEventType {
    PanicButton,
    PullOver,
    Collision,
    Medical,
    Fire,
    Other,
}

/// Emergency severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmergencySeverity {
    Low,
    Medium,
    High,
    Critical,
}

/// Trip request status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TripStatus {
    Confirmed,
    NoVehicle,
    Pending,
}

/// Pickup side preference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PickupSide {
    SameSide,
    Any,
}

/// Visual contrast options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContrastMode {
    Normal,
    High,
}

/// Text size options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TextSize {
    Small,
    Medium,
    Large,
    ExtraLarge,
}

/// Color scheme options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorScheme {
    Default,
    HighContrast,
    Dark,
    Light,
}

/// TTS speed options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TtsSpeed {
    Slow,
    Normal,
    Fast,
}

/// Find vehicle features
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FindVehicleFeature {
    Horn,
    Lights,
    Melody,
}

/// Message source/destination
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageEndpoint {
    PassengerApp,
    Vehicle,
    FleetMgmt,
    Support,
}

/// Message types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Profile,
    Capabilities,
    TripRequest,
    TripResponse,
    HmiConfig,
    Securement,
    Emergency,
}

/// Message priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessagePriority {
    Low,
    Normal,
    High,
    Critical,
}

// ============================================================================
// Structs - Common
// ============================================================================

/// Geographic location
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct GeoLocation {
    #[validate(range(min = -90.0, max = 90.0))]
    pub latitude: f64,

    #[validate(range(min = -180.0, max = 180.0))]
    pub longitude: f64,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub address: Option<String>,
}

/// Physical dimensions
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct Dimensions {
    #[validate(range(min = 0.0))]
    pub width_cm: f64,

    #[validate(range(min = 0.0))]
    pub length_cm: f64,

    #[validate(range(min = 0.0))]
    pub height_cm: f64,

    #[validate(range(min = 0.0))]
    pub weight_kg: f64,
}

/// Emergency contact information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyContact {
    pub name: String,
    pub phone: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub relationship: Option<String>,
}

// ============================================================================
// Structs - Passenger Profile
// ============================================================================

/// Service animal information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceAnimal {
    pub has_animal: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub animal_type: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub animal_size: Option<AnimalSize>,
}

/// Mobility aid information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilityAid {
    #[serde(rename = "type")]
    pub aid_type: MobilityAidType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<Dimensions>,

    #[serde(default)]
    pub requires_ramp: bool,

    #[serde(default)]
    pub requires_lift: bool,

    #[serde(default)]
    pub requires_securement: bool,
}

/// Communication preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationPrefs {
    /// ISO 639-1 language code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_language: Option<String>,

    #[serde(default)]
    pub preferred_modalities: Vec<InteractionModality>,

    #[serde(default)]
    pub aac_device: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language: Option<String>,
}

/// Assistance requirements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssistanceNeeds {
    #[serde(default)]
    pub needs_boarding_help: bool,

    #[serde(default)]
    pub needs_securement_help: bool,

    #[serde(default)]
    pub companion_present: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub emergency_contact: Option<EmergencyContact>,
}

/// User preferences
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct UserPreferences {
    #[validate(range(min = 0, max = 100))]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_volume: Option<u8>,

    #[validate(range(min = 0, max = 100))]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_brightness: Option<u8>,

    #[serde(default)]
    pub high_contrast: bool,

    #[serde(default)]
    pub large_text: bool,

    #[validate(range(min = 0, max = 100))]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic_intensity: Option<u8>,

    #[serde(default)]
    pub minimize_walking: bool,
}

/// Passenger information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassengerInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    pub disabilities: Vec<DisabilityType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub service_animal: Option<ServiceAnimal>,
}

/// Complete passenger accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct PassengerProfile {
    pub profile_id: Uuid,
    pub version: String,
    pub passenger: PassengerInfo,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub mobility_aid: Option<MobilityAid>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub communication: Option<CommunicationPrefs>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistance: Option<AssistanceNeeds>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate]
    pub preferences: Option<UserPreferences>,
}

// ============================================================================
// Structs - Vehicle Capabilities
// ============================================================================

/// Vehicle basic information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleInfo {
    pub make: String,
    pub model: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub year: Option<u16>,

    pub sae_level: SAELevel,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub license_plate: Option<String>,
}

/// Ramp features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RampFeatures {
    pub available: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub ramp_type: Option<RampType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_weight_kg: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub width_cm: Option<f64>,

    #[serde(default)]
    pub auto_deploy: bool,
}

/// Lift features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LiftFeatures {
    pub available: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub lift_type: Option<LiftType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_weight_kg: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub width_cm: Option<f64>,
}

/// Door features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoorFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub door_type: Option<DoorType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub width_cm: Option<f64>,

    #[serde(default)]
    pub auto_open: bool,

    #[serde(default)]
    pub low_step: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_height_cm: Option<f64>,
}

/// Entry features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntryFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ramp: Option<RampFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub lift: Option<LiftFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub doors: Option<DoorFeatures>,
}

/// Wheelchair securement features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurementFeatures {
    pub available: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(rename = "type")]
    pub securement_type: Option<SecurementType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_width_cm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_length_cm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_weight_kg: Option<f64>,
}

/// Interior features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InteriorFeatures {
    #[serde(default)]
    pub wheelchair_spaces: u8,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub wheelchair_securement: Option<SecurementFeatures>,

    #[serde(default)]
    pub transfer_seat: bool,

    #[serde(default)]
    pub lowered_floor: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub floor_height_cm: Option<f64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub headroom_cm: Option<f64>,
}

/// Screen features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenFeatures {
    pub available: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub size_inches: Option<f64>,

    #[serde(default)]
    pub touch: bool,

    #[serde(default)]
    pub high_contrast: bool,

    #[serde(default)]
    pub adjustable_brightness: bool,
}

/// Audio features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioFeatures {
    #[serde(default)]
    pub tts: bool,

    #[serde(default)]
    pub speech_recognition: bool,

    #[serde(default)]
    pub languages: Vec<String>,

    #[serde(default)]
    pub volume_control: bool,

    #[serde(default)]
    pub chimes: bool,
}

/// Haptic features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticFeatures {
    #[serde(default)]
    pub vibration: bool,

    #[serde(default)]
    pub force_feedback: bool,
}

/// Physical control features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalControlFeatures {
    #[serde(default)]
    pub braille_labels: bool,

    #[serde(default)]
    pub tactile_buttons: bool,

    #[serde(default)]
    pub panic_button: bool,

    #[serde(default)]
    pub pull_over_button: bool,
}

/// HMI features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HmiFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen: Option<ScreenFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio: Option<AudioFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic: Option<HapticFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical_controls: Option<PhysicalControlFeatures>,
}

/// Communication features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationFeatures {
    #[serde(default)]
    pub live_support: bool,

    #[serde(default)]
    pub video_call: bool,

    #[serde(default)]
    pub text_chat: bool,

    #[serde(default)]
    pub sign_language_support: bool,
}

/// All accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entry: Option<EntryFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub interior: Option<InteriorFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub hmi: Option<HmiFeatures>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub communication: Option<CommunicationFeatures>,
}

/// Vehicle capacity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleCapacity {
    pub total_passengers: u8,

    #[serde(default)]
    pub wheelchair_passengers: u8,

    #[serde(default)]
    pub service_animals: bool,
}

/// Complete vehicle capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleCapabilities {
    pub vehicle_id: Uuid,
    pub version: String,
    pub vehicle_info: VehicleInfo,
    pub accessibility_features: AccessibilityFeatures,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub capacity: Option<VehicleCapacity>,
}

// ============================================================================
// Structs - Trip Request/Response
// ============================================================================

/// Location with optional pickup details
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TripLocation {
    #[validate]
    pub location: GeoLocation,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub pickup_side: Option<PickupSide>,

    #[serde(default)]
    pub curb_to_curb: bool,
}

/// Trip details
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TripDetails {
    #[validate]
    pub pickup: TripLocation,

    #[validate]
    pub dropoff: TripLocation,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub scheduled_time: Option<DateTime<Utc>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub flexibility_minutes: Option<u32>,
}

/// Accessibility requirements for a trip
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityRequirements {
    #[serde(default)]
    pub wheelchair_accessible: bool,

    #[serde(default)]
    pub ramp_required: bool,

    #[serde(default)]
    pub lift_required: bool,

    #[serde(default)]
    pub service_animal_space: bool,

    #[serde(default)]
    pub companion_space: u8,

    #[serde(default)]
    pub preferred_modalities: Vec<InteractionModality>,
}

/// Trip preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TripPreferences {
    #[serde(default)]
    pub minimize_walking: bool,

    #[serde(default)]
    pub audio_guidance: bool,

    #[serde(default)]
    pub visual_guidance: bool,

    #[serde(default)]
    pub haptic_feedback: bool,

    #[serde(default)]
    pub quiet_ride: bool,
}

/// Trip request
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct TripRequest {
    pub request_id: Uuid,
    pub version: String,
    pub timestamp: DateTime<Utc>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub passenger_profile_id: Option<Uuid>,

    #[validate]
    pub trip: TripDetails,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_requirements: Option<AccessibilityRequirements>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferences: Option<TripPreferences>,
}

/// Vehicle assignment in response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleAssignment {
    pub vehicle_id: Uuid,
    pub eta_minutes: f64,
    pub distance_km: f64,
}

/// Accessibility match result
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct AccessibilityMatch {
    pub wheelchair_accessible: bool,
    pub ramp_available: bool,
    pub lift_available: bool,
    pub service_animal_ok: bool,

    #[serde(default)]
    pub modalities_supported: Vec<InteractionModality>,

    #[validate(range(min = 0, max = 100))]
    pub match_score: u8,
}

/// Wayfinding information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WayfindingInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pickup_instructions: Option<String>,

    #[serde(default)]
    pub audio_guidance_available: bool,

    #[serde(default)]
    pub find_vehicle_features: Vec<FindVehicleFeature>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub braille_identifier: Option<String>,
}

/// Trip response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TripResponse {
    pub response_id: Uuid,
    pub request_id: Uuid,
    pub version: String,
    pub timestamp: DateTime<Utc>,
    pub status: TripStatus,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle: Option<VehicleAssignment>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility_match: Option<AccessibilityMatch>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub wayfinding: Option<WayfindingInfo>,
}

// ============================================================================
// Structs - HMI Configuration
// ============================================================================

/// Visual settings
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct VisualConfig {
    #[serde(default = "default_true")]
    pub enabled: bool,

    #[validate(range(min = 0, max = 100))]
    #[serde(default = "default_brightness")]
    pub brightness: u8,

    #[serde(default)]
    pub contrast: ContrastMode,

    #[serde(default)]
    pub text_size: TextSize,

    #[serde(default)]
    pub color_scheme: ColorScheme,

    #[serde(default = "default_true")]
    pub animations: bool,
}

/// Audio settings
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct AudioConfig {
    #[serde(default = "default_true")]
    pub enabled: bool,

    #[validate(range(min = 0, max = 100))]
    #[serde(default = "default_volume")]
    pub volume: u8,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub tts_voice: Option<String>,

    #[serde(default)]
    pub tts_speed: TtsSpeed,

    #[serde(default = "default_true")]
    pub chimes_enabled: bool,

    #[serde(default = "default_true")]
    pub speech_recognition: bool,

    #[serde(default = "default_language")]
    pub language: String,
}

/// Haptic pattern settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPatterns {
    #[serde(default = "default_true")]
    pub notification: bool,

    #[serde(default = "default_true")]
    pub navigation: bool,

    #[serde(default = "default_true")]
    pub warning: bool,

    #[serde(default = "default_true")]
    pub confirmation: bool,
}

/// Haptic settings
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct HapticConfig {
    #[serde(default = "default_true")]
    pub enabled: bool,

    #[validate(range(min = 0, max = 100))]
    #[serde(default = "default_haptic")]
    pub intensity: u8,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub patterns: Option<HapticPatterns>,
}

/// Physical control settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicalConfig {
    #[serde(default)]
    pub braille_output: bool,

    #[serde(default = "default_true")]
    pub button_audio_feedback: bool,

    #[serde(default = "default_true")]
    pub button_haptic_feedback: bool,
}

/// Complete HMI configuration
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct HmiConfig {
    pub config_id: Uuid,
    pub version: String,

    #[serde(default = "default_true")]
    pub active: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate]
    pub visual: Option<VisualConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate]
    pub audio: Option<AudioConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    #[validate]
    pub haptic: Option<HapticConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical: Option<PhysicalConfig>,
}

// ============================================================================
// Structs - Securement Status
// ============================================================================

/// Single securement point status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurementPoint {
    pub point_id: String,
    pub position: SecurementPosition,
    pub status: SecurementPointStatus,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub force_newtons: Option<f64>,

    #[serde(default)]
    pub locked: bool,
}

/// Complete securement status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurementStatus {
    pub status_id: Uuid,
    pub version: String,
    pub timestamp: DateTime<Utc>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_id: Option<Uuid>,

    #[serde(default)]
    pub securement_points: Vec<SecurementPoint>,

    pub overall_status: OverallSecurementStatus,

    #[serde(default)]
    pub wheelchair_detected: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub wheelchair_type: Option<WheelchairType>,

    #[serde(default)]
    pub safety_check_passed: bool,

    #[serde(default)]
    pub ready_to_move: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_message: Option<String>,
}

// ============================================================================
// Structs - Emergency Event
// ============================================================================

/// Passenger info in emergency
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyPassengerInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile_id: Option<Uuid>,

    #[serde(default)]
    pub disabilities: Vec<DisabilityType>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub emergency_contact: Option<EmergencyContact>,
}

/// Vehicle status in emergency
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyVehicleStatus {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_kmh: Option<f64>,

    #[serde(default)]
    pub is_moving: bool,

    #[serde(default)]
    pub doors_locked: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub securement_status: Option<OverallSecurementStatus>,
}

/// Emergency response status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyResponse {
    #[serde(default)]
    pub auto_pulled_over: bool,

    #[serde(default)]
    pub support_contacted: bool,

    #[serde(default)]
    pub emergency_services_called: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub eta_support_minutes: Option<f64>,
}

/// Complete emergency event
#[derive(Debug, Clone, Serialize, Deserialize, Validate)]
pub struct EmergencyEvent {
    pub event_id: Uuid,
    pub version: String,
    pub timestamp: DateTime<Utc>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_id: Option<Uuid>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub trip_id: Option<Uuid>,

    pub event_type: EmergencyEventType,
    pub severity: EmergencySeverity,

    #[validate]
    pub location: GeoLocation,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub passenger: Option<EmergencyPassengerInfo>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_status: Option<EmergencyVehicleStatus>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub response: Option<EmergencyResponse>,
}

// ============================================================================
// Structs - Message Envelope
// ============================================================================

/// Message envelope payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MessagePayload {
    Profile(PassengerProfile),
    Capabilities(VehicleCapabilities),
    TripRequest(TripRequest),
    TripResponse(TripResponse),
    HmiConfig(HmiConfig),
    Securement(SecurementStatus),
    Emergency(EmergencyEvent),
}

/// WIA Auto message content
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaAutoMessage {
    pub version: String,
    pub message_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub source: MessageEndpoint,
    pub destination: MessageEndpoint,
    pub message_type: MessageType,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<Uuid>,

    #[serde(default)]
    pub priority: MessagePriority,

    pub payload: serde_json::Value,
}

/// Message envelope wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageEnvelope {
    pub wia_auto: WiaAutoMessage,
}

// ============================================================================
// Default value functions
// ============================================================================

fn default_true() -> bool {
    true
}

fn default_brightness() -> u8 {
    70
}

fn default_volume() -> u8 {
    70
}

fn default_haptic() -> u8 {
    50
}

fn default_language() -> String {
    "en".to_string()
}

impl Default for ContrastMode {
    fn default() -> Self {
        ContrastMode::Normal
    }
}

impl Default for TextSize {
    fn default() -> Self {
        TextSize::Medium
    }
}

impl Default for ColorScheme {
    fn default() -> Self {
        ColorScheme::Default
    }
}

impl Default for TtsSpeed {
    fn default() -> Self {
        TtsSpeed::Normal
    }
}

impl Default for MessagePriority {
    fn default() -> Self {
        MessagePriority::Normal
    }
}
