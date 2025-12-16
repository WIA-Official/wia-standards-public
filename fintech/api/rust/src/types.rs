//! WIA Fintech Type Definitions
//!
//! Core type definitions for the WIA Financial Technology Accessibility Standard.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Common Enums
// ============================================================================

/// Accessibility level classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccessibilityLevel {
    None,
    Basic,
    Enhanced,
    Full,
    Universal,
}

/// Visual impairment levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum VisualLevel {
    None,
    LowVision,
    LegallyBlind,
    TotallyBlind,
}

/// Auditory impairment levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AuditoryLevel {
    None,
    Mild,
    Moderate,
    Severe,
    Profound,
    Deaf,
}

/// Motor impairment levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotorLevel {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Cognitive impairment levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveLevel {
    None,
    Mild,
    Moderate,
    Significant,
}

/// Color vision types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorVisionType {
    Normal,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
    Protanomaly,
    Deuteranomaly,
    Tritanomaly,
}

/// Sign language types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignLanguageType {
    ASL, // American
    BSL, // British
    KSL, // Korean
    JSL, // Japanese
    DGS, // German
    LSF, // French
    ISL, // International
    Other,
}

/// Limb functionality levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimbFunctionality {
    Full,
    Limited,
    Minimal,
    None,
}

/// Fine motor skill levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FineMotorLevel {
    Normal,
    Reduced,
    Limited,
    None,
}

/// Font size preferences
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum FontSize {
    Normal,
    Large,
    XLarge,
    XxLarge,
}

/// Authentication methods
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthenticationMethod {
    Pin,
    Password,
    BiometricFingerprint,
    BiometricFace,
    BiometricVoice,
    Pattern,
    OtpSms,
    OtpEmail,
    OtpApp,
    HardwareToken,
    Passkey,
}

/// Screen reader types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScreenReaderType {
    Voiceover,
    Talkback,
    Nvda,
    Jaws,
    Other,
}

/// Financial service types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FinancialServiceType {
    Banking,
    Atm,
    MobileBanking,
    OnlineBanking,
    PaymentTerminal,
    Cryptocurrency,
    Investment,
    Insurance,
    Lending,
}

/// Transaction types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransactionType {
    BalanceInquiry,
    Withdrawal,
    Deposit,
    Transfer,
    Payment,
    BillPay,
    Purchase,
    Refund,
}

/// Notification types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationType {
    Transaction,
    BalanceAlert,
    PaymentDue,
    FraudAlert,
    SecurityAlert,
    Promotional,
    AccountUpdate,
    DocumentReady,
    AppointmentReminder,
}

/// Notification priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationPriority {
    Low,
    Normal,
    High,
    Urgent,
    Emergency,
}

/// Notification category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationCategory {
    Financial,
    Security,
    Informational,
    Promotional,
    Legal,
}

/// Action types for notification actions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionType {
    Confirm,
    Deny,
    Review,
    Pay,
    Call,
    Visit,
    Upload,
    Acknowledge,
}

/// Display modes for bionic eye
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplayMode {
    Overlay,
    Full,
    Minimal,
}

/// Color vision types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ColorVision {
    Normal,
    Protanopia,
    Deuteranopia,
    Tritanopia,
    Achromatopsia,
    Protanomaly,
    Deuteranomaly,
    Tritanomaly,
}

/// Light sensitivity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LightSensitivity {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Connection status for WIA devices
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionStatus {
    Connected,
    Disconnected,
    Pairing,
}

/// WIA Level for ATM certification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WIALevel {
    Bronze,
    Silver,
    Gold,
    Platinum,
}

/// WIA device types for financial integration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WIADeviceType {
    Exoskeleton,
    BionicEye,
    VoiceSign,
    SmartWheelchair,
    HearingAid,
}

/// WIA certification levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WIACertificationLevel {
    Bronze,
    Silver,
    Gold,
    Platinum,
}

/// ATM status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ATMStatus {
    Operational,
    Limited,
    Offline,
}

/// Receipt format options
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReceiptFormat {
    Paper,
    Email,
    Sms,
    App,
    Braille,
    Audio,
    None,
}

/// Accessible card types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccessibleCardType {
    Standard,
    Notched,
    Braille,
    HighContrast,
    LargePrint,
}

// ============================================================================
// User Financial Accessibility Profile
// ============================================================================

/// User's financial accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UserFinancialAccessibilityProfile {
    pub profile_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_id: Option<String>,
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub updated_at: Option<DateTime<Utc>>,
    pub personal_info: PersonalInfo,
    pub accessibility_needs: AccessibilityNeeds,
    pub financial_preferences: FinancialPreferences,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub atm_preferences: Option<ATMPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub card_preferences: Option<CardPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_integration: Option<WIAIntegration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub emergency_settings: Option<EmergencySettings>,
}

/// Personal information
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PersonalInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_name: Option<String>,
    pub preferred_language: String,
    pub region: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timezone: Option<String>,
}

/// Accessibility needs container
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AccessibilityNeeds {
    pub sensory: SensoryAccessibilityNeeds,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub motor: Option<MotorAccessibilityNeeds>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cognitive: Option<CognitiveAccessibilityNeeds>,
}

/// Sensory accessibility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SensoryAccessibilityNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual: Option<VisualAccessibilityNeeds>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory: Option<AuditoryAccessibilityNeeds>,
}

/// Visual accessibility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VisualAccessibilityNeeds {
    pub level: VisualLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_vision: Option<ColorVision>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub light_sensitivity: Option<LightSensitivity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferences: Option<VisualPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistive_tech: Option<VisualAssistiveTech>,
}

/// Visual preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VisualPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub font_size: Option<FontSize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub high_contrast: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dark_mode: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reduce_motion: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_magnification: Option<f64>,
}

/// Visual assistive technology
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VisualAssistiveTech {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_reader: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_reader_type: Option<ScreenReaderType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub braille_display: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnifier: Option<bool>,
}

/// Auditory accessibility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AuditoryAccessibilityNeeds {
    pub level: AuditoryLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_hearing_aid: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_cochlear_implant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_sign_language: Option<SignLanguageType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferences: Option<AuditoryPreferences>,
}

/// Auditory preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AuditoryPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual_alerts: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub captions_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language_video: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mono_audio: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume_boost: Option<bool>,
}

/// Motor accessibility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorAccessibilityNeeds {
    pub level: MotorLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub upper_limb: Option<UpperLimbFunction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fine_motor: Option<FineMotorLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferences: Option<MotorPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistive_devices: Option<MotorAssistiveDevices>,
}

/// Upper limb functionality
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UpperLimbFunction {
    pub left: LimbFunctionality,
    pub right: LimbFunctionality,
}

/// Motor preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub large_targets: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_timeouts: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_control: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub switch_access: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eye_tracking: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dwell_click: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dwell_time: Option<u32>,
}

/// Motor assistive devices
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotorAssistiveDevices {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_wheelchair: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uses_exoskeleton: Option<bool>,
}

/// Cognitive accessibility needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CognitiveAccessibilityNeeds {
    pub level: CognitiveLevel,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferences: Option<CognitivePreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reading_level: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_needs: Option<CognitiveSupportNeeds>,
}

/// Cognitive preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CognitivePreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub simplified_interface: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clear_labels: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confirm_before_actions: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_by_step_guidance: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_aids: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time_extensions: Option<bool>,
}

/// Cognitive support needs
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CognitiveSupportNeeds {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub caregiver_access: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transaction_limits: Option<TransactionLimits>,
}

/// Transaction limits for cognitive support
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TransactionLimits {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub daily_limit: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub single_transaction_limit: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_confirmation: Option<bool>,
}

/// Financial preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FinancialPreferences {
    pub preferred_auth_method: Vec<AuthenticationMethod>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transaction_confirmation: Option<ConfirmationPreference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notification_preferences: Option<NotificationPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub security_preferences: Option<SecurityAccessibilityPreferences>,
}

/// Confirmation preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConfirmationPreference {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_explicit_confirm: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repeat_confirmation: Option<bool>,
}

/// Notification preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NotificationPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channels: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quiet_hours: Option<QuietHours>,
}

/// Quiet hours configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct QuietHours {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub allow_urgent: Option<bool>,
}

/// Security accessibility preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SecurityAccessibilityPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pin_preferences: Option<PinPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub otp_preferences: Option<OtpPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_preferences: Option<SessionPreferences>,
}

/// PIN preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PinPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub length: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_feedback: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic_feedback: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_timeout: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
}

/// OTP preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OtpPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery_method: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_validity: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validity_seconds: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_readout: Option<bool>,
}

/// Session preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SessionPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_session: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_timeout_minutes: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inactivity_warning: Option<bool>,
}

/// ATM preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ATMPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_interface: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio_guidance: Option<AudioGuidanceSettings>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout: Option<TimeoutPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub receipt_format: Option<ReceiptFormat>,
}

/// Audio guidance settings
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AudioGuidanceSettings {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speech_rate: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,
}

/// Timeout preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TimeoutPreferences {
    pub extended: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seconds: Option<u32>,
}

/// Card preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CardPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_card_type: Option<AccessibleCardType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pin_entry_method: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub contactless_preferred: Option<bool>,
}

/// WIA integration settings
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WIAIntegration {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub connected_devices: Option<Vec<WIADeviceConnection>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_output_device: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cross_device_sync: Option<bool>,
}

/// WIA device connection
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WIADeviceConnection {
    pub device_id: String,
    pub device_type: WIADeviceType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub connection_status: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<DateTime<Utc>>,
}

/// Emergency settings
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EmergencySettings {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub emergency_contacts: Option<Vec<EmergencyContact>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fraud_alert_preferences: Option<FraudAlertPreferences>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub panic_button_enabled: Option<bool>,
}

/// Emergency contact
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EmergencyContact {
    pub name: String,
    pub phone: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub relationship: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notify_on_fraud: Option<bool>,
}

/// Fraud alert preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FraudAlertPreferences {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub immediate_call: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sms_alert: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email_alert: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_device_alert: Option<bool>,
}

// ============================================================================
// ATM Accessibility Profile
// ============================================================================

/// ATM accessibility profile
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ATMAccessibilityProfile {
    pub atm_id: String,
    pub bank_code: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    pub location: ATMLocation,
    pub status: ATMStatus,
    pub physical_accessibility: PhysicalAccessibility,
    pub audio_accessibility: AudioAccessibility,
    pub tactile_accessibility: TactileAccessibility,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_accessibility: Option<ScreenAccessibility>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub operational_features: Option<OperationalFeatures>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_integration: Option<ATMWIAIntegration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certification: Option<ATMCertification>,
}

/// ATM location
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ATMLocation {
    pub address: String,
    pub city: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state: Option<String>,
    pub country: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub postal_code: Option<String>,
    pub latitude: f64,
    pub longitude: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location_description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub access_instructions: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nearby_landmarks: Option<Vec<String>>,
}

/// Physical accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PhysicalAccessibility {
    pub wheelchair_accessible: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub height_adjustable: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub screen_height: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keypad_height: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub clear_floor_space: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub indoor_outdoor: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sheltered: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub well_lit: Option<bool>,
}

/// Audio accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AudioAccessibility {
    pub audio_guidance: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub headphone_jack: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jack_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jack_location: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speaker_output: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume_adjustable: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub supported_languages: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub privacy_mode: Option<bool>,
}

/// Tactile accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TactileAccessibility {
    pub braille_labels: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tactile_keypad: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keypad_layout: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub key5_marker: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confirm_button_marker: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cancel_button_marker: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub card_slot_tactile: Option<bool>,
}

/// Screen accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ScreenAccessibility {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub touchscreen: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub physical_buttons: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub high_contrast_mode: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub font_size_adjustable: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color_blind_mode: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anti_glare: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub privacy_screen: Option<bool>,
}

/// Operational features
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OperationalFeatures {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_timeout: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub receipt_options: Option<Vec<ReceiptFormat>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cardless_transaction: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nfc_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qr_code_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mobile_pre_staging: Option<bool>,
}

/// ATM WIA integration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ATMWIAIntegration {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_profile_sync: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exoskeleton_guidance: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bionic_eye_display: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_sign_support: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smart_wheelchair_positioning: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bluetooth_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nfc_wia_enabled: Option<bool>,
}

/// ATM certification
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ATMCertification {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ada_compliant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eaa_compliant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_level: Option<WIACertificationLevel>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_inspection_date: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certifications: Option<Vec<String>>,
}

// ============================================================================
// Accessible Notification
// ============================================================================

/// Accessible financial notification
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AccessibleNotification {
    pub notification_id: String,
    pub timestamp: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<DateTime<Utc>>,
    pub notification_type: NotificationType,
    pub priority: NotificationPriority,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<NotificationCategory>,
    pub content: NotificationContent,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action: Option<NotificationAction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery: Option<DeliveryConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accessibility: Option<NotificationAccessibility>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_delivery: Option<WIADeliveryConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tracking: Option<NotificationTracking>,
}

/// Notification content
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NotificationContent {
    pub title: String,
    pub body: String,
    pub formats: ContentFormats,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub localizations: Option<HashMap<String, LocalizedContent>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language: Option<SignLanguageContent>,
}

/// Content formats
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ContentFormats {
    pub plain_text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rich_text: Option<String>,
    pub simple_language: String,
    pub voice_script: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ssml: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub braille: Option<String>,
}

/// Localized content
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LocalizedContent {
    pub title: String,
    pub body: String,
    pub voice_script: String,
}

/// Sign language content
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SignLanguageContent {
    pub available: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub languages: Option<Vec<SignLanguageType>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub video_urls: Option<HashMap<String, String>>,
}

/// Notification action
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NotificationAction {
    pub required: bool,
    pub action_type: String,
    pub action_label: String,
    pub accessible_instructions: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deadline: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
}

/// Delivery configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DeliveryConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channels: Option<Vec<DeliveryChannel>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub respect_quiet_hours: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub escalation: Option<EscalationConfig>,
}

/// Delivery channel
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DeliveryChannel {
    pub channel_type: String,
    pub priority: u8,
    pub enabled: bool,
}

/// Escalation configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EscalationConfig {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub escalate_after_minutes: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_escalations: Option<u32>,
}

/// Notification accessibility settings
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NotificationAccessibility {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visual: Option<VisualNotificationConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auditory: Option<AuditoryNotificationConfig>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub haptic: Option<HapticNotificationConfig>,
}

/// Visual notification config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VisualNotificationConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icon_code: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icon_alt_text: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub background_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text_color: Option<String>,
}

/// Auditory notification config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AuditoryNotificationConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sound_file: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tts_enabled: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tts_voice: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tts_rate: Option<f32>,
}

/// Haptic notification config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HapticNotificationConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pattern: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intensity: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<u32>,
}

/// WIA delivery configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WIADeliveryConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exoskeleton: Option<ExoskeletonDelivery>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bionic_eye: Option<BionicEyeDelivery>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub voice_sign: Option<VoiceSignDelivery>,
}

/// Exoskeleton delivery config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExoskeletonDelivery {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pattern: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub body_region: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intensity: Option<u8>,
}

/// Bionic eye delivery config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BionicEyeDelivery {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display_mode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<u32>,
}

/// Voice-Sign delivery config
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VoiceSignDelivery {
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language: Option<SignLanguageType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub avatar_enabled: Option<bool>,
}

/// Notification tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct NotificationTracking {
    pub sent: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sent_at: Option<DateTime<Utc>>,
    pub delivered: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivered_at: Option<DateTime<Utc>>,
    pub read: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub read_at: Option<DateTime<Utc>>,
    pub actioned: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub actioned_at: Option<DateTime<Utc>>,
}

// ============================================================================
// Accessibility Score
// ============================================================================

/// Accessibility score result
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AccessibilityScore {
    pub overall: f32,
    pub visual: f32,
    pub auditory: f32,
    pub motor: f32,
    pub cognitive: f32,
    pub wia_integration: f32,
    pub certification_level: WIACertificationLevel,
}

/// Compatibility result between user and service/ATM
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CompatibilityResult {
    pub compatible: bool,
    pub score: f32,
    pub issues: Vec<CompatibilityIssue>,
    pub recommendations: Vec<String>,
}

/// Compatibility issue
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CompatibilityIssue {
    pub category: String,
    pub severity: String,
    pub description: String,
    pub user_need: String,
    pub service_capability: String,
}
