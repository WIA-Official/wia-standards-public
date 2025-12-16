//! Core medical device accessibility logic

use async_trait::async_trait;
use chrono::Utc;

use crate::error::{MedicalError, Result};
use crate::types::*;

// ============================================================================
// Profile Manager Trait
// ============================================================================

/// Trait for managing medical device accessibility profiles
#[async_trait]
pub trait ProfileManager: Send + Sync {
    /// Get a device profile by ID
    async fn get_device_profile(&self, profile_id: &str) -> Result<MedicalDeviceAccessibilityProfile>;

    /// Save a device profile
    async fn save_device_profile(&self, profile: &MedicalDeviceAccessibilityProfile) -> Result<()>;

    /// Delete a device profile
    async fn delete_device_profile(&self, profile_id: &str) -> Result<()>;

    /// List all device profiles
    async fn list_device_profiles(&self) -> Result<Vec<String>>;

    /// Get a user profile by ID
    async fn get_user_profile(&self, user_id: &str) -> Result<UserMedicalAccessibilityProfile>;

    /// Save a user profile
    async fn save_user_profile(&self, profile: &UserMedicalAccessibilityProfile) -> Result<()>;

    /// Delete a user profile
    async fn delete_user_profile(&self, user_id: &str) -> Result<()>;

    /// List all user profiles
    async fn list_user_profiles(&self) -> Result<Vec<String>>;
}

// ============================================================================
// Accessibility Score Calculator
// ============================================================================

/// Calculate accessibility scores for devices
pub struct AccessibilityScoreCalculator;

impl AccessibilityScoreCalculator {
    /// Calculate overall accessibility score for a device
    pub fn calculate(features: &DeviceAccessibilityFeatures) -> AccessibilityScore {
        let visual = Self::calculate_visual_score(&features.visual);
        let auditory = Self::calculate_auditory_score(&features.auditory);
        let motor = Self::calculate_motor_score(&features.motor);
        let cognitive = Self::calculate_cognitive_score(&features.cognitive);

        let overall = (visual + auditory + motor + cognitive) / 4.0;

        AccessibilityScore {
            overall,
            visual,
            auditory,
            motor,
            cognitive,
        }
    }

    fn calculate_visual_score(visual: &VisualAccessibility) -> f32 {
        let mut score = 0.0;
        let mut max_score = 0.0;

        // Screen reader support (25 points)
        max_score += 25.0;
        if let Some(ref sr) = visual.screen_reader {
            if sr.supported {
                score += match sr.level {
                    SupportLevel::Full => 25.0,
                    SupportLevel::Partial => 15.0,
                    SupportLevel::None => 0.0,
                };
            }
        }

        // Voice output (25 points)
        max_score += 25.0;
        if let Some(ref vo) = visual.voice_output {
            if vo.supported {
                score += 15.0;
                if vo.speed_adjustable {
                    score += 5.0;
                }
                if vo.volume_adjustable {
                    score += 5.0;
                }
            }
        }

        // Display accessibility (25 points)
        max_score += 25.0;
        if let Some(ref da) = visual.display_accessibility {
            if da.high_contrast_mode {
                score += 8.0;
            }
            if da.large_text_mode || da.text_size_adjustable {
                score += 8.0;
            }
            if !da.color_blind_modes.is_empty() {
                score += 9.0;
            }
        }

        // Non-visual alternatives (25 points)
        max_score += 25.0;
        if let Some(ref nva) = visual.non_visual_alternatives {
            if nva.audio_feedback {
                score += 10.0;
            }
            if nva.haptic_feedback {
                score += 10.0;
            }
            if nva.braille_output {
                score += 5.0;
            }
        }

        (score / max_score) * 100.0
    }

    fn calculate_auditory_score(auditory: &AuditoryAccessibility) -> f32 {
        let mut score = 0.0;
        let mut max_score = 0.0;

        // Visual alerts (30 points)
        max_score += 30.0;
        if let Some(ref va) = auditory.visual_alerts {
            if va.supported {
                score += 20.0;
                if va.customizable {
                    score += 10.0;
                }
            }
        }

        // Haptic alerts (30 points)
        max_score += 30.0;
        if let Some(ref ha) = auditory.haptic_alerts {
            if ha.supported {
                score += 20.0;
                if ha.intensity_levels > 1 {
                    score += 10.0;
                }
            }
        }

        // Audio adjustments (20 points)
        max_score += 20.0;
        if let Some(ref aa) = auditory.audio_adjustments {
            if aa.hearing_aid_compatible {
                score += 10.0;
            }
            if aa.t_coil_compatible {
                score += 5.0;
            }
            if aa.mono_audio {
                score += 5.0;
            }
        }

        // Text alternatives (20 points)
        max_score += 20.0;
        if let Some(ref ta) = auditory.text_alternatives {
            if ta.on_screen_text {
                score += 10.0;
            }
            if ta.closed_captions {
                score += 5.0;
            }
            if ta.real_time_transcription {
                score += 5.0;
            }
        }

        (score / max_score) * 100.0
    }

    fn calculate_motor_score(motor: &MotorAccessibility) -> f32 {
        let mut score = 0.0;
        let mut max_score = 0.0;

        // Alternative input (30 points)
        max_score += 30.0;
        if let Some(ref ai) = motor.alternative_input {
            if ai.voice_control {
                score += 10.0;
            }
            if ai.switch_access {
                score += 8.0;
            }
            if ai.eye_tracking || ai.head_tracking {
                score += 7.0;
            }
            if ai.wia_exoskeleton {
                score += 5.0;
            }
        }

        // Physical controls (25 points)
        max_score += 25.0;
        if let Some(ref pc) = motor.physical_controls {
            if pc.large_buttons {
                score += 6.0;
            }
            if pc.button_spacing_adequate {
                score += 6.0;
            }
            if pc.low_force_buttons {
                score += 5.0;
            }
            if pc.one_handed_operation {
                score += 5.0;
            }
            if pc.no_fine_motor_required {
                score += 3.0;
            }
        }

        // Touchscreen (25 points)
        max_score += 25.0;
        if let Some(ref ts) = motor.touchscreen {
            if ts.gesture_alternatives {
                score += 8.0;
            }
            if ts.touch_accommodation {
                score += 8.0;
            }
            if ts.dwell_control {
                score += 6.0;
            }
            if ts.haptic_feedback {
                score += 3.0;
            }
        }

        // Automation (20 points)
        max_score += 20.0;
        if let Some(ref auto) = motor.automation {
            if auto.auto_measurement {
                score += 8.0;
            }
            if auto.scheduled_operation {
                score += 6.0;
            }
            if auto.remote_control {
                score += 6.0;
            }
        }

        (score / max_score) * 100.0
    }

    fn calculate_cognitive_score(cognitive: &CognitiveAccessibility) -> f32 {
        let mut score = 0.0;
        let mut max_score = 0.0;

        // Simplified interface (30 points)
        max_score += 30.0;
        if let Some(ref si) = cognitive.simplified_interface {
            if si.available {
                score += 10.0;
            }
            if si.reduced_options {
                score += 7.0;
            }
            if si.step_by_step_guidance {
                score += 8.0;
            }
            if si.clear_icons {
                score += 5.0;
            }
        }

        // Memory support (25 points)
        max_score += 25.0;
        if let Some(ref ms) = cognitive.memory_support {
            if ms.reminders {
                score += 8.0;
            }
            if ms.history_log {
                score += 7.0;
            }
            if ms.caregiver_notifications {
                score += 5.0;
            }
            if ms.auto_data_sync {
                score += 5.0;
            }
        }

        // Error prevention (25 points)
        max_score += 25.0;
        if let Some(ref ep) = cognitive.error_prevention {
            if ep.confirmation_prompts {
                score += 7.0;
            }
            if ep.undo_capability {
                score += 6.0;
            }
            if ep.clear_error_messages {
                score += 7.0;
            }
            if ep.recovery_guidance {
                score += 5.0;
            }
        }

        // Language support (20 points)
        max_score += 20.0;
        if let Some(ref ls) = cognitive.language_support {
            if !ls.languages.is_empty() {
                score += 5.0;
            }
            if ls.simple_language_mode {
                score += 6.0;
            }
            if ls.icon_based_navigation {
                score += 5.0;
            }
            if ls.pictogram_support {
                score += 4.0;
            }
        }

        (score / max_score) * 100.0
    }
}

// ============================================================================
// Profile Matcher
// ============================================================================

/// Match user profiles with device capabilities
pub struct ProfileMatcher;

impl ProfileMatcher {
    /// Check if a device meets user accessibility needs
    pub fn is_compatible(
        device: &MedicalDeviceAccessibilityProfile,
        user: &UserMedicalAccessibilityProfile,
    ) -> CompatibilityResult {
        let mut issues: Vec<CompatibilityIssue> = Vec::new();
        let mut score = 100.0;

        // Check visual compatibility
        if let Some(ref sensory) = user.accessibility_needs.sensory {
            if let Some(ref visual_needs) = sensory.visual {
                let visual_issues = Self::check_visual_compatibility(
                    &device.accessibility.visual,
                    visual_needs,
                );
                for issue in &visual_issues {
                    score -= issue.severity_score();
                }
                issues.extend(visual_issues);
            }

            if let Some(ref auditory_needs) = sensory.auditory {
                let auditory_issues = Self::check_auditory_compatibility(
                    &device.accessibility.auditory,
                    auditory_needs,
                );
                for issue in &auditory_issues {
                    score -= issue.severity_score();
                }
                issues.extend(auditory_issues);
            }
        }

        // Check motor compatibility
        if let Some(ref motor_needs) = user.accessibility_needs.motor {
            let motor_issues = Self::check_motor_compatibility(
                &device.accessibility.motor,
                motor_needs,
            );
            for issue in &motor_issues {
                score -= issue.severity_score();
            }
            issues.extend(motor_issues);
        }

        // Check input method compatibility
        let input_issues = Self::check_input_compatibility(
            &device.accessibility,
            &user.input_preferences,
        );
        for issue in &input_issues {
            score -= issue.severity_score();
        }
        issues.extend(input_issues);

        CompatibilityResult {
            compatible: issues.iter().all(|i| i.severity != IssueSeverity::Critical),
            score: score.max(0.0),
            issues,
        }
    }

    fn check_visual_compatibility(
        device_visual: &VisualAccessibility,
        user_needs: &VisualNeeds,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        match user_needs.level {
            VisualLevel::TotallyBlind => {
                // Must have full screen reader or voice output
                let has_screen_reader = device_visual
                    .screen_reader
                    .as_ref()
                    .map(|sr| sr.supported && sr.level == SupportLevel::Full)
                    .unwrap_or(false);

                let has_voice = device_visual
                    .voice_output
                    .as_ref()
                    .map(|vo| vo.supported)
                    .unwrap_or(false);

                if !has_screen_reader && !has_voice {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Visual,
                        severity: IssueSeverity::Critical,
                        description: "Device lacks screen reader or voice output for blind users".to_string(),
                        recommendation: "Consider a device with full screen reader support".to_string(),
                    });
                }
            }
            VisualLevel::LegallyBlind | VisualLevel::LowVision => {
                let has_large_text = device_visual
                    .display_accessibility
                    .as_ref()
                    .map(|da| da.large_text_mode || da.text_size_adjustable)
                    .unwrap_or(false);

                if !has_large_text {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Visual,
                        severity: IssueSeverity::Major,
                        description: "Device lacks large text support".to_string(),
                        recommendation: "Look for devices with adjustable text size".to_string(),
                    });
                }
            }
            VisualLevel::None => {}
        }

        // Check color blindness support
        if let Some(ref color_blind) = user_needs.color_blind {
            let has_mode = device_visual
                .display_accessibility
                .as_ref()
                .map(|da| da.color_blind_modes.contains(color_blind))
                .unwrap_or(false);

            if !has_mode {
                issues.push(CompatibilityIssue {
                    category: IssueCategory::Visual,
                    severity: IssueSeverity::Minor,
                    description: format!("Device lacks {:?} color blind mode", color_blind),
                    recommendation: "Check if device has alternative visual indicators".to_string(),
                });
            }
        }

        issues
    }

    fn check_auditory_compatibility(
        device_auditory: &AuditoryAccessibility,
        user_needs: &AuditoryNeeds,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        match user_needs.level {
            AuditoryLevel::Deaf => {
                let has_visual_alerts = device_auditory
                    .visual_alerts
                    .as_ref()
                    .map(|va| va.supported)
                    .unwrap_or(false);

                let has_haptic_alerts = device_auditory
                    .haptic_alerts
                    .as_ref()
                    .map(|ha| ha.supported)
                    .unwrap_or(false);

                if !has_visual_alerts && !has_haptic_alerts {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Auditory,
                        severity: IssueSeverity::Critical,
                        description: "Device lacks visual or haptic alerts for deaf users".to_string(),
                        recommendation: "Choose a device with visual or haptic alert options".to_string(),
                    });
                }
            }
            AuditoryLevel::HardOfHearing => {
                if user_needs.uses_hearing_aid.unwrap_or(false) {
                    let hearing_aid_compatible = device_auditory
                        .audio_adjustments
                        .as_ref()
                        .map(|aa| aa.hearing_aid_compatible)
                        .unwrap_or(false);

                    if !hearing_aid_compatible {
                        issues.push(CompatibilityIssue {
                            category: IssueCategory::Auditory,
                            severity: IssueSeverity::Major,
                            description: "Device is not hearing aid compatible".to_string(),
                            recommendation: "Look for HAC-rated devices".to_string(),
                        });
                    }
                }
            }
            AuditoryLevel::None => {}
        }

        issues
    }

    fn check_motor_compatibility(
        device_motor: &MotorAccessibility,
        user_needs: &MotorNeeds,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        // Check upper limb needs
        if let Some(ref upper_limb) = user_needs.upper_limb {
            if upper_limb.level == ImpairmentLevel::Severe {
                let has_alternative = device_motor
                    .alternative_input
                    .as_ref()
                    .map(|ai| ai.voice_control || ai.switch_access || ai.wia_exoskeleton)
                    .unwrap_or(false);

                if !has_alternative {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Motor,
                        severity: IssueSeverity::Critical,
                        description: "Device lacks alternative input for severe motor impairment".to_string(),
                        recommendation: "Consider devices with voice control or switch access".to_string(),
                    });
                }
            }

            if upper_limb.tremor.unwrap_or(false) {
                let has_accommodation = device_motor
                    .touchscreen
                    .as_ref()
                    .map(|ts| ts.touch_accommodation)
                    .unwrap_or(false);

                if !has_accommodation {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Motor,
                        severity: IssueSeverity::Major,
                        description: "Device lacks touch accommodation for tremor".to_string(),
                        recommendation: "Look for devices with touch accommodation features".to_string(),
                    });
                }
            }
        }

        // Check fine motor needs
        if let Some(ref fine_motor) = user_needs.fine_motor {
            if fine_motor.level == ImpairmentLevel::Severe {
                let has_large_buttons = device_motor
                    .physical_controls
                    .as_ref()
                    .map(|pc| pc.large_buttons && pc.no_fine_motor_required)
                    .unwrap_or(false);

                if !has_large_buttons {
                    issues.push(CompatibilityIssue {
                        category: IssueCategory::Motor,
                        severity: IssueSeverity::Major,
                        description: "Device requires fine motor control".to_string(),
                        recommendation: "Choose devices with large buttons or voice control".to_string(),
                    });
                }
            }
        }

        issues
    }

    fn check_input_compatibility(
        device_accessibility: &DeviceAccessibilityFeatures,
        user_input: &UserInputPreferences,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        let supports_input = match user_input.primary_input {
            InputMethod::Voice => device_accessibility
                .motor
                .alternative_input
                .as_ref()
                .map(|ai| ai.voice_control)
                .unwrap_or(false),
            InputMethod::Switch => device_accessibility
                .motor
                .alternative_input
                .as_ref()
                .map(|ai| ai.switch_access)
                .unwrap_or(false),
            InputMethod::EyeTracking => device_accessibility
                .motor
                .alternative_input
                .as_ref()
                .map(|ai| ai.eye_tracking)
                .unwrap_or(false),
            InputMethod::HeadTracking => device_accessibility
                .motor
                .alternative_input
                .as_ref()
                .map(|ai| ai.head_tracking)
                .unwrap_or(false),
            InputMethod::WiaExoskeleton => device_accessibility
                .motor
                .alternative_input
                .as_ref()
                .map(|ai| ai.wia_exoskeleton)
                .unwrap_or(false),
            InputMethod::Touch | InputMethod::Buttons => true,
        };

        if !supports_input {
            issues.push(CompatibilityIssue {
                category: IssueCategory::Input,
                severity: IssueSeverity::Critical,
                description: format!(
                    "Device does not support user's primary input method: {:?}",
                    user_input.primary_input
                ),
                recommendation: "Check for devices supporting your preferred input method".to_string(),
            });
        }

        issues
    }
}

/// Compatibility check result
#[derive(Debug, Clone)]
pub struct CompatibilityResult {
    /// Whether device is compatible
    pub compatible: bool,
    /// Compatibility score (0-100)
    pub score: f32,
    /// List of compatibility issues
    pub issues: Vec<CompatibilityIssue>,
}

/// Compatibility issue
#[derive(Debug, Clone)]
pub struct CompatibilityIssue {
    /// Issue category
    pub category: IssueCategory,
    /// Issue severity
    pub severity: IssueSeverity,
    /// Description of the issue
    pub description: String,
    /// Recommendation to resolve
    pub recommendation: String,
}

impl CompatibilityIssue {
    fn severity_score(&self) -> f32 {
        match self.severity {
            IssueSeverity::Critical => 30.0,
            IssueSeverity::Major => 15.0,
            IssueSeverity::Minor => 5.0,
        }
    }
}

/// Issue category
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssueCategory {
    Visual,
    Auditory,
    Motor,
    Cognitive,
    Input,
    Physical,
}

/// Issue severity
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssueSeverity {
    Critical,
    Major,
    Minor,
}

// ============================================================================
// Device Profile Builder
// ============================================================================

/// Builder for creating device accessibility profiles
pub struct DeviceProfileBuilder {
    profile: MedicalDeviceAccessibilityProfile,
}

impl DeviceProfileBuilder {
    /// Create a new profile builder
    pub fn new() -> Self {
        Self {
            profile: MedicalDeviceAccessibilityProfile::default(),
        }
    }

    /// Set device information
    pub fn device(mut self, device: MedicalDeviceInfo) -> Self {
        self.profile.device = device;
        self
    }

    /// Set manufacturer
    pub fn manufacturer(mut self, manufacturer: &str) -> Self {
        self.profile.device.manufacturer = manufacturer.to_string();
        self
    }

    /// Set model
    pub fn model(mut self, model: &str) -> Self {
        self.profile.device.model = model.to_string();
        self
    }

    /// Set device name
    pub fn device_name(mut self, name: &str) -> Self {
        self.profile.device.device_name = name.to_string();
        self
    }

    /// Set device type
    pub fn device_type(mut self, device_type: MedicalDeviceType) -> Self {
        self.profile.device.device_type = device_type;
        self
    }

    /// Set device category
    pub fn device_category(mut self, category: DeviceCategory) -> Self {
        self.profile.device.device_category = category;
        self
    }

    /// Set accessibility features
    pub fn accessibility(mut self, accessibility: DeviceAccessibilityFeatures) -> Self {
        self.profile.accessibility = accessibility;
        self
    }

    /// Set WIA integration
    pub fn wia_integration(mut self, integration: WIAIntegration) -> Self {
        self.profile.wia_integration = Some(integration);
        self
    }

    /// Enable voice output
    pub fn with_voice_output(mut self, languages: Vec<String>) -> Self {
        self.profile.accessibility.visual.voice_output = Some(VoiceOutput {
            supported: true,
            readings: Vec::new(),
            languages,
            speed_adjustable: true,
            volume_adjustable: true,
        });
        self
    }

    /// Enable haptic feedback
    pub fn with_haptic_feedback(mut self) -> Self {
        self.profile.accessibility.auditory.haptic_alerts = Some(HapticAlerts {
            supported: true,
            patterns: Vec::new(),
            intensity_levels: 5,
        });
        self
    }

    /// Build the profile
    pub fn build(mut self) -> Result<MedicalDeviceAccessibilityProfile> {
        // Calculate accessibility score
        self.profile.accessibility.accessibility_score =
            Some(AccessibilityScoreCalculator::calculate(&self.profile.accessibility));

        // Update timestamp
        self.profile.updated_at = Utc::now();

        // Validate
        self.validate()?;

        Ok(self.profile)
    }

    fn validate(&self) -> Result<()> {
        if self.profile.device.manufacturer.is_empty() {
            return Err(MedicalError::ValidationError(
                "Manufacturer is required".to_string(),
            ));
        }
        if self.profile.device.model.is_empty() {
            return Err(MedicalError::ValidationError(
                "Model is required".to_string(),
            ));
        }
        Ok(())
    }
}

impl Default for DeviceProfileBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// User Profile Builder
// ============================================================================

/// Builder for creating user accessibility profiles
pub struct UserProfileBuilder {
    profile: UserMedicalAccessibilityProfile,
}

impl UserProfileBuilder {
    /// Create a new user profile builder
    pub fn new() -> Self {
        Self {
            profile: UserMedicalAccessibilityProfile::default(),
        }
    }

    /// Set visual needs
    pub fn visual_needs(mut self, level: VisualLevel) -> Self {
        let sensory = self.profile.accessibility_needs.sensory.get_or_insert_with(Default::default);
        sensory.visual = Some(VisualNeeds {
            level,
            color_blind: None,
            light_sensitivity: None,
            field_of_vision: None,
        });
        self
    }

    /// Set color blindness
    pub fn color_blind(mut self, mode: ColorBlindMode) -> Self {
        let sensory = self.profile.accessibility_needs.sensory.get_or_insert_with(Default::default);
        if let Some(ref mut visual) = sensory.visual {
            visual.color_blind = Some(mode);
        }
        self
    }

    /// Set auditory needs
    pub fn auditory_needs(mut self, level: AuditoryLevel) -> Self {
        let sensory = self.profile.accessibility_needs.sensory.get_or_insert_with(Default::default);
        sensory.auditory = Some(AuditoryNeeds {
            level,
            uses_hearing_aid: None,
            uses_cochlear_implant: None,
            frequency_range_affected: None,
        });
        self
    }

    /// Set motor needs
    pub fn motor_needs(mut self, level: ImpairmentLevel) -> Self {
        let motor = self.profile.accessibility_needs.motor.get_or_insert_with(Default::default);
        motor.upper_limb = Some(UpperLimbNeeds {
            level,
            affected_side: None,
            tremor: None,
            grip_strength: None,
        });
        self
    }

    /// Set primary input method
    pub fn primary_input(mut self, method: InputMethod) -> Self {
        self.profile.input_preferences.primary_input = method;
        self
    }

    /// Set visual preferences
    pub fn visual_preferences(
        mut self,
        text_size: TextSize,
        high_contrast: bool,
        dark_mode: bool,
    ) -> Self {
        let sensory = self.profile.sensory_preferences.visual.get_or_insert_with(|| {
            VisualPreferences {
                text_size: TextSize::Medium,
                high_contrast: false,
                dark_mode: false,
                color_scheme: None,
                reduce_motion: false,
                reduce_transparency: None,
            }
        });
        sensory.text_size = text_size;
        sensory.high_contrast = high_contrast;
        sensory.dark_mode = dark_mode;
        self
    }

    /// Enable WIA exoskeleton
    pub fn with_exoskeleton(mut self, device_id: &str, assistance_level: u8) -> Self {
        let wia = self.profile.wia_devices.get_or_insert_with(Default::default);
        wia.exoskeleton = Some(ExoskeletonSettings {
            device_id: device_id.to_string(),
            enabled: true,
            haptic_intensity: 50,
            assistance_level,
        });
        self.profile.input_preferences.primary_input = InputMethod::WiaExoskeleton;
        self
    }

    /// Build the profile
    pub fn build(self) -> UserMedicalAccessibilityProfile {
        self.profile
    }
}

impl Default for UserProfileBuilder {
    fn default() -> Self {
        Self::new()
    }
}
