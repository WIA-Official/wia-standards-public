//! Core Fintech Accessibility Logic
//!
//! Business logic for financial accessibility operations.

use async_trait::async_trait;
use crate::error::FintechResult;
use crate::types::*;
use serde::{Deserialize, Serialize};

// ============================================================================
// Traits
// ============================================================================

/// Profile manager trait for user profiles
#[async_trait]
pub trait ProfileManager: Send + Sync {
    /// Create a new user profile
    async fn create_profile(&self, profile: UserFinancialAccessibilityProfile) -> FintechResult<UserFinancialAccessibilityProfile>;

    /// Get a user profile by ID
    async fn get_profile(&self, profile_id: &str) -> FintechResult<UserFinancialAccessibilityProfile>;

    /// Update a user profile
    async fn update_profile(&self, profile: UserFinancialAccessibilityProfile) -> FintechResult<UserFinancialAccessibilityProfile>;

    /// Delete a user profile
    async fn delete_profile(&self, profile_id: &str) -> FintechResult<()>;

    /// List all profiles
    async fn list_profiles(&self) -> FintechResult<Vec<UserFinancialAccessibilityProfile>>;
}

/// ATM manager trait
#[async_trait]
pub trait ATMManager: Send + Sync {
    /// Get an ATM by ID
    async fn get_atm(&self, atm_id: &str) -> FintechResult<ATMAccessibilityProfile>;

    /// Search ATMs by location
    async fn search_atms(&self, latitude: f64, longitude: f64, radius_km: f64) -> FintechResult<Vec<ATMAccessibilityProfile>>;

    /// Search accessible ATMs matching user needs
    async fn search_accessible_atms(
        &self,
        latitude: f64,
        longitude: f64,
        radius_km: f64,
        user_profile: &UserFinancialAccessibilityProfile,
    ) -> FintechResult<Vec<ATMWithCompatibility>>;

    /// Register a new ATM
    async fn register_atm(&self, atm: ATMAccessibilityProfile) -> FintechResult<ATMAccessibilityProfile>;

    /// Update ATM status
    async fn update_atm_status(&self, atm_id: &str, status: ATMStatus) -> FintechResult<()>;
}

/// ATM with compatibility information
#[derive(Debug, Clone)]
pub struct ATMWithCompatibility {
    pub atm: ATMAccessibilityProfile,
    pub compatibility: CompatibilityResult,
    pub distance_km: f64,
}

/// Severity level for compatibility issues
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueSeverity {
    /// Critical issue - prevents usage
    Critical,
    /// Major issue - significantly impacts usage
    Major,
    /// Minor issue - inconvenience but usable
    Minor,
    /// Informational - suggestion for improvement
    Informational,
}

impl std::fmt::Display for IssueSeverity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IssueSeverity::Critical => write!(f, "critical"),
            IssueSeverity::Major => write!(f, "major"),
            IssueSeverity::Minor => write!(f, "minor"),
            IssueSeverity::Informational => write!(f, "informational"),
        }
    }
}

/// Compatibility issue between user needs and service
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompatibilityIssue {
    pub category: String,
    pub severity: IssueSeverity,
    pub description: String,
    pub user_need: String,
    pub service_capability: String,
}

/// Result of compatibility check
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompatibilityResult {
    pub is_compatible: bool,
    pub compatibility_score: f32,
    pub issues: Vec<CompatibilityIssue>,
    pub recommendations: Vec<String>,
}

/// Accessibility score breakdown
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityScore {
    pub overall: f32,
    pub visual: f32,
    pub auditory: f32,
    pub motor: f32,
    pub cognitive: f32,
    pub wia_integration: f32,
    pub certification_level: WIACertificationLevel,
}

/// WIA Certification level
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum WIACertificationLevel {
    Platinum,
    Gold,
    Silver,
    Bronze,
}

// ============================================================================
// Accessibility Score Calculator
// ============================================================================

/// Calculator for accessibility scores
pub struct AccessibilityScoreCalculator;

impl AccessibilityScoreCalculator {
    /// Create a new calculator
    pub fn new() -> Self {
        Self
    }

    /// Calculate accessibility score for an ATM
    pub fn calculate_atm_score(&self, atm: &ATMAccessibilityProfile) -> AccessibilityScore {
        let visual = Self::calculate_visual_score_atm(atm);
        let auditory = Self::calculate_auditory_score_atm(atm);
        let motor = Self::calculate_motor_score_atm(atm);
        let cognitive = Self::calculate_cognitive_score_atm(atm);
        let wia = Self::calculate_wia_score_atm(atm);

        let overall = (visual * 0.25 + auditory * 0.25 + motor * 0.25 + cognitive * 0.15 + wia * 0.10) as f32;

        let level = Self::determine_certification_level(overall);

        AccessibilityScore {
            overall,
            visual: visual as f32,
            auditory: auditory as f32,
            motor: motor as f32,
            cognitive: cognitive as f32,
            wia_integration: wia as f32,
            certification_level: level,
        }
    }

    fn calculate_visual_score_atm(atm: &ATMAccessibilityProfile) -> f64 {
        let mut score: f64 = 0.0;

        // Audio guidance is essential for blind users
        if atm.audio_accessibility.audio_guidance {
            score += 40.0;
        }

        // Headphone jack for privacy
        if atm.audio_accessibility.headphone_jack.unwrap_or(false) {
            score += 20.0;
        }

        // Braille labels
        if atm.tactile_accessibility.braille_labels {
            score += 15.0;
        }

        // Tactile keypad features
        if atm.tactile_accessibility.key5_marker.unwrap_or(false) {
            score += 10.0;
        }

        // Screen accessibility for low vision
        if let Some(ref screen) = atm.screen_accessibility {
            if screen.high_contrast_mode.unwrap_or(false) {
                score += 10.0;
            }
            if screen.font_size_adjustable.unwrap_or(false) {
                score += 5.0;
            }
        }

        score.min(100.0)
    }

    fn calculate_auditory_score_atm(atm: &ATMAccessibilityProfile) -> f64 {
        let mut score: f64 = 0.0;

        // Visual alternatives are essential for deaf users
        if let Some(ref screen) = atm.screen_accessibility {
            if screen.touchscreen.unwrap_or(false) || screen.physical_buttons.unwrap_or(false) {
                score += 40.0;
            }
        }

        // Privacy mode (audio only through headphones)
        if atm.audio_accessibility.privacy_mode.unwrap_or(false) {
            score += 20.0;
        }

        // Multiple languages support
        if let Some(ref langs) = atm.audio_accessibility.supported_languages {
            if langs.len() > 1 {
                score += 15.0;
            }
        }

        // Tactile feedback as alternative
        if atm.tactile_accessibility.tactile_keypad.unwrap_or(false) {
            score += 15.0;
        }

        // WIA Voice-Sign support
        if let Some(ref wia) = atm.wia_integration {
            if wia.voice_sign_support.unwrap_or(false) {
                score += 10.0;
            }
        }

        score.min(100.0)
    }

    fn calculate_motor_score_atm(atm: &ATMAccessibilityProfile) -> f64 {
        let mut score: f64 = 0.0;

        // Wheelchair accessibility is essential
        if atm.physical_accessibility.wheelchair_accessible {
            score += 35.0;
        }

        // Height adjustability
        if atm.physical_accessibility.height_adjustable.unwrap_or(false) {
            score += 15.0;
        }

        // Clear floor space
        if atm.physical_accessibility.clear_floor_space.unwrap_or(false) {
            score += 10.0;
        }

        // Extended timeout
        if let Some(ref ops) = atm.operational_features {
            if ops.extended_timeout.unwrap_or(false) {
                score += 15.0;
            }

            // Cardless/NFC transactions
            if ops.cardless_transaction.unwrap_or(false) || ops.nfc_enabled.unwrap_or(false) {
                score += 15.0;
            }
        }

        // WIA exoskeleton support
        if let Some(ref wia) = atm.wia_integration {
            if wia.exoskeleton_guidance.unwrap_or(false) {
                score += 10.0;
            }
        }

        score.min(100.0)
    }

    fn calculate_cognitive_score_atm(atm: &ATMAccessibilityProfile) -> f64 {
        let mut score: f64 = 0.0;

        // Simple interface options
        if let Some(ref screen) = atm.screen_accessibility {
            if screen.touchscreen.unwrap_or(false) {
                score += 20.0;
            }
        }

        // Extended timeout for slower processing
        if let Some(ref ops) = atm.operational_features {
            if ops.extended_timeout.unwrap_or(false) {
                score += 25.0;
            }

            // Multiple receipt options
            if let Some(ref receipts) = ops.receipt_options {
                if receipts.len() > 2 {
                    score += 15.0;
                }
            }
        }

        // Audio guidance helps cognitive accessibility
        if atm.audio_accessibility.audio_guidance {
            score += 25.0;
        }

        // Multiple language support
        if let Some(ref langs) = atm.audio_accessibility.supported_languages {
            if langs.len() > 1 {
                score += 15.0;
            }
        }

        score.min(100.0)
    }

    fn calculate_wia_score_atm(atm: &ATMAccessibilityProfile) -> f64 {
        let Some(ref wia) = atm.wia_integration else {
            return 0.0;
        };

        if !wia.enabled {
            return 0.0;
        }

        let mut score: f64 = 20.0; // Base score for having WIA enabled

        if wia.wia_profile_sync.unwrap_or(false) {
            score += 20.0;
        }
        if wia.exoskeleton_guidance.unwrap_or(false) {
            score += 15.0;
        }
        if wia.bionic_eye_display.unwrap_or(false) {
            score += 15.0;
        }
        if wia.voice_sign_support.unwrap_or(false) {
            score += 15.0;
        }
        if wia.smart_wheelchair_positioning.unwrap_or(false) {
            score += 15.0;
        }

        score.min(100.0)
    }

    fn determine_certification_level(score: f32) -> WIACertificationLevel {
        if score >= 95.0 {
            WIACertificationLevel::Platinum
        } else if score >= 85.0 {
            WIACertificationLevel::Gold
        } else if score >= 70.0 {
            WIACertificationLevel::Silver
        } else {
            WIACertificationLevel::Bronze
        }
    }
}

// ============================================================================
// Compatibility Checker
// ============================================================================

/// Checks compatibility between user needs and services/ATMs
pub struct CompatibilityChecker;

impl CompatibilityChecker {
    /// Create a new checker
    pub fn new() -> Self {
        Self
    }

    /// Check compatibility between user profile and ATM
    pub fn check_compatibility(
        &self,
        user: &UserFinancialAccessibilityProfile,
        atm: &ATMAccessibilityProfile,
    ) -> CompatibilityResult {
        let mut issues = Vec::new();
        let mut total_score: f64 = 100.0;

        // Check visual accessibility
        if let Some(ref visual) = user.accessibility_needs.sensory.visual {
            let visual_issues = Self::check_visual_compatibility(visual, atm);
            for issue in &visual_issues {
                total_score -= Self::severity_penalty(&issue.severity);
            }
            issues.extend(visual_issues);
        }

        // Check auditory accessibility
        if let Some(ref auditory) = user.accessibility_needs.sensory.auditory {
            let auditory_issues = Self::check_auditory_compatibility(auditory, atm);
            for issue in &auditory_issues {
                total_score -= Self::severity_penalty(&issue.severity);
            }
            issues.extend(auditory_issues);
        }

        // Check motor accessibility
        if let Some(ref motor) = user.accessibility_needs.motor {
            let motor_issues = Self::check_motor_compatibility(motor, atm);
            for issue in &motor_issues {
                total_score -= Self::severity_penalty(&issue.severity);
            }
            issues.extend(motor_issues);
        }

        // Check cognitive accessibility
        if let Some(ref cognitive) = user.accessibility_needs.cognitive {
            let cognitive_issues = Self::check_cognitive_compatibility(cognitive, atm);
            for issue in &cognitive_issues {
                total_score -= Self::severity_penalty(&issue.severity);
            }
            issues.extend(cognitive_issues);
        }

        let score = total_score.max(0.0) as f32;
        let compatible = score >= 60.0 && !issues.iter().any(|i| i.severity == IssueSeverity::Critical);
        let recommendations = Self::generate_recommendations(&issues, user, atm);

        CompatibilityResult {
            is_compatible: compatible,
            compatibility_score: score,
            issues,
            recommendations,
        }
    }

    fn check_visual_compatibility(
        visual: &VisualAccessibilityNeeds,
        atm: &ATMAccessibilityProfile,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        match visual.level {
            VisualLevel::TotallyBlind | VisualLevel::LegallyBlind => {
                // Must have audio guidance
                if !atm.audio_accessibility.audio_guidance {
                    issues.push(CompatibilityIssue {
                        category: "visual".to_string(),
                        severity: IssueSeverity::Critical,
                        description: "ATM does not have audio guidance".to_string(),
                        user_need: "Requires audio guidance for screen-free operation".to_string(),
                        service_capability: "Audio guidance not available".to_string(),
                    });
                }

                // Should have headphone jack for privacy
                if !atm.audio_accessibility.headphone_jack.unwrap_or(false) {
                    issues.push(CompatibilityIssue {
                        category: "visual".to_string(),
                        severity: IssueSeverity::Major,
                        description: "ATM does not have headphone jack".to_string(),
                        user_need: "Prefers private audio output".to_string(),
                        service_capability: "No headphone jack available".to_string(),
                    });
                }

                // Should have braille
                if !atm.tactile_accessibility.braille_labels {
                    issues.push(CompatibilityIssue {
                        category: "visual".to_string(),
                        severity: IssueSeverity::Minor,
                        description: "ATM does not have Braille labels".to_string(),
                        user_need: "Braille labels preferred".to_string(),
                        service_capability: "No Braille labels".to_string(),
                    });
                }
            }
            VisualLevel::LowVision => {
                // Check for high contrast
                if let Some(ref screen) = atm.screen_accessibility {
                    if !screen.high_contrast_mode.unwrap_or(false) {
                        issues.push(CompatibilityIssue {
                            category: "visual".to_string(),
                            severity: IssueSeverity::Major,
                            description: "ATM does not have high contrast mode".to_string(),
                            user_need: "Requires high contrast display".to_string(),
                            service_capability: "High contrast mode not available".to_string(),
                        });
                    }
                }
            }
            VisualLevel::None => {}
        }

        issues
    }

    fn check_auditory_compatibility(
        auditory: &AuditoryAccessibilityNeeds,
        atm: &ATMAccessibilityProfile,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        match auditory.level {
            AuditoryLevel::Deaf | AuditoryLevel::Profound => {
                // Must have visual display
                if atm.screen_accessibility.is_none() {
                    issues.push(CompatibilityIssue {
                        category: "auditory".to_string(),
                        severity: IssueSeverity::Critical,
                        description: "ATM may not have adequate visual feedback".to_string(),
                        user_need: "Requires visual feedback for all operations".to_string(),
                        service_capability: "Screen accessibility information unavailable".to_string(),
                    });
                }

                // Check for sign language support if preferred
                if auditory.preferred_sign_language.is_some() {
                    if let Some(ref wia) = atm.wia_integration {
                        if !wia.voice_sign_support.unwrap_or(false) {
                            issues.push(CompatibilityIssue {
                                category: "auditory".to_string(),
                                severity: IssueSeverity::Minor,
                                description: "ATM does not support sign language".to_string(),
                                user_need: "Sign language support preferred".to_string(),
                                service_capability: "No WIA Voice-Sign integration".to_string(),
                            });
                        }
                    }
                }
            }
            _ => {}
        }

        issues
    }

    fn check_motor_compatibility(
        motor: &MotorAccessibilityNeeds,
        atm: &ATMAccessibilityProfile,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        // Check wheelchair accessibility
        if let Some(ref devices) = motor.assistive_devices {
            if devices.uses_wheelchair.unwrap_or(false) && !atm.physical_accessibility.wheelchair_accessible {
                issues.push(CompatibilityIssue {
                    category: "motor".to_string(),
                    severity: IssueSeverity::Critical,
                    description: "ATM is not wheelchair accessible".to_string(),
                    user_need: "Requires wheelchair accessible ATM".to_string(),
                    service_capability: "Not wheelchair accessible".to_string(),
                });
            }
        }

        // Check for extended timeouts
        if let Some(ref prefs) = motor.preferences {
            if prefs.extended_timeouts.unwrap_or(false) {
                if let Some(ref ops) = atm.operational_features {
                    if !ops.extended_timeout.unwrap_or(false) {
                        issues.push(CompatibilityIssue {
                            category: "motor".to_string(),
                            severity: IssueSeverity::Major,
                            description: "ATM does not support extended timeouts".to_string(),
                            user_need: "Requires extended timeout for operations".to_string(),
                            service_capability: "Standard timeout only".to_string(),
                        });
                    }
                }
            }
        }

        // Check for voice control
        if let Some(ref prefs) = motor.preferences {
            if prefs.voice_control.unwrap_or(false) {
                // ATMs typically don't have voice control, but WIA integration might help
                if atm.wia_integration.is_none() || !atm.wia_integration.as_ref().unwrap().enabled {
                    issues.push(CompatibilityIssue {
                        category: "motor".to_string(),
                        severity: IssueSeverity::Minor,
                        description: "ATM does not support voice control".to_string(),
                        user_need: "Prefers voice control for input".to_string(),
                        service_capability: "No voice control available".to_string(),
                    });
                }
            }
        }

        issues
    }

    fn check_cognitive_compatibility(
        cognitive: &CognitiveAccessibilityNeeds,
        atm: &ATMAccessibilityProfile,
    ) -> Vec<CompatibilityIssue> {
        let mut issues = Vec::new();

        match cognitive.level {
            CognitiveLevel::Moderate | CognitiveLevel::Significant => {
                // Check for extended timeout
                if let Some(ref ops) = atm.operational_features {
                    if !ops.extended_timeout.unwrap_or(false) {
                        issues.push(CompatibilityIssue {
                            category: "cognitive".to_string(),
                            severity: IssueSeverity::Major,
                            description: "ATM may not allow enough time for operations".to_string(),
                            user_need: "Requires extended time for decisions".to_string(),
                            service_capability: "Standard timeout only".to_string(),
                        });
                    }
                }

                // Check if audio guidance available (helpful for step-by-step)
                if !atm.audio_accessibility.audio_guidance {
                    issues.push(CompatibilityIssue {
                        category: "cognitive".to_string(),
                        severity: IssueSeverity::Minor,
                        description: "ATM does not have audio guidance for step-by-step help".to_string(),
                        user_need: "Benefits from audio guidance".to_string(),
                        service_capability: "No audio guidance".to_string(),
                    });
                }
            }
            _ => {}
        }

        issues
    }

    fn severity_penalty(severity: &IssueSeverity) -> f64 {
        match severity {
            IssueSeverity::Critical => 40.0,
            IssueSeverity::Major => 20.0,
            IssueSeverity::Minor => 10.0,
            IssueSeverity::Informational => 5.0,
        }
    }

    fn generate_recommendations(
        issues: &[CompatibilityIssue],
        user: &UserFinancialAccessibilityProfile,
        atm: &ATMAccessibilityProfile,
    ) -> Vec<String> {
        let mut recommendations = Vec::new();

        // Check if WIA devices could help
        if let Some(ref wia) = user.wia_integration {
            if wia.enabled {
                if let Some(ref atm_wia) = atm.wia_integration {
                    if atm_wia.enabled {
                        recommendations.push(
                            "Connect your WIA device for enhanced accessibility features".to_string()
                        );
                    }
                }
            }
        }

        // Specific recommendations based on issues
        for issue in issues {
            match (&issue.category[..], &issue.severity) {
                ("visual", IssueSeverity::Critical) => {
                    recommendations.push("Bring headphones to use audio guidance privately".to_string());
                }
                ("motor", IssueSeverity::Critical) => {
                    recommendations.push("Search for wheelchair-accessible ATMs nearby".to_string());
                }
                ("cognitive", _) => {
                    recommendations.push("Consider using mobile pre-staging if available".to_string());
                }
                _ => {}
            }
        }

        // General recommendation for better ATMs
        if issues.iter().any(|i| i.severity == IssueSeverity::Critical || i.severity == IssueSeverity::Major) {
            recommendations.push("Consider searching for ATMs with WIA Gold or Platinum certification".to_string());
        }

        recommendations
    }
}

// ============================================================================
// Notification Builder
// ============================================================================

/// Builder for accessible notifications
#[derive(Default)]
pub struct NotificationBuilder {
    notification_type: Option<NotificationType>,
    priority: Option<NotificationPriority>,
    category: Option<NotificationCategory>,
    title: Option<String>,
    body: Option<String>,
    simple_language: Option<String>,
    voice_script: Option<String>,
    ssml: Option<String>,
    braille: Option<String>,
    action_required: bool,
    action_type: Option<ActionType>,
    action_label: Option<String>,
    accessible_instructions: Option<String>,
    exoskeleton_enabled: bool,
    exoskeleton_pattern: Option<String>,
    exoskeleton_intensity: Option<f64>,
    exoskeleton_body_region: Option<String>,
    bionic_eye_enabled: bool,
    bionic_eye_mode: Option<DisplayMode>,
    voice_sign_enabled: bool,
}

impl NotificationBuilder {
    /// Create a new notification builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set notification type
    pub fn notification_type(mut self, t: NotificationType) -> Self {
        self.notification_type = Some(t);
        self
    }

    /// Set priority
    pub fn priority(mut self, p: NotificationPriority) -> Self {
        self.priority = Some(p);
        self
    }

    /// Set category
    pub fn category(mut self, c: NotificationCategory) -> Self {
        self.category = Some(c);
        self
    }

    /// Set title
    pub fn title(mut self, t: &str) -> Self {
        self.title = Some(t.to_string());
        self
    }

    /// Set body
    pub fn body(mut self, b: &str) -> Self {
        self.body = Some(b.to_string());
        self
    }

    /// Set simple language version
    pub fn simple_language(mut self, text: &str) -> Self {
        self.simple_language = Some(text.to_string());
        self
    }

    /// Set voice script
    pub fn voice_script(mut self, script: &str) -> Self {
        self.voice_script = Some(script.to_string());
        self
    }

    /// Set SSML for text-to-speech
    pub fn ssml(mut self, ssml: &str) -> Self {
        self.ssml = Some(ssml.to_string());
        self
    }

    /// Set braille text
    pub fn braille(mut self, braille: &str) -> Self {
        self.braille = Some(braille.to_string());
        self
    }

    /// Set action required
    pub fn action_required(mut self, required: bool) -> Self {
        self.action_required = required;
        self
    }

    /// Set action type
    pub fn action_type(mut self, t: ActionType) -> Self {
        self.action_type = Some(t);
        self
    }

    /// Set action label
    pub fn action_label(mut self, label: &str) -> Self {
        self.action_label = Some(label.to_string());
        self
    }

    /// Set accessible instructions
    pub fn accessible_instructions(mut self, instructions: &str) -> Self {
        self.accessible_instructions = Some(instructions.to_string());
        self
    }

    /// Enable exoskeleton delivery
    pub fn exoskeleton_enabled(mut self, enabled: bool) -> Self {
        self.exoskeleton_enabled = enabled;
        self
    }

    /// Set exoskeleton pattern
    pub fn exoskeleton_pattern(mut self, pattern: &str) -> Self {
        self.exoskeleton_pattern = Some(pattern.to_string());
        self
    }

    /// Set exoskeleton intensity
    pub fn exoskeleton_intensity(mut self, intensity: f64) -> Self {
        self.exoskeleton_intensity = Some(intensity);
        self
    }

    /// Set exoskeleton body region
    pub fn exoskeleton_body_region(mut self, region: &str) -> Self {
        self.exoskeleton_body_region = Some(region.to_string());
        self
    }

    /// Enable bionic eye delivery
    pub fn bionic_eye_enabled(mut self, enabled: bool) -> Self {
        self.bionic_eye_enabled = enabled;
        self
    }

    /// Set bionic eye display mode
    pub fn bionic_eye_mode(mut self, mode: DisplayMode) -> Self {
        self.bionic_eye_mode = Some(mode);
        self
    }

    /// Enable voice sign delivery
    pub fn voice_sign_enabled(mut self, enabled: bool) -> Self {
        self.voice_sign_enabled = enabled;
        self
    }

    /// Build the notification
    pub fn build(self) -> Result<AccessibleNotification, String> {
        let notification_type = self.notification_type.ok_or("notification_type is required")?;
        let priority = self.priority.ok_or("priority is required")?;
        let title = self.title.ok_or("title is required")?;
        let body = self.body.clone().unwrap_or_default();
        let simple_language = self.simple_language.unwrap_or_else(|| body.clone());
        let voice_script = self.voice_script.unwrap_or_else(|| body.clone());

        let action = if self.action_type.is_some() || self.action_required {
            Some(NotificationAction {
                required: self.action_required,
                action_type: self.action_type.map(|t| format!("{:?}", t).to_lowercase()).unwrap_or_default(),
                action_label: self.action_label.unwrap_or_default(),
                accessible_instructions: self.accessible_instructions.unwrap_or_default(),
                deadline: None,
                url: None,
            })
        } else {
            None
        };

        let wia_delivery = if self.exoskeleton_enabled || self.bionic_eye_enabled || self.voice_sign_enabled {
            Some(WIADeliveryConfig {
                exoskeleton: if self.exoskeleton_enabled {
                    Some(ExoskeletonDelivery {
                        enabled: true,
                        pattern: self.exoskeleton_pattern,
                        body_region: self.exoskeleton_body_region,
                        intensity: self.exoskeleton_intensity.map(|i| i.min(255.0) as u8),
                    })
                } else {
                    None
                },
                bionic_eye: if self.bionic_eye_enabled {
                    Some(BionicEyeDelivery {
                        enabled: true,
                        display_mode: self.bionic_eye_mode.map(|m| format!("{:?}", m).to_lowercase()),
                        position: None,
                        duration: None,
                    })
                } else {
                    None
                },
                voice_sign: if self.voice_sign_enabled {
                    Some(VoiceSignDelivery {
                        enabled: true,
                        sign_language: None,
                        avatar_enabled: Some(true),
                    })
                } else {
                    None
                },
            })
        } else {
            None
        };

        Ok(AccessibleNotification {
            notification_id: uuid::Uuid::new_v4().to_string(),
            timestamp: chrono::Utc::now(),
            expires_at: None,
            notification_type,
            priority,
            category: self.category,
            content: NotificationContent {
                title,
                body: body.clone(),
                formats: ContentFormats {
                    plain_text: body,
                    rich_text: None,
                    simple_language,
                    voice_script,
                    ssml: self.ssml,
                    braille: self.braille,
                },
                localizations: None,
                sign_language: None,
            },
            action,
            delivery: None,
            accessibility: None,
            wia_delivery,
            tracking: None,
        })
    }
}

// ============================================================================
// Profile Builder
// ============================================================================

/// Builder for user financial accessibility profiles
#[derive(Default)]
pub struct UserProfileBuilder {
    profile_id: String,
    wia_id: Option<String>,
    preferred_name: Option<String>,
    preferred_language: Option<String>,
    region: Option<String>,
    timezone: Option<String>,
    visual_level: Option<VisualLevel>,
    color_vision: Option<ColorVision>,
    light_sensitivity: Option<LightSensitivity>,
    font_size: Option<FontSize>,
    high_contrast: bool,
    dark_mode: bool,
    reduce_motion: bool,
    screen_magnification: Option<f64>,
    uses_screen_reader: bool,
    screen_reader_type: Option<ScreenReaderType>,
    uses_braille_display: bool,
    auditory_level: Option<AuditoryLevel>,
    motor_level: Option<MotorLevel>,
    uses_wheelchair: bool,
    large_touch_targets: bool,
    extended_timeouts: bool,
    cognitive_level: Option<CognitiveLevel>,
    simplified_interface: bool,
    step_by_step_guidance: bool,
    auth_methods: Vec<AuthenticationMethod>,
    wia_devices: Vec<(String, WIADeviceType)>,
}

impl UserProfileBuilder {
    /// Create a new profile builder
    pub fn new(profile_id: &str) -> Self {
        Self {
            profile_id: profile_id.to_string(),
            ..Default::default()
        }
    }

    /// Set WIA ID
    pub fn wia_id(mut self, id: &str) -> Self {
        self.wia_id = Some(id.to_string());
        self
    }

    /// Set preferred name
    pub fn preferred_name(mut self, name: &str) -> Self {
        self.preferred_name = Some(name.to_string());
        self
    }

    /// Set preferred language
    pub fn preferred_language(mut self, lang: &str) -> Self {
        self.preferred_language = Some(lang.to_string());
        self
    }

    /// Set region
    pub fn region(mut self, region: &str) -> Self {
        self.region = Some(region.to_string());
        self
    }

    /// Set timezone
    pub fn timezone(mut self, tz: &str) -> Self {
        self.timezone = Some(tz.to_string());
        self
    }

    /// Set visual accessibility level
    pub fn visual_level(mut self, level: VisualLevel) -> Self {
        self.visual_level = Some(level);
        self
    }

    /// Set color vision type
    pub fn color_vision(mut self, cv: ColorVision) -> Self {
        self.color_vision = Some(cv);
        self
    }

    /// Set light sensitivity
    pub fn light_sensitivity(mut self, ls: LightSensitivity) -> Self {
        self.light_sensitivity = Some(ls);
        self
    }

    /// Set font size preference
    pub fn font_size(mut self, size: FontSize) -> Self {
        self.font_size = Some(size);
        self
    }

    /// Enable high contrast
    pub fn high_contrast(mut self, enabled: bool) -> Self {
        self.high_contrast = enabled;
        self
    }

    /// Enable dark mode
    pub fn dark_mode(mut self, enabled: bool) -> Self {
        self.dark_mode = enabled;
        self
    }

    /// Enable reduce motion
    pub fn reduce_motion(mut self, enabled: bool) -> Self {
        self.reduce_motion = enabled;
        self
    }

    /// Set screen magnification level
    pub fn screen_magnification(mut self, level: f64) -> Self {
        self.screen_magnification = Some(level);
        self
    }

    /// Enable screen reader
    pub fn uses_screen_reader(mut self, uses: bool) -> Self {
        self.uses_screen_reader = uses;
        self
    }

    /// Set screen reader type
    pub fn screen_reader_type(mut self, t: ScreenReaderType) -> Self {
        self.screen_reader_type = Some(t);
        self
    }

    /// Enable braille display
    pub fn uses_braille_display(mut self, uses: bool) -> Self {
        self.uses_braille_display = uses;
        self
    }

    /// Set auditory accessibility level
    pub fn auditory_level(mut self, level: AuditoryLevel) -> Self {
        self.auditory_level = Some(level);
        self
    }

    /// Set motor accessibility level
    pub fn motor_level(mut self, level: MotorLevel) -> Self {
        self.motor_level = Some(level);
        self
    }

    /// Enable wheelchair usage
    pub fn uses_wheelchair(mut self, uses: bool) -> Self {
        self.uses_wheelchair = uses;
        self
    }

    /// Enable large touch targets
    pub fn large_touch_targets(mut self, enabled: bool) -> Self {
        self.large_touch_targets = enabled;
        self
    }

    /// Enable extended timeouts
    pub fn extended_timeouts(mut self, enabled: bool) -> Self {
        self.extended_timeouts = enabled;
        self
    }

    /// Set cognitive accessibility level
    pub fn cognitive_level(mut self, level: CognitiveLevel) -> Self {
        self.cognitive_level = Some(level);
        self
    }

    /// Enable simplified interface
    pub fn simplified_interface(mut self, enabled: bool) -> Self {
        self.simplified_interface = enabled;
        self
    }

    /// Enable step by step guidance
    pub fn step_by_step_guidance(mut self, enabled: bool) -> Self {
        self.step_by_step_guidance = enabled;
        self
    }

    /// Add authentication method
    pub fn auth_method(mut self, method: AuthenticationMethod) -> Self {
        self.auth_methods.push(method);
        self
    }

    /// Add WIA device
    pub fn wia_device(mut self, device_id: &str, device_type: WIADeviceType) -> Self {
        self.wia_devices.push((device_id.to_string(), device_type));
        self
    }

    /// Build the profile
    pub fn build(self) -> Result<UserFinancialAccessibilityProfile, String> {
        let preferred_language = self.preferred_language.ok_or("preferred_language is required")?;
        let region = self.region.ok_or("region is required")?;

        let visual = if self.visual_level.is_some() || self.uses_screen_reader || self.uses_braille_display {
            Some(VisualAccessibilityNeeds {
                level: self.visual_level.unwrap_or(VisualLevel::None),
                color_vision: self.color_vision,
                light_sensitivity: self.light_sensitivity,
                preferences: Some(VisualPreferences {
                    font_size: self.font_size,
                    high_contrast: Some(self.high_contrast),
                    dark_mode: Some(self.dark_mode),
                    reduce_motion: Some(self.reduce_motion),
                    screen_magnification: self.screen_magnification,
                }),
                assistive_tech: Some(VisualAssistiveTech {
                    screen_reader: Some(self.uses_screen_reader),
                    screen_reader_type: self.screen_reader_type,
                    braille_display: Some(self.uses_braille_display),
                    magnifier: Some(self.screen_magnification.is_some()),
                }),
            })
        } else {
            None
        };

        let auditory = self.auditory_level.map(|level| AuditoryAccessibilityNeeds {
            level,
            uses_hearing_aid: None,
            uses_cochlear_implant: None,
            preferred_sign_language: None,
            preferences: None,
        });

        let motor = if self.motor_level.is_some() || self.uses_wheelchair {
            Some(MotorAccessibilityNeeds {
                level: self.motor_level.unwrap_or(MotorLevel::None),
                upper_limb: None,
                fine_motor: None,
                preferences: Some(MotorPreferences {
                    large_targets: Some(self.large_touch_targets),
                    extended_timeouts: Some(self.extended_timeouts),
                    voice_control: None,
                    switch_access: None,
                    eye_tracking: None,
                    dwell_click: None,
                    dwell_time: None,
                }),
                assistive_devices: Some(MotorAssistiveDevices {
                    uses_wheelchair: Some(self.uses_wheelchair),
                    uses_exoskeleton: None,
                }),
            })
        } else {
            None
        };

        let cognitive = if self.cognitive_level.is_some() || self.simplified_interface || self.step_by_step_guidance {
            Some(CognitiveAccessibilityNeeds {
                level: self.cognitive_level.unwrap_or(CognitiveLevel::None),
                preferences: Some(CognitivePreferences {
                    simplified_interface: Some(self.simplified_interface),
                    clear_labels: Some(true),
                    confirm_before_actions: Some(true),
                    step_by_step_guidance: Some(self.step_by_step_guidance),
                    memory_aids: None,
                    time_extensions: Some(self.extended_timeouts),
                }),
                reading_level: None,
                support_needs: None,
            })
        } else {
            None
        };

        let wia_integration = if !self.wia_devices.is_empty() {
            Some(WIAIntegration {
                enabled: true,
                connected_devices: Some(
                    self.wia_devices
                        .iter()
                        .map(|(id, dt)| WIADeviceConnection {
                            device_id: id.clone(),
                            device_type: dt.clone(),
                            connection_status: Some("connected".to_string()),
                            last_seen: Some(chrono::Utc::now()),
                        })
                        .collect(),
                ),
                preferred_output_device: self.wia_devices.first().map(|(id, _)| id.clone()),
                cross_device_sync: Some(true),
            })
        } else {
            None
        };

        Ok(UserFinancialAccessibilityProfile {
            profile_id: self.profile_id,
            wia_id: self.wia_id,
            version: "1.0.0".to_string(),
            created_at: Some(chrono::Utc::now()),
            updated_at: Some(chrono::Utc::now()),
            personal_info: PersonalInfo {
                preferred_name: self.preferred_name,
                preferred_language,
                region,
                timezone: self.timezone,
            },
            accessibility_needs: AccessibilityNeeds {
                sensory: SensoryAccessibilityNeeds { visual, auditory },
                motor,
                cognitive,
            },
            financial_preferences: FinancialPreferences {
                preferred_auth_method: if self.auth_methods.is_empty() {
                    vec![AuthenticationMethod::Pin]
                } else {
                    self.auth_methods
                },
                transaction_confirmation: None,
                notification_preferences: None,
                security_preferences: None,
            },
            atm_preferences: None,
            card_preferences: None,
            wia_integration,
            emergency_settings: None,
        })
    }
}
