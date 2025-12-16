//! Simulator adapter for testing and development

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::core::ProfileManager;
use crate::error::{MedicalError, Result};
use crate::types::*;

/// In-memory simulator for testing
pub struct SimulatorAdapter {
    device_profiles: Arc<RwLock<HashMap<String, MedicalDeviceAccessibilityProfile>>>,
    user_profiles: Arc<RwLock<HashMap<String, UserMedicalAccessibilityProfile>>>,
    alarm_systems: Arc<RwLock<HashMap<String, MedicalAlarmSystem>>>,
}

impl SimulatorAdapter {
    /// Create a new simulator adapter
    pub fn new() -> Self {
        Self {
            device_profiles: Arc::new(RwLock::new(HashMap::new())),
            user_profiles: Arc::new(RwLock::new(HashMap::new())),
            alarm_systems: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create a simulator with sample data
    pub async fn with_sample_data() -> Self {
        let adapter = Self::new();

        // Add sample CGM device profile
        let cgm_profile = Self::create_sample_cgm_profile();
        adapter.save_device_profile(&cgm_profile).await.unwrap();

        // Add sample blood pressure monitor profile
        let bp_profile = Self::create_sample_bp_monitor_profile();
        adapter.save_device_profile(&bp_profile).await.unwrap();

        // Add sample user profiles
        let blind_user = Self::create_sample_blind_user_profile();
        adapter.save_user_profile(&blind_user).await.unwrap();

        let deaf_user = Self::create_sample_deaf_user_profile();
        adapter.save_user_profile(&deaf_user).await.unwrap();

        adapter
    }

    fn create_sample_cgm_profile() -> MedicalDeviceAccessibilityProfile {
        MedicalDeviceAccessibilityProfile {
            profile_id: "cgm_dexcom_g7".to_string(),
            profile_version: "1.0.0".to_string(),
            created_at: chrono::Utc::now(),
            updated_at: chrono::Utc::now(),
            device: MedicalDeviceInfo {
                device_id: "dexcom-g7-001".to_string(),
                manufacturer: "Dexcom".to_string(),
                model: "G7".to_string(),
                device_name: "Dexcom G7 CGM System".to_string(),
                fda_classification: Some(FDAClassification {
                    class: FDAClass::ClassII,
                    product_code: Some("NBW".to_string()),
                    regulation_number: None,
                    clearance_type: Some(ClearanceType::K510),
                }),
                device_type: MedicalDeviceType::Monitoring,
                device_category: DeviceCategory::Cgm,
                use_environment: UseEnvironment::Home,
                intended_users: Some(vec![
                    IntendedUser {
                        user_type: IntendedUserType::Patient,
                        training_required: true,
                        supervision_required: false,
                    },
                    IntendedUser {
                        user_type: IntendedUserType::Caregiver,
                        training_required: true,
                        supervision_required: false,
                    },
                ]),
                interface: None,
                connectivity: Some(DeviceConnectivity {
                    wireless: Some(WirelessConnectivity {
                        bluetooth: Some(BluetoothConfig {
                            version: "5.0".to_string(),
                            profiles: vec!["GATT".to_string()],
                        }),
                        wifi: None,
                        cellular: None,
                        nfc: None,
                    }),
                    wired: None,
                    companion_app: Some(CompanionApp {
                        platforms: vec![Platform::Ios, Platform::Android],
                        accessibility_features: vec![
                            "VoiceOver support".to_string(),
                            "TalkBack support".to_string(),
                            "Dynamic text sizing".to_string(),
                        ],
                    }),
                    cloud_integration: Some(CloudIntegration {
                        provider: "Dexcom Clarity".to_string(),
                        data_sync: true,
                        remote_monitoring: true,
                    }),
                }),
            },
            accessibility: DeviceAccessibilityFeatures {
                visual: VisualAccessibility {
                    screen_reader: Some(ScreenReaderSupport {
                        supported: true,
                        level: SupportLevel::Full,
                        protocols: vec![
                            ScreenReaderProtocol::Voiceover,
                            ScreenReaderProtocol::Talkback,
                        ],
                    }),
                    voice_output: Some(VoiceOutput {
                        supported: true,
                        readings: vec![VoiceReading {
                            data_type: "blood_glucose".to_string(),
                            format: "{value} milligrams per deciliter".to_string(),
                            units_spoken: true,
                            range_indication: true,
                        }],
                        languages: vec!["en".to_string(), "es".to_string()],
                        speed_adjustable: true,
                        volume_adjustable: true,
                    }),
                    display_accessibility: Some(DisplayAccessibility {
                        high_contrast_mode: true,
                        large_text_mode: true,
                        text_size_adjustable: true,
                        color_inversion: false,
                        color_blind_modes: vec![
                            ColorBlindMode::Protanopia,
                            ColorBlindMode::Deuteranopia,
                        ],
                    }),
                    non_visual_alternatives: Some(NonVisualAlternatives {
                        audio_feedback: true,
                        haptic_feedback: true,
                        braille_output: false,
                    }),
                },
                auditory: AuditoryAccessibility {
                    visual_alerts: Some(VisualAlerts {
                        supported: true,
                        types: vec![
                            VisualAlertType::ScreenFlash,
                            VisualAlertType::IconDisplay,
                            VisualAlertType::ColorChange,
                        ],
                        customizable: true,
                    }),
                    haptic_alerts: Some(HapticAlerts {
                        supported: true,
                        patterns: vec![
                            HapticPattern {
                                id: "urgent_low".to_string(),
                                name: "Urgent Low Alert".to_string(),
                                meaning: "Blood glucose critically low".to_string(),
                                duration_ms: 1000,
                                intensity: 1.0,
                            },
                            HapticPattern {
                                id: "high".to_string(),
                                name: "High Alert".to_string(),
                                meaning: "Blood glucose high".to_string(),
                                duration_ms: 500,
                                intensity: 0.7,
                            },
                        ],
                        intensity_levels: 5,
                    }),
                    audio_adjustments: Some(AudioAdjustments {
                        volume_range: (0.0, 100.0),
                        frequency_adjustable: false,
                        mono_audio: true,
                        hearing_aid_compatible: true,
                        t_coil_compatible: false,
                    }),
                    text_alternatives: Some(TextAlternatives {
                        on_screen_text: true,
                        closed_captions: false,
                        real_time_transcription: false,
                    }),
                },
                motor: MotorAccessibility {
                    alternative_input: Some(AlternativeInput {
                        voice_control: true,
                        switch_access: false,
                        eye_tracking: false,
                        head_tracking: false,
                        wia_exoskeleton: true,
                    }),
                    physical_controls: None,
                    touchscreen: Some(TouchscreenAccessibility {
                        gesture_alternatives: true,
                        touch_accommodation: true,
                        dwell_control: true,
                        haptic_feedback: true,
                    }),
                    automation: Some(AutomationFeatures {
                        auto_measurement: true,
                        scheduled_operation: true,
                        remote_control: true,
                    }),
                },
                cognitive: CognitiveAccessibility {
                    simplified_interface: Some(SimplifiedInterface {
                        available: true,
                        reduced_options: true,
                        step_by_step_guidance: true,
                        clear_icons: true,
                    }),
                    memory_support: Some(MemorySupport {
                        reminders: true,
                        history_log: true,
                        caregiver_notifications: true,
                        auto_data_sync: true,
                    }),
                    error_prevention: Some(ErrorPrevention {
                        confirmation_prompts: true,
                        undo_capability: false,
                        clear_error_messages: true,
                        recovery_guidance: true,
                    }),
                    language_support: Some(LanguageSupport {
                        languages: vec!["en".to_string(), "es".to_string(), "fr".to_string()],
                        simple_language_mode: true,
                        icon_based_navigation: true,
                        pictogram_support: false,
                    }),
                },
                physical: None,
                accessibility_score: Some(AccessibilityScore {
                    overall: 85.0,
                    visual: 90.0,
                    auditory: 85.0,
                    motor: 80.0,
                    cognitive: 85.0,
                }),
            },
            compliance: RegulatoryCompliance {
                fda: Some(FDACompliance {
                    registered: true,
                    clearance_number: Some("K123456".to_string()),
                    human_factors_validated: true,
                    accessibility_tested: true,
                }),
                mde_standards: None,
                ada: Some(ADACompliance {
                    title_ii_compliant: true,
                    title_iii_compliant: true,
                }),
                international: Some(InternationalCompliance {
                    iec_62366: true,
                    iso_14971: true,
                    en_301_549: false,
                    mdr_compliant: true,
                }),
                wia_certification: Some(WIACertification {
                    level: CertificationLevel::Gold,
                    certificate_id: "WIA-MED-2024-001".to_string(),
                    valid_until: "2026-12-31".to_string(),
                }),
                accessibility_conformance: Some(AccessibilityConformance {
                    vpat_available: true,
                    vpat_url: Some("https://dexcom.com/vpat".to_string()),
                    wcag_level: Some(WCAGLevel::AA),
                    section_508_compliant: true,
                }),
            },
            wia_integration: Some(WIAIntegration {
                supported_protocols: WIASupportedProtocols {
                    exoskeleton: true,
                    bionic_eye: false,
                    voice_sign: true,
                    haptic: true,
                    smart_wheelchair: false,
                },
                exoskeleton: Some(ExoskeletonIntegration {
                    haptic_feedback_mapping: vec![HapticMapping {
                        device_event: "urgent_low_glucose".to_string(),
                        haptic_pattern: "urgent_pulse".to_string(),
                        target: HapticTarget::LeftHand,
                        intensity: 1.0,
                    }],
                    motion_assistance: false,
                    rehabilitation_mode: false,
                }),
                bionic_eye: None,
                voice_sign: Some(VoiceSignIntegration {
                    medical_terminology_support: true,
                    real_time_translation: true,
                    emergency_phrases: vec![
                        "Blood sugar is dangerously low".to_string(),
                        "Need sugar immediately".to_string(),
                    ],
                }),
                haptic: Some(HapticIntegration {
                    alarm_mapping: vec![HapticAlarmMapping {
                        alarm_priority: "critical".to_string(),
                        haptic_pattern: "urgent_continuous".to_string(),
                        body_location: "wrist".to_string(),
                    }],
                    data_haptic_encoding: true,
                }),
                smart_wheelchair: None,
            }),
            metadata: ProfileMetadata {
                profile_type: ProfileType::Device,
                schema_version: "1.0.0".to_string(),
                language: Some("en".to_string()),
                created_by: Some("WIA Medical Team".to_string()),
                validation: Some(ValidationInfo {
                    validated: true,
                    validator: Some("WIA Certification".to_string()),
                    validation_date: Some("2024-12-01".to_string()),
                }),
                tags: Some(vec![
                    "cgm".to_string(),
                    "diabetes".to_string(),
                    "wearable".to_string(),
                ]),
                notes: Some("Reference implementation for CGM accessibility".to_string()),
            },
        }
    }

    fn create_sample_bp_monitor_profile() -> MedicalDeviceAccessibilityProfile {
        MedicalDeviceAccessibilityProfile {
            profile_id: "bp_talking_monitor".to_string(),
            profile_version: "1.0.0".to_string(),
            created_at: chrono::Utc::now(),
            updated_at: chrono::Utc::now(),
            device: MedicalDeviceInfo {
                device_id: "bp-monitor-001".to_string(),
                manufacturer: "WIA Medical".to_string(),
                model: "TalkingBP Pro".to_string(),
                device_name: "WIA Talking Blood Pressure Monitor".to_string(),
                fda_classification: Some(FDAClassification {
                    class: FDAClass::ClassII,
                    product_code: Some("DXN".to_string()),
                    regulation_number: None,
                    clearance_type: Some(ClearanceType::K510),
                }),
                device_type: MedicalDeviceType::Monitoring,
                device_category: DeviceCategory::BloodPressureMonitor,
                use_environment: UseEnvironment::Home,
                intended_users: Some(vec![IntendedUser {
                    user_type: IntendedUserType::Patient,
                    training_required: false,
                    supervision_required: false,
                }]),
                interface: Some(DeviceInterface {
                    display: Some(DisplayConfig {
                        display_type: DisplayType::Lcd,
                        size_inches: Some(3.5),
                        resolution: Some(Resolution {
                            width: 320,
                            height: 240,
                        }),
                        touch_enabled: false,
                        color: true,
                        brightness_adjustable: true,
                        contrast_adjustable: true,
                    }),
                    physical_controls: Some(PhysicalControls {
                        buttons: Some(vec![
                            ButtonInfo {
                                id: "start".to_string(),
                                label: "START/STOP".to_string(),
                                tactile_marking: true,
                                size_mm: 20.0,
                                force_required_grams: 100.0,
                                location: "Front center".to_string(),
                            },
                            ButtonInfo {
                                id: "memory".to_string(),
                                label: "MEMORY".to_string(),
                                tactile_marking: true,
                                size_mm: 15.0,
                                force_required_grams: 100.0,
                                location: "Front left".to_string(),
                            },
                        ]),
                        dials: None,
                        switches: None,
                    }),
                    audio: Some(AudioCapabilities {
                        speaker: true,
                        speaker_volume_adjustable: true,
                        microphone: false,
                        audio_jack: true,
                        bluetooth_audio: false,
                    }),
                    haptic: Some(HapticCapabilities {
                        vibration: true,
                        intensity_levels: 3,
                        patterns_supported: true,
                    }),
                }),
                connectivity: Some(DeviceConnectivity {
                    wireless: Some(WirelessConnectivity {
                        bluetooth: Some(BluetoothConfig {
                            version: "4.2".to_string(),
                            profiles: vec!["SPP".to_string()],
                        }),
                        wifi: None,
                        cellular: None,
                        nfc: None,
                    }),
                    wired: Some(WiredConnectivity {
                        usb: Some(UsbConfig {
                            usb_type: "Micro-B".to_string(),
                            version: "2.0".to_string(),
                        }),
                        audio_jack: Some(true),
                        proprietary: None,
                    }),
                    companion_app: None,
                    cloud_integration: None,
                }),
            },
            accessibility: DeviceAccessibilityFeatures {
                visual: VisualAccessibility {
                    screen_reader: None,
                    voice_output: Some(VoiceOutput {
                        supported: true,
                        readings: vec![VoiceReading {
                            data_type: "blood_pressure".to_string(),
                            format: "Systolic {systolic}, Diastolic {diastolic}, Pulse {pulse}".to_string(),
                            units_spoken: true,
                            range_indication: true,
                        }],
                        languages: vec!["en".to_string(), "ko".to_string()],
                        speed_adjustable: true,
                        volume_adjustable: true,
                    }),
                    display_accessibility: Some(DisplayAccessibility {
                        high_contrast_mode: true,
                        large_text_mode: true,
                        text_size_adjustable: false,
                        color_inversion: false,
                        color_blind_modes: vec![],
                    }),
                    non_visual_alternatives: Some(NonVisualAlternatives {
                        audio_feedback: true,
                        haptic_feedback: true,
                        braille_output: false,
                    }),
                },
                auditory: AuditoryAccessibility {
                    visual_alerts: Some(VisualAlerts {
                        supported: true,
                        types: vec![VisualAlertType::LedIndicator, VisualAlertType::IconDisplay],
                        customizable: false,
                    }),
                    haptic_alerts: Some(HapticAlerts {
                        supported: true,
                        patterns: vec![HapticPattern {
                            id: "measurement_complete".to_string(),
                            name: "Measurement Complete".to_string(),
                            meaning: "Blood pressure reading is ready".to_string(),
                            duration_ms: 300,
                            intensity: 0.5,
                        }],
                        intensity_levels: 3,
                    }),
                    audio_adjustments: Some(AudioAdjustments {
                        volume_range: (20.0, 100.0),
                        frequency_adjustable: false,
                        mono_audio: true,
                        hearing_aid_compatible: true,
                        t_coil_compatible: true,
                    }),
                    text_alternatives: Some(TextAlternatives {
                        on_screen_text: true,
                        closed_captions: false,
                        real_time_transcription: false,
                    }),
                },
                motor: MotorAccessibility {
                    alternative_input: Some(AlternativeInput {
                        voice_control: false,
                        switch_access: false,
                        eye_tracking: false,
                        head_tracking: false,
                        wia_exoskeleton: false,
                    }),
                    physical_controls: Some(PhysicalControlsAccessibility {
                        large_buttons: true,
                        button_spacing_adequate: true,
                        low_force_buttons: true,
                        one_handed_operation: true,
                        no_fine_motor_required: true,
                    }),
                    touchscreen: None,
                    automation: Some(AutomationFeatures {
                        auto_measurement: false,
                        scheduled_operation: false,
                        remote_control: false,
                    }),
                },
                cognitive: CognitiveAccessibility {
                    simplified_interface: Some(SimplifiedInterface {
                        available: true,
                        reduced_options: true,
                        step_by_step_guidance: false,
                        clear_icons: true,
                    }),
                    memory_support: Some(MemorySupport {
                        reminders: false,
                        history_log: true,
                        caregiver_notifications: false,
                        auto_data_sync: false,
                    }),
                    error_prevention: Some(ErrorPrevention {
                        confirmation_prompts: false,
                        undo_capability: false,
                        clear_error_messages: true,
                        recovery_guidance: true,
                    }),
                    language_support: Some(LanguageSupport {
                        languages: vec!["en".to_string(), "ko".to_string()],
                        simple_language_mode: true,
                        icon_based_navigation: false,
                        pictogram_support: false,
                    }),
                },
                physical: None,
                accessibility_score: Some(AccessibilityScore {
                    overall: 72.0,
                    visual: 75.0,
                    auditory: 80.0,
                    motor: 65.0,
                    cognitive: 68.0,
                }),
            },
            compliance: RegulatoryCompliance {
                fda: Some(FDACompliance {
                    registered: true,
                    clearance_number: Some("K234567".to_string()),
                    human_factors_validated: true,
                    accessibility_tested: true,
                }),
                mde_standards: None,
                ada: Some(ADACompliance {
                    title_ii_compliant: true,
                    title_iii_compliant: true,
                }),
                international: Some(InternationalCompliance {
                    iec_62366: true,
                    iso_14971: true,
                    en_301_549: false,
                    mdr_compliant: false,
                }),
                wia_certification: Some(WIACertification {
                    level: CertificationLevel::Silver,
                    certificate_id: "WIA-MED-2024-002".to_string(),
                    valid_until: "2026-06-30".to_string(),
                }),
                accessibility_conformance: None,
            },
            wia_integration: None,
            metadata: ProfileMetadata {
                profile_type: ProfileType::Device,
                schema_version: "1.0.0".to_string(),
                language: Some("en".to_string()),
                created_by: Some("WIA Medical Team".to_string()),
                validation: None,
                tags: Some(vec![
                    "blood_pressure".to_string(),
                    "talking".to_string(),
                    "home_use".to_string(),
                ]),
                notes: None,
            },
        }
    }

    fn create_sample_blind_user_profile() -> UserMedicalAccessibilityProfile {
        UserMedicalAccessibilityProfile {
            user_id: "user_blind_001".to_string(),
            profile_version: "1.0.0".to_string(),
            accessibility_needs: UserAccessibilityNeeds {
                sensory: Some(SensoryNeeds {
                    visual: Some(VisualNeeds {
                        level: VisualLevel::TotallyBlind,
                        color_blind: None,
                        light_sensitivity: None,
                        field_of_vision: None,
                    }),
                    auditory: None,
                }),
                motor: None,
                cognitive: None,
                medical_conditions: Some(vec![MedicalCondition {
                    condition: "Type 1 Diabetes".to_string(),
                    relevance_to_device_use: "Requires CGM for glucose monitoring".to_string(),
                }]),
            },
            sensory_preferences: UserSensoryPreferences {
                visual: None,
                auditory: Some(AuditoryPreferences {
                    volume_level: 80,
                    prefer_voice: true,
                    voice_speed: 1.2,
                    voice_pitch: VoicePitch::Medium,
                    prefer_tones: false,
                    tone_frequency_preference: None,
                }),
                haptic: Some(HapticPreferences {
                    enabled: true,
                    intensity: 70,
                    prefer_haptic_over_audio: false,
                    wia_haptic_device: None,
                }),
            },
            input_preferences: UserInputPreferences {
                primary_input: InputMethod::Voice,
                fallback_inputs: Some(vec![InputMethod::Touch]),
                touch_settings: Some(TouchSettings {
                    touch_duration_ms: 500,
                    ignore_repeated_touches: true,
                    touch_accommodation: true,
                }),
                voice_settings: Some(VoiceSettings {
                    voice_language: "en-US".to_string(),
                    wake_word: Some("Hey Medical".to_string()),
                    confirmation_required: true,
                }),
                switch_settings: None,
                dwell_settings: None,
            },
            cognitive_support: None,
            medical_context: Some(MedicalContext {
                conditions_monitored: Some(vec!["blood_glucose".to_string()]),
                devices_used: Some(vec![DeviceUsage {
                    device_id: "dexcom-g7-001".to_string(),
                    device_type: "CGM".to_string(),
                    usage_frequency: UsageFrequency::Continuous,
                }]),
                healthcare_provider: Some(HealthcareProviderInfo {
                    data_sharing_enabled: true,
                    provider_id: Some("provider_001".to_string()),
                }),
            }),
            wia_devices: None,
        }
    }

    fn create_sample_deaf_user_profile() -> UserMedicalAccessibilityProfile {
        UserMedicalAccessibilityProfile {
            user_id: "user_deaf_001".to_string(),
            profile_version: "1.0.0".to_string(),
            accessibility_needs: UserAccessibilityNeeds {
                sensory: Some(SensoryNeeds {
                    visual: None,
                    auditory: Some(AuditoryNeeds {
                        level: AuditoryLevel::Deaf,
                        uses_hearing_aid: Some(false),
                        uses_cochlear_implant: Some(false),
                        frequency_range_affected: Some(FrequencyRange::All),
                    }),
                }),
                motor: None,
                cognitive: None,
                medical_conditions: Some(vec![MedicalCondition {
                    condition: "Hypertension".to_string(),
                    relevance_to_device_use: "Requires blood pressure monitoring".to_string(),
                }]),
            },
            sensory_preferences: UserSensoryPreferences {
                visual: Some(VisualPreferences {
                    text_size: TextSize::Large,
                    high_contrast: true,
                    dark_mode: false,
                    color_scheme: None,
                    reduce_motion: false,
                    reduce_transparency: None,
                }),
                auditory: None,
                haptic: Some(HapticPreferences {
                    enabled: true,
                    intensity: 90,
                    prefer_haptic_over_audio: true,
                    wia_haptic_device: Some("wia_exo_v1".to_string()),
                }),
            },
            input_preferences: UserInputPreferences {
                primary_input: InputMethod::Touch,
                fallback_inputs: Some(vec![InputMethod::Buttons]),
                touch_settings: None,
                voice_settings: None,
                switch_settings: None,
                dwell_settings: None,
            },
            cognitive_support: None,
            medical_context: Some(MedicalContext {
                conditions_monitored: Some(vec!["blood_pressure".to_string()]),
                devices_used: Some(vec![DeviceUsage {
                    device_id: "bp-monitor-001".to_string(),
                    device_type: "Blood Pressure Monitor".to_string(),
                    usage_frequency: UsageFrequency::Daily,
                }]),
                healthcare_provider: None,
            }),
            wia_devices: Some(WIADeviceSettings {
                exoskeleton: Some(ExoskeletonSettings {
                    device_id: "wia_exo_v1".to_string(),
                    enabled: true,
                    haptic_intensity: 80,
                    assistance_level: 0,
                }),
                bionic_eye: None,
                voice_sign: Some(VoiceSignSettings {
                    enabled: true,
                    sign_language: SignLanguage::Asl,
                    medical_terms_mode: true,
                }),
                smart_wheelchair: None,
            }),
        }
    }

    /// Get alarm system configuration
    pub async fn get_alarm_system(&self, system_id: &str) -> Result<MedicalAlarmSystem> {
        let alarms = self.alarm_systems.read().await;
        alarms
            .get(system_id)
            .cloned()
            .ok_or_else(|| MedicalError::AlarmError(format!("Alarm system not found: {}", system_id)))
    }

    /// Save alarm system configuration
    pub async fn save_alarm_system(&self, system: &MedicalAlarmSystem) -> Result<()> {
        let mut alarms = self.alarm_systems.write().await;
        alarms.insert(system.alarm_system_id.clone(), system.clone());
        Ok(())
    }
}

impl Default for SimulatorAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ProfileManager for SimulatorAdapter {
    async fn get_device_profile(&self, profile_id: &str) -> Result<MedicalDeviceAccessibilityProfile> {
        let profiles = self.device_profiles.read().await;
        profiles
            .get(profile_id)
            .cloned()
            .ok_or_else(|| MedicalError::ProfileNotFound(profile_id.to_string()))
    }

    async fn save_device_profile(&self, profile: &MedicalDeviceAccessibilityProfile) -> Result<()> {
        let mut profiles = self.device_profiles.write().await;
        profiles.insert(profile.profile_id.clone(), profile.clone());
        Ok(())
    }

    async fn delete_device_profile(&self, profile_id: &str) -> Result<()> {
        let mut profiles = self.device_profiles.write().await;
        profiles
            .remove(profile_id)
            .map(|_| ())
            .ok_or_else(|| MedicalError::ProfileNotFound(profile_id.to_string()))
    }

    async fn list_device_profiles(&self) -> Result<Vec<String>> {
        let profiles = self.device_profiles.read().await;
        Ok(profiles.keys().cloned().collect())
    }

    async fn get_user_profile(&self, user_id: &str) -> Result<UserMedicalAccessibilityProfile> {
        let profiles = self.user_profiles.read().await;
        profiles
            .get(user_id)
            .cloned()
            .ok_or_else(|| MedicalError::ProfileNotFound(user_id.to_string()))
    }

    async fn save_user_profile(&self, profile: &UserMedicalAccessibilityProfile) -> Result<()> {
        let mut profiles = self.user_profiles.write().await;
        profiles.insert(profile.user_id.clone(), profile.clone());
        Ok(())
    }

    async fn delete_user_profile(&self, user_id: &str) -> Result<()> {
        let mut profiles = self.user_profiles.write().await;
        profiles
            .remove(user_id)
            .map(|_| ())
            .ok_or_else(|| MedicalError::ProfileNotFound(user_id.to_string()))
    }

    async fn list_user_profiles(&self) -> Result<Vec<String>> {
        let profiles = self.user_profiles.read().await;
        Ok(profiles.keys().cloned().collect())
    }
}
