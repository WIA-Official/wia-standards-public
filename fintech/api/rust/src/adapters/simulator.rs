//! Simulator Adapter for WIA Fintech
//!
//! In-memory adapter for testing and development.

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::core::{ATMManager, ATMWithCompatibility, CompatibilityChecker, ProfileManager};
use crate::error::{FintechError, FintechResult};
use crate::types::*;

/// Simulator adapter with in-memory storage
pub struct SimulatorAdapter {
    profiles: Arc<RwLock<HashMap<String, UserFinancialAccessibilityProfile>>>,
    atms: Arc<RwLock<HashMap<String, ATMAccessibilityProfile>>>,
}

impl SimulatorAdapter {
    /// Create a new empty simulator
    pub fn new() -> Self {
        Self {
            profiles: Arc::new(RwLock::new(HashMap::new())),
            atms: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Create simulator with sample data
    pub async fn with_sample_data() -> Self {
        let adapter = Self::new();

        // Add sample user profiles
        for profile in Self::create_sample_profiles() {
            let _ = adapter.create_profile(profile).await;
        }

        // Add sample ATMs
        for atm in Self::create_sample_atms() {
            let _ = adapter.register_atm(atm).await;
        }

        adapter
    }

    fn create_sample_profiles() -> Vec<UserFinancialAccessibilityProfile> {
        vec![
            // Blind user profile
            UserFinancialAccessibilityProfile {
                profile_id: "user_blind_001".to_string(),
                wia_id: Some("wia_user_001".to_string()),
                version: "1.0.0".to_string(),
                created_at: Some(chrono::Utc::now()),
                updated_at: Some(chrono::Utc::now()),
                personal_info: PersonalInfo {
                    preferred_name: Some("Alex".to_string()),
                    preferred_language: "en-US".to_string(),
                    region: "US".to_string(),
                    timezone: Some("America/New_York".to_string()),
                },
                accessibility_needs: AccessibilityNeeds {
                    sensory: SensoryAccessibilityNeeds {
                        visual: Some(VisualAccessibilityNeeds {
                            level: VisualLevel::TotallyBlind,
                            color_vision: None,
                            light_sensitivity: None,
                            preferences: None,
                            assistive_tech: Some(VisualAssistiveTech {
                                screen_reader: Some(true),
                                screen_reader_type: Some(ScreenReaderType::Voiceover),
                                braille_display: Some(true),
                                magnifier: None,
                            }),
                        }),
                        auditory: Some(AuditoryAccessibilityNeeds {
                            level: AuditoryLevel::None,
                            uses_hearing_aid: None,
                            uses_cochlear_implant: None,
                            preferred_sign_language: None,
                            preferences: None,
                        }),
                    },
                    motor: None,
                    cognitive: None,
                },
                financial_preferences: FinancialPreferences {
                    preferred_auth_method: vec![
                        AuthenticationMethod::BiometricFingerprint,
                        AuthenticationMethod::Pin,
                    ],
                    transaction_confirmation: Some(ConfirmationPreference {
                        method: Some("audio".to_string()),
                        requires_explicit_confirm: Some(true),
                        repeat_confirmation: Some(true),
                    }),
                    notification_preferences: None,
                    security_preferences: Some(SecurityAccessibilityPreferences {
                        pin_preferences: Some(PinPreferences {
                            length: Some(4),
                            audio_feedback: Some(true),
                            haptic_feedback: Some(true),
                            extended_timeout: Some(true),
                            timeout_seconds: Some(120),
                        }),
                        otp_preferences: Some(OtpPreferences {
                            delivery_method: Some("voice".to_string()),
                            extended_validity: Some(true),
                            validity_seconds: Some(120),
                            audio_readout: Some(true),
                        }),
                        session_preferences: None,
                    }),
                },
                atm_preferences: Some(ATMPreferences {
                    preferred_interface: Some("audio".to_string()),
                    audio_guidance: Some(AudioGuidanceSettings {
                        enabled: true,
                        volume: Some(80),
                        speech_rate: Some(1.0),
                        language: Some("en-US".to_string()),
                    }),
                    timeout: Some(TimeoutPreferences {
                        extended: true,
                        seconds: Some(180),
                    }),
                    receipt_format: Some(ReceiptFormat::Audio),
                }),
                card_preferences: Some(CardPreferences {
                    preferred_card_type: Some(AccessibleCardType::Notched),
                    pin_entry_method: Some(vec!["keypad".to_string()]),
                    contactless_preferred: Some(true),
                }),
                wia_integration: Some(WIAIntegration {
                    enabled: true,
                    connected_devices: Some(vec![WIADeviceConnection {
                        device_id: "exo_001".to_string(),
                        device_type: WIADeviceType::Exoskeleton,
                        connection_status: Some("connected".to_string()),
                        last_seen: Some(chrono::Utc::now()),
                    }]),
                    preferred_output_device: Some("exo_001".to_string()),
                    cross_device_sync: Some(true),
                }),
                emergency_settings: None,
            },
            // Deaf user profile
            UserFinancialAccessibilityProfile {
                profile_id: "user_deaf_001".to_string(),
                wia_id: Some("wia_user_002".to_string()),
                version: "1.0.0".to_string(),
                created_at: Some(chrono::Utc::now()),
                updated_at: Some(chrono::Utc::now()),
                personal_info: PersonalInfo {
                    preferred_name: Some("Jordan".to_string()),
                    preferred_language: "en-US".to_string(),
                    region: "US".to_string(),
                    timezone: Some("America/Los_Angeles".to_string()),
                },
                accessibility_needs: AccessibilityNeeds {
                    sensory: SensoryAccessibilityNeeds {
                        visual: None,
                        auditory: Some(AuditoryAccessibilityNeeds {
                            level: AuditoryLevel::Deaf,
                            uses_hearing_aid: Some(false),
                            uses_cochlear_implant: Some(false),
                            preferred_sign_language: Some(SignLanguageType::ASL),
                            preferences: Some(AuditoryPreferences {
                                visual_alerts: Some(true),
                                captions_enabled: Some(true),
                                sign_language_video: Some(true),
                                mono_audio: None,
                                volume_boost: None,
                            }),
                        }),
                    },
                    motor: None,
                    cognitive: None,
                },
                financial_preferences: FinancialPreferences {
                    preferred_auth_method: vec![
                        AuthenticationMethod::BiometricFace,
                        AuthenticationMethod::Pin,
                    ],
                    transaction_confirmation: Some(ConfirmationPreference {
                        method: Some("visual".to_string()),
                        requires_explicit_confirm: Some(true),
                        repeat_confirmation: None,
                    }),
                    notification_preferences: None,
                    security_preferences: None,
                },
                atm_preferences: Some(ATMPreferences {
                    preferred_interface: Some("visual".to_string()),
                    audio_guidance: None,
                    timeout: Some(TimeoutPreferences {
                        extended: true,
                        seconds: Some(120),
                    }),
                    receipt_format: Some(ReceiptFormat::Paper),
                }),
                card_preferences: Some(CardPreferences {
                    preferred_card_type: Some(AccessibleCardType::HighContrast),
                    pin_entry_method: Some(vec!["keypad".to_string()]),
                    contactless_preferred: Some(true),
                }),
                wia_integration: Some(WIAIntegration {
                    enabled: true,
                    connected_devices: Some(vec![WIADeviceConnection {
                        device_id: "vs_001".to_string(),
                        device_type: WIADeviceType::VoiceSign,
                        connection_status: Some("connected".to_string()),
                        last_seen: Some(chrono::Utc::now()),
                    }]),
                    preferred_output_device: Some("vs_001".to_string()),
                    cross_device_sync: Some(true),
                }),
                emergency_settings: None,
            },
            // Wheelchair user profile
            UserFinancialAccessibilityProfile {
                profile_id: "user_wheelchair_001".to_string(),
                wia_id: Some("wia_user_003".to_string()),
                version: "1.0.0".to_string(),
                created_at: Some(chrono::Utc::now()),
                updated_at: Some(chrono::Utc::now()),
                personal_info: PersonalInfo {
                    preferred_name: Some("Sam".to_string()),
                    preferred_language: "en-US".to_string(),
                    region: "US".to_string(),
                    timezone: Some("America/Chicago".to_string()),
                },
                accessibility_needs: AccessibilityNeeds {
                    sensory: SensoryAccessibilityNeeds {
                        visual: None,
                        auditory: None,
                    },
                    motor: Some(MotorAccessibilityNeeds {
                        level: MotorLevel::Moderate,
                        upper_limb: Some(UpperLimbFunction {
                            left: LimbFunctionality::Full,
                            right: LimbFunctionality::Limited,
                        }),
                        fine_motor: Some(FineMotorLevel::Reduced),
                        preferences: Some(MotorPreferences {
                            large_targets: Some(true),
                            extended_timeouts: Some(true),
                            voice_control: Some(false),
                            switch_access: Some(false),
                            eye_tracking: Some(false),
                            dwell_click: Some(false),
                            dwell_time: None,
                        }),
                        assistive_devices: Some(MotorAssistiveDevices {
                            uses_wheelchair: Some(true),
                            uses_exoskeleton: Some(false),
                        }),
                    }),
                    cognitive: None,
                },
                financial_preferences: FinancialPreferences {
                    preferred_auth_method: vec![
                        AuthenticationMethod::BiometricFingerprint,
                        AuthenticationMethod::Pin,
                    ],
                    transaction_confirmation: None,
                    notification_preferences: None,
                    security_preferences: Some(SecurityAccessibilityPreferences {
                        pin_preferences: Some(PinPreferences {
                            length: Some(4),
                            audio_feedback: None,
                            haptic_feedback: Some(true),
                            extended_timeout: Some(true),
                            timeout_seconds: Some(90),
                        }),
                        otp_preferences: None,
                        session_preferences: None,
                    }),
                },
                atm_preferences: Some(ATMPreferences {
                    preferred_interface: Some("touchscreen".to_string()),
                    audio_guidance: None,
                    timeout: Some(TimeoutPreferences {
                        extended: true,
                        seconds: Some(120),
                    }),
                    receipt_format: Some(ReceiptFormat::Email),
                }),
                card_preferences: Some(CardPreferences {
                    preferred_card_type: Some(AccessibleCardType::Standard),
                    pin_entry_method: Some(vec!["keypad".to_string()]),
                    contactless_preferred: Some(true),
                }),
                wia_integration: Some(WIAIntegration {
                    enabled: true,
                    connected_devices: Some(vec![WIADeviceConnection {
                        device_id: "wheelchair_001".to_string(),
                        device_type: WIADeviceType::SmartWheelchair,
                        connection_status: Some("connected".to_string()),
                        last_seen: Some(chrono::Utc::now()),
                    }]),
                    preferred_output_device: Some("wheelchair_001".to_string()),
                    cross_device_sync: Some(true),
                }),
                emergency_settings: None,
            },
        ]
    }

    fn create_sample_atms() -> Vec<ATMAccessibilityProfile> {
        vec![
            // Fully accessible WIA Gold ATM
            ATMAccessibilityProfile {
                atm_id: "atm_gold_001".to_string(),
                bank_code: "WIA_BANK".to_string(),
                manufacturer: Some("NCR".to_string()),
                model: Some("SelfServ 80".to_string()),
                location: ATMLocation {
                    address: "123 Accessibility Ave".to_string(),
                    city: "San Francisco".to_string(),
                    state: Some("CA".to_string()),
                    country: "US".to_string(),
                    postal_code: Some("94102".to_string()),
                    latitude: 37.7749,
                    longitude: -122.4194,
                    location_description: Some("Inside WIA Bank main lobby".to_string()),
                    access_instructions: Some("Enter through automatic doors, ATM is on the right".to_string()),
                    nearby_landmarks: Some(vec!["City Hall".to_string(), "Main Library".to_string()]),
                },
                status: ATMStatus::Operational,
                physical_accessibility: PhysicalAccessibility {
                    wheelchair_accessible: true,
                    height_adjustable: Some(true),
                    screen_height: Some(120.0),
                    keypad_height: Some(100.0),
                    clear_floor_space: Some(true),
                    indoor_outdoor: Some("indoor".to_string()),
                    sheltered: Some(true),
                    well_lit: Some(true),
                },
                audio_accessibility: AudioAccessibility {
                    audio_guidance: true,
                    headphone_jack: Some(true),
                    jack_type: Some("3.5mm".to_string()),
                    jack_location: Some("Right side, below keypad".to_string()),
                    speaker_output: Some(true),
                    volume_adjustable: Some(true),
                    supported_languages: Some(vec!["en".to_string(), "es".to_string(), "ko".to_string()]),
                    privacy_mode: Some(true),
                },
                tactile_accessibility: TactileAccessibility {
                    braille_labels: true,
                    tactile_keypad: Some(true),
                    keypad_layout: Some("phone".to_string()),
                    key5_marker: Some(true),
                    confirm_button_marker: Some("circle".to_string()),
                    cancel_button_marker: Some("cross".to_string()),
                    card_slot_tactile: Some(true),
                },
                screen_accessibility: Some(ScreenAccessibility {
                    touchscreen: Some(true),
                    physical_buttons: Some(true),
                    high_contrast_mode: Some(true),
                    font_size_adjustable: Some(true),
                    color_blind_mode: Some(true),
                    anti_glare: Some(true),
                    privacy_screen: Some(true),
                }),
                operational_features: Some(OperationalFeatures {
                    extended_timeout: Some(true),
                    timeout_seconds: Some(180),
                    receipt_options: Some(vec![
                        ReceiptFormat::Paper,
                        ReceiptFormat::Email,
                        ReceiptFormat::Sms,
                        ReceiptFormat::Audio,
                    ]),
                    cardless_transaction: Some(true),
                    nfc_enabled: Some(true),
                    qr_code_enabled: Some(true),
                    mobile_pre_staging: Some(true),
                }),
                wia_integration: Some(ATMWIAIntegration {
                    enabled: true,
                    wia_profile_sync: Some(true),
                    exoskeleton_guidance: Some(true),
                    bionic_eye_display: Some(true),
                    voice_sign_support: Some(true),
                    smart_wheelchair_positioning: Some(true),
                    bluetooth_enabled: Some(true),
                    nfc_wia_enabled: Some(true),
                }),
                certification: Some(ATMCertification {
                    ada_compliant: Some(true),
                    eaa_compliant: Some(true),
                    wia_level: Some(WIACertificationLevel::Gold),
                    last_inspection_date: Some("2025-01-01".to_string()),
                    certifications: Some(vec![
                        "ADA Title III".to_string(),
                        "EAA 2025".to_string(),
                        "WIA Gold".to_string(),
                    ]),
                }),
            },
            // Basic accessible ATM (Bronze level)
            ATMAccessibilityProfile {
                atm_id: "atm_bronze_001".to_string(),
                bank_code: "BASIC_BANK".to_string(),
                manufacturer: Some("Diebold".to_string()),
                model: Some("Opteva 500".to_string()),
                location: ATMLocation {
                    address: "456 Standard St".to_string(),
                    city: "San Francisco".to_string(),
                    state: Some("CA".to_string()),
                    country: "US".to_string(),
                    postal_code: Some("94103".to_string()),
                    latitude: 37.7751,
                    longitude: -122.4180,
                    location_description: Some("Outside convenience store".to_string()),
                    access_instructions: None,
                    nearby_landmarks: None,
                },
                status: ATMStatus::Operational,
                physical_accessibility: PhysicalAccessibility {
                    wheelchair_accessible: true,
                    height_adjustable: Some(false),
                    screen_height: Some(130.0),
                    keypad_height: Some(110.0),
                    clear_floor_space: Some(true),
                    indoor_outdoor: Some("outdoor".to_string()),
                    sheltered: Some(false),
                    well_lit: Some(true),
                },
                audio_accessibility: AudioAccessibility {
                    audio_guidance: true,
                    headphone_jack: Some(true),
                    jack_type: Some("3.5mm".to_string()),
                    jack_location: Some("Below screen".to_string()),
                    speaker_output: Some(false),
                    volume_adjustable: Some(false),
                    supported_languages: Some(vec!["en".to_string()]),
                    privacy_mode: Some(true),
                },
                tactile_accessibility: TactileAccessibility {
                    braille_labels: true,
                    tactile_keypad: Some(true),
                    keypad_layout: Some("phone".to_string()),
                    key5_marker: Some(true),
                    confirm_button_marker: Some("raised".to_string()),
                    cancel_button_marker: Some("raised".to_string()),
                    card_slot_tactile: Some(false),
                },
                screen_accessibility: Some(ScreenAccessibility {
                    touchscreen: Some(false),
                    physical_buttons: Some(true),
                    high_contrast_mode: Some(false),
                    font_size_adjustable: Some(false),
                    color_blind_mode: Some(false),
                    anti_glare: Some(false),
                    privacy_screen: Some(false),
                }),
                operational_features: Some(OperationalFeatures {
                    extended_timeout: Some(false),
                    timeout_seconds: Some(60),
                    receipt_options: Some(vec![ReceiptFormat::Paper, ReceiptFormat::None]),
                    cardless_transaction: Some(false),
                    nfc_enabled: Some(false),
                    qr_code_enabled: Some(false),
                    mobile_pre_staging: Some(false),
                }),
                wia_integration: None,
                certification: Some(ATMCertification {
                    ada_compliant: Some(true),
                    eaa_compliant: Some(false),
                    wia_level: Some(WIACertificationLevel::Bronze),
                    last_inspection_date: Some("2024-06-15".to_string()),
                    certifications: Some(vec!["ADA Title III".to_string()]),
                }),
            },
            // Non-accessible ATM (for testing)
            ATMAccessibilityProfile {
                atm_id: "atm_basic_001".to_string(),
                bank_code: "OLD_BANK".to_string(),
                manufacturer: Some("Generic".to_string()),
                model: Some("ATM-1000".to_string()),
                location: ATMLocation {
                    address: "789 Old Way".to_string(),
                    city: "San Francisco".to_string(),
                    state: Some("CA".to_string()),
                    country: "US".to_string(),
                    postal_code: Some("94104".to_string()),
                    latitude: 37.7755,
                    longitude: -122.4170,
                    location_description: Some("Inside old building".to_string()),
                    access_instructions: None,
                    nearby_landmarks: None,
                },
                status: ATMStatus::Operational,
                physical_accessibility: PhysicalAccessibility {
                    wheelchair_accessible: false,
                    height_adjustable: Some(false),
                    screen_height: Some(150.0),
                    keypad_height: Some(130.0),
                    clear_floor_space: Some(false),
                    indoor_outdoor: Some("indoor".to_string()),
                    sheltered: Some(true),
                    well_lit: Some(false),
                },
                audio_accessibility: AudioAccessibility {
                    audio_guidance: false,
                    headphone_jack: Some(false),
                    jack_type: None,
                    jack_location: None,
                    speaker_output: Some(false),
                    volume_adjustable: Some(false),
                    supported_languages: None,
                    privacy_mode: Some(false),
                },
                tactile_accessibility: TactileAccessibility {
                    braille_labels: false,
                    tactile_keypad: Some(false),
                    keypad_layout: Some("calculator".to_string()),
                    key5_marker: Some(false),
                    confirm_button_marker: Some("none".to_string()),
                    cancel_button_marker: Some("none".to_string()),
                    card_slot_tactile: Some(false),
                },
                screen_accessibility: None,
                operational_features: Some(OperationalFeatures {
                    extended_timeout: Some(false),
                    timeout_seconds: Some(30),
                    receipt_options: Some(vec![ReceiptFormat::Paper]),
                    cardless_transaction: Some(false),
                    nfc_enabled: Some(false),
                    qr_code_enabled: Some(false),
                    mobile_pre_staging: Some(false),
                }),
                wia_integration: None,
                certification: Some(ATMCertification {
                    ada_compliant: Some(false),
                    eaa_compliant: Some(false),
                    wia_level: None,
                    last_inspection_date: None,
                    certifications: None,
                }),
            },
        ]
    }

    /// Calculate distance between two coordinates (Haversine formula)
    fn calculate_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
        let r = 6371.0; // Earth's radius in km

        let d_lat = (lat2 - lat1).to_radians();
        let d_lon = (lon2 - lon1).to_radians();

        let a = (d_lat / 2.0).sin().powi(2)
            + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);

        let c = 2.0 * a.sqrt().asin();

        r * c
    }
}

impl Default for SimulatorAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ProfileManager for SimulatorAdapter {
    async fn create_profile(&self, profile: UserFinancialAccessibilityProfile) -> FintechResult<UserFinancialAccessibilityProfile> {
        let mut profiles = self.profiles.write().await;

        if profiles.contains_key(&profile.profile_id) {
            return Err(FintechError::ValidationError(format!(
                "Profile with ID {} already exists",
                profile.profile_id
            )));
        }

        profiles.insert(profile.profile_id.clone(), profile.clone());
        Ok(profile)
    }

    async fn get_profile(&self, profile_id: &str) -> FintechResult<UserFinancialAccessibilityProfile> {
        let profiles = self.profiles.read().await;

        profiles
            .get(profile_id)
            .cloned()
            .ok_or_else(|| FintechError::ProfileNotFound(profile_id.to_string()))
    }

    async fn update_profile(&self, profile: UserFinancialAccessibilityProfile) -> FintechResult<UserFinancialAccessibilityProfile> {
        let mut profiles = self.profiles.write().await;

        if !profiles.contains_key(&profile.profile_id) {
            return Err(FintechError::ProfileNotFound(profile.profile_id.clone()));
        }

        profiles.insert(profile.profile_id.clone(), profile.clone());
        Ok(profile)
    }

    async fn delete_profile(&self, profile_id: &str) -> FintechResult<()> {
        let mut profiles = self.profiles.write().await;

        profiles
            .remove(profile_id)
            .map(|_| ())
            .ok_or_else(|| FintechError::ProfileNotFound(profile_id.to_string()))
    }

    async fn list_profiles(&self) -> FintechResult<Vec<UserFinancialAccessibilityProfile>> {
        let profiles = self.profiles.read().await;
        Ok(profiles.values().cloned().collect())
    }
}

impl SimulatorAdapter {
    /// List all ATMs
    pub async fn list_atms(&self) -> FintechResult<Vec<ATMAccessibilityProfile>> {
        let atms = self.atms.read().await;
        Ok(atms.values().cloned().collect())
    }

    /// Find ATMs nearby a location (alias for search_atms)
    pub async fn find_atms_nearby(&self, latitude: f64, longitude: f64, radius_km: f64) -> FintechResult<Vec<ATMAccessibilityProfile>> {
        self.search_atms(latitude, longitude, radius_km).await
    }

    /// Find accessible ATMs for a user
    pub async fn find_accessible_atms(
        &self,
        user_profile: &UserFinancialAccessibilityProfile,
        latitude: f64,
        longitude: f64,
        radius_km: f64,
    ) -> FintechResult<Vec<ATMAccessibilityProfile>> {
        let checker = CompatibilityChecker::new();
        let atms = self.search_atms(latitude, longitude, radius_km).await?;

        let accessible: Vec<ATMAccessibilityProfile> = atms
            .into_iter()
            .filter(|atm| {
                let result = checker.check_compatibility(user_profile, atm);
                result.is_compatible
            })
            .collect();

        Ok(accessible)
    }
}

#[async_trait]
impl ATMManager for SimulatorAdapter {
    async fn get_atm(&self, atm_id: &str) -> FintechResult<ATMAccessibilityProfile> {
        let atms = self.atms.read().await;

        atms.get(atm_id)
            .cloned()
            .ok_or_else(|| FintechError::ATMNotFound(atm_id.to_string()))
    }

    async fn search_atms(&self, latitude: f64, longitude: f64, radius_km: f64) -> FintechResult<Vec<ATMAccessibilityProfile>> {
        let atms = self.atms.read().await;

        let mut results: Vec<(f64, ATMAccessibilityProfile)> = atms
            .values()
            .filter_map(|atm| {
                let distance = Self::calculate_distance(
                    latitude,
                    longitude,
                    atm.location.latitude,
                    atm.location.longitude,
                );

                if distance <= radius_km {
                    Some((distance, atm.clone()))
                } else {
                    None
                }
            })
            .collect();

        // Sort by distance
        results.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        Ok(results.into_iter().map(|(_, atm)| atm).collect())
    }

    async fn search_accessible_atms(
        &self,
        latitude: f64,
        longitude: f64,
        radius_km: f64,
        user_profile: &UserFinancialAccessibilityProfile,
    ) -> FintechResult<Vec<ATMWithCompatibility>> {
        let atms = self.search_atms(latitude, longitude, radius_km).await?;

        let checker = CompatibilityChecker::new();
        let mut results: Vec<ATMWithCompatibility> = atms
            .into_iter()
            .map(|atm| {
                let compatibility = checker.check_compatibility(user_profile, &atm);
                let distance = Self::calculate_distance(
                    latitude,
                    longitude,
                    atm.location.latitude,
                    atm.location.longitude,
                );

                ATMWithCompatibility {
                    atm,
                    compatibility,
                    distance_km: distance,
                }
            })
            .collect();

        // Sort by compatibility score (descending), then by distance
        results.sort_by(|a, b| {
            b.compatibility
                .compatibility_score
                .partial_cmp(&a.compatibility.compatibility_score)
                .unwrap()
                .then_with(|| a.distance_km.partial_cmp(&b.distance_km).unwrap())
        });

        Ok(results)
    }

    async fn register_atm(&self, atm: ATMAccessibilityProfile) -> FintechResult<ATMAccessibilityProfile> {
        let mut atms = self.atms.write().await;
        atms.insert(atm.atm_id.clone(), atm.clone());
        Ok(atm)
    }

    async fn update_atm_status(&self, atm_id: &str, status: ATMStatus) -> FintechResult<()> {
        let mut atms = self.atms.write().await;

        if let Some(atm) = atms.get_mut(atm_id) {
            atm.status = status;
            Ok(())
        } else {
            Err(FintechError::ATMNotFound(atm_id.to_string()))
        }
    }
}
