//! CGM (Continuous Glucose Monitor) Accessibility Example
//!
//! This example demonstrates how to:
//! - Configure accessibility for CGM devices
//! - Set up multi-sensory alerts for glucose readings
//! - Integrate with WIA devices (Exoskeleton, Voice-Sign)

use wia_medical::prelude::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== WIA CGM Accessibility Configuration ===\n");

    // Create a CGM device profile with comprehensive accessibility
    let cgm_profile = create_accessible_cgm();

    println!("Device: {}", cgm_profile.device.device_name);
    println!("Manufacturer: {}", cgm_profile.device.manufacturer);
    println!();

    // Show accessibility features
    println!("--- Accessibility Features ---\n");

    // Visual accessibility
    println!("Visual Accessibility:");
    if let Some(ref vo) = cgm_profile.accessibility.visual.voice_output {
        if vo.supported {
            println!("  Voice Output: Enabled");
            println!("  Languages: {}", vo.languages.join(", "));
            for reading in &vo.readings {
                println!("  Reading Format: \"{}\"", reading.format);
            }
        }
    }
    println!();

    // Auditory accessibility
    println!("Auditory Accessibility:");
    if let Some(ref ha) = cgm_profile.accessibility.auditory.haptic_alerts {
        if ha.supported {
            println!("  Haptic Alerts: {} patterns", ha.patterns.len());
            for pattern in &ha.patterns {
                println!(
                    "    - {} ({}): intensity {:.0}%",
                    pattern.name,
                    pattern.meaning,
                    pattern.intensity * 100.0
                );
            }
        }
    }
    println!();

    // WIA Integration
    println!("--- WIA Integration ---\n");
    if let Some(ref wia) = cgm_profile.wia_integration {
        println!("Supported Protocols:");
        println!("  Exoskeleton: {}", wia.supported_protocols.exoskeleton);
        println!("  Voice-Sign: {}", wia.supported_protocols.voice_sign);
        println!("  Haptic: {}", wia.supported_protocols.haptic);

        if let Some(ref exo) = wia.exoskeleton {
            println!("\nExoskeleton Integration:");
            for mapping in &exo.haptic_feedback_mapping {
                println!(
                    "  {} -> {} ({:?}, intensity: {:.0}%)",
                    mapping.device_event,
                    mapping.haptic_pattern,
                    mapping.target,
                    mapping.intensity * 100.0
                );
            }
        }

        if let Some(ref vs) = wia.voice_sign {
            println!("\nVoice-Sign Integration:");
            println!("  Medical Terms: {}", vs.medical_terminology_support);
            println!("  Emergency Phrases:");
            for phrase in &vs.emergency_phrases {
                println!("    - \"{}\"", phrase);
            }
        }
    }
    println!();

    // Create alarm system configuration
    let alarm_system = create_cgm_alarm_system();
    println!("--- Alarm System Configuration ---\n");
    println!("Alarm Categories:");
    for alarm in &alarm_system.alarm_categories {
        println!("  {} ({:?}):", alarm.name, alarm.priority);
        println!("    Meaning: {}", alarm.meaning);
        println!("    Action: {}", alarm.recommended_action);

        if let Some(ref visual) = alarm.visual_config {
            println!("    Visual: {} {}", visual.color, format!("{:?}", visual.pattern).to_lowercase());
        }
        if let Some(ref auditory) = alarm.auditory_config {
            if let Some(ref voice) = auditory.voice_announcement {
                println!("    Voice: \"{}\"", voice.text);
            }
        }
        if let Some(ref haptic) = alarm.haptic_config {
            println!("    Haptic: {} at {:.0}% intensity", haptic.pattern_id, haptic.intensity * 100.0);
        }
        println!();
    }

    // Test compatibility with different users
    println!("--- User Compatibility Tests ---\n");

    // Blind user
    let blind_user = UserProfileBuilder::new()
        .visual_needs(VisualLevel::TotallyBlind)
        .primary_input(InputMethod::Voice)
        .build();

    let result = ProfileMatcher::is_compatible(&cgm_profile, &blind_user);
    println!("Blind User:");
    println!("  Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("  Score: {:.1}%", result.score);
    if !result.issues.is_empty() {
        for issue in &result.issues {
            println!("  Issue: {}", issue.description);
        }
    }
    println!();

    // Deaf user
    let deaf_user = UserProfileBuilder::new()
        .auditory_needs(AuditoryLevel::Deaf)
        .primary_input(InputMethod::Touch)
        .build();

    let result = ProfileMatcher::is_compatible(&cgm_profile, &deaf_user);
    println!("Deaf User:");
    println!("  Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("  Score: {:.1}%", result.score);
    println!();

    // Motor impaired user with exoskeleton
    let motor_user = UserProfileBuilder::new()
        .motor_needs(ImpairmentLevel::Severe)
        .with_exoskeleton("wia_exo_v2", 80)
        .build();

    let result = ProfileMatcher::is_compatible(&cgm_profile, &motor_user);
    println!("Motor Impaired User (with WIA Exoskeleton):");
    println!("  Compatible: {}", if result.compatible { "Yes" } else { "No" });
    println!("  Score: {:.1}%", result.score);
    println!();

    // Calculate and display accessibility score
    println!("--- Final Accessibility Score ---\n");
    let score = AccessibilityScoreCalculator::calculate(&cgm_profile.accessibility);
    println!("Overall: {:.1}%", score.overall);
    println!("  Visual: {:.1}%", score.visual);
    println!("  Auditory: {:.1}%", score.auditory);
    println!("  Motor: {:.1}%", score.motor);
    println!("  Cognitive: {:.1}%", score.cognitive);

    println!("\n=== Configuration Complete ===");
    println!("弘益人間 - Benefit All Humanity");

    Ok(())
}

fn create_accessible_cgm() -> MedicalDeviceAccessibilityProfile {
    DeviceProfileBuilder::new()
        .manufacturer("WIA Medical")
        .model("AccessibleCGM-X1")
        .device_name("WIA Accessible CGM System")
        .device_type(MedicalDeviceType::Monitoring)
        .device_category(DeviceCategory::Cgm)
        .accessibility(DeviceAccessibilityFeatures {
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
                    readings: vec![
                        VoiceReading {
                            data_type: "blood_glucose".to_string(),
                            format: "Your blood glucose is {value} milligrams per deciliter, {trend}".to_string(),
                            units_spoken: true,
                            range_indication: true,
                        },
                        VoiceReading {
                            data_type: "trend".to_string(),
                            format: "glucose is {direction}".to_string(),
                            units_spoken: false,
                            range_indication: true,
                        },
                    ],
                    languages: vec![
                        "en".to_string(),
                        "ko".to_string(),
                        "ja".to_string(),
                        "es".to_string(),
                    ],
                    speed_adjustable: true,
                    volume_adjustable: true,
                }),
                display_accessibility: Some(DisplayAccessibility {
                    high_contrast_mode: true,
                    large_text_mode: true,
                    text_size_adjustable: true,
                    color_inversion: true,
                    color_blind_modes: vec![
                        ColorBlindMode::Protanopia,
                        ColorBlindMode::Deuteranopia,
                        ColorBlindMode::Tritanopia,
                    ],
                }),
                non_visual_alternatives: Some(NonVisualAlternatives {
                    audio_feedback: true,
                    haptic_feedback: true,
                    braille_output: true,
                }),
            },
            auditory: AuditoryAccessibility {
                visual_alerts: Some(VisualAlerts {
                    supported: true,
                    types: vec![
                        VisualAlertType::ScreenFlash,
                        VisualAlertType::LedIndicator,
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
                            name: "Urgent Low".to_string(),
                            meaning: "Blood glucose critically low (<55 mg/dL)".to_string(),
                            duration_ms: 2000,
                            intensity: 1.0,
                        },
                        HapticPattern {
                            id: "low".to_string(),
                            name: "Low".to_string(),
                            meaning: "Blood glucose low (55-70 mg/dL)".to_string(),
                            duration_ms: 1000,
                            intensity: 0.8,
                        },
                        HapticPattern {
                            id: "high".to_string(),
                            name: "High".to_string(),
                            meaning: "Blood glucose high (>180 mg/dL)".to_string(),
                            duration_ms: 800,
                            intensity: 0.6,
                        },
                        HapticPattern {
                            id: "rising_rapidly".to_string(),
                            name: "Rising Rapidly".to_string(),
                            meaning: "Blood glucose rising more than 3 mg/dL/min".to_string(),
                            duration_ms: 500,
                            intensity: 0.5,
                        },
                        HapticPattern {
                            id: "falling_rapidly".to_string(),
                            name: "Falling Rapidly".to_string(),
                            meaning: "Blood glucose falling more than 3 mg/dL/min".to_string(),
                            duration_ms: 500,
                            intensity: 0.7,
                        },
                    ],
                    intensity_levels: 10,
                }),
                audio_adjustments: Some(AudioAdjustments {
                    volume_range: (0.0, 100.0),
                    frequency_adjustable: true,
                    mono_audio: true,
                    hearing_aid_compatible: true,
                    t_coil_compatible: true,
                }),
                text_alternatives: Some(TextAlternatives {
                    on_screen_text: true,
                    closed_captions: true,
                    real_time_transcription: false,
                }),
            },
            motor: MotorAccessibility {
                alternative_input: Some(AlternativeInput {
                    voice_control: true,
                    switch_access: true,
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
                    undo_capability: true,
                    clear_error_messages: true,
                    recovery_guidance: true,
                }),
                language_support: Some(LanguageSupport {
                    languages: vec![
                        "en".to_string(),
                        "ko".to_string(),
                        "ja".to_string(),
                        "es".to_string(),
                        "zh".to_string(),
                    ],
                    simple_language_mode: true,
                    icon_based_navigation: true,
                    pictogram_support: true,
                }),
            },
            physical: None,
            accessibility_score: None,
        })
        .wia_integration(WIAIntegration {
            supported_protocols: WIASupportedProtocols {
                exoskeleton: true,
                bionic_eye: true,
                voice_sign: true,
                haptic: true,
                smart_wheelchair: false,
            },
            exoskeleton: Some(ExoskeletonIntegration {
                haptic_feedback_mapping: vec![
                    HapticMapping {
                        device_event: "urgent_low_glucose".to_string(),
                        haptic_pattern: "urgent_pulse".to_string(),
                        target: HapticTarget::LeftHand,
                        intensity: 1.0,
                    },
                    HapticMapping {
                        device_event: "low_glucose".to_string(),
                        haptic_pattern: "gentle_pulse".to_string(),
                        target: HapticTarget::LeftHand,
                        intensity: 0.7,
                    },
                    HapticMapping {
                        device_event: "high_glucose".to_string(),
                        haptic_pattern: "double_tap".to_string(),
                        target: HapticTarget::RightHand,
                        intensity: 0.5,
                    },
                    HapticMapping {
                        device_event: "trend_change".to_string(),
                        haptic_pattern: "wave".to_string(),
                        target: HapticTarget::Torso,
                        intensity: 0.3,
                    },
                ],
                motion_assistance: false,
                rehabilitation_mode: false,
            }),
            bionic_eye: Some(BionicEyeIntegration {
                display_optimization: true,
                contrast_enhancement: true,
                pattern_simplification: true,
            }),
            voice_sign: Some(VoiceSignIntegration {
                medical_terminology_support: true,
                real_time_translation: true,
                emergency_phrases: vec![
                    "My blood sugar is very low".to_string(),
                    "I need sugar immediately".to_string(),
                    "I am having a hypoglycemic episode".to_string(),
                    "Please call emergency services".to_string(),
                    "I have diabetes".to_string(),
                ],
            }),
            haptic: Some(HapticIntegration {
                alarm_mapping: vec![
                    HapticAlarmMapping {
                        alarm_priority: "critical".to_string(),
                        haptic_pattern: "urgent_continuous".to_string(),
                        body_location: "wrist".to_string(),
                    },
                    HapticAlarmMapping {
                        alarm_priority: "high".to_string(),
                        haptic_pattern: "pulsing".to_string(),
                        body_location: "wrist".to_string(),
                    },
                    HapticAlarmMapping {
                        alarm_priority: "medium".to_string(),
                        haptic_pattern: "gentle_tap".to_string(),
                        body_location: "wrist".to_string(),
                    },
                ],
                data_haptic_encoding: true,
            }),
            smart_wheelchair: None,
        })
        .build()
        .expect("Failed to build CGM profile")
}

fn create_cgm_alarm_system() -> MedicalAlarmSystem {
    MedicalAlarmSystem {
        alarm_system_id: "cgm_alarm_v1".to_string(),
        alarm_system_version: "1.0.0".to_string(),
        device_id: Some("accessible_cgm_x1".to_string()),
        alarm_categories: vec![
            AlarmCategory {
                id: "urgent_low".to_string(),
                name: "Urgent Low Glucose".to_string(),
                priority: AlarmPriority::Critical,
                meaning: "Blood glucose is critically low and requires immediate attention".to_string(),
                recommended_action: "Consume 15-20 grams of fast-acting carbohydrates immediately".to_string(),
                medical_urgency: Some(MedicalUrgency::Emergency),
                visual_config: Some(VisualAlarmConfig {
                    enabled: true,
                    color: "#FF0000".to_string(),
                    pattern: VisualPattern::Flashing,
                    flash_rate_hz: Some(2.0),
                    icon: Some("glucose_critical_low".to_string()),
                    text: Some("URGENT: Low Blood Sugar".to_string()),
                }),
                auditory_config: Some(AuditoryAlarmConfig {
                    enabled: true,
                    tone_frequency_hz: Some(1000.0),
                    pattern: Some(AuditoryPattern::Escalating),
                    volume_db: 85.0,
                    voice_announcement: Some(VoiceAnnouncement {
                        enabled: true,
                        text: "Urgent! Your blood sugar is critically low. Please eat sugar immediately.".to_string(),
                        text_simple: Some("Low sugar! Eat now!".to_string()),
                        language: Some("en".to_string()),
                    }),
                }),
                haptic_config: Some(HapticAlarmConfig {
                    enabled: true,
                    pattern_id: "urgent_continuous".to_string(),
                    intensity: 1.0,
                    duration_ms: 3000,
                    wia_haptic_device: Some(WIAHapticDeviceConfig {
                        enabled: true,
                        target_body_location: "wrist_left".to_string(),
                    }),
                }),
            },
            AlarmCategory {
                id: "low".to_string(),
                name: "Low Glucose".to_string(),
                priority: AlarmPriority::High,
                meaning: "Blood glucose is low and may need attention soon".to_string(),
                recommended_action: "Monitor closely and consider having a snack".to_string(),
                medical_urgency: Some(MedicalUrgency::AttentionRequired),
                visual_config: Some(VisualAlarmConfig {
                    enabled: true,
                    color: "#FFA500".to_string(),
                    pattern: VisualPattern::Pulsing,
                    flash_rate_hz: Some(1.0),
                    icon: Some("glucose_low".to_string()),
                    text: Some("Low Blood Sugar".to_string()),
                }),
                auditory_config: Some(AuditoryAlarmConfig {
                    enabled: true,
                    tone_frequency_hz: Some(800.0),
                    pattern: Some(AuditoryPattern::Intermittent),
                    volume_db: 75.0,
                    voice_announcement: Some(VoiceAnnouncement {
                        enabled: true,
                        text: "Your blood sugar is low. Please check and consider eating.".to_string(),
                        text_simple: Some("Sugar low. Check now.".to_string()),
                        language: Some("en".to_string()),
                    }),
                }),
                haptic_config: Some(HapticAlarmConfig {
                    enabled: true,
                    pattern_id: "pulsing".to_string(),
                    intensity: 0.7,
                    duration_ms: 1500,
                    wia_haptic_device: None,
                }),
            },
            AlarmCategory {
                id: "high".to_string(),
                name: "High Glucose".to_string(),
                priority: AlarmPriority::Medium,
                meaning: "Blood glucose is above target range".to_string(),
                recommended_action: "Check if insulin is needed. Stay hydrated.".to_string(),
                medical_urgency: Some(MedicalUrgency::Informational),
                visual_config: Some(VisualAlarmConfig {
                    enabled: true,
                    color: "#FFFF00".to_string(),
                    pattern: VisualPattern::Solid,
                    flash_rate_hz: None,
                    icon: Some("glucose_high".to_string()),
                    text: Some("High Blood Sugar".to_string()),
                }),
                auditory_config: Some(AuditoryAlarmConfig {
                    enabled: true,
                    tone_frequency_hz: Some(600.0),
                    pattern: Some(AuditoryPattern::Pulsing),
                    volume_db: 65.0,
                    voice_announcement: Some(VoiceAnnouncement {
                        enabled: true,
                        text: "Your blood sugar is high. Consider checking your insulin.".to_string(),
                        text_simple: Some("Sugar high.".to_string()),
                        language: Some("en".to_string()),
                    }),
                }),
                haptic_config: Some(HapticAlarmConfig {
                    enabled: true,
                    pattern_id: "double_tap".to_string(),
                    intensity: 0.5,
                    duration_ms: 800,
                    wia_haptic_device: None,
                }),
            },
        ],
        output_modalities: OutputModalities {
            visual: ModalitySupport {
                supported: true,
                required_for_critical: true,
                default_enabled: true,
            },
            auditory: ModalitySupport {
                supported: true,
                required_for_critical: true,
                default_enabled: true,
            },
            haptic: ModalitySupport {
                supported: true,
                required_for_critical: true,
                default_enabled: true,
            },
        },
        accessibility_settings: Some(AlarmAccessibilitySettings {
            allow_single_modality: true,
            minimum_modalities_critical: 2,
            minimum_modalities_other: 1,
            cognitive_support: Some(AlarmCognitiveSupport {
                simplified_messages: true,
                step_by_step_guidance: true,
                confirmation_required: false,
                caregiver_notification: true,
            }),
        }),
        escalation: Some(EscalationConfig {
            enabled: true,
            timeout_seconds: 60,
            steps: vec![
                EscalationStep {
                    delay_seconds: 30,
                    action: EscalationAction::IncreaseVolume,
                },
                EscalationStep {
                    delay_seconds: 60,
                    action: EscalationAction::NotifyCaregiver,
                },
                EscalationStep {
                    delay_seconds: 120,
                    action: EscalationAction::EmergencyCall,
                },
            ],
            emergency_contact: Some(EmergencyContact {
                enabled: true,
                contacts: vec![ContactInfo {
                    name: "Emergency Contact".to_string(),
                    phone: "+1-555-0123".to_string(),
                    relationship: Some("Caregiver".to_string()),
                    notification_methods: vec![
                        ContactMethod::Call,
                        ContactMethod::Sms,
                        ContactMethod::AppNotification,
                    ],
                }],
            }),
        }),
    }
}
