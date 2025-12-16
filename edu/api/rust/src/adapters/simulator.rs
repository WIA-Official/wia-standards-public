//! Learner Simulator
//! Simulates different learner profiles for testing
//! 弘益人間 - Education for Everyone

use chrono::Utc;
use uuid::Uuid;

use crate::types::*;

/// Simulates learners with different accessibility needs
#[derive(Debug, Default)]
pub struct LearnerSimulator;

impl LearnerSimulator {
    /// Create a new simulator
    pub fn new() -> Self {
        Self
    }

    /// Create a simulated blind learner profile
    pub fn create_blind_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("Simulated Blind Learner".to_string()),
                preferred_language: Some("en".to_string()),
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Blind],
                notes: Some("Uses screen reader for all content".to_string()),
            },
            display_preferences: DisplayPreferences {
                screen_reader: ScreenReaderSettings {
                    enabled: true,
                    preferred_reader: Some(ScreenReaderType::Nvda),
                    speech_rate: 1.5,
                    verbosity: Some(Verbosity::Verbose),
                },
                ..Default::default()
            },
            control_preferences: ControlPreferences {
                input_method: InputMethodSettings {
                    primary: InputMethod::Keyboard,
                    keyboard_only: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            content_preferences: ContentPreferences {
                audio_description: AudioDescriptionSettings {
                    required: true,
                    extended: true,
                },
                transcripts: TranscriptSettings {
                    required: true,
                    interactive: true,
                },
                text_to_speech: TextToSpeechSettings {
                    enabled: true,
                    rate: 1.5,
                    ..Default::default()
                },
                ..Default::default()
            },
            learning_style: LearningStyle {
                representation: RepresentationPreferences {
                    preferred_modalities: vec![LearningModality::Auditory],
                    ..Default::default()
                },
                ..Default::default()
            },
            assessment_accommodations: AssessmentAccommodations {
                timing: AssessmentTimingAccommodations {
                    extended_time: true,
                    time_multiplier: 1.5,
                    ..Default::default()
                },
                assistance: AssistanceAccommodations {
                    reader: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            assistive_technology: AssistiveTechnology {
                software: vec![ATSoftware {
                    software_type: ATSoftwareType::ScreenReader,
                    name: Some("NVDA".to_string()),
                    version: Some("2024.1".to_string()),
                }],
                ..Default::default()
            },
            wia_integrations: WIAIntegrations::default(),
        }
    }

    /// Create a simulated deaf learner profile
    pub fn create_deaf_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("Simulated Deaf Learner".to_string()),
                preferred_language: Some("en".to_string()),
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Deaf],
                notes: Some("Primary language is ASL".to_string()),
            },
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences {
                captions: CaptionSettings {
                    required: true,
                    style: Some(CaptionStyle::Large),
                    language: Some("en".to_string()),
                },
                transcripts: TranscriptSettings {
                    required: true,
                    interactive: true,
                },
                sign_language: SignLanguageSettings {
                    preferred: true,
                    language: Some(SignLanguageType::Asl),
                },
                ..Default::default()
            },
            learning_style: LearningStyle {
                representation: RepresentationPreferences {
                    preferred_modalities: vec![LearningModality::Visual, LearningModality::Reading],
                    ..Default::default()
                },
                ..Default::default()
            },
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        }
    }

    /// Create a simulated dyslexic learner profile
    pub fn create_dyslexic_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("Simulated Dyslexic Learner".to_string()),
                preferred_language: Some("en".to_string()),
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Dyslexia],
                notes: None,
            },
            display_preferences: DisplayPreferences {
                text_settings: TextSettings {
                    font_family: Some(FontFamily::Opendyslexic),
                    font_size: FontSize::Large,
                    line_spacing: 2.0,
                    letter_spacing: Spacing::Wide,
                    word_spacing: Spacing::Wide,
                    ..Default::default()
                },
                reading_guide: ReadingGuide {
                    enabled: true,
                    guide_type: Some(ReadingGuideType::LineHighlight),
                },
                color_settings: ColorSettings {
                    background_color: Some("#FFFBEB".to_string()), // Cream background
                    ..Default::default()
                },
                ..Default::default()
            },
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences {
                text_to_speech: TextToSpeechSettings {
                    enabled: true,
                    rate: 0.9, // Slightly slower
                    ..Default::default()
                },
                simplification: SimplificationSettings {
                    reading_level: ReadingLevel::Simplified,
                    definitions: true,
                    chunked_content: true,
                },
                ..Default::default()
            },
            learning_style: LearningStyle {
                representation: RepresentationPreferences {
                    preferred_modalities: vec![LearningModality::Auditory, LearningModality::Visual],
                    ..Default::default()
                },
                ..Default::default()
            },
            assessment_accommodations: AssessmentAccommodations {
                timing: AssessmentTimingAccommodations {
                    extended_time: true,
                    time_multiplier: 1.5,
                    ..Default::default()
                },
                assistance: AssistanceAccommodations {
                    spell_check: true,
                    grammar_check: true,
                    reader: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            assistive_technology: AssistiveTechnology {
                software: vec![ATSoftware {
                    software_type: ATSoftwareType::ReadingAssistance,
                    name: Some("Natural Reader".to_string()),
                    version: None,
                }],
                ..Default::default()
            },
            wia_integrations: WIAIntegrations::default(),
        }
    }

    /// Create a simulated ADHD learner profile
    pub fn create_adhd_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("Simulated ADHD Learner".to_string()),
                preferred_language: Some("en".to_string()),
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Adhd],
                notes: None,
            },
            display_preferences: DisplayPreferences {
                reduce_motion: true,
                ..Default::default()
            },
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences {
                simplification: SimplificationSettings {
                    chunked_content: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            learning_style: LearningStyle {
                engagement: EngagementPreferences {
                    interest_triggers: vec![
                        InterestTrigger::Gamification,
                        InterestTrigger::Choice,
                        InterestTrigger::Challenge,
                    ],
                    self_regulation_support: true,
                    feedback_style: Some(FeedbackStyle::Immediate),
                    ..Default::default()
                },
                representation: RepresentationPreferences {
                    preferred_modalities: vec![
                        LearningModality::Interactive,
                        LearningModality::Visual,
                    ],
                    ..Default::default()
                },
                action_expression: ActionExpressionPreferences {
                    planning_support: true,
                    progress_tracking: Some(ProgressTrackingStyle::Visual),
                    ..Default::default()
                },
            },
            assessment_accommodations: AssessmentAccommodations {
                timing: AssessmentTimingAccommodations {
                    extended_time: true,
                    time_multiplier: 1.5,
                    breaks_allowed: true,
                    break_frequency_minutes: Some(30),
                    ..Default::default()
                },
                environment: EnvironmentAccommodations {
                    reduced_distractions: true,
                    ..Default::default()
                },
                ..Default::default()
            },
            assistive_technology: AssistiveTechnology {
                software: vec![
                    ATSoftware {
                        software_type: ATSoftwareType::Focus,
                        name: Some("Focus@Will".to_string()),
                        version: None,
                    },
                    ATSoftware {
                        software_type: ATSoftwareType::Organization,
                        name: Some("Notion".to_string()),
                        version: None,
                    },
                ],
                ..Default::default()
            },
            wia_integrations: WIAIntegrations::default(),
        }
    }

    /// Create a simulated motor impaired learner profile
    pub fn create_motor_impaired_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("Simulated Motor Impaired Learner".to_string()),
                preferred_language: Some("en".to_string()),
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::MotorImpairment],
                notes: Some("Uses switch access".to_string()),
            },
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences {
                input_method: InputMethodSettings {
                    primary: InputMethod::Switch,
                    keyboard_only: true,
                    switch_access: SwitchAccessSettings {
                        enabled: true,
                        num_switches: Some(2),
                        scan_speed_ms: 1500,
                    },
                    ..Default::default()
                },
                timing: TimingPreferences {
                    extended_time: true,
                    time_multiplier: 2.0,
                    disable_auto_advance: true,
                    ..Default::default()
                },
                click_settings: ClickSettings {
                    large_targets: true,
                    min_target_size_px: 64,
                    double_click_delay_ms: Some(1000),
                    sticky_keys: true,
                },
            },
            content_preferences: ContentPreferences::default(),
            learning_style: LearningStyle::default(),
            assessment_accommodations: AssessmentAccommodations {
                timing: AssessmentTimingAccommodations {
                    extended_time: true,
                    time_multiplier: 2.0,
                    breaks_allowed: true,
                    ..Default::default()
                },
                format: FormatAccommodations {
                    response_formats: vec![ResponseFormat::Typed, ResponseFormat::Verbal],
                    ..Default::default()
                },
                ..Default::default()
            },
            assistive_technology: AssistiveTechnology {
                devices: vec![ATDevice {
                    device_type: ATDeviceType::SwitchDevice,
                    name: Some("AbleNet Switch".to_string()),
                    version: None,
                    wia_device_id: None,
                }],
                ..Default::default()
            },
            wia_integrations: WIAIntegrations::default(),
        }
    }

    /// Create a simulated Korean deaf learner profile
    pub fn create_korean_deaf_learner(&self) -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo {
                display_name: Some("시뮬레이션 청각장애 학습자".to_string()),
                preferred_language: Some("ko".to_string()),
                secondary_languages: vec!["en".to_string()],
                ..Default::default()
            },
            disability_profile: DisabilityProfile {
                disclosed: true,
                disability_types: vec![DisabilityType::Deaf],
                notes: Some("한국수어(KSL) 사용자".to_string()),
            },
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences {
                captions: CaptionSettings {
                    required: true,
                    style: Some(CaptionStyle::Large),
                    language: Some("ko".to_string()),
                },
                transcripts: TranscriptSettings {
                    required: true,
                    interactive: true,
                },
                sign_language: SignLanguageSettings {
                    preferred: true,
                    language: Some(SignLanguageType::Ksl),
                },
                ..Default::default()
            },
            learning_style: LearningStyle {
                representation: RepresentationPreferences {
                    preferred_modalities: vec![LearningModality::Visual],
                    ..Default::default()
                },
                ..Default::default()
            },
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_blind_learner() {
        let sim = LearnerSimulator::new();
        let profile = sim.create_blind_learner();

        assert!(profile.display_preferences.screen_reader.enabled);
        assert!(profile.content_preferences.audio_description.required);
        assert!(profile.control_preferences.input_method.keyboard_only);
    }

    #[test]
    fn test_create_deaf_learner() {
        let sim = LearnerSimulator::new();
        let profile = sim.create_deaf_learner();

        assert!(profile.content_preferences.captions.required);
        assert!(profile.content_preferences.sign_language.preferred);
    }

    #[test]
    fn test_create_korean_deaf_learner() {
        let sim = LearnerSimulator::new();
        let profile = sim.create_korean_deaf_learner();

        assert_eq!(
            profile.content_preferences.sign_language.language,
            Some(SignLanguageType::Ksl)
        );
        assert_eq!(profile.learner_info.preferred_language, Some("ko".to_string()));
    }
}
