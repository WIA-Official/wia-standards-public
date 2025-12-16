//! Validation utilities
//! 弘益人間 - Education for Everyone

use crate::error::{EduError, Result};
use crate::types::*;

/// Validator for education data structures
pub struct Validator;

impl Validator {
    /// Validate a learner profile
    pub fn validate_profile(profile: &LearnerProfile) -> Result<()> {
        // Validate schema version
        if profile.schema_version != "1.0.0" {
            return Err(EduError::VersionMismatch {
                expected: "1.0.0".to_string(),
                actual: profile.schema_version.clone(),
            });
        }

        // Validate magnification level
        if profile.display_preferences.magnification.level < 1.0
            || profile.display_preferences.magnification.level > 10.0
        {
            return Err(EduError::ValidationError(
                "Magnification level must be between 1.0 and 10.0".to_string(),
            ));
        }

        // Validate speech rate
        if profile.display_preferences.screen_reader.speech_rate < 0.5
            || profile.display_preferences.screen_reader.speech_rate > 3.0
        {
            return Err(EduError::ValidationError(
                "Speech rate must be between 0.5 and 3.0".to_string(),
            ));
        }

        // Validate time multiplier
        if profile.assessment_accommodations.timing.time_multiplier < 1.0
            || profile.assessment_accommodations.timing.time_multiplier > 5.0
        {
            return Err(EduError::ValidationError(
                "Time multiplier must be between 1.0 and 5.0".to_string(),
            ));
        }

        // Validate dwell time
        if profile.control_preferences.input_method.eye_gaze.enabled {
            let dwell = profile.control_preferences.input_method.eye_gaze.dwell_time_ms;
            if dwell < 200 || dwell > 5000 {
                return Err(EduError::ValidationError(
                    "Eye gaze dwell time must be between 200ms and 5000ms".to_string(),
                ));
            }
        }

        // Validate target size
        if profile.control_preferences.click_settings.min_target_size_px < 24
            || profile.control_preferences.click_settings.min_target_size_px > 100
        {
            return Err(EduError::ValidationError(
                "Minimum target size must be between 24px and 100px".to_string(),
            ));
        }

        Ok(())
    }

    /// Validate a course
    pub fn validate_course(course: &Course) -> Result<()> {
        // Validate schema version
        if course.schema_version != "1.0.0" {
            return Err(EduError::VersionMismatch {
                expected: "1.0.0".to_string(),
                actual: course.schema_version.clone(),
            });
        }

        // Validate title is not empty
        if course.title.is_empty() {
            return Err(EduError::ValidationError(
                "Course title cannot be empty".to_string(),
            ));
        }

        // Validate module sequences
        for (i, module) in course.modules.iter().enumerate() {
            if module.title.is_empty() {
                return Err(EduError::ValidationError(format!(
                    "Module {} title cannot be empty",
                    i + 1
                )));
            }
        }

        Ok(())
    }

    /// Validate an assessment
    pub fn validate_assessment(assessment: &Assessment) -> Result<()> {
        // Validate schema version
        if assessment.schema_version != "1.0.0" {
            return Err(EduError::VersionMismatch {
                expected: "1.0.0".to_string(),
                actual: assessment.schema_version.clone(),
            });
        }

        // Validate title is not empty
        if assessment.title.is_empty() {
            return Err(EduError::ValidationError(
                "Assessment title cannot be empty".to_string(),
            ));
        }

        // Validate questions
        for question in &assessment.questions {
            Self::validate_question(question)?;
        }

        // Validate grading settings
        if let Some(passing_pct) = assessment.grading.passing_percentage {
            if passing_pct < 0.0 || passing_pct > 100.0 {
                return Err(EduError::ValidationError(
                    "Passing percentage must be between 0 and 100".to_string(),
                ));
            }
        }

        Ok(())
    }

    /// Validate a question
    pub fn validate_question(question: &Question) -> Result<()> {
        // Validate content
        if question.content.text.is_empty() {
            return Err(EduError::ValidationError(
                "Question text cannot be empty".to_string(),
            ));
        }

        // Validate points
        if question.points < 0.0 {
            return Err(EduError::ValidationError(
                "Question points cannot be negative".to_string(),
            ));
        }

        // Validate options for multiple choice
        match question.question_type {
            QuestionType::MultipleChoice | QuestionType::MultipleSelect => {
                if question.options.is_empty() {
                    return Err(EduError::ValidationError(
                        "Multiple choice questions must have options".to_string(),
                    ));
                }
                if question.correct_answers.is_empty() {
                    return Err(EduError::ValidationError(
                        "Multiple choice questions must have correct answers".to_string(),
                    ));
                }
            }
            QuestionType::TrueFalse => {
                if question.options.len() != 2 {
                    return Err(EduError::ValidationError(
                        "True/false questions must have exactly 2 options".to_string(),
                    ));
                }
            }
            _ => {}
        }

        Ok(())
    }

    /// Check WCAG conformance for content
    pub fn check_wcag_conformance(content: &ContentItem) -> Vec<WCAGIssue> {
        let mut issues = Vec::new();
        let meta = &content.accessibility_metadata;

        // WCAG 1.1.1 - Non-text Content
        if content.content_type == ContentType::Image && !meta.alt_text_provided {
            issues.push(WCAGIssue {
                criterion: "1.1.1".to_string(),
                level: WCAGLevel::A,
                description: "Image is missing alternative text".to_string(),
                severity: IssueSeverity::Serious,
            });
        }

        // WCAG 1.2.2 - Captions (Prerecorded)
        if content.content_type == ContentType::Video && !meta.has_captions {
            issues.push(WCAGIssue {
                criterion: "1.2.2".to_string(),
                level: WCAGLevel::A,
                description: "Video is missing captions".to_string(),
                severity: IssueSeverity::Serious,
            });
        }

        // WCAG 1.2.3 - Audio Description
        if content.content_type == ContentType::Video && !meta.has_audio_description {
            issues.push(WCAGIssue {
                criterion: "1.2.3".to_string(),
                level: WCAGLevel::A,
                description: "Video is missing audio description".to_string(),
                severity: IssueSeverity::Moderate,
            });
        }

        // WCAG 1.2.1 - Audio-only and Video-only
        if content.content_type == ContentType::Audio && !meta.has_transcript {
            issues.push(WCAGIssue {
                criterion: "1.2.1".to_string(),
                level: WCAGLevel::A,
                description: "Audio content is missing transcript".to_string(),
                severity: IssueSeverity::Serious,
            });
        }

        // WCAG 2.1.1 - Keyboard
        if content.content_type == ContentType::Interactive && !meta.keyboard_accessible {
            issues.push(WCAGIssue {
                criterion: "2.1.1".to_string(),
                level: WCAGLevel::A,
                description: "Interactive content is not keyboard accessible".to_string(),
                severity: IssueSeverity::Critical,
            });
        }

        // WCAG 2.3.1 - Three Flashes
        if meta.hazards.contains(&ContentHazard::Flashing) && !meta.no_flashing {
            issues.push(WCAGIssue {
                criterion: "2.3.1".to_string(),
                level: WCAGLevel::A,
                description: "Content may contain flashing that could cause seizures".to_string(),
                severity: IssueSeverity::Critical,
            });
        }

        // WCAG 4.1.2 - Name, Role, Value
        if !meta.screen_reader_compatible {
            issues.push(WCAGIssue {
                criterion: "4.1.2".to_string(),
                level: WCAGLevel::A,
                description: "Content may not be fully compatible with screen readers".to_string(),
                severity: IssueSeverity::Serious,
            });
        }

        issues
    }
}

/// WCAG compliance issue
#[derive(Debug, Clone)]
pub struct WCAGIssue {
    pub criterion: String,
    pub level: WCAGLevel,
    pub description: String,
    pub severity: IssueSeverity,
}

/// Issue severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssueSeverity {
    Minor,
    Moderate,
    Serious,
    Critical,
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use uuid::Uuid;

    fn create_test_profile() -> LearnerProfile {
        LearnerProfile {
            profile_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            created_at: Utc::now(),
            updated_at: None,
            learner_info: LearnerInfo::default(),
            disability_profile: DisabilityProfile::default(),
            display_preferences: DisplayPreferences::default(),
            control_preferences: ControlPreferences::default(),
            content_preferences: ContentPreferences::default(),
            learning_style: LearningStyle::default(),
            assessment_accommodations: AssessmentAccommodations::default(),
            assistive_technology: AssistiveTechnology::default(),
            wia_integrations: WIAIntegrations::default(),
        }
    }

    #[test]
    fn test_validate_valid_profile() {
        let profile = create_test_profile();
        assert!(Validator::validate_profile(&profile).is_ok());
    }

    #[test]
    fn test_validate_invalid_magnification() {
        let mut profile = create_test_profile();
        profile.display_preferences.magnification.level = 15.0;
        assert!(Validator::validate_profile(&profile).is_err());
    }

    #[test]
    fn test_validate_invalid_time_multiplier() {
        let mut profile = create_test_profile();
        profile.assessment_accommodations.timing.time_multiplier = 10.0;
        assert!(Validator::validate_profile(&profile).is_err());
    }

    #[test]
    fn test_wcag_check_missing_captions() {
        let content = ContentItem {
            content_id: Uuid::new_v4(),
            content_type: ContentType::Video,
            title: "Test Video".to_string(),
            description: None,
            sequence: Some(1),
            required: true,
            duration_minutes: Some(10),
            url: None,
            accessibility_metadata: ContentAccessibilityMetadata {
                has_captions: false,
                ..Default::default()
            },
            alternatives: Vec::new(),
        };

        let issues = Validator::check_wcag_conformance(&content);
        assert!(issues.iter().any(|i| i.criterion == "1.2.2"));
    }
}
