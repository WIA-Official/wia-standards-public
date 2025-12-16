//! Assessment Manager
//! 弘益人間 - Education for Everyone

use std::collections::HashMap;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::*;

/// Manages assessments and accommodations
#[derive(Debug, Default)]
pub struct AssessmentManager {
    assessments: HashMap<AssessmentId, Assessment>,
}

impl AssessmentManager {
    /// Create a new assessment manager
    pub fn new() -> Self {
        Self {
            assessments: HashMap::new(),
        }
    }

    /// Create a new assessment
    pub fn create_assessment(
        &mut self,
        title: String,
        assessment_type: AssessmentType,
    ) -> Assessment {
        let assessment = Assessment {
            assessment_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            title,
            description: None,
            assessment_type,
            created_at: Utc::now(),
            updated_at: None,
            assessment_info: AssessmentInfo::default(),
            timing: AssessmentTiming::default(),
            questions: Vec::new(),
            accessibility_settings: AssessmentAccessibilitySettings::default(),
            accommodations_available: AssessmentAccommodationsAvailable::default(),
            grading: GradingSettings::default(),
        };

        self.assessments.insert(assessment.assessment_id, assessment.clone());
        assessment
    }

    /// Get an assessment by ID
    pub fn get_assessment(&self, assessment_id: AssessmentId) -> Result<&Assessment> {
        self.assessments
            .get(&assessment_id)
            .ok_or(EduError::AssessmentNotFound(assessment_id))
    }

    /// Get a mutable assessment by ID
    pub fn get_assessment_mut(&mut self, assessment_id: AssessmentId) -> Result<&mut Assessment> {
        self.assessments
            .get_mut(&assessment_id)
            .ok_or(EduError::AssessmentNotFound(assessment_id))
    }

    /// Update an assessment
    pub fn update_assessment(&mut self, mut assessment: Assessment) -> Result<()> {
        if !self.assessments.contains_key(&assessment.assessment_id) {
            return Err(EduError::AssessmentNotFound(assessment.assessment_id));
        }
        assessment.updated_at = Some(Utc::now());
        self.assessments.insert(assessment.assessment_id, assessment);
        Ok(())
    }

    /// Delete an assessment
    pub fn delete_assessment(&mut self, assessment_id: AssessmentId) -> Result<Assessment> {
        self.assessments
            .remove(&assessment_id)
            .ok_or(EduError::AssessmentNotFound(assessment_id))
    }

    /// List all assessments
    pub fn list_assessments(&self) -> Vec<&Assessment> {
        self.assessments.values().collect()
    }

    /// Add a question to an assessment
    pub fn add_question(&mut self, assessment_id: AssessmentId, question: Question) -> Result<()> {
        let assessment = self.get_assessment_mut(assessment_id)?;
        assessment.questions.push(question);
        assessment.updated_at = Some(Utc::now());
        Ok(())
    }

    /// Create a simple multiple choice question
    pub fn create_multiple_choice_question(
        &self,
        text: String,
        options: Vec<(String, String)>, // (option_id, text)
        correct_answer: String,
        points: f32,
    ) -> Question {
        Question {
            question_id: Uuid::new_v4(),
            question_type: QuestionType::MultipleChoice,
            content: QuestionContent {
                text: text.clone(),
                plain_text: Some(text),
                ..Default::default()
            },
            points,
            required: true,
            options: options
                .into_iter()
                .map(|(id, text)| AnswerOption {
                    option_id: id,
                    text,
                    plain_text: None,
                    image: None,
                    feedback: None,
                })
                .collect(),
            correct_answers: vec![correct_answer],
            feedback: QuestionFeedback::default(),
            hints: Vec::new(),
            accessibility: QuestionAccessibility::default(),
            bloom_level: None,
            difficulty: None,
            tags: Vec::new(),
        }
    }

    /// Set time limit for assessment
    pub fn set_time_limit(&mut self, assessment_id: AssessmentId, minutes: u32) -> Result<()> {
        let assessment = self.get_assessment_mut(assessment_id)?;
        assessment.timing.time_limit_minutes = Some(minutes);
        assessment.updated_at = Some(Utc::now());
        Ok(())
    }

    /// Calculate extended time based on multiplier
    pub fn calculate_extended_time(base_minutes: u32, multiplier: f32) -> u32 {
        (base_minutes as f32 * multiplier).ceil() as u32
    }

    /// Get effective time limit for a learner based on accommodations
    pub fn get_effective_time_limit(
        &self,
        assessment_id: AssessmentId,
        profile: &LearnerProfile,
    ) -> Result<Option<u32>> {
        let assessment = self.get_assessment(assessment_id)?;

        let base_time = match assessment.timing.time_limit_minutes {
            Some(t) => t,
            None => return Ok(None), // Unlimited
        };

        // Check if learner has extended time accommodation
        if profile.assessment_accommodations.timing.unlimited_time {
            return Ok(None); // Unlimited
        }

        if profile.assessment_accommodations.timing.extended_time {
            let multiplier = profile.assessment_accommodations.timing.time_multiplier;

            // Check if the multiplier is available for this assessment
            if assessment.accommodations_available.timing.extended_time.available
                && assessment
                    .accommodations_available
                    .timing
                    .extended_time
                    .multipliers
                    .iter()
                    .any(|&m| (m - multiplier).abs() < 0.01)
            {
                return Ok(Some(Self::calculate_extended_time(base_time, multiplier)));
            }
        }

        Ok(Some(base_time))
    }

    /// Check if assessment supports an accommodation
    pub fn supports_accommodation(
        &self,
        assessment_id: AssessmentId,
        accommodation: &str,
    ) -> Result<bool> {
        let assessment = self.get_assessment(assessment_id)?;
        let available = &assessment.accommodations_available;

        let supported = match accommodation {
            "extended_time" => available.timing.extended_time.available,
            "unlimited_time" => available.timing.unlimited_time,
            "breaks" => available.setting.breaks.allowed,
            "large_print" => available.presentation.large_print,
            "high_contrast" => available.presentation.high_contrast,
            "screen_reader" => available.presentation.screen_reader,
            "text_to_speech" => available.presentation.text_to_speech,
            "braille" => available.presentation.braille,
            "sign_language" => available.presentation.sign_language_video,
            "speech_to_text" => available.response.speech_to_text,
            "spell_check" => available.response.spell_check,
            "calculator" => available.assistance.calculator.basic
                || available.assistance.calculator.scientific
                || available.assistance.calculator.graphing,
            "dictionary" => available.assistance.dictionary.standard
                || available.assistance.dictionary.bilingual,
            "reader" => available.assistance.reader,
            "separate_location" => available.setting.separate_location,
            _ => false,
        };

        Ok(supported)
    }

    /// Get all questions that require visual ability
    pub fn get_visual_questions(&self, assessment_id: AssessmentId) -> Result<Vec<&Question>> {
        let assessment = self.get_assessment(assessment_id)?;
        Ok(assessment
            .questions
            .iter()
            .filter(|q| q.accessibility.requires_vision)
            .collect())
    }

    /// Check if assessment is fully accessible
    pub fn is_fully_accessible(&self, assessment_id: AssessmentId) -> Result<bool> {
        let assessment = self.get_assessment(assessment_id)?;

        // Check basic accessibility settings
        if !assessment.accessibility_settings.keyboard_navigable
            || !assessment.accessibility_settings.screen_reader_compatible
        {
            return Ok(false);
        }

        // Check for questions with visual/hearing/motor requirements without alternatives
        for question in &assessment.questions {
            let access = &question.accessibility;
            if (access.requires_vision || access.requires_hearing || access.requires_fine_motor)
                && !access.alternative_available
            {
                return Ok(false);
            }
        }

        Ok(true)
    }

    /// Export assessment to JSON
    pub fn export_assessment(&self, assessment_id: AssessmentId) -> Result<String> {
        let assessment = self.get_assessment(assessment_id)?;
        serde_json::to_string_pretty(assessment).map_err(EduError::from)
    }

    /// Import assessment from JSON
    pub fn import_assessment(&mut self, json: &str) -> Result<Assessment> {
        let mut assessment: Assessment = serde_json::from_str(json)?;
        assessment.assessment_id = Uuid::new_v4();
        assessment.created_at = Utc::now();
        assessment.updated_at = None;
        self.assessments.insert(assessment.assessment_id, assessment.clone());
        Ok(assessment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_assessment() {
        let mut manager = AssessmentManager::new();
        let assessment = manager.create_assessment(
            "Midterm Exam".to_string(),
            AssessmentType::Exam,
        );

        assert!(!assessment.assessment_id.is_nil());
        assert_eq!(assessment.title, "Midterm Exam");
        assert_eq!(assessment.assessment_type, AssessmentType::Exam);
    }

    #[test]
    fn test_add_question() {
        let mut manager = AssessmentManager::new();
        let assessment = manager.create_assessment(
            "Quiz".to_string(),
            AssessmentType::Quiz,
        );

        let question = manager.create_multiple_choice_question(
            "What is 2 + 2?".to_string(),
            vec![
                ("a".to_string(), "3".to_string()),
                ("b".to_string(), "4".to_string()),
                ("c".to_string(), "5".to_string()),
            ],
            "b".to_string(),
            10.0,
        );

        manager.add_question(assessment.assessment_id, question).unwrap();

        let updated = manager.get_assessment(assessment.assessment_id).unwrap();
        assert_eq!(updated.questions.len(), 1);
    }

    #[test]
    fn test_extended_time_calculation() {
        assert_eq!(AssessmentManager::calculate_extended_time(60, 1.5), 90);
        assert_eq!(AssessmentManager::calculate_extended_time(45, 2.0), 90);
    }

    #[test]
    fn test_effective_time_with_extended_time() {
        let mut manager = AssessmentManager::new();
        let assessment = manager.create_assessment(
            "Timed Quiz".to_string(),
            AssessmentType::Quiz,
        );
        manager.set_time_limit(assessment.assessment_id, 60).unwrap();

        let mut profile = LearnerProfile {
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
        };

        profile.assessment_accommodations.timing.extended_time = true;
        profile.assessment_accommodations.timing.time_multiplier = 1.5;

        let effective_time = manager.get_effective_time_limit(
            assessment.assessment_id,
            &profile,
        ).unwrap();

        assert_eq!(effective_time, Some(90));
    }
}
