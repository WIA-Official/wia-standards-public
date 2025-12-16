//! xAPI Statement Generator
//! 弘益人間 - Generate accessibility-related learning statements

use uuid::Uuid;
use std::collections::HashMap;
use serde_json::json;

use crate::types::{LearnerProfile, Course, Assessment};
use super::statements::{
    XapiStatement, XapiActor, XapiObject, XapiResult, XapiContext,
    Agent, Activity, ActivityDefinition, Score,
};
use super::verbs::{AccessibilityVerb, AccessibilityActivityType, AccessibilityExtension};

/// Statement Generator for accessibility learning activities
pub struct StatementGenerator {
    /// Platform identifier
    platform: String,
    /// Default language
    language: String,
}

impl StatementGenerator {
    /// Create a new statement generator
    pub fn new(platform: &str) -> Self {
        Self {
            platform: platform.to_string(),
            language: "en-US".to_string(),
        }
    }

    /// Set default language
    pub fn with_language(mut self, language: &str) -> Self {
        self.language = language.to_string();
        self
    }

    /// Create actor from learner profile
    pub fn create_actor(&self, profile: &LearnerProfile) -> XapiActor {
        XapiActor::Agent(Agent::with_account(
            &self.platform,
            &profile.profile_id.to_string(),
            profile.learner_info.display_name.clone(),
        ))
    }

    /// Create actor from email
    pub fn create_actor_from_email(&self, email: &str, name: Option<String>) -> XapiActor {
        XapiActor::Agent(Agent::with_email(email, name))
    }

    // ========================================================================
    // Course Activity Statements
    // ========================================================================

    /// Generate statement for launching a course
    pub fn course_launched(
        &self,
        profile: &LearnerProfile,
        course: &Course,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::launched();
        let object = self.create_course_activity(course);

        let context = self.create_accessibility_context(profile, None);

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for completing a course
    pub fn course_completed(
        &self,
        profile: &LearnerProfile,
        course: &Course,
        success: bool,
        score: Option<f64>,
        duration: Option<&str>,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = if success {
            AccessibilityVerb::passed()
        } else {
            AccessibilityVerb::completed()
        };
        let object = self.create_course_activity(course);

        let mut result = XapiResult::completed(success);
        if let Some(s) = score {
            result = result.with_score(Score::scaled(s));
        }
        if let Some(d) = duration {
            result = result.with_duration(d);
        }

        let context = self.create_accessibility_context(profile, None);

        XapiStatement::new(actor, verb, object)
            .with_result(result)
            .with_context(context)
    }

    // ========================================================================
    // Assessment Statements
    // ========================================================================

    /// Generate statement for starting an assessment
    pub fn assessment_started(
        &self,
        profile: &LearnerProfile,
        assessment: &Assessment,
        accommodations_applied: Vec<String>,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::attempted();
        let object = self.create_assessment_activity(assessment);

        let mut extensions = HashMap::new();
        if !accommodations_applied.is_empty() {
            extensions.insert(
                AccessibilityExtension::features_used(),
                json!(accommodations_applied),
            );
        }
        if profile.assessment_accommodations.timing.extended_time {
            extensions.insert(
                AccessibilityExtension::time_multiplier(),
                json!(profile.assessment_accommodations.timing.time_multiplier),
            );
        }

        let mut context = self.create_accessibility_context(profile, None);
        if !extensions.is_empty() {
            context.extensions = Some(extensions);
        }

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for completing an assessment
    pub fn assessment_completed(
        &self,
        profile: &LearnerProfile,
        assessment: &Assessment,
        score: f64,
        passed: bool,
        duration: &str,
        used_extended_time: bool,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = if passed {
            AccessibilityVerb::passed()
        } else {
            AccessibilityVerb::failed()
        };
        let object = self.create_assessment_activity(assessment);

        let result = XapiResult::completed(passed)
            .with_score(Score::scaled(score))
            .with_duration(duration);

        let mut extensions = HashMap::new();
        if used_extended_time {
            extensions.insert(
                AccessibilityExtension::time_multiplier(),
                json!(profile.assessment_accommodations.timing.time_multiplier),
            );
        }

        let mut context = self.create_accessibility_context(profile, None);
        if !extensions.is_empty() {
            context.extensions = Some(extensions);
        }

        XapiStatement::new(actor, verb, object)
            .with_result(result)
            .with_context(context)
    }

    // ========================================================================
    // Accessibility Feature Statements
    // ========================================================================

    /// Generate statement for enabling an accessibility feature
    pub fn accessibility_feature_enabled(
        &self,
        profile: &LearnerProfile,
        feature_name: &str,
        feature_value: serde_json::Value,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::enabled_accessibility();

        let object = XapiObject::Activity(
            Activity::new(&format!(
                "{}/setting/{}",
                AccessibilityActivityType::NAMESPACE,
                feature_name.to_lowercase().replace(' ', "-")
            ))
            .with_definition(
                ActivityDefinition::with_type(&AccessibilityActivityType::accessibility_setting())
                    .with_name(&self.language, feature_name)
            )
        );

        let mut extensions = HashMap::new();
        extensions.insert(
            format!("{}/{}", AccessibilityExtension::NAMESPACE, feature_name.to_lowercase().replace(' ', "-")),
            feature_value,
        );

        let mut context = self.create_accessibility_context(profile, None);
        context.extensions = Some(extensions);

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for enabling captions
    pub fn captions_enabled(
        &self,
        profile: &LearnerProfile,
        content_id: &str,
        language: Option<&str>,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::enabled_captions();

        let object = XapiObject::Activity(
            Activity::new(&format!("{}/captions/{}", AccessibilityActivityType::NAMESPACE, content_id))
                .with_definition(
                    ActivityDefinition::with_type(&AccessibilityActivityType::caption())
                        .with_name(&self.language, "Captions")
                )
        );

        let mut extensions = HashMap::new();
        if let Some(lang) = language {
            extensions.insert(
                format!("{}/caption-language", AccessibilityExtension::NAMESPACE),
                json!(lang),
            );
        }

        let mut context = self.create_accessibility_context(profile, None);
        if !extensions.is_empty() {
            context.extensions = Some(extensions);
        }

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for enabling audio description
    pub fn audio_description_enabled(
        &self,
        profile: &LearnerProfile,
        content_id: &str,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::enabled_audio_description();

        let object = XapiObject::Activity(
            Activity::new(&format!("{}/audio-description/{}", AccessibilityActivityType::NAMESPACE, content_id))
                .with_definition(
                    ActivityDefinition::with_type(&AccessibilityActivityType::audio_description())
                        .with_name(&self.language, "Audio Description")
                )
        );

        let context = self.create_accessibility_context(profile, None);

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for using extended time
    pub fn extended_time_used(
        &self,
        profile: &LearnerProfile,
        assessment: &Assessment,
        original_time_minutes: u32,
        actual_time_minutes: u32,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::used_extended_time();
        let object = self.create_assessment_activity(assessment);

        let mut extensions = HashMap::new();
        extensions.insert(
            AccessibilityExtension::time_multiplier(),
            json!(profile.assessment_accommodations.timing.time_multiplier),
        );
        extensions.insert(
            format!("{}/original-time-minutes", AccessibilityExtension::NAMESPACE),
            json!(original_time_minutes),
        );
        extensions.insert(
            format!("{}/actual-time-minutes", AccessibilityExtension::NAMESPACE),
            json!(actual_time_minutes),
        );

        let mut context = self.create_accessibility_context(profile, None);
        context.extensions = Some(extensions);

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for taking a break
    pub fn break_taken(
        &self,
        profile: &LearnerProfile,
        activity_id: &str,
        break_duration: &str,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::took_break();

        let object = XapiObject::Activity(
            Activity::new(activity_id)
        );

        let result = XapiResult {
            duration: Some(break_duration.to_string()),
            ..Default::default()
        };

        let context = self.create_accessibility_context(profile, None);

        XapiStatement::new(actor, verb, object)
            .with_result(result)
            .with_context(context)
    }

    // ========================================================================
    // Profile Statements
    // ========================================================================

    /// Generate statement for updating profile
    pub fn profile_updated(
        &self,
        profile: &LearnerProfile,
        changed_settings: Vec<String>,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::updated_profile();

        let object = XapiObject::Activity(
            Activity::new(&format!(
                "{}/profile/{}",
                AccessibilityActivityType::NAMESPACE,
                profile.profile_id
            ))
            .with_definition(
                ActivityDefinition::with_type(&AccessibilityActivityType::accessibility_profile())
                    .with_name(&self.language, "Accessibility Profile")
            )
        );

        let mut extensions = HashMap::new();
        extensions.insert(
            format!("{}/changed-settings", AccessibilityExtension::NAMESPACE),
            json!(changed_settings),
        );
        extensions.insert(
            AccessibilityExtension::profile_version(),
            json!(profile.schema_version),
        );

        let context = XapiContext::default()
            .with_platform(&self.platform)
            .with_language(&self.language)
            .with_extensions(extensions);

        XapiStatement::new(actor, verb, object)
            .with_context(context)
    }

    /// Generate statement for syncing profile
    pub fn profile_synced(
        &self,
        profile: &LearnerProfile,
        target_platform: &str,
        success: bool,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::synced_profile();

        let object = XapiObject::Activity(
            Activity::new(&format!(
                "{}/profile/{}",
                AccessibilityActivityType::NAMESPACE,
                profile.profile_id
            ))
            .with_definition(
                ActivityDefinition::with_type(&AccessibilityActivityType::accessibility_profile())
            )
        );

        let result = XapiResult {
            success: Some(success),
            completion: Some(true),
            ..Default::default()
        };

        let mut extensions = HashMap::new();
        extensions.insert(
            format!("{}/target-platform", AccessibilityExtension::NAMESPACE),
            json!(target_platform),
        );

        let context = XapiContext::default()
            .with_platform(&self.platform)
            .with_language(&self.language)
            .with_extensions(extensions);

        XapiStatement::new(actor, verb, object)
            .with_result(result)
            .with_context(context)
    }

    // ========================================================================
    // Barrier Reporting
    // ========================================================================

    /// Generate statement for reporting an accessibility barrier
    pub fn barrier_reported(
        &self,
        profile: &LearnerProfile,
        content_id: &str,
        barrier_type: &str,
        description: &str,
    ) -> XapiStatement {
        let actor = self.create_actor(profile);
        let verb = AccessibilityVerb::reported_issue();

        let object = XapiObject::Activity(
            Activity::new(content_id)
        );

        let result = XapiResult {
            response: Some(description.to_string()),
            ..Default::default()
        };

        let mut extensions = HashMap::new();
        extensions.insert(
            AccessibilityExtension::barrier_type(),
            json!(barrier_type),
        );

        let mut context = self.create_accessibility_context(profile, None);
        context.extensions = Some(extensions);

        XapiStatement::new(actor, verb, object)
            .with_result(result)
            .with_context(context)
    }

    // ========================================================================
    // Helper Methods
    // ========================================================================

    /// Create course activity object
    fn create_course_activity(&self, course: &Course) -> XapiObject {
        XapiObject::Activity(
            Activity::new(&format!(
                "{}/course/{}",
                AccessibilityActivityType::NAMESPACE,
                course.course_id
            ))
            .with_definition(
                ActivityDefinition::with_type(&AccessibilityActivityType::accessible_course())
                    .with_name(&self.language, &course.title)
                    .with_description(&self.language, course.description.as_deref().unwrap_or(""))
            )
        )
    }

    /// Create assessment activity object
    fn create_assessment_activity(&self, assessment: &Assessment) -> XapiObject {
        XapiObject::Activity(
            Activity::new(&format!(
                "{}/assessment/{}",
                AccessibilityActivityType::NAMESPACE,
                assessment.assessment_id
            ))
            .with_definition(
                ActivityDefinition::with_type(&AccessibilityActivityType::accessible_assessment())
                    .with_name(&self.language, &assessment.title)
            )
        )
    }

    /// Create accessibility context with profile info
    fn create_accessibility_context(
        &self,
        profile: &LearnerProfile,
        registration: Option<Uuid>,
    ) -> XapiContext {
        let mut extensions = HashMap::new();

        // Add disability types if disclosed
        if profile.disability_profile.disclosed && !profile.disability_profile.disability_types.is_empty() {
            let types: Vec<String> = profile.disability_profile.disability_types
                .iter()
                .map(|t| format!("{:?}", t).to_lowercase())
                .collect();
            extensions.insert(
                AccessibilityExtension::disability_type(),
                json!(types),
            );
        }

        // Add key accessibility settings
        let mut features = Vec::new();
        if profile.display_preferences.screen_reader.enabled {
            features.push("screen_reader");
        }
        if profile.content_preferences.captions.required {
            features.push("captions");
        }
        if profile.content_preferences.audio_description.required {
            features.push("audio_description");
        }
        if profile.assessment_accommodations.timing.extended_time {
            features.push("extended_time");
        }

        if !features.is_empty() {
            extensions.insert(
                AccessibilityExtension::features_used(),
                json!(features),
            );
        }

        let mut context = XapiContext::default()
            .with_platform(&self.platform)
            .with_language(&self.language);

        if let Some(reg) = registration {
            context = context.with_registration(reg);
        }

        if !extensions.is_empty() {
            context = context.with_extensions(extensions);
        }

        context
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileManager;
    use crate::types::DisabilityType;
    use chrono::Utc;

    fn create_test_profile() -> LearnerProfile {
        let mut manager = ProfileManager::new();
        let mut profile = manager.create_profile_for_disability(DisabilityType::Blind);
        profile.profile_id = Uuid::new_v4();
        profile
    }

    fn create_test_course() -> Course {
        Course {
            course_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            title: "Accessible Learning 101".to_string(),
            description: Some("Learn about accessibility".to_string()),
            created_at: Utc::now(),
            updated_at: None,
            course_info: Default::default(),
            accessibility_statement: Default::default(),
            modules: vec![],
            learning_outcomes: vec![],
            accessibility_features: Default::default(),
            accommodations_available: Default::default(),
        }
    }

    fn create_test_assessment() -> Assessment {
        Assessment {
            assessment_id: Uuid::new_v4(),
            schema_version: "1.0.0".to_string(),
            title: "Accessibility Knowledge Test".to_string(),
            description: None,
            assessment_type: crate::types::AssessmentType::Quiz,
            created_at: Utc::now(),
            updated_at: None,
            assessment_info: Default::default(),
            timing: Default::default(),
            questions: vec![],
            accessibility_settings: Default::default(),
            accommodations_available: Default::default(),
            grading: Default::default(),
        }
    }

    #[test]
    fn test_create_generator() {
        let generator = StatementGenerator::new("https://lms.example.com");
        assert_eq!(generator.platform, "https://lms.example.com");
    }

    #[test]
    fn test_course_launched_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();
        let course = create_test_course();

        let statement = generator.course_launched(&profile, &course);

        assert!(statement.id.is_some());
        assert!(matches!(statement.object, XapiObject::Activity(_)));
    }

    #[test]
    fn test_course_completed_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();
        let course = create_test_course();

        let statement = generator.course_completed(
            &profile,
            &course,
            true,
            Some(0.85),
            Some("PT2H30M"),
        );

        assert!(statement.result.is_some());
        let result = statement.result.unwrap();
        assert_eq!(result.success, Some(true));
        assert!(result.score.is_some());
    }

    #[test]
    fn test_assessment_started_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();
        let assessment = create_test_assessment();

        let statement = generator.assessment_started(
            &profile,
            &assessment,
            vec!["extended_time".to_string(), "screen_reader".to_string()],
        );

        assert!(statement.context.is_some());
    }

    #[test]
    fn test_captions_enabled_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();

        let statement = generator.captions_enabled(
            &profile,
            "video-123",
            Some("en-US"),
        );

        assert_eq!(statement.verb.id, format!("{}/enabled-captions", AccessibilityVerb::NAMESPACE));
    }

    #[test]
    fn test_profile_updated_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();

        let statement = generator.profile_updated(
            &profile,
            vec!["screen_reader".to_string(), "speech_rate".to_string()],
        );

        assert!(statement.context.is_some());
        let context = statement.context.unwrap();
        assert!(context.extensions.is_some());
    }

    #[test]
    fn test_barrier_reported_statement() {
        let generator = StatementGenerator::new("https://lms.example.com");
        let profile = create_test_profile();

        let statement = generator.barrier_reported(
            &profile,
            "https://example.com/content/123",
            "missing_captions",
            "Video has no captions available",
        );

        assert!(statement.result.is_some());
        let result = statement.result.unwrap();
        assert!(result.response.is_some());
    }
}
