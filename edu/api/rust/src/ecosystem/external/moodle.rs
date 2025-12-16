//! Moodle LMS Integration
//! 弘益人間 - Integration with Moodle Learning Management System

use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::LearnerProfile;
use super::{
    LMSAdapter, LMSConfig, LMSPlatform, LMSCourse, LMSAccommodations,
    SyncStatus, AssessmentSubmission, SubmissionResult, EnrollmentStatus,
};

/// Moodle LMS Adapter
pub struct MoodleAdapter {
    /// Configuration
    config: LMSConfig,
    /// Connection status
    connected: bool,
    /// Web service token
    ws_token: Option<String>,
}

impl MoodleAdapter {
    /// Create a new Moodle adapter
    pub fn new(config: LMSConfig) -> Self {
        let ws_token = config.api_key.clone();
        Self {
            config,
            connected: true,
            ws_token,
        }
    }

    /// Create with base URL and token
    pub fn with_token(base_url: &str, token: &str) -> Self {
        Self::new(LMSConfig::moodle(base_url, token))
    }

    /// Build web service URL
    fn ws_url(&self, function: &str) -> String {
        format!(
            "{}/webservice/rest/server.php?wstoken={}&wsfunction={}&moodlewsrestformat=json",
            self.config.base_url,
            self.ws_token.as_deref().unwrap_or(""),
            function
        )
    }
}

#[async_trait]
impl LMSAdapter for MoodleAdapter {
    fn platform(&self) -> LMSPlatform {
        LMSPlatform::Moodle
    }

    async fn sync_profile(&self, profile: &LearnerProfile) -> Result<SyncStatus> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        // In real implementation, would call Moodle Web Services
        // core_user_update_users with custom profile fields

        Ok(SyncStatus {
            success: true,
            profile_id: profile.profile_id,
            remote_id: Some(format!("moodle_user_{}", profile.profile_id)),
            synced_at: Utc::now(),
            fields_synced: vec![
                "accessibility_profile".to_string(),
            ],
            error: None,
        })
    }

    async fn get_courses(&self, user_id: &str) -> Result<Vec<LMSCourse>> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        // core_enrol_get_users_courses

        Ok(vec![
            LMSCourse {
                id: "moodle_course_1".to_string(),
                code: Some("MATH101".to_string()),
                name: "Mathematics 101".to_string(),
                description: Some("Introduction to mathematics".to_string()),
                start_date: Some(Utc::now()),
                end_date: None,
                enrollment_status: EnrollmentStatus::Active,
                accessibility_features: vec![
                    "alt_text".to_string(),
                ],
            },
        ])
    }

    async fn get_course(&self, course_id: &str) -> Result<LMSCourse> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        // core_course_get_courses

        Ok(LMSCourse {
            id: course_id.to_string(),
            code: Some("MATH101".to_string()),
            name: "Mathematics 101".to_string(),
            description: Some("Introduction to mathematics".to_string()),
            start_date: Some(Utc::now()),
            end_date: None,
            enrollment_status: EnrollmentStatus::Active,
            accessibility_features: vec![],
        })
    }

    async fn submit_assessment(&self, submission: &AssessmentSubmission) -> Result<SubmissionResult> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        // mod_assign_submit_for_grading

        Ok(SubmissionResult {
            success: true,
            submission_id: Some(format!("moodle_sub_{}", Uuid::new_v4())),
            grade: None,
            feedback: None,
            error: None,
        })
    }

    async fn get_accommodations(&self, user_id: &str) -> Result<LMSAccommodations> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        Ok(LMSAccommodations::default())
    }

    async fn set_accommodations(&self, user_id: &str, accommodations: &LMSAccommodations) -> Result<()> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Moodle".to_string()));
        }

        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_moodle_adapter() {
        let adapter = MoodleAdapter::with_token("https://moodle.example.com", "test-token");
        assert!(adapter.is_connected());
        assert_eq!(adapter.platform(), LMSPlatform::Moodle);
    }

    #[tokio::test]
    async fn test_get_courses() {
        let adapter = MoodleAdapter::with_token("https://moodle.example.com", "test-token");
        let courses = adapter.get_courses("user123").await.unwrap();
        assert!(!courses.is_empty());
    }
}
