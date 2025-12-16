//! Canvas LMS Integration
//! 弘益人間 - Integration with Instructure Canvas

use async_trait::async_trait;
use chrono::Utc;
use uuid::Uuid;

use crate::error::{EduError, Result};
use crate::types::LearnerProfile;
use super::{
    LMSAdapter, LMSConfig, LMSPlatform, LMSCourse, LMSAccommodations,
    SyncStatus, AssessmentSubmission, SubmissionResult, EnrollmentStatus,
};

/// Canvas LMS Adapter
pub struct CanvasAdapter {
    /// Configuration
    config: LMSConfig,
    /// Connection status
    connected: bool,
}

impl CanvasAdapter {
    /// Create a new Canvas adapter
    pub fn new(config: LMSConfig) -> Self {
        Self {
            config,
            connected: true,
        }
    }

    /// Create with base URL and API key
    pub fn with_api_key(base_url: &str, api_key: &str) -> Self {
        Self::new(LMSConfig::canvas(base_url, api_key))
    }

    /// Get API key
    fn api_key(&self) -> Option<&str> {
        self.config.api_key.as_deref()
    }

    /// Build API URL
    fn api_url(&self, endpoint: &str) -> String {
        format!("{}/api/v1{}", self.config.base_url, endpoint)
    }
}

#[async_trait]
impl LMSAdapter for CanvasAdapter {
    fn platform(&self) -> LMSPlatform {
        LMSPlatform::Canvas
    }

    async fn sync_profile(&self, profile: &LearnerProfile) -> Result<SyncStatus> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // In real implementation, would call Canvas API
        // PUT /api/v1/users/:user_id/custom_data/wia_accessibility

        let accommodations = LMSAccommodations::from_profile(profile);

        Ok(SyncStatus {
            success: true,
            profile_id: profile.profile_id,
            remote_id: Some(format!("canvas_user_{}", profile.profile_id)),
            synced_at: Utc::now(),
            fields_synced: vec![
                "display_preferences".to_string(),
                "content_preferences".to_string(),
                "assessment_accommodations".to_string(),
            ],
            error: None,
        })
    }

    async fn get_courses(&self, user_id: &str) -> Result<Vec<LMSCourse>> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // In real implementation, would call Canvas API
        // GET /api/v1/users/:user_id/courses

        // Return mock data for testing
        Ok(vec![
            LMSCourse {
                id: "course_1".to_string(),
                code: Some("CS101".to_string()),
                name: "Introduction to Computer Science".to_string(),
                description: Some("Learn programming fundamentals".to_string()),
                start_date: Some(Utc::now()),
                end_date: None,
                enrollment_status: EnrollmentStatus::Active,
                accessibility_features: vec![
                    "captions".to_string(),
                    "transcripts".to_string(),
                ],
            },
        ])
    }

    async fn get_course(&self, course_id: &str) -> Result<LMSCourse> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // GET /api/v1/courses/:course_id

        Ok(LMSCourse {
            id: course_id.to_string(),
            code: Some("CS101".to_string()),
            name: "Introduction to Computer Science".to_string(),
            description: Some("Learn programming fundamentals".to_string()),
            start_date: Some(Utc::now()),
            end_date: None,
            enrollment_status: EnrollmentStatus::Active,
            accessibility_features: vec![
                "captions".to_string(),
                "transcripts".to_string(),
                "screen_reader_compatible".to_string(),
            ],
        })
    }

    async fn submit_assessment(&self, submission: &AssessmentSubmission) -> Result<SubmissionResult> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // POST /api/v1/courses/:course_id/assignments/:assignment_id/submissions

        Ok(SubmissionResult {
            success: true,
            submission_id: Some(format!("canvas_sub_{}", Uuid::new_v4())),
            grade: None, // Grade comes later
            feedback: None,
            error: None,
        })
    }

    async fn get_accommodations(&self, user_id: &str) -> Result<LMSAccommodations> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // GET /api/v1/users/:user_id/custom_data/wia_accessibility

        Ok(LMSAccommodations::default())
    }

    async fn set_accommodations(&self, user_id: &str, accommodations: &LMSAccommodations) -> Result<()> {
        if !self.connected {
            return Err(EduError::WIAIntegrationError("Not connected to Canvas".to_string()));
        }

        // PUT /api/v1/users/:user_id/custom_data/wia_accessibility

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
    async fn test_create_canvas_adapter() {
        let adapter = CanvasAdapter::with_api_key("https://canvas.example.com", "test-key");
        assert!(adapter.is_connected());
        assert_eq!(adapter.platform(), LMSPlatform::Canvas);
    }

    #[tokio::test]
    async fn test_get_courses() {
        let adapter = CanvasAdapter::with_api_key("https://canvas.example.com", "test-key");
        let courses = adapter.get_courses("user123").await.unwrap();
        assert!(!courses.is_empty());
    }

    #[tokio::test]
    async fn test_sync_profile() {
        use crate::core::ProfileManager;
        use crate::types::DisabilityType;

        let adapter = CanvasAdapter::with_api_key("https://canvas.example.com", "test-key");
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Blind);

        let status = adapter.sync_profile(&profile).await.unwrap();
        assert!(status.success);
        assert!(!status.fields_synced.is_empty());
    }
}
