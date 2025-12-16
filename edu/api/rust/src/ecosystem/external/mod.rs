//! External LMS Platform Integrations
//! 弘益人間 - Connect with major Learning Management Systems

pub mod canvas;
pub mod moodle;
pub mod blackboard;
pub mod google;

pub use canvas::CanvasAdapter;
pub use moodle::MoodleAdapter;
pub use blackboard::BlackboardAdapter;
pub use google::GoogleClassroomAdapter;

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::Result;
use crate::types::{LearnerProfile, Course, Assessment};

/// LMS Adapter trait for external platform integration
#[async_trait]
pub trait LMSAdapter: Send + Sync {
    /// Get the platform type
    fn platform(&self) -> LMSPlatform;

    /// Sync learner profile to LMS
    async fn sync_profile(&self, profile: &LearnerProfile) -> Result<SyncStatus>;

    /// Get courses from LMS
    async fn get_courses(&self, user_id: &str) -> Result<Vec<LMSCourse>>;

    /// Get course details
    async fn get_course(&self, course_id: &str) -> Result<LMSCourse>;

    /// Submit assessment
    async fn submit_assessment(&self, submission: &AssessmentSubmission) -> Result<SubmissionResult>;

    /// Get accommodations for user
    async fn get_accommodations(&self, user_id: &str) -> Result<LMSAccommodations>;

    /// Set accommodations for user
    async fn set_accommodations(&self, user_id: &str, accommodations: &LMSAccommodations) -> Result<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;
}

/// LMS Platform types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LMSPlatform {
    /// Canvas LMS
    Canvas,
    /// Moodle
    Moodle,
    /// Blackboard
    Blackboard,
    /// Google Classroom
    GoogleClassroom,
    /// Custom LMS
    Custom,
}

impl std::fmt::Display for LMSPlatform {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LMSPlatform::Canvas => write!(f, "Canvas"),
            LMSPlatform::Moodle => write!(f, "Moodle"),
            LMSPlatform::Blackboard => write!(f, "Blackboard"),
            LMSPlatform::GoogleClassroom => write!(f, "Google Classroom"),
            LMSPlatform::Custom => write!(f, "Custom"),
        }
    }
}

/// LMS configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LMSConfig {
    /// Platform type
    pub platform: LMSPlatform,
    /// API base URL
    pub base_url: String,
    /// API key or token
    pub api_key: Option<String>,
    /// Client ID (for OAuth)
    pub client_id: Option<String>,
    /// Client secret (for OAuth)
    pub client_secret: Option<String>,
    /// OAuth token
    pub oauth_token: Option<String>,
    /// Additional settings
    pub settings: std::collections::HashMap<String, String>,
}

impl LMSConfig {
    /// Create new Canvas config
    pub fn canvas(base_url: &str, api_key: &str) -> Self {
        Self {
            platform: LMSPlatform::Canvas,
            base_url: base_url.to_string(),
            api_key: Some(api_key.to_string()),
            client_id: None,
            client_secret: None,
            oauth_token: None,
            settings: std::collections::HashMap::new(),
        }
    }

    /// Create new Moodle config
    pub fn moodle(base_url: &str, token: &str) -> Self {
        Self {
            platform: LMSPlatform::Moodle,
            base_url: base_url.to_string(),
            api_key: Some(token.to_string()),
            client_id: None,
            client_secret: None,
            oauth_token: None,
            settings: std::collections::HashMap::new(),
        }
    }

    /// Create new Blackboard config
    pub fn blackboard(base_url: &str, client_id: &str, client_secret: &str) -> Self {
        Self {
            platform: LMSPlatform::Blackboard,
            base_url: base_url.to_string(),
            api_key: None,
            client_id: Some(client_id.to_string()),
            client_secret: Some(client_secret.to_string()),
            oauth_token: None,
            settings: std::collections::HashMap::new(),
        }
    }

    /// Create new Google Classroom config
    pub fn google_classroom(client_id: &str, client_secret: &str) -> Self {
        Self {
            platform: LMSPlatform::GoogleClassroom,
            base_url: "https://classroom.googleapis.com".to_string(),
            api_key: None,
            client_id: Some(client_id.to_string()),
            client_secret: Some(client_secret.to_string()),
            oauth_token: None,
            settings: std::collections::HashMap::new(),
        }
    }
}

/// Sync status result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SyncStatus {
    /// Whether sync succeeded
    pub success: bool,
    /// Synced profile ID
    pub profile_id: Uuid,
    /// Remote profile ID (in LMS)
    pub remote_id: Option<String>,
    /// Last sync timestamp
    pub synced_at: chrono::DateTime<chrono::Utc>,
    /// Fields synced
    pub fields_synced: Vec<String>,
    /// Error message if failed
    pub error: Option<String>,
}

/// LMS Course representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LMSCourse {
    /// Course ID in LMS
    pub id: String,
    /// Course code
    pub code: Option<String>,
    /// Course name
    pub name: String,
    /// Course description
    pub description: Option<String>,
    /// Start date
    pub start_date: Option<chrono::DateTime<chrono::Utc>>,
    /// End date
    pub end_date: Option<chrono::DateTime<chrono::Utc>>,
    /// Enrollment status
    pub enrollment_status: EnrollmentStatus,
    /// Accessibility features available
    pub accessibility_features: Vec<String>,
}

/// Enrollment status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EnrollmentStatus {
    /// Active enrollment
    Active,
    /// Pending enrollment
    Pending,
    /// Completed course
    Completed,
    /// Dropped/withdrawn
    Dropped,
    /// Invited but not enrolled
    Invited,
}

/// Assessment submission
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentSubmission {
    /// Assessment ID
    pub assessment_id: String,
    /// User ID
    pub user_id: String,
    /// Submission content
    pub content: SubmissionContent,
    /// Accommodations used
    pub accommodations_used: Vec<String>,
    /// Time taken (seconds)
    pub time_taken_seconds: u32,
    /// Submitted at
    pub submitted_at: chrono::DateTime<chrono::Utc>,
}

/// Submission content
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SubmissionContent {
    /// Answers to questions
    Answers(Vec<AnswerSubmission>),
    /// File upload
    File { filename: String, content_type: String, url: String },
    /// Text essay
    Text(String),
    /// URL submission
    Url(String),
}

/// Single answer submission
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnswerSubmission {
    /// Question ID
    pub question_id: String,
    /// Answer text
    pub answer: String,
    /// Input method used (for accessibility tracking)
    pub input_method: Option<String>,
}

/// Submission result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubmissionResult {
    /// Whether submission succeeded
    pub success: bool,
    /// Submission ID in LMS
    pub submission_id: Option<String>,
    /// Grade if available
    pub grade: Option<f32>,
    /// Feedback if available
    pub feedback: Option<String>,
    /// Error message if failed
    pub error: Option<String>,
}

/// LMS Accommodations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LMSAccommodations {
    /// Extended time multiplier
    pub extended_time: Option<f32>,
    /// Allow breaks
    pub breaks_allowed: bool,
    /// Separate room
    pub separate_room: bool,
    /// Screen reader access
    pub screen_reader: bool,
    /// Magnification
    pub magnification: bool,
    /// Braille
    pub braille: bool,
    /// Sign language interpreter
    pub sign_language: bool,
    /// Text-to-speech
    pub text_to_speech: bool,
    /// Calculator
    pub calculator: bool,
    /// Dictionary
    pub dictionary: bool,
    /// Custom accommodations
    pub custom: Vec<String>,
}

impl Default for LMSAccommodations {
    fn default() -> Self {
        Self {
            extended_time: None,
            breaks_allowed: false,
            separate_room: false,
            screen_reader: false,
            magnification: false,
            braille: false,
            sign_language: false,
            text_to_speech: false,
            calculator: false,
            dictionary: false,
            custom: vec![],
        }
    }
}

impl LMSAccommodations {
    /// Create from learner profile
    pub fn from_profile(profile: &LearnerProfile) -> Self {
        Self {
            extended_time: if profile.assessment_accommodations.timing.extended_time {
                Some(profile.assessment_accommodations.timing.time_multiplier)
            } else {
                None
            },
            breaks_allowed: profile.assessment_accommodations.timing.breaks_allowed,
            separate_room: profile.assessment_accommodations.environment.separate_room,
            screen_reader: profile.display_preferences.screen_reader.enabled,
            magnification: profile.display_preferences.magnification.enabled,
            braille: false, // Would check if braille display in profile
            sign_language: profile.content_preferences.sign_language.preferred,
            text_to_speech: profile.content_preferences.text_to_speech.enabled,
            calculator: profile.assessment_accommodations.assistance.calculator,
            dictionary: profile.assessment_accommodations.assistance.dictionary,
            custom: vec![],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileManager;
    use crate::types::DisabilityType;

    #[test]
    fn test_lms_config_canvas() {
        let config = LMSConfig::canvas("https://canvas.example.com", "test-key");
        assert_eq!(config.platform, LMSPlatform::Canvas);
        assert!(config.api_key.is_some());
    }

    #[test]
    fn test_lms_config_moodle() {
        let config = LMSConfig::moodle("https://moodle.example.com", "test-token");
        assert_eq!(config.platform, LMSPlatform::Moodle);
    }

    #[test]
    fn test_accommodations_from_profile() {
        let mut manager = ProfileManager::new();
        let profile = manager.create_profile_for_disability(DisabilityType::Blind);

        let accommodations = LMSAccommodations::from_profile(&profile);
        assert!(accommodations.screen_reader);
        // Blind profile has audio description, screen reader - check those are captured
    }

    #[test]
    fn test_platform_display() {
        assert_eq!(format!("{}", LMSPlatform::Canvas), "Canvas");
        assert_eq!(format!("{}", LMSPlatform::GoogleClassroom), "Google Classroom");
    }
}
