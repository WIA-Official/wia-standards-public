//! # WIA Education Accessibility API
//!
//! A Rust library for educational technology accessibility standards.
//!
//! ## Features
//!
//! - **Learner Profiles**: Manage learner accessibility preferences (AccessForAll PNP)
//! - **Course Management**: Create courses with UDL options and accessibility metadata
//! - **Content Metadata**: Track content accessibility features (AccessForAll DRD)
//! - **Assessment Accommodations**: Configure and apply testing accommodations
//! - **Accessibility Matching**: Match learner needs with available content
//! - **WCAG Validation**: Check content against WCAG guidelines
//! - **LTI 1.3 Protocol**: Integrate with Learning Management Systems
//! - **xAPI Statements**: Track accessibility learning activities
//! - **Profile Sync**: Synchronize profiles across platforms
//! - **Real-time Events**: Accessibility adaptation event streaming
//! - **Ecosystem Integration**: Connect with WIA assistive devices and LMS platforms
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_edu::{EduController, types::*};
//!
//! fn main() {
//!     // Create controller
//!     let mut controller = EduController::new();
//!
//!     // Create a profile for a blind learner
//!     let profile = controller.create_profile_for_disability(DisabilityType::Blind);
//!
//!     // Profile has recommended settings for blind learners
//!     assert!(profile.display_preferences.screen_reader.enabled);
//!     assert!(profile.content_preferences.audio_description.required);
//! }
//! ```
//!
//! ## 弘益人間 - Education for Everyone

pub mod adapters;
pub mod core;
pub mod ecosystem;
pub mod error;
pub mod protocol;
pub mod types;

pub use adapters::*;
pub use core::*;
pub use ecosystem::*;
pub use error::{EduError, Result};
pub use protocol::*;
pub use types::*;

use std::sync::Arc;

/// Main education accessibility controller
#[derive(Debug)]
pub struct EduController {
    profile_manager: ProfileManager,
    course_manager: CourseManager,
    assessment_manager: AssessmentManager,
    storage: Option<Arc<dyn ProfileStorage>>,
}

impl EduController {
    /// Create a new education controller
    pub fn new() -> Self {
        Self {
            profile_manager: ProfileManager::new(),
            course_manager: CourseManager::new(),
            assessment_manager: AssessmentManager::new(),
            storage: None,
        }
    }

    /// Set profile storage adapter
    pub fn with_storage(mut self, storage: Arc<dyn ProfileStorage>) -> Self {
        self.storage = Some(storage);
        self
    }

    // ========================================================================
    // Profile Management
    // ========================================================================

    /// Create a new learner profile with default settings
    pub fn create_profile(&mut self) -> LearnerProfile {
        self.profile_manager.create_profile()
    }

    /// Create a profile optimized for a specific disability
    pub fn create_profile_for_disability(&mut self, disability: DisabilityType) -> LearnerProfile {
        self.profile_manager.create_profile_for_disability(disability)
    }

    /// Get a profile by ID
    pub fn get_profile(&self, profile_id: ProfileId) -> Result<&LearnerProfile> {
        self.profile_manager.get_profile(profile_id)
    }

    /// Update a profile
    pub fn update_profile(&mut self, profile: LearnerProfile) -> Result<()> {
        Validator::validate_profile(&profile)?;
        self.profile_manager.update_profile(profile)
    }

    /// Delete a profile
    pub fn delete_profile(&mut self, profile_id: ProfileId) -> Result<LearnerProfile> {
        self.profile_manager.delete_profile(profile_id)
    }

    /// List all profiles
    pub fn list_profiles(&self) -> Vec<&LearnerProfile> {
        self.profile_manager.list_profiles()
    }

    /// Export profile to JSON
    pub fn export_profile(&self, profile_id: ProfileId) -> Result<String> {
        self.profile_manager.export_profile(profile_id)
    }

    /// Import profile from JSON
    pub fn import_profile(&mut self, json: &str) -> Result<LearnerProfile> {
        self.profile_manager.import_profile(json)
    }

    /// Find profiles by disability
    pub fn find_profiles_by_disability(&self, disability: DisabilityType) -> Vec<&LearnerProfile> {
        self.profile_manager.find_by_disability(disability)
    }

    /// Link WIA device profile
    pub fn link_wia_profile(
        &mut self,
        profile_id: ProfileId,
        device_type: &str,
        device_profile_id: uuid::Uuid,
    ) -> Result<()> {
        self.profile_manager.link_wia_profile(profile_id, device_type, device_profile_id)
    }

    // ========================================================================
    // Async Storage Operations
    // ========================================================================

    /// Save profile to storage
    pub async fn save_profile(&self, profile: &LearnerProfile) -> Result<()> {
        match &self.storage {
            Some(storage) => storage.save_profile(profile).await,
            None => Err(EduError::InvalidConfiguration("No storage configured".to_string())),
        }
    }

    /// Load profile from storage
    pub async fn load_profile(&mut self, profile_id: ProfileId) -> Result<LearnerProfile> {
        match &self.storage {
            Some(storage) => {
                let profile = storage.load_profile(profile_id).await?;
                self.profile_manager.update_profile(profile.clone()).ok();
                Ok(profile)
            }
            None => Err(EduError::InvalidConfiguration("No storage configured".to_string())),
        }
    }

    // ========================================================================
    // Course Management
    // ========================================================================

    /// Create a new course
    pub fn create_course(&mut self, title: String) -> Course {
        self.course_manager.create_course(title)
    }

    /// Get a course by ID
    pub fn get_course(&self, course_id: CourseId) -> Result<&Course> {
        self.course_manager.get_course(course_id)
    }

    /// Update a course
    pub fn update_course(&mut self, course: Course) -> Result<()> {
        Validator::validate_course(&course)?;
        self.course_manager.update_course(course)
    }

    /// Delete a course
    pub fn delete_course(&mut self, course_id: CourseId) -> Result<Course> {
        self.course_manager.delete_course(course_id)
    }

    /// List all courses
    pub fn list_courses(&self) -> Vec<&Course> {
        self.course_manager.list_courses()
    }

    /// Add a module to a course
    pub fn add_module(&mut self, course_id: CourseId, title: String) -> Result<Module> {
        self.course_manager.add_module(course_id, title)
    }

    /// Add content to a module
    pub fn add_content_to_module(
        &mut self,
        course_id: CourseId,
        module_id: ModuleId,
        content: ContentItem,
    ) -> Result<()> {
        self.course_manager.add_content_to_module(course_id, module_id, content)
    }

    /// Set accessibility statement for a course
    pub fn set_accessibility_statement(
        &mut self,
        course_id: CourseId,
        statement: AccessibilityStatement,
    ) -> Result<()> {
        self.course_manager.set_accessibility_statement(course_id, statement)
    }

    /// Get course accessibility report
    pub fn get_course_accessibility_report(
        &self,
        course_id: CourseId,
    ) -> Result<CourseAccessibilityReport> {
        self.course_manager.get_accessibility_report(course_id)
    }

    /// Find courses by WCAG level
    pub fn find_courses_by_wcag_level(&self, level: WCAGLevel) -> Vec<&Course> {
        self.course_manager.find_by_wcag_level(level)
    }

    // ========================================================================
    // Assessment Management
    // ========================================================================

    /// Create a new assessment
    pub fn create_assessment(
        &mut self,
        title: String,
        assessment_type: AssessmentType,
    ) -> Assessment {
        self.assessment_manager.create_assessment(title, assessment_type)
    }

    /// Get an assessment by ID
    pub fn get_assessment(&self, assessment_id: AssessmentId) -> Result<&Assessment> {
        self.assessment_manager.get_assessment(assessment_id)
    }

    /// Update an assessment
    pub fn update_assessment(&mut self, assessment: Assessment) -> Result<()> {
        Validator::validate_assessment(&assessment)?;
        self.assessment_manager.update_assessment(assessment)
    }

    /// Delete an assessment
    pub fn delete_assessment(&mut self, assessment_id: AssessmentId) -> Result<Assessment> {
        self.assessment_manager.delete_assessment(assessment_id)
    }

    /// Add a question to an assessment
    pub fn add_question(&mut self, assessment_id: AssessmentId, question: Question) -> Result<()> {
        self.assessment_manager.add_question(assessment_id, question)
    }

    /// Set time limit for assessment
    pub fn set_assessment_time_limit(
        &mut self,
        assessment_id: AssessmentId,
        minutes: u32,
    ) -> Result<()> {
        self.assessment_manager.set_time_limit(assessment_id, minutes)
    }

    /// Get effective time limit for a learner
    pub fn get_effective_time_limit(
        &self,
        assessment_id: AssessmentId,
        profile: &LearnerProfile,
    ) -> Result<Option<u32>> {
        self.assessment_manager.get_effective_time_limit(assessment_id, profile)
    }

    /// Check if assessment supports an accommodation
    pub fn assessment_supports_accommodation(
        &self,
        assessment_id: AssessmentId,
        accommodation: &str,
    ) -> Result<bool> {
        self.assessment_manager.supports_accommodation(assessment_id, accommodation)
    }

    // ========================================================================
    // Accessibility Matching
    // ========================================================================

    /// Check if content meets learner's accessibility needs
    pub fn check_content_accessibility(
        profile: &LearnerProfile,
        content: &ContentItem,
    ) -> MatchResult {
        AccessibilityMatcher::content_meets_needs(profile, content)
    }

    /// Check if course meets learner's accessibility needs
    pub fn check_course_accessibility(
        profile: &LearnerProfile,
        course: &Course,
    ) -> CourseMatchResult {
        AccessibilityMatcher::course_meets_needs(profile, course)
    }

    /// Get recommended accommodations for a learner
    pub fn get_recommended_accommodations(
        profile: &LearnerProfile,
    ) -> Vec<RecommendedAccommodation> {
        AccessibilityMatcher::get_recommended_accommodations(profile)
    }

    // ========================================================================
    // Validation
    // ========================================================================

    /// Validate a learner profile
    pub fn validate_profile(profile: &LearnerProfile) -> Result<()> {
        Validator::validate_profile(profile)
    }

    /// Validate a course
    pub fn validate_course(course: &Course) -> Result<()> {
        Validator::validate_course(course)
    }

    /// Validate an assessment
    pub fn validate_assessment(assessment: &Assessment) -> Result<()> {
        Validator::validate_assessment(assessment)
    }

    /// Check WCAG conformance for content
    pub fn check_wcag_conformance(content: &ContentItem) -> Vec<WCAGIssue> {
        Validator::check_wcag_conformance(content)
    }
}

impl Default for EduController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_controller() {
        let controller = EduController::new();
        assert!(controller.list_profiles().is_empty());
        assert!(controller.list_courses().is_empty());
    }

    #[test]
    fn test_profile_workflow() {
        let mut controller = EduController::new();

        // Create profile
        let profile = controller.create_profile();
        let id = profile.profile_id;

        // Get profile
        let retrieved = controller.get_profile(id).unwrap();
        assert_eq!(retrieved.profile_id, id);

        // Delete profile
        controller.delete_profile(id).unwrap();
        assert!(controller.get_profile(id).is_err());
    }

    #[test]
    fn test_disability_profile() {
        let mut controller = EduController::new();

        let profile = controller.create_profile_for_disability(DisabilityType::Blind);

        assert!(profile.display_preferences.screen_reader.enabled);
        assert!(profile.content_preferences.audio_description.required);
    }

    #[test]
    fn test_course_workflow() {
        let mut controller = EduController::new();

        // Create course
        let course = controller.create_course("Intro to CS".to_string());
        let course_id = course.course_id;

        // Add module
        let module = controller.add_module(course_id, "Week 1".to_string()).unwrap();

        // Verify
        let updated = controller.get_course(course_id).unwrap();
        assert_eq!(updated.modules.len(), 1);
        assert_eq!(updated.modules[0].module_id, module.module_id);
    }

    #[test]
    fn test_assessment_workflow() {
        let mut controller = EduController::new();

        // Create assessment
        let assessment = controller.create_assessment(
            "Midterm".to_string(),
            AssessmentType::Exam,
        );
        let id = assessment.assessment_id;

        // Set time limit
        controller.set_assessment_time_limit(id, 60).unwrap();

        // Verify
        let updated = controller.get_assessment(id).unwrap();
        assert_eq!(updated.timing.time_limit_minutes, Some(60));
    }

    #[test]
    fn test_export_import() {
        let mut controller = EduController::new();

        let original = controller.create_profile_for_disability(DisabilityType::LowVision);
        let json = controller.export_profile(original.profile_id).unwrap();

        let imported = controller.import_profile(&json).unwrap();
        assert_eq!(
            original.display_preferences.magnification.level,
            imported.display_preferences.magnification.level
        );
    }
}
