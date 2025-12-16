//! WIA Education Core Module
//! 弘益人間 - Education for Everyone

mod profile_manager;
mod course_manager;
mod assessment_manager;
mod validator;
mod matcher;

pub use profile_manager::ProfileManager;
pub use course_manager::{CourseManager, CourseAccessibilityReport};
pub use assessment_manager::AssessmentManager;
pub use validator::{Validator, WCAGIssue, IssueSeverity};
pub use matcher::{
    AccessibilityMatcher, MatchResult, MatchIssue, MatchSeverity,
    AdaptationNeeded, CourseMatchResult, ModuleMatchResult,
    RecommendedAccommodation, AccommodationPriority,
};
