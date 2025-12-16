//! Safety module for WIA Voice-Sign API
//!
//! Provides quality assurance, accessibility, emergency handling,
//! content safety, and privacy protection.

pub mod quality;
pub mod accessibility;
pub mod emergency;
pub mod content;
pub mod privacy;

pub use quality::*;
pub use accessibility::*;
pub use emergency::*;
pub use content::*;
pub use privacy::*;

/// Safety check result
#[derive(Debug, Clone)]
pub struct SafetyCheckResult {
    /// Overall passed status
    pub passed: bool,

    /// Quality check result
    pub quality: Option<QualityCheckResult>,

    /// Accessibility check result
    pub accessibility: Option<AccessibilityCheckResult>,

    /// Content safety check result
    pub content_safety: Option<ContentSafetyResult>,

    /// Warnings generated
    pub warnings: Vec<SafetyWarning>,

    /// Errors encountered
    pub errors: Vec<SafetyError>,
}

/// Safety warning
#[derive(Debug, Clone)]
pub struct SafetyWarning {
    pub code: String,
    pub message: String,
    pub severity: WarningSeverity,
    pub category: SafetyCategory,
}

/// Warning severity levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarningSeverity {
    Info,
    Low,
    Medium,
    High,
    Critical,
}

/// Safety error
#[derive(Debug, Clone)]
pub struct SafetyError {
    pub code: String,
    pub message: String,
    pub category: SafetyCategory,
    pub recoverable: bool,
}

/// Safety categories
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyCategory {
    Quality,
    Accessibility,
    Emergency,
    ContentSafety,
    Privacy,
}

/// Main safety checker that runs all safety checks
pub struct SafetyChecker {
    quality_checker: QualityChecker,
    accessibility_checker: AccessibilityChecker,
    emergency_detector: EmergencyDetector,
    content_filter: ContentFilter,
    privacy_guard: PrivacyGuard,
}

impl SafetyChecker {
    /// Create a new safety checker with default configuration
    pub fn new() -> Self {
        Self {
            quality_checker: QualityChecker::new(),
            accessibility_checker: AccessibilityChecker::new(),
            emergency_detector: EmergencyDetector::new(),
            content_filter: ContentFilter::new(),
            privacy_guard: PrivacyGuard::new(),
        }
    }

    /// Run all safety checks on input text
    pub fn check_input(&self, text: &str, language: &str) -> SafetyCheckResult {
        let mut warnings = Vec::new();
        let mut errors = Vec::new();
        let mut passed = true;

        // Check for emergency
        let emergency_result = self.emergency_detector.detect(text, language);
        if emergency_result.is_emergency {
            warnings.push(SafetyWarning {
                code: "EMERGENCY_DETECTED".to_string(),
                message: format!("Emergency keyword detected: {:?}", emergency_result.keywords),
                severity: WarningSeverity::Critical,
                category: SafetyCategory::Emergency,
            });
        }

        // Check content safety
        let content_result = self.content_filter.filter(text);
        if !content_result.passed {
            passed = false;
            for issue in &content_result.issues {
                errors.push(SafetyError {
                    code: format!("CONTENT_{}", issue.category.to_uppercase()),
                    message: issue.message.clone(),
                    category: SafetyCategory::ContentSafety,
                    recoverable: issue.action != FilterAction::Block,
                });
            }
        }

        // Check for PII
        let pii_result = self.privacy_guard.scan_pii(text);
        if !pii_result.detected.is_empty() {
            warnings.push(SafetyWarning {
                code: "PII_DETECTED".to_string(),
                message: format!("PII detected: {:?}", pii_result.categories),
                severity: WarningSeverity::High,
                category: SafetyCategory::Privacy,
            });
        }

        SafetyCheckResult {
            passed,
            quality: None,
            accessibility: None,
            content_safety: Some(content_result),
            warnings,
            errors,
        }
    }

    /// Run quality check on translation output
    pub fn check_quality(
        &self,
        input: &str,
        gloss_coverage: f32,
        confidence: f32,
    ) -> QualityCheckResult {
        self.quality_checker.check(input, gloss_coverage, confidence)
    }

    /// Check accessibility compliance
    pub fn check_accessibility(&self, config: &AccessibilityConfig) -> AccessibilityCheckResult {
        self.accessibility_checker.check(config)
    }

    /// Get emergency detector for priority processing
    pub fn emergency_detector(&self) -> &EmergencyDetector {
        &self.emergency_detector
    }

    /// Get content filter
    pub fn content_filter(&self) -> &ContentFilter {
        &self.content_filter
    }

    /// Get privacy guard
    pub fn privacy_guard(&self) -> &PrivacyGuard {
        &self.privacy_guard
    }
}

impl Default for SafetyChecker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safety_checker_creation() {
        let checker = SafetyChecker::new();
        let result = checker.check_input("Hello, how are you?", "en");
        assert!(result.passed);
    }

    #[test]
    fn test_emergency_detection() {
        let checker = SafetyChecker::new();
        let result = checker.check_input("Help! I need an ambulance!", "en");
        assert!(!result.warnings.is_empty());
        assert!(result.warnings.iter().any(|w| w.code == "EMERGENCY_DETECTED"));
    }
}
