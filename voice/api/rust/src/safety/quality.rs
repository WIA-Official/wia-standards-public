//! Quality assurance module for translation quality

use serde::{Deserialize, Serialize};

/// Quality check configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityConfig {
    /// Minimum acceptable confidence threshold
    pub minimum_confidence: f32,

    /// Warning level threshold
    pub warning_threshold: f32,

    /// High confidence threshold
    pub high_confidence: f32,

    /// Enable strict mode
    pub strict_mode: bool,

    /// Enable grammar checking
    pub grammar_check: bool,

    /// Enable coverage checking
    pub coverage_check: bool,
}

impl Default for QualityConfig {
    fn default() -> Self {
        Self {
            minimum_confidence: 0.70,
            warning_threshold: 0.85,
            high_confidence: 0.95,
            strict_mode: false,
            grammar_check: true,
            coverage_check: true,
        }
    }
}

/// Confidence level classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConfidenceLevel {
    /// < 0.50 - Unreliable
    Critical,
    /// 0.50 - 0.70 - Low confidence
    Low,
    /// 0.70 - 0.85 - Acceptable
    Medium,
    /// 0.85 - 0.95 - High confidence
    High,
    /// > 0.95 - Excellent
    Excellent,
}

impl ConfidenceLevel {
    /// Classify a confidence score
    pub fn from_score(score: f32) -> Self {
        match score {
            s if s < 0.50 => ConfidenceLevel::Critical,
            s if s < 0.70 => ConfidenceLevel::Low,
            s if s < 0.85 => ConfidenceLevel::Medium,
            s if s < 0.95 => ConfidenceLevel::High,
            _ => ConfidenceLevel::Excellent,
        }
    }

    /// Check if this level is acceptable
    pub fn is_acceptable(&self) -> bool {
        matches!(
            self,
            ConfidenceLevel::Medium | ConfidenceLevel::High | ConfidenceLevel::Excellent
        )
    }
}

/// Quality check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityCheckResult {
    /// Overall quality score (0.0 - 1.0)
    pub overall_score: f32,

    /// Confidence level classification
    pub confidence_level: ConfidenceLevel,

    /// Gloss coverage percentage
    pub gloss_coverage: f32,

    /// Grammar check passed
    pub grammar_passed: bool,

    /// Context consistency score
    pub context_score: f32,

    /// Quality warnings
    pub warnings: Vec<QualityWarning>,

    /// Suggested actions
    pub suggestions: Vec<String>,

    /// Is acceptable for output
    pub acceptable: bool,
}

/// Quality warning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityWarning {
    pub code: String,
    pub message: String,
    pub segment: Option<String>,
    pub confidence: f32,
}

/// Low confidence handling strategy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LowConfidenceStrategy {
    /// Use fingerspelling for uncertain words
    Fingerspell,
    /// Show alternative translations
    ShowAlternatives,
    /// Use simplified gloss
    SimplifiedGloss,
    /// Display text instead
    TextDisplay,
    /// Request human review
    HumanReview,
}

/// Quality checker
pub struct QualityChecker {
    config: QualityConfig,
}

impl QualityChecker {
    /// Create a new quality checker
    pub fn new() -> Self {
        Self {
            config: QualityConfig::default(),
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: QualityConfig) -> Self {
        Self { config }
    }

    /// Check translation quality
    pub fn check(
        &self,
        _input: &str,
        gloss_coverage: f32,
        confidence: f32,
    ) -> QualityCheckResult {
        let mut warnings = Vec::new();
        let mut suggestions = Vec::new();

        let confidence_level = ConfidenceLevel::from_score(confidence);

        // Check confidence level
        if confidence < self.config.minimum_confidence {
            warnings.push(QualityWarning {
                code: "LOW_CONFIDENCE".to_string(),
                message: format!(
                    "Translation confidence ({:.2}) is below minimum threshold ({:.2})",
                    confidence, self.config.minimum_confidence
                ),
                segment: None,
                confidence,
            });
            suggestions.push("Consider using fingerspelling for uncertain terms".to_string());
        } else if confidence < self.config.warning_threshold {
            warnings.push(QualityWarning {
                code: "MODERATE_CONFIDENCE".to_string(),
                message: format!(
                    "Translation confidence ({:.2}) is below optimal ({:.2})",
                    confidence, self.config.warning_threshold
                ),
                segment: None,
                confidence,
            });
        }

        // Check gloss coverage
        if gloss_coverage < 0.70 {
            warnings.push(QualityWarning {
                code: "LOW_COVERAGE".to_string(),
                message: format!(
                    "Gloss coverage ({:.0}%) is low - many words may be fingerspelled",
                    gloss_coverage * 100.0
                ),
                segment: None,
                confidence: gloss_coverage,
            });
            suggestions.push("Some words may not have direct sign translations".to_string());
        }

        // Calculate overall score
        let overall_score = (confidence * 0.6 + gloss_coverage * 0.4).min(1.0);

        // Determine if acceptable
        let acceptable = if self.config.strict_mode {
            confidence >= self.config.warning_threshold && gloss_coverage >= 0.85
        } else {
            confidence >= self.config.minimum_confidence
        };

        QualityCheckResult {
            overall_score,
            confidence_level,
            gloss_coverage,
            grammar_passed: true, // Simplified
            context_score: 0.90,  // Simplified
            warnings,
            suggestions,
            acceptable,
        }
    }

    /// Get recommended fallback strategy for low confidence
    pub fn get_fallback_strategy(&self, confidence_level: ConfidenceLevel) -> LowConfidenceStrategy {
        match confidence_level {
            ConfidenceLevel::Critical => LowConfidenceStrategy::HumanReview,
            ConfidenceLevel::Low => LowConfidenceStrategy::ShowAlternatives,
            ConfidenceLevel::Medium => LowConfidenceStrategy::Fingerspell,
            _ => LowConfidenceStrategy::Fingerspell,
        }
    }

    /// Get configuration
    pub fn config(&self) -> &QualityConfig {
        &self.config
    }
}

impl Default for QualityChecker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_confidence_level_classification() {
        assert_eq!(ConfidenceLevel::from_score(0.40), ConfidenceLevel::Critical);
        assert_eq!(ConfidenceLevel::from_score(0.60), ConfidenceLevel::Low);
        assert_eq!(ConfidenceLevel::from_score(0.75), ConfidenceLevel::Medium);
        assert_eq!(ConfidenceLevel::from_score(0.90), ConfidenceLevel::High);
        assert_eq!(ConfidenceLevel::from_score(0.98), ConfidenceLevel::Excellent);
    }

    #[test]
    fn test_quality_check_acceptable() {
        let checker = QualityChecker::new();
        let result = checker.check("Hello", 0.90, 0.88);
        assert!(result.acceptable);
        assert!(result.warnings.is_empty() || result.warnings.len() == 1);
    }

    #[test]
    fn test_quality_check_low_confidence() {
        let checker = QualityChecker::new();
        let result = checker.check("Complex sentence", 0.50, 0.55);
        assert!(!result.acceptable);
        assert!(!result.warnings.is_empty());
    }
}
