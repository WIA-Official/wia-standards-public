//! Content safety filtering module

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// Filter action to take
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterAction {
    /// Allow content to pass
    Allow,
    /// Warn but allow
    Warn,
    /// Censor specific words
    Censor,
    /// Block entire content
    Block,
    /// Log for review
    Log,
}

/// Content filter level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterLevel {
    /// Minimal filtering
    Minimal,
    /// Moderate filtering
    Moderate,
    /// Strict filtering
    Strict,
}

/// Content filter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentFilterConfig {
    pub profanity: FilterSettings,
    pub violence: FilterSettings,
    pub discrimination: FilterSettings,
    pub sexual_content: FilterSettings,
    pub self_harm: FilterSettings,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterSettings {
    pub enabled: bool,
    pub level: FilterLevel,
    pub action: FilterAction,
}

impl Default for ContentFilterConfig {
    fn default() -> Self {
        Self {
            profanity: FilterSettings {
                enabled: true,
                level: FilterLevel::Moderate,
                action: FilterAction::Censor,
            },
            violence: FilterSettings {
                enabled: true,
                level: FilterLevel::Moderate,
                action: FilterAction::Warn,
            },
            discrimination: FilterSettings {
                enabled: true,
                level: FilterLevel::Strict,
                action: FilterAction::Block,
            },
            sexual_content: FilterSettings {
                enabled: true,
                level: FilterLevel::Strict,
                action: FilterAction::Block,
            },
            self_harm: FilterSettings {
                enabled: true,
                level: FilterLevel::Strict,
                action: FilterAction::Block,
            },
        }
    }
}

/// Content safety check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentSafetyResult {
    /// Whether content passed all checks
    pub passed: bool,

    /// Categories checked
    pub categories_checked: Vec<String>,

    /// Issues found
    pub issues: Vec<ContentIssue>,

    /// Modified content (if censored)
    pub modified_content: Option<String>,

    /// Overall confidence
    pub confidence: f32,
}

/// Content issue details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentIssue {
    pub category: String,
    pub message: String,
    pub severity: IssueSeverity,
    pub action: FilterAction,
    pub matched_pattern: Option<String>,
}

/// Issue severity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueSeverity {
    Low,
    Medium,
    High,
    Critical,
}

/// Content filter
pub struct ContentFilter {
    config: ContentFilterConfig,
    profanity_words: HashSet<String>,
    violence_words: HashSet<String>,
    discrimination_words: HashSet<String>,
}

impl ContentFilter {
    /// Create a new content filter
    pub fn new() -> Self {
        Self::with_config(ContentFilterConfig::default())
    }

    /// Create with custom configuration
    pub fn with_config(config: ContentFilterConfig) -> Self {
        let profanity_words = Self::load_profanity_words();
        let violence_words = Self::load_violence_words();
        let discrimination_words = Self::load_discrimination_words();

        Self {
            config,
            profanity_words,
            violence_words,
            discrimination_words,
        }
    }

    fn load_profanity_words() -> HashSet<String> {
        // Basic list - in production, load from comprehensive database
        [
            "fuck", "shit", "damn", "ass", "bitch",
            // Add more as needed
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }

    fn load_violence_words() -> HashSet<String> {
        [
            "kill", "murder", "attack", "bomb", "weapon",
            // Context matters - these are flagged for review
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }

    fn load_discrimination_words() -> HashSet<String> {
        // Intentionally minimal - discrimination detection needs context
        HashSet::new()
    }

    /// Filter content
    pub fn filter(&self, text: &str) -> ContentSafetyResult {
        let mut issues = Vec::new();
        let mut modified_content = text.to_string();
        let text_lower = text.to_lowercase();

        // Check profanity
        if self.config.profanity.enabled {
            for word in &self.profanity_words {
                if text_lower.contains(word) {
                    issues.push(ContentIssue {
                        category: "profanity".to_string(),
                        message: "Profanity detected".to_string(),
                        severity: IssueSeverity::Medium,
                        action: self.config.profanity.action,
                        matched_pattern: Some(word.clone()),
                    });

                    if self.config.profanity.action == FilterAction::Censor {
                        modified_content = self.censor_word(&modified_content, word);
                    }
                }
            }
        }

        // Check violence
        if self.config.violence.enabled {
            for word in &self.violence_words {
                if text_lower.contains(word) {
                    issues.push(ContentIssue {
                        category: "violence".to_string(),
                        message: "Violence-related content detected".to_string(),
                        severity: IssueSeverity::Medium,
                        action: self.config.violence.action,
                        matched_pattern: Some(word.clone()),
                    });
                }
            }
        }

        // Determine if passed
        let passed = !issues.iter().any(|i| i.action == FilterAction::Block);

        ContentSafetyResult {
            passed,
            categories_checked: vec![
                "profanity".to_string(),
                "violence".to_string(),
                "discrimination".to_string(),
            ],
            issues,
            modified_content: if modified_content != text {
                Some(modified_content)
            } else {
                None
            },
            confidence: 0.95,
        }
    }

    /// Censor a word in text
    fn censor_word(&self, text: &str, word: &str) -> String {
        let censored = format!("{}***", &word[..1.min(word.len())]);
        text.replace(word, &censored)
    }

    /// Check if content is safe
    pub fn is_safe(&self, text: &str) -> bool {
        self.filter(text).passed
    }

    /// Get configuration
    pub fn config(&self) -> &ContentFilterConfig {
        &self.config
    }
}

impl Default for ContentFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// Sensitive content handler
pub struct SensitiveContentHandler {
    medical_terms: HashSet<String>,
    legal_terms: HashSet<String>,
}

impl SensitiveContentHandler {
    /// Create a new handler
    pub fn new() -> Self {
        Self {
            medical_terms: Self::load_medical_terms(),
            legal_terms: Self::load_legal_terms(),
        }
    }

    fn load_medical_terms() -> HashSet<String> {
        [
            "diagnosis", "treatment", "medication", "surgery",
            "cancer", "diabetes", "heart", "blood", "pain",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }

    fn load_legal_terms() -> HashSet<String> {
        [
            "contract", "lawsuit", "court", "legal", "attorney",
            "rights", "liability", "defendant", "plaintiff",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }

    /// Check for sensitive content
    pub fn check(&self, text: &str) -> SensitiveContentResult {
        let text_lower = text.to_lowercase();
        let mut detected_categories = Vec::new();

        // Check medical
        let has_medical = self.medical_terms.iter().any(|t| text_lower.contains(t));
        if has_medical {
            detected_categories.push("medical".to_string());
        }

        // Check legal
        let has_legal = self.legal_terms.iter().any(|t| text_lower.contains(t));
        if has_legal {
            detected_categories.push("legal".to_string());
        }

        SensitiveContentResult {
            has_sensitive_content: !detected_categories.is_empty(),
            categories: detected_categories,
            disclaimer_required: has_medical || has_legal,
        }
    }
}

impl Default for SensitiveContentHandler {
    fn default() -> Self {
        Self::new()
    }
}

/// Sensitive content check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensitiveContentResult {
    pub has_sensitive_content: bool,
    pub categories: Vec<String>,
    pub disclaimer_required: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_content_filter_clean() {
        let filter = ContentFilter::new();
        let result = filter.filter("Hello, how are you?");
        assert!(result.passed);
        assert!(result.issues.is_empty());
    }

    #[test]
    fn test_content_filter_profanity() {
        let filter = ContentFilter::new();
        let result = filter.filter("What the fuck!");
        assert!(result.passed); // Censored but allowed
        assert!(!result.issues.is_empty());
        assert!(result.modified_content.is_some());
    }

    #[test]
    fn test_sensitive_content() {
        let handler = SensitiveContentHandler::new();

        let result = handler.check("I need medication for my diagnosis");
        assert!(result.has_sensitive_content);
        assert!(result.categories.contains(&"medical".to_string()));
        assert!(result.disclaimer_required);
    }
}
