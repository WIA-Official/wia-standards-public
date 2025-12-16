//! Privacy protection module

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// Data retention policy
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RetentionPolicy {
    /// No retention - immediately delete
    None,
    /// Keep only for current session
    SessionOnly,
    /// Anonymized retention for improvement
    Anonymized,
    /// Encrypted retention with user key
    Encrypted,
}

/// Privacy configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrivacyConfig {
    /// Audio data retention
    pub audio_retention: RetentionPolicy,
    /// Transcript retention
    pub transcript_retention: RetentionPolicy,
    /// Gloss data retention
    pub gloss_retention: RetentionPolicy,
    /// Pose data retention
    pub pose_retention: RetentionPolicy,
    /// PII detection enabled
    pub pii_detection: bool,
    /// Auto-redact PII
    pub auto_redact_pii: bool,
    /// GDPR compliance mode
    pub gdpr_mode: bool,
    /// Korean PIPA compliance mode
    pub korean_pipa_mode: bool,
}

impl Default for PrivacyConfig {
    fn default() -> Self {
        Self {
            audio_retention: RetentionPolicy::None,
            transcript_retention: RetentionPolicy::Anonymized,
            gloss_retention: RetentionPolicy::SessionOnly,
            pose_retention: RetentionPolicy::SessionOnly,
            pii_detection: true,
            auto_redact_pii: true,
            gdpr_mode: true,
            korean_pipa_mode: true,
        }
    }
}

/// PII category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PiiCategory {
    Name,
    Email,
    Phone,
    Address,
    SocialSecurityNumber,
    CreditCard,
    DateOfBirth,
    MedicalId,
    IpAddress,
    Location,
}

/// PII scan result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PiiScanResult {
    /// PII detected
    pub detected: Vec<PiiMatch>,
    /// Categories found
    pub categories: Vec<PiiCategory>,
    /// Redacted text (if auto-redact enabled)
    pub redacted_text: Option<String>,
    /// Risk level
    pub risk_level: PrivacyRiskLevel,
}

/// PII match
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PiiMatch {
    pub category: PiiCategory,
    pub start: usize,
    pub end: usize,
    pub confidence: f32,
}

/// Privacy risk level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PrivacyRiskLevel {
    Low,
    Medium,
    High,
    Critical,
}

/// Privacy guard
pub struct PrivacyGuard {
    config: PrivacyConfig,
    email_pattern: regex::Regex,
    phone_patterns: Vec<regex::Regex>,
    ssn_pattern: regex::Regex,
    credit_card_pattern: regex::Regex,
}

impl PrivacyGuard {
    /// Create a new privacy guard
    pub fn new() -> Self {
        Self::with_config(PrivacyConfig::default())
    }

    /// Create with custom configuration
    pub fn with_config(config: PrivacyConfig) -> Self {
        Self {
            config,
            email_pattern: regex::Regex::new(r"[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}")
                .unwrap(),
            phone_patterns: vec![
                // US format
                regex::Regex::new(r"\b\d{3}[-.]?\d{3}[-.]?\d{4}\b").unwrap(),
                // Korean format
                regex::Regex::new(r"\b0\d{1,2}[-.]?\d{3,4}[-.]?\d{4}\b").unwrap(),
                // International
                regex::Regex::new(r"\+\d{1,3}[-.]?\d{2,4}[-.]?\d{3,4}[-.]?\d{3,4}\b").unwrap(),
            ],
            ssn_pattern: regex::Regex::new(r"\b\d{3}[-]?\d{2}[-]?\d{4}\b").unwrap(),
            credit_card_pattern: regex::Regex::new(r"\b\d{4}[-\s]?\d{4}[-\s]?\d{4}[-\s]?\d{4}\b")
                .unwrap(),
        }
    }

    /// Scan text for PII
    pub fn scan_pii(&self, text: &str) -> PiiScanResult {
        if !self.config.pii_detection {
            return PiiScanResult {
                detected: vec![],
                categories: vec![],
                redacted_text: None,
                risk_level: PrivacyRiskLevel::Low,
            };
        }

        let mut detected = Vec::new();
        let mut categories = HashSet::new();

        // Check for emails
        for mat in self.email_pattern.find_iter(text) {
            detected.push(PiiMatch {
                category: PiiCategory::Email,
                start: mat.start(),
                end: mat.end(),
                confidence: 0.95,
            });
            categories.insert(PiiCategory::Email);
        }

        // Check for phone numbers
        for pattern in &self.phone_patterns {
            for mat in pattern.find_iter(text) {
                detected.push(PiiMatch {
                    category: PiiCategory::Phone,
                    start: mat.start(),
                    end: mat.end(),
                    confidence: 0.85,
                });
                categories.insert(PiiCategory::Phone);
            }
        }

        // Check for SSN
        for mat in self.ssn_pattern.find_iter(text) {
            detected.push(PiiMatch {
                category: PiiCategory::SocialSecurityNumber,
                start: mat.start(),
                end: mat.end(),
                confidence: 0.80,
            });
            categories.insert(PiiCategory::SocialSecurityNumber);
        }

        // Check for credit cards
        for mat in self.credit_card_pattern.find_iter(text) {
            // Validate with Luhn algorithm in production
            detected.push(PiiMatch {
                category: PiiCategory::CreditCard,
                start: mat.start(),
                end: mat.end(),
                confidence: 0.75,
            });
            categories.insert(PiiCategory::CreditCard);
        }

        // Calculate risk level
        let risk_level = if categories.contains(&PiiCategory::SocialSecurityNumber)
            || categories.contains(&PiiCategory::CreditCard)
        {
            PrivacyRiskLevel::Critical
        } else if categories.contains(&PiiCategory::Email) && categories.contains(&PiiCategory::Phone)
        {
            PrivacyRiskLevel::High
        } else if !categories.is_empty() {
            PrivacyRiskLevel::Medium
        } else {
            PrivacyRiskLevel::Low
        };

        // Redact if enabled
        let redacted_text = if self.config.auto_redact_pii && !detected.is_empty() {
            Some(self.redact_pii(text, &detected))
        } else {
            None
        };

        PiiScanResult {
            detected,
            categories: categories.into_iter().collect(),
            redacted_text,
            risk_level,
        }
    }

    /// Redact PII from text
    pub fn redact_pii(&self, text: &str, matches: &[PiiMatch]) -> String {
        let mut result = text.to_string();

        // Sort matches by start position in reverse order
        let mut sorted_matches: Vec<_> = matches.iter().collect();
        sorted_matches.sort_by(|a, b| b.start.cmp(&a.start));

        for m in sorted_matches {
            let redaction = self.get_redaction_text(m.category);
            result.replace_range(m.start..m.end, &redaction);
        }

        result
    }

    fn get_redaction_text(&self, category: PiiCategory) -> String {
        match category {
            PiiCategory::Email => "[EMAIL REDACTED]".to_string(),
            PiiCategory::Phone => "[PHONE REDACTED]".to_string(),
            PiiCategory::SocialSecurityNumber => "[SSN REDACTED]".to_string(),
            PiiCategory::CreditCard => "[CARD REDACTED]".to_string(),
            PiiCategory::Name => "[NAME REDACTED]".to_string(),
            PiiCategory::Address => "[ADDRESS REDACTED]".to_string(),
            PiiCategory::DateOfBirth => "[DOB REDACTED]".to_string(),
            PiiCategory::MedicalId => "[MEDICAL ID REDACTED]".to_string(),
            PiiCategory::IpAddress => "[IP REDACTED]".to_string(),
            PiiCategory::Location => "[LOCATION REDACTED]".to_string(),
        }
    }

    /// Get configuration
    pub fn config(&self) -> &PrivacyConfig {
        &self.config
    }

    /// Check if data should be retained
    pub fn should_retain(&self, data_type: DataType) -> bool {
        let policy = match data_type {
            DataType::Audio => self.config.audio_retention,
            DataType::Transcript => self.config.transcript_retention,
            DataType::Gloss => self.config.gloss_retention,
            DataType::Pose => self.config.pose_retention,
        };

        !matches!(policy, RetentionPolicy::None)
    }
}

impl Default for PrivacyGuard {
    fn default() -> Self {
        Self::new()
    }
}

/// Data type for retention policy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Audio,
    Transcript,
    Gloss,
    Pose,
}

/// Consent record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentRecord {
    /// User identifier (anonymized)
    pub user_id: String,
    /// Consent timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
    /// Consents given
    pub consents: Vec<ConsentItem>,
    /// IP address hash
    pub ip_hash: String,
}

/// Individual consent item
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsentItem {
    pub category: ConsentCategory,
    pub granted: bool,
    pub version: String,
}

/// Consent categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConsentCategory {
    /// Basic data processing (required)
    DataProcessing,
    /// Privacy policy acceptance (required)
    PrivacyPolicy,
    /// Service improvement (optional)
    ServiceImprovement,
    /// Analytics (optional)
    Analytics,
    /// Marketing (optional)
    Marketing,
}

/// Data subject request types (GDPR/PIPA)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataSubjectRequest {
    /// Access to personal data
    Access,
    /// Correction of data
    Rectification,
    /// Deletion of data
    Erasure,
    /// Restriction of processing
    Restriction,
    /// Data portability
    Portability,
    /// Object to processing
    Objection,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pii_detection_email() {
        let guard = PrivacyGuard::new();
        let result = guard.scan_pii("Contact me at test@example.com please");
        assert!(!result.detected.is_empty());
        assert!(result.categories.contains(&PiiCategory::Email));
    }

    #[test]
    fn test_pii_detection_phone() {
        let guard = PrivacyGuard::new();
        let result = guard.scan_pii("Call me at 010-1234-5678");
        assert!(!result.detected.is_empty());
        assert!(result.categories.contains(&PiiCategory::Phone));
    }

    #[test]
    fn test_pii_redaction() {
        let guard = PrivacyGuard::new();
        let result = guard.scan_pii("Email: test@example.com");
        assert!(result.redacted_text.is_some());
        let redacted = result.redacted_text.unwrap();
        assert!(redacted.contains("[EMAIL REDACTED]"));
        assert!(!redacted.contains("test@example.com"));
    }

    #[test]
    fn test_no_pii() {
        let guard = PrivacyGuard::new();
        let result = guard.scan_pii("Hello, how are you today?");
        assert!(result.detected.is_empty());
        assert_eq!(result.risk_level, PrivacyRiskLevel::Low);
    }

    #[test]
    fn test_retention_policy() {
        let guard = PrivacyGuard::new();
        assert!(!guard.should_retain(DataType::Audio)); // Default: None
        assert!(guard.should_retain(DataType::Transcript)); // Default: Anonymized
    }
}
