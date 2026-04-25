//! WIA Refugee Credential API
//!
//! 국가가 무너져도, 사람의 가치는 무너지지 않습니다.
//! When nations fall, human value remains.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

pub mod types;
pub mod core;
pub mod server;

pub use types::*;
pub use core::*;

/// Library version
pub const VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "홍익인간 (弘益人間) - Benefit All Humanity";

/// Verification level confidence mappings
pub const VERIFICATION_CONFIDENCE: [(u8, f64); 4] = [
    (1, 0.30),  // Self-declaration
    (2, 0.60),  // Peer verification
    (3, 0.80),  // Assessment
    (4, 0.95),  // Document verification
];

/// Supported languages for refugees
pub const SUPPORTED_LANGUAGES: &[(&str, &str)] = &[
    ("ar", "العربية"),           // Arabic
    ("uk", "Українська"),        // Ukrainian
    ("fa", "فارسی"),              // Persian/Dari
    ("ps", "پښتو"),               // Pashto
    ("ti", "ትግርኛ"),              // Tigrinya
    ("so", "Soomaali"),          // Somali
    ("fr", "Français"),          // French
    ("es", "Español"),           // Spanish
    ("en", "English"),           // English
    ("de", "Deutsch"),           // German
    ("tr", "Türkçe"),            // Turkish
];

// ============================================================================
// Utility Functions
// ============================================================================

/// Get base confidence for verification level
pub fn get_base_confidence(level: u8) -> f64 {
    VERIFICATION_CONFIDENCE
        .iter()
        .find(|(l, _)| *l == level)
        .map(|(_, c)| *c)
        .unwrap_or(0.30)
}

/// Validate DID format (did:wia:...)
pub fn validate_did(did: &str) -> bool {
    did.starts_with("did:wia:") && did.len() > 12
}

/// Generate a new DID
pub fn generate_did() -> String {
    let uuid = uuid::Uuid::now_v7();
    format!("did:wia:{}", base64_url_encode(&uuid.as_bytes()[..]))
}

/// Base64 URL-safe encoding
pub fn base64_url_encode(data: &[u8]) -> String {
    base64::Engine::encode(&base64::engine::general_purpose::URL_SAFE_NO_PAD, data)
}

/// Classify verification level
pub fn classify_verification_level(confidence: f64) -> VerificationLevelClassification {
    match confidence {
        c if c >= 0.90 => VerificationLevelClassification::VeryHigh,
        c if c >= 0.75 => VerificationLevelClassification::High,
        c if c >= 0.60 => VerificationLevelClassification::Moderate,
        c if c >= 0.40 => VerificationLevelClassification::Low,
        _ => VerificationLevelClassification::VeryLow,
    }
}

/// Verification level classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum VerificationLevelClassification {
    VeryHigh,   // 0.90-0.99 - Direct recognition
    High,       // 0.75-0.89 - Conditional recognition
    Moderate,   // 0.60-0.74 - Requires additional verification
    Low,        // 0.40-0.59 - Limited acceptance
    VeryLow,    // 0.00-0.39 - Initial claims only
}

impl VerificationLevelClassification {
    pub fn description(&self) -> &'static str {
        match self {
            Self::VeryHigh => "Direct credential recognition",
            Self::High => "Conditional recognition",
            Self::Moderate => "Requires additional verification",
            Self::Low => "Limited acceptance",
            Self::VeryLow => "Initial claims only",
        }
    }
}

/// Fuzzy string matching for institution names
pub fn fuzzy_match(query: &str, target: &str) -> f64 {
    let query_lower = query.to_lowercase();
    let target_lower = target.to_lowercase();

    // Use Jaro-Winkler similarity
    strsim::jaro_winkler(&query_lower, &target_lower)
}

/// Calculate year overlap between two ranges
pub fn calculate_year_overlap(range1: (u32, Option<u32>), range2: (u32, Option<u32>)) -> f64 {
    let (start1, end1) = range1;
    let (start2, end2) = range2;

    let end1 = end1.unwrap_or(start1 + 4);
    let end2 = end2.unwrap_or(start2 + 4);

    let overlap_start = start1.max(start2);
    let overlap_end = end1.min(end2);

    if overlap_start > overlap_end {
        return 0.0;
    }

    let overlap = (overlap_end - overlap_start) as f64;
    let total = (end1 - start1).max(end2 - start2) as f64;

    if total > 0.0 { overlap / total } else { 0.0 }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_base_confidence() {
        assert_eq!(get_base_confidence(1), 0.30);
        assert_eq!(get_base_confidence(2), 0.60);
        assert_eq!(get_base_confidence(3), 0.80);
        assert_eq!(get_base_confidence(4), 0.95);
    }

    #[test]
    fn test_validate_did() {
        assert!(validate_did("did:wia:z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH"));
        assert!(!validate_did("invalid"));
        assert!(!validate_did("did:other:123"));
    }

    #[test]
    fn test_fuzzy_match() {
        assert!(fuzzy_match("Damascus University", "University of Damascus") > 0.7);
        assert!(fuzzy_match("Kyiv National", "Київський національний") < 0.5);
    }

    #[test]
    fn test_year_overlap() {
        assert!(calculate_year_overlap((2003, Some(2009)), (2005, Some(2011))) > 0.5);
        assert_eq!(calculate_year_overlap((2000, Some(2005)), (2010, Some(2015))), 0.0);
    }
}
