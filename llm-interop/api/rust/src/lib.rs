//! WIA LLM Interoperability Standard
//!
//! AI들의 연동 표준 - 언어, 국경, 회사의 벽을 넘어 모든 AI가 협업합니다.
//!
//! 홍익인간 (弘益人間) - Benefit All Humanity

pub mod core;
pub mod server;
pub mod types;

pub use core::*;
pub use types::*;

/// WIA-LLM-INTEROP version
pub const VERSION: &str = "1.0.0";

/// Philosophy
pub const PHILOSOPHY: &str = "홍익인간 (弘益人間) - Benefit All Humanity";

/// WIA Version for protocol
pub const WIA_VERSION: &str = "1.0";

/// Supported capability levels
pub const CAPABILITY_LEVELS: [u8; 4] = [1, 2, 3, 4];

/// Standard domains
pub const STANDARD_DOMAINS: &[&str] = &[
    "code",
    "math",
    "science",
    "engineering",
    "data_science",
    "cybersecurity",
    "medical",
    "legal",
    "finance",
    "education",
    "writing",
    "translation",
    "marketing",
    "general",
    "reasoning",
    "research",
];

/// Standard error codes
pub mod error_codes {
    // 1xx: Capability errors
    pub const E100_CAPABILITY_MISMATCH: &str = "E100";
    pub const E101_UNSUPPORTED_LANGUAGE: &str = "E101";
    pub const E102_UNSUPPORTED_MODALITY: &str = "E102";
    pub const E103_TOOL_UNAVAILABLE: &str = "E103";

    // 2xx: Message errors
    pub const E200_INVALID_FORMAT: &str = "E200";
    pub const E201_MISSING_FIELD: &str = "E201";
    pub const E202_INVALID_PAYLOAD: &str = "E202";
    pub const E203_MESSAGE_TOO_LARGE: &str = "E203";

    // 3xx: Resource errors
    pub const E300_RATE_LIMITED: &str = "E300";
    pub const E301_CONTEXT_TOO_LONG: &str = "E301";
    pub const E302_TIMEOUT: &str = "E302";
    pub const E303_QUOTA_EXCEEDED: &str = "E303";

    // 4xx: Auth errors
    pub const E400_AUTH_FAILED: &str = "E400";
    pub const E401_AUTH_EXPIRED: &str = "E401";
    pub const E402_PERMISSION_DENIED: &str = "E402";

    // 5xx: Service errors
    pub const E500_SERVICE_UNAVAILABLE: &str = "E500";
    pub const E501_INTERNAL_ERROR: &str = "E501";
    pub const E502_UPSTREAM_ERROR: &str = "E502";

    // 6xx: Federation errors
    pub const E600_FEDERATION_NOT_FOUND: &str = "E600";
    pub const E601_TASK_FAILED: &str = "E601";
    pub const E602_CONSENSUS_FAILED: &str = "E602";
    pub const E603_NODE_UNAVAILABLE: &str = "E603";

    // 7xx: Quality errors
    pub const E700_CONFIDENCE_TOO_LOW: &str = "E700";
    pub const E701_CANNOT_VERIFY: &str = "E701";
    pub const E702_CONFLICTING_SOURCES: &str = "E702";
}

/// Generate a new DID for an LLM
pub fn generate_did(provider: &str, model: &str) -> String {
    format!("did:wia:llm:{}:{}", provider, model)
}

/// Generate a unique message ID (UUID v7)
pub fn generate_message_id() -> uuid::Uuid {
    uuid::Uuid::now_v7()
}

/// Generate a trace ID for conversation tracking
pub fn generate_trace_id() -> uuid::Uuid {
    uuid::Uuid::now_v7()
}

/// Calculate capability match score
pub fn calculate_match_score(
    required_domains: &[String],
    candidate_domains: &[(String, f64)],
    min_level: u8,
    candidate_level: u8,
) -> (f64, bool) {
    // Check level requirement
    if candidate_level < min_level {
        return (0.0, false);
    }

    // Check domain requirements
    let mut domain_score = 0.0;
    let mut matched_count = 0;

    for required in required_domains {
        for (domain, proficiency) in candidate_domains {
            if domain == required {
                domain_score += proficiency;
                matched_count += 1;
                break;
            }
        }
    }

    // All required domains must be present
    if matched_count < required_domains.len() {
        return (0.0, false);
    }

    // Calculate average domain score
    let avg_domain_score = if !required_domains.is_empty() {
        domain_score / required_domains.len() as f64
    } else {
        1.0
    };

    // Level score (normalized to 0-1)
    let level_score = candidate_level as f64 / 4.0;

    // Combined score (weighted)
    let final_score = avg_domain_score * 0.6 + level_score * 0.4;

    (final_score, true)
}

/// Fuzzy string matching for capability search
pub fn fuzzy_match(query: &str, target: &str) -> f64 {
    strsim::jaro_winkler(query.to_lowercase().as_str(), target.to_lowercase().as_str())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_did() {
        let did = generate_did("anthropic", "claude-opus");
        assert_eq!(did, "did:wia:llm:anthropic:claude-opus");
    }

    #[test]
    fn test_match_score_basic() {
        let required = vec!["code".to_string(), "math".to_string()];
        let candidate = vec![
            ("code".to_string(), 0.9),
            ("math".to_string(), 0.8),
            ("general".to_string(), 0.7),
        ];

        let (score, eligible) = calculate_match_score(&required, &candidate, 3, 4);
        assert!(eligible);
        assert!(score > 0.7);
    }

    #[test]
    fn test_match_score_level_too_low() {
        let required = vec!["code".to_string()];
        let candidate = vec![("code".to_string(), 0.9)];

        let (score, eligible) = calculate_match_score(&required, &candidate, 4, 2);
        assert!(!eligible);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn test_match_score_missing_domain() {
        let required = vec!["medical".to_string()];
        let candidate = vec![("code".to_string(), 0.9)];

        let (score, eligible) = calculate_match_score(&required, &candidate, 1, 4);
        assert!(!eligible);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn test_fuzzy_match() {
        let score = fuzzy_match("claude", "Claude Opus");
        assert!(score > 0.7);
    }
}
