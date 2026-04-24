//! WIA Security Event Validator

use regex::Regex;
use std::sync::LazyLock;

use crate::types::WiaSecurityEvent;

// ============================================================================
// Validation Patterns
// ============================================================================

static UUID_PATTERN: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$").unwrap()
});

static ISO8601_PATTERN: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$").unwrap()
});

static SEMVER_PATTERN: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^\d+\.\d+\.\d+$").unwrap());

static TACTIC_PATTERN: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^TA\d{4}$").unwrap());

static TECHNIQUE_PATTERN: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^T\d{4}(\.\d{3})?$").unwrap());

static CVE_PATTERN: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^CVE-\d{4}-\d{4,}$").unwrap());

// ============================================================================
// Validation Result
// ============================================================================

/// Result of event validation
#[derive(Debug, Clone, Default)]
pub struct ValidationResult {
    pub valid: bool,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

impl ValidationResult {
    pub fn new() -> Self {
        Self {
            valid: true,
            errors: Vec::new(),
            warnings: Vec::new(),
        }
    }

    pub fn is_valid(&self) -> bool {
        self.valid
    }

    pub fn add_error(&mut self, error: impl Into<String>) {
        self.errors.push(error.into());
        self.valid = false;
    }

    pub fn add_warning(&mut self, warning: impl Into<String>) {
        self.warnings.push(warning.into());
    }
}

// ============================================================================
// Validator
// ============================================================================

/// Validate a WIA Security Event
pub fn validate_event(event: &WiaSecurityEvent) -> ValidationResult {
    let mut result = ValidationResult::new();

    // Validate version
    if !SEMVER_PATTERN.is_match(&event.version) {
        result.add_error("Invalid version format. Expected SemVer (e.g., 1.0.0)");
    }

    // Validate ID
    let id_str = event.id.to_string().to_lowercase();
    if !UUID_PATTERN.is_match(&id_str) {
        result.add_error("Invalid id format. Expected UUID v4");
    }

    // Validate timestamp
    if !ISO8601_PATTERN.is_match(&event.timestamp) {
        result.add_error("Invalid timestamp format. Expected ISO 8601");
    }

    // Validate severity
    if event.severity < 0.0 || event.severity > 10.0 {
        result.add_error("Invalid severity. Expected number between 0 and 10");
    }

    // Validate MITRE mapping
    if let Some(ref mitre) = event.mitre {
        if let Some(ref tactic) = mitre.tactic {
            if !TACTIC_PATTERN.is_match(tactic) {
                result.add_error(format!("Invalid MITRE tactic ID format: {}", tactic));
            }
        }
        if let Some(ref technique) = mitre.technique {
            if !TECHNIQUE_PATTERN.is_match(technique) {
                result.add_error(format!("Invalid MITRE technique ID format: {}", technique));
            }
        }
    }

    // Validate meta
    if let Some(ref meta) = event.meta {
        if let Some(confidence) = meta.confidence {
            if confidence < 0.0 || confidence > 1.0 {
                result.add_error("meta.confidence must be between 0 and 1");
            }
        }
    }

    // Validate event-specific data
    validate_event_data(&event.event_type, &event.data, &mut result);

    // Warnings
    if event.schema.is_none() {
        result.add_warning("Recommended field $schema is missing");
    }

    result
}

fn validate_event_data(
    event_type: &crate::types::EventType,
    data: &serde_json::Value,
    result: &mut ValidationResult,
) {
    use crate::types::EventType;

    match event_type {
        EventType::Alert => validate_alert_data(data, result),
        EventType::ThreatIntel => validate_threat_intel_data(data, result),
        EventType::Vulnerability => validate_vulnerability_data(data, result),
        EventType::Incident => validate_incident_data(data, result),
        EventType::NetworkEvent => validate_network_event_data(data, result),
        EventType::EndpointEvent => validate_endpoint_event_data(data, result),
        EventType::AuthEvent => validate_auth_event_data(data, result),
    }
}

fn validate_alert_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["alert_id", "title", "category", "status", "priority"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in alert data: {}", field));
        }
    }

    let valid_categories = ["malware", "intrusion", "policy", "reconnaissance", "other"];
    if let Some(category) = data.get("category").and_then(|v| v.as_str()) {
        if !valid_categories.contains(&category) {
            result.add_error(format!("Invalid alert category: {}", category));
        }
    }

    let valid_statuses = ["new", "investigating", "resolved", "false_positive", "closed"];
    if let Some(status) = data.get("status").and_then(|v| v.as_str()) {
        if !valid_statuses.contains(&status) {
            result.add_error(format!("Invalid alert status: {}", status));
        }
    }
}

fn validate_threat_intel_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["threat_type", "threat_name", "status"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in threat_intel data: {}", field));
        }
    }

    let valid_types = ["malware", "apt", "campaign", "botnet", "ransomware", "phishing"];
    if let Some(threat_type) = data.get("threat_type").and_then(|v| v.as_str()) {
        if !valid_types.contains(&threat_type) {
            result.add_error(format!("Invalid threat_type: {}", threat_type));
        }
    }
}

fn validate_vulnerability_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["vuln_id", "title", "cvss"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in vulnerability data: {}", field));
        }
    }

    if let Some(vuln_id) = data.get("vuln_id").and_then(|v| v.as_str()) {
        if !CVE_PATTERN.is_match(vuln_id) {
            result.add_error(format!("Invalid CVE format: {}", vuln_id));
        }
    }
}

fn validate_incident_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["incident_id", "title", "category", "status", "priority"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in incident data: {}", field));
        }
    }
}

fn validate_network_event_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["event_type", "protocol", "source", "destination"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in network_event data: {}", field));
        }
    }
}

fn validate_endpoint_event_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["event_type", "host"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in endpoint_event data: {}", field));
        }
    }
}

fn validate_auth_event_data(data: &serde_json::Value, result: &mut ValidationResult) {
    let required = ["event_type", "result", "user", "target"];
    for field in required {
        if data.get(field).is_none() {
            result.add_error(format!("Missing required field in auth_event data: {}", field));
        }
    }

    if let Some(risk_score) = data.get("risk_score").and_then(|v| v.as_f64()) {
        if risk_score < 0.0 || risk_score > 1.0 {
            result.add_error("risk_score must be between 0 and 1");
        }
    }
}

// ============================================================================
// Type Guards
// ============================================================================

/// Check if event is an alert
pub fn is_alert_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::Alert)
}

/// Check if event is threat intelligence
pub fn is_threat_intel_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::ThreatIntel)
}

/// Check if event is a vulnerability
pub fn is_vulnerability_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::Vulnerability)
}

/// Check if event is an incident
pub fn is_incident_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::Incident)
}

/// Check if event is a network event
pub fn is_network_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::NetworkEvent)
}

/// Check if event is an endpoint event
pub fn is_endpoint_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::EndpointEvent)
}

/// Check if event is an auth event
pub fn is_auth_event(event: &WiaSecurityEvent) -> bool {
    matches!(event.event_type, crate::types::EventType::AuthEvent)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uuid_pattern() {
        assert!(UUID_PATTERN.is_match("550e8400-e29b-41d4-a716-446655440000"));
        assert!(!UUID_PATTERN.is_match("invalid-uuid"));
    }

    #[test]
    fn test_iso8601_pattern() {
        assert!(ISO8601_PATTERN.is_match("2025-12-14T10:30:00.000Z"));
        assert!(ISO8601_PATTERN.is_match("2025-12-14T10:30:00Z"));
        assert!(!ISO8601_PATTERN.is_match("2025-12-14"));
    }
}
