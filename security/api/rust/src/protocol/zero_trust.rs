//! Zero Trust Protocol
//!
//! Implementation of Zero Trust access decision protocol based on NIST SP 800-207.

use chrono::Utc;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

// ============================================================================
// Access Subject
// ============================================================================

/// Device posture information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DevicePosture {
    /// Operating system version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub os_version: Option<String>,
    /// Last patch date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub patch_level: Option<String>,
    /// Antivirus status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub antivirus_status: Option<String>,
    /// Disk encryption enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub disk_encryption: Option<bool>,
    /// Firewall enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub firewall_enabled: Option<bool>,
    /// Trust score (0.0 - 1.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trust_score: Option<f64>,
}

impl DevicePosture {
    /// Create a new device posture
    pub fn new() -> Self {
        Self::default()
    }

    /// Set trust score
    pub fn with_trust_score(mut self, score: f64) -> Self {
        self.trust_score = Some(score.clamp(0.0, 1.0));
        self
    }

    /// Check if device is compliant (trust score >= threshold)
    pub fn is_compliant(&self, threshold: f64) -> bool {
        self.trust_score.map(|s| s >= threshold).unwrap_or(false)
    }
}

/// Network location information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NetworkLocation {
    /// IP address
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    /// Country code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub country: Option<String>,
    /// City
    #[serde(skip_serializing_if = "Option::is_none")]
    pub city: Option<String>,
    /// Network type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub network_type: Option<String>,
}

/// Authentication information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AuthenticationInfo {
    /// Authentication method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    /// Authentication factors used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub factors: Option<Vec<String>>,
    /// Authentication time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth_time: Option<String>,
    /// Session ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
}

impl AuthenticationInfo {
    /// Check if MFA was used
    pub fn is_mfa(&self) -> bool {
        self.factors.as_ref().map(|f| f.len() >= 2).unwrap_or(false)
    }
}

/// Risk signals
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RiskSignals {
    /// Impossible travel detected
    #[serde(skip_serializing_if = "Option::is_none")]
    pub impossible_travel: Option<bool>,
    /// Unusual time access
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unusual_time: Option<bool>,
    /// Unusual device
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unusual_device: Option<bool>,
    /// Compromised credential detected
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compromised_credential: Option<bool>,
    /// Anomaly score (0.0 - 1.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anomaly_score: Option<f64>,
}

impl RiskSignals {
    /// Calculate overall risk level
    pub fn risk_level(&self) -> RiskLevel {
        let mut score = 0.0;

        if self.impossible_travel.unwrap_or(false) {
            score += 0.4;
        }
        if self.unusual_time.unwrap_or(false) {
            score += 0.1;
        }
        if self.unusual_device.unwrap_or(false) {
            score += 0.2;
        }
        if self.compromised_credential.unwrap_or(false) {
            score += 0.5;
        }
        if let Some(anomaly) = self.anomaly_score {
            score += anomaly * 0.3;
        }

        if score >= 0.7 {
            RiskLevel::Critical
        } else if score >= 0.5 {
            RiskLevel::High
        } else if score >= 0.3 {
            RiskLevel::Medium
        } else {
            RiskLevel::Low
        }
    }
}

/// Risk level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RiskLevel {
    Low,
    Medium,
    High,
    Critical,
}

/// Access subject (who is requesting access)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessSubject {
    /// User ID
    pub user_id: String,
    /// Device ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_id: Option<String>,
    /// Device posture
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_posture: Option<DevicePosture>,
    /// Network location
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<NetworkLocation>,
    /// Authentication info
    #[serde(skip_serializing_if = "Option::is_none")]
    pub authentication: Option<AuthenticationInfo>,
    /// Risk signals
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_signals: Option<RiskSignals>,
}

impl AccessSubject {
    /// Create a new access subject
    pub fn new(user_id: impl Into<String>) -> Self {
        Self {
            user_id: user_id.into(),
            device_id: None,
            device_posture: None,
            location: None,
            authentication: None,
            risk_signals: None,
        }
    }

    /// Set device ID
    pub fn with_device(mut self, device_id: impl Into<String>) -> Self {
        self.device_id = Some(device_id.into());
        self
    }

    /// Set device posture
    pub fn with_posture(mut self, posture: DevicePosture) -> Self {
        self.device_posture = Some(posture);
        self
    }
}

// ============================================================================
// Access Resource
// ============================================================================

/// Resource classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResourceClassification {
    Public,
    Internal,
    Confidential,
    Restricted,
    TopSecret,
}

/// Environment type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Environment {
    Development,
    Staging,
    Production,
}

/// Access resource (what is being accessed)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessResource {
    /// Resource type
    #[serde(rename = "type")]
    pub resource_type: String,
    /// Resource ID
    pub id: String,
    /// Classification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub classification: Option<ResourceClassification>,
    /// Environment
    #[serde(skip_serializing_if = "Option::is_none")]
    pub environment: Option<Environment>,
    /// Location
    #[serde(skip_serializing_if = "Option::is_none")]
    pub location: Option<String>,
}

impl AccessResource {
    /// Create a new resource
    pub fn new(resource_type: impl Into<String>, id: impl Into<String>) -> Self {
        Self {
            resource_type: resource_type.into(),
            id: id.into(),
            classification: None,
            environment: None,
            location: None,
        }
    }

    /// Set classification
    pub fn with_classification(mut self, classification: ResourceClassification) -> Self {
        self.classification = Some(classification);
        self
    }

    /// Set environment
    pub fn with_environment(mut self, environment: Environment) -> Self {
        self.environment = Some(environment);
        self
    }
}

// ============================================================================
// Access Context
// ============================================================================

/// Access request context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessContext {
    /// Request time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub request_time: Option<String>,
    /// Day type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub day_type: Option<String>,
    /// Business justification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub business_justification: Option<String>,
}

// ============================================================================
// Access Decision Request
// ============================================================================

/// Access decision request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessDecisionRequest {
    /// Request ID
    pub request_id: Uuid,
    /// Subject (who)
    pub subject: AccessSubject,
    /// Resource (what)
    pub resource: AccessResource,
    /// Action (how)
    pub action: String,
    /// Context
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<AccessContext>,
}

impl AccessDecisionRequest {
    /// Create a new access decision request
    pub fn new(
        subject: AccessSubject,
        resource: AccessResource,
        action: impl Into<String>,
    ) -> Self {
        Self {
            request_id: Uuid::new_v4(),
            subject,
            resource,
            action: action.into(),
            context: None,
        }
    }

    /// Set context
    pub fn with_context(mut self, context: AccessContext) -> Self {
        self.context = Some(context);
        self
    }
}

// ============================================================================
// Access Decision Response
// ============================================================================

/// Access decision result
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AccessDecision {
    Permit,
    Deny,
    Indeterminate,
    NotApplicable,
}

/// Access condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessCondition {
    /// Condition type
    #[serde(rename = "type")]
    pub condition_type: String,
    /// Constraint name
    pub constraint: String,
    /// Constraint value
    pub value: serde_json::Value,
}

impl AccessCondition {
    /// Create time limit condition
    pub fn time_limit(expires_at: impl Into<String>) -> Self {
        Self {
            condition_type: "time_limit".to_string(),
            constraint: "session_expires_at".to_string(),
            value: serde_json::json!(expires_at.into()),
        }
    }

    /// Create continuous auth condition
    pub fn continuous_auth(interval_minutes: u32) -> Self {
        Self {
            condition_type: "continuous_auth".to_string(),
            constraint: "reauth_interval_minutes".to_string(),
            value: serde_json::json!(interval_minutes),
        }
    }

    /// Create data limit condition
    pub fn data_limit(max_records: u32) -> Self {
        Self {
            condition_type: "data_limit".to_string(),
            constraint: "max_records_per_query".to_string(),
            value: serde_json::json!(max_records),
        }
    }
}

/// Access obligation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessObligation {
    /// Obligation type
    #[serde(rename = "type")]
    pub obligation_type: String,
    /// Action to perform
    pub action: String,
    /// Target component
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    /// Trigger condition
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition: Option<String>,
    /// Parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<HashMap<String, serde_json::Value>>,
}

impl AccessObligation {
    /// Create logging obligation
    pub fn logging(target: impl Into<String>, detail_level: impl Into<String>) -> Self {
        let mut params = HashMap::new();
        params.insert("detail_level".to_string(), serde_json::json!(detail_level.into()));

        Self {
            obligation_type: "logging".to_string(),
            action: "log_all_queries".to_string(),
            target: Some(target.into()),
            condition: None,
            parameters: Some(params),
        }
    }

    /// Create alerting obligation
    pub fn alerting(condition: impl Into<String>, target: impl Into<String>) -> Self {
        Self {
            obligation_type: "alerting".to_string(),
            action: "alert_on_anomaly".to_string(),
            target: Some(target.into()),
            condition: Some(condition.into()),
            parameters: None,
        }
    }

    /// Create masking obligation
    pub fn masking(fields: Vec<String>) -> Self {
        let mut params = HashMap::new();
        params.insert("fields".to_string(), serde_json::json!(fields));

        Self {
            obligation_type: "masking".to_string(),
            action: "mask_pii".to_string(),
            target: None,
            condition: None,
            parameters: Some(params),
        }
    }
}

/// Access decision response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessDecisionResponse {
    /// Decision ID
    pub decision_id: Uuid,
    /// Decision result
    pub decision: AccessDecision,
    /// Conditions (if permitted)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions: Option<Vec<AccessCondition>>,
    /// Obligations (if permitted)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub obligations: Option<Vec<AccessObligation>>,
    /// Valid from
    #[serde(skip_serializing_if = "Option::is_none")]
    pub valid_from: Option<String>,
    /// Valid until
    #[serde(skip_serializing_if = "Option::is_none")]
    pub valid_until: Option<String>,
    /// Session token
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_token: Option<String>,
    /// Decision time (ms)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub decision_time_ms: Option<u32>,
}

impl AccessDecisionResponse {
    /// Create permit response
    pub fn permit() -> Self {
        Self {
            decision_id: Uuid::new_v4(),
            decision: AccessDecision::Permit,
            conditions: None,
            obligations: None,
            valid_from: Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()),
            valid_until: None,
            session_token: None,
            decision_time_ms: None,
        }
    }

    /// Create deny response
    pub fn deny() -> Self {
        Self {
            decision_id: Uuid::new_v4(),
            decision: AccessDecision::Deny,
            conditions: None,
            obligations: None,
            valid_from: None,
            valid_until: None,
            session_token: None,
            decision_time_ms: None,
        }
    }

    /// Add conditions
    pub fn with_conditions(mut self, conditions: Vec<AccessCondition>) -> Self {
        self.conditions = Some(conditions);
        self
    }

    /// Add obligations
    pub fn with_obligations(mut self, obligations: Vec<AccessObligation>) -> Self {
        self.obligations = Some(obligations);
        self
    }

    /// Set validity period
    pub fn valid_for_hours(mut self, hours: i64) -> Self {
        use chrono::Duration;
        let now = Utc::now();
        self.valid_from = Some(now.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string());
        self.valid_until = Some((now + Duration::hours(hours)).format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string());
        self
    }
}

// ============================================================================
// Policy Definition
// ============================================================================

/// Policy effect
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyEffect {
    Permit,
    Deny,
}

/// Policy condition operator
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConditionOperator {
    Eq,
    Ne,
    Gt,
    Gte,
    Lt,
    Lte,
    In,
    NotIn,
    Contains,
    Between,
    Matches,
}

/// Policy condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyCondition {
    /// Attribute name
    pub attribute: String,
    /// Operator
    pub operator: ConditionOperator,
    /// Value
    pub value: serde_json::Value,
}

/// Policy definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyDefinition {
    /// Policy ID
    pub policy_id: String,
    /// Policy name
    pub name: String,
    /// Version
    pub version: String,
    /// Effect
    pub effect: PolicyEffect,
    /// Subject conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subjects: Option<PolicyConditionSet>,
    /// Resource conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resources: Option<PolicyConditionSet>,
    /// Allowed actions
    pub actions: Vec<String>,
    /// Additional conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditions: Option<PolicyConditionSet>,
    /// Obligations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub obligations: Option<Vec<AccessObligation>>,
    /// Priority (higher = more important)
    pub priority: i32,
    /// Enabled
    pub enabled: bool,
}

/// Policy condition set
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyConditionSet {
    /// Match mode
    #[serde(rename = "match")]
    pub match_mode: MatchMode,
    /// Conditions
    pub conditions: Vec<PolicyCondition>,
}

/// Match mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MatchMode {
    All,
    Any,
}

impl PolicyDefinition {
    /// Create a new policy
    pub fn new(
        policy_id: impl Into<String>,
        name: impl Into<String>,
        effect: PolicyEffect,
    ) -> Self {
        Self {
            policy_id: policy_id.into(),
            name: name.into(),
            version: "1.0.0".to_string(),
            effect,
            subjects: None,
            resources: None,
            actions: vec![],
            conditions: None,
            obligations: None,
            priority: 0,
            enabled: true,
        }
    }

    /// Set actions
    pub fn with_actions(mut self, actions: Vec<String>) -> Self {
        self.actions = actions;
        self
    }

    /// Set priority
    pub fn with_priority(mut self, priority: i32) -> Self {
        self.priority = priority;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_access_decision_request() {
        let subject = AccessSubject::new("user@example.com")
            .with_device("device-001")
            .with_posture(DevicePosture::new().with_trust_score(0.9));

        let resource = AccessResource::new("database", "customer-db")
            .with_classification(ResourceClassification::Confidential)
            .with_environment(Environment::Production);

        let request = AccessDecisionRequest::new(subject, resource, "read");

        assert_eq!(request.action, "read");
        assert_eq!(request.subject.user_id, "user@example.com");
    }

    #[test]
    fn test_access_decision_response() {
        let response = AccessDecisionResponse::permit()
            .with_conditions(vec![
                AccessCondition::time_limit("2024-12-14T18:00:00Z"),
                AccessCondition::continuous_auth(30),
            ])
            .with_obligations(vec![
                AccessObligation::logging("siem-central", "verbose"),
            ])
            .valid_for_hours(8);

        assert_eq!(response.decision, AccessDecision::Permit);
        assert!(response.conditions.is_some());
        assert!(response.obligations.is_some());
    }

    #[test]
    fn test_device_posture_compliance() {
        let posture = DevicePosture::new().with_trust_score(0.85);
        assert!(posture.is_compliant(0.8));
        assert!(!posture.is_compliant(0.9));
    }

    #[test]
    fn test_risk_signals() {
        let signals = RiskSignals {
            impossible_travel: Some(true),
            unusual_time: Some(false),
            unusual_device: Some(false),
            compromised_credential: Some(false),
            anomaly_score: Some(0.2),
        };

        assert_eq!(signals.risk_level(), RiskLevel::Medium);
    }
}
