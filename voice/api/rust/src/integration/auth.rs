//! Authentication and authorization for WIA integration

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// JWT claims for WIA authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaClaims {
    /// Subject (user ID or service ID)
    pub sub: String,

    /// Issuer
    pub iss: String,

    /// Audience
    pub aud: String,

    /// Expiration time
    pub exp: i64,

    /// Issued at
    pub iat: i64,

    /// WIA user ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wia_id: Option<String>,

    /// Service ID (for service-to-service auth)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub service_id: Option<String>,

    /// Scopes
    pub scopes: Vec<String>,

    /// Connected devices
    #[serde(skip_serializing_if = "Option::is_none")]
    pub devices: Option<Vec<String>>,
}

impl WiaClaims {
    /// Check if token is expired
    pub fn is_expired(&self) -> bool {
        let now = Utc::now().timestamp();
        self.exp < now
    }

    /// Check if claim has scope
    pub fn has_scope(&self, scope: &str) -> bool {
        self.scopes.iter().any(|s| s == scope || s == "*")
    }

    /// Check if claim has any of the scopes
    pub fn has_any_scope(&self, scopes: &[&str]) -> bool {
        scopes.iter().any(|s| self.has_scope(s))
    }

    /// Check if claim has all scopes
    pub fn has_all_scopes(&self, scopes: &[&str]) -> bool {
        scopes.iter().all(|s| self.has_scope(s))
    }
}

/// WIA scopes
pub mod scopes {
    /// OpenID Connect scopes
    pub const OPENID: &str = "openid";
    pub const PROFILE: &str = "profile";

    /// Voice-Sign scopes
    pub const VOICE_SIGN_READ: &str = "wia.voice-sign:read";
    pub const VOICE_SIGN_WRITE: &str = "wia.voice-sign:write";
    pub const VOICE_SIGN_ADMIN: &str = "wia.voice-sign:admin";

    /// Exoskeleton scopes
    pub const EXOSKELETON_READ: &str = "wia.exoskeleton:read";
    pub const EXOSKELETON_CONTROL: &str = "wia.exoskeleton:control";

    /// Bionic Eye scopes
    pub const BIONIC_EYE_READ: &str = "wia.bionic-eye:read";
    pub const BIONIC_EYE_DISPLAY: &str = "wia.bionic-eye:display";

    /// Integration scopes
    pub const INTEGRATION: &str = "wia.integration";
    pub const INTEGRATION_EMERGENCY: &str = "wia.integration:emergency";
}

/// API key for service authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiKey {
    /// Key ID
    pub key_id: String,

    /// Hashed key value
    pub key_hash: String,

    /// Service name
    pub service_name: String,

    /// Allowed scopes
    pub scopes: Vec<String>,

    /// Created at
    pub created_at: DateTime<Utc>,

    /// Expires at
    pub expires_at: Option<DateTime<Utc>>,

    /// Is active
    pub active: bool,

    /// Rate limit (requests per minute)
    pub rate_limit: Option<u32>,
}

impl ApiKey {
    /// Check if key is valid
    pub fn is_valid(&self) -> bool {
        if !self.active {
            return false;
        }

        if let Some(expires) = self.expires_at {
            if Utc::now() > expires {
                return false;
            }
        }

        true
    }
}

/// Authorization policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthorizationPolicy {
    /// Policy name
    pub name: String,

    /// Required scopes
    pub required_scopes: Vec<String>,

    /// Required conditions
    pub conditions: Vec<PolicyCondition>,

    /// Override conditions (bypass normal permissions)
    pub override_conditions: Option<Vec<PolicyCondition>>,
}

/// Policy condition
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PolicyCondition {
    /// User consent required
    UserConsent { required: bool },

    /// Device must be paired
    DevicePaired { device_type: String },

    /// Safety check must pass
    SafetyCheck { check_type: String },

    /// Emergency mode
    EmergencyDetected { urgency_minimum: String },

    /// Time-based restriction
    TimeRestriction {
        allowed_hours: (u32, u32),
        timezone: String,
    },
}

/// Authorization result
#[derive(Debug, Clone)]
pub struct AuthorizationResult {
    /// Is authorized
    pub authorized: bool,

    /// Reason (if denied)
    pub reason: Option<String>,

    /// Missing requirements
    pub missing_requirements: Vec<String>,

    /// Applied policy
    pub policy: Option<String>,
}

impl AuthorizationResult {
    /// Create allowed result
    pub fn allowed(policy: &str) -> Self {
        Self {
            authorized: true,
            reason: None,
            missing_requirements: vec![],
            policy: Some(policy.to_string()),
        }
    }

    /// Create denied result
    pub fn denied(reason: &str, missing: Vec<String>) -> Self {
        Self {
            authorized: false,
            reason: Some(reason.to_string()),
            missing_requirements: missing,
            policy: None,
        }
    }
}

/// Authorization context
#[derive(Debug, Clone)]
pub struct AuthorizationContext {
    /// User/service claims
    pub claims: WiaClaims,

    /// User consent status
    pub user_consent: HashMap<String, bool>,

    /// Paired devices
    pub paired_devices: HashSet<String>,

    /// Safety check results
    pub safety_checks: HashMap<String, bool>,

    /// Emergency status
    pub emergency_active: bool,
    pub emergency_urgency: Option<String>,
}

/// Authorization engine
pub struct AuthorizationEngine {
    policies: HashMap<String, AuthorizationPolicy>,
}

impl AuthorizationEngine {
    /// Create a new authorization engine
    pub fn new() -> Self {
        Self {
            policies: HashMap::new(),
        }
    }

    /// Add a policy
    pub fn add_policy(&mut self, policy: AuthorizationPolicy) {
        self.policies.insert(policy.name.clone(), policy);
    }

    /// Load default policies
    pub fn load_default_policies(&mut self) {
        // Voice-Sign to Exoskeleton policy
        self.add_policy(AuthorizationPolicy {
            name: "voice_sign_to_exoskeleton".to_string(),
            required_scopes: vec![
                scopes::VOICE_SIGN_READ.to_string(),
                scopes::EXOSKELETON_CONTROL.to_string(),
                scopes::INTEGRATION.to_string(),
            ],
            conditions: vec![
                PolicyCondition::UserConsent { required: true },
                PolicyCondition::DevicePaired {
                    device_type: "exoskeleton".to_string(),
                },
                PolicyCondition::SafetyCheck {
                    check_type: "gesture_safety".to_string(),
                },
            ],
            override_conditions: None,
        });

        // Voice-Sign to Bionic Eye policy
        self.add_policy(AuthorizationPolicy {
            name: "voice_sign_to_bionic_eye".to_string(),
            required_scopes: vec![
                scopes::VOICE_SIGN_READ.to_string(),
                scopes::BIONIC_EYE_DISPLAY.to_string(),
            ],
            conditions: vec![PolicyCondition::DevicePaired {
                device_type: "bionic_eye".to_string(),
            }],
            override_conditions: None,
        });

        // Emergency broadcast policy
        self.add_policy(AuthorizationPolicy {
            name: "emergency_broadcast".to_string(),
            required_scopes: vec![scopes::INTEGRATION_EMERGENCY.to_string()],
            conditions: vec![PolicyCondition::EmergencyDetected {
                urgency_minimum: "critical".to_string(),
            }],
            override_conditions: Some(vec![PolicyCondition::EmergencyDetected {
                urgency_minimum: "critical".to_string(),
            }]),
        });
    }

    /// Authorize an action
    pub fn authorize(&self, action: &str, context: &AuthorizationContext) -> AuthorizationResult {
        // Check token expiration
        if context.claims.is_expired() {
            return AuthorizationResult::denied("Token expired", vec!["valid_token".to_string()]);
        }

        // Get policy
        let policy = match self.policies.get(action) {
            Some(p) => p,
            None => return AuthorizationResult::denied("Unknown action", vec![]),
        };

        // Check override conditions first (emergency bypass)
        if let Some(ref overrides) = policy.override_conditions {
            if self.check_conditions(overrides, context) {
                return AuthorizationResult::allowed(&policy.name);
            }
        }

        // Check scopes
        let mut missing = Vec::new();
        for scope in &policy.required_scopes {
            if !context.claims.has_scope(scope) {
                missing.push(format!("scope:{}", scope));
            }
        }

        if !missing.is_empty() {
            return AuthorizationResult::denied("Missing required scopes", missing);
        }

        // Check conditions
        for condition in &policy.conditions {
            if !self.check_condition(condition, context) {
                missing.push(format!("condition:{:?}", condition));
            }
        }

        if !missing.is_empty() {
            return AuthorizationResult::denied("Conditions not met", missing);
        }

        AuthorizationResult::allowed(&policy.name)
    }

    fn check_conditions(&self, conditions: &[PolicyCondition], context: &AuthorizationContext) -> bool {
        conditions.iter().all(|c| self.check_condition(c, context))
    }

    fn check_condition(&self, condition: &PolicyCondition, context: &AuthorizationContext) -> bool {
        match condition {
            PolicyCondition::UserConsent { required } => {
                if *required {
                    context.user_consent.values().any(|&v| v)
                } else {
                    true
                }
            }
            PolicyCondition::DevicePaired { device_type } => {
                context.paired_devices.contains(device_type)
            }
            PolicyCondition::SafetyCheck { check_type } => {
                context.safety_checks.get(check_type).copied().unwrap_or(false)
            }
            PolicyCondition::EmergencyDetected { urgency_minimum } => {
                if !context.emergency_active {
                    return false;
                }
                match (context.emergency_urgency.as_deref(), urgency_minimum.as_str()) {
                    (Some("critical"), _) => true,
                    (Some("high"), "high" | "medium" | "low") => true,
                    (Some("medium"), "medium" | "low") => true,
                    (Some("low"), "low") => true,
                    _ => false,
                }
            }
            PolicyCondition::TimeRestriction { allowed_hours, .. } => {
                let hour = Utc::now().format("%H").to_string().parse::<u32>().unwrap_or(0);
                hour >= allowed_hours.0 && hour <= allowed_hours.1
            }
        }
    }
}

impl Default for AuthorizationEngine {
    fn default() -> Self {
        let mut engine = Self::new();
        engine.load_default_policies();
        engine
    }
}

/// Token validator (stub - integrate with actual JWT library)
pub struct TokenValidator {
    issuer: String,
    audience: String,
}

impl TokenValidator {
    pub fn new(issuer: String, audience: String) -> Self {
        Self { issuer, audience }
    }

    /// Validate token (stub - implement with actual JWT validation)
    pub fn validate(&self, _token: &str) -> Result<WiaClaims, AuthError> {
        // In production, decode and validate JWT
        // This is a stub that returns a sample claims object
        Err(AuthError::InvalidToken("Token validation not implemented".to_string()))
    }
}

/// Authentication error
#[derive(Debug, thiserror::Error)]
pub enum AuthError {
    #[error("Invalid token: {0}")]
    InvalidToken(String),

    #[error("Token expired")]
    TokenExpired,

    #[error("Invalid signature")]
    InvalidSignature,

    #[error("Missing required claim: {0}")]
    MissingClaim(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_claims() -> WiaClaims {
        WiaClaims {
            sub: "user-123".to_string(),
            iss: "https://id.wia.org".to_string(),
            aud: "wia-services".to_string(),
            exp: (Utc::now() + Duration::hours(1)).timestamp(),
            iat: Utc::now().timestamp(),
            wia_id: Some("wia-123".to_string()),
            service_id: None,
            scopes: vec![
                scopes::OPENID.to_string(),
                scopes::VOICE_SIGN_READ.to_string(),
                scopes::EXOSKELETON_CONTROL.to_string(),
                scopes::INTEGRATION.to_string(),
            ],
            devices: None,
        }
    }

    #[test]
    fn test_claims_scope_check() {
        let claims = sample_claims();
        assert!(claims.has_scope(scopes::VOICE_SIGN_READ));
        assert!(!claims.has_scope(scopes::VOICE_SIGN_ADMIN));
    }

    #[test]
    fn test_claims_expiration() {
        let mut claims = sample_claims();
        assert!(!claims.is_expired());

        claims.exp = (Utc::now() - Duration::hours(1)).timestamp();
        assert!(claims.is_expired());
    }

    #[test]
    fn test_authorization_engine() {
        let engine = AuthorizationEngine::default();

        let mut consent = HashMap::new();
        consent.insert("gesture".to_string(), true);

        let mut devices = HashSet::new();
        devices.insert("exoskeleton".to_string());

        let mut safety = HashMap::new();
        safety.insert("gesture_safety".to_string(), true);

        let context = AuthorizationContext {
            claims: sample_claims(),
            user_consent: consent,
            paired_devices: devices,
            safety_checks: safety,
            emergency_active: false,
            emergency_urgency: None,
        };

        let result = engine.authorize("voice_sign_to_exoskeleton", &context);
        assert!(result.authorized);
    }

    #[test]
    fn test_authorization_denied_missing_scope() {
        let engine = AuthorizationEngine::default();

        let mut claims = sample_claims();
        claims.scopes = vec![scopes::OPENID.to_string()]; // Missing required scopes

        let context = AuthorizationContext {
            claims,
            user_consent: HashMap::new(),
            paired_devices: HashSet::new(),
            safety_checks: HashMap::new(),
            emergency_active: false,
            emergency_urgency: None,
        };

        let result = engine.authorize("voice_sign_to_exoskeleton", &context);
        assert!(!result.authorized);
        assert!(result.missing_requirements.iter().any(|r| r.contains("scope")));
    }
}
