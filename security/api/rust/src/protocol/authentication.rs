//! Authentication Module
//!
//! JWT, mTLS, and message signing implementations.

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// JWT Token
// ============================================================================

/// JWT algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JwtAlgorithm {
    #[serde(rename = "EdDSA")]
    EdDsa,
    #[serde(rename = "ES256")]
    Es256,
    #[serde(rename = "RS256")]
    Rs256,
}

/// JWT header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JwtHeader {
    /// Algorithm
    pub alg: JwtAlgorithm,
    /// Token type
    pub typ: String,
    /// Key ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kid: Option<String>,
}

impl Default for JwtHeader {
    fn default() -> Self {
        Self {
            alg: JwtAlgorithm::EdDsa,
            typ: "JWT".to_string(),
            kid: None,
        }
    }
}

/// JWT claims
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JwtClaims {
    /// Issuer
    pub iss: String,
    /// Subject
    pub sub: String,
    /// Audience
    pub aud: String,
    /// Expiration time (Unix timestamp)
    pub exp: i64,
    /// Issued at (Unix timestamp)
    pub iat: i64,
    /// JWT ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jti: Option<String>,
    /// Scopes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scope: Option<Vec<String>>,
    /// Component info
    #[serde(skip_serializing_if = "Option::is_none")]
    pub component: Option<ComponentClaims>,
}

/// Component claims in JWT
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentClaims {
    /// Component ID
    pub id: String,
    /// Component type
    #[serde(rename = "type")]
    pub component_type: String,
    /// Organization
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organization: Option<String>,
}

impl JwtClaims {
    /// Create new claims
    pub fn new(
        issuer: impl Into<String>,
        subject: impl Into<String>,
        audience: impl Into<String>,
        expires_in: Duration,
    ) -> Self {
        let now = Utc::now();
        Self {
            iss: issuer.into(),
            sub: subject.into(),
            aud: audience.into(),
            exp: (now + expires_in).timestamp(),
            iat: now.timestamp(),
            jti: Some(uuid::Uuid::new_v4().to_string()),
            scope: None,
            component: None,
        }
    }

    /// Set scopes
    pub fn with_scopes(mut self, scopes: Vec<String>) -> Self {
        self.scope = Some(scopes);
        self
    }

    /// Set component info
    pub fn with_component(mut self, component: ComponentClaims) -> Self {
        self.component = Some(component);
        self
    }

    /// Check if token is expired
    pub fn is_expired(&self) -> bool {
        Utc::now().timestamp() > self.exp
    }

    /// Check if scope is allowed
    pub fn has_scope(&self, required: &str) -> bool {
        self.scope
            .as_ref()
            .map(|scopes| scopes.iter().any(|s| s == required))
            .unwrap_or(false)
    }
}

/// JWT token (unsigned representation)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JwtToken {
    /// Header
    pub header: JwtHeader,
    /// Claims
    pub claims: JwtClaims,
    /// Signature (base64 encoded)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<String>,
}

impl JwtToken {
    /// Create a new token
    pub fn new(claims: JwtClaims) -> Self {
        Self {
            header: JwtHeader::default(),
            claims,
            signature: None,
        }
    }

    /// Set key ID
    pub fn with_key_id(mut self, kid: impl Into<String>) -> Self {
        self.header.kid = Some(kid.into());
        self
    }

    /// Validate token (without signature verification)
    pub fn validate(&self) -> Result<(), TokenValidationError> {
        if self.claims.is_expired() {
            return Err(TokenValidationError::Expired);
        }
        Ok(())
    }
}

/// Token validation error
#[derive(Debug, Clone, thiserror::Error)]
pub enum TokenValidationError {
    #[error("Token has expired")]
    Expired,
    #[error("Invalid signature")]
    InvalidSignature,
    #[error("Invalid issuer")]
    InvalidIssuer,
    #[error("Invalid audience")]
    InvalidAudience,
    #[error("Missing required scope: {0}")]
    MissingScope(String),
    #[error("Malformed token")]
    Malformed,
}

// ============================================================================
// API Key
// ============================================================================

/// API key info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiKey {
    /// Key ID
    pub key_id: String,
    /// Key prefix (first 8 chars)
    pub prefix: String,
    /// Hashed key value
    pub hashed_value: String,
    /// Scopes
    pub scopes: Vec<String>,
    /// Created at
    pub created_at: String,
    /// Expires at
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<String>,
    /// Last used at
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_used_at: Option<String>,
    /// Owner
    pub owner: String,
    /// Active status
    pub active: bool,
}

impl ApiKey {
    /// Generate new API key
    pub fn generate(owner: impl Into<String>, scopes: Vec<String>) -> (Self, String) {
        let key_value = format!("wia_{}", uuid::Uuid::new_v4().to_string().replace("-", ""));
        let prefix = key_value[..12].to_string();

        let key = Self {
            key_id: uuid::Uuid::new_v4().to_string(),
            prefix,
            hashed_value: Self::hash_key(&key_value),
            scopes,
            created_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            expires_at: None,
            last_used_at: None,
            owner: owner.into(),
            active: true,
        };

        (key, key_value)
    }

    /// Hash API key (placeholder - use proper hashing in production)
    fn hash_key(key: &str) -> String {
        // In production, use bcrypt, argon2, or similar
        format!("hashed:{}", key)
    }

    /// Verify API key
    pub fn verify(&self, provided_key: &str) -> bool {
        self.active && self.hashed_value == Self::hash_key(provided_key)
    }
}

// ============================================================================
// Certificate Info
// ============================================================================

/// Certificate information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CertificateInfo {
    /// Subject CN
    pub subject: String,
    /// Issuer CN
    pub issuer: String,
    /// Serial number
    pub serial_number: String,
    /// Not before
    pub not_before: String,
    /// Not after
    pub not_after: String,
    /// SHA-256 fingerprint
    pub fingerprint_sha256: String,
    /// Subject Alternative Names
    #[serde(skip_serializing_if = "Option::is_none")]
    pub san: Option<Vec<String>>,
    /// Key usage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub key_usage: Option<Vec<String>>,
}

impl CertificateInfo {
    /// Check if certificate is expired
    pub fn is_expired(&self) -> bool {
        // Parse not_after and compare with now
        DateTime::parse_from_rfc3339(&self.not_after)
            .map(|dt| Utc::now() > dt)
            .unwrap_or(true)
    }

    /// Days until expiration
    pub fn days_until_expiry(&self) -> Option<i64> {
        DateTime::parse_from_rfc3339(&self.not_after)
            .ok()
            .map(|dt| (dt.signed_duration_since(Utc::now())).num_days())
    }
}

// ============================================================================
// Message Signing
// ============================================================================

/// Signature algorithm for message signing
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageSignatureAlgorithm {
    Ed25519,
    Dilithium3,
    EcdsaP256,
}

/// Message signature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageSignature {
    /// Algorithm used
    pub algorithm: MessageSignatureAlgorithm,
    /// Base64-encoded signature
    pub value: String,
    /// Key ID used for signing
    pub key_id: String,
    /// Signed at timestamp
    pub signed_at: String,
}

impl MessageSignature {
    /// Create a new signature
    pub fn new(
        algorithm: MessageSignatureAlgorithm,
        value: impl Into<String>,
        key_id: impl Into<String>,
    ) -> Self {
        Self {
            algorithm,
            value: value.into(),
            key_id: key_id.into(),
            signed_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
        }
    }
}

/// Canonicalization for signing
pub trait Canonicalize {
    /// Convert to canonical bytes for signing
    fn to_canonical_bytes(&self) -> Vec<u8>;
}

impl<T: Serialize> Canonicalize for T {
    fn to_canonical_bytes(&self) -> Vec<u8> {
        // Sort keys and format consistently
        serde_json::to_vec(self).unwrap_or_default()
    }
}

// ============================================================================
// Authentication Context
// ============================================================================

/// Authentication method
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthMethod {
    Jwt,
    ApiKey,
    Mtls,
    None,
}

/// Authenticated identity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthenticatedIdentity {
    /// Identity ID
    pub id: String,
    /// Authentication method used
    pub method: AuthMethod,
    /// Scopes/permissions
    pub scopes: Vec<String>,
    /// Component type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub component_type: Option<String>,
    /// Organization
    #[serde(skip_serializing_if = "Option::is_none")]
    pub organization: Option<String>,
    /// Authenticated at
    pub authenticated_at: String,
    /// Certificate info (if mTLS)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certificate: Option<CertificateInfo>,
}

impl AuthenticatedIdentity {
    /// Create from JWT claims
    pub fn from_jwt(claims: &JwtClaims) -> Self {
        Self {
            id: claims.sub.clone(),
            method: AuthMethod::Jwt,
            scopes: claims.scope.clone().unwrap_or_default(),
            component_type: claims.component.as_ref().map(|c| c.component_type.clone()),
            organization: claims.component.as_ref().and_then(|c| c.organization.clone()),
            authenticated_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            certificate: None,
        }
    }

    /// Create from API key
    pub fn from_api_key(api_key: &ApiKey) -> Self {
        Self {
            id: api_key.owner.clone(),
            method: AuthMethod::ApiKey,
            scopes: api_key.scopes.clone(),
            component_type: None,
            organization: None,
            authenticated_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            certificate: None,
        }
    }

    /// Create from certificate
    pub fn from_certificate(cert: CertificateInfo, scopes: Vec<String>) -> Self {
        Self {
            id: cert.subject.clone(),
            method: AuthMethod::Mtls,
            scopes,
            component_type: None,
            organization: None,
            authenticated_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            certificate: Some(cert),
        }
    }

    /// Check if identity has required scope
    pub fn has_scope(&self, required: &str) -> bool {
        self.scopes.iter().any(|s| s == required)
    }

    /// Check if identity has all required scopes
    pub fn has_all_scopes(&self, required: &[&str]) -> bool {
        required.iter().all(|r| self.has_scope(r))
    }
}

// ============================================================================
// Standard Scopes
// ============================================================================

/// Standard API scopes
pub mod scopes {
    // Read scopes
    pub const READ_ALERTS: &str = "read:alerts";
    pub const READ_EVENTS: &str = "read:events";
    pub const READ_THREATS: &str = "read:threats";
    pub const READ_POLICIES: &str = "read:policies";

    // Write scopes
    pub const WRITE_ALERTS: &str = "write:alerts";
    pub const WRITE_EVENTS: &str = "write:events";
    pub const WRITE_THREATS: &str = "write:threats";
    pub const WRITE_POLICIES: &str = "write:policies";

    // Admin scopes
    pub const ADMIN_POLICIES: &str = "admin:policies";
    pub const ADMIN_USERS: &str = "admin:users";
    pub const ADMIN_SYSTEM: &str = "admin:system";

    // Special scopes
    pub const ACCESS_DECISION: &str = "access:decision";
    pub const STREAM_EVENTS: &str = "stream:events";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jwt_claims() {
        let claims = JwtClaims::new(
            "https://auth.wia.live",
            "component-001",
            "wia-security-api",
            Duration::hours(8),
        )
        .with_scopes(vec!["read:alerts".to_string(), "write:events".to_string()]);

        assert!(!claims.is_expired());
        assert!(claims.has_scope("read:alerts"));
        assert!(!claims.has_scope("admin:system"));
    }

    #[test]
    fn test_api_key_generation() {
        let (key, secret) = ApiKey::generate("test-user", vec!["read:alerts".to_string()]);

        assert!(secret.starts_with("wia_"));
        assert!(key.verify(&secret));
        assert!(!key.verify("invalid_key"));
    }

    #[test]
    fn test_authenticated_identity() {
        let claims = JwtClaims::new(
            "issuer",
            "subject",
            "audience",
            Duration::hours(1),
        )
        .with_scopes(vec!["read:alerts".to_string()]);

        let identity = AuthenticatedIdentity::from_jwt(&claims);
        assert!(identity.has_scope("read:alerts"));
        assert!(!identity.has_scope("write:alerts"));
    }
}
