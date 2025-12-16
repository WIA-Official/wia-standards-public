//! Authentication utilities for WIA Auto

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::{Error, Result};

/// JWT token claims
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TokenClaims {
    /// Subject (user/vehicle ID)
    pub sub: String,

    /// Issuer
    pub iss: String,

    /// Audience
    pub aud: String,

    /// Expiration time (Unix timestamp)
    pub exp: i64,

    /// Issued at (Unix timestamp)
    pub iat: i64,

    /// Not before (Unix timestamp)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nbf: Option<i64>,

    /// JWT ID
    pub jti: String,

    /// Client type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_type: Option<ClientType>,

    /// Scopes/permissions
    #[serde(default)]
    pub scopes: Vec<String>,
}

impl TokenClaims {
    /// Create new token claims
    pub fn new(subject: impl Into<String>, issuer: impl Into<String>, audience: impl Into<String>) -> Self {
        let now = Utc::now();
        Self {
            sub: subject.into(),
            iss: issuer.into(),
            aud: audience.into(),
            exp: (now + Duration::minutes(15)).timestamp(),
            iat: now.timestamp(),
            nbf: Some(now.timestamp()),
            jti: Uuid::new_v4().to_string(),
            client_type: None,
            scopes: vec![],
        }
    }

    /// Set expiration duration
    pub fn expires_in(mut self, duration: Duration) -> Self {
        let now = Utc::now();
        self.exp = (now + duration).timestamp();
        self
    }

    /// Set client type
    pub fn with_client_type(mut self, client_type: ClientType) -> Self {
        self.client_type = Some(client_type);
        self
    }

    /// Add scope
    pub fn with_scope(mut self, scope: impl Into<String>) -> Self {
        self.scopes.push(scope.into());
        self
    }

    /// Add multiple scopes
    pub fn with_scopes(mut self, scopes: impl IntoIterator<Item = impl Into<String>>) -> Self {
        for scope in scopes {
            self.scopes.push(scope.into());
        }
        self
    }

    /// Check if token is expired
    pub fn is_expired(&self) -> bool {
        Utc::now().timestamp() >= self.exp
    }

    /// Check if token has scope
    pub fn has_scope(&self, scope: &str) -> bool {
        self.scopes.iter().any(|s| s == scope)
    }

    /// Get expiration as DateTime
    pub fn expires_at(&self) -> DateTime<Utc> {
        DateTime::from_timestamp(self.exp, 0).unwrap_or_else(Utc::now)
    }
}

/// Client types for authentication
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClientType {
    /// Passenger mobile app
    PassengerApp,
    /// Vehicle system
    Vehicle,
    /// Fleet management server
    FleetServer,
    /// Support center
    Support,
}

/// API key credentials
#[derive(Debug, Clone)]
pub struct ApiKeyCredentials {
    /// API key ID
    pub key_id: String,
    /// API secret
    pub secret: String,
}

impl ApiKeyCredentials {
    /// Create new API key credentials
    pub fn new(key_id: impl Into<String>, secret: impl Into<String>) -> Self {
        Self {
            key_id: key_id.into(),
            secret: secret.into(),
        }
    }

    /// Generate signature for a request
    pub fn sign(&self, timestamp: i64, method: &str, path: &str, body_hash: &str) -> String {
        use super::crypto::hmac_sha256;

        let message = format!("{}.{}.{}.{}", timestamp, method, path, body_hash);
        hmac_sha256(self.secret.as_bytes(), message.as_bytes())
    }

    /// Create authorization headers
    pub fn auth_headers(&self, method: &str, path: &str, body_hash: &str) -> Vec<(String, String)> {
        let timestamp = Utc::now().timestamp();
        let signature = self.sign(timestamp, method, path, body_hash);

        vec![
            ("X-API-Key".to_string(), self.key_id.clone()),
            ("X-API-Signature".to_string(), signature),
            ("X-Timestamp".to_string(), timestamp.to_string()),
        ]
    }
}

/// OAuth token response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TokenResponse {
    /// Access token
    pub access_token: String,

    /// Token type (always "Bearer")
    pub token_type: String,

    /// Expiration in seconds
    pub expires_in: i64,

    /// Refresh token (optional)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refresh_token: Option<String>,

    /// Granted scopes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scope: Option<String>,
}

/// Token storage trait
pub trait TokenStorage: Send + Sync {
    /// Store access token
    fn store_access_token(&self, token: &str) -> Result<()>;

    /// Get access token
    fn get_access_token(&self) -> Result<Option<String>>;

    /// Store refresh token
    fn store_refresh_token(&self, token: &str) -> Result<()>;

    /// Get refresh token
    fn get_refresh_token(&self) -> Result<Option<String>>;

    /// Clear all tokens
    fn clear(&self) -> Result<()>;
}

/// In-memory token storage for testing
#[derive(Debug, Default)]
pub struct MemoryTokenStorage {
    access_token: std::sync::RwLock<Option<String>>,
    refresh_token: std::sync::RwLock<Option<String>>,
}

impl MemoryTokenStorage {
    pub fn new() -> Self {
        Self::default()
    }
}

impl TokenStorage for MemoryTokenStorage {
    fn store_access_token(&self, token: &str) -> Result<()> {
        *self.access_token.write().unwrap() = Some(token.to_string());
        Ok(())
    }

    fn get_access_token(&self) -> Result<Option<String>> {
        Ok(self.access_token.read().unwrap().clone())
    }

    fn store_refresh_token(&self, token: &str) -> Result<()> {
        *self.refresh_token.write().unwrap() = Some(token.to_string());
        Ok(())
    }

    fn get_refresh_token(&self) -> Result<Option<String>> {
        Ok(self.refresh_token.read().unwrap().clone())
    }

    fn clear(&self) -> Result<()> {
        *self.access_token.write().unwrap() = None;
        *self.refresh_token.write().unwrap() = None;
        Ok(())
    }
}

/// Standard OAuth scopes for WIA Auto
pub mod scopes {
    /// Read passenger profiles
    pub const PROFILE_READ: &str = "profile:read";
    /// Write passenger profiles
    pub const PROFILE_WRITE: &str = "profile:write";
    /// Request trips
    pub const TRIP_REQUEST: &str = "trip:request";
    /// Read trip status
    pub const TRIP_READ: &str = "trip:read";
    /// Cancel trips
    pub const TRIP_CANCEL: &str = "trip:cancel";
    /// View vehicles
    pub const VEHICLE_READ: &str = "vehicle:read";
    /// Control vehicle HMI
    pub const VEHICLE_HMI: &str = "vehicle:hmi";
    /// Report emergencies
    pub const EMERGENCY_REPORT: &str = "emergency:report";
    /// Handle emergencies (support)
    pub const EMERGENCY_HANDLE: &str = "emergency:handle";
    /// Fleet management
    pub const FLEET_MANAGE: &str = "fleet:manage";
}

/// Check if required scopes are present
pub fn check_scopes(required: &[&str], granted: &[String]) -> Result<()> {
    for scope in required {
        if !granted.iter().any(|s| s == *scope) {
            return Err(Error::auth(format!("Missing required scope: {}", scope)));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_token_claims() {
        let claims = TokenClaims::new("user123", "wia-auto", "api")
            .with_client_type(ClientType::PassengerApp)
            .with_scopes(["profile:read", "trip:request"]);

        assert_eq!(claims.sub, "user123");
        assert!(!claims.is_expired());
        assert!(claims.has_scope("profile:read"));
        assert!(!claims.has_scope("admin"));
    }

    #[test]
    fn test_api_key_signing() {
        let creds = ApiKeyCredentials::new("key123", "secret456");
        let sig1 = creds.sign(1234567890, "POST", "/trips", "abc");
        let sig2 = creds.sign(1234567890, "POST", "/trips", "abc");
        assert_eq!(sig1, sig2);

        // Different timestamp = different signature
        let sig3 = creds.sign(1234567891, "POST", "/trips", "abc");
        assert_ne!(sig1, sig3);
    }

    #[test]
    fn test_scope_checking() {
        let granted = vec![
            "profile:read".to_string(),
            "trip:request".to_string(),
        ];

        assert!(check_scopes(&["profile:read"], &granted).is_ok());
        assert!(check_scopes(&["admin:all"], &granted).is_err());
    }

    #[test]
    fn test_memory_token_storage() {
        let storage = MemoryTokenStorage::new();

        storage.store_access_token("access123").unwrap();
        storage.store_refresh_token("refresh456").unwrap();

        assert_eq!(storage.get_access_token().unwrap(), Some("access123".to_string()));
        assert_eq!(storage.get_refresh_token().unwrap(), Some("refresh456".to_string()));

        storage.clear().unwrap();
        assert_eq!(storage.get_access_token().unwrap(), None);
    }
}
