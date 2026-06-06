//! Security for the WIA Perception Clarity API — Phase 2 §5, Phase 1 §8.
//!
//! - Bearer-token auth with scope checks (`clarity:report`, `clarity:read`).
//! - An optional message-signing stub: clarity reports expose an agent's safety
//!   state, so the standard recommends signing the payload (Phase 1 §8). The
//!   stub here defines the surface; a real deployment plugs in HMAC-SHA256 or an
//!   Ed25519 signature.

use serde::{Deserialize, Serialize};

use crate::error::{Error, Result};

/// Auth scopes — Phase 2 §5.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Scope {
    /// `clarity:report` — may `POST /reports` (agent side).
    #[serde(rename = "clarity:report")]
    Report,
    /// `clarity:read` — may query + subscribe (control side).
    #[serde(rename = "clarity:read")]
    Read,
}

/// A bearer token plus its granted scopes — Phase 2 §5.
#[derive(Debug, Clone)]
pub struct BearerAuth {
    token: String,
    scopes: Vec<Scope>,
}

impl BearerAuth {
    /// Create a bearer credential with explicit scopes.
    pub fn new(token: impl Into<String>, scopes: Vec<Scope>) -> Self {
        Self {
            token: token.into(),
            scopes,
        }
    }

    /// The raw token value.
    pub fn token(&self) -> &str {
        &self.token
    }

    /// The `Authorization` header value, e.g. `Bearer <token>`.
    pub fn header_value(&self) -> String {
        format!("Bearer {}", self.token)
    }

    /// Whether this credential holds a scope.
    pub fn has_scope(&self, scope: Scope) -> bool {
        self.scopes.contains(&scope)
    }

    /// Require a scope or fail with [`Error::InsufficientScope`] — Phase 2 §10.2.
    pub fn require_scope(&self, scope: Scope) -> Result<()> {
        if self.has_scope(scope) {
            Ok(())
        } else {
            Err(Error::InsufficientScope(format!("{:?} required", scope)))
        }
    }
}

/// Parse a bearer token out of an `Authorization` header value.
///
/// Returns [`Error::Auth`] when the header is missing the `Bearer ` prefix.
pub fn parse_bearer(header: &str) -> Result<String> {
    header
        .strip_prefix("Bearer ")
        .map(|t| t.trim().to_string())
        .filter(|t| !t.is_empty())
        .ok_or_else(|| Error::auth("missing or malformed Bearer token"))
}

/// A pluggable message signer for clarity payloads — Phase 1 §8 (recommended).
///
/// This is a stub trait so transports can sign/verify reports without the SDK
/// pulling in a specific crypto crate. A default no-op pass-through is provided
/// for environments where transport-level TLS is considered sufficient.
pub trait MessageSigner {
    /// Produce a detached signature over the serialized payload bytes.
    fn sign(&self, payload: &[u8]) -> Result<String>;

    /// Verify a detached signature over the serialized payload bytes.
    fn verify(&self, payload: &[u8], signature: &str) -> Result<()>;
}

/// A no-op signer: signing yields an empty signature and verification accepts
/// only that empty signature. Intended as an explicit "unsigned" marker, not for
/// production tamper protection.
#[derive(Debug, Clone, Default)]
pub struct NoopSigner;

impl MessageSigner for NoopSigner {
    fn sign(&self, _payload: &[u8]) -> Result<String> {
        Ok(String::new())
    }

    fn verify(&self, _payload: &[u8], signature: &str) -> Result<()> {
        if signature.is_empty() {
            Ok(())
        } else {
            Err(Error::signature("NoopSigner expects an empty signature"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_bearer_and_checks_scope() {
        let token = parse_bearer("Bearer abc.def.ghi").unwrap();
        assert_eq!(token, "abc.def.ghi");

        let auth = BearerAuth::new(token, vec![Scope::Read]);
        assert!(auth.require_scope(Scope::Read).is_ok());
        assert!(auth.require_scope(Scope::Report).is_err());
        assert_eq!(auth.header_value(), "Bearer abc.def.ghi");
    }

    #[test]
    fn rejects_missing_prefix() {
        assert!(parse_bearer("Token xyz").is_err());
    }
}
