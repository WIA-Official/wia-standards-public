//! Communication protocol for the WIA Perception Clarity API (Phase 2).
//!
//! - [`rest`] — the reporting/query client trait + request/response envelopes.
//! - [`websocket`] — the real-time clarity event stream message types.

pub mod rest;
pub mod websocket;

pub use rest::*;
pub use websocket::*;

use serde::{Deserialize, Serialize};

/// Client configuration — base URL + bearer token (Phase 2 §4, §5).
#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// REST base URL, e.g. `https://perception-clarity.wiastandards.com/api/v1`.
    pub base_url: String,
    /// WebSocket base URL, e.g. `wss://perception-clarity.wiastandards.com/api/v1`.
    pub ws_url: String,
    /// Bearer token (Phase 2 §5).
    pub token: Option<String>,
    /// Request timeout in seconds.
    pub timeout_secs: u64,
}

impl Default for ClientConfig {
    fn default() -> Self {
        Self {
            base_url: "https://perception-clarity.wiastandards.com/api/v1".to_string(),
            ws_url: "wss://perception-clarity.wiastandards.com/api/v1".to_string(),
            token: None,
            timeout_secs: 30,
        }
    }
}

impl ClientConfig {
    /// Create a configuration with a base URL and bearer token.
    pub fn new(base_url: impl Into<String>, token: impl Into<String>) -> Self {
        let base_url = base_url.into();
        let ws_url = base_url.replacen("https://", "wss://", 1).replacen("http://", "ws://", 1);
        Self {
            base_url,
            ws_url,
            token: Some(token.into()),
            timeout_secs: 30,
        }
    }

    /// Set the bearer token.
    pub fn with_token(mut self, token: impl Into<String>) -> Self {
        self.token = Some(token.into());
        self
    }
}

/// The standard error envelope returned by the API (Phase 2 §10.1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiError {
    pub error: ApiErrorDetails,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<ApiErrorMeta>,
}

/// Body of an [`ApiError`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorDetails {
    /// Machine-readable code, e.g. `"validation_failed"` (Phase 2 §10.2).
    pub code: String,
    /// Human-readable message.
    pub message: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub errors: Vec<ApiErrorField>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc_url: Option<String>,
}

/// A single field-level validation issue.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorField {
    pub path: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub actual: Option<String>,
}

/// Error-envelope metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorMeta {
    #[serde(rename = "requestId")]
    pub request_id: String,
    #[serde(rename = "generatedAt")]
    pub generated_at: String,
}

impl std::fmt::Display for ApiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.error.code, self.error.message)
    }
}

impl std::error::Error for ApiError {}
