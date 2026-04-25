//! WIA AI Transport Module
//!
//! This module provides transport layer implementations for the WIA AI protocol,
//! including HTTP and mock transports for testing.
//!
//! ## Features
//!
//! - HTTP transport with SSE streaming support (requires `http` feature)
//! - Mock transport for testing
//! - Transport trait for custom implementations
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_ai::transport::*;
//! use wia_ai::protocol::*;
//!
//! async fn example() {
//!     // Create a mock transport for testing
//!     let transport = MockTransport::new();
//!
//!     // Queue a response
//!     transport.queue_response(ProtocolMessage::pong("ping-001")).await;
//!
//!     // Send a message
//!     let ping = ProtocolMessage::ping();
//!     let response = transport.send(ping).await.unwrap();
//! }
//! ```

mod mock;

#[cfg(feature = "http")]
mod http;

pub use mock::*;

#[cfg(feature = "http")]
pub use http::*;

use async_trait::async_trait;
use crate::protocol::{ProtocolError, ProtocolMessage, ProtocolResult};

/// Trait for protocol transports
#[async_trait]
pub trait Transport: Send + Sync {
    /// Send a message and receive a response
    async fn send(&self, message: ProtocolMessage) -> ProtocolResult<ProtocolMessage>;

    /// Send a message without waiting for a response
    async fn send_no_response(&self, message: ProtocolMessage) -> ProtocolResult<()>;

    /// Check if the transport is connected
    async fn is_connected(&self) -> bool;

    /// Close the transport
    async fn close(&self) -> ProtocolResult<()>;
}

/// Trait for streaming transports
#[async_trait]
pub trait StreamingTransport: Transport {
    /// Send a message and receive a stream of responses
    async fn send_streaming(
        &self,
        message: ProtocolMessage,
        callback: Box<dyn Fn(ProtocolMessage) + Send + Sync>,
    ) -> ProtocolResult<()>;
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Base URL for HTTP transport
    pub base_url: String,

    /// Request timeout in milliseconds
    pub timeout_ms: u64,

    /// Maximum retry attempts
    pub max_retries: u32,

    /// Authentication token
    pub auth_token: Option<String>,

    /// Additional headers
    pub headers: std::collections::HashMap<String, String>,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            base_url: "https://api.wia.live".to_string(),
            timeout_ms: 30000,
            max_retries: 3,
            auth_token: None,
            headers: std::collections::HashMap::new(),
        }
    }
}

impl TransportConfig {
    /// Create a new transport configuration
    pub fn new(base_url: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            ..Default::default()
        }
    }

    /// Set the authentication token
    pub fn with_auth_token(mut self, token: impl Into<String>) -> Self {
        self.auth_token = Some(token.into());
        self
    }

    /// Set the request timeout
    pub fn with_timeout(mut self, ms: u64) -> Self {
        self.timeout_ms = ms;
        self
    }

    /// Set max retries
    pub fn with_max_retries(mut self, retries: u32) -> Self {
        self.max_retries = retries;
        self
    }

    /// Add a header
    pub fn with_header(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.headers.insert(key.into(), value.into());
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_transport() {
        let transport = MockTransport::new();

        // Queue a response
        transport
            .queue_response(ProtocolMessage::pong("ping-001"))
            .await;

        // Send a message
        let ping = ProtocolMessage::ping();
        let response = transport.send(ping).await.unwrap();

        assert_eq!(response.message_type, crate::protocol::MessageType::Pong);
    }

    #[test]
    fn test_transport_config() {
        let config = TransportConfig::new("https://api.example.com")
            .with_auth_token("secret-token")
            .with_timeout(60000)
            .with_header("X-Custom", "value");

        assert_eq!(config.base_url, "https://api.example.com");
        assert_eq!(config.auth_token, Some("secret-token".to_string()));
        assert_eq!(config.timeout_ms, 60000);
        assert_eq!(config.headers.get("X-Custom"), Some(&"value".to_string()));
    }
}
