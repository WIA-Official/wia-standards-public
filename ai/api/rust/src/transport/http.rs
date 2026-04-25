//! HTTP Transport Implementation
//!
//! This module provides an HTTP transport for the WIA AI protocol,
//! with support for SSE streaming.
//!
//! Note: This module requires the `http` feature to be enabled.

use async_trait::async_trait;
use futures_util::StreamExt;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::RwLock;

use super::{StreamingTransport, Transport, TransportConfig};
use crate::protocol::{ProtocolError, ProtocolMessage, ProtocolResult, PROTOCOL_VERSION};
use crate::streaming::SseParser;

/// HTTP transport for the WIA AI protocol
pub struct HttpTransport {
    config: TransportConfig,
    client: reqwest::Client,
    connected: Arc<RwLock<bool>>,
}

impl HttpTransport {
    /// Create a new HTTP transport with the given configuration
    pub fn new(config: TransportConfig) -> ProtocolResult<Self> {
        let mut headers = reqwest::header::HeaderMap::new();

        // Add content type
        headers.insert(
            reqwest::header::CONTENT_TYPE,
            reqwest::header::HeaderValue::from_static("application/json"),
        );

        // Add protocol version header
        headers.insert(
            reqwest::header::HeaderName::from_static("x-wia-version"),
            reqwest::header::HeaderValue::from_str(PROTOCOL_VERSION)
                .map_err(|e| ProtocolError::validation_error(e.to_string()))?,
        );

        // Add auth token if present
        if let Some(ref token) = config.auth_token {
            headers.insert(
                reqwest::header::AUTHORIZATION,
                reqwest::header::HeaderValue::from_str(&format!("Bearer {}", token))
                    .map_err(|e| ProtocolError::validation_error(e.to_string()))?,
            );
        }

        // Add custom headers
        for (key, value) in &config.headers {
            headers.insert(
                reqwest::header::HeaderName::from_bytes(key.as_bytes())
                    .map_err(|e| ProtocolError::validation_error(e.to_string()))?,
                reqwest::header::HeaderValue::from_str(value)
                    .map_err(|e| ProtocolError::validation_error(e.to_string()))?,
            );
        }

        let client = reqwest::Client::builder()
            .default_headers(headers)
            .timeout(Duration::from_millis(config.timeout_ms))
            .build()
            .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

        Ok(Self {
            config,
            client,
            connected: Arc::new(RwLock::new(true)),
        })
    }

    /// Create an HTTP transport with default configuration
    pub fn with_base_url(base_url: impl Into<String>) -> ProtocolResult<Self> {
        Self::new(TransportConfig::new(base_url))
    }

    /// Get the base URL
    pub fn base_url(&self) -> &str {
        &self.config.base_url
    }

    /// Send a request to the given endpoint
    async fn send_request(&self, endpoint: &str, message: &ProtocolMessage) -> ProtocolResult<reqwest::Response> {
        let url = format!("{}{}", self.config.base_url, endpoint);

        let response = self
            .client
            .post(&url)
            .json(message)
            .send()
            .await
            .map_err(|e| {
                if e.is_timeout() {
                    ProtocolError::new(crate::protocol::ErrorCode::AgentTimeout, e.to_string())
                } else if e.is_connect() {
                    ProtocolError::connection_lost(e.to_string())
                } else {
                    ProtocolError::validation_error(e.to_string())
                }
            })?;

        // Handle HTTP errors
        let status = response.status();
        if !status.is_success() {
            let error_text = response.text().await.unwrap_or_default();

            return Err(match status.as_u16() {
                401 => ProtocolError::auth_failed("Authentication required"),
                403 => ProtocolError::permission_denied("Access denied"),
                404 => ProtocolError::agent_not_found("Resource not found"),
                429 => {
                    // Try to get retry-after header
                    ProtocolError::rate_limited(60000)
                }
                500..=599 => ProtocolError::new(
                    crate::protocol::ErrorCode::AgentError,
                    format!("Server error: {}", error_text),
                ),
                _ => ProtocolError::validation_error(format!(
                    "HTTP error {}: {}",
                    status, error_text
                )),
            });
        }

        Ok(response)
    }
}

#[async_trait]
impl Transport for HttpTransport {
    async fn send(&self, message: ProtocolMessage) -> ProtocolResult<ProtocolMessage> {
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Transport is closed"));
        }

        let response = self.send_request("/v1/messages", &message).await?;

        response.json().await.map_err(|e| {
            ProtocolError::validation_error(format!("Failed to parse response: {}", e))
        })
    }

    async fn send_no_response(&self, message: ProtocolMessage) -> ProtocolResult<()> {
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Transport is closed"));
        }

        self.send_request("/v1/messages", &message).await?;
        Ok(())
    }

    async fn is_connected(&self) -> bool {
        *self.connected.read().await
    }

    async fn close(&self) -> ProtocolResult<()> {
        *self.connected.write().await = false;
        Ok(())
    }
}

#[async_trait]
impl StreamingTransport for HttpTransport {
    async fn send_streaming(
        &self,
        message: ProtocolMessage,
        callback: Box<dyn Fn(ProtocolMessage) + Send + Sync>,
    ) -> ProtocolResult<()> {
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Transport is closed"));
        }

        let url = format!("{}/v1/messages", self.config.base_url);

        // Add stream parameter
        let mut message_with_stream = message;
        if let Some(ref mut payload) = message_with_stream.payload {
            if let Some(obj) = payload.as_object_mut() {
                obj.insert("stream".to_string(), serde_json::Value::Bool(true));
            }
        }

        let response = self
            .client
            .post(&url)
            .header(reqwest::header::ACCEPT, "text/event-stream")
            .json(&message_with_stream)
            .send()
            .await
            .map_err(|e| ProtocolError::connection_lost(e.to_string()))?;

        let status = response.status();
        if !status.is_success() {
            let error_text = response.text().await.unwrap_or_default();
            return Err(ProtocolError::validation_error(format!(
                "HTTP error {}: {}",
                status, error_text
            )));
        }

        // Process SSE stream
        let mut stream = response.bytes_stream();
        let mut parser = SseParser::new();

        while let Some(chunk) = stream.next().await {
            let bytes = chunk.map_err(|e| {
                ProtocolError::connection_lost(format!("Stream error: {}", e))
            })?;

            let text = String::from_utf8_lossy(&bytes);
            let events = parser.feed(&text);

            for event in events {
                // Skip ping events
                if event.is_ping() {
                    continue;
                }

                // Check for [DONE] marker
                if event.is_done() {
                    return Ok(());
                }

                // Parse the event data as a protocol message
                if let Some(ref data) = event.data {
                    if let Ok(msg) = serde_json::from_str::<ProtocolMessage>(data) {
                        callback(msg);
                    }
                }
            }
        }

        Ok(())
    }
}

/// Builder for creating HTTP transports
pub struct HttpTransportBuilder {
    config: TransportConfig,
}

impl HttpTransportBuilder {
    /// Create a new builder with the given base URL
    pub fn new(base_url: impl Into<String>) -> Self {
        Self {
            config: TransportConfig::new(base_url),
        }
    }

    /// Set the authentication token
    pub fn with_auth_token(mut self, token: impl Into<String>) -> Self {
        self.config.auth_token = Some(token.into());
        self
    }

    /// Set the request timeout
    pub fn with_timeout(mut self, ms: u64) -> Self {
        self.config.timeout_ms = ms;
        self
    }

    /// Set max retries
    pub fn with_max_retries(mut self, retries: u32) -> Self {
        self.config.max_retries = retries;
        self
    }

    /// Add a custom header
    pub fn with_header(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.config.headers.insert(key.into(), value.into());
        self
    }

    /// Build the HTTP transport
    pub fn build(self) -> ProtocolResult<HttpTransport> {
        HttpTransport::new(self.config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_builder() {
        let transport = HttpTransportBuilder::new("https://api.example.com")
            .with_auth_token("test-token")
            .with_timeout(60000)
            .with_header("X-Custom", "value")
            .build()
            .unwrap();

        assert_eq!(transport.base_url(), "https://api.example.com");
    }

    #[test]
    fn test_default_config() {
        let config = TransportConfig::default();
        assert_eq!(config.base_url, "https://api.wia.live");
        assert_eq!(config.timeout_ms, 30000);
        assert_eq!(config.max_retries, 3);
    }
}
