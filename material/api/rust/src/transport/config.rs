//! Transport configuration
//!
//! Configuration options for HTTP and WebSocket transports.

use std::time::Duration;

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Base URL for the API
    pub base_url: String,

    /// API key for authentication
    pub api_key: Option<String>,

    /// JWT token for authentication
    pub jwt_token: Option<String>,

    /// Connection timeout
    pub connect_timeout: Duration,

    /// Request timeout
    pub request_timeout: Duration,

    /// Maximum retry attempts
    pub max_retries: u32,

    /// Retry delay base (for exponential backoff)
    pub retry_delay: Duration,

    /// Enable TLS certificate verification
    pub verify_tls: bool,

    /// User agent string
    pub user_agent: String,
}

impl TransportConfig {
    /// Create a new configuration with the given base URL
    pub fn new(base_url: &str) -> Self {
        Self {
            base_url: base_url.trim_end_matches('/').to_string(),
            api_key: None,
            jwt_token: None,
            connect_timeout: Duration::from_secs(30),
            request_timeout: Duration::from_secs(60),
            max_retries: 3,
            retry_delay: Duration::from_secs(1),
            verify_tls: true,
            user_agent: format!("wia-material-rust/{}", crate::VERSION),
        }
    }

    /// Set the API key
    pub fn with_api_key(mut self, api_key: &str) -> Self {
        self.api_key = Some(api_key.to_string());
        self
    }

    /// Set the JWT token
    pub fn with_jwt_token(mut self, token: &str) -> Self {
        self.jwt_token = Some(token.to_string());
        self
    }

    /// Set connection timeout
    pub fn with_connect_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = timeout;
        self
    }

    /// Set request timeout
    pub fn with_request_timeout(mut self, timeout: Duration) -> Self {
        self.request_timeout = timeout;
        self
    }

    /// Set maximum retries
    pub fn with_max_retries(mut self, retries: u32) -> Self {
        self.max_retries = retries;
        self
    }

    /// Disable TLS verification (not recommended for production)
    pub fn with_insecure_tls(mut self) -> Self {
        self.verify_tls = false;
        self
    }

    /// Get the materials endpoint URL
    pub fn materials_url(&self) -> String {
        format!("{}/api/v1/materials", self.base_url)
    }

    /// Get the stream endpoint URL
    pub fn stream_url(&self) -> String {
        let ws_url = self
            .base_url
            .replace("https://", "wss://")
            .replace("http://", "ws://");
        format!("{}/api/v1/stream", ws_url)
    }

    /// Get authorization header value
    pub fn auth_header(&self) -> Option<String> {
        if let Some(ref token) = self.jwt_token {
            Some(format!("Bearer {}", token))
        } else {
            None
        }
    }
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self::new("https://localhost:8080")
    }
}

/// WebSocket-specific configuration
#[derive(Debug, Clone)]
pub struct WebSocketConfig {
    /// Base transport config
    pub transport: TransportConfig,

    /// Ping interval
    pub ping_interval: Duration,

    /// Pong timeout
    pub pong_timeout: Duration,

    /// Auto-reconnect on disconnect
    pub auto_reconnect: bool,

    /// Maximum reconnect attempts
    pub max_reconnect_attempts: u32,

    /// Reconnect delay (exponential backoff base)
    pub reconnect_delay: Duration,

    /// Maximum reconnect delay
    pub max_reconnect_delay: Duration,
}

impl WebSocketConfig {
    /// Create a new WebSocket configuration
    pub fn new(base_url: &str) -> Self {
        Self {
            transport: TransportConfig::new(base_url),
            ping_interval: Duration::from_secs(30),
            pong_timeout: Duration::from_secs(5),
            auto_reconnect: true,
            max_reconnect_attempts: 10,
            reconnect_delay: Duration::from_secs(1),
            max_reconnect_delay: Duration::from_secs(60),
        }
    }

    /// Set the API key
    pub fn with_api_key(mut self, api_key: &str) -> Self {
        self.transport = self.transport.with_api_key(api_key);
        self
    }

    /// Set the JWT token
    pub fn with_jwt_token(mut self, token: &str) -> Self {
        self.transport = self.transport.with_jwt_token(token);
        self
    }

    /// Disable auto-reconnect
    pub fn without_auto_reconnect(mut self) -> Self {
        self.auto_reconnect = false;
        self
    }

    /// Set ping interval
    pub fn with_ping_interval(mut self, interval: Duration) -> Self {
        self.ping_interval = interval;
        self
    }
}

impl From<TransportConfig> for WebSocketConfig {
    fn from(config: TransportConfig) -> Self {
        Self {
            transport: config,
            ping_interval: Duration::from_secs(30),
            pong_timeout: Duration::from_secs(5),
            auto_reconnect: true,
            max_reconnect_attempts: 10,
            reconnect_delay: Duration::from_secs(1),
            max_reconnect_delay: Duration::from_secs(60),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_creation() {
        let config = TransportConfig::new("https://api.example.com");
        assert_eq!(config.base_url, "https://api.example.com");
        assert!(config.api_key.is_none());
    }

    #[test]
    fn test_config_with_api_key() {
        let config = TransportConfig::new("https://api.example.com").with_api_key("test-key");
        assert_eq!(config.api_key, Some("test-key".to_string()));
    }

    #[test]
    fn test_materials_url() {
        let config = TransportConfig::new("https://api.example.com/");
        assert_eq!(
            config.materials_url(),
            "https://api.example.com/api/v1/materials"
        );
    }

    #[test]
    fn test_stream_url() {
        let config = TransportConfig::new("https://api.example.com");
        assert_eq!(
            config.stream_url(),
            "wss://api.example.com/api/v1/stream"
        );
    }

    #[test]
    fn test_auth_header() {
        let config = TransportConfig::new("https://api.example.com").with_jwt_token("my-jwt");
        assert_eq!(config.auth_header(), Some("Bearer my-jwt".to_string()));
    }

    #[test]
    fn test_websocket_config() {
        let config = WebSocketConfig::new("https://api.example.com")
            .with_api_key("test")
            .without_auto_reconnect();
        assert!(!config.auto_reconnect);
        assert_eq!(config.transport.api_key, Some("test".to_string()));
    }
}
