//! Adapter configuration

use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Adapter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdapterConfig {
    /// Base URL for the service
    pub base_url: String,
    /// API key (if applicable)
    pub api_key: Option<String>,
    /// Authentication type
    pub auth_type: AuthType,
    /// Request timeout in milliseconds
    pub timeout_ms: u64,
    /// Number of retries
    pub retry_count: u32,
    /// Cache TTL in seconds
    pub cache_ttl_sec: u64,
    /// Additional headers
    #[serde(default)]
    pub headers: std::collections::HashMap<String, String>,
}

impl Default for AdapterConfig {
    fn default() -> Self {
        Self {
            base_url: String::new(),
            api_key: None,
            auth_type: AuthType::None,
            timeout_ms: 30000,
            retry_count: 3,
            cache_ttl_sec: 300,
            headers: std::collections::HashMap::new(),
        }
    }
}

impl AdapterConfig {
    /// Create config with base URL
    pub fn new(base_url: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            ..Default::default()
        }
    }

    /// Set API key
    pub fn with_api_key(mut self, api_key: impl Into<String>) -> Self {
        self.api_key = Some(api_key.into());
        self.auth_type = AuthType::ApiKey;
        self
    }

    /// Set OAuth2 authentication
    pub fn with_oauth2(mut self, client_id: impl Into<String>, client_secret: impl Into<String>) -> Self {
        self.auth_type = AuthType::OAuth2 {
            client_id: client_id.into(),
            client_secret: client_secret.into(),
            token_url: None,
        };
        self
    }

    /// Set bearer token
    pub fn with_bearer_token(mut self, token: impl Into<String>) -> Self {
        self.auth_type = AuthType::Bearer {
            token: token.into(),
        };
        self
    }

    /// Set timeout
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Set retry count
    pub fn with_retries(mut self, count: u32) -> Self {
        self.retry_count = count;
        self
    }

    /// Set cache TTL
    pub fn with_cache_ttl(mut self, ttl_sec: u64) -> Self {
        self.cache_ttl_sec = ttl_sec;
        self
    }

    /// Add header
    pub fn with_header(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.headers.insert(key.into(), value.into());
        self
    }

    /// Get timeout as Duration
    pub fn timeout(&self) -> Duration {
        Duration::from_millis(self.timeout_ms)
    }

    /// Get cache TTL as Duration
    pub fn cache_ttl(&self) -> Duration {
        Duration::from_secs(self.cache_ttl_sec)
    }
}

/// Authentication types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum AuthType {
    /// No authentication
    None,
    /// API key authentication
    ApiKey,
    /// OAuth2 authentication
    OAuth2 {
        client_id: String,
        client_secret: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        token_url: Option<String>,
    },
    /// Bearer token authentication
    Bearer { token: String },
    /// Basic authentication
    Basic { username: String, password: String },
}

impl Default for AuthType {
    fn default() -> Self {
        Self::None
    }
}

/// Retry policy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetryPolicy {
    /// Maximum number of retries
    pub max_retries: u32,
    /// Initial delay in milliseconds
    pub initial_delay_ms: u64,
    /// Maximum delay in milliseconds
    pub max_delay_ms: u64,
    /// Exponential backoff base
    pub exponential_base: f64,
    /// Jitter factor (0.0 to 1.0)
    pub jitter: f64,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_retries: 3,
            initial_delay_ms: 1000,
            max_delay_ms: 30000,
            exponential_base: 2.0,
            jitter: 0.1,
        }
    }
}

impl RetryPolicy {
    /// Calculate delay for a given attempt
    pub fn delay_for_attempt(&self, attempt: u32) -> Duration {
        let base_delay = self.initial_delay_ms as f64 * self.exponential_base.powi(attempt as i32);
        let clamped = base_delay.min(self.max_delay_ms as f64);

        // Add jitter
        let jitter_range = clamped * self.jitter;
        let jitter = rand::random::<f64>() * jitter_range * 2.0 - jitter_range;
        let final_delay = (clamped + jitter).max(0.0);

        Duration::from_millis(final_delay as u64)
    }

    /// Check if should retry
    pub fn should_retry(&self, attempt: u32) -> bool {
        attempt < self.max_retries
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_builder() {
        let config = AdapterConfig::new("https://api.example.com")
            .with_api_key("my-api-key")
            .with_timeout(5000)
            .with_retries(5)
            .with_cache_ttl(600)
            .with_header("X-Custom", "value");

        assert_eq!(config.base_url, "https://api.example.com");
        assert_eq!(config.api_key, Some("my-api-key".to_string()));
        assert_eq!(config.timeout_ms, 5000);
        assert_eq!(config.retry_count, 5);
        assert_eq!(config.cache_ttl_sec, 600);
        assert!(matches!(config.auth_type, AuthType::ApiKey));
        assert!(config.headers.contains_key("X-Custom"));
    }

    #[test]
    fn test_oauth2_config() {
        let config = AdapterConfig::new("https://api.example.com")
            .with_oauth2("client-id", "client-secret");

        if let AuthType::OAuth2 { client_id, client_secret, .. } = config.auth_type {
            assert_eq!(client_id, "client-id");
            assert_eq!(client_secret, "client-secret");
        } else {
            panic!("Expected OAuth2 auth type");
        }
    }

    #[test]
    fn test_retry_policy() {
        let policy = RetryPolicy::default();

        assert!(policy.should_retry(0));
        assert!(policy.should_retry(2));
        assert!(!policy.should_retry(3));

        let delay = policy.delay_for_attempt(0);
        assert!(delay.as_millis() >= 900 && delay.as_millis() <= 1100);
    }
}
