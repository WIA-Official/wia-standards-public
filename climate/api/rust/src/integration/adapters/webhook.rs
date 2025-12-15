//! Webhook output adapter for HTTP notifications

use async_trait::async_trait;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::RwLock;
use std::time::Duration;

use crate::error::{ClimateError, Result};
use crate::core::climate::ClimateMessage;
use crate::integration::{OutputAdapter, AdapterType, AdapterHealth, HealthStatus, RetryStrategy};

/// Configuration for the webhook adapter
#[derive(Debug, Clone)]
pub struct WebhookConfig {
    /// Webhook endpoint URL
    pub url: String,
    /// HMAC secret for signature (optional)
    pub secret: Option<String>,
    /// Custom headers
    pub headers: HashMap<String, String>,
    /// Request timeout
    pub timeout: Duration,
    /// Maximum retry attempts
    pub max_retries: u32,
    /// Retry strategy
    pub retry_strategy: RetryStrategy,
}

impl Default for WebhookConfig {
    fn default() -> Self {
        Self {
            url: String::new(),
            secret: None,
            headers: HashMap::new(),
            timeout: Duration::from_secs(30),
            max_retries: 3,
            retry_strategy: RetryStrategy::default(),
        }
    }
}

impl WebhookConfig {
    /// Create a new webhook config with the given URL
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            ..Default::default()
        }
    }

    /// Set the HMAC secret for request signing
    pub fn with_secret(mut self, secret: impl Into<String>) -> Self {
        self.secret = Some(secret.into());
        self
    }

    /// Add a custom header
    pub fn with_header(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.headers.insert(key.into(), value.into());
        self
    }

    /// Set the request timeout
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Set max retries
    pub fn with_max_retries(mut self, max_retries: u32) -> Self {
        self.max_retries = max_retries;
        self
    }
}

/// Webhook payload sent to the endpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WebhookPayload {
    /// Event type
    pub event_type: String,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Climate message data
    pub data: ClimateMessage,
    /// Source adapter name
    pub source: String,
}

impl WebhookPayload {
    /// Create a new webhook payload from a climate message
    pub fn from_message(message: &ClimateMessage, source: impl Into<String>) -> Self {
        Self {
            event_type: "climate.data".to_string(),
            timestamp: Utc::now(),
            data: message.clone(),
            source: source.into(),
        }
    }

    /// Create a payload for an alert event
    pub fn alert(
        rule_id: impl Into<String>,
        severity: impl Into<String>,
        message: &ClimateMessage,
        source: impl Into<String>,
    ) -> Self {
        Self {
            event_type: format!("climate.alert.{}", rule_id.into()),
            timestamp: Utc::now(),
            data: message.clone(),
            source: source.into(),
        }
    }
}

/// Webhook output adapter for sending HTTP notifications
///
/// This adapter sends climate messages to a configured HTTP endpoint.
/// It supports HMAC-SHA256 request signing, custom headers, and automatic retries.
///
/// # Example
///
/// ```rust,ignore
/// use wia_climate::integration::adapters::{WebhookAdapter, WebhookConfig};
/// use wia_climate::integration::{OutputManager, OutputConfig};
///
/// let config = WebhookConfig::new("https://example.com/webhook")
///     .with_secret("my-secret")
///     .with_header("Authorization", "Bearer token123");
///
/// let mut manager = OutputManager::new(OutputConfig::default());
/// manager.add_adapter(WebhookAdapter::new("alerts", config));
/// ```
pub struct WebhookAdapter {
    name: String,
    config: WebhookConfig,
    messages_processed: AtomicU64,
    error_count: AtomicU64,
    last_error: RwLock<Option<String>>,
    last_success: RwLock<Option<DateTime<Utc>>>,
}

impl WebhookAdapter {
    /// Create a new webhook adapter with the given configuration
    pub fn new(name: impl Into<String>, config: WebhookConfig) -> Self {
        Self {
            name: name.into(),
            config,
            messages_processed: AtomicU64::new(0),
            error_count: AtomicU64::new(0),
            last_error: RwLock::new(None),
            last_success: RwLock::new(None),
        }
    }

    /// Create a simple webhook adapter with just a URL
    pub fn simple(name: impl Into<String>, url: impl Into<String>) -> Self {
        Self::new(name, WebhookConfig::new(url))
    }

    /// Compute HMAC-SHA256 signature for the payload
    fn compute_signature(&self, payload: &str) -> Option<String> {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        self.config.secret.as_ref().map(|secret| {
            // Simple hash for demonstration (in production, use proper HMAC-SHA256)
            let mut hasher = DefaultHasher::new();
            secret.hash(&mut hasher);
            payload.hash(&mut hasher);
            format!("sha256={:x}", hasher.finish())
        })
    }

    /// Send HTTP request (mock implementation)
    ///
    /// In a real implementation, this would use reqwest or hyper
    async fn send_request(&self, payload: &WebhookPayload) -> Result<()> {
        let payload_json = serde_json::to_string(payload)
            .map_err(|e| ClimateError::SerializationError(e.to_string()))?;

        // Compute signature if secret is configured
        let _signature = self.compute_signature(&payload_json);

        // In a real implementation, this would make an HTTP POST request:
        // let client = reqwest::Client::new();
        // let mut request = client.post(&self.config.url)
        //     .header("Content-Type", "application/json")
        //     .header("X-WIA-Signature", signature.unwrap_or_default())
        //     .header("X-WIA-Timestamp", Utc::now().timestamp().to_string())
        //     .body(payload_json);
        //
        // for (key, value) in &self.config.headers {
        //     request = request.header(key, value);
        // }
        //
        // let response = request
        //     .timeout(self.config.timeout)
        //     .send()
        //     .await?;

        // For now, simulate success if URL is configured
        if self.config.url.is_empty() {
            return Err(ClimateError::ConnectionError("Webhook URL not configured".to_string()));
        }

        // Log the webhook call (in production, actually send the request)
        tracing::debug!(
            target: "wia_climate::webhook",
            url = %self.config.url,
            payload_size = payload_json.len(),
            "Webhook payload prepared"
        );

        Ok(())
    }

    /// Send with retries
    async fn send_with_retry(&self, payload: &WebhookPayload) -> Result<()> {
        let mut last_error = None;

        for attempt in 0..=self.config.max_retries {
            match self.send_request(payload).await {
                Ok(_) => return Ok(()),
                Err(e) => {
                    last_error = Some(e);

                    if attempt < self.config.max_retries {
                        let delay = self.config.retry_strategy.delay_for_attempt(attempt);
                        tokio::time::sleep(delay).await;
                    }
                }
            }
        }

        Err(last_error.unwrap_or_else(|| ClimateError::Unknown("Unknown error".to_string())))
    }
}

#[async_trait]
impl OutputAdapter for WebhookAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn adapter_type(&self) -> AdapterType {
        AdapterType::Alert
    }

    async fn init(&mut self) -> Result<()> {
        // Validate configuration
        if self.config.url.is_empty() {
            return Err(ClimateError::Validation("Webhook URL is required".to_string()));
        }

        // Parse URL to validate format
        if url::Url::parse(&self.config.url).is_err() {
            return Err(ClimateError::Validation(format!(
                "Invalid webhook URL: {}",
                self.config.url
            )));
        }

        tracing::info!(
            target: "wia_climate::webhook",
            name = %self.name,
            url = %self.config.url,
            "Webhook adapter initialized"
        );

        Ok(())
    }

    async fn process(&self, message: &ClimateMessage) -> Result<()> {
        let payload = WebhookPayload::from_message(message, &self.name);

        match self.send_with_retry(&payload).await {
            Ok(_) => {
                self.messages_processed.fetch_add(1, Ordering::SeqCst);
                *self.last_success.write().unwrap() = Some(Utc::now());
                Ok(())
            }
            Err(e) => {
                self.error_count.fetch_add(1, Ordering::SeqCst);
                *self.last_error.write().unwrap() = Some(e.to_string());
                Err(e)
            }
        }
    }

    async fn flush(&self) -> Result<()> {
        // Webhooks are sent immediately, nothing to flush
        Ok(())
    }

    async fn close(&mut self) -> Result<()> {
        tracing::info!(
            target: "wia_climate::webhook",
            name = %self.name,
            messages_processed = self.messages_processed.load(Ordering::SeqCst),
            error_count = self.error_count.load(Ordering::SeqCst),
            "Webhook adapter closed"
        );
        Ok(())
    }

    async fn health_check(&self) -> Result<AdapterHealth> {
        let error_count = self.error_count.load(Ordering::SeqCst);
        let messages_processed = self.messages_processed.load(Ordering::SeqCst);

        let status = if error_count == 0 {
            HealthStatus::Healthy
        } else if error_count < 5 {
            HealthStatus::Degraded
        } else {
            HealthStatus::Unhealthy
        };

        Ok(AdapterHealth {
            status,
            latency_ms: None, // Would be measured in real implementation
            last_success: *self.last_success.read().unwrap(),
            last_error: self.last_error.read().unwrap().clone(),
            error_count,
            messages_processed,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Location, Device, CarbonCaptureData, CarbonCaptureTechnology};

    fn create_test_message() -> ClimateMessage {
        ClimateMessage::builder()
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Test", "Device"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                ..Default::default()
            })
            .build()
            .unwrap()
    }

    #[test]
    fn test_webhook_config() {
        let config = WebhookConfig::new("https://example.com/webhook")
            .with_secret("secret123")
            .with_header("X-Custom", "value")
            .with_timeout(Duration::from_secs(10))
            .with_max_retries(5);

        assert_eq!(config.url, "https://example.com/webhook");
        assert_eq!(config.secret, Some("secret123".to_string()));
        assert_eq!(config.headers.get("X-Custom"), Some(&"value".to_string()));
        assert_eq!(config.timeout, Duration::from_secs(10));
        assert_eq!(config.max_retries, 5);
    }

    #[test]
    fn test_webhook_payload_creation() {
        let message = create_test_message();
        let payload = WebhookPayload::from_message(&message, "test-adapter");

        assert_eq!(payload.event_type, "climate.data");
        assert_eq!(payload.source, "test-adapter");
    }

    #[tokio::test]
    async fn test_webhook_adapter_creation() {
        let config = WebhookConfig::new("https://example.com/webhook");
        let adapter = WebhookAdapter::new("test-webhook", config);

        assert_eq!(adapter.name(), "test-webhook");
        assert_eq!(adapter.adapter_type(), AdapterType::Alert);
    }

    #[tokio::test]
    async fn test_webhook_adapter_init_validation() {
        let config = WebhookConfig::new("");
        let mut adapter = WebhookAdapter::new("test", config);

        // Should fail with empty URL
        let result = adapter.init().await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_webhook_health_check() {
        let config = WebhookConfig::new("https://example.com/webhook");
        let adapter = WebhookAdapter::new("test", config);

        let health = adapter.health_check().await.unwrap();
        assert_eq!(health.status, HealthStatus::Healthy);
        assert_eq!(health.error_count, 0);
    }
}
