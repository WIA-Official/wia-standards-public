//! Output adapter trait and related types

use async_trait::async_trait;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::time::Duration;

use crate::error::Result;
use crate::core::climate::ClimateMessage;

/// Category of output adapter
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdapterType {
    /// Visualization dashboard (Grafana, Cesium)
    Dashboard,
    /// Time-series storage (InfluxDB, TimescaleDB)
    Storage,
    /// Alert/notification (Webhook, Slack)
    Alert,
    /// Data export (NetCDF, Parquet)
    Export,
    /// Console output for testing
    Console,
    /// Custom adapter
    Custom,
}

impl std::fmt::Display for AdapterType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AdapterType::Dashboard => write!(f, "dashboard"),
            AdapterType::Storage => write!(f, "storage"),
            AdapterType::Alert => write!(f, "alert"),
            AdapterType::Export => write!(f, "export"),
            AdapterType::Console => write!(f, "console"),
            AdapterType::Custom => write!(f, "custom"),
        }
    }
}

/// Health status of an adapter
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HealthStatus {
    /// Adapter is functioning normally
    Healthy,
    /// Adapter is experiencing issues but operational
    Degraded,
    /// Adapter is not functioning
    Unhealthy,
}

impl Default for HealthStatus {
    fn default() -> Self {
        HealthStatus::Healthy
    }
}

/// Adapter health information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdapterHealth {
    /// Current health status
    pub status: HealthStatus,
    /// Response latency in milliseconds
    pub latency_ms: Option<u64>,
    /// Last successful operation timestamp
    pub last_success: Option<DateTime<Utc>>,
    /// Last error message
    pub last_error: Option<String>,
    /// Error count since startup
    pub error_count: u64,
    /// Messages processed since startup
    pub messages_processed: u64,
}

impl Default for AdapterHealth {
    fn default() -> Self {
        Self {
            status: HealthStatus::Healthy,
            latency_ms: None,
            last_success: None,
            last_error: None,
            error_count: 0,
            messages_processed: 0,
        }
    }
}

impl AdapterHealth {
    /// Create a healthy status
    pub fn healthy() -> Self {
        Self::default()
    }

    /// Create an unhealthy status with error message
    pub fn unhealthy(error: impl Into<String>) -> Self {
        Self {
            status: HealthStatus::Unhealthy,
            last_error: Some(error.into()),
            ..Default::default()
        }
    }

    /// Create a degraded status
    pub fn degraded(error: impl Into<String>) -> Self {
        Self {
            status: HealthStatus::Degraded,
            last_error: Some(error.into()),
            ..Default::default()
        }
    }
}

/// Result of processing a batch of messages
#[derive(Debug, Clone, Default)]
pub struct BatchResult {
    /// Number of successfully processed messages
    pub success: usize,
    /// Number of failed messages
    pub failed: usize,
}

impl BatchResult {
    /// Create a new batch result
    pub fn new(success: usize, failed: usize) -> Self {
        Self { success, failed }
    }

    /// Total messages in the batch
    pub fn total(&self) -> usize {
        self.success + self.failed
    }

    /// Success rate (0.0 to 1.0)
    pub fn success_rate(&self) -> f64 {
        if self.total() == 0 {
            1.0
        } else {
            self.success as f64 / self.total() as f64
        }
    }
}

/// Output adapter trait for processing climate messages
///
/// Implement this trait to create custom output adapters that can receive
/// and process WIA Climate messages for storage, visualization, or alerting.
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Returns the adapter's unique name
    fn name(&self) -> &str;

    /// Returns the adapter type category
    fn adapter_type(&self) -> AdapterType;

    /// Initialize the adapter with configuration
    ///
    /// Called once when the adapter is added to the output manager.
    async fn init(&mut self) -> Result<()>;

    /// Process a single climate message
    ///
    /// This is the main method called for each incoming message.
    async fn process(&self, message: &ClimateMessage) -> Result<()>;

    /// Process a batch of messages
    ///
    /// Default implementation iterates and calls process() for each message.
    /// Override for more efficient batch processing.
    async fn process_batch(&self, messages: &[ClimateMessage]) -> Result<BatchResult> {
        let mut success = 0;
        let mut failed = 0;
        for msg in messages {
            match self.process(msg).await {
                Ok(_) => success += 1,
                Err(_) => failed += 1,
            }
        }
        Ok(BatchResult { success, failed })
    }

    /// Flush any buffered data
    ///
    /// Called periodically or when the manager is shutting down.
    async fn flush(&self) -> Result<()>;

    /// Gracefully close the adapter
    ///
    /// Called when the adapter is being removed or the manager is shutting down.
    async fn close(&mut self) -> Result<()>;

    /// Check adapter health status
    ///
    /// Should return current health information including latency and error counts.
    async fn health_check(&self) -> Result<AdapterHealth>;
}

/// Retry strategy for handling transient failures
#[derive(Debug, Clone)]
pub struct RetryStrategy {
    /// Maximum number of retries
    pub max_retries: u32,
    /// Initial backoff duration
    pub initial_backoff: Duration,
    /// Maximum backoff duration
    pub max_backoff: Duration,
    /// Backoff multiplier
    pub multiplier: f64,
    /// Add jitter to prevent thundering herd
    pub jitter: bool,
}

impl Default for RetryStrategy {
    fn default() -> Self {
        Self {
            max_retries: 3,
            initial_backoff: Duration::from_secs(1),
            max_backoff: Duration::from_secs(60),
            multiplier: 2.0,
            jitter: true,
        }
    }
}

impl RetryStrategy {
    /// Create a new retry strategy
    pub fn new(max_retries: u32) -> Self {
        Self {
            max_retries,
            ..Default::default()
        }
    }

    /// Set initial backoff duration
    pub fn with_initial_backoff(mut self, duration: Duration) -> Self {
        self.initial_backoff = duration;
        self
    }

    /// Set maximum backoff duration
    pub fn with_max_backoff(mut self, duration: Duration) -> Self {
        self.max_backoff = duration;
        self
    }

    /// Set backoff multiplier
    pub fn with_multiplier(mut self, multiplier: f64) -> Self {
        self.multiplier = multiplier;
        self
    }

    /// Enable or disable jitter
    pub fn with_jitter(mut self, jitter: bool) -> Self {
        self.jitter = jitter;
        self
    }

    /// Calculate delay for given attempt number (0-indexed)
    pub fn delay_for_attempt(&self, attempt: u32) -> Duration {
        let base_ms = self.initial_backoff.as_millis() as f64
            * self.multiplier.powi(attempt as i32);
        let capped_ms = base_ms.min(self.max_backoff.as_millis() as f64);

        if self.jitter {
            // Add up to 25% random jitter
            let jitter_factor = 1.0 + (rand_simple() * 0.25);
            Duration::from_millis((capped_ms * jitter_factor) as u64)
        } else {
            Duration::from_millis(capped_ms as u64)
        }
    }
}

/// Simple random number generator (0.0 to 1.0) without external dependencies
fn rand_simple() -> f64 {
    use std::time::SystemTime;
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_default()
        .subsec_nanos();
    (nanos as f64 % 1000.0) / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adapter_type_display() {
        assert_eq!(AdapterType::Dashboard.to_string(), "dashboard");
        assert_eq!(AdapterType::Storage.to_string(), "storage");
        assert_eq!(AdapterType::Alert.to_string(), "alert");
    }

    #[test]
    fn test_batch_result() {
        let result = BatchResult::new(8, 2);
        assert_eq!(result.total(), 10);
        assert!((result.success_rate() - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_retry_strategy() {
        let strategy = RetryStrategy::new(5)
            .with_initial_backoff(Duration::from_millis(100))
            .with_multiplier(2.0)
            .with_jitter(false);

        assert_eq!(strategy.delay_for_attempt(0), Duration::from_millis(100));
        assert_eq!(strategy.delay_for_attempt(1), Duration::from_millis(200));
        assert_eq!(strategy.delay_for_attempt(2), Duration::from_millis(400));
    }

    #[test]
    fn test_adapter_health() {
        let healthy = AdapterHealth::healthy();
        assert_eq!(healthy.status, HealthStatus::Healthy);

        let unhealthy = AdapterHealth::unhealthy("Connection failed");
        assert_eq!(unhealthy.status, HealthStatus::Unhealthy);
        assert_eq!(unhealthy.last_error, Some("Connection failed".to_string()));
    }
}
