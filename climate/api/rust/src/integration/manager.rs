//! Output Manager for coordinating multiple adapters

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::RwLock;
use futures_util::future::join_all;

use crate::error::Result;
use crate::core::climate::ClimateMessage;
use super::adapter::{OutputAdapter, AdapterHealth, HealthStatus};

/// Processing strategy for output adapters
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProcessStrategy {
    /// Process through all adapters in parallel
    Fanout,
    /// Process through adapters sequentially
    Pipeline,
    /// Process only through selected adapters by name
    Selective(Vec<String>),
}

impl Default for ProcessStrategy {
    fn default() -> Self {
        ProcessStrategy::Fanout
    }
}

/// Configuration for the output manager
#[derive(Debug, Clone)]
pub struct OutputConfig {
    /// Processing strategy
    pub strategy: ProcessStrategy,
    /// Message buffer size
    pub buffer_size: usize,
    /// Auto-flush interval
    pub flush_interval: Duration,
    /// Enable circuit breaker
    pub circuit_breaker_enabled: bool,
    /// Circuit breaker threshold (failures before open)
    pub circuit_breaker_threshold: u32,
    /// Circuit breaker reset timeout
    pub circuit_breaker_timeout: Duration,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            strategy: ProcessStrategy::Fanout,
            buffer_size: 1000,
            flush_interval: Duration::from_secs(5),
            circuit_breaker_enabled: true,
            circuit_breaker_threshold: 5,
            circuit_breaker_timeout: Duration::from_secs(30),
        }
    }
}

impl OutputConfig {
    /// Create a new output config with fanout strategy
    pub fn fanout() -> Self {
        Self {
            strategy: ProcessStrategy::Fanout,
            ..Default::default()
        }
    }

    /// Create a new output config with pipeline strategy
    pub fn pipeline() -> Self {
        Self {
            strategy: ProcessStrategy::Pipeline,
            ..Default::default()
        }
    }

    /// Set buffer size
    pub fn with_buffer_size(mut self, size: usize) -> Self {
        self.buffer_size = size;
        self
    }

    /// Set flush interval
    pub fn with_flush_interval(mut self, interval: Duration) -> Self {
        self.flush_interval = interval;
        self
    }

    /// Enable or disable circuit breaker
    pub fn with_circuit_breaker(mut self, enabled: bool) -> Self {
        self.circuit_breaker_enabled = enabled;
        self
    }
}

/// Circuit breaker state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircuitState {
    /// Normal operation
    Closed,
    /// Allowing limited traffic to test recovery
    HalfOpen,
    /// Rejecting all traffic
    Open,
}

/// Circuit breaker for an adapter
#[derive(Debug)]
struct CircuitBreaker {
    state: CircuitState,
    failure_count: u32,
    last_failure: Option<std::time::Instant>,
    threshold: u32,
    timeout: Duration,
}

impl CircuitBreaker {
    fn new(threshold: u32, timeout: Duration) -> Self {
        Self {
            state: CircuitState::Closed,
            failure_count: 0,
            last_failure: None,
            threshold,
            timeout,
        }
    }

    fn allow_request(&mut self) -> bool {
        match self.state {
            CircuitState::Closed => true,
            CircuitState::Open => {
                // Check if timeout has elapsed
                if let Some(last) = self.last_failure {
                    if last.elapsed() >= self.timeout {
                        self.state = CircuitState::HalfOpen;
                        return true;
                    }
                }
                false
            }
            CircuitState::HalfOpen => true,
        }
    }

    fn record_success(&mut self) {
        self.failure_count = 0;
        self.state = CircuitState::Closed;
    }

    fn record_failure(&mut self) {
        self.failure_count += 1;
        self.last_failure = Some(std::time::Instant::now());

        if self.failure_count >= self.threshold {
            self.state = CircuitState::Open;
        }
    }
}

/// Result of processing a message through the output manager
#[derive(Debug, Clone)]
pub struct ProcessResult {
    /// Adapters that successfully processed the message
    pub success: Vec<String>,
    /// Adapters that failed to process the message
    pub failed: Vec<(String, String)>, // (adapter_name, error)
    /// Processing time in milliseconds
    pub duration_ms: u64,
}

impl ProcessResult {
    /// Check if all adapters succeeded
    pub fn all_success(&self) -> bool {
        self.failed.is_empty()
    }

    /// Get total number of adapters that processed
    pub fn total(&self) -> usize {
        self.success.len() + self.failed.len()
    }
}

/// Output Manager for coordinating multiple adapters
///
/// The OutputManager is responsible for routing climate messages to
/// one or more output adapters based on the configured processing strategy.
pub struct OutputManager {
    adapters: Vec<Arc<RwLock<Box<dyn OutputAdapter>>>>,
    adapter_names: Vec<String>,
    config: OutputConfig,
    circuit_breakers: HashMap<String, CircuitBreaker>,
    messages_processed: u64,
}

impl OutputManager {
    /// Create a new output manager with the given configuration
    pub fn new(config: OutputConfig) -> Self {
        Self {
            adapters: Vec::new(),
            adapter_names: Vec::new(),
            config,
            circuit_breakers: HashMap::new(),
            messages_processed: 0,
        }
    }

    /// Create a new output manager with default configuration
    pub fn default_config() -> Self {
        Self::new(OutputConfig::default())
    }

    /// Add an adapter to the manager
    pub fn add_adapter<A: OutputAdapter + 'static>(&mut self, adapter: A) {
        let name = adapter.name().to_string();

        // Create circuit breaker for this adapter
        if self.config.circuit_breaker_enabled {
            self.circuit_breakers.insert(
                name.clone(),
                CircuitBreaker::new(
                    self.config.circuit_breaker_threshold,
                    self.config.circuit_breaker_timeout,
                ),
            );
        }

        self.adapter_names.push(name);
        self.adapters.push(Arc::new(RwLock::new(Box::new(adapter))));
    }

    /// Remove an adapter by name
    pub fn remove_adapter(&mut self, name: &str) -> bool {
        if let Some(idx) = self.adapter_names.iter().position(|n| n == name) {
            self.adapters.remove(idx);
            self.adapter_names.remove(idx);
            self.circuit_breakers.remove(name);
            true
        } else {
            false
        }
    }

    /// Get list of adapter names
    pub fn adapter_names(&self) -> &[String] {
        &self.adapter_names
    }

    /// Get number of adapters
    pub fn adapter_count(&self) -> usize {
        self.adapters.len()
    }

    /// Initialize all adapters
    pub async fn init_all(&mut self) -> Result<()> {
        for adapter in &self.adapters {
            let mut adapter = adapter.write().await;
            adapter.init().await?;
        }
        Ok(())
    }

    /// Process a single message through all adapters
    pub async fn process(&mut self, message: &ClimateMessage) -> Result<ProcessResult> {
        let start = std::time::Instant::now();
        let mut success = Vec::new();
        let mut failed = Vec::new();

        let adapters_to_process = match &self.config.strategy {
            ProcessStrategy::Fanout | ProcessStrategy::Pipeline => {
                self.adapter_names.clone()
            }
            ProcessStrategy::Selective(names) => names.clone(),
        };

        match self.config.strategy {
            ProcessStrategy::Fanout | ProcessStrategy::Selective(_) => {
                // Process in parallel
                let futures: Vec<_> = adapters_to_process
                    .iter()
                    .filter_map(|name| {
                        let idx = self.adapter_names.iter().position(|n| n == name)?;
                        Some((name.clone(), self.adapters[idx].clone()))
                    })
                    .filter(|(name, _)| {
                        // Check circuit breaker
                        if let Some(cb) = self.circuit_breakers.get_mut(name) {
                            cb.allow_request()
                        } else {
                            true
                        }
                    })
                    .map(|(name, adapter)| {
                        let message = message.clone();
                        async move {
                            let adapter = adapter.read().await;
                            let result = adapter.process(&message).await;
                            (name, result)
                        }
                    })
                    .collect();

                let results = join_all(futures).await;

                for (name, result) in results {
                    match result {
                        Ok(_) => {
                            if let Some(cb) = self.circuit_breakers.get_mut(&name) {
                                cb.record_success();
                            }
                            success.push(name);
                        }
                        Err(e) => {
                            if let Some(cb) = self.circuit_breakers.get_mut(&name) {
                                cb.record_failure();
                            }
                            failed.push((name, e.to_string()));
                        }
                    }
                }
            }
            ProcessStrategy::Pipeline => {
                // Process sequentially
                for name in adapters_to_process {
                    let idx = match self.adapter_names.iter().position(|n| *n == name) {
                        Some(i) => i,
                        None => continue,
                    };

                    // Check circuit breaker
                    if let Some(cb) = self.circuit_breakers.get_mut(&name) {
                        if !cb.allow_request() {
                            failed.push((name, "Circuit breaker open".to_string()));
                            continue;
                        }
                    }

                    let adapter = self.adapters[idx].read().await;
                    match adapter.process(message).await {
                        Ok(_) => {
                            if let Some(cb) = self.circuit_breakers.get_mut(&name) {
                                cb.record_success();
                            }
                            success.push(name);
                        }
                        Err(e) => {
                            if let Some(cb) = self.circuit_breakers.get_mut(&name) {
                                cb.record_failure();
                            }
                            failed.push((name, e.to_string()));
                            // In pipeline mode, stop on first failure
                            break;
                        }
                    }
                }
            }
        }

        self.messages_processed += 1;

        Ok(ProcessResult {
            success,
            failed,
            duration_ms: start.elapsed().as_millis() as u64,
        })
    }

    /// Process a batch of messages
    pub async fn process_batch(&mut self, messages: &[ClimateMessage]) -> Result<Vec<ProcessResult>> {
        let mut results = Vec::with_capacity(messages.len());
        for message in messages {
            results.push(self.process(message).await?);
        }
        Ok(results)
    }

    /// Flush all adapters
    pub async fn flush_all(&self) -> Result<()> {
        for adapter in &self.adapters {
            let adapter = adapter.read().await;
            adapter.flush().await?;
        }
        Ok(())
    }

    /// Get health status of all adapters
    pub async fn health(&self) -> HashMap<String, AdapterHealth> {
        let mut health_map = HashMap::new();

        for (i, adapter) in self.adapters.iter().enumerate() {
            let name = &self.adapter_names[i];
            let adapter = adapter.read().await;

            let health = match adapter.health_check().await {
                Ok(h) => h,
                Err(e) => AdapterHealth::unhealthy(e.to_string()),
            };

            health_map.insert(name.clone(), health);
        }

        health_map
    }

    /// Check if all adapters are healthy
    pub async fn is_healthy(&self) -> bool {
        let health = self.health().await;
        health.values().all(|h| h.status == HealthStatus::Healthy)
    }

    /// Shutdown all adapters gracefully
    pub async fn shutdown(&mut self) -> Result<()> {
        // Flush first
        self.flush_all().await?;

        // Close all adapters
        for adapter in &self.adapters {
            let mut adapter = adapter.write().await;
            adapter.close().await?;
        }

        Ok(())
    }

    /// Get total messages processed
    pub fn messages_processed(&self) -> u64 {
        self.messages_processed
    }

    /// Get circuit breaker state for an adapter
    pub fn circuit_state(&self, adapter_name: &str) -> Option<CircuitState> {
        self.circuit_breakers.get(adapter_name).map(|cb| cb.state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_config_default() {
        let config = OutputConfig::default();
        assert_eq!(config.strategy, ProcessStrategy::Fanout);
        assert_eq!(config.buffer_size, 1000);
        assert!(config.circuit_breaker_enabled);
    }

    #[test]
    fn test_output_config_builder() {
        let config = OutputConfig::fanout()
            .with_buffer_size(500)
            .with_circuit_breaker(false);

        assert_eq!(config.strategy, ProcessStrategy::Fanout);
        assert_eq!(config.buffer_size, 500);
        assert!(!config.circuit_breaker_enabled);
    }

    #[test]
    fn test_circuit_breaker() {
        let mut cb = CircuitBreaker::new(3, Duration::from_secs(30));

        // Initially closed
        assert!(cb.allow_request());

        // Record failures
        cb.record_failure();
        cb.record_failure();
        assert!(cb.allow_request()); // Still closed

        cb.record_failure();
        assert_eq!(cb.state, CircuitState::Open);
        assert!(!cb.allow_request()); // Now open

        // Record success resets
        cb.state = CircuitState::HalfOpen;
        cb.record_success();
        assert_eq!(cb.state, CircuitState::Closed);
        assert!(cb.allow_request());
    }

    #[test]
    fn test_output_manager_creation() {
        let manager = OutputManager::default_config();
        assert_eq!(manager.adapter_count(), 0);
        assert_eq!(manager.messages_processed(), 0);
    }

    #[test]
    fn test_process_result() {
        let result = ProcessResult {
            success: vec!["a".to_string(), "b".to_string()],
            failed: vec![("c".to_string(), "error".to_string())],
            duration_ms: 10,
        };

        assert!(!result.all_success());
        assert_eq!(result.total(), 3);
    }
}
