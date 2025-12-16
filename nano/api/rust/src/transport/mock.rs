//! Mock transport for testing
//!
//! Provides a configurable mock transport for unit and integration testing.

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use crate::transport::{Transport, TransportMethod, TransportResult};
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

/// Configuration for mock transport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MockConfig {
    /// Fixed delivery time in seconds
    pub delivery_time_s: f64,
    /// Fixed signal strength (0.0 - 1.0)
    pub signal_strength: f64,
    /// Whether sends should succeed
    pub should_succeed: bool,
    /// Maximum range (0 for unlimited)
    pub max_range_nm: f64,
    /// Simulated latency in milliseconds
    pub latency_ms: u64,
}

impl Default for MockConfig {
    fn default() -> Self {
        Self {
            delivery_time_s: 0.001,
            signal_strength: 1.0,
            should_succeed: true,
            max_range_nm: 0.0, // Unlimited
            latency_ms: 0,
        }
    }
}

impl MockConfig {
    /// Create a config that always succeeds
    pub fn always_success() -> Self {
        Self::default()
    }

    /// Create a config that always fails
    pub fn always_fail() -> Self {
        Self {
            should_succeed: false,
            ..Default::default()
        }
    }

    /// Create a config with limited range
    pub fn with_range(max_range_nm: f64) -> Self {
        Self {
            max_range_nm,
            ..Default::default()
        }
    }

    /// Create a config with simulated latency
    pub fn with_latency(latency_ms: u64) -> Self {
        Self {
            latency_ms,
            ..Default::default()
        }
    }
}

/// Mock transport for testing
pub struct MockTransport {
    config: MockConfig,
    send_count: Arc<AtomicUsize>,
    last_source: Arc<std::sync::RwLock<Option<Position3D>>>,
    last_destination: Arc<std::sync::RwLock<Option<Position3D>>>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new(config: MockConfig) -> Self {
        Self {
            config,
            send_count: Arc::new(AtomicUsize::new(0)),
            last_source: Arc::new(std::sync::RwLock::new(None)),
            last_destination: Arc::new(std::sync::RwLock::new(None)),
        }
    }

    /// Get the number of times send was called
    pub fn send_count(&self) -> usize {
        self.send_count.load(Ordering::SeqCst)
    }

    /// Get the last source position
    pub fn last_source(&self) -> Option<Position3D> {
        self.last_source.read().unwrap().clone()
    }

    /// Get the last destination position
    pub fn last_destination(&self) -> Option<Position3D> {
        self.last_destination.read().unwrap().clone()
    }

    /// Reset statistics
    pub fn reset(&self) {
        self.send_count.store(0, Ordering::SeqCst);
        *self.last_source.write().unwrap() = None;
        *self.last_destination.write().unwrap() = None;
    }

    /// Get config
    pub fn config(&self) -> &MockConfig {
        &self.config
    }
}

impl Clone for MockTransport {
    fn clone(&self) -> Self {
        Self {
            config: self.config.clone(),
            send_count: Arc::clone(&self.send_count),
            last_source: Arc::clone(&self.last_source),
            last_destination: Arc::clone(&self.last_destination),
        }
    }
}

#[async_trait]
impl Transport for MockTransport {
    fn method(&self) -> TransportMethod {
        TransportMethod::Mock
    }

    fn estimate_delivery_time(
        &self,
        _source: &Position3D,
        _destination: &Position3D,
    ) -> NanoResult<f64> {
        Ok(self.config.delivery_time_s)
    }

    fn estimate_signal_strength(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _time_s: f64,
    ) -> NanoResult<f64> {
        if self.config.max_range_nm > 0.0 {
            let distance = source.distance_to(destination);
            if distance > self.config.max_range_nm {
                return Ok(0.0);
            }
        }
        Ok(self.config.signal_strength)
    }

    async fn send(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _payload_size: usize,
    ) -> NanoResult<TransportResult> {
        // Record the call
        self.send_count.fetch_add(1, Ordering::SeqCst);
        *self.last_source.write().unwrap() = Some(source.clone());
        *self.last_destination.write().unwrap() = Some(destination.clone());

        // Simulate latency
        if self.config.latency_ms > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(self.config.latency_ms)).await;
        }

        // Check range if configured
        if self.config.max_range_nm > 0.0 {
            let distance = source.distance_to(destination);
            if distance > self.config.max_range_nm {
                return Ok(TransportResult::failure(
                    NanoError::TransportFailed(format!(
                        "Distance {} nm exceeds mock range {} nm",
                        distance, self.config.max_range_nm
                    ))
                ));
            }
        }

        if self.config.should_succeed {
            Ok(TransportResult::success(
                self.config.delivery_time_s,
                self.config.signal_strength,
            ))
        } else {
            Ok(TransportResult::failure(
                NanoError::TransportFailed("Mock transport configured to fail".into())
            ))
        }
    }

    fn is_reachable(&self, source: &Position3D, destination: &Position3D) -> bool {
        if self.config.max_range_nm <= 0.0 {
            return true; // Unlimited range
        }
        source.distance_to(destination) <= self.config.max_range_nm
    }
}

/// Builder for mock transport scenarios
pub struct MockTransportBuilder {
    config: MockConfig,
}

impl MockTransportBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            config: MockConfig::default(),
        }
    }

    /// Set delivery time
    pub fn delivery_time(mut self, seconds: f64) -> Self {
        self.config.delivery_time_s = seconds;
        self
    }

    /// Set signal strength
    pub fn signal_strength(mut self, strength: f64) -> Self {
        self.config.signal_strength = strength.clamp(0.0, 1.0);
        self
    }

    /// Set success/failure
    pub fn should_succeed(mut self, succeed: bool) -> Self {
        self.config.should_succeed = succeed;
        self
    }

    /// Set maximum range
    pub fn max_range(mut self, range_nm: f64) -> Self {
        self.config.max_range_nm = range_nm;
        self
    }

    /// Set latency
    pub fn latency(mut self, ms: u64) -> Self {
        self.config.latency_ms = ms;
        self
    }

    /// Build the mock transport
    pub fn build(self) -> MockTransport {
        MockTransport::new(self.config)
    }
}

impl Default for MockTransportBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_success() {
        let transport = MockTransport::new(MockConfig::always_success());

        let source = Position3D::new(0.0, 0.0, 0.0);
        let dest = Position3D::new(100.0, 0.0, 0.0);

        let result = transport.send(&source, &dest, 100).await.unwrap();
        assert!(result.success);
        assert_eq!(transport.send_count(), 1);
    }

    #[tokio::test]
    async fn test_mock_failure() {
        let transport = MockTransport::new(MockConfig::always_fail());

        let source = Position3D::new(0.0, 0.0, 0.0);
        let dest = Position3D::new(100.0, 0.0, 0.0);

        let result = transport.send(&source, &dest, 100).await.unwrap();
        assert!(!result.success);
    }

    #[tokio::test]
    async fn test_mock_range_limit() {
        let transport = MockTransport::new(MockConfig::with_range(50.0));

        let source = Position3D::new(0.0, 0.0, 0.0);
        let close = Position3D::new(30.0, 0.0, 0.0);
        let far = Position3D::new(100.0, 0.0, 0.0);

        let result_close = transport.send(&source, &close, 100).await.unwrap();
        assert!(result_close.success);

        let result_far = transport.send(&source, &far, 100).await.unwrap();
        assert!(!result_far.success);
    }

    #[test]
    fn test_builder() {
        let transport = MockTransportBuilder::new()
            .delivery_time(0.5)
            .signal_strength(0.8)
            .max_range(1000.0)
            .build();

        assert_eq!(transport.config().delivery_time_s, 0.5);
        assert_eq!(transport.config().signal_strength, 0.8);
        assert_eq!(transport.config().max_range_nm, 1000.0);
    }

    #[test]
    fn test_position_tracking() {
        let transport = MockTransport::new(MockConfig::default());

        let source = Position3D::new(10.0, 20.0, 30.0);
        let dest = Position3D::new(40.0, 50.0, 60.0);

        // Initially no positions recorded
        assert!(transport.last_source().is_none());
        assert!(transport.last_destination().is_none());

        // After send, positions should be recorded
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            transport.send(&source, &dest, 100).await.unwrap();
        });

        assert!(transport.last_source().is_some());
        assert!(transport.last_destination().is_some());
    }
}
