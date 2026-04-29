//! Transport layer for WIA Nano Protocol
//!
//! This module provides various transport mechanisms for nanoscale communication:
//! - Diffusion-based transport (molecular communication)
//! - Guided transport (magnetic, acoustic, optical)
//! - Direct transfer (contact-based)
//! - Mock transport for testing

mod base;
mod diffusion;
mod guided;
mod mock;

pub use base::*;
pub use diffusion::*;
pub use guided::*;
pub use mock::*;

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use async_trait::async_trait;
use serde::{Deserialize, Serialize};

/// Transport method enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportMethod {
    /// Diffusion-based molecular communication
    Diffusion,
    /// Externally guided transport
    Guided,
    /// Direct contact-based transfer
    Direct,
    /// Mock transport for testing
    Mock,
}

/// Transport configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransportConfig {
    /// Transport method
    pub method: TransportMethod,
    /// Method-specific parameters
    pub parameters: TransportParameters,
}

/// Transport-specific parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum TransportParameters {
    /// Diffusion transport parameters
    Diffusion(DiffusionConfig),
    /// Guided transport parameters
    Guided(GuidedConfig),
    /// Direct transfer parameters
    Direct(DirectConfig),
    /// Mock transport parameters
    Mock(MockConfig),
}

/// Result of a transport operation
#[derive(Debug)]
pub struct TransportResult {
    /// Whether the transport was successful
    pub success: bool,
    /// Estimated delivery time in seconds
    pub delivery_time_s: Option<f64>,
    /// Signal strength at destination (0.0 - 1.0)
    pub signal_strength: Option<f64>,
    /// Error if transport failed
    pub error: Option<NanoError>,
}

impl TransportResult {
    /// Create a successful transport result
    pub fn success(delivery_time_s: f64, signal_strength: f64) -> Self {
        Self {
            success: true,
            delivery_time_s: Some(delivery_time_s),
            signal_strength: Some(signal_strength),
            error: None,
        }
    }

    /// Create a failed transport result
    pub fn failure(error: NanoError) -> Self {
        Self {
            success: false,
            delivery_time_s: None,
            signal_strength: None,
            error: Some(error),
        }
    }
}

/// Transport trait for all transport mechanisms
#[async_trait]
pub trait Transport: Send + Sync {
    /// Get the transport method
    fn method(&self) -> TransportMethod;

    /// Estimate delivery time between two positions
    fn estimate_delivery_time(
        &self,
        source: &Position3D,
        destination: &Position3D,
    ) -> NanoResult<f64>;

    /// Estimate signal strength at destination
    fn estimate_signal_strength(
        &self,
        source: &Position3D,
        destination: &Position3D,
        time_s: f64,
    ) -> NanoResult<f64>;

    /// Send a message
    async fn send(
        &self,
        source: &Position3D,
        destination: &Position3D,
        payload_size: usize,
    ) -> NanoResult<TransportResult>;

    /// Check if destination is reachable
    fn is_reachable(&self, source: &Position3D, destination: &Position3D) -> bool;
}

/// Create a transport instance from configuration
pub fn create_transport(config: TransportConfig) -> Box<dyn Transport> {
    match config.parameters {
        TransportParameters::Diffusion(cfg) => Box::new(DiffusionTransport::new(cfg)),
        TransportParameters::Guided(cfg) => Box::new(GuidedTransport::new(cfg)),
        TransportParameters::Direct(cfg) => Box::new(DirectTransport::new(cfg)),
        TransportParameters::Mock(cfg) => Box::new(MockTransport::new(cfg)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_result_success() {
        let result = TransportResult::success(1.5, 0.8);
        assert!(result.success);
        assert_eq!(result.delivery_time_s, Some(1.5));
        assert_eq!(result.signal_strength, Some(0.8));
    }

    #[test]
    fn test_transport_result_failure() {
        let error = NanoError::TransportFailed("Test error".into());
        let result = TransportResult::failure(error);
        assert!(!result.success);
        assert!(result.error.is_some());
    }
}
