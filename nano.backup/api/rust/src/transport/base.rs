//! Base transport types and configurations

use serde::{Deserialize, Serialize};

/// Configuration for direct transfer transport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DirectConfig {
    /// Transfer mechanism
    pub mechanism: DirectMechanism,
    /// Channel conductance in picosiemens
    pub channel_conductance_ps: f64,
    /// Contact duration in milliseconds
    pub contact_duration_ms: f64,
    /// Maximum distance for direct transfer in nanometers
    pub max_distance_nm: f64,
}

impl Default for DirectConfig {
    fn default() -> Self {
        Self {
            mechanism: DirectMechanism::GapJunction,
            channel_conductance_ps: 100.0,
            contact_duration_ms: 50.0,
            max_distance_nm: 10.0, // Must be very close for direct transfer
        }
    }
}

/// Direct transfer mechanism
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DirectMechanism {
    /// Gap junction channel
    GapJunction,
    /// Carbon nanotube channel
    Nanotube,
    /// Bacterial conjugation-like
    Conjugation,
    /// Vesicle-mediated transfer
    Vesicle,
}

/// Direct transfer transport implementation
pub struct DirectTransport {
    config: DirectConfig,
}

impl DirectTransport {
    pub fn new(config: DirectConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &DirectConfig {
        &self.config
    }
}

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use crate::transport::{Transport, TransportMethod, TransportResult};
use async_trait::async_trait;

#[async_trait]
impl Transport for DirectTransport {
    fn method(&self) -> TransportMethod {
        TransportMethod::Direct
    }

    fn estimate_delivery_time(
        &self,
        _source: &Position3D,
        _destination: &Position3D,
    ) -> NanoResult<f64> {
        // Direct transfer is nearly instantaneous once contact is made
        Ok(self.config.contact_duration_ms / 1000.0)
    }

    fn estimate_signal_strength(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _time_s: f64,
    ) -> NanoResult<f64> {
        let distance = source.distance_to(destination);

        if distance <= self.config.max_distance_nm {
            // Full strength if within contact range
            Ok(1.0)
        } else {
            Ok(0.0)
        }
    }

    async fn send(
        &self,
        source: &Position3D,
        destination: &Position3D,
        _payload_size: usize,
    ) -> NanoResult<TransportResult> {
        let distance = source.distance_to(destination);

        if distance > self.config.max_distance_nm {
            return Ok(TransportResult::failure(
                NanoError::TransportFailed(format!(
                    "Distance {} nm exceeds maximum {} nm for direct transfer",
                    distance, self.config.max_distance_nm
                ))
            ));
        }

        let delivery_time = self.estimate_delivery_time(source, destination)?;
        let signal_strength = self.estimate_signal_strength(source, destination, delivery_time)?;

        Ok(TransportResult::success(delivery_time, signal_strength))
    }

    fn is_reachable(&self, source: &Position3D, destination: &Position3D) -> bool {
        source.distance_to(destination) <= self.config.max_distance_nm
    }
}
