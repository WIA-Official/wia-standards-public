//! Utility functions for underwater communication

use crate::types::*;
use uuid::Uuid;

pub fn generate_message_id() -> Uuid {
    Uuid::new_v4()
}

pub fn calculate_propagation_delay(distance_meters: f64) -> f64 {
    // Speed of sound in water: ~1500 m/s
    distance_meters / 1500.0
}

pub fn estimate_transmission_time(data_size_bytes: usize, data_rate_bps: u32) -> f64 {
    (data_size_bytes * 8) as f64 / data_rate_bps as f64
}
