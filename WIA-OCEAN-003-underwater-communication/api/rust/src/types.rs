//! Type definitions for underwater communication

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcousticMessage {
    pub id: Uuid,
    pub sender_id: String,
    pub receiver_id: String,
    pub payload: Vec<u8>,
    pub frequency_khz: f64,
    pub transmission_power_db: f64,
    pub timestamp: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CommunicationProtocol {
    AcousticModem,
    OpticalWireless,
    Rf,
    UltrasonicBeacon,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionConfig {
    pub protocol: CommunicationProtocol,
    pub frequency_khz: f64,
    pub bandwidth_hz: f64,
    pub max_range_meters: f64,
    pub data_rate_bps: u32,
}
