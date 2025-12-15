//! Measurement and sensor reading types

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Sensor reading
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorReading {
    pub sensor_id: String,
    pub sensor_type: SensorType,
    pub value: f64,
    pub unit: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accuracy: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
}

impl SensorReading {
    pub fn new(id: impl Into<String>, sensor_type: SensorType, value: f64, unit: impl Into<String>) -> Self {
        Self {
            sensor_id: id.into(),
            sensor_type,
            value,
            unit: unit.into(),
            target: None,
            accuracy: None,
            timestamp: Some(Utc::now()),
        }
    }

    pub fn with_target(mut self, target: impl Into<String>) -> Self {
        self.target = Some(target.into());
        self
    }

    pub fn with_accuracy(mut self, accuracy: f64) -> Self {
        self.accuracy = Some(accuracy);
        self
    }
}

/// Sensor type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorType {
    Chemical,
    Ph,
    Temperature,
    Pressure,
    Optical,
    Magnetic,
    Ultrasonic,
    Electrical,
}

impl SensorType {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Chemical => "chemical",
            Self::Ph => "ph",
            Self::Temperature => "temperature",
            Self::Pressure => "pressure",
            Self::Optical => "optical",
            Self::Magnetic => "magnetic",
            Self::Ultrasonic => "ultrasonic",
            Self::Electrical => "electrical",
        }
    }
}

/// Measurement with full context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Measurement {
    pub value: f64,
    pub unit: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<String>,
}

impl Measurement {
    pub fn new(value: f64, unit: impl Into<String>) -> Self {
        Self {
            value,
            unit: unit.into(),
            timestamp: Some(Utc::now().to_rfc3339()),
        }
    }
}

/// Performance metrics for sensors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorPerformance {
    pub detection_limit: f64,
    pub dynamic_range_min: f64,
    pub dynamic_range_max: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensitivity: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub specificity: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub response_time_ms: Option<f64>,
}

/// Signal characteristics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Signal {
    pub raw_intensity: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub baseline: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snr_db: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub emission_wavelength_nm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub excitation_wavelength_nm: Option<f64>,
}

/// Calibration information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Calibration {
    pub last_calibrated: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub curve_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub r_squared: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub valid_until: Option<String>,
}
