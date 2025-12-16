//! Common types and enums used across all nano systems

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Nano system type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NanoSystemType {
    MolecularAssembler,
    NanoMachine,
    MolecularMemory,
    Nanomedicine,
    Nanorobot,
    Nanosensor,
    Custom,
}

impl NanoSystemType {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::MolecularAssembler => "molecular_assembler",
            Self::NanoMachine => "nano_machine",
            Self::MolecularMemory => "molecular_memory",
            Self::Nanomedicine => "nanomedicine",
            Self::Nanorobot => "nanorobot",
            Self::Nanosensor => "nanosensor",
            Self::Custom => "custom",
        }
    }
}

/// Device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Device {
    pub manufacturer: String,
    pub model: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub firmware: Option<String>,
}

impl Device {
    pub fn new(manufacturer: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            manufacturer: manufacturer.into(),
            model: model.into(),
            serial: None,
            firmware: None,
        }
    }

    pub fn with_serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }

    pub fn with_firmware(mut self, firmware: impl Into<String>) -> Self {
        self.firmware = Some(firmware.into());
        self
    }
}

/// Timestamp with multiple representations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timestamp {
    pub iso8601: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unix_ms: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unix_ns: Option<i64>,
}

impl Timestamp {
    pub fn now() -> Self {
        let now = Utc::now();
        Self {
            iso8601: now.to_rfc3339(),
            unix_ms: Some(now.timestamp_millis()),
            unix_ns: Some(now.timestamp_nanos_opt().unwrap_or(0)),
        }
    }

    pub fn from_datetime(dt: DateTime<Utc>) -> Self {
        Self {
            iso8601: dt.to_rfc3339(),
            unix_ms: Some(dt.timestamp_millis()),
            unix_ns: dt.timestamp_nanos_opt(),
        }
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::now()
    }
}

/// Scale/unit information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scale {
    #[serde(default = "default_length_unit")]
    pub length_unit: LengthUnit,
    #[serde(default = "default_time_unit")]
    pub time_unit: TimeUnit,
    #[serde(default = "default_mass_unit")]
    pub mass_unit: MassUnit,
}

fn default_length_unit() -> LengthUnit { LengthUnit::Nanometer }
fn default_time_unit() -> TimeUnit { TimeUnit::Nanosecond }
fn default_mass_unit() -> MassUnit { MassUnit::Dalton }

impl Default for Scale {
    fn default() -> Self {
        Self {
            length_unit: LengthUnit::Nanometer,
            time_unit: TimeUnit::Nanosecond,
            mass_unit: MassUnit::Dalton,
        }
    }
}

/// Length unit enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LengthUnit {
    Picometer,
    Angstrom,
    Nanometer,
    Micrometer,
}

/// Time unit enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimeUnit {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
}

/// Mass unit enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MassUnit {
    Dalton,
    Femtogram,
    Picogram,
}

/// Metadata for messages
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Metadata {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_margin: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calibration_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub experiment_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

impl Metadata {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_confidence(mut self, confidence: f64) -> Self {
        self.confidence = Some(confidence);
        self
    }

    pub fn with_calibration(mut self, id: impl Into<String>) -> Self {
        self.calibration_id = Some(id.into());
        self
    }
}
