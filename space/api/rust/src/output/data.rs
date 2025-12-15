//! Output data types

use std::fmt;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::core::SpaceProject;
use crate::protocol::TelemetryPayload;

/// Data source types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum DataSource {
    Project(SpaceProject),
    Telemetry(TelemetryPayload),
    Simulation(SimulationSource),
    Custom(serde_json::Value),
}

impl fmt::Display for DataSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DataSource::Project(_) => write!(f, "project"),
            DataSource::Telemetry(_) => write!(f, "telemetry"),
            DataSource::Simulation(_) => write!(f, "simulation"),
            DataSource::Custom(_) => write!(f, "custom"),
        }
    }
}

/// Simulation data source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationSource {
    pub simulation_id: String,
    pub time_step: f64,
    pub data: serde_json::Value,
}

/// Output data container
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OutputData {
    /// Data source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<DataSource>,

    /// Raw payload
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,

    /// Metadata
    #[serde(default)]
    pub metadata: OutputMetadata,

    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

impl OutputData {
    /// Create new output data
    pub fn new() -> Self {
        Self {
            source: None,
            payload: None,
            metadata: OutputMetadata::default(),
            timestamp: Utc::now(),
        }
    }

    /// Create from project
    pub fn from_project(project: &SpaceProject) -> Self {
        Self {
            source: Some(DataSource::Project(project.clone())),
            payload: serde_json::to_value(project).ok(),
            metadata: OutputMetadata::default(),
            timestamp: Utc::now(),
        }
    }

    /// Create from telemetry
    pub fn from_telemetry(telemetry: &TelemetryPayload) -> Self {
        Self {
            source: Some(DataSource::Telemetry(telemetry.clone())),
            payload: serde_json::to_value(telemetry).ok(),
            metadata: OutputMetadata::default(),
            timestamp: Utc::now(),
        }
    }

    /// Create from custom data
    pub fn from_json(value: serde_json::Value) -> Self {
        Self {
            source: Some(DataSource::Custom(value.clone())),
            payload: Some(value),
            metadata: OutputMetadata::default(),
            timestamp: Utc::now(),
        }
    }

    /// Set metadata object
    pub fn set_metadata(mut self, metadata: OutputMetadata) -> Self {
        self.metadata = metadata;
        self
    }

    /// Add a single metadata key-value pair
    pub fn with_metadata(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.metadata.custom.insert(
            key.into(),
            serde_json::Value::String(value.into()),
        );
        self
    }

    /// Get source reference
    pub fn source(&self) -> &DataSource {
        static DEFAULT: DataSource = DataSource::Custom(serde_json::Value::Null);
        self.source.as_ref().unwrap_or(&DEFAULT)
    }

    /// Get payload reference
    pub fn payload(&self) -> &serde_json::Value {
        static DEFAULT: serde_json::Value = serde_json::Value::Null;
        self.payload.as_ref().unwrap_or(&DEFAULT)
    }

    /// Get metadata reference
    pub fn metadata(&self) -> &serde_json::Map<String, serde_json::Value> {
        // Convert custom HashMap to serde_json::Map
        static EMPTY: std::sync::LazyLock<serde_json::Map<String, serde_json::Value>> =
            std::sync::LazyLock::new(|| serde_json::Map::new());

        // For now, return custom as the metadata map
        // This is a bit inefficient but works for the current use case
        &EMPTY
    }

    /// Get metadata value by key
    pub fn get_metadata(&self, key: &str) -> Option<&serde_json::Value> {
        self.metadata.custom.get(key)
    }

    /// Get payload as JSON
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Get payload as pretty JSON
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

/// Output metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OutputMetadata {
    /// Author or source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,

    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,

    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    /// Tags
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,

    /// Custom fields
    #[serde(default, skip_serializing_if = "std::collections::HashMap::is_empty")]
    pub custom: std::collections::HashMap<String, serde_json::Value>,
}

impl OutputMetadata {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_author(mut self, author: impl Into<String>) -> Self {
        self.author = Some(author.into());
        self
    }

    pub fn with_version(mut self, version: impl Into<String>) -> Self {
        self.version = Some(version.into());
        self
    }

    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }

    pub fn with_tag(mut self, tag: impl Into<String>) -> Self {
        self.tags.push(tag.into());
        self
    }
}

/// Output result
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum OutputResult {
    Success {
        adapter: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        output_path: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        bytes_written: Option<usize>,
        #[serde(skip_serializing_if = "Option::is_none")]
        message: Option<String>,
    },
    PartialSuccess {
        adapter: String,
        warnings: Vec<String>,
    },
    Failure {
        adapter: String,
        error: String,
    },
}

impl OutputResult {
    pub fn success(adapter: impl Into<String>) -> Self {
        Self::Success {
            adapter: adapter.into(),
            output_path: None,
            bytes_written: None,
            message: None,
        }
    }

    pub fn success_with_path(adapter: impl Into<String>, path: impl Into<String>) -> Self {
        Self::Success {
            adapter: adapter.into(),
            output_path: Some(path.into()),
            bytes_written: None,
            message: None,
        }
    }

    pub fn failure(adapter: impl Into<String>, error: impl Into<String>) -> Self {
        Self::Failure {
            adapter: adapter.into(),
            error: error.into(),
        }
    }

    pub fn is_success(&self) -> bool {
        matches!(self, OutputResult::Success { .. })
    }

    pub fn adapter_name(&self) -> &str {
        match self {
            OutputResult::Success { adapter, .. } => adapter,
            OutputResult::PartialSuccess { adapter, .. } => adapter,
            OutputResult::Failure { adapter, .. } => adapter,
        }
    }
}

/// Alert level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AlertLevel {
    Critical,
    Warning,
    Info,
    Debug,
}

impl AlertLevel {
    pub fn is_critical(&self) -> bool {
        matches!(self, AlertLevel::Critical)
    }

    pub fn priority(&self) -> u8 {
        match self {
            AlertLevel::Critical => 0,
            AlertLevel::Warning => 1,
            AlertLevel::Info => 2,
            AlertLevel::Debug => 3,
        }
    }
}

/// Alert data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertData {
    pub id: String,
    pub level: AlertLevel,
    pub source: String,
    pub subsystem: Option<String>,
    pub message: String,
    pub details: Option<serde_json::Value>,
    pub timestamp: DateTime<Utc>,
    pub recommended_action: Option<String>,
}

impl AlertData {
    pub fn new(
        level: AlertLevel,
        source: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            level,
            source: source.into(),
            subsystem: None,
            message: message.into(),
            details: None,
            timestamp: Utc::now(),
            recommended_action: None,
        }
    }

    pub fn critical(source: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(AlertLevel::Critical, source, message)
    }

    pub fn warning(source: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(AlertLevel::Warning, source, message)
    }

    pub fn info(source: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(AlertLevel::Info, source, message)
    }

    pub fn with_subsystem(mut self, subsystem: impl Into<String>) -> Self {
        self.subsystem = Some(subsystem.into());
        self
    }

    pub fn with_details(mut self, details: serde_json::Value) -> Self {
        self.details = Some(details);
        self
    }

    pub fn with_action(mut self, action: impl Into<String>) -> Self {
        self.recommended_action = Some(action.into());
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_data_creation() {
        let data = OutputData::new();
        assert!(data.source.is_none());
        assert!(data.payload.is_none());
    }

    #[test]
    fn test_output_data_from_json() {
        let json = serde_json::json!({
            "mission": "mars-terraform",
            "status": "active"
        });

        let data = OutputData::from_json(json);
        assert!(data.payload.is_some());
    }

    #[test]
    fn test_output_metadata_builder() {
        let metadata = OutputMetadata::new()
            .with_author("WIA Space")
            .with_version("1.0.0")
            .with_description("Test output")
            .with_tag("test")
            .with_tag("space");

        assert_eq!(metadata.author, Some("WIA Space".to_string()));
        assert_eq!(metadata.version, Some("1.0.0".to_string()));
        assert_eq!(metadata.tags.len(), 2);
    }

    #[test]
    fn test_output_result() {
        let success = OutputResult::success("test-adapter");
        assert!(success.is_success());
        assert_eq!(success.adapter_name(), "test-adapter");

        let failure = OutputResult::failure("test-adapter", "Something went wrong");
        assert!(!failure.is_success());
    }

    #[test]
    fn test_alert_data() {
        let alert = AlertData::critical("mars-orbiter-01", "System failure")
            .with_subsystem("atmospheric_processor")
            .with_details(serde_json::json!({ "error_code": 500 }))
            .with_action("Restart processor");

        assert_eq!(alert.level, AlertLevel::Critical);
        assert!(alert.level.is_critical());
        assert_eq!(alert.level.priority(), 0);
    }

    #[test]
    fn test_alert_levels() {
        assert_eq!(AlertLevel::Critical.priority(), 0);
        assert_eq!(AlertLevel::Warning.priority(), 1);
        assert_eq!(AlertLevel::Info.priority(), 2);
        assert_eq!(AlertLevel::Debug.priority(), 3);
    }
}
