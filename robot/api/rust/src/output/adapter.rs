//! Output adapter trait and types

use crate::error::{RobotError, RobotResult};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Output type enumeration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputType {
    /// 3D visualization (RViz, Gazebo, Unity)
    Visualization,
    /// Data export (JSON, CSV, URDF)
    Export,
    /// Medical systems (HL7 FHIR, DICOM)
    Medical,
    /// Dashboard streaming (WebSocket)
    Dashboard,
    /// AI/ML dataset generation
    AiMl,
    /// Alert notifications (Webhook, Email)
    Alert,
    /// Logging systems
    Logger,
    /// Custom output type
    Custom(String),
}

impl std::fmt::Display for OutputType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OutputType::Visualization => write!(f, "visualization"),
            OutputType::Export => write!(f, "export"),
            OutputType::Medical => write!(f, "medical"),
            OutputType::Dashboard => write!(f, "dashboard"),
            OutputType::AiMl => write!(f, "aiml"),
            OutputType::Alert => write!(f, "alert"),
            OutputType::Logger => write!(f, "logger"),
            OutputType::Custom(name) => write!(f, "custom:{}", name),
        }
    }
}

/// Output adapter configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OutputConfig {
    /// Endpoint URL or file path
    pub endpoint: Option<String>,
    /// Output format
    pub format: String,
    /// Additional options
    pub options: HashMap<String, serde_json::Value>,
}

impl OutputConfig {
    /// Create a new output configuration
    pub fn new(format: &str) -> Self {
        Self {
            endpoint: None,
            format: format.to_string(),
            options: HashMap::new(),
        }
    }

    /// Set endpoint
    pub fn with_endpoint(mut self, endpoint: &str) -> Self {
        self.endpoint = Some(endpoint.to_string());
        self
    }

    /// Add option
    pub fn with_option(mut self, key: &str, value: serde_json::Value) -> Self {
        self.options.insert(key.to_string(), value);
        self
    }

    /// Get option value
    pub fn get_option(&self, key: &str) -> Option<&serde_json::Value> {
        self.options.get(key)
    }

    /// Get string option
    pub fn get_string_option(&self, key: &str) -> Option<&str> {
        self.options.get(key).and_then(|v| v.as_str())
    }

    /// Get bool option
    pub fn get_bool_option(&self, key: &str) -> Option<bool> {
        self.options.get(key).and_then(|v| v.as_bool())
    }
}

/// Data to be output
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputData {
    /// Device identifier
    pub device_id: String,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Robot type (exoskeleton, prosthetic, etc.)
    pub robot_type: String,
    /// Actual data payload
    pub data: serde_json::Value,
    /// Optional metadata
    pub metadata: Option<serde_json::Value>,
}

impl OutputData {
    /// Create new output data
    pub fn new(device_id: &str, robot_type: &str) -> Self {
        Self {
            device_id: device_id.to_string(),
            timestamp: Utc::now(),
            robot_type: robot_type.to_string(),
            data: serde_json::Value::Null,
            metadata: None,
        }
    }

    /// Set data payload
    pub fn with_data(mut self, data: serde_json::Value) -> Self {
        self.data = data;
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: serde_json::Value) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp: DateTime<Utc>) -> Self {
        self.timestamp = timestamp;
        self
    }
}

/// Result of output operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputResult {
    /// Whether the operation succeeded
    pub success: bool,
    /// Result message
    pub message: String,
    /// Optional result metadata
    pub metadata: Option<serde_json::Value>,
    /// Processing duration in milliseconds
    pub duration_ms: u64,
}

impl OutputResult {
    /// Create a success result
    pub fn success(message: &str) -> Self {
        Self {
            success: true,
            message: message.to_string(),
            metadata: None,
            duration_ms: 0,
        }
    }

    /// Create a failure result
    pub fn failure(message: &str) -> Self {
        Self {
            success: false,
            message: message.to_string(),
            metadata: None,
            duration_ms: 0,
        }
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: serde_json::Value) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration_ms: u64) -> Self {
        self.duration_ms = duration_ms;
        self
    }
}

/// Output adapter trait
///
/// All output adapters must implement this trait to integrate with the OutputManager.
pub trait OutputAdapter: Send + Sync {
    /// Get the output type
    fn output_type(&self) -> OutputType;

    /// Get the adapter name
    fn name(&self) -> &str;

    /// Initialize the adapter with configuration
    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()>;

    /// Output data
    fn output(&self, data: &OutputData) -> RobotResult<OutputResult>;

    /// Check if the adapter is available
    fn is_available(&self) -> bool;

    /// Dispose resources
    fn dispose(&mut self) -> RobotResult<()>;
}

/// Base adapter implementation helper
#[derive(Debug, Clone)]
pub struct BaseAdapter {
    /// Adapter name
    pub name: String,
    /// Output type
    pub output_type: OutputType,
    /// Configuration
    pub config: Option<OutputConfig>,
    /// Availability status
    pub available: bool,
}

impl BaseAdapter {
    /// Create a new base adapter
    pub fn new(name: &str, output_type: OutputType) -> Self {
        Self {
            name: name.to_string(),
            output_type,
            config: None,
            available: true,
        }
    }

    /// Set configuration
    pub fn set_config(&mut self, config: OutputConfig) {
        self.config = Some(config);
    }

    /// Set availability
    pub fn set_available(&mut self, available: bool) {
        self.available = available;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_type_display() {
        assert_eq!(OutputType::Visualization.to_string(), "visualization");
        assert_eq!(OutputType::Medical.to_string(), "medical");
        assert_eq!(OutputType::Custom("test".to_string()).to_string(), "custom:test");
    }

    #[test]
    fn test_output_config() {
        let config = OutputConfig::new("json")
            .with_endpoint("./output")
            .with_option("pretty", serde_json::json!(true));

        assert_eq!(config.format, "json");
        assert_eq!(config.endpoint, Some("./output".to_string()));
        assert_eq!(config.get_bool_option("pretty"), Some(true));
    }

    #[test]
    fn test_output_data() {
        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({"status": "active"}))
            .with_metadata(serde_json::json!({"source": "test"}));

        assert_eq!(data.device_id, "exo-001");
        assert_eq!(data.robot_type, "exoskeleton");
        assert!(data.metadata.is_some());
    }

    #[test]
    fn test_output_result() {
        let result = OutputResult::success("Data exported")
            .with_metadata(serde_json::json!({"file": "output.json"}))
            .with_duration(150);

        assert!(result.success);
        assert_eq!(result.duration_ms, 150);
    }

    #[test]
    fn test_base_adapter() {
        let mut adapter = BaseAdapter::new("test-adapter", OutputType::Export);
        assert_eq!(adapter.name, "test-adapter");
        assert!(adapter.available);

        adapter.set_available(false);
        assert!(!adapter.available);
    }
}
