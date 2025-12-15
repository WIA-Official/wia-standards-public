//! Integration adapter traits and types

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Integration type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationType {
    /// EPICS Channel Access
    Epics,
    /// TANGO Controls
    Tango,
    /// HDF5 File Archive
    Hdf5,
    /// InfluxDB Time-Series
    InfluxDb,
    /// Grafana Dashboard
    Grafana,
    /// ROOT File Format
    Root,
    /// OPC UA
    OpcUa,
    /// Custom integration
    Custom,
}

impl std::fmt::Display for IntegrationType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Epics => write!(f, "EPICS"),
            Self::Tango => write!(f, "TANGO"),
            Self::Hdf5 => write!(f, "HDF5"),
            Self::InfluxDb => write!(f, "InfluxDB"),
            Self::Grafana => write!(f, "Grafana"),
            Self::Root => write!(f, "ROOT"),
            Self::OpcUa => write!(f, "OPC UA"),
            Self::Custom => write!(f, "Custom"),
        }
    }
}

/// Integration adapter state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationState {
    /// Not connected
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Connected and ready
    Connected,
    /// Export in progress
    Exporting,
    /// Error state
    Error,
}

/// Integration options
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IntegrationOptions {
    /// Connection endpoint (URL, path, etc.)
    pub endpoint: Option<String>,

    /// Authentication credentials
    pub credentials: Option<Credentials>,

    /// Timeout in milliseconds
    pub timeout_ms: Option<u64>,

    /// Additional options
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

/// Authentication credentials
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Credentials {
    pub username: Option<String>,
    pub password: Option<String>,
    pub token: Option<String>,
}

/// Physics data type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicsDataType {
    Fusion,
    Particle,
    DarkMatter,
    Antimatter,
    QuantumGravity,
    TimeCrystal,
}

/// Data metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DataMetadata {
    /// Unique identifier
    pub id: String,

    /// Experiment name
    pub experiment: Option<String>,

    /// Creation timestamp (ISO 8601)
    pub created: Option<String>,

    /// WIA schema version
    pub schema_version: Option<String>,

    /// Additional metadata
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

/// Unified physics data for export
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsData {
    /// Data type
    pub data_type: PhysicsDataType,

    /// Metadata
    pub metadata: DataMetadata,

    /// Payload (serialized physics data)
    pub payload: serde_json::Value,
}

/// Export result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportResult {
    /// Success status
    pub success: bool,

    /// Number of records exported
    pub records_exported: u64,

    /// Destination (file path, URL, etc.)
    pub destination: String,

    /// Timestamp (milliseconds since epoch)
    pub timestamp: i64,

    /// Additional details
    pub details: Option<String>,
}

/// Integration error
#[derive(Debug, Clone)]
pub enum IntegrationError {
    /// Adapter not found
    AdapterNotFound(IntegrationType),

    /// Adapter not available
    AdapterNotAvailable(IntegrationType),

    /// Not connected
    NotConnected,

    /// Connection failed
    ConnectionFailed(String),

    /// Export failed
    ExportFailed(String),

    /// Import failed
    ImportFailed(String),

    /// Not supported
    NotSupported(String),

    /// Invalid configuration
    InvalidConfig(String),

    /// IO error
    IoError(String),

    /// Timeout
    Timeout,

    /// Authentication failed
    AuthFailed,
}

impl std::fmt::Display for IntegrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AdapterNotFound(t) => write!(f, "Adapter not found: {}", t),
            Self::AdapterNotAvailable(t) => write!(f, "Adapter not available: {}", t),
            Self::NotConnected => write!(f, "Not connected"),
            Self::ConnectionFailed(s) => write!(f, "Connection failed: {}", s),
            Self::ExportFailed(s) => write!(f, "Export failed: {}", s),
            Self::ImportFailed(s) => write!(f, "Import failed: {}", s),
            Self::NotSupported(s) => write!(f, "Not supported: {}", s),
            Self::InvalidConfig(s) => write!(f, "Invalid configuration: {}", s),
            Self::IoError(s) => write!(f, "IO error: {}", s),
            Self::Timeout => write!(f, "Operation timed out"),
            Self::AuthFailed => write!(f, "Authentication failed"),
        }
    }
}

impl std::error::Error for IntegrationError {}

/// Integration adapter trait
pub trait IntegrationAdapter: Send + Sync {
    /// Get integration type
    fn integration_type(&self) -> IntegrationType;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Get current state
    fn state(&self) -> IntegrationState;

    /// Initialize adapter
    fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError>;

    /// Connect to external system
    fn connect(&mut self) -> Result<(), IntegrationError>;

    /// Disconnect from external system
    fn disconnect(&mut self) -> Result<(), IntegrationError>;

    /// Export physics data
    fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError>;

    /// Import physics data (if supported)
    fn import(&self, source: &str) -> Result<PhysicsData, IntegrationError>;

    /// Check if adapter is available
    fn is_available(&self) -> bool;

    /// Dispose resources
    fn dispose(&mut self) -> Result<(), IntegrationError>;
}

/// Integration event types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrationEvent {
    /// Adapter registered
    AdapterRegistered,
    /// Adapter unregistered
    AdapterUnregistered,
    /// Connection established
    Connected,
    /// Connection lost
    Disconnected,
    /// Export started
    ExportStarted,
    /// Export completed
    ExportCompleted,
    /// Export failed
    ExportFailed,
    /// Import started
    ImportStarted,
    /// Import completed
    ImportCompleted,
    /// Import failed
    ImportFailed,
}

/// Integration event data
#[derive(Debug, Clone)]
pub struct IntegrationEventData {
    /// Event type
    pub event: IntegrationEvent,

    /// Integration type
    pub integration_type: IntegrationType,

    /// Timestamp (milliseconds)
    pub timestamp: i64,

    /// Details
    pub details: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_integration_type_display() {
        assert_eq!(IntegrationType::Epics.to_string(), "EPICS");
        assert_eq!(IntegrationType::Hdf5.to_string(), "HDF5");
        assert_eq!(IntegrationType::InfluxDb.to_string(), "InfluxDB");
    }

    #[test]
    fn test_integration_error_display() {
        let err = IntegrationError::NotConnected;
        assert_eq!(err.to_string(), "Not connected");

        let err = IntegrationError::ConnectionFailed("timeout".to_string());
        assert_eq!(err.to_string(), "Connection failed: timeout");
    }

    #[test]
    fn test_physics_data_serialization() {
        let data = PhysicsData {
            data_type: PhysicsDataType::Fusion,
            metadata: DataMetadata {
                id: "test-001".to_string(),
                experiment: Some("ITER".to_string()),
                ..Default::default()
            },
            payload: serde_json::json!({"plasma": {"temperature": 150000000.0}}),
        };

        let json = serde_json::to_string(&data).unwrap();
        assert!(json.contains("fusion"));
        assert!(json.contains("test-001"));
    }
}
