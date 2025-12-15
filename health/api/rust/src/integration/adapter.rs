//! Integration Adapter Traits
//!
//! Base traits and types for integration adapters

use async_trait::async_trait;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::Duration;

use crate::error::Result;
use crate::types::HealthProfile;

/// Adapter type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdapterType {
    /// FHIR healthcare interoperability
    Fhir,
    /// Apple HealthKit
    HealthKit,
    /// Google Health Connect
    HealthConnect,
    /// Fitbit API
    Fitbit,
    /// Garmin Connect
    Garmin,
    /// Real-time dashboard
    Dashboard,
    /// Digital twin visualizer
    DigitalTwin,
    /// Data export
    Export,
    /// Custom adapter
    Custom,
}

impl std::fmt::Display for AdapterType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AdapterType::Fhir => write!(f, "FHIR"),
            AdapterType::HealthKit => write!(f, "HealthKit"),
            AdapterType::HealthConnect => write!(f, "Health Connect"),
            AdapterType::Fitbit => write!(f, "Fitbit"),
            AdapterType::Garmin => write!(f, "Garmin"),
            AdapterType::Dashboard => write!(f, "Dashboard"),
            AdapterType::DigitalTwin => write!(f, "Digital Twin"),
            AdapterType::Export => write!(f, "Export"),
            AdapterType::Custom => write!(f, "Custom"),
        }
    }
}

/// Adapter configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AdapterConfig {
    /// Base URL for API endpoints
    pub base_url: Option<String>,
    /// Authentication token
    pub auth_token: Option<String>,
    /// API key
    pub api_key: Option<String>,
    /// Request timeout
    pub timeout: Option<Duration>,
    /// Additional configuration
    pub extra: HashMap<String, serde_json::Value>,
}

impl AdapterConfig {
    /// Create new empty config
    pub fn new() -> Self {
        Self::default()
    }

    /// Set base URL
    pub fn with_url(mut self, url: impl Into<String>) -> Self {
        self.base_url = Some(url.into());
        self
    }

    /// Set auth token
    pub fn with_auth(mut self, token: impl Into<String>) -> Self {
        self.auth_token = Some(token.into());
        self
    }

    /// Set API key
    pub fn with_api_key(mut self, key: impl Into<String>) -> Self {
        self.api_key = Some(key.into());
        self
    }

    /// Set timeout
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }
}

/// Adapter capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdapterCapabilities {
    /// Can import data from external source
    pub can_import: bool,
    /// Can export data to external system
    pub can_export: bool,
    /// Supports real-time streaming
    pub can_stream: bool,
    /// Supports authentication
    pub supports_auth: bool,
    /// Supported data types
    pub data_types: Vec<String>,
}

impl Default for AdapterCapabilities {
    fn default() -> Self {
        Self {
            can_import: false,
            can_export: false,
            can_stream: false,
            supports_auth: true,
            data_types: Vec::new(),
        }
    }
}

/// Base integration adapter trait
#[async_trait]
pub trait IntegrationAdapter: Send + Sync {
    /// Get adapter type
    fn adapter_type(&self) -> AdapterType;

    /// Get human-readable name
    fn name(&self) -> &str;

    /// Check if adapter is available
    async fn is_available(&self) -> bool;

    /// Initialize the adapter
    async fn initialize(&mut self, config: AdapterConfig) -> Result<()>;

    /// Shutdown the adapter
    async fn shutdown(&mut self) -> Result<()>;

    /// Get adapter capabilities
    fn capabilities(&self) -> AdapterCapabilities;
}

/// Import options
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ImportOptions {
    /// Data types to import
    pub data_types: Vec<String>,
    /// Start date filter
    pub start_date: Option<DateTime<Utc>>,
    /// End date filter
    pub end_date: Option<DateTime<Utc>>,
    /// Maximum records to import
    pub limit: Option<u32>,
    /// Include historical data
    pub include_history: bool,
}

/// Import result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportResult {
    /// Number of records imported
    pub records_imported: u64,
    /// Data types that were imported
    pub data_types: Vec<String>,
    /// Updated profile (if applicable)
    pub profile_updates: Option<HealthProfile>,
    /// Import errors
    pub errors: Vec<ImportError>,
    /// Import timestamp
    pub imported_at: DateTime<Utc>,
}

/// Import error
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportError {
    /// Error code
    pub code: String,
    /// Error message
    pub message: String,
    /// Affected data type
    pub data_type: Option<String>,
    /// Recoverable
    pub recoverable: bool,
}

/// Stream handle for managing active streams
#[derive(Debug, Clone)]
pub struct StreamHandle {
    /// Unique stream ID
    pub id: String,
    /// Stream type
    pub stream_type: String,
    /// Created timestamp
    pub created_at: DateTime<Utc>,
}

/// Import event for streaming
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportEvent {
    /// Event type
    pub event_type: ImportEventType,
    /// Event data
    pub data: serde_json::Value,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

/// Import event types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImportEventType {
    /// New data received
    Data,
    /// Import progress update
    Progress,
    /// Error occurred
    Error,
    /// Import completed
    Complete,
}

/// Import adapter trait
#[async_trait]
pub trait ImportAdapter: IntegrationAdapter {
    /// Import data from external source
    async fn import(&self, options: ImportOptions) -> Result<ImportResult>;

    /// Start continuous import stream
    async fn start_import_stream(
        &self,
        callback: Box<dyn Fn(ImportEvent) + Send + Sync>,
    ) -> Result<StreamHandle>;

    /// Stop import stream
    async fn stop_import_stream(&self, handle: StreamHandle) -> Result<()>;
}

/// Export options
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ExportOptions {
    /// Export format
    pub format: ExportFormat,
    /// Include metadata
    pub include_metadata: bool,
    /// Anonymize data
    pub anonymize: bool,
    /// Destination URL or path
    pub destination: Option<String>,
}

/// Export format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExportFormat {
    /// FHIR JSON format
    #[default]
    FhirJson,
    /// FHIR XML format
    FhirXml,
    /// CSV format
    Csv,
    /// Plain JSON
    Json,
    /// PDF report
    Pdf,
    /// HL7 v2 format
    Hl7v2,
}

/// Export result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportResult {
    /// Export was successful
    pub success: bool,
    /// Number of records exported
    pub records_exported: u64,
    /// Export destination
    pub destination: String,
    /// Resource IDs (for FHIR)
    pub resource_ids: Vec<String>,
    /// Export timestamp
    pub exported_at: DateTime<Utc>,
}

/// Export data container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportData {
    /// Data type
    pub data_type: String,
    /// Data content
    pub data: serde_json::Value,
}

/// Export adapter trait
#[async_trait]
pub trait ExportAdapter: IntegrationAdapter {
    /// Export profile to external system
    async fn export(&self, profile: &HealthProfile, options: ExportOptions) -> Result<ExportResult>;

    /// Export specific data
    async fn export_data(&self, data: ExportData, options: ExportOptions) -> Result<ExportResult>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adapter_config() {
        let config = AdapterConfig::new()
            .with_url("https://api.example.com")
            .with_auth("Bearer token123")
            .with_timeout(Duration::from_secs(30));

        assert_eq!(config.base_url, Some("https://api.example.com".to_string()));
        assert_eq!(config.auth_token, Some("Bearer token123".to_string()));
        assert_eq!(config.timeout, Some(Duration::from_secs(30)));
    }

    #[test]
    fn test_adapter_type_display() {
        assert_eq!(AdapterType::Fhir.to_string(), "FHIR");
        assert_eq!(AdapterType::HealthKit.to_string(), "HealthKit");
        assert_eq!(AdapterType::Dashboard.to_string(), "Dashboard");
    }
}
