//! Output adapter trait and types

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::data::{OutputData, OutputResult};

/// Output type enumeration
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputType {
    Visualization,
    Export,
    Dashboard,
    Alert,
    Logger,
    Custom(String),
}

impl OutputType {
    /// Check if this is a visual output type
    pub fn is_visual(&self) -> bool {
        matches!(self, OutputType::Visualization | OutputType::Dashboard)
    }

    /// Check if this is an export output type
    pub fn is_export(&self) -> bool {
        matches!(self, OutputType::Export)
    }

    /// Check if this is a notification type
    pub fn is_notification(&self) -> bool {
        matches!(self, OutputType::Alert)
    }
}

/// Output format enumeration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputFormat {
    // WIA formats
    WiaJson,
    WiaCompact,

    // CCSDS formats
    CcsdsOem,
    CcsdsOpm,
    CcsdsOmm,

    // NASA formats
    GmatScript,

    // Generic formats
    Json,
    JsonPretty,
    Csv,
    Xml,

    // Visualization formats
    Czml,
    GeoJson,
    Kml,
}

impl OutputFormat {
    /// Get file extension for this format
    pub fn extension(&self) -> &'static str {
        match self {
            OutputFormat::WiaJson | OutputFormat::Json | OutputFormat::JsonPretty => "json",
            OutputFormat::WiaCompact => "wia.json",
            OutputFormat::CcsdsOem => "oem",
            OutputFormat::CcsdsOpm => "opm",
            OutputFormat::CcsdsOmm => "omm",
            OutputFormat::GmatScript => "script",
            OutputFormat::Csv => "csv",
            OutputFormat::Xml => "xml",
            OutputFormat::Czml => "czml",
            OutputFormat::GeoJson => "geojson",
            OutputFormat::Kml => "kml",
        }
    }

    /// Get MIME type for this format
    pub fn mime_type(&self) -> &'static str {
        match self {
            OutputFormat::WiaJson
            | OutputFormat::WiaCompact
            | OutputFormat::Json
            | OutputFormat::JsonPretty
            | OutputFormat::Czml
            | OutputFormat::GeoJson => "application/json",
            OutputFormat::Csv => "text/csv",
            OutputFormat::Xml
            | OutputFormat::CcsdsOem
            | OutputFormat::CcsdsOpm
            | OutputFormat::CcsdsOmm
            | OutputFormat::Kml => "application/xml",
            OutputFormat::GmatScript => "text/plain",
        }
    }
}

/// Output configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OutputConfig {
    /// Output format
    pub format: Option<OutputFormat>,

    /// Target path or URL
    pub target: Option<String>,

    /// Include metadata in output
    #[serde(default)]
    pub include_metadata: bool,

    /// Compress output
    #[serde(default)]
    pub compress: bool,

    /// Pretty print (for JSON)
    #[serde(default)]
    pub pretty: bool,

    /// Additional options
    #[serde(default)]
    pub options: HashMap<String, serde_json::Value>,
}

impl OutputConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_format(mut self, format: OutputFormat) -> Self {
        self.format = Some(format);
        self
    }

    pub fn with_target(mut self, target: impl Into<String>) -> Self {
        self.target = Some(target.into());
        self
    }

    pub fn with_metadata(mut self) -> Self {
        self.include_metadata = true;
        self
    }

    pub fn with_compression(mut self) -> Self {
        self.compress = true;
        self
    }

    pub fn pretty(mut self) -> Self {
        self.pretty = true;
        self
    }
}

/// Output adapter error
#[derive(Debug, thiserror::Error)]
pub enum OutputError {
    #[error("Adapter not available: {0}")]
    NotAvailable(String),

    #[error("Output failed: {0}")]
    OutputFailed(String),

    #[error("Format error: {0}")]
    FormatError(String),

    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    #[error("Configuration error: {0}")]
    Config(String),
}

/// Output adapter trait
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Get output type
    fn output_type(&self) -> OutputType;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Initialize adapter
    async fn initialize(&mut self, config: &OutputConfig) -> Result<(), OutputError>;

    /// Output data
    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError>;

    /// Check availability
    fn is_available(&self) -> bool;

    /// Dispose adapter
    async fn dispose(&mut self) -> Result<(), OutputError>;
}

/// Mock output adapter for testing
pub struct MockOutputAdapter {
    name: String,
    output_type: OutputType,
    available: bool,
    outputs: std::sync::Arc<std::sync::Mutex<Vec<OutputData>>>,
}

impl MockOutputAdapter {
    pub fn new(name: impl Into<String>, output_type: OutputType) -> Self {
        Self {
            name: name.into(),
            output_type,
            available: true,
            outputs: std::sync::Arc::new(std::sync::Mutex::new(Vec::new())),
        }
    }

    pub fn visualization(name: impl Into<String>) -> Self {
        Self::new(name, OutputType::Visualization)
    }

    pub fn exporter(name: impl Into<String>) -> Self {
        Self::new(name, OutputType::Export)
    }

    pub fn set_available(&mut self, available: bool) {
        self.available = available;
    }

    pub fn get_outputs(&self) -> Vec<OutputData> {
        self.outputs.lock().unwrap().clone()
    }

    pub fn clear(&self) {
        self.outputs.lock().unwrap().clear();
    }
}

#[async_trait]
impl OutputAdapter for MockOutputAdapter {
    fn output_type(&self) -> OutputType {
        self.output_type.clone()
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &OutputConfig) -> Result<(), OutputError> {
        Ok(())
    }

    async fn output(&self, data: &OutputData) -> Result<OutputResult, OutputError> {
        if !self.available {
            return Err(OutputError::NotAvailable(self.name.clone()));
        }

        self.outputs.lock().unwrap().push(data.clone());

        Ok(OutputResult::Success {
            adapter: self.name.clone(),
            output_path: None,
            bytes_written: None,
            message: Some("Mock output successful".to_string()),
        })
    }

    fn is_available(&self) -> bool {
        self.available
    }

    async fn dispose(&mut self) -> Result<(), OutputError> {
        self.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_format_extension() {
        assert_eq!(OutputFormat::Json.extension(), "json");
        assert_eq!(OutputFormat::CcsdsOem.extension(), "oem");
        assert_eq!(OutputFormat::Csv.extension(), "csv");
    }

    #[test]
    fn test_output_format_mime() {
        assert_eq!(OutputFormat::Json.mime_type(), "application/json");
        assert_eq!(OutputFormat::Csv.mime_type(), "text/csv");
    }

    #[test]
    fn test_output_config_builder() {
        let config = OutputConfig::new()
            .with_format(OutputFormat::Json)
            .with_target("/output/data.json")
            .with_metadata()
            .pretty();

        assert_eq!(config.format, Some(OutputFormat::Json));
        assert_eq!(config.target, Some("/output/data.json".to_string()));
        assert!(config.include_metadata);
        assert!(config.pretty);
    }

    #[tokio::test]
    async fn test_mock_adapter() {
        let mut adapter = MockOutputAdapter::visualization("test-vis");

        assert!(adapter.is_available());
        assert_eq!(adapter.name(), "test-vis");
        assert_eq!(adapter.output_type(), OutputType::Visualization);

        adapter.initialize(&OutputConfig::new()).await.unwrap();

        let data = OutputData::default();
        let result = adapter.output(&data).await.unwrap();

        match result {
            OutputResult::Success { adapter: name, .. } => {
                assert_eq!(name, "test-vis");
            }
            _ => panic!("Expected success"),
        }

        assert_eq!(adapter.get_outputs().len(), 1);
    }

    #[tokio::test]
    async fn test_mock_adapter_unavailable() {
        let mut adapter = MockOutputAdapter::visualization("test");
        adapter.set_available(false);

        let data = OutputData::default();
        let result = adapter.output(&data).await;

        assert!(matches!(result, Err(OutputError::NotAvailable(_))));
    }
}
