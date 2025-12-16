//! JSON data exporter

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use chrono::Utc;
use serde::{Deserialize, Serialize};

/// JSON export structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonExport {
    /// Export metadata
    pub export_info: ExportInfo,
    /// Device information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device: Option<serde_json::Value>,
    /// Robot state
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state: Option<serde_json::Value>,
    /// Robot specification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spec: Option<serde_json::Value>,
    /// Safety data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety: Option<serde_json::Value>,
    /// Session data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sessions: Option<Vec<SessionInfo>>,
    /// Raw data points
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
}

/// Export metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportInfo {
    /// Export format identifier
    pub format: String,
    /// Format version
    pub version: String,
    /// Export timestamp
    pub exported_at: String,
    /// Device ID
    pub device_id: String,
    /// Robot type
    pub robot_type: String,
}

/// Session information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionInfo {
    /// Session ID
    pub session_id: String,
    /// Start time
    pub start_time: String,
    /// End time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_time: Option<String>,
    /// Number of data points
    pub data_points: usize,
}

impl JsonExport {
    /// Create a new JSON export
    pub fn new(device_id: &str, robot_type: &str) -> Self {
        Self {
            export_info: ExportInfo {
                format: "wia-robot-export".to_string(),
                version: "1.0.0".to_string(),
                exported_at: Utc::now().to_rfc3339(),
                device_id: device_id.to_string(),
                robot_type: robot_type.to_string(),
            },
            device: None,
            state: None,
            spec: None,
            safety: None,
            sessions: None,
            data: None,
        }
    }

    /// Set device data
    pub fn with_device(mut self, device: serde_json::Value) -> Self {
        self.device = Some(device);
        self
    }

    /// Set state data
    pub fn with_state(mut self, state: serde_json::Value) -> Self {
        self.state = Some(state);
        self
    }

    /// Set spec data
    pub fn with_spec(mut self, spec: serde_json::Value) -> Self {
        self.spec = Some(spec);
        self
    }

    /// Set safety data
    pub fn with_safety(mut self, safety: serde_json::Value) -> Self {
        self.safety = Some(safety);
        self
    }

    /// Set raw data
    pub fn with_data(mut self, data: serde_json::Value) -> Self {
        self.data = Some(data);
        self
    }

    /// Add session
    pub fn add_session(&mut self, session: SessionInfo) {
        if self.sessions.is_none() {
            self.sessions = Some(Vec::new());
        }
        if let Some(sessions) = &mut self.sessions {
            sessions.push(session);
        }
    }

    /// Convert to JSON string
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Convert to compact JSON string
    pub fn to_json_compact(&self) -> RobotResult<String> {
        Ok(serde_json::to_string(self)?)
    }
}

/// JSON exporter adapter
pub struct JsonExporter {
    base: BaseAdapter,
    output_dir: String,
    pretty_print: bool,
}

impl Default for JsonExporter {
    fn default() -> Self {
        Self::new("./export")
    }
}

impl JsonExporter {
    /// Create a new JSON exporter
    pub fn new(output_dir: &str) -> Self {
        Self {
            base: BaseAdapter::new("json", OutputType::Export),
            output_dir: output_dir.to_string(),
            pretty_print: true,
        }
    }

    /// Set pretty print option
    pub fn with_pretty_print(mut self, pretty: bool) -> Self {
        self.pretty_print = pretty;
        self
    }

    /// Set output directory
    pub fn with_output_dir(mut self, dir: &str) -> Self {
        self.output_dir = dir.to_string();
        self
    }

    /// Get output directory
    pub fn output_dir(&self) -> &str {
        &self.output_dir
    }

    /// Create export from output data
    pub fn create_export(&self, data: &OutputData) -> JsonExport {
        let mut export = JsonExport::new(&data.device_id, &data.robot_type)
            .with_data(data.data.clone());

        // Extract specific sections if present
        if let Some(device) = data.data.get("device") {
            export.device = Some(device.clone());
        }
        if let Some(state) = data.data.get("state") {
            export.state = Some(state.clone());
        }
        if let Some(spec) = data.data.get("spec") {
            export.spec = Some(spec.clone());
        }
        if let Some(safety) = data.data.get("safety") {
            export.safety = Some(safety.clone());
        }

        // Add metadata if present
        if let Some(metadata) = &data.metadata {
            if let Some(obj) = export.data.as_mut().and_then(|d| d.as_object_mut()) {
                obj.insert("_metadata".to_string(), metadata.clone());
            }
        }

        export
    }

    /// Generate suggested filename
    pub fn suggested_filename(&self, data: &OutputData) -> String {
        let timestamp = data.timestamp.format("%Y%m%d_%H%M%S");
        format!("{}_{}.json", data.device_id, timestamp)
    }
}

impl OutputAdapter for JsonExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Export
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(dir) = config.get_string_option("output_dir") {
            self.output_dir = dir.to_string();
        }
        if let Some(pretty) = config.get_bool_option("pretty_print") {
            self.pretty_print = pretty;
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let export = self.create_export(data);
        let json = if self.pretty_print {
            export.to_json()?
        } else {
            export.to_json_compact()?
        };

        let filename = self.suggested_filename(data);

        Ok(OutputResult::success("Exported to JSON")
            .with_metadata(serde_json::json!({
                "format": "json",
                "filename": filename,
                "size_bytes": json.len(),
                "pretty_print": self.pretty_print,
                "output": json
            })))
    }

    fn is_available(&self) -> bool {
        self.base.available
    }

    fn dispose(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_json_export_creation() {
        let export = JsonExport::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({"status": "active"}));

        assert_eq!(export.export_info.device_id, "exo-001");
        assert_eq!(export.export_info.robot_type, "exoskeleton");
        assert_eq!(export.export_info.format, "wia-robot-export");
    }

    #[test]
    fn test_json_export_with_sections() {
        let mut export = JsonExport::new("exo-001", "exoskeleton")
            .with_device(serde_json::json!({"name": "Test Exo"}))
            .with_state(serde_json::json!({"operational": true}))
            .with_safety(serde_json::json!({"level": "normal"}));

        let session = SessionInfo {
            session_id: "session-001".to_string(),
            start_time: "2025-01-15T09:00:00Z".to_string(),
            end_time: Some("2025-01-15T10:00:00Z".to_string()),
            data_points: 3600,
        };
        export.add_session(session);

        assert!(export.device.is_some());
        assert!(export.state.is_some());
        assert!(export.safety.is_some());
        assert_eq!(export.sessions.as_ref().unwrap().len(), 1);
    }

    #[test]
    fn test_json_export_to_string() {
        let export = JsonExport::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({"test": true}));

        let json = export.to_json().unwrap();
        assert!(json.contains("wia-robot-export"));
        assert!(json.contains("exo-001"));
    }

    #[test]
    fn test_json_exporter() {
        let exporter = JsonExporter::new("./test_export")
            .with_pretty_print(true);

        assert_eq!(exporter.output_type(), OutputType::Export);
        assert_eq!(exporter.name(), "json");
        assert_eq!(exporter.output_dir(), "./test_export");
    }

    #[test]
    fn test_json_exporter_output() {
        let exporter = JsonExporter::new("./test_export");

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active",
                "joints": [{"name": "hip", "angle": 15.0}]
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);

        let metadata = result.metadata.unwrap();
        assert_eq!(metadata["format"], "json");
    }

    #[test]
    fn test_suggested_filename() {
        let exporter = JsonExporter::new("./export");
        let data = OutputData::new("exo-001", "exoskeleton");

        let filename = exporter.suggested_filename(&data);
        assert!(filename.starts_with("exo-001_"));
        assert!(filename.ends_with(".json"));
    }
}
