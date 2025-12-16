//! CSV data exporter

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use chrono::Utc;

/// CSV column definition
#[derive(Debug, Clone)]
pub struct CsvColumn {
    /// Column name
    pub name: String,
    /// JSON path to extract value
    pub json_path: String,
    /// Default value if missing
    pub default: String,
}

impl CsvColumn {
    /// Create a new column definition
    pub fn new(name: &str, json_path: &str) -> Self {
        Self {
            name: name.to_string(),
            json_path: json_path.to_string(),
            default: String::new(),
        }
    }

    /// Set default value
    pub fn with_default(mut self, default: &str) -> Self {
        self.default = default.to_string();
        self
    }

    /// Extract value from JSON
    pub fn extract(&self, data: &serde_json::Value) -> String {
        let parts: Vec<&str> = self.json_path.split('.').collect();
        let mut current = data;

        for part in parts {
            // Try to parse as array index first
            if let Ok(index) = part.parse::<usize>() {
                match current.get(index) {
                    Some(v) => current = v,
                    None => return self.default.clone(),
                }
            } else {
                // Try as object key
                match current.get(part) {
                    Some(v) => current = v,
                    None => return self.default.clone(),
                }
            }
        }

        match current {
            serde_json::Value::Null => self.default.clone(),
            serde_json::Value::Bool(b) => b.to_string(),
            serde_json::Value::Number(n) => n.to_string(),
            serde_json::Value::String(s) => s.clone(),
            _ => current.to_string(),
        }
    }
}

/// CSV row data
#[derive(Debug, Clone)]
pub struct CsvRow {
    pub values: Vec<String>,
}

impl CsvRow {
    /// Create a new CSV row
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }

    /// Add value to row
    pub fn push(&mut self, value: String) {
        self.values.push(value);
    }

    /// Convert to CSV string
    pub fn to_csv(&self) -> String {
        self.values
            .iter()
            .map(|v| {
                if v.contains(',') || v.contains('"') || v.contains('\n') {
                    format!("\"{}\"", v.replace('"', "\"\""))
                } else {
                    v.clone()
                }
            })
            .collect::<Vec<_>>()
            .join(",")
    }
}

impl Default for CsvRow {
    fn default() -> Self {
        Self::new()
    }
}

/// CSV exporter adapter
pub struct CsvExporter {
    base: BaseAdapter,
    output_dir: String,
    columns: Vec<CsvColumn>,
    include_header: bool,
    include_timestamp: bool,
}

impl Default for CsvExporter {
    fn default() -> Self {
        Self::new("./export")
    }
}

impl CsvExporter {
    /// Create a new CSV exporter
    pub fn new(output_dir: &str) -> Self {
        Self {
            base: BaseAdapter::new("csv", OutputType::Export),
            output_dir: output_dir.to_string(),
            columns: Vec::new(),
            include_header: true,
            include_timestamp: true,
        }
    }

    /// Add column definition
    pub fn add_column(mut self, column: CsvColumn) -> Self {
        self.columns.push(column);
        self
    }

    /// Set include header option
    pub fn with_header(mut self, include: bool) -> Self {
        self.include_header = include;
        self
    }

    /// Set include timestamp option
    pub fn with_timestamp(mut self, include: bool) -> Self {
        self.include_timestamp = include;
        self
    }

    /// Use default columns for exoskeleton data
    pub fn with_exoskeleton_columns(mut self) -> Self {
        self.columns = vec![
            CsvColumn::new("device_id", "device_id"),
            CsvColumn::new("status", "status"),
            CsvColumn::new("assist_level", "assist_level").with_default("0.0"),
            CsvColumn::new("hip_left_angle", "joints.0.angle_deg").with_default("0.0"),
            CsvColumn::new("hip_right_angle", "joints.1.angle_deg").with_default("0.0"),
            CsvColumn::new("knee_left_angle", "joints.2.angle_deg").with_default("0.0"),
            CsvColumn::new("knee_right_angle", "joints.3.angle_deg").with_default("0.0"),
            CsvColumn::new("gait_phase", "gait.phase").with_default("unknown"),
            CsvColumn::new("velocity_m_s", "gait.velocity_m_s").with_default("0.0"),
            CsvColumn::new("safety_level", "safety.level").with_default("unknown"),
        ];
        self
    }

    /// Use default columns for prosthetic data
    pub fn with_prosthetic_columns(mut self) -> Self {
        self.columns = vec![
            CsvColumn::new("device_id", "device_id"),
            CsvColumn::new("status", "status"),
            CsvColumn::new("grip_force_n", "grip.force_n").with_default("0.0"),
            CsvColumn::new("grip_aperture_mm", "grip.aperture_mm").with_default("0.0"),
            CsvColumn::new("emg_0", "emg_sensors.0.signal_mv").with_default("0.0"),
            CsvColumn::new("emg_1", "emg_sensors.1.signal_mv").with_default("0.0"),
            CsvColumn::new("emg_2", "emg_sensors.2.signal_mv").with_default("0.0"),
            CsvColumn::new("emg_3", "emg_sensors.3.signal_mv").with_default("0.0"),
            CsvColumn::new("battery_percent", "battery_percent").with_default("0"),
        ];
        self
    }

    /// Use default columns for rehabilitation data
    pub fn with_rehabilitation_columns(mut self) -> Self {
        self.columns = vec![
            CsvColumn::new("device_id", "device_id"),
            CsvColumn::new("session_id", "session.id"),
            CsvColumn::new("exercise_type", "exercise.type"),
            CsvColumn::new("repetition", "exercise.repetition").with_default("0"),
            CsvColumn::new("total_reps", "exercise.total_repetitions").with_default("0"),
            CsvColumn::new("rom_target", "performance.rom_target_deg").with_default("0.0"),
            CsvColumn::new("rom_achieved", "performance.rom_achieved_deg").with_default("0.0"),
            CsvColumn::new("force_target", "performance.force_target_n").with_default("0.0"),
            CsvColumn::new("force_achieved", "performance.force_achieved_n").with_default("0.0"),
        ];
        self
    }

    /// Use generic columns
    pub fn with_generic_columns(mut self) -> Self {
        self.columns = vec![
            CsvColumn::new("device_id", "device_id"),
            CsvColumn::new("robot_type", "robot_type"),
            CsvColumn::new("status", "status").with_default("unknown"),
        ];
        self
    }

    /// Get output directory
    pub fn output_dir(&self) -> &str {
        &self.output_dir
    }

    /// Generate header row
    pub fn header(&self) -> String {
        let mut header_parts = Vec::new();

        if self.include_timestamp {
            header_parts.push("timestamp".to_string());
        }

        for col in &self.columns {
            header_parts.push(col.name.clone());
        }

        header_parts.join(",")
    }

    /// Generate data row from OutputData
    pub fn data_row(&self, data: &OutputData) -> String {
        let mut row = CsvRow::new();

        if self.include_timestamp {
            row.push(data.timestamp.to_rfc3339());
        }

        // Create combined data with device_id and robot_type
        let mut combined = data.data.clone();
        if let Some(obj) = combined.as_object_mut() {
            obj.insert("device_id".to_string(), serde_json::json!(data.device_id));
            obj.insert("robot_type".to_string(), serde_json::json!(data.robot_type));
        }

        for col in &self.columns {
            row.push(col.extract(&combined));
        }

        row.to_csv()
    }

    /// Generate suggested filename
    pub fn suggested_filename(&self, data: &OutputData) -> String {
        let timestamp = data.timestamp.format("%Y%m%d_%H%M%S");
        format!("{}_{}.csv", data.device_id, timestamp)
    }

    /// Export multiple data points to CSV string
    pub fn export_multiple(&self, data_points: &[OutputData]) -> String {
        let mut output = String::new();

        if self.include_header {
            output.push_str(&self.header());
            output.push('\n');
        }

        for data in data_points {
            output.push_str(&self.data_row(data));
            output.push('\n');
        }

        output
    }
}

impl OutputAdapter for CsvExporter {
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
        if let Some(header) = config.get_bool_option("include_header") {
            self.include_header = header;
        }
        if let Some(ts) = config.get_bool_option("include_timestamp") {
            self.include_timestamp = ts;
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let mut csv = String::new();

        if self.include_header {
            csv.push_str(&self.header());
            csv.push('\n');
        }

        csv.push_str(&self.data_row(data));
        csv.push('\n');

        let filename = self.suggested_filename(data);

        Ok(OutputResult::success("Exported to CSV")
            .with_metadata(serde_json::json!({
                "format": "csv",
                "filename": filename,
                "columns": self.columns.len(),
                "output": csv
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
    fn test_csv_column_extraction() {
        let col = CsvColumn::new("status", "status");
        let data = serde_json::json!({"status": "active"});

        assert_eq!(col.extract(&data), "active");
    }

    #[test]
    fn test_csv_column_nested_extraction() {
        let col = CsvColumn::new("angle", "joints.0.angle_deg").with_default("0.0");
        let data = serde_json::json!({
            "joints": [{"angle_deg": 15.5}]
        });

        assert_eq!(col.extract(&data), "15.5");
    }

    #[test]
    fn test_csv_column_default() {
        let col = CsvColumn::new("missing", "nonexistent").with_default("N/A");
        let data = serde_json::json!({});

        assert_eq!(col.extract(&data), "N/A");
    }

    #[test]
    fn test_csv_row() {
        let mut row = CsvRow::new();
        row.push("value1".to_string());
        row.push("value, with comma".to_string());
        row.push("value \"with\" quotes".to_string());

        let csv = row.to_csv();
        assert!(csv.contains("value1"));
        assert!(csv.contains("\"value, with comma\""));
        assert!(csv.contains("\"value \"\"with\"\" quotes\""));
    }

    #[test]
    fn test_csv_exporter_creation() {
        let exporter = CsvExporter::new("./test_export")
            .with_header(true)
            .with_timestamp(true)
            .add_column(CsvColumn::new("status", "status"));

        assert_eq!(exporter.output_type(), OutputType::Export);
        assert_eq!(exporter.name(), "csv");
        assert_eq!(exporter.output_dir(), "./test_export");
    }

    #[test]
    fn test_csv_exporter_with_presets() {
        let exporter = CsvExporter::new("./export")
            .with_exoskeleton_columns();

        let header = exporter.header();
        assert!(header.contains("timestamp"));
        assert!(header.contains("device_id"));
        assert!(header.contains("hip_left_angle"));
    }

    #[test]
    fn test_csv_exporter_output() {
        let exporter = CsvExporter::new("./export")
            .with_generic_columns();

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active"
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);

        let metadata = result.metadata.unwrap();
        assert_eq!(metadata["format"], "csv");
    }

    #[test]
    fn test_csv_export_multiple() {
        let exporter = CsvExporter::new("./export")
            .with_generic_columns();

        let data1 = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({"status": "active"}));
        let data2 = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({"status": "idle"}));

        let csv = exporter.export_multiple(&[data1, data2]);
        let lines: Vec<&str> = csv.lines().collect();

        assert_eq!(lines.len(), 3); // header + 2 data rows
        assert!(lines[0].contains("device_id"));
    }
}
