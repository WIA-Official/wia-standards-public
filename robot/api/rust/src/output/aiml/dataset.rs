//! AI/ML dataset generation for training

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::io::Write;

/// Training datapoint for ML models
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingDatapoint {
    /// Timestamp as Unix epoch
    pub timestamp: i64,
    /// Device identifier
    pub device_id: String,
    /// Feature vector
    pub features: Vec<f64>,
    /// Label for supervised learning
    pub label: String,
    /// Optional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

impl TrainingDatapoint {
    /// Create a new training datapoint
    pub fn new(device_id: &str, features: Vec<f64>, label: &str) -> Self {
        Self {
            timestamp: Utc::now().timestamp(),
            device_id: device_id.to_string(),
            features,
            label: label.to_string(),
            metadata: None,
        }
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp: DateTime<Utc>) -> Self {
        self.timestamp = timestamp.timestamp();
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: serde_json::Value) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Convert to JSON Lines format
    pub fn to_jsonl(&self) -> RobotResult<String> {
        Ok(serde_json::to_string(self)?)
    }

    /// Convert to CSV row
    pub fn to_csv_row(&self) -> String {
        let features_str = self
            .features
            .iter()
            .map(|f| f.to_string())
            .collect::<Vec<_>>()
            .join(";");
        format!(
            "{},{},{},{}",
            self.timestamp, self.device_id, features_str, self.label
        )
    }
}

/// Dataset metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DatasetMetadata {
    /// Dataset name
    pub dataset_name: String,
    /// Version
    pub version: String,
    /// Creation timestamp
    pub created_at: String,
    /// Description
    pub description: String,
    /// Feature definitions
    pub features: Vec<FeatureDefinition>,
    /// Available labels
    pub labels: Vec<String>,
    /// Sample rate in Hz
    pub sample_rate_hz: Option<u32>,
    /// Total number of samples
    pub total_samples: usize,
    /// Number of participants/devices
    pub participants: Option<usize>,
}

impl DatasetMetadata {
    /// Create new dataset metadata
    pub fn new(name: &str, description: &str) -> Self {
        Self {
            dataset_name: name.to_string(),
            version: "1.0.0".to_string(),
            created_at: Utc::now().to_rfc3339(),
            description: description.to_string(),
            features: Vec::new(),
            labels: Vec::new(),
            sample_rate_hz: None,
            total_samples: 0,
            participants: None,
        }
    }

    /// Add feature definition
    pub fn add_feature(&mut self, name: &str, feature_type: &str, unit: &str) {
        self.features.push(FeatureDefinition {
            name: name.to_string(),
            feature_type: feature_type.to_string(),
            unit: unit.to_string(),
        });
    }

    /// Add label
    pub fn add_label(&mut self, label: &str) {
        self.labels.push(label.to_string());
    }

    /// Set sample rate
    pub fn with_sample_rate(mut self, rate: u32) -> Self {
        self.sample_rate_hz = Some(rate);
        self
    }

    /// Set total samples
    pub fn with_total_samples(mut self, count: usize) -> Self {
        self.total_samples = count;
        self
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// Feature definition for dataset
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureDefinition {
    /// Feature name
    pub name: String,
    /// Data type (float, int, bool)
    #[serde(rename = "type")]
    pub feature_type: String,
    /// Unit of measurement
    pub unit: String,
}

/// Dataset format
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DatasetFormat {
    /// JSON Lines format (.jsonl)
    JsonLines,
    /// CSV format (.csv)
    Csv,
    /// NumPy compatible format (.npy)
    NumPy,
}

/// Dataset exporter for ML training data
pub struct DatasetExporter {
    base: BaseAdapter,
    output_dir: String,
    format: DatasetFormat,
    buffer: Vec<TrainingDatapoint>,
    metadata: Option<DatasetMetadata>,
}

impl Default for DatasetExporter {
    fn default() -> Self {
        Self::new("./dataset")
    }
}

impl DatasetExporter {
    /// Create a new dataset exporter
    pub fn new(output_dir: &str) -> Self {
        Self {
            base: BaseAdapter::new("dataset", OutputType::AiMl),
            output_dir: output_dir.to_string(),
            format: DatasetFormat::JsonLines,
            buffer: Vec::new(),
            metadata: None,
        }
    }

    /// Set output format
    pub fn with_format(mut self, format: DatasetFormat) -> Self {
        self.format = format;
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: DatasetMetadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Add datapoint to buffer
    pub fn add_datapoint(&mut self, datapoint: TrainingDatapoint) {
        self.buffer.push(datapoint);
    }

    /// Get buffer size
    pub fn buffer_size(&self) -> usize {
        self.buffer.len()
    }

    /// Clear buffer
    pub fn clear_buffer(&mut self) {
        self.buffer.clear();
    }

    /// Extract EMG features from robot data
    pub fn extract_emg_features(&self, data: &OutputData) -> Vec<f64> {
        let mut features = Vec::new();

        if let Some(emg_sensors) = data.data.get("emg_sensors").and_then(|v| v.as_array()) {
            for sensor in emg_sensors {
                if let Some(signal) = sensor.get("signal_mv").and_then(|v| v.as_f64()) {
                    features.push(signal);
                }
                if let Some(activation) = sensor.get("activation_level").and_then(|v| v.as_f64()) {
                    features.push(activation);
                }
            }
        }

        features
    }

    /// Extract IMU features from robot data
    pub fn extract_imu_features(&self, data: &OutputData) -> Vec<f64> {
        let mut features = Vec::new();

        if let Some(imu) = data.data.get("imu") {
            // Accelerometer
            if let Some(accel) = imu.get("accelerometer") {
                features.push(accel.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0));
                features.push(accel.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0));
                features.push(accel.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0));
            }

            // Gyroscope
            if let Some(gyro) = imu.get("gyroscope") {
                features.push(gyro.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0));
                features.push(gyro.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0));
                features.push(gyro.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0));
            }
        }

        features
    }

    /// Extract joint features from robot data
    pub fn extract_joint_features(&self, data: &OutputData) -> Vec<f64> {
        let mut features = Vec::new();

        if let Some(joints) = data.data.get("joints").and_then(|v| v.as_array()) {
            for joint in joints {
                if let Some(angle) = joint.get("angle_deg").and_then(|v| v.as_f64()) {
                    features.push(angle);
                }
                if let Some(velocity) = joint.get("velocity_deg_s").and_then(|v| v.as_f64()) {
                    features.push(velocity);
                }
                if let Some(torque) = joint.get("torque_nm").and_then(|v| v.as_f64()) {
                    features.push(torque);
                }
            }
        }

        features
    }

    /// Create datapoint from robot data
    pub fn create_datapoint(&self, data: &OutputData, label: &str) -> TrainingDatapoint {
        let mut features = Vec::new();

        // Extract all available features
        features.extend(self.extract_emg_features(data));
        features.extend(self.extract_imu_features(data));
        features.extend(self.extract_joint_features(data));

        // If no specific features found, try to extract generic numeric data
        if features.is_empty() {
            if let Some(obj) = data.data.as_object() {
                for value in obj.values() {
                    if let Some(num) = value.as_f64() {
                        features.push(num);
                    }
                }
            }
        }

        TrainingDatapoint::new(&data.device_id, features, label)
            .with_timestamp(data.timestamp)
    }

    /// Export buffer to JSON Lines format
    pub fn export_jsonl(&self) -> RobotResult<String> {
        let mut output = String::new();
        for dp in &self.buffer {
            output.push_str(&dp.to_jsonl()?);
            output.push('\n');
        }
        Ok(output)
    }

    /// Export buffer to CSV format
    pub fn export_csv(&self) -> RobotResult<String> {
        let mut output = String::from("timestamp,device_id,features,label\n");
        for dp in &self.buffer {
            output.push_str(&dp.to_csv_row());
            output.push('\n');
        }
        Ok(output)
    }

    /// Get feature count from first datapoint
    pub fn feature_count(&self) -> usize {
        self.buffer.first().map(|dp| dp.features.len()).unwrap_or(0)
    }

    /// Get unique labels in buffer
    pub fn unique_labels(&self) -> Vec<String> {
        let mut labels: Vec<String> = self.buffer.iter().map(|dp| dp.label.clone()).collect();
        labels.sort();
        labels.dedup();
        labels
    }
}

impl OutputAdapter for DatasetExporter {
    fn output_type(&self) -> OutputType {
        OutputType::AiMl
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(dir) = config.get_string_option("output_dir") {
            self.output_dir = dir.to_string();
        }
        if let Some(format) = config.get_string_option("format") {
            self.format = match format {
                "csv" => DatasetFormat::Csv,
                "numpy" => DatasetFormat::NumPy,
                _ => DatasetFormat::JsonLines,
            };
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        // Extract label from data or use default
        let label = data.data.get("label")
            .and_then(|l| l.as_str())
            .unwrap_or("unlabeled");

        let datapoint = self.create_datapoint(data, label);

        let output = match self.format {
            DatasetFormat::JsonLines => datapoint.to_jsonl()?,
            DatasetFormat::Csv => datapoint.to_csv_row(),
            DatasetFormat::NumPy => {
                // For NumPy, return JSON that can be converted by Python
                serde_json::to_string(&datapoint)?
            }
        };

        Ok(OutputResult::success("Created training datapoint")
            .with_metadata(serde_json::json!({
                "format": match self.format {
                    DatasetFormat::JsonLines => "jsonl",
                    DatasetFormat::Csv => "csv",
                    DatasetFormat::NumPy => "numpy",
                },
                "feature_count": datapoint.features.len(),
                "label": label,
                "output": output
            })))
    }

    fn is_available(&self) -> bool {
        self.base.available
    }

    fn dispose(&mut self) -> RobotResult<()> {
        self.clear_buffer();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_training_datapoint() {
        let dp = TrainingDatapoint::new("device-001", vec![0.5, 0.3, 0.8, 0.2], "grip");

        assert_eq!(dp.device_id, "device-001");
        assert_eq!(dp.features.len(), 4);
        assert_eq!(dp.label, "grip");
    }

    #[test]
    fn test_datapoint_to_jsonl() {
        let dp = TrainingDatapoint::new("device-001", vec![0.5, 0.3], "rest");
        let jsonl = dp.to_jsonl().unwrap();

        assert!(jsonl.contains("\"device_id\":\"device-001\""));
        assert!(jsonl.contains("\"label\":\"rest\""));
    }

    #[test]
    fn test_datapoint_to_csv() {
        let dp = TrainingDatapoint::new("device-001", vec![0.5, 0.3], "rest");
        let csv = dp.to_csv_row();

        assert!(csv.contains("device-001"));
        assert!(csv.contains("0.5;0.3"));
        assert!(csv.contains("rest"));
    }

    #[test]
    fn test_dataset_metadata() {
        let mut metadata = DatasetMetadata::new(
            "wia_emg_gestures",
            "EMG gesture dataset from prosthetic devices",
        )
        .with_sample_rate(1000)
        .with_total_samples(10000);

        metadata.add_feature("emg_0", "float", "mV");
        metadata.add_feature("emg_1", "float", "mV");
        metadata.add_label("grip");
        metadata.add_label("rest");

        assert_eq!(metadata.features.len(), 2);
        assert_eq!(metadata.labels.len(), 2);
        assert_eq!(metadata.sample_rate_hz, Some(1000));
    }

    #[test]
    fn test_dataset_exporter() {
        let mut exporter = DatasetExporter::new("./test_data")
            .with_format(DatasetFormat::JsonLines);

        let dp1 = TrainingDatapoint::new("device-001", vec![0.5, 0.3], "grip");
        let dp2 = TrainingDatapoint::new("device-001", vec![0.1, 0.1], "rest");

        exporter.add_datapoint(dp1);
        exporter.add_datapoint(dp2);

        assert_eq!(exporter.buffer_size(), 2);

        let labels = exporter.unique_labels();
        assert!(labels.contains(&"grip".to_string()));
        assert!(labels.contains(&"rest".to_string()));
    }

    #[test]
    fn test_export_jsonl() {
        let mut exporter = DatasetExporter::new("./test_data");
        exporter.add_datapoint(TrainingDatapoint::new("device-001", vec![0.5], "test"));

        let jsonl = exporter.export_jsonl().unwrap();
        assert!(jsonl.contains("device-001"));
    }

    #[test]
    fn test_export_csv() {
        let mut exporter = DatasetExporter::new("./test_data");
        exporter.add_datapoint(TrainingDatapoint::new("device-001", vec![0.5], "test"));

        let csv = exporter.export_csv().unwrap();
        assert!(csv.starts_with("timestamp,device_id,features,label\n"));
    }

    #[test]
    fn test_extract_features() {
        let exporter = DatasetExporter::new("./test_data");

        let data = OutputData::new("prosthetic-001", "prosthetic")
            .with_data(serde_json::json!({
                "emg_sensors": [
                    {"signal_mv": 0.5, "activation_level": 0.8},
                    {"signal_mv": 0.3, "activation_level": 0.6}
                ],
                "imu": {
                    "accelerometer": {"x": 0.1, "y": -0.2, "z": 9.8},
                    "gyroscope": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
            }));

        let emg_features = exporter.extract_emg_features(&data);
        assert_eq!(emg_features.len(), 4); // 2 sensors * 2 values

        let imu_features = exporter.extract_imu_features(&data);
        assert_eq!(imu_features.len(), 6); // 3 accel + 3 gyro
    }

    #[test]
    fn test_dataset_output_adapter() {
        let exporter = DatasetExporter::new("./test_data");

        assert_eq!(exporter.output_type(), OutputType::AiMl);
        assert_eq!(exporter.name(), "dataset");

        let data = OutputData::new("prosthetic-001", "prosthetic")
            .with_data(serde_json::json!({
                "label": "grip",
                "emg_sensors": [{"signal_mv": 0.5, "activation_level": 0.8}]
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);
    }
}
