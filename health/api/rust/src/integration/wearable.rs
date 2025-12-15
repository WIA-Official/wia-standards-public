//! Wearable Device Integration Adapters
//!
//! Adapters for Apple HealthKit, Google Health Connect, Fitbit, and Garmin

use async_trait::async_trait;
use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::adapter::*;
use crate::error::{HealthError, Result};
use crate::types::{BiomarkerProfile, Measurement, MetabolicMarkers};

/// Wearable data types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WearableDataType {
    HeartRate,
    HeartRateVariability,
    BloodGlucose,
    BloodPressure,
    OxygenSaturation,
    BodyTemperature,
    Steps,
    Distance,
    Calories,
    Sleep,
    Stress,
    BodyBattery,
    RespiratoryRate,
}

impl std::fmt::Display for WearableDataType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WearableDataType::HeartRate => write!(f, "heart_rate"),
            WearableDataType::HeartRateVariability => write!(f, "hrv"),
            WearableDataType::BloodGlucose => write!(f, "blood_glucose"),
            WearableDataType::BloodPressure => write!(f, "blood_pressure"),
            WearableDataType::OxygenSaturation => write!(f, "spo2"),
            WearableDataType::BodyTemperature => write!(f, "body_temperature"),
            WearableDataType::Steps => write!(f, "steps"),
            WearableDataType::Distance => write!(f, "distance"),
            WearableDataType::Calories => write!(f, "calories"),
            WearableDataType::Sleep => write!(f, "sleep"),
            WearableDataType::Stress => write!(f, "stress"),
            WearableDataType::BodyBattery => write!(f, "body_battery"),
            WearableDataType::RespiratoryRate => write!(f, "respiratory_rate"),
        }
    }
}

/// Wearable data sample
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WearableSample {
    /// Data type
    pub data_type: WearableDataType,
    /// Value
    pub value: f64,
    /// Unit
    pub unit: String,
    /// Start time
    pub start_time: DateTime<Utc>,
    /// End time (for aggregated data)
    pub end_time: Option<DateTime<Utc>>,
    /// Source device
    pub source: Option<String>,
    /// Quality score (0.0-1.0)
    pub quality: Option<f64>,
    /// Additional metadata
    pub metadata: Option<HashMap<String, serde_json::Value>>,
}

impl WearableSample {
    /// Convert to WIA Measurement
    pub fn to_measurement(&self) -> Measurement {
        Measurement {
            value: self.value,
            unit: self.unit.clone(),
            reference_range: None,
            timestamp: Some(self.start_time),
            method: self.source.clone(),
            laboratory: None,
            flags: None,
            notes: None,
        }
    }
}

/// Base wearable adapter trait
#[async_trait]
pub trait WearableAdapter: ImportAdapter {
    /// Get supported data types
    fn supported_data_types(&self) -> Vec<WearableDataType>;

    /// Query samples for a data type
    async fn query_samples(
        &self,
        data_type: WearableDataType,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
    ) -> Result<Vec<WearableSample>>;

    /// Convert samples to biomarker profile
    fn samples_to_biomarkers(&self, samples: &[WearableSample]) -> BiomarkerProfile {
        let mut biomarkers = BiomarkerProfile::default();

        // Group samples by type
        let mut hr_samples: Vec<&WearableSample> = Vec::new();
        let mut glucose_samples: Vec<&WearableSample> = Vec::new();

        for sample in samples {
            match sample.data_type {
                WearableDataType::HeartRate => hr_samples.push(sample),
                WearableDataType::BloodGlucose => glucose_samples.push(sample),
                _ => {}
            }
        }

        // Note: Heart rate samples collected but cardiovascular_markers not in current schema
        // TODO: Add cardiovascular markers to BiomarkerProfile if needed
        let _ = hr_samples; // suppress unused warning

        // Convert glucose
        if let Some(latest) = glucose_samples.last() {
            biomarkers.metabolic_markers = Some(MetabolicMarkers {
                glucose: Some(latest.to_measurement()),
                ..Default::default()
            });
        }

        biomarkers
    }
}

/// Mock HealthKit adapter (for non-iOS platforms)
pub struct MockHealthKitAdapter {
    name: String,
    config: AdapterConfig,
    initialized: bool,
    mock_data: Vec<WearableSample>,
}

impl MockHealthKitAdapter {
    /// Create new mock HealthKit adapter
    pub fn new() -> Self {
        Self {
            name: "HealthKit Adapter (Mock)".to_string(),
            config: AdapterConfig::default(),
            initialized: false,
            mock_data: Self::generate_mock_data(),
        }
    }

    /// Generate mock health data
    fn generate_mock_data() -> Vec<WearableSample> {
        let now = Utc::now();
        let mut samples = Vec::new();

        // Generate 24 hours of heart rate data (every 5 minutes)
        for i in 0..288 {
            let time = now - Duration::minutes(i * 5);
            let base_hr = 70.0;
            let variation = (i as f64 * 0.1).sin() * 10.0;

            samples.push(WearableSample {
                data_type: WearableDataType::HeartRate,
                value: base_hr + variation,
                unit: "bpm".to_string(),
                start_time: time,
                end_time: None,
                source: Some("Apple Watch".to_string()),
                quality: Some(0.95),
                metadata: None,
            });
        }

        // Generate daily steps
        samples.push(WearableSample {
            data_type: WearableDataType::Steps,
            value: 8500.0,
            unit: "steps".to_string(),
            start_time: now - Duration::hours(24),
            end_time: Some(now),
            source: Some("iPhone".to_string()),
            quality: Some(1.0),
            metadata: None,
        });

        samples
    }
}

impl Default for MockHealthKitAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IntegrationAdapter for MockHealthKitAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::HealthKit
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn is_available(&self) -> bool {
        self.initialized
    }

    async fn initialize(&mut self, config: AdapterConfig) -> Result<()> {
        self.config = config;
        self.initialized = true;
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        self.initialized = false;
        Ok(())
    }

    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            can_import: true,
            can_export: false,
            can_stream: true,
            supports_auth: true,
            data_types: vec![
                "heart_rate".to_string(),
                "steps".to_string(),
                "sleep".to_string(),
                "blood_glucose".to_string(),
            ],
        }
    }
}

#[async_trait]
impl ImportAdapter for MockHealthKitAdapter {
    async fn import(&self, options: ImportOptions) -> Result<ImportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("HealthKit adapter not initialized"));
        }

        let start = options.start_date.unwrap_or(Utc::now() - Duration::days(7));
        let end = options.end_date.unwrap_or(Utc::now());

        let filtered: Vec<_> = self
            .mock_data
            .iter()
            .filter(|s| s.start_time >= start && s.start_time <= end)
            .cloned()
            .collect();

        let biomarkers = self.samples_to_biomarkers(&filtered);

        Ok(ImportResult {
            records_imported: filtered.len() as u64,
            data_types: vec!["heart_rate".to_string(), "steps".to_string()],
            profile_updates: None,
            errors: Vec::new(),
            imported_at: Utc::now(),
        })
    }

    async fn start_import_stream(
        &self,
        _callback: Box<dyn Fn(ImportEvent) + Send + Sync>,
    ) -> Result<StreamHandle> {
        Ok(StreamHandle {
            id: uuid::Uuid::new_v4().to_string(),
            stream_type: "healthkit".to_string(),
            created_at: Utc::now(),
        })
    }

    async fn stop_import_stream(&self, _handle: StreamHandle) -> Result<()> {
        Ok(())
    }
}

#[async_trait]
impl WearableAdapter for MockHealthKitAdapter {
    fn supported_data_types(&self) -> Vec<WearableDataType> {
        vec![
            WearableDataType::HeartRate,
            WearableDataType::HeartRateVariability,
            WearableDataType::Steps,
            WearableDataType::Sleep,
            WearableDataType::BloodGlucose,
            WearableDataType::OxygenSaturation,
        ]
    }

    async fn query_samples(
        &self,
        data_type: WearableDataType,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
    ) -> Result<Vec<WearableSample>> {
        Ok(self
            .mock_data
            .iter()
            .filter(|s| {
                s.data_type == data_type && s.start_time >= start && s.start_time <= end
            })
            .cloned()
            .collect())
    }
}

/// Mock Health Connect adapter (for non-Android platforms)
pub struct MockHealthConnectAdapter {
    name: String,
    config: AdapterConfig,
    initialized: bool,
    mock_data: Vec<WearableSample>,
}

impl MockHealthConnectAdapter {
    /// Create new mock Health Connect adapter
    pub fn new() -> Self {
        Self {
            name: "Health Connect Adapter (Mock)".to_string(),
            config: AdapterConfig::default(),
            initialized: false,
            mock_data: Self::generate_mock_data(),
        }
    }

    fn generate_mock_data() -> Vec<WearableSample> {
        let now = Utc::now();
        let mut samples = Vec::new();

        // Generate heart rate data
        for i in 0..100 {
            let time = now - Duration::minutes(i * 10);
            samples.push(WearableSample {
                data_type: WearableDataType::HeartRate,
                value: 65.0 + (i as f64 % 20.0),
                unit: "bpm".to_string(),
                start_time: time,
                end_time: None,
                source: Some("Samsung Galaxy Watch".to_string()),
                quality: Some(0.92),
                metadata: None,
            });
        }

        samples
    }
}

impl Default for MockHealthConnectAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IntegrationAdapter for MockHealthConnectAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::HealthConnect
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn is_available(&self) -> bool {
        self.initialized
    }

    async fn initialize(&mut self, config: AdapterConfig) -> Result<()> {
        self.config = config;
        self.initialized = true;
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        self.initialized = false;
        Ok(())
    }

    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            can_import: true,
            can_export: true,
            can_stream: true,
            supports_auth: true,
            data_types: vec![
                "HeartRateRecord".to_string(),
                "StepsRecord".to_string(),
                "SleepSessionRecord".to_string(),
            ],
        }
    }
}

#[async_trait]
impl ImportAdapter for MockHealthConnectAdapter {
    async fn import(&self, options: ImportOptions) -> Result<ImportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("Health Connect adapter not initialized"));
        }

        let start = options.start_date.unwrap_or(Utc::now() - Duration::days(30));
        let end = options.end_date.unwrap_or(Utc::now());

        let filtered: Vec<_> = self
            .mock_data
            .iter()
            .filter(|s| s.start_time >= start && s.start_time <= end)
            .cloned()
            .collect();

        Ok(ImportResult {
            records_imported: filtered.len() as u64,
            data_types: vec!["HeartRateRecord".to_string()],
            profile_updates: None,
            errors: Vec::new(),
            imported_at: Utc::now(),
        })
    }

    async fn start_import_stream(
        &self,
        _callback: Box<dyn Fn(ImportEvent) + Send + Sync>,
    ) -> Result<StreamHandle> {
        Ok(StreamHandle {
            id: uuid::Uuid::new_v4().to_string(),
            stream_type: "health_connect".to_string(),
            created_at: Utc::now(),
        })
    }

    async fn stop_import_stream(&self, _handle: StreamHandle) -> Result<()> {
        Ok(())
    }
}

#[async_trait]
impl WearableAdapter for MockHealthConnectAdapter {
    fn supported_data_types(&self) -> Vec<WearableDataType> {
        vec![
            WearableDataType::HeartRate,
            WearableDataType::Steps,
            WearableDataType::Sleep,
            WearableDataType::BloodGlucose,
        ]
    }

    async fn query_samples(
        &self,
        data_type: WearableDataType,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
    ) -> Result<Vec<WearableSample>> {
        Ok(self
            .mock_data
            .iter()
            .filter(|s| {
                s.data_type == data_type && s.start_time >= start && s.start_time <= end
            })
            .cloned()
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_healthkit_adapter() {
        let mut adapter = MockHealthKitAdapter::new();
        adapter.initialize(AdapterConfig::default()).await.unwrap();

        assert!(adapter.is_available().await);
        assert_eq!(adapter.adapter_type(), AdapterType::HealthKit);

        let result = adapter.import(ImportOptions::default()).await.unwrap();
        assert!(result.records_imported > 0);
    }

    #[tokio::test]
    async fn test_wearable_sample_conversion() {
        let sample = WearableSample {
            data_type: WearableDataType::HeartRate,
            value: 72.0,
            unit: "bpm".to_string(),
            start_time: Utc::now(),
            end_time: None,
            source: Some("Test Device".to_string()),
            quality: Some(0.95),
            metadata: None,
        };

        let measurement = sample.to_measurement();
        assert_eq!(measurement.value, 72.0);
        assert_eq!(measurement.unit, "bpm");
    }
}
