//! Climate message handling and builder
//!
//! This module provides the main `ClimateMessage` struct and its builder
//! for creating standards-compliant climate data messages.

use crate::error::{ClimateError, Result};
use crate::types::*;
use crate::SPEC_VERSION;
use serde::{Deserialize, Serialize};

/// Main climate data message
///
/// This struct represents a complete WIA Climate Standard message
/// containing all required and optional fields.
///
/// # Example
///
/// ```rust
/// use wia_climate::prelude::*;
///
/// let message = ClimateMessage::builder()
///     .data_type(DataType::CarbonCapture)
///     .location(Location::new(64.0, -21.0))
///     .device(Device::new("Climeworks", "Orca DAC"))
///     .carbon_capture_data(CarbonCaptureData {
///         technology: CarbonCaptureTechnology::Dac,
///         capture_rate_kg_per_hour: 125.5,
///         ..Default::default()
///     })
///     .build()
///     .unwrap();
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClimateMessage {
    /// JSON Schema URI (optional)
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,

    /// Specification version
    pub version: String,

    /// Data type identifier
    #[serde(rename = "type")]
    pub data_type: DataType,

    /// Timestamp
    pub timestamp: Timestamp,

    /// Geographic location
    pub location: Location,

    /// Device/sensor information
    pub device: Device,

    /// Domain-specific data
    pub data: ClimateData,

    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<Metadata>,
}

impl ClimateMessage {
    /// Create a new message builder
    pub fn builder() -> ClimateMessageBuilder {
        ClimateMessageBuilder::new()
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string(self).map_err(ClimateError::from)
    }

    /// Serialize to pretty JSON string
    pub fn to_json_pretty(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(ClimateError::from)
    }

    /// Deserialize from JSON string
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(ClimateError::from)
    }

    /// Validate the message against the schema
    pub fn validate(&self) -> Result<()> {
        // Validate location
        if !self.location.is_valid() {
            return Err(ClimateError::InvalidCoordinates {
                lat: self.location.latitude,
                lon: self.location.longitude,
            });
        }

        // Validate metadata quality score
        if let Some(ref meta) = self.meta {
            if let Some(quality) = meta.quality_score {
                if !(0.0..=1.0).contains(&quality) {
                    return Err(ClimateError::out_of_range(
                        "quality_score",
                        quality,
                        0.0,
                        1.0,
                    ));
                }
            }
        }

        // Validate domain-specific data
        match &self.data {
            ClimateData::CarbonCapture(data) => self.validate_carbon_capture(data)?,
            ClimateData::WeatherControl(data) => self.validate_weather_control(data)?,
            ClimateData::VerticalFarming(data) => self.validate_vertical_farming(data)?,
            ClimateData::OceanCleanup(data) => self.validate_ocean_cleanup(data)?,
            _ => {}
        }

        Ok(())
    }

    fn validate_carbon_capture(&self, data: &CarbonCaptureData) -> Result<()> {
        if data.capture_rate_kg_per_hour < 0.0 {
            return Err(ClimateError::validation(
                "capture_rate_kg_per_hour must be non-negative",
            ));
        }

        if let Some(purity) = data.co2_purity_percentage {
            if !(0.0..=100.0).contains(&purity) {
                return Err(ClimateError::out_of_range(
                    "co2_purity_percentage",
                    purity,
                    0.0,
                    100.0,
                ));
            }
        }

        Ok(())
    }

    fn validate_weather_control(&self, data: &WeatherControlData) -> Result<()> {
        if !(0.0..=100.0).contains(&data.atmospheric_conditions.humidity_percentage) {
            return Err(ClimateError::out_of_range(
                "humidity_percentage",
                data.atmospheric_conditions.humidity_percentage,
                0.0,
                100.0,
            ));
        }

        Ok(())
    }

    fn validate_vertical_farming(&self, data: &VerticalFarmingData) -> Result<()> {
        if !(0.0..=100.0).contains(&data.environment.humidity_percentage) {
            return Err(ClimateError::out_of_range(
                "humidity_percentage",
                data.environment.humidity_percentage,
                0.0,
                100.0,
            ));
        }

        if !(0.0..=14.0).contains(&data.nutrient_solution.ph) {
            return Err(ClimateError::out_of_range(
                "ph",
                data.nutrient_solution.ph,
                0.0,
                14.0,
            ));
        }

        Ok(())
    }

    fn validate_ocean_cleanup(&self, data: &OceanCleanupData) -> Result<()> {
        if data.collection.total_mass_kg < 0.0 {
            return Err(ClimateError::validation(
                "total_mass_kg must be non-negative",
            ));
        }

        if data.area.swept_km2 < 0.0 {
            return Err(ClimateError::validation("swept_km2 must be non-negative"));
        }

        Ok(())
    }
}

impl Default for ClimateMessage {
    fn default() -> Self {
        Self {
            schema: Some("https://wia.live/climate/data/v1/schema.json".to_string()),
            version: SPEC_VERSION.to_string(),
            data_type: DataType::Custom,
            timestamp: Timestamp::now(),
            location: Location::default(),
            device: Device::default(),
            data: ClimateData::default(),
            meta: None,
        }
    }
}

/// Builder for creating ClimateMessage instances
#[derive(Debug, Default)]
pub struct ClimateMessageBuilder {
    schema: Option<String>,
    version: Option<String>,
    data_type: Option<DataType>,
    timestamp: Option<Timestamp>,
    location: Option<Location>,
    device: Option<Device>,
    data: Option<ClimateData>,
    meta: Option<Metadata>,
}

impl ClimateMessageBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the JSON schema URI
    pub fn schema(mut self, schema: impl Into<String>) -> Self {
        self.schema = Some(schema.into());
        self
    }

    /// Set the specification version
    pub fn version(mut self, version: impl Into<String>) -> Self {
        self.version = Some(version.into());
        self
    }

    /// Set the data type
    pub fn data_type(mut self, data_type: DataType) -> Self {
        self.data_type = Some(data_type);
        self
    }

    /// Set the timestamp
    pub fn timestamp(mut self, timestamp: Timestamp) -> Self {
        self.timestamp = Some(timestamp);
        self
    }

    /// Set the location
    pub fn location(mut self, location: Location) -> Self {
        self.location = Some(location);
        self
    }

    /// Set the device
    pub fn device(mut self, device: Device) -> Self {
        self.device = Some(device);
        self
    }

    /// Set carbon capture data
    pub fn carbon_capture_data(mut self, data: CarbonCaptureData) -> Self {
        self.data_type = Some(DataType::CarbonCapture);
        self.data = Some(ClimateData::CarbonCapture(data));
        self
    }

    /// Set weather control data
    pub fn weather_control_data(mut self, data: WeatherControlData) -> Self {
        self.data_type = Some(DataType::WeatherControl);
        self.data = Some(ClimateData::WeatherControl(data));
        self
    }

    /// Set geoengineering data
    pub fn geoengineering_data(mut self, data: GeoengineeringData) -> Self {
        self.data_type = Some(DataType::Geoengineering);
        self.data = Some(ClimateData::Geoengineering(data));
        self
    }

    /// Set vertical farming data
    pub fn vertical_farming_data(mut self, data: VerticalFarmingData) -> Self {
        self.data_type = Some(DataType::VerticalFarming);
        self.data = Some(ClimateData::VerticalFarming(data));
        self
    }

    /// Set ocean cleanup data
    pub fn ocean_cleanup_data(mut self, data: OceanCleanupData) -> Self {
        self.data_type = Some(DataType::OceanCleanup);
        self.data = Some(ClimateData::OceanCleanup(data));
        self
    }

    /// Set climate model data
    pub fn climate_model_data(mut self, data: ClimateModelData) -> Self {
        self.data_type = Some(DataType::ClimateModel);
        self.data = Some(ClimateData::ClimateModel(data));
        self
    }

    /// Set custom data
    pub fn custom_data(mut self, data: serde_json::Value) -> Self {
        self.data_type = Some(DataType::Custom);
        self.data = Some(ClimateData::Custom(data));
        self
    }

    /// Set the raw data (any ClimateData variant)
    pub fn data(mut self, data: ClimateData) -> Self {
        self.data = Some(data);
        self
    }

    /// Set the metadata
    pub fn meta(mut self, meta: Metadata) -> Self {
        self.meta = Some(meta);
        self
    }

    /// Build the ClimateMessage
    pub fn build(self) -> Result<ClimateMessage> {
        let data_type = self
            .data_type
            .ok_or_else(|| ClimateError::builder("data_type is required"))?;

        let location = self
            .location
            .ok_or_else(|| ClimateError::builder("location is required"))?;

        let device = self
            .device
            .ok_or_else(|| ClimateError::builder("device is required"))?;

        let data = self
            .data
            .ok_or_else(|| ClimateError::builder("data is required"))?;

        let message = ClimateMessage {
            schema: self.schema.or(Some(
                "https://wia.live/climate/data/v1/schema.json".to_string(),
            )),
            version: self.version.unwrap_or_else(|| SPEC_VERSION.to_string()),
            data_type,
            timestamp: self.timestamp.unwrap_or_else(Timestamp::now),
            location,
            device,
            data,
            meta: self.meta,
        };

        // Validate before returning
        message.validate()?;

        Ok(message)
    }
}

/// Trait for climate data adapters
#[async_trait::async_trait]
pub trait ClimateAdapter: Send + Sync {
    /// Get the adapter name
    fn name(&self) -> &str;

    /// Check if the adapter is connected
    fn is_connected(&self) -> bool;

    /// Read a single data point
    async fn read(&self) -> Result<ClimateMessage>;

    /// Start continuous reading
    async fn start_stream(&mut self) -> Result<()>;

    /// Stop continuous reading
    async fn stop_stream(&mut self) -> Result<()>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_builder() {
        let message = ClimateMessage::builder()
            .data_type(DataType::CarbonCapture)
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Climeworks", "Orca DAC"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                ..Default::default()
            })
            .build()
            .unwrap();

        assert_eq!(message.data_type, DataType::CarbonCapture);
        assert_eq!(message.version, SPEC_VERSION);
    }

    #[test]
    fn test_message_serialization() {
        let message = ClimateMessage::builder()
            .data_type(DataType::CarbonCapture)
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Climeworks", "Orca DAC"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                co2_purity_percentage: Some(99.2),
                ..Default::default()
            })
            .build()
            .unwrap();

        let json = message.to_json().unwrap();
        assert!(json.contains("carbon_capture"));
        assert!(json.contains("125.5"));

        // Deserialize and verify
        let parsed = ClimateMessage::from_json(&json).unwrap();
        assert_eq!(parsed.data_type, DataType::CarbonCapture);
    }

    #[test]
    fn test_validation_invalid_coordinates() {
        let result = ClimateMessage::builder()
            .data_type(DataType::CarbonCapture)
            .location(Location::new(100.0, 200.0)) // Invalid
            .device(Device::new("Test", "Test"))
            .carbon_capture_data(CarbonCaptureData::default())
            .build();

        assert!(result.is_err());
    }

    #[test]
    fn test_validation_invalid_humidity() {
        let result = ClimateMessage::builder()
            .data_type(DataType::WeatherControl)
            .location(Location::new(35.0, 127.0))
            .device(Device::new("Test", "Test"))
            .weather_control_data(WeatherControlData {
                operation_type: WeatherOperationType::CloudSeeding,
                atmospheric_conditions: AtmosphericConditions {
                    temperature_celsius: 20.0,
                    humidity_percentage: 150.0, // Invalid
                    ..Default::default()
                },
                ..Default::default()
            })
            .build();

        assert!(result.is_err());
    }

    #[test]
    fn test_builder_missing_required() {
        let result = ClimateMessage::builder().build();
        assert!(result.is_err());
    }
}
