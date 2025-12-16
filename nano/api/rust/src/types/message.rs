//! WIA Nano message types

use serde::{Deserialize, Serialize};
use serde_json::Value;
use super::{Device, Timestamp, Scale, Environment, Metadata, NanoSystemType};

/// Base WIA Nano message structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NanoMessage {
    #[serde(rename = "$schema")]
    pub schema: String,
    pub version: String,
    pub system_type: NanoSystemType,
    pub device: Device,
    pub timestamp: Timestamp,
    pub sequence: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub environment: Option<Environment>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scale: Option<Scale>,
    pub data: Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<Metadata>,
}

impl NanoMessage {
    /// Create a new message builder
    pub fn builder() -> NanoMessageBuilder {
        NanoMessageBuilder::default()
    }

    /// Create a new message builder with system type
    pub fn builder_for(system_type: NanoSystemType) -> NanoMessageBuilder {
        NanoMessageBuilder::new(system_type)
    }

    /// Parse data into a specific type
    pub fn parse_data<T: for<'de> Deserialize<'de>>(&self) -> Result<T, serde_json::Error> {
        serde_json::from_value(self.data.clone())
    }

    /// Get the schema URL
    pub fn schema_url(&self) -> &str {
        &self.schema
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Deserialize from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

/// Builder for NanoMessage
pub struct NanoMessageBuilder {
    system_type: NanoSystemType,
    device: Option<Device>,
    environment: Option<Environment>,
    scale: Option<Scale>,
    meta: Option<Metadata>,
    sequence: u64,
}

impl Default for NanoMessageBuilder {
    fn default() -> Self {
        Self {
            system_type: NanoSystemType::Nanosensor,
            device: None,
            environment: None,
            scale: None,
            meta: None,
            sequence: 0,
        }
    }
}

impl NanoMessageBuilder {
    pub fn new(system_type: NanoSystemType) -> Self {
        Self {
            system_type,
            device: None,
            environment: None,
            scale: None,
            meta: None,
            sequence: 0,
        }
    }

    pub fn device(mut self, device: Device) -> Self {
        self.device = Some(device);
        self
    }

    pub fn environment(mut self, env: Environment) -> Self {
        self.environment = Some(env);
        self
    }

    pub fn scale(mut self, scale: Scale) -> Self {
        self.scale = Some(scale);
        self
    }

    pub fn meta(mut self, meta: Metadata) -> Self {
        self.meta = Some(meta);
        self
    }

    pub fn sequence(mut self, seq: u64) -> Self {
        self.sequence = seq;
        self
    }

    /// Convenience method to set device by ID
    pub fn device_id(mut self, id: &str) -> Self {
        self.device = Some(Device::new("WIA", id));
        self
    }

    /// Convenience method to set system type
    pub fn system_type(mut self, sys_type: NanoSystemType) -> Self {
        self.system_type = sys_type;
        self
    }

    /// Set payload and build the message
    pub fn payload(self, data: serde_json::Value) -> Result<NanoMessage, serde_json::Error> {
        self.build(data)
    }

    pub fn build<T: Serialize>(self, data: T) -> Result<NanoMessage, serde_json::Error> {
        let device = self.device.unwrap_or_else(|| {
            Device::new("WIA", "Unknown")
        });

        Ok(NanoMessage {
            schema: "https://wia.live/nano/v1/schema.json".to_string(),
            version: "1.0.0".to_string(),
            system_type: self.system_type,
            device,
            timestamp: Timestamp::now(),
            sequence: self.sequence,
            environment: self.environment,
            scale: self.scale,
            data: serde_json::to_value(data)?,
            meta: self.meta,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_builder() {
        let data = serde_json::json!({
            "test": "value"
        });

        let msg = NanoMessage::builder_for(NanoSystemType::Nanosensor)
            .device(Device::new("WIA", "Test-Sensor"))
            .sequence(1)
            .build(data)
            .unwrap();

        assert_eq!(msg.system_type, NanoSystemType::Nanosensor);
        assert_eq!(msg.sequence, 1);
    }

    #[test]
    fn test_message_serialization() {
        let msg = NanoMessage::builder_for(NanoSystemType::Nanorobot)
            .device(Device::new("WIA", "Test-Robot"))
            .build(serde_json::json!({"position": {"x": 0, "y": 0, "z": 0}}))
            .unwrap();

        let json = msg.to_json().unwrap();
        let parsed: NanoMessage = NanoMessage::from_json(&json).unwrap();

        assert_eq!(parsed.system_type, NanoSystemType::Nanorobot);
    }
}
