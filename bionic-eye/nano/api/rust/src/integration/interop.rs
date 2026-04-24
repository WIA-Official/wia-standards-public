//! Cross-domain interoperability with other WIA standards

use crate::error::{NanoError, NanoResult};
use crate::types::NanoMessage;
use serde::{Deserialize, Serialize};

/// WIA domain types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WiaDomain {
    /// Augmentative and Alternative Communication
    Aac,
    /// Climate and Environmental Monitoring
    Climate,
    /// Nanotechnology
    Nano,
    /// Industrial IoT
    Iiot,
    /// Healthcare
    Health,
    /// Smart Grid
    Energy,
    /// Custom domain
    Custom,
}

impl WiaDomain {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Aac => "aac",
            Self::Climate => "climate",
            Self::Nano => "nano",
            Self::Iiot => "iiot",
            Self::Health => "health",
            Self::Energy => "energy",
            Self::Custom => "custom",
        }
    }
}

/// Cross-domain message wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrossDomainMessage {
    pub source_domain: WiaDomain,
    pub target_domain: WiaDomain,
    pub message_type: CrossDomainMessageType,
    pub payload: serde_json::Value,
    pub timestamp: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

impl CrossDomainMessage {
    pub fn new(
        source: WiaDomain,
        target: WiaDomain,
        msg_type: CrossDomainMessageType,
    ) -> Self {
        Self {
            source_domain: source,
            target_domain: target,
            message_type: msg_type,
            payload: serde_json::json!({}),
            timestamp: chrono::Utc::now().timestamp_millis() as u64,
            correlation_id: None,
        }
    }

    pub fn with_payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = payload;
        self
    }

    pub fn with_correlation(mut self, id: impl Into<String>) -> Self {
        self.correlation_id = Some(id.into());
        self
    }
}

/// Cross-domain message types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CrossDomainMessageType {
    /// Data sharing
    DataShare,
    /// Request for data
    DataRequest,
    /// Service invocation
    ServiceCall,
    /// Service response
    ServiceResponse,
    /// Event notification
    Event,
    /// Discovery/announcement
    Discovery,
    /// Capability query
    CapabilityQuery,
}

/// Domain adapter trait for converting between domains
pub trait DomainAdapter: Send + Sync {
    /// Get source domain
    fn source_domain(&self) -> WiaDomain;

    /// Get target domain
    fn target_domain(&self) -> WiaDomain;

    /// Transform message from source to target format
    fn transform(&self, message: &serde_json::Value) -> NanoResult<serde_json::Value>;

    /// Check if transformation is supported
    fn supports(&self, message_type: &str) -> bool;
}

/// Nano to AAC adapter
pub struct NanoToAacAdapter;

impl DomainAdapter for NanoToAacAdapter {
    fn source_domain(&self) -> WiaDomain {
        WiaDomain::Nano
    }

    fn target_domain(&self) -> WiaDomain {
        WiaDomain::Aac
    }

    fn transform(&self, message: &serde_json::Value) -> NanoResult<serde_json::Value> {
        // Transform nano sensor readings to AAC input format
        // Example: Brain-computer interface sensor data -> AAC symbol selection

        let mut aac_message = serde_json::json!({
            "type": "sensor_input",
            "domain": "aac",
            "version": "1.0.0"
        });

        // Copy relevant fields
        if let Some(sensor_data) = message.get("data") {
            aac_message["input_data"] = sensor_data.clone();
        }

        if let Some(timestamp) = message.get("timestamp") {
            aac_message["timestamp"] = timestamp.clone();
        }

        Ok(aac_message)
    }

    fn supports(&self, message_type: &str) -> bool {
        matches!(message_type, "sensor_reading" | "brain_interface" | "gesture")
    }
}

/// Nano to Climate adapter
pub struct NanoToClimateAdapter;

impl DomainAdapter for NanoToClimateAdapter {
    fn source_domain(&self) -> WiaDomain {
        WiaDomain::Nano
    }

    fn target_domain(&self) -> WiaDomain {
        WiaDomain::Climate
    }

    fn transform(&self, message: &serde_json::Value) -> NanoResult<serde_json::Value> {
        // Transform nano environmental sensor data to climate format
        // Example: Nanosensor pollution readings -> Climate air quality data

        let mut climate_message = serde_json::json!({
            "type": "environmental_reading",
            "domain": "climate",
            "version": "1.0.0"
        });

        // Map sensor readings to climate parameters
        if let Some(data) = message.get("data") {
            let mut readings = serde_json::Map::new();

            // Temperature
            if let Some(temp) = data.get("temperature_k") {
                readings.insert(
                    "temperature".to_string(),
                    serde_json::json!({
                        "value": temp,
                        "unit": "K"
                    }),
                );
            }

            // pH -> water quality
            if let Some(ph) = data.get("ph") {
                readings.insert(
                    "water_quality".to_string(),
                    serde_json::json!({
                        "ph": ph
                    }),
                );
            }

            climate_message["readings"] = serde_json::Value::Object(readings);
        }

        if let Some(position) = message.get("position") {
            climate_message["location"] = position.clone();
        }

        Ok(climate_message)
    }

    fn supports(&self, message_type: &str) -> bool {
        matches!(
            message_type,
            "nanosensor" | "environmental" | "pollution" | "water_quality"
        )
    }
}

/// Nano to Health adapter
pub struct NanoToHealthAdapter;

impl DomainAdapter for NanoToHealthAdapter {
    fn source_domain(&self) -> WiaDomain {
        WiaDomain::Nano
    }

    fn target_domain(&self) -> WiaDomain {
        WiaDomain::Health
    }

    fn transform(&self, message: &serde_json::Value) -> NanoResult<serde_json::Value> {
        // Transform nanomedicine data to health records format
        // Example: Drug delivery status -> Patient treatment record

        let mut health_message = serde_json::json!({
            "type": "treatment_event",
            "domain": "health",
            "version": "1.0.0"
        });

        // Map nanomedicine data to health event
        if let Some(data) = message.get("data") {
            // Drug delivery info
            if let Some(drug) = data.get("drug_name") {
                health_message["treatment"] = serde_json::json!({
                    "medication": drug,
                    "delivery_method": "nanomedicine",
                    "status": data.get("release_status").unwrap_or(&serde_json::Value::Null)
                });
            }

            // Biomarker readings
            if let Some(readings) = data.get("sensor_readings") {
                health_message["biomarkers"] = readings.clone();
            }
        }

        if let Some(timestamp) = message.get("timestamp") {
            health_message["event_time"] = timestamp.clone();
        }

        Ok(health_message)
    }

    fn supports(&self, message_type: &str) -> bool {
        matches!(
            message_type,
            "nanomedicine" | "drug_delivery" | "biomarker" | "diagnostic"
        )
    }
}

/// Interoperability hub for managing cross-domain communication
pub struct InteropHub {
    adapters: Vec<Box<dyn DomainAdapter>>,
}

impl InteropHub {
    pub fn new() -> Self {
        Self {
            adapters: Vec::new(),
        }
    }

    /// Create with default adapters
    pub fn with_defaults() -> Self {
        let mut hub = Self::new();
        hub.register_adapter(Box::new(NanoToAacAdapter));
        hub.register_adapter(Box::new(NanoToClimateAdapter));
        hub.register_adapter(Box::new(NanoToHealthAdapter));
        hub
    }

    /// Register an adapter
    pub fn register_adapter(&mut self, adapter: Box<dyn DomainAdapter>) {
        self.adapters.push(adapter);
    }

    /// Find adapter for transformation
    pub fn find_adapter(
        &self,
        source: WiaDomain,
        target: WiaDomain,
    ) -> Option<&dyn DomainAdapter> {
        self.adapters
            .iter()
            .find(|a| a.source_domain() == source && a.target_domain() == target)
            .map(|a| a.as_ref())
    }

    /// Transform message between domains
    pub fn transform(
        &self,
        source: WiaDomain,
        target: WiaDomain,
        message: &serde_json::Value,
    ) -> NanoResult<serde_json::Value> {
        let adapter = self.find_adapter(source, target).ok_or_else(|| {
            NanoError::NotFound(format!(
                "No adapter for {} -> {}",
                source.as_str(),
                target.as_str()
            ))
        })?;

        adapter.transform(message)
    }

    /// List available transformations
    pub fn list_transformations(&self) -> Vec<(WiaDomain, WiaDomain)> {
        self.adapters
            .iter()
            .map(|a| (a.source_domain(), a.target_domain()))
            .collect()
    }
}

impl Default for InteropHub {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Capability descriptor for interoperability
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DomainCapability {
    pub domain: WiaDomain,
    pub capability_type: String,
    pub version: String,
    pub description: Option<String>,
    pub input_schema: Option<serde_json::Value>,
    pub output_schema: Option<serde_json::Value>,
}

/// Domain discovery response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DomainInfo {
    pub domain: WiaDomain,
    pub version: String,
    pub capabilities: Vec<DomainCapability>,
    pub supported_transformations: Vec<WiaDomain>,
}

impl DomainInfo {
    /// Get nano domain info
    pub fn nano() -> Self {
        Self {
            domain: WiaDomain::Nano,
            version: "1.0.0".to_string(),
            capabilities: vec![
                DomainCapability {
                    domain: WiaDomain::Nano,
                    capability_type: "molecular_assembly".to_string(),
                    version: "1.0.0".to_string(),
                    description: Some("Molecular-scale assembly operations".to_string()),
                    input_schema: None,
                    output_schema: None,
                },
                DomainCapability {
                    domain: WiaDomain::Nano,
                    capability_type: "nanosensing".to_string(),
                    version: "1.0.0".to_string(),
                    description: Some("Nanoscale sensing and measurement".to_string()),
                    input_schema: None,
                    output_schema: None,
                },
                DomainCapability {
                    domain: WiaDomain::Nano,
                    capability_type: "drug_delivery".to_string(),
                    version: "1.0.0".to_string(),
                    description: Some("Targeted drug delivery systems".to_string()),
                    input_schema: None,
                    output_schema: None,
                },
            ],
            supported_transformations: vec![WiaDomain::Aac, WiaDomain::Climate, WiaDomain::Health],
        }
    }
}
