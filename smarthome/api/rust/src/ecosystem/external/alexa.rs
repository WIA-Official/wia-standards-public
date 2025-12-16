//! # Amazon Alexa Smart Home Integration
//!
//! Alexa Smart Home Skill API integration.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Alexa Smart Home namespaces
pub mod alexa_namespace {
    pub const DISCOVERY: &str = "Alexa.Discovery";
    pub const POWER_CONTROLLER: &str = "Alexa.PowerController";
    pub const BRIGHTNESS_CONTROLLER: &str = "Alexa.BrightnessController";
    pub const COLOR_CONTROLLER: &str = "Alexa.ColorController";
    pub const THERMOSTAT_CONTROLLER: &str = "Alexa.ThermostatController";
    pub const LOCK_CONTROLLER: &str = "Alexa.LockController";
    pub const SPEAKER: &str = "Alexa.Speaker";
}

/// Alexa directive header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaHeader {
    pub namespace: String,
    pub name: String,
    #[serde(rename = "messageId")]
    pub message_id: String,
    #[serde(rename = "correlationToken")]
    pub correlation_token: Option<String>,
    #[serde(rename = "payloadVersion")]
    pub payload_version: String,
}

/// Alexa endpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaEndpoint {
    #[serde(rename = "endpointId")]
    pub endpoint_id: String,
    pub scope: Option<AlexaScope>,
}

/// Alexa scope for authorization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaScope {
    #[serde(rename = "type")]
    pub scope_type: String,
    pub token: String,
}

/// Alexa directive
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaDirective {
    pub header: AlexaHeader,
    pub endpoint: Option<AlexaEndpoint>,
    pub payload: serde_json::Value,
}

/// Alexa response event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaEvent {
    pub header: AlexaHeader,
    pub endpoint: Option<AlexaEndpoint>,
    pub payload: serde_json::Value,
}

/// Alexa adapter for smart home
#[derive(Debug)]
pub struct AlexaAdapter {
    config: AlexaConfig,
    device_mapping: HashMap<String, DeviceId>,
    capability_mapping: HashMap<DeviceId, Vec<String>>,
    connected: bool,
    last_sync_ms: u64,
}

/// Alexa adapter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaConfig {
    pub skill_id: String,
    pub client_id: String,
    pub client_secret: String,
    pub accessibility_features: AlexaAccessibility,
}

impl Default for AlexaConfig {
    fn default() -> Self {
        Self {
            skill_id: "".to_string(),
            client_id: "".to_string(),
            client_secret: "".to_string(),
            accessibility_features: AlexaAccessibility::default(),
        }
    }
}

/// Alexa accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlexaAccessibility {
    /// Slow down speech rate
    pub slow_speech: bool,
    /// Require explicit confirmation
    pub confirmation_required: bool,
    /// Enable Korean language
    pub korean_support: bool,
    /// Extended timeout for responses
    pub extended_timeout: bool,
}

impl Default for AlexaAccessibility {
    fn default() -> Self {
        Self {
            slow_speech: false,
            confirmation_required: true,
            korean_support: true,
            extended_timeout: false,
        }
    }
}

impl AlexaAdapter {
    pub fn new(config: AlexaConfig) -> Self {
        Self {
            config,
            device_mapping: HashMap::new(),
            capability_mapping: HashMap::new(),
            connected: false,
            last_sync_ms: 0,
        }
    }

    /// Register device with Alexa
    pub fn register_device(
        &mut self,
        alexa_id: String,
        device_id: DeviceId,
        capabilities: Vec<String>,
    ) {
        self.device_mapping.insert(alexa_id, device_id);
        self.capability_mapping.insert(device_id, capabilities);
    }

    /// Process Alexa directive
    pub async fn process_directive(&self, directive: AlexaDirective) -> Result<AlexaEvent> {
        let namespace = &directive.header.namespace;
        let name = &directive.header.name;

        match namespace.as_str() {
            alexa_namespace::DISCOVERY => self.handle_discovery(&directive),
            alexa_namespace::POWER_CONTROLLER => self.handle_power(&directive, name),
            alexa_namespace::BRIGHTNESS_CONTROLLER => self.handle_brightness(&directive, name),
            alexa_namespace::THERMOSTAT_CONTROLLER => self.handle_thermostat(&directive, name),
            alexa_namespace::LOCK_CONTROLLER => self.handle_lock(&directive, name),
            _ => self.create_error_response(&directive, "INVALID_DIRECTIVE", "Unsupported namespace"),
        }
    }

    fn handle_discovery(&self, directive: &AlexaDirective) -> Result<AlexaEvent> {
        let endpoints: Vec<serde_json::Value> = self
            .device_mapping
            .iter()
            .map(|(alexa_id, device_id)| {
                let capabilities = self
                    .capability_mapping
                    .get(device_id)
                    .cloned()
                    .unwrap_or_default();

                serde_json::json!({
                    "endpointId": alexa_id,
                    "manufacturerName": "WIA",
                    "friendlyName": format!("Device {}", alexa_id),
                    "description": "WIA Smart Home Device",
                    "displayCategories": ["LIGHT"],
                    "capabilities": capabilities.iter().map(|cap| {
                        serde_json::json!({
                            "type": "AlexaInterface",
                            "interface": cap,
                            "version": "3"
                        })
                    }).collect::<Vec<_>>()
                })
            })
            .collect();

        Ok(AlexaEvent {
            header: AlexaHeader {
                namespace: "Alexa.Discovery".to_string(),
                name: "Discover.Response".to_string(),
                message_id: uuid::Uuid::new_v4().to_string(),
                correlation_token: directive.header.correlation_token.clone(),
                payload_version: "3".to_string(),
            },
            endpoint: None,
            payload: serde_json::json!({ "endpoints": endpoints }),
        })
    }

    fn handle_power(&self, directive: &AlexaDirective, name: &str) -> Result<AlexaEvent> {
        let on = match name {
            "TurnOn" => true,
            "TurnOff" => false,
            _ => return self.create_error_response(directive, "INVALID_DIRECTIVE", "Unknown power directive"),
        };

        Ok(self.create_response(
            directive,
            "Alexa",
            "Response",
            serde_json::json!({
                "properties": [{
                    "namespace": "Alexa.PowerController",
                    "name": "powerState",
                    "value": if on { "ON" } else { "OFF" },
                    "timeOfSample": chrono::Utc::now().to_rfc3339(),
                    "uncertaintyInMilliseconds": 0
                }]
            }),
        ))
    }

    fn handle_brightness(&self, directive: &AlexaDirective, name: &str) -> Result<AlexaEvent> {
        let brightness = directive
            .payload
            .get("brightness")
            .and_then(|v| v.as_u64())
            .unwrap_or(50) as u8;

        Ok(self.create_response(
            directive,
            "Alexa",
            "Response",
            serde_json::json!({
                "properties": [{
                    "namespace": "Alexa.BrightnessController",
                    "name": "brightness",
                    "value": brightness,
                    "timeOfSample": chrono::Utc::now().to_rfc3339(),
                    "uncertaintyInMilliseconds": 0
                }]
            }),
        ))
    }

    fn handle_thermostat(&self, directive: &AlexaDirective, name: &str) -> Result<AlexaEvent> {
        let target_temp = directive
            .payload
            .get("targetSetpoint")
            .and_then(|v| v.get("value"))
            .and_then(|v| v.as_f64())
            .unwrap_or(22.0);

        Ok(self.create_response(
            directive,
            "Alexa",
            "Response",
            serde_json::json!({
                "properties": [{
                    "namespace": "Alexa.ThermostatController",
                    "name": "targetSetpoint",
                    "value": {
                        "value": target_temp,
                        "scale": "CELSIUS"
                    },
                    "timeOfSample": chrono::Utc::now().to_rfc3339(),
                    "uncertaintyInMilliseconds": 0
                }]
            }),
        ))
    }

    fn handle_lock(&self, directive: &AlexaDirective, name: &str) -> Result<AlexaEvent> {
        let locked = match name {
            "Lock" => "LOCKED",
            "Unlock" => "UNLOCKED",
            _ => return self.create_error_response(directive, "INVALID_DIRECTIVE", "Unknown lock directive"),
        };

        Ok(self.create_response(
            directive,
            "Alexa",
            "Response",
            serde_json::json!({
                "properties": [{
                    "namespace": "Alexa.LockController",
                    "name": "lockState",
                    "value": locked,
                    "timeOfSample": chrono::Utc::now().to_rfc3339(),
                    "uncertaintyInMilliseconds": 0
                }]
            }),
        ))
    }

    fn create_response(
        &self,
        directive: &AlexaDirective,
        namespace: &str,
        name: &str,
        payload: serde_json::Value,
    ) -> AlexaEvent {
        AlexaEvent {
            header: AlexaHeader {
                namespace: namespace.to_string(),
                name: name.to_string(),
                message_id: uuid::Uuid::new_v4().to_string(),
                correlation_token: directive.header.correlation_token.clone(),
                payload_version: "3".to_string(),
            },
            endpoint: directive.endpoint.clone(),
            payload,
        }
    }

    fn create_error_response(
        &self,
        directive: &AlexaDirective,
        error_type: &str,
        message: &str,
    ) -> Result<AlexaEvent> {
        Ok(AlexaEvent {
            header: AlexaHeader {
                namespace: "Alexa".to_string(),
                name: "ErrorResponse".to_string(),
                message_id: uuid::Uuid::new_v4().to_string(),
                correlation_token: directive.header.correlation_token.clone(),
                payload_version: "3".to_string(),
            },
            endpoint: directive.endpoint.clone(),
            payload: serde_json::json!({
                "type": error_type,
                "message": message
            }),
        })
    }

    /// Convert Alexa directive to unified command
    pub fn directive_to_command(&self, directive: &AlexaDirective) -> Option<UnifiedCommand> {
        let endpoint = directive.endpoint.as_ref()?;
        let device_id = self.device_mapping.get(&endpoint.endpoint_id)?;

        let action = match directive.header.namespace.as_str() {
            alexa_namespace::POWER_CONTROLLER => {
                let on = directive.header.name == "TurnOn";
                DeviceAction::Power { on }
            }
            alexa_namespace::BRIGHTNESS_CONTROLLER => {
                let level = directive
                    .payload
                    .get("brightness")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(50) as u8;
                DeviceAction::Brightness { level }
            }
            alexa_namespace::LOCK_CONTROLLER => {
                let locked = directive.header.name == "Lock";
                DeviceAction::Lock { locked }
            }
            _ => return None,
        };

        Some(UnifiedCommand::new(
            CommandSource::Alexa {
                intent: directive.header.name.clone(),
                slots: HashMap::new(),
            },
            CommandTarget::Device(*device_id),
            action,
        ))
    }

    /// Get status
    pub fn get_status(&self) -> PlatformStatus {
        PlatformStatus {
            platform: ExternalPlatform::Alexa,
            connected: self.connected,
            last_sync_ms: self.last_sync_ms,
            device_count: self.device_mapping.len() as u32,
            error: None,
        }
    }

    /// Process external request
    pub async fn process_request(&self, request: ExternalRequest) -> Result<ExternalResponse> {
        // Parse directive from payload
        let directive: AlexaDirective = serde_json::from_value(request.payload)
            .map_err(|e| SmartHomeError::ValidationError(e.to_string()))?;

        let event = self.process_directive(directive).await?;

        Ok(ExternalResponse::success(serde_json::to_value(event).unwrap()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alexa_adapter_creation() {
        let adapter = AlexaAdapter::new(AlexaConfig::default());
        assert!(!adapter.connected);
    }

    #[test]
    fn test_device_registration() {
        let mut adapter = AlexaAdapter::new(AlexaConfig::default());

        let device_id = Uuid::new_v4();
        adapter.register_device(
            "alexa-light-1".to_string(),
            device_id,
            vec!["Alexa.PowerController".to_string()],
        );

        assert!(adapter.device_mapping.contains_key("alexa-light-1"));
    }

    #[tokio::test]
    async fn test_power_directive() {
        let adapter = AlexaAdapter::new(AlexaConfig::default());

        let directive = AlexaDirective {
            header: AlexaHeader {
                namespace: alexa_namespace::POWER_CONTROLLER.to_string(),
                name: "TurnOn".to_string(),
                message_id: "test".to_string(),
                correlation_token: Some("token".to_string()),
                payload_version: "3".to_string(),
            },
            endpoint: Some(AlexaEndpoint {
                endpoint_id: "test-device".to_string(),
                scope: None,
            }),
            payload: serde_json::json!({}),
        };

        let response = adapter.process_directive(directive).await.unwrap();
        assert_eq!(response.header.name, "Response");
    }
}
