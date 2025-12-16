//! # Google Home Smart Home Integration
//!
//! Google Smart Home API integration.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Google Smart Home device types
pub mod google_device_types {
    pub const LIGHT: &str = "action.devices.types.LIGHT";
    pub const SWITCH: &str = "action.devices.types.SWITCH";
    pub const OUTLET: &str = "action.devices.types.OUTLET";
    pub const THERMOSTAT: &str = "action.devices.types.THERMOSTAT";
    pub const LOCK: &str = "action.devices.types.LOCK";
    pub const DOOR: &str = "action.devices.types.DOOR";
    pub const FAN: &str = "action.devices.types.FAN";
    pub const SPEAKER: &str = "action.devices.types.SPEAKER";
}

/// Google Smart Home traits
pub mod google_traits {
    pub const ON_OFF: &str = "action.devices.traits.OnOff";
    pub const BRIGHTNESS: &str = "action.devices.traits.Brightness";
    pub const COLOR_SETTING: &str = "action.devices.traits.ColorSetting";
    pub const TEMPERATURE_SETTING: &str = "action.devices.traits.TemperatureSetting";
    pub const LOCK_UNLOCK: &str = "action.devices.traits.LockUnlock";
    pub const OPEN_CLOSE: &str = "action.devices.traits.OpenClose";
    pub const FAN_SPEED: &str = "action.devices.traits.FanSpeed";
    pub const VOLUME: &str = "action.devices.traits.Volume";
}

/// Google Smart Home intent types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GoogleIntent {
    #[serde(rename = "action.devices.SYNC")]
    Sync,
    #[serde(rename = "action.devices.QUERY")]
    Query,
    #[serde(rename = "action.devices.EXECUTE")]
    Execute,
    #[serde(rename = "action.devices.DISCONNECT")]
    Disconnect,
}

/// Google Smart Home request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleRequest {
    #[serde(rename = "requestId")]
    pub request_id: String,
    pub inputs: Vec<GoogleInput>,
}

/// Google input
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleInput {
    pub intent: String,
    pub payload: Option<GooglePayload>,
}

/// Google payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GooglePayload {
    #[serde(rename = "agentUserId")]
    pub agent_user_id: Option<String>,
    pub devices: Option<Vec<GoogleDeviceQuery>>,
    pub commands: Option<Vec<GoogleCommand>>,
}

/// Device query
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleDeviceQuery {
    pub id: String,
}

/// Google command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleCommand {
    pub devices: Vec<GoogleDeviceQuery>,
    pub execution: Vec<GoogleExecution>,
}

/// Google execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleExecution {
    pub command: String,
    pub params: serde_json::Value,
}

/// Google response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleResponse {
    #[serde(rename = "requestId")]
    pub request_id: String,
    pub payload: GoogleResponsePayload,
}

/// Google response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleResponsePayload {
    #[serde(rename = "agentUserId", skip_serializing_if = "Option::is_none")]
    pub agent_user_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub devices: Option<Vec<GoogleDevice>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub commands: Option<Vec<GoogleCommandResponse>>,
}

/// Google device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleDevice {
    pub id: String,
    #[serde(rename = "type")]
    pub device_type: String,
    pub traits: Vec<String>,
    pub name: GoogleDeviceName,
    #[serde(rename = "willReportState")]
    pub will_report_state: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attributes: Option<serde_json::Value>,
}

/// Google device name
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleDeviceName {
    pub name: String,
    #[serde(rename = "defaultNames", skip_serializing_if = "Option::is_none")]
    pub default_names: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nicknames: Option<Vec<String>>,
}

/// Google command response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleCommandResponse {
    pub ids: Vec<String>,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub states: Option<serde_json::Value>,
    #[serde(rename = "errorCode", skip_serializing_if = "Option::is_none")]
    pub error_code: Option<String>,
}

/// Google Home adapter
#[derive(Debug)]
pub struct GoogleHomeAdapter {
    config: GoogleHomeConfig,
    device_mapping: HashMap<String, DeviceId>,
    device_info: HashMap<String, GoogleDevice>,
    connected: bool,
    last_sync_ms: u64,
}

/// Google Home configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleHomeConfig {
    pub project_id: String,
    pub agent_user_id: String,
    pub accessibility_features: GoogleAccessibility,
}

impl Default for GoogleHomeConfig {
    fn default() -> Self {
        Self {
            project_id: "".to_string(),
            agent_user_id: "wia-user".to_string(),
            accessibility_features: GoogleAccessibility::default(),
        }
    }
}

/// Google accessibility features
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GoogleAccessibility {
    pub slow_response: bool,
    pub confirmation_required: bool,
    pub korean_support: bool,
}

impl Default for GoogleAccessibility {
    fn default() -> Self {
        Self {
            slow_response: false,
            confirmation_required: true,
            korean_support: true,
        }
    }
}

impl GoogleHomeAdapter {
    pub fn new(config: GoogleHomeConfig) -> Self {
        Self {
            config,
            device_mapping: HashMap::new(),
            device_info: HashMap::new(),
            connected: false,
            last_sync_ms: 0,
        }
    }

    /// Register device
    pub fn register_device(&mut self, google_id: String, device_id: DeviceId, device: GoogleDevice) {
        self.device_mapping.insert(google_id.clone(), device_id);
        self.device_info.insert(google_id, device);
    }

    /// Process Google Smart Home request
    pub async fn process_google_request(&self, request: GoogleRequest) -> Result<GoogleResponse> {
        let input = request.inputs.first().ok_or_else(|| {
            SmartHomeError::ValidationError("No inputs in request".to_string())
        })?;

        let payload = match input.intent.as_str() {
            "action.devices.SYNC" => self.handle_sync(),
            "action.devices.QUERY" => self.handle_query(input.payload.as_ref()),
            "action.devices.EXECUTE" => self.handle_execute(input.payload.as_ref()),
            "action.devices.DISCONNECT" => self.handle_disconnect(),
            _ => GoogleResponsePayload {
                agent_user_id: None,
                devices: None,
                commands: Some(vec![GoogleCommandResponse {
                    ids: vec![],
                    status: "ERROR".to_string(),
                    states: None,
                    error_code: Some("notSupported".to_string()),
                }]),
            },
        };

        Ok(GoogleResponse {
            request_id: request.request_id,
            payload,
        })
    }

    fn handle_sync(&self) -> GoogleResponsePayload {
        let devices: Vec<GoogleDevice> = self.device_info.values().cloned().collect();

        GoogleResponsePayload {
            agent_user_id: Some(self.config.agent_user_id.clone()),
            devices: Some(devices),
            commands: None,
        }
    }

    fn handle_query(&self, payload: Option<&GooglePayload>) -> GoogleResponsePayload {
        let devices = payload
            .and_then(|p| p.devices.as_ref())
            .map(|devices| {
                devices
                    .iter()
                    .map(|d| GoogleCommandResponse {
                        ids: vec![d.id.clone()],
                        status: "SUCCESS".to_string(),
                        states: Some(serde_json::json!({
                            "online": true,
                            "on": true
                        })),
                        error_code: None,
                    })
                    .collect()
            })
            .unwrap_or_default();

        GoogleResponsePayload {
            agent_user_id: None,
            devices: None,
            commands: Some(devices),
        }
    }

    fn handle_execute(&self, payload: Option<&GooglePayload>) -> GoogleResponsePayload {
        let commands = payload
            .and_then(|p| p.commands.as_ref())
            .map(|commands| {
                commands
                    .iter()
                    .flat_map(|cmd| {
                        cmd.devices.iter().map(|d| {
                            // Get execution results
                            let states = cmd.execution.first().map(|exec| {
                                self.execute_command(&exec.command, &exec.params)
                            });

                            GoogleCommandResponse {
                                ids: vec![d.id.clone()],
                                status: "SUCCESS".to_string(),
                                states,
                                error_code: None,
                            }
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();

        GoogleResponsePayload {
            agent_user_id: None,
            devices: None,
            commands: Some(commands),
        }
    }

    fn handle_disconnect(&self) -> GoogleResponsePayload {
        GoogleResponsePayload {
            agent_user_id: None,
            devices: None,
            commands: None,
        }
    }

    fn execute_command(&self, command: &str, params: &serde_json::Value) -> serde_json::Value {
        match command {
            "action.devices.commands.OnOff" => {
                let on = params.get("on").and_then(|v| v.as_bool()).unwrap_or(false);
                serde_json::json!({ "on": on, "online": true })
            }
            "action.devices.commands.BrightnessAbsolute" => {
                let brightness = params.get("brightness").and_then(|v| v.as_u64()).unwrap_or(50);
                serde_json::json!({ "brightness": brightness, "online": true })
            }
            "action.devices.commands.ThermostatTemperatureSetpoint" => {
                let temp = params
                    .get("thermostatTemperatureSetpoint")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(22.0);
                serde_json::json!({
                    "thermostatTemperatureSetpoint": temp,
                    "online": true
                })
            }
            "action.devices.commands.LockUnlock" => {
                let lock = params.get("lock").and_then(|v| v.as_bool()).unwrap_or(true);
                serde_json::json!({ "isLocked": lock, "online": true })
            }
            _ => serde_json::json!({ "online": true }),
        }
    }

    /// Convert Google command to unified command
    pub fn execution_to_command(
        &self,
        device_id: &str,
        execution: &GoogleExecution,
    ) -> Option<UnifiedCommand> {
        let wia_device_id = self.device_mapping.get(device_id)?;

        let action = match execution.command.as_str() {
            "action.devices.commands.OnOff" => {
                let on = execution.params.get("on").and_then(|v| v.as_bool()).unwrap_or(false);
                DeviceAction::Power { on }
            }
            "action.devices.commands.BrightnessAbsolute" => {
                let level = execution
                    .params
                    .get("brightness")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(50) as u8;
                DeviceAction::Brightness { level }
            }
            "action.devices.commands.ThermostatTemperatureSetpoint" => {
                let celsius = execution
                    .params
                    .get("thermostatTemperatureSetpoint")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(22.0) as f32;
                DeviceAction::Temperature { celsius }
            }
            "action.devices.commands.LockUnlock" => {
                let locked = execution
                    .params
                    .get("lock")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(true);
                DeviceAction::Lock { locked }
            }
            _ => return None,
        };

        Some(UnifiedCommand::new(
            CommandSource::GoogleHome {
                trait_name: execution.command.clone(),
                params: execution.params.clone(),
            },
            CommandTarget::Device(*wia_device_id),
            action,
        ))
    }

    /// Get status
    pub fn get_status(&self) -> PlatformStatus {
        PlatformStatus {
            platform: ExternalPlatform::GoogleHome,
            connected: self.connected,
            last_sync_ms: self.last_sync_ms,
            device_count: self.device_mapping.len() as u32,
            error: None,
        }
    }

    /// Process external request
    pub async fn process_request(&self, request: ExternalRequest) -> Result<ExternalResponse> {
        let google_request: GoogleRequest = serde_json::from_value(request.payload)
            .map_err(|e| SmartHomeError::ValidationError(e.to_string()))?;

        let response = self.process_google_request(google_request).await?;

        Ok(ExternalResponse::success(serde_json::to_value(response).unwrap()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_google_adapter_creation() {
        let adapter = GoogleHomeAdapter::new(GoogleHomeConfig::default());
        assert!(!adapter.connected);
    }

    #[test]
    fn test_device_registration() {
        let mut adapter = GoogleHomeAdapter::new(GoogleHomeConfig::default());

        let device_id = Uuid::new_v4();
        adapter.register_device(
            "google-light-1".to_string(),
            device_id,
            GoogleDevice {
                id: "google-light-1".to_string(),
                device_type: google_device_types::LIGHT.to_string(),
                traits: vec![google_traits::ON_OFF.to_string()],
                name: GoogleDeviceName {
                    name: "Living Room Light".to_string(),
                    default_names: None,
                    nicknames: None,
                },
                will_report_state: true,
                attributes: None,
            },
        );

        assert!(adapter.device_mapping.contains_key("google-light-1"));
    }

    #[tokio::test]
    async fn test_sync_request() {
        let mut adapter = GoogleHomeAdapter::new(GoogleHomeConfig::default());

        let device_id = Uuid::new_v4();
        adapter.register_device(
            "test-device".to_string(),
            device_id,
            GoogleDevice {
                id: "test-device".to_string(),
                device_type: google_device_types::LIGHT.to_string(),
                traits: vec![google_traits::ON_OFF.to_string()],
                name: GoogleDeviceName {
                    name: "Test".to_string(),
                    default_names: None,
                    nicknames: None,
                },
                will_report_state: true,
                attributes: None,
            },
        );

        let request = GoogleRequest {
            request_id: "test-123".to_string(),
            inputs: vec![GoogleInput {
                intent: "action.devices.SYNC".to_string(),
                payload: None,
            }],
        };

        let response = adapter.process_google_request(request).await.unwrap();
        assert!(response.payload.devices.is_some());
        assert_eq!(response.payload.devices.unwrap().len(), 1);
    }

    #[test]
    fn test_execute_command() {
        let adapter = GoogleHomeAdapter::new(GoogleHomeConfig::default());

        let result = adapter.execute_command(
            "action.devices.commands.OnOff",
            &serde_json::json!({ "on": true }),
        );

        assert_eq!(result.get("on").unwrap(), true);
    }
}
