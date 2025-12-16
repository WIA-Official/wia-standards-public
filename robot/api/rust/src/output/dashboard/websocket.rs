//! WebSocket dashboard streaming adapter

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use chrono::Utc;
use serde::{Deserialize, Serialize};

/// Dashboard message type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DashboardMessageType {
    Telemetry,
    Status,
    Alert,
    Command,
    Config,
}

/// Dashboard telemetry message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DashboardMessage {
    /// Message type
    #[serde(rename = "type")]
    pub message_type: DashboardMessageType,
    /// Device ID
    pub device_id: String,
    /// Timestamp (ISO 8601)
    pub timestamp: String,
    /// Data payload
    pub data: DashboardData,
}

impl DashboardMessage {
    /// Create a new telemetry message
    pub fn telemetry(device_id: &str, data: DashboardData) -> Self {
        Self {
            message_type: DashboardMessageType::Telemetry,
            device_id: device_id.to_string(),
            timestamp: Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
            data,
        }
    }

    /// Create a status message
    pub fn status(device_id: &str, data: DashboardData) -> Self {
        Self {
            message_type: DashboardMessageType::Status,
            device_id: device_id.to_string(),
            timestamp: Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
            data,
        }
    }

    /// Create an alert message
    pub fn alert(device_id: &str, data: DashboardData) -> Self {
        Self {
            message_type: DashboardMessageType::Alert,
            device_id: device_id.to_string(),
            timestamp: Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
            data,
        }
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string(self)?)
    }

    /// Convert to pretty JSON
    pub fn to_json_pretty(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// Dashboard data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DashboardData {
    /// Robot status
    pub status: String,
    /// Battery level (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub battery: Option<u8>,
    /// Uptime in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uptime: Option<u64>,
    /// Joint data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub joints: Option<Vec<JointTelemetry>>,
    /// Gait data (for exoskeletons)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gait: Option<GaitTelemetry>,
    /// Safety data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety: Option<SafetyTelemetry>,
    /// Vital signs (for care robots)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vitals: Option<VitalsTelemetry>,
    /// Additional metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<serde_json::Value>,
}

impl Default for DashboardData {
    fn default() -> Self {
        Self {
            status: "unknown".to_string(),
            battery: None,
            uptime: None,
            joints: None,
            gait: None,
            safety: None,
            vitals: None,
            metrics: None,
        }
    }
}

impl DashboardData {
    /// Create a new dashboard data payload
    pub fn new(status: &str) -> Self {
        Self {
            status: status.to_string(),
            ..Default::default()
        }
    }

    /// Set battery level
    pub fn with_battery(mut self, battery: u8) -> Self {
        self.battery = Some(battery.min(100));
        self
    }

    /// Set uptime
    pub fn with_uptime(mut self, uptime: u64) -> Self {
        self.uptime = Some(uptime);
        self
    }

    /// Set joint data
    pub fn with_joints(mut self, joints: Vec<JointTelemetry>) -> Self {
        self.joints = Some(joints);
        self
    }

    /// Set gait data
    pub fn with_gait(mut self, gait: GaitTelemetry) -> Self {
        self.gait = Some(gait);
        self
    }

    /// Set safety data
    pub fn with_safety(mut self, safety: SafetyTelemetry) -> Self {
        self.safety = Some(safety);
        self
    }

    /// Set vitals data
    pub fn with_vitals(mut self, vitals: VitalsTelemetry) -> Self {
        self.vitals = Some(vitals);
        self
    }

    /// Set additional metrics
    pub fn with_metrics(mut self, metrics: serde_json::Value) -> Self {
        self.metrics = Some(metrics);
        self
    }
}

/// Joint telemetry data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointTelemetry {
    pub name: String,
    pub angle: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub torque: Option<f64>,
}

impl JointTelemetry {
    pub fn new(name: &str, angle: f64) -> Self {
        Self {
            name: name.to_string(),
            angle,
            velocity: None,
            torque: None,
        }
    }

    pub fn with_velocity(mut self, velocity: f64) -> Self {
        self.velocity = Some(velocity);
        self
    }

    pub fn with_torque(mut self, torque: f64) -> Self {
        self.torque = Some(torque);
        self
    }
}

/// Gait telemetry data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GaitTelemetry {
    pub phase: String,
    pub velocity: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cadence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stride_length: Option<f64>,
}

impl GaitTelemetry {
    pub fn new(phase: &str, velocity: f64) -> Self {
        Self {
            phase: phase.to_string(),
            velocity,
            cadence: None,
            stride_length: None,
        }
    }
}

/// Safety telemetry data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyTelemetry {
    pub level: String,
    pub score: u8,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alerts: Option<Vec<String>>,
}

impl SafetyTelemetry {
    pub fn new(level: &str, score: u8) -> Self {
        Self {
            level: level.to_string(),
            score,
            alerts: None,
        }
    }

    pub fn with_alerts(mut self, alerts: Vec<String>) -> Self {
        self.alerts = Some(alerts);
        self
    }
}

/// Vital signs telemetry
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct VitalsTelemetry {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heart_rate: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub blood_pressure: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spo2: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature: Option<f64>,
}

impl Default for VitalsTelemetry {
    fn default() -> Self {
        Self {
            heart_rate: None,
            blood_pressure: None,
            spo2: None,
            temperature: None,
        }
    }
}

/// Dashboard adapter for WebSocket streaming
pub struct DashboardAdapter {
    base: BaseAdapter,
    ws_url: Option<String>,
    message_buffer: Vec<DashboardMessage>,
}

impl Default for DashboardAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl DashboardAdapter {
    /// Create a new dashboard adapter
    pub fn new() -> Self {
        Self {
            base: BaseAdapter::new("dashboard", OutputType::Dashboard),
            ws_url: None,
            message_buffer: Vec::new(),
        }
    }

    /// Set WebSocket URL
    pub fn with_url(mut self, url: &str) -> Self {
        self.ws_url = Some(url.to_string());
        self
    }

    /// Get buffered messages
    pub fn get_buffer(&self) -> &[DashboardMessage] {
        &self.message_buffer
    }

    /// Clear message buffer
    pub fn clear_buffer(&mut self) {
        self.message_buffer.clear();
    }

    /// Buffer size
    pub fn buffer_size(&self) -> usize {
        self.message_buffer.len()
    }

    /// Convert robot data to dashboard message
    pub fn robot_to_dashboard(&self, data: &OutputData) -> DashboardMessage {
        let mut dashboard_data = DashboardData::new("operational");

        // Extract status
        if let Some(status) = data.data.get("status").and_then(|s| s.as_str()) {
            dashboard_data.status = status.to_string();
        }

        // Extract battery
        if let Some(battery) = data.data.get("battery_percent").and_then(|b| b.as_u64()) {
            dashboard_data.battery = Some(battery.min(100) as u8);
        }

        // Extract joints
        if let Some(joints) = data.data.get("joints").and_then(|j| j.as_array()) {
            let joint_telemetry: Vec<JointTelemetry> = joints
                .iter()
                .filter_map(|j| {
                    let name = j.get("name").and_then(|n| n.as_str())?;
                    let angle = j.get("angle_deg").and_then(|a| a.as_f64())?;
                    let mut jt = JointTelemetry::new(name, angle);
                    if let Some(v) = j.get("velocity_deg_s").and_then(|v| v.as_f64()) {
                        jt = jt.with_velocity(v);
                    }
                    if let Some(t) = j.get("torque_nm").and_then(|t| t.as_f64()) {
                        jt = jt.with_torque(t);
                    }
                    Some(jt)
                })
                .collect();

            if !joint_telemetry.is_empty() {
                dashboard_data.joints = Some(joint_telemetry);
            }
        }

        // Extract gait data
        if let Some(gait) = data.data.get("gait") {
            if let (Some(phase), Some(velocity)) = (
                gait.get("phase").and_then(|p| p.as_str()),
                gait.get("velocity_m_s").and_then(|v| v.as_f64()),
            ) {
                dashboard_data.gait = Some(GaitTelemetry::new(phase, velocity));
            }
        }

        // Extract safety data
        if let Some(safety) = data.data.get("safety") {
            if let Some(level) = safety.get("level").and_then(|l| l.as_str()) {
                let score = safety.get("score").and_then(|s| s.as_u64()).unwrap_or(100) as u8;
                dashboard_data.safety = Some(SafetyTelemetry::new(level, score));
            }
        }

        // Extract vitals
        if let Some(vitals) = data.data.get("vital_signs") {
            let mut vitals_telemetry = VitalsTelemetry::default();
            if let Some(hr) = vitals.get("heart_rate_bpm").and_then(|h| h.as_u64()) {
                vitals_telemetry.heart_rate = Some(hr as u16);
            }
            if let Some(spo2) = vitals.get("spo2_percent").and_then(|s| s.as_u64()) {
                vitals_telemetry.spo2 = Some(spo2 as u8);
            }
            if let Some(temp) = vitals.get("body_temp_c").and_then(|t| t.as_f64()) {
                vitals_telemetry.temperature = Some(temp);
            }
            dashboard_data.vitals = Some(vitals_telemetry);
        }

        DashboardMessage::telemetry(&data.device_id, dashboard_data)
    }
}

impl OutputAdapter for DashboardAdapter {
    fn output_type(&self) -> OutputType {
        OutputType::Dashboard
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(url) = config.get_string_option("ws_url") {
            self.ws_url = Some(url.to_string());
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let message = self.robot_to_dashboard(data);
        let json = message.to_json()?;

        Ok(OutputResult::success("Created dashboard message")
            .with_metadata(serde_json::json!({
                "message_type": "telemetry",
                "device_id": data.device_id,
                "output": json
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
    fn test_dashboard_message_creation() {
        let data = DashboardData::new("operational")
            .with_battery(85)
            .with_uptime(3600);

        let message = DashboardMessage::telemetry("exo-001", data);

        assert_eq!(message.message_type, DashboardMessageType::Telemetry);
        assert_eq!(message.device_id, "exo-001");
    }

    #[test]
    fn test_dashboard_data_serialization() {
        let joints = vec![
            JointTelemetry::new("hip_left", 15.5).with_torque(45.2),
            JointTelemetry::new("knee_left", 30.0).with_velocity(5.0),
        ];

        let data = DashboardData::new("operational")
            .with_battery(85)
            .with_joints(joints)
            .with_safety(SafetyTelemetry::new("normal", 95));

        let message = DashboardMessage::telemetry("exo-001", data);
        let json = message.to_json().unwrap();

        assert!(json.contains("\"type\":\"telemetry\""));
        assert!(json.contains("\"deviceId\":\"exo-001\""));
        assert!(json.contains("\"battery\":85"));
    }

    #[test]
    fn test_dashboard_adapter() {
        let adapter = DashboardAdapter::new();

        assert_eq!(adapter.output_type(), OutputType::Dashboard);
        assert_eq!(adapter.name(), "dashboard");
    }

    #[test]
    fn test_robot_to_dashboard() {
        let adapter = DashboardAdapter::new();

        let data = OutputData::new("exo-001", "exoskeleton")
            .with_data(serde_json::json!({
                "status": "active",
                "battery_percent": 75,
                "joints": [
                    {"name": "hip_left", "angle_deg": 15.5, "torque_nm": 45.2}
                ],
                "safety": {
                    "level": "normal",
                    "score": 95
                }
            }));

        let message = adapter.robot_to_dashboard(&data);

        assert_eq!(message.device_id, "exo-001");
        assert_eq!(message.data.status, "active");
        assert_eq!(message.data.battery, Some(75));
    }

    #[test]
    fn test_dashboard_output() {
        let adapter = DashboardAdapter::new();

        let data = OutputData::new("care-001", "care_robot")
            .with_data(serde_json::json!({
                "status": "monitoring",
                "vital_signs": {
                    "heart_rate_bpm": 72,
                    "spo2_percent": 98,
                    "body_temp_c": 36.5
                }
            }));

        let result = adapter.output(&data).unwrap();
        assert!(result.success);
    }
}
