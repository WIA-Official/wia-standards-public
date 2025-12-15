//! WIA Health Protocol Messages
//!
//! Message types and builders for the WIA Health communication protocol

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Protocol identifier
pub const PROTOCOL_NAME: &str = "wia-health";

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Message type enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    // Connection
    Connect,
    ConnectAck,
    Disconnect,
    DisconnectAck,

    // Subscription
    Subscribe,
    SubscribeAck,
    Unsubscribe,
    UnsubscribeAck,

    // Data
    Biomarker,
    ProfileUpdate,
    SimulationResult,

    // Control
    Ping,
    Pong,
    Alert,
    Error,
}

/// Base protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message {
    pub protocol: String,
    pub version: String,
    #[serde(rename = "messageId")]
    pub message_id: Uuid,
    pub timestamp: i64,
    #[serde(rename = "type")]
    pub message_type: MessageType,
    #[serde(rename = "correlationId", skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<Uuid>,
    pub payload: serde_json::Value,
}

impl Message {
    /// Create a new message with the given type and payload
    pub fn new(message_type: MessageType, payload: serde_json::Value) -> Self {
        Self {
            protocol: PROTOCOL_NAME.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4(),
            timestamp: Utc::now().timestamp_millis(),
            message_type,
            correlation_id: None,
            payload,
        }
    }

    /// Create a response message with correlation ID
    pub fn response(
        message_type: MessageType,
        correlation_id: Uuid,
        payload: serde_json::Value,
    ) -> Self {
        let mut msg = Self::new(message_type, payload);
        msg.correlation_id = Some(correlation_id);
        msg
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Deserialize from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

// ============================================================================
// Connection Payloads
// ============================================================================

/// Client authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthPayload {
    #[serde(rename = "type")]
    pub auth_type: String,
    pub token: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refresh_token: Option<String>,
}

/// Client connection options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectOptions {
    #[serde(rename = "streamRate", skip_serializing_if = "Option::is_none")]
    pub stream_rate: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compression: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub format: Option<String>,
}

impl Default for ConnectOptions {
    fn default() -> Self {
        Self {
            stream_rate: Some(60),
            compression: Some(false),
            format: Some("json".to_string()),
        }
    }
}

/// Connect message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectPayload {
    #[serde(rename = "clientId")]
    pub client_id: String,
    #[serde(rename = "clientName", skip_serializing_if = "Option::is_none")]
    pub client_name: Option<String>,
    #[serde(rename = "clientVersion", skip_serializing_if = "Option::is_none")]
    pub client_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subscriptions: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<ConnectOptions>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<AuthPayload>,
}

/// Server settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerSettings {
    #[serde(rename = "maxStreamRate")]
    pub max_stream_rate: u32,
    #[serde(rename = "keepAliveInterval")]
    pub keep_alive_interval: u64,
    #[serde(rename = "sessionTimeout")]
    pub session_timeout: u64,
}

impl Default for ServerSettings {
    fn default() -> Self {
        Self {
            max_stream_rate: 100,
            keep_alive_interval: 30000,
            session_timeout: 3600000,
        }
    }
}

/// Connect acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectAckPayload {
    #[serde(rename = "sessionId")]
    pub session_id: String,
    #[serde(rename = "serverId")]
    pub server_id: String,
    #[serde(rename = "serverVersion")]
    pub server_version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub settings: Option<ServerSettings>,
}

/// Disconnect reason codes
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum DisconnectCode {
    #[serde(rename = "1000")]
    Normal = 1000,
    #[serde(rename = "1001")]
    GoingAway = 1001,
    #[serde(rename = "1002")]
    ProtocolError = 1002,
    #[serde(rename = "1003")]
    Unsupported = 1003,
}

/// Disconnect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisconnectPayload {
    pub reason: String,
    pub code: u16,
}

// ============================================================================
// Subscription Payloads
// ============================================================================

/// Stream filter options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamFilter {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub markers: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sources: Option<Vec<String>>,
    #[serde(rename = "minQuality", skip_serializing_if = "Option::is_none")]
    pub min_quality: Option<f64>,
    #[serde(rename = "clockTypes", skip_serializing_if = "Option::is_none")]
    pub clock_types: Option<Vec<String>>,
}

/// Stream options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamOptions {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aggregation: Option<String>,
}

/// Stream subscription
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamSubscription {
    #[serde(rename = "type")]
    pub stream_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<StreamFilter>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<StreamOptions>,
}

/// Subscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribePayload {
    pub streams: Vec<StreamSubscription>,
}

/// Stream status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamStatus {
    #[serde(rename = "type")]
    pub stream_type: String,
    #[serde(rename = "streamId")]
    pub stream_id: String,
    pub status: String,
}

/// Subscribe acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribeAckPayload {
    #[serde(rename = "subscriptionId")]
    pub subscription_id: String,
    pub streams: Vec<StreamStatus>,
}

/// Unsubscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnsubscribePayload {
    #[serde(rename = "streamIds")]
    pub stream_ids: Vec<String>,
}

// ============================================================================
// Data Payloads
// ============================================================================

/// Data source information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataSource {
    pub device: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub firmware: Option<String>,
}

/// Biomarker data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiomarkerData {
    pub marker: String,
    pub value: f64,
    pub unit: String,
    pub timestamp: i64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<DataSource>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

/// Biomarker message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiomarkerPayload {
    #[serde(rename = "streamId")]
    pub stream_id: String,
    #[serde(rename = "subjectId")]
    pub subject_id: String,
    pub sequence: u64,
    pub data: BiomarkerData,
}

/// Profile update operation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum UpdateOperation {
    Create,
    Update,
    Delete,
}

/// Profile change
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileChange {
    pub path: String,
    pub operation: UpdateOperation,
    pub data: serde_json::Value,
}

/// Profile update payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileUpdatePayload {
    #[serde(rename = "subjectId")]
    pub subject_id: String,
    #[serde(rename = "updateType")]
    pub update_type: String,
    pub changes: ProfileChange,
    pub version: u64,
}

/// Simulation prediction
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationPrediction {
    pub outcome: String,
    pub value: f64,
    pub confidence: f64,
    pub timeframe: String,
}

/// Simulation recommendation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationRecommendation {
    pub intervention: String,
    #[serde(rename = "expectedBenefit")]
    pub expected_benefit: f64,
    pub confidence: f64,
}

/// Simulation results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationResults {
    pub predictions: Vec<SimulationPrediction>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recommendations: Option<Vec<SimulationRecommendation>>,
}

/// Simulation result payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationResultPayload {
    #[serde(rename = "simulationId")]
    pub simulation_id: String,
    #[serde(rename = "subjectId")]
    pub subject_id: String,
    #[serde(rename = "simulationType")]
    pub simulation_type: String,
    pub status: String,
    pub results: SimulationResults,
    pub duration: u64,
}

// ============================================================================
// Control Payloads
// ============================================================================

/// Ping payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PingPayload {
    pub timestamp: i64,
}

/// Pong payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PongPayload {
    pub timestamp: i64,
    #[serde(rename = "serverTime")]
    pub server_time: i64,
    pub latency: i64,
}

/// Alert severity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
}

/// Alert action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertAction {
    #[serde(rename = "type")]
    pub action_type: String,
    pub label: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
}

/// Alert payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertPayload {
    #[serde(rename = "alertId")]
    pub alert_id: String,
    #[serde(rename = "subjectId")]
    pub subject_id: String,
    pub severity: AlertSeverity,
    pub category: String,
    pub title: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub actions: Option<Vec<AlertAction>>,
}

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPayload {
    pub code: u16,
    pub name: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    pub recoverable: bool,
    #[serde(rename = "retryAfter", skip_serializing_if = "Option::is_none")]
    pub retry_after: Option<u64>,
}

// ============================================================================
// Message Builder
// ============================================================================

/// Message builder for creating protocol messages
pub struct MessageBuilder;

impl MessageBuilder {
    /// Create a connect message
    pub fn connect(payload: ConnectPayload) -> Message {
        Message::new(
            MessageType::Connect,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create a connect acknowledgment
    pub fn connect_ack(correlation_id: Uuid, payload: ConnectAckPayload) -> Message {
        Message::response(
            MessageType::ConnectAck,
            correlation_id,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create a disconnect message
    pub fn disconnect(reason: &str, code: u16) -> Message {
        Message::new(
            MessageType::Disconnect,
            serde_json::json!({
                "reason": reason,
                "code": code
            }),
        )
    }

    /// Create a subscribe message
    pub fn subscribe(streams: Vec<StreamSubscription>) -> Message {
        Message::new(
            MessageType::Subscribe,
            serde_json::to_value(SubscribePayload { streams }).unwrap(),
        )
    }

    /// Create a subscribe acknowledgment
    pub fn subscribe_ack(correlation_id: Uuid, payload: SubscribeAckPayload) -> Message {
        Message::response(
            MessageType::SubscribeAck,
            correlation_id,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create an unsubscribe message
    pub fn unsubscribe(stream_ids: Vec<String>) -> Message {
        Message::new(
            MessageType::Unsubscribe,
            serde_json::to_value(UnsubscribePayload { stream_ids }).unwrap(),
        )
    }

    /// Create a biomarker data message
    pub fn biomarker(payload: BiomarkerPayload) -> Message {
        Message::new(
            MessageType::Biomarker,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create a profile update message
    pub fn profile_update(payload: ProfileUpdatePayload) -> Message {
        Message::new(
            MessageType::ProfileUpdate,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create a simulation result message
    pub fn simulation_result(payload: SimulationResultPayload) -> Message {
        Message::new(
            MessageType::SimulationResult,
            serde_json::to_value(payload).unwrap(),
        )
    }

    /// Create a ping message
    pub fn ping() -> Message {
        Message::new(
            MessageType::Ping,
            serde_json::json!({
                "timestamp": Utc::now().timestamp_millis()
            }),
        )
    }

    /// Create a pong message
    pub fn pong(correlation_id: Uuid, client_timestamp: i64) -> Message {
        let server_time = Utc::now().timestamp_millis();
        Message::response(
            MessageType::Pong,
            correlation_id,
            serde_json::json!({
                "timestamp": client_timestamp,
                "serverTime": server_time,
                "latency": server_time - client_timestamp
            }),
        )
    }

    /// Create an alert message
    pub fn alert(payload: AlertPayload) -> Message {
        Message::new(MessageType::Alert, serde_json::to_value(payload).unwrap())
    }

    /// Create an error message
    pub fn error(payload: ErrorPayload) -> Message {
        Message::new(MessageType::Error, serde_json::to_value(payload).unwrap())
    }

    /// Create an error message from code and message
    pub fn error_simple(code: u16, name: &str, message: &str, recoverable: bool) -> Message {
        Self::error(ErrorPayload {
            code,
            name: name.to_string(),
            message: message.to_string(),
            details: None,
            recoverable,
            retry_after: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_creation() {
        let msg = Message::new(MessageType::Ping, serde_json::json!({"timestamp": 12345}));
        assert_eq!(msg.protocol, PROTOCOL_NAME);
        assert_eq!(msg.version, PROTOCOL_VERSION);
        assert_eq!(msg.message_type, MessageType::Ping);
    }

    #[test]
    fn test_message_serialization() {
        let msg = MessageBuilder::ping();
        let json = msg.to_json().unwrap();
        let parsed = Message::from_json(&json).unwrap();
        assert_eq!(parsed.message_type, MessageType::Ping);
    }

    #[test]
    fn test_connect_message() {
        let payload = ConnectPayload {
            client_id: "test-client".to_string(),
            client_name: Some("Test App".to_string()),
            client_version: Some("1.0.0".to_string()),
            capabilities: Some(vec!["biomarkers".to_string()]),
            subscriptions: None,
            options: Some(ConnectOptions::default()),
            auth: Some(AuthPayload {
                auth_type: "bearer".to_string(),
                token: "test-token".to_string(),
                refresh_token: None,
            }),
        };

        let msg = MessageBuilder::connect(payload);
        assert_eq!(msg.message_type, MessageType::Connect);
    }

    #[test]
    fn test_biomarker_message() {
        let payload = BiomarkerPayload {
            stream_id: "stream-001".to_string(),
            subject_id: "patient-001".to_string(),
            sequence: 1,
            data: BiomarkerData {
                marker: "heart_rate".to_string(),
                value: 72.0,
                unit: "bpm".to_string(),
                timestamp: 1702483200000,
                source: Some(DataSource {
                    device: "apple_watch".to_string(),
                    model: Some("Series 9".to_string()),
                    firmware: None,
                }),
                quality: Some(0.95),
                metadata: None,
            },
        };

        let msg = MessageBuilder::biomarker(payload);
        assert_eq!(msg.message_type, MessageType::Biomarker);
    }
}
