//! WPP Message Types
//!
//! Defines all message types for the WIA Physics Protocol.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::{PhysicsError, PhysicsResult};
use crate::types::*;

// ============================================================================
// Message Envelope
// ============================================================================

/// WPP Message envelope - wraps all protocol messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WppMessage {
    /// Protocol version
    pub wpp: String,
    /// Unique message ID
    pub id: String,
    /// Message type
    #[serde(rename = "type")]
    pub msg_type: MessageType,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Message payload
    pub payload: MessagePayload,
    /// Optional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

impl WppMessage {
    /// Create a new message with auto-generated ID and timestamp
    pub fn new(msg_type: MessageType, payload: MessagePayload) -> Self {
        Self {
            wpp: super::WPP_VERSION.to_string(),
            id: Uuid::new_v4().to_string(),
            msg_type,
            timestamp: Utc::now(),
            payload,
            metadata: None,
        }
    }

    /// Create a connect message
    pub fn connect(client_id: &str, client_name: &str) -> Self {
        Self::new(
            MessageType::Connect,
            MessagePayload::Connect(ConnectPayload {
                client_id: client_id.to_string(),
                client_name: Some(client_name.to_string()),
                client_version: Some(env!("CARGO_PKG_VERSION").to_string()),
                capabilities: vec![
                    Capability::Streaming,
                    Capability::Commands,
                    Capability::Events,
                ],
                auth: None,
                options: Some(ConnectionOptions::default()),
            }),
        )
    }

    /// Create a subscribe message
    pub fn subscribe(channel: &str) -> Self {
        Self::new(
            MessageType::Subscribe,
            MessagePayload::Subscribe(SubscribePayload {
                channel: channel.to_string(),
                filter: None,
                options: None,
            }),
        )
    }

    /// Create an unsubscribe message
    pub fn unsubscribe(channel: &str) -> Self {
        Self::new(
            MessageType::Unsubscribe,
            MessagePayload::Unsubscribe(UnsubscribePayload {
                channel: Some(channel.to_string()),
                subscription_id: None,
            }),
        )
    }

    /// Create a command message
    pub fn command(target: &str, action: &str, parameters: serde_json::Value) -> Self {
        Self::new(
            MessageType::Command,
            MessagePayload::Command(CommandPayload {
                target: target.to_string(),
                action: action.to_string(),
                parameters: Some(parameters),
                timeout: Some(super::DEFAULT_COMMAND_TIMEOUT),
                priority: Some(Priority::Normal),
            }),
        )
    }

    /// Create a ping message
    pub fn ping() -> Self {
        Self::new(MessageType::Ping, MessagePayload::Ping(PingPayload {}))
    }

    /// Create a pong message
    pub fn pong(ping_id: &str, latency_ms: u64) -> Self {
        Self::new(
            MessageType::Pong,
            MessagePayload::Pong(PongPayload {
                ping_id: ping_id.to_string(),
                latency_ms: Some(latency_ms),
            }),
        )
    }

    /// Create a disconnect message
    pub fn disconnect(reason: DisconnectReason, message: Option<&str>) -> Self {
        Self::new(
            MessageType::Disconnect,
            MessagePayload::Disconnect(DisconnectPayload {
                reason,
                message: message.map(|s| s.to_string()),
            }),
        )
    }

    /// Create an error message
    pub fn error(code: u32, category: ErrorCategory, message: &str) -> Self {
        Self::new(
            MessageType::Error,
            MessagePayload::Error(ErrorPayload {
                code,
                category,
                message: message.to_string(),
                details: None,
                related_message_id: None,
            }),
        )
    }

    /// Create a data message
    pub fn data<T: Serialize>(channel: &str, data_type: DataType, data: &T, sequence: u64) -> PhysicsResult<Self> {
        let data_value = serde_json::to_value(data)
            .map_err(|e| PhysicsError::serialization(e.to_string()))?;

        Ok(Self::new(
            MessageType::Data,
            MessagePayload::Data(DataPayload {
                channel: channel.to_string(),
                sequence: Some(sequence),
                data_type,
                data: Some(data_value),
                batch: None,
                sequence_start: None,
                count: None,
                items: None,
            }),
        ))
    }

    /// Parse a JSON string into a message
    pub fn from_json(json: &str) -> PhysicsResult<Self> {
        serde_json::from_str(json).map_err(|e| PhysicsError::parse(e.to_string()))
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> PhysicsResult<String> {
        serde_json::to_string(self).map_err(|e| PhysicsError::serialization(e.to_string()))
    }

    /// Serialize to pretty JSON
    pub fn to_json_pretty(&self) -> PhysicsResult<String> {
        serde_json::to_string_pretty(self).map_err(|e| PhysicsError::serialization(e.to_string()))
    }
}

// ============================================================================
// Message Types
// ============================================================================

/// Message types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Connect,
    ConnectAck,
    Disconnect,
    Subscribe,
    SubscribeAck,
    Unsubscribe,
    UnsubscribeAck,
    Data,
    Command,
    Response,
    Event,
    Error,
    Ping,
    Pong,
}

// ============================================================================
// Message Payloads
// ============================================================================

/// Union of all message payloads
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MessagePayload {
    Connect(ConnectPayload),
    ConnectAck(ConnectAckPayload),
    Disconnect(DisconnectPayload),
    Subscribe(SubscribePayload),
    SubscribeAck(SubscribeAckPayload),
    Unsubscribe(UnsubscribePayload),
    UnsubscribeAck(UnsubscribeAckPayload),
    Data(DataPayload),
    Command(CommandPayload),
    Response(ResponsePayload),
    Event(EventPayload),
    Error(ErrorPayload),
    Ping(PingPayload),
    Pong(PongPayload),
}

// ============================================================================
// Connection Messages
// ============================================================================

/// Connect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectPayload {
    pub client_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_version: Option<String>,
    #[serde(default)]
    pub capabilities: Vec<Capability>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<AuthInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<ConnectionOptions>,
}

/// Connect acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectAckPayload {
    pub session_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_version: Option<String>,
    #[serde(default)]
    pub capabilities: Vec<Capability>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heartbeat_interval: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_subscriptions: Option<u32>,
    #[serde(default)]
    pub channels: Vec<ChannelInfo>,
}

/// Disconnect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisconnectPayload {
    pub reason: DisconnectReason,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// Disconnect reasons
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisconnectReason {
    ClientShutdown,
    ServerShutdown,
    Timeout,
    Error,
    AuthFailure,
}

/// Client/server capabilities
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Capability {
    Streaming,
    Commands,
    Events,
    Binary,
}

/// Authentication info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthInfo {
    pub method: AuthMethod,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub api_key: Option<String>,
}

/// Authentication methods
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthMethod {
    Token,
    ApiKey,
    Certificate,
}

/// Connection options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionOptions {
    #[serde(default = "default_heartbeat_interval")]
    pub heartbeat_interval: u64,
    #[serde(default)]
    pub compression: Compression,
    #[serde(default)]
    pub binary_mode: bool,
}

impl Default for ConnectionOptions {
    fn default() -> Self {
        Self {
            heartbeat_interval: super::DEFAULT_HEARTBEAT_INTERVAL,
            compression: Compression::None,
            binary_mode: false,
        }
    }
}

fn default_heartbeat_interval() -> u64 {
    super::DEFAULT_HEARTBEAT_INTERVAL
}

/// Compression options
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Compression {
    #[default]
    None,
    Lz4,
    Zstd,
}

/// Channel information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelInfo {
    pub name: String,
    #[serde(rename = "type")]
    pub channel_type: DataType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

// ============================================================================
// Subscription Messages
// ============================================================================

/// Subscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribePayload {
    pub channel: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<SubscriptionFilter>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<SubscriptionOptions>,
}

/// Subscribe acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribeAckPayload {
    pub channel: String,
    pub subscription_id: String,
    pub status: SubscriptionStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub effective_rate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub history_sent: Option<u64>,
}

/// Unsubscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnsubscribePayload {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channel: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subscription_id: Option<String>,
}

/// Unsubscribe acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnsubscribeAckPayload {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channel: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subscription_id: Option<String>,
    pub status: UnsubscribeStatus,
}

/// Subscription filter
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscriptionFilter {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality: Option<Vec<QualityFlag>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_rate: Option<f64>,
}

/// Subscription options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscriptionOptions {
    #[serde(default = "default_buffer_size")]
    pub buffer_size: u32,
    #[serde(default)]
    pub throttle_ms: u32,
    #[serde(default)]
    pub include_history: bool,
    #[serde(default = "default_history_limit")]
    pub history_limit: u32,
    #[serde(default)]
    pub qos: QoS,
}

impl Default for SubscriptionOptions {
    fn default() -> Self {
        Self {
            buffer_size: 100,
            throttle_ms: 0,
            include_history: false,
            history_limit: 100,
            qos: QoS::AtMostOnce,
        }
    }
}

fn default_buffer_size() -> u32 {
    100
}
fn default_history_limit() -> u32 {
    100
}

/// Subscription status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubscriptionStatus {
    Active,
    Pending,
}

/// Unsubscribe status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnsubscribeStatus {
    Success,
    NotFound,
}

/// Quality of Service levels
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QoS {
    #[default]
    AtMostOnce,
    AtLeastOnce,
    ExactlyOnce,
}

// ============================================================================
// Data Messages
// ============================================================================

/// Data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPayload {
    pub channel: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<u64>,
    pub data_type: DataType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub batch: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_start: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub count: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub items: Option<Vec<serde_json::Value>>,
}

impl DataPayload {
    /// Extract typed data from payload
    pub fn into_typed<T: for<'de> Deserialize<'de>>(&self) -> PhysicsResult<T> {
        match &self.data {
            Some(value) => serde_json::from_value(value.clone())
                .map_err(|e| PhysicsError::parse(e.to_string())),
            None => Err(PhysicsError::validation("No data in payload")),
        }
    }
}

/// Data types (physics domains)
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataType {
    Fusion,
    TimeCrystal,
    Particle,
    DarkMatter,
    Antimatter,
    QuantumGravity,
}

// ============================================================================
// Command Messages
// ============================================================================

/// Command payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPayload {
    pub target: String,
    pub action: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub priority: Option<Priority>,
}

/// Response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponsePayload {
    pub command_id: String,
    pub status: CommandStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution_time: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<CommandError>,
}

/// Command priority
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Priority {
    Low,
    #[default]
    Normal,
    High,
    Critical,
}

/// Command status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandStatus {
    Success,
    Pending,
    Executing,
    Failed,
    Timeout,
    Rejected,
    Cancelled,
}

/// Command error details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandError {
    pub code: u32,
    pub message: String,
}

// ============================================================================
// Event Messages
// ============================================================================

/// Event payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventPayload {
    pub source: String,
    pub event_type: EventType,
    pub severity: Severity,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<serde_json::Value>,
}

/// Event types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventType {
    StateChange,
    Threshold,
    Alarm,
    Warning,
    Info,
    Discovery,
}

/// Severity levels
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    Critical,
    Error,
    Warning,
    Info,
    Debug,
}

// ============================================================================
// Error Messages
// ============================================================================

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPayload {
    pub code: u32,
    pub category: ErrorCategory,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub related_message_id: Option<String>,
}

/// Error categories
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCategory {
    Connection,
    Authentication,
    Protocol,
    Subscription,
    Command,
    Data,
    Server,
}

/// Error codes
pub mod error_codes {
    // Connection errors (1000-1999)
    pub const CONNECTION_REFUSED: u32 = 1001;
    pub const CONNECTION_TIMEOUT: u32 = 1002;
    pub const CONNECTION_CLOSED: u32 = 1003;
    pub const MAX_CONNECTIONS_EXCEEDED: u32 = 1004;
    pub const INVALID_PROTOCOL_VERSION: u32 = 1005;

    // Authentication errors (2000-2999)
    pub const AUTH_REQUIRED: u32 = 2001;
    pub const INVALID_CREDENTIALS: u32 = 2002;
    pub const TOKEN_EXPIRED: u32 = 2003;
    pub const INSUFFICIENT_PERMISSIONS: u32 = 2004;

    // Protocol errors (3000-3999)
    pub const INVALID_MESSAGE_FORMAT: u32 = 3001;
    pub const UNKNOWN_MESSAGE_TYPE: u32 = 3002;
    pub const MISSING_REQUIRED_FIELD: u32 = 3003;
    pub const INVALID_FIELD_VALUE: u32 = 3004;
    pub const MESSAGE_TOO_LARGE: u32 = 3005;

    // Subscription errors (4000-4999)
    pub const CHANNEL_NOT_FOUND: u32 = 4001;
    pub const INVALID_CHANNEL_PATTERN: u32 = 4002;
    pub const SUBSCRIPTION_LIMIT_EXCEEDED: u32 = 4003;
    pub const ALREADY_SUBSCRIBED: u32 = 4004;
    pub const NOT_SUBSCRIBED: u32 = 4005;

    // Command errors (5000-5999)
    pub const UNKNOWN_COMMAND: u32 = 5001;
    pub const INVALID_PARAMETERS: u32 = 5002;
    pub const TARGET_NOT_FOUND: u32 = 5003;
    pub const COMMAND_TIMEOUT: u32 = 5004;
    pub const COMMAND_REJECTED: u32 = 5005;

    // Data errors (6000-6999)
    pub const INVALID_DATA_FORMAT: u32 = 6001;
    pub const DATA_VALIDATION_FAILED: u32 = 6002;

    // Server errors (7000-7999)
    pub const INTERNAL_ERROR: u32 = 7001;
    pub const SERVICE_UNAVAILABLE: u32 = 7002;
}

// ============================================================================
// Keepalive Messages
// ============================================================================

/// Ping payload (empty)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PingPayload {}

/// Pong payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PongPayload {
    pub ping_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latency_ms: Option<u64>,
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connect_message() {
        let msg = WppMessage::connect("client-001", "Test Client");
        assert_eq!(msg.wpp, "1.0");
        assert_eq!(msg.msg_type, MessageType::Connect);

        let json = msg.to_json().unwrap();
        assert!(json.contains("client-001"));

        let parsed = WppMessage::from_json(&json).unwrap();
        assert_eq!(parsed.msg_type, MessageType::Connect);
    }

    #[test]
    fn test_subscribe_message() {
        let msg = WppMessage::subscribe("fusion/iter/plasma");
        assert_eq!(msg.msg_type, MessageType::Subscribe);

        let json = msg.to_json().unwrap();
        assert!(json.contains("fusion/iter/plasma"));
    }

    #[test]
    fn test_data_message() {
        use crate::types::Measurement;

        let data = Measurement::new(150e6, 1e4, "K");
        let msg = WppMessage::data("fusion/iter/plasma", DataType::Fusion, &data, 1).unwrap();

        assert_eq!(msg.msg_type, MessageType::Data);
        let json = msg.to_json().unwrap();
        assert!(json.contains("150000000"));
    }

    #[test]
    fn test_error_message() {
        let msg = WppMessage::error(
            error_codes::CHANNEL_NOT_FOUND,
            ErrorCategory::Subscription,
            "Channel not found",
        );
        assert_eq!(msg.msg_type, MessageType::Error);

        let json = msg.to_json().unwrap();
        assert!(json.contains("4001"));
    }

    #[test]
    fn test_ping_pong() {
        let ping = WppMessage::ping();
        assert_eq!(ping.msg_type, MessageType::Ping);

        let pong = WppMessage::pong(&ping.id, 5);
        assert_eq!(pong.msg_type, MessageType::Pong);
    }
}
