//! Protocol message types and structures

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::Utc;

use super::{PROTOCOL_ID, PROTOCOL_VERSION};
use super::message_types::*;

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// Connection request (Client → Server)
    Connect,
    /// Connection acknowledgment (Server → Client)
    ConnectAck,
    /// Disconnect notification (Both directions)
    Disconnect,
    /// Climate data message (typically Server → Client)
    Data,
    /// Command request (Client → Server)
    Command,
    /// Command acknowledgment (Server → Client)
    CommandAck,
    /// Subscribe to data stream (Client → Server)
    Subscribe,
    /// Subscription acknowledgment (Server → Client)
    SubscribeAck,
    /// Unsubscribe from data stream (Client → Server)
    Unsubscribe,
    /// Error message (Both directions)
    Error,
    /// Connection heartbeat (Client → Server)
    Ping,
    /// Heartbeat response (Server → Client)
    Pong,
}

impl std::fmt::Display for MessageType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MessageType::Connect => write!(f, "connect"),
            MessageType::ConnectAck => write!(f, "connect_ack"),
            MessageType::Disconnect => write!(f, "disconnect"),
            MessageType::Data => write!(f, "data"),
            MessageType::Command => write!(f, "command"),
            MessageType::CommandAck => write!(f, "command_ack"),
            MessageType::Subscribe => write!(f, "subscribe"),
            MessageType::SubscribeAck => write!(f, "subscribe_ack"),
            MessageType::Unsubscribe => write!(f, "unsubscribe"),
            MessageType::Error => write!(f, "error"),
            MessageType::Ping => write!(f, "ping"),
            MessageType::Pong => write!(f, "pong"),
        }
    }
}

/// Message metadata for signatures and tracing
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MessageMeta {
    /// Message signature for integrity verification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<MessageSignature>,

    /// Correlation ID for request tracing
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,

    /// Source identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
}

/// Message signature for integrity verification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageSignature {
    /// Signature algorithm
    pub algorithm: SignatureAlgorithm,
    /// Base64-encoded signature value
    pub value: String,
}

/// Supported signature algorithms
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignatureAlgorithm {
    /// HMAC with SHA-256
    #[serde(rename = "HMAC-SHA256")]
    HmacSha256,
    /// HMAC with SHA-512
    #[serde(rename = "HMAC-SHA512")]
    HmacSha512,
    /// RSA with SHA-256
    #[serde(rename = "RSA-SHA256")]
    RsaSha256,
}

/// The main protocol message structure
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProtocolMessage {
    /// Protocol identifier (always "wia-climate")
    pub protocol: String,

    /// Protocol version (SemVer format)
    pub version: String,

    /// Unique message identifier (UUID v4)
    pub message_id: String,

    /// Message creation timestamp (Unix milliseconds)
    pub timestamp: i64,

    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,

    /// Message payload (type-dependent)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,

    /// Optional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub meta: Option<MessageMeta>,
}

impl ProtocolMessage {
    /// Create a new protocol message with the given type
    pub fn new(message_type: MessageType) -> Self {
        Self {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis(),
            message_type,
            payload: None,
            meta: None,
        }
    }

    /// Create a new protocol message with a specific payload
    pub fn with_payload<T: Serialize>(message_type: MessageType, payload: T) -> Result<Self, serde_json::Error> {
        let mut msg = Self::new(message_type);
        msg.payload = Some(serde_json::to_value(payload)?);
        Ok(msg)
    }

    /// Create a ping message
    pub fn ping() -> Self {
        Self::new(MessageType::Ping)
    }

    /// Create a pong message
    pub fn pong() -> Self {
        Self::new(MessageType::Pong)
    }

    /// Create a connect message
    pub fn connect(payload: ConnectPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Connect, payload)
    }

    /// Create a connect_ack message
    pub fn connect_ack(payload: ConnectAckPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::ConnectAck, payload)
    }

    /// Create a disconnect message
    pub fn disconnect(payload: Option<DisconnectPayload>) -> Result<Self, serde_json::Error> {
        match payload {
            Some(p) => Self::with_payload(MessageType::Disconnect, p),
            None => Ok(Self::new(MessageType::Disconnect)),
        }
    }

    /// Create a data message with ClimateMessage payload
    pub fn data(climate_message: &crate::ClimateMessage) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Data, climate_message)
    }

    /// Create a command message
    pub fn command(payload: CommandPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Command, payload)
    }

    /// Create a command_ack message
    pub fn command_ack(payload: CommandAckPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::CommandAck, payload)
    }

    /// Create a subscribe message
    pub fn subscribe(payload: SubscribePayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Subscribe, payload)
    }

    /// Create a subscribe_ack message
    pub fn subscribe_ack(payload: SubscribeAckPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::SubscribeAck, payload)
    }

    /// Create an unsubscribe message
    pub fn unsubscribe(payload: UnsubscribePayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Unsubscribe, payload)
    }

    /// Create an error message
    pub fn error(payload: ErrorPayload) -> Result<Self, serde_json::Error> {
        Self::with_payload(MessageType::Error, payload)
    }

    /// Serialize the message to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Serialize the message to pretty JSON string
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Deserialize a message from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Get the payload as a specific type
    pub fn get_payload<T: for<'de> Deserialize<'de>>(&self) -> Option<Result<T, serde_json::Error>> {
        self.payload.as_ref().map(|v| serde_json::from_value(v.clone()))
    }

    /// Check if this is a request message (expects a response)
    pub fn is_request(&self) -> bool {
        matches!(
            self.message_type,
            MessageType::Connect
                | MessageType::Command
                | MessageType::Subscribe
                | MessageType::Unsubscribe
                | MessageType::Ping
        )
    }

    /// Check if this is a response message
    pub fn is_response(&self) -> bool {
        matches!(
            self.message_type,
            MessageType::ConnectAck
                | MessageType::CommandAck
                | MessageType::SubscribeAck
                | MessageType::Pong
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_ping() {
        let msg = ProtocolMessage::ping();
        assert_eq!(msg.protocol, PROTOCOL_ID);
        assert_eq!(msg.version, PROTOCOL_VERSION);
        assert_eq!(msg.message_type, MessageType::Ping);
        assert!(msg.payload.is_none());
    }

    #[test]
    fn test_create_connect() {
        let payload = ConnectPayload {
            client_id: "test-client".to_string(),
            client_type: Some(ClientType::Sensor),
            capabilities: Some(vec!["carbon_capture".to_string()]),
            auth: None,
            metadata: None,
        };

        let msg = ProtocolMessage::connect(payload).unwrap();
        assert_eq!(msg.message_type, MessageType::Connect);
        assert!(msg.payload.is_some());
    }

    #[test]
    fn test_serialize_deserialize() {
        let msg = ProtocolMessage::ping();
        let json = msg.to_json().unwrap();
        let parsed = ProtocolMessage::from_json(&json).unwrap();

        assert_eq!(parsed.protocol, msg.protocol);
        assert_eq!(parsed.message_type, msg.message_type);
    }

    #[test]
    fn test_is_request_response() {
        assert!(ProtocolMessage::ping().is_request());
        assert!(ProtocolMessage::pong().is_response());

        let connect = ProtocolMessage::connect(ConnectPayload {
            client_id: "test".to_string(),
            client_type: None,
            capabilities: None,
            auth: None,
            metadata: None,
        }).unwrap();
        assert!(connect.is_request());
    }
}
