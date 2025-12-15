//! Protocol message types for WIA Biotechnology Standard

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

use crate::types::{Sequence, CrisprExperiment, ProteinStructure, BioPart};

/// Main protocol message envelope
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BioMessage {
    /// Protocol identifier (always "wia-bio")
    pub protocol: String,
    /// Protocol version
    pub version: String,
    /// Unique message identifier
    pub message_id: String,
    /// Message timestamp (Unix milliseconds)
    pub timestamp: i64,
    /// Message sequence number for ordering
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<u64>,
    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,
    /// Message payload
    pub payload: MessagePayload,
}

impl BioMessage {
    /// Create a new message with generated ID and current timestamp
    pub fn new(message_type: MessageType, payload: MessagePayload) -> Self {
        Self {
            protocol: super::PROTOCOL_ID.to_string(),
            version: super::PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis(),
            sequence: None,
            message_type,
            payload,
        }
    }

    /// Create message with sequence number
    pub fn with_sequence(mut self, seq: u64) -> Self {
        self.sequence = Some(seq);
        self
    }

    /// Validate message structure
    pub fn validate(&self) -> Result<(), ProtocolError> {
        if self.protocol != super::PROTOCOL_ID {
            return Err(ProtocolError::InvalidProtocol(self.protocol.clone()));
        }
        if self.message_id.is_empty() {
            return Err(ProtocolError::MissingField("message_id".to_string()));
        }
        Ok(())
    }
}

/// Message types supported by the protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// Connection request (client → server)
    Connect,
    /// Connection acknowledgment (server → client)
    ConnectAck,
    /// Disconnection notice (bidirectional)
    Disconnect,
    /// Data message containing bio data (server → client)
    Data,
    /// Sequence data stream
    Sequence,
    /// Experiment data stream
    Experiment,
    /// Structure data stream
    Structure,
    /// Command request (client → server)
    Command,
    /// Command acknowledgment (server → client)
    CommandAck,
    /// Error message (bidirectional)
    Error,
    /// Heartbeat ping (client → server)
    Ping,
    /// Heartbeat pong (server → client)
    Pong,
    /// Subscription request
    Subscribe,
    /// Subscription acknowledgment
    SubscribeAck,
    /// Unsubscribe request
    Unsubscribe,
}

/// Message payload variants
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MessagePayload {
    /// Connection request payload
    Connect(ConnectPayload),
    /// Connection acknowledgment payload
    ConnectAck(ConnectAckPayload),
    /// Disconnect payload
    Disconnect(DisconnectPayload),
    /// Sequence data payload
    Sequence(SequencePayload),
    /// Experiment data payload
    Experiment(ExperimentPayload),
    /// Structure data payload
    Structure(StructurePayload),
    /// Part data payload
    Part(PartPayload),
    /// Command payload
    Command(CommandPayload),
    /// Command acknowledgment payload
    CommandAck(CommandAckPayload),
    /// Error payload
    Error(ErrorPayload),
    /// Ping/Pong payload
    Heartbeat(HeartbeatPayload),
    /// Subscribe payload
    Subscribe(SubscribePayload),
    /// Empty payload
    Empty(EmptyPayload),
}

/// Connection request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectPayload {
    /// Client identifier
    pub client_id: String,
    /// Client application name
    pub client_name: String,
    /// Client version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_version: Option<String>,
    /// Requested capabilities
    #[serde(default)]
    pub capabilities: Vec<ProtocolCapability>,
    /// Connection options
    #[serde(skip_serializing_if = "Option::is_none")]
    pub options: Option<ConnectionOptions>,
}

/// Protocol capabilities
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProtocolCapability {
    /// Real-time sequence streaming
    SequenceStreaming,
    /// CRISPR experiment data
    CrisprExperiment,
    /// Protein structure data
    ProteinStructure,
    /// Synthetic biology parts
    SyntheticBiology,
    /// FHIR genomics integration
    FhirGenomics,
    /// Batch data transfer
    BatchTransfer,
    /// Compression support
    Compression,
}

/// Connection options
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectionOptions {
    /// Data streaming rate (messages per second)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stream_rate: Option<u32>,
    /// Enable data compression
    #[serde(default)]
    pub compression: bool,
    /// Buffer size for streaming
    #[serde(skip_serializing_if = "Option::is_none")]
    pub buffer_size: Option<usize>,
    /// Heartbeat interval in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heartbeat_interval: Option<u64>,
}

/// Connection acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectAckPayload {
    /// Server-assigned session ID
    pub session_id: String,
    /// Connection status
    pub status: ConnectionStatus,
    /// Server capabilities
    pub capabilities: Vec<ProtocolCapability>,
    /// Server version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_version: Option<String>,
    /// Session timeout in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout: Option<u64>,
}

/// Connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionStatus {
    /// Successfully connected
    Connected,
    /// Authentication required
    AuthRequired,
    /// Connection rejected
    Rejected,
}

/// Disconnect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DisconnectPayload {
    /// Disconnect reason code
    pub reason_code: u32,
    /// Human-readable reason
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// Whether reconnection is allowed
    #[serde(default)]
    pub reconnect_allowed: bool,
}

/// Sequence data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SequencePayload {
    /// Sequence data (WIA Bio format)
    pub sequence: Sequence,
    /// Data source identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_id: Option<String>,
    /// Batch indicator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub batch_id: Option<String>,
    /// Position in batch
    #[serde(skip_serializing_if = "Option::is_none")]
    pub batch_index: Option<u32>,
}

/// Experiment data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExperimentPayload {
    /// CRISPR experiment data
    pub experiment: CrisprExperiment,
    /// Status update flag
    #[serde(default)]
    pub is_update: bool,
}

/// Structure data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StructurePayload {
    /// Protein structure data
    pub structure: ProteinStructure,
    /// Prediction job ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub job_id: Option<String>,
}

/// BioPart data payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PartPayload {
    /// BioPart data
    pub part: BioPart,
    /// Registry source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub registry: Option<String>,
}

/// Command payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandPayload {
    /// Command type
    pub command: BioCommand,
    /// Command parameters
    #[serde(default)]
    pub params: serde_json::Value,
    /// Request correlation ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

/// Bio protocol commands
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BioCommand {
    /// Start sequence analysis
    AnalyzeSequence,
    /// Design CRISPR guide RNA
    DesignGuideRna,
    /// Submit structure prediction
    PredictStructure,
    /// Query experiment status
    GetExperimentStatus,
    /// Start data stream
    StartStream,
    /// Stop data stream
    StopStream,
    /// Validate data format
    ValidateData,
    /// Export data
    ExportData,
}

/// Command acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandAckPayload {
    /// Original command
    pub command: BioCommand,
    /// Execution status
    pub status: CommandStatus,
    /// Correlation ID from request
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
    /// Result data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,
    /// Error message if failed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Command execution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandStatus {
    /// Command executed successfully
    Success,
    /// Command failed
    Failed,
    /// Command is pending
    Pending,
    /// Command is in progress
    InProgress,
    /// Command cancelled
    Cancelled,
}

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorPayload {
    /// Error code
    pub code: u32,
    /// Error name
    pub name: String,
    /// Error message
    pub message: String,
    /// Additional details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    /// Whether error is recoverable
    #[serde(default)]
    pub recoverable: bool,
}

/// Heartbeat payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HeartbeatPayload {
    /// Client timestamp
    pub client_timestamp: i64,
    /// Server timestamp (for pong)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_timestamp: Option<i64>,
}

/// Subscribe payload
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubscribePayload {
    /// Topic to subscribe
    pub topic: SubscriptionTopic,
    /// Filter criteria
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<serde_json::Value>,
}

/// Subscription topics
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubscriptionTopic {
    /// All sequences
    Sequences,
    /// CRISPR experiments
    Experiments,
    /// Protein structures
    Structures,
    /// Bio parts
    Parts,
    /// Project updates
    Projects,
    /// All data
    All,
}

/// Empty payload for ping/pong
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmptyPayload {}

/// Protocol error types
#[derive(Debug, Clone, thiserror::Error)]
pub enum ProtocolError {
    /// Invalid protocol identifier
    #[error("Invalid protocol: {0}")]
    InvalidProtocol(String),

    /// Invalid message type
    #[error("Invalid message type: {0}")]
    InvalidMessageType(String),

    /// Missing required field
    #[error("Missing required field: {0}")]
    MissingField(String),

    /// Serialization error
    #[error("Serialization error: {0}")]
    SerializationError(String),

    /// Deserialization error
    #[error("Deserialization error: {0}")]
    DeserializationError(String),

    /// Invalid payload for message type
    #[error("Invalid payload for message type {0}")]
    InvalidPayload(String),

    /// Protocol version mismatch
    #[error("Protocol version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: String, actual: String },
}

/// Standard error codes
#[derive(Debug, Clone, Copy)]
pub struct ErrorCode;

impl ErrorCode {
    // Connection errors (1xxx)
    /// Normal connection close
    pub const CONNECTION_CLOSED: u32 = 1000;
    /// Connection lost unexpectedly
    pub const CONNECTION_LOST: u32 = 1001;
    /// Protocol error
    pub const PROTOCOL_ERROR: u32 = 1002;
    /// Unsupported message type
    pub const UNSUPPORTED_TYPE: u32 = 1003;
    /// Invalid message format
    pub const INVALID_FORMAT: u32 = 1004;

    // Data errors (2xxx)
    /// Invalid sequence data
    pub const INVALID_SEQUENCE: u32 = 2001;
    /// Invalid experiment data
    pub const INVALID_EXPERIMENT: u32 = 2002;
    /// Invalid structure data
    pub const INVALID_STRUCTURE: u32 = 2003;
    /// Data validation failed
    pub const VALIDATION_FAILED: u32 = 2004;
    /// Data not found
    pub const DATA_NOT_FOUND: u32 = 2005;

    // Service errors (3xxx)
    /// Authentication failed
    pub const AUTH_FAILED: u32 = 3001;
    /// Permission denied
    pub const PERMISSION_DENIED: u32 = 3002;
    /// Rate limit exceeded
    pub const RATE_LIMITED: u32 = 3003;
    /// Service unavailable
    pub const SERVICE_UNAVAILABLE: u32 = 3004;

    // Command errors (4xxx)
    /// Unknown command
    pub const UNKNOWN_COMMAND: u32 = 4001;
    /// Invalid command parameters
    pub const INVALID_PARAMS: u32 = 4002;
    /// Command execution failed
    pub const COMMAND_FAILED: u32 = 4003;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_creation() {
        let payload = MessagePayload::Heartbeat(HeartbeatPayload {
            client_timestamp: Utc::now().timestamp_millis(),
            server_timestamp: None,
        });

        let msg = BioMessage::new(MessageType::Ping, payload);

        assert_eq!(msg.protocol, "wia-bio");
        assert_eq!(msg.version, "1.0.0");
        assert_eq!(msg.message_type, MessageType::Ping);
        assert!(msg.validate().is_ok());
    }

    #[test]
    fn test_message_serialization() {
        let payload = MessagePayload::Connect(ConnectPayload {
            client_id: "test-client".to_string(),
            client_name: "Test App".to_string(),
            client_version: Some("1.0.0".to_string()),
            capabilities: vec![ProtocolCapability::SequenceStreaming],
            options: None,
        });

        let msg = BioMessage::new(MessageType::Connect, payload);
        let json = serde_json::to_string(&msg).unwrap();

        assert!(json.contains("wia-bio"));
        assert!(json.contains("test-client"));
    }

    #[test]
    fn test_error_codes() {
        assert_eq!(ErrorCode::CONNECTION_CLOSED, 1000);
        assert_eq!(ErrorCode::INVALID_SEQUENCE, 2001);
        assert_eq!(ErrorCode::AUTH_FAILED, 3001);
    }
}
