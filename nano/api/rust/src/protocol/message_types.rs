//! Protocol message types for nanoscale communication

use serde::{Deserialize, Serialize};
use crate::types::{Position3D, NanoSystemType};

/// Protocol message header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolHeader {
    /// Protocol version
    pub version: u8,
    /// Message type
    pub msg_type: MessageType,
    /// Source device ID
    pub source_id: String,
    /// Destination device ID (None for broadcast)
    pub dest_id: Option<String>,
    /// Message sequence number
    pub sequence: u64,
    /// Time-to-live (hop count)
    pub ttl: u8,
    /// Priority level (0-7)
    pub priority: u8,
    /// Message flags
    pub flags: MessageFlags,
    /// Timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
}

impl ProtocolHeader {
    pub fn new(source_id: impl Into<String>, msg_type: MessageType) -> Self {
        Self {
            version: 1,
            msg_type,
            source_id: source_id.into(),
            dest_id: None,
            sequence: 0,
            ttl: 64,
            priority: 4,
            flags: MessageFlags::default(),
            timestamp_ns: chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0) as u64,
        }
    }

    pub fn with_dest(mut self, dest_id: impl Into<String>) -> Self {
        self.dest_id = Some(dest_id.into());
        self
    }

    pub fn with_sequence(mut self, seq: u64) -> Self {
        self.sequence = seq;
        self
    }

    pub fn with_priority(mut self, priority: u8) -> Self {
        self.priority = priority.min(7);
        self
    }

    pub fn with_ttl(mut self, ttl: u8) -> Self {
        self.ttl = ttl;
        self
    }
}

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// Status/telemetry data
    Status,
    /// Command message
    Command,
    /// Command acknowledgment
    Ack,
    /// Negative acknowledgment
    Nack,
    /// Discovery/announcement
    Discovery,
    /// Heartbeat/keepalive
    Heartbeat,
    /// Data transfer
    Data,
    /// Stream start
    StreamStart,
    /// Stream data
    StreamData,
    /// Stream end
    StreamEnd,
    /// Emergency/alert
    Emergency,
    /// Swarm coordination
    Swarm,
    /// Custom message type
    Custom(u8),
}

/// Message flags bitfield
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct MessageFlags {
    /// Requires acknowledgment
    pub requires_ack: bool,
    /// Is a response message
    pub is_response: bool,
    /// Is encrypted
    pub encrypted: bool,
    /// Is compressed
    pub compressed: bool,
    /// Is fragmented
    pub fragmented: bool,
    /// Last fragment
    pub last_fragment: bool,
    /// Broadcast message
    pub broadcast: bool,
    /// Time-critical
    pub urgent: bool,
}

impl MessageFlags {
    pub fn to_byte(&self) -> u8 {
        let mut byte = 0u8;
        if self.requires_ack { byte |= 0x01; }
        if self.is_response { byte |= 0x02; }
        if self.encrypted { byte |= 0x04; }
        if self.compressed { byte |= 0x08; }
        if self.fragmented { byte |= 0x10; }
        if self.last_fragment { byte |= 0x20; }
        if self.broadcast { byte |= 0x40; }
        if self.urgent { byte |= 0x80; }
        byte
    }

    pub fn from_byte(byte: u8) -> Self {
        Self {
            requires_ack: byte & 0x01 != 0,
            is_response: byte & 0x02 != 0,
            encrypted: byte & 0x04 != 0,
            compressed: byte & 0x08 != 0,
            fragmented: byte & 0x10 != 0,
            last_fragment: byte & 0x20 != 0,
            broadcast: byte & 0x40 != 0,
            urgent: byte & 0x80 != 0,
        }
    }
}

/// Complete protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage {
    pub header: ProtocolHeader,
    pub payload: MessagePayload,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub checksum: Option<u32>,
}

impl ProtocolMessage {
    pub fn new(header: ProtocolHeader, payload: MessagePayload) -> Self {
        Self {
            header,
            payload,
            checksum: None,
        }
    }

    pub fn status(source_id: impl Into<String>, status: StatusPayload) -> Self {
        Self::new(
            ProtocolHeader::new(source_id, MessageType::Status),
            MessagePayload::Status(status),
        )
    }

    pub fn command(source_id: impl Into<String>, dest_id: impl Into<String>, command: CommandPayload) -> Self {
        Self::new(
            ProtocolHeader::new(source_id, MessageType::Command).with_dest(dest_id),
            MessagePayload::Command(command),
        )
    }

    pub fn ack(source_id: impl Into<String>, dest_id: impl Into<String>, ack: AckPayload) -> Self {
        let mut header = ProtocolHeader::new(source_id, MessageType::Ack).with_dest(dest_id);
        header.flags.is_response = true;
        Self::new(header, MessagePayload::Ack(ack))
    }

    pub fn discovery(source_id: impl Into<String>, discovery: DiscoveryPayload) -> Self {
        let mut header = ProtocolHeader::new(source_id, MessageType::Discovery);
        header.flags.broadcast = true;
        Self::new(header, MessagePayload::Discovery(discovery))
    }

    pub fn with_checksum(mut self) -> Self {
        self.checksum = Some(self.calculate_checksum());
        self
    }

    pub fn calculate_checksum(&self) -> u32 {
        // Simple CRC32-like checksum (simplified)
        let data = serde_json::to_vec(&self.payload).unwrap_or_default();
        let mut crc: u32 = 0xFFFFFFFF;
        for byte in data {
            crc ^= byte as u32;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        !crc
    }

    pub fn verify_checksum(&self) -> bool {
        match self.checksum {
            Some(expected) => self.calculate_checksum() == expected,
            None => true, // No checksum to verify
        }
    }
}

/// Message payload variants
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "data")]
pub enum MessagePayload {
    Status(StatusPayload),
    Command(CommandPayload),
    Ack(AckPayload),
    Nack(NackPayload),
    Discovery(DiscoveryPayload),
    Heartbeat(HeartbeatPayload),
    Data(DataPayload),
    Stream(StreamPayload),
    Emergency(EmergencyPayload),
    Swarm(SwarmPayload),
    Custom(serde_json::Value),
}

/// Status/telemetry payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusPayload {
    pub system_type: NanoSystemType,
    pub operational_state: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position3D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_level: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_code: Option<u32>,
    pub metrics: serde_json::Value,
}

/// Command payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPayload {
    pub command_id: String,
    pub command_type: CommandType,
    pub parameters: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_ms: Option<u64>,
}

/// Command types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandType {
    Move,
    Stop,
    Start,
    Shutdown,
    Reset,
    Configure,
    Execute,
    Abort,
    Query,
    Custom,
}

/// Acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AckPayload {
    pub ack_sequence: u64,
    pub command_id: String,
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,
}

/// Negative acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NackPayload {
    pub nack_sequence: u64,
    pub command_id: String,
    pub error_code: u32,
    pub error_message: String,
}

/// Discovery/announcement payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveryPayload {
    pub system_type: NanoSystemType,
    pub capabilities: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position3D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub communication_range_nm: Option<f64>,
    pub protocol_version: String,
}

/// Heartbeat payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeartbeatPayload {
    pub uptime_ms: u64,
    pub sequence: u64,
    pub health_status: HealthStatus,
}

/// Health status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HealthStatus {
    Healthy,
    Degraded,
    Warning,
    Critical,
}

/// Data transfer payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPayload {
    pub data_type: String,
    pub encoding: DataEncoding,
    pub data: Vec<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_size: Option<usize>,
}

/// Data encoding
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataEncoding {
    Raw,
    Base64,
    Hex,
    Compressed,
}

/// Stream payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamPayload {
    pub stream_id: String,
    pub stream_type: StreamType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fragment_index: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_fragments: Option<u32>,
    pub data: Vec<u8>,
}

/// Stream type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamType {
    Start,
    Continue,
    End,
    Abort,
}

/// Emergency payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyPayload {
    pub emergency_type: EmergencyType,
    pub severity: u8,
    pub description: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position3D>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recommended_action: Option<String>,
}

/// Emergency type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmergencyType {
    Collision,
    LowEnergy,
    SystemFailure,
    EnvironmentHazard,
    CommunicationLoss,
    MissionAbort,
    Custom,
}

/// Swarm coordination payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmPayload {
    pub swarm_id: String,
    pub swarm_action: SwarmAction,
    pub data: serde_json::Value,
}

/// Swarm action types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SwarmAction {
    Join,
    Leave,
    LeaderElection,
    TaskAssignment,
    StateSync,
    Formation,
    Consensus,
}
