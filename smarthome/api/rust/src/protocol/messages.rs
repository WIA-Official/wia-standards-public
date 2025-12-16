//! WIA Protocol Message Definitions
//! 弘益人間 - Benefit All Humanity

use crate::types::{InputModality, OutputModality};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Protocol version
pub const PROTOCOL_VERSION: u8 = 1;

/// Message type identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u16)]
pub enum MessageType {
    Command = 0x0001,
    Response = 0x0002,
    Event = 0x0003,
    Subscribe = 0x0004,
    Unsubscribe = 0x0005,
    Discovery = 0x0010,
    Announce = 0x0011,
    AccessibilityFeedback = 0x0020,
}

/// Message flags
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct MessageFlags {
    /// Requires acknowledgment
    pub ack_required: bool,
    /// Message is encrypted
    pub encrypted: bool,
    /// Contains accessibility context
    pub has_accessibility_context: bool,
    /// Is a broadcast message
    pub broadcast: bool,
}

impl MessageFlags {
    pub fn to_byte(&self) -> u8 {
        let mut flags = 0u8;
        if self.ack_required {
            flags |= 0x01;
        }
        if self.encrypted {
            flags |= 0x02;
        }
        if self.has_accessibility_context {
            flags |= 0x04;
        }
        if self.broadcast {
            flags |= 0x08;
        }
        flags
    }

    pub fn from_byte(byte: u8) -> Self {
        Self {
            ack_required: byte & 0x01 != 0,
            encrypted: byte & 0x02 != 0,
            has_accessibility_context: byte & 0x04 != 0,
            broadcast: byte & 0x08 != 0,
        }
    }
}

/// Message header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageHeader {
    /// Protocol version
    pub version: u8,
    /// Message type
    pub message_type: MessageType,
    /// Unique message ID
    pub message_id: u32,
    /// Source node ID
    pub source_node: u64,
    /// Destination node ID
    pub dest_node: u64,
    /// Unix timestamp in milliseconds
    pub timestamp: u64,
    /// Message flags
    pub flags: MessageFlags,
}

impl MessageHeader {
    /// Create a new message header
    pub fn new(
        message_type: MessageType,
        source_node: u64,
        dest_node: u64,
    ) -> Self {
        use std::time::{SystemTime, UNIX_EPOCH};

        static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

        Self {
            version: PROTOCOL_VERSION,
            message_type,
            message_id: COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst),
            source_node,
            dest_node,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            flags: MessageFlags::default(),
        }
    }
}

/// Accessibility context for messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityContext {
    /// User profile ID
    pub user_profile_id: Uuid,
    /// Input modality used
    pub input_modality: InputModality,
    /// Preferred output modalities
    pub preferred_outputs: Vec<OutputModality>,
    /// Response timeout in milliseconds
    pub response_timeout_ms: u32,
    /// Whether confirmation is required
    pub confirmation_required: bool,
}

impl Default for AccessibilityContext {
    fn default() -> Self {
        Self {
            user_profile_id: Uuid::nil(),
            input_modality: InputModality::Touch,
            preferred_outputs: vec![OutputModality::VisualScreen],
            response_timeout_ms: 5000,
            confirmation_required: false,
        }
    }
}

/// Message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessagePayload {
    /// Target cluster ID
    pub cluster_id: u16,
    /// Command ID within cluster
    pub command_id: u8,
    /// Serialized command data
    pub data: Vec<u8>,
    /// Accessibility context
    pub accessibility: Option<AccessibilityContext>,
}

/// Complete WIA protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaMessage {
    /// Message header
    pub header: MessageHeader,
    /// Message payload
    pub payload: MessagePayload,
    /// Optional signature for authentication
    pub signature: Option<Vec<u8>>,
}

impl WiaMessage {
    /// Create a new command message
    pub fn command(
        source_node: u64,
        dest_node: u64,
        cluster_id: u16,
        command_id: u8,
        data: Vec<u8>,
    ) -> Self {
        Self {
            header: MessageHeader::new(MessageType::Command, source_node, dest_node),
            payload: MessagePayload {
                cluster_id,
                command_id,
                data,
                accessibility: None,
            },
            signature: None,
        }
    }

    /// Create a response message
    pub fn response(
        source_node: u64,
        dest_node: u64,
        request_id: u32,
        cluster_id: u16,
        data: Vec<u8>,
    ) -> Self {
        let mut msg = Self {
            header: MessageHeader::new(MessageType::Response, source_node, dest_node),
            payload: MessagePayload {
                cluster_id,
                command_id: 0,
                data,
                accessibility: None,
            },
            signature: None,
        };
        msg.header.message_id = request_id;
        msg
    }

    /// Create an event message
    pub fn event(
        source_node: u64,
        cluster_id: u16,
        event_id: u8,
        data: Vec<u8>,
    ) -> Self {
        let mut msg = Self {
            header: MessageHeader::new(MessageType::Event, source_node, 0),
            payload: MessagePayload {
                cluster_id,
                command_id: event_id,
                data,
                accessibility: None,
            },
            signature: None,
        };
        msg.header.flags.broadcast = true;
        msg
    }

    /// Add accessibility context
    pub fn with_accessibility(mut self, context: AccessibilityContext) -> Self {
        self.payload.accessibility = Some(context);
        self.header.flags.has_accessibility_context = true;
        self
    }

    /// Sign the message
    pub fn sign(mut self, signature: Vec<u8>) -> Self {
        self.signature = Some(signature);
        self
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    /// Deserialize from bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self, serde_json::Error> {
        serde_json::from_slice(data)
    }
}

/// Protocol error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum ProtocolError {
    Success = 0x00,
    Unsupported = 0x01,
    InvalidParam = 0x02,
    NotFound = 0x03,
    Timeout = 0x04,
    Busy = 0x05,
    Offline = 0x06,
    AuthFailed = 0x07,
    AccessDenied = 0x08,
    AccessibilityError = 0x10,
}

impl ProtocolError {
    pub fn is_success(&self) -> bool {
        *self == ProtocolError::Success
    }

    pub fn message(&self) -> &'static str {
        match self {
            ProtocolError::Success => "Operation successful",
            ProtocolError::Unsupported => "Feature not supported",
            ProtocolError::InvalidParam => "Invalid parameter",
            ProtocolError::NotFound => "Resource not found",
            ProtocolError::Timeout => "Operation timed out",
            ProtocolError::Busy => "Device busy",
            ProtocolError::Offline => "Device offline",
            ProtocolError::AuthFailed => "Authentication failed",
            ProtocolError::AccessDenied => "Permission denied",
            ProtocolError::AccessibilityError => "Accessibility feature error",
        }
    }
}

/// Response data wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseData {
    /// Error code
    pub error: ProtocolError,
    /// Response data (if successful)
    pub data: Option<Vec<u8>>,
    /// Error message (if failed)
    pub message: Option<String>,
}

impl ResponseData {
    pub fn success(data: Vec<u8>) -> Self {
        Self {
            error: ProtocolError::Success,
            data: Some(data),
            message: None,
        }
    }

    pub fn error(error: ProtocolError) -> Self {
        Self {
            error,
            data: None,
            message: Some(error.message().to_string()),
        }
    }

    pub fn error_with_message(error: ProtocolError, message: String) -> Self {
        Self {
            error,
            data: None,
            message: Some(message),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_flags() {
        let flags = MessageFlags {
            ack_required: true,
            encrypted: true,
            has_accessibility_context: false,
            broadcast: false,
        };

        let byte = flags.to_byte();
        assert_eq!(byte, 0x03);

        let restored = MessageFlags::from_byte(byte);
        assert_eq!(restored.ack_required, true);
        assert_eq!(restored.encrypted, true);
        assert_eq!(restored.has_accessibility_context, false);
    }

    #[test]
    fn test_message_serialization() {
        let msg = WiaMessage::command(1, 2, 0x0006, 0x01, vec![0x01]);
        let bytes = msg.to_bytes().unwrap();
        let restored = WiaMessage::from_bytes(&bytes).unwrap();

        assert_eq!(restored.header.source_node, 1);
        assert_eq!(restored.header.dest_node, 2);
        assert_eq!(restored.payload.cluster_id, 0x0006);
    }

    #[test]
    fn test_accessibility_context() {
        let context = AccessibilityContext {
            user_profile_id: Uuid::new_v4(),
            input_modality: InputModality::Voice,
            preferred_outputs: vec![OutputModality::AudioTts],
            response_timeout_ms: 3000,
            confirmation_required: true,
        };

        let msg = WiaMessage::command(1, 2, 0x0006, 0x01, vec![])
            .with_accessibility(context);

        assert!(msg.header.flags.has_accessibility_context);
        assert!(msg.payload.accessibility.is_some());
    }
}
