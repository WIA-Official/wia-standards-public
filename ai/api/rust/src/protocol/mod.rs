//! WIA AI Communication Protocol
//!
//! This module implements the WIA AI Communication Protocol for
//! agent-to-agent, agent-to-tool, and streaming communications.
//!
//! ## Features
//!
//! - Message types for all protocol operations
//! - Message builder for constructing messages
//! - Protocol handler for processing messages
//! - Error types for protocol errors
//!
//! ## Example
//!
//! ```rust
//! use wia_ai::protocol::*;
//!
//! // Create a message
//! let message = ProtocolMessageBuilder::new()
//!     .message_type(MessageType::Message)
//!     .source(EntityReference::agent("agent-001", "Researcher"))
//!     .target(EntityReference::agent("agent-002", "Coder"))
//!     .payload(serde_json::json!({
//!         "role": "user",
//!         "content": [{"type": "text", "text": "Hello!"}]
//!     }))
//!     .build()
//!     .unwrap();
//!
//! // Serialize to JSON
//! let json = serde_json::to_string(&message).unwrap();
//! ```

mod message;
mod builder;
mod error;
mod handler;

pub use message::*;
pub use builder::*;
pub use error::*;
pub use handler::*;

/// Protocol version constant
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-ai";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_constants() {
        assert_eq!(PROTOCOL_ID, "wia-ai");
        assert_eq!(PROTOCOL_VERSION, "1.0.0");
    }

    #[test]
    fn test_message_creation() {
        let message = ProtocolMessageBuilder::new()
            .message_type(MessageType::Ping)
            .build()
            .unwrap();

        assert_eq!(message.protocol, PROTOCOL_ID);
        assert_eq!(message.version, PROTOCOL_VERSION);
        assert_eq!(message.message_type, MessageType::Ping);
    }

    #[test]
    fn test_message_serialization() {
        let message = ProtocolMessageBuilder::new()
            .message_type(MessageType::Message)
            .source(EntityReference::client("client-001"))
            .payload(serde_json::json!({
                "role": "user",
                "content": [{"type": "text", "text": "Hello"}]
            }))
            .build()
            .unwrap();

        let json = serde_json::to_string(&message).unwrap();
        assert!(json.contains("wia-ai"));
        assert!(json.contains("message"));

        // Deserialize
        let parsed: ProtocolMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.message_type, MessageType::Message);
    }

    #[test]
    fn test_error_codes() {
        let error = ProtocolError::connection_error(ErrorCode::ConnectionLost, "Connection lost");
        assert_eq!(error.code, ErrorCode::ConnectionLost);
        assert!(error.retryable);
    }
}
