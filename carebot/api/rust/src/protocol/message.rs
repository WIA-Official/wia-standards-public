//! Protocol message types and serialization

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0";

/// CareBot WebSocket Protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CwpMessage {
    /// Protocol identifier
    pub protocol: String,
    /// Protocol version
    pub version: String,
    /// Unique message ID
    pub message_id: String,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,
    /// Payload
    pub payload: serde_json::Value,
    /// Authentication info
    pub auth: MessageAuth,
}

impl CwpMessage {
    /// Create a new CWP message
    pub fn new(message_type: MessageType, payload: serde_json::Value, device_id: &str, token: &str) -> Self {
        Self {
            protocol: "cwp".to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: format!("msg-{}", uuid::Uuid::new_v4()),
            timestamp: Timestamp::now(),
            message_type,
            payload,
            auth: MessageAuth {
                token: token.to_string(),
                device_id: device_id.to_string(),
            },
        }
    }

    /// Create an emotion event message
    pub fn emotion_event(payload: serde_json::Value, device_id: &str, token: &str) -> Self {
        Self::new(MessageType::CareEmotion, payload, device_id, token)
    }

    /// Create a safety alert message
    pub fn safety_alert(payload: serde_json::Value, device_id: &str, token: &str) -> Self {
        Self::new(MessageType::SafetyAlert, payload, device_id, token)
    }

    /// Create an emergency message
    pub fn emergency(payload: serde_json::Value, device_id: &str, token: &str) -> Self {
        Self::new(MessageType::SafetyEmergency, payload, device_id, token)
    }

    /// Create a health vital message
    pub fn health_vital(payload: serde_json::Value, device_id: &str, token: &str) -> Self {
        Self::new(MessageType::HealthVital, payload, device_id, token)
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

/// Message types following CWP specification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    // Care events
    #[serde(rename = "care/emotion")]
    CareEmotion,
    #[serde(rename = "care/conversation")]
    CareConversation,

    // Health events
    #[serde(rename = "health/vital")]
    HealthVital,
    #[serde(rename = "health/medication")]
    HealthMedication,

    // Safety events
    #[serde(rename = "safety/alert")]
    SafetyAlert,
    #[serde(rename = "safety/emergency")]
    SafetyEmergency,

    // Family communication
    #[serde(rename = "family/call")]
    FamilyCall,
    #[serde(rename = "family/notification")]
    FamilyNotification,

    // Device management
    #[serde(rename = "device/status")]
    DeviceStatus,
    #[serde(rename = "device/command")]
    DeviceCommand,

    // System messages
    #[serde(rename = "system/ack")]
    SystemAck,
    #[serde(rename = "system/error")]
    SystemError,
    #[serde(rename = "system/ping")]
    SystemPing,
    #[serde(rename = "system/pong")]
    SystemPong,
}

impl MessageType {
    /// Get the priority level (1 = highest)
    pub fn priority(&self) -> u8 {
        match self {
            MessageType::SafetyEmergency => 1,
            MessageType::SafetyAlert => 2,
            MessageType::HealthVital => 3,
            MessageType::FamilyCall => 3,
            MessageType::HealthMedication => 4,
            MessageType::CareEmotion => 5,
            MessageType::CareConversation => 5,
            MessageType::FamilyNotification => 6,
            MessageType::DeviceStatus => 7,
            MessageType::DeviceCommand => 4,
            MessageType::SystemAck => 8,
            MessageType::SystemError => 3,
            MessageType::SystemPing => 9,
            MessageType::SystemPong => 9,
        }
    }

    /// Check if message requires acknowledgment
    pub fn requires_ack(&self) -> bool {
        matches!(
            self,
            MessageType::SafetyEmergency
                | MessageType::SafetyAlert
                | MessageType::HealthVital
                | MessageType::DeviceCommand
        )
    }

    /// Get the MQTT QoS level
    pub fn mqtt_qos(&self) -> u8 {
        match self {
            MessageType::SafetyEmergency => 2,
            MessageType::SafetyAlert => 2,
            MessageType::HealthVital => 1,
            MessageType::DeviceCommand => 1,
            _ => 0,
        }
    }
}

/// Message authentication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageAuth {
    /// JWT token
    pub token: String,
    /// Device ID
    pub device_id: String,
}

/// Message acknowledgment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageAck {
    /// Original message ID being acknowledged
    pub message_id: String,
    /// Acknowledgment status
    pub status: AckStatus,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Optional error message
    pub error: Option<String>,
}

impl MessageAck {
    /// Create a success acknowledgment
    pub fn success(message_id: &str) -> Self {
        Self {
            message_id: message_id.to_string(),
            status: AckStatus::Success,
            timestamp: Timestamp::now(),
            error: None,
        }
    }

    /// Create a failure acknowledgment
    pub fn failure(message_id: &str, error: &str) -> Self {
        Self {
            message_id: message_id.to_string(),
            status: AckStatus::Failed,
            timestamp: Timestamp::now(),
            error: Some(error.to_string()),
        }
    }
}

/// Acknowledgment status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AckStatus {
    Success,
    Failed,
    Pending,
    Timeout,
}

/// Batch message for offline sync
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatchMessage {
    /// Batch ID
    pub batch_id: String,
    /// Messages in batch
    pub messages: Vec<CwpMessage>,
    /// Total count
    pub total_count: usize,
    /// Batch index (for chunked uploads)
    pub batch_index: usize,
    /// Is last batch
    pub is_last: bool,
}

impl BatchMessage {
    /// Create a new batch message
    pub fn new(messages: Vec<CwpMessage>) -> Self {
        let count = messages.len();
        Self {
            batch_id: format!("batch-{}", uuid::Uuid::new_v4()),
            messages,
            total_count: count,
            batch_index: 0,
            is_last: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_creation() {
        let payload = serde_json::json!({"emotion": "happy", "confidence": 0.85});
        let msg = CwpMessage::emotion_event(payload, "carebot-001", "test-token");

        assert_eq!(msg.protocol, "cwp");
        assert_eq!(msg.message_type, MessageType::CareEmotion);
        assert_eq!(msg.auth.device_id, "carebot-001");
    }

    #[test]
    fn test_message_priority() {
        assert_eq!(MessageType::SafetyEmergency.priority(), 1);
        assert_eq!(MessageType::SafetyAlert.priority(), 2);
        assert_eq!(MessageType::SystemPing.priority(), 9);
    }

    #[test]
    fn test_message_serialization() {
        let payload = serde_json::json!({"test": true});
        let msg = CwpMessage::new(MessageType::DeviceStatus, payload, "carebot-001", "token");

        let json = msg.to_json().unwrap();
        let parsed = CwpMessage::from_json(&json).unwrap();

        assert_eq!(parsed.message_id, msg.message_id);
    }
}
