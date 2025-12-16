//! Message builder for WRP

use crate::protocol::message::*;
use chrono::Utc;
use std::sync::atomic::{AtomicU64, Ordering};

static SEQUENCE_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Builder for creating WRP messages
pub struct MessageBuilder {
    message_type: MessageType,
    priority: Option<Priority>,
    source: Option<Endpoint>,
    destination: Option<Endpoint>,
    safety: SafetyInfo,
    payload: serde_json::Value,
    with_checksum: bool,
}

impl MessageBuilder {
    /// Create a new message builder
    pub fn new(message_type: MessageType) -> Self {
        Self {
            message_type,
            priority: None,
            source: None,
            destination: None,
            safety: SafetyInfo::default(),
            payload: serde_json::json!({}),
            with_checksum: true,
        }
    }

    /// Create a telemetry message builder
    pub fn telemetry() -> Self {
        Self::new(MessageType::Telemetry)
    }

    /// Create a control message builder
    pub fn control() -> Self {
        Self::new(MessageType::Control)
            .requires_ack(true)
    }

    /// Create an emergency stop message builder
    pub fn emergency_stop() -> Self {
        Self::new(MessageType::EmergencyStop)
            .priority(Priority::Emergency)
            .safety_level(SafetyLevel::Emergency)
            .emergency(true)
            .requires_ack(true)
            .ack_timeout(100)
    }

    /// Create a heartbeat message builder
    pub fn heartbeat() -> Self {
        Self::new(MessageType::Heartbeat)
    }

    /// Create a handshake message builder
    pub fn handshake() -> Self {
        Self::new(MessageType::Handshake)
            .priority(Priority::High)
    }

    /// Set message priority
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = Some(priority);
        self
    }

    /// Set source endpoint
    pub fn source(mut self, endpoint: Endpoint) -> Self {
        self.source = Some(endpoint);
        self
    }

    /// Set source from device info
    pub fn from_device(mut self, device_id: &str, device_type: &str) -> Self {
        self.source = Some(Endpoint::new(device_id, device_type));
        self
    }

    /// Set destination endpoint
    pub fn destination(mut self, endpoint: Endpoint) -> Self {
        self.destination = Some(endpoint);
        self
    }

    /// Set destination to broadcast
    pub fn broadcast(mut self) -> Self {
        self.destination = Some(Endpoint::broadcast());
        self
    }

    /// Set destination from device info
    pub fn to_device(mut self, device_id: &str, device_type: &str) -> Self {
        self.destination = Some(Endpoint::new(device_id, device_type));
        self
    }

    /// Set emergency stop flag
    pub fn emergency(mut self, is_emergency: bool) -> Self {
        self.safety.emergency_stop = is_emergency;
        self
    }

    /// Set safety level
    pub fn safety_level(mut self, level: SafetyLevel) -> Self {
        self.safety.safety_level = level;
        self
    }

    /// Set requires acknowledgment
    pub fn requires_ack(mut self, requires: bool) -> Self {
        self.safety.requires_ack = requires;
        self
    }

    /// Set acknowledgment timeout
    pub fn ack_timeout(mut self, timeout_ms: u64) -> Self {
        self.safety.ack_timeout_ms = timeout_ms;
        self
    }

    /// Set message payload
    pub fn payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = payload;
        self
    }

    /// Set whether to include checksum
    pub fn with_checksum(mut self, include: bool) -> Self {
        self.with_checksum = include;
        self
    }

    /// Build the message
    pub fn build(self) -> WrpMessage {
        let priority = self.priority.unwrap_or_else(|| self.message_type.default_priority());
        let sequence = SEQUENCE_COUNTER.fetch_add(1, Ordering::SeqCst);

        let mut message = WrpMessage {
            protocol: "wia-robot".to_string(),
            version: "1.0.0".to_string(),
            message_id: uuid::Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            sequence,
            message_type: self.message_type,
            priority,
            source: self.source.unwrap_or_else(|| Endpoint::new("unknown", "unknown")),
            destination: self.destination.unwrap_or_else(|| Endpoint::new("unknown", "unknown")),
            safety: self.safety,
            payload: self.payload,
            checksum: None,
        };

        if self.with_checksum {
            message.calculate_checksum();
        }

        message
    }
}

/// Quick message creation functions
pub fn emergency_stop_message(source: Endpoint, reason: EStopReason) -> WrpMessage {
    MessageBuilder::emergency_stop()
        .source(source)
        .broadcast()
        .payload(serde_json::json!({
            "reason": reason,
            "timestamp": Utc::now().to_rfc3339()
        }))
        .build()
}

/// Create a telemetry message
pub fn telemetry_message(
    source: Endpoint,
    destination: Endpoint,
    data: serde_json::Value,
) -> WrpMessage {
    MessageBuilder::telemetry()
        .source(source)
        .destination(destination)
        .payload(data)
        .build()
}

/// Create a control message
pub fn control_message(
    source: Endpoint,
    destination: Endpoint,
    command: &str,
    parameters: serde_json::Value,
) -> WrpMessage {
    let command_id = uuid::Uuid::new_v4().to_string();
    MessageBuilder::control()
        .source(source)
        .destination(destination)
        .payload(serde_json::json!({
            "command": command,
            "command_id": command_id,
            "parameters": parameters
        }))
        .build()
}

/// Create a heartbeat message
pub fn heartbeat_message(source: Endpoint, destination: Endpoint) -> WrpMessage {
    MessageBuilder::heartbeat()
        .source(source)
        .destination(destination)
        .payload(serde_json::json!({
            "timestamp": Utc::now().to_rfc3339()
        }))
        .with_checksum(false)
        .build()
}

/// Create a handshake message
pub fn handshake_message(
    source: Endpoint,
    destination: Endpoint,
    capabilities: Vec<String>,
) -> WrpMessage {
    MessageBuilder::handshake()
        .source(source)
        .destination(destination)
        .payload(serde_json::json!({
            "protocol_version": "1.0.0",
            "device_info": {
                "capabilities": capabilities
            },
            "heartbeat_interval_ms": 1000
        }))
        .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_builder() {
        let msg = MessageBuilder::telemetry()
            .from_device("exo-001", "exoskeleton")
            .to_device("server-001", "server")
            .payload(serde_json::json!({"test": true}))
            .build();

        assert_eq!(msg.protocol, "wia-robot");
        assert_eq!(msg.message_type, MessageType::Telemetry);
        assert_eq!(msg.source.device_id, "exo-001");
        assert_eq!(msg.destination.device_id, "server-001");
        assert!(msg.checksum.is_some());
    }

    #[test]
    fn test_emergency_stop_builder() {
        let msg = MessageBuilder::emergency_stop()
            .from_device("exo-001", "exoskeleton")
            .build();

        assert_eq!(msg.message_type, MessageType::EmergencyStop);
        assert_eq!(msg.priority, Priority::Emergency);
        assert!(msg.safety.emergency_stop);
        assert!(msg.safety.requires_ack);
        assert_eq!(msg.safety.ack_timeout_ms, 100);
    }

    #[test]
    fn test_emergency_stop_message() {
        let source = Endpoint::new("exo-001", "exoskeleton");
        let msg = emergency_stop_message(source, EStopReason::FallDetected);

        assert_eq!(msg.destination.device_id, "broadcast");
        assert!(msg.is_emergency());
    }

    #[test]
    fn test_control_message() {
        let source = Endpoint::new("server-001", "server");
        let dest = Endpoint::new("exo-001", "exoskeleton");
        let msg = control_message(
            source,
            dest,
            "set_assist_level",
            serde_json::json!({"level": 0.75}),
        );

        assert_eq!(msg.message_type, MessageType::Control);
        assert!(msg.requires_ack());
    }

    #[test]
    fn test_checksum() {
        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("test", "test")
            .build();

        assert!(msg.verify_checksum());
    }

    #[test]
    fn test_sequence_increment() {
        let msg1 = MessageBuilder::heartbeat()
            .from_device("test", "test")
            .to_device("test", "test")
            .build();
        let msg2 = MessageBuilder::heartbeat()
            .from_device("test", "test")
            .to_device("test", "test")
            .build();

        assert!(msg2.sequence > msg1.sequence);
    }
}
