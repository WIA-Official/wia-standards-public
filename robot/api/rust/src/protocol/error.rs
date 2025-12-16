//! Protocol-specific error types

use serde::{Deserialize, Serialize};

/// Protocol error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u16)]
pub enum ProtocolErrorCode {
    // Message errors (1000-1999)
    InvalidMessage = 1000,
    ChecksumFailed = 1001,
    UnsupportedVersion = 1002,
    UnknownMessageType = 1003,

    // Connection errors (2000-2999)
    ConnectionFailed = 2000,
    HandshakeFailed = 2001,
    HeartbeatTimeout = 2002,
    AckTimeout = 2003,

    // Safety errors (3000-3999)
    SafetyViolation = 3000,
    EmergencyStop = 3001,
    SafetyLimitExceeded = 3002,

    // Device errors (4000-4999)
    DeviceError = 4000,
    DeviceOffline = 4001,
    DeviceBusy = 4002,

    // Command errors (5000-5999)
    CommandRejected = 5000,
    InvalidParameter = 5001,
    CommandTimeout = 5002,
}

impl ProtocolErrorCode {
    /// Get error name
    pub fn name(&self) -> &'static str {
        match self {
            Self::InvalidMessage => "INVALID_MESSAGE",
            Self::ChecksumFailed => "CHECKSUM_FAILED",
            Self::UnsupportedVersion => "UNSUPPORTED_VERSION",
            Self::UnknownMessageType => "UNKNOWN_MESSAGE_TYPE",
            Self::ConnectionFailed => "CONNECTION_FAILED",
            Self::HandshakeFailed => "HANDSHAKE_FAILED",
            Self::HeartbeatTimeout => "HEARTBEAT_TIMEOUT",
            Self::AckTimeout => "ACK_TIMEOUT",
            Self::SafetyViolation => "SAFETY_VIOLATION",
            Self::EmergencyStop => "EMERGENCY_STOP",
            Self::SafetyLimitExceeded => "SAFETY_LIMIT_EXCEEDED",
            Self::DeviceError => "DEVICE_ERROR",
            Self::DeviceOffline => "DEVICE_OFFLINE",
            Self::DeviceBusy => "DEVICE_BUSY",
            Self::CommandRejected => "COMMAND_REJECTED",
            Self::InvalidParameter => "INVALID_PARAMETER",
            Self::CommandTimeout => "COMMAND_TIMEOUT",
        }
    }

    /// Check if error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            Self::ChecksumFailed |
            Self::HeartbeatTimeout |
            Self::AckTimeout |
            Self::DeviceBusy |
            Self::CommandTimeout
        )
    }

    /// Get suggested action
    pub fn suggested_action(&self) -> SuggestedAction {
        match self {
            Self::ChecksumFailed => SuggestedAction::ResendMessage,
            Self::ConnectionFailed | Self::HeartbeatTimeout => SuggestedAction::Reconnect,
            Self::DeviceError => SuggestedAction::ResetDevice,
            Self::SafetyViolation | Self::EmergencyStop => SuggestedAction::ManualIntervention,
            Self::InvalidParameter => SuggestedAction::CheckParameters,
            Self::UnsupportedVersion => SuggestedAction::UpdateFirmware,
            _ => SuggestedAction::None,
        }
    }
}

/// Suggested actions for error recovery
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuggestedAction {
    ResendMessage,
    Reconnect,
    ResetDevice,
    ManualIntervention,
    ContactSupport,
    UpdateFirmware,
    Recalibrate,
    CheckParameters,
    None,
}

/// Protocol error payload for error messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolErrorPayload {
    pub error_code: u16,
    pub error_name: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    pub recoverable: bool,
    pub suggested_action: SuggestedAction,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub related_message_id: Option<String>,
}

impl ProtocolErrorPayload {
    /// Create a new error payload
    pub fn new(code: ProtocolErrorCode, message: &str) -> Self {
        Self {
            error_code: code as u16,
            error_name: code.name().to_string(),
            message: message.to_string(),
            details: None,
            recoverable: code.is_recoverable(),
            suggested_action: code.suggested_action(),
            related_message_id: None,
        }
    }

    /// Add details to error
    pub fn with_details(mut self, details: serde_json::Value) -> Self {
        self.details = Some(details);
        self
    }

    /// Add related message ID
    pub fn with_related_message(mut self, message_id: &str) -> Self {
        self.related_message_id = Some(message_id.to_string());
        self
    }

    /// Convert to JSON value for message payload
    pub fn to_payload(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap_or_default()
    }
}

/// Create error message helpers
pub mod errors {
    use super::*;

    pub fn checksum_failed(expected: &str, actual: &str, message_id: &str) -> ProtocolErrorPayload {
        ProtocolErrorPayload::new(
            ProtocolErrorCode::ChecksumFailed,
            "Message checksum verification failed",
        )
        .with_details(serde_json::json!({
            "expected": expected,
            "actual": actual
        }))
        .with_related_message(message_id)
    }

    pub fn heartbeat_timeout(device_id: &str, last_seen: &str) -> ProtocolErrorPayload {
        ProtocolErrorPayload::new(
            ProtocolErrorCode::HeartbeatTimeout,
            "Heartbeat timeout - device may be offline",
        )
        .with_details(serde_json::json!({
            "device_id": device_id,
            "last_seen": last_seen
        }))
    }

    pub fn emergency_stop(source: &str, reason: &str) -> ProtocolErrorPayload {
        ProtocolErrorPayload::new(
            ProtocolErrorCode::EmergencyStop,
            "Emergency stop has been triggered",
        )
        .with_details(serde_json::json!({
            "source": source,
            "reason": reason,
            "timestamp": chrono::Utc::now().to_rfc3339()
        }))
    }

    pub fn invalid_parameter(param: &str, expected: &str, actual: &str) -> ProtocolErrorPayload {
        ProtocolErrorPayload::new(
            ProtocolErrorCode::InvalidParameter,
            &format!("Invalid parameter: {}", param),
        )
        .with_details(serde_json::json!({
            "parameter": param,
            "expected": expected,
            "actual": actual
        }))
    }

    pub fn command_rejected(command: &str, reason: &str) -> ProtocolErrorPayload {
        ProtocolErrorPayload::new(
            ProtocolErrorCode::CommandRejected,
            &format!("Command rejected: {}", command),
        )
        .with_details(serde_json::json!({
            "command": command,
            "reason": reason
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_code_properties() {
        assert!(ProtocolErrorCode::ChecksumFailed.is_recoverable());
        assert!(!ProtocolErrorCode::EmergencyStop.is_recoverable());

        assert_eq!(
            ProtocolErrorCode::ChecksumFailed.suggested_action(),
            SuggestedAction::ResendMessage
        );
    }

    #[test]
    fn test_error_payload() {
        let error = errors::checksum_failed("abc123", "xyz789", "msg-001");

        assert_eq!(error.error_code, 1001);
        assert_eq!(error.error_name, "CHECKSUM_FAILED");
        assert!(error.recoverable);
        assert!(error.details.is_some());
        assert_eq!(error.related_message_id, Some("msg-001".to_string()));
    }

    #[test]
    fn test_error_to_payload() {
        let error = errors::emergency_stop("UserButton", "Manual stop");
        let payload = error.to_payload();

        assert!(payload.is_object());
        assert_eq!(payload["error_code"], 3001);
    }
}
