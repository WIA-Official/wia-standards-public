//! Protocol error types for WIA Space Protocol

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Protocol error type
#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error("Connection error: {0}")]
    Connection(String),

    #[error("Not connected")]
    NotConnected,

    #[error("Connection timeout: {0}ms")]
    Timeout(u64),

    #[error("Invalid message: {0}")]
    InvalidMessage(String),

    #[error("Unsupported message type: {0}")]
    UnsupportedMessageType(String),

    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    #[error("Channel closed")]
    ChannelClosed,

    #[error("Session not found: {0}")]
    SessionNotFound(String),

    #[error("Protocol version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: String, actual: String },

    #[error("Authentication failed: {0}")]
    AuthFailed(String),

    #[error("Permission denied: {0}")]
    PermissionDenied(String),

    #[error("Space error: {0}")]
    SpaceError(#[from] crate::error::SpaceError),

    #[error("Transport error: {0}")]
    Transport(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// Error code enumeration matching the protocol spec
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u32)]
pub enum ErrorCode {
    // Connection errors (1xxx)
    ConnectionClosed = 1000,
    ConnectionLost = 1001,
    ConnectionTimeout = 1002,
    ProtocolError = 1003,
    VersionMismatch = 1004,

    // Communication errors (2xxx)
    SpacecraftUnreachable = 2001,
    GroundStationBusy = 2002,
    LinkDegraded = 2003,
    SignalLost = 2004,
    LatencyExceeded = 2005,

    // Data errors (3xxx)
    InvalidMessage = 3001,
    PayloadTooLarge = 3002,
    ChecksumFailed = 3003,
    SequenceGap = 3004,
    DuplicateMessage = 3005,

    // Mission errors (4xxx)
    MissionNotFound = 4001,
    InvalidCommand = 4002,
    CommandRejected = 4003,
    SubsystemOffline = 4004,
    ResourceExhausted = 4005,

    // Authentication errors (5xxx)
    AuthRequired = 5001,
    AuthFailed = 5002,
    PermissionDenied = 5003,
    SessionExpired = 5004,
}

impl ErrorCode {
    /// Get error name
    pub fn name(&self) -> &'static str {
        match self {
            ErrorCode::ConnectionClosed => "CONNECTION_CLOSED",
            ErrorCode::ConnectionLost => "CONNECTION_LOST",
            ErrorCode::ConnectionTimeout => "CONNECTION_TIMEOUT",
            ErrorCode::ProtocolError => "PROTOCOL_ERROR",
            ErrorCode::VersionMismatch => "VERSION_MISMATCH",
            ErrorCode::SpacecraftUnreachable => "SPACECRAFT_UNREACHABLE",
            ErrorCode::GroundStationBusy => "GROUND_STATION_BUSY",
            ErrorCode::LinkDegraded => "LINK_DEGRADED",
            ErrorCode::SignalLost => "SIGNAL_LOST",
            ErrorCode::LatencyExceeded => "LATENCY_EXCEEDED",
            ErrorCode::InvalidMessage => "INVALID_MESSAGE",
            ErrorCode::PayloadTooLarge => "PAYLOAD_TOO_LARGE",
            ErrorCode::ChecksumFailed => "CHECKSUM_FAILED",
            ErrorCode::SequenceGap => "SEQUENCE_GAP",
            ErrorCode::DuplicateMessage => "DUPLICATE_MESSAGE",
            ErrorCode::MissionNotFound => "MISSION_NOT_FOUND",
            ErrorCode::InvalidCommand => "INVALID_COMMAND",
            ErrorCode::CommandRejected => "COMMAND_REJECTED",
            ErrorCode::SubsystemOffline => "SUBSYSTEM_OFFLINE",
            ErrorCode::ResourceExhausted => "RESOURCE_EXHAUSTED",
            ErrorCode::AuthRequired => "AUTH_REQUIRED",
            ErrorCode::AuthFailed => "AUTH_FAILED",
            ErrorCode::PermissionDenied => "PERMISSION_DENIED",
            ErrorCode::SessionExpired => "SESSION_EXPIRED",
        }
    }

    /// Get numeric code
    pub fn code(&self) -> u32 {
        *self as u32
    }

    /// Check if error is recoverable
    pub fn is_recoverable(&self) -> bool {
        (*self as u32) < 5000
    }

    /// Get suggested retry delay in seconds
    pub fn retry_delay_s(&self) -> Option<u32> {
        match self {
            ErrorCode::ConnectionLost | ErrorCode::ConnectionTimeout => Some(30),
            ErrorCode::SpacecraftUnreachable | ErrorCode::GroundStationBusy => Some(60),
            ErrorCode::LinkDegraded | ErrorCode::LatencyExceeded => Some(120),
            ErrorCode::SequenceGap => Some(5),
            _ => None,
        }
    }

    /// Get error category
    pub fn category(&self) -> ErrorCategory {
        match (*self as u32) / 1000 {
            1 => ErrorCategory::Connection,
            2 => ErrorCategory::Communication,
            3 => ErrorCategory::Data,
            4 => ErrorCategory::Mission,
            5 => ErrorCategory::Authentication,
            _ => ErrorCategory::Unknown,
        }
    }
}

/// Error category
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCategory {
    Connection,
    Communication,
    Data,
    Mission,
    Authentication,
    Unknown,
}

/// Protocol error payload for messages
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorPayload {
    pub code: u32,
    pub name: String,
    pub message: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,

    #[serde(default)]
    pub recoverable: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after_s: Option<u32>,
}

impl ErrorPayload {
    /// Create error payload from error code
    pub fn from_code(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            code: code.code(),
            name: code.name().to_string(),
            message: message.into(),
            details: None,
            recoverable: code.is_recoverable(),
            retry_after_s: code.retry_delay_s(),
        }
    }

    /// Add details to error payload
    pub fn with_details(mut self, details: serde_json::Value) -> Self {
        self.details = Some(details);
        self
    }
}

impl From<ErrorCode> for ErrorPayload {
    fn from(code: ErrorCode) -> Self {
        Self::from_code(code, code.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_code_properties() {
        let code = ErrorCode::ConnectionLost;
        assert_eq!(code.code(), 1001);
        assert_eq!(code.name(), "CONNECTION_LOST");
        assert!(code.is_recoverable());
        assert_eq!(code.retry_delay_s(), Some(30));
        assert_eq!(code.category(), ErrorCategory::Connection);
    }

    #[test]
    fn test_error_categories() {
        assert_eq!(ErrorCode::ConnectionClosed.category(), ErrorCategory::Connection);
        assert_eq!(ErrorCode::SpacecraftUnreachable.category(), ErrorCategory::Communication);
        assert_eq!(ErrorCode::InvalidMessage.category(), ErrorCategory::Data);
        assert_eq!(ErrorCode::MissionNotFound.category(), ErrorCategory::Mission);
        assert_eq!(ErrorCode::AuthFailed.category(), ErrorCategory::Authentication);
    }

    #[test]
    fn test_error_payload() {
        let payload = ErrorPayload::from_code(
            ErrorCode::SpacecraftUnreachable,
            "Cannot contact mars-orbiter-01",
        )
        .with_details(serde_json::json!({
            "lastContact": "2035-03-15T11:45:00Z"
        }));

        assert_eq!(payload.code, 2001);
        assert_eq!(payload.name, "SPACECRAFT_UNREACHABLE");
        assert!(payload.recoverable);
        assert!(payload.details.is_some());
    }

    #[test]
    fn test_auth_not_recoverable() {
        let code = ErrorCode::AuthFailed;
        assert!(!code.is_recoverable());
        assert!(code.retry_delay_s().is_none());
    }

    #[test]
    fn test_error_payload_serialization() {
        let payload = ErrorPayload::from_code(ErrorCode::InvalidMessage, "Test error");
        let json = serde_json::to_string(&payload).unwrap();
        assert!(json.contains("INVALID_MESSAGE"));
        assert!(json.contains("3001"));
    }
}
