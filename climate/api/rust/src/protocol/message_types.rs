//! Message payload types for different protocol messages

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Authentication
// ============================================================================

/// Authentication method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthMethod {
    /// API key authentication
    ApiKey,
    /// JWT token authentication
    Jwt,
    /// OAuth2 authentication
    Oauth2,
    /// No authentication
    None,
}

/// Authentication information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthInfo {
    /// Authentication method
    pub method: AuthMethod,
    /// Authentication token
    #[serde(skip_serializing_if = "Option::is_none")]
    pub token: Option<String>,
}

// ============================================================================
// Client Types
// ============================================================================

/// Type of client connecting to the server
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClientType {
    /// Physical sensor device
    Sensor,
    /// Gateway device aggregating multiple sensors
    Gateway,
    /// Dashboard or visualization client
    Dashboard,
    /// Backend service
    Service,
    /// Other client type
    Other,
}

// ============================================================================
// Connect / ConnectAck
// ============================================================================

/// Payload for connect message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectPayload {
    /// Unique client identifier
    pub client_id: String,

    /// Type of client
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_type: Option<ClientType>,

    /// Supported data types/capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,

    /// Authentication information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<AuthInfo>,

    /// Additional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<HashMap<String, serde_json::Value>>,
}

/// Server information in connect_ack
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerInfo {
    /// Server name
    pub name: String,
    /// Server version
    pub version: String,
}

/// Payload for connect_ack message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConnectAckPayload {
    /// Connection success status
    pub success: bool,

    /// Session identifier (on success)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,

    /// Server information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_info: Option<ServerInfo>,

    /// Keep-alive interval in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub keep_alive_interval: Option<u64>,

    /// Error information (on failure)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorPayload>,
}

impl ConnectAckPayload {
    /// Create a successful connect acknowledgment
    pub fn success(session_id: String) -> Self {
        Self {
            success: true,
            session_id: Some(session_id),
            server_info: Some(ServerInfo {
                name: "WIA Climate Server".to_string(),
                version: "1.0.0".to_string(),
            }),
            keep_alive_interval: Some(30000),
            error: None,
        }
    }

    /// Create a failed connect acknowledgment
    pub fn failure(error: ErrorPayload) -> Self {
        Self {
            success: false,
            session_id: None,
            server_info: None,
            keep_alive_interval: None,
            error: Some(error),
        }
    }
}

// ============================================================================
// Disconnect
// ============================================================================

/// Reason for disconnection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisconnectReason {
    /// Normal disconnect initiated by client
    Normal,
    /// Disconnect due to error
    Error,
    /// Disconnect due to timeout
    Timeout,
    /// Authentication expired
    AuthExpired,
    /// Server shutdown
    ServerShutdown,
}

/// Payload for disconnect message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisconnectPayload {
    /// Reason for disconnection
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<DisconnectReason>,

    /// Human-readable message
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

impl DisconnectPayload {
    /// Create a normal disconnect payload
    pub fn normal() -> Self {
        Self {
            reason: Some(DisconnectReason::Normal),
            message: None,
        }
    }

    /// Create an error disconnect payload
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            reason: Some(DisconnectReason::Error),
            message: Some(message.into()),
        }
    }
}

// ============================================================================
// Command / CommandAck
// ============================================================================

/// Payload for command message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandPayload {
    /// Target device or service ID
    pub target_id: String,

    /// Command action name
    pub action: String,

    /// Command parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<HashMap<String, serde_json::Value>>,

    /// Command timeout in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout: Option<u64>,
}

/// Payload for command_ack message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CommandAckPayload {
    /// Original command message ID
    pub command_id: String,

    /// Command execution success status
    pub success: bool,

    /// Command result (on success)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<HashMap<String, serde_json::Value>>,

    /// Error information (on failure)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorPayload>,

    /// Execution time in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution_time: Option<u64>,
}

impl CommandAckPayload {
    /// Create a successful command acknowledgment
    pub fn success(command_id: String, result: Option<HashMap<String, serde_json::Value>>) -> Self {
        Self {
            command_id,
            success: true,
            result,
            error: None,
            execution_time: None,
        }
    }

    /// Create a failed command acknowledgment
    pub fn failure(command_id: String, error: ErrorPayload) -> Self {
        Self {
            command_id,
            success: false,
            result: None,
            error: Some(error),
            execution_time: None,
        }
    }
}

// ============================================================================
// Subscribe / SubscribeAck / Unsubscribe
// ============================================================================

/// Topic subscription with optional filter
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicSubscription {
    /// Topic pattern (supports * and ** wildcards)
    pub pattern: String,

    /// Data filter conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<HashMap<String, serde_json::Value>>,
}

/// Quality of Service level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum QoS {
    /// At most once (fire and forget)
    #[serde(rename = "0")]
    AtMostOnce = 0,
    /// At least once (guaranteed delivery, may duplicate)
    #[serde(rename = "1")]
    AtLeastOnce = 1,
    /// Exactly once (guaranteed single delivery)
    #[serde(rename = "2")]
    ExactlyOnce = 2,
}

impl Default for QoS {
    fn default() -> Self {
        QoS::AtMostOnce
    }
}

/// Payload for subscribe message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribePayload {
    /// Topics to subscribe to
    pub topics: Vec<TopicSubscription>,

    /// Quality of Service level
    #[serde(default)]
    pub qos: QoS,
}

/// Payload for subscribe_ack message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SubscribeAckPayload {
    /// Subscription identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subscription_id: Option<String>,

    /// Subscribed topics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub topics: Option<Vec<String>>,

    /// Subscription success status
    pub success: bool,

    /// Error information (on failure)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorPayload>,
}

impl SubscribeAckPayload {
    /// Create a successful subscription acknowledgment
    pub fn success(subscription_id: String, topics: Vec<String>) -> Self {
        Self {
            subscription_id: Some(subscription_id),
            topics: Some(topics),
            success: true,
            error: None,
        }
    }

    /// Create a failed subscription acknowledgment
    pub fn failure(error: ErrorPayload) -> Self {
        Self {
            subscription_id: None,
            topics: None,
            success: false,
            error: Some(error),
        }
    }
}

/// Payload for unsubscribe message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UnsubscribePayload {
    /// Subscription ID to cancel
    pub subscription_id: String,
}

// ============================================================================
// Error
// ============================================================================

/// Error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ErrorCode {
    /// Protocol format error
    ProtocolError,
    /// Protocol version mismatch
    VersionMismatch,
    /// Authentication failed
    AuthFailed,
    /// Authentication token expired
    AuthExpired,
    /// Permission denied
    PermissionDenied,
    /// Data validation error
    ValidationError,
    /// Resource not found
    NotFound,
    /// Conflict (e.g., duplicate subscription)
    Conflict,
    /// Rate limit exceeded
    RateLimited,
    /// Operation timeout
    Timeout,
    /// Internal server error
    InternalError,
    /// Service unavailable
    ServiceUnavailable,
}

impl std::fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ErrorCode::ProtocolError => write!(f, "PROTOCOL_ERROR"),
            ErrorCode::VersionMismatch => write!(f, "VERSION_MISMATCH"),
            ErrorCode::AuthFailed => write!(f, "AUTH_FAILED"),
            ErrorCode::AuthExpired => write!(f, "AUTH_EXPIRED"),
            ErrorCode::PermissionDenied => write!(f, "PERMISSION_DENIED"),
            ErrorCode::ValidationError => write!(f, "VALIDATION_ERROR"),
            ErrorCode::NotFound => write!(f, "NOT_FOUND"),
            ErrorCode::Conflict => write!(f, "CONFLICT"),
            ErrorCode::RateLimited => write!(f, "RATE_LIMITED"),
            ErrorCode::Timeout => write!(f, "TIMEOUT"),
            ErrorCode::InternalError => write!(f, "INTERNAL_ERROR"),
            ErrorCode::ServiceUnavailable => write!(f, "SERVICE_UNAVAILABLE"),
        }
    }
}

/// Payload for error message
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ErrorPayload {
    /// Error code
    pub code: ErrorCode,

    /// Human-readable error message
    pub message: String,

    /// Additional error details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<HashMap<String, serde_json::Value>>,

    /// Related message ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub related_message_id: Option<String>,

    /// Whether the operation can be retried
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retryable: Option<bool>,

    /// Suggested retry delay in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after: Option<u64>,
}

impl ErrorPayload {
    /// Create a new error payload
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
            details: None,
            related_message_id: None,
            retryable: None,
            retry_after: None,
        }
    }

    /// Create a validation error
    pub fn validation_error(message: impl Into<String>, field: Option<&str>) -> Self {
        let mut error = Self::new(ErrorCode::ValidationError, message);
        if let Some(f) = field {
            let mut details = HashMap::new();
            details.insert("field".to_string(), serde_json::Value::String(f.to_string()));
            error.details = Some(details);
        }
        error
    }

    /// Create an auth failed error
    pub fn auth_failed(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::AuthFailed, message)
    }

    /// Create an internal error
    pub fn internal_error(message: impl Into<String>) -> Self {
        Self::new(ErrorCode::InternalError, message)
    }

    /// Set the related message ID
    pub fn with_related_message(mut self, message_id: String) -> Self {
        self.related_message_id = Some(message_id);
        self
    }

    /// Mark as retryable with optional delay
    pub fn with_retry(mut self, delay_ms: Option<u64>) -> Self {
        self.retryable = Some(true);
        self.retry_after = delay_ms;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connect_payload() {
        let payload = ConnectPayload {
            client_id: "test-client".to_string(),
            client_type: Some(ClientType::Sensor),
            capabilities: Some(vec!["carbon_capture".to_string()]),
            auth: Some(AuthInfo {
                method: AuthMethod::ApiKey,
                token: Some("test-token".to_string()),
            }),
            metadata: None,
        };

        let json = serde_json::to_string(&payload).unwrap();
        let parsed: ConnectPayload = serde_json::from_str(&json).unwrap();

        assert_eq!(parsed.client_id, "test-client");
        assert_eq!(parsed.client_type, Some(ClientType::Sensor));
    }

    #[test]
    fn test_error_payload() {
        let error = ErrorPayload::validation_error("Invalid latitude", Some("location.latitude"))
            .with_related_message("msg-123".to_string())
            .with_retry(Some(5000));

        assert_eq!(error.code, ErrorCode::ValidationError);
        assert!(error.retryable.unwrap());
        assert_eq!(error.retry_after, Some(5000));
    }

    #[test]
    fn test_qos_default() {
        let payload = SubscribePayload {
            topics: vec![TopicSubscription {
                pattern: "carbon_capture/*".to_string(),
                filter: None,
            }],
            qos: QoS::default(),
        };

        assert_eq!(payload.qos, QoS::AtMostOnce);
    }
}
