//! Protocol message types and envelope structure
//!
//! Implements the WIA Material Protocol message format as defined in Phase 3.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::types::MaterialData;
use crate::MaterialType;

use super::{PROTOCOL_ID, PROTOCOL_VERSION};

// ============================================================================
// Message Types
// ============================================================================

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    // Request types (Client → Server)
    Query,
    Get,
    Create,
    Update,
    Delete,
    Subscribe,
    Unsubscribe,
    Command,
    Ping,
    Handshake,
    FlowControl,

    // Response types (Server → Client)
    QueryResponse,
    GetResponse,
    CreateResponse,
    UpdateResponse,
    DeleteResponse,
    SubscribeAck,
    UnsubscribeAck,
    Data,
    CommandAck,
    Pong,
    HandshakeAck,
    Error,
}

impl MessageType {
    /// Check if this is a request type
    pub fn is_request(&self) -> bool {
        matches!(
            self,
            MessageType::Query
                | MessageType::Get
                | MessageType::Create
                | MessageType::Update
                | MessageType::Delete
                | MessageType::Subscribe
                | MessageType::Unsubscribe
                | MessageType::Command
                | MessageType::Ping
                | MessageType::Handshake
                | MessageType::FlowControl
        )
    }

    /// Check if this is a response type
    pub fn is_response(&self) -> bool {
        !self.is_request()
    }
}

// ============================================================================
// Message Envelope
// ============================================================================

/// Protocol message envelope
///
/// All messages in the WIA Material Protocol use this common envelope structure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message<T: Serialize> {
    /// Protocol identifier (always "wia-material")
    pub protocol: String,

    /// Protocol version (semver)
    pub version: String,

    /// Unique message identifier (UUID v4)
    pub message_id: String,

    /// ISO 8601 UTC timestamp
    pub timestamp: DateTime<Utc>,

    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,

    /// Optional correlation ID linking response to request
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,

    /// Type-specific payload
    pub payload: T,
}

impl<T: Serialize> Message<T> {
    /// Create a new message with the given type and payload
    pub fn new(message_type: MessageType, payload: T) -> Self {
        Self {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            message_type,
            correlation_id: None,
            payload,
        }
    }

    /// Create a new message with correlation ID
    pub fn with_correlation(message_type: MessageType, payload: T, correlation_id: &str) -> Self {
        Self {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            message_type,
            correlation_id: Some(correlation_id.to_string()),
            payload,
        }
    }

    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Serialize to pretty-printed JSON string
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

// ============================================================================
// Request Payloads
// ============================================================================

/// Query request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPayload {
    /// Material type filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material_type: Option<MaterialType>,

    /// Filter conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<super::Filter>,

    /// Sort configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sort: Option<SortConfig>,

    /// Pagination configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pagination: Option<Pagination>,

    /// Fields to include in response
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fields: Option<Vec<String>>,
}

impl Default for QueryPayload {
    fn default() -> Self {
        Self {
            material_type: None,
            filter: None,
            sort: None,
            pagination: Some(Pagination::default()),
            fields: None,
        }
    }
}

/// Sort configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SortConfig {
    /// Field to sort by
    pub field: String,

    /// Sort order
    #[serde(default)]
    pub order: SortOrder,
}

/// Sort order
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SortOrder {
    #[default]
    Asc,
    Desc,
}

/// Pagination configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pagination {
    /// Number of items to skip
    #[serde(default)]
    pub offset: usize,

    /// Maximum number of items to return
    #[serde(default = "default_limit")]
    pub limit: usize,
}

fn default_limit() -> usize {
    100
}

impl Default for Pagination {
    fn default() -> Self {
        Self {
            offset: 0,
            limit: default_limit(),
        }
    }
}

/// Get request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetPayload {
    /// Material ID to retrieve
    pub material_id: String,

    /// Fields to include
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fields: Option<Vec<String>>,
}

/// Create request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreatePayload {
    /// Material data to create
    pub material: MaterialData,
}

/// Update request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdatePayload {
    /// Material ID to update
    pub material_id: String,

    /// Updated material data
    pub material: MaterialData,

    /// Whether to perform partial update
    #[serde(default)]
    pub partial: bool,
}

/// Delete request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeletePayload {
    /// Material ID to delete
    pub material_id: String,
}

/// Subscribe request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribePayload {
    /// Channel to subscribe to
    pub channel: SubscriptionChannel,

    /// Material type filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material_type: Option<MaterialType>,

    /// Optional filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filter: Option<super::Filter>,
}

/// Subscription channels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SubscriptionChannel {
    Materials,
    Measurements,
    Instruments,
    Alerts,
}

/// Unsubscribe request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnsubscribePayload {
    /// Subscription ID to cancel
    pub subscription_id: String,
}

/// Command request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPayload {
    /// Command name
    pub command: String,

    /// Command arguments
    #[serde(default)]
    pub args: serde_json::Value,
}

/// Handshake request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandshakePayload {
    /// Requested protocol version
    pub protocol_version: String,

    /// Client identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub client_id: Option<String>,

    /// Authentication token
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth_token: Option<String>,
}

impl Default for HandshakePayload {
    fn default() -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION.to_string(),
            client_id: Some(Uuid::new_v4().to_string()),
            auth_token: None,
        }
    }
}

/// Flow control request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlowControlPayload {
    /// Channel to control
    pub channel: SubscriptionChannel,

    /// Action to take
    pub action: FlowAction,
}

/// Flow control actions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FlowAction {
    Pause,
    Resume,
}

/// Empty payload for ping/pong
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EmptyPayload {}

// ============================================================================
// Response Payloads
// ============================================================================

/// Query response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryResponsePayload {
    /// Matching materials
    pub data: Vec<MaterialData>,

    /// Response metadata
    pub meta: QueryMeta,
}

/// Query response metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryMeta {
    /// Total number of matching materials
    pub total_count: usize,

    /// Number of materials returned
    pub returned_count: usize,

    /// Offset used
    pub offset: usize,

    /// Whether there are more results
    pub has_more: bool,
}

/// Get response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GetResponsePayload {
    /// Retrieved material
    pub data: MaterialData,
}

/// Create response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateResponsePayload {
    /// Created material with assigned ID
    pub data: MaterialData,
}

/// Update response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdateResponsePayload {
    /// Updated material
    pub data: MaterialData,
}

/// Delete response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeleteResponsePayload {
    /// Deleted material ID
    pub material_id: String,

    /// Whether deletion was successful
    pub success: bool,
}

/// Subscribe acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribeAckPayload {
    /// Assigned subscription ID
    pub subscription_id: String,

    /// Channel subscribed to
    pub channel: SubscriptionChannel,
}

/// Unsubscribe acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnsubscribeAckPayload {
    /// Unsubscribed subscription ID
    pub subscription_id: String,

    /// Success status
    pub success: bool,
}

/// Data stream payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPayload {
    /// Channel this data is from
    pub channel: SubscriptionChannel,

    /// Sequence number
    pub sequence: u64,

    /// Material data
    pub material: MaterialData,
}

/// Command acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandAckPayload {
    /// Command that was executed
    pub command: String,

    /// Success status
    pub success: bool,

    /// Result data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,
}

/// Handshake acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandshakeAckPayload {
    /// Assigned session ID
    pub session_id: String,

    /// Server protocol version
    pub server_version: String,

    /// Available capabilities
    pub capabilities: Vec<String>,
}

/// Error response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPayload {
    /// Error code
    pub code: ErrorCode,

    /// Human-readable message
    pub message: String,

    /// Additional error details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

/// Error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ErrorCode {
    InvalidRequest,
    ValidationError,
    AuthRequired,
    AuthInvalid,
    Forbidden,
    NotFound,
    MaterialNotFound,
    DuplicateId,
    RateLimited,
    InternalError,
    ServiceUnavailable,
}

impl ErrorCode {
    /// Get HTTP status code equivalent
    pub fn http_status(&self) -> u16 {
        match self {
            ErrorCode::InvalidRequest => 400,
            ErrorCode::ValidationError => 422,
            ErrorCode::AuthRequired => 401,
            ErrorCode::AuthInvalid => 401,
            ErrorCode::Forbidden => 403,
            ErrorCode::NotFound => 404,
            ErrorCode::MaterialNotFound => 404,
            ErrorCode::DuplicateId => 409,
            ErrorCode::RateLimited => 429,
            ErrorCode::InternalError => 500,
            ErrorCode::ServiceUnavailable => 503,
        }
    }
}

// ============================================================================
// Message Builders
// ============================================================================

/// Builder for creating protocol messages
pub struct MessageBuilder;

impl MessageBuilder {
    /// Create a query request
    pub fn query(payload: QueryPayload) -> Message<QueryPayload> {
        Message::new(MessageType::Query, payload)
    }

    /// Create a get request
    pub fn get(material_id: &str) -> Message<GetPayload> {
        Message::new(
            MessageType::Get,
            GetPayload {
                material_id: material_id.to_string(),
                fields: None,
            },
        )
    }

    /// Create a create request
    pub fn create(material: MaterialData) -> Message<CreatePayload> {
        Message::new(MessageType::Create, CreatePayload { material })
    }

    /// Create an update request
    pub fn update(material_id: &str, material: MaterialData) -> Message<UpdatePayload> {
        Message::new(
            MessageType::Update,
            UpdatePayload {
                material_id: material_id.to_string(),
                material,
                partial: false,
            },
        )
    }

    /// Create a delete request
    pub fn delete(material_id: &str) -> Message<DeletePayload> {
        Message::new(
            MessageType::Delete,
            DeletePayload {
                material_id: material_id.to_string(),
            },
        )
    }

    /// Create a subscribe request
    pub fn subscribe(channel: SubscriptionChannel) -> Message<SubscribePayload> {
        Message::new(
            MessageType::Subscribe,
            SubscribePayload {
                channel,
                material_type: None,
                filter: None,
            },
        )
    }

    /// Create an unsubscribe request
    pub fn unsubscribe(subscription_id: &str) -> Message<UnsubscribePayload> {
        Message::new(
            MessageType::Unsubscribe,
            UnsubscribePayload {
                subscription_id: subscription_id.to_string(),
            },
        )
    }

    /// Create a handshake request
    pub fn handshake() -> Message<HandshakePayload> {
        Message::new(MessageType::Handshake, HandshakePayload::default())
    }

    /// Create a handshake with auth token
    pub fn handshake_with_auth(auth_token: &str) -> Message<HandshakePayload> {
        Message::new(
            MessageType::Handshake,
            HandshakePayload {
                protocol_version: PROTOCOL_VERSION.to_string(),
                client_id: Some(Uuid::new_v4().to_string()),
                auth_token: Some(auth_token.to_string()),
            },
        )
    }

    /// Create a ping request
    pub fn ping() -> Message<EmptyPayload> {
        Message::new(MessageType::Ping, EmptyPayload {})
    }

    /// Create a pong response
    pub fn pong(correlation_id: &str) -> Message<EmptyPayload> {
        Message::with_correlation(MessageType::Pong, EmptyPayload {}, correlation_id)
    }

    /// Create an error response
    pub fn error(code: ErrorCode, message: &str) -> Message<ErrorPayload> {
        Message::new(
            MessageType::Error,
            ErrorPayload {
                code,
                message: message.to_string(),
                details: None,
            },
        )
    }

    /// Create an error response with correlation
    pub fn error_with_correlation(
        code: ErrorCode,
        message: &str,
        correlation_id: &str,
    ) -> Message<ErrorPayload> {
        Message::with_correlation(
            MessageType::Error,
            ErrorPayload {
                code,
                message: message.to_string(),
                details: None,
            },
            correlation_id,
        )
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_creation() {
        let msg = MessageBuilder::ping();
        assert_eq!(msg.protocol, PROTOCOL_ID);
        assert_eq!(msg.version, PROTOCOL_VERSION);
        assert_eq!(msg.message_type, MessageType::Ping);
        assert!(!msg.message_id.is_empty());
    }

    #[test]
    fn test_message_serialization() {
        let msg = MessageBuilder::get("wia-mat-00000001");
        let json = msg.to_json().unwrap();
        assert!(json.contains("wia-material"));
        assert!(json.contains("wia-mat-00000001"));
    }

    #[test]
    fn test_query_payload() {
        let payload = QueryPayload {
            material_type: Some(MaterialType::Superconductor),
            filter: None,
            sort: Some(SortConfig {
                field: "timestamp".to_string(),
                order: SortOrder::Desc,
            }),
            pagination: Some(Pagination {
                offset: 0,
                limit: 50,
            }),
            fields: None,
        };

        let msg = MessageBuilder::query(payload);
        let json = msg.to_json().unwrap();
        assert!(json.contains("superconductor"));
        assert!(json.contains("\"limit\":50"));
    }

    #[test]
    fn test_error_codes() {
        assert_eq!(ErrorCode::InvalidRequest.http_status(), 400);
        assert_eq!(ErrorCode::MaterialNotFound.http_status(), 404);
        assert_eq!(ErrorCode::RateLimited.http_status(), 429);
    }

    #[test]
    fn test_message_types() {
        assert!(MessageType::Query.is_request());
        assert!(!MessageType::Query.is_response());
        assert!(MessageType::QueryResponse.is_response());
        assert!(!MessageType::QueryResponse.is_request());
    }

    #[test]
    fn test_handshake() {
        let msg = MessageBuilder::handshake_with_auth("test-token");
        let json = msg.to_json().unwrap();
        assert!(json.contains("test-token"));
        assert!(json.contains("handshake"));
    }
}
