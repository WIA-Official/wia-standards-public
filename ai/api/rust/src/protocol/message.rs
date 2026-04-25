//! Protocol Message Types
//!
//! This module defines all message types for the WIA AI Communication Protocol.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

use super::{PROTOCOL_ID, PROTOCOL_VERSION};

/// Main protocol message structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage {
    /// Protocol identifier (always "wia-ai")
    pub protocol: String,

    /// Protocol version
    pub version: String,

    /// Unique message identifier
    pub message_id: String,

    /// Unix timestamp in milliseconds
    pub timestamp: u64,

    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,

    /// Message sender
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<EntityReference>,

    /// Message recipient
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<EntityReference>,

    /// Message payload
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,

    /// Additional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<HashMap<String, serde_json::Value>>,
}

impl Default for ProtocolMessage {
    fn default() -> Self {
        Self {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis() as u64,
            message_type: MessageType::Message,
            source: None,
            target: None,
            payload: None,
            metadata: None,
        }
    }
}

impl ProtocolMessage {
    /// Create a new protocol message with the given type
    pub fn new(message_type: MessageType) -> Self {
        Self {
            message_type,
            ..Default::default()
        }
    }

    /// Create a ping message
    pub fn ping() -> Self {
        Self::new(MessageType::Ping)
    }

    /// Create a pong message responding to a ping
    pub fn pong(ping_id: &str) -> Self {
        let mut msg = Self::new(MessageType::Pong);
        msg.payload = Some(serde_json::json!({
            "ping_id": ping_id
        }));
        msg
    }

    /// Create an error message
    pub fn error(code: ErrorCode, name: &str, message: &str) -> Self {
        let mut msg = Self::new(MessageType::Error);
        msg.payload = Some(serde_json::json!({
            "error_code": code as u32,
            "error_name": name,
            "message": message,
            "retryable": code.is_retryable()
        }));
        msg
    }
}

/// Message types supported by the protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    // Connection messages
    /// Connection request
    Connect,
    /// Connection acknowledgment
    ConnectAck,
    /// Disconnect notification
    Disconnect,
    /// Ping for keepalive
    Ping,
    /// Pong response
    Pong,

    // Content messages
    /// General message
    Message,

    // Streaming messages
    /// Stream started
    StreamStart,
    /// Stream content delta
    StreamDelta,
    /// Stream ended
    StreamEnd,

    // Tool messages
    /// Tool invocation request
    ToolCall,
    /// Tool execution result
    ToolResult,

    // Agent coordination
    /// Agent handoff request
    Handoff,
    /// Handoff acknowledgment
    HandoffAck,
    /// Broadcast message
    Broadcast,
    /// State synchronization
    StateSync,

    // Error
    /// Error message
    Error,
}

/// Entity type in the protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EntityType {
    /// AI Agent
    Agent,
    /// Client application
    Client,
    /// Server
    Server,
    /// Tool
    Tool,
    /// Orchestrator agent
    Orchestrator,
}

/// Reference to an entity in the protocol
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EntityReference {
    /// Entity identifier
    pub id: String,

    /// Entity type
    #[serde(rename = "type")]
    pub entity_type: EntityType,

    /// Entity name (optional)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    /// Entity version (optional)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

impl EntityReference {
    /// Create a new entity reference
    pub fn new(id: impl Into<String>, entity_type: EntityType) -> Self {
        Self {
            id: id.into(),
            entity_type,
            name: None,
            version: None,
        }
    }

    /// Create an agent reference
    pub fn agent(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            entity_type: EntityType::Agent,
            name: Some(name.into()),
            version: None,
        }
    }

    /// Create a client reference
    pub fn client(id: impl Into<String>) -> Self {
        Self::new(id, EntityType::Client)
    }

    /// Create a server reference
    pub fn server(id: impl Into<String>) -> Self {
        Self::new(id, EntityType::Server)
    }

    /// Create a tool reference
    pub fn tool(id: impl Into<String>) -> Self {
        Self::new(id, EntityType::Tool)
    }

    /// Create an orchestrator reference
    pub fn orchestrator(id: impl Into<String>, name: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            entity_type: EntityType::Orchestrator,
            name: Some(name.into()),
            version: None,
        }
    }

    /// Set the name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set the version
    pub fn with_version(mut self, version: impl Into<String>) -> Self {
        self.version = Some(version.into());
        self
    }
}

/// Connect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectPayload {
    /// Supported capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,

    /// Agent ID to connect to
    #[serde(skip_serializing_if = "Option::is_none")]
    pub agent_id: Option<String>,

    /// Authentication information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auth: Option<AuthInfo>,
}

/// Authentication information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthInfo {
    /// Auth type (bearer, api_key)
    #[serde(rename = "type")]
    pub auth_type: String,

    /// Token or key value
    pub token: String,
}

/// Connect acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectAckPayload {
    /// Session identifier
    pub session_id: String,

    /// Supported capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Vec<String>>,

    /// Session timeout in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
}

/// Disconnect payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisconnectPayload {
    /// Disconnect reason
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<DisconnectReason>,

    /// Disconnect code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<u32>,

    /// Optional message
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
}

/// Disconnect reason
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisconnectReason {
    /// Normal disconnection
    Normal,
    /// Error occurred
    Error,
    /// Connection timeout
    Timeout,
    /// Server shutdown
    Shutdown,
}

/// Message role
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    /// User message
    User,
    /// Assistant message
    Assistant,
    /// System message
    System,
    /// Tool result message
    Tool,
}

/// Content block in a message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentBlock {
    /// Content type
    #[serde(rename = "type")]
    pub content_type: ContentType,

    /// Text content (for text type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,

    /// ID (for tool_use type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,

    /// Tool name (for tool_use type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,

    /// Tool input (for tool_use type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input: Option<serde_json::Value>,

    /// Tool use ID (for tool_result type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_use_id: Option<String>,

    /// Content (for tool_result type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,

    /// Image source (for image type)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<ImageSource>,
}

impl ContentBlock {
    /// Create a text content block
    pub fn text(text: impl Into<String>) -> Self {
        Self {
            content_type: ContentType::Text,
            text: Some(text.into()),
            id: None,
            name: None,
            input: None,
            tool_use_id: None,
            content: None,
            source: None,
        }
    }

    /// Create a tool use content block
    pub fn tool_use(id: impl Into<String>, name: impl Into<String>, input: serde_json::Value) -> Self {
        Self {
            content_type: ContentType::ToolUse,
            text: None,
            id: Some(id.into()),
            name: Some(name.into()),
            input: Some(input),
            tool_use_id: None,
            content: None,
            source: None,
        }
    }

    /// Create a tool result content block
    pub fn tool_result(tool_use_id: impl Into<String>, content: impl Into<String>) -> Self {
        Self {
            content_type: ContentType::ToolResult,
            text: None,
            id: None,
            name: None,
            input: None,
            tool_use_id: Some(tool_use_id.into()),
            content: Some(content.into()),
            source: None,
        }
    }
}

/// Content type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentType {
    /// Text content
    Text,
    /// Image content
    Image,
    /// Tool use request
    ToolUse,
    /// Tool execution result
    ToolResult,
}

/// Image source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageSource {
    /// Source type (base64, url)
    #[serde(rename = "type")]
    pub source_type: String,

    /// Media type (image/jpeg, image/png, etc.)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub media_type: Option<String>,

    /// Base64 encoded data or URL
    pub data: String,
}

/// Message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessagePayload {
    /// Message role
    pub role: Role,

    /// Content blocks
    pub content: Vec<ContentBlock>,
}

impl MessagePayload {
    /// Create a user text message
    pub fn user(text: impl Into<String>) -> Self {
        Self {
            role: Role::User,
            content: vec![ContentBlock::text(text)],
        }
    }

    /// Create an assistant text message
    pub fn assistant(text: impl Into<String>) -> Self {
        Self {
            role: Role::Assistant,
            content: vec![ContentBlock::text(text)],
        }
    }

    /// Create a system message
    pub fn system(text: impl Into<String>) -> Self {
        Self {
            role: Role::System,
            content: vec![ContentBlock::text(text)],
        }
    }
}

/// Stream start payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamStartPayload {
    /// Stream identifier
    pub stream_id: String,

    /// Model used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,

    /// Input token count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input_tokens: Option<u32>,
}

/// Delta type for streaming
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeltaType {
    /// Text delta
    TextDelta,
    /// Tool use delta
    ToolUseDelta,
    /// Thinking delta
    ThinkingDelta,
}

/// Stream delta content
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Delta {
    /// Delta type
    #[serde(rename = "type")]
    pub delta_type: DeltaType,

    /// Text content (for text_delta)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,

    /// Partial JSON (for tool_use_delta)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partial_json: Option<String>,
}

impl Delta {
    /// Create a text delta
    pub fn text(text: impl Into<String>) -> Self {
        Self {
            delta_type: DeltaType::TextDelta,
            text: Some(text.into()),
            partial_json: None,
        }
    }

    /// Create a tool use delta
    pub fn tool_use(partial_json: impl Into<String>) -> Self {
        Self {
            delta_type: DeltaType::ToolUseDelta,
            text: None,
            partial_json: Some(partial_json.into()),
        }
    }
}

/// Stream delta payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamDeltaPayload {
    /// Stream identifier
    pub stream_id: String,

    /// Delta index
    #[serde(skip_serializing_if = "Option::is_none")]
    pub index: Option<u32>,

    /// Delta content
    pub delta: Delta,
}

/// Stop reason for stream end
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopReason {
    /// Normal end of turn
    EndTurn,
    /// Maximum tokens reached
    MaxTokens,
    /// Stop sequence matched
    StopSequence,
    /// Tool use requested
    ToolUse,
    /// Error occurred
    Error,
}

/// Usage statistics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Usage {
    /// Input tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input_tokens: Option<u32>,

    /// Output tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output_tokens: Option<u32>,

    /// Total tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_tokens: Option<u32>,

    /// Cache read tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cache_read_tokens: Option<u32>,

    /// Cache creation tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cache_creation_tokens: Option<u32>,
}

/// Stream end payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamEndPayload {
    /// Stream identifier
    pub stream_id: String,

    /// Stop reason
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stop_reason: Option<StopReason>,

    /// Usage statistics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub usage: Option<Usage>,
}

/// Tool call payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolCallPayload {
    /// Unique call identifier
    pub call_id: String,

    /// Tool name
    pub tool_name: String,

    /// Tool arguments
    pub arguments: serde_json::Value,
}

/// Tool result status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolResultStatus {
    /// Successful execution
    Success,
    /// Error during execution
    Error,
    /// Execution timeout
    Timeout,
    /// Execution cancelled
    Cancelled,
}

/// Tool result payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolResultPayload {
    /// Call identifier
    pub call_id: String,

    /// Execution status
    pub status: ToolResultStatus,

    /// Execution result (for success)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,

    /// Error information (for error)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ToolError>,

    /// Execution time in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution_time_ms: Option<u64>,
}

/// Tool error information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolError {
    /// Error code
    pub code: String,

    /// Error message
    pub message: String,
}

/// Handoff priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandoffPriority {
    /// Critical priority
    Critical,
    /// High priority
    High,
    /// Normal priority
    Normal,
    /// Low priority
    Low,
}

impl Default for HandoffPriority {
    fn default() -> Self {
        Self::Normal
    }
}

/// Handoff context
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HandoffContext {
    /// Task description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub task: Option<String>,

    /// Requirements
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requirements: Option<Vec<String>>,

    /// Artifact references
    #[serde(skip_serializing_if = "Option::is_none")]
    pub artifacts: Option<Vec<String>>,

    /// Conversation history
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conversation_history: Option<Vec<MessagePayload>>,

    /// Additional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<HashMap<String, serde_json::Value>>,
}

/// Handoff payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandoffPayload {
    /// Handoff identifier
    pub handoff_id: String,

    /// Handoff reason
    pub reason: String,

    /// Priority
    #[serde(default)]
    pub priority: HandoffPriority,

    /// Context
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<HandoffContext>,

    /// Deadline in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deadline_ms: Option<u64>,
}

/// Handoff acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandoffAckPayload {
    /// Handoff identifier
    pub handoff_id: String,

    /// Whether handoff was accepted
    pub accepted: bool,

    /// Rejection reason (if not accepted)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,

    /// Estimated duration in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub estimated_duration_ms: Option<u64>,
}

/// Error codes for the protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u32)]
pub enum ErrorCode {
    // Connection errors (1xxx)
    /// Normal connection close
    ConnectionClosed = 1000,
    /// Connection lost unexpectedly
    ConnectionLost = 1001,
    /// Protocol error
    ProtocolError = 1002,
    /// Invalid message format
    InvalidMessage = 1003,
    /// Protocol version mismatch
    VersionMismatch = 1004,

    // Agent errors (2xxx)
    /// Agent not found
    AgentNotFound = 2001,
    /// Agent is busy
    AgentBusy = 2002,
    /// Agent internal error
    AgentError = 2003,
    /// Agent response timeout
    AgentTimeout = 2004,
    /// Handoff rejected
    HandoffRejected = 2005,

    // Tool errors (3xxx)
    /// Tool not found
    ToolNotFound = 3001,
    /// Tool execution failed
    ToolExecutionFailed = 3002,
    /// Tool timeout
    ToolTimeout = 3003,
    /// Invalid tool arguments
    ToolInvalidArgs = 3004,
    /// Tool permission denied
    ToolPermissionDenied = 3005,

    // Resource errors (4xxx)
    /// Rate limited
    RateLimited = 4001,
    /// Token limit exceeded
    TokenLimitExceeded = 4002,
    /// Context overflow
    ContextOverflow = 4003,
    /// Budget exceeded
    BudgetExceeded = 4004,
    /// Quota exceeded
    QuotaExceeded = 4005,

    // Security errors (5xxx)
    /// Authentication failed
    AuthFailed = 5001,
    /// Permission denied
    PermissionDenied = 5002,
    /// Token expired
    TokenExpired = 5003,
    /// Invalid token
    InvalidToken = 5004,
    /// Session expired
    SessionExpired = 5005,
}

impl ErrorCode {
    /// Check if this error is retryable
    pub fn is_retryable(&self) -> bool {
        matches!(
            self,
            ErrorCode::ConnectionLost
                | ErrorCode::AgentBusy
                | ErrorCode::AgentTimeout
                | ErrorCode::ToolTimeout
                | ErrorCode::RateLimited
        )
    }

    /// Get error name as string
    pub fn name(&self) -> &'static str {
        match self {
            ErrorCode::ConnectionClosed => "CONNECTION_CLOSED",
            ErrorCode::ConnectionLost => "CONNECTION_LOST",
            ErrorCode::ProtocolError => "PROTOCOL_ERROR",
            ErrorCode::InvalidMessage => "INVALID_MESSAGE",
            ErrorCode::VersionMismatch => "VERSION_MISMATCH",
            ErrorCode::AgentNotFound => "AGENT_NOT_FOUND",
            ErrorCode::AgentBusy => "AGENT_BUSY",
            ErrorCode::AgentError => "AGENT_ERROR",
            ErrorCode::AgentTimeout => "AGENT_TIMEOUT",
            ErrorCode::HandoffRejected => "HANDOFF_REJECTED",
            ErrorCode::ToolNotFound => "TOOL_NOT_FOUND",
            ErrorCode::ToolExecutionFailed => "TOOL_EXECUTION_FAILED",
            ErrorCode::ToolTimeout => "TOOL_TIMEOUT",
            ErrorCode::ToolInvalidArgs => "TOOL_INVALID_ARGS",
            ErrorCode::ToolPermissionDenied => "TOOL_PERMISSION_DENIED",
            ErrorCode::RateLimited => "RATE_LIMITED",
            ErrorCode::TokenLimitExceeded => "TOKEN_LIMIT_EXCEEDED",
            ErrorCode::ContextOverflow => "CONTEXT_OVERFLOW",
            ErrorCode::BudgetExceeded => "BUDGET_EXCEEDED",
            ErrorCode::QuotaExceeded => "QUOTA_EXCEEDED",
            ErrorCode::AuthFailed => "AUTH_FAILED",
            ErrorCode::PermissionDenied => "PERMISSION_DENIED",
            ErrorCode::TokenExpired => "TOKEN_EXPIRED",
            ErrorCode::InvalidToken => "INVALID_TOKEN",
            ErrorCode::SessionExpired => "SESSION_EXPIRED",
        }
    }
}

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPayload {
    /// Error code
    pub error_code: u32,

    /// Error name
    pub error_name: String,

    /// Error message
    pub message: String,

    /// Additional details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<HashMap<String, serde_json::Value>>,

    /// Whether the error is retryable
    #[serde(default)]
    pub retryable: bool,

    /// Retry after milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after_ms: Option<u64>,
}

impl ErrorPayload {
    /// Create a new error payload
    pub fn new(code: ErrorCode, message: impl Into<String>) -> Self {
        Self {
            error_code: code as u32,
            error_name: code.name().to_string(),
            message: message.into(),
            details: None,
            retryable: code.is_retryable(),
            retry_after_ms: None,
        }
    }

    /// Add retry after
    pub fn with_retry_after(mut self, ms: u64) -> Self {
        self.retry_after_ms = Some(ms);
        self
    }

    /// Add details
    pub fn with_details(mut self, details: HashMap<String, serde_json::Value>) -> Self {
        self.details = Some(details);
        self
    }
}
