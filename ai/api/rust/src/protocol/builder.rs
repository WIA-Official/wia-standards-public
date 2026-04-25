//! Protocol Message Builder
//!
//! This module provides a builder pattern for constructing protocol messages.

use chrono::Utc;
use std::collections::HashMap;
use uuid::Uuid;

use super::error::ProtocolError;
use super::message::*;
use super::{PROTOCOL_ID, PROTOCOL_VERSION};

/// Builder for constructing protocol messages
#[derive(Debug, Default)]
pub struct ProtocolMessageBuilder {
    message_type: Option<MessageType>,
    message_id: Option<String>,
    timestamp: Option<u64>,
    source: Option<EntityReference>,
    target: Option<EntityReference>,
    payload: Option<serde_json::Value>,
    metadata: Option<HashMap<String, serde_json::Value>>,
}

impl ProtocolMessageBuilder {
    /// Create a new message builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the message type
    pub fn message_type(mut self, message_type: MessageType) -> Self {
        self.message_type = Some(message_type);
        self
    }

    /// Set a custom message ID (default: UUID v4)
    pub fn message_id(mut self, id: impl Into<String>) -> Self {
        self.message_id = Some(id.into());
        self
    }

    /// Set a custom timestamp (default: current time)
    pub fn timestamp(mut self, timestamp: u64) -> Self {
        self.timestamp = Some(timestamp);
        self
    }

    /// Set the message source
    pub fn source(mut self, source: EntityReference) -> Self {
        self.source = Some(source);
        self
    }

    /// Set the message target
    pub fn target(mut self, target: EntityReference) -> Self {
        self.target = Some(target);
        self
    }

    /// Set the payload as JSON value
    pub fn payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = Some(payload);
        self
    }

    /// Set a typed payload (serializes to JSON)
    pub fn typed_payload<T: serde::Serialize>(mut self, payload: &T) -> Result<Self, ProtocolError> {
        self.payload = Some(serde_json::to_value(payload).map_err(|e| {
            ProtocolError::serialization_error(format!("Failed to serialize payload: {}", e))
        })?);
        Ok(self)
    }

    /// Add metadata
    pub fn metadata(mut self, metadata: HashMap<String, serde_json::Value>) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Add a single metadata entry
    pub fn add_metadata(mut self, key: impl Into<String>, value: serde_json::Value) -> Self {
        let metadata = self.metadata.get_or_insert_with(HashMap::new);
        metadata.insert(key.into(), value);
        self
    }

    /// Build the protocol message
    pub fn build(self) -> Result<ProtocolMessage, ProtocolError> {
        let message_type = self.message_type.ok_or_else(|| {
            ProtocolError::validation_error("Message type is required")
        })?;

        Ok(ProtocolMessage {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: self.message_id.unwrap_or_else(|| Uuid::new_v4().to_string()),
            timestamp: self.timestamp.unwrap_or_else(|| Utc::now().timestamp_millis() as u64),
            message_type,
            source: self.source,
            target: self.target,
            payload: self.payload,
            metadata: self.metadata,
        })
    }
}

/// Builder for connect messages
pub struct ConnectMessageBuilder {
    source: Option<EntityReference>,
    agent_id: Option<String>,
    capabilities: Vec<String>,
    auth: Option<AuthInfo>,
}

impl ConnectMessageBuilder {
    /// Create a new connect message builder
    pub fn new() -> Self {
        Self {
            source: None,
            agent_id: None,
            capabilities: vec![],
            auth: None,
        }
    }

    /// Set the source client
    pub fn source(mut self, source: EntityReference) -> Self {
        self.source = Some(source);
        self
    }

    /// Set the target agent ID
    pub fn agent_id(mut self, id: impl Into<String>) -> Self {
        self.agent_id = Some(id.into());
        self
    }

    /// Add a capability
    pub fn capability(mut self, cap: impl Into<String>) -> Self {
        self.capabilities.push(cap.into());
        self
    }

    /// Set capabilities
    pub fn capabilities(mut self, caps: Vec<String>) -> Self {
        self.capabilities = caps;
        self
    }

    /// Set bearer token authentication
    pub fn bearer_token(mut self, token: impl Into<String>) -> Self {
        self.auth = Some(AuthInfo {
            auth_type: "bearer".to_string(),
            token: token.into(),
        });
        self
    }

    /// Set API key authentication
    pub fn api_key(mut self, key: impl Into<String>) -> Self {
        self.auth = Some(AuthInfo {
            auth_type: "api_key".to_string(),
            token: key.into(),
        });
        self
    }

    /// Build the connect message
    pub fn build(self) -> Result<ProtocolMessage, ProtocolError> {
        let payload = ConnectPayload {
            capabilities: if self.capabilities.is_empty() {
                None
            } else {
                Some(self.capabilities)
            },
            agent_id: self.agent_id,
            auth: self.auth,
        };

        ProtocolMessageBuilder::new()
            .message_type(MessageType::Connect)
            .source(self.source.unwrap_or_else(|| EntityReference::client("client")))
            .typed_payload(&payload)?
            .build()
    }
}

impl Default for ConnectMessageBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for stream messages
pub struct StreamMessageBuilder {
    stream_id: String,
    source: Option<EntityReference>,
}

impl StreamMessageBuilder {
    /// Create a new stream message builder
    pub fn new(stream_id: impl Into<String>) -> Self {
        Self {
            stream_id: stream_id.into(),
            source: None,
        }
    }

    /// Set the source agent
    pub fn source(mut self, source: EntityReference) -> Self {
        self.source = Some(source);
        self
    }

    /// Build a stream start message
    pub fn start(self, model: Option<String>, input_tokens: Option<u32>) -> Result<ProtocolMessage, ProtocolError> {
        let payload = StreamStartPayload {
            stream_id: self.stream_id,
            model,
            input_tokens,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::StreamStart)
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }

        builder.build()
    }

    /// Build a stream delta message
    pub fn delta(self, delta: Delta, index: Option<u32>) -> Result<ProtocolMessage, ProtocolError> {
        let payload = StreamDeltaPayload {
            stream_id: self.stream_id,
            index,
            delta,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::StreamDelta)
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }

        builder.build()
    }

    /// Build a stream end message
    pub fn end(self, stop_reason: Option<StopReason>, usage: Option<Usage>) -> Result<ProtocolMessage, ProtocolError> {
        let payload = StreamEndPayload {
            stream_id: self.stream_id,
            stop_reason,
            usage,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::StreamEnd)
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }

        builder.build()
    }
}

/// Builder for tool call messages
pub struct ToolCallBuilder {
    call_id: String,
    tool_name: String,
    arguments: serde_json::Value,
    source: Option<EntityReference>,
}

impl ToolCallBuilder {
    /// Create a new tool call builder
    pub fn new(tool_name: impl Into<String>) -> Self {
        Self {
            call_id: Uuid::new_v4().to_string(),
            tool_name: tool_name.into(),
            arguments: serde_json::Value::Object(serde_json::Map::new()),
            source: None,
        }
    }

    /// Set the call ID
    pub fn call_id(mut self, id: impl Into<String>) -> Self {
        self.call_id = id.into();
        self
    }

    /// Set the arguments
    pub fn arguments(mut self, args: serde_json::Value) -> Self {
        self.arguments = args;
        self
    }

    /// Set the source agent
    pub fn source(mut self, source: EntityReference) -> Self {
        self.source = Some(source);
        self
    }

    /// Build the tool call message
    pub fn build(self) -> Result<ProtocolMessage, ProtocolError> {
        let payload = ToolCallPayload {
            call_id: self.call_id,
            tool_name: self.tool_name.clone(),
            arguments: self.arguments,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::ToolCall)
            .target(EntityReference::tool(&self.tool_name))
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }

        builder.build()
    }
}

/// Builder for tool result messages
pub struct ToolResultBuilder {
    call_id: String,
    source: Option<EntityReference>,
    target: Option<EntityReference>,
}

impl ToolResultBuilder {
    /// Create a new tool result builder
    pub fn new(call_id: impl Into<String>) -> Self {
        Self {
            call_id: call_id.into(),
            source: None,
            target: None,
        }
    }

    /// Set the source tool
    pub fn source(mut self, source: EntityReference) -> Self {
        self.source = Some(source);
        self
    }

    /// Set the target agent
    pub fn target(mut self, target: EntityReference) -> Self {
        self.target = Some(target);
        self
    }

    /// Build a success result
    pub fn success(self, result: serde_json::Value, execution_time_ms: Option<u64>) -> Result<ProtocolMessage, ProtocolError> {
        let payload = ToolResultPayload {
            call_id: self.call_id,
            status: ToolResultStatus::Success,
            result: Some(result),
            error: None,
            execution_time_ms,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::ToolResult)
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }
        if let Some(target) = self.target {
            builder = builder.target(target);
        }

        builder.build()
    }

    /// Build an error result
    pub fn error(self, code: impl Into<String>, message: impl Into<String>) -> Result<ProtocolMessage, ProtocolError> {
        let payload = ToolResultPayload {
            call_id: self.call_id,
            status: ToolResultStatus::Error,
            result: None,
            error: Some(ToolError {
                code: code.into(),
                message: message.into(),
            }),
            execution_time_ms: None,
        };

        let mut builder = ProtocolMessageBuilder::new()
            .message_type(MessageType::ToolResult)
            .typed_payload(&payload)?;

        if let Some(source) = self.source {
            builder = builder.source(source);
        }
        if let Some(target) = self.target {
            builder = builder.target(target);
        }

        builder.build()
    }
}

/// Builder for handoff messages
pub struct HandoffBuilder {
    source: EntityReference,
    target: EntityReference,
    reason: String,
    priority: HandoffPriority,
    context: Option<HandoffContext>,
    deadline_ms: Option<u64>,
}

impl HandoffBuilder {
    /// Create a new handoff builder
    pub fn new(
        source: EntityReference,
        target: EntityReference,
        reason: impl Into<String>,
    ) -> Self {
        Self {
            source,
            target,
            reason: reason.into(),
            priority: HandoffPriority::Normal,
            context: None,
            deadline_ms: None,
        }
    }

    /// Set the priority
    pub fn priority(mut self, priority: HandoffPriority) -> Self {
        self.priority = priority;
        self
    }

    /// Set the context
    pub fn context(mut self, context: HandoffContext) -> Self {
        self.context = Some(context);
        self
    }

    /// Set the task
    pub fn task(mut self, task: impl Into<String>) -> Self {
        let context = self.context.get_or_insert_with(HandoffContext::default);
        context.task = Some(task.into());
        self
    }

    /// Add a requirement
    pub fn requirement(mut self, requirement: impl Into<String>) -> Self {
        let context = self.context.get_or_insert_with(HandoffContext::default);
        let requirements = context.requirements.get_or_insert_with(Vec::new);
        requirements.push(requirement.into());
        self
    }

    /// Set the deadline
    pub fn deadline_ms(mut self, ms: u64) -> Self {
        self.deadline_ms = Some(ms);
        self
    }

    /// Build the handoff message
    pub fn build(self) -> Result<ProtocolMessage, ProtocolError> {
        let payload = HandoffPayload {
            handoff_id: Uuid::new_v4().to_string(),
            reason: self.reason,
            priority: self.priority,
            context: self.context,
            deadline_ms: self.deadline_ms,
        };

        ProtocolMessageBuilder::new()
            .message_type(MessageType::Handoff)
            .source(self.source)
            .target(self.target)
            .typed_payload(&payload)?
            .build()
    }
}
