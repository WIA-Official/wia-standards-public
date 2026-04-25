//! Protocol Handler
//!
//! This module provides handlers for processing protocol messages.

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use super::error::{ProtocolError, ProtocolResult};
use super::message::*;

/// Trait for handling protocol messages
#[async_trait]
pub trait MessageHandler: Send + Sync {
    /// Handle an incoming message
    async fn handle(&self, message: ProtocolMessage) -> ProtocolResult<Option<ProtocolMessage>>;

    /// Get the message types this handler can process
    fn supported_types(&self) -> Vec<MessageType>;
}

/// Protocol router for dispatching messages to handlers
pub struct ProtocolRouter {
    handlers: HashMap<MessageType, Vec<Arc<dyn MessageHandler>>>,
    default_handler: Option<Arc<dyn MessageHandler>>,
}

impl ProtocolRouter {
    /// Create a new protocol router
    pub fn new() -> Self {
        Self {
            handlers: HashMap::new(),
            default_handler: None,
        }
    }

    /// Register a handler for specific message types
    pub fn register(&mut self, handler: Arc<dyn MessageHandler>) {
        for msg_type in handler.supported_types() {
            self.handlers
                .entry(msg_type)
                .or_insert_with(Vec::new)
                .push(Arc::clone(&handler));
        }
    }

    /// Set a default handler for unhandled message types
    pub fn set_default_handler(&mut self, handler: Arc<dyn MessageHandler>) {
        self.default_handler = Some(handler);
    }

    /// Route a message to appropriate handlers
    pub async fn route(&self, message: ProtocolMessage) -> ProtocolResult<Vec<ProtocolMessage>> {
        let mut responses = Vec::new();

        if let Some(handlers) = self.handlers.get(&message.message_type) {
            for handler in handlers {
                if let Some(response) = handler.handle(message.clone()).await? {
                    responses.push(response);
                }
            }
        } else if let Some(ref default) = self.default_handler {
            if let Some(response) = default.handle(message).await? {
                responses.push(response);
            }
        }

        Ok(responses)
    }
}

impl Default for ProtocolRouter {
    fn default() -> Self {
        Self::new()
    }
}

/// Handler for ping/pong messages
pub struct PingPongHandler;

#[async_trait]
impl MessageHandler for PingPongHandler {
    async fn handle(&self, message: ProtocolMessage) -> ProtocolResult<Option<ProtocolMessage>> {
        match message.message_type {
            MessageType::Ping => {
                Ok(Some(ProtocolMessage::pong(&message.message_id)))
            }
            _ => Ok(None),
        }
    }

    fn supported_types(&self) -> Vec<MessageType> {
        vec![MessageType::Ping]
    }
}

/// Connection state
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    /// Not connected
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Connected and ready
    Connected,
    /// Disconnecting
    Disconnecting,
}

/// Protocol session for managing connection state
pub struct ProtocolSession {
    /// Session ID
    pub session_id: String,

    /// Connection state
    state: Arc<RwLock<ConnectionState>>,

    /// Session metadata
    metadata: Arc<RwLock<HashMap<String, serde_json::Value>>>,

    /// Capabilities
    capabilities: Vec<String>,
}

impl ProtocolSession {
    /// Create a new protocol session
    pub fn new(session_id: impl Into<String>) -> Self {
        Self {
            session_id: session_id.into(),
            state: Arc::new(RwLock::new(ConnectionState::Disconnected)),
            metadata: Arc::new(RwLock::new(HashMap::new())),
            capabilities: vec![],
        }
    }

    /// Get the current connection state
    pub async fn state(&self) -> ConnectionState {
        self.state.read().await.clone()
    }

    /// Set the connection state
    pub async fn set_state(&self, state: ConnectionState) {
        *self.state.write().await = state;
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        *self.state.read().await == ConnectionState::Connected
    }

    /// Set a metadata value
    pub async fn set_metadata(&self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.write().await.insert(key.into(), value);
    }

    /// Get a metadata value
    pub async fn get_metadata(&self, key: &str) -> Option<serde_json::Value> {
        self.metadata.read().await.get(key).cloned()
    }

    /// Get capabilities
    pub fn capabilities(&self) -> &[String] {
        &self.capabilities
    }

    /// Set capabilities
    pub fn set_capabilities(&mut self, capabilities: Vec<String>) {
        self.capabilities = capabilities;
    }

    /// Check if a capability is supported
    pub fn has_capability(&self, cap: &str) -> bool {
        self.capabilities.iter().any(|c| c == cap)
    }
}

/// Stream state for tracking streaming operations
#[derive(Debug, Clone)]
pub struct StreamState {
    /// Stream ID
    pub stream_id: String,

    /// Whether the stream is active
    pub active: bool,

    /// Current delta index
    pub delta_index: u32,

    /// Accumulated text
    pub accumulated_text: String,

    /// Model used
    pub model: Option<String>,

    /// Usage statistics
    pub usage: Usage,
}

impl StreamState {
    /// Create a new stream state
    pub fn new(stream_id: impl Into<String>) -> Self {
        Self {
            stream_id: stream_id.into(),
            active: true,
            delta_index: 0,
            accumulated_text: String::new(),
            model: None,
            usage: Usage::default(),
        }
    }

    /// Apply a delta to the stream
    pub fn apply_delta(&mut self, delta: &Delta) {
        self.delta_index += 1;

        if let Some(ref text) = delta.text {
            self.accumulated_text.push_str(text);
        }
    }

    /// End the stream
    pub fn end(&mut self, usage: Option<Usage>) {
        self.active = false;
        if let Some(u) = usage {
            self.usage = u;
        }
    }
}

/// Stream manager for tracking multiple streams
pub struct StreamManager {
    streams: Arc<RwLock<HashMap<String, StreamState>>>,
}

impl StreamManager {
    /// Create a new stream manager
    pub fn new() -> Self {
        Self {
            streams: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Start a new stream
    pub async fn start_stream(&self, stream_id: impl Into<String>, model: Option<String>) -> String {
        let id = stream_id.into();
        let mut state = StreamState::new(&id);
        state.model = model;

        self.streams.write().await.insert(id.clone(), state);
        id
    }

    /// Apply a delta to a stream
    pub async fn apply_delta(&self, stream_id: &str, delta: &Delta) -> ProtocolResult<()> {
        let mut streams = self.streams.write().await;
        let state = streams
            .get_mut(stream_id)
            .ok_or_else(|| ProtocolError::validation_error("Stream not found"))?;

        if !state.active {
            return Err(ProtocolError::validation_error("Stream is not active"));
        }

        state.apply_delta(delta);
        Ok(())
    }

    /// End a stream
    pub async fn end_stream(&self, stream_id: &str, usage: Option<Usage>) -> ProtocolResult<StreamState> {
        let mut streams = self.streams.write().await;
        let state = streams
            .get_mut(stream_id)
            .ok_or_else(|| ProtocolError::validation_error("Stream not found"))?;

        state.end(usage);
        Ok(state.clone())
    }

    /// Get a stream state
    pub async fn get_stream(&self, stream_id: &str) -> Option<StreamState> {
        self.streams.read().await.get(stream_id).cloned()
    }

    /// Remove a stream
    pub async fn remove_stream(&self, stream_id: &str) -> Option<StreamState> {
        self.streams.write().await.remove(stream_id)
    }
}

impl Default for StreamManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Tool call tracker for managing tool invocations
pub struct ToolCallTracker {
    pending_calls: Arc<RwLock<HashMap<String, ToolCallPayload>>>,
}

impl ToolCallTracker {
    /// Create a new tool call tracker
    pub fn new() -> Self {
        Self {
            pending_calls: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Track a new tool call
    pub async fn track_call(&self, payload: ToolCallPayload) {
        self.pending_calls
            .write()
            .await
            .insert(payload.call_id.clone(), payload);
    }

    /// Complete a tool call
    pub async fn complete_call(&self, call_id: &str) -> Option<ToolCallPayload> {
        self.pending_calls.write().await.remove(call_id)
    }

    /// Check if a call is pending
    pub async fn is_pending(&self, call_id: &str) -> bool {
        self.pending_calls.read().await.contains_key(call_id)
    }

    /// Get all pending calls
    pub async fn pending_calls(&self) -> Vec<ToolCallPayload> {
        self.pending_calls.read().await.values().cloned().collect()
    }
}

impl Default for ToolCallTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_ping_pong_handler() {
        let handler = PingPongHandler;
        let ping = ProtocolMessage::ping();

        let response = handler.handle(ping.clone()).await.unwrap();
        assert!(response.is_some());

        let pong = response.unwrap();
        assert_eq!(pong.message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_protocol_router() {
        let mut router = ProtocolRouter::new();
        router.register(Arc::new(PingPongHandler));

        let ping = ProtocolMessage::ping();
        let responses = router.route(ping).await.unwrap();

        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_protocol_session() {
        let session = ProtocolSession::new("session-001");

        assert_eq!(session.state().await, ConnectionState::Disconnected);

        session.set_state(ConnectionState::Connected).await;
        assert!(session.is_connected().await);
    }

    #[tokio::test]
    async fn test_stream_manager() {
        let manager = StreamManager::new();

        let stream_id = manager.start_stream("stream-001", Some("claude".to_string())).await;

        manager
            .apply_delta(&stream_id, &Delta::text("Hello"))
            .await
            .unwrap();
        manager
            .apply_delta(&stream_id, &Delta::text(" World"))
            .await
            .unwrap();

        let state = manager.get_stream(&stream_id).await.unwrap();
        assert_eq!(state.accumulated_text, "Hello World");
        assert_eq!(state.delta_index, 2);

        let final_state = manager.end_stream(&stream_id, None).await.unwrap();
        assert!(!final_state.active);
    }
}
