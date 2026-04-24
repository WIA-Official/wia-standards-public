//! Protocol handler for processing WSP messages

use std::collections::HashMap;
use std::sync::Arc;

use async_trait::async_trait;
use tokio::sync::RwLock;

use super::error::ProtocolError;
use super::message::*;

/// Handler result type
pub type HandlerResult<T> = Result<T, ProtocolError>;

/// Message handler trait
#[async_trait]
pub trait MessageHandler: Send + Sync {
    /// Handle incoming message
    async fn handle(&self, message: WspMessage) -> HandlerResult<Option<WspMessage>>;

    /// Get handler name
    fn name(&self) -> &str;

    /// Get supported message types
    fn supported_types(&self) -> Vec<MessageType>;
}

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Error,
}

impl ConnectionState {
    pub fn is_connected(&self) -> bool {
        matches!(self, ConnectionState::Connected)
    }

    pub fn can_send(&self) -> bool {
        matches!(self, ConnectionState::Connected | ConnectionState::Reconnecting)
    }
}

/// Session information
#[derive(Debug, Clone)]
pub struct Session {
    pub id: String,
    pub client_endpoint: Endpoint,
    pub server_endpoint: Endpoint,
    pub state: ConnectionState,
    pub connected_at: Option<i64>,
    pub last_activity: i64,
    pub capabilities: Vec<String>,
}

impl Session {
    pub fn new(client: Endpoint, server: Endpoint) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            client_endpoint: client,
            server_endpoint: server,
            state: ConnectionState::Disconnected,
            connected_at: None,
            last_activity: chrono::Utc::now().timestamp_millis(),
            capabilities: Vec::new(),
        }
    }

    pub fn set_connected(&mut self) {
        self.state = ConnectionState::Connected;
        self.connected_at = Some(chrono::Utc::now().timestamp_millis());
    }

    pub fn set_disconnected(&mut self) {
        self.state = ConnectionState::Disconnected;
    }

    pub fn update_activity(&mut self) {
        self.last_activity = chrono::Utc::now().timestamp_millis();
    }

    pub fn idle_time_ms(&self) -> i64 {
        chrono::Utc::now().timestamp_millis() - self.last_activity
    }
}

/// Protocol handler for WSP
pub struct ProtocolHandler {
    handlers: HashMap<MessageType, Arc<dyn MessageHandler>>,
    sessions: Arc<RwLock<HashMap<String, Session>>>,
    pending_responses: Arc<RwLock<HashMap<String, tokio::sync::oneshot::Sender<WspMessage>>>>,
}

impl ProtocolHandler {
    pub fn new() -> Self {
        Self {
            handlers: HashMap::new(),
            sessions: Arc::new(RwLock::new(HashMap::new())),
            pending_responses: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register a message handler
    pub fn register_handler(&mut self, handler: Arc<dyn MessageHandler>) {
        for msg_type in handler.supported_types() {
            self.handlers.insert(msg_type, handler.clone());
        }
    }

    /// Process incoming message
    pub async fn process(&self, message: WspMessage) -> HandlerResult<Option<WspMessage>> {
        // Check for pending response
        {
            let mut pending = self.pending_responses.write().await;
            if let Some(sender) = pending.remove(&message.message_id) {
                let _ = sender.send(message);
                return Ok(None);
            }
        }

        // Find handler for message type
        if let Some(handler) = self.handlers.get(&message.message_type) {
            handler.handle(message).await
        } else {
            // Default handling for unregistered types
            self.default_handle(message).await
        }
    }

    /// Default message handling
    async fn default_handle(&self, message: WspMessage) -> HandlerResult<Option<WspMessage>> {
        match message.message_type {
            MessageType::Ping => {
                // Auto-respond to ping
                let pong = WspMessage::new(
                    MessageType::Pong,
                    message.destination.clone(),
                    message.source.clone(),
                    serde_json::json!({
                        "pingId": message.message_id,
                        "receivedAt": chrono::Utc::now().timestamp_millis(),
                        "processedAt": chrono::Utc::now().timestamp_millis()
                    }),
                );
                Ok(Some(pong))
            }
            MessageType::Pong => {
                // Pong is already handled by pending response check
                Ok(None)
            }
            MessageType::Error | MessageType::Warning => {
                // Log errors/warnings, don't respond
                Ok(None)
            }
            _ => {
                // Unknown message type
                Err(ProtocolError::UnsupportedMessageType(format!(
                    "{:?}",
                    message.message_type
                )))
            }
        }
    }

    /// Create a new session
    pub async fn create_session(&self, client: Endpoint, server: Endpoint) -> String {
        let session = Session::new(client, server);
        let id = session.id.clone();
        self.sessions.write().await.insert(id.clone(), session);
        id
    }

    /// Get session by ID
    pub async fn get_session(&self, session_id: &str) -> Option<Session> {
        self.sessions.read().await.get(session_id).cloned()
    }

    /// Update session state
    pub async fn update_session_state(&self, session_id: &str, state: ConnectionState) {
        if let Some(session) = self.sessions.write().await.get_mut(session_id) {
            session.state = state;
            if state == ConnectionState::Connected {
                session.set_connected();
            }
        }
    }

    /// Remove session
    pub async fn remove_session(&self, session_id: &str) {
        self.sessions.write().await.remove(session_id);
    }

    /// Wait for response to a message
    pub async fn wait_for_response(
        &self,
        message_id: String,
        timeout_ms: u64,
    ) -> HandlerResult<WspMessage> {
        let (tx, rx) = tokio::sync::oneshot::channel();

        self.pending_responses
            .write()
            .await
            .insert(message_id.clone(), tx);

        match tokio::time::timeout(std::time::Duration::from_millis(timeout_ms), rx).await {
            Ok(Ok(response)) => Ok(response),
            Ok(Err(_)) => {
                self.pending_responses.write().await.remove(&message_id);
                Err(ProtocolError::ChannelClosed)
            }
            Err(_) => {
                self.pending_responses.write().await.remove(&message_id);
                Err(ProtocolError::Timeout(timeout_ms))
            }
        }
    }
}

impl Default for ProtocolHandler {
    fn default() -> Self {
        Self::new()
    }
}

/// Ping handler implementation
pub struct PingHandler;

#[async_trait]
impl MessageHandler for PingHandler {
    async fn handle(&self, message: WspMessage) -> HandlerResult<Option<WspMessage>> {
        let now = chrono::Utc::now().timestamp_millis();
        let pong = WspMessage::new(
            MessageType::Pong,
            message.destination.clone(),
            message.source.clone(),
            serde_json::json!({
                "pingId": message.message_id,
                "receivedAt": now,
                "processedAt": now
            }),
        );
        Ok(Some(pong))
    }

    fn name(&self) -> &str {
        "PingHandler"
    }

    fn supported_types(&self) -> Vec<MessageType> {
        vec![MessageType::Ping]
    }
}

/// Connection handler implementation
pub struct ConnectionHandler;

#[async_trait]
impl MessageHandler for ConnectionHandler {
    async fn handle(&self, message: WspMessage) -> HandlerResult<Option<WspMessage>> {
        match message.message_type {
            MessageType::Connect => {
                let ack = WspMessage::new(
                    MessageType::ConnectAck,
                    message.destination.clone(),
                    message.source.clone(),
                    serde_json::json!({
                        "sessionId": uuid::Uuid::new_v4().to_string(),
                        "accepted": true,
                        "serverName": "WIA Space Protocol Server",
                        "heartbeatInterval_s": 30
                    }),
                );
                Ok(Some(ack))
            }
            MessageType::Disconnect => Ok(None),
            _ => Ok(None),
        }
    }

    fn name(&self) -> &str {
        "ConnectionHandler"
    }

    fn supported_types(&self) -> Vec<MessageType> {
        vec![MessageType::Connect, MessageType::Disconnect]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connection_state() {
        assert!(ConnectionState::Connected.is_connected());
        assert!(!ConnectionState::Disconnected.is_connected());
        assert!(ConnectionState::Connected.can_send());
        assert!(ConnectionState::Reconnecting.can_send());
        assert!(!ConnectionState::Disconnected.can_send());
    }

    #[test]
    fn test_session_creation() {
        let client = Endpoint::ground_station("gs-01");
        let server = Endpoint::ground_station("mission-control");
        let session = Session::new(client, server);

        assert_eq!(session.state, ConnectionState::Disconnected);
        assert!(session.connected_at.is_none());
    }

    #[tokio::test]
    async fn test_protocol_handler_creation() {
        let handler = ProtocolHandler::new();
        let session_id = handler
            .create_session(
                Endpoint::ground_station("gs-01"),
                Endpoint::ground_station("mission-control"),
            )
            .await;

        let session = handler.get_session(&session_id).await;
        assert!(session.is_some());
    }

    #[tokio::test]
    async fn test_ping_handling() {
        let handler = ProtocolHandler::new();

        let ping = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::ground_station("mission-control"),
            serde_json::json!({ "sentAt": 1234567890 }),
        );

        let result = handler.process(ping).await;
        assert!(result.is_ok());

        let response = result.unwrap();
        assert!(response.is_some());

        let pong = response.unwrap();
        assert_eq!(pong.message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_register_handler() {
        let mut handler = ProtocolHandler::new();
        handler.register_handler(Arc::new(PingHandler));

        let ping = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::ground_station("mission-control"),
            serde_json::json!({ "sentAt": 1234567890 }),
        );

        let result = handler.process(ping).await;
        assert!(result.is_ok());
    }
}
