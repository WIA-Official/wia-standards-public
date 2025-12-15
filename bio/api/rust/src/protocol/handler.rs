//! Protocol handler for WIA Bio Protocol

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::RwLock;

use super::message::*;
use super::builder::*;

/// Handler callback type
pub type MessageCallback = Box<dyn Fn(&BioMessage) + Send + Sync>;

/// Protocol handler for managing message flow
pub struct ProtocolHandler {
    /// Sequence number counter
    sequence_counter: AtomicU64,
    /// Registered message handlers
    handlers: RwLock<HashMap<MessageType, Vec<Arc<MessageCallback>>>>,
    /// Pending command acknowledgments
    pending_commands: RwLock<HashMap<String, PendingCommand>>,
    /// Connection state
    state: RwLock<ConnectionState>,
    /// Handler configuration
    config: HandlerConfig,
}

/// Pending command tracking
struct PendingCommand {
    command: BioCommand,
    timestamp: i64,
    correlation_id: String,
}

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Not connected
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Connected and ready
    Connected,
    /// Reconnecting after disconnect
    Reconnecting,
    /// Error state
    Error,
}

/// Handler configuration
#[derive(Debug, Clone)]
pub struct HandlerConfig {
    /// Command timeout in milliseconds
    pub command_timeout: u64,
    /// Maximum pending commands
    pub max_pending_commands: usize,
    /// Auto-respond to ping
    pub auto_pong: bool,
}

impl Default for HandlerConfig {
    fn default() -> Self {
        Self {
            command_timeout: 30000,
            max_pending_commands: 100,
            auto_pong: true,
        }
    }
}

impl ProtocolHandler {
    /// Create a new protocol handler
    pub fn new() -> Self {
        Self::with_config(HandlerConfig::default())
    }

    /// Create a handler with custom configuration
    pub fn with_config(config: HandlerConfig) -> Self {
        Self {
            sequence_counter: AtomicU64::new(0),
            handlers: RwLock::new(HashMap::new()),
            pending_commands: RwLock::new(HashMap::new()),
            state: RwLock::new(ConnectionState::Disconnected),
            config,
        }
    }

    /// Get the next sequence number
    pub fn next_sequence(&self) -> u64 {
        self.sequence_counter.fetch_add(1, Ordering::SeqCst)
    }

    /// Get current connection state
    pub async fn state(&self) -> ConnectionState {
        *self.state.read().await
    }

    /// Set connection state
    pub async fn set_state(&self, state: ConnectionState) {
        *self.state.write().await = state;
    }

    /// Register a handler for a message type
    pub async fn on_message<F>(&self, message_type: MessageType, handler: F)
    where
        F: Fn(&BioMessage) + Send + Sync + 'static,
    {
        let mut handlers = self.handlers.write().await;
        handlers
            .entry(message_type)
            .or_insert_with(Vec::new)
            .push(Arc::new(Box::new(handler)));
    }

    /// Handle an incoming message
    pub async fn handle(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        // Validate message
        message.validate()?;

        // Check state
        let state = self.state().await;
        if state == ConnectionState::Disconnected && message.message_type != MessageType::Connect {
            return Err(ProtocolError::InvalidMessageType(
                "Not connected".to_string(),
            ));
        }

        // Process by type
        let response = match message.message_type {
            MessageType::Connect => self.handle_connect(message).await?,
            MessageType::ConnectAck => self.handle_connect_ack(message).await?,
            MessageType::Disconnect => self.handle_disconnect(message).await?,
            MessageType::Ping => self.handle_ping(message).await?,
            MessageType::Pong => self.handle_pong(message).await?,
            MessageType::Command => self.handle_command(message).await?,
            MessageType::CommandAck => self.handle_command_ack(message).await?,
            MessageType::Error => self.handle_error(message).await?,
            _ => None,
        };

        // Call registered handlers
        let handlers = self.handlers.read().await;
        if let Some(type_handlers) = handlers.get(&message.message_type) {
            for handler in type_handlers {
                handler(message);
            }
        }

        Ok(response)
    }

    async fn handle_connect(&self, _message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        self.set_state(ConnectionState::Connecting).await;
        // Server-side: generate connect_ack
        // Client-side: this shouldn't be received
        Ok(None)
    }

    async fn handle_connect_ack(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        if let MessagePayload::ConnectAck(payload) = &message.payload {
            match payload.status {
                ConnectionStatus::Connected => {
                    self.set_state(ConnectionState::Connected).await;
                }
                ConnectionStatus::AuthRequired => {
                    // Handle auth requirement
                }
                ConnectionStatus::Rejected => {
                    self.set_state(ConnectionState::Error).await;
                }
            }
        }
        Ok(None)
    }

    async fn handle_disconnect(&self, _message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        self.set_state(ConnectionState::Disconnected).await;
        Ok(None)
    }

    async fn handle_ping(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        if !self.config.auto_pong {
            return Ok(None);
        }

        if let MessagePayload::Heartbeat(payload) = &message.payload {
            let pong = MessageBuilder::pong(payload.client_timestamp)
                .with_sequence(self.next_sequence())
                .build();
            Ok(Some(pong))
        } else {
            Ok(None)
        }
    }

    async fn handle_pong(&self, _message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        // Update latency metrics, etc.
        Ok(None)
    }

    async fn handle_command(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        if let MessagePayload::Command(payload) = &message.payload {
            // Track pending command
            if let Some(correlation_id) = &payload.correlation_id {
                let mut pending = self.pending_commands.write().await;
                if pending.len() >= self.config.max_pending_commands {
                    return Err(ProtocolError::InvalidPayload(
                        "Too many pending commands".to_string(),
                    ));
                }
                pending.insert(
                    correlation_id.clone(),
                    PendingCommand {
                        command: payload.command,
                        timestamp: message.timestamp,
                        correlation_id: correlation_id.clone(),
                    },
                );
            }
        }
        Ok(None)
    }

    async fn handle_command_ack(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        if let MessagePayload::CommandAck(payload) = &message.payload {
            if let Some(correlation_id) = &payload.correlation_id {
                let mut pending = self.pending_commands.write().await;
                pending.remove(correlation_id);
            }
        }
        Ok(None)
    }

    async fn handle_error(&self, message: &BioMessage) -> Result<Option<BioMessage>, ProtocolError> {
        if let MessagePayload::Error(payload) = &message.payload {
            if !payload.recoverable {
                self.set_state(ConnectionState::Error).await;
            }
        }
        Ok(None)
    }

    /// Create a connect message
    pub fn create_connect(
        &self,
        client_id: impl Into<String>,
        client_name: impl Into<String>,
        capabilities: Vec<ProtocolCapability>,
    ) -> BioMessage {
        MessageBuilder::connect(client_id, client_name)
            .with_capabilities(capabilities)
            .with_sequence(self.next_sequence())
            .build()
    }

    /// Create a command message with auto-generated correlation ID
    pub fn create_command(&self, command: BioCommand, params: serde_json::Value) -> BioMessage {
        let correlation_id = uuid::Uuid::new_v4().to_string();
        MessageBuilder::command(command)
            .with_params(params)
            .with_correlation_id(correlation_id)
            .with_sequence(self.next_sequence())
            .build()
    }

    /// Create a ping message
    pub fn create_ping(&self) -> BioMessage {
        MessageBuilder::ping()
            .with_sequence(self.next_sequence())
            .build()
    }

    /// Create a disconnect message
    pub fn create_disconnect(&self, reason_code: u32, reason: Option<String>) -> BioMessage {
        let mut builder = MessageBuilder::disconnect(reason_code);
        if let Some(r) = reason {
            builder = builder.with_reason(r);
        }
        builder.with_sequence(self.next_sequence()).build()
    }

    /// Clean up timed-out pending commands
    pub async fn cleanup_pending_commands(&self, current_time: i64) {
        let mut pending = self.pending_commands.write().await;
        pending.retain(|_, cmd| {
            current_time - cmd.timestamp < self.config.command_timeout as i64
        });
    }
}

impl Default for ProtocolHandler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_handler_creation() {
        let handler = ProtocolHandler::new();
        assert_eq!(handler.state().await, ConnectionState::Disconnected);
        assert_eq!(handler.next_sequence(), 0);
        assert_eq!(handler.next_sequence(), 1);
    }

    #[tokio::test]
    async fn test_ping_pong_auto() {
        let handler = ProtocolHandler::new();
        handler.set_state(ConnectionState::Connected).await;

        let ping = MessageBuilder::ping().build();
        let response = handler.handle(&ping).await.unwrap();

        assert!(response.is_some());
        let pong = response.unwrap();
        assert_eq!(pong.message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_state_transitions() {
        let handler = ProtocolHandler::new();

        // Initial state
        assert_eq!(handler.state().await, ConnectionState::Disconnected);

        // Connect
        handler.set_state(ConnectionState::Connecting).await;
        assert_eq!(handler.state().await, ConnectionState::Connecting);

        // Connected
        handler.set_state(ConnectionState::Connected).await;
        assert_eq!(handler.state().await, ConnectionState::Connected);

        // Disconnect
        let disconnect = MessageBuilder::disconnect(ErrorCode::CONNECTION_CLOSED).build();
        handler.handle(&disconnect).await.unwrap();
        assert_eq!(handler.state().await, ConnectionState::Disconnected);
    }

    #[tokio::test]
    async fn test_message_handler_registration() {
        let handler = ProtocolHandler::new();
        handler.set_state(ConnectionState::Connected).await;

        let counter = Arc::new(AtomicU64::new(0));
        let counter_clone = counter.clone();

        handler
            .on_message(MessageType::Ping, move |_msg| {
                counter_clone.fetch_add(1, Ordering::SeqCst);
            })
            .await;

        let ping = MessageBuilder::ping().build();
        handler.handle(&ping).await.unwrap();

        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_create_command() {
        let handler = ProtocolHandler::new();

        let cmd = handler.create_command(
            BioCommand::AnalyzeSequence,
            serde_json::json!({"sequence": "ATCGATCG"}),
        );

        assert_eq!(cmd.message_type, MessageType::Command);
        if let MessagePayload::Command(payload) = cmd.payload {
            assert_eq!(payload.command, BioCommand::AnalyzeSequence);
            assert!(payload.correlation_id.is_some());
        }
    }
}
