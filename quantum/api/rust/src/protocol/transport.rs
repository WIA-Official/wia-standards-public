//! Transport layer abstractions

use crate::error::Result;
use async_trait::async_trait;
use std::sync::Arc;
use tokio::sync::mpsc;

use super::Message;

/// Message handler callback type
pub type MessageHandler = Box<dyn Fn(Message) + Send + Sync>;

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Error,
}

/// Transport trait for different communication backends
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to the server
    async fn connect(&mut self, url: &str) -> Result<()>;

    /// Disconnect from the server
    async fn disconnect(&mut self) -> Result<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Get connection state
    fn state(&self) -> ConnectionState;

    /// Send a message
    async fn send(&self, message: &Message) -> Result<()>;

    /// Receive a message (blocking)
    async fn receive(&mut self) -> Result<Option<Message>>;
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Connection timeout in milliseconds
    pub connect_timeout: u64,
    /// Message send timeout in milliseconds
    pub send_timeout: u64,
    /// Enable auto-reconnect
    pub auto_reconnect: bool,
    /// Maximum reconnect attempts
    pub max_reconnect_attempts: u32,
    /// Initial reconnect delay in milliseconds
    pub initial_reconnect_delay: u64,
    /// Maximum reconnect delay in milliseconds
    pub max_reconnect_delay: u64,
    /// Reconnect backoff multiplier
    pub backoff_multiplier: f64,
    /// Ping interval in milliseconds
    pub ping_interval: u64,
    /// Pong timeout in milliseconds
    pub pong_timeout: u64,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            connect_timeout: 30000,
            send_timeout: 10000,
            auto_reconnect: true,
            max_reconnect_attempts: 5,
            initial_reconnect_delay: 1000,
            max_reconnect_delay: 30000,
            backoff_multiplier: 2.0,
            ping_interval: 30000,
            pong_timeout: 10000,
        }
    }
}

/// Mock transport for testing
#[derive(Debug)]
pub struct MockTransport {
    state: ConnectionState,
    config: TransportConfig,
    sent_messages: Vec<Message>,
    receive_queue: Vec<Message>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            state: ConnectionState::Disconnected,
            config: TransportConfig::default(),
            sent_messages: Vec::new(),
            receive_queue: Vec::new(),
        }
    }

    /// Create with custom config
    pub fn with_config(config: TransportConfig) -> Self {
        Self {
            state: ConnectionState::Disconnected,
            config,
            sent_messages: Vec::new(),
            receive_queue: Vec::new(),
        }
    }

    /// Queue a message to be received
    pub fn queue_receive(&mut self, message: Message) {
        self.receive_queue.push(message);
    }

    /// Get sent messages
    pub fn sent_messages(&self) -> &[Message] {
        &self.sent_messages
    }

    /// Clear sent messages
    pub fn clear_sent(&mut self) {
        self.sent_messages.clear();
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for MockTransport {
    async fn connect(&mut self, _url: &str) -> Result<()> {
        self.state = ConnectionState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.state = ConnectionState::Disconnected;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.state == ConnectionState::Connected
    }

    fn state(&self) -> ConnectionState {
        self.state
    }

    async fn send(&self, message: &Message) -> Result<()> {
        // In a real implementation, we'd send over the wire
        // For mock, we just acknowledge
        Ok(())
    }

    async fn receive(&mut self) -> Result<Option<Message>> {
        if self.receive_queue.is_empty() {
            Ok(None)
        } else {
            Ok(Some(self.receive_queue.remove(0)))
        }
    }
}

/// WebSocket transport (placeholder - requires tokio-tungstenite)
#[cfg(feature = "websocket")]
pub struct WebSocketTransport {
    state: ConnectionState,
    config: TransportConfig,
    url: Option<String>,
}

#[cfg(feature = "websocket")]
impl WebSocketTransport {
    /// Create a new WebSocket transport
    pub fn new() -> Self {
        Self {
            state: ConnectionState::Disconnected,
            config: TransportConfig::default(),
            url: None,
        }
    }

    /// Create with custom config
    pub fn with_config(config: TransportConfig) -> Self {
        Self {
            state: ConnectionState::Disconnected,
            config,
            url: None,
        }
    }
}

/// Transport event
#[derive(Debug, Clone)]
pub enum TransportEvent {
    Connected,
    Disconnected { reason: String },
    Message(Message),
    Error { message: String },
    Reconnecting { attempt: u32 },
}

/// Transport event sender
pub type TransportEventSender = mpsc::UnboundedSender<TransportEvent>;

/// Transport event receiver
pub type TransportEventReceiver = mpsc::UnboundedReceiver<TransportEvent>;

/// Create transport event channel
pub fn transport_channel() -> (TransportEventSender, TransportEventReceiver) {
    mpsc::unbounded_channel()
}
