//! WIA Health Transport Layer
//!
//! Transport implementations for the WIA Health protocol

mod websocket;

pub use websocket::*;

use async_trait::async_trait;
use crate::error::Result;
use crate::protocol::Message;

/// Transport status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportStatus {
    /// Not connected
    Disconnected,
    /// Attempting to connect
    Connecting,
    /// Connected and ready
    Connected,
    /// Connection error occurred
    Error,
}

/// Transport event
#[derive(Debug, Clone)]
pub enum TransportEvent {
    /// Connection established
    Connected,
    /// Connection closed
    Disconnected { reason: String },
    /// Message received
    Message(Message),
    /// Error occurred
    Error(String),
}

/// Transport layer trait for different connection types
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to the server
    async fn connect(&mut self, url: &str) -> Result<()>;

    /// Disconnect from the server
    async fn disconnect(&mut self) -> Result<()>;

    /// Send a message
    async fn send(&self, message: Message) -> Result<()>;

    /// Get current status
    fn status(&self) -> TransportStatus;

    /// Check if connected
    fn is_connected(&self) -> bool {
        self.status() == TransportStatus::Connected
    }
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Connection URL
    pub url: String,
    /// Reconnect automatically on disconnect
    pub auto_reconnect: bool,
    /// Reconnect delay in milliseconds
    pub reconnect_delay_ms: u64,
    /// Maximum reconnect attempts
    pub max_reconnect_attempts: u32,
    /// Connection timeout in milliseconds
    pub connect_timeout_ms: u64,
    /// Enable TLS/SSL
    pub use_tls: bool,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            url: "wss://localhost:8080/wia-health".to_string(),
            auto_reconnect: true,
            reconnect_delay_ms: 1000,
            max_reconnect_attempts: 5,
            connect_timeout_ms: 10000,
            use_tls: true,
        }
    }
}

impl TransportConfig {
    /// Create config with WebSocket URL
    pub fn with_url(url: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            ..Default::default()
        }
    }

    /// Disable auto-reconnect
    pub fn no_reconnect(mut self) -> Self {
        self.auto_reconnect = false;
        self
    }

    /// Set reconnect delay
    pub fn reconnect_delay(mut self, ms: u64) -> Self {
        self.reconnect_delay_ms = ms;
        self
    }

    /// Set connection timeout
    pub fn timeout(mut self, ms: u64) -> Self {
        self.connect_timeout_ms = ms;
        self
    }
}
