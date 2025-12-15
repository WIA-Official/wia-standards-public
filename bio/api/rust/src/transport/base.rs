//! Base transport interface and types

use async_trait::async_trait;
use std::fmt::Debug;
use thiserror::Error;

use crate::protocol::BioMessage;

/// Transport error types
#[derive(Debug, Error)]
pub enum TransportError {
    /// Connection failed
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    /// Connection closed
    #[error("Connection closed: {0}")]
    ConnectionClosed(String),

    /// Send failed
    #[error("Send failed: {0}")]
    SendFailed(String),

    /// Receive failed
    #[error("Receive failed: {0}")]
    ReceiveFailed(String),

    /// Timeout
    #[error("Operation timed out")]
    Timeout,

    /// Invalid URL
    #[error("Invalid URL: {0}")]
    InvalidUrl(String),

    /// TLS/SSL error
    #[error("TLS error: {0}")]
    TlsError(String),

    /// Transport not connected
    #[error("Transport not connected")]
    NotConnected,

    /// Transport already connected
    #[error("Transport already connected")]
    AlreadyConnected,

    /// Internal error
    #[error("Internal error: {0}")]
    Internal(String),
}

/// Transport connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportState {
    /// Not connected
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Connected and ready
    Connected,
    /// Reconnecting after disconnect
    Reconnecting,
    /// Closed (terminal state)
    Closed,
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Connection URL
    pub url: String,
    /// Connection timeout in milliseconds
    pub connect_timeout: u64,
    /// Read timeout in milliseconds
    pub read_timeout: u64,
    /// Write timeout in milliseconds
    pub write_timeout: u64,
    /// Enable automatic reconnection
    pub auto_reconnect: bool,
    /// Maximum reconnection attempts
    pub max_reconnect_attempts: u32,
    /// Reconnection delay in milliseconds
    pub reconnect_delay: u64,
    /// Enable TLS verification
    pub tls_verify: bool,
    /// Buffer size for messages
    pub buffer_size: usize,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            url: String::new(),
            connect_timeout: 10000,
            read_timeout: 30000,
            write_timeout: 10000,
            auto_reconnect: true,
            max_reconnect_attempts: 5,
            reconnect_delay: 1000,
            tls_verify: true,
            buffer_size: 65536,
        }
    }
}

impl TransportConfig {
    /// Create config with URL
    pub fn with_url(url: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            ..Default::default()
        }
    }

    /// Set connection timeout
    pub fn connect_timeout(mut self, ms: u64) -> Self {
        self.connect_timeout = ms;
        self
    }

    /// Set auto reconnect
    pub fn auto_reconnect(mut self, enabled: bool) -> Self {
        self.auto_reconnect = enabled;
        self
    }

    /// Set max reconnect attempts
    pub fn max_reconnect_attempts(mut self, attempts: u32) -> Self {
        self.max_reconnect_attempts = attempts;
        self
    }

    /// Set TLS verification
    pub fn tls_verify(mut self, verify: bool) -> Self {
        self.tls_verify = verify;
        self
    }
}

/// Transport events
#[derive(Debug, Clone)]
pub enum TransportEvent {
    /// Connected successfully
    Connected,
    /// Disconnected
    Disconnected { reason: String },
    /// Message received
    Message(BioMessage),
    /// Error occurred
    Error(String),
    /// Reconnecting
    Reconnecting { attempt: u32 },
}

/// Message handler callback type
pub type MessageHandler = Box<dyn Fn(BioMessage) + Send + Sync>;

/// Error handler callback type
pub type ErrorHandler = Box<dyn Fn(TransportError) + Send + Sync>;

/// Close handler callback type
pub type CloseHandler = Box<dyn Fn(String) + Send + Sync>;

/// Event handler callback type
pub type EventHandler = Box<dyn Fn(TransportEvent) + Send + Sync>;

/// Abstract transport interface
///
/// All transport implementations must implement this trait to work
/// with the WIA Bio protocol layer.
#[async_trait]
pub trait ITransport: Send + Sync + Debug {
    /// Connect to the remote endpoint
    async fn connect(&mut self, config: &TransportConfig) -> Result<(), TransportError>;

    /// Disconnect from the remote endpoint
    async fn disconnect(&mut self) -> Result<(), TransportError>;

    /// Send a message
    async fn send(&mut self, message: &BioMessage) -> Result<(), TransportError>;

    /// Receive a message (blocking)
    async fn receive(&mut self) -> Result<BioMessage, TransportError>;

    /// Try to receive a message (non-blocking)
    async fn try_receive(&mut self) -> Result<Option<BioMessage>, TransportError>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Get current state
    fn state(&self) -> TransportState;

    /// Get transport type name
    fn transport_type(&self) -> &str;
}

/// Transport metrics
#[derive(Debug, Clone, Default)]
pub struct TransportMetrics {
    /// Messages sent
    pub messages_sent: u64,
    /// Messages received
    pub messages_received: u64,
    /// Bytes sent
    pub bytes_sent: u64,
    /// Bytes received
    pub bytes_received: u64,
    /// Connection uptime in seconds
    pub uptime_seconds: u64,
    /// Reconnection count
    pub reconnect_count: u32,
    /// Last error
    pub last_error: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_builder() {
        let config = TransportConfig::with_url("wss://bio.wia.live/ws")
            .connect_timeout(5000)
            .auto_reconnect(true)
            .max_reconnect_attempts(10)
            .tls_verify(true);

        assert_eq!(config.url, "wss://bio.wia.live/ws");
        assert_eq!(config.connect_timeout, 5000);
        assert!(config.auto_reconnect);
        assert_eq!(config.max_reconnect_attempts, 10);
    }

    #[test]
    fn test_transport_state() {
        assert_ne!(TransportState::Connected, TransportState::Disconnected);
        assert_eq!(TransportState::Connected, TransportState::Connected);
    }
}
