//! # WIA Climate Transport Module
//!
//! This module provides transport layer implementations for the WIA Climate protocol.
//! It supports WebSocket, MQTT, and mock transports for testing.

mod websocket;
mod mock;

pub use websocket::*;
pub use mock::*;

use async_trait::async_trait;
use crate::protocol::ProtocolMessage;
use crate::error::Result;

/// Transport trait for sending and receiving protocol messages
///
/// This trait abstracts the underlying transport mechanism (WebSocket, MQTT, etc.)
/// allowing the protocol layer to remain transport-agnostic.
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to the remote endpoint
    async fn connect(&mut self, url: &str) -> Result<()>;

    /// Disconnect from the remote endpoint
    async fn disconnect(&mut self) -> Result<()>;

    /// Send a protocol message
    async fn send(&self, message: &ProtocolMessage) -> Result<()>;

    /// Receive a protocol message
    ///
    /// This method blocks until a message is received or the connection is closed.
    async fn receive(&mut self) -> Result<ProtocolMessage>;

    /// Check if the transport is connected
    fn is_connected(&self) -> bool;

    /// Get the transport name
    fn name(&self) -> &str;
}

/// Transport event for async handling
#[derive(Debug, Clone)]
pub enum TransportEvent {
    /// Connection established
    Connected,
    /// Connection closed
    Disconnected {
        /// Reason for disconnection
        reason: Option<String>,
    },
    /// Message received
    Message(ProtocolMessage),
    /// Error occurred
    Error {
        /// Error description
        message: String,
    },
}

/// Configuration for transport connections
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Connection URL
    pub url: String,

    /// Connection timeout in milliseconds
    pub connect_timeout_ms: u64,

    /// Read timeout in milliseconds
    pub read_timeout_ms: Option<u64>,

    /// Write timeout in milliseconds
    pub write_timeout_ms: Option<u64>,

    /// Enable automatic reconnection
    pub auto_reconnect: bool,

    /// Maximum reconnection attempts
    pub max_reconnect_attempts: u32,

    /// Base delay for reconnection (exponential backoff)
    pub reconnect_delay_ms: u64,

    /// Maximum reconnection delay
    pub max_reconnect_delay_ms: u64,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            url: String::new(),
            connect_timeout_ms: 10000,
            read_timeout_ms: None,
            write_timeout_ms: Some(5000),
            auto_reconnect: true,
            max_reconnect_attempts: 10,
            reconnect_delay_ms: 1000,
            max_reconnect_delay_ms: 30000,
        }
    }
}

impl TransportConfig {
    /// Create a new transport config with the given URL
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            ..Default::default()
        }
    }

    /// Set connection timeout
    pub fn connect_timeout(mut self, ms: u64) -> Self {
        self.connect_timeout_ms = ms;
        self
    }

    /// Disable automatic reconnection
    pub fn no_auto_reconnect(mut self) -> Self {
        self.auto_reconnect = false;
        self
    }

    /// Set maximum reconnection attempts
    pub fn max_reconnect_attempts(mut self, attempts: u32) -> Self {
        self.max_reconnect_attempts = attempts;
        self
    }
}
