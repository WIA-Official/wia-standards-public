//! Transport Traits
//!
//! Defines the core traits for transport implementations.

use async_trait::async_trait;
use tokio::sync::mpsc;

use crate::error::PhysicsResult;
use crate::protocol::WppMessage;

// ============================================================================
// Transport Trait
// ============================================================================

/// Core transport trait for WPP communication
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to the server
    async fn connect(&mut self) -> PhysicsResult<()>;

    /// Disconnect from the server
    async fn disconnect(&mut self) -> PhysicsResult<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Send a message
    async fn send(&mut self, message: WppMessage) -> PhysicsResult<()>;

    /// Receive a message (blocking)
    async fn receive(&mut self) -> PhysicsResult<WppMessage>;

    /// Get the remote address
    fn remote_addr(&self) -> Option<String>;
}

// ============================================================================
// Transport Factory
// ============================================================================

/// Factory for creating transport instances
#[async_trait]
pub trait TransportFactory: Send + Sync {
    /// Create a new transport instance
    async fn create(&self, url: &str) -> PhysicsResult<Box<dyn Transport>>;
}

// ============================================================================
// Stream Transport
// ============================================================================

/// Extended transport with streaming support
#[async_trait]
pub trait StreamTransport: Transport {
    /// Start receiving messages in background
    /// Returns a channel receiver for incoming messages
    async fn start_receiving(&mut self) -> PhysicsResult<mpsc::Receiver<WppMessage>>;

    /// Stop receiving
    async fn stop_receiving(&mut self) -> PhysicsResult<()>;
}

// ============================================================================
// Transport Events
// ============================================================================

/// Events that can occur on a transport
#[derive(Debug, Clone)]
pub enum TransportEvent {
    /// Connected to server
    Connected,
    /// Disconnected from server
    Disconnected { reason: String },
    /// Message received
    MessageReceived(WppMessage),
    /// Error occurred
    Error { message: String },
    /// Reconnecting
    Reconnecting { attempt: u32 },
}

/// Transport event handler
#[async_trait]
pub trait TransportEventHandler: Send + Sync {
    /// Handle transport event
    async fn on_event(&self, event: TransportEvent);
}

// ============================================================================
// Transport Stats
// ============================================================================

/// Transport statistics
#[derive(Debug, Clone, Default)]
pub struct TransportStats {
    /// Number of messages sent
    pub messages_sent: u64,
    /// Number of messages received
    pub messages_received: u64,
    /// Bytes sent
    pub bytes_sent: u64,
    /// Bytes received
    pub bytes_received: u64,
    /// Connection time in seconds
    pub connection_time: f64,
    /// Number of reconnections
    pub reconnections: u32,
    /// Last latency in milliseconds
    pub last_latency_ms: u64,
    /// Average latency in milliseconds
    pub avg_latency_ms: f64,
}

impl TransportStats {
    /// Create new stats
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a sent message
    pub fn record_sent(&mut self, bytes: u64) {
        self.messages_sent += 1;
        self.bytes_sent += bytes;
    }

    /// Record a received message
    pub fn record_received(&mut self, bytes: u64) {
        self.messages_received += 1;
        self.bytes_received += bytes;
    }

    /// Record latency
    pub fn record_latency(&mut self, latency_ms: u64) {
        self.last_latency_ms = latency_ms;
        // Simple moving average
        let count = self.messages_received as f64;
        self.avg_latency_ms = (self.avg_latency_ms * (count - 1.0) + latency_ms as f64) / count;
    }
}

// ============================================================================
// Transport State
// ============================================================================

/// Transport connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportState {
    /// Not connected
    Disconnected,
    /// Connecting
    Connecting,
    /// Connected
    Connected,
    /// Disconnecting
    Disconnecting,
    /// Reconnecting
    Reconnecting,
    /// Error state
    Error,
}
