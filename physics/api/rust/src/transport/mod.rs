//! Transport Layer Implementation
//!
//! This module provides transport adapters for the WIA Physics Protocol.
//! Supports WebSocket, TCP, and mock transports.

mod traits;
mod websocket;
mod tcp;
mod mock;

pub use traits::*;
pub use websocket::*;
pub use tcp::*;
pub use mock::*;

/// Transport type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportType {
    /// WebSocket transport (primary for web)
    WebSocket,
    /// TCP transport (high performance)
    Tcp,
    /// Mock transport (testing)
    Mock,
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Transport type
    pub transport_type: TransportType,
    /// Server URL or address
    pub url: String,
    /// Use TLS
    pub tls: bool,
    /// Connection timeout in milliseconds
    pub connect_timeout_ms: u64,
    /// Read timeout in milliseconds
    pub read_timeout_ms: u64,
    /// Write timeout in milliseconds
    pub write_timeout_ms: u64,
    /// Buffer size
    pub buffer_size: usize,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            transport_type: TransportType::WebSocket,
            url: "wss://localhost:443/wpp/v1".to_string(),
            tls: true,
            connect_timeout_ms: 10000,
            read_timeout_ms: 30000,
            write_timeout_ms: 10000,
            buffer_size: 65536,
        }
    }
}

impl TransportConfig {
    /// Create WebSocket transport config
    pub fn websocket(url: &str) -> Self {
        Self {
            transport_type: TransportType::WebSocket,
            url: url.to_string(),
            tls: url.starts_with("wss://"),
            ..Default::default()
        }
    }

    /// Create TCP transport config
    pub fn tcp(address: &str) -> Self {
        Self {
            transport_type: TransportType::Tcp,
            url: address.to_string(),
            tls: false,
            ..Default::default()
        }
    }

    /// Create mock transport config
    pub fn mock() -> Self {
        Self {
            transport_type: TransportType::Mock,
            url: "mock://localhost".to_string(),
            tls: false,
            ..Default::default()
        }
    }

    /// Set TLS
    pub fn with_tls(mut self, tls: bool) -> Self {
        self.tls = tls;
        self
    }

    /// Set connection timeout
    pub fn with_connect_timeout(mut self, ms: u64) -> Self {
        self.connect_timeout_ms = ms;
        self
    }
}
