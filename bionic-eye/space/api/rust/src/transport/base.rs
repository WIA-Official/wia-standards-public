//! Base transport trait and types

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::protocol::{ProtocolError, WspMessage};

/// Transport type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportType {
    Mock,
    WebSocket,
    Tcp,
    Dtn,
}

impl TransportType {
    pub fn is_reliable(&self) -> bool {
        matches!(self, TransportType::Tcp | TransportType::Dtn)
    }

    pub fn supports_streaming(&self) -> bool {
        matches!(self, TransportType::WebSocket | TransportType::Tcp)
    }
}

/// Transport error type
#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Not connected")]
    NotConnected,

    #[error("Connection closed")]
    ConnectionClosed,

    #[error("Send failed: {0}")]
    SendFailed(String),

    #[error("Receive failed: {0}")]
    ReceiveFailed(String),

    #[error("Timeout: {0}ms")]
    Timeout(u64),

    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Protocol error: {0}")]
    Protocol(#[from] ProtocolError),
}

/// Transport result type
pub type TransportResult<T> = Result<T, TransportError>;

/// Transport trait for WSP communication
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to endpoint
    async fn connect(&mut self, endpoint: &str) -> TransportResult<()>;

    /// Disconnect
    async fn disconnect(&mut self) -> TransportResult<()>;

    /// Send message
    async fn send(&self, message: &WspMessage) -> TransportResult<()>;

    /// Receive message
    async fn receive(&mut self) -> TransportResult<WspMessage>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Get transport type
    fn transport_type(&self) -> TransportType;

    /// Get endpoint URL
    fn endpoint(&self) -> Option<&str>;
}

/// Transport configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransportConfig {
    /// Connection timeout in milliseconds
    pub connect_timeout_ms: u64,

    /// Read timeout in milliseconds
    pub read_timeout_ms: u64,

    /// Write timeout in milliseconds
    pub write_timeout_ms: u64,

    /// Enable automatic reconnection
    pub auto_reconnect: bool,

    /// Maximum reconnection attempts
    pub max_reconnect_attempts: u32,

    /// Reconnection delay in milliseconds
    pub reconnect_delay_ms: u64,

    /// Enable message compression
    pub compression: bool,

    /// Enable TLS/encryption
    pub encryption: bool,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            connect_timeout_ms: 30000,
            read_timeout_ms: 60000,
            write_timeout_ms: 30000,
            auto_reconnect: true,
            max_reconnect_attempts: 5,
            reconnect_delay_ms: 2000,
            compression: false,
            encryption: false,
        }
    }
}

impl TransportConfig {
    /// Create config for low-latency environment (LEO, ground)
    pub fn low_latency() -> Self {
        Self {
            connect_timeout_ms: 5000,
            read_timeout_ms: 10000,
            write_timeout_ms: 5000,
            auto_reconnect: true,
            max_reconnect_attempts: 10,
            reconnect_delay_ms: 1000,
            compression: false,
            encryption: true,
        }
    }

    /// Create config for high-latency environment (deep space)
    pub fn high_latency() -> Self {
        Self {
            connect_timeout_ms: 300000,  // 5 minutes
            read_timeout_ms: 3600000,    // 1 hour
            write_timeout_ms: 300000,    // 5 minutes
            auto_reconnect: true,
            max_reconnect_attempts: 100,
            reconnect_delay_ms: 60000,   // 1 minute
            compression: true,
            encryption: true,
        }
    }

    /// Create config for Mars communication
    pub fn mars() -> Self {
        Self {
            connect_timeout_ms: 1800000, // 30 minutes
            read_timeout_ms: 7200000,    // 2 hours
            write_timeout_ms: 1800000,   // 30 minutes
            auto_reconnect: true,
            max_reconnect_attempts: 50,
            reconnect_delay_ms: 300000,  // 5 minutes
            compression: true,
            encryption: true,
        }
    }
}

/// Transport factory for creating transports
pub struct TransportFactory;

impl TransportFactory {
    /// Create transport from URL scheme
    pub fn from_url(url: &str) -> TransportResult<Box<dyn Transport>> {
        if url.starts_with("mock://") {
            Ok(Box::new(super::MockTransport::new()))
        } else if url.starts_with("ws://") || url.starts_with("wss://") {
            // TODO: Implement WebSocket transport
            Err(TransportError::ConnectionFailed(
                "WebSocket transport not yet implemented".to_string(),
            ))
        } else if url.starts_with("tcp://") {
            // TODO: Implement TCP transport
            Err(TransportError::ConnectionFailed(
                "TCP transport not yet implemented".to_string(),
            ))
        } else if url.starts_with("dtn://") {
            // TODO: Implement DTN transport
            Err(TransportError::ConnectionFailed(
                "DTN transport not yet implemented".to_string(),
            ))
        } else {
            Err(TransportError::ConnectionFailed(format!(
                "Unknown URL scheme: {}",
                url
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_type() {
        assert!(TransportType::Tcp.is_reliable());
        assert!(TransportType::Dtn.is_reliable());
        assert!(!TransportType::Mock.is_reliable());

        assert!(TransportType::WebSocket.supports_streaming());
        assert!(!TransportType::Dtn.supports_streaming());
    }

    #[test]
    fn test_transport_config_defaults() {
        let config = TransportConfig::default();
        assert_eq!(config.connect_timeout_ms, 30000);
        assert!(config.auto_reconnect);
    }

    #[test]
    fn test_transport_config_presets() {
        let low = TransportConfig::low_latency();
        let high = TransportConfig::high_latency();
        let mars = TransportConfig::mars();

        assert!(low.connect_timeout_ms < high.connect_timeout_ms);
        assert!(high.compression);
        assert!(mars.encryption);
    }

    #[tokio::test]
    async fn test_transport_factory_mock() {
        let transport = TransportFactory::from_url("mock://test");
        assert!(transport.is_ok());
        assert_eq!(transport.unwrap().transport_type(), TransportType::Mock);
    }

    #[tokio::test]
    async fn test_transport_factory_unknown() {
        let transport = TransportFactory::from_url("unknown://test");
        assert!(transport.is_err());
    }
}
