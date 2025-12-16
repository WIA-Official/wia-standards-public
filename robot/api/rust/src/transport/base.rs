//! Transport layer abstractions

use crate::error::{RobotError, RobotResult};
use crate::protocol::WrpMessage;

/// Transport type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportType {
    /// WebSocket transport
    WebSocket,
    /// MQTT transport
    Mqtt,
    /// ROS2 DDS transport
    Ros2Dds,
    /// Raw TCP transport
    Tcp,
    /// Mock transport for testing
    Mock,
}

impl std::fmt::Display for TransportType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportType::WebSocket => write!(f, "websocket"),
            TransportType::Mqtt => write!(f, "mqtt"),
            TransportType::Ros2Dds => write!(f, "ros2_dds"),
            TransportType::Tcp => write!(f, "tcp"),
            TransportType::Mock => write!(f, "mock"),
        }
    }
}

/// Transport configuration
#[derive(Debug, Clone)]
pub struct TransportConfig {
    /// Endpoint URL or address
    pub endpoint: String,
    /// Port number
    pub port: u16,
    /// Connection timeout in milliseconds
    pub timeout_ms: u64,
    /// Number of retry attempts
    pub retry_count: u32,
    /// Use TLS encryption
    pub use_tls: bool,
    /// Additional options
    pub options: std::collections::HashMap<String, String>,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            endpoint: "localhost".to_string(),
            port: 8080,
            timeout_ms: 5000,
            retry_count: 3,
            use_tls: false,
            options: std::collections::HashMap::new(),
        }
    }
}

impl TransportConfig {
    /// Create a new transport config
    pub fn new(endpoint: &str, port: u16) -> Self {
        Self {
            endpoint: endpoint.to_string(),
            port,
            ..Default::default()
        }
    }

    /// Enable TLS
    pub fn with_tls(mut self) -> Self {
        self.use_tls = true;
        self
    }

    /// Set timeout
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Set retry count
    pub fn with_retries(mut self, count: u32) -> Self {
        self.retry_count = count;
        self
    }

    /// Add option
    pub fn with_option(mut self, key: &str, value: &str) -> Self {
        self.options.insert(key.to_string(), value.to_string());
        self
    }

    /// Get connection URL
    pub fn url(&self) -> String {
        let scheme = if self.use_tls { "wss" } else { "ws" };
        format!("{}://{}:{}", scheme, self.endpoint, self.port)
    }
}

/// Transport trait for sending and receiving WRP messages
pub trait Transport: Send + Sync {
    /// Get transport type
    fn transport_type(&self) -> TransportType;

    /// Connect to the remote endpoint
    fn connect(&mut self, config: &TransportConfig) -> RobotResult<()>;

    /// Disconnect from the remote endpoint
    fn disconnect(&mut self) -> RobotResult<()>;

    /// Send a message
    fn send(&mut self, message: &WrpMessage) -> RobotResult<()>;

    /// Receive a message (blocking)
    fn receive(&mut self) -> RobotResult<WrpMessage>;

    /// Try to receive a message (non-blocking)
    fn try_receive(&mut self) -> RobotResult<Option<WrpMessage>>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Get current latency in milliseconds
    fn latency_ms(&self) -> u64;

    /// Get transport statistics
    fn stats(&self) -> TransportStats;
}

/// Transport statistics
#[derive(Debug, Clone, Default)]
pub struct TransportStats {
    /// Total messages sent
    pub messages_sent: u64,
    /// Total messages received
    pub messages_received: u64,
    /// Total bytes sent
    pub bytes_sent: u64,
    /// Total bytes received
    pub bytes_received: u64,
    /// Average latency in milliseconds
    pub avg_latency_ms: f64,
    /// Number of errors
    pub errors: u64,
    /// Number of reconnects
    pub reconnects: u64,
}

impl TransportStats {
    /// Record a sent message
    pub fn record_send(&mut self, bytes: u64) {
        self.messages_sent += 1;
        self.bytes_sent += bytes;
    }

    /// Record a received message
    pub fn record_receive(&mut self, bytes: u64) {
        self.messages_received += 1;
        self.bytes_received += bytes;
    }

    /// Record an error
    pub fn record_error(&mut self) {
        self.errors += 1;
    }

    /// Record a reconnect
    pub fn record_reconnect(&mut self) {
        self.reconnects += 1;
    }

    /// Update average latency
    pub fn update_latency(&mut self, latency_ms: f64) {
        let total = self.messages_sent + self.messages_received;
        if total > 0 {
            self.avg_latency_ms = (self.avg_latency_ms * (total - 1) as f64 + latency_ms) / total as f64;
        } else {
            self.avg_latency_ms = latency_ms;
        }
    }
}

/// QoS settings for message delivery
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosLevel {
    /// At most once delivery (fire and forget)
    AtMostOnce,
    /// At least once delivery (with acknowledgment)
    AtLeastOnce,
    /// Exactly once delivery (full handshake)
    ExactlyOnce,
}

impl Default for QosLevel {
    fn default() -> Self {
        QosLevel::AtLeastOnce
    }
}

/// QoS profile for different message types
#[derive(Debug, Clone)]
pub struct QosProfile {
    /// Reliability level
    pub reliability: QosLevel,
    /// Message durability
    pub durability: Durability,
    /// Maximum latency in milliseconds (0 = unlimited)
    pub max_latency_ms: u64,
}

impl Default for QosProfile {
    fn default() -> Self {
        Self {
            reliability: QosLevel::AtLeastOnce,
            durability: Durability::Volatile,
            max_latency_ms: 0,
        }
    }
}

impl QosProfile {
    /// Create a reliable profile
    pub fn reliable() -> Self {
        Self {
            reliability: QosLevel::AtLeastOnce,
            durability: Durability::Transient,
            max_latency_ms: 0,
        }
    }

    /// Create a best-effort profile
    pub fn best_effort() -> Self {
        Self {
            reliability: QosLevel::AtMostOnce,
            durability: Durability::Volatile,
            max_latency_ms: 0,
        }
    }

    /// Create an emergency profile
    pub fn emergency() -> Self {
        Self {
            reliability: QosLevel::AtLeastOnce,
            durability: Durability::Transient,
            max_latency_ms: 10,
        }
    }

    /// Create a control profile
    pub fn control() -> Self {
        Self {
            reliability: QosLevel::AtLeastOnce,
            durability: Durability::Volatile,
            max_latency_ms: 50,
        }
    }

    /// Create a telemetry profile
    pub fn telemetry() -> Self {
        Self {
            reliability: QosLevel::AtMostOnce,
            durability: Durability::Volatile,
            max_latency_ms: 100,
        }
    }
}

/// Message durability
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Message is not stored
    Volatile,
    /// Message is stored temporarily
    Transient,
    /// Message is stored persistently
    Persistent,
}

impl Default for Durability {
    fn default() -> Self {
        Durability::Volatile
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_config() {
        let config = TransportConfig::new("example.com", 443)
            .with_tls()
            .with_timeout(10000)
            .with_retries(5);

        assert_eq!(config.endpoint, "example.com");
        assert_eq!(config.port, 443);
        assert!(config.use_tls);
        assert_eq!(config.timeout_ms, 10000);
        assert_eq!(config.retry_count, 5);
        assert_eq!(config.url(), "wss://example.com:443");
    }

    #[test]
    fn test_transport_stats() {
        let mut stats = TransportStats::default();
        stats.record_send(100);
        stats.record_receive(200);
        stats.update_latency(50.0);

        assert_eq!(stats.messages_sent, 1);
        assert_eq!(stats.messages_received, 1);
        assert_eq!(stats.bytes_sent, 100);
        assert_eq!(stats.bytes_received, 200);
    }

    #[test]
    fn test_qos_profiles() {
        let emergency = QosProfile::emergency();
        assert_eq!(emergency.reliability, QosLevel::AtLeastOnce);
        assert_eq!(emergency.max_latency_ms, 10);

        let telemetry = QosProfile::telemetry();
        assert_eq!(telemetry.reliability, QosLevel::AtMostOnce);
    }
}
