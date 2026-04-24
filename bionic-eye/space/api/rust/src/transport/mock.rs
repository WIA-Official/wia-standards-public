//! Mock transport for testing

use std::collections::VecDeque;
use std::sync::Arc;

use async_trait::async_trait;
use tokio::sync::Mutex;

use crate::protocol::WspMessage;

use super::base::{Transport, TransportError, TransportResult, TransportType};
use super::latency::{LatencyConfig, LatencySimulator};

/// Mock transport for testing
pub struct MockTransport {
    connected: bool,
    endpoint: Option<String>,
    outgoing: Arc<Mutex<VecDeque<WspMessage>>>,
    incoming: Arc<Mutex<VecDeque<WspMessage>>>,
    latency_simulator: Option<LatencySimulator>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            connected: false,
            endpoint: None,
            outgoing: Arc::new(Mutex::new(VecDeque::new())),
            incoming: Arc::new(Mutex::new(VecDeque::new())),
            latency_simulator: None,
        }
    }

    /// Create with latency simulation
    pub fn with_latency(mut self, config: LatencyConfig) -> Self {
        self.latency_simulator = Some(LatencySimulator::new(config));
        self
    }

    /// Get outgoing messages (for testing)
    pub async fn get_outgoing(&self) -> Vec<WspMessage> {
        self.outgoing.lock().await.iter().cloned().collect()
    }

    /// Add incoming message (for testing)
    pub async fn add_incoming(&self, message: WspMessage) {
        self.incoming.lock().await.push_back(message);
    }

    /// Clear all messages
    pub async fn clear(&self) {
        self.outgoing.lock().await.clear();
        self.incoming.lock().await.clear();
    }

    /// Get sent message count
    pub async fn sent_count(&self) -> usize {
        self.outgoing.lock().await.len()
    }

    /// Get pending incoming count
    pub async fn incoming_count(&self) -> usize {
        self.incoming.lock().await.len()
    }

    /// Create a pair of connected mock transports
    pub fn create_pair() -> (MockTransport, MockTransport) {
        let outgoing_a = Arc::new(Mutex::new(VecDeque::new()));
        let outgoing_b = Arc::new(Mutex::new(VecDeque::new()));

        let transport_a = MockTransport {
            connected: true,
            endpoint: Some("mock://peer-b".to_string()),
            outgoing: outgoing_a.clone(),
            incoming: outgoing_b.clone(),
            latency_simulator: None,
        };

        let transport_b = MockTransport {
            connected: true,
            endpoint: Some("mock://peer-a".to_string()),
            outgoing: outgoing_b,
            incoming: outgoing_a,
            latency_simulator: None,
        };

        (transport_a, transport_b)
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for MockTransport {
    async fn connect(&mut self, endpoint: &str) -> TransportResult<()> {
        if self.connected {
            return Err(TransportError::ConnectionFailed(
                "Already connected".to_string(),
            ));
        }

        // Simulate connection delay
        if let Some(ref simulator) = self.latency_simulator {
            simulator.apply_delay().await;
        }

        self.connected = true;
        self.endpoint = Some(endpoint.to_string());
        Ok(())
    }

    async fn disconnect(&mut self) -> TransportResult<()> {
        if !self.connected {
            return Err(TransportError::NotConnected);
        }

        self.connected = false;
        self.endpoint = None;
        Ok(())
    }

    async fn send(&self, message: &WspMessage) -> TransportResult<()> {
        if !self.connected {
            return Err(TransportError::NotConnected);
        }

        // Simulate send delay
        if let Some(ref simulator) = self.latency_simulator {
            simulator.apply_delay().await;
        }

        self.outgoing.lock().await.push_back(message.clone());
        Ok(())
    }

    async fn receive(&mut self) -> TransportResult<WspMessage> {
        if !self.connected {
            return Err(TransportError::NotConnected);
        }

        // Simulate receive delay
        if let Some(ref simulator) = self.latency_simulator {
            simulator.apply_delay().await;
        }

        let mut incoming = self.incoming.lock().await;
        incoming
            .pop_front()
            .ok_or_else(|| TransportError::ReceiveFailed("No messages available".to_string()))
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn transport_type(&self) -> TransportType {
        TransportType::Mock
    }

    fn endpoint(&self) -> Option<&str> {
        self.endpoint.as_deref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{Endpoint, MessageType};

    #[tokio::test]
    async fn test_mock_transport_connect() {
        let mut transport = MockTransport::new();

        assert!(!transport.is_connected());

        transport.connect("mock://test").await.unwrap();
        assert!(transport.is_connected());
        assert_eq!(transport.endpoint(), Some("mock://test"));
    }

    #[tokio::test]
    async fn test_mock_transport_disconnect() {
        let mut transport = MockTransport::new();
        transport.connect("mock://test").await.unwrap();

        transport.disconnect().await.unwrap();
        assert!(!transport.is_connected());
        assert!(transport.endpoint().is_none());
    }

    #[tokio::test]
    async fn test_mock_transport_send() {
        let mut transport = MockTransport::new();
        transport.connect("mock://test").await.unwrap();

        let message = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::spacecraft("sc-01"),
            serde_json::json!({}),
        );

        transport.send(&message).await.unwrap();
        assert_eq!(transport.sent_count().await, 1);
    }

    #[tokio::test]
    async fn test_mock_transport_receive() {
        let mut transport = MockTransport::new();
        transport.connect("mock://test").await.unwrap();

        let message = WspMessage::new(
            MessageType::Pong,
            Endpoint::spacecraft("sc-01"),
            Endpoint::ground_station("gs-01"),
            serde_json::json!({}),
        );

        transport.add_incoming(message.clone()).await;

        let received = transport.receive().await.unwrap();
        assert_eq!(received.message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_mock_transport_pair() {
        let (mut transport_a, mut transport_b) = MockTransport::create_pair();

        let message = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::spacecraft("sc-01"),
            serde_json::json!({}),
        );

        transport_a.send(&message).await.unwrap();

        let received = transport_b.receive().await.unwrap();
        assert_eq!(received.message_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_mock_transport_not_connected() {
        let transport = MockTransport::new();

        let message = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::spacecraft("sc-01"),
            serde_json::json!({}),
        );

        let result = transport.send(&message).await;
        assert!(matches!(result, Err(TransportError::NotConnected)));
    }

    #[tokio::test]
    async fn test_mock_transport_with_latency() {
        use std::time::Instant;

        let config = LatencyConfig::custom(100, 0.0);
        let mut transport = MockTransport::new().with_latency(config);
        transport.connect("mock://test").await.unwrap();

        let message = WspMessage::new(
            MessageType::Ping,
            Endpoint::ground_station("gs-01"),
            Endpoint::spacecraft("sc-01"),
            serde_json::json!({}),
        );

        let start = Instant::now();
        transport.send(&message).await.unwrap();
        let elapsed = start.elapsed();

        // Should take at least 100ms due to latency simulation
        assert!(elapsed.as_millis() >= 100);
    }
}
