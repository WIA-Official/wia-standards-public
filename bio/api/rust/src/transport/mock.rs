//! Mock transport for testing

use async_trait::async_trait;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use super::base::*;
use crate::protocol::BioMessage;

/// Mock transport for testing
#[derive(Debug)]
pub struct MockTransport {
    /// Connection state
    state: TransportState,
    /// Outgoing messages (sent by client)
    outgoing: Arc<Mutex<VecDeque<BioMessage>>>,
    /// Incoming messages (to be received by client)
    incoming: Arc<Mutex<VecDeque<BioMessage>>>,
    /// Metrics
    metrics: TransportMetrics,
    /// Simulate connection failure
    fail_connect: bool,
    /// Simulate send failure
    fail_send: bool,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            state: TransportState::Disconnected,
            outgoing: Arc::new(Mutex::new(VecDeque::new())),
            incoming: Arc::new(Mutex::new(VecDeque::new())),
            metrics: TransportMetrics::default(),
            fail_connect: false,
            fail_send: false,
        }
    }

    /// Set connect failure mode
    pub fn set_fail_connect(&mut self, fail: bool) {
        self.fail_connect = fail;
    }

    /// Set send failure mode
    pub fn set_fail_send(&mut self, fail: bool) {
        self.fail_send = fail;
    }

    /// Queue a message to be received
    pub fn queue_incoming(&self, message: BioMessage) {
        let mut incoming = self.incoming.lock().unwrap();
        incoming.push_back(message);
    }

    /// Queue multiple messages to be received
    pub fn queue_incoming_batch(&self, messages: Vec<BioMessage>) {
        let mut incoming = self.incoming.lock().unwrap();
        for msg in messages {
            incoming.push_back(msg);
        }
    }

    /// Get sent messages
    pub fn get_outgoing(&self) -> Vec<BioMessage> {
        let outgoing = self.outgoing.lock().unwrap();
        outgoing.iter().cloned().collect()
    }

    /// Clear sent messages
    pub fn clear_outgoing(&self) {
        let mut outgoing = self.outgoing.lock().unwrap();
        outgoing.clear();
    }

    /// Get incoming queue length
    pub fn incoming_count(&self) -> usize {
        let incoming = self.incoming.lock().unwrap();
        incoming.len()
    }

    /// Get outgoing queue length
    pub fn outgoing_count(&self) -> usize {
        let outgoing = self.outgoing.lock().unwrap();
        outgoing.len()
    }

    /// Get metrics
    pub fn metrics(&self) -> &TransportMetrics {
        &self.metrics
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ITransport for MockTransport {
    async fn connect(&mut self, _config: &TransportConfig) -> Result<(), TransportError> {
        if self.fail_connect {
            return Err(TransportError::ConnectionFailed("Mock connection failure".to_string()));
        }

        if self.state == TransportState::Connected {
            return Err(TransportError::AlreadyConnected);
        }

        self.state = TransportState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::NotConnected);
        }

        self.state = TransportState::Disconnected;
        Ok(())
    }

    async fn send(&mut self, message: &BioMessage) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::NotConnected);
        }

        if self.fail_send {
            return Err(TransportError::SendFailed("Mock send failure".to_string()));
        }

        let mut outgoing = self.outgoing.lock().unwrap();
        outgoing.push_back(message.clone());
        self.metrics.messages_sent += 1;

        Ok(())
    }

    async fn receive(&mut self) -> Result<BioMessage, TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::NotConnected);
        }

        let mut incoming = self.incoming.lock().unwrap();
        match incoming.pop_front() {
            Some(msg) => {
                self.metrics.messages_received += 1;
                Ok(msg)
            }
            None => Err(TransportError::ReceiveFailed("No messages available".to_string())),
        }
    }

    async fn try_receive(&mut self) -> Result<Option<BioMessage>, TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::NotConnected);
        }

        let mut incoming = self.incoming.lock().unwrap();
        match incoming.pop_front() {
            Some(msg) => {
                self.metrics.messages_received += 1;
                Ok(Some(msg))
            }
            None => Ok(None),
        }
    }

    fn is_connected(&self) -> bool {
        self.state == TransportState::Connected
    }

    fn state(&self) -> TransportState {
        self.state
    }

    fn transport_type(&self) -> &str {
        "mock"
    }
}

/// Mock transport builder
pub struct MockTransportBuilder {
    transport: MockTransport,
    initial_messages: Vec<BioMessage>,
}

impl MockTransportBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            transport: MockTransport::new(),
            initial_messages: Vec::new(),
        }
    }

    /// Configure to fail on connect
    pub fn fail_connect(mut self) -> Self {
        self.transport.fail_connect = true;
        self
    }

    /// Configure to fail on send
    pub fn fail_send(mut self) -> Self {
        self.transport.fail_send = true;
        self
    }

    /// Add initial incoming message
    pub fn with_incoming(mut self, message: BioMessage) -> Self {
        self.initial_messages.push(message);
        self
    }

    /// Add multiple initial incoming messages
    pub fn with_incoming_batch(mut self, messages: Vec<BioMessage>) -> Self {
        self.initial_messages.extend(messages);
        self
    }

    /// Build the mock transport
    pub fn build(self) -> MockTransport {
        let mut transport = self.transport;
        for msg in self.initial_messages {
            transport.queue_incoming(msg);
        }
        transport
    }
}

impl Default for MockTransportBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{MessageBuilder, MessageType, ProtocolCapability};

    #[tokio::test]
    async fn test_mock_transport_connect() {
        let mut transport = MockTransport::new();
        let config = TransportConfig::default();

        assert!(!transport.is_connected());
        transport.connect(&config).await.unwrap();
        assert!(transport.is_connected());
    }

    #[tokio::test]
    async fn test_mock_transport_send_receive() {
        let mut transport = MockTransport::new();
        let config = TransportConfig::default();
        transport.connect(&config).await.unwrap();

        // Queue incoming message
        let incoming_msg = MessageBuilder::ping().build();
        transport.queue_incoming(incoming_msg.clone());

        // Receive
        let received = transport.receive().await.unwrap();
        assert_eq!(received.message_type, MessageType::Ping);

        // Send
        let outgoing_msg = MessageBuilder::pong(123).build();
        transport.send(&outgoing_msg).await.unwrap();

        // Check outgoing
        let sent = transport.get_outgoing();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_mock_transport_fail_connect() {
        let mut transport = MockTransportBuilder::new()
            .fail_connect()
            .build();
        let config = TransportConfig::default();

        let result = transport.connect(&config).await;
        assert!(matches!(result, Err(TransportError::ConnectionFailed(_))));
    }

    #[tokio::test]
    async fn test_mock_transport_fail_send() {
        let mut transport = MockTransportBuilder::new()
            .fail_send()
            .build();
        let config = TransportConfig::default();
        transport.connect(&config).await.unwrap();

        let msg = MessageBuilder::ping().build();
        let result = transport.send(&msg).await;
        assert!(matches!(result, Err(TransportError::SendFailed(_))));
    }

    #[tokio::test]
    async fn test_mock_transport_try_receive_empty() {
        let mut transport = MockTransport::new();
        let config = TransportConfig::default();
        transport.connect(&config).await.unwrap();

        let result = transport.try_receive().await.unwrap();
        assert!(result.is_none());
    }

    #[tokio::test]
    async fn test_mock_transport_builder_with_messages() {
        let msg1 = MessageBuilder::ping().build();
        let msg2 = MessageBuilder::connect("test", "Test App").build();

        let mut transport = MockTransportBuilder::new()
            .with_incoming(msg1)
            .with_incoming(msg2)
            .build();

        let config = TransportConfig::default();
        transport.connect(&config).await.unwrap();

        assert_eq!(transport.incoming_count(), 2);

        let received1 = transport.receive().await.unwrap();
        assert_eq!(received1.message_type, MessageType::Ping);

        let received2 = transport.receive().await.unwrap();
        assert_eq!(received2.message_type, MessageType::Connect);
    }

    #[tokio::test]
    async fn test_mock_transport_metrics() {
        let mut transport = MockTransport::new();
        let config = TransportConfig::default();
        transport.connect(&config).await.unwrap();

        // Queue and receive
        transport.queue_incoming(MessageBuilder::ping().build());
        transport.receive().await.unwrap();

        // Send
        transport.send(&MessageBuilder::pong(123).build()).await.unwrap();

        let metrics = transport.metrics();
        assert_eq!(metrics.messages_sent, 1);
        assert_eq!(metrics.messages_received, 1);
    }
}
