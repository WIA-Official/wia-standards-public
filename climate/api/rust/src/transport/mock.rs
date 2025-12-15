//! Mock transport for testing

use async_trait::async_trait;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use super::Transport;
use crate::protocol::ProtocolMessage;
use crate::error::{ClimateError, Result};

/// Mock transport for testing protocol interactions
///
/// This transport allows you to queue messages for receiving and
/// capture sent messages for assertions.
pub struct MockTransport {
    connected: bool,
    receive_queue: Arc<Mutex<VecDeque<ProtocolMessage>>>,
    sent_messages: Arc<Mutex<Vec<ProtocolMessage>>>,
    connection_url: Option<String>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            connected: false,
            receive_queue: Arc::new(Mutex::new(VecDeque::new())),
            sent_messages: Arc::new(Mutex::new(Vec::new())),
            connection_url: None,
        }
    }

    /// Queue a message to be received
    pub fn queue_receive(&self, message: ProtocolMessage) {
        self.receive_queue.lock().unwrap().push_back(message);
    }

    /// Queue multiple messages to be received
    pub fn queue_receives(&self, messages: Vec<ProtocolMessage>) {
        let mut queue = self.receive_queue.lock().unwrap();
        for msg in messages {
            queue.push_back(msg);
        }
    }

    /// Get all sent messages
    pub fn sent_messages(&self) -> Vec<ProtocolMessage> {
        self.sent_messages.lock().unwrap().clone()
    }

    /// Get the last sent message
    pub fn last_sent(&self) -> Option<ProtocolMessage> {
        self.sent_messages.lock().unwrap().last().cloned()
    }

    /// Clear sent messages
    pub fn clear_sent(&self) {
        self.sent_messages.lock().unwrap().clear();
    }

    /// Get the connection URL
    pub fn connection_url(&self) -> Option<&str> {
        self.connection_url.as_deref()
    }

    /// Set the connected state manually
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for MockTransport {
    async fn connect(&mut self, url: &str) -> Result<()> {
        self.connection_url = Some(url.to_string());
        self.connected = true;
        tracing::debug!(url = %url, "MockTransport connected");
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        tracing::debug!("MockTransport disconnected");
        Ok(())
    }

    async fn send(&self, message: &ProtocolMessage) -> Result<()> {
        if !self.connected {
            return Err(ClimateError::ConnectionError("Not connected".to_string()));
        }

        self.sent_messages.lock().unwrap().push(message.clone());
        tracing::debug!(
            message_type = %message.message_type,
            message_id = %message.message_id,
            "MockTransport sent message"
        );
        Ok(())
    }

    async fn receive(&mut self) -> Result<ProtocolMessage> {
        if !self.connected {
            return Err(ClimateError::ConnectionError("Not connected".to_string()));
        }

        match self.receive_queue.lock().unwrap().pop_front() {
            Some(msg) => {
                tracing::debug!(
                    message_type = %msg.message_type,
                    message_id = %msg.message_id,
                    "MockTransport received message"
                );
                Ok(msg)
            }
            None => Err(ClimateError::ConnectionError(
                "No messages in queue".to_string(),
            )),
        }
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn name(&self) -> &str {
        "mock"
    }
}

/// Mock server for testing client-server interactions
///
/// Simulates a WIA Climate server for testing purposes.
pub struct MockServer {
    transport: MockTransport,
    auto_ack_connect: bool,
}

impl MockServer {
    /// Create a new mock server
    pub fn new() -> Self {
        Self {
            transport: MockTransport::new(),
            auto_ack_connect: true,
        }
    }

    /// Disable automatic connect acknowledgment
    pub fn no_auto_ack(mut self) -> Self {
        self.auto_ack_connect = false;
        self
    }

    /// Queue a connect_ack response
    pub fn queue_connect_ack(&self) {
        use crate::protocol::{ConnectAckPayload, ServerInfo};

        let payload = ConnectAckPayload {
            success: true,
            session_id: Some(uuid::Uuid::new_v4().to_string()),
            server_info: Some(ServerInfo {
                name: "Mock Server".to_string(),
                version: "1.0.0".to_string(),
            }),
            keep_alive_interval: Some(30000),
            error: None,
        };

        let msg = ProtocolMessage::connect_ack(payload).unwrap();
        self.transport.queue_receive(msg);
    }

    /// Queue any protocol message
    pub fn queue_message(&self, message: ProtocolMessage) {
        self.transport.queue_receive(message);
    }

    /// Get messages sent to the server
    pub fn received_messages(&self) -> Vec<ProtocolMessage> {
        self.transport.sent_messages()
    }

    /// Get the mock transport
    pub fn transport(&self) -> &MockTransport {
        &self.transport
    }

    /// Get mutable mock transport
    pub fn transport_mut(&mut self) -> &mut MockTransport {
        &mut self.transport
    }
}

impl Default for MockServer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::MessageType;

    #[tokio::test]
    async fn test_mock_transport_connect() {
        let mut transport = MockTransport::new();

        assert!(!transport.is_connected());

        transport.connect("ws://test.example.com").await.unwrap();

        assert!(transport.is_connected());
        assert_eq!(
            transport.connection_url(),
            Some("ws://test.example.com")
        );
    }

    #[tokio::test]
    async fn test_mock_transport_send_receive() {
        let mut transport = MockTransport::new();
        transport.connect("ws://test").await.unwrap();

        // Queue a message to receive
        let ping = ProtocolMessage::ping();
        transport.queue_receive(ping.clone());

        // Send a message
        let pong = ProtocolMessage::pong();
        transport.send(&pong).await.unwrap();

        // Receive the queued message
        let received = transport.receive().await.unwrap();
        assert_eq!(received.message_type, MessageType::Ping);

        // Check sent messages
        let sent = transport.sent_messages();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_mock_transport_not_connected() {
        let transport = MockTransport::new();

        let result = transport.send(&ProtocolMessage::ping()).await;
        assert!(result.is_err());
    }

    #[test]
    fn test_mock_server() {
        let server = MockServer::new();
        server.queue_connect_ack();

        let transport = server.transport();
        assert_eq!(transport.receive_queue.lock().unwrap().len(), 1);
    }
}
