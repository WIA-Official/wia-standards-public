//! Mock transport for testing

use crate::error::{RobotError, RobotResult};
use crate::protocol::WrpMessage;
use crate::transport::base::*;
use std::collections::VecDeque;

/// Mock transport for testing WRP communication
pub struct MockTransport {
    connected: bool,
    send_queue: VecDeque<WrpMessage>,
    receive_queue: VecDeque<WrpMessage>,
    latency_ms: u64,
    stats: TransportStats,
    fail_next_send: bool,
    fail_next_receive: bool,
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            connected: false,
            send_queue: VecDeque::new(),
            receive_queue: VecDeque::new(),
            latency_ms: 0,
            stats: TransportStats::default(),
            fail_next_send: false,
            fail_next_receive: false,
        }
    }

    /// Create a mock transport with simulated latency
    pub fn with_latency(latency_ms: u64) -> Self {
        let mut transport = Self::new();
        transport.latency_ms = latency_ms;
        transport
    }

    /// Add a message to the receive queue (for testing incoming messages)
    pub fn enqueue_receive(&mut self, message: WrpMessage) {
        self.receive_queue.push_back(message);
    }

    /// Get next message from send queue (for testing outgoing messages)
    pub fn dequeue_sent(&mut self) -> Option<WrpMessage> {
        self.send_queue.pop_front()
    }

    /// Get all sent messages
    pub fn get_sent_messages(&self) -> Vec<WrpMessage> {
        self.send_queue.iter().cloned().collect()
    }

    /// Clear all queues
    pub fn clear(&mut self) {
        self.send_queue.clear();
        self.receive_queue.clear();
    }

    /// Set to fail next send operation
    pub fn fail_next_send(&mut self) {
        self.fail_next_send = true;
    }

    /// Set to fail next receive operation
    pub fn fail_next_receive(&mut self) {
        self.fail_next_receive = true;
    }

    /// Get number of messages in receive queue
    pub fn receive_queue_len(&self) -> usize {
        self.receive_queue.len()
    }

    /// Get number of messages in send queue
    pub fn send_queue_len(&self) -> usize {
        self.send_queue.len()
    }

    /// Simulate a loopback (send queue goes to receive queue)
    pub fn loopback(&mut self) {
        while let Some(msg) = self.send_queue.pop_front() {
            self.receive_queue.push_back(msg);
        }
    }
}

impl Transport for MockTransport {
    fn transport_type(&self) -> TransportType {
        TransportType::Mock
    }

    fn connect(&mut self, _config: &TransportConfig) -> RobotResult<()> {
        self.connected = true;
        Ok(())
    }

    fn disconnect(&mut self) -> RobotResult<()> {
        self.connected = false;
        self.clear();
        Ok(())
    }

    fn send(&mut self, message: &WrpMessage) -> RobotResult<()> {
        if !self.connected {
            return Err(RobotError::CommunicationError(
                "Transport not connected".into(),
            ));
        }

        if self.fail_next_send {
            self.fail_next_send = false;
            self.stats.record_error();
            return Err(RobotError::CommunicationError(
                "Simulated send failure".into(),
            ));
        }

        // Simulate latency
        if self.latency_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(self.latency_ms));
        }

        let json = message.to_json().map_err(|e| {
            RobotError::CommunicationError(format!("Serialization error: {}", e))
        })?;

        self.stats.record_send(json.len() as u64);
        self.stats.update_latency(self.latency_ms as f64);
        self.send_queue.push_back(message.clone());

        Ok(())
    }

    fn receive(&mut self) -> RobotResult<WrpMessage> {
        if !self.connected {
            return Err(RobotError::CommunicationError(
                "Transport not connected".into(),
            ));
        }

        if self.fail_next_receive {
            self.fail_next_receive = false;
            self.stats.record_error();
            return Err(RobotError::CommunicationError(
                "Simulated receive failure".into(),
            ));
        }

        // Simulate latency
        if self.latency_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(self.latency_ms));
        }

        self.receive_queue.pop_front().map(|msg| {
            let json = msg.to_json().unwrap_or_default();
            self.stats.record_receive(json.len() as u64);
            msg
        }).ok_or_else(|| {
            RobotError::CommunicationError("No message available".into())
        })
    }

    fn try_receive(&mut self) -> RobotResult<Option<WrpMessage>> {
        if !self.connected {
            return Err(RobotError::CommunicationError(
                "Transport not connected".into(),
            ));
        }

        Ok(self.receive_queue.pop_front().map(|msg| {
            let json = msg.to_json().unwrap_or_default();
            self.stats.record_receive(json.len() as u64);
            msg
        }))
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn latency_ms(&self) -> u64 {
        self.latency_ms
    }

    fn stats(&self) -> TransportStats {
        self.stats.clone()
    }
}

/// Create a pair of connected mock transports for testing
pub fn create_mock_pair() -> (MockTransport, MockTransport) {
    let mut client = MockTransport::new();
    let mut server = MockTransport::new();

    client.connected = true;
    server.connected = true;

    (client, server)
}

/// Pipe messages between two mock transports
pub fn pipe_messages(from: &mut MockTransport, to: &mut MockTransport) {
    while let Some(msg) = from.dequeue_sent() {
        to.enqueue_receive(msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::builder::MessageBuilder;

    #[test]
    fn test_mock_transport_connect() {
        let mut transport = MockTransport::new();
        assert!(!transport.is_connected());

        transport.connect(&TransportConfig::default()).unwrap();
        assert!(transport.is_connected());

        transport.disconnect().unwrap();
        assert!(!transport.is_connected());
    }

    #[test]
    fn test_mock_transport_send_receive() {
        let mut transport = MockTransport::new();
        transport.connect(&TransportConfig::default()).unwrap();

        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("server", "server")
            .build();

        transport.send(&msg).unwrap();
        assert_eq!(transport.send_queue_len(), 1);

        // Move to receive queue
        transport.loopback();
        assert_eq!(transport.send_queue_len(), 0);
        assert_eq!(transport.receive_queue_len(), 1);

        let received = transport.receive().unwrap();
        assert_eq!(received.message_id, msg.message_id);
    }

    #[test]
    fn test_mock_transport_not_connected() {
        let mut transport = MockTransport::new();

        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("server", "server")
            .build();

        assert!(transport.send(&msg).is_err());
        assert!(transport.receive().is_err());
    }

    #[test]
    fn test_mock_transport_failure_simulation() {
        let mut transport = MockTransport::new();
        transport.connect(&TransportConfig::default()).unwrap();

        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("server", "server")
            .build();

        transport.fail_next_send();
        assert!(transport.send(&msg).is_err());

        // Should succeed after failure
        assert!(transport.send(&msg).is_ok());
    }

    #[test]
    fn test_mock_pair() {
        let (mut client, mut server) = create_mock_pair();

        let msg = MessageBuilder::control()
            .from_device("server", "server")
            .to_device("robot", "exoskeleton")
            .payload(serde_json::json!({"command": "stop"}))
            .build();

        client.send(&msg).unwrap();
        pipe_messages(&mut client, &mut server);

        let received = server.receive().unwrap();
        assert_eq!(received.message_type, msg.message_type);
    }

    #[test]
    fn test_transport_stats() {
        let mut transport = MockTransport::new();
        transport.connect(&TransportConfig::default()).unwrap();

        let msg = MessageBuilder::telemetry()
            .from_device("test", "test")
            .to_device("server", "server")
            .build();

        transport.send(&msg).unwrap();
        transport.loopback();
        transport.receive().unwrap();

        let stats = transport.stats();
        assert_eq!(stats.messages_sent, 1);
        assert_eq!(stats.messages_received, 1);
    }
}
