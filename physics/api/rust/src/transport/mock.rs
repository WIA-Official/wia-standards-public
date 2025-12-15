//! Mock Transport Implementation
//!
//! In-memory transport for testing.

use async_trait::async_trait;
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex, RwLock};

use crate::error::{PhysicsError, PhysicsResult};
use crate::protocol::*;
use super::traits::*;

// ============================================================================
// Mock Transport
// ============================================================================

/// Mock transport for testing
///
/// Simulates a WPP server in-memory for testing purposes.
///
/// # Example
///
/// ```rust
/// use wia_physics::transport::*;
/// use wia_physics::protocol::*;
///
/// #[tokio::main]
/// async fn main() -> Result<(), Box<dyn std::error::Error>> {
///     let mut transport = MockTransport::new();
///     transport.connect().await?;
///
///     // Send a message
///     let msg = WppMessage::ping();
///     transport.send(msg.clone()).await?;
///
///     // Mock auto-responds to pings with pongs
///     let response = transport.receive().await?;
///     assert_eq!(response.msg_type, MessageType::Pong);
///
///     Ok(())
/// }
/// ```
pub struct MockTransport {
    state: Arc<RwLock<TransportState>>,
    stats: Arc<RwLock<TransportStats>>,
    /// Messages waiting to be received
    inbox: Arc<Mutex<VecDeque<WppMessage>>>,
    /// Messages that were sent
    outbox: Arc<Mutex<VecDeque<WppMessage>>>,
    /// Session ID for this mock connection
    session_id: String,
    /// Auto-respond to certain messages
    auto_respond: bool,
    /// Available channels
    channels: Vec<ChannelInfo>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            state: Arc::new(RwLock::new(TransportState::Disconnected)),
            stats: Arc::new(RwLock::new(TransportStats::new())),
            inbox: Arc::new(Mutex::new(VecDeque::new())),
            outbox: Arc::new(Mutex::new(VecDeque::new())),
            session_id: uuid::Uuid::new_v4().to_string(),
            auto_respond: true,
            channels: vec![
                ChannelInfo {
                    name: "fusion/iter/plasma".to_string(),
                    channel_type: DataType::Fusion,
                    description: Some("ITER plasma data".to_string()),
                },
                ChannelInfo {
                    name: "particle/lhc/atlas".to_string(),
                    channel_type: DataType::Particle,
                    description: Some("ATLAS detector data".to_string()),
                },
            ],
        }
    }

    /// Create with auto-respond disabled
    pub fn without_auto_respond() -> Self {
        let mut transport = Self::new();
        transport.auto_respond = false;
        transport
    }

    /// Get transport statistics
    pub async fn stats(&self) -> TransportStats {
        self.stats.read().await.clone()
    }

    /// Get current state
    pub async fn state(&self) -> TransportState {
        *self.state.read().await
    }

    /// Inject a message into the inbox
    pub async fn inject_message(&self, message: WppMessage) {
        self.inbox.lock().await.push_back(message);
    }

    /// Get sent messages
    pub async fn sent_messages(&self) -> Vec<WppMessage> {
        self.outbox.lock().await.iter().cloned().collect()
    }

    /// Clear sent messages
    pub async fn clear_sent(&self) {
        self.outbox.lock().await.clear();
    }

    /// Set available channels
    pub fn with_channels(mut self, channels: Vec<ChannelInfo>) -> Self {
        self.channels = channels;
        self
    }

    /// Handle auto-response for common message types
    async fn auto_respond_to(&self, message: &WppMessage) -> Option<WppMessage> {
        if !self.auto_respond {
            return None;
        }

        match message.msg_type {
            MessageType::Connect => {
                Some(WppMessage::new(
                    MessageType::ConnectAck,
                    MessagePayload::ConnectAck(ConnectAckPayload {
                        session_id: self.session_id.clone(),
                        server_name: Some("Mock Server".to_string()),
                        server_version: Some("1.0.0".to_string()),
                        capabilities: vec![
                            Capability::Streaming,
                            Capability::Commands,
                            Capability::Events,
                        ],
                        heartbeat_interval: Some(30000),
                        max_subscriptions: Some(100),
                        channels: self.channels.clone(),
                    }),
                ))
            }
            MessageType::Ping => {
                Some(WppMessage::pong(&message.id, 1))
            }
            MessageType::Subscribe => {
                if let MessagePayload::Subscribe(ref payload) = message.payload {
                    Some(WppMessage::new(
                        MessageType::SubscribeAck,
                        MessagePayload::SubscribeAck(SubscribeAckPayload {
                            channel: payload.channel.clone(),
                            subscription_id: uuid::Uuid::new_v4().to_string(),
                            status: SubscriptionStatus::Active,
                            effective_rate: Some(10.0),
                            history_sent: Some(0),
                        }),
                    ))
                } else {
                    None
                }
            }
            MessageType::Unsubscribe => {
                if let MessagePayload::Unsubscribe(ref payload) = message.payload {
                    Some(WppMessage::new(
                        MessageType::UnsubscribeAck,
                        MessagePayload::UnsubscribeAck(UnsubscribeAckPayload {
                            channel: payload.channel.clone(),
                            subscription_id: payload.subscription_id.clone(),
                            status: UnsubscribeStatus::Success,
                        }),
                    ))
                } else {
                    None
                }
            }
            MessageType::Command => {
                if let MessagePayload::Command(ref _payload) = message.payload {
                    Some(WppMessage::new(
                        MessageType::Response,
                        MessagePayload::Response(ResponsePayload {
                            command_id: message.id.clone(),
                            status: CommandStatus::Success,
                            result: Some(serde_json::json!({"mock": true})),
                            execution_time: Some(10),
                            error: None,
                        }),
                    ))
                } else {
                    None
                }
            }
            MessageType::Disconnect => {
                // No response needed
                None
            }
            _ => None,
        }
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for MockTransport {
    async fn connect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Connecting;
        *self.state.write().await = TransportState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Disconnecting;
        *self.state.write().await = TransportState::Disconnected;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.state
            .try_read()
            .map(|s| *s == TransportState::Connected)
            .unwrap_or(false)
    }

    async fn send(&mut self, message: WppMessage) -> PhysicsResult<()> {
        if *self.state.read().await != TransportState::Connected {
            return Err(PhysicsError::protocol("Not connected"));
        }

        let json = message.to_json()?;
        self.stats.write().await.record_sent(json.len() as u64);

        // Store in outbox
        self.outbox.lock().await.push_back(message.clone());

        // Auto-respond if enabled
        if let Some(response) = self.auto_respond_to(&message).await {
            self.inbox.lock().await.push_back(response);
        }

        Ok(())
    }

    async fn receive(&mut self) -> PhysicsResult<WppMessage> {
        if *self.state.read().await != TransportState::Connected {
            return Err(PhysicsError::protocol("Not connected"));
        }

        let mut inbox = self.inbox.lock().await;
        if let Some(msg) = inbox.pop_front() {
            let json = msg.to_json()?;
            self.stats.write().await.record_received(json.len() as u64);
            return Ok(msg);
        }

        Err(PhysicsError::protocol("No messages available"))
    }

    fn remote_addr(&self) -> Option<String> {
        Some("mock://localhost".to_string())
    }
}

#[async_trait]
impl StreamTransport for MockTransport {
    async fn start_receiving(&mut self) -> PhysicsResult<mpsc::Receiver<WppMessage>> {
        let (tx, rx) = mpsc::channel(100);

        // Drain current inbox
        let inbox = std::mem::take(&mut *self.inbox.lock().await);
        for msg in inbox {
            let _ = tx.send(msg).await;
        }

        Ok(rx)
    }

    async fn stop_receiving(&mut self) -> PhysicsResult<()> {
        Ok(())
    }
}

// ============================================================================
// Mock Server
// ============================================================================

/// Mock server that can generate data
pub struct MockServer {
    transport: Arc<Mutex<MockTransport>>,
}

impl MockServer {
    /// Create a new mock server
    pub fn new() -> Self {
        Self {
            transport: Arc::new(Mutex::new(MockTransport::new())),
        }
    }

    /// Get transport handle
    pub fn transport(&self) -> Arc<Mutex<MockTransport>> {
        self.transport.clone()
    }

    /// Inject fusion data
    pub async fn inject_fusion_data(&self, channel: &str, data: crate::types::FusionData, sequence: u64) -> PhysicsResult<()> {
        let msg = WppMessage::data(channel, DataType::Fusion, &data, sequence)?;
        self.transport.lock().await.inject_message(msg).await;
        Ok(())
    }

    /// Inject an event
    pub async fn inject_event(&self, source: &str, event_type: EventType, severity: Severity) {
        let msg = WppMessage::new(
            MessageType::Event,
            MessagePayload::Event(EventPayload {
                source: source.to_string(),
                event_type,
                severity,
                data: None,
            }),
        );
        self.transport.lock().await.inject_message(msg).await;
    }
}

impl Default for MockServer {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Mock Factory
// ============================================================================

/// Factory for creating mock transports
pub struct MockTransportFactory;

impl MockTransportFactory {
    pub fn new() -> Self {
        Self
    }
}

impl Default for MockTransportFactory {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl TransportFactory for MockTransportFactory {
    async fn create(&self, _url: &str) -> PhysicsResult<Box<dyn Transport>> {
        Ok(Box::new(MockTransport::new()))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_transport_creation() {
        let transport = MockTransport::new();
        assert_eq!(transport.state().await, TransportState::Disconnected);
    }

    #[tokio::test]
    async fn test_mock_connect_disconnect() {
        let mut transport = MockTransport::new();

        transport.connect().await.unwrap();
        assert!(transport.is_connected());

        transport.disconnect().await.unwrap();
        assert!(!transport.is_connected());
    }

    #[tokio::test]
    async fn test_mock_auto_respond_ping() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        // Send ping
        let ping = WppMessage::ping();
        transport.send(ping).await.unwrap();

        // Should auto-receive pong
        let response = transport.receive().await.unwrap();
        assert_eq!(response.msg_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_mock_auto_respond_connect() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        // Send connect
        let connect = WppMessage::connect("test-client", "Test");
        transport.send(connect).await.unwrap();

        // Should auto-receive connect_ack
        let response = transport.receive().await.unwrap();
        assert_eq!(response.msg_type, MessageType::ConnectAck);
    }

    #[tokio::test]
    async fn test_mock_auto_respond_subscribe() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        // Send subscribe
        let subscribe = WppMessage::subscribe("fusion/iter/plasma");
        transport.send(subscribe).await.unwrap();

        // Should auto-receive subscribe_ack
        let response = transport.receive().await.unwrap();
        assert_eq!(response.msg_type, MessageType::SubscribeAck);
    }

    #[tokio::test]
    async fn test_mock_inject_message() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        // Inject a message
        let msg = WppMessage::ping();
        transport.inject_message(msg).await;

        // Receive it
        let received = transport.receive().await.unwrap();
        assert_eq!(received.msg_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_mock_sent_messages() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        let ping = WppMessage::ping();
        transport.send(ping).await.unwrap();

        let sent = transport.sent_messages().await;
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].msg_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_mock_without_auto_respond() {
        let mut transport = MockTransport::without_auto_respond();
        transport.connect().await.unwrap();

        let ping = WppMessage::ping();
        transport.send(ping).await.unwrap();

        // No auto-response, so receive should fail
        let result = transport.receive().await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mock_stats() {
        let mut transport = MockTransport::new();
        transport.connect().await.unwrap();

        transport.send(WppMessage::ping()).await.unwrap();
        transport.receive().await.unwrap(); // pong

        let stats = transport.stats().await;
        assert_eq!(stats.messages_sent, 1);
        assert_eq!(stats.messages_received, 1);
    }
}
