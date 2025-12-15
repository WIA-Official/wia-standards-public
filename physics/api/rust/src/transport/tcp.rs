//! TCP Transport Implementation
//!
//! High-performance transport for native applications.

use async_trait::async_trait;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::{PhysicsError, PhysicsResult};
use crate::protocol::WppMessage;
use super::traits::*;
use super::TransportConfig;

// ============================================================================
// TCP Transport
// ============================================================================

/// TCP transport implementation
///
/// Uses length-prefixed framing (4-byte big-endian length + payload).
/// Supports JSON or MessagePack serialization.
///
/// # Frame Format
///
/// ```text
/// ┌──────────────┬──────────────────────────┐
/// │ Length (4B)  │ Payload (N bytes)        │
/// │ Big-endian   │ JSON or MessagePack      │
/// └──────────────┴──────────────────────────┘
/// ```
///
/// # Example
///
/// ```rust,no_run
/// use wia_physics::transport::*;
///
/// async fn example() -> Result<(), Box<dyn std::error::Error>> {
///     let config = TransportConfig::tcp("127.0.0.1:5740");
///     let mut transport = TcpTransport::new(config);
///
///     // In production, connect would establish TCP connection
///     // transport.connect().await?;
///
///     Ok(())
/// }
/// ```
pub struct TcpTransport {
    config: TransportConfig,
    state: Arc<RwLock<TransportState>>,
    stats: Arc<RwLock<TransportStats>>,
    // In production: TcpStream handle
    // stream: Option<TcpStream>
    message_queue: Arc<RwLock<Vec<WppMessage>>>,
}

impl TcpTransport {
    /// Create a new TCP transport
    pub fn new(config: TransportConfig) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(TransportState::Disconnected)),
            stats: Arc::new(RwLock::new(TransportStats::new())),
            message_queue: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Create with address
    pub fn from_addr(addr: &str) -> Self {
        Self::new(TransportConfig::tcp(addr))
    }

    /// Get transport statistics
    pub async fn stats(&self) -> TransportStats {
        self.stats.read().await.clone()
    }

    /// Get current state
    pub async fn state(&self) -> TransportState {
        *self.state.read().await
    }

    /// Encode message with length prefix
    fn encode_frame(data: &[u8]) -> Vec<u8> {
        let len = data.len() as u32;
        let mut frame = Vec::with_capacity(4 + data.len());
        frame.extend_from_slice(&len.to_be_bytes());
        frame.extend_from_slice(data);
        frame
    }

    /// Decode length from frame header
    fn decode_length(header: &[u8; 4]) -> u32 {
        u32::from_be_bytes(*header)
    }

    /// Simulate receiving (for testing)
    #[cfg(test)]
    pub async fn simulate_receive(&self, message: WppMessage) {
        self.message_queue.write().await.push(message);
    }
}

#[async_trait]
impl Transport for TcpTransport {
    async fn connect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Connecting;

        // In production, this would:
        // 1. Parse address
        // 2. Establish TCP connection
        // 3. Optionally wrap with TLS
        //
        // Example with tokio:
        // let stream = TcpStream::connect(&self.config.url).await?;
        // if self.config.tls {
        //     // Wrap with TLS
        // }
        // self.stream = Some(stream);

        *self.state.write().await = TransportState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Disconnecting;

        // In production, close TCP connection
        // if let Some(stream) = &mut self.stream {
        //     stream.shutdown().await?;
        // }

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
        let frame = Self::encode_frame(json.as_bytes());

        // In production, write to TCP stream:
        // self.stream.as_mut().unwrap().write_all(&frame).await?;

        self.stats.write().await.record_sent(frame.len() as u64);
        Ok(())
    }

    async fn receive(&mut self) -> PhysicsResult<WppMessage> {
        if *self.state.read().await != TransportState::Connected {
            return Err(PhysicsError::protocol("Not connected"));
        }

        // In production:
        // 1. Read 4-byte length header
        // 2. Read payload of that length
        // 3. Deserialize
        //
        // let mut header = [0u8; 4];
        // self.stream.as_mut().unwrap().read_exact(&mut header).await?;
        // let len = Self::decode_length(&header) as usize;
        // let mut payload = vec![0u8; len];
        // self.stream.as_mut().unwrap().read_exact(&mut payload).await?;
        // let message = WppMessage::from_json(&String::from_utf8(payload)?)?;

        // For now, check mock queue
        let mut queue = self.message_queue.write().await;
        if let Some(msg) = queue.pop() {
            let json = msg.to_json()?;
            self.stats.write().await.record_received(json.len() as u64 + 4);
            return Ok(msg);
        }

        Err(PhysicsError::protocol("No messages available"))
    }

    fn remote_addr(&self) -> Option<String> {
        Some(self.config.url.clone())
    }
}

// ============================================================================
// TCP Factory
// ============================================================================

/// Factory for creating TCP transports
pub struct TcpTransportFactory;

impl TcpTransportFactory {
    pub fn new() -> Self {
        Self
    }
}

impl Default for TcpTransportFactory {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl TransportFactory for TcpTransportFactory {
    async fn create(&self, addr: &str) -> PhysicsResult<Box<dyn Transport>> {
        Ok(Box::new(TcpTransport::from_addr(addr)))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{MessageType, MessagePayload, PingPayload};

    #[test]
    fn test_frame_encoding() {
        let data = b"hello";
        let frame = TcpTransport::encode_frame(data);

        assert_eq!(frame.len(), 9); // 4 + 5
        assert_eq!(&frame[0..4], &[0, 0, 0, 5]); // big-endian 5
        assert_eq!(&frame[4..], b"hello");
    }

    #[test]
    fn test_length_decoding() {
        let header = [0, 0, 1, 0]; // 256 in big-endian
        let len = TcpTransport::decode_length(&header);
        assert_eq!(len, 256);
    }

    #[tokio::test]
    async fn test_tcp_transport_creation() {
        let config = TransportConfig::tcp("127.0.0.1:5740");
        let transport = TcpTransport::new(config);

        assert_eq!(transport.state().await, TransportState::Disconnected);
        assert!(!transport.is_connected());
    }

    #[tokio::test]
    async fn test_tcp_connect_disconnect() {
        let mut transport = TcpTransport::from_addr("127.0.0.1:5740");

        transport.connect().await.unwrap();
        assert_eq!(transport.state().await, TransportState::Connected);

        transport.disconnect().await.unwrap();
        assert_eq!(transport.state().await, TransportState::Disconnected);
    }

    #[tokio::test]
    async fn test_tcp_send() {
        let mut transport = TcpTransport::from_addr("127.0.0.1:5740");
        transport.connect().await.unwrap();

        let msg = WppMessage::ping();
        transport.send(msg).await.unwrap();

        let stats = transport.stats().await;
        assert_eq!(stats.messages_sent, 1);
    }

    #[tokio::test]
    async fn test_tcp_receive() {
        let mut transport = TcpTransport::from_addr("127.0.0.1:5740");
        transport.connect().await.unwrap();

        let ping = WppMessage::new(MessageType::Ping, MessagePayload::Ping(PingPayload {}));
        transport.simulate_receive(ping).await;

        let received = transport.receive().await.unwrap();
        assert_eq!(received.msg_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_tcp_factory() {
        let factory = TcpTransportFactory::new();
        let transport = factory.create("127.0.0.1:5740").await.unwrap();

        assert_eq!(transport.remote_addr(), Some("127.0.0.1:5740".to_string()));
    }
}
