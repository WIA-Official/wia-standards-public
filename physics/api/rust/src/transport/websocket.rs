//! WebSocket Transport Implementation
//!
//! Primary transport for web clients and general use.

use async_trait::async_trait;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};

use crate::error::{PhysicsError, PhysicsResult};
use crate::protocol::WppMessage;
use super::traits::*;
use super::TransportConfig;

// ============================================================================
// WebSocket Transport
// ============================================================================

/// WebSocket transport implementation
///
/// This is a skeleton implementation. In production, this would use
/// tokio-tungstenite or similar WebSocket library.
///
/// # Example
///
/// ```rust,no_run
/// use wia_physics::transport::*;
///
/// async fn example() -> Result<(), Box<dyn std::error::Error>> {
///     let config = TransportConfig::websocket("wss://physics.wia.live/wpp/v1");
///     let mut transport = WebSocketTransport::new(config);
///
///     // In production, connect would establish actual WebSocket connection
///     // transport.connect().await?;
///
///     Ok(())
/// }
/// ```
pub struct WebSocketTransport {
    config: TransportConfig,
    state: Arc<RwLock<TransportState>>,
    stats: Arc<RwLock<TransportStats>>,
    // In production: WebSocket connection handle
    // ws: Option<WebSocketStream<...>>
    message_queue: Arc<RwLock<Vec<WppMessage>>>,
}

impl WebSocketTransport {
    /// Create a new WebSocket transport
    pub fn new(config: TransportConfig) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(TransportState::Disconnected)),
            stats: Arc::new(RwLock::new(TransportStats::new())),
            message_queue: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Create with URL
    pub fn from_url(url: &str) -> Self {
        Self::new(TransportConfig::websocket(url))
    }

    /// Get transport statistics
    pub async fn stats(&self) -> TransportStats {
        self.stats.read().await.clone()
    }

    /// Get current state
    pub async fn state(&self) -> TransportState {
        *self.state.read().await
    }

    /// Simulate sending (for testing)
    #[cfg(test)]
    pub async fn simulate_receive(&self, message: WppMessage) {
        self.message_queue.write().await.push(message);
    }
}

#[async_trait]
impl Transport for WebSocketTransport {
    async fn connect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Connecting;

        // In production, this would:
        // 1. Parse URL
        // 2. Establish WebSocket connection with TLS if needed
        // 3. Perform WebSocket handshake with WPP subprotocol
        //
        // Example with tokio-tungstenite:
        // let (ws_stream, _) = connect_async(&self.config.url).await?;
        // self.ws = Some(ws_stream);

        *self.state.write().await = TransportState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> PhysicsResult<()> {
        *self.state.write().await = TransportState::Disconnecting;

        // In production, close WebSocket connection gracefully
        // if let Some(ws) = &mut self.ws {
        //     ws.close(None).await?;
        // }

        *self.state.write().await = TransportState::Disconnected;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        // Note: This is a synchronous check, using try_read
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
        let bytes = json.len() as u64;

        // In production, send via WebSocket:
        // self.ws.as_mut().unwrap().send(Message::Text(json)).await?;

        self.stats.write().await.record_sent(bytes);
        Ok(())
    }

    async fn receive(&mut self) -> PhysicsResult<WppMessage> {
        if *self.state.read().await != TransportState::Connected {
            return Err(PhysicsError::protocol("Not connected"));
        }

        // In production, receive from WebSocket:
        // let msg = self.ws.as_mut().unwrap().next().await?;
        // let text = msg.to_text()?;
        // let message = WppMessage::from_json(text)?;

        // For now, check mock queue
        let mut queue = self.message_queue.write().await;
        if let Some(msg) = queue.pop() {
            let json = msg.to_json()?;
            self.stats.write().await.record_received(json.len() as u64);
            return Ok(msg);
        }

        // Block until message available (in production)
        Err(PhysicsError::protocol("No messages available"))
    }

    fn remote_addr(&self) -> Option<String> {
        Some(self.config.url.clone())
    }
}

#[async_trait]
impl StreamTransport for WebSocketTransport {
    async fn start_receiving(&mut self) -> PhysicsResult<mpsc::Receiver<WppMessage>> {
        let (tx, rx) = mpsc::channel(100);

        // In production, spawn a task to receive messages:
        // let ws = self.ws.clone();
        // tokio::spawn(async move {
        //     while let Some(msg) = ws.next().await {
        //         if let Ok(text) = msg.to_text() {
        //             if let Ok(wpp_msg) = WppMessage::from_json(text) {
        //                 let _ = tx.send(wpp_msg).await;
        //             }
        //         }
        //     }
        // });

        // For testing, drain the queue
        let queue = std::mem::take(&mut *self.message_queue.write().await);
        for msg in queue {
            let _ = tx.send(msg).await;
        }

        Ok(rx)
    }

    async fn stop_receiving(&mut self) -> PhysicsResult<()> {
        // In production, cancel the receiving task
        Ok(())
    }
}

// ============================================================================
// WebSocket Factory
// ============================================================================

/// Factory for creating WebSocket transports
pub struct WebSocketTransportFactory;

impl WebSocketTransportFactory {
    pub fn new() -> Self {
        Self
    }
}

impl Default for WebSocketTransportFactory {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl TransportFactory for WebSocketTransportFactory {
    async fn create(&self, url: &str) -> PhysicsResult<Box<dyn Transport>> {
        Ok(Box::new(WebSocketTransport::from_url(url)))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{MessageType, MessagePayload, PingPayload};

    #[tokio::test]
    async fn test_websocket_transport_creation() {
        let config = TransportConfig::websocket("wss://localhost/wpp/v1");
        let transport = WebSocketTransport::new(config);

        assert_eq!(transport.state().await, TransportState::Disconnected);
        assert!(!transport.is_connected());
    }

    #[tokio::test]
    async fn test_websocket_connect_disconnect() {
        let mut transport = WebSocketTransport::from_url("wss://localhost/wpp/v1");

        transport.connect().await.unwrap();
        assert_eq!(transport.state().await, TransportState::Connected);
        assert!(transport.is_connected());

        transport.disconnect().await.unwrap();
        assert_eq!(transport.state().await, TransportState::Disconnected);
    }

    #[tokio::test]
    async fn test_websocket_send() {
        let mut transport = WebSocketTransport::from_url("wss://localhost/wpp/v1");
        transport.connect().await.unwrap();

        let msg = WppMessage::ping();
        transport.send(msg).await.unwrap();

        let stats = transport.stats().await;
        assert_eq!(stats.messages_sent, 1);
    }

    #[tokio::test]
    async fn test_websocket_receive() {
        let mut transport = WebSocketTransport::from_url("wss://localhost/wpp/v1");
        transport.connect().await.unwrap();

        // Simulate receiving a message
        let ping = WppMessage::new(MessageType::Ping, MessagePayload::Ping(PingPayload {}));
        transport.simulate_receive(ping).await;

        let received = transport.receive().await.unwrap();
        assert_eq!(received.msg_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_send_not_connected() {
        let mut transport = WebSocketTransport::from_url("wss://localhost/wpp/v1");

        let msg = WppMessage::ping();
        let result = transport.send(msg).await;

        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_transport_factory() {
        let factory = WebSocketTransportFactory::new();
        let transport = factory.create("wss://localhost/wpp/v1").await.unwrap();

        assert!(transport.remote_addr().is_some());
    }
}
