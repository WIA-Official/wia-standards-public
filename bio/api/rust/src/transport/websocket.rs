//! WebSocket transport implementation

use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex};

use super::base::*;
use crate::protocol::{BioMessage, ProtocolError};

/// WebSocket transport for WIA Bio protocol
#[derive(Debug)]
pub struct WebSocketTransport {
    /// Connection state
    state: TransportState,
    /// Configuration
    config: Option<TransportConfig>,
    /// Message sender channel
    tx: Option<mpsc::Sender<String>>,
    /// Message receiver channel
    rx: Option<mpsc::Receiver<String>>,
    /// Connected flag
    connected: Arc<AtomicBool>,
    /// Metrics
    metrics: TransportMetrics,
}

impl WebSocketTransport {
    /// Create a new WebSocket transport
    pub fn new() -> Self {
        Self {
            state: TransportState::Disconnected,
            config: None,
            tx: None,
            rx: None,
            connected: Arc::new(AtomicBool::new(false)),
            metrics: TransportMetrics::default(),
        }
    }

    /// Get metrics
    pub fn metrics(&self) -> &TransportMetrics {
        &self.metrics
    }

    /// Parse message from JSON
    fn parse_message(json: &str) -> Result<BioMessage, TransportError> {
        serde_json::from_str(json)
            .map_err(|e| TransportError::ReceiveFailed(format!("Parse error: {}", e)))
    }

    /// Serialize message to JSON
    fn serialize_message(message: &BioMessage) -> Result<String, TransportError> {
        serde_json::to_string(message)
            .map_err(|e| TransportError::SendFailed(format!("Serialize error: {}", e)))
    }
}

impl Default for WebSocketTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl ITransport for WebSocketTransport {
    async fn connect(&mut self, config: &TransportConfig) -> Result<(), TransportError> {
        if self.state == TransportState::Connected {
            return Err(TransportError::AlreadyConnected);
        }

        self.state = TransportState::Connecting;
        self.config = Some(config.clone());

        // In a real implementation, this would establish an actual WebSocket connection
        // For now, we create channels for message passing
        let (tx, rx) = mpsc::channel(config.buffer_size);

        // Simulate connection
        // In production, use tokio-tungstenite or similar:
        // let (ws_stream, _) = tokio_tungstenite::connect_async(&config.url).await?;

        self.tx = Some(tx);
        self.rx = Some(rx);
        self.connected.store(true, Ordering::SeqCst);
        self.state = TransportState::Connected;

        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), TransportError> {
        if self.state != TransportState::Connected {
            return Err(TransportError::NotConnected);
        }

        self.connected.store(false, Ordering::SeqCst);
        self.tx = None;
        self.rx = None;
        self.state = TransportState::Disconnected;

        Ok(())
    }

    async fn send(&mut self, message: &BioMessage) -> Result<(), TransportError> {
        if !self.is_connected() {
            return Err(TransportError::NotConnected);
        }

        let json = Self::serialize_message(message)?;
        let bytes_len = json.len() as u64;

        if let Some(tx) = &self.tx {
            tx.send(json)
                .await
                .map_err(|e| TransportError::SendFailed(e.to_string()))?;

            self.metrics.messages_sent += 1;
            self.metrics.bytes_sent += bytes_len;
        }

        Ok(())
    }

    async fn receive(&mut self) -> Result<BioMessage, TransportError> {
        if !self.is_connected() {
            return Err(TransportError::NotConnected);
        }

        if let Some(rx) = &mut self.rx {
            match rx.recv().await {
                Some(json) => {
                    self.metrics.messages_received += 1;
                    self.metrics.bytes_received += json.len() as u64;
                    Self::parse_message(&json)
                }
                None => Err(TransportError::ConnectionClosed("Channel closed".to_string())),
            }
        } else {
            Err(TransportError::NotConnected)
        }
    }

    async fn try_receive(&mut self) -> Result<Option<BioMessage>, TransportError> {
        if !self.is_connected() {
            return Err(TransportError::NotConnected);
        }

        if let Some(rx) = &mut self.rx {
            match rx.try_recv() {
                Ok(json) => {
                    self.metrics.messages_received += 1;
                    self.metrics.bytes_received += json.len() as u64;
                    Ok(Some(Self::parse_message(&json)?))
                }
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => {
                    Err(TransportError::ConnectionClosed("Channel disconnected".to_string()))
                }
            }
        } else {
            Err(TransportError::NotConnected)
        }
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    fn state(&self) -> TransportState {
        self.state
    }

    fn transport_type(&self) -> &str {
        "websocket"
    }
}

/// WebSocket client for simplified usage
pub struct WebSocketClient {
    transport: Arc<Mutex<WebSocketTransport>>,
    config: TransportConfig,
}

impl WebSocketClient {
    /// Create a new WebSocket client
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            transport: Arc::new(Mutex::new(WebSocketTransport::new())),
            config: TransportConfig::with_url(url),
        }
    }

    /// Create with custom config
    pub fn with_config(config: TransportConfig) -> Self {
        Self {
            transport: Arc::new(Mutex::new(WebSocketTransport::new())),
            config,
        }
    }

    /// Connect to the server
    pub async fn connect(&self) -> Result<(), TransportError> {
        let mut transport = self.transport.lock().await;
        transport.connect(&self.config).await
    }

    /// Disconnect from the server
    pub async fn disconnect(&self) -> Result<(), TransportError> {
        let mut transport = self.transport.lock().await;
        transport.disconnect().await
    }

    /// Send a message
    pub async fn send(&self, message: &BioMessage) -> Result<(), TransportError> {
        let mut transport = self.transport.lock().await;
        transport.send(message).await
    }

    /// Receive a message
    pub async fn receive(&self) -> Result<BioMessage, TransportError> {
        let mut transport = self.transport.lock().await;
        transport.receive().await
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        let transport = self.transport.lock().await;
        transport.is_connected()
    }

    /// Get metrics
    pub async fn metrics(&self) -> TransportMetrics {
        let transport = self.transport.lock().await;
        transport.metrics().clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{MessageBuilder, MessageType};

    #[tokio::test]
    async fn test_websocket_transport_new() {
        let transport = WebSocketTransport::new();
        assert_eq!(transport.state(), TransportState::Disconnected);
        assert!(!transport.is_connected());
    }

    #[tokio::test]
    async fn test_websocket_connect_disconnect() {
        let mut transport = WebSocketTransport::new();
        let config = TransportConfig::with_url("wss://test.example.com/ws");

        // Connect
        transport.connect(&config).await.unwrap();
        assert!(transport.is_connected());
        assert_eq!(transport.state(), TransportState::Connected);

        // Disconnect
        transport.disconnect().await.unwrap();
        assert!(!transport.is_connected());
        assert_eq!(transport.state(), TransportState::Disconnected);
    }

    #[tokio::test]
    async fn test_websocket_double_connect() {
        let mut transport = WebSocketTransport::new();
        let config = TransportConfig::with_url("wss://test.example.com/ws");

        transport.connect(&config).await.unwrap();
        let result = transport.connect(&config).await;
        assert!(matches!(result, Err(TransportError::AlreadyConnected)));
    }

    #[tokio::test]
    async fn test_websocket_client() {
        let client = WebSocketClient::new("wss://test.example.com/ws");

        client.connect().await.unwrap();
        assert!(client.is_connected().await);

        client.disconnect().await.unwrap();
        assert!(!client.is_connected().await);
    }

    #[tokio::test]
    async fn test_transport_type() {
        let transport = WebSocketTransport::new();
        assert_eq!(transport.transport_type(), "websocket");
    }
}
