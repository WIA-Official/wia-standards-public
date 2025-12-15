//! WebSocket Transport Implementation
//!
//! WebSocket-based transport for the WIA Health protocol

use std::sync::Arc;

use async_trait::async_trait;
use futures_util::{SinkExt, StreamExt};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, RwLock};
use tokio::time::{timeout, Duration};
use tokio_tungstenite::{
    connect_async, tungstenite::protocol::Message as WsMessage, MaybeTlsStream, WebSocketStream,
};

use super::{Transport, TransportConfig, TransportEvent, TransportStatus};
use crate::error::{HealthError, Result};
use crate::protocol::Message;

/// WebSocket transport implementation
pub struct WebSocketTransport {
    config: TransportConfig,
    status: Arc<RwLock<TransportStatus>>,
    write_tx: Option<mpsc::Sender<WsMessage>>,
    event_tx: mpsc::Sender<TransportEvent>,
}

impl WebSocketTransport {
    /// Create a new WebSocket transport
    pub fn new(config: TransportConfig, event_tx: mpsc::Sender<TransportEvent>) -> Self {
        Self {
            config,
            status: Arc::new(RwLock::new(TransportStatus::Disconnected)),
            write_tx: None,
            event_tx,
        }
    }

    /// Create with default config and event channel
    pub fn with_defaults(url: &str, event_tx: mpsc::Sender<TransportEvent>) -> Self {
        Self::new(TransportConfig::with_url(url), event_tx)
    }

    /// Handle reconnection logic
    async fn reconnect(&mut self) -> Result<()> {
        if !self.config.auto_reconnect {
            return Err(HealthError::ConnectionError(
                "Auto-reconnect disabled".into(),
            ));
        }

        let mut attempts = 0;
        let mut delay = self.config.reconnect_delay_ms;

        while attempts < self.config.max_reconnect_attempts {
            attempts += 1;

            tokio::time::sleep(Duration::from_millis(delay)).await;

            match self.connect(&self.config.url.clone()).await {
                Ok(()) => return Ok(()),
                Err(_) => {
                    // Exponential backoff
                    delay = (delay * 2).min(30000);
                }
            }
        }

        Err(HealthError::ConnectionError(format!(
            "Failed to reconnect after {} attempts",
            attempts
        )))
    }

    /// Set transport status
    async fn set_status(&self, status: TransportStatus) {
        *self.status.write().await = status;
    }

    /// Start the read loop
    fn spawn_read_loop(
        mut read: futures_util::stream::SplitStream<WebSocketStream<MaybeTlsStream<TcpStream>>>,
        event_tx: mpsc::Sender<TransportEvent>,
        status: Arc<RwLock<TransportStatus>>,
    ) {
        tokio::spawn(async move {
            while let Some(msg_result) = read.next().await {
                match msg_result {
                    Ok(ws_msg) => {
                        match ws_msg {
                            WsMessage::Text(text) => {
                                match Message::from_json(&text) {
                                    Ok(message) => {
                                        let _ = event_tx.send(TransportEvent::Message(message)).await;
                                    }
                                    Err(e) => {
                                        let _ = event_tx
                                            .send(TransportEvent::Error(format!(
                                                "Parse error: {}",
                                                e
                                            )))
                                            .await;
                                    }
                                }
                            }
                            WsMessage::Binary(data) => {
                                // Try to parse binary as JSON
                                if let Ok(text) = String::from_utf8(data) {
                                    if let Ok(message) = Message::from_json(&text) {
                                        let _ = event_tx.send(TransportEvent::Message(message)).await;
                                    }
                                }
                            }
                            WsMessage::Close(frame) => {
                                let reason = frame
                                    .map(|f| f.reason.to_string())
                                    .unwrap_or_else(|| "Connection closed".to_string());
                                *status.write().await = TransportStatus::Disconnected;
                                let _ = event_tx
                                    .send(TransportEvent::Disconnected { reason })
                                    .await;
                                break;
                            }
                            WsMessage::Ping(_) | WsMessage::Pong(_) => {
                                // Handled automatically by tungstenite
                            }
                            WsMessage::Frame(_) => {
                                // Raw frames - ignore
                            }
                        }
                    }
                    Err(e) => {
                        *status.write().await = TransportStatus::Error;
                        let _ = event_tx
                            .send(TransportEvent::Error(e.to_string()))
                            .await;
                        break;
                    }
                }
            }
        });
    }

    /// Start the write loop
    fn spawn_write_loop(
        mut write: futures_util::stream::SplitSink<
            WebSocketStream<MaybeTlsStream<TcpStream>>,
            WsMessage,
        >,
        mut write_rx: mpsc::Receiver<WsMessage>,
        status: Arc<RwLock<TransportStatus>>,
        event_tx: mpsc::Sender<TransportEvent>,
    ) {
        tokio::spawn(async move {
            while let Some(msg) = write_rx.recv().await {
                if let Err(e) = write.send(msg).await {
                    *status.write().await = TransportStatus::Error;
                    let _ = event_tx.send(TransportEvent::Error(e.to_string())).await;
                    break;
                }
            }
        });
    }
}

#[async_trait]
impl Transport for WebSocketTransport {
    async fn connect(&mut self, url: &str) -> Result<()> {
        self.set_status(TransportStatus::Connecting).await;

        // Connect with timeout
        let connect_result = timeout(
            Duration::from_millis(self.config.connect_timeout_ms),
            connect_async(url),
        )
        .await;

        let (ws_stream, _response) = match connect_result {
            Ok(Ok((stream, response))) => (stream, response),
            Ok(Err(e)) => {
                self.set_status(TransportStatus::Error).await;
                return Err(HealthError::ConnectionError(e.to_string()));
            }
            Err(_) => {
                self.set_status(TransportStatus::Error).await;
                return Err(HealthError::ConnectionError("Connection timeout".into()));
            }
        };

        // Split the stream
        let (write, read) = ws_stream.split();

        // Create write channel
        let (write_tx, write_rx) = mpsc::channel::<WsMessage>(100);
        self.write_tx = Some(write_tx);

        // Spawn read and write loops
        Self::spawn_read_loop(read, self.event_tx.clone(), self.status.clone());
        Self::spawn_write_loop(write, write_rx, self.status.clone(), self.event_tx.clone());

        self.set_status(TransportStatus::Connected).await;

        // Notify connected
        let _ = self.event_tx.send(TransportEvent::Connected).await;

        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        if let Some(ref tx) = self.write_tx {
            let _ = tx.send(WsMessage::Close(None)).await;
        }

        self.set_status(TransportStatus::Disconnected).await;
        self.write_tx = None;

        let _ = self
            .event_tx
            .send(TransportEvent::Disconnected {
                reason: "Client disconnect".to_string(),
            })
            .await;

        Ok(())
    }

    async fn send(&self, message: Message) -> Result<()> {
        let tx = self
            .write_tx
            .as_ref()
            .ok_or_else(|| HealthError::ConnectionError("Not connected".into()))?;

        let json = message
            .to_json()
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        tx.send(WsMessage::Text(json))
            .await
            .map_err(|_| HealthError::ConnectionError("Send channel closed".into()))
    }

    fn status(&self) -> TransportStatus {
        // Use try_read to avoid blocking
        self.status
            .try_read()
            .map(|s| *s)
            .unwrap_or(TransportStatus::Disconnected)
    }
}

/// WebSocket client builder
pub struct WebSocketClientBuilder {
    config: TransportConfig,
}

impl WebSocketClientBuilder {
    /// Create new builder with URL
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            config: TransportConfig::with_url(url),
        }
    }

    /// Set auto-reconnect
    pub fn auto_reconnect(mut self, enabled: bool) -> Self {
        self.config.auto_reconnect = enabled;
        self
    }

    /// Set reconnect delay
    pub fn reconnect_delay(mut self, ms: u64) -> Self {
        self.config.reconnect_delay_ms = ms;
        self
    }

    /// Set max reconnect attempts
    pub fn max_reconnect_attempts(mut self, attempts: u32) -> Self {
        self.config.max_reconnect_attempts = attempts;
        self
    }

    /// Set connection timeout
    pub fn timeout(mut self, ms: u64) -> Self {
        self.config.connect_timeout_ms = ms;
        self
    }

    /// Build the transport
    pub fn build(self, event_tx: mpsc::Sender<TransportEvent>) -> WebSocketTransport {
        WebSocketTransport::new(self.config, event_tx)
    }
}

/// High-level WebSocket client for WIA Health protocol
pub struct WiaHealthClient {
    transport: WebSocketTransport,
    event_rx: mpsc::Receiver<TransportEvent>,
}

impl WiaHealthClient {
    /// Create a new client with URL
    pub fn new(url: &str) -> Self {
        let (event_tx, event_rx) = mpsc::channel(100);
        let transport = WebSocketTransport::with_defaults(url, event_tx);
        Self {
            transport,
            event_rx,
        }
    }

    /// Create client with custom config
    pub fn with_config(config: TransportConfig) -> Self {
        let (event_tx, event_rx) = mpsc::channel(100);
        let transport = WebSocketTransport::new(config, event_tx);
        Self {
            transport,
            event_rx,
        }
    }

    /// Connect to the server
    pub async fn connect(&mut self) -> Result<()> {
        let url = self.transport.config.url.clone();
        self.transport.connect(&url).await
    }

    /// Disconnect from the server
    pub async fn disconnect(&mut self) -> Result<()> {
        self.transport.disconnect().await
    }

    /// Send a message
    pub async fn send(&self, message: Message) -> Result<()> {
        self.transport.send(message).await
    }

    /// Receive the next event
    pub async fn recv(&mut self) -> Option<TransportEvent> {
        self.event_rx.recv().await
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.transport.is_connected()
    }

    /// Get transport status
    pub fn status(&self) -> TransportStatus {
        self.transport.status()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_config() {
        let config = TransportConfig::with_url("wss://example.com/ws")
            .no_reconnect()
            .timeout(5000);

        assert_eq!(config.url, "wss://example.com/ws");
        assert!(!config.auto_reconnect);
        assert_eq!(config.connect_timeout_ms, 5000);
    }

    #[test]
    fn test_client_builder() {
        let (tx, _rx) = mpsc::channel(100);
        let transport = WebSocketClientBuilder::new("wss://example.com/ws")
            .auto_reconnect(false)
            .timeout(3000)
            .max_reconnect_attempts(3)
            .build(tx);

        assert_eq!(transport.config.url, "wss://example.com/ws");
        assert!(!transport.config.auto_reconnect);
        assert_eq!(transport.config.connect_timeout_ms, 3000);
        assert_eq!(transport.config.max_reconnect_attempts, 3);
    }
}
