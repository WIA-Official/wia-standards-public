//! WebSocket transport implementation

use async_trait::async_trait;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::Mutex;
use tokio_tungstenite::{
    connect_async,
    tungstenite::{Error as WsError, Message as WsMessage},
    MaybeTlsStream, WebSocketStream,
};
use tokio::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use super::{Transport, TransportConfig};
use crate::protocol::ProtocolMessage;
use crate::error::{ClimateError, Result};

type WsStream = WebSocketStream<MaybeTlsStream<TcpStream>>;

/// WebSocket transport for WIA Climate protocol
pub struct WebSocketTransport {
    config: TransportConfig,
    stream: Arc<Mutex<Option<WsStream>>>,
    connected: Arc<AtomicBool>,
}

impl WebSocketTransport {
    /// Create a new WebSocket transport with default configuration
    pub fn new() -> Self {
        Self {
            config: TransportConfig::default(),
            stream: Arc::new(Mutex::new(None)),
            connected: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Create a new WebSocket transport with the given configuration
    pub fn with_config(config: TransportConfig) -> Self {
        Self {
            config,
            stream: Arc::new(Mutex::new(None)),
            connected: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Create a new WebSocket transport and connect to the given URL
    pub async fn connect_to(url: &str) -> Result<Self> {
        let mut transport = Self::new();
        transport.connect(url).await?;
        Ok(transport)
    }
}

impl Default for WebSocketTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for WebSocketTransport {
    async fn connect(&mut self, url: &str) -> Result<()> {
        let ws_url = if url.starts_with("ws://") || url.starts_with("wss://") {
            url.to_string()
        } else {
            format!("wss://{}", url)
        };

        tracing::info!(url = %ws_url, "Connecting to WebSocket server");

        let timeout = tokio::time::Duration::from_millis(self.config.connect_timeout_ms);

        let connect_future = connect_async(&ws_url);
        let result = tokio::time::timeout(timeout, connect_future).await;

        match result {
            Ok(Ok((ws_stream, _response))) => {
                let mut stream = self.stream.lock().await;
                *stream = Some(ws_stream);
                self.connected.store(true, Ordering::SeqCst);
                tracing::info!("WebSocket connection established");
                Ok(())
            }
            Ok(Err(e)) => {
                tracing::error!(error = %e, "WebSocket connection failed");
                Err(ClimateError::ConnectionError(format!(
                    "Failed to connect: {}",
                    e
                )))
            }
            Err(_) => {
                tracing::error!("WebSocket connection timeout");
                Err(ClimateError::ConnectionError(
                    "Connection timeout".to_string(),
                ))
            }
        }
    }

    async fn disconnect(&mut self) -> Result<()> {
        let mut stream = self.stream.lock().await;

        if let Some(ws) = stream.take() {
            tracing::info!("Disconnecting from WebSocket server");

            // Try to send close frame
            let (mut write, _read) = ws.split();
            let _ = write.close().await;

            self.connected.store(false, Ordering::SeqCst);
            tracing::info!("WebSocket connection closed");
        }

        Ok(())
    }

    async fn send(&self, message: &ProtocolMessage) -> Result<()> {
        let stream = self.stream.lock().await;

        if let Some(ws) = stream.as_ref() {
            // Clone the stream for sending
            // Note: In production, you'd want to use a split stream
            // This is a simplified implementation
            let json = message.to_json().map_err(|e| {
                ClimateError::SerializationError(format!("Failed to serialize message: {}", e))
            })?;

            tracing::debug!(
                message_type = %message.message_type,
                message_id = %message.message_id,
                "Sending message"
            );

            // We need to get mutable access - this is a limitation of the current design
            // In production, use channels or split the stream
            drop(stream);

            let mut stream = self.stream.lock().await;
            if let Some(ws) = stream.as_mut() {
                ws.send(WsMessage::Text(json.into()))
                    .await
                    .map_err(|e| ClimateError::ConnectionError(format!("Send failed: {}", e)))?;
            } else {
                return Err(ClimateError::ConnectionError("Not connected".to_string()));
            }

            Ok(())
        } else {
            Err(ClimateError::ConnectionError("Not connected".to_string()))
        }
    }

    async fn receive(&mut self) -> Result<ProtocolMessage> {
        let mut stream = self.stream.lock().await;

        if let Some(ws) = stream.as_mut() {
            loop {
                match ws.next().await {
                    Some(Ok(WsMessage::Text(text))) => {
                        let message = ProtocolMessage::from_json(&text).map_err(|e| {
                            ClimateError::SerializationError(format!(
                                "Failed to parse message: {}",
                                e
                            ))
                        })?;

                        tracing::debug!(
                            message_type = %message.message_type,
                            message_id = %message.message_id,
                            "Message received"
                        );

                        return Ok(message);
                    }
                    Some(Ok(WsMessage::Ping(data))) => {
                        tracing::trace!("Received WebSocket ping");
                        let _ = ws.send(WsMessage::Pong(data)).await;
                    }
                    Some(Ok(WsMessage::Pong(_))) => {
                        tracing::trace!("Received WebSocket pong");
                    }
                    Some(Ok(WsMessage::Close(frame))) => {
                        let reason = frame.map(|f| f.reason.to_string());
                        tracing::info!(reason = ?reason, "WebSocket connection closed by server");
                        self.connected.store(false, Ordering::SeqCst);
                        return Err(ClimateError::ConnectionError(
                            reason.unwrap_or_else(|| "Connection closed".to_string()),
                        ));
                    }
                    Some(Ok(WsMessage::Binary(_))) => {
                        tracing::warn!("Received binary message (not supported)");
                    }
                    Some(Ok(WsMessage::Frame(_))) => {
                        // Raw frame - skip
                    }
                    Some(Err(e)) => {
                        tracing::error!(error = %e, "WebSocket receive error");
                        self.connected.store(false, Ordering::SeqCst);
                        return Err(ClimateError::ConnectionError(format!(
                            "Receive error: {}",
                            e
                        )));
                    }
                    None => {
                        tracing::info!("WebSocket stream ended");
                        self.connected.store(false, Ordering::SeqCst);
                        return Err(ClimateError::ConnectionError(
                            "Connection closed".to_string(),
                        ));
                    }
                }
            }
        } else {
            Err(ClimateError::ConnectionError("Not connected".to_string()))
        }
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    fn name(&self) -> &str {
        "websocket"
    }
}

/// WebSocket client for WIA Climate protocol
///
/// Provides a high-level interface for connecting to a WIA Climate server
/// and exchanging messages.
pub struct WebSocketClient {
    transport: WebSocketTransport,
    client_id: String,
    session_id: Option<String>,
}

impl WebSocketClient {
    /// Create a new WebSocket client
    pub fn new(client_id: impl Into<String>) -> Self {
        Self {
            transport: WebSocketTransport::new(),
            client_id: client_id.into(),
            session_id: None,
        }
    }

    /// Create a new WebSocket client with configuration
    pub fn with_config(client_id: impl Into<String>, config: TransportConfig) -> Self {
        Self {
            transport: WebSocketTransport::with_config(config),
            client_id: client_id.into(),
            session_id: None,
        }
    }

    /// Connect to the server and perform protocol handshake
    pub async fn connect(&mut self, url: &str) -> Result<()> {
        // Connect transport
        self.transport.connect(url).await?;

        // Send connect message
        use crate::protocol::{ConnectBuilder, ClientType, MessageType};

        let connect_msg = ConnectBuilder::new()
            .client_id(&self.client_id)
            .client_type(ClientType::Service)
            .build()
            .map_err(|e| ClimateError::Validation(e.to_string()))?;

        self.transport.send(&connect_msg).await?;

        // Wait for connect_ack
        let response = self.transport.receive().await?;

        if response.message_type == MessageType::ConnectAck {
            if let Some(payload) = response.get_payload::<crate::protocol::ConnectAckPayload>() {
                let ack = payload.map_err(|e| ClimateError::SerializationError(e.to_string()))?;
                if ack.success {
                    self.session_id = ack.session_id;
                    tracing::info!(session_id = ?self.session_id, "Connected to server");
                    return Ok(());
                } else if let Some(error) = ack.error {
                    return Err(ClimateError::Validation(error.message));
                }
            }
        }

        Err(ClimateError::ConnectionError(
            "Invalid connect response".to_string(),
        ))
    }

    /// Disconnect from the server
    pub async fn disconnect(&mut self) -> Result<()> {
        use crate::protocol::DisconnectPayload;

        let disconnect_msg = ProtocolMessage::disconnect(Some(DisconnectPayload::normal()))
            .map_err(|e| ClimateError::SerializationError(e.to_string()))?;

        let _ = self.transport.send(&disconnect_msg).await;
        self.transport.disconnect().await?;
        self.session_id = None;

        Ok(())
    }

    /// Send a protocol message
    pub async fn send(&self, message: &ProtocolMessage) -> Result<()> {
        self.transport.send(message).await
    }

    /// Receive a protocol message
    pub async fn receive(&mut self) -> Result<ProtocolMessage> {
        self.transport.receive().await
    }

    /// Send climate data
    pub async fn send_data(&self, data: &crate::ClimateMessage) -> Result<()> {
        let msg = ProtocolMessage::data(data)
            .map_err(|e| ClimateError::SerializationError(e.to_string()))?;
        self.transport.send(&msg).await
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.transport.is_connected()
    }

    /// Get session ID
    pub fn session_id(&self) -> Option<&str> {
        self.session_id.as_deref()
    }

    /// Get client ID
    pub fn client_id(&self) -> &str {
        &self.client_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_websocket_transport_new() {
        let transport = WebSocketTransport::new();
        assert!(!transport.is_connected());
        assert_eq!(transport.name(), "websocket");
    }

    #[test]
    fn test_websocket_client_new() {
        let client = WebSocketClient::new("test-client");
        assert!(!client.is_connected());
        assert_eq!(client.client_id(), "test-client");
        assert!(client.session_id().is_none());
    }
}
