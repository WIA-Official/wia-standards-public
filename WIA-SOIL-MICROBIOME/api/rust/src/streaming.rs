//! WebSocket streaming client for real-time updates
//!
//! This module provides WebSocket-based streaming for real-time soil microbiome data updates.
//!
//! 弘益人間 (Benefit All Humanity)

use crate::error::{Error, Result};
use crate::types::Environment;
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::sync::mpsc;
use tokio_tungstenite::{connect_async, tungstenite::Message};

/// Message types for the streaming protocol
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum MessageType {
    /// Authentication request
    AuthRequest,
    /// Authentication response
    AuthResponse,
    /// Sample submission
    SampleSubmit,
    /// Sample acknowledgment
    SampleAck,
    /// Analysis request
    AnalysisRequest,
    /// Analysis result
    AnalysisResult,
    /// Add subscription
    SubscriptionAdd,
    /// Remove subscription
    SubscriptionRemove,
    /// Alert notification
    AlertNotify,
    /// Heartbeat ping
    HeartbeatPing,
    /// Heartbeat pong
    HeartbeatPong,
    /// Error message
    Error,
}

/// Alert level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum AlertLevel {
    /// Informational alert
    Info,
    /// Warning alert
    Warning,
    /// Critical alert
    Critical,
}

/// Alert notification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alert {
    /// Alert level
    pub level: AlertLevel,
    /// Alert message
    pub message: String,
    /// Related sample ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sample_id: Option<String>,
    /// Alert timestamp
    pub timestamp: String,
}

/// Message header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageHeader {
    /// Protocol version
    pub version: String,
    /// Unique message identifier
    pub message_id: String,
    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,
    /// Timestamp
    pub timestamp: String,
    /// Message source
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<MessageEndpoint>,
    /// Message destination
    #[serde(skip_serializing_if = "Option::is_none")]
    pub destination: Option<MessageEndpoint>,
}

/// Message endpoint information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageEndpoint {
    /// Endpoint ID
    pub id: String,
    /// Endpoint type
    #[serde(rename = "type")]
    pub endpoint_type: String,
}

/// Security information for messages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageSecurity {
    /// Signature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<String>,
    /// Encryption method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub encryption: Option<String>,
    /// Key identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub key_id: Option<String>,
    /// Initialization vector
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iv: Option<String>,
}

/// Protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage {
    /// Message header
    pub header: MessageHeader,
    /// Message payload
    pub payload: serde_json::Value,
    /// Security information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub security: Option<MessageSecurity>,
}

/// WebSocket streaming client for real-time updates
///
/// # Example
///
/// ```rust,no_run
/// use wia_soil_microbiome_sdk::{StreamingClient, Environment};
///
/// #[tokio::main]
/// async fn main() -> Result<(), Box<dyn std::error::Error>> {
///     let mut client = StreamingClient::new("your-api-key", Environment::Production);
///
///     client.connect().await?;
///
///     // Subscribe to sample updates
///     client.subscribe_sample("sample-123").await?;
///
///     // Receive messages
///     while let Some(message) = client.receive().await {
///         println!("Received: {:?}", message);
///     }
///
///     Ok(())
/// }
/// ```
pub struct StreamingClient {
    api_key: String,
    environment: Environment,
    tx: Option<mpsc::UnboundedSender<Message>>,
    rx: Option<mpsc::UnboundedReceiver<ProtocolMessage>>,
}

impl StreamingClient {
    /// Create a new streaming client
    pub fn new(api_key: impl Into<String>, environment: Environment) -> Self {
        Self {
            api_key: api_key.into(),
            environment,
            tx: None,
            rx: None,
        }
    }

    /// Connect to the WebSocket server
    pub async fn connect(&mut self) -> Result<()> {
        let url = self.environment.ws_url();
        let (ws_stream, _) = connect_async(url)
            .await
            .map_err(|e| Error::WebSocket(e.to_string()))?;

        let (mut write, mut read) = ws_stream.split();

        // Create channels for sending and receiving messages
        let (msg_tx, mut msg_rx) = mpsc::unbounded_channel::<Message>();
        let (proto_tx, proto_rx) = mpsc::unbounded_channel::<ProtocolMessage>();

        self.tx = Some(msg_tx);
        self.rx = Some(proto_rx);

        // Task for sending messages
        tokio::spawn(async move {
            while let Some(msg) = msg_rx.recv().await {
                if let Err(e) = write.send(msg).await {
                    tracing::error!("Failed to send message: {}", e);
                    break;
                }
            }
        });

        // Task for receiving messages
        tokio::spawn(async move {
            while let Some(msg_result) = read.next().await {
                match msg_result {
                    Ok(Message::Text(text)) => {
                        if let Ok(proto_msg) = serde_json::from_str::<ProtocolMessage>(&text) {
                            if proto_tx.send(proto_msg).is_err() {
                                break;
                            }
                        }
                    }
                    Ok(Message::Close(_)) => break,
                    Err(e) => {
                        tracing::error!("WebSocket error: {}", e);
                        break;
                    }
                    _ => {}
                }
            }
        });

        // Send authentication
        self.authenticate().await?;

        Ok(())
    }

    /// Authenticate with the server
    async fn authenticate(&self) -> Result<()> {
        let auth_message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-SOIL-MICROBIOME/1.0".to_string(),
                message_id: format!("auth_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::AuthRequest,
                timestamp: chrono::Utc::now().to_rfc3339(),
                source: None,
                destination: None,
            },
            payload: serde_json::json!({
                "token": self.api_key
            }),
            security: None,
        };

        self.send_message(auth_message).await
    }

    /// Send a protocol message
    async fn send_message(&self, message: ProtocolMessage) -> Result<()> {
        if let Some(tx) = &self.tx {
            let json = serde_json::to_string(&message)?;
            tx.send(Message::Text(json))
                .map_err(|_| Error::WebSocket("Failed to send message".to_string()))?;
            Ok(())
        } else {
            Err(Error::WebSocket("Not connected".to_string()))
        }
    }

    /// Subscribe to sample updates
    pub async fn subscribe_sample(&self, sample_id: impl Into<String>) -> Result<()> {
        let message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-SOIL-MICROBIOME/1.0".to_string(),
                message_id: format!("sub_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::SubscriptionAdd,
                timestamp: chrono::Utc::now().to_rfc3339(),
                source: None,
                destination: None,
            },
            payload: serde_json::json!({
                "sampleId": sample_id.into()
            }),
            security: None,
        };

        self.send_message(message).await
    }

    /// Unsubscribe from sample updates
    pub async fn unsubscribe_sample(&self, sample_id: impl Into<String>) -> Result<()> {
        let message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-SOIL-MICROBIOME/1.0".to_string(),
                message_id: format!("unsub_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::SubscriptionRemove,
                timestamp: chrono::Utc::now().to_rfc3339(),
                source: None,
                destination: None,
            },
            payload: serde_json::json!({
                "sampleId": sample_id.into()
            }),
            security: None,
        };

        self.send_message(message).await
    }

    /// Receive the next protocol message
    pub async fn receive(&mut self) -> Option<ProtocolMessage> {
        if let Some(rx) = &mut self.rx {
            rx.recv().await
        } else {
            None
        }
    }

    /// Send a heartbeat ping
    pub async fn ping(&self) -> Result<()> {
        let message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-SOIL-MICROBIOME/1.0".to_string(),
                message_id: format!("ping_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::HeartbeatPing,
                timestamp: chrono::Utc::now().to_rfc3339(),
                source: None,
                destination: None,
            },
            payload: serde_json::json!({}),
            security: None,
        };

        self.send_message(message).await
    }

    /// Disconnect from the server
    pub async fn disconnect(&self) -> Result<()> {
        if let Some(tx) = &self.tx {
            tx.send(Message::Close(None))
                .map_err(|_| Error::WebSocket("Failed to close connection".to_string()))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_type_serialization() {
        let msg_type = MessageType::AuthRequest;
        let json = serde_json::to_string(&msg_type).unwrap();
        assert_eq!(json, "\"auth-request\"");
    }

    #[test]
    fn test_alert_level() {
        let alert = Alert {
            level: AlertLevel::Warning,
            message: "High temperature detected".to_string(),
            sample_id: Some("sample-123".to_string()),
            timestamp: "2024-01-01T00:00:00Z".to_string(),
        };
        assert_eq!(alert.level, AlertLevel::Warning);
    }
}
