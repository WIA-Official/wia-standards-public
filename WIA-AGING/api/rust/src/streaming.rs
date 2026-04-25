//! WebSocket streaming client for real-time data
//!
//! This module provides WebSocket connectivity for real-time biomarker streaming
//! from wearable devices and other data sources.

use crate::error::{Error, Result};
use crate::types::{Biomarker, Config, Environment};
use chrono::{DateTime, Utc};
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};
use tokio_tungstenite::{connect_async, tungstenite::Message};

/// Message types for the streaming protocol
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    /// Authentication request
    AuthRequest,
    /// Authentication response
    AuthResponse,
    /// Biomarker update
    BiomarkerUpdate,
    /// Biomarker acknowledgment
    BiomarkerAck,
    /// Assessment request
    AssessmentRequest,
    /// Assessment result
    AssessmentResult,
    /// Subscription add
    SubscriptionAdd,
    /// Subscription remove
    SubscriptionRemove,
    /// Alert notification
    AlertNotify,
    /// Heartbeat ping
    HeartbeatPing,
    /// Heartbeat pong
    HeartbeatPong,
    /// Error
    Error,
}

/// Alert level
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AlertLevel {
    /// Informational
    Info,
    /// Warning
    Warning,
    /// Critical
    Critical,
}

/// Alert notification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alert {
    /// Alert level
    pub level: AlertLevel,
    /// Alert message
    pub message: String,
    /// Related biomarker code
    #[serde(skip_serializing_if = "Option::is_none")]
    pub biomarker_code: Option<String>,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

/// Message header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageHeader {
    /// Protocol version
    pub version: String,
    /// Message ID
    pub message_id: String,
    /// Message type
    #[serde(rename = "type")]
    pub message_type: MessageType,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

/// Protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage<T> {
    /// Message header
    pub header: MessageHeader,
    /// Message payload
    pub payload: T,
}

/// Authentication payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthPayload {
    /// API token
    pub token: String,
}

/// Biomarker update payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiomarkerUpdatePayload {
    /// Profile ID
    pub profile_id: String,
    /// Biomarkers
    pub biomarkers: Vec<Biomarker>,
}

/// Biomarker acknowledgment payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiomarkerAckPayload {
    /// Event ID
    pub event_id: String,
    /// Accepted
    pub accepted: bool,
    /// Status message
    pub status: String,
    /// Alerts
    #[serde(default)]
    pub alerts: Vec<Alert>,
}

/// Streaming event received from the server
#[derive(Debug, Clone)]
pub enum StreamEvent {
    /// Connected to server
    Connected,
    /// Authenticated successfully
    Authenticated,
    /// Biomarker acknowledgment
    BiomarkerAck(BiomarkerAckPayload),
    /// Alert received
    Alert(Alert),
    /// Error received
    Error(String),
    /// Disconnected
    Disconnected,
}

/// Streaming client for real-time data
pub struct StreamingClient {
    config: Config,
    ws_url: String,
    event_tx: Option<mpsc::Sender<StreamEvent>>,
    connected: Arc<RwLock<bool>>,
}

impl StreamingClient {
    /// Create a new streaming client
    pub fn new(config: Config) -> Self {
        let ws_url = config.environment.ws_url().to_string();
        Self {
            config,
            ws_url,
            event_tx: None,
            connected: Arc::new(RwLock::new(false)),
        }
    }

    /// Connect to the streaming service
    ///
    /// Returns a receiver for streaming events
    pub async fn connect(&mut self) -> Result<mpsc::Receiver<StreamEvent>> {
        let (event_tx, event_rx) = mpsc::channel(100);
        self.event_tx = Some(event_tx.clone());

        let url = url::Url::parse(&self.ws_url)
            .map_err(|e| Error::Config(format!("Invalid WebSocket URL: {}", e)))?;

        let (ws_stream, _) = connect_async(url).await?;
        let (mut write, mut read) = ws_stream.split();

        // Send authentication
        let auth_message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-AGING/1.0".to_string(),
                message_id: format!("auth_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::AuthRequest,
                timestamp: Utc::now(),
            },
            payload: AuthPayload {
                token: self.config.api_key.clone(),
            },
        };

        let auth_json = serde_json::to_string(&auth_message)?;
        write.send(Message::Text(auth_json)).await?;

        *self.connected.write().await = true;
        let _ = event_tx.send(StreamEvent::Connected).await;

        let connected = self.connected.clone();
        let tx = event_tx.clone();

        // Spawn message handler
        tokio::spawn(async move {
            while let Some(msg) = read.next().await {
                match msg {
                    Ok(Message::Text(text)) => {
                        if let Ok(json) = serde_json::from_str::<serde_json::Value>(&text) {
                            if let Some(msg_type) = json
                                .get("header")
                                .and_then(|h| h.get("type"))
                                .and_then(|t| t.as_str())
                            {
                                match msg_type {
                                    "auth_response" => {
                                        let _ = tx.send(StreamEvent::Authenticated).await;
                                    }
                                    "biomarker_ack" => {
                                        if let Ok(ack) = serde_json::from_value::<
                                            ProtocolMessage<BiomarkerAckPayload>,
                                        >(
                                            json.clone()
                                        ) {
                                            let _ = tx
                                                .send(StreamEvent::BiomarkerAck(ack.payload))
                                                .await;
                                        }
                                    }
                                    "alert_notify" => {
                                        if let Ok(payload) = json
                                            .get("payload")
                                            .cloned()
                                            .map(serde_json::from_value::<Alert>)
                                            .transpose()
                                        {
                                            if let Some(alert) = payload {
                                                let _ = tx.send(StreamEvent::Alert(alert)).await;
                                            }
                                        }
                                    }
                                    "error" => {
                                        if let Some(msg) = json
                                            .get("payload")
                                            .and_then(|p| p.get("message"))
                                            .and_then(|m| m.as_str())
                                        {
                                            let _ =
                                                tx.send(StreamEvent::Error(msg.to_string())).await;
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                    Ok(Message::Close(_)) => {
                        *connected.write().await = false;
                        let _ = tx.send(StreamEvent::Disconnected).await;
                        break;
                    }
                    Err(e) => {
                        let _ = tx.send(StreamEvent::Error(e.to_string())).await;
                    }
                    _ => {}
                }
            }
        });

        Ok(event_rx)
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        *self.connected.read().await
    }

    /// Send biomarker update
    pub async fn send_biomarkers(
        &self,
        profile_id: &str,
        biomarkers: Vec<Biomarker>,
    ) -> Result<()> {
        if !self.is_connected().await {
            return Err(Error::Connection("Not connected".to_string()));
        }

        let message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-AGING/1.0".to_string(),
                message_id: format!("bio_{}", chrono::Utc::now().timestamp_millis()),
                message_type: MessageType::BiomarkerUpdate,
                timestamp: Utc::now(),
            },
            payload: BiomarkerUpdatePayload {
                profile_id: profile_id.to_string(),
                biomarkers,
            },
        };

        let _json = serde_json::to_string(&message)?;
        // In a real implementation, we'd send this through the WebSocket
        // For now, this is a placeholder

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_streaming_client_creation() {
        let client = StreamingClient::new(Config {
            api_key: "test-key".to_string(),
            environment: Environment::Sandbox,
            ..Default::default()
        });

        assert_eq!(client.ws_url, Environment::Sandbox.ws_url());
    }

    #[test]
    fn test_protocol_message_serialization() {
        let message = ProtocolMessage {
            header: MessageHeader {
                version: "WIA-AGING/1.0".to_string(),
                message_id: "test_123".to_string(),
                message_type: MessageType::AuthRequest,
                timestamp: Utc::now(),
            },
            payload: AuthPayload {
                token: "test-token".to_string(),
            },
        };

        let json = serde_json::to_string(&message).unwrap();
        assert!(json.contains("WIA-AGING/1.0"));
        assert!(json.contains("test_123"));
    }
}
