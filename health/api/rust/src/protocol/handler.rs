//! WIA Health Protocol Handler
//!
//! Connection and message handling for the WIA Health protocol

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use tokio::sync::{mpsc, RwLock};
use uuid::Uuid;

use super::message::*;
use crate::error::{HealthError, Result};

/// Connection state enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
}

/// Stream subscription information
#[derive(Debug, Clone)]
pub struct Subscription {
    pub stream_id: String,
    pub stream_type: String,
    pub filter: Option<StreamFilter>,
    pub options: Option<StreamOptions>,
    pub created_at: Instant,
}

/// Client session information
#[derive(Debug, Clone)]
pub struct Session {
    pub session_id: String,
    pub client_id: String,
    pub client_name: Option<String>,
    pub connected_at: Instant,
    pub last_activity: Instant,
    pub subscriptions: HashMap<String, Subscription>,
}

impl Session {
    /// Create a new session
    pub fn new(session_id: String, client_id: String, client_name: Option<String>) -> Self {
        let now = Instant::now();
        Self {
            session_id,
            client_id,
            client_name,
            connected_at: now,
            last_activity: now,
            subscriptions: HashMap::new(),
        }
    }

    /// Update last activity timestamp
    pub fn touch(&mut self) {
        self.last_activity = Instant::now();
    }

    /// Add a subscription
    pub fn add_subscription(&mut self, sub: Subscription) {
        self.subscriptions.insert(sub.stream_id.clone(), sub);
    }

    /// Remove a subscription
    pub fn remove_subscription(&mut self, stream_id: &str) -> Option<Subscription> {
        self.subscriptions.remove(stream_id)
    }
}

/// Protocol handler callback trait
#[async_trait]
pub trait ProtocolCallback: Send + Sync {
    /// Called when connection is established
    async fn on_connect(&self, session: &Session) -> Result<()>;

    /// Called when connection is closed
    async fn on_disconnect(&self, session: &Session, reason: &str) -> Result<()>;

    /// Called when subscription is created
    async fn on_subscribe(&self, session: &Session, subscription: &Subscription) -> Result<()>;

    /// Called when subscription is removed
    async fn on_unsubscribe(&self, session: &Session, stream_id: &str) -> Result<()>;

    /// Called when biomarker data is received
    async fn on_biomarker(&self, session: &Session, payload: &BiomarkerPayload) -> Result<()>;

    /// Called when profile update is received
    async fn on_profile_update(
        &self,
        session: &Session,
        payload: &ProfileUpdatePayload,
    ) -> Result<()>;

    /// Called when ping is received
    async fn on_ping(&self, session: &Session) -> Result<()>;

    /// Called when alert is triggered
    async fn on_alert(&self, session: &Session, payload: &AlertPayload) -> Result<()>;
}

/// Default no-op callback implementation
pub struct NoOpCallback;

#[async_trait]
impl ProtocolCallback for NoOpCallback {
    async fn on_connect(&self, _session: &Session) -> Result<()> {
        Ok(())
    }
    async fn on_disconnect(&self, _session: &Session, _reason: &str) -> Result<()> {
        Ok(())
    }
    async fn on_subscribe(&self, _session: &Session, _subscription: &Subscription) -> Result<()> {
        Ok(())
    }
    async fn on_unsubscribe(&self, _session: &Session, _stream_id: &str) -> Result<()> {
        Ok(())
    }
    async fn on_biomarker(&self, _session: &Session, _payload: &BiomarkerPayload) -> Result<()> {
        Ok(())
    }
    async fn on_profile_update(
        &self,
        _session: &Session,
        _payload: &ProfileUpdatePayload,
    ) -> Result<()> {
        Ok(())
    }
    async fn on_ping(&self, _session: &Session) -> Result<()> {
        Ok(())
    }
    async fn on_alert(&self, _session: &Session, _payload: &AlertPayload) -> Result<()> {
        Ok(())
    }
}

/// Protocol handler configuration
#[derive(Debug, Clone)]
pub struct HandlerConfig {
    /// Maximum stream rate (messages per second)
    pub max_stream_rate: u32,
    /// Keep-alive interval in milliseconds
    pub keep_alive_interval: Duration,
    /// Session timeout in milliseconds
    pub session_timeout: Duration,
    /// Maximum message size in bytes
    pub max_message_size: usize,
    /// Server identifier
    pub server_id: String,
    /// Server version
    pub server_version: String,
    /// Supported capabilities
    pub capabilities: Vec<String>,
}

impl Default for HandlerConfig {
    fn default() -> Self {
        Self {
            max_stream_rate: 100,
            keep_alive_interval: Duration::from_secs(30),
            session_timeout: Duration::from_secs(3600),
            max_message_size: 1024 * 1024, // 1MB
            server_id: format!("wia-health-{}", &Uuid::new_v4().to_string()[..8]),
            server_version: PROTOCOL_VERSION.to_string(),
            capabilities: vec![
                "biomarkers".to_string(),
                "digital-twin".to_string(),
                "alerts".to_string(),
                "simulation".to_string(),
            ],
        }
    }
}

/// Protocol message handler
pub struct ProtocolHandler {
    config: HandlerConfig,
    state: Arc<RwLock<ConnectionState>>,
    session: Arc<RwLock<Option<Session>>>,
    outbound_tx: mpsc::Sender<Message>,
    callback: Arc<dyn ProtocolCallback>,
}

impl ProtocolHandler {
    /// Create a new protocol handler
    pub fn new(
        config: HandlerConfig,
        outbound_tx: mpsc::Sender<Message>,
        callback: Arc<dyn ProtocolCallback>,
    ) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(ConnectionState::Disconnected)),
            session: Arc::new(RwLock::new(None)),
            outbound_tx,
            callback,
        }
    }

    /// Create handler with default callback
    pub fn with_defaults(config: HandlerConfig, outbound_tx: mpsc::Sender<Message>) -> Self {
        Self::new(config, outbound_tx, Arc::new(NoOpCallback))
    }

    /// Get current connection state
    pub async fn state(&self) -> ConnectionState {
        *self.state.read().await
    }

    /// Get current session
    pub async fn session(&self) -> Option<Session> {
        self.session.read().await.clone()
    }

    /// Set connection state
    async fn set_state(&self, state: ConnectionState) {
        *self.state.write().await = state;
    }

    /// Send a message through the outbound channel
    async fn send(&self, message: Message) -> Result<()> {
        self.outbound_tx
            .send(message)
            .await
            .map_err(|_| HealthError::ProtocolError("Failed to send message".into()))
    }

    /// Handle incoming message
    pub async fn handle_message(&self, message: Message) -> Result<()> {
        // Update last activity
        if let Some(session) = self.session.write().await.as_mut() {
            session.touch();
        }

        match message.message_type {
            MessageType::Connect => self.handle_connect(message).await,
            MessageType::ConnectAck => self.handle_connect_ack(message).await,
            MessageType::Disconnect => self.handle_disconnect(message).await,
            MessageType::DisconnectAck => Ok(()),
            MessageType::Subscribe => self.handle_subscribe(message).await,
            MessageType::SubscribeAck => self.handle_subscribe_ack(message).await,
            MessageType::Unsubscribe => self.handle_unsubscribe(message).await,
            MessageType::UnsubscribeAck => Ok(()),
            MessageType::Biomarker => self.handle_biomarker(message).await,
            MessageType::ProfileUpdate => self.handle_profile_update(message).await,
            MessageType::SimulationResult => self.handle_simulation_result(message).await,
            MessageType::Ping => self.handle_ping(message).await,
            MessageType::Pong => self.handle_pong(message).await,
            MessageType::Alert => self.handle_alert(message).await,
            MessageType::Error => self.handle_error(message).await,
        }
    }

    /// Handle connect request (server-side)
    async fn handle_connect(&self, message: Message) -> Result<()> {
        let payload: ConnectPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        // Create new session
        let session_id = Uuid::new_v4().to_string();
        let session = Session::new(
            session_id.clone(),
            payload.client_id.clone(),
            payload.client_name.clone(),
        );

        *self.session.write().await = Some(session.clone());
        self.set_state(ConnectionState::Connected).await;

        // Invoke callback
        self.callback.on_connect(&session).await?;

        // Send connect acknowledgment
        let ack_payload = ConnectAckPayload {
            session_id,
            server_id: self.config.server_id.clone(),
            server_version: self.config.server_version.clone(),
            capabilities: Some(self.config.capabilities.clone()),
            settings: Some(ServerSettings {
                max_stream_rate: self.config.max_stream_rate,
                keep_alive_interval: self.config.keep_alive_interval.as_millis() as u64,
                session_timeout: self.config.session_timeout.as_millis() as u64,
            }),
        };

        let ack = MessageBuilder::connect_ack(message.message_id, ack_payload);
        self.send(ack).await
    }

    /// Handle connect acknowledgment (client-side)
    async fn handle_connect_ack(&self, message: Message) -> Result<()> {
        let payload: ConnectAckPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        // Update session with server info
        if let Some(session) = self.session.write().await.as_mut() {
            session.session_id = payload.session_id;
        }

        self.set_state(ConnectionState::Connected).await;
        Ok(())
    }

    /// Handle disconnect request
    async fn handle_disconnect(&self, message: Message) -> Result<()> {
        let payload: DisconnectPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.read().await.as_ref() {
            self.callback
                .on_disconnect(session, &payload.reason)
                .await?;
        }

        self.set_state(ConnectionState::Disconnected).await;
        *self.session.write().await = None;

        // Send disconnect acknowledgment
        let ack = Message::response(
            MessageType::DisconnectAck,
            message.message_id,
            serde_json::json!({}),
        );
        self.send(ack).await
    }

    /// Handle subscribe request
    async fn handle_subscribe(&self, message: Message) -> Result<()> {
        let payload: SubscribePayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        let subscription_id = Uuid::new_v4().to_string();
        let mut stream_statuses = Vec::new();

        let session_guard = self.session.read().await;
        let session = session_guard
            .as_ref()
            .ok_or_else(|| HealthError::ProtocolError("Not connected".into()))?;

        for stream_sub in payload.streams {
            let stream_id = Uuid::new_v4().to_string();
            let subscription = Subscription {
                stream_id: stream_id.clone(),
                stream_type: stream_sub.stream_type.clone(),
                filter: stream_sub.filter,
                options: stream_sub.options,
                created_at: Instant::now(),
            };

            self.callback.on_subscribe(session, &subscription).await?;

            stream_statuses.push(StreamStatus {
                stream_type: stream_sub.stream_type,
                stream_id,
                status: "active".to_string(),
            });
        }
        drop(session_guard);

        // Add subscriptions to session
        if let Some(session) = self.session.write().await.as_mut() {
            for status in &stream_statuses {
                let sub = Subscription {
                    stream_id: status.stream_id.clone(),
                    stream_type: status.stream_type.clone(),
                    filter: None,
                    options: None,
                    created_at: Instant::now(),
                };
                session.add_subscription(sub);
            }
        }

        // Send subscribe acknowledgment
        let ack_payload = SubscribeAckPayload {
            subscription_id,
            streams: stream_statuses,
        };
        let ack = MessageBuilder::subscribe_ack(message.message_id, ack_payload);
        self.send(ack).await
    }

    /// Handle subscribe acknowledgment
    async fn handle_subscribe_ack(&self, message: Message) -> Result<()> {
        let payload: SubscribeAckPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        // Add subscriptions to session
        if let Some(session) = self.session.write().await.as_mut() {
            for status in payload.streams {
                let sub = Subscription {
                    stream_id: status.stream_id,
                    stream_type: status.stream_type,
                    filter: None,
                    options: None,
                    created_at: Instant::now(),
                };
                session.add_subscription(sub);
            }
        }

        Ok(())
    }

    /// Handle unsubscribe request
    async fn handle_unsubscribe(&self, message: Message) -> Result<()> {
        let payload: UnsubscribePayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.write().await.as_mut() {
            for stream_id in &payload.stream_ids {
                session.remove_subscription(stream_id);
                self.callback.on_unsubscribe(session, stream_id).await?;
            }
        }

        // Send unsubscribe acknowledgment
        let ack = Message::response(
            MessageType::UnsubscribeAck,
            message.message_id,
            serde_json::json!({
                "streamIds": payload.stream_ids
            }),
        );
        self.send(ack).await
    }

    /// Handle biomarker data
    async fn handle_biomarker(&self, message: Message) -> Result<()> {
        let payload: BiomarkerPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.read().await.as_ref() {
            self.callback.on_biomarker(session, &payload).await?;
        }

        Ok(())
    }

    /// Handle profile update
    async fn handle_profile_update(&self, message: Message) -> Result<()> {
        let payload: ProfileUpdatePayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.read().await.as_ref() {
            self.callback.on_profile_update(session, &payload).await?;
        }

        Ok(())
    }

    /// Handle simulation result
    async fn handle_simulation_result(&self, _message: Message) -> Result<()> {
        // Simulation results are typically handled by specific simulation callbacks
        Ok(())
    }

    /// Handle ping request
    async fn handle_ping(&self, message: Message) -> Result<()> {
        let payload: PingPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.read().await.as_ref() {
            self.callback.on_ping(session).await?;
        }

        // Send pong response
        let pong = MessageBuilder::pong(message.message_id, payload.timestamp);
        self.send(pong).await
    }

    /// Handle pong response
    async fn handle_pong(&self, _message: Message) -> Result<()> {
        // Pong received - connection is alive
        Ok(())
    }

    /// Handle alert
    async fn handle_alert(&self, message: Message) -> Result<()> {
        let payload: AlertPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        if let Some(session) = self.session.read().await.as_ref() {
            self.callback.on_alert(session, &payload).await?;
        }

        Ok(())
    }

    /// Handle error
    async fn handle_error(&self, message: Message) -> Result<()> {
        let payload: ErrorPayload = serde_json::from_value(message.payload.clone())
            .map_err(|e| HealthError::SerializationError(e.to_string()))?;

        // Log error or handle based on error code
        if !payload.recoverable {
            self.set_state(ConnectionState::Disconnected).await;
        }

        Err(HealthError::ProtocolError(format!(
            "[{}] {}: {}",
            payload.code, payload.name, payload.message
        )))
    }

    /// Initiate connection (client-side)
    pub async fn connect(&self, payload: ConnectPayload) -> Result<()> {
        self.set_state(ConnectionState::Connecting).await;

        // Create pending session
        let session = Session::new(
            String::new(), // Will be set by server
            payload.client_id.clone(),
            payload.client_name.clone(),
        );
        *self.session.write().await = Some(session);

        // Send connect message
        let msg = MessageBuilder::connect(payload);
        self.send(msg).await
    }

    /// Initiate graceful disconnect
    pub async fn disconnect(&self, reason: &str) -> Result<()> {
        let msg = MessageBuilder::disconnect(reason, 1000);
        self.send(msg).await?;
        self.set_state(ConnectionState::Disconnected).await;
        Ok(())
    }

    /// Subscribe to data streams
    pub async fn subscribe(&self, streams: Vec<StreamSubscription>) -> Result<()> {
        let msg = MessageBuilder::subscribe(streams);
        self.send(msg).await
    }

    /// Unsubscribe from data streams
    pub async fn unsubscribe(&self, stream_ids: Vec<String>) -> Result<()> {
        let msg = MessageBuilder::unsubscribe(stream_ids);
        self.send(msg).await
    }

    /// Send ping to keep connection alive
    pub async fn ping(&self) -> Result<()> {
        let msg = MessageBuilder::ping();
        self.send(msg).await
    }

    /// Send biomarker data
    pub async fn send_biomarker(&self, payload: BiomarkerPayload) -> Result<()> {
        let msg = MessageBuilder::biomarker(payload);
        self.send(msg).await
    }

    /// Send alert
    pub async fn send_alert(&self, payload: AlertPayload) -> Result<()> {
        let msg = MessageBuilder::alert(payload);
        self.send(msg).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::mpsc;

    #[tokio::test]
    async fn test_handler_creation() {
        let (tx, _rx) = mpsc::channel(100);
        let config = HandlerConfig::default();
        let handler = ProtocolHandler::with_defaults(config, tx);

        assert_eq!(handler.state().await, ConnectionState::Disconnected);
        assert!(handler.session().await.is_none());
    }

    #[tokio::test]
    async fn test_session_management() {
        let mut session = Session::new(
            "test-session".to_string(),
            "test-client".to_string(),
            Some("Test Client".to_string()),
        );

        assert_eq!(session.subscriptions.len(), 0);

        let sub = Subscription {
            stream_id: "stream-1".to_string(),
            stream_type: "biomarkers".to_string(),
            filter: None,
            options: None,
            created_at: Instant::now(),
        };

        session.add_subscription(sub);
        assert_eq!(session.subscriptions.len(), 1);

        session.remove_subscription("stream-1");
        assert_eq!(session.subscriptions.len(), 0);
    }
}
