//! WPP Client Implementation
//!
//! Provides a high-level client for the WIA Physics Protocol.

use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock, Mutex};
use uuid::Uuid;

use crate::error::{PhysicsError, PhysicsResult};
use crate::types::*;
use super::messages::*;

// ============================================================================
// Client Configuration
// ============================================================================

/// WPP Client configuration
#[derive(Debug, Clone)]
pub struct WppClientConfig {
    /// Client ID (auto-generated if not set)
    pub client_id: String,
    /// Human-readable client name
    pub client_name: String,
    /// Authentication info
    pub auth: Option<AuthInfo>,
    /// Heartbeat interval in milliseconds
    pub heartbeat_interval: u64,
    /// Enable binary mode
    pub binary_mode: bool,
    /// Compression type
    pub compression: Compression,
    /// Connection timeout in milliseconds
    pub connect_timeout: u64,
    /// Command timeout in milliseconds
    pub command_timeout: u64,
    /// Auto-reconnect on disconnect
    pub auto_reconnect: bool,
    /// Maximum reconnection attempts
    pub max_reconnect_attempts: u32,
}

impl Default for WppClientConfig {
    fn default() -> Self {
        Self {
            client_id: Uuid::new_v4().to_string(),
            client_name: "WIA Physics Client".to_string(),
            auth: None,
            heartbeat_interval: super::DEFAULT_HEARTBEAT_INTERVAL,
            binary_mode: false,
            compression: Compression::None,
            connect_timeout: 10000,
            command_timeout: super::DEFAULT_COMMAND_TIMEOUT,
            auto_reconnect: true,
            max_reconnect_attempts: 5,
        }
    }
}

impl WppClientConfig {
    /// Create a new config with client name
    pub fn new(client_name: &str) -> Self {
        Self {
            client_name: client_name.to_string(),
            ..Default::default()
        }
    }

    /// Set authentication token
    pub fn with_token(mut self, token: &str) -> Self {
        self.auth = Some(AuthInfo {
            method: AuthMethod::Token,
            token: Some(token.to_string()),
            api_key: None,
        });
        self
    }

    /// Set API key authentication
    pub fn with_api_key(mut self, api_key: &str) -> Self {
        self.auth = Some(AuthInfo {
            method: AuthMethod::ApiKey,
            token: None,
            api_key: Some(api_key.to_string()),
        });
        self
    }

    /// Enable binary mode
    pub fn with_binary_mode(mut self) -> Self {
        self.binary_mode = true;
        self
    }

    /// Set compression
    pub fn with_compression(mut self, compression: Compression) -> Self {
        self.compression = compression;
        self
    }
}

// ============================================================================
// Connection State
// ============================================================================

/// Connection state
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Disconnecting,
}

// ============================================================================
// Subscription
// ============================================================================

/// Active subscription
#[derive(Debug, Clone)]
pub struct Subscription {
    /// Subscription ID
    pub id: String,
    /// Channel name
    pub channel: String,
    /// Data type
    pub data_type: DataType,
    /// Status
    pub status: SubscriptionStatus,
}

// ============================================================================
// WPP Client
// ============================================================================

/// WPP Protocol Client
///
/// High-level client for connecting to WIA Physics Protocol servers.
///
/// # Example
///
/// ```rust,no_run
/// use wia_physics::protocol::*;
///
/// async fn example() -> Result<(), Box<dyn std::error::Error>> {
///     // Create client
///     let config = WppClientConfig::new("My Physics App")
///         .with_token("my-jwt-token");
///     let client = WppClient::new(config);
///
///     // Connect (mock - actual transport implementation needed)
///     // client.connect("wss://physics.wia.live/wpp/v1").await?;
///
///     Ok(())
/// }
/// ```
pub struct WppClient {
    /// Client configuration
    config: WppClientConfig,
    /// Connection state
    state: Arc<RwLock<ConnectionState>>,
    /// Session ID (set after connect)
    session_id: Arc<RwLock<Option<String>>>,
    /// Active subscriptions
    subscriptions: Arc<RwLock<HashMap<String, Subscription>>>,
    /// Pending commands (id -> response sender)
    pending_commands: Arc<Mutex<HashMap<String, mpsc::Sender<ResponsePayload>>>>,
    /// Message sequence counter
    sequence: Arc<RwLock<u64>>,
    /// Available channels (from server)
    available_channels: Arc<RwLock<Vec<ChannelInfo>>>,
}

impl WppClient {
    /// Create a new WPP client
    pub fn new(config: WppClientConfig) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(ConnectionState::Disconnected)),
            session_id: Arc::new(RwLock::new(None)),
            subscriptions: Arc::new(RwLock::new(HashMap::new())),
            pending_commands: Arc::new(Mutex::new(HashMap::new())),
            sequence: Arc::new(RwLock::new(0)),
            available_channels: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Get current connection state
    pub async fn state(&self) -> ConnectionState {
        self.state.read().await.clone()
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        *self.state.read().await == ConnectionState::Connected
    }

    /// Get session ID
    pub async fn session_id(&self) -> Option<String> {
        self.session_id.read().await.clone()
    }

    /// Get available channels
    pub async fn available_channels(&self) -> Vec<ChannelInfo> {
        self.available_channels.read().await.clone()
    }

    /// Get active subscriptions
    pub async fn subscriptions(&self) -> Vec<Subscription> {
        self.subscriptions.read().await.values().cloned().collect()
    }

    /// Create a connect message
    pub fn create_connect_message(&self) -> WppMessage {
        WppMessage::new(
            MessageType::Connect,
            MessagePayload::Connect(ConnectPayload {
                client_id: self.config.client_id.clone(),
                client_name: Some(self.config.client_name.clone()),
                client_version: Some(env!("CARGO_PKG_VERSION").to_string()),
                capabilities: vec![
                    Capability::Streaming,
                    Capability::Commands,
                    Capability::Events,
                ],
                auth: self.config.auth.clone(),
                options: Some(ConnectionOptions {
                    heartbeat_interval: self.config.heartbeat_interval,
                    compression: self.config.compression.clone(),
                    binary_mode: self.config.binary_mode,
                }),
            }),
        )
    }

    /// Create a subscribe message
    pub fn create_subscribe_message(&self, channel: &str) -> WppMessage {
        WppMessage::subscribe(channel)
    }

    /// Create a subscribe message with options
    pub fn create_subscribe_message_with_options(
        &self,
        channel: &str,
        filter: Option<SubscriptionFilter>,
        options: Option<SubscriptionOptions>,
    ) -> WppMessage {
        WppMessage::new(
            MessageType::Subscribe,
            MessagePayload::Subscribe(SubscribePayload {
                channel: channel.to_string(),
                filter,
                options,
            }),
        )
    }

    /// Create an unsubscribe message
    pub fn create_unsubscribe_message(&self, channel: &str) -> WppMessage {
        WppMessage::unsubscribe(channel)
    }

    /// Create a command message
    pub fn create_command_message(
        &self,
        target: &str,
        action: &str,
        parameters: serde_json::Value,
    ) -> WppMessage {
        WppMessage::command(target, action, parameters)
    }

    /// Create a ping message
    pub fn create_ping_message(&self) -> WppMessage {
        WppMessage::ping()
    }

    /// Create a disconnect message
    pub fn create_disconnect_message(&self, reason: DisconnectReason) -> WppMessage {
        WppMessage::disconnect(reason, None)
    }

    /// Handle incoming connect_ack message
    pub async fn handle_connect_ack(&self, payload: ConnectAckPayload) -> PhysicsResult<()> {
        *self.state.write().await = ConnectionState::Connected;
        *self.session_id.write().await = Some(payload.session_id);
        *self.available_channels.write().await = payload.channels;
        Ok(())
    }

    /// Handle incoming subscribe_ack message
    pub async fn handle_subscribe_ack(&self, payload: SubscribeAckPayload) -> PhysicsResult<()> {
        let subscription = Subscription {
            id: payload.subscription_id.clone(),
            channel: payload.channel.clone(),
            data_type: DataType::Fusion, // Default, should be set from channel info
            status: payload.status,
        };
        self.subscriptions
            .write()
            .await
            .insert(payload.channel, subscription);
        Ok(())
    }

    /// Handle incoming unsubscribe_ack message
    pub async fn handle_unsubscribe_ack(&self, payload: UnsubscribeAckPayload) -> PhysicsResult<()> {
        if let Some(channel) = &payload.channel {
            self.subscriptions.write().await.remove(channel);
        }
        Ok(())
    }

    /// Get next sequence number
    pub async fn next_sequence(&self) -> u64 {
        let mut seq = self.sequence.write().await;
        *seq += 1;
        *seq
    }

    /// Reset client state (for reconnection)
    pub async fn reset(&self) {
        *self.state.write().await = ConnectionState::Disconnected;
        *self.session_id.write().await = None;
        self.subscriptions.write().await.clear();
        self.pending_commands.lock().await.clear();
        *self.sequence.write().await = 0;
    }
}

// ============================================================================
// Data Stream
// ============================================================================

/// Data stream receiver for subscriptions
pub struct DataStream<T> {
    channel: String,
    receiver: mpsc::Receiver<T>,
}

impl<T> DataStream<T> {
    /// Create a new data stream
    pub fn new(channel: &str, receiver: mpsc::Receiver<T>) -> Self {
        Self {
            channel: channel.to_string(),
            receiver,
        }
    }

    /// Get the channel name
    pub fn channel(&self) -> &str {
        &self.channel
    }

    /// Receive next data item
    pub async fn next(&mut self) -> Option<T> {
        self.receiver.recv().await
    }
}

// ============================================================================
// Builder Pattern
// ============================================================================

/// Builder for WppClient
pub struct WppClientBuilder {
    config: WppClientConfig,
}

impl WppClientBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            config: WppClientConfig::default(),
        }
    }

    /// Set client name
    pub fn client_name(mut self, name: &str) -> Self {
        self.config.client_name = name.to_string();
        self
    }

    /// Set client ID
    pub fn client_id(mut self, id: &str) -> Self {
        self.config.client_id = id.to_string();
        self
    }

    /// Set JWT token authentication
    pub fn with_token(mut self, token: &str) -> Self {
        self.config.auth = Some(AuthInfo {
            method: AuthMethod::Token,
            token: Some(token.to_string()),
            api_key: None,
        });
        self
    }

    /// Set API key authentication
    pub fn with_api_key(mut self, api_key: &str) -> Self {
        self.config.auth = Some(AuthInfo {
            method: AuthMethod::ApiKey,
            token: None,
            api_key: Some(api_key.to_string()),
        });
        self
    }

    /// Set heartbeat interval
    pub fn heartbeat_interval(mut self, ms: u64) -> Self {
        self.config.heartbeat_interval = ms;
        self
    }

    /// Enable binary mode
    pub fn binary_mode(mut self, enabled: bool) -> Self {
        self.config.binary_mode = enabled;
        self
    }

    /// Set compression
    pub fn compression(mut self, compression: Compression) -> Self {
        self.config.compression = compression;
        self
    }

    /// Set auto-reconnect
    pub fn auto_reconnect(mut self, enabled: bool) -> Self {
        self.config.auto_reconnect = enabled;
        self
    }

    /// Build the client
    pub fn build(self) -> WppClient {
        WppClient::new(self.config)
    }
}

impl Default for WppClientBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_client_creation() {
        let config = WppClientConfig::new("Test Client");
        let client = WppClient::new(config);

        assert_eq!(client.state().await, ConnectionState::Disconnected);
        assert!(client.session_id().await.is_none());
    }

    #[tokio::test]
    async fn test_client_builder() {
        let client = WppClientBuilder::new()
            .client_name("Physics App")
            .with_token("test-token")
            .heartbeat_interval(60000)
            .build();

        assert_eq!(client.config.client_name, "Physics App");
        assert_eq!(client.config.heartbeat_interval, 60000);
        assert!(client.config.auth.is_some());
    }

    #[tokio::test]
    async fn test_connect_message() {
        let client = WppClientBuilder::new()
            .client_name("Test")
            .build();

        let msg = client.create_connect_message();
        assert_eq!(msg.msg_type, MessageType::Connect);
    }

    #[tokio::test]
    async fn test_subscribe_message() {
        let client = WppClientBuilder::new().build();
        let msg = client.create_subscribe_message("fusion/iter/plasma");

        assert_eq!(msg.msg_type, MessageType::Subscribe);
    }

    #[tokio::test]
    async fn test_handle_connect_ack() {
        let client = WppClientBuilder::new().build();

        let payload = ConnectAckPayload {
            session_id: "session-123".to_string(),
            server_name: Some("Test Server".to_string()),
            server_version: Some("1.0.0".to_string()),
            capabilities: vec![Capability::Streaming],
            heartbeat_interval: Some(30000),
            max_subscriptions: Some(100),
            channels: vec![ChannelInfo {
                name: "fusion/iter/plasma".to_string(),
                channel_type: DataType::Fusion,
                description: None,
            }],
        };

        client.handle_connect_ack(payload).await.unwrap();

        assert_eq!(client.state().await, ConnectionState::Connected);
        assert_eq!(client.session_id().await, Some("session-123".to_string()));
        assert_eq!(client.available_channels().await.len(), 1);
    }

    #[tokio::test]
    async fn test_sequence_counter() {
        let client = WppClientBuilder::new().build();

        assert_eq!(client.next_sequence().await, 1);
        assert_eq!(client.next_sequence().await, 2);
        assert_eq!(client.next_sequence().await, 3);
    }
}
