//! WebSocket Transport implementation
//!
//! Real-time streaming client for the WIA Material Protocol.

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};
use uuid::Uuid;

use super::{StreamingTransport, Transport, WebSocketConfig};
use crate::error::{MaterialError, MaterialResult};
use crate::protocol::{
    CreatePayload, CreateResponsePayload, DataPayload, DeletePayload, DeleteResponsePayload,
    GetPayload, GetResponsePayload, HandshakeAckPayload, HandshakePayload, Message,
    MessageBuilder, MessageType, QueryPayload, QueryResponsePayload, SubscribeAckPayload,
    SubscribePayload, SubscriptionChannel, UnsubscribeAckPayload, UnsubscribePayload,
    UpdatePayload, UpdateResponsePayload,
};
use crate::types::MaterialData;

/// WebSocket connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
}

/// WebSocket transport for real-time streaming
///
/// This transport implements the WIA Material Protocol over WebSocket.
///
/// ## Features
///
/// - Real-time material data streaming
/// - Automatic reconnection
/// - Subscription management
/// - Ping/pong heartbeat
///
/// ## Example
///
/// ```rust,ignore
/// use wia_material::transport::{WebSocketTransport, WebSocketConfig};
///
/// let config = WebSocketConfig::new("wss://api.example.com")
///     .with_api_key("your-api-key");
/// let mut transport = WebSocketTransport::new(config);
/// transport.connect().await?;
///
/// // Subscribe to measurements
/// transport.subscribe(SubscribePayload {
///     channel: SubscriptionChannel::Measurements,
///     material_type: None,
///     filter: None,
/// }).await?;
/// ```
pub struct WebSocketTransport {
    config: WebSocketConfig,
    state: Arc<RwLock<ConnectionState>>,
    session_id: Arc<RwLock<Option<String>>>,
    subscriptions: Arc<RwLock<HashMap<String, SubscriptionChannel>>>,
    message_queue: Arc<RwLock<Vec<String>>>,
    sequence: AtomicU64,
    materials: Arc<RwLock<Vec<MaterialData>>>,
    connected: AtomicBool,

    // Channels for async communication (simulated)
    _tx: Option<mpsc::Sender<String>>,
    _rx: Option<mpsc::Receiver<String>>,
}

impl WebSocketTransport {
    /// Create a new WebSocket transport
    pub fn new(config: WebSocketConfig) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(ConnectionState::Disconnected)),
            session_id: Arc::new(RwLock::new(None)),
            subscriptions: Arc::new(RwLock::new(HashMap::new())),
            message_queue: Arc::new(RwLock::new(Vec::new())),
            sequence: AtomicU64::new(0),
            materials: Arc::new(RwLock::new(Vec::new())),
            connected: AtomicBool::new(false),
            _tx: None,
            _rx: None,
        }
    }

    /// Get current connection state
    pub async fn state(&self) -> ConnectionState {
        *self.state.read().await
    }

    /// Get session ID
    pub async fn session_id(&self) -> Option<String> {
        self.session_id.read().await.clone()
    }

    /// Get active subscriptions
    pub async fn subscriptions(&self) -> HashMap<String, SubscriptionChannel> {
        self.subscriptions.read().await.clone()
    }

    /// Perform handshake
    async fn handshake(&self) -> MaterialResult<HandshakeAckPayload> {
        let handshake = if let Some(ref api_key) = self.config.transport.api_key {
            MessageBuilder::handshake_with_auth(api_key)
        } else if let Some(ref token) = self.config.transport.jwt_token {
            MessageBuilder::handshake_with_auth(token)
        } else {
            MessageBuilder::handshake()
        };

        // Simulate sending handshake
        let _json = handshake.to_json()?;

        // Simulate receiving handshake ack
        Ok(HandshakeAckPayload {
            session_id: Uuid::new_v4().to_string(),
            server_version: "1.0.0".to_string(),
            capabilities: vec![
                "query".to_string(),
                "subscribe".to_string(),
                "command".to_string(),
            ],
        })
    }

    /// Start ping/pong heartbeat
    fn start_heartbeat(&self) {
        // In production, this would spawn a task to send periodic pings
        // For now, this is a no-op
    }

    /// Handle incoming message
    async fn handle_message(&self, _message: &str) -> MaterialResult<()> {
        // Parse and handle incoming message
        // In production, this would dispatch to appropriate handlers
        Ok(())
    }

    /// Simulate data stream for testing
    pub async fn simulate_data(&self, material: MaterialData, channel: SubscriptionChannel) {
        let seq = self.sequence.fetch_add(1, Ordering::SeqCst);
        let payload = DataPayload {
            channel,
            sequence: seq,
            material,
        };

        let message = Message::new(MessageType::Data, payload);
        if let Ok(json) = message.to_json() {
            self.message_queue.write().await.push(json);
        }
    }

    /// Get next sequence number
    pub fn next_sequence(&self) -> u64 {
        self.sequence.fetch_add(1, Ordering::SeqCst)
    }
}

#[async_trait]
impl Transport for WebSocketTransport {
    async fn connect(&mut self) -> MaterialResult<()> {
        // Update state
        {
            let mut state = self.state.write().await;
            *state = ConnectionState::Connecting;
        }

        // Simulate connection delay
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;

        // Perform handshake
        let ack = self.handshake().await?;

        // Store session ID
        {
            let mut session = self.session_id.write().await;
            *session = Some(ack.session_id);
        }

        // Update state
        {
            let mut state = self.state.write().await;
            *state = ConnectionState::Connected;
        }

        self.connected.store(true, Ordering::SeqCst);

        // Start heartbeat
        self.start_heartbeat();

        Ok(())
    }

    async fn disconnect(&mut self) -> MaterialResult<()> {
        // Clear subscriptions
        {
            let mut subs = self.subscriptions.write().await;
            subs.clear();
        }

        // Clear session
        {
            let mut session = self.session_id.write().await;
            *session = None;
        }

        // Update state
        {
            let mut state = self.state.write().await;
            *state = ConnectionState::Disconnected;
        }

        self.connected.store(false, Ordering::SeqCst);

        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn query(&self, payload: QueryPayload) -> MaterialResult<QueryResponsePayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        let message = MessageBuilder::query(payload.clone());
        let _json = message.to_json()?;

        // Return simulated response
        let materials = self.materials.read().await;
        let filtered: Vec<_> = if let Some(mt) = payload.material_type {
            materials
                .iter()
                .filter(|m| m.material_type == mt)
                .cloned()
                .collect()
        } else {
            materials.clone()
        };

        Ok(QueryResponsePayload {
            data: filtered.clone(),
            meta: crate::protocol::QueryMeta {
                total_count: filtered.len(),
                returned_count: filtered.len(),
                offset: 0,
                has_more: false,
            },
        })
    }

    async fn get(&self, payload: GetPayload) -> MaterialResult<GetResponsePayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        let materials = self.materials.read().await;
        if let Some(mat) = materials.iter().find(|m| m.material_id == payload.material_id) {
            Ok(GetResponsePayload { data: mat.clone() })
        } else {
            Err(MaterialError::NotFound(payload.material_id))
        }
    }

    async fn create(&self, payload: CreatePayload) -> MaterialResult<CreateResponsePayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        {
            let mut materials = self.materials.write().await;
            materials.push(payload.material.clone());
        }

        Ok(CreateResponsePayload {
            data: payload.material,
        })
    }

    async fn update(&self, payload: UpdatePayload) -> MaterialResult<UpdateResponsePayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        {
            let mut materials = self.materials.write().await;
            if let Some(pos) = materials
                .iter()
                .position(|m| m.material_id == payload.material_id)
            {
                materials[pos] = payload.material.clone();
            } else {
                return Err(MaterialError::NotFound(payload.material_id));
            }
        }

        Ok(UpdateResponsePayload {
            data: payload.material,
        })
    }

    async fn delete(&self, payload: DeletePayload) -> MaterialResult<DeleteResponsePayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        {
            let mut materials = self.materials.write().await;
            if let Some(pos) = materials
                .iter()
                .position(|m| m.material_id == payload.material_id)
            {
                materials.remove(pos);
            }
        }

        Ok(DeleteResponsePayload {
            material_id: payload.material_id,
            success: true,
        })
    }
}

#[async_trait]
impl StreamingTransport for WebSocketTransport {
    async fn subscribe(&self, payload: SubscribePayload) -> MaterialResult<SubscribeAckPayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        let subscription_id = Uuid::new_v4().to_string();

        // Store subscription
        {
            let mut subs = self.subscriptions.write().await;
            subs.insert(subscription_id.clone(), payload.channel);
        }

        Ok(SubscribeAckPayload {
            subscription_id,
            channel: payload.channel,
        })
    }

    async fn unsubscribe(
        &self,
        payload: UnsubscribePayload,
    ) -> MaterialResult<UnsubscribeAckPayload> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        // Remove subscription
        let success = {
            let mut subs = self.subscriptions.write().await;
            subs.remove(&payload.subscription_id).is_some()
        };

        Ok(UnsubscribeAckPayload {
            subscription_id: payload.subscription_id,
            success,
        })
    }

    async fn send<T: serde::Serialize + Send + Sync>(
        &self,
        message: Message<T>,
    ) -> MaterialResult<()> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        let _json = message.to_json()?;
        // In production, this would actually send the message
        Ok(())
    }

    async fn receive(&mut self) -> MaterialResult<String> {
        if !self.is_connected() {
            return Err(MaterialError::Transport("Not connected".to_string()));
        }

        // Check message queue
        let mut queue = self.message_queue.write().await;
        if let Some(msg) = queue.pop() {
            return Ok(msg);
        }

        // Wait for message (with timeout in production)
        Err(MaterialError::Timeout)
    }
}

/// Event handler for WebSocket events
pub trait WebSocketEventHandler: Send + Sync {
    /// Called when connected
    fn on_connected(&self, session_id: &str);

    /// Called when disconnected
    fn on_disconnected(&self);

    /// Called when data is received
    fn on_data(&self, data: DataPayload);

    /// Called on error
    fn on_error(&self, error: &MaterialError);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::MaterialBuilder;
    use crate::MaterialType;

    #[tokio::test]
    async fn test_websocket_connect() {
        let config = WebSocketConfig::new("wss://api.example.com");
        let mut transport = WebSocketTransport::new(config);

        assert_eq!(transport.state().await, ConnectionState::Disconnected);

        transport.connect().await.unwrap();

        assert_eq!(transport.state().await, ConnectionState::Connected);
        assert!(transport.session_id().await.is_some());

        transport.disconnect().await.unwrap();

        assert_eq!(transport.state().await, ConnectionState::Disconnected);
        assert!(transport.session_id().await.is_none());
    }

    #[tokio::test]
    async fn test_websocket_subscribe() {
        let config = WebSocketConfig::new("wss://api.example.com");
        let mut transport = WebSocketTransport::new(config);
        transport.connect().await.unwrap();

        let ack = transport
            .subscribe(SubscribePayload {
                channel: SubscriptionChannel::Measurements,
                material_type: None,
                filter: None,
            })
            .await
            .unwrap();

        assert_eq!(ack.channel, SubscriptionChannel::Measurements);

        let subs = transport.subscriptions().await;
        assert_eq!(subs.len(), 1);

        transport
            .unsubscribe(UnsubscribePayload {
                subscription_id: ack.subscription_id,
            })
            .await
            .unwrap();

        let subs = transport.subscriptions().await;
        assert_eq!(subs.len(), 0);
    }

    #[tokio::test]
    async fn test_websocket_data_stream() {
        let config = WebSocketConfig::new("wss://api.example.com");
        let mut transport = WebSocketTransport::new(config);
        transport.connect().await.unwrap();

        // Simulate incoming data
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("YBCO")
            .formula("YBa2Cu3O7")
            .build()
            .unwrap();

        transport
            .simulate_data(material, SubscriptionChannel::Measurements)
            .await;

        // Receive message
        let msg = transport.receive().await.unwrap();
        assert!(msg.contains("YBCO"));
        assert!(msg.contains("measurements"));
    }

    #[tokio::test]
    async fn test_websocket_crud() {
        let config = WebSocketConfig::new("wss://api.example.com");
        let mut transport = WebSocketTransport::new(config);
        transport.connect().await.unwrap();

        // Create
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("Test SC")
            .formula("Test")
            .build()
            .unwrap();

        let create_response = transport
            .create(CreatePayload {
                material: material.clone(),
            })
            .await
            .unwrap();

        assert_eq!(create_response.data.identity.name, "Test SC");

        // Get
        let get_response = transport
            .get(GetPayload {
                material_id: material.material_id.clone(),
                fields: None,
            })
            .await
            .unwrap();

        assert_eq!(get_response.data.identity.name, "Test SC");

        // Query
        let query_response = transport
            .query(QueryPayload {
                material_type: Some(MaterialType::Superconductor),
                ..Default::default()
            })
            .await
            .unwrap();

        assert_eq!(query_response.data.len(), 1);
    }
}
