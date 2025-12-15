//! WIA Gateway integration for cross-domain communication

use crate::error::{NanoError, NanoResult};
use crate::types::NanoMessage;
use async_trait::async_trait;
use serde::{Deserialize, Serialize};

/// WIA Gateway client for cross-domain messaging
pub struct WiaGateway {
    config: GatewayConfig,
    connection_state: ConnectionState,
    session_id: Option<String>,
}

impl WiaGateway {
    /// Create a new gateway client
    pub fn new(config: GatewayConfig) -> Self {
        Self {
            config,
            connection_state: ConnectionState::Disconnected,
            session_id: None,
        }
    }

    /// Connect to the WIA Gateway
    pub async fn connect(&mut self) -> NanoResult<()> {
        self.connection_state = ConnectionState::Connecting;

        // In real implementation, would establish WebSocket/gRPC connection
        self.session_id = Some(uuid::Uuid::new_v4().to_string());
        self.connection_state = ConnectionState::Connected;

        Ok(())
    }

    /// Disconnect from the gateway
    pub async fn disconnect(&mut self) -> NanoResult<()> {
        self.connection_state = ConnectionState::Disconnecting;
        self.session_id = None;
        self.connection_state = ConnectionState::Disconnected;
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        matches!(self.connection_state, ConnectionState::Connected)
    }

    /// Get session ID
    pub fn session_id(&self) -> Option<&str> {
        self.session_id.as_deref()
    }

    /// Publish a nano message to the gateway
    pub async fn publish(&self, topic: &str, message: NanoMessage) -> NanoResult<()> {
        if !self.is_connected() {
            return Err(NanoError::Communication("Not connected to gateway".into()));
        }

        let _envelope = GatewayEnvelope {
            topic: topic.to_string(),
            source_domain: "nano".to_string(),
            payload: serde_json::to_value(&message)?,
            timestamp: chrono::Utc::now().timestamp_millis() as u64,
            correlation_id: None,
        };

        // In real implementation, would send to gateway
        Ok(())
    }

    /// Subscribe to a topic
    pub async fn subscribe(&self, topic: &str) -> NanoResult<Subscription> {
        if !self.is_connected() {
            return Err(NanoError::Communication("Not connected to gateway".into()));
        }

        Ok(Subscription {
            topic: topic.to_string(),
            subscription_id: uuid::Uuid::new_v4().to_string(),
        })
    }

    /// Send a request and wait for response
    pub async fn request(&self, target_domain: &str, request: GatewayRequest) -> NanoResult<GatewayResponse> {
        if !self.is_connected() {
            return Err(NanoError::Communication("Not connected to gateway".into()));
        }

        // In real implementation, would send request and await response
        Ok(GatewayResponse {
            correlation_id: request.correlation_id,
            source_domain: target_domain.to_string(),
            status: ResponseStatus::Success,
            payload: serde_json::json!({}),
            error: None,
        })
    }

    /// Register this nano system with the gateway
    pub async fn register_system(&self, registration: SystemRegistration) -> NanoResult<String> {
        if !self.is_connected() {
            return Err(NanoError::Communication("Not connected to gateway".into()));
        }

        // Return assigned system ID
        Ok(format!("nano-{}", uuid::Uuid::new_v4()))
    }

    /// Send heartbeat to gateway
    pub async fn heartbeat(&self) -> NanoResult<()> {
        if !self.is_connected() {
            return Err(NanoError::Communication("Not connected to gateway".into()));
        }

        // In real implementation, would send heartbeat
        Ok(())
    }
}

/// Gateway configuration
#[derive(Debug, Clone)]
pub struct GatewayConfig {
    /// Gateway URL
    pub url: String,
    /// API key for authentication
    pub api_key: Option<String>,
    /// Connection timeout in milliseconds
    pub timeout_ms: u64,
    /// Reconnect automatically
    pub auto_reconnect: bool,
    /// Reconnect interval in milliseconds
    pub reconnect_interval_ms: u64,
    /// Enable TLS
    pub tls_enabled: bool,
}

impl Default for GatewayConfig {
    fn default() -> Self {
        Self {
            url: "wss://gateway.wia.live/v1".to_string(),
            api_key: None,
            timeout_ms: 30000,
            auto_reconnect: true,
            reconnect_interval_ms: 5000,
            tls_enabled: true,
        }
    }
}

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Disconnecting,
    Reconnecting,
    Failed,
}

/// Gateway message envelope
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GatewayEnvelope {
    pub topic: String,
    pub source_domain: String,
    pub payload: serde_json::Value,
    pub timestamp: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<String>,
}

/// Topic subscription
#[derive(Debug, Clone)]
pub struct Subscription {
    pub topic: String,
    pub subscription_id: String,
}

/// Gateway request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GatewayRequest {
    pub correlation_id: String,
    pub method: String,
    pub target_domain: String,
    pub payload: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_ms: Option<u64>,
}

impl GatewayRequest {
    pub fn new(method: impl Into<String>, target_domain: impl Into<String>) -> Self {
        Self {
            correlation_id: uuid::Uuid::new_v4().to_string(),
            method: method.into(),
            target_domain: target_domain.into(),
            payload: serde_json::json!({}),
            timeout_ms: None,
        }
    }

    pub fn with_payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = payload;
        self
    }

    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = Some(timeout_ms);
        self
    }
}

/// Gateway response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GatewayResponse {
    pub correlation_id: String,
    pub source_domain: String,
    pub status: ResponseStatus,
    pub payload: serde_json::Value,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<GatewayError>,
}

/// Response status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseStatus {
    Success,
    Error,
    Timeout,
    NotFound,
    Unauthorized,
}

/// Gateway error
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GatewayError {
    pub code: u32,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
}

/// System registration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemRegistration {
    pub system_type: String,
    pub capabilities: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
    pub protocol_version: String,
}

impl SystemRegistration {
    pub fn new(system_type: impl Into<String>) -> Self {
        Self {
            system_type: system_type.into(),
            capabilities: Vec::new(),
            metadata: None,
            protocol_version: "1.0.0".to_string(),
        }
    }

    pub fn with_capabilities(mut self, capabilities: Vec<String>) -> Self {
        self.capabilities = capabilities;
        self
    }

    pub fn with_metadata(mut self, metadata: serde_json::Value) -> Self {
        self.metadata = Some(metadata);
        self
    }
}

/// Gateway event handler trait
#[async_trait]
pub trait GatewayEventHandler: Send + Sync {
    /// Handle incoming message
    async fn on_message(&self, envelope: GatewayEnvelope);

    /// Handle connection established
    async fn on_connected(&self);

    /// Handle disconnection
    async fn on_disconnected(&self);

    /// Handle error
    async fn on_error(&self, error: GatewayError);
}
