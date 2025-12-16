//! Real-Time Streaming Protocol
//!
//! WebSocket-based event streaming implementation.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;
use chrono::Utc;

// ============================================================================
// Stream Actions
// ============================================================================

/// WebSocket stream action
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamAction {
    Subscribe,
    Subscribed,
    Unsubscribe,
    Unsubscribed,
    Event,
    Heartbeat,
    Ping,
    Pong,
    Error,
    Close,
}

// ============================================================================
// Stream Channels
// ============================================================================

/// Available stream channels
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamChannel {
    Alerts,
    ThreatIntel,
    Vulnerabilities,
    Incidents,
    PolicyDecisions,
    SystemStatus,
    Custom(String),
}

impl StreamChannel {
    /// Get channel name as string
    pub fn as_str(&self) -> &str {
        match self {
            StreamChannel::Alerts => "alerts",
            StreamChannel::ThreatIntel => "threat_intel",
            StreamChannel::Vulnerabilities => "vulnerabilities",
            StreamChannel::Incidents => "incidents",
            StreamChannel::PolicyDecisions => "policy_decisions",
            StreamChannel::SystemStatus => "system_status",
            StreamChannel::Custom(name) => name.as_str(),
        }
    }
}

// ============================================================================
// Stream Messages
// ============================================================================

/// Base stream message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamMessage {
    /// Action type
    pub action: StreamAction,
    /// Timestamp
    pub timestamp: String,
    /// Channel (for event messages)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channel: Option<String>,
    /// Sequence number (for ordering)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence: Option<u64>,
    /// Message payload
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload: Option<serde_json::Value>,
    /// Error information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<StreamError>,
}

impl StreamMessage {
    /// Create new stream message
    pub fn new(action: StreamAction) -> Self {
        Self {
            action,
            timestamp: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            channel: None,
            sequence: None,
            payload: None,
            error: None,
        }
    }

    /// Set channel
    pub fn with_channel(mut self, channel: impl Into<String>) -> Self {
        self.channel = Some(channel.into());
        self
    }

    /// Set sequence
    pub fn with_sequence(mut self, sequence: u64) -> Self {
        self.sequence = Some(sequence);
        self
    }

    /// Set payload
    pub fn with_payload(mut self, payload: serde_json::Value) -> Self {
        self.payload = Some(payload);
        self
    }

    /// Set error
    pub fn with_error(mut self, error: StreamError) -> Self {
        self.error = Some(error);
        self
    }
}

/// Stream error
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamError {
    /// Error code
    pub code: String,
    /// Error message
    pub message: String,
    /// Retry allowed
    pub retryable: bool,
}

impl StreamError {
    /// Create authentication error
    pub fn auth_required() -> Self {
        Self {
            code: "AUTH_REQUIRED".to_string(),
            message: "Authentication required".to_string(),
            retryable: true,
        }
    }

    /// Create rate limit error
    pub fn rate_limited(retry_after_seconds: u32) -> Self {
        Self {
            code: "RATE_LIMITED".to_string(),
            message: format!("Rate limited. Retry after {} seconds", retry_after_seconds),
            retryable: true,
        }
    }

    /// Create invalid channel error
    pub fn invalid_channel(channel: &str) -> Self {
        Self {
            code: "INVALID_CHANNEL".to_string(),
            message: format!("Invalid channel: {}", channel),
            retryable: false,
        }
    }
}

// ============================================================================
// Subscription Request
// ============================================================================

/// Channel filter
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ChannelFilter {
    /// Severity filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<Vec<String>>,
    /// Categories filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub categories: Option<Vec<String>>,
    /// IOC types filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ioc_types: Option<Vec<String>>,
    /// Source types filter
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source_types: Option<Vec<String>>,
    /// Custom filters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom: Option<HashMap<String, serde_json::Value>>,
}

impl ChannelFilter {
    /// Create filter for critical and high severity
    pub fn critical_high() -> Self {
        Self {
            severity: Some(vec!["critical".to_string(), "high".to_string()]),
            ..Default::default()
        }
    }

    /// Create filter for specific categories
    pub fn for_categories(categories: Vec<String>) -> Self {
        Self {
            categories: Some(categories),
            ..Default::default()
        }
    }
}

/// Channel subscription
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelSubscription {
    /// Channel name
    pub channel: String,
    /// Filters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filters: Option<ChannelFilter>,
}

impl ChannelSubscription {
    /// Create new channel subscription
    pub fn new(channel: impl Into<String>) -> Self {
        Self {
            channel: channel.into(),
            filters: None,
        }
    }

    /// Set filters
    pub fn with_filters(mut self, filters: ChannelFilter) -> Self {
        self.filters = Some(filters);
        self
    }
}

/// Subscribe request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribeRequest {
    /// Action (should be "subscribe")
    pub action: StreamAction,
    /// Subscriptions
    pub subscriptions: Vec<ChannelSubscription>,
}

impl SubscribeRequest {
    /// Create new subscribe request
    pub fn new(subscriptions: Vec<ChannelSubscription>) -> Self {
        Self {
            action: StreamAction::Subscribe,
            subscriptions,
        }
    }

    /// Create from channel names
    pub fn channels(channels: Vec<&str>) -> Self {
        Self::new(
            channels
                .into_iter()
                .map(ChannelSubscription::new)
                .collect(),
        )
    }
}

// ============================================================================
// Subscription Response
// ============================================================================

/// Subscribed response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribedResponse {
    /// Action (should be "subscribed")
    pub action: StreamAction,
    /// Subscription ID
    pub subscription_id: String,
    /// Channels subscribed
    pub channels: Vec<String>,
    /// Connected at timestamp
    pub connected_at: String,
    /// Connection config
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<ConnectionConfig>,
}

impl SubscribedResponse {
    /// Create new subscribed response
    pub fn new(channels: Vec<String>) -> Self {
        Self {
            action: StreamAction::Subscribed,
            subscription_id: Uuid::new_v4().to_string(),
            channels,
            connected_at: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            config: Some(ConnectionConfig::default()),
        }
    }
}

/// Connection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionConfig {
    /// Heartbeat interval in seconds
    pub heartbeat_interval_seconds: u32,
    /// Message rate limit per second
    pub rate_limit_per_second: u32,
    /// Maximum message size
    pub max_message_size: u32,
}

impl Default for ConnectionConfig {
    fn default() -> Self {
        Self {
            heartbeat_interval_seconds: 30,
            rate_limit_per_second: 100,
            max_message_size: 1048576, // 1MB
        }
    }
}

// ============================================================================
// Event Message
// ============================================================================

/// Stream event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamEvent {
    /// Action (should be "event")
    pub action: StreamAction,
    /// Channel
    pub channel: String,
    /// Sequence number
    pub sequence: u64,
    /// Event timestamp
    pub timestamp: String,
    /// Event data
    pub event: EventPayload,
}

/// Event payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventPayload {
    /// Event type
    #[serde(rename = "type")]
    pub event_type: String,
    /// Severity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<u32>,
    /// Title
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Event data
    pub data: serde_json::Value,
}

impl StreamEvent {
    /// Create new event
    pub fn new(channel: impl Into<String>, sequence: u64, payload: EventPayload) -> Self {
        Self {
            action: StreamAction::Event,
            channel: channel.into(),
            sequence,
            timestamp: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            event: payload,
        }
    }
}

// ============================================================================
// Heartbeat
// ============================================================================

/// Heartbeat message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeartbeatMessage {
    /// Action (should be "heartbeat")
    pub action: StreamAction,
    /// Client timestamp
    pub timestamp: String,
    /// Server timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_time: Option<String>,
}

impl HeartbeatMessage {
    /// Create client heartbeat
    pub fn client() -> Self {
        Self {
            action: StreamAction::Heartbeat,
            timestamp: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            server_time: None,
        }
    }

    /// Create server heartbeat response
    pub fn server(client_timestamp: impl Into<String>) -> Self {
        Self {
            action: StreamAction::Heartbeat,
            timestamp: client_timestamp.into(),
            server_time: Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()),
        }
    }
}

// ============================================================================
// Close Message
// ============================================================================

/// Close reason
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CloseReason {
    ClientDisconnect,
    ServerMaintenance,
    AuthExpired,
    RateLimited,
    Error,
    Timeout,
}

/// Close message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CloseMessage {
    /// Action (should be "close")
    pub action: StreamAction,
    /// Close reason
    pub reason: CloseReason,
    /// Human-readable message
    pub message: String,
    /// Reconnect allowed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reconnect_allowed: Option<bool>,
    /// Reconnect delay (seconds)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reconnect_delay_seconds: Option<u32>,
}

impl CloseMessage {
    /// Create close message
    pub fn new(reason: CloseReason, message: impl Into<String>) -> Self {
        Self {
            action: StreamAction::Close,
            reason,
            message: message.into(),
            reconnect_allowed: Some(true),
            reconnect_delay_seconds: None,
        }
    }

    /// Create maintenance close
    pub fn maintenance(message: impl Into<String>, reconnect_after_seconds: u32) -> Self {
        Self {
            action: StreamAction::Close,
            reason: CloseReason::ServerMaintenance,
            message: message.into(),
            reconnect_allowed: Some(true),
            reconnect_delay_seconds: Some(reconnect_after_seconds),
        }
    }
}

// ============================================================================
// Stream State
// ============================================================================

/// Stream connection state
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StreamState {
    Connecting,
    Connected,
    Subscribed,
    Reconnecting,
    Disconnected,
    Error,
}

/// Stream connection info
#[derive(Debug, Clone)]
pub struct StreamConnection {
    /// Connection ID
    pub id: String,
    /// State
    pub state: StreamState,
    /// Subscribed channels
    pub channels: Vec<String>,
    /// Connected at
    pub connected_at: Option<String>,
    /// Last message at
    pub last_message_at: Option<String>,
    /// Message count
    pub message_count: u64,
}

impl StreamConnection {
    /// Create new connection
    pub fn new() -> Self {
        Self {
            id: Uuid::new_v4().to_string(),
            state: StreamState::Connecting,
            channels: vec![],
            connected_at: None,
            last_message_at: None,
            message_count: 0,
        }
    }

    /// Mark as connected
    pub fn connected(mut self) -> Self {
        self.state = StreamState::Connected;
        self.connected_at = Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string());
        self
    }

    /// Add subscribed channel
    pub fn subscribe(&mut self, channel: impl Into<String>) {
        self.channels.push(channel.into());
        self.state = StreamState::Subscribed;
    }

    /// Record message received
    pub fn message_received(&mut self) {
        self.last_message_at = Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string());
        self.message_count += 1;
    }
}

impl Default for StreamConnection {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_subscribe_request() {
        let request = SubscribeRequest::new(vec![
            ChannelSubscription::new("alerts")
                .with_filters(ChannelFilter::critical_high()),
            ChannelSubscription::new("threat_intel"),
        ]);

        assert_eq!(request.action, StreamAction::Subscribe);
        assert_eq!(request.subscriptions.len(), 2);
    }

    #[test]
    fn test_subscribed_response() {
        let response = SubscribedResponse::new(vec![
            "alerts".to_string(),
            "threat_intel".to_string(),
        ]);

        assert_eq!(response.action, StreamAction::Subscribed);
        assert_eq!(response.channels.len(), 2);
        assert!(response.config.is_some());
    }

    #[test]
    fn test_stream_event() {
        let payload = EventPayload {
            event_type: "alert".to_string(),
            severity: Some(9),
            title: Some("Critical Alert".to_string()),
            data: serde_json::json!({"alert_id": "ALERT-001"}),
        };

        let event = StreamEvent::new("alerts", 123, payload);
        assert_eq!(event.action, StreamAction::Event);
        assert_eq!(event.sequence, 123);
    }

    #[test]
    fn test_heartbeat() {
        let client_hb = HeartbeatMessage::client();
        assert_eq!(client_hb.action, StreamAction::Heartbeat);
        assert!(client_hb.server_time.is_none());

        let server_hb = HeartbeatMessage::server(&client_hb.timestamp);
        assert!(server_hb.server_time.is_some());
    }

    #[test]
    fn test_stream_connection() {
        let mut conn = StreamConnection::new().connected();
        assert_eq!(conn.state, StreamState::Connected);

        conn.subscribe("alerts");
        assert_eq!(conn.state, StreamState::Subscribed);
        assert_eq!(conn.channels.len(), 1);

        conn.message_received();
        assert_eq!(conn.message_count, 1);
    }
}
