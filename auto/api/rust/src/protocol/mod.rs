//! Communication protocol implementations for WIA Auto
//!
//! This module provides clients for the various communication protocols:
//! - REST API client
//! - WebSocket handler
//! - Message envelope handling

pub mod rest;
pub mod websocket;

pub use rest::*;
pub use websocket::*;

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

use crate::types::*;

/// WebSocket event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WsEventType {
    /// Trip status changed
    TripStatusUpdated,
    /// Vehicle location update
    VehicleLocationUpdated,
    /// ETA changed
    VehicleEtaUpdated,
    /// Wheelchair securement status
    SecurementStatusChanged,
    /// Emergency notification
    EmergencyAlert,
    /// HMI command request
    HmiCommand,
    /// HMI command response
    HmiResponse,
    /// Support chat message
    SupportMessage,
    /// Subscribe to events
    Subscribe,
    /// Unsubscribe from events
    Unsubscribe,
    /// Ping/pong for keepalive
    Ping,
    Pong,
}

/// WebSocket message wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WsMessage {
    /// Event type
    #[serde(rename = "type")]
    pub event_type: WsEventType,

    /// Unique message ID
    pub id: Uuid,

    /// Message timestamp
    pub timestamp: DateTime<Utc>,

    /// Message payload
    pub payload: serde_json::Value,

    /// Correlation ID for request-response
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<Uuid>,
}

impl WsMessage {
    /// Create a new WebSocket message
    pub fn new(event_type: WsEventType, payload: impl Serialize) -> Self {
        Self {
            event_type,
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            payload: serde_json::to_value(payload).unwrap_or(serde_json::Value::Null),
            correlation_id: None,
        }
    }

    /// Create a response message
    pub fn response(event_type: WsEventType, payload: impl Serialize, correlation_id: Uuid) -> Self {
        Self {
            event_type,
            id: Uuid::new_v4(),
            timestamp: Utc::now(),
            payload: serde_json::to_value(payload).unwrap_or(serde_json::Value::Null),
            correlation_id: Some(correlation_id),
        }
    }

    /// Create a ping message
    pub fn ping() -> Self {
        Self::new(WsEventType::Ping, serde_json::Value::Null)
    }

    /// Create a pong message
    pub fn pong(correlation_id: Uuid) -> Self {
        Self::response(WsEventType::Pong, serde_json::Value::Null, correlation_id)
    }
}

/// Trip status update payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TripStatusPayload {
    pub trip_id: Uuid,
    pub previous_status: TripStatus,
    pub new_status: TripStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eta_minutes: Option<f32>,
}

/// Vehicle location update payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleLocationPayload {
    pub vehicle_id: Uuid,
    pub location: GeoLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_degrees: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_kmh: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub eta_minutes: Option<f32>,
}

/// Subscribe request payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscribePayload {
    pub events: Vec<WsEventType>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trip_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vehicle_id: Option<Uuid>,
}

/// HMI command payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HmiCommandPayload {
    pub vehicle_id: Uuid,
    pub command: HmiCommandType,
    pub parameters: std::collections::HashMap<String, String>,
}

/// HMI command types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HmiCommandType {
    /// TTS announcement
    Announce,
    /// Display message
    Display,
    /// Haptic feedback
    Haptic,
    /// Audio chime
    Chime,
    /// Find vehicle (horn/lights/melody)
    FindVehicle,
}

/// API client configuration
#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// Base URL for REST API
    pub base_url: String,

    /// WebSocket URL
    pub ws_url: String,

    /// API key (for server-to-server)
    pub api_key: Option<String>,

    /// OAuth token (for user apps)
    pub access_token: Option<String>,

    /// Request timeout in seconds
    pub timeout_secs: u64,

    /// Maximum retries for failed requests
    pub max_retries: u32,

    /// Retry delay in milliseconds
    pub retry_delay_ms: u64,
}

impl Default for ClientConfig {
    fn default() -> Self {
        Self {
            base_url: "https://api.wia-auto.org/v1".to_string(),
            ws_url: "wss://ws.wia-auto.org/v1/stream".to_string(),
            api_key: None,
            access_token: None,
            timeout_secs: 30,
            max_retries: 3,
            retry_delay_ms: 1000,
        }
    }
}

impl ClientConfig {
    /// Create configuration for sandbox environment
    pub fn sandbox() -> Self {
        Self {
            base_url: "https://sandbox.wia-auto.org/v1".to_string(),
            ws_url: "wss://sandbox-ws.wia-auto.org/v1/stream".to_string(),
            ..Default::default()
        }
    }

    /// Create configuration for mock/testing environment
    pub fn mock() -> Self {
        Self {
            base_url: "https://mock.wia-auto.org/v1".to_string(),
            ws_url: "wss://mock-ws.wia-auto.org/v1/stream".to_string(),
            ..Default::default()
        }
    }

    /// Set API key authentication
    pub fn with_api_key(mut self, api_key: impl Into<String>) -> Self {
        self.api_key = Some(api_key.into());
        self
    }

    /// Set OAuth token authentication
    pub fn with_token(mut self, token: impl Into<String>) -> Self {
        self.access_token = Some(token.into());
        self
    }

    /// Set custom timeout
    pub fn with_timeout(mut self, secs: u64) -> Self {
        self.timeout_secs = secs;
        self
    }
}

/// Error response from API
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiError {
    pub error: ApiErrorDetails,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorDetails {
    pub code: String,
    pub message: String,
    #[serde(default)]
    pub details: Vec<ApiErrorField>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub request_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub documentation_url: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiErrorField {
    pub field: String,
    pub issue: String,
}

impl std::fmt::Display for ApiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.error.code, self.error.message)
    }
}

impl std::error::Error for ApiError {}
