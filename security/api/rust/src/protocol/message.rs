//! Protocol Message Types
//!
//! Core message structures for the WIA Security Communication Protocol.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

use super::PROTOCOL_VERSION;

// ============================================================================
// Message Types
// ============================================================================

/// Message type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Request,
    Response,
    Event,
    Notification,
}

/// Component type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComponentType {
    Scanner,
    Siem,
    Pdp,
    Pep,
    ThreatIntel,
    Endpoint,
    Firewall,
    Ids,
    Edr,
    Soar,
    Gateway,
    ApiServer,
    Custom,
}

/// Encryption type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EncryptionType {
    #[serde(rename = "TLS1.3")]
    Tls13,
    #[serde(rename = "PQC_HYBRID")]
    PqcHybrid,
    #[serde(rename = "NONE")]
    None,
}

/// Signature algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignatureAlgorithm {
    Ed25519,
    Dilithium3,
    #[serde(rename = "ECDSA_P256")]
    EcdsaP256,
    #[serde(rename = "RSA_PSS")]
    RsaPss,
}

/// Content type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentType {
    #[serde(rename = "application/json")]
    Json,
    #[serde(rename = "application/stix+json")]
    StixJson,
    #[serde(rename = "application/taxii+json")]
    TaxiiJson,
    #[serde(rename = "application/octet-stream")]
    OctetStream,
}

/// Priority level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Priority {
    Critical,
    High,
    Medium,
    Low,
}

// ============================================================================
// Component
// ============================================================================

/// Security component information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Component {
    /// Component unique identifier
    pub id: String,
    /// Component type
    #[serde(rename = "type")]
    pub component_type: ComponentType,
    /// Human-readable name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// TLS certificate fingerprint (SHA-256)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certificate_fingerprint: Option<String>,
}

impl Component {
    /// Create a new component
    pub fn new(id: impl Into<String>, component_type: ComponentType) -> Self {
        Self {
            id: id.into(),
            component_type,
            name: None,
            certificate_fingerprint: None,
        }
    }

    /// Set component name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set certificate fingerprint
    pub fn with_fingerprint(mut self, fingerprint: impl Into<String>) -> Self {
        self.certificate_fingerprint = Some(fingerprint.into());
        self
    }
}

// ============================================================================
// Security Info
// ============================================================================

/// Security information for message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecurityInfo {
    /// Encryption type used
    pub encryption: EncryptionType,
    /// Signature algorithm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature_algorithm: Option<SignatureAlgorithm>,
    /// Base64-encoded signature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<String>,
    /// Nonce for replay protection
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nonce: Option<String>,
    /// Certificate chain
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certificate_chain: Option<Vec<String>>,
}

impl SecurityInfo {
    /// Create security info with encryption type
    pub fn new(encryption: EncryptionType) -> Self {
        Self {
            encryption,
            signature_algorithm: None,
            signature: None,
            nonce: None,
            certificate_chain: None,
        }
    }

    /// Set signature
    pub fn with_signature(
        mut self,
        algorithm: SignatureAlgorithm,
        signature: impl Into<String>,
    ) -> Self {
        self.signature_algorithm = Some(algorithm);
        self.signature = Some(signature.into());
        self
    }

    /// Set nonce
    pub fn with_nonce(mut self, nonce: impl Into<String>) -> Self {
        self.nonce = Some(nonce.into());
        self
    }
}

impl Default for SecurityInfo {
    fn default() -> Self {
        Self::new(EncryptionType::Tls13)
    }
}

// ============================================================================
// Payload
// ============================================================================

/// Message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Payload<T = serde_json::Value> {
    /// Content type
    pub content_type: ContentType,
    /// Character encoding
    #[serde(skip_serializing_if = "Option::is_none")]
    pub encoding: Option<String>,
    /// Payload data
    pub data: T,
    /// Whether payload is compressed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compressed: Option<bool>,
    /// Compression algorithm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compression_algorithm: Option<String>,
}

impl<T> Payload<T> {
    /// Create a new payload
    pub fn new(content_type: ContentType, data: T) -> Self {
        Self {
            content_type,
            encoding: Some("utf-8".to_string()),
            data,
            compressed: None,
            compression_algorithm: None,
        }
    }
}

// ============================================================================
// Metadata
// ============================================================================

/// Message metadata
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Metadata {
    /// Correlation ID for request/response pairing
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation_id: Option<Uuid>,
    /// Causation ID (parent message)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub causation_id: Option<Uuid>,
    /// Message priority
    #[serde(skip_serializing_if = "Option::is_none")]
    pub priority: Option<Priority>,
    /// Time-to-live in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ttl_seconds: Option<u32>,
    /// Distributed tracing ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trace_id: Option<String>,
    /// Span ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub span_id: Option<String>,
    /// Whether acknowledgment is required
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_acknowledgment: Option<bool>,
    /// Processing time in milliseconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub processing_time_ms: Option<u32>,
}

impl Metadata {
    /// Create new metadata with correlation ID
    pub fn with_correlation(id: Uuid) -> Self {
        Self {
            correlation_id: Some(id),
            ..Default::default()
        }
    }

    /// Set priority
    pub fn with_priority(mut self, priority: Priority) -> Self {
        self.priority = Some(priority);
        self
    }

    /// Set TTL
    pub fn with_ttl(mut self, seconds: u32) -> Self {
        self.ttl_seconds = Some(seconds);
        self
    }
}

// ============================================================================
// Protocol Message
// ============================================================================

/// Base protocol message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtocolMessage<T = serde_json::Value> {
    /// Schema URL
    #[serde(rename = "$schema", skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,
    /// Protocol version
    pub protocol_version: String,
    /// Unique message ID
    pub message_id: Uuid,
    /// Message timestamp
    pub timestamp: String,
    /// Message type
    pub message_type: MessageType,
    /// Sender component
    pub sender: Component,
    /// Receiver component
    pub receiver: Component,
    /// Security information
    pub security: SecurityInfo,
    /// Message payload
    pub payload: Payload<T>,
    /// Message metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<Metadata>,
}

impl<T: Serialize> ProtocolMessage<T> {
    /// Create a new protocol message
    pub fn new(
        message_type: MessageType,
        sender: Component,
        receiver: Component,
        payload: Payload<T>,
    ) -> Self {
        Self {
            schema: Some(super::MESSAGE_SCHEMA_URL.to_string()),
            protocol_version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4(),
            timestamp: Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            message_type,
            sender,
            receiver,
            security: SecurityInfo::default(),
            payload,
            metadata: None,
        }
    }

    /// Set security info
    pub fn with_security(mut self, security: SecurityInfo) -> Self {
        self.security = security;
        self
    }

    /// Set metadata
    pub fn with_metadata(mut self, metadata: Metadata) -> Self {
        self.metadata = Some(metadata);
        self
    }

    /// Convert to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

impl<T: for<'de> Deserialize<'de>> ProtocolMessage<T> {
    /// Parse from JSON string
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }
}

// ============================================================================
// Request Types
// ============================================================================

/// Request type enumeration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestType {
    AccessDecision,
    ScanStart,
    ScanStop,
    ThreatQuery,
    IocLookup,
    PolicyUpdate,
    Subscribe,
    Unsubscribe,
    HealthCheck,
    Discovery,
    Collections,
    Objects,
    Inference,
    Custom,
}

/// Request payload data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RequestData {
    /// Request type
    pub request_type: RequestType,
    /// Request-specific parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<serde_json::Value>,
    /// Request timeout
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_ms: Option<u32>,
}

impl RequestData {
    /// Create a new request data
    pub fn new(request_type: RequestType) -> Self {
        Self {
            request_type,
            parameters: None,
            timeout_ms: None,
        }
    }

    /// Set parameters
    pub fn with_parameters(mut self, params: serde_json::Value) -> Self {
        self.parameters = Some(params);
        self
    }
}

// ============================================================================
// Response Types
// ============================================================================

/// Response status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseStatus {
    Success,
    Error,
    Partial,
    Pending,
}

/// Error information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorInfo {
    /// Error code (E1xxx-E5xxx)
    pub code: String,
    /// Error name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Human-readable message
    pub message: String,
    /// Additional details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    /// Retry after seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after: Option<u32>,
    /// Help URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub help_url: Option<String>,
}

impl ErrorInfo {
    /// Create a new error
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            code: code.into(),
            name: None,
            message: message.into(),
            details: None,
            retry_after: None,
            help_url: None,
        }
    }

    /// Common error codes
    pub fn invalid_message(message: impl Into<String>) -> Self {
        Self::new("E1001", message)
    }

    pub fn invalid_signature(message: impl Into<String>) -> Self {
        Self::new("E1002", message)
    }

    pub fn auth_required() -> Self {
        Self::new("E2001", "Authentication required")
    }

    pub fn auth_failed(message: impl Into<String>) -> Self {
        Self::new("E2002", message)
    }

    pub fn rate_limited(retry_after: u32) -> Self {
        Self {
            code: "E3002".to_string(),
            name: Some("RATE_LIMITED".to_string()),
            message: "Rate limit exceeded".to_string(),
            details: None,
            retry_after: Some(retry_after),
            help_url: None,
        }
    }
}

/// Response payload data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseData {
    /// Response status
    pub status: ResponseStatus,
    /// Result data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<serde_json::Value>,
    /// Error information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorInfo>,
}

impl ResponseData {
    /// Create success response
    pub fn success(result: serde_json::Value) -> Self {
        Self {
            status: ResponseStatus::Success,
            result: Some(result),
            error: None,
        }
    }

    /// Create error response
    pub fn error(error: ErrorInfo) -> Self {
        Self {
            status: ResponseStatus::Error,
            result: None,
            error: Some(error),
        }
    }
}

// ============================================================================
// Event Types
// ============================================================================

/// Event type enumeration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProtocolEventType {
    VulnerabilityFound,
    ThreatDetected,
    PolicyViolation,
    AccessDenied,
    ScanStarted,
    ScanCompleted,
    IncidentCreated,
    IncidentUpdated,
    IocMatched,
    AnomalyDetected,
    ConfigChanged,
    ConnectionEstablished,
    ConnectionClosed,
    Heartbeat,
    Custom,
}

/// Event payload data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EventData {
    /// Event type
    pub event_type: ProtocolEventType,
    /// Event ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_id: Option<String>,
    /// When the event occurred
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_time: Option<String>,
    /// Event-specific data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub event_data: Option<serde_json::Value>,
}

impl EventData {
    /// Create a new event
    pub fn new(event_type: ProtocolEventType) -> Self {
        Self {
            event_type,
            event_id: Some(Uuid::new_v4().to_string()),
            event_time: Some(Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()),
            event_data: None,
        }
    }

    /// Set event data
    pub fn with_data(mut self, data: serde_json::Value) -> Self {
        self.event_data = Some(data);
        self
    }
}

// ============================================================================
// Notification Types
// ============================================================================

/// Notification type enumeration
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationType {
    SystemStatus,
    Heartbeat,
    ConfigChange,
    Maintenance,
    RateLimitWarning,
    CertificateExpiry,
    SubscriptionStatus,
    Acknowledgment,
    Custom,
}

/// Notification payload data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationData {
    /// Notification type
    pub notification_type: NotificationType,
    /// Notification ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notification_id: Option<String>,
    /// Notification-specific data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notification_data: Option<serde_json::Value>,
}

impl NotificationData {
    /// Create a new notification
    pub fn new(notification_type: NotificationType) -> Self {
        Self {
            notification_type,
            notification_id: Some(Uuid::new_v4().to_string()),
            notification_data: None,
        }
    }

    /// Set notification data
    pub fn with_data(mut self, data: serde_json::Value) -> Self {
        self.notification_data = Some(data);
        self
    }
}

/// Heartbeat notification data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeartbeatData {
    /// Health status
    pub status: String,
    /// Uptime in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uptime_seconds: Option<u64>,
    /// Last activity time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_activity: Option<String>,
    /// Metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<HeartbeatMetrics>,
}

/// Heartbeat metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeartbeatMetrics {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpu_percent: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory_percent: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub connections_active: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub messages_processed: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub errors_count: Option<u64>,
}

// ============================================================================
// Message Builders
// ============================================================================

/// Helper to create request messages
pub fn create_request(
    sender: Component,
    receiver: Component,
    request: RequestData,
) -> ProtocolMessage<RequestData> {
    ProtocolMessage::new(
        MessageType::Request,
        sender,
        receiver,
        Payload::new(ContentType::Json, request),
    )
}

/// Helper to create response messages
pub fn create_response(
    sender: Component,
    receiver: Component,
    response: ResponseData,
    correlation_id: Uuid,
) -> ProtocolMessage<ResponseData> {
    ProtocolMessage::new(
        MessageType::Response,
        sender,
        receiver,
        Payload::new(ContentType::Json, response),
    )
    .with_metadata(Metadata::with_correlation(correlation_id))
}

/// Helper to create event messages
pub fn create_event(
    sender: Component,
    receiver: Component,
    event: EventData,
    priority: Priority,
) -> ProtocolMessage<EventData> {
    ProtocolMessage::new(
        MessageType::Event,
        sender,
        receiver,
        Payload::new(ContentType::Json, event),
    )
    .with_metadata(Metadata::default().with_priority(priority))
}

/// Helper to create notification messages
pub fn create_notification(
    sender: Component,
    receiver: Component,
    notification: NotificationData,
) -> ProtocolMessage<NotificationData> {
    ProtocolMessage::new(
        MessageType::Notification,
        sender,
        receiver,
        Payload::new(ContentType::Json, notification),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_request() {
        let sender = Component::new("scanner-001", ComponentType::Scanner);
        let receiver = Component::new("siem-central", ComponentType::Siem);
        let request = RequestData::new(RequestType::ScanStart);

        let msg = create_request(sender, receiver, request);
        assert_eq!(msg.message_type, MessageType::Request);
        assert_eq!(msg.payload.data.request_type, RequestType::ScanStart);
    }

    #[test]
    fn test_create_response() {
        let sender = Component::new("siem-central", ComponentType::Siem);
        let receiver = Component::new("scanner-001", ComponentType::Scanner);
        let response = ResponseData::success(serde_json::json!({"status": "ok"}));
        let correlation_id = Uuid::new_v4();

        let msg = create_response(sender, receiver, response, correlation_id);
        assert_eq!(msg.message_type, MessageType::Response);
        assert_eq!(msg.payload.data.status, ResponseStatus::Success);
    }

    #[test]
    fn test_message_serialization() {
        let sender = Component::new("test-sender", ComponentType::ApiServer);
        let receiver = Component::new("test-receiver", ComponentType::Siem);
        let request = RequestData::new(RequestType::HealthCheck);

        let msg = create_request(sender, receiver, request);
        let json = msg.to_json().unwrap();

        assert!(json.contains("protocol_version"));
        assert!(json.contains("message_id"));
        assert!(json.contains("health_check"));
    }
}
