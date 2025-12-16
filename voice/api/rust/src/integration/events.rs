//! WIA event bus integration

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use super::wia_bridge::WiaSystem;

/// CloudEvents specification version
pub const CLOUDEVENTS_SPEC_VERSION: &str = "1.0";

/// WIA events version
pub const WIA_EVENTS_VERSION: &str = "1.0.0";

/// CloudEvent wrapper for WIA events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaCloudEvent<T> {
    /// CloudEvents spec version
    pub specversion: String,

    /// Event type
    #[serde(rename = "type")]
    pub event_type: String,

    /// Event source
    pub source: String,

    /// Event ID
    pub id: String,

    /// Event time
    pub time: DateTime<Utc>,

    /// Data content type
    pub datacontenttype: String,

    /// Subject
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subject: Option<String>,

    /// WIA version
    pub wiaversion: String,

    /// Correlation ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wiacorrelationid: Option<String>,

    /// Event data
    pub data: T,
}

impl<T: Serialize> WiaCloudEvent<T> {
    /// Create a new WIA cloud event
    pub fn new(event_type: impl Into<String>, source: WiaSystem, data: T) -> Self {
        Self {
            specversion: CLOUDEVENTS_SPEC_VERSION.to_string(),
            event_type: event_type.into(),
            source: format!("/wia/{}/api/v1", source),
            id: format!("evt-{}", uuid::Uuid::new_v4()),
            time: Utc::now(),
            datacontenttype: "application/json".to_string(),
            subject: None,
            wiaversion: WIA_EVENTS_VERSION.to_string(),
            wiacorrelationid: None,
            data,
        }
    }

    /// Set subject
    pub fn with_subject(mut self, subject: impl Into<String>) -> Self {
        self.subject = Some(subject.into());
        self
    }

    /// Set correlation ID
    pub fn with_correlation_id(mut self, id: impl Into<String>) -> Self {
        self.wiacorrelationid = Some(id.into());
        self
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }
}

/// Voice-Sign event types
pub mod voice_sign_events {
    use super::*;

    /// Translation requested event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct TranslationRequestedData {
        pub request_id: String,
        pub source_language: String,
        pub target_language: String,
        pub input_type: InputType,
        pub priority: Priority,
    }

    /// Translation completed event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct TranslationCompletedData {
        pub request_id: String,
        pub source_language: String,
        pub target_language: String,
        pub status: TranslationStatus,
        pub quality_score: f32,
        pub gloss_sequence: Vec<String>,
        pub pose_available: bool,
        pub render_available: bool,
        pub duration_ms: u64,
    }

    /// Emergency detected event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct EmergencyDetectedData {
        pub request_id: String,
        pub urgency_level: UrgencyLevel,
        pub keywords: Vec<String>,
        pub action: String,
        pub requires_immediate_response: bool,
    }

    /// Quality alert event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct QualityAlertData {
        pub alert_type: QualityAlertType,
        pub current_score: f32,
        pub threshold: f32,
        pub affected_languages: Vec<String>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "snake_case")]
    pub enum InputType {
        Audio,
        Text,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "lowercase")]
    pub enum Priority {
        Normal,
        High,
        Emergency,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "lowercase")]
    pub enum TranslationStatus {
        Success,
        Partial,
        Failed,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "lowercase")]
    pub enum UrgencyLevel {
        Low,
        Medium,
        High,
        Critical,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "snake_case")]
    pub enum QualityAlertType {
        LowScore,
        HighFallbackRate,
        ModelDegradation,
    }

    /// Event type constants
    pub const TRANSLATION_REQUESTED: &str = "wia.voice-sign.translation.requested";
    pub const TRANSLATION_COMPLETED: &str = "wia.voice-sign.translation.completed";
    pub const EMERGENCY_DETECTED: &str = "wia.voice-sign.emergency.detected";
    pub const QUALITY_ALERT: &str = "wia.voice-sign.quality.alert";
}

/// Integration event types
pub mod integration_events {
    use super::*;
    use crate::types::PoseData;

    /// Sign gesture command event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct SignGestureCommandData {
        pub source: String,
        pub target: String,
        pub gesture: GestureData,
        pub user_consent: bool,
        pub assistance_level: AssistanceLevel,
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct GestureData {
        pub gesture_type: String,
        pub gloss: String,
        pub pose_data: Option<PoseData>,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "lowercase")]
    pub enum AssistanceLevel {
        Full,
        Partial,
        Guide,
    }

    /// Visual display request event
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct VisualDisplayRequestData {
        pub source: String,
        pub target: String,
        pub display: DisplayData,
        pub priority: DisplayPriority,
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct DisplayData {
        pub display_type: String,
        pub content: serde_json::Value,
        pub position: String,
        pub duration_ms: u32,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    #[serde(rename_all = "lowercase")]
    pub enum DisplayPriority {
        Normal,
        Urgent,
    }

    /// Event type constants
    pub const SIGN_GESTURE_COMMAND: &str = "wia.integration.sign-gesture.command";
    pub const VISUAL_DISPLAY_REQUEST: &str = "wia.integration.visual-display.request";
}

/// Event publisher trait
#[async_trait::async_trait]
pub trait EventPublisher: Send + Sync {
    /// Publish an event
    async fn publish<T: Serialize + Send + Sync>(
        &self,
        event: &WiaCloudEvent<T>,
    ) -> Result<(), EventError>;

    /// Publish to specific topic
    async fn publish_to<T: Serialize + Send + Sync>(
        &self,
        topic: &str,
        event: &WiaCloudEvent<T>,
    ) -> Result<(), EventError>;
}

/// Event subscriber trait
#[async_trait::async_trait]
pub trait EventSubscriber: Send + Sync {
    /// Subscribe to events
    async fn subscribe(&self, event_types: &[&str]) -> Result<(), EventError>;

    /// Receive next event
    async fn receive(&self) -> Result<serde_json::Value, EventError>;
}

/// Event error
#[derive(Debug, thiserror::Error)]
pub enum EventError {
    #[error("Connection error: {0}")]
    ConnectionError(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Publish failed: {0}")]
    PublishFailed(String),

    #[error("Subscribe failed: {0}")]
    SubscribeFailed(String),

    #[error("Timeout")]
    Timeout,
}

/// In-memory event bus (for testing/development)
pub struct InMemoryEventBus {
    events: std::sync::RwLock<Vec<String>>,
}

impl InMemoryEventBus {
    pub fn new() -> Self {
        Self {
            events: std::sync::RwLock::new(Vec::new()),
        }
    }

    pub fn events(&self) -> Vec<String> {
        self.events.read().unwrap().clone()
    }
}

impl Default for InMemoryEventBus {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait::async_trait]
impl EventPublisher for InMemoryEventBus {
    async fn publish<T: Serialize + Send + Sync>(
        &self,
        event: &WiaCloudEvent<T>,
    ) -> Result<(), EventError> {
        let json = serde_json::to_string(event)
            .map_err(|e| EventError::SerializationError(e.to_string()))?;

        self.events.write().unwrap().push(json);
        Ok(())
    }

    async fn publish_to<T: Serialize + Send + Sync>(
        &self,
        _topic: &str,
        event: &WiaCloudEvent<T>,
    ) -> Result<(), EventError> {
        self.publish(event).await
    }
}

/// Event builder helpers
pub struct VoiceSignEvents;

impl VoiceSignEvents {
    /// Create translation requested event
    pub fn translation_requested(
        request_id: &str,
        source_language: &str,
        target_language: &str,
        input_type: voice_sign_events::InputType,
        priority: voice_sign_events::Priority,
    ) -> WiaCloudEvent<voice_sign_events::TranslationRequestedData> {
        WiaCloudEvent::new(
            voice_sign_events::TRANSLATION_REQUESTED,
            WiaSystem::VoiceSign,
            voice_sign_events::TranslationRequestedData {
                request_id: request_id.to_string(),
                source_language: source_language.to_string(),
                target_language: target_language.to_string(),
                input_type,
                priority,
            },
        )
        .with_subject(format!("translation/{}", request_id))
    }

    /// Create translation completed event
    pub fn translation_completed(
        request_id: &str,
        source_language: &str,
        target_language: &str,
        status: voice_sign_events::TranslationStatus,
        quality_score: f32,
        gloss_sequence: Vec<String>,
        duration_ms: u64,
    ) -> WiaCloudEvent<voice_sign_events::TranslationCompletedData> {
        WiaCloudEvent::new(
            voice_sign_events::TRANSLATION_COMPLETED,
            WiaSystem::VoiceSign,
            voice_sign_events::TranslationCompletedData {
                request_id: request_id.to_string(),
                source_language: source_language.to_string(),
                target_language: target_language.to_string(),
                status,
                quality_score,
                gloss_sequence,
                pose_available: true,
                render_available: false,
                duration_ms,
            },
        )
        .with_subject(format!("translation/{}", request_id))
    }

    /// Create emergency detected event
    pub fn emergency_detected(
        request_id: &str,
        urgency_level: voice_sign_events::UrgencyLevel,
        keywords: Vec<String>,
        action: &str,
    ) -> WiaCloudEvent<voice_sign_events::EmergencyDetectedData> {
        WiaCloudEvent::new(
            voice_sign_events::EMERGENCY_DETECTED,
            WiaSystem::VoiceSign,
            voice_sign_events::EmergencyDetectedData {
                request_id: request_id.to_string(),
                urgency_level,
                keywords,
                action: action.to_string(),
                requires_immediate_response: matches!(
                    urgency_level,
                    voice_sign_events::UrgencyLevel::Critical
                ),
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cloud_event_creation() {
        let event = VoiceSignEvents::translation_requested(
            "req-001",
            "en",
            "ASL",
            voice_sign_events::InputType::Audio,
            voice_sign_events::Priority::Normal,
        );

        assert_eq!(event.specversion, "1.0");
        assert_eq!(event.event_type, voice_sign_events::TRANSLATION_REQUESTED);
        assert!(event.source.contains("voice-sign"));
    }

    #[test]
    fn test_event_serialization() {
        let event = VoiceSignEvents::translation_completed(
            "req-001",
            "en",
            "ASL",
            voice_sign_events::TranslationStatus::Success,
            0.95,
            vec!["HELLO".to_string(), "WORLD".to_string()],
            250,
        );

        let json = event.to_json().unwrap();
        assert!(json.contains("translation.completed"));
        assert!(json.contains("HELLO"));
    }

    #[tokio::test]
    async fn test_in_memory_event_bus() {
        let bus = InMemoryEventBus::new();

        let event = VoiceSignEvents::translation_requested(
            "req-001",
            "en",
            "ASL",
            voice_sign_events::InputType::Text,
            voice_sign_events::Priority::Normal,
        );

        bus.publish(&event).await.unwrap();
        assert_eq!(bus.events().len(), 1);
    }
}
