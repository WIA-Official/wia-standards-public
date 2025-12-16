//! Accessibility Event Types
//! 弘益人間 - Event definitions for accessibility adaptations

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};
use std::collections::HashMap;

use crate::types::ContentType;

/// Accessibility event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AccessibilityEvent {
    /// Event ID
    pub event_id: Uuid,
    /// Event type
    pub event_type: EventType,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
    /// Source platform/system
    pub source: String,
    /// Target learner profile ID
    pub profile_id: Option<Uuid>,
    /// Event payload
    pub payload: EventPayload,
    /// Event metadata
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub metadata: HashMap<String, serde_json::Value>,
}

impl AccessibilityEvent {
    /// Create a new event
    pub fn new(event_type: EventType, source: &str, payload: EventPayload) -> Self {
        Self {
            event_id: Uuid::new_v4(),
            event_type,
            timestamp: Utc::now(),
            source: source.to_string(),
            profile_id: None,
            payload,
            metadata: HashMap::new(),
        }
    }

    /// Set profile ID
    pub fn with_profile(mut self, profile_id: Uuid) -> Self {
        self.profile_id = Some(profile_id);
        self
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: &str, value: serde_json::Value) -> Self {
        self.metadata.insert(key.to_string(), value);
        self
    }

    /// Check if event is for a specific profile
    pub fn is_for_profile(&self, profile_id: &Uuid) -> bool {
        self.profile_id.as_ref() == Some(profile_id)
    }
}

/// Event type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventType {
    /// Adaptation request from learner/system
    AdaptationRequest,
    /// Adaptation response (granted/denied)
    AdaptationResponse,
    /// Setting change event
    SettingChange,
    /// Content alternative request
    ContentAlternativeRequest,
    /// Content alternative available
    ContentAlternativeAvailable,
    /// Profile sync event
    ProfileSync,
    /// Accommodation applied
    AccommodationApplied,
    /// Accessibility barrier encountered
    BarrierEncountered,
    /// Feature enabled
    FeatureEnabled,
    /// Feature disabled
    FeatureDisabled,
    /// Session started
    SessionStart,
    /// Session ended
    SessionEnd,
}

/// Event payload (varies by event type)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum EventPayload {
    /// Adaptation request payload
    Adaptation(AdaptationRequest),
    /// Adaptation response payload
    AdaptationResp(AdaptationResponse),
    /// Setting change payload
    Setting(SettingChangeEvent),
    /// Content alternative payload
    ContentAlternative(ContentAlternativeRequest),
    /// Generic payload
    Generic(serde_json::Value),
}

/// Adaptation request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptationRequest {
    /// Request ID
    pub request_id: Uuid,
    /// Adaptation type requested
    pub adaptation_type: AdaptationType,
    /// Target content/resource ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_id: Option<String>,
    /// Urgency level
    pub urgency: UrgencyLevel,
    /// Specific parameters for the adaptation
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub parameters: HashMap<String, serde_json::Value>,
    /// Reason for the request
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl AdaptationRequest {
    /// Create a new adaptation request
    pub fn new(adaptation_type: AdaptationType) -> Self {
        Self {
            request_id: Uuid::new_v4(),
            adaptation_type,
            target_id: None,
            urgency: UrgencyLevel::Normal,
            parameters: HashMap::new(),
            reason: None,
        }
    }

    /// Set target
    pub fn with_target(mut self, target_id: &str) -> Self {
        self.target_id = Some(target_id.to_string());
        self
    }

    /// Set urgency
    pub fn with_urgency(mut self, urgency: UrgencyLevel) -> Self {
        self.urgency = urgency;
        self
    }

    /// Add parameter
    pub fn with_parameter(mut self, key: &str, value: serde_json::Value) -> Self {
        self.parameters.insert(key.to_string(), value);
        self
    }

    /// Set reason
    pub fn with_reason(mut self, reason: &str) -> Self {
        self.reason = Some(reason.to_string());
        self
    }
}

/// Adaptation type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptationType {
    /// Request extended time
    ExtendedTime,
    /// Request break
    Break,
    /// Request content alternative
    ContentAlternative,
    /// Request display adjustment
    DisplayAdjustment,
    /// Request input method change
    InputMethodChange,
    /// Request assistive technology support
    AssistiveTechSupport,
    /// Request caption
    Caption,
    /// Request audio description
    AudioDescription,
    /// Request sign language
    SignLanguage,
    /// Request transcript
    Transcript,
    /// Request simplified content
    SimplifiedContent,
    /// Request larger text
    LargerText,
    /// Request higher contrast
    HighContrast,
    /// Request reduced motion
    ReducedMotion,
    /// Custom adaptation
    Custom,
}

/// Urgency level for requests
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum UrgencyLevel {
    /// Low urgency - can be handled when convenient
    Low,
    /// Normal urgency
    Normal,
    /// High urgency - needs attention soon
    High,
    /// Critical - immediate attention required
    Critical,
}

/// Adaptation response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptationResponse {
    /// Original request ID
    pub request_id: Uuid,
    /// Response status
    pub status: AdaptationStatus,
    /// Adaptation type that was requested
    pub adaptation_type: AdaptationType,
    /// Response message
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message: Option<String>,
    /// Applied adaptation details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub applied: Option<AppliedAdaptation>,
    /// Alternatives if request couldn't be fulfilled
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub alternatives: Vec<AdaptationAlternative>,
}

impl AdaptationResponse {
    /// Create a granted response
    pub fn granted(request_id: Uuid, adaptation_type: AdaptationType, applied: AppliedAdaptation) -> Self {
        Self {
            request_id,
            status: AdaptationStatus::Granted,
            adaptation_type,
            message: Some("Adaptation applied successfully".to_string()),
            applied: Some(applied),
            alternatives: vec![],
        }
    }

    /// Create a denied response
    pub fn denied(request_id: Uuid, adaptation_type: AdaptationType, reason: &str) -> Self {
        Self {
            request_id,
            status: AdaptationStatus::Denied,
            adaptation_type,
            message: Some(reason.to_string()),
            applied: None,
            alternatives: vec![],
        }
    }

    /// Create a partial response
    pub fn partial(request_id: Uuid, adaptation_type: AdaptationType, applied: AppliedAdaptation, alternatives: Vec<AdaptationAlternative>) -> Self {
        Self {
            request_id,
            status: AdaptationStatus::Partial,
            adaptation_type,
            message: Some("Partial adaptation applied".to_string()),
            applied: Some(applied),
            alternatives,
        }
    }

    /// Add alternatives
    pub fn with_alternatives(mut self, alternatives: Vec<AdaptationAlternative>) -> Self {
        self.alternatives = alternatives;
        self
    }
}

/// Adaptation status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AdaptationStatus {
    /// Request granted
    Granted,
    /// Request denied
    Denied,
    /// Partial adaptation applied
    Partial,
    /// Request pending
    Pending,
    /// Request expired
    Expired,
}

/// Applied adaptation details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppliedAdaptation {
    /// Adaptation ID
    pub adaptation_id: Uuid,
    /// What was applied
    pub description: String,
    /// Applied timestamp
    pub applied_at: DateTime<Utc>,
    /// Expiration (if applicable)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expires_at: Option<DateTime<Utc>>,
    /// Applied parameters
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub parameters: HashMap<String, serde_json::Value>,
}

impl AppliedAdaptation {
    /// Create a new applied adaptation
    pub fn new(description: &str) -> Self {
        Self {
            adaptation_id: Uuid::new_v4(),
            description: description.to_string(),
            applied_at: Utc::now(),
            expires_at: None,
            parameters: HashMap::new(),
        }
    }

    /// Set expiration
    pub fn with_expiration(mut self, expires_at: DateTime<Utc>) -> Self {
        self.expires_at = Some(expires_at);
        self
    }

    /// Add parameter
    pub fn with_parameter(mut self, key: &str, value: serde_json::Value) -> Self {
        self.parameters.insert(key.to_string(), value);
        self
    }
}

/// Alternative adaptation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptationAlternative {
    /// Alternative adaptation type
    pub adaptation_type: AdaptationType,
    /// Description
    pub description: String,
    /// Availability
    pub available: bool,
    /// How well it meets the need (0.0 - 1.0)
    pub match_score: f32,
}

/// Setting change event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SettingChangeEvent {
    /// Setting path (e.g., "display_preferences.screen_reader.enabled")
    pub setting_path: String,
    /// Previous value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub previous_value: Option<serde_json::Value>,
    /// New value
    pub new_value: serde_json::Value,
    /// Change source
    pub source: SettingChangeSource,
    /// Reason for change
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl SettingChangeEvent {
    /// Create a new setting change event
    pub fn new(
        setting_path: &str,
        previous_value: Option<serde_json::Value>,
        new_value: serde_json::Value,
        source: SettingChangeSource,
    ) -> Self {
        Self {
            setting_path: setting_path.to_string(),
            previous_value,
            new_value,
            source,
            reason: None,
        }
    }

    /// Set reason
    pub fn with_reason(mut self, reason: &str) -> Self {
        self.reason = Some(reason.to_string());
        self
    }
}

/// Source of setting change
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SettingChangeSource {
    /// Changed by user
    User,
    /// Changed by system/auto-adaptation
    System,
    /// Changed by sync from another platform
    Sync,
    /// Changed by administrator
    Admin,
    /// Changed by API
    Api,
}

/// Content alternative request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentAlternativeRequest {
    /// Request ID
    pub request_id: Uuid,
    /// Content ID
    pub content_id: String,
    /// Original content type
    pub content_type: ContentType,
    /// Requested alternative type
    pub alternative_type: AlternativeType,
    /// Preferred language
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preferred_language: Option<String>,
    /// Additional requirements
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub requirements: HashMap<String, serde_json::Value>,
}

impl ContentAlternativeRequest {
    /// Create a new content alternative request
    pub fn new(content_id: &str, content_type: ContentType, alternative_type: AlternativeType) -> Self {
        Self {
            request_id: Uuid::new_v4(),
            content_id: content_id.to_string(),
            content_type,
            alternative_type,
            preferred_language: None,
            requirements: HashMap::new(),
        }
    }

    /// Set preferred language
    pub fn with_language(mut self, language: &str) -> Self {
        self.preferred_language = Some(language.to_string());
        self
    }

    /// Add requirement
    pub fn with_requirement(mut self, key: &str, value: serde_json::Value) -> Self {
        self.requirements.insert(key.to_string(), value);
        self
    }
}

/// Alternative content type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AlternativeType {
    /// Caption/Subtitle
    Caption,
    /// Audio description
    AudioDescription,
    /// Sign language interpretation
    SignLanguage,
    /// Transcript
    Transcript,
    /// Simplified text
    SimplifiedText,
    /// Braille format
    Braille,
    /// Large print
    LargePrint,
    /// Audio version
    Audio,
    /// Tactile diagram
    Tactile,
    /// Easy-read version
    EasyRead,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_event() {
        let request = AdaptationRequest::new(AdaptationType::ExtendedTime)
            .with_target("assessment-123")
            .with_urgency(UrgencyLevel::High);

        let event = AccessibilityEvent::new(
            EventType::AdaptationRequest,
            "lms.example.com",
            EventPayload::Adaptation(request),
        );

        assert!(event.event_id != Uuid::nil());
        assert_eq!(event.event_type, EventType::AdaptationRequest);
    }

    #[test]
    fn test_adaptation_request() {
        let request = AdaptationRequest::new(AdaptationType::Caption)
            .with_target("video-456")
            .with_parameter("language", serde_json::json!("en-US"))
            .with_reason("Need captions for video content");

        assert_eq!(request.adaptation_type, AdaptationType::Caption);
        assert!(request.parameters.contains_key("language"));
    }

    #[test]
    fn test_adaptation_response_granted() {
        let request_id = Uuid::new_v4();
        let applied = AppliedAdaptation::new("Extended time applied: 1.5x multiplier")
            .with_parameter("multiplier", serde_json::json!(1.5));

        let response = AdaptationResponse::granted(
            request_id,
            AdaptationType::ExtendedTime,
            applied,
        );

        assert_eq!(response.status, AdaptationStatus::Granted);
        assert!(response.applied.is_some());
    }

    #[test]
    fn test_setting_change_event() {
        let event = SettingChangeEvent::new(
            "display_preferences.screen_reader.enabled",
            Some(serde_json::json!(false)),
            serde_json::json!(true),
            SettingChangeSource::User,
        ).with_reason("User enabled screen reader");

        assert_eq!(event.setting_path, "display_preferences.screen_reader.enabled");
        assert!(event.reason.is_some());
    }

    #[test]
    fn test_content_alternative_request() {
        let request = ContentAlternativeRequest::new(
            "video-789",
            ContentType::Video,
            AlternativeType::Caption,
        ).with_language("ko-KR");

        assert_eq!(request.alternative_type, AlternativeType::Caption);
        assert_eq!(request.preferred_language, Some("ko-KR".to_string()));
    }
}
