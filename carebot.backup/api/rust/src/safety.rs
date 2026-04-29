//! Safety event detection and emergency response

use serde::{Deserialize, Serialize};
use crate::types::{Location, Severity, Timestamp};

/// Safety event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafetyEventType {
    /// Fall detected
    FallDetected,
    /// Wandering detected
    WanderingDetected,
    /// Abnormal activity pattern
    AbnormalActivity,
    /// No movement for extended period
    NoMovementExtended,
    /// SOS button pressed
    SosButton,
    /// Medication missed
    MedicationMissed,
    /// Vital sign alert
    VitalSignAlert,
    /// Environmental hazard
    EnvironmentalHazard,
}

/// Safety event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyEvent {
    /// Event ID
    pub event_id: String,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Recipient ID
    pub recipient_id: String,
    /// Device ID
    pub device_id: Option<String>,
    /// Event type
    pub event_type: SafetyEventType,
    /// Severity level
    pub severity: Severity,
    /// Location
    pub location: Option<Location>,
    /// Detection details
    pub detection: DetectionDetails,
    /// Recipient response
    pub recipient_response: Option<RecipientResponse>,
    /// Automated actions taken
    pub automated_actions: Vec<AutomatedAction>,
    /// Resolution status
    pub resolution: Resolution,
}

impl SafetyEvent {
    /// Create a fall detected event
    pub fn fall_detected(recipient_id: &str, room: &str) -> Self {
        let event_id = format!(
            "safety-{}-{}",
            chrono::Utc::now().timestamp(),
            uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
        );
        Self {
            event_id,
            timestamp: Timestamp::now(),
            recipient_id: recipient_id.to_string(),
            device_id: None,
            event_type: SafetyEventType::FallDetected,
            severity: Severity::Emergency,
            location: Some(Location::new(room)),
            detection: DetectionDetails {
                method: "camera_ai".to_string(),
                confidence: 0.95,
                supporting_evidence: vec![
                    "sudden_vertical_movement".to_string(),
                    "impact_detected".to_string(),
                    "lying_posture".to_string(),
                ],
            },
            recipient_response: None,
            automated_actions: Vec::new(),
            resolution: Resolution::default(),
        }
    }

    /// Create an SOS button event
    pub fn sos_pressed(recipient_id: &str) -> Self {
        let event_id = format!(
            "safety-{}-{}",
            chrono::Utc::now().timestamp(),
            uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
        );
        Self {
            event_id,
            timestamp: Timestamp::now(),
            recipient_id: recipient_id.to_string(),
            device_id: None,
            event_type: SafetyEventType::SosButton,
            severity: Severity::Emergency,
            location: None,
            detection: DetectionDetails {
                method: "button_press".to_string(),
                confidence: 1.0,
                supporting_evidence: vec!["sos_button_activated".to_string()],
            },
            recipient_response: None,
            automated_actions: Vec::new(),
            resolution: Resolution::default(),
        }
    }

    /// Create a wandering alert
    pub fn wandering_detected(recipient_id: &str, area: &str) -> Self {
        let event_id = format!(
            "safety-{}-{}",
            chrono::Utc::now().timestamp(),
            uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
        );
        Self {
            event_id,
            timestamp: Timestamp::now(),
            recipient_id: recipient_id.to_string(),
            device_id: None,
            event_type: SafetyEventType::WanderingDetected,
            severity: Severity::Urgent,
            location: Some(Location::new(area)),
            detection: DetectionDetails {
                method: "location_tracking".to_string(),
                confidence: 0.88,
                supporting_evidence: vec![
                    "outside_safe_zone".to_string(),
                    "unusual_time".to_string(),
                ],
            },
            recipient_response: None,
            automated_actions: Vec::new(),
            resolution: Resolution::default(),
        }
    }

    /// Create vital sign alert
    pub fn vital_alert(recipient_id: &str, vital_type: &str, message: &str) -> Self {
        let event_id = format!(
            "safety-{}-{}",
            chrono::Utc::now().timestamp(),
            uuid::Uuid::new_v4().to_string().split('-').next().unwrap()
        );
        Self {
            event_id,
            timestamp: Timestamp::now(),
            recipient_id: recipient_id.to_string(),
            device_id: None,
            event_type: SafetyEventType::VitalSignAlert,
            severity: Severity::Critical,
            location: None,
            detection: DetectionDetails {
                method: "vital_monitoring".to_string(),
                confidence: 0.99,
                supporting_evidence: vec![format!("{}_{}", vital_type, message)],
            },
            recipient_response: None,
            automated_actions: Vec::new(),
            resolution: Resolution::default(),
        }
    }

    /// Check if event is an emergency
    pub fn is_emergency(&self) -> bool {
        self.severity >= Severity::Critical
    }

    /// Check if event requires immediate action
    pub fn requires_immediate_action(&self) -> bool {
        matches!(
            self.event_type,
            SafetyEventType::FallDetected
                | SafetyEventType::SosButton
                | SafetyEventType::VitalSignAlert
        ) || self.severity == Severity::Emergency
    }

    /// Add automated action
    pub fn add_action(&mut self, action: AutomatedAction) {
        self.automated_actions.push(action);
    }

    /// Mark as resolved
    pub fn resolve(&mut self, resolved_by: &str, outcome: &str) {
        self.resolution.status = ResolutionStatus::Resolved;
        self.resolution.resolved_at = Some(Timestamp::now());
        self.resolution.resolved_by = Some(resolved_by.to_string());
        self.resolution.outcome = Some(outcome.to_string());
    }
}

/// Detection details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectionDetails {
    /// Detection method
    pub method: String,
    /// Confidence (0.0-1.0)
    pub confidence: f64,
    /// Supporting evidence
    pub supporting_evidence: Vec<String>,
}

/// Recipient response to safety event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecipientResponse {
    /// Verbal response received
    pub verbal_response: bool,
    /// Movement detected
    pub movement_detected: bool,
    /// Button press received
    pub button_press: bool,
    /// Response latency in seconds
    pub response_latency_seconds: Option<f64>,
}

/// Automated action taken
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomatedAction {
    /// Action type
    pub action: AutomatedActionType,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Message sent
    pub message: Option<String>,
    /// Response received
    pub response_received: bool,
    /// Contacts notified
    pub contacts_notified: Vec<String>,
    /// Notification method
    pub notification_method: Option<String>,
    /// Service called
    pub service: Option<String>,
    /// Call initiated
    pub call_initiated: bool,
}

/// Types of automated actions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AutomatedActionType {
    /// Voice prompt to recipient
    VoicePrompt,
    /// Notification to family
    FamilyNotification,
    /// Call to emergency services
    EmergencyCall,
    /// Alert to care team
    CareTeamAlert,
    /// Automated video call
    VideoCall,
}

impl AutomatedAction {
    /// Create a voice prompt action
    pub fn voice_prompt(message: &str) -> Self {
        Self {
            action: AutomatedActionType::VoicePrompt,
            timestamp: Timestamp::now(),
            message: Some(message.to_string()),
            response_received: false,
            contacts_notified: Vec::new(),
            notification_method: None,
            service: None,
            call_initiated: false,
        }
    }

    /// Create a family notification action
    pub fn family_notification(contacts: Vec<String>, method: &str) -> Self {
        Self {
            action: AutomatedActionType::FamilyNotification,
            timestamp: Timestamp::now(),
            message: None,
            response_received: false,
            contacts_notified: contacts,
            notification_method: Some(method.to_string()),
            service: None,
            call_initiated: false,
        }
    }

    /// Create an emergency call action
    pub fn emergency_call(service: &str) -> Self {
        Self {
            action: AutomatedActionType::EmergencyCall,
            timestamp: Timestamp::now(),
            message: None,
            response_received: false,
            contacts_notified: Vec::new(),
            notification_method: None,
            service: Some(service.to_string()),
            call_initiated: true,
        }
    }
}

/// Resolution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResolutionStatus {
    Pending,
    InProgress,
    Resolved,
    Escalated,
}

/// Resolution details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Resolution {
    /// Status
    pub status: ResolutionStatus,
    /// Resolution timestamp
    pub resolved_at: Option<Timestamp>,
    /// Resolved by
    pub resolved_by: Option<String>,
    /// Outcome
    pub outcome: Option<String>,
    /// Follow-up required
    pub follow_up_required: bool,
    /// Notes
    pub notes: Option<String>,
}

impl Default for Resolution {
    fn default() -> Self {
        Self {
            status: ResolutionStatus::Pending,
            resolved_at: None,
            resolved_by: None,
            outcome: None,
            follow_up_required: false,
            notes: None,
        }
    }
}
