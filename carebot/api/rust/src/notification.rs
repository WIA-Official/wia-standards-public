//! Family notification system

use serde::{Deserialize, Serialize};
use crate::types::{Severity, Timestamp};

/// Notification types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NotificationType {
    /// Daily summary report
    DailySummary,
    /// Safety alert
    SafetyAlert,
    /// Health alert
    HealthAlert,
    /// Medication reminder
    MedicationReminder,
    /// Activity update
    ActivityUpdate,
    /// Emotional concern
    EmotionalConcern,
    /// Cognitive assessment result
    CognitiveResult,
    /// Video call request
    VideoCallRequest,
    /// General information
    Information,
}

/// Family notification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FamilyNotification {
    /// Notification ID
    pub notification_id: String,
    /// Recipient ID (care recipient, not notification recipient)
    pub care_recipient_id: String,
    /// Notification type
    pub notification_type: NotificationType,
    /// Severity level
    pub severity: Severity,
    /// Title
    pub title: String,
    /// Message body
    pub message: String,
    /// Detailed items (for summaries)
    pub details: Vec<String>,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Target contacts (family member IDs or "all")
    pub targets: Vec<String>,
    /// Delivery status
    pub delivery: DeliveryStatus,
    /// Action required
    pub action_required: bool,
    /// Action URL or deep link
    pub action_url: Option<String>,
}

impl FamilyNotification {
    /// Create a daily summary notification
    pub fn daily_summary(
        care_recipient_id: &str,
        title: &str,
        summary_items: Vec<&str>,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::DailySummary,
            severity: Severity::Info,
            title: title.to_string(),
            message: format!("오늘의 활동 요약: {} 항목", summary_items.len()),
            details: summary_items.iter().map(|s| s.to_string()).collect(),
            timestamp: Timestamp::now(),
            targets: vec!["all".to_string()],
            delivery: DeliveryStatus::default(),
            action_required: false,
            action_url: None,
        }
    }

    /// Create a safety alert notification
    pub fn safety_alert(
        care_recipient_id: &str,
        event_type: &str,
        message: &str,
        severity: Severity,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::SafetyAlert,
            severity,
            title: format!("안전 알림: {}", event_type),
            message: message.to_string(),
            details: Vec::new(),
            timestamp: Timestamp::now(),
            targets: vec!["all".to_string()],
            delivery: DeliveryStatus::default(),
            action_required: severity >= Severity::Urgent,
            action_url: None,
        }
    }

    /// Create a health alert notification
    pub fn health_alert(
        care_recipient_id: &str,
        vital_type: &str,
        message: &str,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::HealthAlert,
            severity: Severity::Warning,
            title: format!("건강 알림: {}", vital_type),
            message: message.to_string(),
            details: Vec::new(),
            timestamp: Timestamp::now(),
            targets: vec!["all".to_string()],
            delivery: DeliveryStatus::default(),
            action_required: false,
            action_url: None,
        }
    }

    /// Create medication reminder notification
    pub fn medication_reminder(
        care_recipient_id: &str,
        medication_name: &str,
        time_slot: &str,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::MedicationReminder,
            severity: Severity::Info,
            title: "약 복용 알림".to_string(),
            message: format!("{} ({}) 복용 시간입니다", medication_name, time_slot),
            details: Vec::new(),
            timestamp: Timestamp::now(),
            targets: vec!["all".to_string()],
            delivery: DeliveryStatus::default(),
            action_required: false,
            action_url: None,
        }
    }

    /// Create emotional concern notification
    pub fn emotional_concern(
        care_recipient_id: &str,
        emotion: &str,
        context: &str,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::EmotionalConcern,
            severity: Severity::Warning,
            title: "감정 상태 알림".to_string(),
            message: format!("{}님이 {}한 상태입니다. {}", care_recipient_id, emotion, context),
            details: Vec::new(),
            timestamp: Timestamp::now(),
            targets: vec!["all".to_string()],
            delivery: DeliveryStatus::default(),
            action_required: true,
            action_url: Some("carebot://video-call".to_string()),
        }
    }

    /// Create video call request notification
    pub fn video_call_request(
        care_recipient_id: &str,
        target_contact: &str,
    ) -> Self {
        Self {
            notification_id: format!("notif-{}", uuid::Uuid::new_v4()),
            care_recipient_id: care_recipient_id.to_string(),
            notification_type: NotificationType::VideoCallRequest,
            severity: Severity::Info,
            title: "영상통화 요청".to_string(),
            message: format!("{}님이 영상통화를 원하십니다", care_recipient_id),
            details: Vec::new(),
            timestamp: Timestamp::now(),
            targets: vec![target_contact.to_string()],
            delivery: DeliveryStatus::default(),
            action_required: true,
            action_url: Some("carebot://video-call/incoming".to_string()),
        }
    }

    /// Check if notification requires immediate attention
    pub fn is_urgent(&self) -> bool {
        self.severity >= Severity::Urgent || self.action_required
    }

    /// Mark as sent
    pub fn mark_sent(&mut self) {
        self.delivery.sent = true;
        self.delivery.sent_at = Some(Timestamp::now());
    }

    /// Mark as delivered
    pub fn mark_delivered(&mut self) {
        self.delivery.delivered = true;
        self.delivery.delivered_at = Some(Timestamp::now());
    }

    /// Mark as read
    pub fn mark_read(&mut self) {
        self.delivery.read = true;
        self.delivery.read_at = Some(Timestamp::now());
    }
}

/// Delivery status for notifications
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DeliveryStatus {
    /// Notification sent
    pub sent: bool,
    /// Sent timestamp
    pub sent_at: Option<Timestamp>,
    /// Notification delivered
    pub delivered: bool,
    /// Delivered timestamp
    pub delivered_at: Option<Timestamp>,
    /// Notification read
    pub read: bool,
    /// Read timestamp
    pub read_at: Option<Timestamp>,
    /// Delivery method used
    pub method: Option<DeliveryMethod>,
    /// Error message if failed
    pub error: Option<String>,
}

/// Notification delivery methods
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMethod {
    /// Push notification
    Push,
    /// SMS message
    Sms,
    /// Email
    Email,
    /// Phone call
    PhoneCall,
    /// In-app notification
    InApp,
}

/// Notification preferences for family members
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotificationPreferences {
    /// Family member ID
    pub family_member_id: String,
    /// Enable push notifications
    pub push_enabled: bool,
    /// Enable SMS
    pub sms_enabled: bool,
    /// Enable email
    pub email_enabled: bool,
    /// Quiet hours start
    pub quiet_hours_start: Option<String>,
    /// Quiet hours end
    pub quiet_hours_end: Option<String>,
    /// Override quiet hours for emergencies
    pub emergency_override: bool,
    /// Notification types to receive
    pub enabled_types: Vec<NotificationType>,
    /// Minimum severity level
    pub minimum_severity: Severity,
}

impl Default for NotificationPreferences {
    fn default() -> Self {
        Self {
            family_member_id: String::new(),
            push_enabled: true,
            sms_enabled: true,
            email_enabled: false,
            quiet_hours_start: Some("23:00".to_string()),
            quiet_hours_end: Some("07:00".to_string()),
            emergency_override: true,
            enabled_types: vec![
                NotificationType::DailySummary,
                NotificationType::SafetyAlert,
                NotificationType::HealthAlert,
                NotificationType::EmotionalConcern,
                NotificationType::VideoCallRequest,
            ],
            minimum_severity: Severity::Info,
        }
    }
}
