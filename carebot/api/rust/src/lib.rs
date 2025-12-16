//! WIA CareBot SDK
//!
//! A Rust library for the WIA CareBot Standard - AI Care Companion Robot.
//! Provides unified APIs for elderly care, health monitoring, emotion detection,
//! and emergency response systems.
//!
//! # Features
//!
//! - **AI Conversation**: Emotion recognition, dialogue management, personalization
//! - **Health Monitoring**: Vital signs, medication tracking, activity analysis
//! - **Safety**: Fall detection, emergency response, wandering alerts
//! - **Cognitive Care**: Memory games, orientation exercises, mental stimulation
//! - **Family Integration**: Notifications, video calls, daily summaries
//!
//! # Quick Start
//!
//! ```rust
//! use wia_carebot::prelude::*;
//!
//! // Create a care recipient profile
//! let recipient = CareRecipient::new("recipient-001", "김영희")
//!     .with_preferred_name("어머니")
//!     .with_cognitive_level(CognitiveLevel::MildImpairment)
//!     .with_mobility_level(MobilityLevel::AssistedWalking);
//!
//! // Create emotion state
//! let emotion = EmotionState::new("recipient-001")
//!     .with_primary(EmotionCategory::Happy, 0.85)
//!     .with_valence(0.7);
//!
//! // Check for safety events
//! let safety = SafetyEvent::fall_detected("recipient-001", "bathroom");
//! if safety.is_emergency() {
//!     println!("Emergency! Initiating response...");
//! }
//! ```

pub mod error;
pub mod types;
pub mod device;
pub mod recipient;
pub mod emotion;
pub mod conversation;
pub mod health;
pub mod safety;
pub mod routine;
pub mod cognitive;
pub mod notification;
pub mod protocol;
pub mod integration;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::error::{CareBotError, CareBotResult};
    pub use crate::types::*;
    pub use crate::device::*;
    pub use crate::recipient::*;
    pub use crate::emotion::*;
    pub use crate::conversation::*;
    pub use crate::health::*;
    pub use crate::safety::*;
    pub use crate::routine::*;
    pub use crate::cognitive::*;
    pub use crate::notification::*;
    // Protocol module types (excluding EmergencyContact to avoid name collision)
    pub use crate::protocol::message::*;
    pub use crate::protocol::websocket::*;
    pub use crate::protocol::security::*;
    pub use crate::protocol::emergency::{
        EmergencyDispatch, EmergencyType, IncidentInfo, EmergencyLocation,
        Coordinates, PatientInfo, VitalsAtEvent, EmergencyProtocol,
        EscalationLevel, EscalationAction,
    };
}

/// Crate version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::prelude::*;

    #[test]
    fn test_recipient_creation() {
        let recipient = CareRecipient::new("recipient-001", "김영희")
            .with_preferred_name("어머니")
            .with_cognitive_level(CognitiveLevel::MildImpairment);

        assert_eq!(recipient.id, "recipient-001");
        assert_eq!(recipient.profile.name, "김영희");
        assert_eq!(recipient.profile.preferred_name, Some("어머니".to_string()));
    }

    #[test]
    fn test_emotion_detection() {
        let emotion = EmotionState::new("recipient-001")
            .with_primary(EmotionCategory::Happy, 0.85)
            .with_valence(0.7)
            .with_arousal(0.5);

        assert_eq!(emotion.primary_emotion.category, EmotionCategory::Happy);
        assert!(emotion.is_positive());
    }

    #[test]
    fn test_safety_event() {
        let event = SafetyEvent::fall_detected("recipient-001", "bathroom");

        assert_eq!(event.event_type, SafetyEventType::FallDetected);
        assert!(event.is_emergency());
    }

    #[test]
    fn test_health_monitoring() {
        let mut health = HealthMonitoring::new("recipient-001");
        health.record_heart_rate(72);
        health.record_blood_pressure(128, 82);

        assert!(health.vital_signs.heart_rate.is_some());
        assert!(health.check_vitals_normal());
    }

    #[test]
    fn test_medication_adherence() {
        let mut health = HealthMonitoring::new("recipient-001");
        health.mark_medication_taken("morning");

        assert!(health.medication_adherence.morning_taken);
    }

    #[test]
    fn test_cognitive_assessment() {
        let mut assessment = CognitiveAssessment::new("recipient-001");
        assessment.set_orientation_time(0.75);
        assessment.set_memory_short_term(0.7);

        let score = assessment.overall_score();
        assert!(score > 0.0 && score <= 1.0);
    }

    #[test]
    fn test_conversation_session() {
        let mut session = ConversationSession::new("recipient-001", "carebot-001");
        session.add_bot_turn("좋은 아침이에요!", "greeting");
        session.add_recipient_turn("응, 좋은 아침", EmotionCategory::Neutral);

        assert_eq!(session.turns.len(), 2);
    }

    #[test]
    fn test_daily_routine() {
        let mut routine = DailyRoutine::new("recipient-001");
        routine.add_activity("08:00", "breakfast", ActivityType::Routine);
        routine.complete_activity("08:00");

        assert_eq!(routine.completed_count(), 1);
    }

    #[test]
    fn test_family_notification() {
        let notification = FamilyNotification::daily_summary(
            "recipient-001",
            "어머니 오늘 하루 요약",
            vec!["약 복용 완료", "산책 30분"],
        );

        assert_eq!(notification.notification_type, NotificationType::DailySummary);
    }
}
