//! Conversation session management for AI dialogue

use serde::{Deserialize, Serialize};
use crate::emotion::EmotionCategory;
use crate::types::Timestamp;

/// Conversation session between carebot and recipient
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationSession {
    /// Session ID
    pub session_id: String,
    /// Recipient ID
    pub recipient_id: String,
    /// Device ID
    pub device_id: String,
    /// Session start time
    pub started_at: Timestamp,
    /// Session end time
    pub ended_at: Option<Timestamp>,
    /// Conversation turns
    pub turns: Vec<ConversationTurn>,
    /// Session context
    pub context: SessionContext,
    /// Session summary (generated at end)
    pub summary: Option<SessionSummary>,
}

impl ConversationSession {
    /// Create a new conversation session
    pub fn new(recipient_id: &str, device_id: &str) -> Self {
        let session_id = format!("session-{}-{}", recipient_id, uuid::Uuid::new_v4());
        Self {
            session_id,
            recipient_id: recipient_id.to_string(),
            device_id: device_id.to_string(),
            started_at: Timestamp::now(),
            ended_at: None,
            turns: Vec::new(),
            context: SessionContext::default(),
            summary: None,
        }
    }

    /// Add a bot turn (carebot speaking)
    pub fn add_bot_turn(&mut self, text: &str, intent: &str) {
        self.turns.push(ConversationTurn {
            turn_id: self.turns.len() as u32 + 1,
            timestamp: Timestamp::now(),
            speaker: Speaker::Bot,
            text: text.to_string(),
            intent: Some(intent.to_string()),
            emotion: None,
            response_latency_ms: None,
        });
    }

    /// Add a recipient turn
    pub fn add_recipient_turn(&mut self, text: &str, emotion: EmotionCategory) {
        self.turns.push(ConversationTurn {
            turn_id: self.turns.len() as u32 + 1,
            timestamp: Timestamp::now(),
            speaker: Speaker::Recipient,
            text: text.to_string(),
            intent: None,
            emotion: Some(emotion),
            response_latency_ms: None,
        });
    }

    /// End the session
    pub fn end_session(&mut self) {
        self.ended_at = Some(Timestamp::now());
    }

    /// Get session duration in seconds
    pub fn duration_seconds(&self) -> Option<i64> {
        self.ended_at
            .as_ref()
            .map(|end| (end.0 - self.started_at.0).num_seconds())
    }

    /// Get total number of turns
    pub fn turn_count(&self) -> usize {
        self.turns.len()
    }
}

/// Speaker in conversation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Speaker {
    /// CareBot
    Bot,
    /// Care recipient
    Recipient,
    /// Family member
    Family,
    /// Caregiver
    Caregiver,
}

/// Single turn in conversation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationTurn {
    /// Turn number
    pub turn_id: u32,
    /// Timestamp
    pub timestamp: Timestamp,
    /// Who is speaking
    pub speaker: Speaker,
    /// Text content
    pub text: String,
    /// Detected intent (for bot turns)
    pub intent: Option<String>,
    /// Detected emotion (for recipient turns)
    pub emotion: Option<EmotionCategory>,
    /// Response latency in milliseconds
    pub response_latency_ms: Option<u32>,
}

/// Session context for personalization
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SessionContext {
    /// Current topic
    pub current_topic: Option<String>,
    /// Previous topics discussed
    pub previous_topics: Vec<String>,
    /// Recipient's current mood
    pub mood: Option<String>,
    /// Time of day
    pub time_of_day: Option<String>,
    /// Special occasion (birthday, anniversary, etc.)
    pub special_occasion: Option<String>,
    /// Reminders to mention
    pub pending_reminders: Vec<String>,
}

/// Session summary generated at end
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionSummary {
    /// Duration in seconds
    pub duration_seconds: i64,
    /// Number of turns
    pub turn_count: u32,
    /// Topics discussed
    pub topics: Vec<String>,
    /// Dominant emotion during session
    pub dominant_emotion: EmotionCategory,
    /// Engagement level (0.0-1.0)
    pub engagement_level: f64,
    /// Key points for follow-up
    pub key_points: Vec<String>,
    /// Concerns detected
    pub concerns: Vec<String>,
}

/// Intent categories for conversation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConversationIntent {
    /// Greeting
    Greeting,
    /// Farewell
    Farewell,
    /// Health inquiry
    HealthInquiry,
    /// Medication reminder
    MedicationReminder,
    /// Emotional support
    EmotionalSupport,
    /// Reminiscence
    Reminiscence,
    /// Daily routine
    DailyRoutine,
    /// Entertainment
    Entertainment,
    /// News/Information
    Information,
    /// Emergency
    Emergency,
    /// Family related
    Family,
    /// Cognitive exercise
    CognitiveExercise,
}
