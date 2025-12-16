//! Care robot adapter
//!
//! Provides control for companion and care robots.

use crate::error::{RobotError, RobotResult};
use crate::types::Pose2D;
use serde::{Deserialize, Serialize};

/// Care robot type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum CareType {
    #[default]
    ElderlyCompanion,
    Pediatric,
    DementiaCare,
    HospitalAssistant,
    ServiceRobot,
}

/// Interaction mode
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum InteractionMode {
    #[default]
    Idle,
    Conversation,
    Entertainment,
    Reminder,
    Monitoring,
    Assistance,
}

/// Emotional state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmotionalState {
    Happy,
    Calm,
    Anxious,
    Sad,
    Neutral,
}

/// Task status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum TaskStatus {
    #[default]
    Pending,
    InProgress,
    Completed,
    Skipped,
    Failed,
}

/// Navigation status
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum NavigationStatus {
    #[default]
    Idle,
    Navigating,
    Arrived,
    Blocked,
}

/// Interaction data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Interaction {
    pub mode: InteractionMode,
    pub active_duration_s: u32,
    pub engagement_level: f64,
    pub last_interaction: Option<String>,
    pub language: String,
}

/// Emotion recognition data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct EmotionRecognition {
    pub detected_emotion: String,
    pub confidence: f64,
    pub valence: f64,
    pub arousal: f64,
    pub face_detected: bool,
}

impl EmotionRecognition {
    /// Determine emotional state from valence-arousal model
    pub fn emotional_state(&self) -> EmotionalState {
        if self.valence > 0.3 && self.arousal > 0.5 {
            EmotionalState::Happy
        } else if self.valence > 0.3 && self.arousal <= 0.5 {
            EmotionalState::Calm
        } else if self.valence < -0.3 && self.arousal > 0.5 {
            EmotionalState::Anxious
        } else if self.valence < -0.3 && self.arousal <= 0.5 {
            EmotionalState::Sad
        } else {
            EmotionalState::Neutral
        }
    }
}

/// Vital signs monitoring
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct VitalSigns {
    pub heart_rate_bpm: u16,
    pub respiratory_rate: u16,
    pub body_temp_c: f64,
    pub blood_pressure: Option<BloodPressure>,
    pub oxygen_saturation: Option<u8>,
    pub fall_detected: bool,
    pub activity_level: ActivityLevel,
    pub sleep_quality: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct BloodPressure {
    pub systolic: u16,
    pub diastolic: u16,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum ActivityLevel {
    Resting,
    #[default]
    Light,
    Moderate,
    Vigorous,
}

impl VitalSigns {
    /// Check if vital signs are within normal range
    pub fn is_normal(&self) -> bool {
        const HEART_RATE_MIN: u16 = 60;
        const HEART_RATE_MAX: u16 = 100;
        const RESP_RATE_MIN: u16 = 12;
        const RESP_RATE_MAX: u16 = 20;
        const TEMP_MIN: f64 = 36.0;
        const TEMP_MAX: f64 = 37.5;

        !self.fall_detected
            && self.heart_rate_bpm >= HEART_RATE_MIN
            && self.heart_rate_bpm <= HEART_RATE_MAX
            && self.respiratory_rate >= RESP_RATE_MIN
            && self.respiratory_rate <= RESP_RATE_MAX
            && self.body_temp_c >= TEMP_MIN
            && self.body_temp_c <= TEMP_MAX
    }

    /// Generate alerts based on vital signs
    pub fn generate_alerts(&self) -> Vec<String> {
        let mut alerts = Vec::new();

        if self.fall_detected {
            alerts.push("CRITICAL: Fall detected!".to_string());
        }

        if self.heart_rate_bpm < 60 {
            alerts.push(format!(
                "WARNING: Low heart rate ({} bpm)",
                self.heart_rate_bpm
            ));
        } else if self.heart_rate_bpm > 100 {
            alerts.push(format!(
                "WARNING: High heart rate ({} bpm)",
                self.heart_rate_bpm
            ));
        }

        if self.body_temp_c > 38.0 {
            alerts.push(format!("WARNING: Fever ({:.1}C)", self.body_temp_c));
        } else if self.body_temp_c < 35.0 {
            alerts.push(format!("WARNING: Hypothermia ({:.1}C)", self.body_temp_c));
        }

        if let Some(spo2) = self.oxygen_saturation {
            if spo2 < 95 {
                alerts.push(format!("WARNING: Low oxygen saturation ({}%)", spo2));
            }
        }

        alerts
    }
}

/// Care task
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareTask {
    pub task_id: String,
    pub task_type: CareTaskType,
    pub description: String,
    pub scheduled_time: Option<String>,
    pub status: TaskStatus,
    pub confirmation: bool,
    pub notes: Option<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CareTaskType {
    MedicationReminder,
    AppointmentReminder,
    ActivityPrompt,
    MealReminder,
    HydrationReminder,
    ExercisePrompt,
    SocialInteraction,
    EmergencyAlert,
}

/// Navigation data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CareNavigation {
    pub current_room: Option<String>,
    pub current_pose: Pose2D,
    pub following_user: bool,
    pub distance_to_user_m: Option<f64>,
    pub map_id: Option<String>,
    pub navigation_status: NavigationStatus,
}

/// Voice interaction data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct VoiceInteraction {
    pub last_utterance: Option<String>,
    pub asr_confidence: f64,
    pub tts_active: bool,
    pub wake_word_detected: bool,
}

/// Care robot specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CareRobotSpec {
    pub care_type: CareType,
    pub interaction: Interaction,
    pub emotion: EmotionRecognition,
    pub vital_signs: VitalSigns,
    pub tasks: Vec<CareTask>,
    pub navigation: CareNavigation,
    pub voice: VoiceInteraction,
}

impl Default for CareRobotSpec {
    fn default() -> Self {
        Self {
            care_type: CareType::ElderlyCompanion,
            interaction: Interaction {
                language: "en-US".to_string(),
                ..Default::default()
            },
            emotion: EmotionRecognition::default(),
            vital_signs: VitalSigns {
                heart_rate_bpm: 72,
                respiratory_rate: 16,
                body_temp_c: 36.5,
                ..Default::default()
            },
            tasks: Vec::new(),
            navigation: CareNavigation::default(),
            voice: VoiceInteraction::default(),
        }
    }
}

impl CareRobotSpec {
    /// Create a new elderly companion robot
    pub fn new_elderly_companion() -> Self {
        Self::default()
    }

    /// Add a new task
    pub fn add_task(&mut self, task: CareTask) {
        self.tasks.push(task);
    }

    /// Get pending tasks
    pub fn pending_tasks(&self) -> Vec<&CareTask> {
        self.tasks
            .iter()
            .filter(|t| t.status == TaskStatus::Pending)
            .collect()
    }

    /// Complete a task by ID
    pub fn complete_task(&mut self, task_id: &str) -> RobotResult<()> {
        let task = self
            .tasks
            .iter_mut()
            .find(|t| t.task_id == task_id)
            .ok_or_else(|| {
                RobotError::InvalidParameter(format!("Task not found: {}", task_id))
            })?;

        task.status = TaskStatus::Completed;
        task.confirmation = true;
        Ok(())
    }

    /// Check if emergency response is needed
    pub fn needs_emergency_response(&self) -> bool {
        self.vital_signs.fall_detected
            || !self.vital_signs.generate_alerts().is_empty()
                && self
                    .vital_signs
                    .generate_alerts()
                    .iter()
                    .any(|a| a.starts_with("CRITICAL"))
    }

    /// Get appropriate response based on detected emotion
    pub fn get_emotional_response(&self) -> &'static str {
        match self.emotion.emotional_state() {
            EmotionalState::Happy => "I'm glad to see you're in a good mood!",
            EmotionalState::Calm => "It's nice and peaceful, isn't it?",
            EmotionalState::Anxious => "You seem a bit worried. Would you like to talk about it?",
            EmotionalState::Sad => "I'm here if you need someone to talk to.",
            EmotionalState::Neutral => "How can I help you today?",
        }
    }

    /// Start following user
    pub fn start_following(&mut self) {
        self.navigation.following_user = true;
        self.navigation.navigation_status = NavigationStatus::Navigating;
    }

    /// Stop following user
    pub fn stop_following(&mut self) {
        self.navigation.following_user = false;
        self.navigation.navigation_status = NavigationStatus::Idle;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vital_signs_normal() {
        let vitals = VitalSigns {
            heart_rate_bpm: 72,
            respiratory_rate: 16,
            body_temp_c: 36.5,
            fall_detected: false,
            ..Default::default()
        };
        assert!(vitals.is_normal());
    }

    #[test]
    fn test_vital_signs_alerts() {
        let vitals = VitalSigns {
            heart_rate_bpm: 120,
            body_temp_c: 39.0,
            fall_detected: true,
            ..Default::default()
        };
        let alerts = vitals.generate_alerts();
        assert!(!alerts.is_empty());
        assert!(alerts.iter().any(|a| a.contains("Fall")));
    }

    #[test]
    fn test_emotional_state() {
        let emotion = EmotionRecognition {
            valence: 0.5,
            arousal: 0.7,
            ..Default::default()
        };
        assert_eq!(emotion.emotional_state(), EmotionalState::Happy);
    }

    #[test]
    fn test_add_and_complete_task() {
        let mut robot = CareRobotSpec::default();
        robot.add_task(CareTask {
            task_id: "task-001".to_string(),
            task_type: CareTaskType::MedicationReminder,
            description: "Take medication".to_string(),
            scheduled_time: None,
            status: TaskStatus::Pending,
            confirmation: false,
            notes: None,
        });

        assert_eq!(robot.pending_tasks().len(), 1);
        assert!(robot.complete_task("task-001").is_ok());
        assert_eq!(robot.pending_tasks().len(), 0);
    }
}
