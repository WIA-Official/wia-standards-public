//! Cognitive assessment and exercises

use serde::{Deserialize, Serialize};
use crate::types::Timestamp;

/// Cognitive assessment result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveAssessment {
    /// Recipient ID
    pub recipient_id: String,
    /// Assessment timestamp
    pub timestamp: Timestamp,
    /// Orientation scores
    pub orientation: OrientationScores,
    /// Memory scores
    pub memory: MemoryScores,
    /// Attention scores
    pub attention: AttentionScores,
    /// Language scores
    pub language: LanguageScores,
    /// Executive function scores
    pub executive: ExecutiveScores,
    /// Overall assessment notes
    pub notes: Vec<String>,
}

impl CognitiveAssessment {
    /// Create a new cognitive assessment
    pub fn new(recipient_id: &str) -> Self {
        Self {
            recipient_id: recipient_id.to_string(),
            timestamp: Timestamp::now(),
            orientation: OrientationScores::default(),
            memory: MemoryScores::default(),
            attention: AttentionScores::default(),
            language: LanguageScores::default(),
            executive: ExecutiveScores::default(),
            notes: Vec::new(),
        }
    }

    /// Set orientation to time score
    pub fn set_orientation_time(&mut self, score: f64) {
        self.orientation.time = Some(score.clamp(0.0, 1.0));
    }

    /// Set orientation to place score
    pub fn set_orientation_place(&mut self, score: f64) {
        self.orientation.place = Some(score.clamp(0.0, 1.0));
    }

    /// Set orientation to person score
    pub fn set_orientation_person(&mut self, score: f64) {
        self.orientation.person = Some(score.clamp(0.0, 1.0));
    }

    /// Set short-term memory score
    pub fn set_memory_short_term(&mut self, score: f64) {
        self.memory.short_term = Some(score.clamp(0.0, 1.0));
    }

    /// Set long-term memory score
    pub fn set_memory_long_term(&mut self, score: f64) {
        self.memory.long_term = Some(score.clamp(0.0, 1.0));
    }

    /// Calculate overall cognitive score
    pub fn overall_score(&self) -> f64 {
        let scores: Vec<f64> = [
            self.orientation.average(),
            self.memory.average(),
            self.attention.average(),
            self.language.average(),
            self.executive.average(),
        ]
        .into_iter()
        .flatten()
        .collect();

        if scores.is_empty() {
            return 0.0;
        }
        scores.iter().sum::<f64>() / scores.len() as f64
    }

    /// Check if any domain shows concern (below threshold)
    pub fn has_concerns(&self, threshold: f64) -> bool {
        [
            self.orientation.average(),
            self.memory.average(),
            self.attention.average(),
            self.language.average(),
            self.executive.average(),
        ]
        .into_iter()
        .any(|score| score.map(|s| s < threshold).unwrap_or(false))
    }

    /// Get domains with concerns
    pub fn get_concern_domains(&self, threshold: f64) -> Vec<String> {
        let mut concerns = Vec::new();

        if self.orientation.average().map(|s| s < threshold).unwrap_or(false) {
            concerns.push("지남력".to_string());
        }
        if self.memory.average().map(|s| s < threshold).unwrap_or(false) {
            concerns.push("기억력".to_string());
        }
        if self.attention.average().map(|s| s < threshold).unwrap_or(false) {
            concerns.push("주의력".to_string());
        }
        if self.language.average().map(|s| s < threshold).unwrap_or(false) {
            concerns.push("언어능력".to_string());
        }
        if self.executive.average().map(|s| s < threshold).unwrap_or(false) {
            concerns.push("집행기능".to_string());
        }

        concerns
    }
}

/// Orientation scores (time, place, person)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OrientationScores {
    /// Orientation to time
    pub time: Option<f64>,
    /// Orientation to place
    pub place: Option<f64>,
    /// Orientation to person
    pub person: Option<f64>,
}

impl OrientationScores {
    /// Calculate average orientation score
    pub fn average(&self) -> Option<f64> {
        let scores: Vec<f64> = [self.time, self.place, self.person]
            .into_iter()
            .flatten()
            .collect();
        if scores.is_empty() {
            return None;
        }
        Some(scores.iter().sum::<f64>() / scores.len() as f64)
    }
}

/// Memory scores
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MemoryScores {
    /// Short-term memory
    pub short_term: Option<f64>,
    /// Long-term memory
    pub long_term: Option<f64>,
    /// Working memory
    pub working: Option<f64>,
    /// Episodic memory
    pub episodic: Option<f64>,
}

impl MemoryScores {
    /// Calculate average memory score
    pub fn average(&self) -> Option<f64> {
        let scores: Vec<f64> = [
            self.short_term,
            self.long_term,
            self.working,
            self.episodic,
        ]
        .into_iter()
        .flatten()
        .collect();
        if scores.is_empty() {
            return None;
        }
        Some(scores.iter().sum::<f64>() / scores.len() as f64)
    }
}

/// Attention scores
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AttentionScores {
    /// Sustained attention
    pub sustained: Option<f64>,
    /// Selective attention
    pub selective: Option<f64>,
    /// Divided attention
    pub divided: Option<f64>,
}

impl AttentionScores {
    /// Calculate average attention score
    pub fn average(&self) -> Option<f64> {
        let scores: Vec<f64> = [self.sustained, self.selective, self.divided]
            .into_iter()
            .flatten()
            .collect();
        if scores.is_empty() {
            return None;
        }
        Some(scores.iter().sum::<f64>() / scores.len() as f64)
    }
}

/// Language scores
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LanguageScores {
    /// Comprehension
    pub comprehension: Option<f64>,
    /// Expression
    pub expression: Option<f64>,
    /// Naming ability
    pub naming: Option<f64>,
    /// Repetition
    pub repetition: Option<f64>,
}

impl LanguageScores {
    /// Calculate average language score
    pub fn average(&self) -> Option<f64> {
        let scores: Vec<f64> = [
            self.comprehension,
            self.expression,
            self.naming,
            self.repetition,
        ]
        .into_iter()
        .flatten()
        .collect();
        if scores.is_empty() {
            return None;
        }
        Some(scores.iter().sum::<f64>() / scores.len() as f64)
    }
}

/// Executive function scores
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ExecutiveScores {
    /// Planning ability
    pub planning: Option<f64>,
    /// Problem solving
    pub problem_solving: Option<f64>,
    /// Flexibility
    pub flexibility: Option<f64>,
    /// Inhibition
    pub inhibition: Option<f64>,
}

impl ExecutiveScores {
    /// Calculate average executive score
    pub fn average(&self) -> Option<f64> {
        let scores: Vec<f64> = [
            self.planning,
            self.problem_solving,
            self.flexibility,
            self.inhibition,
        ]
        .into_iter()
        .flatten()
        .collect();
        if scores.is_empty() {
            return None;
        }
        Some(scores.iter().sum::<f64>() / scores.len() as f64)
    }
}

/// Cognitive exercise types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CognitiveExerciseType {
    /// Memory game
    MemoryGame,
    /// Word puzzle
    WordPuzzle,
    /// Number game
    NumberGame,
    /// Pattern recognition
    PatternRecognition,
    /// Reminiscence therapy
    Reminiscence,
    /// Orientation exercise
    OrientationExercise,
    /// Attention training
    AttentionTraining,
}

/// Cognitive exercise session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CognitiveExercise {
    /// Exercise ID
    pub exercise_id: String,
    /// Recipient ID
    pub recipient_id: String,
    /// Exercise type
    pub exercise_type: CognitiveExerciseType,
    /// Started at
    pub started_at: Timestamp,
    /// Ended at
    pub ended_at: Option<Timestamp>,
    /// Difficulty level (1-5)
    pub difficulty: u8,
    /// Score achieved (0.0-1.0)
    pub score: Option<f64>,
    /// Completion status
    pub completed: bool,
    /// Time taken in seconds
    pub time_seconds: Option<u32>,
    /// Errors made
    pub errors: u32,
    /// Hints used
    pub hints_used: u32,
}

impl CognitiveExercise {
    /// Create a new cognitive exercise
    pub fn new(recipient_id: &str, exercise_type: CognitiveExerciseType, difficulty: u8) -> Self {
        Self {
            exercise_id: format!("exercise-{}", uuid::Uuid::new_v4()),
            recipient_id: recipient_id.to_string(),
            exercise_type,
            started_at: Timestamp::now(),
            ended_at: None,
            difficulty: difficulty.clamp(1, 5),
            score: None,
            completed: false,
            time_seconds: None,
            errors: 0,
            hints_used: 0,
        }
    }

    /// Complete the exercise
    pub fn complete(&mut self, score: f64, errors: u32, hints: u32) {
        self.ended_at = Some(Timestamp::now());
        self.score = Some(score.clamp(0.0, 1.0));
        self.errors = errors;
        self.hints_used = hints;
        self.completed = true;
        self.time_seconds = Some(
            (self.ended_at.as_ref().unwrap().0 - self.started_at.0).num_seconds() as u32,
        );
    }
}
