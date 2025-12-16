//! Rehabilitation robot adapter
//!
//! Provides control for rehabilitation therapy robots.

use crate::error::{RobotError, RobotResult};
use crate::types::Position3D;
use serde::{Deserialize, Serialize};

/// Therapy type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum TherapyType {
    #[default]
    UpperLimb,
    LowerLimb,
    Hand,
    Gait,
    Balance,
}

/// Device type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum RehabDeviceType {
    #[default]
    EndEffector,
    Exoskeleton,
    Treadmill,
    Platform,
}

/// Path type for trajectory
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum PathType {
    #[default]
    Linear,
    Circular,
    Custom,
}

/// Exercise data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Exercise {
    pub name: String,
    pub exercise_type: String,
    pub repetition: u32,
    pub total_repetitions: u32,
    pub set: u32,
    pub total_sets: u32,
    pub duration_seconds: u32,
    pub difficulty_level: u8,
    pub game_id: Option<String>,
}

impl Default for Exercise {
    fn default() -> Self {
        Self {
            name: "default".to_string(),
            exercise_type: "point_to_point".to_string(),
            repetition: 0,
            total_repetitions: 10,
            set: 1,
            total_sets: 3,
            duration_seconds: 0,
            difficulty_level: 5,
            game_id: None,
        }
    }
}

/// Trajectory data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Trajectory {
    pub current_position: Position3D,
    pub target_position: Position3D,
    pub start_position: Position3D,
    pub velocity_m_s: f64,
    pub path_completion: f64,
    pub path_type: PathType,
}

/// Patient effort data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PatientEffort {
    pub active_participation: f64,
    pub assist_as_needed: f64,
    pub resistance_nm: f64,
    pub force_applied: Position3D,
    pub emg_activation: f64,
}

/// Performance metrics
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PerformanceMetrics {
    pub rom_achieved_deg: f64,
    pub rom_target_deg: f64,
    pub rom_baseline_deg: f64,
    pub smoothness_score: f64,
    pub accuracy_cm: f64,
    pub speed_score: f64,
    pub error_rate: f64,
}

/// Session data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionData {
    pub session_id: String,
    pub therapist_id: Option<String>,
    pub start_time: String,
    pub elapsed_seconds: u32,
    pub progress_percent: f64,
    pub notes: Option<String>,
}

impl Default for SessionData {
    fn default() -> Self {
        Self {
            session_id: "session-001".to_string(),
            therapist_id: None,
            start_time: chrono::Utc::now().to_rfc3339(),
            elapsed_seconds: 0,
            progress_percent: 0.0,
            notes: None,
        }
    }
}

/// Biofeedback data
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Biofeedback {
    pub heart_rate_bpm: Option<u16>,
    pub fatigue_level: f64,
    pub pain_reported: u8,
}

/// Rehabilitation specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RehabilitationSpec {
    pub therapy_type: TherapyType,
    pub device_type: RehabDeviceType,
    pub exercise: Exercise,
    pub trajectory: Trajectory,
    pub patient_effort: PatientEffort,
    pub performance: PerformanceMetrics,
    pub session: SessionData,
    pub biofeedback: Biofeedback,
}

impl Default for RehabilitationSpec {
    fn default() -> Self {
        Self {
            therapy_type: TherapyType::UpperLimb,
            device_type: RehabDeviceType::EndEffector,
            exercise: Exercise::default(),
            trajectory: Trajectory::default(),
            patient_effort: PatientEffort::default(),
            performance: PerformanceMetrics::default(),
            session: SessionData::default(),
            biofeedback: Biofeedback::default(),
        }
    }
}

impl RehabilitationSpec {
    /// Create a new upper limb rehabilitation spec
    pub fn new_upper_limb() -> Self {
        Self {
            therapy_type: TherapyType::UpperLimb,
            ..Default::default()
        }
    }

    /// Calculate exercise progress (0.0 - 1.0)
    pub fn exercise_progress(&self) -> f64 {
        if self.exercise.total_repetitions == 0 {
            return 0.0;
        }
        (self.exercise.repetition as f64 / self.exercise.total_repetitions as f64).min(1.0)
    }

    /// Calculate ROM achievement rate (0.0 - 1.0)
    pub fn rom_achievement_rate(&self) -> f64 {
        if self.performance.rom_target_deg == 0.0 {
            return 0.0;
        }
        (self.performance.rom_achieved_deg / self.performance.rom_target_deg).min(1.0)
    }

    /// Calculate assist-as-needed level based on patient effort
    pub fn calculate_assistance(&self) -> f64 {
        let base_assist = 1.0 - self.patient_effort.active_participation;
        let difficulty_factor = self.exercise.difficulty_level as f64 / 10.0;
        (base_assist * difficulty_factor).clamp(0.0, 1.0)
    }

    /// Calculate trajectory error (distance to target)
    pub fn trajectory_error(&self) -> f64 {
        self.trajectory
            .current_position
            .distance_to(&self.trajectory.target_position)
    }

    /// Interpolate next target position (linear)
    pub fn interpolate_next_target(&self, dt: f64) -> Position3D {
        let distance = self.trajectory_error();
        if distance < 1e-6 {
            return self.trajectory.target_position;
        }

        let direction = self
            .trajectory
            .target_position
            .sub(&self.trajectory.current_position)
            .normalize();

        let step = (self.trajectory.velocity_m_s * dt).min(distance);

        self.trajectory.current_position.add(&direction.scale(step))
    }

    /// Increment repetition count
    pub fn complete_repetition(&mut self) {
        self.exercise.repetition += 1;
        if self.exercise.repetition >= self.exercise.total_repetitions {
            self.exercise.set += 1;
            self.exercise.repetition = 0;
        }
        self.update_progress();
    }

    /// Update session progress
    fn update_progress(&mut self) {
        let rep_progress = self.exercise_progress();
        let set_progress = if self.exercise.total_sets > 0 {
            self.exercise.set as f64 / self.exercise.total_sets as f64
        } else {
            0.0
        };
        self.session.progress_percent = ((rep_progress + set_progress) / 2.0 * 100.0).min(100.0);
    }

    /// Check if patient needs rest based on biofeedback
    pub fn needs_rest(&self) -> bool {
        self.biofeedback.fatigue_level > 0.8
            || self.biofeedback.pain_reported > 6
            || self.biofeedback.heart_rate_bpm.map_or(false, |hr| hr > 140)
    }

    /// Calculate overall performance score
    pub fn performance_score(&self) -> f64 {
        let rom_score = self.rom_achievement_rate() * 0.4;
        let smoothness_score = self.performance.smoothness_score * 0.3;
        let accuracy_score = (1.0 - (self.performance.accuracy_cm / 10.0).min(1.0)) * 0.3;

        (rom_score + smoothness_score + accuracy_score).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exercise_progress() {
        let mut spec = RehabilitationSpec::default();
        spec.exercise.repetition = 5;
        spec.exercise.total_repetitions = 10;
        assert!((spec.exercise_progress() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_rom_achievement_rate() {
        let mut spec = RehabilitationSpec::default();
        spec.performance.rom_achieved_deg = 45.0;
        spec.performance.rom_target_deg = 90.0;
        assert!((spec.rom_achievement_rate() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_calculate_assistance() {
        let mut spec = RehabilitationSpec::default();
        spec.patient_effort.active_participation = 0.5;
        spec.exercise.difficulty_level = 5;
        let assistance = spec.calculate_assistance();
        assert!(assistance >= 0.0 && assistance <= 1.0);
    }

    #[test]
    fn test_trajectory_error() {
        let mut spec = RehabilitationSpec::default();
        spec.trajectory.current_position = Position3D::new(0.0, 0.0, 0.0);
        spec.trajectory.target_position = Position3D::new(3.0, 4.0, 0.0);
        assert!((spec.trajectory_error() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_needs_rest() {
        let mut spec = RehabilitationSpec::default();
        assert!(!spec.needs_rest());

        spec.biofeedback.fatigue_level = 0.9;
        assert!(spec.needs_rest());
    }
}
