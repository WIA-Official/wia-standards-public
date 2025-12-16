//! Brain-Computer Interface Gaming Integration
//!
//! Provides BCI-based game controls using motor imagery, SSVEP, P300, and other paradigms.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// BCI adapter for gaming
#[derive(Debug)]
pub struct BCIAdapter {
    config: BCIConfig,
    current_state: BCIState,
    command_history: Vec<(MentalCommand, Instant)>,
    fatigue_tracker: FatigueTracker,
    simulated_command: Option<MentalCommand>,
}

/// BCI configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BCIConfig {
    /// Enable BCI input
    pub enabled: bool,
    /// Minimum confidence for actions
    pub confidence_thresholds: ConfidenceThresholds,
    /// Command to game action mapping
    pub command_mapping: HashMap<String, String>,
    /// Fatigue detection settings
    pub fatigue_detection: FatigueDetectionConfig,
    /// Session settings
    pub session: SessionConfig,
}

/// Confidence thresholds for different action types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceThresholds {
    /// Threshold for movement commands
    pub movement: f32,
    /// Threshold for combat commands
    pub combat: f32,
    /// Threshold for critical actions
    pub critical: f32,
    /// Threshold for menu navigation
    pub menu: f32,
}

/// Fatigue detection settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FatigueDetectionConfig {
    pub enabled: bool,
    /// Maximum session duration before break reminder (minutes)
    pub max_session_mins: u32,
    /// Accuracy drop threshold for fatigue detection
    pub accuracy_drop_threshold: f32,
    /// Auto-pause on fatigue
    pub auto_pause: bool,
}

/// Session configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionConfig {
    /// Warmup period (seconds)
    pub warmup_secs: u32,
    /// Cooldown notifications
    pub cooldown_notifications: bool,
}

/// Current BCI state
#[derive(Debug, Clone)]
pub struct BCIState {
    /// Current detected mental command
    pub mental_command: Option<MentalCommand>,
    /// Confidence level (0.0-1.0)
    pub confidence: f32,
    /// Current fatigue level (0.0-1.0)
    pub fatigue_level: f32,
    /// Session duration
    pub session_duration: Duration,
    /// Last error potential detection
    pub last_error_potential: Option<Instant>,
    /// Session start time
    pub session_start: Option<Instant>,
}

/// Mental commands from BCI
#[derive(Debug, Clone, PartialEq)]
pub enum MentalCommand {
    /// Motor imagery of a limb
    MotorImagery { limb: ImaginaryLimb, intensity: f32 },
    /// SSVEP frequency detection
    SSVEP { frequency_hz: f32, target_id: u8 },
    /// P300 grid selection
    P300 { row: u8, col: u8 },
    /// Relaxation state
    Relax,
    /// Push mental command
    Push,
    /// Pull mental command
    Pull,
    /// Neutral/baseline
    Neutral,
}

/// Imaginary limb for motor imagery
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImaginaryLimb {
    LeftHand,
    RightHand,
    BothHands,
    LeftFoot,
    RightFoot,
    BothFeet,
    Tongue,
}

/// Fatigue tracking
#[derive(Debug)]
struct FatigueTracker {
    accuracy_history: Vec<f32>,
    baseline_accuracy: f32,
    current_level: f32,
    last_break: Option<Instant>,
}

/// BCI input for game processing
#[derive(Debug, Clone)]
pub struct BCIInput {
    /// Current mental command
    pub command: Option<MentalCommand>,
    /// Confidence level
    pub confidence: f32,
    /// Fatigue level
    pub fatigue_level: f32,
    /// Error potential detected (undo signal)
    pub error_potential: bool,
    /// Session is in warmup phase
    pub in_warmup: bool,
}

/// Game action from BCI
#[derive(Debug, Clone)]
pub enum BCIGameAction {
    /// Movement in direction
    Move { direction: MovementDirection, intensity: f32 },
    /// Attack/fire
    Attack { intensity: f32 },
    /// Defend/block
    Defend,
    /// Use ability
    UseAbility { slot: u8 },
    /// Select menu item
    SelectItem { row: u8, col: u8 },
    /// Confirm action
    Confirm,
    /// Cancel/undo
    Cancel,
    /// Pause for fatigue
    FatiguePause { recommended_break_mins: u32 },
}

/// Movement directions from motor imagery
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementDirection {
    Forward,
    Backward,
    Left,
    Right,
    Up,
    Down,
}

impl BCIAdapter {
    /// Create a new BCI adapter
    pub fn new() -> Self {
        Self {
            config: BCIConfig::default(),
            current_state: BCIState::default(),
            command_history: Vec::new(),
            fatigue_tracker: FatigueTracker::new(),
            simulated_command: None,
        }
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: BCIConfig) {
        self.config = config;
    }

    /// Start a BCI session
    pub fn start_session(&mut self) {
        self.current_state.session_start = Some(Instant::now());
        self.current_state.session_duration = Duration::ZERO;
        self.fatigue_tracker.reset();
    }

    /// Set simulated command for testing
    pub fn set_simulated_command(&mut self, command: MentalCommand, confidence: f32) {
        self.simulated_command = Some(command.clone());
        self.current_state.mental_command = Some(command);
        self.current_state.confidence = confidence;
    }

    /// Update BCI state with new data
    pub fn update(&mut self, command: Option<MentalCommand>, confidence: f32) {
        self.current_state.mental_command = command.clone();
        self.current_state.confidence = confidence;

        // Update session duration
        if let Some(start) = self.current_state.session_start {
            self.current_state.session_duration = start.elapsed();
        }

        // Update fatigue tracking
        self.fatigue_tracker.update_accuracy(confidence);
        self.current_state.fatigue_level = self.fatigue_tracker.current_level;

        // Store command history
        if let Some(cmd) = command {
            self.command_history.push((cmd, Instant::now()));
            // Keep only last 100 commands
            if self.command_history.len() > 100 {
                self.command_history.remove(0);
            }
        }
    }

    /// Report error potential detection
    pub fn report_error_potential(&mut self) {
        self.current_state.last_error_potential = Some(Instant::now());
    }

    /// Check if in warmup phase
    pub fn is_in_warmup(&self) -> bool {
        if let Some(start) = self.current_state.session_start {
            start.elapsed().as_secs() < self.config.session.warmup_secs as u64
        } else {
            true
        }
    }

    /// Check if fatigue break is recommended
    pub fn should_take_break(&self) -> Option<u32> {
        if !self.config.fatigue_detection.enabled {
            return None;
        }

        // Check session duration
        let session_mins = self.current_state.session_duration.as_secs() / 60;
        if session_mins >= self.config.fatigue_detection.max_session_mins as u64 {
            return Some(10);
        }

        // Check fatigue level
        if self.current_state.fatigue_level > 0.8 {
            return Some(5);
        }

        None
    }

    /// Get confidence threshold for action type
    fn get_threshold(&self, action_type: &str) -> f32 {
        match action_type {
            "movement" => self.config.confidence_thresholds.movement,
            "combat" => self.config.confidence_thresholds.combat,
            "critical" => self.config.confidence_thresholds.critical,
            "menu" => self.config.confidence_thresholds.menu,
            _ => 0.7,
        }
    }

    /// Poll for input
    pub fn poll(&mut self) -> Option<BCIInput> {
        // Use simulated command if set
        if let Some(ref cmd) = self.simulated_command {
            return Some(BCIInput {
                command: Some(cmd.clone()),
                confidence: self.current_state.confidence,
                fatigue_level: self.current_state.fatigue_level,
                error_potential: self.current_state.last_error_potential
                    .map(|t| t.elapsed() < Duration::from_millis(500))
                    .unwrap_or(false),
                in_warmup: self.is_in_warmup(),
            });
        }

        // Check for valid command
        if self.current_state.mental_command.is_none() {
            return None;
        }

        Some(BCIInput {
            command: self.current_state.mental_command.clone(),
            confidence: self.current_state.confidence,
            fatigue_level: self.current_state.fatigue_level,
            error_potential: self.current_state.last_error_potential
                .map(|t| t.elapsed() < Duration::from_millis(500))
                .unwrap_or(false),
            in_warmup: self.is_in_warmup(),
        })
    }

    /// Convert mental command to game action
    pub fn get_action(&self) -> Option<BCIGameAction> {
        // Check for fatigue pause
        if let Some(break_mins) = self.should_take_break() {
            if self.config.fatigue_detection.auto_pause {
                return Some(BCIGameAction::FatiguePause {
                    recommended_break_mins: break_mins,
                });
            }
        }

        // Check for error potential (undo)
        if let Some(error_time) = self.current_state.last_error_potential {
            if error_time.elapsed() < Duration::from_millis(500) {
                return Some(BCIGameAction::Cancel);
            }
        }

        let cmd = self.current_state.mental_command.as_ref()?;
        let confidence = self.current_state.confidence;

        match cmd {
            MentalCommand::MotorImagery { limb, intensity } => {
                if confidence < self.get_threshold("movement") {
                    return None;
                }

                let direction = match limb {
                    ImaginaryLimb::LeftHand => MovementDirection::Left,
                    ImaginaryLimb::RightHand => MovementDirection::Right,
                    ImaginaryLimb::BothFeet | ImaginaryLimb::LeftFoot => MovementDirection::Forward,
                    ImaginaryLimb::RightFoot => MovementDirection::Backward,
                    ImaginaryLimb::BothHands => MovementDirection::Up,
                    ImaginaryLimb::Tongue => MovementDirection::Down,
                };

                Some(BCIGameAction::Move {
                    direction,
                    intensity: *intensity,
                })
            }

            MentalCommand::Push => {
                if confidence < self.get_threshold("combat") {
                    return None;
                }
                Some(BCIGameAction::Attack { intensity: confidence })
            }

            MentalCommand::Pull => {
                if confidence < self.get_threshold("combat") {
                    return None;
                }
                Some(BCIGameAction::Defend)
            }

            MentalCommand::P300 { row, col } => {
                if confidence < self.get_threshold("menu") {
                    return None;
                }
                Some(BCIGameAction::SelectItem { row: *row, col: *col })
            }

            MentalCommand::SSVEP { target_id, .. } => {
                if confidence < self.get_threshold("menu") {
                    return None;
                }
                Some(BCIGameAction::UseAbility { slot: *target_id })
            }

            MentalCommand::Relax => Some(BCIGameAction::Confirm),

            MentalCommand::Neutral => None,
        }
    }
}

impl Default for BCIAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for BCIConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            confidence_thresholds: ConfidenceThresholds {
                movement: 0.6,
                combat: 0.8,
                critical: 0.9,
                menu: 0.5,
            },
            command_mapping: HashMap::new(),
            fatigue_detection: FatigueDetectionConfig {
                enabled: true,
                max_session_mins: 30,
                accuracy_drop_threshold: 0.2,
                auto_pause: false,
            },
            session: SessionConfig {
                warmup_secs: 30,
                cooldown_notifications: true,
            },
        }
    }
}

impl Default for BCIState {
    fn default() -> Self {
        Self {
            mental_command: None,
            confidence: 0.0,
            fatigue_level: 0.0,
            session_duration: Duration::ZERO,
            last_error_potential: None,
            session_start: None,
        }
    }
}

impl FatigueTracker {
    fn new() -> Self {
        Self {
            accuracy_history: Vec::new(),
            baseline_accuracy: 0.8,
            current_level: 0.0,
            last_break: None,
        }
    }

    fn reset(&mut self) {
        self.accuracy_history.clear();
        self.current_level = 0.0;
        self.last_break = Some(Instant::now());
    }

    fn update_accuracy(&mut self, accuracy: f32) {
        self.accuracy_history.push(accuracy);

        // Keep last 50 samples
        if self.accuracy_history.len() > 50 {
            self.accuracy_history.remove(0);
        }

        // Calculate current accuracy
        if self.accuracy_history.len() >= 10 {
            let recent_avg: f32 = self.accuracy_history.iter()
                .rev()
                .take(10)
                .sum::<f32>() / 10.0;

            // Compare to baseline
            let drop = self.baseline_accuracy - recent_avg;
            self.current_level = (drop / 0.3).clamp(0.0, 1.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bci_adapter_creation() {
        let adapter = BCIAdapter::new();
        assert!(adapter.current_state.mental_command.is_none());
    }

    #[test]
    fn test_motor_imagery_mapping() {
        let mut adapter = BCIAdapter::new();
        adapter.start_session();

        adapter.set_simulated_command(
            MentalCommand::MotorImagery {
                limb: ImaginaryLimb::LeftHand,
                intensity: 0.8,
            },
            0.9,
        );

        let action = adapter.get_action();
        assert!(matches!(
            action,
            Some(BCIGameAction::Move { direction: MovementDirection::Left, .. })
        ));
    }

    #[test]
    fn test_push_command() {
        let mut adapter = BCIAdapter::new();
        adapter.start_session();
        adapter.set_simulated_command(MentalCommand::Push, 0.9);

        let action = adapter.get_action();
        assert!(matches!(action, Some(BCIGameAction::Attack { .. })));
    }

    #[test]
    fn test_confidence_threshold() {
        let mut adapter = BCIAdapter::new();
        adapter.start_session();

        // Low confidence should not trigger action
        adapter.set_simulated_command(MentalCommand::Push, 0.3);

        let action = adapter.get_action();
        assert!(action.is_none());
    }

    #[test]
    fn test_error_potential() {
        let mut adapter = BCIAdapter::new();
        adapter.start_session();
        adapter.report_error_potential();

        let action = adapter.get_action();
        assert!(matches!(action, Some(BCIGameAction::Cancel)));
    }

    #[test]
    fn test_poll_input() {
        let mut adapter = BCIAdapter::new();
        adapter.start_session();
        adapter.set_simulated_command(MentalCommand::Relax, 0.8);

        let input = adapter.poll();
        assert!(input.is_some());
        assert!(input.unwrap().command.is_some());
    }
}
