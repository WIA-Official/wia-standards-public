//! Unified Input Processor
//!
//! Fuses inputs from multiple sources into game actions with priority resolution.

use super::{GameAction, AimTarget};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::time::{Duration, Instant};
use uuid::Uuid;

/// Unified input processor for multi-modal input fusion
#[derive(Debug)]
pub struct UnifiedInputProcessor {
    /// Input sources
    input_queue: VecDeque<TimestampedInput>,
    /// Fusion rules
    fusion_rules: Vec<FusionRule>,
    /// Output queue
    output_queue: VecDeque<GameAction>,
    /// Rule cooldowns
    cooldowns: Vec<(String, Instant)>,
    /// Configuration
    config: InputProcessorConfig,
}

/// Configuration for input processor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InputProcessorConfig {
    /// Maximum input age before discarding (ms)
    pub max_input_age_ms: u32,
    /// Enable input fusion
    pub fusion_enabled: bool,
    /// Maximum queue size
    pub max_queue_size: usize,
}

/// Input event with timestamp
#[derive(Debug, Clone)]
pub struct TimestampedInput {
    pub event: InputEvent,
    pub timestamp: Instant,
    pub source_priority: u8,
}

/// Input events from various sources
#[derive(Debug, Clone)]
pub enum InputEvent {
    /// Eye gaze input
    EyeGaze(super::EyeGazeInput),
    /// BCI input
    BCI(super::BCIInput),
    /// AAC input
    AAC(super::AACInput),
    /// Wheelchair input
    Wheelchair(super::WheelchairInput),
    /// Direct controller input
    Controller(ControllerInput),
    /// Custom input
    Custom { name: String, value: f32 },
}

/// Controller input data
#[derive(Debug, Clone)]
pub struct ControllerInput {
    pub buttons: u32,
    pub left_stick: (f32, f32),
    pub right_stick: (f32, f32),
    pub triggers: (f32, f32),
}

/// Input fusion rule
#[derive(Debug, Clone)]
pub struct FusionRule {
    /// Rule name
    pub name: String,
    /// Input conditions that must be met
    pub conditions: Vec<InputCondition>,
    /// Output action when conditions are met
    pub output: GameAction,
    /// Priority (higher = more important)
    pub priority: u8,
    /// Cooldown in milliseconds
    pub cooldown_ms: u32,
    /// All conditions must be met within this window (ms)
    pub time_window_ms: u32,
}

/// Input condition for fusion rules
#[derive(Debug, Clone)]
pub enum InputCondition {
    /// Gaze dwell on a point
    GazeDwell { min_duration_ms: u32 },
    /// Gaze at specific region
    GazeRegion { x_min: f32, x_max: f32, y_min: f32, y_max: f32 },
    /// Blink detected
    BlinkDetected,
    /// BCI command with confidence
    BCICommand { command_type: String, min_confidence: f32 },
    /// Voice command
    VoiceCommand { phrase: String },
    /// Symbol selected
    SymbolSelected { symbol: String },
    /// Wheelchair tilt
    WheelchairTilt { direction: TiltDirection, min_angle: f32 },
    /// Controller button
    ControllerButton { button: u32 },
    /// Any input from source
    AnyFromSource { source: InputSourceType },
}

/// Tilt directions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TiltDirection {
    Forward,
    Backward,
    Left,
    Right,
    Any,
}

/// Input source types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputSourceType {
    EyeGaze,
    BCI,
    AAC,
    Wheelchair,
    Controller,
}

/// Priority levels for input sources
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum InputPriority {
    /// Emergency (voice "help")
    Emergency = 10,
    /// Direct physical input
    Direct = 9,
    /// High confidence BCI
    BCIHigh = 8,
    /// Eye gaze with confirmation
    GazeConfirmed = 7,
    /// Voice command
    Voice = 6,
    /// AAC symbol
    Symbol = 5,
    /// Wheelchair motion
    Wheelchair = 4,
    /// Automation/macro
    Automation = 3,
}

impl UnifiedInputProcessor {
    /// Create a new input processor
    pub fn new() -> Self {
        let mut processor = Self {
            input_queue: VecDeque::new(),
            fusion_rules: Vec::new(),
            output_queue: VecDeque::new(),
            cooldowns: Vec::new(),
            config: InputProcessorConfig::default(),
        };

        // Add default fusion rules
        processor.add_default_rules();
        processor
    }

    /// Add default fusion rules
    fn add_default_rules(&mut self) {
        // Gaze + Blink = Confirm
        self.fusion_rules.push(FusionRule {
            name: "gaze_blink_confirm".to_string(),
            conditions: vec![
                InputCondition::GazeDwell { min_duration_ms: 500 },
                InputCondition::BlinkDetected,
            ],
            output: GameAction::Confirm,
            priority: 7,
            cooldown_ms: 500,
            time_window_ms: 1000,
        });

        // Gaze + BCI Push = Attack
        self.fusion_rules.push(FusionRule {
            name: "gaze_bci_attack".to_string(),
            conditions: vec![
                InputCondition::GazeRegion { x_min: 0.0, x_max: 1.0, y_min: 0.0, y_max: 1.0 },
                InputCondition::BCICommand {
                    command_type: "push".to_string(),
                    min_confidence: 0.7,
                },
            ],
            output: GameAction::Attack,
            priority: 8,
            cooldown_ms: 200,
            time_window_ms: 500,
        });

        // Wheelchair forward tilt = Move forward
        self.fusion_rules.push(FusionRule {
            name: "wheelchair_move_forward".to_string(),
            conditions: vec![
                InputCondition::WheelchairTilt {
                    direction: TiltDirection::Forward,
                    min_angle: 0.2,
                },
            ],
            output: GameAction::Move {
                direction: (0.0, 1.0),
                speed: 1.0,
            },
            priority: 4,
            cooldown_ms: 0,
            time_window_ms: 100,
        });
    }

    /// Add a fusion rule
    pub fn add_rule(&mut self, rule: FusionRule) {
        self.fusion_rules.push(rule);
        // Sort by priority (highest first)
        self.fusion_rules.sort_by(|a, b| b.priority.cmp(&a.priority));
    }

    /// Add input to processing queue
    pub fn add_input(&mut self, event: InputEvent) {
        let priority = Self::get_source_priority(&event);

        self.input_queue.push_back(TimestampedInput {
            event,
            timestamp: Instant::now(),
            source_priority: priority,
        });

        // Limit queue size
        while self.input_queue.len() > self.config.max_queue_size {
            self.input_queue.pop_front();
        }
    }

    /// Get priority for input source
    fn get_source_priority(event: &InputEvent) -> u8 {
        match event {
            InputEvent::Controller(_) => InputPriority::Direct as u8,
            InputEvent::BCI(bci) if bci.confidence > 0.9 => InputPriority::BCIHigh as u8,
            InputEvent::EyeGaze(_) => InputPriority::GazeConfirmed as u8,
            InputEvent::AAC(aac) => {
                if aac.voice_command.is_some() {
                    InputPriority::Voice as u8
                } else {
                    InputPriority::Symbol as u8
                }
            }
            InputEvent::Wheelchair(_) => InputPriority::Wheelchair as u8,
            InputEvent::BCI(_) => InputPriority::Automation as u8,
            InputEvent::Custom { .. } => InputPriority::Automation as u8,
        }
    }

    /// Process inputs and produce game actions
    pub fn process(&mut self) -> Vec<GameAction> {
        let now = Instant::now();

        // Clean up old inputs
        self.cleanup_old_inputs(now);

        // Clean up expired cooldowns
        self.cooldowns.retain(|(_, time)| {
            now.duration_since(*time) < Duration::from_secs(10)
        });

        // Process direct inputs first
        let mut actions = self.process_direct_inputs();

        // Try to match fusion rules
        if self.config.fusion_enabled {
            if let Some(action) = self.try_fusion_rules(now) {
                actions.push(action);
            }
        }

        // Add to output queue
        self.output_queue.extend(actions.clone());

        actions
    }

    /// Clean up old inputs
    fn cleanup_old_inputs(&mut self, now: Instant) {
        let max_age = Duration::from_millis(self.config.max_input_age_ms as u64);
        self.input_queue.retain(|input| {
            now.duration_since(input.timestamp) < max_age
        });
    }

    /// Process direct inputs (non-fusion)
    fn process_direct_inputs(&mut self) -> Vec<GameAction> {
        let mut actions = Vec::new();

        for input in &self.input_queue {
            match &input.event {
                InputEvent::EyeGaze(gaze) => {
                    if gaze.dwell_progress >= 1.0 {
                        actions.push(GameAction::Aim {
                            target: AimTarget::Point {
                                x: gaze.gaze_point.0,
                                y: gaze.gaze_point.1,
                            },
                            magnetism: 0.5,
                        });
                    }
                }
                InputEvent::AAC(aac) => {
                    if let Some(ref cmd) = aac.voice_command {
                        if cmd.phrase.to_lowercase() == "help" {
                            actions.push(GameAction::Pause);
                        }
                    }
                }
                _ => {}
            }
        }

        actions
    }

    /// Try to match fusion rules
    fn try_fusion_rules(&mut self, now: Instant) -> Option<GameAction> {
        for rule in &self.fusion_rules {
            // Check cooldown
            if self.is_on_cooldown(&rule.name, now) {
                continue;
            }

            // Check if all conditions are met within time window
            if self.check_conditions(&rule.conditions, rule.time_window_ms, now) {
                // Set cooldown
                self.cooldowns.push((rule.name.clone(), now));

                return Some(rule.output.clone());
            }
        }

        None
    }

    /// Check if rule is on cooldown
    fn is_on_cooldown(&self, rule_name: &str, now: Instant) -> bool {
        for (name, time) in &self.cooldowns {
            if name == rule_name {
                // Find the rule to get cooldown duration
                for rule in &self.fusion_rules {
                    if rule.name == *name {
                        let cooldown = Duration::from_millis(rule.cooldown_ms as u64);
                        if now.duration_since(*time) < cooldown {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Check if all conditions are met
    fn check_conditions(&self, conditions: &[InputCondition], time_window_ms: u32, now: Instant) -> bool {
        let window = Duration::from_millis(time_window_ms as u64);

        for condition in conditions {
            let matched = self.input_queue.iter().any(|input| {
                if now.duration_since(input.timestamp) > window {
                    return false;
                }
                self.matches_condition(condition, &input.event)
            });

            if !matched {
                return false;
            }
        }

        true
    }

    /// Check if input matches condition
    fn matches_condition(&self, condition: &InputCondition, event: &InputEvent) -> bool {
        match (condition, event) {
            (InputCondition::GazeDwell { min_duration_ms }, InputEvent::EyeGaze(gaze)) => {
                gaze.dwell_progress >= (*min_duration_ms as f32 / 1000.0)
            }

            (InputCondition::GazeRegion { x_min, x_max, y_min, y_max }, InputEvent::EyeGaze(gaze)) => {
                gaze.gaze_point.0 >= *x_min
                    && gaze.gaze_point.0 <= *x_max
                    && gaze.gaze_point.1 >= *y_min
                    && gaze.gaze_point.1 <= *y_max
            }

            (InputCondition::BlinkDetected, InputEvent::EyeGaze(gaze)) => {
                gaze.blink_detected
            }

            (InputCondition::BCICommand { command_type, min_confidence }, InputEvent::BCI(bci)) => {
                if bci.confidence < *min_confidence {
                    return false;
                }
                // Simplified: check if any command matches type
                bci.command.is_some()
            }

            (InputCondition::VoiceCommand { phrase }, InputEvent::AAC(aac)) => {
                if let Some(ref cmd) = aac.voice_command {
                    cmd.phrase.to_lowercase().contains(&phrase.to_lowercase())
                } else {
                    false
                }
            }

            (InputCondition::SymbolSelected { symbol }, InputEvent::AAC(aac)) => {
                aac.symbol.as_ref() == Some(symbol)
            }

            (InputCondition::WheelchairTilt { direction, min_angle }, InputEvent::Wheelchair(wc)) => {
                let (x, y) = wc.tilt;
                match direction {
                    TiltDirection::Forward => y >= *min_angle,
                    TiltDirection::Backward => y <= -*min_angle,
                    TiltDirection::Left => x <= -*min_angle,
                    TiltDirection::Right => x >= *min_angle,
                    TiltDirection::Any => x.abs() >= *min_angle || y.abs() >= *min_angle,
                }
            }

            (InputCondition::ControllerButton { button }, InputEvent::Controller(ctrl)) => {
                (ctrl.buttons & button) != 0
            }

            (InputCondition::AnyFromSource { source }, event) => {
                match (source, event) {
                    (InputSourceType::EyeGaze, InputEvent::EyeGaze(_)) => true,
                    (InputSourceType::BCI, InputEvent::BCI(_)) => true,
                    (InputSourceType::AAC, InputEvent::AAC(_)) => true,
                    (InputSourceType::Wheelchair, InputEvent::Wheelchair(_)) => true,
                    (InputSourceType::Controller, InputEvent::Controller(_)) => true,
                    _ => false,
                }
            }

            _ => false,
        }
    }

    /// Get pending actions
    pub fn drain_actions(&mut self) -> Vec<GameAction> {
        self.output_queue.drain(..).collect()
    }

    /// Clear all inputs
    pub fn clear(&mut self) {
        self.input_queue.clear();
        self.output_queue.clear();
    }
}

impl Default for UnifiedInputProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for InputProcessorConfig {
    fn default() -> Self {
        Self {
            max_input_age_ms: 500,
            fusion_enabled: true,
            max_queue_size: 100,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_input_processor_creation() {
        let processor = UnifiedInputProcessor::new();
        assert!(!processor.fusion_rules.is_empty());
    }

    #[test]
    fn test_add_input() {
        let mut processor = UnifiedInputProcessor::new();

        processor.add_input(InputEvent::Custom {
            name: "test".to_string(),
            value: 1.0,
        });

        assert_eq!(processor.input_queue.len(), 1);
    }

    #[test]
    fn test_priority_ordering() {
        assert!(InputPriority::Emergency as u8 > InputPriority::Direct as u8);
        assert!(InputPriority::Direct as u8 > InputPriority::BCIHigh as u8);
        assert!(InputPriority::Voice as u8 > InputPriority::Wheelchair as u8);
    }

    #[test]
    fn test_add_custom_rule() {
        let mut processor = UnifiedInputProcessor::new();
        let initial_count = processor.fusion_rules.len();

        processor.add_rule(FusionRule {
            name: "custom_rule".to_string(),
            conditions: vec![],
            output: GameAction::Jump,
            priority: 5,
            cooldown_ms: 100,
            time_window_ms: 500,
        });

        assert_eq!(processor.fusion_rules.len(), initial_count + 1);
    }

    #[test]
    fn test_input_cleanup() {
        let mut processor = UnifiedInputProcessor::new();
        processor.config.max_input_age_ms = 1;

        processor.add_input(InputEvent::Custom {
            name: "test".to_string(),
            value: 1.0,
        });

        std::thread::sleep(std::time::Duration::from_millis(10));
        processor.process();

        assert!(processor.input_queue.is_empty());
    }
}
