//! Haptic Gaming Integration
//!
//! Provides haptic feedback for game events, including spatial audio to haptic conversion.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// Haptic adapter for gaming
#[derive(Debug)]
pub struct HapticAdapter {
    config: HapticConfig,
    feedback_queue: VecDeque<HapticFeedback>,
    active_patterns: Vec<ActivePattern>,
    spatial_converter: SpatialHapticConverter,
}

/// Haptic configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticConfig {
    /// Enable haptic feedback
    pub enabled: bool,
    /// Overall intensity (0.0-1.0)
    pub intensity: f32,
    /// Enable spatial haptics for deaf users
    pub spatial_enabled: bool,
    /// Enable directional feedback
    pub directional_enabled: bool,
    /// Priority threshold (lower = more feedback)
    pub priority_threshold: HapticPriority,
}

/// Haptic feedback event
#[derive(Debug, Clone)]
pub struct HapticFeedback {
    /// Source game event
    pub event: GameEventType,
    /// Haptic pattern to play
    pub pattern: HapticPattern,
    /// Direction (0-360 degrees, None for omnidirectional)
    pub direction: Option<f32>,
    /// Intensity (0.0-1.0)
    pub intensity: f32,
    /// Duration in milliseconds
    pub duration_ms: u32,
    /// Priority level
    pub priority: HapticPriority,
    /// Timestamp
    pub timestamp: Instant,
}

/// Game event types for haptic feedback
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GameEventType {
    DamageTaken,
    HealthLow,
    EnemyNearby,
    WeaponFire,
    Explosion,
    Footsteps,
    UISelection,
    Achievement,
    Victory,
    Defeat,
    Collision,
    Environment,
    Custom(String),
}

/// Haptic patterns
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HapticPattern {
    /// Single pulse
    Pulse { count: u8, interval_ms: u32 },
    /// Continuous rumble with attack/decay
    Rumble { attack_ms: u32, sustain_ms: u32, decay_ms: u32 },
    /// Heartbeat rhythm
    Heartbeat { bpm: u32 },
    /// Directional pulse (requires spatial)
    Directional { angle: f32, spread: f32 },
    /// Wave pattern
    Wave { frequency_hz: f32, duration_ms: u32 },
    /// Custom waveform
    Custom { waveform: Vec<f32>, sample_rate_hz: u32 },
}

/// Priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum HapticPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

/// Active haptic pattern being played
#[derive(Debug)]
struct ActivePattern {
    feedback: HapticFeedback,
    start_time: Instant,
    progress: f32,
}

/// Spatial audio to haptic converter
#[derive(Debug)]
pub struct SpatialHapticConverter {
    /// Enable spatial conversion
    pub enabled: bool,
    /// Sensitivity for sound detection
    pub sensitivity: f32,
    /// Minimum sound intensity to trigger haptic
    pub threshold: f32,
    /// Sound type to haptic pattern mapping
    pub sound_patterns: Vec<SoundHapticMapping>,
}

/// Mapping from sound types to haptic patterns
#[derive(Debug, Clone)]
pub struct SoundHapticMapping {
    /// Sound category
    pub sound_type: SoundType,
    /// Haptic pattern to use
    pub pattern: HapticPattern,
    /// Intensity multiplier
    pub intensity_multiplier: f32,
}

/// Types of sounds for haptic conversion
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SoundType {
    Gunshot,
    Footstep,
    Explosion,
    Voice,
    Alert,
    Music,
    Environment,
}

/// Direction in 8 cardinal/intercardinal directions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction8 {
    Front,
    FrontRight,
    Right,
    BackRight,
    Back,
    BackLeft,
    Left,
    FrontLeft,
}

impl HapticAdapter {
    /// Create a new haptic adapter
    pub fn new() -> Self {
        Self {
            config: HapticConfig::default(),
            feedback_queue: VecDeque::new(),
            active_patterns: Vec::new(),
            spatial_converter: SpatialHapticConverter::new(),
        }
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: HapticConfig) {
        self.spatial_converter.enabled = config.spatial_enabled;
        self.config = config;
    }

    /// Trigger haptic feedback for a game event
    pub fn trigger(&mut self, event: super::GameEvent) {
        if !self.config.enabled {
            return;
        }

        let feedback = self.event_to_feedback(event);

        // Check priority threshold
        if feedback.priority < self.config.priority_threshold {
            return;
        }

        self.feedback_queue.push_back(feedback);
    }

    /// Convert game event to haptic feedback
    fn event_to_feedback(&self, event: super::GameEvent) -> HapticFeedback {
        use super::GameEvent;

        match event {
            GameEvent::DamageTaken { amount, direction } => HapticFeedback {
                event: GameEventType::DamageTaken,
                pattern: HapticPattern::Pulse {
                    count: (amount / 10.0).clamp(1.0, 5.0) as u8,
                    interval_ms: 50,
                },
                direction,
                intensity: (amount / 100.0).clamp(0.3, 1.0),
                duration_ms: 200,
                priority: HapticPriority::High,
                timestamp: Instant::now(),
            },

            GameEvent::HealthLow { percent } => HapticFeedback {
                event: GameEventType::HealthLow,
                pattern: HapticPattern::Heartbeat {
                    bpm: (120.0 - percent * 60.0) as u32,
                },
                direction: None,
                intensity: 0.5,
                duration_ms: 0, // Continuous until health restored
                priority: HapticPriority::Normal,
                timestamp: Instant::now(),
            },

            GameEvent::EnemyNearby { distance, direction } => HapticFeedback {
                event: GameEventType::EnemyNearby,
                pattern: HapticPattern::Directional {
                    angle: direction,
                    spread: 30.0,
                },
                direction: Some(direction),
                intensity: (1.0 - distance / 10.0).clamp(0.2, 0.6),
                duration_ms: 500,
                priority: HapticPriority::Normal,
                timestamp: Instant::now(),
            },

            GameEvent::WeaponFire { weapon_type } => {
                let (intensity, duration) = match weapon_type.as_str() {
                    "pistol" => (0.4, 100),
                    "rifle" => (0.6, 80),
                    "shotgun" => (0.9, 150),
                    "sniper" => (0.8, 200),
                    _ => (0.5, 100),
                };
                HapticFeedback {
                    event: GameEventType::WeaponFire,
                    pattern: HapticPattern::Rumble {
                        attack_ms: 10,
                        sustain_ms: duration / 2,
                        decay_ms: duration / 2,
                    },
                    direction: None,
                    intensity,
                    duration_ms: duration,
                    priority: HapticPriority::High,
                    timestamp: Instant::now(),
                }
            }

            GameEvent::Explosion { distance, direction } => HapticFeedback {
                event: GameEventType::Explosion,
                pattern: HapticPattern::Rumble {
                    attack_ms: 50,
                    sustain_ms: 400,
                    decay_ms: 350,
                },
                direction,
                intensity: (1.0 - distance / 20.0).clamp(0.3, 1.0),
                duration_ms: 800,
                priority: HapticPriority::Critical,
                timestamp: Instant::now(),
            },

            GameEvent::Footsteps { direction, distance } => HapticFeedback {
                event: GameEventType::Footsteps,
                pattern: HapticPattern::Pulse {
                    count: 1,
                    interval_ms: 0,
                },
                direction: Some(direction),
                intensity: (0.3 - distance / 20.0).clamp(0.1, 0.3),
                duration_ms: 50,
                priority: HapticPriority::Low,
                timestamp: Instant::now(),
            },

            GameEvent::UISelection => HapticFeedback {
                event: GameEventType::UISelection,
                pattern: HapticPattern::Pulse {
                    count: 1,
                    interval_ms: 0,
                },
                direction: None,
                intensity: 0.2,
                duration_ms: 50,
                priority: HapticPriority::Low,
                timestamp: Instant::now(),
            },

            GameEvent::Achievement { .. } => HapticFeedback {
                event: GameEventType::Achievement,
                pattern: HapticPattern::Wave {
                    frequency_hz: 5.0,
                    duration_ms: 1000,
                },
                direction: None,
                intensity: 0.6,
                duration_ms: 1000,
                priority: HapticPriority::Normal,
                timestamp: Instant::now(),
            },

            GameEvent::Victory => HapticFeedback {
                event: GameEventType::Victory,
                pattern: HapticPattern::Wave {
                    frequency_hz: 3.0,
                    duration_ms: 2000,
                },
                direction: None,
                intensity: 0.8,
                duration_ms: 2000,
                priority: HapticPriority::High,
                timestamp: Instant::now(),
            },

            GameEvent::Defeat => HapticFeedback {
                event: GameEventType::Defeat,
                pattern: HapticPattern::Rumble {
                    attack_ms: 500,
                    sustain_ms: 500,
                    decay_ms: 1000,
                },
                direction: None,
                intensity: 0.5,
                duration_ms: 2000,
                priority: HapticPriority::Normal,
                timestamp: Instant::now(),
            },

            GameEvent::Collision { intensity } => HapticFeedback {
                event: GameEventType::Collision,
                pattern: HapticPattern::Pulse {
                    count: 1,
                    interval_ms: 0,
                },
                direction: None,
                intensity,
                duration_ms: 100,
                priority: HapticPriority::Normal,
                timestamp: Instant::now(),
            },

            GameEvent::EnvironmentEffect { effect_type } => HapticFeedback {
                event: GameEventType::Environment,
                pattern: HapticPattern::Wave {
                    frequency_hz: 2.0,
                    duration_ms: 500,
                },
                direction: None,
                intensity: 0.3,
                duration_ms: 500,
                priority: HapticPriority::Low,
                timestamp: Instant::now(),
            },
        }
    }

    /// Convert spatial audio to haptic feedback (for deaf users)
    pub fn convert_spatial_audio(
        &mut self,
        sound_type: SoundType,
        direction: f32,
        intensity: f32,
    ) {
        if !self.config.spatial_enabled {
            return;
        }

        if intensity < self.spatial_converter.threshold {
            return;
        }

        let pattern = match sound_type {
            SoundType::Gunshot => HapticPattern::Pulse { count: 2, interval_ms: 30 },
            SoundType::Footstep => HapticPattern::Pulse { count: 1, interval_ms: 0 },
            SoundType::Explosion => HapticPattern::Rumble {
                attack_ms: 50,
                sustain_ms: 300,
                decay_ms: 300,
            },
            SoundType::Voice => HapticPattern::Wave { frequency_hz: 8.0, duration_ms: 200 },
            SoundType::Alert => HapticPattern::Pulse { count: 3, interval_ms: 100 },
            _ => HapticPattern::Pulse { count: 1, interval_ms: 0 },
        };

        let feedback = HapticFeedback {
            event: GameEventType::Custom(format!("spatial_{:?}", sound_type)),
            pattern,
            direction: Some(direction),
            intensity: intensity * self.spatial_converter.sensitivity,
            duration_ms: 200,
            priority: HapticPriority::Normal,
            timestamp: Instant::now(),
        };

        self.feedback_queue.push_back(feedback);
    }

    /// Get next feedback from queue
    pub fn poll(&mut self) -> Option<HapticFeedback> {
        self.feedback_queue.pop_front()
    }

    /// Get all pending feedback
    pub fn drain(&mut self) -> Vec<HapticFeedback> {
        self.feedback_queue.drain(..).collect()
    }

    /// Update active patterns (call each frame)
    pub fn update(&mut self) {
        let now = Instant::now();

        // Remove completed patterns
        self.active_patterns.retain(|p| {
            let elapsed = now.duration_since(p.start_time).as_millis() as u32;
            elapsed < p.feedback.duration_ms || p.feedback.duration_ms == 0
        });
    }

    /// Get direction as 8-way compass
    pub fn direction_to_8way(angle: f32) -> Direction8 {
        let normalized = ((angle % 360.0) + 360.0) % 360.0;

        match normalized as u32 {
            0..=22 | 338..=360 => Direction8::Front,
            23..=67 => Direction8::FrontRight,
            68..=112 => Direction8::Right,
            113..=157 => Direction8::BackRight,
            158..=202 => Direction8::Back,
            203..=247 => Direction8::BackLeft,
            248..=292 => Direction8::Left,
            293..=337 => Direction8::FrontLeft,
            _ => Direction8::Front,
        }
    }
}

impl Default for HapticAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for HapticConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            intensity: 0.8,
            spatial_enabled: true,
            directional_enabled: true,
            priority_threshold: HapticPriority::Low,
        }
    }
}

impl SpatialHapticConverter {
    fn new() -> Self {
        Self {
            enabled: true,
            sensitivity: 1.0,
            threshold: 0.1,
            sound_patterns: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haptic_adapter_creation() {
        let adapter = HapticAdapter::new();
        assert!(adapter.config.enabled);
    }

    #[test]
    fn test_trigger_damage() {
        let mut adapter = HapticAdapter::new();
        adapter.trigger(super::super::GameEvent::DamageTaken {
            amount: 50.0,
            direction: Some(90.0),
        });

        let feedback = adapter.poll();
        assert!(feedback.is_some());
        assert_eq!(feedback.unwrap().event, GameEventType::DamageTaken);
    }

    #[test]
    fn test_spatial_audio_conversion() {
        let mut adapter = HapticAdapter::new();
        adapter.convert_spatial_audio(SoundType::Gunshot, 45.0, 0.8);

        let feedback = adapter.poll();
        assert!(feedback.is_some());
    }

    #[test]
    fn test_direction_8way() {
        assert_eq!(HapticAdapter::direction_to_8way(0.0), Direction8::Front);
        assert_eq!(HapticAdapter::direction_to_8way(90.0), Direction8::Right);
        assert_eq!(HapticAdapter::direction_to_8way(180.0), Direction8::Back);
        assert_eq!(HapticAdapter::direction_to_8way(270.0), Direction8::Left);
    }

    #[test]
    fn test_priority_threshold() {
        let mut adapter = HapticAdapter::new();
        adapter.config.priority_threshold = HapticPriority::High;

        // Low priority should be filtered
        adapter.trigger(super::super::GameEvent::UISelection);
        assert!(adapter.poll().is_none());

        // High priority should pass
        adapter.trigger(super::super::GameEvent::DamageTaken {
            amount: 50.0,
            direction: None,
        });
        assert!(adapter.poll().is_some());
    }
}
