//! Haptic pattern definitions

use crate::types::{Envelope, WaveformType, BodyLocation};

/// A single haptic primitive - the basic building block
#[derive(Debug, Clone)]
pub struct HapticPrimitive {
    /// Unique identifier
    pub id: &'static str,
    /// Human-readable name
    pub name: &'static str,
    /// Waveform type
    pub waveform: WaveformType,
    /// ADSR envelope
    pub envelope: Envelope,
    /// Frequency in Hz (1-300)
    pub frequency: u16,
    /// Intensity (0.0-1.0)
    pub intensity: f32,
    /// Duration in milliseconds
    pub duration: u16,
}

impl HapticPrimitive {
    /// Create a new haptic primitive
    pub const fn new(
        id: &'static str,
        name: &'static str,
        waveform: WaveformType,
        envelope: Envelope,
        frequency: u16,
        intensity: f32,
        duration: u16,
    ) -> Self {
        Self {
            id,
            name,
            waveform,
            envelope,
            frequency,
            intensity,
            duration,
        }
    }

    /// Quick tick primitive
    pub const fn tick() -> Self {
        Self {
            id: "tick",
            name: "Tick",
            waveform: WaveformType::Square,
            envelope: Envelope::SHARP,
            frequency: 150,
            intensity: 0.6,
            duration: 20,
        }
    }

    /// Buzz primitive
    pub const fn buzz() -> Self {
        Self {
            id: "buzz",
            name: "Buzz",
            waveform: WaveformType::Square,
            envelope: Envelope::PUNCH,
            frequency: 180,
            intensity: 0.7,
            duration: 100,
        }
    }

    /// Warning pulse primitive
    pub const fn warning_pulse() -> Self {
        Self {
            id: "warning_pulse",
            name: "Warning Pulse",
            waveform: WaveformType::Sine,
            envelope: Envelope {
                attack: 50,
                decay: 100,
                sustain: 0.5,
                release: 100,
            },
            frequency: 200,
            intensity: 0.8,
            duration: 300,
        }
    }

    /// Soft wave primitive
    pub const fn soft_wave() -> Self {
        Self {
            id: "soft_wave",
            name: "Soft Wave",
            waveform: WaveformType::Sine,
            envelope: Envelope::SMOOTH,
            frequency: 60,
            intensity: 0.4,
            duration: 400,
        }
    }

    /// Click primitive
    pub const fn click() -> Self {
        Self {
            id: "click",
            name: "Click",
            waveform: WaveformType::Square,
            envelope: Envelope {
                attack: 2,
                decay: 5,
                sustain: 0.0,
                release: 3,
            },
            frequency: 200,
            intensity: 0.5,
            duration: 10,
        }
    }
}

/// A step in a haptic sequence
#[derive(Debug, Clone)]
pub struct SequenceStep {
    /// The primitive to play
    pub primitive: HapticPrimitive,
    /// Delay before this step in milliseconds
    pub delay_before: u16,
    /// Number of times to repeat
    pub repeat_count: u8,
    /// Delay between repeats in milliseconds
    pub repeat_delay: u16,
    /// Intensity scale factor (0.0-2.0)
    pub intensity_scale: f32,
}

impl SequenceStep {
    /// Create a new sequence step
    pub const fn new(primitive: HapticPrimitive) -> Self {
        Self {
            primitive,
            delay_before: 0,
            repeat_count: 1,
            repeat_delay: 0,
            intensity_scale: 1.0,
        }
    }

    /// Set delay before step
    pub const fn with_delay(mut self, delay_ms: u16) -> Self {
        self.delay_before = delay_ms;
        self
    }

    /// Set repeat count
    pub const fn with_repeat(mut self, count: u8, delay_ms: u16) -> Self {
        self.repeat_count = count;
        self.repeat_delay = delay_ms;
        self
    }

    /// Set intensity scale
    pub const fn with_intensity_scale(mut self, scale: f32) -> Self {
        self.intensity_scale = scale;
        self
    }
}

/// A sequence of haptic primitives
#[derive(Debug, Clone)]
pub struct HapticSequence<const N: usize> {
    /// Unique identifier
    pub id: &'static str,
    /// Human-readable name
    pub name: &'static str,
    /// Steps in the sequence
    pub steps: [SequenceStep; N],
    /// Whether to loop
    pub do_loop: bool,
    /// Number of times to loop (0 = infinite)
    pub loop_count: u8,
}

impl<const N: usize> HapticSequence<N> {
    /// Calculate total duration of the sequence
    pub fn total_duration(&self) -> u32 {
        let mut duration: u32 = 0;
        for step in &self.steps {
            duration += step.delay_before as u32;
            let repeat_count = step.repeat_count as u32;
            duration += step.primitive.duration as u32 * repeat_count;
            duration += step.repeat_delay as u32 * (repeat_count.saturating_sub(1));
        }
        if self.do_loop && self.loop_count > 0 {
            duration *= self.loop_count as u32;
        }
        duration
    }
}

/// Pre-defined success sequence
pub fn success_sequence() -> HapticSequence<2> {
    HapticSequence {
        id: "success",
        name: "Success",
        steps: [
            SequenceStep::new(HapticPrimitive::tick()),
            SequenceStep::new(HapticPrimitive::tick()).with_delay(100),
        ],
        do_loop: false,
        loop_count: 1,
    }
}

/// Pre-defined failure sequence
pub fn failure_sequence() -> HapticSequence<2> {
    HapticSequence {
        id: "failure",
        name: "Failure",
        steps: [
            SequenceStep::new(HapticPrimitive::buzz()),
            SequenceStep::new(HapticPrimitive::buzz())
                .with_delay(150)
                .with_intensity_scale(0.7),
        ],
        do_loop: false,
        loop_count: 1,
    }
}

/// Spatial actuation - pattern at a body location
#[derive(Debug, Clone)]
pub struct SpatialActuation {
    /// Body location
    pub location: BodyLocation,
    /// Primitive to play
    pub primitive: HapticPrimitive,
    /// Start time offset in milliseconds
    pub start_time: u16,
    /// Intensity scale factor
    pub intensity_scale: f32,
}

impl SpatialActuation {
    /// Create a new spatial actuation
    pub const fn new(location: BodyLocation, primitive: HapticPrimitive) -> Self {
        Self {
            location,
            primitive,
            start_time: 0,
            intensity_scale: 1.0,
        }
    }

    /// Set start time offset
    pub const fn at_time(mut self, time_ms: u16) -> Self {
        self.start_time = time_ms;
        self
    }

    /// Set intensity scale
    pub const fn with_intensity(mut self, scale: f32) -> Self {
        self.intensity_scale = scale;
        self
    }
}

/// Direction for navigation patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    Left,
    Right,
    Up,
    Down,
}

/// Create a navigation pulse for a direction
pub fn navigation_pulse(direction: Direction, intensity: f32) -> HapticPrimitive {
    HapticPrimitive {
        id: match direction {
            Direction::Forward => "nav_forward",
            Direction::Backward => "nav_backward",
            Direction::Left => "nav_left",
            Direction::Right => "nav_right",
            Direction::Up => "nav_up",
            Direction::Down => "nav_down",
        },
        name: "Navigation Pulse",
        waveform: WaveformType::Sine,
        envelope: Envelope::SMOOTH,
        frequency: 80,
        intensity,
        duration: 300,
    }
}
