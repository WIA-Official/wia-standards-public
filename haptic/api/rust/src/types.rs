//! Core type definitions for the WIA Haptic Standard

use core::fmt;

/// Waveform types for haptic signal generation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WaveformType {
    Sine = 0,
    Square = 1,
    Triangle = 2,
    Sawtooth = 3,
    Noise = 4,
}

impl Default for WaveformType {
    fn default() -> Self {
        Self::Sine
    }
}

/// Noise types for noise waveform
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NoiseType {
    White = 0,
    Pink = 1,
    Brown = 2,
}

/// ADSR envelope for shaping haptic signal amplitude
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Envelope {
    /// Attack time in milliseconds (0-500)
    pub attack: u16,
    /// Decay time in milliseconds (0-500)
    pub decay: u16,
    /// Sustain level (0.0-1.0)
    pub sustain: f32,
    /// Release time in milliseconds (0-500)
    pub release: u16,
}

impl Default for Envelope {
    fn default() -> Self {
        Self {
            attack: 10,
            decay: 50,
            sustain: 0.7,
            release: 100,
        }
    }
}

impl Envelope {
    /// Sharp envelope preset - quick tap
    pub const SHARP: Self = Self {
        attack: 5,
        decay: 20,
        sustain: 0.0,
        release: 30,
    };

    /// Punch envelope preset - impact feel
    pub const PUNCH: Self = Self {
        attack: 10,
        decay: 50,
        sustain: 0.3,
        release: 50,
    };

    /// Smooth envelope preset - gentle wave
    pub const SMOOTH: Self = Self {
        attack: 100,
        decay: 100,
        sustain: 0.7,
        release: 150,
    };

    /// Pulse envelope preset - on/off
    pub const PULSE: Self = Self {
        attack: 5,
        decay: 0,
        sustain: 1.0,
        release: 5,
    };

    /// Swell envelope preset - growing intensity
    pub const SWELL: Self = Self {
        attack: 200,
        decay: 50,
        sustain: 0.8,
        release: 200,
    };

    /// Fade envelope preset - gradual decay
    pub const FADE: Self = Self {
        attack: 50,
        decay: 200,
        sustain: 0.5,
        release: 300,
    };
}

/// Frequency bands for haptic feedback
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyBand {
    /// Very low frequency: 1-30 Hz
    VeryLow,
    /// Low frequency: 30-80 Hz
    Low,
    /// Mid frequency: 80-150 Hz
    Mid,
    /// High frequency: 150-250 Hz
    High,
    /// Very high frequency: 250-300 Hz
    VeryHigh,
}

impl FrequencyBand {
    /// Get the frequency range for this band
    pub fn range(&self) -> (u16, u16) {
        match self {
            Self::VeryLow => (1, 30),
            Self::Low => (30, 80),
            Self::Mid => (80, 150),
            Self::High => (150, 250),
            Self::VeryHigh => (250, 300),
        }
    }

    /// Get the center frequency for this band
    pub fn center(&self) -> u16 {
        let (min, max) = self.range();
        (min + max) / 2
    }
}

/// Intensity levels for haptic feedback
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntensityLevel {
    /// Subtle intensity: 0.2
    Subtle,
    /// Light intensity: 0.4
    Light,
    /// Medium intensity: 0.6
    Medium,
    /// Strong intensity: 0.8
    Strong,
    /// Maximum intensity: 1.0
    Maximum,
}

impl IntensityLevel {
    /// Get the intensity value for this level
    pub fn value(&self) -> f32 {
        match self {
            Self::Subtle => 0.2,
            Self::Light => 0.4,
            Self::Medium => 0.6,
            Self::Strong => 0.8,
            Self::Maximum => 1.0,
        }
    }
}

/// Haptic actuator types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ActuatorType {
    /// Eccentric Rotating Mass
    Erm = 0,
    /// Linear Resonant Actuator
    Lra = 1,
    /// Piezoelectric
    Piezo = 2,
    /// Voice Coil
    VoiceCoil = 3,
}

/// Body locations for haptic actuator placement
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BodyLocation {
    // Head
    ForeheadLeft = 0,
    ForeheadCenter = 1,
    ForeheadRight = 2,
    TempleLeft = 3,
    TempleRight = 4,

    // Wrists
    WristLeftDorsal = 10,
    WristLeftVolar = 11,
    WristRightDorsal = 12,
    WristRightVolar = 13,

    // Hands
    PalmLeft = 20,
    PalmRight = 21,

    // Torso
    ChestLeft = 30,
    ChestCenter = 31,
    ChestRight = 32,
    BackUpperLeft = 33,
    BackUpperCenter = 34,
    BackUpperRight = 35,

    // Waist
    WaistLeft = 40,
    WaistFront = 41,
    WaistRight = 42,
    WaistBack = 43,
}

/// Error type for haptic operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HapticError {
    /// Driver not initialized
    NotInitialized,
    /// Invalid parameter
    InvalidParameter,
    /// Hardware error
    HardwareError,
    /// Operation not supported
    NotSupported,
    /// Buffer overflow
    BufferOverflow,
    /// Timeout
    Timeout,
}

impl fmt::Display for HapticError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotInitialized => write!(f, "Driver not initialized"),
            Self::InvalidParameter => write!(f, "Invalid parameter"),
            Self::HardwareError => write!(f, "Hardware error"),
            Self::NotSupported => write!(f, "Operation not supported"),
            Self::BufferOverflow => write!(f, "Buffer overflow"),
            Self::Timeout => write!(f, "Operation timeout"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for HapticError {}

/// Result type for haptic operations
pub type HapticResult<T> = Result<T, HapticError>;
