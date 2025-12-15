//! Haptic driver trait and base implementations

use crate::types::{HapticError, HapticResult, ActuatorType, BodyLocation};
use crate::patterns::{HapticPrimitive, HapticSequence, SequenceStep};

/// Device capabilities
#[derive(Debug, Clone)]
pub struct HapticCapabilities {
    /// Actuator type
    pub actuator_type: ActuatorType,
    /// Minimum supported frequency in Hz
    pub min_frequency: u16,
    /// Maximum supported frequency in Hz
    pub max_frequency: u16,
    /// Available body locations
    pub locations: &'static [BodyLocation],
    /// Maximum intensity (0.0-1.0)
    pub max_intensity: f32,
    /// Response latency in milliseconds
    pub latency: u16,
    /// Number of independent actuators
    pub actuator_count: u8,
    /// Supports custom waveforms
    pub supports_custom_waveforms: bool,
    /// Supports amplitude modulation
    pub supports_amplitude_modulation: bool,
    /// Supports frequency modulation
    pub supports_frequency_modulation: bool,
}

impl Default for HapticCapabilities {
    fn default() -> Self {
        Self {
            actuator_type: ActuatorType::Erm,
            min_frequency: 50,
            max_frequency: 200,
            locations: &[BodyLocation::WristLeftDorsal],
            max_intensity: 1.0,
            latency: 20,
            actuator_count: 1,
            supports_custom_waveforms: false,
            supports_amplitude_modulation: true,
            supports_frequency_modulation: false,
        }
    }
}

/// Core haptic driver trait
///
/// Implement this trait for specific hardware drivers.
pub trait HapticDriver {
    /// Initialize the driver
    fn init(&mut self) -> HapticResult<()>;

    /// Deinitialize the driver
    fn deinit(&mut self) -> HapticResult<()>;

    /// Check if driver is initialized
    fn is_initialized(&self) -> bool;

    /// Get device capabilities
    fn capabilities(&self) -> &HapticCapabilities;

    /// Set vibration frequency in Hz
    fn set_frequency(&mut self, hz: u16) -> HapticResult<()>;

    /// Set vibration amplitude (0.0-1.0)
    fn set_amplitude(&mut self, amplitude: f32) -> HapticResult<()>;

    /// Start vibration
    fn trigger(&mut self) -> HapticResult<()>;

    /// Stop vibration
    fn stop(&mut self) -> HapticResult<()>;

    /// Play a haptic primitive
    fn play_primitive(&mut self, primitive: &HapticPrimitive) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        // Set up the primitive
        self.set_frequency(primitive.frequency)?;
        self.set_amplitude(primitive.intensity)?;

        // Trigger and handle envelope
        // Note: Real implementation would handle ADSR envelope
        self.trigger()?;

        Ok(())
    }

    /// Play a haptic sequence
    fn play_sequence<const N: usize>(
        &mut self,
        sequence: &HapticSequence<N>,
        delay_fn: impl Fn(u32),
    ) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        let loop_count = if sequence.do_loop {
            sequence.loop_count.max(1)
        } else {
            1
        };

        for _ in 0..loop_count {
            for step in &sequence.steps {
                // Delay before step
                if step.delay_before > 0 {
                    delay_fn(step.delay_before as u32);
                }

                // Play with repeats
                for i in 0..step.repeat_count {
                    // Apply intensity scale
                    let scaled_intensity = step.primitive.intensity * step.intensity_scale;
                    let mut scaled_primitive = step.primitive.clone();
                    scaled_primitive.intensity = scaled_intensity.min(1.0);

                    self.play_primitive(&scaled_primitive)?;

                    // Wait for primitive duration
                    delay_fn(step.primitive.duration as u32);

                    // Repeat delay
                    if i < step.repeat_count - 1 && step.repeat_delay > 0 {
                        delay_fn(step.repeat_delay as u32);
                    }
                }
            }
        }

        Ok(())
    }

    /// Trigger a quick pulse
    fn pulse(&mut self, duration_ms: u16, intensity: f32) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        self.set_amplitude(intensity)?;
        self.trigger()?;

        Ok(())
    }
}

/// Multi-location haptic driver trait
///
/// Extend this for devices with multiple actuators.
pub trait MultiLocationDriver: HapticDriver {
    /// Set active location
    fn set_location(&mut self, location: BodyLocation) -> HapticResult<()>;

    /// Set amplitude for a specific location
    fn set_amplitude_at(
        &mut self,
        location: BodyLocation,
        amplitude: f32,
    ) -> HapticResult<()>;

    /// Trigger at a specific location
    fn trigger_at(&mut self, location: BodyLocation) -> HapticResult<()>;

    /// Stop at a specific location
    fn stop_at(&mut self, location: BodyLocation) -> HapticResult<()>;

    /// Stop all locations
    fn stop_all(&mut self) -> HapticResult<()>;
}

/// Configuration for haptic drivers
#[derive(Debug, Clone)]
pub struct DriverConfig {
    /// Global intensity scale (0.0-2.0)
    pub intensity_scale: f32,
    /// Default frequency in Hz
    pub default_frequency: u16,
    /// Enable debug output
    pub debug: bool,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self {
            intensity_scale: 1.0,
            default_frequency: 150,
            debug: false,
        }
    }
}

/// Driver state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverState {
    /// Not initialized
    Uninitialized,
    /// Ready to use
    Ready,
    /// Currently playing
    Playing,
    /// Error state
    Error,
}
