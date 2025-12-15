//! Software/simulation haptic driver
//!
//! This driver provides a software simulation of haptic feedback,
//! useful for testing and development without physical hardware.

use crate::driver::{DriverConfig, DriverState, HapticCapabilities, HapticDriver};
use crate::types::{ActuatorType, BodyLocation, HapticError, HapticResult, Envelope};
use crate::patterns::HapticPrimitive;

/// Software haptic driver for testing and simulation
///
/// This driver simulates haptic feedback without requiring physical hardware.
/// It can be used for:
/// - Unit testing haptic patterns
/// - Development without hardware
/// - Visualizing haptic feedback
///
/// # Example
///
/// ```rust
/// use wia_haptic::{SoftwareHapticDriver, HapticDriver, HapticPrimitive};
///
/// let mut driver = SoftwareHapticDriver::new();
/// driver.init().unwrap();
///
/// // Play a primitive
/// driver.play_primitive(&HapticPrimitive::tick()).unwrap();
///
/// // Check the log
/// for entry in driver.log() {
///     println!("{:?}", entry);
/// }
/// ```
pub struct SoftwareHapticDriver {
    /// Configuration
    config: DriverConfig,
    /// Current state
    state: DriverState,
    /// Current frequency
    current_frequency: u16,
    /// Current amplitude
    current_amplitude: f32,
    /// Capabilities
    capabilities: HapticCapabilities,
    /// Is currently active
    is_active: bool,
    /// Event log for testing
    #[cfg(feature = "std")]
    log: Vec<HapticLogEntry>,
    /// Callback for haptic events
    callback: Option<fn(HapticEvent)>,
}

/// Log entry for software driver
#[cfg(feature = "std")]
#[derive(Debug, Clone)]
pub struct HapticLogEntry {
    /// Event type
    pub event: HapticEvent,
    /// Timestamp (relative to driver start)
    pub timestamp_ms: u64,
}

/// Haptic event types
#[derive(Debug, Clone, Copy)]
pub enum HapticEvent {
    /// Driver initialized
    Initialized,
    /// Driver deinitialized
    Deinitialized,
    /// Frequency changed
    FrequencySet(u16),
    /// Amplitude changed
    AmplitudeSet(f32),
    /// Vibration started
    Triggered,
    /// Vibration stopped
    Stopped,
    /// Primitive started
    PrimitiveStart {
        frequency: u16,
        intensity: f32,
        duration: u16,
    },
    /// Primitive ended
    PrimitiveEnd,
}

impl SoftwareHapticDriver {
    /// Create a new software haptic driver
    pub fn new() -> Self {
        Self {
            config: DriverConfig::default(),
            state: DriverState::Uninitialized,
            current_frequency: 150,
            current_amplitude: 0.0,
            capabilities: HapticCapabilities {
                actuator_type: ActuatorType::Lra,
                min_frequency: 1,
                max_frequency: 300,
                locations: &[
                    BodyLocation::WristLeftDorsal,
                    BodyLocation::WristRightDorsal,
                ],
                max_intensity: 1.0,
                latency: 0,
                actuator_count: 2,
                supports_custom_waveforms: true,
                supports_amplitude_modulation: true,
                supports_frequency_modulation: true,
            },
            is_active: false,
            #[cfg(feature = "std")]
            log: Vec::new(),
            callback: None,
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: DriverConfig) -> Self {
        let mut driver = Self::new();
        driver.config = config;
        driver
    }

    /// Set event callback
    pub fn set_callback(&mut self, callback: fn(HapticEvent)) {
        self.callback = callback;
    }

    /// Get the event log
    #[cfg(feature = "std")]
    pub fn log(&self) -> &[HapticLogEntry] {
        &self.log
    }

    /// Clear the event log
    #[cfg(feature = "std")]
    pub fn clear_log(&mut self) {
        self.log.clear();
    }

    /// Record an event
    fn record_event(&mut self, event: HapticEvent) {
        #[cfg(feature = "std")]
        {
            // In real implementation, would use actual timestamp
            let timestamp_ms = self.log.len() as u64 * 10;
            self.log.push(HapticLogEntry { event, timestamp_ms });
        }

        if let Some(callback) = self.callback {
            callback(event);
        }
    }

    /// Get current state
    pub fn current_state(&self) -> (u16, f32, bool) {
        (self.current_frequency, self.current_amplitude, self.is_active)
    }
}

impl Default for SoftwareHapticDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl HapticDriver for SoftwareHapticDriver {
    fn init(&mut self) -> HapticResult<()> {
        if self.state != DriverState::Uninitialized {
            return Ok(());
        }

        self.state = DriverState::Ready;
        self.record_event(HapticEvent::Initialized);

        if self.config.debug {
            #[cfg(feature = "std")]
            println!("[SoftwareDriver] Initialized");
        }

        Ok(())
    }

    fn deinit(&mut self) -> HapticResult<()> {
        if self.state == DriverState::Uninitialized {
            return Ok(());
        }

        self.stop()?;
        self.state = DriverState::Uninitialized;
        self.record_event(HapticEvent::Deinitialized);

        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.state != DriverState::Uninitialized
    }

    fn capabilities(&self) -> &HapticCapabilities {
        &self.capabilities
    }

    fn set_frequency(&mut self, hz: u16) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        if hz < self.capabilities.min_frequency || hz > self.capabilities.max_frequency {
            return Err(HapticError::InvalidParameter);
        }

        self.current_frequency = hz;
        self.record_event(HapticEvent::FrequencySet(hz));

        if self.config.debug {
            #[cfg(feature = "std")]
            println!("[SoftwareDriver] Frequency set to {}Hz", hz);
        }

        Ok(())
    }

    fn set_amplitude(&mut self, amplitude: f32) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        if amplitude < 0.0 || amplitude > 1.0 {
            return Err(HapticError::InvalidParameter);
        }

        let scaled = amplitude * self.config.intensity_scale;
        self.current_amplitude = scaled.min(1.0);
        self.record_event(HapticEvent::AmplitudeSet(self.current_amplitude));

        if self.config.debug {
            #[cfg(feature = "std")]
            println!("[SoftwareDriver] Amplitude set to {:.2}", self.current_amplitude);
        }

        Ok(())
    }

    fn trigger(&mut self) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        self.is_active = true;
        self.state = DriverState::Playing;
        self.record_event(HapticEvent::Triggered);

        if self.config.debug {
            #[cfg(feature = "std")]
            println!(
                "[SoftwareDriver] Triggered: {}Hz @ {:.0}%",
                self.current_frequency,
                self.current_amplitude * 100.0
            );
        }

        Ok(())
    }

    fn stop(&mut self) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        self.is_active = false;
        self.state = DriverState::Ready;
        self.record_event(HapticEvent::Stopped);

        if self.config.debug {
            #[cfg(feature = "std")]
            println!("[SoftwareDriver] Stopped");
        }

        Ok(())
    }

    fn play_primitive(&mut self, primitive: &HapticPrimitive) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        self.record_event(HapticEvent::PrimitiveStart {
            frequency: primitive.frequency,
            intensity: primitive.intensity,
            duration: primitive.duration,
        });

        // Set up and trigger
        self.set_frequency(primitive.frequency)?;
        self.set_amplitude(primitive.intensity)?;
        self.trigger()?;

        if self.config.debug {
            #[cfg(feature = "std")]
            println!(
                "[SoftwareDriver] Playing '{}': {}Hz, {:.0}%, {}ms",
                primitive.name,
                primitive.frequency,
                primitive.intensity * 100.0,
                primitive.duration
            );
        }

        self.record_event(HapticEvent::PrimitiveEnd);

        Ok(())
    }
}

/// Generate envelope samples for visualization
///
/// Returns amplitude samples at regular intervals
#[cfg(feature = "std")]
pub fn generate_envelope_samples(
    envelope: &Envelope,
    duration_ms: u16,
    sample_count: usize,
) -> Vec<f32> {
    let mut samples = Vec::with_capacity(sample_count);
    let total_duration = duration_ms as f32;

    for i in 0..sample_count {
        let t = (i as f32 / sample_count as f32) * total_duration;
        let amplitude = calculate_envelope_amplitude(envelope, t, duration_ms);
        samples.push(amplitude);
    }

    samples
}

/// Calculate envelope amplitude at a given time
fn calculate_envelope_amplitude(envelope: &Envelope, time_ms: f32, total_duration: u16) -> f32 {
    let attack = envelope.attack as f32;
    let decay = envelope.decay as f32;
    let sustain = envelope.sustain;
    let release = envelope.release as f32;

    let sustain_end = total_duration as f32 - release;

    if time_ms < attack {
        // Attack phase: ramp up from 0 to 1
        time_ms / attack
    } else if time_ms < attack + decay {
        // Decay phase: ramp down from 1 to sustain
        let decay_progress = (time_ms - attack) / decay;
        1.0 - (1.0 - sustain) * decay_progress
    } else if time_ms < sustain_end {
        // Sustain phase: constant
        sustain
    } else {
        // Release phase: ramp down from sustain to 0
        let release_progress = (time_ms - sustain_end) / release;
        sustain * (1.0 - release_progress).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_software_driver() {
        let mut driver = SoftwareHapticDriver::new();

        // Test initialization
        assert!(!driver.is_initialized());
        driver.init().unwrap();
        assert!(driver.is_initialized());

        // Test frequency
        driver.set_frequency(200).unwrap();
        assert_eq!(driver.current_state().0, 200);

        // Test amplitude
        driver.set_amplitude(0.75).unwrap();
        assert!((driver.current_state().1 - 0.75).abs() < 0.001);

        // Test trigger/stop
        driver.trigger().unwrap();
        assert!(driver.current_state().2);
        driver.stop().unwrap();
        assert!(!driver.current_state().2);
    }

    #[test]
    fn test_primitive_playback() {
        let mut driver = SoftwareHapticDriver::new();
        driver.init().unwrap();

        let tick = HapticPrimitive::tick();
        driver.play_primitive(&tick).unwrap();

        // Check log
        let log = driver.log();
        assert!(log.len() >= 4); // Init, PrimitiveStart, ..., PrimitiveEnd
    }

    #[test]
    fn test_envelope_calculation() {
        let envelope = Envelope {
            attack: 100,
            decay: 50,
            sustain: 0.7,
            release: 100,
        };

        // At start (attack)
        let amp = calculate_envelope_amplitude(&envelope, 50.0, 500);
        assert!((amp - 0.5).abs() < 0.1);

        // In sustain
        let amp = calculate_envelope_amplitude(&envelope, 250.0, 500);
        assert!((amp - 0.7).abs() < 0.1);
    }
}
