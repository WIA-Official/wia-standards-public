//! Mock Haptic Driver for Testing
//!
//! 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

use crate::{HapticDriver, HapticError, Waveform, Envelope, Capabilities, ActuatorType};

/// Mock haptic driver for testing without hardware.
pub struct MockHapticDriver {
    initialized: bool,
    frequency: f32,
    amplitude: f32,
    running: bool,
    waveform: Waveform,
    envelope: Option<Envelope>,
    trigger_count: u32,
    stop_count: u32,
}

impl MockHapticDriver {
    /// Create a new mock driver.
    pub fn new() -> Self {
        Self {
            initialized: false,
            frequency: 80.0,
            amplitude: 0.0,
            running: false,
            waveform: Waveform::Sine,
            envelope: None,
            trigger_count: 0,
            stop_count: 0,
        }
    }

    /// Get the number of times trigger() was called.
    pub fn trigger_count(&self) -> u32 {
        self.trigger_count
    }

    /// Get the number of times stop() was called.
    pub fn stop_count(&self) -> u32 {
        self.stop_count
    }

    /// Check if currently running.
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Get the current waveform.
    pub fn current_waveform(&self) -> Waveform {
        self.waveform
    }

    /// Get the current envelope.
    pub fn current_envelope(&self) -> Option<&Envelope> {
        self.envelope.as_ref()
    }

    /// Reset counters and state.
    pub fn reset(&mut self) {
        self.initialized = false;
        self.frequency = 80.0;
        self.amplitude = 0.0;
        self.running = false;
        self.waveform = Waveform::Sine;
        self.envelope = None;
        self.trigger_count = 0;
        self.stop_count = 0;
    }

    /// Get device capabilities.
    pub fn capabilities(&self) -> Capabilities {
        Capabilities {
            actuator_type: ActuatorType::LRA,
            freq_min: 1.0,
            freq_max: 300.0,
            channels: 1,
            latency_ms: 1,
            supports_adsr: true,
            waveform_mask: 0b11111, // All waveforms
        }
    }
}

impl Default for MockHapticDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl HapticDriver for MockHapticDriver {
    fn init(&mut self) -> Result<(), HapticError> {
        self.initialized = true;
        Ok(())
    }

    fn set_frequency(&mut self, hz: f32) -> Result<(), HapticError> {
        if hz < 1.0 || hz > 300.0 {
            return Err(HapticError::FrequencyOutOfRange);
        }
        self.frequency = hz;
        Ok(())
    }

    fn set_amplitude(&mut self, amplitude: f32) -> Result<(), HapticError> {
        self.amplitude = amplitude.clamp(0.0, 1.0);
        Ok(())
    }

    fn trigger(&mut self) -> Result<(), HapticError> {
        if !self.initialized {
            return Err(HapticError::NotInitialized);
        }
        self.running = true;
        self.trigger_count += 1;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), HapticError> {
        self.running = false;
        self.stop_count += 1;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn get_frequency(&self) -> f32 {
        self.frequency
    }

    fn get_amplitude(&self) -> f32 {
        self.amplitude
    }

    fn set_waveform(&mut self, waveform: Waveform) -> Result<(), HapticError> {
        self.waveform = waveform;
        Ok(())
    }

    fn supports_waveform(&self, _waveform: Waveform) -> bool {
        true // Mock supports all
    }

    fn set_envelope(&mut self, envelope: &Envelope) -> Result<(), HapticError> {
        self.envelope = Some(*envelope);
        Ok(())
    }

    fn supports_envelope(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_driver_lifecycle() {
        let mut driver = MockHapticDriver::new();

        // Not initialized yet
        assert!(!driver.is_initialized());
        assert!(driver.trigger().is_err());

        // Initialize
        driver.init().unwrap();
        assert!(driver.is_initialized());

        // Set parameters
        driver.set_frequency(100.0).unwrap();
        driver.set_amplitude(0.7).unwrap();

        assert_eq!(driver.get_frequency(), 100.0);
        assert_eq!(driver.get_amplitude(), 0.7);

        // Trigger and stop
        driver.trigger().unwrap();
        assert!(driver.is_running());
        assert_eq!(driver.trigger_count(), 1);

        driver.stop().unwrap();
        assert!(!driver.is_running());
        assert_eq!(driver.stop_count(), 1);
    }

    #[test]
    fn test_amplitude_clamping() {
        let mut driver = MockHapticDriver::new();
        driver.init().unwrap();

        driver.set_amplitude(1.5).unwrap();
        assert_eq!(driver.get_amplitude(), 1.0);

        driver.set_amplitude(-0.5).unwrap();
        assert_eq!(driver.get_amplitude(), 0.0);
    }

    #[test]
    fn test_frequency_range() {
        let mut driver = MockHapticDriver::new();
        driver.init().unwrap();

        assert!(driver.set_frequency(100.0).is_ok());
        assert!(driver.set_frequency(0.5).is_err());
        assert!(driver.set_frequency(350.0).is_err());
    }

    #[test]
    fn test_waveform_and_envelope() {
        let mut driver = MockHapticDriver::new();
        driver.init().unwrap();

        driver.set_waveform(Waveform::Triangle).unwrap();
        assert_eq!(driver.current_waveform(), Waveform::Triangle);

        let envelope = Envelope::TAP;
        driver.set_envelope(&envelope).unwrap();
        assert!(driver.current_envelope().is_some());
    }
}
