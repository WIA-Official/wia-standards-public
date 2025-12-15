//! PWM-based haptic driver
//!
//! This driver uses Pulse Width Modulation to control haptic actuators.
//! Suitable for ERM and LRA actuators connected via GPIO pins.

use crate::driver::{DriverConfig, DriverState, HapticCapabilities, HapticDriver};
use crate::types::{ActuatorType, BodyLocation, HapticError, HapticResult};

/// PWM haptic driver configuration
#[derive(Debug, Clone)]
pub struct PwmConfig {
    /// Base driver configuration
    pub base: DriverConfig,
    /// PWM frequency in Hz (carrier frequency)
    pub pwm_frequency: u32,
    /// Minimum duty cycle (0.0-1.0)
    pub min_duty_cycle: f32,
    /// Maximum duty cycle (0.0-1.0)
    pub max_duty_cycle: f32,
    /// Actuator type
    pub actuator_type: ActuatorType,
    /// Body location of this actuator
    pub location: BodyLocation,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            base: DriverConfig::default(),
            pwm_frequency: 20000, // 20kHz carrier
            min_duty_cycle: 0.0,
            max_duty_cycle: 1.0,
            actuator_type: ActuatorType::Erm,
            location: BodyLocation::WristLeftDorsal,
        }
    }
}

/// PWM-based haptic driver
///
/// Generic driver that can work with any PWM-capable GPIO pin.
///
/// # Example
///
/// ```rust,ignore
/// use wia_haptic::{PwmHapticDriver, PwmConfig, HapticDriver};
///
/// // Create driver with pin 18 and config
/// let config = PwmConfig::default();
/// let mut driver = PwmHapticDriver::new(18, config);
///
/// // Initialize
/// driver.init()?;
///
/// // Set frequency and amplitude
/// driver.set_frequency(150)?;
/// driver.set_amplitude(0.5)?;
///
/// // Trigger vibration
/// driver.trigger()?;
///
/// // Stop after some time
/// driver.stop()?;
/// ```
pub struct PwmHapticDriver {
    /// GPIO pin number
    pin: u8,
    /// Configuration
    config: PwmConfig,
    /// Current state
    state: DriverState,
    /// Current frequency setting
    current_frequency: u16,
    /// Current amplitude setting
    current_amplitude: f32,
    /// Cached capabilities
    capabilities: HapticCapabilities,
    /// Is currently vibrating
    is_active: bool,
}

impl PwmHapticDriver {
    /// Create a new PWM haptic driver
    ///
    /// # Arguments
    ///
    /// * `pin` - GPIO pin number for PWM output
    /// * `config` - Driver configuration
    pub fn new(pin: u8, config: PwmConfig) -> Self {
        let capabilities = HapticCapabilities {
            actuator_type: config.actuator_type,
            min_frequency: 30,
            max_frequency: match config.actuator_type {
                ActuatorType::Erm => 200,
                ActuatorType::Lra => 300,
                ActuatorType::Piezo => 300,
                ActuatorType::VoiceCoil => 250,
            },
            locations: &[config.location],
            max_intensity: 1.0,
            latency: 10,
            actuator_count: 1,
            supports_custom_waveforms: false,
            supports_amplitude_modulation: true,
            supports_frequency_modulation: config.actuator_type != ActuatorType::Lra,
        };

        Self {
            pin,
            config,
            state: DriverState::Uninitialized,
            current_frequency: 150,
            current_amplitude: 0.0,
            capabilities,
            is_active: false,
        }
    }

    /// Get the GPIO pin number
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// Get current driver state
    pub fn state(&self) -> DriverState {
        self.state
    }

    /// Calculate duty cycle from amplitude
    fn calculate_duty_cycle(&self, amplitude: f32) -> f32 {
        let scaled = amplitude * self.config.base.intensity_scale;
        let clamped = scaled.clamp(0.0, 1.0);

        // Map to configured duty cycle range
        let range = self.config.max_duty_cycle - self.config.min_duty_cycle;
        self.config.min_duty_cycle + (clamped * range)
    }

    /// Apply PWM settings to hardware
    ///
    /// This is a placeholder - real implementation would interface with
    /// hardware-specific PWM peripherals.
    fn apply_pwm(&mut self, duty_cycle: f32) -> HapticResult<()> {
        // In a real implementation, this would:
        // 1. Configure PWM peripheral
        // 2. Set duty cycle
        // 3. Enable/disable output

        if self.config.base.debug {
            #[cfg(feature = "std")]
            println!(
                "[PWM] Pin {}: duty={:.2}%, freq={}Hz",
                self.pin,
                duty_cycle * 100.0,
                self.current_frequency
            );
        }

        Ok(())
    }
}

impl HapticDriver for PwmHapticDriver {
    fn init(&mut self) -> HapticResult<()> {
        if self.state != DriverState::Uninitialized {
            return Ok(()); // Already initialized
        }

        // In real implementation: Initialize PWM peripheral
        // - Configure GPIO pin for PWM alternate function
        // - Set up PWM timer with configured frequency
        // - Set initial duty cycle to 0

        self.state = DriverState::Ready;
        self.current_amplitude = 0.0;
        self.is_active = false;

        if self.config.base.debug {
            #[cfg(feature = "std")]
            println!(
                "[PWM] Initialized on pin {} @ {}Hz",
                self.pin, self.config.pwm_frequency
            );
        }

        Ok(())
    }

    fn deinit(&mut self) -> HapticResult<()> {
        if self.state == DriverState::Uninitialized {
            return Ok(());
        }

        // Stop any active vibration
        self.stop()?;

        // In real implementation: Deinitialize PWM peripheral
        // - Disable PWM output
        // - Release GPIO pin

        self.state = DriverState::Uninitialized;

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

        // Validate frequency range
        if hz < self.capabilities.min_frequency || hz > self.capabilities.max_frequency {
            return Err(HapticError::InvalidParameter);
        }

        self.current_frequency = hz;

        // If currently active, update the hardware
        if self.is_active {
            // In real implementation: Update PWM frequency/pattern
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

        self.current_amplitude = amplitude;

        // If currently active, update the duty cycle
        if self.is_active {
            let duty = self.calculate_duty_cycle(amplitude);
            self.apply_pwm(duty)?;
        }

        Ok(())
    }

    fn trigger(&mut self) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        let duty = self.calculate_duty_cycle(self.current_amplitude);
        self.apply_pwm(duty)?;
        self.is_active = true;
        self.state = DriverState::Playing;

        Ok(())
    }

    fn stop(&mut self) -> HapticResult<()> {
        if !self.is_initialized() {
            return Err(HapticError::NotInitialized);
        }

        self.apply_pwm(0.0)?;
        self.is_active = false;
        self.state = DriverState::Ready;

        Ok(())
    }
}

/// Multi-channel PWM driver for devices with multiple actuators
pub struct MultiPwmDriver {
    /// Individual PWM drivers for each channel
    channels: [Option<PwmHapticDriver>; 16],
    /// Number of active channels
    channel_count: usize,
    /// Combined capabilities
    capabilities: HapticCapabilities,
    /// Initialization state
    initialized: bool,
}

impl MultiPwmDriver {
    /// Create a new multi-channel PWM driver
    pub fn new() -> Self {
        Self {
            channels: Default::default(),
            channel_count: 0,
            capabilities: HapticCapabilities::default(),
            initialized: false,
        }
    }

    /// Add a channel
    pub fn add_channel(&mut self, index: usize, driver: PwmHapticDriver) -> HapticResult<()> {
        if index >= 16 {
            return Err(HapticError::InvalidParameter);
        }

        self.channels[index] = Some(driver);
        self.channel_count += 1;
        self.update_capabilities();

        Ok(())
    }

    /// Get a channel
    pub fn channel(&mut self, index: usize) -> Option<&mut PwmHapticDriver> {
        self.channels.get_mut(index)?.as_mut()
    }

    fn update_capabilities(&mut self) {
        self.capabilities.actuator_count = self.channel_count as u8;
    }

    /// Initialize all channels
    pub fn init_all(&mut self) -> HapticResult<()> {
        for channel in self.channels.iter_mut().flatten() {
            channel.init()?;
        }
        self.initialized = true;
        Ok(())
    }

    /// Stop all channels
    pub fn stop_all(&mut self) -> HapticResult<()> {
        for channel in self.channels.iter_mut().flatten() {
            channel.stop()?;
        }
        Ok(())
    }

    /// Trigger all channels
    pub fn trigger_all(&mut self) -> HapticResult<()> {
        for channel in self.channels.iter_mut().flatten() {
            channel.trigger()?;
        }
        Ok(())
    }
}

impl Default for MultiPwmDriver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_driver_init() {
        let mut driver = PwmHapticDriver::new(18, PwmConfig::default());
        assert!(!driver.is_initialized());

        driver.init().unwrap();
        assert!(driver.is_initialized());
        assert_eq!(driver.state(), DriverState::Ready);
    }

    #[test]
    fn test_pwm_driver_amplitude() {
        let mut driver = PwmHapticDriver::new(18, PwmConfig::default());
        driver.init().unwrap();

        driver.set_amplitude(0.5).unwrap();
        assert!(driver.set_amplitude(1.5).is_err()); // Out of range
    }

    #[test]
    fn test_duty_cycle_calculation() {
        let config = PwmConfig {
            min_duty_cycle: 0.1,
            max_duty_cycle: 0.9,
            ..Default::default()
        };
        let driver = PwmHapticDriver::new(18, config);

        assert!((driver.calculate_duty_cycle(0.0) - 0.1).abs() < 0.001);
        assert!((driver.calculate_duty_cycle(1.0) - 0.9).abs() < 0.001);
        assert!((driver.calculate_duty_cycle(0.5) - 0.5).abs() < 0.001);
    }
}
