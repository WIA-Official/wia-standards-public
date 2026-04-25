//! WIA Haptic Standard - Waveform Generator
//!
//! Software waveform generation for haptic signals.
//!
//! 홍익인간 (弘益人間) - 널리 인간을 이롭게 하라

use crate::types::Waveform;

#[cfg(feature = "libm")]
use libm::{sinf, fabsf};

/// Waveform generator for producing haptic signal values.
pub struct WaveformGenerator {
    waveform: Waveform,
    frequency: f32,
    sample_rate: u32,
    phase: f32,
    noise_state: u32,
}

impl WaveformGenerator {
    /// Create a new waveform generator.
    ///
    /// # Arguments
    /// * `waveform` - Type of waveform to generate
    /// * `frequency` - Frequency in Hz
    /// * `sample_rate` - Output sample rate in Hz
    pub fn new(waveform: Waveform, frequency: f32, sample_rate: u32) -> Self {
        Self {
            waveform,
            frequency,
            sample_rate,
            phase: 0.0,
            noise_state: 0x12345678,
        }
    }

    /// Set the waveform type.
    pub fn set_waveform(&mut self, waveform: Waveform) {
        self.waveform = waveform;
    }

    /// Set the frequency.
    pub fn set_frequency(&mut self, frequency: f32) {
        self.frequency = frequency;
    }

    /// Reset the phase.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Generate the next sample value (-1.0 to 1.0).
    pub fn next_sample(&mut self) -> f32 {
        let value = self.sample_at_phase(self.phase);

        // Advance phase
        let phase_increment = self.frequency / self.sample_rate as f32;
        self.phase += phase_increment;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        value
    }

    /// Generate a sample at a specific phase (0.0 to 1.0).
    pub fn sample_at_phase(&mut self, phase: f32) -> f32 {
        match self.waveform {
            Waveform::Sine => self.sine_sample(phase),
            Waveform::Square => self.square_sample(phase),
            Waveform::Triangle => self.triangle_sample(phase),
            Waveform::Sawtooth => self.sawtooth_sample(phase),
            Waveform::Noise => self.noise_sample(),
        }
    }

    /// Generate samples into a buffer.
    pub fn generate(&mut self, buffer: &mut [f32]) {
        for sample in buffer.iter_mut() {
            *sample = self.next_sample();
        }
    }

    /// Generate samples with amplitude modulation.
    pub fn generate_with_amplitude(&mut self, buffer: &mut [f32], amplitude: f32) {
        for sample in buffer.iter_mut() {
            *sample = self.next_sample() * amplitude;
        }
    }

    // Waveform implementations

    #[cfg(feature = "libm")]
    fn sine_sample(&self, phase: f32) -> f32 {
        sinf(phase * core::f32::consts::TAU)
    }

    #[cfg(not(feature = "libm"))]
    fn sine_sample(&self, phase: f32) -> f32 {
        // Polynomial approximation of sine for no_std without libm
        // Using Bhaskara I's approximation
        let x = phase * 2.0 - 1.0; // Map 0-1 to -1-1
        let x2 = x * x;
        let numerator = 16.0 * x * (1.0 - x2.abs());
        let denominator = 5.0 - x2;
        if denominator != 0.0 {
            numerator / denominator
        } else {
            0.0
        }
    }

    fn square_sample(&self, phase: f32) -> f32 {
        if phase < 0.5 { 1.0 } else { -1.0 }
    }

    fn triangle_sample(&self, phase: f32) -> f32 {
        if phase < 0.5 {
            4.0 * phase - 1.0
        } else {
            3.0 - 4.0 * phase
        }
    }

    fn sawtooth_sample(&self, phase: f32) -> f32 {
        2.0 * phase - 1.0
    }

    fn noise_sample(&mut self) -> f32 {
        // Simple xorshift PRNG for noise
        self.noise_state ^= self.noise_state << 13;
        self.noise_state ^= self.noise_state >> 17;
        self.noise_state ^= self.noise_state << 5;

        // Convert to -1.0 to 1.0
        (self.noise_state as f32 / u32::MAX as f32) * 2.0 - 1.0
    }
}

/// PWM (Pulse Width Modulation) generator for haptic output.
pub struct PwmGenerator {
    frequency: f32,
    duty_cycle: f32,
    sample_rate: u32,
    phase: f32,
}

impl PwmGenerator {
    /// Create a new PWM generator.
    pub fn new(frequency: f32, duty_cycle: f32, sample_rate: u32) -> Self {
        Self {
            frequency,
            duty_cycle: duty_cycle.clamp(0.0, 1.0),
            sample_rate,
            phase: 0.0,
        }
    }

    /// Set the duty cycle (0.0 to 1.0).
    pub fn set_duty_cycle(&mut self, duty_cycle: f32) {
        self.duty_cycle = duty_cycle.clamp(0.0, 1.0);
    }

    /// Set the frequency.
    pub fn set_frequency(&mut self, frequency: f32) {
        self.frequency = frequency;
    }

    /// Generate the next PWM sample (0 or 1).
    pub fn next_sample(&mut self) -> bool {
        let value = self.phase < self.duty_cycle;

        // Advance phase
        let phase_increment = self.frequency / self.sample_rate as f32;
        self.phase += phase_increment;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }

        value
    }

    /// Calculate duty cycle from intensity using perceptual scaling.
    ///
    /// Uses logarithmic scaling based on Weber-Fechner law.
    pub fn intensity_to_duty_cycle(intensity: f32) -> f32 {
        if intensity <= 0.0 {
            return 0.0;
        }
        if intensity >= 1.0 {
            return 1.0;
        }

        // Logarithmic perceptual scaling
        // perceived = log10(1 + 9 * physical)
        // physical = (10^perceived - 1) / 9
        let perceived = intensity;
        (fast_pow10(perceived) - 1.0) / 9.0
    }
}

/// Fast approximation of 10^x for no_std.
fn fast_pow10(x: f32) -> f32 {
    // Using e^(x * ln(10)) approximation
    // ln(10) ≈ 2.302585
    fast_exp(x * 2.302585)
}

/// Fast approximation of e^x for no_std.
fn fast_exp(x: f32) -> f32 {
    // Schraudolph's approximation
    // Works well for x in range [-10, 10]
    let x = x.clamp(-10.0, 10.0);

    // 2^(x * log2(e)) = 2^(x * 1.4426950408889634)
    let y = x * 1.4426950408889634;

    // Fast 2^y approximation
    let i = (y * 8388608.0 + 1065353216.0) as i32;
    f32::from_bits(i as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_wave() {
        let mut gen = WaveformGenerator::new(Waveform::Square, 100.0, 1000);

        // First half should be positive
        gen.phase = 0.25;
        assert!(gen.sample_at_phase(0.25) > 0.0);

        // Second half should be negative
        assert!(gen.sample_at_phase(0.75) < 0.0);
    }

    #[test]
    fn test_triangle_wave() {
        let mut gen = WaveformGenerator::new(Waveform::Triangle, 100.0, 1000);

        // Start at -1
        assert!((gen.sample_at_phase(0.0) - (-1.0)).abs() < 0.01);

        // Peak at 0.5
        assert!((gen.sample_at_phase(0.5) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_pwm_duty_cycle() {
        let mut pwm = PwmGenerator::new(100.0, 0.5, 1000);

        let mut high_count = 0;
        let mut low_count = 0;

        for _ in 0..1000 {
            if pwm.next_sample() {
                high_count += 1;
            } else {
                low_count += 1;
            }
        }

        // Should be roughly 50/50
        let ratio = high_count as f32 / (high_count + low_count) as f32;
        assert!((ratio - 0.5).abs() < 0.1);
    }
}
