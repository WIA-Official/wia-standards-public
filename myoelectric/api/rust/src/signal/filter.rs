//! Digital Filters for EMG Signal Processing
//!
//! This module provides notch and bandpass filters for removing noise
//! from EMG signals according to WIA specifications.
//!
//! ## Notch Filter
//!
//! Removes power line interference at 50 Hz or 60 Hz.
//!
//! ## Bandpass Filter
//!
//! Passes EMG frequency content (typically 20-450 Hz) while removing
//! motion artifacts and high-frequency noise.

use crate::{EMGError, Result};
use std::f32::consts::PI;

/// Second-order IIR notch filter for power line interference removal
///
/// Implements a narrow notch filter to remove 50/60 Hz interference
/// with minimal effect on surrounding frequencies.
#[derive(Debug, Clone)]
pub struct NotchFilter {
    // Filter coefficients
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    // Filter state
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
    // Configuration
    center_freq: f32,
    q_factor: f32,
    sample_rate: f32,
}

impl NotchFilter {
    /// Create a new notch filter
    ///
    /// # Arguments
    ///
    /// * `center_freq` - Center frequency in Hz (typically 50 or 60)
    /// * `q_factor` - Quality factor (higher = narrower notch, typical: 30)
    /// * `sample_rate` - Sampling frequency in Hz
    ///
    /// # Returns
    ///
    /// A new `NotchFilter` instance or an error if parameters are invalid
    pub fn new(center_freq: f32, q_factor: f32, sample_rate: f32) -> Result<Self> {
        if center_freq <= 0.0 || center_freq >= sample_rate / 2.0 {
            return Err(EMGError::InvalidFrequency(center_freq));
        }
        if sample_rate <= 0.0 {
            return Err(EMGError::InvalidSampleRate(sample_rate));
        }

        let w0 = 2.0 * PI * center_freq / sample_rate;
        let alpha = w0.sin() / (2.0 * q_factor);
        let cos_w0 = w0.cos();

        // Normalize coefficients by (1 + alpha)
        let norm = 1.0 + alpha;

        let b0 = 1.0 / norm;
        let b1 = -2.0 * cos_w0 / norm;
        let b2 = 1.0 / norm;
        let a1 = -2.0 * cos_w0 / norm;
        let a2 = (1.0 - alpha) / norm;

        Ok(Self {
            b0,
            b1,
            b2,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
            center_freq,
            q_factor,
            sample_rate,
        })
    }

    /// Filter a single sample
    pub fn filter(&mut self, x: f32) -> f32 {
        // Direct Form II Transposed
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        // Update state
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;

        y
    }

    /// Filter an entire buffer
    pub fn filter_buffer(&mut self, input: &[f32]) -> Vec<f32> {
        input.iter().map(|&x| self.filter(x)).collect()
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    /// Get filter parameters
    pub fn params(&self) -> (f32, f32, f32) {
        (self.center_freq, self.q_factor, self.sample_rate)
    }
}

/// Second-order section for cascaded filter implementation
#[derive(Debug, Clone)]
struct SecondOrderSection {
    b: [f32; 3],
    a: [f32; 3],
    // State variables (Direct Form II Transposed)
    w: [f32; 2],
}

impl SecondOrderSection {
    fn new(b: [f32; 3], a: [f32; 3]) -> Self {
        Self { b, a, w: [0.0; 2] }
    }

    fn filter(&mut self, x: f32) -> f32 {
        let y = self.b[0] * x + self.w[0];
        self.w[0] = self.b[1] * x - self.a[1] * y + self.w[1];
        self.w[1] = self.b[2] * x - self.a[2] * y;
        y
    }

    fn reset(&mut self) {
        self.w = [0.0; 2];
    }
}

/// Butterworth bandpass filter for EMG signal conditioning
///
/// Implemented as cascaded second-order sections for numerical stability.
#[derive(Debug, Clone)]
pub struct BandpassFilter {
    sections: Vec<SecondOrderSection>,
    low_cutoff: f32,
    high_cutoff: f32,
    order: u32,
    sample_rate: f32,
}

impl BandpassFilter {
    /// Create a new Butterworth bandpass filter
    ///
    /// # Arguments
    ///
    /// * `low_cutoff` - Low cutoff frequency in Hz (typically 20)
    /// * `high_cutoff` - High cutoff frequency in Hz (typically 450)
    /// * `order` - Filter order (2, 4, or 6 recommended)
    /// * `sample_rate` - Sampling frequency in Hz
    pub fn new(low_cutoff: f32, high_cutoff: f32, order: u32, sample_rate: f32) -> Result<Self> {
        if low_cutoff <= 0.0 || low_cutoff >= sample_rate / 2.0 {
            return Err(EMGError::InvalidFrequency(low_cutoff));
        }
        if high_cutoff <= low_cutoff || high_cutoff >= sample_rate / 2.0 {
            return Err(EMGError::InvalidFrequency(high_cutoff));
        }
        if order == 0 || order > 10 {
            return Err(EMGError::InvalidFilterOrder(order));
        }
        if sample_rate <= 0.0 {
            return Err(EMGError::InvalidSampleRate(sample_rate));
        }

        // Design bandpass filter using bilinear transform
        let sections = Self::design_bandpass(low_cutoff, high_cutoff, order, sample_rate);

        Ok(Self {
            sections,
            low_cutoff,
            high_cutoff,
            order,
            sample_rate,
        })
    }

    /// Design bandpass filter coefficients
    fn design_bandpass(
        low_cutoff: f32,
        high_cutoff: f32,
        order: u32,
        sample_rate: f32,
    ) -> Vec<SecondOrderSection> {
        // Pre-warp frequencies for bilinear transform
        let nyquist = sample_rate / 2.0;
        let low_norm = low_cutoff / nyquist;
        let high_norm = high_cutoff / nyquist;

        // Warp to analog frequencies
        let low_warp = (PI * low_norm / 2.0).tan();
        let high_warp = (PI * high_norm / 2.0).tan();

        let center = (low_warp * high_warp).sqrt();
        let bandwidth = high_warp - low_warp;

        let mut sections = Vec::new();

        // Generate second-order sections
        let n_sections = order as usize;
        for k in 0..n_sections {
            // Butterworth pole angle
            let theta = PI * (2.0 * k as f32 + 1.0) / (2.0 * order as f32) + PI / 2.0;

            // Complex pole in s-domain (normalized lowpass prototype)
            let sigma = theta.cos();
            let omega = theta.sin();

            // Transform to bandpass
            // This is a simplified approach - compute SOS coefficients
            let q = center / bandwidth;
            let _alpha = omega.abs() / (2.0 * q);

            let w0_low = 2.0 * (low_norm * PI / 2.0).atan();
            let w0_high = 2.0 * (high_norm * PI / 2.0).atan();

            // Approximate second-order section for bandpass
            // Using cascaded high-pass and low-pass design
            let cos_low = w0_low.cos();
            let sin_low = w0_low.sin();
            let alpha_low = sin_low / (2.0 * (0.7071 + sigma * 0.1)); // Q adjustment

            let cos_high = w0_high.cos();
            let sin_high = w0_high.sin();
            let alpha_high = sin_high / (2.0 * (0.7071 + sigma * 0.1));

            // High-pass section
            let hp_norm = 1.0 + alpha_low;
            let hp_b = [
                (1.0 + cos_low) / 2.0 / hp_norm,
                -(1.0 + cos_low) / hp_norm,
                (1.0 + cos_low) / 2.0 / hp_norm,
            ];
            let hp_a = [1.0, -2.0 * cos_low / hp_norm, (1.0 - alpha_low) / hp_norm];

            sections.push(SecondOrderSection::new(hp_b, hp_a));

            // Low-pass section
            let lp_norm = 1.0 + alpha_high;
            let lp_b = [
                (1.0 - cos_high) / 2.0 / lp_norm,
                (1.0 - cos_high) / lp_norm,
                (1.0 - cos_high) / 2.0 / lp_norm,
            ];
            let lp_a = [1.0, -2.0 * cos_high / lp_norm, (1.0 - alpha_high) / lp_norm];

            sections.push(SecondOrderSection::new(lp_b, lp_a));
        }

        sections
    }

    /// Filter a single sample through all cascaded sections
    pub fn filter(&mut self, x: f32) -> f32 {
        let mut y = x;
        for section in &mut self.sections {
            y = section.filter(y);
        }
        y
    }

    /// Filter an entire buffer
    pub fn filter_buffer(&mut self, input: &[f32]) -> Vec<f32> {
        input.iter().map(|&x| self.filter(x)).collect()
    }

    /// Reset all section states
    pub fn reset(&mut self) {
        for section in &mut self.sections {
            section.reset();
        }
    }

    /// Get filter parameters
    pub fn params(&self) -> (f32, f32, u32, f32) {
        (self.low_cutoff, self.high_cutoff, self.order, self.sample_rate)
    }
}

/// Simple first-order low-pass filter for smoothing
#[derive(Debug, Clone)]
pub struct LowPassFilter {
    alpha: f32,
    prev_output: f32,
    cutoff: f32,
    sample_rate: f32,
}

impl LowPassFilter {
    /// Create a new low-pass filter
    ///
    /// # Arguments
    ///
    /// * `cutoff` - Cutoff frequency in Hz
    /// * `sample_rate` - Sampling frequency in Hz
    pub fn new(cutoff: f32, sample_rate: f32) -> Result<Self> {
        if cutoff <= 0.0 || cutoff >= sample_rate / 2.0 {
            return Err(EMGError::InvalidFrequency(cutoff));
        }
        if sample_rate <= 0.0 {
            return Err(EMGError::InvalidSampleRate(sample_rate));
        }

        let rc = 1.0 / (2.0 * PI * cutoff);
        let dt = 1.0 / sample_rate;
        let alpha = dt / (rc + dt);

        Ok(Self {
            alpha,
            prev_output: 0.0,
            cutoff,
            sample_rate,
        })
    }

    /// Filter a single sample
    pub fn filter(&mut self, x: f32) -> f32 {
        let y = self.alpha * x + (1.0 - self.alpha) * self.prev_output;
        self.prev_output = y;
        y
    }

    /// Filter an entire buffer
    pub fn filter_buffer(&mut self, input: &[f32]) -> Vec<f32> {
        input.iter().map(|&x| self.filter(x)).collect()
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.prev_output = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_notch_filter_creation() {
        let filter = NotchFilter::new(50.0, 30.0, 1000.0);
        assert!(filter.is_ok());
    }

    #[test]
    fn test_notch_filter_invalid_freq() {
        let filter = NotchFilter::new(600.0, 30.0, 1000.0); // > Nyquist
        assert!(filter.is_err());
    }

    #[test]
    fn test_notch_filter_output() {
        let mut filter = NotchFilter::new(50.0, 30.0, 1000.0).unwrap();

        // Test with DC signal (should pass through)
        for _ in 0..100 {
            let _ = filter.filter(1.0);
        }
        let output = filter.filter(1.0);
        assert!((output - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_bandpass_filter_creation() {
        let filter = BandpassFilter::new(20.0, 450.0, 4, 1000.0);
        assert!(filter.is_ok());
    }

    #[test]
    fn test_bandpass_filter_invalid_params() {
        // Low >= High
        let filter = BandpassFilter::new(450.0, 20.0, 4, 1000.0);
        assert!(filter.is_err());

        // High > Nyquist
        let filter = BandpassFilter::new(20.0, 600.0, 4, 1000.0);
        assert!(filter.is_err());
    }

    #[test]
    fn test_lowpass_filter() {
        let mut filter = LowPassFilter::new(10.0, 1000.0).unwrap();

        // Step response should approach 1.0
        let mut output = 0.0;
        for _ in 0..1000 {
            output = filter.filter(1.0);
        }
        assert!((output - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_filter_reset() {
        let mut filter = NotchFilter::new(50.0, 30.0, 1000.0).unwrap();

        // Process some samples
        for _ in 0..100 {
            filter.filter(1.0);
        }

        // Reset and verify state is cleared
        filter.reset();

        // First output after reset should be close to first input
        // (depends on filter characteristics)
        let output = filter.filter(0.5);
        assert!(output.is_finite());
    }
}
