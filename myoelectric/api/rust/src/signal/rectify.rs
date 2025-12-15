//! Signal Rectification Methods
//!
//! This module provides rectification functions for converting
//! bipolar EMG signals to unipolar signals representing muscle
//! activation intensity.
//!
//! ## Methods
//!
//! - **Full-wave**: Takes absolute value of all samples (recommended)
//! - **Half-wave**: Keeps only positive samples
//! - **Square**: Squares each sample (emphasizes high amplitudes)

use serde::{Deserialize, Serialize};

/// Rectification method selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RectifyMethod {
    /// Full-wave rectification: |x|
    FullWave,
    /// Half-wave rectification: max(0, x)
    HalfWave,
    /// Square rectification: x²
    Square,
}

impl Default for RectifyMethod {
    fn default() -> Self {
        Self::FullWave
    }
}

/// Rectify a single sample using the specified method
///
/// # Arguments
///
/// * `sample` - Input sample value
/// * `method` - Rectification method to apply
///
/// # Returns
///
/// Rectified sample value (always non-negative)
///
/// # Example
///
/// ```rust
/// use wia_emg_signal::signal::rectify::{rectify, RectifyMethod};
///
/// let sample = -0.5;
/// let rectified = rectify(sample, RectifyMethod::FullWave);
/// assert_eq!(rectified, 0.5);
/// ```
#[inline]
pub fn rectify(sample: f32, method: RectifyMethod) -> f32 {
    match method {
        RectifyMethod::FullWave => full_wave(sample),
        RectifyMethod::HalfWave => half_wave(sample),
        RectifyMethod::Square => square(sample),
    }
}

/// Full-wave rectification
///
/// Returns the absolute value of the input.
/// This is the recommended method for EMG signal processing as it
/// preserves all signal energy.
///
/// # Formula
///
/// y = |x|
#[inline]
pub fn full_wave(sample: f32) -> f32 {
    sample.abs()
}

/// Half-wave rectification
///
/// Returns the input if positive, otherwise zero.
/// This method discards negative portions of the signal.
///
/// # Formula
///
/// y = max(0, x)
#[inline]
pub fn half_wave(sample: f32) -> f32 {
    sample.max(0.0)
}

/// Square rectification
///
/// Returns the square of the input.
/// This method emphasizes larger amplitudes and is always positive.
///
/// # Formula
///
/// y = x²
#[inline]
pub fn square(sample: f32) -> f32 {
    sample * sample
}

/// Rectify an entire buffer of samples
///
/// # Arguments
///
/// * `samples` - Input sample buffer
/// * `method` - Rectification method to apply
///
/// # Returns
///
/// Vector of rectified samples
pub fn rectify_buffer(samples: &[f32], method: RectifyMethod) -> Vec<f32> {
    samples.iter().map(|&s| rectify(s, method)).collect()
}

/// Rectify samples in-place
///
/// # Arguments
///
/// * `samples` - Mutable sample buffer to rectify
/// * `method` - Rectification method to apply
pub fn rectify_inplace(samples: &mut [f32], method: RectifyMethod) {
    for sample in samples.iter_mut() {
        *sample = rectify(*sample, method);
    }
}

/// Full-wave rectify a buffer
///
/// Convenience function for the most common rectification method.
pub fn full_wave_buffer(samples: &[f32]) -> Vec<f32> {
    samples.iter().map(|&s| s.abs()).collect()
}

/// Streaming rectifier for real-time processing
#[derive(Debug, Clone)]
pub struct Rectifier {
    method: RectifyMethod,
}

impl Rectifier {
    /// Create a new rectifier with the specified method
    pub fn new(method: RectifyMethod) -> Self {
        Self { method }
    }

    /// Create a full-wave rectifier (default)
    pub fn full_wave() -> Self {
        Self::new(RectifyMethod::FullWave)
    }

    /// Rectify a single sample
    #[inline]
    pub fn process(&self, sample: f32) -> f32 {
        rectify(sample, self.method)
    }

    /// Rectify a buffer of samples
    pub fn process_buffer(&self, samples: &[f32]) -> Vec<f32> {
        rectify_buffer(samples, self.method)
    }

    /// Get the current rectification method
    pub fn method(&self) -> RectifyMethod {
        self.method
    }

    /// Set the rectification method
    pub fn set_method(&mut self, method: RectifyMethod) {
        self.method = method;
    }
}

impl Default for Rectifier {
    fn default() -> Self {
        Self::full_wave()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_full_wave() {
        assert_eq!(full_wave(1.0), 1.0);
        assert_eq!(full_wave(-1.0), 1.0);
        assert_eq!(full_wave(0.0), 0.0);
        assert_eq!(full_wave(-0.5), 0.5);
    }

    #[test]
    fn test_half_wave() {
        assert_eq!(half_wave(1.0), 1.0);
        assert_eq!(half_wave(-1.0), 0.0);
        assert_eq!(half_wave(0.0), 0.0);
        assert_eq!(half_wave(-0.5), 0.0);
        assert_eq!(half_wave(0.5), 0.5);
    }

    #[test]
    fn test_square() {
        assert_eq!(square(2.0), 4.0);
        assert_eq!(square(-2.0), 4.0);
        assert_eq!(square(0.0), 0.0);
        assert_eq!(square(0.5), 0.25);
    }

    #[test]
    fn test_rectify_with_method() {
        let sample = -0.75;

        assert_eq!(rectify(sample, RectifyMethod::FullWave), 0.75);
        assert_eq!(rectify(sample, RectifyMethod::HalfWave), 0.0);
        assert_eq!(rectify(sample, RectifyMethod::Square), 0.5625);
    }

    #[test]
    fn test_rectify_buffer() {
        let samples = vec![-1.0, 0.5, -0.5, 1.0];

        let full = rectify_buffer(&samples, RectifyMethod::FullWave);
        assert_eq!(full, vec![1.0, 0.5, 0.5, 1.0]);

        let half = rectify_buffer(&samples, RectifyMethod::HalfWave);
        assert_eq!(half, vec![0.0, 0.5, 0.0, 1.0]);
    }

    #[test]
    fn test_rectify_inplace() {
        let mut samples = vec![-1.0, 0.5, -0.5, 1.0];
        rectify_inplace(&mut samples, RectifyMethod::FullWave);
        assert_eq!(samples, vec![1.0, 0.5, 0.5, 1.0]);
    }

    #[test]
    fn test_rectifier_struct() {
        let rectifier = Rectifier::new(RectifyMethod::FullWave);
        assert_eq!(rectifier.process(-0.5), 0.5);

        let rectifier = Rectifier::default();
        assert_eq!(rectifier.method(), RectifyMethod::FullWave);
    }

    #[test]
    fn test_energy_preservation() {
        let samples = vec![-1.0, 1.0, -0.5, 0.5];

        // Full-wave preserves total absolute amplitude
        let full_sum: f32 = rectify_buffer(&samples, RectifyMethod::FullWave)
            .iter()
            .sum();
        let original_sum: f32 = samples.iter().map(|x| x.abs()).sum();
        assert_eq!(full_sum, original_sum);

        // Half-wave loses negative portions
        let half_sum: f32 = rectify_buffer(&samples, RectifyMethod::HalfWave)
            .iter()
            .sum();
        assert!(half_sum < original_sum);
    }
}
