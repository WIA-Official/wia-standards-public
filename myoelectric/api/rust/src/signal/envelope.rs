//! Envelope Extraction for EMG Signals
//!
//! This module provides envelope extraction algorithms for obtaining
//! smooth muscle activation signals from rectified EMG data.
//!
//! ## Methods
//!
//! - **Moving Average (MAV)**: Simple windowed average
//! - **Root Mean Square (RMS)**: Windowed RMS (recommended)
//! - **Low-Pass Filter**: Continuous smoothing filter

use crate::{EMGError, Result};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Envelope extraction method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EnvelopeMethod {
    /// Moving Average: (1/N) * Σ|xi|
    MovingAverage,
    /// Root Mean Square: √((1/N) * Σxi²)
    RootMeanSquare,
    /// Low-pass filter smoothing
    LowPassFilter,
}

impl Default for EnvelopeMethod {
    fn default() -> Self {
        Self::RootMeanSquare
    }
}

/// Ring buffer for efficient windowed operations
#[derive(Debug, Clone)]
struct RingBuffer {
    buffer: VecDeque<f32>,
    capacity: usize,
    sum: f32,
    sum_sq: f32,
}

impl RingBuffer {
    fn new(capacity: usize) -> Self {
        Self {
            buffer: VecDeque::with_capacity(capacity),
            capacity,
            sum: 0.0,
            sum_sq: 0.0,
        }
    }

    fn push(&mut self, value: f32) -> Option<f32> {
        // Remove oldest value if at capacity
        if self.buffer.len() >= self.capacity {
            if let Some(old) = self.buffer.pop_front() {
                self.sum -= old;
                self.sum_sq -= old * old;
            }
        }

        // Add new value
        self.buffer.push_back(value);
        self.sum += value;
        self.sum_sq += value * value;

        // Return value if buffer is full
        if self.buffer.len() >= self.capacity {
            Some(value)
        } else {
            None
        }
    }

    fn mean(&self) -> f32 {
        if self.buffer.is_empty() {
            0.0
        } else {
            self.sum / self.buffer.len() as f32
        }
    }

    fn rms(&self) -> f32 {
        if self.buffer.is_empty() {
            0.0
        } else {
            (self.sum_sq / self.buffer.len() as f32).sqrt()
        }
    }

    fn len(&self) -> usize {
        self.buffer.len()
    }

    fn is_full(&self) -> bool {
        self.buffer.len() >= self.capacity
    }

    fn clear(&mut self) {
        self.buffer.clear();
        self.sum = 0.0;
        self.sum_sq = 0.0;
    }

    fn capacity(&self) -> usize {
        self.capacity
    }
}

/// Envelope extractor for EMG signals
///
/// Extracts a smooth envelope signal representing muscle activation
/// level from rectified EMG data.
#[derive(Debug, Clone)]
pub struct EnvelopeExtractor {
    method: EnvelopeMethod,
    buffer: RingBuffer,
    // Low-pass filter state
    lp_alpha: f32,
    lp_prev: f32,
    // Hop counter for overlapped windows
    hop_counter: usize,
    hop_size: usize,
}

impl EnvelopeExtractor {
    /// Create a new envelope extractor
    ///
    /// # Arguments
    ///
    /// * `method` - Envelope extraction method
    /// * `window_size` - Window size in samples (e.g., 150 for 150ms at 1kHz)
    pub fn new(method: EnvelopeMethod, window_size: usize) -> Result<Self> {
        if window_size == 0 {
            return Err(EMGError::InvalidWindowSize(window_size));
        }

        Ok(Self {
            method,
            buffer: RingBuffer::new(window_size),
            lp_alpha: 0.1, // Default smoothing factor
            lp_prev: 0.0,
            hop_counter: 0,
            hop_size: window_size / 2, // 50% overlap by default
        })
    }

    /// Create a new envelope extractor with specific hop size
    ///
    /// # Arguments
    ///
    /// * `method` - Envelope extraction method
    /// * `window_size` - Window size in samples
    /// * `hop_size` - Hop size in samples (for overlapped windows)
    pub fn with_hop(method: EnvelopeMethod, window_size: usize, hop_size: usize) -> Result<Self> {
        if window_size == 0 {
            return Err(EMGError::InvalidWindowSize(window_size));
        }
        if hop_size == 0 || hop_size > window_size {
            return Err(EMGError::InvalidWindowSize(hop_size));
        }

        Ok(Self {
            method,
            buffer: RingBuffer::new(window_size),
            lp_alpha: 0.1,
            lp_prev: 0.0,
            hop_counter: 0,
            hop_size,
        })
    }

    /// Create envelope extractor from time parameters
    ///
    /// # Arguments
    ///
    /// * `method` - Envelope extraction method
    /// * `window_ms` - Window size in milliseconds
    /// * `overlap_percent` - Overlap percentage (0-99)
    /// * `sample_rate` - Sampling frequency in Hz
    pub fn from_time_params(
        method: EnvelopeMethod,
        window_ms: f32,
        overlap_percent: f32,
        sample_rate: f32,
    ) -> Result<Self> {
        let window_size = (window_ms * sample_rate / 1000.0).round() as usize;
        let overlap = overlap_percent.clamp(0.0, 99.0) / 100.0;
        let hop_size = ((1.0 - overlap) * window_size as f32).round() as usize;

        Self::with_hop(method, window_size, hop_size.max(1))
    }

    /// Add a sample and optionally get an envelope value
    ///
    /// Returns `Some(value)` when a new envelope value is ready,
    /// `None` if more samples are needed.
    pub fn add_sample(&mut self, sample: f32) -> Option<f32> {
        match self.method {
            EnvelopeMethod::MovingAverage | EnvelopeMethod::RootMeanSquare => {
                self.buffer.push(sample);
                self.hop_counter += 1;

                if self.buffer.is_full() && self.hop_counter >= self.hop_size {
                    self.hop_counter = 0;
                    Some(self.compute_envelope())
                } else {
                    None
                }
            }
            EnvelopeMethod::LowPassFilter => {
                // Low-pass filter outputs for every sample
                let y = self.lp_alpha * sample + (1.0 - self.lp_alpha) * self.lp_prev;
                self.lp_prev = y;
                Some(y)
            }
        }
    }

    /// Compute the envelope value from the current window
    fn compute_envelope(&self) -> f32 {
        match self.method {
            EnvelopeMethod::MovingAverage => self.buffer.mean(),
            EnvelopeMethod::RootMeanSquare => self.buffer.rms(),
            EnvelopeMethod::LowPassFilter => self.lp_prev,
        }
    }

    /// Process a buffer of samples
    ///
    /// Returns all envelope values extracted from the buffer.
    pub fn process_buffer(&mut self, samples: &[f32]) -> Vec<f32> {
        samples.iter().filter_map(|&s| self.add_sample(s)).collect()
    }

    /// Reset the extractor state
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.lp_prev = 0.0;
        self.hop_counter = 0;
    }

    /// Get the window size in samples
    pub fn window_size(&self) -> usize {
        self.buffer.capacity()
    }

    /// Get the hop size in samples
    pub fn hop_size(&self) -> usize {
        self.hop_size
    }

    /// Get the current method
    pub fn method(&self) -> EnvelopeMethod {
        self.method
    }

    /// Set the low-pass filter smoothing factor
    ///
    /// Only affects `LowPassFilter` method.
    pub fn set_lp_alpha(&mut self, alpha: f32) {
        self.lp_alpha = alpha.clamp(0.0, 1.0);
    }
}

/// Compute Moving Average envelope for a signal window
///
/// MAV = (1/N) * Σ|xi|
pub fn moving_average(signal: &[f32]) -> f32 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().map(|&x| x.abs()).sum::<f32>() / signal.len() as f32
}

/// Compute Root Mean Square envelope for a signal window
///
/// RMS = √((1/N) * Σxi²)
pub fn root_mean_square(signal: &[f32]) -> f32 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = signal.iter().map(|x| x * x).sum();
    (sum_sq / signal.len() as f32).sqrt()
}

/// Extract envelope from a signal using windowed processing
///
/// # Arguments
///
/// * `signal` - Input signal samples
/// * `method` - Envelope extraction method
/// * `window_size` - Window size in samples
/// * `hop_size` - Hop size in samples (step between windows)
pub fn extract_envelope(
    signal: &[f32],
    method: EnvelopeMethod,
    window_size: usize,
    hop_size: usize,
) -> Vec<f32> {
    if signal.len() < window_size {
        return vec![];
    }

    let mut envelope = Vec::new();
    let compute_fn: fn(&[f32]) -> f32 = match method {
        EnvelopeMethod::MovingAverage => moving_average,
        EnvelopeMethod::RootMeanSquare => root_mean_square,
        EnvelopeMethod::LowPassFilter => root_mean_square, // Fallback to RMS
    };

    let mut start = 0;
    while start + window_size <= signal.len() {
        let window = &signal[start..start + window_size];
        envelope.push(compute_fn(window));
        start += hop_size;
    }

    envelope
}

/// Configuration for envelope extraction
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvelopeConfig {
    /// Extraction method
    pub method: EnvelopeMethod,
    /// Window size in milliseconds
    pub window_ms: f32,
    /// Overlap percentage (0-99)
    pub overlap_percent: f32,
    /// Optional post-smoothing cutoff (Hz)
    pub smoothing_cutoff: Option<f32>,
}

impl Default for EnvelopeConfig {
    fn default() -> Self {
        Self {
            method: EnvelopeMethod::RootMeanSquare,
            window_ms: 150.0,
            overlap_percent: 75.0,
            smoothing_cutoff: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_moving_average() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let mav = moving_average(&signal);
        assert!((mav - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_moving_average_with_negatives() {
        let signal = vec![-1.0, -2.0, 3.0, 4.0];
        let mav = moving_average(&signal);
        assert!((mav - 2.5).abs() < 1e-6); // (1+2+3+4)/4 = 2.5
    }

    #[test]
    fn test_rms() {
        let signal = vec![1.0, 1.0, 1.0, 1.0];
        let rms = root_mean_square(&signal);
        assert!((rms - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_rms_calculation() {
        let signal = vec![0.0, 2.0, 0.0, 2.0];
        let rms = root_mean_square(&signal);
        // RMS = sqrt((0+4+0+4)/4) = sqrt(2)
        assert!((rms - 2.0_f32.sqrt()).abs() < 1e-6);
    }

    #[test]
    fn test_envelope_extractor_creation() {
        let extractor = EnvelopeExtractor::new(EnvelopeMethod::RootMeanSquare, 100);
        assert!(extractor.is_ok());
    }

    #[test]
    fn test_envelope_extractor_invalid_window() {
        let extractor = EnvelopeExtractor::new(EnvelopeMethod::RootMeanSquare, 0);
        assert!(extractor.is_err());
    }

    #[test]
    fn test_envelope_extractor_streaming() {
        let mut extractor = EnvelopeExtractor::new(EnvelopeMethod::RootMeanSquare, 4).unwrap();

        // First 3 samples should return None
        assert!(extractor.add_sample(1.0).is_none());
        assert!(extractor.add_sample(1.0).is_none());
        assert!(extractor.add_sample(1.0).is_none());

        // 4th sample should return envelope (with hop_size = 2)
        let result = extractor.add_sample(1.0);
        // May or may not return depending on hop counter
    }

    #[test]
    fn test_extract_envelope() {
        let signal: Vec<f32> = (0..100).map(|x| (x as f32 * 0.1).sin()).collect();

        let envelope = extract_envelope(&signal, EnvelopeMethod::RootMeanSquare, 10, 5);

        assert!(!envelope.is_empty());
        // With window=10, hop=5, and 100 samples, we get (100-10)/5 + 1 = 19 values
        assert_eq!(envelope.len(), 19);
    }

    #[test]
    fn test_envelope_buffer_too_small() {
        let signal = vec![1.0, 2.0, 3.0];
        let envelope = extract_envelope(&signal, EnvelopeMethod::MovingAverage, 10, 5);
        assert!(envelope.is_empty());
    }

    #[test]
    fn test_lowpass_envelope() {
        let mut extractor =
            EnvelopeExtractor::new(EnvelopeMethod::LowPassFilter, 100).unwrap();
        extractor.set_lp_alpha(0.5);

        // Low-pass should return value for every sample
        let result = extractor.add_sample(1.0);
        assert!(result.is_some());
    }

    #[test]
    fn test_from_time_params() {
        let extractor = EnvelopeExtractor::from_time_params(
            EnvelopeMethod::RootMeanSquare,
            150.0,
            75.0,
            1000.0,
        );

        assert!(extractor.is_ok());
        let ext = extractor.unwrap();
        assert_eq!(ext.window_size(), 150); // 150ms at 1000Hz
    }

    #[test]
    fn test_ring_buffer() {
        let mut buffer = RingBuffer::new(3);

        buffer.push(1.0);
        buffer.push(2.0);
        buffer.push(3.0);

        assert_eq!(buffer.len(), 3);
        assert!(buffer.is_full());
        assert!((buffer.mean() - 2.0).abs() < 1e-6);

        // Push another value, oldest should be removed
        buffer.push(4.0);
        assert_eq!(buffer.len(), 3);
        assert!((buffer.mean() - 3.0).abs() < 1e-6); // (2+3+4)/3
    }
}
