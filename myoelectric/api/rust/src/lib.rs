//! # WIA EMG Signal Processing Library
//!
//! This library provides standard EMG signal processing functions for
//! myoelectric prosthetic control systems according to WIA specifications.
//!
//! ## Modules
//!
//! - `signal::filter` - Notch and bandpass filters for noise removal
//! - `signal::rectify` - Signal rectification methods
//! - `signal::envelope` - Envelope extraction algorithms
//! - `signal::features` - Feature extraction for gesture recognition
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_emg_signal::prelude::*;
//!
//! // Create preprocessing pipeline
//! let mut pipeline = EMGPipeline::new(PipelineConfig {
//!     sample_rate: 1000.0,
//!     notch_freq: 50.0,
//!     bandpass_low: 20.0,
//!     bandpass_high: 450.0,
//!     envelope_window_ms: 150.0,
//! }).expect("Invalid config");
//!
//! // Process samples (replace with actual EMG data)
//! let raw_samples: Vec<f32> = vec![0.1, -0.2, 0.3, -0.1];
//! let processed = pipeline.process(&raw_samples);
//! ```

pub mod classifier;
pub mod realtime;
pub mod signal;

use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Re-exports for convenient usage
pub mod prelude {
    pub use crate::signal::envelope::{EnvelopeExtractor, EnvelopeMethod};
    pub use crate::signal::features::{EMGFeatures, FeatureExtractor, FeatureVector};
    pub use crate::signal::filter::{BandpassFilter, NotchFilter};
    pub use crate::signal::rectify::{rectify, RectifyMethod};
    pub use crate::{EMGPipeline, PipelineConfig};
}

/// Errors that can occur during signal processing
#[derive(Error, Debug)]
pub enum EMGError {
    #[error("Invalid sample rate: {0} Hz (must be > 0)")]
    InvalidSampleRate(f32),

    #[error("Invalid frequency: {0} Hz")]
    InvalidFrequency(f32),

    #[error("Invalid filter order: {0}")]
    InvalidFilterOrder(u32),

    #[error("Invalid window size: {0}")]
    InvalidWindowSize(usize),

    #[error("Insufficient samples: got {got}, need {need}")]
    InsufficientSamples { got: usize, need: usize },

    #[error("Configuration error: {0}")]
    ConfigError(String),
}

pub type Result<T> = std::result::Result<T, EMGError>;

/// Configuration for the EMG preprocessing pipeline
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PipelineConfig {
    /// Sampling frequency in Hz
    pub sample_rate: f32,
    /// Notch filter center frequency (50 or 60 Hz)
    pub notch_freq: f32,
    /// Bandpass filter low cutoff in Hz
    pub bandpass_low: f32,
    /// Bandpass filter high cutoff in Hz
    pub bandpass_high: f32,
    /// Envelope window size in milliseconds
    pub envelope_window_ms: f32,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1000.0,
            notch_freq: 50.0,
            bandpass_low: 20.0,
            bandpass_high: 450.0,
            envelope_window_ms: 150.0,
        }
    }
}

/// Complete EMG preprocessing pipeline
pub struct EMGPipeline {
    config: PipelineConfig,
    notch_filter: signal::filter::NotchFilter,
    bandpass_filter: signal::filter::BandpassFilter,
    envelope_extractor: signal::envelope::EnvelopeExtractor,
}

impl EMGPipeline {
    /// Create a new EMG pipeline with the given configuration
    pub fn new(config: PipelineConfig) -> Result<Self> {
        if config.sample_rate <= 0.0 {
            return Err(EMGError::InvalidSampleRate(config.sample_rate));
        }

        let notch_filter =
            signal::filter::NotchFilter::new(config.notch_freq, 30.0, config.sample_rate)?;

        let bandpass_filter = signal::filter::BandpassFilter::new(
            config.bandpass_low,
            config.bandpass_high,
            4,
            config.sample_rate,
        )?;

        let window_samples =
            (config.envelope_window_ms * config.sample_rate / 1000.0).round() as usize;
        let envelope_extractor = signal::envelope::EnvelopeExtractor::new(
            signal::envelope::EnvelopeMethod::RootMeanSquare,
            window_samples,
        )?;

        Ok(Self {
            config,
            notch_filter,
            bandpass_filter,
            envelope_extractor,
        })
    }

    /// Process a buffer of raw EMG samples
    pub fn process(&mut self, samples: &[f32]) -> Vec<f32> {
        // Stage 1: Notch filter
        let notched: Vec<f32> = samples
            .iter()
            .map(|&s| self.notch_filter.filter(s))
            .collect();

        // Stage 2: Bandpass filter
        let filtered: Vec<f32> = notched
            .iter()
            .map(|&s| self.bandpass_filter.filter(s))
            .collect();

        // Stage 3: Rectification (full-wave)
        let rectified: Vec<f32> = filtered.iter().map(|&s| s.abs()).collect();

        // Stage 4: Envelope extraction
        let mut envelope = Vec::new();
        for &sample in &rectified {
            if let Some(env_value) = self.envelope_extractor.add_sample(sample) {
                envelope.push(env_value);
            }
        }

        envelope
    }

    /// Process a single sample (for streaming applications)
    pub fn process_sample(&mut self, sample: f32) -> Option<f32> {
        let notched = self.notch_filter.filter(sample);
        let filtered = self.bandpass_filter.filter(notched);
        let rectified = filtered.abs();
        self.envelope_extractor.add_sample(rectified)
    }

    /// Reset all filter states
    pub fn reset(&mut self) {
        self.notch_filter.reset();
        self.bandpass_filter.reset();
        self.envelope_extractor.reset();
    }

    /// Get the current configuration
    pub fn config(&self) -> &PipelineConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = PipelineConfig::default();
        assert_eq!(config.sample_rate, 1000.0);
        assert_eq!(config.notch_freq, 50.0);
    }

    #[test]
    fn test_pipeline_creation() {
        let config = PipelineConfig::default();
        let pipeline = EMGPipeline::new(config);
        assert!(pipeline.is_ok());
    }

    #[test]
    fn test_invalid_sample_rate() {
        let config = PipelineConfig {
            sample_rate: -100.0,
            ..Default::default()
        };
        let pipeline = EMGPipeline::new(config);
        assert!(pipeline.is_err());
    }
}
