//! EMG Feature Extraction
//!
//! This module provides feature extraction algorithms for EMG signal analysis,
//! including both time-domain and frequency-domain features commonly used
//! in gesture recognition and prosthetic control.
//!
//! ## Time-Domain Features
//!
//! - MAV (Mean Absolute Value)
//! - RMS (Root Mean Square)
//! - WL (Waveform Length)
//! - ZC (Zero Crossings)
//! - SSC (Slope Sign Changes)
//! - IEMG (Integrated EMG)
//! - VAR (Variance)
//! - WAMP (Willison Amplitude)
//!
//! ## Frequency-Domain Features
//!
//! - MNF (Mean Frequency)
//! - MDF (Median Frequency)
//! - PKF (Peak Frequency)
//! - Total Power

use num_complex::Complex;
use rustfft::{FftDirection, FftPlanner};
use serde::{Deserialize, Serialize};
use std::f32::consts::PI;

/// Collection of EMG features for a single channel window
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EMGFeatures {
    /// Mean Absolute Value
    pub mav: f32,
    /// Root Mean Square
    pub rms: f32,
    /// Waveform Length
    pub wl: f32,
    /// Zero Crossings
    pub zc: u32,
    /// Slope Sign Changes
    pub ssc: u32,
    /// Integrated EMG
    pub iemg: f32,
    /// Variance
    pub var: f32,
    /// Mean Frequency (Hz)
    pub mnf: f32,
    /// Median Frequency (Hz)
    pub mdf: f32,
}

impl EMGFeatures {
    /// Convert to feature vector (for ML input)
    pub fn to_vec(&self) -> Vec<f32> {
        vec![
            self.mav,
            self.rms,
            self.wl,
            self.zc as f32,
            self.ssc as f32,
            self.iemg,
            self.var,
            self.mnf,
            self.mdf,
        ]
    }

    /// Get feature names
    pub fn feature_names() -> Vec<&'static str> {
        vec!["mav", "rms", "wl", "zc", "ssc", "iemg", "var", "mnf", "mdf"]
    }
}

/// Feature vector for multi-channel EMG
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureVector {
    /// Timestamp in milliseconds
    pub timestamp: u64,
    /// Number of channels
    pub channel_count: usize,
    /// Feature names
    pub feature_names: Vec<String>,
    /// Flattened feature values
    pub features: Vec<f32>,
}

impl FeatureVector {
    /// Get features for a specific channel
    pub fn channel_features(&self, channel: usize) -> Option<&[f32]> {
        let features_per_channel = self.features.len() / self.channel_count;
        let start = channel * features_per_channel;
        let end = start + features_per_channel;
        if end <= self.features.len() {
            Some(&self.features[start..end])
        } else {
            None
        }
    }
}

/// Feature extractor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureConfig {
    /// Zero crossing threshold (for noise rejection)
    pub zc_threshold: f32,
    /// Slope sign change threshold
    pub ssc_threshold: f32,
    /// Willison amplitude threshold
    pub wamp_threshold: f32,
    /// Sample rate for frequency calculations
    pub sample_rate: f32,
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self {
            zc_threshold: 0.01,
            ssc_threshold: 0.0001,
            wamp_threshold: 0.05,
            sample_rate: 1000.0,
        }
    }
}

/// Multi-channel feature extractor
#[derive(Debug, Clone)]
pub struct FeatureExtractor {
    config: FeatureConfig,
}

impl FeatureExtractor {
    /// Create a new feature extractor with default configuration
    pub fn new() -> Self {
        Self {
            config: FeatureConfig::default(),
        }
    }

    /// Create a feature extractor with custom configuration
    pub fn with_config(config: FeatureConfig) -> Self {
        Self { config }
    }

    /// Extract features from a single channel window
    pub fn extract_channel(&self, signal: &[f32]) -> EMGFeatures {
        let mav = mean_absolute_value(signal);
        let rms = root_mean_square(signal);
        let wl = waveform_length(signal);
        let zc = zero_crossings(signal, self.config.zc_threshold);
        let ssc = slope_sign_changes(signal, self.config.ssc_threshold);
        let iemg = integrated_emg(signal);
        let var = variance(signal);

        // Frequency features
        let psd = power_spectral_density(signal, self.config.sample_rate);
        let mnf = mean_frequency(&psd);
        let mdf = median_frequency(&psd);

        EMGFeatures {
            mav,
            rms,
            wl,
            zc,
            ssc,
            iemg,
            var,
            mnf,
            mdf,
        }
    }

    /// Extract features from multiple channels
    pub fn extract_multi_channel(
        &self,
        channels: &[Vec<f32>],
        timestamp: u64,
    ) -> FeatureVector {
        let mut features = Vec::new();
        let mut feature_names = Vec::new();

        for (ch_idx, signal) in channels.iter().enumerate() {
            let ch_features = self.extract_channel(signal);

            for (name, value) in EMGFeatures::feature_names()
                .iter()
                .zip(ch_features.to_vec().iter())
            {
                feature_names.push(format!("ch{}_{}", ch_idx, name));
                features.push(*value);
            }
        }

        FeatureVector {
            timestamp,
            channel_count: channels.len(),
            feature_names,
            features,
        }
    }

    /// Extract standard feature set (MAV, RMS, WL, ZC, SSC)
    pub fn extract_standard(&self, signal: &[f32]) -> Vec<f32> {
        vec![
            mean_absolute_value(signal),
            root_mean_square(signal),
            waveform_length(signal),
            zero_crossings(signal, self.config.zc_threshold) as f32,
            slope_sign_changes(signal, self.config.ssc_threshold) as f32,
        ]
    }

    /// Get configuration
    pub fn config(&self) -> &FeatureConfig {
        &self.config
    }
}

impl Default for FeatureExtractor {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Time-Domain Features
// ============================================================================

/// Mean Absolute Value (MAV)
///
/// MAV = (1/N) * Σ|xi|
///
/// Represents average rectified signal amplitude.
pub fn mean_absolute_value(signal: &[f32]) -> f32 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().map(|x| x.abs()).sum::<f32>() / signal.len() as f32
}

/// Root Mean Square (RMS)
///
/// RMS = √((1/N) * Σxi²)
///
/// Represents signal power.
pub fn root_mean_square(signal: &[f32]) -> f32 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = signal.iter().map(|x| x * x).sum();
    (sum_sq / signal.len() as f32).sqrt()
}

/// Waveform Length (WL)
///
/// WL = Σ|xi - xi-1|
///
/// Measures signal complexity and cumulative length.
pub fn waveform_length(signal: &[f32]) -> f32 {
    signal
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .sum()
}

/// Zero Crossings (ZC)
///
/// Counts sign changes with threshold for noise rejection.
pub fn zero_crossings(signal: &[f32], threshold: f32) -> u32 {
    signal
        .windows(2)
        .filter(|w| w[0].signum() != w[1].signum() && (w[1] - w[0]).abs() > threshold)
        .count() as u32
}

/// Slope Sign Changes (SSC)
///
/// Counts changes in signal slope direction.
pub fn slope_sign_changes(signal: &[f32], threshold: f32) -> u32 {
    signal
        .windows(3)
        .filter(|w| {
            let diff1 = w[1] - w[0];
            let diff2 = w[1] - w[2];
            diff1 * diff2 > threshold
        })
        .count() as u32
}

/// Integrated EMG (IEMG)
///
/// IEMG = Σ|xi|
pub fn integrated_emg(signal: &[f32]) -> f32 {
    signal.iter().map(|x| x.abs()).sum()
}

/// Variance (VAR)
///
/// VAR = (1/(N-1)) * Σ(xi - μ)²
pub fn variance(signal: &[f32]) -> f32 {
    if signal.len() < 2 {
        return 0.0;
    }
    let mean: f32 = signal.iter().sum::<f32>() / signal.len() as f32;
    let sum_sq: f32 = signal.iter().map(|x| (x - mean).powi(2)).sum();
    sum_sq / (signal.len() - 1) as f32
}

/// Willison Amplitude (WAMP)
///
/// Counts amplitude changes exceeding threshold.
pub fn willison_amplitude(signal: &[f32], threshold: f32) -> u32 {
    signal
        .windows(2)
        .filter(|w| (w[1] - w[0]).abs() > threshold)
        .count() as u32
}

/// Simple Square Integral (SSI)
///
/// SSI = Σxi²
pub fn simple_square_integral(signal: &[f32]) -> f32 {
    signal.iter().map(|x| x * x).sum()
}

/// Log Detector
///
/// LOG = exp((1/N) * Σlog|xi|)
pub fn log_detector(signal: &[f32]) -> f32 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_log: f32 = signal
        .iter()
        .map(|x| x.abs().max(1e-10).ln())
        .sum();
    (sum_log / signal.len() as f32).exp()
}

// ============================================================================
// Frequency-Domain Features
// ============================================================================

/// Power Spectral Density result
#[derive(Debug, Clone)]
pub struct PowerSpectrum {
    /// Frequency bins (Hz)
    pub frequencies: Vec<f32>,
    /// Power values
    pub powers: Vec<f32>,
}

/// Compute Power Spectral Density using FFT
pub fn power_spectral_density(signal: &[f32], sample_rate: f32) -> PowerSpectrum {
    let n = signal.len();
    if n == 0 {
        return PowerSpectrum {
            frequencies: vec![],
            powers: vec![],
        };
    }

    // Apply Hann window
    let windowed: Vec<Complex<f32>> = signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let window = 0.5 * (1.0 - (2.0 * PI * i as f32 / (n - 1) as f32).cos());
            Complex::new(x * window, 0.0)
        })
        .collect();

    // Compute FFT
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft(n, FftDirection::Forward);

    let mut buffer = windowed;
    fft.process(&mut buffer);

    // Compute power spectrum (only positive frequencies)
    let n_freqs = n / 2 + 1;
    let freq_resolution = sample_rate / n as f32;

    let frequencies: Vec<f32> = (0..n_freqs).map(|i| i as f32 * freq_resolution).collect();

    let powers: Vec<f32> = buffer[..n_freqs]
        .iter()
        .map(|c| c.norm_sqr() / n as f32)
        .collect();

    PowerSpectrum {
        frequencies,
        powers,
    }
}

/// Mean Frequency (MNF)
///
/// MNF = Σ(fi * Pi) / ΣPi
///
/// Center of mass of the power spectrum.
pub fn mean_frequency(psd: &PowerSpectrum) -> f32 {
    if psd.powers.is_empty() {
        return 0.0;
    }

    let weighted_sum: f32 = psd
        .frequencies
        .iter()
        .zip(psd.powers.iter())
        .map(|(f, p)| f * p)
        .sum();

    let power_sum: f32 = psd.powers.iter().sum();

    if power_sum > 0.0 {
        weighted_sum / power_sum
    } else {
        0.0
    }
}

/// Median Frequency (MDF)
///
/// Frequency that divides power spectrum into equal halves.
pub fn median_frequency(psd: &PowerSpectrum) -> f32 {
    if psd.powers.is_empty() {
        return 0.0;
    }

    let total_power: f32 = psd.powers.iter().sum();
    let half_power = total_power / 2.0;

    let mut cumulative = 0.0;
    for (freq, power) in psd.frequencies.iter().zip(psd.powers.iter()) {
        cumulative += power;
        if cumulative >= half_power {
            return *freq;
        }
    }

    psd.frequencies.last().copied().unwrap_or(0.0)
}

/// Peak Frequency (PKF)
///
/// Frequency with maximum power.
pub fn peak_frequency(psd: &PowerSpectrum) -> f32 {
    psd.frequencies
        .iter()
        .zip(psd.powers.iter())
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(f, _)| *f)
        .unwrap_or(0.0)
}

/// Total Power
///
/// Sum of all spectral power.
pub fn total_power(psd: &PowerSpectrum) -> f32 {
    psd.powers.iter().sum()
}

/// Spectral Entropy
///
/// Measure of spectral distribution uniformity.
pub fn spectral_entropy(psd: &PowerSpectrum) -> f32 {
    let total: f32 = psd.powers.iter().sum();
    if total <= 0.0 {
        return 0.0;
    }

    -psd.powers
        .iter()
        .filter(|&&p| p > 0.0)
        .map(|p| {
            let normalized = p / total;
            normalized * normalized.ln()
        })
        .sum::<f32>()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mav() {
        let signal = vec![1.0, -2.0, 3.0, -4.0];
        let mav = mean_absolute_value(&signal);
        assert!((mav - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_rms() {
        let signal = vec![1.0, 1.0, 1.0, 1.0];
        let rms = root_mean_square(&signal);
        assert!((rms - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_waveform_length() {
        let signal = vec![0.0, 1.0, 0.0, 1.0];
        let wl = waveform_length(&signal);
        assert!((wl - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_zero_crossings() {
        let signal = vec![-1.0, 1.0, -1.0, 1.0];
        let zc = zero_crossings(&signal, 0.0);
        assert_eq!(zc, 3);
    }

    #[test]
    fn test_zero_crossings_with_threshold() {
        let signal = vec![-0.001, 0.001, -0.001, 0.001];
        let zc = zero_crossings(&signal, 0.01);
        assert_eq!(zc, 0); // Changes too small
    }

    #[test]
    fn test_variance() {
        let signal = vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let var = variance(&signal);
        // Known variance for this dataset
        assert!((var - 4.571).abs() < 0.01);
    }

    #[test]
    fn test_slope_sign_changes() {
        let signal = vec![0.0, 1.0, 0.0, 1.0, 0.0];
        let ssc = slope_sign_changes(&signal, 0.0);
        assert_eq!(ssc, 3);
    }

    #[test]
    fn test_feature_extractor() {
        let signal: Vec<f32> = (0..100)
            .map(|i| (i as f32 * 0.1).sin() * 0.5)
            .collect();

        let extractor = FeatureExtractor::new();
        let features = extractor.extract_channel(&signal);

        assert!(features.mav > 0.0);
        assert!(features.rms > 0.0);
        assert!(features.wl > 0.0);
    }

    #[test]
    fn test_multi_channel_features() {
        let ch1: Vec<f32> = (0..100).map(|i| (i as f32 * 0.1).sin()).collect();
        let ch2: Vec<f32> = (0..100).map(|i| (i as f32 * 0.2).cos()).collect();

        let extractor = FeatureExtractor::new();
        let features = extractor.extract_multi_channel(&[ch1, ch2], 1234567890);

        assert_eq!(features.channel_count, 2);
        assert_eq!(features.features.len(), 18); // 9 features * 2 channels
        assert_eq!(features.timestamp, 1234567890);
    }

    #[test]
    fn test_power_spectrum() {
        // Create a simple sine wave
        let sample_rate = 1000.0;
        let freq = 50.0;
        let signal: Vec<f32> = (0..1000)
            .map(|i| (2.0 * PI * freq * i as f32 / sample_rate).sin())
            .collect();

        let psd = power_spectral_density(&signal, sample_rate);

        assert!(!psd.frequencies.is_empty());
        assert!(!psd.powers.is_empty());

        // Peak should be near 50 Hz
        let pkf = peak_frequency(&psd);
        assert!((pkf - 50.0).abs() < 5.0);
    }

    #[test]
    fn test_mean_frequency() {
        let sample_rate = 1000.0;
        let signal: Vec<f32> = (0..1000)
            .map(|i| (2.0 * PI * 100.0 * i as f32 / sample_rate).sin())
            .collect();

        let psd = power_spectral_density(&signal, sample_rate);
        let mnf = mean_frequency(&psd);

        // Mean frequency should be close to signal frequency
        assert!((mnf - 100.0).abs() < 20.0);
    }

    #[test]
    fn test_empty_signal() {
        let signal: Vec<f32> = vec![];

        assert_eq!(mean_absolute_value(&signal), 0.0);
        assert_eq!(root_mean_square(&signal), 0.0);
        assert_eq!(waveform_length(&signal), 0.0);
        assert_eq!(variance(&signal), 0.0);
    }

    #[test]
    fn test_feature_vector_channel_access() {
        let features = FeatureVector {
            timestamp: 0,
            channel_count: 2,
            feature_names: vec!["a".to_string(), "b".to_string(), "c".to_string(), "d".to_string()],
            features: vec![1.0, 2.0, 3.0, 4.0],
        };

        let ch0 = features.channel_features(0).unwrap();
        assert_eq!(ch0, &[1.0, 2.0]);

        let ch1 = features.channel_features(1).unwrap();
        assert_eq!(ch1, &[3.0, 4.0]);
    }
}
