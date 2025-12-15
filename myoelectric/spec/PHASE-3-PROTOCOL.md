# WIA EMG Feature Extraction Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines the standard feature extraction methods for EMG signal analysis in myoelectric prosthetic control. Features are numerical descriptors computed from EMG signal windows that serve as inputs to gesture classification algorithms.

## 2. Feature Extraction Pipeline

```
Preprocessed EMG → Windowing → Feature Computation → Feature Vector → Classifier
                      ↓              ↓
                  200-300 ms     Temporal + Frequency
                  50-75% overlap
```

## 3. Windowing Parameters

### 3.1 Configuration

```rust
struct WindowConfig {
    window_size_ms: f32,    // 200-300 ms recommended
    overlap_percent: f32,   // 50-75% overlap
    sample_rate: u32,       // Input sample rate
}

impl WindowConfig {
    fn window_samples(&self) -> usize {
        (self.window_size_ms * self.sample_rate as f32 / 1000.0) as usize
    }

    fn hop_samples(&self) -> usize {
        let overlap = self.overlap_percent / 100.0;
        (self.window_samples() as f32 * (1.0 - overlap)) as usize
    }
}
```

### 3.2 Recommended Parameters

| Application | Window Size | Overlap | Output Rate |
|-------------|-------------|---------|-------------|
| Real-time control | 200 ms | 50% | 10 Hz |
| Gesture recognition | 250 ms | 75% | 16 Hz |
| High accuracy | 300 ms | 75% | 13 Hz |

### 3.3 Latency Considerations

Total system latency = Window size + Processing time + Control delay

For responsive prosthetic control, target < 300 ms total latency.

## 4. Time-Domain Features

### 4.1 Mean Absolute Value (MAV)

Represents average rectified signal amplitude.

```rust
/// Mean Absolute Value
/// MAV = (1/N) * Σ|xi|
fn mav(signal: &[f32]) -> f32 {
    let sum: f32 = signal.iter().map(|x| x.abs()).sum();
    sum / signal.len() as f32
}
```

**Properties:**
- Simple and computationally efficient
- Robust to outliers
- Good indicator of muscle contraction level

### 4.2 Root Mean Square (RMS)

Represents signal power, more sensitive to amplitude variations.

```rust
/// Root Mean Square
/// RMS = √((1/N) * Σxi²)
fn rms(signal: &[f32]) -> f32 {
    let sum_sq: f32 = signal.iter().map(|x| x * x).sum();
    (sum_sq / signal.len() as f32).sqrt()
}
```

**Properties:**
- Physically related to signal power
- More sensitive to peaks than MAV
- Standard amplitude measure

### 4.3 Waveform Length (WL)

Measures signal complexity/cumulative length.

```rust
/// Waveform Length
/// WL = Σ|xi - xi-1|
fn waveform_length(signal: &[f32]) -> f32 {
    signal.windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .sum()
}
```

**Properties:**
- Captures signal complexity
- Sensitive to motor unit firing rate
- Good discriminator for different gestures

### 4.4 Zero Crossings (ZC)

Counts frequency of signal crossing zero (with threshold to avoid noise).

```rust
/// Zero Crossings
/// ZC = Σ[sign(xi) ≠ sign(xi-1) AND |xi - xi-1| > threshold]
fn zero_crossings(signal: &[f32], threshold: f32) -> u32 {
    signal.windows(2)
        .filter(|w| {
            w[0].signum() != w[1].signum() &&
            (w[1] - w[0]).abs() > threshold
        })
        .count() as u32
}
```

**Properties:**
- Related to dominant frequency
- Threshold prevents noise-induced crossings
- Default threshold: 0.01 mV or 1% of signal range

### 4.5 Slope Sign Changes (SSC)

Counts changes in signal slope direction.

```rust
/// Slope Sign Changes
/// SSC = Σ[(xi - xi-1)(xi - xi+1) > threshold]
fn slope_sign_changes(signal: &[f32], threshold: f32) -> u32 {
    signal.windows(3)
        .filter(|w| {
            let diff1 = w[1] - w[0];
            let diff2 = w[1] - w[2];
            diff1 * diff2 > threshold
        })
        .count() as u32
}
```

**Properties:**
- Related to signal frequency content
- Indicates motor unit complexity
- Default threshold: 0.0001 mV²

### 4.6 Integrated EMG (IEMG)

Summation of absolute signal values.

```rust
/// Integrated EMG
/// IEMG = Σ|xi|
fn iemg(signal: &[f32]) -> f32 {
    signal.iter().map(|x| x.abs()).sum()
}
```

### 4.7 Variance (VAR)

Statistical variance of the signal.

```rust
/// Variance
/// VAR = (1/(N-1)) * Σ(xi - μ)²
fn variance(signal: &[f32]) -> f32 {
    let mean: f32 = signal.iter().sum::<f32>() / signal.len() as f32;
    let sum_sq: f32 = signal.iter().map(|x| (x - mean).powi(2)).sum();
    sum_sq / (signal.len() - 1) as f32
}
```

### 4.8 Willison Amplitude (WAMP)

Counts amplitude changes exceeding a threshold.

```rust
/// Willison Amplitude
/// WAMP = Σ[|xi - xi-1| > threshold]
fn willison_amplitude(signal: &[f32], threshold: f32) -> u32 {
    signal.windows(2)
        .filter(|w| (w[1] - w[0]).abs() > threshold)
        .count() as u32
}
```

### 4.9 Simple Square Integral (SSI)

Energy measure of the signal.

```rust
/// Simple Square Integral
/// SSI = Σxi²
fn ssi(signal: &[f32]) -> f32 {
    signal.iter().map(|x| x * x).sum()
}
```

### 4.10 Log Detector (LOG)

Logarithmic energy measure.

```rust
/// Log Detector
/// LOG = exp((1/N) * Σlog|xi|)
fn log_detector(signal: &[f32]) -> f32 {
    let sum_log: f32 = signal.iter()
        .map(|x| x.abs().max(1e-10).ln())
        .sum();
    (sum_log / signal.len() as f32).exp()
}
```

## 5. Frequency-Domain Features

### 5.1 Power Spectral Density

```rust
struct PowerSpectrum {
    frequencies: Vec<f32>,
    powers: Vec<f32>,
}

fn compute_psd(signal: &[f32], sample_rate: f32) -> PowerSpectrum {
    // Apply window function
    let windowed = apply_hann_window(signal);

    // Compute FFT
    let fft_result = fft(&windowed);

    // Compute power spectrum
    let powers: Vec<f32> = fft_result.iter()
        .map(|c| c.norm_sqr() / signal.len() as f32)
        .collect();

    // Generate frequency bins
    let frequencies: Vec<f32> = (0..powers.len())
        .map(|i| i as f32 * sample_rate / signal.len() as f32)
        .collect();

    PowerSpectrum { frequencies, powers }
}
```

### 5.2 Mean Frequency (MNF)

Center of mass of the power spectrum.

```rust
/// Mean Frequency
/// MNF = Σ(fi * Pi) / ΣPi
fn mean_frequency(psd: &PowerSpectrum) -> f32 {
    let weighted_sum: f32 = psd.frequencies.iter()
        .zip(psd.powers.iter())
        .map(|(f, p)| f * p)
        .sum();
    let power_sum: f32 = psd.powers.iter().sum();
    weighted_sum / power_sum
}
```

**Properties:**
- Shifts lower with muscle fatigue
- Good fatigue indicator
- Range: 50-150 Hz typical

### 5.3 Median Frequency (MDF)

Frequency that divides power spectrum into equal halves.

```rust
/// Median Frequency
/// MDF: frequency where cumulative power = 50% of total
fn median_frequency(psd: &PowerSpectrum) -> f32 {
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
```

**Properties:**
- Less sensitive to noise than MNF
- Better fatigue indicator
- Range: 60-120 Hz typical

### 5.4 Peak Frequency (PKF)

Frequency with maximum power.

```rust
/// Peak Frequency
fn peak_frequency(psd: &PowerSpectrum) -> f32 {
    psd.frequencies.iter()
        .zip(psd.powers.iter())
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(f, _)| *f)
        .unwrap_or(0.0)
}
```

### 5.5 Total Power (TTP)

Sum of all spectral power.

```rust
/// Total Power
fn total_power(psd: &PowerSpectrum) -> f32 {
    psd.powers.iter().sum()
}
```

### 5.6 Band Power Ratios

Power in specific frequency bands.

```rust
struct BandPowers {
    low: f32,     // 20-60 Hz
    mid: f32,     // 60-120 Hz
    high: f32,    // 120-250 Hz
}

fn compute_band_powers(psd: &PowerSpectrum) -> BandPowers {
    let mut low = 0.0;
    let mut mid = 0.0;
    let mut high = 0.0;

    for (freq, power) in psd.frequencies.iter().zip(psd.powers.iter()) {
        if *freq >= 20.0 && *freq < 60.0 {
            low += power;
        } else if *freq >= 60.0 && *freq < 120.0 {
            mid += power;
        } else if *freq >= 120.0 && *freq < 250.0 {
            high += power;
        }
    }

    BandPowers { low, mid, high }
}
```

### 5.7 Spectral Entropy

Measure of spectral distribution uniformity.

```rust
/// Spectral Entropy
fn spectral_entropy(psd: &PowerSpectrum) -> f32 {
    let total: f32 = psd.powers.iter().sum();
    let normalized: Vec<f32> = psd.powers.iter()
        .map(|p| p / total)
        .collect();

    -normalized.iter()
        .filter(|&&p| p > 0.0)
        .map(|p| p * p.ln())
        .sum::<f32>()
}
```

## 6. Feature Vector Construction

### 6.1 Standard Feature Set

```rust
struct FeatureVector {
    timestamp: u64,
    channel_count: usize,
    feature_names: Vec<String>,
    features: Vec<f32>,
}

struct FeatureExtractor {
    config: ExtractorConfig,
}

impl FeatureExtractor {
    fn extract_standard(&self, channels: &[Vec<f32>]) -> FeatureVector {
        let mut features = Vec::new();
        let mut names = Vec::new();

        for (ch_idx, signal) in channels.iter().enumerate() {
            // Time domain features
            features.push(mav(signal));
            names.push(format!("ch{}_mav", ch_idx));

            features.push(rms(signal));
            names.push(format!("ch{}_rms", ch_idx));

            features.push(waveform_length(signal));
            names.push(format!("ch{}_wl", ch_idx));

            features.push(zero_crossings(signal, self.config.zc_threshold) as f32);
            names.push(format!("ch{}_zc", ch_idx));

            features.push(slope_sign_changes(signal, self.config.ssc_threshold) as f32);
            names.push(format!("ch{}_ssc", ch_idx));

            // Frequency domain features
            let psd = compute_psd(signal, self.config.sample_rate);
            features.push(mean_frequency(&psd));
            names.push(format!("ch{}_mnf", ch_idx));

            features.push(median_frequency(&psd));
            names.push(format!("ch{}_mdf", ch_idx));
        }

        FeatureVector {
            timestamp: current_timestamp(),
            channel_count: channels.len(),
            feature_names: names,
            features,
        }
    }
}
```

### 6.2 Feature Set Recommendations

| Use Case | Features | Dimension |
|----------|----------|-----------|
| Basic (4 ch) | MAV, RMS, WL, ZC | 16 |
| Standard (4 ch) | MAV, RMS, WL, ZC, SSC, MNF, MDF | 28 |
| Advanced (8 ch) | All time + frequency features | 80+ |

### 6.3 Feature Normalization

```rust
enum FeatureNormalization {
    None,
    ZScore { mean: Vec<f32>, std: Vec<f32> },
    MinMax { min: Vec<f32>, max: Vec<f32> },
    UnitLength,  // L2 normalization
}

fn normalize_features(features: &[f32], norm: &FeatureNormalization) -> Vec<f32> {
    match norm {
        FeatureNormalization::ZScore { mean, std } => {
            features.iter()
                .zip(mean.iter().zip(std.iter()))
                .map(|(f, (m, s))| (f - m) / s)
                .collect()
        }
        FeatureNormalization::MinMax { min, max } => {
            features.iter()
                .zip(min.iter().zip(max.iter()))
                .map(|(f, (mn, mx))| (f - mn) / (mx - mn))
                .collect()
        }
        FeatureNormalization::UnitLength => {
            let norm: f32 = features.iter().map(|x| x * x).sum::<f32>().sqrt();
            features.iter().map(|x| x / norm).collect()
        }
        FeatureNormalization::None => features.to_vec(),
    }
}
```

## 7. Real-Time Feature Extraction

### 7.1 Streaming Architecture

```rust
struct StreamingFeatureExtractor {
    window_config: WindowConfig,
    channel_buffers: Vec<RingBuffer<f32>>,
    feature_config: FeatureConfig,
    last_extraction_time: u64,
}

impl StreamingFeatureExtractor {
    fn add_samples(&mut self, channel: usize, samples: &[f32]) {
        self.channel_buffers[channel].extend(samples);
    }

    fn try_extract(&mut self) -> Option<FeatureVector> {
        // Check if enough samples available
        if !self.has_full_window() {
            return None;
        }

        // Check if hop interval elapsed
        if !self.hop_elapsed() {
            return None;
        }

        // Extract features from current windows
        let windows: Vec<Vec<f32>> = self.channel_buffers
            .iter()
            .map(|buf| buf.get_window(self.window_config.window_samples()))
            .collect();

        let features = self.extract_features(&windows);
        self.advance_hop();

        Some(features)
    }
}
```

### 7.2 Performance Requirements

| Metric | Requirement |
|--------|-------------|
| Extraction time | < 5 ms per window |
| Memory per channel | < 10 KB |
| CPU usage | < 10% single core |

## 8. Feature Selection Guidelines

### 8.1 Correlation Analysis

Remove highly correlated features (|r| > 0.95):
- MAV and IEMG are often redundant
- ZC and MNF are related

### 8.2 Recommended Minimal Set

For efficient real-time control with 4 channels:
1. MAV (4 features)
2. WL (4 features)
3. ZC (4 features)
4. SSC (4 features)

Total: 16 features

### 8.3 Enhanced Set for Better Accuracy

Add frequency domain features:
1. MNF (4 features)
2. MDF (4 features)

Total: 24 features

## 9. Output Format

### 9.1 Feature Vector JSON

```json
{
  "timestamp": 1705312800000,
  "windowSizeMs": 200,
  "channelCount": 4,
  "featureCount": 28,
  "featureNames": ["ch0_mav", "ch0_rms", "ch0_wl", ...],
  "features": [0.123, 0.156, 45.2, ...],
  "metadata": {
    "extractorVersion": "1.0.0",
    "normalization": "zscore"
  }
}
```

### 9.2 Binary Format

```
Header (16 bytes):
  [0-3]   Timestamp (ms): uint32
  [4-5]   Channel count: uint16
  [6-7]   Features per channel: uint16
  [8-11]  Total features: uint32
  [12-15] Flags: uint32

Data:
  Features: float32[total_features]
```

---

*WIA Myoelectric Standard - Feature Extraction v1.0.0*
