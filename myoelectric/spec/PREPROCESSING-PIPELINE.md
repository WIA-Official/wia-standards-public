# WIA EMG Preprocessing Pipeline Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines the standard preprocessing pipeline for EMG signals in myoelectric prosthetic systems. The pipeline transforms raw EMG signals into clean, normalized data suitable for gesture recognition and prosthetic control.

## 2. Pipeline Architecture

```
Raw EMG → Notch Filter → Bandpass Filter → Rectification → Envelope → Normalization → Output
              ↓              ↓                 ↓              ↓            ↓
          50/60 Hz       20-450 Hz         Full-wave      RMS/MAV       MVC/%
```

## 3. Stage 1: Notch Filter (Power Line Interference Removal)

### 3.1 Purpose
Remove 50 Hz (Europe/Asia) or 60 Hz (Americas) power line interference and harmonics.

### 3.2 Implementation

```rust
/// Notch filter configuration
struct NotchFilterConfig {
    center_frequency: f32,  // 50.0 or 60.0 Hz
    quality_factor: f32,    // Q = 30.0 (default)
    sample_rate: f32,       // Input sample rate
}
```

### 3.3 Algorithm: Second-Order IIR Notch Filter

Transfer function:
```
H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
```

Coefficients:
```
w0 = 2 * PI * f0 / fs
alpha = sin(w0) / (2 * Q)

b0 = 1
b1 = -2 * cos(w0)
b2 = 1
a1 = -2 * cos(w0)
a2 = 1 - alpha

// Normalize by (1 + alpha)
```

### 3.4 Requirements

| Parameter | Value | Notes |
|-----------|-------|-------|
| Attenuation | > 40 dB | At center frequency |
| Bandwidth (-3dB) | < 2 Hz | Narrow notch |
| Harmonics | 2nd and 3rd | Optional filtering at 100/120 Hz, 150/180 Hz |

## 4. Stage 2: Bandpass Filter

### 4.1 Purpose
Remove motion artifacts (low frequency) and high-frequency noise while preserving EMG content.

### 4.2 Configuration

```rust
struct BandpassFilterConfig {
    low_cutoff: f32,    // 20.0 Hz (default)
    high_cutoff: f32,   // 450.0 Hz (default)
    order: u32,         // 4 (Butterworth)
    sample_rate: f32,
}
```

### 4.3 Recommended Parameters

| Application | Low Cutoff | High Cutoff | Notes |
|-------------|------------|-------------|-------|
| General | 20 Hz | 450 Hz | Standard EMG range |
| Fine control | 20 Hz | 500 Hz | Higher bandwidth |
| Motion artifact rejection | 30 Hz | 450 Hz | More aggressive low cut |
| Limited bandwidth | 20 Hz | 350 Hz | Lower sample rates |

### 4.4 Algorithm: Butterworth Filter

4th-order Butterworth bandpass implemented as cascaded 2nd-order sections (SOS) for numerical stability.

```rust
struct SecondOrderSection {
    b: [f32; 3],  // Numerator coefficients
    a: [f32; 3],  // Denominator coefficients
}

struct ButterworthBandpass {
    sections: Vec<SecondOrderSection>,
    states: Vec<[f32; 2]>,  // Filter states for each section
}
```

### 4.5 Requirements

| Parameter | Value |
|-----------|-------|
| Passband ripple | < 0.5 dB |
| Stopband attenuation | > 40 dB |
| Phase response | Linear phase preferred |
| Group delay | < 10 ms |

## 5. Stage 3: Rectification

### 5.1 Purpose
Convert bipolar EMG signal to unipolar signal representing muscle activation intensity.

### 5.2 Methods

#### 5.2.1 Full-Wave Rectification (Recommended)
```rust
fn full_wave_rectify(sample: f32) -> f32 {
    sample.abs()
}
```

#### 5.2.2 Half-Wave Rectification
```rust
fn half_wave_rectify(sample: f32) -> f32 {
    sample.max(0.0)
}
```

#### 5.2.3 Squaring
```rust
fn square_rectify(sample: f32) -> f32 {
    sample * sample
}
```

### 5.3 Recommendation
Full-wave rectification is recommended for prosthetic control due to:
- Preservation of all signal energy
- Simpler interpretation
- Better temporal resolution

## 6. Stage 4: Envelope Extraction

### 6.1 Purpose
Extract the smooth envelope representing muscle activation level over time.

### 6.2 Methods

#### 6.2.1 Moving Average (MAV)
```rust
struct MovingAverageEnvelope {
    window_size: usize,  // Samples (e.g., 100 for 100ms at 1kHz)
    buffer: RingBuffer<f32>,
}

fn compute_mav(window: &[f32]) -> f32 {
    window.iter().map(|x| x.abs()).sum::<f32>() / window.len() as f32
}
```

#### 6.2.2 Root Mean Square (RMS)
```rust
fn compute_rms(window: &[f32]) -> f32 {
    let sum_sq: f32 = window.iter().map(|x| x * x).sum();
    (sum_sq / window.len() as f32).sqrt()
}
```

#### 6.2.3 Low-Pass Filter
```rust
struct LowPassEnvelope {
    cutoff: f32,      // 2-6 Hz typical
    sample_rate: f32,
    filter: ButterworthLowpass,
}
```

### 6.3 Window Parameters

| Parameter | Typical Value | Notes |
|-----------|---------------|-------|
| Window size | 100-300 ms | Shorter = faster response, more noise |
| Overlap | 50-90% | Higher = smoother output |
| Update rate | 20-50 Hz | Output frame rate |

### 6.4 Recommended Configuration

```rust
struct EnvelopeConfig {
    method: EnvelopeMethod,    // RMS recommended
    window_ms: f32,            // 150 ms default
    overlap_percent: f32,      // 75% default
    smoothing_cutoff: f32,     // 3 Hz optional post-smoothing
}

enum EnvelopeMethod {
    MovingAverage,
    RootMeanSquare,  // Recommended
    LowPassFilter,
}
```

## 7. Stage 5: Normalization

### 7.1 Purpose
Scale EMG values to a standard range for consistent interpretation across users and sessions.

### 7.2 Methods

#### 7.2.1 MVC Normalization (Recommended)
```rust
/// Maximum Voluntary Contraction normalization
struct MVCNormalization {
    mvc_value: f32,        // MVC reference value
    calibration_date: u64, // Unix timestamp
}

fn normalize_mvc(value: f32, mvc: f32) -> f32 {
    (value / mvc * 100.0).clamp(0.0, 150.0)  // Output: % MVC
}
```

#### 7.2.2 Z-Score Normalization
```rust
struct ZScoreNormalization {
    mean: f32,
    std_dev: f32,
}

fn normalize_zscore(value: f32, mean: f32, std: f32) -> f32 {
    (value - mean) / std
}
```

#### 7.2.3 Min-Max Normalization
```rust
struct MinMaxNormalization {
    min_value: f32,
    max_value: f32,
}

fn normalize_minmax(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}
```

### 7.3 MVC Calibration Protocol

1. **Rest baseline** (5 seconds): Record resting EMG level
2. **Submaximal contraction** (3 contractions): 50% effort warm-up
3. **Maximum contraction** (3 contractions): 3-5 seconds each, 30 second rest between
4. **Calculate MVC**: Use peak value or mean of top 3 peaks

```rust
struct MVCCalibrationResult {
    muscle: String,
    mvc_value: f32,
    rest_baseline: f32,
    confidence: f32,        // Quality metric
    calibration_date: u64,
    num_trials: u32,
}
```

## 8. Real-Time Processing Requirements

### 8.1 Latency Budget

| Stage | Maximum Latency |
|-------|-----------------|
| Filtering | 5 ms |
| Rectification | 0.1 ms |
| Envelope | 10 ms (window dependent) |
| Normalization | 0.1 ms |
| **Total** | **< 50 ms** |

### 8.2 Streaming Processing

```rust
struct EMGPreprocessor {
    notch_filter: NotchFilter,
    bandpass_filter: ButterworthBandpass,
    envelope_extractor: EnvelopeExtractor,
    normalizer: Normalizer,
}

impl EMGPreprocessor {
    /// Process a single sample (for streaming)
    fn process_sample(&mut self, sample: f32) -> Option<f32> {
        let filtered = self.notch_filter.filter(sample);
        let filtered = self.bandpass_filter.filter(filtered);
        let rectified = filtered.abs();

        // Envelope returns value when window is complete
        self.envelope_extractor.add_sample(rectified)
            .map(|env| self.normalizer.normalize(env))
    }

    /// Process a buffer of samples
    fn process_buffer(&mut self, samples: &[f32]) -> Vec<f32> {
        samples.iter()
            .filter_map(|&s| self.process_sample(s))
            .collect()
    }
}
```

## 9. Quality Monitoring

### 9.1 Signal Quality Metrics

```rust
struct SignalQuality {
    snr_db: f32,              // Signal-to-noise ratio
    baseline_drift: f32,       // mV/s
    saturation_percent: f32,   // % of samples at rail
    contact_impedance: f32,    // If available from hardware
}

fn assess_quality(signal: &[f32], sample_rate: f32) -> SignalQuality {
    // Implementation details...
}
```

### 9.2 Quality Thresholds

| Metric | Good | Acceptable | Poor |
|--------|------|------------|------|
| SNR | > 25 dB | 15-25 dB | < 15 dB |
| Baseline drift | < 0.05 mV/s | 0.05-0.1 mV/s | > 0.1 mV/s |
| Saturation | < 0.1% | 0.1-1% | > 1% |

## 10. Configuration Profiles

### 10.1 Default Profile
```json
{
  "notch": { "frequency": 50, "q_factor": 30 },
  "bandpass": { "low": 20, "high": 450, "order": 4 },
  "envelope": { "method": "rms", "window_ms": 150, "overlap": 75 },
  "normalization": { "method": "mvc" }
}
```

### 10.2 Low-Latency Profile
```json
{
  "notch": { "frequency": 50, "q_factor": 30 },
  "bandpass": { "low": 20, "high": 450, "order": 2 },
  "envelope": { "method": "rms", "window_ms": 50, "overlap": 50 },
  "normalization": { "method": "mvc" }
}
```

### 10.3 High-Quality Profile
```json
{
  "notch": { "frequency": 50, "q_factor": 50, "harmonics": [100, 150] },
  "bandpass": { "low": 20, "high": 500, "order": 6 },
  "envelope": { "method": "rms", "window_ms": 200, "overlap": 90 },
  "normalization": { "method": "mvc" }
}
```

---

*WIA Myoelectric Standard - Preprocessing Pipeline v1.0.0*
