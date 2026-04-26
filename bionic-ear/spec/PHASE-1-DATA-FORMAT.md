# WIA-AUG-009 PHASE 1 — Data Format Specification

**Standard:** WIA-AUG-009
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUG-009: Bionic Ear Specification v1.0

> **Standard ID:** WIA-AUG-009
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Human Augmentation Auditory Bionics Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Device Classification System](#2-device-classification-system)
3. [Sound Processing Strategies](#3-sound-processing-strategies)
4. [Electrode Array Configurations](#4-electrode-array-configurations)
5. [Frequency Mapping and Tonotopic Organization](#5-frequency-mapping-and-tonotopic-organization)
6. [Speech Recognition Optimization](#6-speech-recognition-optimization)
7. [Music Perception Enhancement](#7-music-perception-enhancement)
8. [Bilateral Implant Synchronization](#8-bilateral-implant-synchronization)
9. [Tinnitus Suppression Features](#9-tinnitus-suppression-features)
10. [Environmental Sound Classification](#10-environmental-sound-classification)
11. [Wireless Connectivity](#11-wireless-connectivity)
12. [Power and Battery Management](#12-power-and-battery-management)
13. [Safety and Biocompatibility](#13-safety-and-biocompatibility)
14. [Calibration and Fitting Procedures](#14-calibration-and-fitting-procedures)
15. [Performance Evaluation](#15-performance-evaluation)
16. [Implementation Guidelines](#16-implementation-guidelines)
17. [References](#17-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for bionic auditory devices, including cochlear implants, auditory brainstem implants, bone conduction devices, and middle ear implants. The standard ensures interoperability, safety, and optimal auditory performance across different manufacturers and processing systems.

### 1.2 Scope

The standard covers:
- Classification of bionic ear device types
- Sound processing strategy specifications
- Electrode array configuration standards
- Frequency mapping protocols
- Speech recognition optimization
- Music perception enhancement
- Bilateral synchronization
- Tinnitus management
- Environmental sound classification
- Wireless connectivity protocols

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Bionic ear technologies should restore not just hearing, but the full richness of auditory perception including speech, music, and environmental awareness. This specification ensures that hearing restoration systems are standardized, accessible, and continuously improving to serve all who need them.

### 1.4 Terminology

- **Cochlea**: Inner ear structure containing auditory receptor cells
- **Electrode Array**: Set of contacts for electrical stimulation
- **Tonotopic Organization**: Frequency-to-location mapping in cochlea
- **Stimulation Rate**: Pulses per second per electrode (Hz)
- **Channel**: Spectral band processed independently
- **MAP**: Programming settings for an individual user
- **T-Level**: Threshold of electrical stimulation perception
- **C-Level**: Comfortable loudness level
- **Dynamic Range**: Difference between T-level and C-level
- **SNR**: Signal-to-Noise Ratio
- **Telecoil**: Electromagnetic loop receiver

---

## 2. Device Classification System

### 2.1 Primary Device Categories

| Category | Implant Location | Electrode Count | Mechanism | Indication |
|----------|------------------|-----------------|-----------|------------|
| COCHLEAR_IMPLANT | Cochlea | 12-24 | Electrical | Sensorineural profound loss |
| ABI | Brainstem | 8-21 | Electrical | No functional cochlea |
| MIDDLE_EAR | Ossicular chain | N/A | Mechanical | Conductive/mixed loss |
| HYBRID | Cochlea (partial) | 6-16 | Electric + Acoustic | Partial hearing preservation |
| BONE_CONDUCTION | Skull (external/implanted) | N/A | Vibration | Conductive loss, SSD |

### 2.2 Classification Algorithm

```typescript
interface DeviceClassification {
  type: DeviceType;
  hearingLossType: 'sensorineural' | 'conductive' | 'mixed' | 'neural';
  hearingLossDegree: 'moderate' | 'severe' | 'profound';
  electrodeCount?: number;
  stimulationType: 'electrical' | 'mechanical' | 'vibrational' | 'hybrid';
  category: 'Basic' | 'Standard' | 'Advanced' | 'Premium';
}

function classifyDevice(input: {
  type: DeviceType;
  electrodes?: number;
  processingStrategy: ProcessingStrategy;
  features: string[];
}): DeviceClassification {
  const featureScore = input.features.length * 2;
  const electrodeScore = (input.electrodes || 0) * 1.5;
  const strategyScore = getStrategyComplexity(input.processingStrategy);

  const score = featureScore + electrodeScore + strategyScore;

  let category: string;
  if (score <= 30) category = 'Basic';
  else if (score <= 60) category = 'Standard';
  else if (score <= 90) category = 'Advanced';
  else category = 'Premium';

  return {
    ...input,
    category,
    stimulationType: getStimulationType(input.type)
  };
}
```

### 2.3 Device Complexity Score

```
Complexity = (Electrodes × 1.5) + (Features × 2) + (Strategy Complexity × 10)
```

Where:
- `Electrodes` = Number of electrode contacts (0-24)
- `Features` = Count of advanced features (0-20)
- `Strategy Complexity` = Processing algorithm sophistication (1-10)

---

## 3. Sound Processing Strategies

### 3.1 Processing Strategy Types

#### 3.1.1 SPEAK (Spectral Peak)

**Overview:** Selects 4-10 spectral peaks with highest amplitude

**Specifications:**
```
Filter Banks: 20-22 bandpass filters
Selected Channels: 4-10 per frame
Frame Rate: 180-300 Hz
Pulse Width: 50-400 µs
Amplitude Encoding: Logarithmic
```

**Algorithm:**
```typescript
interface SpeakStrategy {
  filterBanks: number; // 20-22
  selectedChannels: number; // 4-10
  frameRate: number; // Hz
  pulseWidth: number; // microseconds
  amplitudeMapping: 'logarithmic' | 'linear';
}

function processSPEAK(audioSignal: number[], config: SpeakStrategy): StimulationPattern {
  // 1. Bandpass filtering
  const filtered = bandpassFilterBank(audioSignal, config.filterBanks);

  // 2. Envelope detection
  const envelopes = filtered.map(band => detectEnvelope(band));

  // 3. Peak selection
  const peaks = selectNLargestPeaks(envelopes, config.selectedChannels);

  // 4. Amplitude mapping
  const amplitudes = peaks.map(p => mapAmplitude(p.value, config.amplitudeMapping));

  // 5. Generate stimulation pulses
  return generatePulses(peaks, amplitudes, config.pulseWidth);
}
```

#### 3.1.2 CIS (Continuous Interleaved Sampling)

**Overview:** Stimulates all channels continuously with interleaved pulses

**Specifications:**
```
Channels: 4-22
Stimulation Rate: 800-2400 Hz per channel
Pulse Width: 10-50 µs
Interleaving: Non-simultaneous pulses
Temporal Jitter: <5 µs
```

**Algorithm:**
```typescript
interface CISStrategy {
  channels: number; // 4-22
  stimulationRate: number; // Hz per channel
  pulseWidth: number; // microseconds
  compressionFunction: CompressionType;
}

function processCIS(audioSignal: number[], config: CISStrategy): StimulationPattern {
  // 1. Bandpass filtering
  const bands = bandpassFilterBank(audioSignal, config.channels);

  // 2. Envelope extraction
  const envelopes = bands.map(band => extractEnvelope(band));

  // 3. Compression
  const compressed = envelopes.map(env =>
    compress(env, config.compressionFunction)
  );

  // 4. Interleaved pulse generation
  const pulses = interleaveChannels(compressed, {
    rate: config.stimulationRate,
    width: config.pulseWidth,
    channels: config.channels
  });

  return pulses;
}
```

#### 3.1.3 ACE (Advanced Combination Encoder)

**Overview:** Combines spectral peak selection with high-rate stimulation

**Specifications:**
```
Total Channels: 8-22
Active Channels: 8-12 per frame
Stimulation Rate: 250-2400 Hz per channel
Frame Rate: 500-1800 Hz
Pulse Width: 20-50 µs
Dynamic Range: 40-60 dB
```

**Algorithm:**
```typescript
interface ACEStrategy {
  totalChannels: number; // 8-22
  activeChannels: number; // 8-12
  stimulationRate: number; // Hz per channel
  frameRate: number; // Hz
  pulseWidth: number; // microseconds
  dynamicRange: number; // dB
}

function processACE(audioSignal: number[], config: ACEStrategy): StimulationPattern {
  // 1. Analysis filterbank
  const spectrum = analyzeSpectrum(audioSignal, config.totalChannels);

  // 2. Channel selection (highest N peaks)
  const selectedChannels = selectMaximaChannels(
    spectrum,
    config.activeChannels
  );

  // 3. Envelope extraction and compression
  const envelopes = selectedChannels.map(ch => ({
    channel: ch.index,
    amplitude: compressAGC(ch.envelope, config.dynamicRange)
  }));

  // 4. High-rate pulse generation
  const stimulation = generateHighRatePulses(envelopes, {
    rate: config.stimulationRate,
    width: config.pulseWidth,
    frameRate: config.frameRate
  });

  return stimulation;
}
```

#### 3.1.4 FSP (Fine Structure Processing)

**Overview:** Preserves temporal fine structure for low frequencies

**Specifications:**
```
Channels: 12-22
Low-Freq Channels: 1-3 (preserve fine structure)
Mid/High Channels: Envelope coding
Stimulation Rate: 500-5000 Hz
Fine Structure Range: 125-1000 Hz
Envelope Range: 1000-8000 Hz
```

**Algorithm:**
```typescript
interface FSPStrategy {
  channels: number;
  fineStructureChannels: number; // 1-3
  envelopeChannels: number;
  fineStructureRange: { low: number; high: number };
  envelopeRange: { low: number; high: number };
  stimulationRate: number;
}

function processFSP(audioSignal: number[], config: FSPStrategy): StimulationPattern {
  // 1. Separate frequency bands
  const lowFreq = bandpassFilter(audioSignal, config.fineStructureRange);
  const highFreq = bandpassFilter(audioSignal, config.envelopeRange);

  // 2. Fine structure preservation (low frequencies)
  const fineStructure = preserveTemporalStructure(
    lowFreq,
    config.fineStructureChannels
  );

  // 3. Envelope extraction (high frequencies)
  const envelopes = extractEnvelopes(highFreq, config.envelopeChannels);

  // 4. Combine stimulation patterns
  const combined = combineStimulation(fineStructure, envelopes);

  return combined;
}
```

#### 3.1.5 HDCIS (High-Definition CIS)

**Overview:** Enhanced CIS with finer temporal and spectral resolution

**Specifications:**
```
Channels: 12-24
Stimulation Rate: 900-3500 Hz per channel
Pulse Width: 10-30 µs
Spectral Resolution: High (narrow filters)
Temporal Resolution: Ultra-high
Current Steering: Available
```

### 3.2 Strategy Comparison

| Strategy | Temporal Detail | Spectral Channels | Stimulation Rate | Best For |
|----------|----------------|-------------------|------------------|----------|
| SPEAK | Moderate | 4-10 selected | 180-300 Hz | Speech in quiet |
| CIS | High | 4-22 all | 800-2400 Hz | Speech in noise |
| ACE | Very High | 8-12 selected | 250-2400 Hz | General purpose |
| FSP | Exceptional | 12-22 (varied) | 500-5000 Hz | Music, pitch |
| HDCIS | Ultra High | 12-24 all | 900-3500 Hz | Premium performance |

### 3.3 Strategy Selection Guidelines

```typescript
function recommendStrategy(requirements: {
  primaryUse: 'speech' | 'music' | 'general';
  environment: 'quiet' | 'noise' | 'varied';
  electrodeCount: number;
  hearingHistory: 'prelingual' | 'postlingual';
}): ProcessingStrategy {
  if (requirements.primaryUse === 'music' && requirements.electrodeCount >= 12) {
    return 'FSP';
  }

  if (requirements.environment === 'noise' && requirements.electrodeCount >= 12) {
    return 'HDCIS';
  }

  if (requirements.electrodeCount >= 16) {
    return 'ACE';
  }

  if (requirements.primaryUse === 'speech' && requirements.environment === 'quiet') {
    return 'SPEAK';
  }

  return 'CIS'; // Default, most robust
}
```

---

## 4. Electrode Array Configurations

### 4.1 Array Types

#### 4.1.1 Perimodiolar Array

**Design:**
- Pre-curved or self-curling design
- Hugs modiolus (central cochlear axis)
- Closer to spiral ganglion cells
- Typically 12-24 contacts

**Specifications:**
```
Insertion Depth: 22-26 mm
Contact Spacing: 0.5-1.5 mm
Total Contacts: 12-24
Contact Area: 0.1-0.4 mm²
Impedance Range: 3-20 kΩ
Material: Platinum-iridium alloy
```

**Advantages:**
- Lower stimulation thresholds
- Reduced current spread
- Better frequency selectivity
- Lower power consumption

**Challenges:**
- Higher insertion trauma risk
- Requires surgical precision

#### 4.1.2 Lateral Wall Array

**Design:**
- Straight or minimally curved
- Follows lateral cochlear wall
- Minimally traumatic insertion
- Typically 16-24 contacts

**Specifications:**
```
Insertion Depth: 20-31 mm
Contact Spacing: 0.5-2.0 mm
Total Contacts: 16-24
Contact Area: 0.2-0.5 mm²
Impedance Range: 4-25 kΩ
Material: Platinum-iridium alloy
```

**Advantages:**
- Atraumatic insertion
- Preserves cochlear structures
- Suitable for hearing preservation
- Lower surgical risk

**Challenges:**
- Higher stimulation currents
- More current spread
- Potentially wider spatial spread

#### 4.1.3 Hybrid Array

**Design:**
- Short electrode (10-20 mm)
- Preserves apical low-frequency hearing
- Combines acoustic and electric stimulation
- Typically 6-16 contacts

**Specifications:**
```
Insertion Depth: 10-20 mm
Contact Spacing: 0.5-1.5 mm
Total Contacts: 6-16
Frequency Coverage: 1500-8000 Hz (electric)
Acoustic Preservation: 125-1500 Hz
Material: Platinum-iridium, silicone
```

**Advantages:**
- Preserves residual hearing
- Natural low-frequency perception
- Better music appreciation
- Combined benefit

**Challenges:**
- Requires residual low-frequency hearing
- Complex programming
- Acoustic component failure risk

### 4.2 Electrode Configuration Standards

```typescript
interface ElectrodeArray {
  type: ElectrodeConfig;
  contactCount: number;
  spacing: number; // mm
  insertionDepth: number; // mm
  material: ElectrodeMaterial;
  impedanceRange: { min: number; max: number }; // kΩ
  contactArea: number; // mm²
}

const standardArrays: ElectrodeArray[] = [
  {
    type: 'PERIMODIOLAR',
    contactCount: 22,
    spacing: 0.75,
    insertionDepth: 25,
    material: 'Platinum-Iridium',
    impedanceRange: { min: 3, max: 20 },
    contactArea: 0.2
  },
  {
    type: 'LATERAL_WALL',
    contactCount: 24,
    spacing: 1.1,
    insertionDepth: 28,
    material: 'Platinum-Iridium',
    impedanceRange: { min: 4, max: 25 },
    contactArea: 0.3
  },
  {
    type: 'HYBRID',
    contactCount: 10,
    spacing: 1.0,
    insertionDepth: 16,
    material: 'Platinum-Iridium',
    impedanceRange: { min: 3, max: 18 },
    contactArea: 0.15
  }
];
```

### 4.3 Current Steering

**Technology:** Virtual channels between physical electrodes

```typescript
interface CurrentSteering {
  enabled: boolean;
  virtualChannels: number; // Total virtual positions
  physicalElectrodes: number; // Actual contacts
  steeringRatio: number; // 0-1 (balance between adjacent electrodes)
  spectralResolution: number; // Effective frequency bands
}

function createVirtualChannel(
  electrode1: number,
  electrode2: number,
  ratio: number
): VirtualChannel {
  // Simultaneous stimulation of adjacent electrodes
  const current1 = (1 - ratio) * totalCurrent;
  const current2 = ratio * totalCurrent;

  return {
    electrodes: [electrode1, electrode2],
    currents: [current1, current2],
    perceptualPitch: interpolatePitch(electrode1, electrode2, ratio)
  };
}
```

**Benefits:**
- Increases effective spectral resolution
- Improves pitch perception
- Enhances music appreciation
- 8× increase in pitch discrimination

---

## 5. Frequency Mapping and Tonotopic Organization

### 5.1 Tonotopic Principles

The cochlea is organized tonotopically:
- Base (near round window): High frequencies (8000+ Hz)
- Apex (cochlear tip): Low frequencies (125-500 Hz)

**Standard Frequency Allocation:**
```
Electrode 1 (Base): 5500-7938 Hz
Electrode 5: 2063-3175 Hz
Electrode 10: 875-1313 Hz
Electrode 15: 413-594 Hz
Electrode 22 (Apex): 188-250 Hz
```

### 5.2 Frequency Mapping Algorithm

```typescript
interface FrequencyMap {
  electrodeNumber: number;
  centerFrequency: number; // Hz
  frequencyRange: { low: number; high: number }; // Hz
  characteristicFrequency: number; // Hz (cochlear position)
  gainAdjustment: number; // dB
}

function mapFrequencies(
  electrodeCount: number,
  totalRange: { low: number; high: number },
  tonotopic: boolean = true
): FrequencyMap[] {
  const maps: FrequencyMap[] = [];

  for (let i = 0; i < electrodeCount; i++) {
    // Logarithmic frequency distribution (matches cochlear physiology)
    const ratio = i / (electrodeCount - 1);

    const low = totalRange.low * Math.pow(
      totalRange.high / totalRange.low,
      ratio
    );

    const high = totalRange.low * Math.pow(
      totalRange.high / totalRange.low,
      (i + 1) / (electrodeCount - 1)
    );

    const center = Math.sqrt(low * high); // Geometric mean

    // Characteristic frequency from Greenwood function
    const characteristicFreq = tonotopic
      ? greenwoodFunction(ratio)
      : center;

    maps.push({
      electrodeNumber: i + 1,
      centerFrequency: center,
      frequencyRange: { low, high },
      characteristicFrequency: characteristicFreq,
      gainAdjustment: 0 // Individualized during fitting
    });
  }

  return maps;
}

// Greenwood frequency-position function for human cochlea
function greenwoodFunction(normalizedPosition: number): number {
  // Position: 0 (apex) to 1 (base)
  const A = 165.4; // Hz
  const a = 2.1; // slope
  const k = 0.88; // offset

  return A * (Math.pow(10, a * (1 - normalizedPosition)) - k);
}
```

### 5.3 Frequency Allocation Tables

#### Standard Frequency Table (22 Electrodes)

```typescript
const standardFrequencyTable: FrequencyMap[] = [
  { electrodeNumber: 1, centerFrequency: 6938, frequencyRange: { low: 5938, high: 7938 }, characteristicFrequency: 7000, gainAdjustment: 0 },
  { electrodeNumber: 2, centerFrequency: 6063, frequencyRange: { low: 5313, high: 6813 }, characteristicFrequency: 6200, gainAdjustment: 0 },
  { electrodeNumber: 3, centerFrequency: 5313, frequencyRange: { low: 4688, high: 5938 }, characteristicFrequency: 5500, gainAdjustment: 0 },
  { electrodeNumber: 4, centerFrequency: 4688, frequencyRange: { low: 4125, high: 5250 }, characteristicFrequency: 4900, gainAdjustment: 0 },
  { electrodeNumber: 5, centerFrequency: 4125, frequencyRange: { low: 3625, high: 4625 }, characteristicFrequency: 4400, gainAdjustment: 0 },
  { electrodeNumber: 6, centerFrequency: 3625, frequencyRange: { low: 3188, high: 4063 }, characteristicFrequency: 3900, gainAdjustment: 0 },
  { electrodeNumber: 7, centerFrequency: 3188, frequencyRange: { low: 2813, high: 3563 }, characteristicFrequency: 3500, gainAdjustment: 0 },
  { electrodeNumber: 8, centerFrequency: 2813, frequencyRange: { low: 2438, high: 3188 }, characteristicFrequency: 3100, gainAdjustment: 0 },
  { electrodeNumber: 9, centerFrequency: 2438, frequencyRange: { low: 2125, high: 2750 }, characteristicFrequency: 2700, gainAdjustment: 0 },
  { electrodeNumber: 10, centerFrequency: 2125, frequencyRange: { low: 1875, high: 2375 }, characteristicFrequency: 2400, gainAdjustment: 0 },
  { electrodeNumber: 11, centerFrequency: 1875, frequencyRange: { low: 1625, high: 2125 }, characteristicFrequency: 2100, gainAdjustment: 0 },
  { electrodeNumber: 12, centerFrequency: 1625, frequencyRange: { low: 1438, high: 1813 }, characteristicFrequency: 1850, gainAdjustment: 0 },
  { electrodeNumber: 13, centerFrequency: 1438, frequencyRange: { low: 1250, high: 1625 }, characteristicFrequency: 1600, gainAdjustment: 0 },
  { electrodeNumber: 14, centerFrequency: 1250, frequencyRange: { low: 1094, high: 1406 }, characteristicFrequency: 1400, gainAdjustment: 0 },
  { electrodeNumber: 15, centerFrequency: 1094, frequencyRange: { low: 938, high: 1250 }, characteristicFrequency: 1200, gainAdjustment: 0 },
  { electrodeNumber: 16, centerFrequency: 938, frequencyRange: { low: 813, high: 1063 }, characteristicFrequency: 1050, gainAdjustment: 0 },
  { electrodeNumber: 17, centerFrequency: 813, frequencyRange: { low: 688, high: 938 }, characteristicFrequency: 900, gainAdjustment: 0 },
  { electrodeNumber: 18, centerFrequency: 688, frequencyRange: { low: 594, high: 781 }, characteristicFrequency: 750, gainAdjustment: 0 },
  { electrodeNumber: 19, centerFrequency: 594, frequencyRange: { low: 500, high: 688 }, characteristicFrequency: 630, gainAdjustment: 0 },
  { electrodeNumber: 20, centerFrequency: 500, frequencyRange: { low: 438, high: 563 }, characteristicFrequency: 530, gainAdjustment: 0 },
  { electrodeNumber: 21, centerFrequency: 438, frequencyRange: { low: 375, high: 500 }, characteristicFrequency: 450, gainAdjustment: 0 },
  { electrodeNumber: 22, centerFrequency: 375, frequencyRange: { low: 188, high: 563 }, characteristicFrequency: 350, gainAdjustment: 0 }
];
```

### 5.4 Frequency Customization

```typescript
interface FrequencyCustomization {
  patientId: string;
  deviceId: string;
  baseMap: FrequencyMap[];
  adjustments: FrequencyAdjustment[];
  validatedDate: Date;
}

interface FrequencyAdjustment {
  electrode: number;
  frequencyShift: number; // Hz (+ or -)
  gainChange: number; // dB (+ or -)
  reason: string; // 'pitch_matching' | 'comfort' | 'speech_clarity'
}

function customizeFrequencyMap(
  baseMap: FrequencyMap[],
  pitchMatchingResults: PitchPerception[],
  speechResults: SpeechRecognitionScore[]
): FrequencyMap[] {
  const customized = [...baseMap];

  // Apply pitch matching adjustments
  pitchMatchingResults.forEach(result => {
    const electrode = customized.find(e => e.electrodeNumber === result.electrode);
    if (electrode) {
      electrode.centerFrequency += result.perceivedPitchShift;
      electrode.frequencyRange.low += result.perceivedPitchShift;
      electrode.frequencyRange.high += result.perceivedPitchShift;
    }
  });

  // Apply speech-based optimizations
  speechResults.forEach(result => {
    if (result.performance < 0.7) {
      // Boost gain for underperforming frequencies
      const electrode = customized.find(e =>
        e.centerFrequency >= result.frequencyBand.low &&
        e.centerFrequency <= result.frequencyBand.high
      );
      if (electrode) {
        electrode.gainAdjustment += 3; // dB
      }
    }
  });

  return customized;
}
```

---

## 6. Speech Recognition Optimization

### 6.1 Speech Processing Enhancements

#### 6.1.1 Noise Reduction

**Algorithms:**
```typescript
type NoiseReductionMode =
  | 'off'
  | 'low'
  | 'medium'
  | 'high'
  | 'adaptive';

interface NoiseReduction {
  mode: NoiseReductionMode;
  snrThreshold: number; // dB
  reductionStrength: number; // 0-1
  windNoiseSuppressionenabled: boolean;
  transientNoiseReduction: boolean;
}

function applyNoiseReduction(
  signal: AudioSignal,
  noise: NoiseProfile,
  config: NoiseReduction
): AudioSignal {
  // Spectral subtraction
  const noiseEstimate = estimateNoiseSpectrum(signal, noise);
  const cleaned = spectralSubtraction(signal, noiseEstimate, config.reductionStrength);

  // Wiener filtering
  const wienerFiltered = wienerFilter(cleaned, noise, config.snrThreshold);

  // Transient suppression
  if (config.transientNoiseReduction) {
    return suppressTransients(wienerFiltered);
  }

  return wienerFiltered;
}
```

#### 6.1.2 Directional Microphones

**Configuration:**
```typescript
interface DirectionalMicrophone {
  mode: 'omnidirectional' | 'narrow' | 'medium' | 'wide' | 'adaptive';
  beamWidth: number; // degrees
  frontBackRatio: number; // dB
  adaptiveSpeed: number; // ms response time
  windProtection: boolean;
}

function configureDirectionality(
  environment: AudioEnvironment,
  userPreference: DirectionalMode
): DirectionalMicrophone {
  if (environment.noiseLevel > 65 && environment.speechPresent) {
    return {
      mode: 'narrow',
      beamWidth: 60,
      frontBackRatio: 12,
      adaptiveSpeed: 200,
      windProtection: environment.wind > 10
    };
  }

  if (environment.noiseLevel < 50) {
    return {
      mode: 'omnidirectional',
      beamWidth: 360,
      frontBackRatio: 0,
      adaptiveSpeed: 0,
      windProtection: false
    };
  }

  return {
    mode: 'adaptive',
    beamWidth: 120,
    frontBackRatio: 8,
    adaptiveSpeed: 500,
    windProtection: true
  };
}
```

#### 6.1.3 ADRO (Adaptive Dynamic Range Optimization)

**Algorithm:**
```typescript
interface ADROConfig {
  targetOutputLevel: number; // dB SPL
  maxGain: number; // dB
  minGain: number; // dB
  attackTime: number; // ms
  releaseTime: number; // ms
  channelSpecific: boolean;
}

function applyADRO(
  signal: AudioSignal,
  channelAmplitudes: number[],
  config: ADROConfig
): AudioSignal {
  const adjustedChannels = channelAmplitudes.map((amplitude, channel) => {
    // Calculate required gain to reach target level
    const currentLevel = 20 * Math.log10(amplitude);
    const requiredGain = config.targetOutputLevel - currentLevel;

    // Clamp gain
    const appliedGain = Math.max(
      config.minGain,
      Math.min(config.maxGain, requiredGain)
    );

    // Time-varying gain with attack/release
    const smoothedGain = smoothGain(
      appliedGain,
      config.attackTime,
      config.releaseTime
    );

    return amplitude * Math.pow(10, smoothedGain / 20);
  });

  return reconstructSignal(adjustedChannels);
}
```

### 6.2 Speech Features Enhancement

```typescript
interface SpeechEnhancement {
  voicedUnvoicedDetection: boolean;
  fundamentalFrequencyTracking: boolean;
  formantEnhancement: boolean;
  consonantEmphasis: number; // dB boost for high frequencies
  pitchShifting: number; // semitones
}

function enhanceSpeech(
  signal: AudioSignal,
  config: SpeechEnhancement
): AudioSignal {
  let enhanced = signal;


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
