# WIA-AUG-009 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-009
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

if (config.voicedUnvoicedDetection) {
    const voicingDecision = detectVoicing(signal);
    enhanced = processVoicing(enhanced, voicingDecision);
  }

  if (config.formantEnhancement) {
    const formants = extractFormants(enhanced);
    enhanced = emphasizeFormants(enhanced, formants, 6); // 6 dB boost
  }

  if (config.consonantEmphasis > 0) {
    enhanced = boostHighFrequencies(enhanced, 2000, config.consonantEmphasis);
  }

  return enhanced;
}
```

### 6.3 Speech Recognition Metrics

```typescript
interface SpeechRecognitionMetrics {
  // Quiet conditions
  quietSentences: number; // % correct
  quietWords: number; // % correct CNC
  quietPhonemes: number; // % correct

  // Noise conditions
  noiseSentencesPlus10dB: number; // % at +10 dB SNR
  noiseSentencesPlus5dB: number; // % at +5 dB SNR
  noiseSentences0dB: number; // % at 0 dB SNR

  // SNR for 50% intelligibility
  snr50: number; // dB

  // Additional metrics
  adaptiveRatio: number; // improvement with adaptive features
  bilateralBenefit: number; // % improvement with both ears
}

const performanceBenchmarks = {
  excellent: { quietSentences: 90, snr50: -5 },
  good: { quietSentences: 75, snr50: 0 },
  fair: { quietSentences: 60, snr50: 5 },
  needsOptimization: { quietSentences: 50, snr50: 10 }
};
```

---

## 7. Music Perception Enhancement

### 7.1 Music Processing Features

#### 7.1.1 Pitch Refinement

**Technology:** Enhanced frequency resolution for pitch discrimination

```typescript
interface PitchRefinement {
  enabled: boolean;
  virtualChannels: number; // via current steering
  fineStructureCoding: boolean;
  fundamentalFrequencyEnhancement: boolean;
  harmonicPreservation: boolean;
}

function refinePitch(
  musicalSignal: AudioSignal,
  config: PitchRefinement
): AudioSignal {
  if (!config.enabled) return musicalSignal;

  // Extract fundamental frequency
  const f0 = extractFundamentalFrequency(musicalSignal);

  // Enhance harmonics
  let enhanced = musicalSignal;
  if (config.harmonicPreservation) {
    const harmonics = extractHarmonics(musicalSignal, f0);
    enhanced = emphasizeHarmonics(enhanced, harmonics);
  }

  // Fine structure coding for low frequencies
  if (config.fineStructureCoding) {
    const lowFreq = bandpassFilter(enhanced, { low: 125, high: 1000 });
    const fineStructure = preserveFineStructure(lowFreq);
    enhanced = combineWithEnvelope(fineStructure, enhanced);
  }

  // Current steering for virtual channels
  if (config.virtualChannels > 0) {
    enhanced = applyCurrentSteering(enhanced, config.virtualChannels);
  }

  return enhanced;
}
```

#### 7.1.2 Harmonic Enhancement

**Algorithm:**
```typescript
interface HarmonicEnhancement {
  harmonicCount: number; // 1-8 harmonics
  fundamentalBoost: number; // dB
  harmonicBoost: number; // dB for 2nd-4th harmonics
  inharmonicityReduction: boolean;
}

function enhanceHarmonics(
  signal: AudioSignal,
  f0: number,
  config: HarmonicEnhancement
): AudioSignal {
  const harmonics: number[] = [];

  // Identify harmonic frequencies
  for (let n = 1; n <= config.harmonicCount; n++) {
    harmonics.push(n * f0);
  }

  // Boost harmonic content
  let enhanced = signal;
  harmonics.forEach((freq, index) => {
    const boost = index === 0
      ? config.fundamentalBoost
      : config.harmonicBoost;

    enhanced = boostFrequencyBand(enhanced, freq, boost, 50); // 50 Hz bandwidth
  });

  return enhanced;
}
```

#### 7.1.3 Temporal Fine Structure

**Specification:**
```
Processing: Preserve phase information in low frequencies
Frequency Range: 125-1000 Hz
Temporal Resolution: <1 ms
Carrier Frequency: Match F0 of musical note
Benefit: Improved pitch perception, timbre, melody recognition
```

```typescript
function preserveTemporalFineStructure(
  signal: AudioSignal,
  frequencyRange: { low: number; high: number }
): AudioSignal {
  // Extract low-frequency component
  const lowFreq = bandpassFilter(signal, frequencyRange);

  // Zero-crossing analysis for phase
  const zeroCrossings = detectZeroCrossings(lowFreq);

  // Generate stimulation pulses at zero-crossings
  const pulses = generatePulsesAtZeroCrossings(zeroCrossings);

  // Modulate with envelope
  const envelope = extractEnvelope(lowFreq);
  const modulated = modulatePulses(pulses, envelope);

  return modulated;
}
```

### 7.2 Music Program Settings

```typescript
interface MusicProgram {
  name: string;
  compressionRatio: number; // Lower for music (1.5-3:1)
  inputDynamicRange: number; // Wider for music (60-80 dB)
  processingStrategy: ProcessingStrategy; // Prefer FSP
  microphone: 'music' | 'speech'; // Different frequency emphasis
  pitchRefinement: PitchRefinement;
  harmonicEnhancement: HarmonicEnhancement;
  tempoTracking: boolean;
}

const musicPresets: Record<string, MusicProgram> = {
  classical: {
    name: 'Classical Music',
    compressionRatio: 2.0,
    inputDynamicRange: 80,
    processingStrategy: 'FSP',
    microphone: 'music',
    pitchRefinement: {
      enabled: true,
      virtualChannels: 120,
      fineStructureCoding: true,
      fundamentalFrequencyEnhancement: true,
      harmonicPreservation: true
    },
    harmonicEnhancement: {
      harmonicCount: 8,
      fundamentalBoost: 3,
      harmonicBoost: 2,
      inharmonicityReduction: true
    },
    tempoTracking: false
  },
  jazz: {
    name: 'Jazz',
    compressionRatio: 2.5,
    inputDynamicRange: 70,
    processingStrategy: 'ACE',
    microphone: 'music',
    pitchRefinement: {
      enabled: true,
      virtualChannels: 80,
      fineStructureCoding: true,
      fundamentalFrequencyEnhancement: true,
      harmonicPreservation: true
    },
    harmonicEnhancement: {
      harmonicCount: 6,
      fundamentalBoost: 2,
      harmonicBoost: 3,
      inharmonicityReduction: false
    },
    tempoTracking: true
  },
  rock: {
    name: 'Rock/Pop',
    compressionRatio: 3.0,
    inputDynamicRange: 65,
    processingStrategy: 'HDCIS',
    microphone: 'music',
    pitchRefinement: {
      enabled: true,
      virtualChannels: 60,
      fineStructureCoding: false,
      fundamentalFrequencyEnhancement: true,
      harmonicPreservation: false
    },
    harmonicEnhancement: {
      harmonicCount: 4,
      fundamentalBoost: 4,
      harmonicBoost: 2,
      inharmonicityReduction: false
    },
    tempoTracking: true
  }
};
```

### 7.3 Music Perception Metrics

```typescript
interface MusicPerceptionAssessment {
  // Pitch perception
  pitchDiscrimination: number; // semitones JND
  melodyRecognition: number; // % correct familiar melodies
  pitchDirection: number; // % correct up/down

  // Timbre
  instrumentRecognition: number; // % correct
  timbreQuality: number; // 1-10 rating

  // Rhythm
  rhythmRecognition: number; // % correct patterns
  beatTracking: number; // % accurate tempo matching

  // Overall quality
  musicEnjoyment: number; // 1-10 rating
  naturalness: number; // 1-10 rating
}

const musicBenchmarks = {
  excellent: { pitchDiscrimination: 1.0, melodyRecognition: 80, musicEnjoyment: 8 },
  good: { pitchDiscrimination: 2.0, melodyRecognition: 65, musicEnjoyment: 7 },
  fair: { pitchDiscrimination: 3.0, melodyRecognition: 50, musicEnjoyment: 5 }
};
```

---

## 8. Bilateral Implant Synchronization

### 8.1 Bilateral Benefits

**Key Advantages:**
- Improved sound localization
- Better speech understanding in noise (binaural squelch)
- Reduced head shadow effect
- Enhanced spatial awareness
- Bilateral summation (redundancy)

**Expected Improvements:**
```
Speech in Noise: 2-5 dB SNR improvement
Localization: <20° error (vs >45° unilateral)
Quality of Life: 15-30% improvement in surveys
Bilateral Summation: 3-6 dB loudness advantage
```

### 8.2 Synchronization Protocol

```typescript
interface BilateralSync {
  leftDevice: string;
  rightDevice: string;
  syncMode: 'independent' | 'linked' | 'coordinated';
  timingAccuracy: number; // microseconds
  interauralLevelDifference: boolean; // Preserve ILD
  interauralTimeDifference: boolean; // Preserve ITD
  bilateralBeamforming: boolean;
}

function synchronizeBilateral(config: BilateralSync): SyncConfig {
  // Time synchronization
  const timeSync = synchronizeClocks(
    config.leftDevice,
    config.rightDevice,
    config.timingAccuracy
  );

  // Interaural cue preservation
  let icdPreservation: InterauralCues | undefined;
  if (config.interauralLevelDifference || config.interauralTimeDifference) {
    icdPreservation = {
      ILD: config.interauralLevelDifference,
      ITD: config.interauralTimeDifference,
      maxITD: 700, // microseconds (natural max)
      maxILD: 20 // dB (natural max)
    };
  }

  // Bilateral beamforming
  let beamforming: BeamformingConfig | undefined;
  if (config.bilateralBeamforming) {
    beamforming = {
      mode: 'adaptive',
      beamWidth: 90,
      spatialNullDepth: 15, // dB noise reduction
      updateRate: 50 // ms
    };
  }

  return {
    timeSync,
    interauralCues: icdPreservation,
    beamforming
  };
}
```

### 8.3 Sound Localization

```typescript
interface LocalizationCues {
  // Interaural Time Difference
  ITD: {
    enabled: boolean;
    range: { min: number; max: number }; // microseconds
    resolution: number; // microseconds
  };

  // Interaural Level Difference
  ILD: {
    enabled: boolean;
    range: { min: number; max: number }; // dB
    resolution: number; // dB
  };

  // Head-Related Transfer Function
  HRTF: {
    enabled: boolean;
    individualized: boolean;
    elevationCues: boolean;
  };
}

function calculateSoundLocation(
  leftSignal: AudioSignal,
  rightSignal: AudioSignal,
  cues: LocalizationCues
): SoundLocation {
  let azimuth = 0;
  let elevation = 0;

  // ITD-based localization (primary cue for low frequencies)
  if (cues.ITD.enabled) {
    const itd = calculateITD(leftSignal, rightSignal);
    azimuth = itdToAzimuth(itd); // -90° to +90°
  }

  // ILD-based localization (primary cue for high frequencies)
  if (cues.ILD.enabled) {
    const ild = calculateILD(leftSignal, rightSignal);
    const ildAzimuth = ildToAzimuth(ild);
    azimuth = (azimuth + ildAzimuth) / 2; // Combine cues
  }

  // HRTF for elevation
  if (cues.HRTF.enabled && cues.HRTF.elevationCues) {
    elevation = estimateElevation(leftSignal, rightSignal);
  }

  return { azimuth, elevation };
}
```

### 8.4 Bilateral Program Linking

```typescript
interface BilateralProgramLink {
  programSync: boolean; // Same program on both sides
  volumeSync: boolean; // Synchronized volume changes
  sensitivitySync: boolean; // Matched input sensitivity
  mixerSync: boolean; // Coordinated input mixing
  streamingMode: 'independent' | 'linked' | 'stereo';
}

function linkBilateralPrograms(
  leftMAP: DeviceMAP,
  rightMAP: DeviceMAP,
  linkConfig: BilateralProgramLink
): BilateralMAP {
  const linked: BilateralMAP = {
    left: leftMAP,
    right: rightMAP,
    sync: linkConfig
  };

  if (linkConfig.programSync) {
    // Ensure same processing strategy on both sides
    linked.right.processingStrategy = linked.left.processingStrategy;
  }

  if (linkConfig.volumeSync) {
    // Link volume controls
    linked.volumeControl = 'synchronized';
  }

  if (linkConfig.sensitivitySync) {
    // Match sensitivity settings
    linked.right.sensitivity = linked.left.sensitivity;
  }

  return linked;
}
```

---

## 9. Tinnitus Suppression Features

### 9.1 Tinnitus Mechanisms in CI Users

**Prevalence:** 60-80% of CI candidates have tinnitus pre-op
**Post-CI Outcomes:**
- 40-60% complete suppression
- 30-40% significant reduction
- 10-20% no change or worsening

### 9.2 Tinnitus Suppression Strategies

#### 9.2.1 Continuous Stimulation

```typescript
interface TinnitusStimulation {
  enabled: boolean;
  frequency: number; // Hz (matched to tinnitus pitch)
  level: number; // dB (minimum effective)
  pattern: 'continuous' | 'pulsed' | 'modulated';
  modulationFrequency?: number; // Hz (if modulated)
  electrodes: number[]; // Active electrodes
}

function configureTinnitusSuppression(
  tinnitusCharacteristics: {
    pitch: number; // Hz
    loudness: number; // dB SL
    quality: 'tonal' | 'noise' | 'pulsatile';
  },
  availableElectrodes: number
): TinnitusStimulation {
  // Match electrode to tinnitus pitch
  const targetElectrode = frequencyToElectrode(
    tinnitusCharacteristics.pitch,
    availableElectrodes
  );

  // Select stimulation pattern based on tinnitus quality
  let pattern: 'continuous' | 'pulsed' | 'modulated';
  if (tinnitusCharacteristics.quality === 'tonal') {
    pattern = 'continuous';
  } else if (tinnitusCharacteristics.quality === 'pulsatile') {
    pattern = 'pulsed';
  } else {
    pattern = 'modulated';
  }

  return {
    enabled: true,
    frequency: tinnitusCharacteristics.pitch,
    level: tinnitusCharacteristics.loudness * 0.7, // 70% of tinnitus loudness
    pattern,
    modulationFrequency: pattern === 'modulated' ? 10 : undefined,
    electrodes: [targetElectrode, targetElectrode + 1, targetElectrode - 1]
  };
}
```

#### 9.2.2 Masking Strategies

```typescript
interface TinnitusMasking {
  type: 'total' | 'partial' | 'residual_inhibition';
  noise: {
    type: 'white' | 'pink' | 'brown' | 'bandpass';
    centerFrequency?: number; // Hz (for bandpass)
    bandwidth?: number; // Hz
    level: number; // dB
  };
  schedule: {
    continuous: boolean;
    onDuration?: number; // minutes
    offDuration?: number; // minutes
  };
}

function generateTinnitusMask(config: TinnitusMasking): AudioSignal {
  let maskingSignal: AudioSignal;

  // Generate noise based on type
  switch (config.noise.type) {
    case 'white':
      maskingSignal = generateWhiteNoise(config.noise.level);
      break;
    case 'pink':
      maskingSignal = generatePinkNoise(config.noise.level);
      break;
    case 'brown':
      maskingSignal = generateBrownNoise(config.noise.level);
      break;
    case 'bandpass':
      const wideband = generateWhiteNoise(config.noise.level);
      maskingSignal = bandpassFilter(wideband, {
        low: config.noise.centerFrequency! - config.noise.bandwidth! / 2,
        high: config.noise.centerFrequency! + config.noise.bandwidth! / 2
      });
      break;
  }

  // Apply scheduling
  if (!config.schedule.continuous) {
    maskingSignal = applyOnOffSchedule(
      maskingSignal,
      config.schedule.onDuration!,
      config.schedule.offDuration!
    );
  }

  return maskingSignal;
}
```

#### 9.2.3 Notched Music Therapy

```typescript
interface NotchedTherapy {
  tinnitusFrequency: number; // Hz
  notchWidth: number; // Hz (typically 1-2 octaves)
  musicType: 'preferred' | 'classical' | 'nature';
  duration: number; // minutes per session
  sessionsPerDay: number;
}

function createNotchedMusic(
  music: AudioSignal,
  config: NotchedTherapy
): AudioSignal {
  // Create notch filter centered at tinnitus frequency
  const notchFilter = createNotchFilter(
    config.tinnitusFrequency,
    config.notchWidth
  );

  // Apply filter to music
  const notched = applyFilter(music, notchFilter);

  return notched;
}
```

### 9.3 Tinnitus Assessment

```typescript
interface TinnitusAssessment {
  // Subjective measures
  THI: number; // Tinnitus Handicap Inventory (0-100)
  VAS: number; // Visual Analog Scale (0-10)
  annoyance: number; // 0-10 scale

  // Psychoacoustic measures
  pitch: number; // Hz
  loudness: number; // dB SL
  minimumMaskingLevel: number; // dB SPL

  // Change tracking
  suppressionDegree: 'complete' | 'significant' | 'moderate' | 'minimal' | 'none';
  improvementPercent: number; // % improvement from baseline
}

function assessTinnitusResponse(
  baseline: TinnitusAssessment,
  followUp: TinnitusAssessment
): TinnitusOutcome {
  const thiChange = baseline.THI - followUp.THI;
  const vasChange = baseline.VAS - followUp.VAS;

  const improvement = (thiChange / baseline.THI) * 100;

  let suppressionDegree: TinnitusAssessment['suppressionDegree'];
  if (improvement >= 80) suppressionDegree = 'complete';
  else if (improvement >= 50) suppressionDegree = 'significant';
  else if (improvement >= 25) suppressionDegree = 'moderate';
  else if (improvement >= 10) suppressionDegree = 'minimal';
  else suppressionDegree = 'none';

  return {
    suppressionDegree,
    improvementPercent: improvement,
    thiChange,
    vasChange,
    clinicallySignificant: thiChange >= 20 // Established threshold
  };
}
```

---

## 10. Environmental Sound Classification

### 10.1 Acoustic Scene Detection

```typescript
type AcousticScene =
  | 'quiet'
  | 'speech_in_quiet'
  | 'speech_in_noise'
  | 'noise'
  | 'music'
  | 'outdoor'
  | 'traffic'
  | 'restaurant'
  | 'conference';

interface SceneClassifier {
  algorithm: 'rule_based' | 'ml_based' | 'hybrid';
  updateRate: number; // Hz (scene detection frequency)
  confidenceThreshold: number; // 0-1
  adaptationSpeed: 'slow' | 'medium' | 'fast';
}

function classifyAcousticScene(
  audioSignal: AudioSignal,
  classifier: SceneClassifier
): SceneClassification {
  // Feature extraction
  const features = extractSceneFeatures(audioSignal);

  // Classification
  let scene: AcousticScene;
  let confidence: number;

  if (classifier.algorithm === 'ml_based') {
    const mlResult = mlSceneClassification(features);
    scene = mlResult.scene;
    confidence = mlResult.confidence;
  } else if (classifier.algorithm === 'rule_based') {
    const ruleResult = ruleBasedClassification(features);
    scene = ruleResult.scene;
    confidence = ruleResult.confidence;
  } else {
    // Hybrid approach
    const mlResult = mlSceneClassification(features);
    const ruleResult = ruleBasedClassification(features);

    // Combine with confidence weighting
    if (mlResult.confidence > ruleResult.confidence) {
      scene = mlResult.scene;
      confidence = mlResult.confidence;
    } else {
      scene = ruleResult.scene;
      confidence = ruleResult.confidence;
    }
  }

  return { scene, confidence, features };
}

function extractSceneFeatures(signal: AudioSignal): SceneFeatures {
  return {
    // Energy features
    rmsLevel: calculateRMS(signal),
    peakLevel: calculatePeak(signal),
    dynamicRange: calculateDynamicRange(signal),

    // Spectral features
    spectralCentroid: calculateSpectralCentroid(signal),
    spectralRolloff: calculateSpectralRolloff(signal),
    spectralFlux: calculateSpectralFlux(signal),

    // Temporal features
    zeroCrossingRate: calculateZCR(signal),
    temporalCentroid: calculateTemporalCentroid(signal),
    modulationSpectrum: calculateModulationSpectrum(signal),

    // Speech-specific
    speechProbability: estimateSpeechProbability(signal),
    voicingRate: calculateVoicingRate(signal),

    // Environment
    noiseLevel: estimateNoiseLevel(signal),
    reverberation: estimateReverberation(signal)
  };
}
```

### 10.2 Automatic Program Selection

```typescript
interface AutomaticProgramSelection {
  enabled: boolean;
  scenes: Record<AcousticScene, ProgramConfig>;
  transitionSmoothing: number; // seconds
  userOverride: boolean;
  learningEnabled: boolean;
}

function selectProgram(
  currentScene: AcousticScene,
  aps: AutomaticProgramSelection,
  userHistory?: UserPreference[]
): ProgramConfig {
  if (!aps.enabled) {
    return aps.scenes['speech_in_quiet']; // Default
  }

  // Get recommended program for scene
  let program = aps.scenes[currentScene];

  // Apply user learning if enabled
  if (aps.learningEnabled && userHistory) {
    const userPreference = findUserPreference(currentScene, userHistory);
    if (userPreference) {
      program = mergeWithPreference(program, userPreference);
    }
  }

  return program;
}

// Example scene-to-program mapping
const defaultScenePrograms: Record<AcousticScene, ProgramConfig> = {
  quiet: {
    strategy: 'ACE',
    noiseReduction: 'off',
    directionality: 'omnidirectional',
    compression: 2.5,
    gain: 0
  },
  speech_in_quiet: {
    strategy: 'ACE',
    noiseReduction: 'low',
    directionality: 'medium',
    compression: 3.0,
    gain: 0
  },
  speech_in_noise: {
    strategy: 'HDCIS',
    noiseReduction: 'adaptive',
    directionality: 'narrow',
    compression: 4.0,
    gain: 3
  },
  noise: {
    strategy: 'ACE',
    noiseReduction: 'high',
    directionality: 'narrow',
    compression: 4.5,
    gain: -3
  },
  music: {
    strategy: 'FSP',
    noiseReduction: 'off',
    directionality: 'omnidirectional',
    compression: 2.0,
    gain: 0
  },
  outdoor: {
    strategy: 'ACE',
    noiseReduction: 'medium',
    directionality: 'wide',
    compression: 3.5,
    gain: 0
  },
  traffic: {
    strategy: 'HDCIS',
    noiseReduction: 'high',
    directionality: 'medium',
    compression: 4.0,
    gain: -3
  },
  restaurant: {
    strategy: 'ACE',
    noiseReduction: 'adaptive',
    directionality: 'narrow',
    compression: 4.0,
    gain: 2
  },
  conference: {
    strategy: 'ACE',
    noiseReduction: 'medium',
    directionality: 'wide',
    compression: 3.5,
    gain: 1
  }
};
```

### 10.3 Environmental Adaptation

```typescript
interface EnvironmentalAdaptation {
  // Automatic gain control
  agc: {
    enabled: boolean;
    targetLevel: number; // dB SPL
    attackTime: number; // ms
    releaseTime: number; // ms
  };

  // Wind noise suppression
  windSuppression: {
    enabled: boolean;
    threshold: number; // Wind level detection
    reductionStrength: number; // 0-1
  };

  // Sudden noise suppression
  transientSuppression: {
    enabled: boolean;
    threshold: number; // dB above background
    suppressionTime: number; // ms
  };

  // Echo/reverb compensation
  reverbCompensation: {
    enabled: boolean;
    estimationMethod: 'adaptive' | 'fixed';
    compensationStrength: number; // 0-1
  };
}

function adaptToEnvironment(


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
