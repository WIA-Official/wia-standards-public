# WIA Haptic Primitives Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines the fundamental building blocks (primitives) for haptic feedback in assistive technology applications. These primitives form the basis of the WIA Haptic Language, enabling consistent and learnable tactile communication for users with visual impairments.

## 2. Design Principles

### 2.1 Accessibility First
- All patterns must be distinguishable without visual reference
- Frequency ranges optimized for human tactile perception
- Patterns designed for quick learning and retention

### 2.2 Hardware Agnostic
- Primitives abstract over device capabilities
- Graceful degradation for limited hardware
- Consistent experience across device types

### 2.3 Energy Efficient
- Optimize for mobile/wearable battery constraints
- Short, effective patterns preferred over long continuous vibrations

## 3. Waveform Types

### 3.1 Basic Waveforms

| Waveform | Description | Tactile Perception | Use Cases |
|----------|-------------|-------------------|-----------|
| `sine` | Smooth, continuous oscillation | Gentle, fluid sensation | Ambient feedback, distance indication |
| `square` | Sharp on/off transitions | Crisp, definitive pulses | Confirmations, alerts |
| `triangle` | Linear rise and fall | Moderate sharpness | Directional cues |
| `sawtooth` | Gradual rise, sharp fall | Ratcheting sensation | Progress indication |
| `noise` | Random variations | Rough, textured feeling | Warnings, obstacles |

### 3.2 Waveform Parameters

```typescript
interface WaveformConfig {
  type: 'sine' | 'square' | 'triangle' | 'sawtooth' | 'noise';

  // Duty cycle for square waves (0.0-1.0)
  dutyCycle?: number;

  // Smoothing factor for transitions (0.0-1.0)
  smoothing?: number;

  // Noise characteristics
  noiseType?: 'white' | 'pink' | 'brown';
}
```

## 4. ADSR Envelope

The Attack-Decay-Sustain-Release (ADSR) envelope shapes the amplitude of haptic signals over time.

### 4.1 Envelope Structure

```typescript
interface HapticEnvelope {
  // Time to reach peak intensity (ms)
  attack: number;    // Range: 0-500ms, Default: 10ms

  // Time to fall to sustain level (ms)
  decay: number;     // Range: 0-500ms, Default: 50ms

  // Sustained intensity level
  sustain: number;   // Range: 0.0-1.0, Default: 0.7

  // Time to fade to zero (ms)
  release: number;   // Range: 0-500ms, Default: 100ms
}
```

### 4.2 Preset Envelopes

| Preset | Attack | Decay | Sustain | Release | Character |
|--------|--------|-------|---------|---------|-----------|
| `sharp` | 5 | 20 | 0.0 | 30 | Quick tap |
| `punch` | 10 | 50 | 0.3 | 50 | Impact feel |
| `smooth` | 100 | 100 | 0.7 | 150 | Gentle wave |
| `pulse` | 5 | 0 | 1.0 | 5 | On/off |
| `swell` | 200 | 50 | 0.8 | 200 | Growing intensity |
| `fade` | 50 | 200 | 0.5 | 300 | Gradual decay |

### 4.3 Envelope Visualization

```
Intensity
   ^
1.0|     /\
   |    /  \____
0.7|   /        \____
   |  /              \
0.0|_/________________\___> Time
   |A | D |  S   | R  |
```

## 5. Frequency Specification

### 5.1 Human Tactile Perception Range

The human skin is most sensitive to vibrations in specific frequency ranges:

| Receptor Type | Frequency Range | Location | Sensitivity |
|---------------|-----------------|----------|-------------|
| Meissner corpuscles | 10-50 Hz | Fingertips, palms | High |
| Pacinian corpuscles | 40-400 Hz | Deep tissue | Moderate |
| Merkel discs | 0.4-3 Hz | Superficial skin | Low freq |
| Ruffini endings | 0.4-3 Hz | Deep skin | Stretch |

### 5.2 WIA Frequency Bands

```typescript
enum FrequencyBand {
  // Low rumble - visceral, warning
  VERY_LOW = 'very_low',      // 1-30 Hz

  // Standard haptic range - most distinguishable
  LOW = 'low',                // 30-80 Hz

  // Optimal perception range
  MID = 'mid',                // 80-150 Hz

  // Alert, attention-getting
  HIGH = 'high',              // 150-250 Hz

  // Sharp, urgent
  VERY_HIGH = 'very_high',    // 250-300 Hz
}

const FREQUENCY_RANGES: Record<FrequencyBand, [number, number]> = {
  very_low: [1, 30],
  low: [30, 80],
  mid: [80, 150],
  high: [150, 250],
  very_high: [250, 300],
};
```

### 5.3 Semantic Frequency Mapping

| Meaning | Frequency Band | Rationale |
|---------|---------------|-----------|
| Safety/Clear | Low (30-80 Hz) | Calm, reassuring |
| Information | Mid (80-150 Hz) | Neutral, clear |
| Attention | High (150-250 Hz) | Alerting |
| Danger/Urgent | Very High (250-300 Hz) | Alarming |

## 6. Intensity Specification

### 6.1 Intensity Levels

```typescript
interface IntensityConfig {
  // Base intensity (0.0-1.0)
  level: number;

  // Optional modulation
  modulation?: {
    type: 'none' | 'pulse' | 'wave' | 'random';
    rate: number;      // Hz
    depth: number;     // 0.0-1.0
  };
}
```

### 6.2 Standard Intensity Presets

| Level | Value | Description | Use Case |
|-------|-------|-------------|----------|
| `subtle` | 0.2 | Barely perceptible | Background, ambient |
| `light` | 0.4 | Gentle feedback | Confirmations |
| `medium` | 0.6 | Standard feedback | Navigation |
| `strong` | 0.8 | Definite feedback | Alerts |
| `maximum` | 1.0 | Full intensity | Emergencies |

### 6.3 Accessibility Considerations

- Users may have varying sensitivity thresholds
- Support user-configurable intensity scaling
- Provide intensity calibration during onboarding

```typescript
interface UserIntensityPreferences {
  // Global scaling factor (0.5-2.0)
  globalScale: number;

  // Per-category adjustments
  categoryScales: Record<HapticCategory, number>;

  // Minimum perceptible intensity for this user
  perceptionThreshold: number;
}
```

## 7. Complete Primitive Definition

### 7.1 Full Interface

```typescript
interface HapticPrimitive {
  // Unique identifier
  id: string;

  // Human-readable name
  name: string;

  // Basic waveform configuration
  waveform: WaveformConfig;

  // Amplitude envelope
  envelope: HapticEnvelope;

  // Vibration frequency in Hz (1-300)
  frequency: number;

  // Base intensity (0.0-1.0)
  intensity: number;

  // Duration in milliseconds
  duration: number;

  // Optional metadata
  metadata?: {
    category?: HapticCategory;
    description?: string;
    learnability?: 'easy' | 'moderate' | 'complex';
  };
}
```

### 7.2 Example Primitives

```typescript
const PRIMITIVE_EXAMPLES: HapticPrimitive[] = [
  {
    id: 'tick',
    name: 'Tick',
    waveform: { type: 'square', dutyCycle: 0.5 },
    envelope: { attack: 5, decay: 10, sustain: 0, release: 5 },
    frequency: 150,
    intensity: 0.6,
    duration: 20,
    metadata: {
      category: 'confirmation',
      description: 'Quick acknowledgment tap',
      learnability: 'easy'
    }
  },
  {
    id: 'warning_pulse',
    name: 'Warning Pulse',
    waveform: { type: 'sine' },
    envelope: { attack: 50, decay: 100, sustain: 0.5, release: 100 },
    frequency: 200,
    intensity: 0.8,
    duration: 300,
    metadata: {
      category: 'notification',
      description: 'Attention-getting warning',
      learnability: 'easy'
    }
  },
  {
    id: 'obstacle_texture',
    name: 'Obstacle Texture',
    waveform: { type: 'noise', noiseType: 'pink' },
    envelope: { attack: 20, decay: 50, sustain: 0.7, release: 50 },
    frequency: 100,
    intensity: 0.7,
    duration: 200,
    metadata: {
      category: 'navigation',
      description: 'Indicates nearby obstacle',
      learnability: 'moderate'
    }
  }
];
```

## 8. Pattern Composition

### 8.1 Sequence Definition

Multiple primitives can be combined into sequences:

```typescript
interface HapticSequence {
  id: string;
  name: string;

  // Ordered list of primitives with timing
  steps: Array<{
    primitiveId: string;
    delayBefore: number;  // ms before this step
    repeatCount?: number;
    repeatDelay?: number;
  }>;

  // Total sequence duration (calculated)
  totalDuration: number;
}
```

### 8.2 Example Sequences

```typescript
const SEQUENCE_EXAMPLES: HapticSequence[] = [
  {
    id: 'success',
    name: 'Success Confirmation',
    steps: [
      { primitiveId: 'tick', delayBefore: 0 },
      { primitiveId: 'tick', delayBefore: 100 },
    ],
    totalDuration: 120
  },
  {
    id: 'urgent_alert',
    name: 'Urgent Alert',
    steps: [
      { primitiveId: 'warning_pulse', delayBefore: 0, repeatCount: 3, repeatDelay: 150 },
    ],
    totalDuration: 1200
  }
];
```

## 9. Platform Compatibility

### 9.1 Apple Core Haptics Mapping

| WIA Primitive | Core Haptics Equivalent |
|---------------|------------------------|
| Waveform types | CHHapticEvent with custom curves |
| ADSR envelope | CHHapticParameterCurve |
| Frequency | CHHapticEventParameter.hapticSharpness |
| Intensity | CHHapticEventParameter.hapticIntensity |

### 9.2 Android Vibration API Mapping

| WIA Primitive | Android Equivalent |
|---------------|-------------------|
| Simple patterns | VibrationEffect.createWaveform() |
| Primitives | VibrationEffect.Composition |
| Envelope | Amplitude array with timing |

### 9.3 Web Vibration API

| WIA Primitive | Web API Equivalent |
|---------------|-------------------|
| Duration | navigator.vibrate(duration) |
| Sequences | navigator.vibrate([d1, p1, d2, p2, ...]) |

Note: Web Vibration API has limited capability; graceful degradation required.

## 10. References

### 10.1 Research Papers
- Verrillo, R.T. (1992). "Vibration sensation in humans"
- Gescheider, G.A. (2002). "The psychophysics of tactile perception"
- Cholewiak, R.W. & Collins, A.A. (2003). "Vibrotactile localization on the arm"

### 10.2 Industry Standards
- Apple Human Interface Guidelines: Haptics
- Android Haptic Design Guidelines
- IEEE P2861: Haptics Taxonomy and Definitions (Draft)

---

## Appendix A: Quick Reference Card

```
╔═══════════════════════════════════════════════════════════════╗
║                    WIA HAPTIC PRIMITIVES                      ║
╠═══════════════════════════════════════════════════════════════╣
║ WAVEFORMS        │ ENVELOPES       │ FREQUENCIES              ║
║ ─────────────────│─────────────────│─────────────────────────║
║ sine    ∿∿∿     │ sharp   ╱╲      │ very_low   1-30 Hz      ║
║ square  ▃▃▃     │ punch   ╱╲_     │ low       30-80 Hz      ║
║ triangle ╱╲╱╲   │ smooth ╱──╲     │ mid      80-150 Hz      ║
║ sawtooth ╱│╱│   │ pulse  ▃▃▃      │ high    150-250 Hz      ║
║ noise   ░░░░    │ swell ╱──────   │ very_high 250-300 Hz    ║
╠═══════════════════════════════════════════════════════════════╣
║ INTENSITY LEVELS                                              ║
║ subtle(0.2) < light(0.4) < medium(0.6) < strong(0.8) < max(1) ║
╚═══════════════════════════════════════════════════════════════╝
```
