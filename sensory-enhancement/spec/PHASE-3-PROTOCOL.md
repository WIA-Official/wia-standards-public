# WIA-AUG-004 — Phase 3: Protocol

> Overload-protection and cross-modal-mapping protocol layer. The protocol exchanges are wire-level with safety-first design — overload mitigation must reach the user within a documented latency budget.

## 8. Overload Protection

### 8.1 Overload Principles

Sensory overload occurs when input exceeds the processing capacity or safe limits of the sensory system.

```typescript
interface OverloadProtection {
  modality: SensoryModality;

  // Thresholds
  warningThreshold: number;   // 80% of max
  criticalThreshold: number;  // 95% of max
  dangerThreshold: number;    // 100% of max

  // Protection mechanisms
  autoLimiting: boolean;
  gradualReduction: boolean;
  emergencyShutoff: boolean;

  // Recovery
  recoveryTime: number;       // ms
  gradualReintroduction: boolean;
}
```

### 8.2 Overload Detection

```
Overload Risk = (Current Intensity / Max Safe Intensity) × Duration Factor

Where:
- Current Intensity: Real-time input level
- Max Safe Intensity: Calibrated safety limit
- Duration Factor: 1.0 + (exposure_time / safe_exposure_time)
```

### 8.3 Protection Mechanisms

#### 8.3.1 Intensity Limiting

```typescript
interface IntensityLimiter {
  softLimit: number;      // Begin gradual reduction
  hardLimit: number;      // Absolute maximum

  // Limiting curve
  limiterType: 'linear' | 'logarithmic' | 'exponential';
  compressionRatio: number;

  // Attack/release
  attackTime: number;     // ms to engage
  releaseTime: number;    // ms to disengage
}
```

#### 8.3.2 Adaptive Filtering

```typescript
interface AdaptiveFilter {
  modality: SensoryModality;

  // Filter parameters
  cutoffFrequency: number;
  filterOrder: number;
  adaptationSpeed: number;

  // Noise reduction
  noiseGate: boolean;
  noiseThreshold: number;

  // Signal preservation
  preserveTransients: boolean;
  preserveDynamics: boolean;
}
```

#### 8.3.3 Temporal Gating

```typescript
interface TemporalGate {
  maxExposureTime: number;   // ms continuous
  mandatoryRestPeriod: number; // ms rest
  dutyCircle: number;        // % of time active

  // Gradual engagement
  fadeIn: number;            // ms
  fadeOut: number;           // ms
}
```

### 8.4 Overload Recovery Protocol

```
1. Detect Overload
   ↓
2. Immediate Protection (Soft limit)
   ↓
3. Gradual Reduction (If continues)
   ↓
4. Emergency Cutoff (If critical)
   ↓
5. Recovery Period (Rest)
   ↓
6. Gradual Reintroduction (Slow ramp)
   ↓
7. Monitor for Recurrence
```

### 8.5 Modality-Specific Limits

#### Visual Overload Protection

```typescript
interface VisualOverloadProtection {
  // Intensity limits
  maxLuminance: 10000;        // cd/m² (retinal safety)
  maxFlickerRate: 60;         // Hz (seizure prevention)

  // UV/IR protection
  uvMaxIntensity: 0.1;        // mW/cm²
  irMaxIntensity: 1.0;        // mW/cm²

  // Exposure limits
  maxContinuousExposure: 3600000; // ms (1 hour)
  mandatoryBreak: 300000;         // ms (5 min)
}
```

#### Auditory Overload Protection

```typescript
interface AuditoryOverloadProtection {
  // Intensity limits
  maxSPL: 85;                 // dB (OSHA limit)
  peakSPL: 120;              // dB (pain threshold)

  // Frequency limits
  infrasonicLimit: 95;        // dB at 10-20 Hz
  ultrasonicLimit: 75;        // dB at 20-50 kHz

  // Exposure limits (NIOSH)
  exposureLimits: {
    85: 28800000,  // 8 hours at 85 dB
    88: 14400000,  // 4 hours at 88 dB
    91: 7200000,   // 2 hours at 91 dB
    94: 3600000,   // 1 hour at 94 dB
    97: 1800000,   // 30 min at 97 dB
    100: 900000    // 15 min at 100 dB
  };
}
```

#### Tactile Overload Protection

```typescript
interface TactileOverloadProtection {
  // Pressure limits
  maxPressure: 500;           // kPa (pain threshold)
  sustainedPressure: 200;     // kPa (continuous)

  // Vibration limits
  maxVibration: 300;          // Hz
  maxIntensity: 5.0;          // m/s² (ISO 5349)

  // Temperature limits
  maxTemperature: 45;         // °C (burn prevention)
  minTemperature: 10;         // °C (cold injury prevention)

  // Exposure limits
  vibrationExposure: 14400000; // ms (4 hours daily)
}
```

### 8.6 Cognitive Load Monitoring

```typescript
interface CognitiveLoadMonitor {
  // Load measurement
  currentLoad: number;        // 0-100%
  maxSustainableLoad: 70;     // %

  // Load sources
  sensoryInputLoad: number;
  processingLoad: number;
  integrationLoad: number;

  // Protection
  reduceComplexity: boolean;
  prioritizeInputs: boolean;
  temporaryDisable: SensoryModality[];
}
```

---


## 9. Cross-Modal Mapping

### 9.1 Mapping Principles

Cross-modal mapping translates information from one sensory modality to another.

```typescript
interface CrossModalMap {
  source: SensoryModality;
  target: SensoryModality;
  mappingFunction: MappingFunction;
  bidirectional: boolean;
  fidelity: number;          // 0-1
}

interface MappingFunction {
  type: 'linear' | 'logarithmic' | 'exponential' | 'custom';
  parameters: Record<string, number>;
  lut?: LookupTable;         // Optional lookup table
}
```

### 9.2 Common Cross-Modal Mappings

#### 9.2.1 Visual ↔ Auditory

```typescript
interface VisualAuditoryMap {
  // Visual → Auditory
  visualToAuditory: {
    brightness: { target: 'pitch', range: [200, 2000] },    // Hz
    hue: { target: 'timbre', values: ['sine', 'square', 'triangle'] },
    saturation: { target: 'harmonics', range: [0, 10] },
    position_x: { target: 'pan', range: [-1, 1] },
    position_y: { target: 'volume', range: [40, 80] },      // dB
    motion: { target: 'tempo', range: [60, 180] }           // BPM
  };

  // Auditory → Visual
  auditoryToVisual: {
    pitch: { target: 'hue', range: [0, 360] },              // degrees
    loudness: { target: 'brightness', range: [0, 100] },    // %
    timbre: { target: 'saturation', range: [0, 100] },
    pan: { target: 'position_x', range: [-1, 1] },
    tempo: { target: 'motion_speed', range: [0, 10] }
  };
}
```

#### 9.2.2 Visual ↔ Tactile

```typescript
interface VisualTactileMap {
  // Visual → Tactile
  visualToTactile: {
    brightness: { target: 'vibration_intensity', range: [0, 5] },    // N
    edges: { target: 'pulse_sharpness', range: [1, 10] },
    texture: { target: 'vibration_frequency', range: [50, 300] },    // Hz
    depth: { target: 'pressure', range: [0, 100] },                  // kPa
    motion: { target: 'vibration_sweep', range: [20, 200] }          // Hz/s
  };

  // Tactile → Visual
  tactileToVisual: {
    pressure: { target: 'brightness', range: [0, 100] },
    texture: { target: 'pattern', values: ['smooth', 'rough', 'ridged'] },
    temperature: { target: 'hue', range: [240, 0] },                 // Blue→Red
    vibration: { target: 'motion_blur', range: [0, 10] }
  };
}
```

#### 9.2.3 Auditory ↔ Tactile

```typescript
interface AuditoryTactileMap {
  // Auditory → Tactile
  auditoryToTactile: {
    frequency: { target: 'vibration_frequency', range: [20, 500] },  // Hz
    amplitude: { target: 'vibration_intensity', range: [0, 5] },     // N
    rhythm: { target: 'pulse_pattern', type: 'timing_array' },
    direction: { target: 'actuator_position', range: [0, 360] }      // degrees
  };

  // Tactile → Auditory
  tactileToAuditory: {
    vibration_frequency: { target: 'pitch', range: [200, 2000] },    // Hz
    pressure: { target: 'loudness', range: [40, 80] },               // dB
    texture: { target: 'noise_color', values: ['white', 'pink', 'brown'] }
  };
}
```

### 9.3 Mapping Quality Metrics

```
Mapping Fidelity = (Preserved Information / Source Information) × Perceptual Similarity

Where:
- Preserved Information: Bits successfully transferred
- Source Information: Total bits in source
- Perceptual Similarity: Subjective match (0-1)
```

### 9.4 Synesthetic Mappings

Synesthesia-inspired mappings for enhanced perception:

```typescript
interface SynestheticMapping {
  // Sound → Color (Chromesthesia)
  chromesthesia: {
    C: { hue: 0, saturation: 100, lightness: 50 },      // Red
    D: { hue: 30, saturation: 100, lightness: 50 },     // Orange
    E: { hue: 60, saturation: 100, lightness: 50 },     // Yellow
    F: { hue: 120, saturation: 100, lightness: 50 },    // Green
    G: { hue: 180, saturation: 100, lightness: 50 },    // Cyan
    A: { hue: 240, saturation: 100, lightness: 50 },    // Blue
    B: { hue: 300, saturation: 100, lightness: 50 }     // Magenta
  };

  // Number → Color (Numerical synesthesia)
  numberColor: Record<number, { hue: number; saturation: number }>;

  // Texture → Sound
  textureSound: {
    smooth: { waveform: 'sine', frequency: 440 },
    rough: { waveform: 'sawtooth', frequency: 220 },
    bumpy: { waveform: 'square', frequency: 330 }
  };
}
```

### 9.5 Adaptive Mapping

```typescript
interface AdaptiveMapping {
  // Learning parameters
  learningEnabled: boolean;
  learningRate: number;      // 0-1

  // User feedback
  feedbackLoop: boolean;
  userCorrections: Correction[];

  // Optimization
  optimizationGoal: 'fidelity' | 'clarity' | 'efficiency';
  adaptationSpeed: number;   // 0-1

  // Personalization
  userPreferences: MappingPreferences;
  contextAware: boolean;
}
```

---



## A.1 Overload-protection protocol

Sensory enhancement carries a real risk of perceptual overload:
too much auditory input causing tinnitus, too much visual
input causing fatigue, too much haptic input causing motor-control
disruption. The overload-protection protocol monitors per-modality
intensity and triggers attenuation when limits are exceeded.

The protocol envelopes carry the per-modality intensity history,
the trigger-threshold envelope, and the attenuation action with a
timestamp accurate to the millisecond. The audit log is preserved
for medical review when adverse events are reported.

## A.2 Cross-modal-mapping protocol

Cross-modal mapping translates signals between modalities (visual
edge → auditory pitch, distance → vibration intensity). The
mapping function appears as a signed envelope; the user's
acceptance of the mapping is signed by the user before activation.

## A.3 Replay defence and audit

Standard 96-bit nonce + 300-second skew window + 600-second
seen-nonce cache. Audit envelopes are written to an append-only
log with retention sized to medical-record requirements (typically
10 years).

## A.4 Operator failover

When a sensory-enhancement controller fails over from primary to
standby, the standby MUST: reload the persistent seen-nonce cache,
reload the user's calibration profile from the last signed
profile envelope, and resume protection within a documented
latency budget (typically 100 ms to avoid noticeable disruption).


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/sensory-enhancement/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-sensory-enhancement-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/sensory-enhancement-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/sensory-enhancement.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.
