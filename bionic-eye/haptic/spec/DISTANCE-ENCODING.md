# WIA Haptic Distance Encoding Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines how distance information is encoded into haptic patterns. Distance encoding enables users to perceive proximity to objects, destinations, or hazards through tactile feedback without visual confirmation.

## 2. Design Principles

### 2.1 Intuitive Perception
- Closer = More urgent/intense sensation
- Farther = Gentler/less frequent sensation
- Natural mapping to human threat response

### 2.2 Distinguishable Ranges
- Clear differentiation between distance zones
- Avoid ambiguous middle zones
- Learnable distance thresholds

### 2.3 Energy Efficiency
- Minimal vibration for safe distances
- Intensive feedback only when necessary
- Battery-conscious design

## 3. Distance Zones

### 3.1 Standard Zone Definition

```
Distance Zones (meters)
─────────────────────────────────────────────────────────►
│ CRITICAL │  NEAR   │  MEDIUM  │    FAR    │   SAFE    │
│  0-0.5m  │ 0.5-2m  │   2-5m   │   5-10m   │   >10m    │
│ ████████ │ ▓▓▓▓▓▓  │ ▒▒▒▒▒▒▒▒ │ ░░░░░░░░░ │           │
│ URGENT!  │ WARNING │ CAUTION  │   INFO    │   CLEAR   │
```

```typescript
interface DistanceZone {
  name: string;
  minDistance: number;  // meters
  maxDistance: number;  // meters
  priority: 'critical' | 'high' | 'medium' | 'low' | 'none';
  color?: string;       // for visualization
}

const STANDARD_ZONES: DistanceZone[] = [
  { name: 'critical', minDistance: 0,    maxDistance: 0.5,  priority: 'critical' },
  { name: 'near',     minDistance: 0.5,  maxDistance: 2,    priority: 'high' },
  { name: 'medium',   minDistance: 2,    maxDistance: 5,    priority: 'medium' },
  { name: 'far',      minDistance: 5,    maxDistance: 10,   priority: 'low' },
  { name: 'safe',     minDistance: 10,   maxDistance: Infinity, priority: 'none' },
];
```

### 3.2 Customizable Zones

```typescript
interface ZoneConfiguration {
  // Scale factor for all distances
  scaleFactor: number;

  // Custom zone boundaries
  customZones?: DistanceZone[];

  // Context-specific adjustments
  context?: 'indoor' | 'outdoor' | 'transit' | 'crowd';
}

// Indoor navigation (smaller scale)
const INDOOR_ZONES = STANDARD_ZONES.map(z => ({
  ...z,
  minDistance: z.minDistance * 0.5,
  maxDistance: z.maxDistance * 0.5,
}));

// Outdoor navigation (larger scale)
const OUTDOOR_ZONES = STANDARD_ZONES.map(z => ({
  ...z,
  minDistance: z.minDistance * 2,
  maxDistance: z.maxDistance * 2,
}));
```

## 4. Encoding Methods

### 4.1 Frequency-Based Encoding

Map distance to vibration pulse rate:

```typescript
interface FrequencyEncoding {
  method: 'frequency';

  // Pulse rate mapping (pulses per second)
  minPulseRate: number;   // at max distance (e.g., 0.5 Hz)
  maxPulseRate: number;   // at zero distance (e.g., 20 Hz)

  // Mapping curve
  curve: 'linear' | 'exponential' | 'logarithmic';
}

function encodeDistanceByFrequency(
  distance: number,
  maxRange: number,
  config: FrequencyEncoding
): number {
  const normalized = Math.min(distance / maxRange, 1);

  let pulseRate: number;
  switch (config.curve) {
    case 'linear':
      pulseRate = config.maxPulseRate -
        (normalized * (config.maxPulseRate - config.minPulseRate));
      break;

    case 'exponential':
      // More sensitive at close range
      pulseRate = config.maxPulseRate *
        Math.pow(config.minPulseRate / config.maxPulseRate, normalized);
      break;

    case 'logarithmic':
      // More sensitive at far range
      const logNorm = Math.log(1 + normalized * 9) / Math.log(10);
      pulseRate = config.maxPulseRate -
        (logNorm * (config.maxPulseRate - config.minPulseRate));
      break;
  }

  return pulseRate;
}
```

#### Frequency Curve Visualization

```
Pulse Rate (Hz)
     │
  20 │████                                    ← Critical (0m)
     │    ████
  15 │        ████                            Exponential
     │            ████                        curve
  10 │                ████
     │                    ████
   5 │                        ████
     │                            ████████
   2 │                                    ████
     │                                        ████
 0.5 │────────────────────────────────────────────█ ← Safe (10m)
     └────────────────────────────────────────────────►
     0    1    2    3    4    5    6    7    8    9    10 m
```

### 4.2 Intensity-Based Encoding

Map distance to vibration strength:

```typescript
interface IntensityEncoding {
  method: 'intensity';

  // Intensity mapping (0.0 - 1.0)
  minIntensity: number;   // at max distance (e.g., 0.1)
  maxIntensity: number;   // at zero distance (e.g., 1.0)

  // Threshold below which no vibration
  perceptionThreshold: number;

  // Mapping curve
  curve: 'linear' | 'quadratic' | 'inverse_square';
}

function encodeDistanceByIntensity(
  distance: number,
  maxRange: number,
  config: IntensityEncoding
): number {
  if (distance >= maxRange) return 0;

  const normalized = distance / maxRange;

  let intensity: number;
  switch (config.curve) {
    case 'linear':
      intensity = config.maxIntensity * (1 - normalized);
      break;

    case 'quadratic':
      intensity = config.maxIntensity * Math.pow(1 - normalized, 2);
      break;

    case 'inverse_square':
      // Mimics natural physical falloff
      const safeDistance = Math.max(distance, 0.1);
      intensity = config.maxIntensity / (safeDistance * safeDistance);
      intensity = Math.min(intensity, config.maxIntensity);
      break;
  }

  // Apply threshold
  if (intensity < config.perceptionThreshold) {
    return 0;
  }

  return Math.max(config.minIntensity, intensity);
}
```

### 4.3 Rhythm-Based Encoding

Map distance to pattern rhythm:

```typescript
interface RhythmEncoding {
  method: 'rhythm';

  // Pattern definitions for each zone
  patterns: {
    critical: RhythmPattern;  // Continuous or very rapid
    near: RhythmPattern;      // Fast rhythm
    medium: RhythmPattern;    // Moderate rhythm
    far: RhythmPattern;       // Slow rhythm
    safe: RhythmPattern;      // Silent or occasional
  };
}

interface RhythmPattern {
  // On/off durations in ms
  onDuration: number;
  offDuration: number;

  // Number of pulses in pattern
  pulseCount: number;

  // Gap between pattern repeats
  repeatGap: number;

  // Total pattern period
  period: number;
}

const RHYTHM_PATTERNS: Record<string, RhythmPattern> = {
  critical: {
    onDuration: 100,
    offDuration: 50,
    pulseCount: 1,
    repeatGap: 0,
    period: 150,  // Continuous rapid
  },
  near: {
    onDuration: 80,
    offDuration: 120,
    pulseCount: 2,
    repeatGap: 200,
    period: 600,
  },
  medium: {
    onDuration: 60,
    offDuration: 200,
    pulseCount: 1,
    repeatGap: 740,
    period: 1000,
  },
  far: {
    onDuration: 50,
    offDuration: 0,
    pulseCount: 1,
    repeatGap: 1950,
    period: 2000,
  },
  safe: {
    onDuration: 0,
    offDuration: 0,
    pulseCount: 0,
    repeatGap: 0,
    period: 0,  // Silent
  },
};
```

### 4.4 Combined Encoding (Recommended)

Combine multiple parameters for richer information:

```typescript
interface CombinedEncoding {
  method: 'combined';

  // Which parameters to modulate
  modulateFrequency: boolean;
  modulateIntensity: boolean;
  modulateWaveform: boolean;

  // Relative weights
  frequencyWeight: number;
  intensityWeight: number;
}

interface DistanceHapticResult {
  pulseRate: number;      // Hz
  intensity: number;      // 0-1
  waveform: WaveformType;
  frequency: number;      // Vibration Hz
}

function encodeCombined(
  distance: number,
  maxRange: number,
  config: CombinedEncoding
): DistanceHapticResult {
  const normalized = Math.min(distance / maxRange, 1);
  const proximity = 1 - normalized;  // 0 = far, 1 = close

  return {
    // Pulse rate: 0.5 Hz (far) to 15 Hz (close)
    pulseRate: 0.5 + proximity * 14.5,

    // Intensity: 0.2 (far) to 1.0 (close)
    intensity: 0.2 + proximity * 0.8,

    // Waveform: sine (far) to square (close)
    waveform: proximity > 0.7 ? 'square' : proximity > 0.3 ? 'triangle' : 'sine',

    // Vibration frequency: 60 Hz (far) to 200 Hz (close)
    frequency: 60 + proximity * 140,
  };
}
```

## 5. Temporal Patterns

### 5.1 Approaching Pattern

Object getting closer over time:

```typescript
function createApproachingPattern(
  startDistance: number,
  endDistance: number,
  duration: number,
  encoder: DistanceEncoder
): HapticSequence {
  const steps = 10;
  const stepDuration = duration / steps;
  const distanceStep = (startDistance - endDistance) / steps;

  return {
    steps: Array(steps).fill(null).map((_, i) => {
      const distance = startDistance - (i * distanceStep);
      const encoded = encoder.encode(distance);

      return {
        time: i * stepDuration,
        duration: stepDuration,
        ...encoded,
      };
    }),
  };
}
```

### 5.2 Receding Pattern

Object moving away:

```typescript
function createRecedingPattern(
  startDistance: number,
  endDistance: number,
  duration: number,
  encoder: DistanceEncoder
): HapticSequence {
  // Inverse of approaching
  return createApproachingPattern(endDistance, startDistance, duration, encoder);
}
```

### 5.3 Proximity Alert Escalation

Escalating urgency as obstacle remains close:

```typescript
interface EscalationPattern {
  initialIntensity: number;
  escalationRate: number;      // intensity increase per second
  maxIntensity: number;
  escalationInterval: number;  // ms between escalations
}

function createEscalatingAlert(
  distance: number,
  config: EscalationPattern
): HapticSequence {
  if (distance > 2) return { steps: [] };  // No escalation needed

  const basePulseRate = 5 + (2 - distance) * 5;  // 5-15 Hz
  const steps = [];
  let intensity = config.initialIntensity;
  let time = 0;

  while (time < 5000 && intensity <= config.maxIntensity) {
    steps.push({
      time,
      pulseRate: basePulseRate,
      intensity,
      duration: config.escalationInterval,
    });

    intensity += config.escalationRate * (config.escalationInterval / 1000);
    time += config.escalationInterval;
  }

  return { steps };
}
```

## 6. Multi-Object Distance

### 6.1 Nearest Object Priority

Focus on closest hazard:

```typescript
interface ObjectDistance {
  id: string;
  distance: number;
  direction: number;
  type: ObstacleType;
}

function encodeNearestObject(
  objects: ObjectDistance[],
  encoder: DistanceEncoder
): HapticPattern {
  if (objects.length === 0) return null;

  // Sort by distance, prioritize by type
  const prioritized = objects.sort((a, b) => {
    const priorityDiff = getTypePriority(b.type) - getTypePriority(a.type);
    if (Math.abs(priorityDiff) > 0.5) return priorityDiff;
    return a.distance - b.distance;
  });

  const nearest = prioritized[0];
  return encoder.encode(nearest.distance);
}
```

### 6.2 Multi-Object Time Division

Show multiple objects in sequence:

```typescript
function encodeMultipleObjects(
  objects: ObjectDistance[],
  encoder: DistanceEncoder,
  cycleDuration: number = 1000  // ms per full cycle
): HapticSequence {
  const slotDuration = cycleDuration / objects.length;
  const gapDuration = 50;  // ms gap between objects

  return {
    steps: objects.map((obj, i) => ({
      time: i * slotDuration,
      duration: slotDuration - gapDuration,
      ...encoder.encode(obj.distance),
      // Include direction hint
      direction: obj.direction,
    })),
    loop: true,
  };
}
```

### 6.3 Layered Distance (Front/Back)

Separate channels for different directions:

```typescript
interface LayeredDistanceEncoding {
  front: DistanceHapticResult | null;
  back: DistanceHapticResult | null;
  left: DistanceHapticResult | null;
  right: DistanceHapticResult | null;
}

function encodeLayeredDistance(
  objects: ObjectDistance[],
  encoder: DistanceEncoder
): LayeredDistanceEncoding {
  const layers: LayeredDistanceEncoding = {
    front: null, back: null, left: null, right: null,
  };

  // Group by quadrant, take nearest in each
  const quadrants = {
    front: objects.filter(o => o.direction >= 315 || o.direction < 45),
    right: objects.filter(o => o.direction >= 45 && o.direction < 135),
    back: objects.filter(o => o.direction >= 135 && o.direction < 225),
    left: objects.filter(o => o.direction >= 225 && o.direction < 315),
  };

  for (const [key, objs] of Object.entries(quadrants)) {
    if (objs.length > 0) {
      const nearest = objs.reduce((a, b) => a.distance < b.distance ? a : b);
      layers[key as keyof LayeredDistanceEncoding] = encoder.encode(nearest.distance);
    }
  }

  return layers;
}
```

## 7. Obstacle Type Modifiers

### 7.1 Type-Specific Encoding

Different objects warrant different urgency:

```typescript
type ObstacleType =
  | 'wall'      // Static, predictable
  | 'furniture' // Static, may be complex
  | 'person'    // Moving, unpredictable
  | 'vehicle'   // Fast moving, dangerous
  | 'stairs'    // Change in elevation
  | 'drop'      // Dangerous drop-off
  | 'door'      // Opportunity, not obstacle
  | 'unknown';  // Unclassified

const TYPE_MODIFIERS: Record<ObstacleType, TypeModifier> = {
  wall: {
    intensityMultiplier: 1.0,
    waveformOverride: null,
    zoneShift: 0,
  },
  person: {
    intensityMultiplier: 1.2,
    waveformOverride: null,
    zoneShift: 0.5,  // Treat as closer
  },
  vehicle: {
    intensityMultiplier: 1.5,
    waveformOverride: 'square',  // Sharp, urgent
    zoneShift: 1.0,  // Much closer perceived
  },
  stairs: {
    intensityMultiplier: 1.3,
    waveformOverride: 'sawtooth',  // Directional texture
    zoneShift: 0,
  },
  drop: {
    intensityMultiplier: 2.0,  // Maximum urgency
    waveformOverride: 'square',
    zoneShift: 1.5,
  },
  door: {
    intensityMultiplier: 0.5,  // Reduced urgency
    waveformOverride: null,
    zoneShift: -1.0,  // Treat as farther
  },
  furniture: {
    intensityMultiplier: 1.0,
    waveformOverride: null,
    zoneShift: 0,
  },
  unknown: {
    intensityMultiplier: 1.1,
    waveformOverride: 'noise',  // Uncertainty
    zoneShift: 0.3,
  },
};
```

### 7.2 Height-Modified Distance

Ground-level vs head-level obstacles:

```typescript
interface HeightAwareDistance {
  horizontalDistance: number;
  height: number;  // Relative to user (0 = ground, 1.7 = head level)
  userHeight: number;
}

function encodeHeightAwareDistance(
  obj: HeightAwareDistance,
  encoder: DistanceEncoder
): DistanceHapticResult {
  const baseEncoding = encoder.encode(obj.horizontalDistance);

  // Obstacles at head level are more urgent
  const headLevelRange = [obj.userHeight - 0.3, obj.userHeight + 0.3];
  const isHeadLevel = obj.height >= headLevelRange[0] && obj.height <= headLevelRange[1];

  // Ground level obstacles (tripping hazard)
  const isGroundLevel = obj.height < 0.3;

  // Overhead obstacles
  const isOverhead = obj.height > obj.userHeight + 0.1;

  if (isHeadLevel) {
    return {
      ...baseEncoding,
      intensity: Math.min(1.0, baseEncoding.intensity * 1.3),
      frequency: 200,  // Higher frequency = head level
    };
  }

  if (isGroundLevel) {
    return {
      ...baseEncoding,
      intensity: Math.min(1.0, baseEncoding.intensity * 1.1),
      frequency: 80,  // Lower frequency = ground level
    };
  }

  if (isOverhead) {
    return {
      ...baseEncoding,
      frequency: 250,  // Highest = overhead
    };
  }

  return baseEncoding;
}
```

## 8. Perceptual Considerations

### 8.1 Just Noticeable Difference (JND)

Minimum detectable change in distance encoding:

```typescript
const PERCEPTUAL_CONSTANTS = {
  // Weber fraction for intensity
  intensityJND: 0.1,  // 10% change detectable

  // Weber fraction for frequency
  frequencyJND: 0.05, // 5% change detectable

  // Minimum distinguishable pulse interval
  minPulseInterval: 100,  // ms

  // Maximum meaningful pulse rate
  maxPulseRate: 20,  // Hz (above this = continuous)
};

function quantizeToJND(
  value: number,
  jnd: number,
  min: number,
  max: number
): number {
  const range = max - min;
  const step = range * jnd;
  const steps = Math.round((value - min) / step);
  return min + steps * step;
}
```

### 8.2 Adaptation Prevention

Vary encoding to prevent sensory adaptation:

```typescript
interface AdaptationPrevention {
  // Slight random variation
  intensityJitter: number;  // ±% variation

  // Subtle frequency drift
  frequencyDrift: number;   // Hz variation

  // Pattern variation
  patternVariation: boolean;
}

function applyAdaptationPrevention(
  encoding: DistanceHapticResult,
  config: AdaptationPrevention
): DistanceHapticResult {
  const intensityVar = (Math.random() - 0.5) * 2 * config.intensityJitter;
  const freqVar = (Math.random() - 0.5) * 2 * config.frequencyDrift;

  return {
    ...encoding,
    intensity: Math.max(0, Math.min(1, encoding.intensity * (1 + intensityVar))),
    frequency: Math.max(30, encoding.frequency + freqVar),
  };
}
```

## 9. Implementation Reference

### 9.1 Complete Distance Encoder Interface

```typescript
interface DistanceEncoder {
  // Configuration
  readonly maxRange: number;
  readonly zones: DistanceZone[];
  readonly encodingMethod: EncodingMethod;

  // Core encoding
  encode(distance: number): DistanceHapticResult;

  // Zone-based queries
  getZone(distance: number): DistanceZone;
  isInCriticalZone(distance: number): boolean;

  // Temporal patterns
  createApproachPattern(startDist: number, endDist: number, duration: number): HapticSequence;
  createAlertPattern(distance: number): HapticPattern;

  // Configuration
  setMaxRange(range: number): void;
  setEncodingMethod(method: EncodingMethod): void;
  calibrate(userPreferences: DistanceCalibration): void;
}
```

### 9.2 Default Implementation

```typescript
class DefaultDistanceEncoder implements DistanceEncoder {
  maxRange = 10;
  zones = STANDARD_ZONES;
  encodingMethod: EncodingMethod = 'combined';

  encode(distance: number): DistanceHapticResult {
    return encodeCombined(distance, this.maxRange, {
      method: 'combined',
      modulateFrequency: true,
      modulateIntensity: true,
      modulateWaveform: true,
      frequencyWeight: 1.0,
      intensityWeight: 1.0,
    });
  }

  getZone(distance: number): DistanceZone {
    return this.zones.find(z =>
      distance >= z.minDistance && distance < z.maxDistance
    ) ?? this.zones[this.zones.length - 1];
  }

  isInCriticalZone(distance: number): boolean {
    return this.getZone(distance).priority === 'critical';
  }
}
```

---

## Appendix A: Quick Reference

```
╔═══════════════════════════════════════════════════════════════════╗
║                 DISTANCE ENCODING QUICK REFERENCE                 ║
╠═══════════════════════════════════════════════════════════════════╣
║                                                                   ║
║  DISTANCE ZONES                                                   ║
║  ──────────────                                                   ║
║  0.0m ████████ CRITICAL  │ Continuous strong vibration           ║
║  0.5m ▓▓▓▓▓▓▓▓ NEAR      │ Rapid pulses (10-15 Hz)               ║
║  2.0m ▒▒▒▒▒▒▒▒ MEDIUM    │ Moderate pulses (3-8 Hz)              ║
║  5.0m ░░░░░░░░ FAR       │ Slow pulses (1-2 Hz)                  ║
║  10m+          SAFE      │ Silent or occasional                  ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  ENCODING METHODS                                                 ║
║  ────────────────                                                 ║
║  Frequency:  Closer = Faster pulses                              ║
║  Intensity:  Closer = Stronger vibration                         ║
║  Rhythm:     Closer = Shorter gaps, more pulses                  ║
║  Combined:   All of the above (RECOMMENDED)                       ║
║                                                                   ║
╠═══════════════════════════════════════════════════════════════════╣
║  OBSTACLE TYPE PRIORITY                                           ║
║  ──────────────────────                                           ║
║  🚗 Vehicle  ████████  (highest urgency)                         ║
║  ⚠️  Drop     ███████                                             ║
║  🚶 Person   █████                                                ║
║  📦 Stairs   ████                                                 ║
║  🧱 Wall     ███                                                  ║
║  🚪 Door     █ (opportunity, low urgency)                        ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
```
