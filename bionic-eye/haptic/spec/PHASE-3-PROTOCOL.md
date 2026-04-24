# WIA Haptic Directional Encoding Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines how directional information (azimuth, elevation) is encoded into haptic patterns. Directional encoding enables users to perceive the direction of objects, destinations, or points of interest through tactile feedback.

## 2. Design Principles

### 2.1 Intuitive Mapping
- Physical location of haptic stimulus maps to real-world direction
- Stronger stimulus indicates "more in that direction"
- Consistent mapping across all device types

### 2.2 Resolution vs. Simplicity
- Trade-off between angular precision and cognitive load
- Simpler encodings for quick decisions
- Higher resolution available when needed

### 2.3 Cross-Device Compatibility
- Algorithms adapt to available actuator count
- Graceful degradation with fewer actuators
- Consistent perception across hardware

## 3. Coordinate System

### 3.1 Azimuth (Horizontal Direction)

```
                    0° (Forward)
                        ↑
                        │
         315° ─────────┼───────── 45°
        (Front-Left)   │      (Front-Right)
                       │
    270° ─────────────●───────────── 90°
    (Left)          User          (Right)
                       │
         225° ─────────┼───────── 135°
        (Back-Left)    │      (Back-Right)
                       │
                        ↓
                   180° (Back)
```

```typescript
interface AzimuthEncoding {
  // Degrees from forward (0-360, clockwise)
  azimuth: number;

  // Convert to normalized range (-180 to 180)
  toSigned(): number;

  // Get cardinal direction
  toCardinal(): 'N' | 'NE' | 'E' | 'SE' | 'S' | 'SW' | 'W' | 'NW';

  // Get clock position (1-12)
  toClockPosition(): number;
}
```

### 3.2 Elevation (Vertical Direction)

```
            +90° (Directly Above)
                    ↑
                    │
          +45° ─────┼───── +45° (Upper)
                    │
    ────────────────●────────────── 0° (Horizon)
                    │
          -45° ─────┼───── -45° (Lower)
                    │
                    ↓
            -90° (Directly Below)
```

```typescript
interface ElevationEncoding {
  // Degrees from horizon (-90 to +90)
  elevation: number;

  // Simplified levels
  toLevel(): 'above' | 'level' | 'below';

  // More granular
  toDetailedLevel(): 'high' | 'upper' | 'level' | 'lower' | 'ground';
}
```

### 3.3 Combined Spherical Coordinates

```typescript
interface SpatialDirection {
  azimuth: number;     // 0-360°
  elevation: number;   // -90 to +90°

  // Compute unit vector
  toUnitVector(): { x: number; y: number; z: number };

  // Angular distance to another direction
  angleTo(other: SpatialDirection): number;
}
```

## 4. Actuator Configurations

### 4.1 Configuration Types

| Config | Actuators | Angular Resolution | Use Case |
|--------|-----------|-------------------|----------|
| Single | 1 | N/A (intensity only) | Basic alerts |
| Dual | 2 | 180° (left/right) | Simple navigation |
| Quad | 4 | 90° | Belt-style navigation |
| Octal | 8 | 45° | Precision navigation |
| Continuous | 12+ | ~30° | Full spatial awareness |

### 4.2 Quad Configuration (4 Actuators)

```
       [Front]
          ↑
    [Left]●[Right]
          ↓
       [Back]

Actuator positions:
- Front: 0° (chest_center / waist_front)
- Right: 90° (chest_right / waist_right)
- Back: 180° (back_upper_center / waist_back)
- Left: 270° (chest_left / waist_left)
```

```typescript
interface QuadEncoder {
  positions: [0, 90, 180, 270];  // degrees

  encode(azimuth: number): ActuatorActivation[] {
    // Calculate activation for each actuator
    const activations = this.positions.map(pos => {
      const delta = angleDifference(azimuth, pos);
      // Cosine falloff within 90° window
      const intensity = Math.max(0, Math.cos(toRadians(delta)));
      return { position: pos, intensity };
    });
    return activations;
  }
}
```

#### Quad Encoding Examples

| Target Direction | Front | Right | Back | Left |
|------------------|-------|-------|------|------|
| 0° (Forward) | 1.0 | 0.0 | 0.0 | 0.0 |
| 45° (Front-Right) | 0.71 | 0.71 | 0.0 | 0.0 |
| 90° (Right) | 0.0 | 1.0 | 0.0 | 0.0 |
| 135° (Back-Right) | 0.0 | 0.71 | 0.71 | 0.0 |
| 180° (Back) | 0.0 | 0.0 | 1.0 | 0.0 |

### 4.3 Octal Configuration (8 Actuators)

```
           [0°]
       [315°] [45°]
    [270°]  ●  [90°]
       [225°] [135°]
          [180°]
```

```typescript
interface OctalEncoder {
  positions: [0, 45, 90, 135, 180, 225, 270, 315];

  encode(azimuth: number): ActuatorActivation[] {
    return this.positions.map(pos => {
      const delta = angleDifference(azimuth, pos);
      // Narrower falloff for higher resolution
      const intensity = Math.max(0, Math.cos(toRadians(delta * 2)));
      return { position: pos, intensity };
    });
  }
}
```

### 4.4 Continuous Configuration (12+ Actuators)

For vest-style devices with many actuators:

```typescript
interface ContinuousEncoder {
  actuatorCount: number;
  actuatorPositions: number[];  // Actual positions in degrees

  encode(azimuth: number): ActuatorActivation[] {
    return this.actuatorPositions.map(pos => {
      const delta = angleDifference(azimuth, pos);
      const halfSpacing = 180 / this.actuatorCount;

      // Gaussian-like falloff
      const sigma = halfSpacing;
      const intensity = Math.exp(-(delta * delta) / (2 * sigma * sigma));

      return { position: pos, intensity };
    });
  }
}
```

## 5. Encoding Algorithms

### 5.1 Nearest Neighbor (Simple)

Activate only the nearest actuator:

```typescript
function nearestNeighborEncode(
  azimuth: number,
  actuatorPositions: number[]
): ActuatorActivation[] {
  let minDelta = Infinity;
  let nearestIndex = 0;

  actuatorPositions.forEach((pos, i) => {
    const delta = Math.abs(angleDifference(azimuth, pos));
    if (delta < minDelta) {
      minDelta = delta;
      nearestIndex = i;
    }
  });

  return actuatorPositions.map((pos, i) => ({
    position: pos,
    intensity: i === nearestIndex ? 1.0 : 0.0,
  }));
}
```

**Pros:** Clear, unambiguous
**Cons:** Limited resolution, abrupt transitions

### 5.2 Linear Interpolation

Interpolate between two nearest actuators:

```typescript
function linearInterpolateEncode(
  azimuth: number,
  actuatorPositions: number[]
): ActuatorActivation[] {
  // Find two nearest actuators
  const sorted = [...actuatorPositions]
    .map((pos, i) => ({ pos, i, delta: angleDifference(azimuth, pos) }))
    .sort((a, b) => Math.abs(a.delta) - Math.abs(b.delta));

  const [first, second] = sorted;

  // Calculate interpolation weights
  const totalDelta = Math.abs(first.delta) + Math.abs(second.delta);
  const weight1 = 1 - Math.abs(first.delta) / totalDelta;
  const weight2 = 1 - Math.abs(second.delta) / totalDelta;

  return actuatorPositions.map((pos, i) => ({
    position: pos,
    intensity: i === first.i ? weight1 : i === second.i ? weight2 : 0,
  }));
}
```

**Pros:** Smooth transitions, better angular perception
**Cons:** Less distinct activation

### 5.3 Cosine Falloff (Recommended)

Smooth activation based on angular distance:

```typescript
function cosineFalloffEncode(
  azimuth: number,
  actuatorPositions: number[],
  falloffAngle: number = 90  // degrees
): ActuatorActivation[] {
  return actuatorPositions.map(pos => {
    const delta = angleDifference(azimuth, pos);
    const normalized = delta / falloffAngle;

    let intensity = 0;
    if (Math.abs(normalized) <= 1) {
      intensity = (Math.cos(normalized * Math.PI) + 1) / 2;
    }

    return { position: pos, intensity };
  });
}
```

**Pros:** Perceptually smooth, natural feel
**Cons:** Multiple simultaneous activations

### 5.4 Gaussian Falloff

For high-resolution continuous arrays:

```typescript
function gaussianFalloffEncode(
  azimuth: number,
  actuatorPositions: number[],
  sigma: number = 30  // degrees
): ActuatorActivation[] {
  return actuatorPositions.map(pos => {
    const delta = angleDifference(azimuth, pos);
    const intensity = Math.exp(-(delta * delta) / (2 * sigma * sigma));
    return { position: pos, intensity };
  });
}
```

## 6. Elevation Encoding

### 6.1 Frequency-Based Elevation

Map elevation to vibration frequency:

```typescript
interface ElevationFrequencyMapping {
  above: {
    high: 250,      // +60° to +90°
    upper: 200,     // +30° to +60°
  };
  level: 150,       // -30° to +30°
  below: {
    lower: 100,     // -30° to -60°
    ground: 60,     // -60° to -90°
  };
}

function encodeElevation(elevation: number): number {
  if (elevation > 60) return 250;
  if (elevation > 30) return 200;
  if (elevation > -30) return 150;
  if (elevation > -60) return 100;
  return 60;
}
```

### 6.2 Pattern-Based Elevation

Use distinct patterns for different elevations:

```typescript
const ELEVATION_PATTERNS = {
  above: {
    pattern: 'rising_sweep',    // Upward feeling
    envelope: 'swell',
  },
  level: {
    pattern: 'steady_pulse',
    envelope: 'pulse',
  },
  below: {
    pattern: 'falling_sweep',   // Downward feeling
    envelope: 'fade',
  },
};
```

### 6.3 Multi-Row Elevation (Vest)

For vests with multiple vertical rows:

```
    Upper Row:  [UL] [UC] [UR]    ← Above (elevation > 20°)
    Middle Row: [ML] [MC] [MR]    ← Level (-20° to 20°)
    Lower Row:  [LL] [LC] [LR]    ← Below (elevation < -20°)
```

```typescript
function encodeWithElevation(
  azimuth: number,
  elevation: number,
  vestConfig: VestConfiguration
): ActuatorActivation[] {
  // Get horizontal activations
  const azimuthActivations = cosineFalloffEncode(
    azimuth,
    vestConfig.columns.map(c => c.azimuth)
  );

  // Determine vertical row
  let activeRow: 'upper' | 'middle' | 'lower';
  if (elevation > 20) activeRow = 'upper';
  else if (elevation < -20) activeRow = 'lower';
  else activeRow = 'middle';

  // Combine
  return vestConfig.actuators
    .filter(a => a.row === activeRow)
    .map(a => {
      const columnActivation = azimuthActivations.find(
        aa => aa.position === a.columnAzimuth
      );
      return {
        location: a.location,
        intensity: columnActivation?.intensity ?? 0,
      };
    });
}
```

## 7. Temporal Patterns for Direction

### 7.1 Sweeping Pattern

Create apparent motion toward the target:

```typescript
function createSweepPattern(
  targetAzimuth: number,
  actuatorPositions: number[],
  sweepDuration: number = 500  // ms
): TemporalPattern {
  // Start from opposite direction
  const startAzimuth = (targetAzimuth + 180) % 360;

  // Calculate sweep through each actuator
  const steps = actuatorPositions
    .map(pos => {
      const deltaFromStart = angleDifference(pos, startAzimuth);
      const deltaToTarget = angleDifference(targetAzimuth, startAzimuth);
      const progress = deltaFromStart / deltaToTarget;

      return {
        position: pos,
        startTime: progress * sweepDuration * 0.8,
        duration: sweepDuration * 0.3,
        intensity: 0.7,
      };
    })
    .filter(s => s.startTime >= 0)
    .sort((a, b) => a.startTime - b.startTime);

  // End with strong pulse at target
  steps.push({
    position: targetAzimuth,
    startTime: sweepDuration * 0.9,
    duration: sweepDuration * 0.2,
    intensity: 1.0,
  });

  return { steps, totalDuration: sweepDuration };
}
```

### 7.2 Pulsing Direction

Rhythmic pulses indicating direction:

```typescript
interface DirectionalPulse {
  direction: number;
  pulseCount: number;
  pulseInterval: number;  // ms
  intensityPattern: 'constant' | 'increasing' | 'decreasing';
}

function createDirectionalPulses(
  azimuth: number,
  config: DirectionalPulse
): HapticSequence {
  const activations = cosineFalloffEncode(azimuth, QUAD_POSITIONS);

  return {
    steps: Array(config.pulseCount).fill(null).map((_, i) => ({
      actuations: activations.map(a => ({
        location: positionToLocation(a.position),
        intensity: a.intensity * getIntensityMultiplier(i, config),
      })),
      delayBefore: i === 0 ? 0 : config.pulseInterval,
      duration: 50,
    })),
  };
}
```

### 7.3 Rotating Beacon

For indicating "turn around" or "look for it":

```typescript
function createRotatingBeacon(
  centerAzimuth: number,
  rotationSpeed: number = 360,  // degrees per second
  duration: number = 2000       // ms
): TemporalPattern {
  const steps = [];
  const stepInterval = 50;  // ms
  const degreesPerStep = rotationSpeed * (stepInterval / 1000);

  for (let t = 0; t < duration; t += stepInterval) {
    const currentAzimuth = (centerAzimuth + (t / 1000) * rotationSpeed) % 360;
    const activations = cosineFalloffEncode(currentAzimuth, OCTAL_POSITIONS);

    steps.push({
      time: t,
      actuations: activations,
    });
  }

  return { steps, totalDuration: duration };
}
```

## 8. Special Direction Patterns

### 8.1 "Arrived" Pattern

Indicates destination reached:

```typescript
const ARRIVED_PATTERN = {
  type: 'converging',
  description: 'All actuators pulse inward',
  steps: [
    { locations: 'outer_ring', intensity: 0.8, time: 0 },
    { locations: 'inner_ring', intensity: 0.9, time: 100 },
    { locations: 'center', intensity: 1.0, time: 200 },
  ],
  totalDuration: 300,
};
```

### 8.2 "Lost" Pattern

Indicates direction unknown:

```typescript
const LOST_PATTERN = {
  type: 'searching',
  description: 'Gentle rotating sweep',
  rotation: 'clockwise',
  speed: 180,  // degrees per second
  intensity: 0.4,
  duration: 3000,
};
```

### 8.3 "Behind You" Pattern

Strong indication of something behind:

```typescript
const BEHIND_PATTERN = {
  type: 'attention_back',
  steps: [
    { location: 'back_center', intensity: 1.0, duration: 100 },
    { location: 'back_center', intensity: 0.5, duration: 50 },
    { location: 'back_center', intensity: 1.0, duration: 100 },
  ],
  envelope: 'sharp',
  frequency: 200,
};
```

## 9. Adaptation and Learning

### 9.1 Progressive Refinement

Start with simple encoding, increase complexity as user learns:

```typescript
interface DirectionEncodingLevel {
  level: 1 | 2 | 3;
  description: string;
  encoding: EncodingAlgorithm;
}

const ENCODING_LEVELS: DirectionEncodingLevel[] = [
  {
    level: 1,
    description: 'Basic 4-way (Front/Back/Left/Right)',
    encoding: 'nearest_neighbor_quad',
  },
  {
    level: 2,
    description: 'Interpolated 8-way',
    encoding: 'linear_interpolate_octal',
  },
  {
    level: 3,
    description: 'Continuous 360°',
    encoding: 'cosine_falloff_continuous',
  },
];
```

### 9.2 User Calibration

```typescript
interface DirectionCalibration {
  // User's perception offsets
  azimuthOffset: number;  // If user perceives front as off-center

  // Intensity adjustments per direction
  intensityScale: Record<CardinalDirection, number>;

  // Preferred encoding method
  preferredEncoding: EncodingAlgorithm;
}
```

## 10. Implementation Reference

### 10.1 TypeScript Interface

```typescript
interface DirectionalEncoder {
  // Configuration
  readonly actuatorCount: number;
  readonly actuatorPositions: number[];
  readonly encodingAlgorithm: EncodingAlgorithm;

  // Encode direction to activations
  encode(azimuth: number, elevation?: number): ActuatorActivation[];

  // Create temporal pattern for direction
  createPattern(azimuth: number, options?: PatternOptions): HapticPattern;

  // Update configuration
  setAlgorithm(algorithm: EncodingAlgorithm): void;
  calibrate(calibration: DirectionCalibration): void;
}
```

### 10.2 Utility Functions

```typescript
// Normalize angle to 0-360
function normalizeAngle(angle: number): number {
  return ((angle % 360) + 360) % 360;
}

// Calculate shortest angular difference
function angleDifference(a: number, b: number): number {
  const diff = normalizeAngle(a - b);
  return diff > 180 ? diff - 360 : diff;
}

// Convert degrees to radians
function toRadians(degrees: number): number {
  return degrees * (Math.PI / 180);
}

// Map actuator position to body location
function positionToLocation(position: number): BodyLocation {
  const mapping: Record<number, BodyLocation> = {
    0: 'chest_center',
    45: 'chest_right',
    90: 'waist_right',
    135: 'back_upper_right',
    180: 'back_upper_center',
    225: 'back_upper_left',
    270: 'waist_left',
    315: 'chest_left',
  };
  return mapping[normalizeAngle(position)] ?? 'chest_center';
}
```

---

## Appendix A: Quick Reference

```
╔════════════════════════════════════════════════════════════════════╗
║                  DIRECTIONAL ENCODING REFERENCE                    ║
╠════════════════════════════════════════════════════════════════════╣
║                                                                    ║
║  AZIMUTH (Top View)              ELEVATION (Side View)            ║
║                                                                    ║
║         0° Front                      +90° Above                   ║
║            ↑                              ↑                        ║
║   270° ←   ●   → 90°              ────────●──────── 0° Level      ║
║            ↓                              ↓                        ║
║        180° Back                      -90° Below                   ║
║                                                                    ║
╠════════════════════════════════════════════════════════════════════╣
║  ENCODING ALGORITHMS                                               ║
║  ─────────────────                                                 ║
║  Nearest Neighbor: Clearest, lowest resolution                    ║
║  Linear Interp:    Smooth transitions, 2 actuators                ║
║  Cosine Falloff:   Natural feel, multiple actuators (RECOMMENDED) ║
║  Gaussian:         Highest resolution, for many actuators          ║
║                                                                    ║
╠════════════════════════════════════════════════════════════════════╣
║  QUAD CONFIG          │  OCTAL CONFIG                              ║
║       [F]             │       [0]                                  ║
║   [L]  ●  [R]         │   [315] [45]                              ║
║       [B]             │  [270]●[90]                               ║
║                       │   [225] [135]                              ║
║  F=0° R=90°           │      [180]                                ║
║  B=180° L=270°        │                                           ║
╚════════════════════════════════════════════════════════════════════╝
```
