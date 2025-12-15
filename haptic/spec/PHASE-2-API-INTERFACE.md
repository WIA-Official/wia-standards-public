# WIA Body Location Mapping Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines standardized body locations for haptic actuator placement and the semantic meanings associated with each location. By establishing consistent location mappings, haptic devices can provide intuitive spatial feedback regardless of hardware form factor.

## 2. Design Principles

### 2.1 Anatomical Consistency
- Locations defined relative to anatomical landmarks
- Consistent across body sizes and shapes
- Standardized naming convention

### 2.2 Perceptual Optimization
- Locations chosen for tactile sensitivity
- Spatial resolution considered
- Minimized interference between adjacent actuators

### 2.3 Device Flexibility
- Supports various wearable form factors
- Graceful degradation when fewer actuators available
- Extensible for future device types

## 3. Body Location Enumeration

### 3.1 Complete Location Map

```typescript
enum BodyLocation {
  // ═══════════════════════════════════════
  // HEAD / 머리
  // ═══════════════════════════════════════
  FOREHEAD_LEFT = 'forehead_left',
  FOREHEAD_CENTER = 'forehead_center',
  FOREHEAD_RIGHT = 'forehead_right',
  TEMPLE_LEFT = 'temple_left',
  TEMPLE_RIGHT = 'temple_right',

  // ═══════════════════════════════════════
  // NECK / 목
  // ═══════════════════════════════════════
  NECK_LEFT = 'neck_left',
  NECK_CENTER = 'neck_center',
  NECK_RIGHT = 'neck_right',
  NECK_BACK = 'neck_back',

  // ═══════════════════════════════════════
  // SHOULDERS / 어깨
  // ═══════════════════════════════════════
  SHOULDER_LEFT = 'shoulder_left',
  SHOULDER_RIGHT = 'shoulder_right',

  // ═══════════════════════════════════════
  // ARMS / 팔
  // ═══════════════════════════════════════
  UPPER_ARM_LEFT = 'upper_arm_left',
  UPPER_ARM_RIGHT = 'upper_arm_right',
  ELBOW_LEFT = 'elbow_left',
  ELBOW_RIGHT = 'elbow_right',
  FOREARM_LEFT = 'forearm_left',
  FOREARM_RIGHT = 'forearm_right',

  // ═══════════════════════════════════════
  // WRISTS / 손목
  // ═══════════════════════════════════════
  WRIST_LEFT_DORSAL = 'wrist_left_dorsal',       // 손등 쪽
  WRIST_LEFT_VOLAR = 'wrist_left_volar',         // 손바닥 쪽
  WRIST_RIGHT_DORSAL = 'wrist_right_dorsal',
  WRIST_RIGHT_VOLAR = 'wrist_right_volar',

  // ═══════════════════════════════════════
  // HANDS / 손
  // ═══════════════════════════════════════
  PALM_LEFT = 'palm_left',
  PALM_RIGHT = 'palm_right',
  BACK_HAND_LEFT = 'back_hand_left',
  BACK_HAND_RIGHT = 'back_hand_right',

  // ═══════════════════════════════════════
  // FINGERS / 손가락
  // ═══════════════════════════════════════
  THUMB_LEFT = 'thumb_left',
  INDEX_LEFT = 'index_left',
  MIDDLE_LEFT = 'middle_left',
  RING_LEFT = 'ring_left',
  PINKY_LEFT = 'pinky_left',
  THUMB_RIGHT = 'thumb_right',
  INDEX_RIGHT = 'index_right',
  MIDDLE_RIGHT = 'middle_right',
  RING_RIGHT = 'ring_right',
  PINKY_RIGHT = 'pinky_right',

  // ═══════════════════════════════════════
  // TORSO FRONT / 몸통 앞
  // ═══════════════════════════════════════
  CHEST_LEFT = 'chest_left',
  CHEST_CENTER = 'chest_center',
  CHEST_RIGHT = 'chest_right',
  ABDOMEN_LEFT = 'abdomen_left',
  ABDOMEN_CENTER = 'abdomen_center',
  ABDOMEN_RIGHT = 'abdomen_right',

  // ═══════════════════════════════════════
  // TORSO BACK / 몸통 뒤
  // ═══════════════════════════════════════
  BACK_UPPER_LEFT = 'back_upper_left',
  BACK_UPPER_CENTER = 'back_upper_center',
  BACK_UPPER_RIGHT = 'back_upper_right',
  BACK_LOWER_LEFT = 'back_lower_left',
  BACK_LOWER_CENTER = 'back_lower_center',
  BACK_LOWER_RIGHT = 'back_lower_right',

  // ═══════════════════════════════════════
  // WAIST / 허리
  // ═══════════════════════════════════════
  WAIST_LEFT = 'waist_left',
  WAIST_FRONT = 'waist_front',
  WAIST_RIGHT = 'waist_right',
  WAIST_BACK = 'waist_back',

  // ═══════════════════════════════════════
  // LEGS / 다리 (Optional, for full-body suits)
  // ═══════════════════════════════════════
  THIGH_LEFT = 'thigh_left',
  THIGH_RIGHT = 'thigh_right',
  KNEE_LEFT = 'knee_left',
  KNEE_RIGHT = 'knee_right',
  CALF_LEFT = 'calf_left',
  CALF_RIGHT = 'calf_right',
  ANKLE_LEFT = 'ankle_left',
  ANKLE_RIGHT = 'ankle_right',

  // ═══════════════════════════════════════
  // FEET / 발
  // ═══════════════════════════════════════
  FOOT_LEFT = 'foot_left',
  FOOT_RIGHT = 'foot_right',
}
```

### 3.2 Location Metadata

```typescript
interface BodyLocationInfo {
  id: BodyLocation;
  name: string;
  nameKorean: string;

  // Anatomical region
  region: 'head' | 'neck' | 'shoulder' | 'arm' | 'wrist' | 'hand' | 'finger' | 'torso' | 'waist' | 'leg' | 'foot';

  // Laterality
  side: 'left' | 'right' | 'center' | 'bilateral';

  // Relative sensitivity (1-10)
  sensitivity: number;

  // Spatial resolution (minimum distinguishable distance in mm)
  spatialResolution: number;

  // Common device types using this location
  commonDevices: string[];
}

const LOCATION_INFO: Record<BodyLocation, BodyLocationInfo> = {
  [BodyLocation.PALM_LEFT]: {
    id: BodyLocation.PALM_LEFT,
    name: 'Left Palm',
    nameKorean: '왼쪽 손바닥',
    region: 'hand',
    side: 'left',
    sensitivity: 9,
    spatialResolution: 2,
    commonDevices: ['haptic_glove', 'handheld_device'],
  },
  [BodyLocation.WRIST_LEFT_VOLAR]: {
    id: BodyLocation.WRIST_LEFT_VOLAR,
    name: 'Left Wrist (Inner)',
    nameKorean: '왼쪽 손목 안쪽',
    region: 'wrist',
    side: 'left',
    sensitivity: 7,
    spatialResolution: 8,
    commonDevices: ['smartwatch', 'wristband'],
  },
  // ... additional entries
};
```

## 4. Tactile Sensitivity Map

### 4.1 Sensitivity by Region

| Region | Sensitivity (1-10) | Two-Point Threshold | Notes |
|--------|-------------------|---------------------|-------|
| Fingertips | 10 | 2-3 mm | Highest resolution |
| Lips | 9 | 4-5 mm | Very sensitive |
| Palm | 8 | 8-10 mm | Good for patterns |
| Forearm | 6 | 35-40 mm | Large area patterns |
| Back | 5 | 40-50 mm | Broad strokes only |
| Thigh | 4 | 45-50 mm | Basic directional |

### 4.2 Sensitivity Visualization

```
         ╭──────╮
         │  8   │  ← Forehead
         ├──┬───┤
        7│  │  │7  ← Temples
         ╰──┴───╯

    ╭─────────────────╮
   6│        7        │6  ← Neck
    ├────┬───────┬────┤
   5│    │   6   │    │5  ← Shoulders
    │ 6  ├───────┤  6 │   ← Upper Arms
    │    │   5   │    │
    ├────┤   5   ├────┤   ← Chest/Back
   6│    │   5   │    │6  ← Forearms
    │    ├───────┤    │
    ├────┤   4   ├────┤   ← Abdomen
   7│    │       │    │7  ← Wrists
    ╰─┬──┴───────┴──┬─╯
     8│             │8    ← Palms
      │   ╭─────╮   │
     9│   │  4  │   │9    ← Fingers / Waist
      ╰───┤     ├───╯
          │  4  │         ← Thighs
          ├─────┤
          │  4  │         ← Calves
          ├─────┤
          │  6  │         ← Feet
          ╰─────╯

    Numbers indicate sensitivity (1-10)
```

## 5. Device Form Factors

### 5.1 Supported Device Types

```typescript
enum HapticDeviceType {
  // Wrist-worn
  SMARTWATCH = 'smartwatch',
  WRISTBAND = 'wristband',
  BRACELET = 'bracelet',

  // Hand-worn
  HAPTIC_GLOVE = 'haptic_glove',
  RING = 'ring',
  HANDHELD = 'handheld',

  // Arm-worn
  ARMBAND = 'armband',

  // Head-worn
  HEADBAND = 'headband',
  SMART_GLASSES = 'smart_glasses',
  HEADSET = 'headset',

  // Body-worn
  VEST = 'vest',
  BELT = 'belt',
  SHIRT = 'shirt',
  FULL_SUIT = 'full_suit',

  // Foot-worn
  INSOLE = 'insole',
  ANKLET = 'anklet',

  // Mobile
  SMARTPHONE = 'smartphone',
  TABLET = 'tablet',
  CANE = 'cane',
}
```

### 5.2 Device to Location Mapping

```typescript
interface DeviceLocationMapping {
  deviceType: HapticDeviceType;
  locations: BodyLocation[];
  actuatorCount: { min: number; typical: number; max: number };
  primaryUseCase: string;
}

const DEVICE_MAPPINGS: DeviceLocationMapping[] = [
  {
    deviceType: HapticDeviceType.SMARTWATCH,
    locations: [
      BodyLocation.WRIST_LEFT_DORSAL,
      BodyLocation.WRIST_LEFT_VOLAR,
    ],
    actuatorCount: { min: 1, typical: 1, max: 2 },
    primaryUseCase: 'notifications, confirmations',
  },
  {
    deviceType: HapticDeviceType.HAPTIC_GLOVE,
    locations: [
      BodyLocation.PALM_LEFT, BodyLocation.BACK_HAND_LEFT,
      BodyLocation.THUMB_LEFT, BodyLocation.INDEX_LEFT,
      BodyLocation.MIDDLE_LEFT, BodyLocation.RING_LEFT,
      BodyLocation.PINKY_LEFT,
    ],
    actuatorCount: { min: 5, typical: 10, max: 22 },
    primaryUseCase: 'spatial exploration, object recognition',
  },
  {
    deviceType: HapticDeviceType.VEST,
    locations: [
      BodyLocation.CHEST_LEFT, BodyLocation.CHEST_CENTER, BodyLocation.CHEST_RIGHT,
      BodyLocation.ABDOMEN_LEFT, BodyLocation.ABDOMEN_CENTER, BodyLocation.ABDOMEN_RIGHT,
      BodyLocation.BACK_UPPER_LEFT, BodyLocation.BACK_UPPER_CENTER, BodyLocation.BACK_UPPER_RIGHT,
      BodyLocation.BACK_LOWER_LEFT, BodyLocation.BACK_LOWER_CENTER, BodyLocation.BACK_LOWER_RIGHT,
    ],
    actuatorCount: { min: 4, typical: 12, max: 40 },
    primaryUseCase: 'navigation, spatial awareness',
  },
  {
    deviceType: HapticDeviceType.HEADBAND,
    locations: [
      BodyLocation.FOREHEAD_LEFT,
      BodyLocation.FOREHEAD_CENTER,
      BodyLocation.FOREHEAD_RIGHT,
      BodyLocation.TEMPLE_LEFT,
      BodyLocation.TEMPLE_RIGHT,
    ],
    actuatorCount: { min: 3, typical: 5, max: 8 },
    primaryUseCase: 'directional guidance, alerts',
  },
  {
    deviceType: HapticDeviceType.BELT,
    locations: [
      BodyLocation.WAIST_LEFT,
      BodyLocation.WAIST_FRONT,
      BodyLocation.WAIST_RIGHT,
      BodyLocation.WAIST_BACK,
    ],
    actuatorCount: { min: 4, typical: 8, max: 16 },
    primaryUseCase: 'directional navigation',
  },
  {
    deviceType: HapticDeviceType.INSOLE,
    locations: [
      BodyLocation.FOOT_LEFT,
      BodyLocation.FOOT_RIGHT,
    ],
    actuatorCount: { min: 2, typical: 6, max: 16 },
    primaryUseCase: 'walking navigation, terrain feedback',
  },
];
```

## 6. Spatial Semantics

### 6.1 Directional Mapping

```typescript
interface DirectionalMapping {
  direction: 'forward' | 'backward' | 'left' | 'right' | 'up' | 'down';
  primaryLocations: BodyLocation[];
  alternateLocations: BodyLocation[];
  pattern: string;
}

const DIRECTIONAL_MAPPINGS: DirectionalMapping[] = [
  {
    direction: 'forward',
    primaryLocations: [
      BodyLocation.CHEST_CENTER,
      BodyLocation.FOREHEAD_CENTER,
    ],
    alternateLocations: [
      BodyLocation.WAIST_FRONT,
      BodyLocation.ABDOMEN_CENTER,
    ],
    pattern: 'Single pulse or continuous low',
  },
  {
    direction: 'backward',
    primaryLocations: [
      BodyLocation.BACK_UPPER_CENTER,
      BodyLocation.BACK_LOWER_CENTER,
    ],
    alternateLocations: [
      BodyLocation.WAIST_BACK,
      BodyLocation.NECK_BACK,
    ],
    pattern: 'Pulling sensation, reverse ramp',
  },
  {
    direction: 'left',
    primaryLocations: [
      BodyLocation.CHEST_LEFT,
      BodyLocation.WRIST_LEFT_DORSAL,
      BodyLocation.WAIST_LEFT,
    ],
    alternateLocations: [
      BodyLocation.SHOULDER_LEFT,
      BodyLocation.TEMPLE_LEFT,
    ],
    pattern: 'Left-side emphasis, sweep left',
  },
  {
    direction: 'right',
    primaryLocations: [
      BodyLocation.CHEST_RIGHT,
      BodyLocation.WRIST_RIGHT_DORSAL,
      BodyLocation.WAIST_RIGHT,
    ],
    alternateLocations: [
      BodyLocation.SHOULDER_RIGHT,
      BodyLocation.TEMPLE_RIGHT,
    ],
    pattern: 'Right-side emphasis, sweep right',
  },
  {
    direction: 'up',
    primaryLocations: [
      BodyLocation.FOREHEAD_CENTER,
      BodyLocation.CHEST_CENTER,
    ],
    alternateLocations: [
      BodyLocation.SHOULDER_LEFT,
      BodyLocation.SHOULDER_RIGHT,
    ],
    pattern: 'Ascending sweep, rising intensity',
  },
  {
    direction: 'down',
    primaryLocations: [
      BodyLocation.ABDOMEN_CENTER,
      BodyLocation.FOOT_LEFT,
      BodyLocation.FOOT_RIGHT,
    ],
    alternateLocations: [
      BodyLocation.WAIST_FRONT,
    ],
    pattern: 'Descending sweep, falling intensity',
  },
];
```

### 6.2 Clock Position Mapping (360° Navigation)

```typescript
// For circular directional cues (like a compass or clock face)
interface ClockPositionMapping {
  position: number;  // 1-12 o'clock
  degrees: number;   // 0-360
  vestLocations: BodyLocation[];
  beltLocations: BodyLocation[];
}

const CLOCK_POSITIONS: ClockPositionMapping[] = [
  { position: 12, degrees: 0,   vestLocations: ['chest_center'], beltLocations: ['waist_front'] },
  { position: 1,  degrees: 30,  vestLocations: ['chest_right'], beltLocations: ['waist_front', 'waist_right'] },
  { position: 2,  degrees: 60,  vestLocations: ['chest_right'], beltLocations: ['waist_right'] },
  { position: 3,  degrees: 90,  vestLocations: ['chest_right', 'back_upper_right'], beltLocations: ['waist_right'] },
  { position: 4,  degrees: 120, vestLocations: ['back_upper_right'], beltLocations: ['waist_right', 'waist_back'] },
  { position: 5,  degrees: 150, vestLocations: ['back_upper_right', 'back_lower_right'], beltLocations: ['waist_back'] },
  { position: 6,  degrees: 180, vestLocations: ['back_upper_center', 'back_lower_center'], beltLocations: ['waist_back'] },
  { position: 7,  degrees: 210, vestLocations: ['back_upper_left', 'back_lower_left'], beltLocations: ['waist_back'] },
  { position: 8,  degrees: 240, vestLocations: ['back_upper_left'], beltLocations: ['waist_left', 'waist_back'] },
  { position: 9,  degrees: 270, vestLocations: ['chest_left', 'back_upper_left'], beltLocations: ['waist_left'] },
  { position: 10, degrees: 300, vestLocations: ['chest_left'], beltLocations: ['waist_left'] },
  { position: 11, degrees: 330, vestLocations: ['chest_left'], beltLocations: ['waist_front', 'waist_left'] },
];
```

### 6.3 Spatial Visualization (Vest Layout)

```
       FRONT                         BACK
    ┌─────────┐                  ┌─────────┐
    │ CL│CC│CR│                  │ BL│BC│BR│
    ├───┼──┼──┤                  ├───┼──┼──┤
    │ AL│AC│AR│                  │LL │LC│LR│
    └─────────┘                  └─────────┘

    CL = Chest Left              BL = Back Upper Left
    CC = Chest Center            BC = Back Upper Center
    CR = Chest Right             BR = Back Upper Right
    AL = Abdomen Left            LL = Back Lower Left
    AC = Abdomen Center          LC = Back Lower Center
    AR = Abdomen Right           LR = Back Lower Right
```

## 7. Coordinate Systems

### 7.1 Absolute Coordinates

```typescript
interface AbsoluteCoordinate {
  // Normalized position (0.0 - 1.0)
  // Origin: top of head = 0, feet = 1
  vertical: number;

  // -1.0 (far left) to 1.0 (far right)
  // 0 = center/spine
  horizontal: number;

  // -1.0 (back) to 1.0 (front)
  // 0 = side
  depth: number;
}

// Convert body location to approximate coordinates
function locationToCoordinates(location: BodyLocation): AbsoluteCoordinate {
  const COORDINATE_MAP: Record<BodyLocation, AbsoluteCoordinate> = {
    [BodyLocation.FOREHEAD_CENTER]: { vertical: 0.05, horizontal: 0, depth: 1.0 },
    [BodyLocation.CHEST_CENTER]: { vertical: 0.35, horizontal: 0, depth: 1.0 },
    [BodyLocation.WAIST_LEFT]: { vertical: 0.50, horizontal: -0.8, depth: 0 },
    [BodyLocation.BACK_UPPER_CENTER]: { vertical: 0.35, horizontal: 0, depth: -1.0 },
    // ... etc
  };
  return COORDINATE_MAP[location];
}
```

### 7.2 Relative Coordinates

```typescript
interface RelativePosition {
  // Reference point
  anchor: BodyLocation;

  // Offset in centimeters
  offsetX: number;  // lateral
  offsetY: number;  // vertical
  offsetZ: number;  // depth
}

// For sub-location precision (e.g., specific finger segment)
interface SubLocationPosition {
  baseLocation: BodyLocation;
  segment?: 'proximal' | 'medial' | 'distal';  // for fingers
  zone?: 'inner' | 'center' | 'outer';          // for larger areas
}
```

## 8. Multi-Actuator Patterns

### 8.1 Sequential Activation (Apparent Motion)

```typescript
interface ApparentMotionPattern {
  // Creates illusion of movement across body
  name: string;
  locations: BodyLocation[];
  timing: number[];      // onset times in ms
  durations: number[];   // per-actuator duration
  intensities: number[]; // per-actuator intensity
}

const MOTION_PATTERNS: ApparentMotionPattern[] = [
  {
    name: 'sweep_left_to_right',
    locations: [
      BodyLocation.CHEST_LEFT,
      BodyLocation.CHEST_CENTER,
      BodyLocation.CHEST_RIGHT,
    ],
    timing: [0, 80, 160],
    durations: [100, 100, 100],
    intensities: [0.7, 0.7, 0.7],
  },
  {
    name: 'ascend_spine',
    locations: [
      BodyLocation.BACK_LOWER_CENTER,
      BodyLocation.BACK_UPPER_CENTER,
      BodyLocation.NECK_BACK,
    ],
    timing: [0, 100, 200],
    durations: [120, 120, 120],
    intensities: [0.5, 0.6, 0.7],
  },
];
```

### 8.2 Simultaneous Activation (Phantom Sensations)

```typescript
interface PhantomSensationPattern {
  // Creates sensation between actuator locations
  name: string;
  locations: [BodyLocation, BodyLocation];
  intensityRatio: number;  // 0.0-1.0 (0 = first, 1 = second, 0.5 = center)
  synchronize: boolean;
}

// Example: Sensation appears between two actuators
const PHANTOM_PATTERNS: PhantomSensationPattern[] = [
  {
    name: 'center_chest',
    locations: [BodyLocation.CHEST_LEFT, BodyLocation.CHEST_RIGHT],
    intensityRatio: 0.5,
    synchronize: true,
  },
];
```

### 8.3 Saltation (Cutaneous Rabbit)

```typescript
interface SaltationPattern {
  // Rapid taps create perceptual filling between locations
  name: string;
  locations: BodyLocation[];
  tapCount: number;
  totalDuration: number;  // ms
  description: string;
}

const SALTATION_PATTERNS: SaltationPattern[] = [
  {
    name: 'arm_crawl',
    locations: [
      BodyLocation.WRIST_LEFT_DORSAL,
      BodyLocation.FOREARM_LEFT,
      BodyLocation.UPPER_ARM_LEFT,
    ],
    tapCount: 6,  // 2 per location
    totalDuration: 300,
    description: 'Creates sensation of movement up the arm',
  },
];
```

## 9. Accessibility Considerations

### 9.1 Reduced Sensitivity Adaptations

```typescript
interface SensitivityAdaptation {
  condition: string;
  affectedLocations: BodyLocation[];
  compensation: {
    intensityMultiplier: number;
    durationMultiplier: number;
    useAlternateLocations: BodyLocation[];
  };
}

const ADAPTATIONS: SensitivityAdaptation[] = [
  {
    condition: 'diabetic_neuropathy',
    affectedLocations: [
      BodyLocation.FOOT_LEFT, BodyLocation.FOOT_RIGHT,
      BodyLocation.ANKLE_LEFT, BodyLocation.ANKLE_RIGHT,
    ],
    compensation: {
      intensityMultiplier: 1.5,
      durationMultiplier: 1.3,
      useAlternateLocations: [
        BodyLocation.CALF_LEFT, BodyLocation.CALF_RIGHT,
      ],
    },
  },
];
```

### 9.2 User Calibration

```typescript
interface UserCalibration {
  userId: string;

  // Per-location sensitivity threshold
  perceptionThresholds: Map<BodyLocation, number>;

  // Per-location comfortable maximum
  comfortMaximum: Map<BodyLocation, number>;

  // Unavailable locations (injury, prosthetic, etc.)
  unavailableLocations: BodyLocation[];

  // Preferred alternate locations
  alternatePreferences: Map<BodyLocation, BodyLocation>;
}
```

## 10. Implementation Guidelines

### 10.1 Minimum Viable Device Configurations

| Use Case | Minimum Locations | Recommended |
|----------|------------------|-------------|
| Basic notifications | 1 (wrist) | 2 (bilateral wrist) |
| Directional navigation | 4 (belt) | 8 (belt) |
| Spatial awareness | 6 (vest) | 12 (vest) |
| Full navigation | 8 (belt + wrist) | 16 (vest + wrist) |
| Object recognition | 5 (glove) | 10 (full glove) |

### 10.2 Fallback Strategies

```typescript
interface FallbackStrategy {
  missingLocation: BodyLocation;
  fallbackChain: BodyLocation[];
  patternModification?: string;
}

const FALLBACK_STRATEGIES: FallbackStrategy[] = [
  {
    missingLocation: BodyLocation.CHEST_CENTER,
    fallbackChain: [
      BodyLocation.CHEST_LEFT,
      BodyLocation.CHEST_RIGHT,
      BodyLocation.ABDOMEN_CENTER,
    ],
    patternModification: 'Use bilateral activation for center sensation',
  },
  {
    missingLocation: BodyLocation.FOREHEAD_CENTER,
    fallbackChain: [
      BodyLocation.TEMPLE_LEFT,
      BodyLocation.TEMPLE_RIGHT,
      BodyLocation.NECK_CENTER,
    ],
  },
];
```

---

## Appendix A: Body Location Quick Reference

```
╔═══════════════════════════════════════════════════════════════════════╗
║                     WIA BODY LOCATION QUICK REFERENCE                 ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║                         ┌───────────┐                                 ║
║                         │  FL FC FR │  ← Forehead                     ║
║                         │ TL     TR │  ← Temples                      ║
║                         └─────┬─────┘                                 ║
║                         ┌─────┼─────┐                                 ║
║                         │ NL NC NR  │  ← Neck                         ║
║              ┌──────────┼───────────┼──────────┐                      ║
║              │    SL    │           │    SR    │  ← Shoulders         ║
║              ├──────────┼───────────┼──────────┤                      ║
║              │   UAL    │  CL CC CR │   UAR    │  ← Upper Arms/Chest  ║
║              ├──────────┼───────────┼──────────┤                      ║
║              │   FAL    │  AL AC AR │   FAR    │  ← Forearms/Abdomen  ║
║              ├──────────┼───────────┼──────────┤                      ║
║              │   WL     │ WF    WB  │   WR     │  ← Wrists/Waist      ║
║              └────┬─────┴───────────┴─────┬────┘                      ║
║                   │                       │                           ║
║              ┌────┴────┐             ┌────┴────┐                      ║
║              │   PL    │             │   PR    │  ← Palms             ║
║              │ T I M R P             │ T I M R P  ← Fingers           ║
║              └─────────┘             └─────────┘                      ║
║                                                                       ║
║  LEGEND: L=Left, R=Right, C=Center, F=Front, B=Back                  ║
║          T=Thumb, I=Index, M=Middle, R=Ring, P=Pinky                 ║
╚═══════════════════════════════════════════════════════════════════════╝
```

## Appendix B: Device Form Factor Diagrams

```
SMARTWATCH           VEST                    BELT
┌─────────┐      ┌───────────┐          ┌─────────────┐
│  [◉]    │      │ [◉][◉][◉] │          │[◉][◉][◉][◉]│
│ wrist   │      │ [◉][◉][◉] │          └─────────────┘
└─────────┘      │ [◉][◉][◉] │           front + back
                 │ [◉][◉][◉] │
                 └───────────┘
                  front + back

GLOVE               HEADBAND              INSOLE
┌─────────┐      ┌───────────┐        ┌───────────┐
│ [◉]     │      │[◉][◉][◉]  │        │ [◉]  [◉]  │
│[◉][◉][◉]│      │[◉]   [◉]  │        │   [◉][◉]  │
│[◉][◉][◉]│      └───────────┘        │ [◉]  [◉]  │
└─────────┘                           └───────────┘
```
