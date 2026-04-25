# WIA Haptic Spatial Scene Encoding Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-01-15

## 1. Overview

This specification defines how complex spatial scenes with multiple objects are encoded into haptic patterns. Spatial scene encoding enables users to build mental models of their environment through tactile feedback, understanding not just individual obstacles but the overall spatial layout.

## 2. Design Principles

### 2.1 Cognitive Load Management
- Limit simultaneous information to prevent overload
- Prioritize based on urgency and relevance
- Progressive disclosure of detail

### 2.2 Spatial Coherence
- Maintain consistent spatial mapping
- Preserve relative positions of objects
- Support mental map construction

### 2.3 Temporal Consistency
- Smooth transitions as scene changes
- Predictable update rhythm
- Avoid jarring switches

## 3. Scene Representation

### 3.1 Core Data Structures

```typescript
interface SpatialScene {
  // Timestamp of scene capture
  timestamp: number;

  // User's position and orientation
  userPose: UserPose;

  // All detected obstacles
  obstacles: Obstacle[];

  // Environmental features
  features: EnvironmentFeature[];

  // Navigation context
  navigation?: NavigationContext;
}

interface UserPose {
  position: Vector3;
  heading: number;       // degrees, 0 = north
  velocity?: Vector3;    // movement vector
  angularVelocity?: number;  // rotation speed
}

interface Obstacle {
  id: string;
  type: ObstacleType;

  // Position relative to user
  direction: number;     // azimuth (0-360°)
  elevation: number;     // vertical angle (-90 to 90°)
  distance: number;      // meters

  // Size and shape
  width?: number;        // angular width (degrees)
  height?: number;       // vertical extent (meters)
  depth?: number;        // thickness (meters)

  // Dynamics
  velocity?: Vector3;    // movement vector
  isMoving: boolean;

  // Classification
  confidence: number;    // 0-1 detection confidence
  priority: number;      // 0-1 computed priority

  // Metadata
  label?: string;
  lastSeen: number;      // timestamp
}

interface EnvironmentFeature {
  type: 'wall' | 'corner' | 'doorway' | 'opening' | 'stairs_up' | 'stairs_down' | 'ramp' | 'curb';
  direction: number;
  distance: number;
  extent?: number;       // angular extent
}

interface NavigationContext {
  destination?: SpatialDirection;
  nextWaypoint?: SpatialDirection;
  pathClear: boolean;
  estimatedDistance?: number;
}
```

### 3.2 Obstacle Types

```typescript
type ObstacleType =
  // Static obstacles
  | 'wall'
  | 'furniture'
  | 'pole'
  | 'sign'
  | 'tree'
  | 'parked_vehicle'

  // Dynamic obstacles
  | 'person'
  | 'group'
  | 'cyclist'
  | 'vehicle'
  | 'animal'

  // Elevation changes
  | 'stairs_up'
  | 'stairs_down'
  | 'ramp_up'
  | 'ramp_down'
  | 'curb_up'
  | 'curb_down'
  | 'drop'

  // Openings
  | 'doorway'
  | 'gap'
  | 'crossing'

  // Unknown
  | 'unknown';
```

## 4. Scene Encoding Strategies

### 4.1 Time Division Multiplexing (TDM)

Present objects sequentially:

```
Time →
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Obj1│ Gap │ Obj2│ Gap │ Obj3│ Gap │ Obj1│ Gap │...
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
   │           │           │           │
   └───────────┴───────────┴───────────┘
              Cycle Duration
```

```typescript
interface TDMEncoding {
  strategy: 'time_division';

  // Time allocation
  slotDuration: number;    // ms per object
  gapDuration: number;     // ms between objects
  cycleDuration: number;   // total cycle time

  // Object ordering
  ordering: 'priority' | 'clockwise' | 'distance' | 'round_robin';

  // Maximum objects per cycle
  maxObjects: number;
}

function encodeTDM(
  scene: SpatialScene,
  config: TDMEncoding
): HapticSequence {
  // Sort and limit objects
  const sortedObstacles = sortObstacles(scene.obstacles, config.ordering)
    .slice(0, config.maxObjects);

  // Calculate timing
  const actualSlotDuration = config.cycleDuration / sortedObstacles.length - config.gapDuration;

  const steps: SequenceStep[] = sortedObstacles.map((obstacle, index) => {
    const startTime = index * (actualSlotDuration + config.gapDuration);

    // Encode this obstacle's direction and distance
    const directionalPattern = encodeDirection(obstacle.direction);
    const distancePattern = encodeDistance(obstacle.distance);

    return {
      startTime,
      duration: actualSlotDuration,
      actuations: mergePatterns(directionalPattern, distancePattern),
      metadata: { obstacleId: obstacle.id },
    };
  });

  return {
    steps,
    loop: true,
    cycleDuration: config.cycleDuration,
  };
}
```

#### TDM Timing Examples

| Scenario | Objects | Slot | Gap | Cycle |
|----------|---------|------|-----|-------|
| Quick scan | 3 | 150ms | 50ms | 600ms |
| Detailed | 5 | 200ms | 50ms | 1250ms |
| Navigation | 2 | 300ms | 100ms | 800ms |

### 4.2 Spatial Division Multiplexing (SDM)

Use different actuators for different spatial regions:

```
        Front Actuators: Forward obstacles
             [F1][F2][F3]
              ↑   ↑   ↑

  Left                      Right
   [L]  ←──── User ────→    [R]

              ↓   ↓   ↓
             [B1][B2][B3]
        Back Actuators: Rear obstacles
```

```typescript
interface SDMEncoding {
  strategy: 'spatial_division';

  // Region definitions
  regions: SpatialRegion[];

  // Actuator assignments
  actuatorMap: Map<string, BodyLocation[]>;

  // Simultaneous encoding
  simultaneousRegions: boolean;
}

interface SpatialRegion {
  id: string;
  azimuthStart: number;
  azimuthEnd: number;
  elevationStart?: number;
  elevationEnd?: number;
  assignedActuators: BodyLocation[];
}

function encodeSDM(
  scene: SpatialScene,
  config: SDMEncoding
): HapticActuationMap {
  const activations: Map<BodyLocation, ActuatorState> = new Map();

  for (const region of config.regions) {
    // Find nearest obstacle in this region
    const regionObstacles = scene.obstacles.filter(o =>
      isInRegion(o.direction, o.elevation, region)
    );

    if (regionObstacles.length === 0) continue;

    const nearest = regionObstacles.reduce((a, b) =>
      a.distance < b.distance ? a : b
    );

    // Encode distance to intensity
    const intensity = encodeDistanceToIntensity(nearest.distance);

    // Activate region's actuators
    for (const location of region.assignedActuators) {
      activations.set(location, {
        intensity,
        frequency: getFrequencyForObstacle(nearest),
        waveform: getWaveformForType(nearest.type),
      });
    }
  }

  return activations;
}
```

### 4.3 Frequency Division Multiplexing (FDM)

Encode different objects with different frequencies:

```typescript
interface FDMEncoding {
  strategy: 'frequency_division';

  // Frequency channels
  channels: FrequencyChannel[];

  // Assignment strategy
  channelAssignment: 'by_type' | 'by_priority' | 'by_direction';
}

interface FrequencyChannel {
  name: string;
  centerFrequency: number;
  bandwidth: number;
}

const FREQUENCY_CHANNELS: FrequencyChannel[] = [
  { name: 'urgent',   centerFrequency: 200, bandwidth: 50 },  // High freq = urgent
  { name: 'warning',  centerFrequency: 150, bandwidth: 40 },
  { name: 'caution',  centerFrequency: 100, bandwidth: 30 },
  { name: 'info',     centerFrequency: 60,  bandwidth: 20 },  // Low freq = info
];

function encodeFDM(
  scene: SpatialScene,
  config: FDMEncoding
): HapticLayers {
  const layers: HapticLayer[] = [];

  // Group obstacles by priority
  const groups = groupByPriority(scene.obstacles);

  for (const [priority, obstacles] of Object.entries(groups)) {
    const channel = FREQUENCY_CHANNELS[priority];
    if (!channel) continue;

    // Nearest in this priority group
    const nearest = obstacles[0];

    layers.push({
      channel: channel.name,
      frequency: channel.centerFrequency,
      intensity: encodeDistanceToIntensity(nearest.distance),
      direction: nearest.direction,
      modulation: {
        type: 'pulse',
        rate: 1 + (1 - nearest.distance / 10) * 10,  // 1-11 Hz
      },
    });
  }

  return {
    layers,
    mixingStrategy: 'additive',
  };
}
```

### 4.4 Hybrid Encoding (Recommended)

Combine strategies based on scene complexity:

```typescript
interface HybridEncoding {
  strategy: 'hybrid';

  // Primary channel: most urgent/nearest
  primaryStrategy: 'continuous';

  // Secondary channel: overview
  secondaryStrategy: 'time_division';

  // Blending
  primaryWeight: number;   // 0-1
  secondaryWeight: number; // 0-1
}

function encodeHybrid(
  scene: SpatialScene,
  config: HybridEncoding
): HapticOutput {
  // Find most critical obstacle
  const critical = findMostCritical(scene.obstacles);

  // Primary: Continuous feedback for critical obstacle
  const primary = critical ? {
    direction: encodeDirection(critical.direction),
    distance: encodeDistance(critical.distance),
    continuous: true,
    intensity: config.primaryWeight,
  } : null;

  // Secondary: Time-divided overview of other obstacles
  const others = scene.obstacles.filter(o => o.id !== critical?.id);
  const secondary = encodeTDM(
    { ...scene, obstacles: others },
    { ...DEFAULT_TDM_CONFIG, maxObjects: 3 }
  );

  return {
    primary,
    secondary,
    mixing: 'layered',
  };
}
```

## 5. Scene Prioritization

### 5.1 Priority Calculation

```typescript
interface PriorityFactors {
  distance: number;       // Closer = higher priority
  velocity: number;       // Approaching = higher priority
  type: number;          // Hazardous types = higher priority
  inPath: number;        // In navigation path = higher priority
  novelty: number;       // New objects = higher priority
}

function calculatePriority(
  obstacle: Obstacle,
  context: NavigationContext
): number {
  let priority = 0;

  // Distance factor (exponential increase as closer)
  const distanceFactor = Math.pow(1 - Math.min(obstacle.distance / 10, 1), 2);
  priority += distanceFactor * 0.3;

  // Velocity factor (approaching objects)
  if (obstacle.velocity) {
    const approachSpeed = -dotProduct(
      obstacle.velocity,
      directionToVector(obstacle.direction)
    );
    if (approachSpeed > 0) {
      priority += Math.min(approachSpeed / 5, 1) * 0.25;
    }
  }

  // Type factor
  priority += TYPE_PRIORITY[obstacle.type] * 0.2;

  // In-path factor
  if (context.destination) {
    const pathDeviation = angleDifference(
      obstacle.direction,
      context.destination.azimuth
    );
    if (Math.abs(pathDeviation) < 30) {
      priority += 0.15;
    }
  }

  // Novelty factor (new objects get attention)
  const age = Date.now() - obstacle.lastSeen;
  if (age < 1000) {
    priority += 0.1 * (1 - age / 1000);
  }

  return Math.min(priority, 1);
}

const TYPE_PRIORITY: Record<ObstacleType, number> = {
  drop: 1.0,
  vehicle: 0.9,
  cyclist: 0.8,
  person: 0.6,
  stairs_down: 0.7,
  stairs_up: 0.5,
  wall: 0.3,
  furniture: 0.3,
  doorway: 0.2,  // Opportunity
  unknown: 0.5,
  // ...
};
```

### 5.2 Attention Management

```typescript
interface AttentionManager {
  // Currently attended objects
  activeAttention: Set<string>;

  // Attention cooldown (prevent rapid switching)
  cooldownMs: number;

  // Maximum simultaneous attention
  maxAttention: number;
}

function updateAttention(
  scene: SpatialScene,
  manager: AttentionManager
): Obstacle[] {
  const prioritized = scene.obstacles
    .map(o => ({ ...o, priority: calculatePriority(o, scene.navigation) }))
    .sort((a, b) => b.priority - a.priority);

  // Select top obstacles, respecting cooldown
  const selected: Obstacle[] = [];
  for (const obstacle of prioritized) {
    if (selected.length >= manager.maxAttention) break;

    // Check if can switch attention
    if (!manager.activeAttention.has(obstacle.id)) {
      // New object - check cooldown
      // ... cooldown logic
    }

    selected.push(obstacle);
    manager.activeAttention.add(obstacle.id);
  }

  // Clear stale attention
  for (const id of manager.activeAttention) {
    if (!selected.find(o => o.id === id)) {
      manager.activeAttention.delete(id);
    }
  }

  return selected;
}
```

## 6. Scene Transitions

### 6.1 Smooth Object Transitions

When objects appear, disappear, or change:

```typescript
interface TransitionConfig {
  fadeInDuration: number;   // ms for new objects
  fadeOutDuration: number;  // ms for removed objects
  morphDuration: number;    // ms for position changes
  interpolation: 'linear' | 'ease_in_out' | 'ease_out';
}

function transitionScenes(
  previousScene: SpatialScene,
  currentScene: SpatialScene,
  config: TransitionConfig
): TransitionPlan {
  const plan: TransitionPlan = {
    added: [],
    removed: [],
    moved: [],
    unchanged: [],
  };

  // Find matching objects
  const previousIds = new Set(previousScene.obstacles.map(o => o.id));
  const currentIds = new Set(currentScene.obstacles.map(o => o.id));

  for (const obstacle of currentScene.obstacles) {
    if (!previousIds.has(obstacle.id)) {
      // New object - fade in
      plan.added.push({
        obstacle,
        transition: 'fade_in',
        duration: config.fadeInDuration,
      });
    } else {
      // Existing object - check for movement
      const previous = previousScene.obstacles.find(o => o.id === obstacle.id)!;
      const moved = hasSignificantMovement(previous, obstacle);

      if (moved) {
        plan.moved.push({
          from: previous,
          to: obstacle,
          transition: 'morph',
          duration: config.morphDuration,
        });
      } else {
        plan.unchanged.push(obstacle);
      }
    }
  }

  for (const obstacle of previousScene.obstacles) {
    if (!currentIds.has(obstacle.id)) {
      // Removed object - fade out
      plan.removed.push({
        obstacle,
        transition: 'fade_out',
        duration: config.fadeOutDuration,
      });
    }
  }

  return plan;
}
```

### 6.2 Scene Change Indicators

Alert user to significant scene changes:

```typescript
const SCENE_CHANGE_PATTERNS = {
  // New obstacle appeared nearby
  newObstacleNear: {
    pattern: 'attention_pulse',
    intensity: 0.8,
    repetitions: 2,
    direction: 'toward_object',
  },

  // Obstacle disappeared (path cleared)
  obstacleCleared: {
    pattern: 'release_sweep',
    intensity: 0.5,
    direction: 'away_from_location',
  },

  // Entering new space (doorway, room)
  newSpace: {
    pattern: 'expanding_pulse',
    intensity: 0.6,
  },

  // Approaching destination
  approachingDestination: {
    pattern: 'converging_pulse',
    intensity: 0.7,
    repetitions: 3,
  },
};
```

## 7. Environment Encoding

### 7.1 Room/Space Shape

Convey general spatial layout:

```typescript
interface SpaceShape {
  type: 'open' | 'corridor' | 'room' | 'intersection' | 'dead_end';
  width?: number;
  length?: number;
  openings: SpatialDirection[];
}

function encodeSpaceShape(shape: SpaceShape): HapticPattern {
  switch (shape.type) {
    case 'corridor':
      return {
        description: 'Parallel walls on sides',
        left: { continuous: true, intensity: 0.3 },
        right: { continuous: true, intensity: 0.3 },
        front: { continuous: false },
        back: { continuous: false },
      };

    case 'room':
      return {
        description: 'Enclosed space, boundaries all around',
        pattern: 'boundary_sweep',
        duration: 500,
        direction: 'clockwise',
      };

    case 'open':
      return {
        description: 'Open space, minimal boundaries',
        pattern: 'gentle_ambient',
        intensity: 0.1,
      };

    case 'intersection':
      return {
        description: 'Multiple paths available',
        pattern: 'multi_directional_pulse',
        directions: shape.openings.map(o => o.azimuth),
      };

    case 'dead_end':
      return {
        description: 'No forward path',
        pattern: 'blocking_wall',
        direction: 'front',
        intensity: 0.6,
      };
  }
}
```

### 7.2 Ground Surface

Convey terrain information:

```typescript
interface GroundSurface {
  type: 'smooth' | 'rough' | 'uneven' | 'stairs' | 'ramp' | 'transition';
  slope?: number;     // degrees
  direction?: number; // direction of slope
}

function encodeGroundSurface(surface: GroundSurface): HapticPattern {
  const patterns: Record<string, HapticPattern> = {
    smooth: {
      texture: 'none',
      intensity: 0,
    },
    rough: {
      texture: 'noise',
      noiseType: 'pink',
      intensity: 0.3,
      frequency: 80,
    },
    uneven: {
      texture: 'random_pulse',
      intensity: 0.4,
      variability: 0.3,
    },
    stairs: {
      texture: 'stepped',
      pattern: surface.slope > 0 ? 'ascending' : 'descending',
      intensity: 0.5,
    },
    ramp: {
      texture: 'continuous_slope',
      direction: surface.direction,
      intensity: Math.abs(surface.slope) / 30,
    },
    transition: {
      texture: 'boundary_marker',
      intensity: 0.4,
      duration: 100,
    },
  };

  return patterns[surface.type];
}
```

## 8. Navigation Integration

### 8.1 Path Guidance Combined with Scene

```typescript
interface NavigationScene extends SpatialScene {
  navigation: {
    destination: SpatialDirection;
    nextWaypoint: SpatialDirection;
    path: Vector3[];
    pathClear: boolean;
    distanceToDestination: number;
    estimatedTime: number;
  };
}

function encodeNavigationScene(
  scene: NavigationScene
): HapticOutput {
  const output: HapticOutput = {
    layers: [],
  };

  // Layer 1: Navigation guidance (low intensity, continuous)
  if (scene.navigation.pathClear) {
    output.layers.push({
      name: 'navigation',
      priority: 0.3,
      encoding: encodeDirection(scene.navigation.nextWaypoint.azimuth),
      intensity: 0.3,
      continuous: true,
    });
  }

  // Layer 2: Obstacles (higher priority when near)
  const criticalObstacles = scene.obstacles.filter(o => o.distance < 3);
  if (criticalObstacles.length > 0) {
    output.layers.push({
      name: 'obstacles',
      priority: 0.8,
      encoding: encodeHybrid(scene, DEFAULT_HYBRID_CONFIG),
      override: true,  // Override navigation when obstacles are critical
    });
  }

  // Layer 3: Contextual (environment awareness)
  if (scene.features.length > 0) {
    output.layers.push({
      name: 'context',
      priority: 0.2,
      encoding: encodeEnvironmentFeatures(scene.features),
      intermittent: true,
      interval: 3000,
    });
  }

  return output;
}
```

### 8.2 Arrival Indication

```typescript
function checkAndEncodeArrival(
  scene: NavigationScene
): HapticPattern | null {
  const distanceToDestination = scene.navigation.distanceToDestination;

  if (distanceToDestination < 1.0) {
    // Very close - arrival imminent
    return {
      pattern: 'arrival_celebration',
      steps: [
        { intensity: 0.6, duration: 100, location: 'all' },
        { intensity: 0.8, duration: 100, location: 'all' },
        { intensity: 1.0, duration: 200, location: 'center' },
      ],
    };
  }

  if (distanceToDestination < 3.0) {
    // Approaching
    const proximity = 1 - distanceToDestination / 3;
    return {
      pattern: 'approaching_destination',
      direction: scene.navigation.destination.azimuth,
      intensity: 0.5 + proximity * 0.3,
      pulseRate: 2 + proximity * 4,
    };
  }

  return null;
}
```

## 9. Implementation Reference

### 9.1 Complete Scene Encoder Interface

```typescript
interface SpatialSceneEncoder {
  // Configuration
  encodingStrategy: EncodingStrategy;
  maxObjects: number;
  updateRate: number;  // Hz

  // Encode full scene
  encode(scene: SpatialScene): HapticOutput;

  // Incremental update
  update(previousScene: SpatialScene, currentScene: SpatialScene): HapticOutput;

  // Priority management
  setPriorityConfig(config: PriorityConfig): void;

  // Strategy selection
  setStrategy(strategy: EncodingStrategy): void;
  selectStrategyForScene(scene: SpatialScene): EncodingStrategy;
}
```

### 9.2 Usage Example

```typescript
const encoder = new SpatialSceneEncoder({
  encodingStrategy: 'hybrid',
  maxObjects: 5,
  updateRate: 10,  // 10 Hz
});

// Process incoming scene data
function processScene(sceneData: SensorData): void {
  const scene = parseScene(sceneData);
  const hapticOutput = encoder.encode(scene);

  // Send to haptic device
  hapticDevice.play(hapticOutput);
}
```

---

## Appendix A: Scene Encoding Quick Reference

```
╔═══════════════════════════════════════════════════════════════════════╗
║                 SPATIAL SCENE ENCODING REFERENCE                      ║
╠═══════════════════════════════════════════════════════════════════════╣
║                                                                       ║
║  ENCODING STRATEGIES                                                  ║
║  ───────────────────                                                  ║
║  TDM (Time Division):  Objects shown sequentially in time            ║
║  SDM (Spatial Division): Different actuators for different regions   ║
║  FDM (Frequency Division): Different frequencies for object types    ║
║  Hybrid (Recommended): Continuous critical + TDM overview            ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║  PRIORITY FACTORS                                                     ║
║  ────────────────                                                     ║
║  Distance (30%)    │ Closer = Higher priority                        ║
║  Velocity (25%)    │ Approaching = Higher priority                   ║
║  Type (20%)        │ drop > vehicle > person > wall                  ║
║  In Path (15%)     │ In navigation path = Higher priority            ║
║  Novelty (10%)     │ New objects get attention                       ║
║                                                                       ║
╠═══════════════════════════════════════════════════════════════════════╣
║  LAYERED OUTPUT                                                       ║
║  ──────────────                                                       ║
║  Layer 1: Navigation (continuous, low intensity)                     ║
║  Layer 2: Obstacles (priority-based, dominant when critical)         ║
║  Layer 3: Context (intermittent environment awareness)               ║
║                                                                       ║
╚═══════════════════════════════════════════════════════════════════════╝
```
