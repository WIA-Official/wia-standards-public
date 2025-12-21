# WIA Pet Emotion API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #F59E0B (Amber)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [Emotion Detection API](#emotion-detection-api)
5. [Sensor Integration API](#sensor-integration-api)
6. [Analysis API](#analysis-api)
7. [Event System](#event-system)
8. [Configuration](#configuration)
9. [Error Handling](#error-handling)
10. [Usage Examples](#usage-examples)

---

## 1. Overview

### 1.1 Purpose

WIA Pet Emotion API Interface Standard defines the programmatic interface for detecting, recording, and analyzing pet emotional states. This Phase 2 specification builds upon the Phase 1 data format to provide language-agnostic APIs for real-time emotion monitoring.

**Core Goals**:
- Unified API for all emotion detection sources
- Real-time emotion streaming
- Multi-modal sensor integration
- Event-driven architecture
- TypeScript/Python/REST implementations

### 1.2 Scope

| Component | Description |
|-----------|-------------|
| **Core API** | Main PetEmotion class interface |
| **Event System** | Real-time emotion event streaming |
| **Sensor Adapters** | Wearable, camera, microphone integration |
| **Analysis Tools** | Temporal, social, pattern analysis |
| **ML Integration** | Model deployment and inference |

### 1.3 Phase 1 Compatibility

Phase 2 API fully implements Phase 1 data formats:

```
Phase 1: Data Format (JSON structures)
    ↓
Phase 2: API Interface (programming interface)
    ↓
Phase 3: Communication Protocol (network transport)
    ↓
Phase 4: Integration (ecosystem connectivity)
```

---

## 2. Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **PetEmotion** | Main API class (entry point) |
| **EmotionDetector** | Component that analyzes inputs for emotions |
| **SensorAdapter** | Interface to physical/virtual sensors |
| **EmotionEvent** | Real-time emotion state change notification |
| **EmotionStream** | Continuous flow of emotion data |
| **EmotionAnalyzer** | Pattern and trend analysis component |

### 2.2 Event Types

| Event | Description | Data |
|-------|-------------|------|
| `emotion_detected` | New emotion state detected | EmotionDetectionOutput |
| `emotion_changed` | Emotion transition occurred | EmotionTransition |
| `stress_alert` | High stress level detected | StressAlert |
| `social_interaction` | Multi-pet interaction event | SocialInteractionEvent |
| `pattern_detected` | Behavioral pattern recognized | BehavioralPattern |
| `sensor_data` | Raw sensor data received | WearableSensorData |
| `error` | Error occurred | WiaError |

---

## 3. Core Interfaces

### 3.1 PetEmotion Class

The main API entry point for emotion detection and monitoring.

#### TypeScript

```typescript
class PetEmotion {
  // Constructor
  constructor(options?: PetEmotionOptions);

  // Lifecycle
  initialize(): Promise<void>;
  shutdown(): Promise<void>;
  isInitialized(): boolean;

  // Pet Management
  registerPet(pet: PetProfile): Promise<string>;
  updatePet(petId: string, updates: Partial<PetProfile>): Promise<void>;
  removePet(petId: string): Promise<void>;
  getPet(petId: string): Promise<PetProfile | null>;
  listPets(): Promise<PetProfile[]>;

  // Emotion Detection
  detectEmotion(petId: string, inputs: EmotionInputs): Promise<EmotionDetectionResult>;
  startContinuousDetection(petId: string, options?: ContinuousOptions): Promise<void>;
  stopContinuousDetection(petId: string): Promise<void>;

  // State Query
  getCurrentEmotion(petId: string): Promise<EmotionState | null>;
  getEmotionHistory(petId: string, options: HistoryOptions): Promise<EmotionTimeline>;
  getEmotionSummary(petId: string, period: TimePeriod): Promise<EmotionSummary>;

  // Event System
  on<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;

  // Sensor Management
  addSensor(petId: string, sensor: ISensorAdapter): Promise<void>;
  removeSensor(petId: string, sensorId: string): Promise<void>;
  listSensors(petId: string): Promise<SensorInfo[]>;

  // Analysis
  analyzePattern(petId: string, options: AnalysisOptions): Promise<PatternAnalysis>;
  detectAnomaly(petId: string, baseline: EmotionBaseline): Promise<AnomalyReport>;
  comparePets(petIds: string[]): Promise<ComparisonReport>;

  // Configuration
  configure(options: Partial<PetEmotionOptions>): void;
  getConfig(): PetEmotionOptions;
}
```

#### Python

```python
class PetEmotion:
    def __init__(self, options: Optional[PetEmotionOptions] = None):
        ...

    # Lifecycle
    async def initialize(self) -> None: ...
    async def shutdown(self) -> None: ...
    def is_initialized(self) -> bool: ...

    # Pet Management
    async def register_pet(self, pet: PetProfile) -> str: ...
    async def update_pet(self, pet_id: str, updates: Dict[str, Any]) -> None: ...
    async def remove_pet(self, pet_id: str) -> None: ...
    async def get_pet(self, pet_id: str) -> Optional[PetProfile]: ...
    async def list_pets(self) -> List[PetProfile]: ...

    # Emotion Detection
    async def detect_emotion(self, pet_id: str, inputs: EmotionInputs) -> EmotionDetectionResult: ...
    async def start_continuous_detection(self, pet_id: str, options: Optional[ContinuousOptions] = None) -> None: ...
    async def stop_continuous_detection(self, pet_id: str) -> None: ...

    # State Query
    async def get_current_emotion(self, pet_id: str) -> Optional[EmotionState]: ...
    async def get_emotion_history(self, pet_id: str, options: HistoryOptions) -> EmotionTimeline: ...
    async def get_emotion_summary(self, pet_id: str, period: TimePeriod) -> EmotionSummary: ...

    # Event System
    def on(self, event: EmotionEventType, handler: EventHandler) -> None: ...
    def off(self, event: EmotionEventType, handler: EventHandler) -> None: ...
    def once(self, event: EmotionEventType, handler: EventHandler) -> None: ...

    # Sensor Management
    async def add_sensor(self, pet_id: str, sensor: ISensorAdapter) -> None: ...
    async def remove_sensor(self, pet_id: str, sensor_id: str) -> None: ...
    async def list_sensors(self, pet_id: str) -> List[SensorInfo]: ...

    # Analysis
    async def analyze_pattern(self, pet_id: str, options: AnalysisOptions) -> PatternAnalysis: ...
    async def detect_anomaly(self, pet_id: str, baseline: EmotionBaseline) -> AnomalyReport: ...
    async def compare_pets(self, pet_ids: List[str]) -> ComparisonReport: ...

    # Configuration
    def configure(self, options: PetEmotionOptions) -> None: ...
    def get_config(self) -> PetEmotionOptions: ...
```

### 3.2 PetEmotionOptions

```typescript
interface PetEmotionOptions {
  // Detection Settings
  detection: {
    mode: 'realtime' | 'batch' | 'on_demand';
    frequency: number;           // Hz for continuous detection
    confidence_threshold: number; // 0.0-1.0
    multi_modal_fusion: boolean;
  };

  // Storage
  storage: {
    enabled: boolean;
    backend: 'memory' | 'sqlite' | 'postgresql' | 'mongodb' | 'custom';
    retention_days: number;
    compression: boolean;
  };

  // AI/ML Models
  models: {
    primary_model: string;
    fallback_model?: string;
    ensemble: boolean;
    gpu_acceleration: boolean;
  };

  // Sensors
  sensors: {
    auto_discover: boolean;
    preferred_types: SensorType[];
    sampling_rate: number;       // Hz
  };

  // Events
  events: {
    emit_all: boolean;
    buffer_size: number;
    async_handlers: boolean;
  };

  // Privacy
  privacy: {
    anonymize: boolean;
    encrypt_storage: boolean;
    data_sharing: 'none' | 'aggregated' | 'full';
  };

  // Logging
  logging: {
    level: 'debug' | 'info' | 'warn' | 'error';
    destination: 'console' | 'file' | 'remote';
  };
}
```

### 3.3 PetProfile

```typescript
interface PetProfile {
  petId: string;
  name: string;
  species: PetSpecies;
  breed?: string;
  dateOfBirth: Date;
  sex: 'male' | 'female' | 'neutered_male' | 'spayed_female';

  // Behavioral baseline
  baseline?: {
    typical_emotions: CoreEmotion[];
    average_valence: number;
    average_arousal: number;
    stress_threshold: number;
  };

  // Health context
  health?: {
    conditions: string[];
    medications: string[];
    allergies: string[];
  };

  // Preferences
  preferences?: {
    detection_sensitivity: 'low' | 'medium' | 'high';
    alert_threshold: number;
    notification_methods: string[];
  };

  // Metadata
  created_at: Date;
  updated_at: Date;
}
```

---

## 4. Emotion Detection API

### 4.1 Single Detection

Detect emotion from a single set of inputs.

```typescript
interface EmotionInputs {
  // Visual input
  image?: {
    data: Buffer | Blob | string;  // raw, blob, or base64
    format: 'jpeg' | 'png' | 'raw';
    width: number;
    height: number;
    timestamp: Date;
  };

  // Video input
  video?: {
    frames: ImageFrame[];
    fps: number;
    duration: number;
  };

  // Audio input
  audio?: {
    data: Buffer | Blob;
    format: 'wav' | 'mp3' | 'raw';
    sample_rate: number;
    channels: number;
    duration: number;
  };

  // Sensor data
  sensors?: {
    accelerometer?: AccelerometerData;
    heart_rate?: number;
    temperature?: number;
    gps?: GPSData;
  };

  // Manual observation
  observation?: {
    body_language: Partial<BodyLanguage>;
    behaviors: string[];
    notes: string;
  };

  // Context
  context?: {
    location: string;
    social_setting: 'alone' | 'with_humans' | 'with_pets';
    activity: string;
  };
}

interface EmotionDetectionResult {
  petId: string;
  timestamp: Date;

  // Primary result
  emotion: CoreEmotion;
  intensity: EmotionIntensity;
  confidence: number;

  // Dimensional model
  dimensions: EmotionDimensions;

  // All predictions
  predictions: {
    emotion: CoreEmotion;
    probability: number;
    confidence: number;
  }[];

  // Contributing factors
  factors: {
    source: 'visual' | 'audio' | 'sensor' | 'observation';
    contribution: number;
    confidence: number;
  }[];

  // Model info
  model: {
    id: string;
    version: string;
    processing_time: number;
  };

  // Quality metrics
  quality: {
    input_quality: number;
    detection_reliability: number;
    data_completeness: number;
  };
}
```

### 4.2 Continuous Detection

Stream emotion detection continuously.

```typescript
interface ContinuousOptions {
  frequency: number;            // Hz (1-60)
  sources: ('camera' | 'microphone' | 'wearable')[];

  // Filters
  min_confidence: number;
  emotion_filter?: CoreEmotion[];

  // Optimization
  low_power_mode: boolean;
  batch_processing: boolean;

  // Alerts
  alerts: {
    enabled: boolean;
    conditions: AlertCondition[];
  };
}

interface AlertCondition {
  type: 'emotion_change' | 'high_stress' | 'prolonged_state' | 'anomaly';

  // Emotion-specific
  target_emotion?: CoreEmotion;
  min_intensity?: number;
  min_duration?: number;       // milliseconds

  // Thresholds
  stress_threshold?: number;
  arousal_threshold?: number;

  // Action
  callback?: (alert: EmotionAlert) => void;
  notification?: boolean;
}
```

### 4.3 Batch Detection

Process multiple inputs in batch mode.

```typescript
interface BatchDetectionRequest {
  petId: string;
  inputs: EmotionInputs[];
  options?: {
    parallel: boolean;
    max_concurrency: number;
    return_on_error: boolean;
  };
}

interface BatchDetectionResult {
  total: number;
  successful: number;
  failed: number;

  results: (EmotionDetectionResult | Error)[];

  // Aggregated insights
  aggregate?: {
    dominant_emotion: CoreEmotion;
    average_valence: number;
    average_arousal: number;
    emotion_distribution: Record<CoreEmotion, number>;
  };

  // Performance
  total_time: number;
  average_time_per_input: number;
}

// API Methods
async function detectEmotionBatch(
  request: BatchDetectionRequest
): Promise<BatchDetectionResult>;
```

---

## 5. Sensor Integration API

### 5.1 Sensor Adapter Interface

```typescript
interface ISensorAdapter {
  // Properties
  readonly sensorId: string;
  readonly sensorType: SensorType;
  readonly manufacturer: string;
  readonly model: string;
  readonly isConnected: boolean;

  // Lifecycle
  connect(config: SensorConfig): Promise<void>;
  disconnect(): Promise<void>;
  calibrate(): Promise<void>;

  // Data acquisition
  startStreaming(options?: StreamOptions): Promise<void>;
  stopStreaming(): Promise<void>;
  getSample(): Promise<SensorSample>;

  // Configuration
  configure(settings: SensorSettings): Promise<void>;
  getCapabilities(): SensorCapabilities;

  // Events
  on(event: SensorEvent, handler: SensorEventHandler): void;
  off(event: SensorEvent, handler: SensorEventHandler): void;

  // Status
  getStatus(): SensorStatus;
  getBatteryLevel?(): Promise<number>;
}
```

### 5.2 Built-in Sensor Adapters

#### Camera Adapter

```typescript
class CameraAdapter implements ISensorAdapter {
  sensorType = SensorType.CAMERA;

  // Camera-specific methods
  captureFrame(): Promise<ImageFrame>;
  captureVideo(duration: number): Promise<VideoClip>;
  setResolution(width: number, height: number): void;
  setFrameRate(fps: number): void;
  enableFaceTracking(enabled: boolean): void;

  // Advanced features
  detectBodyPose(): Promise<BodyPose>;
  trackMovement(): AsyncIterator<MovementData>;
}
```

#### Microphone Adapter

```typescript
class MicrophoneAdapter implements ISensorAdapter {
  sensorType = SensorType.MICROPHONE;

  // Audio-specific methods
  recordAudio(duration: number): Promise<AudioBuffer>;
  detectVocalization(): Promise<VocalizationEvent | null>;
  analyzeAcoustics(audio: AudioBuffer): Promise<AcousticFeatures>;

  // Real-time analysis
  streamVocalizationDetection(): AsyncIterator<VocalizationEvent>;
  getNoiseLevel(): Promise<number>;
}
```

#### Wearable Adapter

```typescript
class WearableAdapter implements ISensorAdapter {
  sensorType = SensorType.WEARABLE;

  // Wearable-specific methods
  getHeartRate(): Promise<number>;
  getActivityLevel(): Promise<number>;
  getLocation(): Promise<GPSCoordinates>;
  getTemperature(): Promise<number>;

  // Continuous monitoring
  streamVitalSigns(): AsyncIterator<VitalSigns>;
  streamActivityData(): AsyncIterator<ActivityData>;

  // Device management
  syncData(): Promise<void>;
  getBatteryLevel(): Promise<number>;
}
```

### 5.3 Multi-Sensor Fusion

```typescript
class SensorFusion {
  // Add sensors
  addSensor(sensor: ISensorAdapter): void;
  removeSensor(sensorId: string): void;

  // Fusion methods
  setFusionStrategy(strategy: FusionStrategy): void;
  setSensorWeights(weights: Record<string, number>): void;

  // Synchronized capture
  captureSynchronized(): Promise<MultiModalData>;

  // Fused emotion detection
  detectEmotionFused(data: MultiModalData): Promise<EmotionDetectionResult>;

  // Calibration
  calibrateSensors(): Promise<void>;
  alignTimestamps(tolerance: number): void;
}

enum FusionStrategy {
  EARLY = 'early',         // Combine raw data
  LATE = 'late',           // Combine predictions
  HYBRID = 'hybrid',       // Mixed approach
  ATTENTION = 'attention'  // Attention-based weighting
}
```

---

## 6. Analysis API

### 6.1 Pattern Analysis

```typescript
interface PatternAnalyzer {
  // Detect patterns
  detectPatterns(
    petId: string,
    timeRange: TimeRange,
    options?: PatternOptions
  ): Promise<DetectedPattern[]>;

  // Specific pattern types
  detectDailyRoutine(petId: string): Promise<DailyRoutine>;
  detectEmotionalTriggers(petId: string): Promise<TriggerAnalysis>;
  detectStressPatterns(petId: string): Promise<StressPattern[]>;

  // Anomaly detection
  detectAnomalies(
    petId: string,
    baseline: EmotionBaseline
  ): Promise<Anomaly[]>;
}

interface DetectedPattern {
  patternId: string;
  type: PatternType;
  confidence: number;

  // Temporal
  occurrence_times: Date[];
  frequency: string;
  duration: number;

  // Description
  description: string;
  triggers: string[];
  associated_emotions: CoreEmotion[];

  // Statistical
  significance: number;
  correlation: number;
}

interface DailyRoutine {
  petId: string;
  analyzed_period: TimeRange;

  // Time blocks
  routine_blocks: {
    time_range: string;      // "08:00-10:00"
    typical_emotion: CoreEmotion;
    typical_activity: string;
    confidence: number;
  }[];

  // Variations
  weekday_weekend_difference: number;
  consistency_score: number;
}
```

### 6.2 Temporal Analysis

```typescript
interface TemporalAnalyzer {
  // Trends
  analyzeTrend(
    petId: string,
    emotion: CoreEmotion,
    period: TimePeriod
  ): Promise<EmotionTrend>;

  // Time series
  getEmotionTimeSeries(
    petId: string,
    timeRange: TimeRange,
    resolution: number
  ): Promise<EmotionTimeSeries>;

  // Transitions
  analyzeTransitions(
    petId: string,
    timeRange: TimeRange
  ): Promise<TransitionAnalysis>;

  // Forecasting
  forecastEmotion(
    petId: string,
    horizon: number
  ): Promise<EmotionForecast>;
}

interface EmotionTrend {
  emotion: CoreEmotion;
  direction: 'increasing' | 'decreasing' | 'stable';
  rate_of_change: number;
  confidence: number;

  // Data points
  data_points: {
    timestamp: Date;
    value: number;
  }[];

  // Statistics
  mean: number;
  variance: number;
  trend_line: {
    slope: number;
    intercept: number;
    r_squared: number;
  };
}

interface TransitionAnalysis {
  total_transitions: number;
  average_duration: number;

  // Transition matrix
  transitions: {
    from: CoreEmotion;
    to: CoreEmotion;
    count: number;
    probability: number;
    average_duration: number;
  }[];

  // Patterns
  common_sequences: {
    sequence: CoreEmotion[];
    frequency: number;
  }[];
}
```

### 6.3 Social Analysis

```typescript
interface SocialAnalyzer {
  // Multi-pet analysis
  analyzeSocialDynamics(
    petIds: string[],
    timeRange: TimeRange
  ): Promise<SocialDynamicsReport>;

  // Interactions
  detectInteractions(
    petIds: string[],
    options?: InteractionOptions
  ): Promise<SocialInteractionEvent[]>;

  // Relationships
  analyzeRelationships(
    petIds: string[]
  ): Promise<RelationshipGraph>;

  // Emotional contagion
  detectEmotionalContagion(
    sourcePetId: string,
    targetPetIds: string[]
  ): Promise<ContagionAnalysis>;
}

interface SocialDynamicsReport {
  group_id: string;
  analyzed_period: TimeRange;

  // Group metrics
  group_cohesion: number;
  interaction_frequency: number;
  conflict_rate: number;

  // Individual roles
  social_roles: {
    petId: string;
    role: 'leader' | 'follower' | 'independent' | 'mediator';
    confidence: number;
  }[];

  // Interaction patterns
  interaction_matrix: number[][];
  preferred_pairs: {
    pet1: string;
    pet2: string;
    affinity: number;
  }[];
}
```

### 6.4 Comparative Analysis

```typescript
interface ComparativeAnalyzer {
  // Compare pets
  comparePets(
    petIds: string[],
    metrics: ComparisonMetric[]
  ): Promise<ComparisonResult>;

  // Compare to baseline
  compareToBaseline(
    petId: string,
    baseline: EmotionBaseline
  ): Promise<BaselineComparison>;

  // Compare time periods
  compareTimePeriods(
    petId: string,
    period1: TimeRange,
    period2: TimeRange
  ): Promise<PeriodComparison>;

  // Breed/species norms
  compareToNorm(
    petId: string,
    normType: 'breed' | 'species' | 'age'
  ): Promise<NormComparison>;
}

interface ComparisonResult {
  pets: {
    petId: string;
    name: string;
  }[];

  metrics: {
    metric: ComparisonMetric;
    values: Record<string, number>;
    ranking: string[];
    significance: number;
  }[];

  // Overall similarity
  similarity_matrix: number[][];
  clusters?: {
    cluster_id: number;
    pet_ids: string[];
    centroid: Record<string, number>;
  }[];
}

enum ComparisonMetric {
  AVERAGE_VALENCE = 'average_valence',
  AVERAGE_AROUSAL = 'average_arousal',
  STRESS_LEVEL = 'stress_level',
  EMOTIONAL_STABILITY = 'emotional_stability',
  ACTIVITY_LEVEL = 'activity_level',
  SOCIAL_ENGAGEMENT = 'social_engagement'
}
```

---

## 7. Event System

### 7.1 Event Types

```typescript
// Emotion events
interface EmotionDetectedEvent {
  type: 'emotion_detected';
  petId: string;
  timestamp: Date;
  result: EmotionDetectionResult;
}

interface EmotionChangedEvent {
  type: 'emotion_changed';
  petId: string;
  timestamp: Date;
  previous: EmotionState;
  current: EmotionState;
  transition_duration: number;
}

interface StressAlertEvent {
  type: 'stress_alert';
  petId: string;
  timestamp: Date;
  stress_level: number;
  duration: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  recommended_action: string;
}

// Social events
interface SocialInteractionEvent {
  type: 'social_interaction';
  timestamp: Date;
  participants: string[];
  interaction_type: InteractionType;
  quality: InteractionQuality;
  duration: number;
}

// Sensor events
interface SensorDataEvent {
  type: 'sensor_data';
  petId: string;
  sensorId: string;
  timestamp: Date;
  data: SensorSample;
}

interface SensorErrorEvent {
  type: 'sensor_error';
  petId: string;
  sensorId: string;
  timestamp: Date;
  error: Error;
}

// Pattern events
interface PatternDetectedEvent {
  type: 'pattern_detected';
  petId: string;
  timestamp: Date;
  pattern: DetectedPattern;
}

interface AnomalyDetectedEvent {
  type: 'anomaly_detected';
  petId: string;
  timestamp: Date;
  anomaly: Anomaly;
  severity: number;
}
```

### 7.2 Event Handlers

```typescript
// Event handler types
type EventHandler<T extends EmotionEvent> = (event: T) => void | Promise<void>;

// Subscribe to events
petEmotion.on('emotion_detected', (event: EmotionDetectedEvent) => {
  console.log(`${event.petId}: ${event.result.emotion}`);
});

petEmotion.on('stress_alert', (event: StressAlertEvent) => {
  if (event.severity === 'critical') {
    notifyOwner(event);
  }
});

// Filtered subscriptions
petEmotion.on('emotion_changed', (event: EmotionChangedEvent) => {
  // Only handle specific emotions
  if (event.current.emotion === 'anxious') {
    handleAnxiety(event);
  }
});

// One-time handlers
petEmotion.once('pattern_detected', (event: PatternDetectedEvent) => {
  console.log('First pattern detected:', event.pattern);
});
```

### 7.3 Event Filtering

```typescript
interface EventFilter<T extends EmotionEvent> {
  // Pet filter
  petId?: string | string[];

  // Time filter
  timeRange?: TimeRange;

  // Condition filter
  condition?: (event: T) => boolean;

  // Throttling
  throttle?: number;        // milliseconds
  debounce?: number;        // milliseconds

  // Priority
  minPriority?: number;
}

// Apply filters
const filteredHandler = petEmotion.filter(
  'emotion_detected',
  {
    petId: 'PET-001',
    condition: (event) => event.result.confidence > 0.8,
    throttle: 1000
  },
  (event) => {
    console.log('High-confidence detection:', event);
  }
);
```

---

## 8. Configuration

### 8.1 Global Configuration

```typescript
interface GlobalConfig {
  // API settings
  api: {
    version: string;
    base_url?: string;
    api_key?: string;
    timeout: number;
  };

  // Model settings
  models: {
    registry: string;
    cache_dir: string;
    auto_update: boolean;
    preferred_models: Record<PetSpecies, string>;
  };

  // Data management
  data: {
    storage_path: string;
    max_storage_size: number;
    auto_cleanup: boolean;
    backup_enabled: boolean;
  };

  // Performance
  performance: {
    max_concurrent_detections: number;
    gpu_enabled: boolean;
    worker_threads: number;
    batch_size: number;
  };

  // Privacy and security
  security: {
    encryption_key?: string;
    ssl_verify: boolean;
    data_anonymization: boolean;
  };
}

// Set configuration
PetEmotion.setGlobalConfig(config: Partial<GlobalConfig>): void;
PetEmotion.getGlobalConfig(): GlobalConfig;
```

### 8.2 Per-Pet Configuration

```typescript
interface PetConfig {
  // Detection preferences
  detection: {
    enabled: boolean;
    auto_start: boolean;
    preferred_sources: SensorType[];
    min_confidence: number;
  };

  // Alert settings
  alerts: {
    enabled: boolean;
    stress_threshold: number;
    notification_channels: ('email' | 'sms' | 'push')[];
    quiet_hours?: {
      start: string;
      end: string;
    };
  };

  // Data retention
  retention: {
    keep_raw_data: boolean;
    keep_duration_days: number;
    aggregate_older_data: boolean;
  };

  // Model selection
  model: {
    model_id: string;
    fallback_model_id?: string;
    custom_weights?: Record<string, number>;
  };
}

// Set per-pet config
await petEmotion.setPetConfig(petId: string, config: Partial<PetConfig>): Promise<void>;
await petEmotion.getPetConfig(petId: string): Promise<PetConfig>;
```

---

## 9. Error Handling

### 9.1 Error Types

```typescript
class PetEmotionError extends Error {
  code: string;
  details?: any;
  recoverable: boolean;
}

class DetectionError extends PetEmotionError {
  constructor(message: string, details?: any) {
    super(message);
    this.code = 'DETECTION_ERROR';
    this.name = 'DetectionError';
  }
}

class SensorError extends PetEmotionError {
  sensorId: string;
  sensorType: SensorType;
}

class ModelError extends PetEmotionError {
  modelId: string;
  modelVersion: string;
}

class DataError extends PetEmotionError {
  dataType: string;
}

class ConfigurationError extends PetEmotionError {
  configKey: string;
}
```

### 9.2 Error Handling Patterns

```typescript
// Try-catch with specific errors
try {
  const result = await petEmotion.detectEmotion(petId, inputs);
} catch (error) {
  if (error instanceof DetectionError) {
    console.error('Detection failed:', error.message);
    // Try fallback method
  } else if (error instanceof SensorError) {
    console.error('Sensor error:', error.sensorId);
    // Reconnect sensor
  } else {
    console.error('Unknown error:', error);
  }
}

// Error events
petEmotion.on('error', (event: ErrorEvent) => {
  logError(event.error);

  if (event.error.recoverable) {
    // Attempt recovery
    retryOperation();
  } else {
    // Notify user
    alertUser(event.error);
  }
});

// Graceful degradation
async function detectEmotionSafe(petId: string, inputs: EmotionInputs) {
  try {
    return await petEmotion.detectEmotion(petId, inputs);
  } catch (error) {
    console.warn('Primary detection failed, using fallback');
    return await detectEmotionFallback(petId, inputs);
  }
}
```

---

## 10. Usage Examples

### 10.1 Basic Emotion Detection

```typescript
import { PetEmotion, CoreEmotion } from 'wia-pet-emotion';

// Initialize
const petEmotion = new PetEmotion({
  detection: {
    mode: 'realtime',
    frequency: 30,
    confidence_threshold: 0.7
  }
});

await petEmotion.initialize();

// Register a pet
const petId = await petEmotion.registerPet({
  name: 'Max',
  species: 'dog',
  breed: 'Golden Retriever',
  dateOfBirth: new Date('2020-03-15'),
  sex: 'neutered_male'
});

// Detect emotion from image
const result = await petEmotion.detectEmotion(petId, {
  image: {
    data: imageBuffer,
    format: 'jpeg',
    width: 1920,
    height: 1080,
    timestamp: new Date()
  }
});

console.log(`Detected emotion: ${result.emotion}`);
console.log(`Confidence: ${result.confidence}`);
console.log(`Valence: ${result.dimensions.valence}`);
console.log(`Arousal: ${result.dimensions.arousal}`);
```

### 10.2 Continuous Monitoring

```typescript
// Start continuous detection
await petEmotion.startContinuousDetection(petId, {
  frequency: 30,  // 30 Hz
  sources: ['camera', 'wearable'],
  alerts: {
    enabled: true,
    conditions: [
      {
        type: 'high_stress',
        stress_threshold: 0.8,
        min_duration: 60000,  // 1 minute
        callback: async (alert) => {
          console.log('High stress detected!');
          await notifyOwner(alert);
        }
      }
    ]
  }
});

// Listen for emotion changes
petEmotion.on('emotion_changed', (event) => {
  console.log(`Emotion changed: ${event.previous.emotion} → ${event.current.emotion}`);

  if (event.current.emotion === 'anxious') {
    console.log('Pet is anxious, check environment');
  }
});

// Listen for stress alerts
petEmotion.on('stress_alert', (event) => {
  console.log(`Stress alert! Level: ${event.stress_level}`);
  console.log(`Recommended action: ${event.recommended_action}`);
});

// Stop continuous detection when done
await petEmotion.stopContinuousDetection(petId);
```

### 10.3 Multi-Sensor Integration

```typescript
import { CameraAdapter, WearableAdapter, SensorFusion } from 'wia-pet-emotion';

// Create sensor fusion
const fusion = new SensorFusion();
fusion.setFusionStrategy(FusionStrategy.HYBRID);

// Add camera
const camera = new CameraAdapter({
  device: '/dev/video0',
  resolution: [1920, 1080],
  fps: 30
});
await camera.connect();
fusion.addSensor(camera);

// Add wearable
const wearable = new WearableAdapter({
  device: 'FitBark-12345',
  bluetooth_address: '00:11:22:33:44:55'
});
await wearable.connect();
fusion.addSensor(wearable);

// Capture synchronized data
const multiModalData = await fusion.captureSynchronized();

// Detect emotion with fusion
const result = await fusion.detectEmotionFused(multiModalData);

console.log(`Fused emotion: ${result.emotion}`);
console.log(`Contributing factors:`);
result.factors.forEach(factor => {
  console.log(`  ${factor.source}: ${(factor.contribution * 100).toFixed(1)}%`);
});
```

### 10.4 Pattern Analysis

```typescript
import { PatternAnalyzer, TemporalAnalyzer } from 'wia-pet-emotion';

const patternAnalyzer = new PatternAnalyzer(petEmotion);
const temporalAnalyzer = new TemporalAnalyzer(petEmotion);

// Detect daily routine
const routine = await patternAnalyzer.detectDailyRoutine(petId);
console.log('Daily Routine:');
routine.routine_blocks.forEach(block => {
  console.log(`  ${block.time_range}: ${block.typical_emotion} (${block.typical_activity})`);
});

// Detect stress patterns
const stressPatterns = await patternAnalyzer.detectStressPatterns(petId);
console.log(`Found ${stressPatterns.length} stress patterns`);

// Analyze emotion trends
const trend = await temporalAnalyzer.analyzeTrend(
  petId,
  CoreEmotion.ANXIOUS,
  { type: 'week' }
);

console.log(`Anxiety trend: ${trend.direction}`);
console.log(`Rate of change: ${trend.rate_of_change.toFixed(3)}`);

// Forecast future emotions
const forecast = await temporalAnalyzer.forecastEmotion(
  petId,
  3600000  // 1 hour ahead
);

console.log(`Predicted emotion in 1 hour: ${forecast.predicted_emotion}`);
console.log(`Confidence: ${forecast.confidence}`);
```

### 10.5 Social Dynamics Analysis

```typescript
import { SocialAnalyzer } from 'wia-pet-emotion';

const socialAnalyzer = new SocialAnalyzer(petEmotion);

// Register multiple pets
const pet1 = await petEmotion.registerPet({ name: 'Max', species: 'dog', ... });
const pet2 = await petEmotion.registerPet({ name: 'Luna', species: 'dog', ... });
const pet3 = await petEmotion.registerPet({ name: 'Charlie', species: 'cat', ... });

// Analyze social dynamics
const dynamics = await socialAnalyzer.analyzeSocialDynamics(
  [pet1, pet2, pet3],
  {
    start: new Date('2025-12-18T00:00:00Z'),
    end: new Date('2025-12-18T23:59:59Z')
  }
);

console.log(`Group cohesion: ${dynamics.group_cohesion}`);
console.log(`Social roles:`);
dynamics.social_roles.forEach(role => {
  console.log(`  ${role.petId}: ${role.role}`);
});

// Detect interactions
const interactions = await socialAnalyzer.detectInteractions(
  [pet1, pet2],
  { min_duration: 5000 }  // At least 5 seconds
);

console.log(`Found ${interactions.length} interactions`);
interactions.forEach(interaction => {
  console.log(`  ${interaction.interaction_type}: ${interaction.quality.valence}`);
});

// Detect emotional contagion
const contagion = await socialAnalyzer.detectEmotionalContagion(
  pet1,
  [pet2, pet3]
);

console.log(`Contagion detected: ${contagion.detected}`);
if (contagion.detected) {
  console.log(`  Strength: ${contagion.strength}`);
  console.log(`  Affected pets: ${contagion.affected_pets.join(', ')}`);
}
```

---

## 11. REST API Endpoints

### 11.1 Pet Management

```
POST   /api/v1/pets
GET    /api/v1/pets
GET    /api/v1/pets/:petId
PUT    /api/v1/pets/:petId
DELETE /api/v1/pets/:petId
```

### 11.2 Emotion Detection

```
POST   /api/v1/pets/:petId/detect
POST   /api/v1/pets/:petId/detect/batch
POST   /api/v1/pets/:petId/detect/start
POST   /api/v1/pets/:petId/detect/stop
GET    /api/v1/pets/:petId/emotion/current
GET    /api/v1/pets/:petId/emotion/history
GET    /api/v1/pets/:petId/emotion/summary
```

### 11.3 Analysis

```
GET    /api/v1/pets/:petId/analysis/patterns
GET    /api/v1/pets/:petId/analysis/trends
GET    /api/v1/pets/:petId/analysis/anomalies
GET    /api/v1/pets/:petId/analysis/routine
POST   /api/v1/analysis/compare
POST   /api/v1/analysis/social
```

### 11.4 Sensors

```
POST   /api/v1/pets/:petId/sensors
GET    /api/v1/pets/:petId/sensors
DELETE /api/v1/pets/:petId/sensors/:sensorId
GET    /api/v1/pets/:petId/sensors/:sensorId/status
POST   /api/v1/pets/:petId/sensors/:sensorId/calibrate
```

---

## 12. Performance Metrics

| Operation | Latency (p50) | Latency (p99) | Throughput |
|-----------|---------------|---------------|------------|
| Single Detection | < 100ms | < 300ms | 100 req/s |
| Continuous (30Hz) | < 33ms | < 50ms | - |
| Batch (100 images) | < 5s | < 10s | - |
| Pattern Analysis | < 2s | < 5s | 10 req/s |
| Event Dispatch | < 5ms | < 20ms | 10,000 events/s |

---

## 13. Compatibility Matrix

| Language | Version | Support |
|----------|---------|---------|
| TypeScript | 4.5+ | Full |
| JavaScript (Node) | 16+ | Full |
| Python | 3.8+ | Full |
| Java | 11+ | Planned |
| Swift | 5.5+ | Planned |
| Go | 1.18+ | Planned |

---

**Document ID**: WIA-PET-EMOTION-PHASE2-001
**Version**: 1.0.0
**Last Updated**: 2025-12-18
**Copyright**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
