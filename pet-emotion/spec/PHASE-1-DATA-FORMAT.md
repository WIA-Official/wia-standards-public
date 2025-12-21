# Phase 1: Pet Emotion Data Format Specification

## WIA-PET-EMOTION Data Format Standard

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-EMOTION-PHASE1-001
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

WIA-PET-EMOTION is a comprehensive standard for detecting, recording, and analyzing emotional states in companion animals. This standard enables veterinarians, pet owners, researchers, and AI systems to understand and respond to pet emotions through multi-modal sensing and analysis.

**Core Objectives**:
- Standardize emotion classification across species
- Enable real-time emotion monitoring via wearables
- Support AI/ML emotion detection models
- Provide temporal emotion pattern analysis
- Facilitate multi-pet interaction emotion tracking
- Ensure interoperability across veterinary and pet care systems

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| **Emotion Taxonomy** | Standardized emotion categories for dogs, cats, and other pets |
| **Behavioral Indicators** | Body language, posture, and movement patterns |
| **Vocalization Analysis** | Sound pattern classification and acoustic features |
| **Physiological Markers** | Heart rate, cortisol, temperature, and biomarkers |
| **AI/ML Integration** | Model outputs and confidence scoring |
| **Temporal Patterns** | Time-series emotion tracking and trends |
| **Multi-Pet Dynamics** | Social interaction emotion detection |

### 1.3 Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity and All Living Beings

Understanding pet emotions improves welfare, strengthens human-animal bonds, and prevents behavioral issues.

---

## 2. Emotion Taxonomy

### 2.1 Core Emotion Categories

```typescript
enum CoreEmotion {
  // Positive Emotions
  HAPPY = 'happy',
  CONTENT = 'content',
  PLAYFUL = 'playful',
  EXCITED = 'excited',
  CURIOUS = 'curious',
  AFFECTIONATE = 'affectionate',

  // Negative Emotions
  FEARFUL = 'fearful',
  ANXIOUS = 'anxious',
  STRESSED = 'stressed',
  ANGRY = 'angry',
  FRUSTRATED = 'frustrated',
  SAD = 'sad',
  LONELY = 'lonely',

  // Neutral/Other
  CALM = 'calm',
  ALERT = 'alert',
  TIRED = 'tired',
  PAIN = 'pain',
  UNKNOWN = 'unknown'
}
```

### 2.2 Emotion Intensity Scale

```typescript
interface EmotionIntensity {
  // Scale from 0.0 to 1.0
  intensity: number;

  // Discrete levels
  level: 'minimal' | 'low' | 'moderate' | 'high' | 'extreme';

  // Confidence in assessment (0.0-1.0)
  confidence: number;
}
```

### 2.3 Emotion Valence and Arousal

```typescript
interface EmotionDimensions {
  // Valence: -1.0 (negative) to +1.0 (positive)
  valence: number;

  // Arousal: 0.0 (calm) to 1.0 (highly aroused)
  arousal: number;

  // Dominance: 0.0 (submissive) to 1.0 (dominant)
  dominance: number;

  // Circumplex model coordinates
  circumplex: {
    x: number;  // valence axis
    y: number;  // arousal axis
  };
}
```

### 2.4 Species-Specific Emotion Mapping

```typescript
interface SpeciesEmotionProfile {
  species: PetSpecies;

  // Species-specific emotion expressions
  emotionExpressions: {
    [key in CoreEmotion]?: {
      typical: boolean;
      indicators: string[];
      reliability: number;
    };
  };

  // Species-specific behavioral markers
  behavioralMarkers: BehavioralMarker[];
}

enum PetSpecies {
  DOG = 'dog',
  CAT = 'cat',
  RABBIT = 'rabbit',
  BIRD = 'bird',
  REPTILE = 'reptile',
  RODENT = 'rodent',
  OTHER = 'other'
}
```

---

## 3. Behavioral Indicators Format

### 3.1 Body Language Signals

```typescript
interface BodyLanguage {
  timestamp: ISO8601;

  // Tail position and movement
  tail: {
    position: 'up' | 'down' | 'neutral' | 'tucked' | 'curved';
    movement: 'wagging' | 'still' | 'twitching' | 'thrashing';
    speed: number;           // movements per second
    amplitude: number;       // 0.0-1.0 (subtle to vigorous)
  };

  // Ear position
  ears: {
    position: 'forward' | 'neutral' | 'back' | 'flat' | 'rotating';
    symmetry: 'symmetrical' | 'asymmetrical';
    angle: number;           // degrees from neutral
  };

  // Body posture
  posture: {
    stance: 'relaxed' | 'tense' | 'crouched' | 'play_bow' | 'defensive';
    height: 'raised' | 'normal' | 'lowered';
    weight_distribution: 'forward' | 'centered' | 'backward';
    muscle_tension: number;  // 0.0-1.0
  };

  // Head position
  head: {
    tilt: number;            // degrees
    direction: 'forward' | 'down' | 'up' | 'averted';
    stability: 'stable' | 'scanning' | 'bobbing';
  };

  // Facial expressions (dogs/cats)
  face: {
    eyes: {
      size: 'normal' | 'dilated' | 'constricted';
      gaze: 'direct' | 'averted' | 'soft' | 'hard';
      blinking_rate: number; // blinks per minute
      whale_eye: boolean;    // showing whites
    };
    mouth: {
      state: 'closed' | 'open' | 'panting' | 'yawning';
      lip_position: 'relaxed' | 'pulled_back' | 'tense';
      teeth_visible: boolean;
      tongue_out: boolean;
    };
    whiskers: {
      position: 'forward' | 'neutral' | 'back';
      spread: number;        // degrees
    };
  };

  // Overall body
  movement: {
    activity_level: number;  // 0.0-1.0
    coordination: 'smooth' | 'jerky' | 'trembling';
    direction: 'approaching' | 'retreating' | 'circling' | 'stationary';
    speed: number;           // m/s
  };
}
```

### 3.2 Behavioral Pattern Recognition

```typescript
interface BehavioralPattern {
  patternId: string;
  type: BehavioralPatternType;

  // Sequence of behaviors
  sequence: {
    behavior: string;
    duration: number;        // milliseconds
    timestamp: ISO8601;
  }[];

  // Pattern characteristics
  frequency: number;         // occurrences per hour
  duration: number;          // total pattern duration in ms

  // Associated emotion
  associatedEmotion: CoreEmotion;
  confidence: number;

  // Context
  context: {
    location: string;
    socialContext: 'alone' | 'with_humans' | 'with_pets' | 'mixed';
    timeOfDay: string;
  };
}

enum BehavioralPatternType {
  // Positive patterns
  PLAY_SOLICITATION = 'play_solicitation',
  AFFECTION_SEEKING = 'affection_seeking',
  EXPLORATION = 'exploration',

  // Negative patterns
  DISPLACEMENT_BEHAVIOR = 'displacement_behavior',
  AVOIDANCE = 'avoidance',
  AGGRESSION = 'aggression',
  STEREOTYPY = 'stereotypy',

  // Neutral
  RESTING = 'resting',
  GROOMING = 'grooming',
  FEEDING = 'feeding'
}
```

### 3.3 Activity Recognition

```typescript
interface ActivityRecognition {
  timestamp: ISO8601;

  // Current activity
  activity: PetActivity;
  confidence: number;
  duration: number;          // milliseconds

  // Activity transitions
  previousActivity: PetActivity;
  transitionType: 'gradual' | 'abrupt';

  // Intensity
  intensity: 'low' | 'moderate' | 'high' | 'vigorous';
}

enum PetActivity {
  SLEEPING = 'sleeping',
  RESTING = 'resting',
  EATING = 'eating',
  DRINKING = 'drinking',
  PLAYING = 'playing',
  WALKING = 'walking',
  RUNNING = 'running',
  GROOMING = 'grooming',
  VOCALIZING = 'vocalizing',
  ELIMINATION = 'elimination',
  HUNTING = 'hunting',
  HIDING = 'hiding',
  UNKNOWN = 'unknown'
}
```

---

## 4. Vocalization Analysis Format

### 4.1 Sound Classification

```typescript
interface VocalizationEvent {
  eventId: string;
  timestamp: ISO8601;

  // Sound type
  type: VocalizationType;
  subtype?: string;

  // Acoustic features
  acoustics: {
    // Frequency
    fundamental_frequency: number;  // Hz
    frequency_range: {
      min: number;
      max: number;
    };

    // Temporal
    duration: number;               // milliseconds
    syllable_count: number;
    syllable_rate: number;          // syllables per second

    // Amplitude
    amplitude: number;              // dB
    amplitude_variation: number;    // 0.0-1.0

    // Pattern
    pattern: 'continuous' | 'pulsed' | 'ascending' | 'descending' | 'complex';
    rhythm: {
      regular: boolean;
      tempo: number;                // beats per minute
    };

    // Quality
    harshness: number;              // 0.0-1.0
    breathiness: number;            // 0.0-1.0
    tonality: number;               // 0.0-1.0 (tonal vs noisy)
  };

  // Context
  context: {
    trigger: string;
    directed_at: 'human' | 'pet' | 'environment' | 'self';
    social_context: string;
  };

  // Emotion correlation
  emotionIndicators: {
    emotion: CoreEmotion;
    confidence: number;
  }[];

  // Audio data
  audioData?: {
    format: 'wav' | 'mp3' | 'raw';
    sampleRate: number;
    channels: number;
    dataUri?: string;
    hash: string;
  };
}

enum VocalizationType {
  // Dog vocalizations
  DOG_BARK = 'dog_bark',
  DOG_GROWL = 'dog_growl',
  DOG_WHINE = 'dog_whine',
  DOG_HOWL = 'dog_howl',
  DOG_YELP = 'dog_yelp',
  DOG_PANTING = 'dog_panting',
  DOG_SIGH = 'dog_sigh',

  // Cat vocalizations
  CAT_MEOW = 'cat_meow',
  CAT_PURR = 'cat_purr',
  CAT_HISS = 'cat_hiss',
  CAT_GROWL = 'cat_growl',
  CAT_CHIRP = 'cat_chirp',
  CAT_TRILL = 'cat_trill',
  CAT_YOWL = 'cat_yowl',
  CAT_CHATTER = 'cat_chatter',

  // Other
  OTHER = 'other',
  UNKNOWN = 'unknown'
}
```

### 4.2 Bark Pattern Analysis (Dogs)

```typescript
interface BarkPattern {
  patternId: string;
  timestamp: ISO8601;

  // Bark characteristics
  bark_count: number;
  bark_sequence: {
    index: number;
    duration: number;         // ms
    interval_after: number;   // ms to next bark
    pitch: number;            // Hz
    intensity: number;        // 0.0-1.0
  }[];

  // Pattern classification
  pattern_type: 'alert' | 'alarm' | 'attention' | 'play' | 'frustration' | 'threat';

  // Emotional content
  urgency: number;            // 0.0-1.0
  stress_level: number;       // 0.0-1.0

  // Interpretation
  likely_meaning: string;
  confidence: number;
}
```

### 4.3 Purr Analysis (Cats)

```typescript
interface PurrPattern {
  patternId: string;
  timestamp: ISO8601;

  // Purr characteristics
  frequency: number;          // Hz (typically 25-150)
  amplitude: number;          // dB
  duration: number;           // milliseconds

  // Pattern
  continuous: boolean;
  interruptions: number;

  // Type classification
  purr_type: 'contentment' | 'solicitation' | 'stress' | 'pain';

  // Emotional interpretation
  valence: number;            // -1.0 to 1.0
  confidence: number;
}
```

---

## 5. Physiological Markers Format

### 5.1 Vital Signs

```typescript
interface VitalSigns {
  timestamp: ISO8601;

  // Heart rate
  heart_rate: {
    bpm: number;
    variability: number;      // SDNN in milliseconds
    rhythm: 'regular' | 'irregular';

    // HRV metrics
    hrv_metrics: {
      rmssd: number;          // root mean square of successive differences
      pnn50: number;          // percentage of successive RR intervals > 50ms
      lf_hf_ratio: number;    // low frequency / high frequency power ratio
    };
  };

  // Respiration
  respiration: {
    rate: number;             // breaths per minute
    pattern: 'normal' | 'rapid' | 'shallow' | 'labored';
    regularity: number;       // 0.0-1.0
  };

  // Temperature
  temperature: {
    value: number;            // Celsius
    location: 'rectal' | 'ear' | 'axillary' | 'surface';
  };

  // Blood pressure (if available)
  blood_pressure?: {
    systolic: number;         // mmHg
    diastolic: number;        // mmHg
  };

  // Emotion correlation
  stress_index: number;       // 0.0-1.0 derived from vitals
}
```

### 5.2 Biomarker Analysis

```typescript
interface BiomarkerProfile {
  profileId: string;
  timestamp: ISO8601;

  // Sample information
  sample: {
    type: 'blood' | 'saliva' | 'urine' | 'feces';
    collectedAt: ISO8601;
    analyzedAt: ISO8601;
  };

  // Stress hormones
  cortisol: {
    value: number;            // nmol/L
    baseline: number;
    percentChange: number;
    interpretation: 'low' | 'normal' | 'elevated' | 'high';
  };

  // Other markers
  markers: {
    name: string;
    value: number;
    unit: string;
    referenceRange: {
      min: number;
      max: number;
    };
    status: 'low' | 'normal' | 'high';
  }[];

  // Emotion interpretation
  stress_level: number;       // 0.0-1.0
  anxiety_indicators: boolean;
}
```

### 5.3 Wearable Sensor Data

```typescript
interface WearableSensorData {
  deviceId: string;
  deviceInfo: {
    manufacturer: string;
    model: string;
    firmwareVersion: string;
    sensorTypes: SensorType[];
  };

  timestamp: ISO8601;

  // Accelerometer
  accelerometer?: {
    x: number;                // m/s²
    y: number;
    z: number;
    magnitude: number;
    activity_count: number;
  };

  // Gyroscope
  gyroscope?: {
    pitch: number;            // degrees
    roll: number;
    yaw: number;
  };

  // Location
  gps?: {
    latitude: number;
    longitude: number;
    accuracy: number;         // meters
    speed: number;            // m/s
  };

  // Environmental sensors
  environment?: {
    temperature: number;      // Celsius
    humidity: number;         // percentage
    light_level: number;      // lux
    noise_level: number;      // dB
  };

  // Derived metrics
  activity_level: number;     // 0.0-1.0
  rest_quality: number;       // 0.0-1.0
  stress_indicators: boolean;
}

enum SensorType {
  ACCELEROMETER = 'accelerometer',
  GYROSCOPE = 'gyroscope',
  GPS = 'gps',
  HEART_RATE = 'heart_rate',
  TEMPERATURE = 'temperature',
  CAMERA = 'camera',
  MICROPHONE = 'microphone'
}
```

---

## 6. AI/ML Model Output Format

### 6.1 Emotion Detection Model Output

```typescript
interface EmotionDetectionOutput {
  modelId: string;
  modelVersion: string;
  timestamp: ISO8601;
  processingTime: number;     // milliseconds

  // Model information
  model: {
    name: string;
    type: 'cnn' | 'lstm' | 'transformer' | 'ensemble' | 'rule_based';
    architecture: string;
    trainedOn: {
      species: PetSpecies[];
      datasetSize: number;
      accuracy: number;
    };
  };

  // Input modalities
  inputs: {
    video?: boolean;
    audio?: boolean;
    sensors?: boolean;
    text?: boolean;
  };

  // Emotion predictions
  predictions: {
    emotion: CoreEmotion;
    probability: number;
    confidence: number;
  }[];

  // Top prediction
  primary_emotion: CoreEmotion;
  primary_confidence: number;

  // Dimensional model output
  dimensions: EmotionDimensions;

  // Attention/saliency
  attention_map?: {
    regions: {
      x: number;
      y: number;
      width: number;
      height: number;
      importance: number;
    }[];
  };

  // Uncertainty
  uncertainty: {
    aleatoric: number;        // data uncertainty
    epistemic: number;        // model uncertainty
    total: number;
  };
}
```

### 6.2 Multi-Modal Fusion

```typescript
interface MultiModalFusion {
  fusionId: string;
  timestamp: ISO8601;

  // Individual modality results
  modalities: {
    video: EmotionDetectionOutput;
    audio: EmotionDetectionOutput;
    sensors: EmotionDetectionOutput;
  };

  // Fusion strategy
  fusion_method: 'early' | 'late' | 'hybrid' | 'attention';

  // Fused result
  fused_emotion: CoreEmotion;
  fused_confidence: number;
  fused_dimensions: EmotionDimensions;

  // Modality weights
  weights: {
    video: number;
    audio: number;
    sensors: number;
  };

  // Agreement metrics
  inter_modality_agreement: number;  // 0.0-1.0
  conflicts: {
    modality1: string;
    modality2: string;
    disagreement: number;
  }[];
}
```

### 6.3 Temporal Emotion Model

```typescript
interface TemporalEmotionModel {
  modelId: string;
  windowSize: number;         // milliseconds

  // Time series
  emotionSequence: {
    timestamp: ISO8601;
    emotion: CoreEmotion;
    intensity: number;
    confidence: number;
  }[];

  // Transition analysis
  transitions: {
    from: CoreEmotion;
    to: CoreEmotion;
    timestamp: ISO8601;
    duration: number;         // ms between states
    probability: number;
  }[];

  // Pattern detection
  patterns: {
    pattern_type: 'stable' | 'oscillating' | 'escalating' | 'de-escalating';
    duration: number;
    significance: number;
  }[];

  // Forecasting
  forecast?: {
    predicted_emotion: CoreEmotion;
    prediction_horizon: number;  // milliseconds
    confidence: number;
  };
}
```

---

## 7. Temporal Pattern Format

### 7.1 Emotion Timeline

```typescript
interface EmotionTimeline {
  timelineId: string;
  petId: string;

  // Time range
  startTime: ISO8601;
  endTime: ISO8601;
  duration: number;           // milliseconds

  // Emotion states
  states: {
    emotion: CoreEmotion;
    startTime: ISO8601;
    endTime: ISO8601;
    duration: number;
    intensity: EmotionIntensity;
    triggers: string[];
    confidence: number;
  }[];

  // Statistics
  statistics: {
    dominant_emotion: CoreEmotion;
    emotion_distribution: {
      emotion: CoreEmotion;
      percentage: number;
      count: number;
    }[];

    transition_count: number;
    average_state_duration: number;
    volatility: number;       // 0.0-1.0
  };
}
```

### 7.2 Daily Emotion Summary

```typescript
interface DailyEmotionSummary {
  date: string;               // YYYY-MM-DD
  petId: string;

  // Overall metrics
  overall: {
    dominant_emotion: CoreEmotion;
    average_valence: number;  // -1.0 to 1.0
    average_arousal: number;  // 0.0 to 1.0
    emotional_stability: number;  // 0.0-1.0
  };

  // Time-of-day patterns
  periods: {
    period: 'morning' | 'afternoon' | 'evening' | 'night';
    dominant_emotion: CoreEmotion;
    activity_level: number;
    stress_level: number;
  }[];

  // Notable events
  events: {
    timestamp: ISO8601;
    event_type: string;
    emotion: CoreEmotion;
    intensity: number;
    description: string;
  }[];

  // Trends
  trends: {
    compared_to_baseline: number;  // percentage change
    stress_trend: 'improving' | 'stable' | 'worsening';
    activity_trend: 'increasing' | 'stable' | 'decreasing';
  };
}
```

### 7.3 Long-Term Emotion Patterns

```typescript
interface LongTermEmotionPattern {
  patternId: string;
  petId: string;

  // Analysis period
  startDate: string;
  endDate: string;

  // Baseline profile
  baseline: {
    typical_emotions: CoreEmotion[];
    average_valence: number;
    average_arousal: number;
    stress_baseline: number;
  };

  // Detected patterns
  patterns: {
    pattern_name: string;
    frequency: string;        // daily, weekly, monthly
    description: string;
    confidence: number;

    // Associated factors
    triggers: string[];
    time_patterns: string[];
    environmental_factors: string[];
  }[];

  // Anomalies
  anomalies: {
    date: string;
    anomaly_type: string;
    severity: 'low' | 'medium' | 'high';
    description: string;
    possible_causes: string[];
  }[];

  // Recommendations
  recommendations: {
    category: 'behavioral' | 'medical' | 'environmental';
    priority: 'low' | 'medium' | 'high';
    recommendation: string;
  }[];
}
```

---

## 8. Multi-Pet Interaction Format

### 8.1 Social Interaction Event

```typescript
interface SocialInteractionEvent {
  eventId: string;
  timestamp: ISO8601;
  duration: number;

  // Participants
  participants: {
    petId: string;
    species: PetSpecies;
    role: 'initiator' | 'responder' | 'observer';

    // Individual emotion
    emotion: CoreEmotion;
    intensity: number;
    arousal: number;
  }[];

  // Interaction type
  interaction_type: InteractionType;

  // Interaction quality
  quality: {
    valence: 'positive' | 'neutral' | 'negative' | 'mixed';
    intensity: number;        // 0.0-1.0
    reciprocity: number;      // 0.0-1.0
    synchrony: number;        // 0.0-1.0
  };

  // Behaviors observed
  behaviors: {
    petId: string;
    behavior: string;
    timestamp: ISO8601;
  }[];

  // Outcome
  outcome: 'positive' | 'neutral' | 'negative' | 'escalated' | 'de-escalated';
}

enum InteractionType {
  PLAY = 'play',
  GROOMING = 'grooming',
  AGGRESSION = 'aggression',
  AVOIDANCE = 'avoidance',
  AFFILIATION = 'affiliation',
  COMPETITION = 'competition',
  COOPERATION = 'cooperation',
  MATING = 'mating'
}
```

### 8.2 Group Emotion Dynamics

```typescript
interface GroupEmotionDynamics {
  groupId: string;
  timestamp: ISO8601;

  // Group composition
  members: {
    petId: string;
    species: PetSpecies;
    current_emotion: CoreEmotion;
  }[];

  // Group-level metrics
  group_metrics: {
    cohesion: number;         // 0.0-1.0
    average_valence: number;
    average_arousal: number;
    emotional_contagion: number;  // 0.0-1.0
    synchrony: number;
  };

  // Social network
  relationships: {
    pet1: string;
    pet2: string;
    relationship_type: 'bonded' | 'neutral' | 'antagonistic';
    strength: number;         // 0.0-1.0
  }[];

  // Influence patterns
  influence: {
    influencer: string;
    influenced: string[];
    influence_strength: number;
  }[];
}
```

---

## 9. Complete Emotion Record Format

### 9.1 Pet Emotion Record

```typescript
interface PetEmotionRecord {
  // Header
  header: {
    magic: 'WIAPETE';
    version: [1, 0, 0];
    standard: 'WIA-PET-EMOTION';
    recordId: string;         // UUID v7
    createdAt: ISO8601;
    lastUpdated: ISO8601;
  };

  // Pet identification
  pet: {
    petId: string;
    passportId?: string;      // Link to WIA-PET-HEALTH-PASSPORT
    species: PetSpecies;
    breed?: string;
    name: string;
    dateOfBirth: ISO8601;
  };

  // Current emotion state
  currentState: {
    timestamp: ISO8601;
    primary_emotion: CoreEmotion;
    intensity: EmotionIntensity;
    dimensions: EmotionDimensions;
    confidence: number;
  };

  // Data sources
  dataSources: {
    behavioral: BodyLanguage;
    vocalization?: VocalizationEvent;
    physiological?: VitalSigns;
    wearable?: WearableSensorData;
    biomarkers?: BiomarkerProfile;
    ai_model?: EmotionDetectionOutput;
  };

  // Temporal data
  timeline: EmotionTimeline;
  dailySummary?: DailyEmotionSummary;
  longTermPattern?: LongTermEmotionPattern;

  // Social context
  socialContext?: {
    alone: boolean;
    with_humans: boolean;
    with_pets: boolean;
    interaction?: SocialInteractionEvent;
    group?: GroupEmotionDynamics;
  };

  // Environmental context
  environment: {
    location: string;
    temperature?: number;
    noise_level?: number;
    light_level?: number;
    weather?: string;
  };

  // Metadata
  metadata: {
    source: 'manual' | 'automated' | 'hybrid';
    observer?: {
      type: 'owner' | 'veterinarian' | 'trainer' | 'researcher' | 'ai';
      id: string;
      qualifications?: string[];
    };
    quality_score: number;    // 0.0-1.0
    data_completeness: number;  // 0.0-1.0
  };

  // Verification
  verification?: {
    verified: boolean;
    verifiedBy: string;
    verifiedAt: ISO8601;
    digitalSignature?: string;
  };
}
```

---

## 10. JSON Schema

### 10.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/pet-emotion/v1.json",
  "title": "WIA Pet Emotion Record",
  "type": "object",
  "required": ["header", "pet", "currentState", "dataSources", "environment"],
  "properties": {
    "header": {
      "type": "object",
      "required": ["magic", "version", "standard", "recordId", "createdAt"],
      "properties": {
        "magic": { "const": "WIAPETE" },
        "version": {
          "type": "array",
          "items": { "type": "integer" },
          "minItems": 3,
          "maxItems": 3
        },
        "standard": { "const": "WIA-PET-EMOTION" },
        "recordId": { "type": "string", "format": "uuid" },
        "createdAt": { "type": "string", "format": "date-time" },
        "lastUpdated": { "type": "string", "format": "date-time" }
      }
    },
    "pet": {
      "type": "object",
      "required": ["petId", "species", "name"],
      "properties": {
        "petId": { "type": "string" },
        "passportId": { "type": "string" },
        "species": {
          "enum": ["dog", "cat", "rabbit", "bird", "reptile", "rodent", "other"]
        },
        "breed": { "type": "string" },
        "name": { "type": "string" },
        "dateOfBirth": { "type": "string", "format": "date-time" }
      }
    },
    "currentState": {
      "type": "object",
      "required": ["timestamp", "primary_emotion", "intensity", "confidence"],
      "properties": {
        "timestamp": { "type": "string", "format": "date-time" },
        "primary_emotion": {
          "enum": ["happy", "content", "playful", "excited", "curious", "affectionate",
                   "fearful", "anxious", "stressed", "angry", "frustrated", "sad", "lonely",
                   "calm", "alert", "tired", "pain", "unknown"]
        },
        "intensity": {
          "type": "object",
          "properties": {
            "intensity": { "type": "number", "minimum": 0, "maximum": 1 },
            "level": { "enum": ["minimal", "low", "moderate", "high", "extreme"] },
            "confidence": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        },
        "dimensions": {
          "type": "object",
          "properties": {
            "valence": { "type": "number", "minimum": -1, "maximum": 1 },
            "arousal": { "type": "number", "minimum": 0, "maximum": 1 },
            "dominance": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        },
        "confidence": { "type": "number", "minimum": 0, "maximum": 1 }
      }
    },
    "dataSources": {
      "type": "object",
      "required": ["behavioral"],
      "properties": {
        "behavioral": { "$ref": "#/definitions/BodyLanguage" },
        "vocalization": { "$ref": "#/definitions/VocalizationEvent" },
        "physiological": { "$ref": "#/definitions/VitalSigns" },
        "wearable": { "$ref": "#/definitions/WearableSensorData" },
        "biomarkers": { "$ref": "#/definitions/BiomarkerProfile" },
        "ai_model": { "$ref": "#/definitions/EmotionDetectionOutput" }
      }
    }
  },
  "definitions": {
    "BodyLanguage": {
      "type": "object",
      "properties": {
        "timestamp": { "type": "string", "format": "date-time" },
        "tail": {
          "type": "object",
          "properties": {
            "position": { "enum": ["up", "down", "neutral", "tucked", "curved"] },
            "movement": { "enum": ["wagging", "still", "twitching", "thrashing"] },
            "speed": { "type": "number", "minimum": 0 },
            "amplitude": { "type": "number", "minimum": 0, "maximum": 1 }
          }
        }
      }
    },
    "VocalizationEvent": {
      "type": "object",
      "properties": {
        "eventId": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" },
        "type": {
          "enum": ["dog_bark", "dog_growl", "dog_whine", "dog_howl", "dog_yelp",
                   "cat_meow", "cat_purr", "cat_hiss", "cat_growl", "cat_chirp",
                   "other", "unknown"]
        }
      }
    },
    "VitalSigns": {
      "type": "object",
      "properties": {
        "timestamp": { "type": "string", "format": "date-time" },
        "heart_rate": {
          "type": "object",
          "properties": {
            "bpm": { "type": "number", "minimum": 0 },
            "variability": { "type": "number", "minimum": 0 }
          }
        }
      }
    }
  }
}
```

---

## 11. Use Cases and Examples

### 11.1 Example: Real-Time Emotion Monitoring

```json
{
  "header": {
    "magic": "WIAPETE",
    "version": [1, 0, 0],
    "standard": "WIA-PET-EMOTION",
    "recordId": "550e8400-e29b-41d4-a716-446655440000",
    "createdAt": "2025-12-18T10:30:00Z",
    "lastUpdated": "2025-12-18T10:30:05Z"
  },
  "pet": {
    "petId": "PET-001",
    "species": "dog",
    "breed": "Golden Retriever",
    "name": "Max",
    "dateOfBirth": "2020-03-15T00:00:00Z"
  },
  "currentState": {
    "timestamp": "2025-12-18T10:30:05Z",
    "primary_emotion": "happy",
    "intensity": {
      "intensity": 0.85,
      "level": "high",
      "confidence": 0.92
    },
    "dimensions": {
      "valence": 0.8,
      "arousal": 0.7,
      "dominance": 0.6
    },
    "confidence": 0.92
  },
  "dataSources": {
    "behavioral": {
      "timestamp": "2025-12-18T10:30:05Z",
      "tail": {
        "position": "up",
        "movement": "wagging",
        "speed": 3.5,
        "amplitude": 0.8
      },
      "ears": {
        "position": "forward",
        "symmetry": "symmetrical",
        "angle": 45
      },
      "posture": {
        "stance": "relaxed",
        "height": "normal",
        "weight_distribution": "centered",
        "muscle_tension": 0.2
      }
    },
    "wearable": {
      "deviceId": "COLLAR-001",
      "deviceInfo": {
        "manufacturer": "FitBark",
        "model": "GPS Plus",
        "firmwareVersion": "2.1.0",
        "sensorTypes": ["accelerometer", "heart_rate", "gps"]
      },
      "timestamp": "2025-12-18T10:30:05Z",
      "heart_rate": {
        "bpm": 95,
        "variability": 45
      },
      "activity_level": 0.75
    }
  },
  "environment": {
    "location": "backyard",
    "temperature": 22,
    "weather": "sunny"
  },
  "metadata": {
    "source": "automated",
    "quality_score": 0.90,
    "data_completeness": 0.88
  }
}
```

### 11.2 Example: Stress Detection

```json
{
  "currentState": {
    "timestamp": "2025-12-18T14:15:30Z",
    "primary_emotion": "anxious",
    "intensity": {
      "intensity": 0.72,
      "level": "high",
      "confidence": 0.88
    },
    "dimensions": {
      "valence": -0.6,
      "arousal": 0.85,
      "dominance": 0.3
    }
  },
  "dataSources": {
    "behavioral": {
      "tail": {
        "position": "tucked",
        "movement": "still"
      },
      "ears": {
        "position": "back",
        "symmetry": "symmetrical"
      },
      "posture": {
        "stance": "crouched",
        "height": "lowered",
        "muscle_tension": 0.9
      }
    },
    "physiological": {
      "heart_rate": {
        "bpm": 145,
        "variability": 22,
        "rhythm": "irregular"
      },
      "respiration": {
        "rate": 48,
        "pattern": "rapid"
      }
    },
    "vocalization": {
      "type": "dog_whine",
      "acoustics": {
        "fundamental_frequency": 850,
        "duration": 1200,
        "amplitude": 45
      }
    }
  }
}
```

### 11.3 Example: Multi-Pet Play Interaction

```json
{
  "socialContext": {
    "interaction": {
      "eventId": "INT-001",
      "timestamp": "2025-12-18T16:00:00Z",
      "duration": 180000,
      "participants": [
        {
          "petId": "PET-001",
          "species": "dog",
          "role": "initiator",
          "emotion": "playful",
          "intensity": 0.9,
          "arousal": 0.85
        },
        {
          "petId": "PET-002",
          "species": "dog",
          "role": "responder",
          "emotion": "excited",
          "intensity": 0.88,
          "arousal": 0.82
        }
      ],
      "interaction_type": "play",
      "quality": {
        "valence": "positive",
        "intensity": 0.87,
        "reciprocity": 0.92,
        "synchrony": 0.85
      },
      "outcome": "positive"
    }
  }
}
```

### 11.4 Example: AI Model Emotion Detection

```json
{
  "dataSources": {
    "ai_model": {
      "modelId": "PET-EMOTION-CNN-V2",
      "modelVersion": "2.1.0",
      "timestamp": "2025-12-18T11:22:15Z",
      "processingTime": 45,
      "model": {
        "name": "PetEmotionNet",
        "type": "cnn",
        "architecture": "ResNet50",
        "trainedOn": {
          "species": ["dog", "cat"],
          "datasetSize": 150000,
          "accuracy": 0.94
        }
      },
      "inputs": {
        "video": true,
        "audio": true,
        "sensors": false
      },
      "predictions": [
        {
          "emotion": "happy",
          "probability": 0.89,
          "confidence": 0.91
        },
        {
          "emotion": "excited",
          "probability": 0.08,
          "confidence": 0.85
        },
        {
          "emotion": "playful",
          "probability": 0.03,
          "confidence": 0.72
        }
      ],
      "primary_emotion": "happy",
      "primary_confidence": 0.91
    }
  }
}
```

### 11.5 Example: Daily Emotion Summary

```json
{
  "dailySummary": {
    "date": "2025-12-18",
    "petId": "PET-001",
    "overall": {
      "dominant_emotion": "content",
      "average_valence": 0.65,
      "average_arousal": 0.45,
      "emotional_stability": 0.82
    },
    "periods": [
      {
        "period": "morning",
        "dominant_emotion": "excited",
        "activity_level": 0.75,
        "stress_level": 0.15
      },
      {
        "period": "afternoon",
        "dominant_emotion": "calm",
        "activity_level": 0.3,
        "stress_level": 0.1
      },
      {
        "period": "evening",
        "dominant_emotion": "playful",
        "activity_level": 0.8,
        "stress_level": 0.2
      }
    ],
    "trends": {
      "compared_to_baseline": 5,
      "stress_trend": "stable",
      "activity_trend": "increasing"
    }
  }
}
```

---

## 12. Performance Requirements

| Metric | Requirement | Target |
|--------|-------------|--------|
| Emotion Detection Latency | < 500ms | < 200ms |
| Continuous Monitoring | 1-60 Hz | 30 Hz |
| Model Accuracy | > 85% | > 92% |
| False Positive Rate | < 15% | < 8% |
| Battery Life (wearable) | > 24 hours | > 72 hours |
| Data Storage per day | < 100MB | < 50MB |

---

## 13. Compatibility

### 13.1 Integration with Other WIA Standards

| Standard | Integration Point |
|----------|------------------|
| WIA-PET-HEALTH-PASSPORT | Link via petId/passportId |
| WIA-PET-CARE-ROBOT | Emotion-aware robot behavior |
| WIA-PET-GENOME | Genetic predisposition to anxiety |
| WIA-AI | Model deployment and versioning |

### 13.2 Version Compatibility

| Version | Compatibility |
|---------|--------------|
| 1.0.x | Full forward/backward |
| 1.x.x | Backward compatible |
| 2.x.x | Migration required |

---

**Document ID**: WIA-PET-EMOTION-PHASE1-001
**Version**: 1.0.0
**Last Updated**: 2025-12-18
**Copyright**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
