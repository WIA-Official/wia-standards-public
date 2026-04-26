# Chapter 4: Data Formats and Schemas

## Overview

This chapter defines the standardized JSON schemas and TypeScript interfaces for all fitness tracking data types in WIA-IND-012. These formats ensure consistent data representation across platforms and enable seamless data exchange.

---

## 4.1 Activity Data Format

### 4.1.1 Basic Activity Schema

```typescript
interface Activity {
  // Identification
  id: string;                    // UUID
  userId: string;                // User identifier
  type: ActivityType;            // Activity category

  // Timing
  startTime: Date;               // ISO 8601 timestamp
  endTime: Date;                 // ISO 8601 timestamp
  duration: number;              // Total seconds
  activeDuration?: number;       // Seconds (excluding pauses)

  // Distance
  distance?: number;             // Meters
  steps?: number;                // Step count

  // Basic metrics
  calories: number;              // kcal burned
  avgPace?: number;              // min/km
  avgSpeed?: number;             // km/h

  // Location
  startLocation?: GPSPoint;
  endLocation?: GPSPoint;

  // Metadata
  source: DataSource;            // Device/app that recorded
  manual: boolean;               // User-entered vs. auto-tracked
  notes?: string;                // User notes
  tags?: string[];               // Custom tags

  // Timestamps
  createdAt: Date;
  updatedAt: Date;
}
```

**Example JSON:**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "user_12345",
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "duration": 1935,
  "distance": 8000,
  "steps": 9840,
  "calories": 680,
  "avgPace": 4.03,
  "avgSpeed": 14.88,
  "source": {
    "device": "Apple Watch Series 9",
    "app": "WIA Fitness Tracker",
    "version": "2.1.0"
  },
  "manual": false,
  "createdAt": "2025-12-27T07:32:15Z",
  "updatedAt": "2025-12-27T07:32:15Z"
}
```

### 4.1.2 Activity Types Enumeration

```typescript
enum ActivityType {
  // Cardio
  RUNNING = 'running',
  WALKING = 'walking',
  CYCLING = 'cycling',
  SWIMMING = 'swimming',
  HIKING = 'hiking',
  ROWING = 'rowing',
  ELLIPTICAL = 'elliptical',
  STAIR_CLIMBING = 'stair_climbing',

  // Strength
  WEIGHT_TRAINING = 'weight_training',
  BODYWEIGHT = 'bodyweight',
  CROSSFIT = 'crossfit',
  POWERLIFTING = 'powerlifting',

  // Sports
  BASKETBALL = 'basketball',
  SOCCER = 'soccer',
  TENNIS = 'tennis',
  GOLF = 'golf',
  VOLLEYBALL = 'volleyball',

  // Mind-body
  YOGA = 'yoga',
  PILATES = 'pilates',
  TAI_CHI = 'tai_chi',
  MEDITATION = 'meditation',

  // Other
  HIIT = 'hiit',
  CIRCUIT_TRAINING = 'circuit_training',
  STRETCHING = 'stretching',
  OTHER = 'other'
}
```

### 4.1.3 GPS Data Format

```typescript
interface GPSPoint {
  latitude: number;              // Decimal degrees (-90 to 90)
  longitude: number;             // Decimal degrees (-180 to 180)
  elevation?: number;            // Meters above sea level
  accuracy?: number;             // Horizontal accuracy in meters
  timestamp: number;             // Unix timestamp (ms)
  speed?: number;                // m/s (device-calculated)
  heading?: number;              // Degrees (0-360, 0=North)
}

interface GPSRoute {
  points: GPSPoint[];
  totalDistance: number;         // Meters (calculated)
  elevationGain: number;         // Meters
  elevationLoss: number;         // Meters
  smoothed: boolean;             // Post-processed?
}
```

**Example GPS Point:**
```json
{
  "latitude": 37.7749,
  "longitude": -122.4194,
  "elevation": 52.3,
  "accuracy": 5.2,
  "timestamp": 1735283400000,
  "speed": 3.8,
  "heading": 245
}
```

---

## 4.2 Workout Data Format

### 4.2.1 Comprehensive Workout Schema

```typescript
interface Workout {
  // Identification
  id: string;
  userId: string;
  type: WorkoutType;
  name?: string;                 // Custom workout name

  // Timing
  startTime: Date;
  endTime: Date;
  duration: number;              // Total seconds
  activeDuration: number;        // Excluding rest periods

  // Activity metrics
  distance?: number;             // Meters
  steps?: number;
  elevation?: {
    gain: number;                // Meters
    loss: number;                // Meters
  };

  // Cardiovascular metrics
  heartRate?: {
    avg: number;                 // BPM
    max: number;                 // BPM
    min: number;                 // BPM
    zones: HeartRateZones;
  };

  // Energy expenditure
  calories: number;
  caloriesSources?: {
    active: number;              // During exercise
    resting: number;             // BMR during duration
    epoc: number;                // Afterburn
  };

  // Performance metrics
  pace?: {
    avg: number;                 // min/km
    best: number;                // Best pace
  };

  speed?: {
    avg: number;                 // km/h
    max: number;                 // km/h
  };

  cadence?: {
    avg: number;                 // steps/min or RPM
    max: number;
  };

  power?: {
    avg: number;                 // Watts
    max: number;                 // Watts
    normalized: number;          // Normalized Power
  };

  // GPS data
  route?: GPSRoute;

  // Structured intervals
  intervals?: Interval[];
  laps?: Lap[];

  // User feedback
  perceivedExertion?: number;    // RPE 1-10
  feeling?: 'great' | 'good' | 'average' | 'tired' | 'exhausted';
  notes?: string;

  // Equipment
  equipment?: Equipment[];

  // Environmental
  weather?: WeatherCondition;

  // Training metrics
  trainingLoad?: number;         // TRIMP or TSS
  tss?: number;                  // Training Stress Score
  intensityFactor?: number;      // IF

  // Metadata
  source: DataSource;
  createdAt: Date;
  updatedAt: Date;
}
```

### 4.2.2 Heart Rate Zone Distribution

```typescript
interface HeartRateZones {
  zone1: number;                 // Minutes in Zone 1 (50-60%)
  zone2: number;                 // Minutes in Zone 2 (60-70%)
  zone3: number;                 // Minutes in Zone 3 (70-80%)
  zone4: number;                 // Minutes in Zone 4 (80-90%)
  zone5: number;                 // Minutes in Zone 5 (90-100%)
}
```

**Example:**
```json
{
  "zone1": 5.2,
  "zone2": 12.8,
  "zone3": 18.5,
  "zone4": 8.3,
  "zone5": 2.1
}
```

### 4.2.3 Interval Data Format

```typescript
interface Interval {
  number: number;                // Interval sequence
  type: 'work' | 'rest' | 'warmup' | 'cooldown';
  duration: number;              // Seconds
  distance?: number;             // Meters

  // Targets (planned)
  targetPace?: number;           // min/km
  targetHeartRate?: number;      // BPM
  targetPower?: number;          // Watts

  // Actuals (achieved)
  avgPace?: number;
  avgHeartRate?: number;
  avgPower?: number;
  calories?: number;

  // Performance
  completed: boolean;
  compliance?: number;           // 0-100% (target adherence)
}
```

**Example Interval Workout:**
```json
{
  "workoutType": "interval_training",
  "intervals": [
    {
      "number": 1,
      "type": "warmup",
      "duration": 600,
      "avgPace": 6.2,
      "avgHeartRate": 128
    },
    {
      "number": 2,
      "type": "work",
      "duration": 300,
      "distance": 1000,
      "targetPace": 4.0,
      "avgPace": 4.05,
      "avgHeartRate": 172,
      "compliance": 98
    },
    {
      "number": 3,
      "type": "rest",
      "duration": 120,
      "avgHeartRate": 145
    }
  ]
}
```

### 4.2.4 Strength Training Format

```typescript
interface StrengthWorkout extends Workout {
  exercises: Exercise[];
  totalVolume: number;           // kg × reps total
  totalSets: number;
  totalReps: number;
}

interface Exercise {
  id: string;
  name: string;                  // "Bench Press", "Squat", etc.
  category: ExerciseCategory;
  equipment: string;             // "Barbell", "Dumbbell", etc.
  muscleGroups: MuscleGroup[];
  sets: Set[];
  personalRecord?: boolean;      // PR achieved?
  notes?: string;
}

interface Set {
  setNumber: number;
  reps: number;
  weight?: number;               // kg or lbs
  duration?: number;             // Seconds (for isometric holds)
  restTime?: number;             // Seconds before next set
  perceivedExertion?: number;    // RPE 1-10
  completed: boolean;
  failedRep?: number;            // Rep number where failure occurred
}

enum ExerciseCategory {
  CHEST = 'chest',
  BACK = 'back',
  LEGS = 'legs',
  SHOULDERS = 'shoulders',
  ARMS = 'arms',
  CORE = 'core',
  FULL_BODY = 'full_body'
}

enum MuscleGroup {
  PECTORALS = 'pectorals',
  LATISSIMUS_DORSI = 'latissimus_dorsi',
  TRAPEZIUS = 'trapezius',
  QUADRICEPS = 'quadriceps',
  HAMSTRINGS = 'hamstrings',
  GLUTES = 'glutes',
  DELTOIDS = 'deltoids',
  BICEPS = 'biceps',
  TRICEPS = 'triceps',
  ABDOMINALS = 'abdominals',
  OBLIQUES = 'obliques'
}
```

**Example Strength Workout:**
```json
{
  "type": "weight_training",
  "exercises": [
    {
      "name": "Barbell Squat",
      "category": "legs",
      "muscleGroups": ["quadriceps", "glutes", "hamstrings"],
      "sets": [
        {"setNumber": 1, "reps": 8, "weight": 100, "restTime": 120},
        {"setNumber": 2, "reps": 8, "weight": 100, "restTime": 120},
        {"setNumber": 3, "reps": 6, "weight": 110, "restTime": 180}
      ]
    }
  ]
}
```

---

## 4.3 Heart Rate Data Format

### 4.3.1 Real-Time Heart Rate

```typescript
interface HeartRateReading {
  timestamp: Date;
  bpm: number;                   // Beats per minute
  confidence?: number;           // 0-100% (sensor confidence)
  source: 'chest_strap' | 'wrist' | 'optical' | 'ecg';
}

interface HeartRateStream {
  userId: string;
  readings: HeartRateReading[];
  avgBpm: number;
  maxBpm: number;
  minBpm: number;
  startTime: Date;
  endTime: Date;
}
```

### 4.3.2 Heart Rate Variability (HRV)

```typescript
interface HRVMeasurement {
  timestamp: Date;
  userId: string;

  // Time-domain metrics
  rmssd: number;                 // Root mean square of successive differences (ms)
  sdnn: number;                  // Standard deviation of NN intervals (ms)
  pnn50: number;                 // % of successive RR intervals > 50ms

  // Frequency-domain metrics (optional)
  lf?: number;                   // Low frequency power (ms²)
  hf?: number;                   // High frequency power (ms²)
  lfhfRatio?: number;            // LF/HF ratio

  // Raw data
  rrIntervals?: number[];        // R-R intervals in ms

  // Context
  measurementType: 'morning' | 'resting' | 'post_exercise' | 'sleeping';
  quality: 'excellent' | 'good' | 'fair' | 'poor';
}
```

**Example HRV Measurement:**
```json
{
  "timestamp": "2025-12-27T06:30:00Z",
  "userId": "user_12345",
  "rmssd": 42.5,
  "sdnn": 65.3,
  "pnn50": 18.2,
  "measurementType": "morning",
  "quality": "excellent"
}
```

---

## 4.4 Body Composition Data Format

### 4.4.1 Body Metrics Schema

```typescript
interface BodyComposition {
  timestamp: Date;
  userId: string;

  // Basic measurements
  weight: number;                // kg
  height: number;                // cm
  bmi: number;                   // Calculated

  // Body fat
  bodyFatPercentage?: number;    // %
  bodyFatMass?: number;          // kg
  leanMass?: number;             // kg

  // Advanced metrics
  visceralFat?: number;          // Level 1-59
  muscleMass?: number;           // kg
  boneMass?: number;             // kg
  waterPercentage?: number;      // %

  // Metabolic
  basalMetabolicRate?: number;   // kcal/day
  metabolicAge?: number;         // years

  // Body measurements
  measurements?: {
    neck?: number;               // cm
    chest?: number;              // cm
    waist?: number;              // cm
    hips?: number;               // cm
    thigh?: number;              // cm
    arm?: number;                // cm
    calf?: number;               // cm
  };

  // Measurement method
  method?: 'bioimpedance' | 'caliper' | 'dexa' | 'manual' | 'estimated';
  deviceId?: string;

  // Metadata
  notes?: string;
  createdAt: Date;
}
```

**Example Body Composition:**
```json
{
  "timestamp": "2025-12-27T07:00:00Z",
  "userId": "user_12345",
  "weight": 75.0,
  "height": 178,
  "bmi": 23.7,
  "bodyFatPercentage": 15.2,
  "bodyFatMass": 11.4,
  "leanMass": 63.6,
  "muscleMass": 58.5,
  "boneMass": 3.2,
  "waterPercentage": 55.8,
  "basalMetabolicRate": 1685,
  "metabolicAge": 28,
  "measurements": {
    "waist": 82,
    "chest": 98,
    "arm": 35
  },
  "method": "bioimpedance"
}
```

---

## 4.5 Sleep Data Format

### 4.5.1 Sleep Session Schema

```typescript
interface SleepSession {
  id: string;
  userId: string;

  // Timing
  startTime: Date;               // When user went to bed
  endTime: Date;                 // When user woke up
  totalDuration: number;         // Minutes in bed

  // Sleep stages
  stages: {
    awake: number;               // Minutes
    light: number;               // Minutes
    deep: number;                // Minutes
    rem: number;                 // Minutes
  };

  // Quality metrics
  quality: {
    score: number;               // 0-100
    efficiency: number;          // % (time asleep / time in bed)
    interruptions: number;       // Count of awakenings
    restlessness: number;        // Movement count
    timeToSleep: number;         // Minutes to fall asleep
  };

  // Physiological metrics
  heartRate?: {
    avg: number;                 // BPM during sleep
    min: number;                 // Lowest BPM
    max: number;                 // Highest BPM
  };

  hrv?: {
    avg: number;                 // Average RMSSD (ms)
  };

  respiratoryRate?: number;      // Breaths per minute
  oxygenSaturation?: {
    avg: number;                 // SpO2 %
    min: number;                 // Lowest SpO2 %
  };

  // Environmental factors
  environment?: {
    temperature?: number;        // °C
    humidity?: number;           // %
    noise?: number;              // dB
    light?: number;              // lux
  };

  // User feedback
  feeling?: 'refreshed' | 'good' | 'fair' | 'tired' | 'exhausted';
  notes?: string;

  // Detection
  autoDetected: boolean;
  source: DataSource;
  createdAt: Date;
}
```

**Example Sleep Session:**
```json
{
  "id": "sleep_550e8400",
  "userId": "user_12345",
  "startTime": "2025-12-26T23:15:00Z",
  "endTime": "2025-12-27T07:00:00Z",
  "totalDuration": 465,
  "stages": {
    "awake": 25,
    "light": 220,
    "deep": 110,
    "rem": 110
  },
  "quality": {
    "score": 85,
    "efficiency": 94.6,
    "interruptions": 2,
    "restlessness": 12,
    "timeToSleep": 8
  },
  "heartRate": {
    "avg": 52,
    "min": 48,
    "max": 68
  },
  "hrv": {
    "avg": 68.5
  },
  "respiratoryRate": 14,
  "feeling": "refreshed",
  "autoDetected": true
}
```

---

## 4.6 Goal and Achievement Data Format

### 4.6.1 Fitness Goal Schema

```typescript
interface FitnessGoal {
  id: string;
  userId: string;

  // Goal definition
  type: GoalType;
  name: string;
  description?: string;

  // Target
  target: number;
  current: number;
  unit: string;                  // "steps", "km", "workouts", "kg", etc.

  // Timeline
  period: 'daily' | 'weekly' | 'monthly' | 'yearly' | 'one-time';
  startDate: Date;
  endDate?: Date;

  // Progress
  progress: number;              // 0-100%
  status: 'active' | 'completed' | 'abandoned' | 'paused';

  // Reminders
  reminders?: Reminder[];

  // History
  progressHistory?: ProgressSnapshot[];

  // Metadata
  createdAt: Date;
  updatedAt: Date;
  completedAt?: Date;
}

enum GoalType {
  DAILY_STEPS = 'daily_steps',
  WEEKLY_DISTANCE = 'weekly_distance',
  MONTHLY_WORKOUTS = 'monthly_workouts',
  ACTIVE_MINUTES = 'active_minutes',
  CALORIE_BURN = 'calorie_burn',
  WEIGHT_LOSS = 'weight_loss',
  RACE_TIME = 'race_time',
  WORKOUT_STREAK = 'workout_streak'
}
```

### 4.6.2 Achievement Schema

```typescript
interface Achievement {
  id: string;
  name: string;
  description: string;
  category: 'distance' | 'duration' | 'frequency' | 'milestone' | 'special';
  tier: 'bronze' | 'silver' | 'gold' | 'platinum';

  // Icon/Badge
  icon: string;                  // URL or icon identifier
  color: string;                 // Hex color code

  // Requirements
  requirements: {
    metric: string;
    value: number;
    operator: '>=' | '<=' | '==' | '>';
  };

  // User progress
  earnedDate?: Date;
  progress: number;              // 0-100%
  unlocked: boolean;
}
```

---

## 4.7 User Profile Data Format

```typescript
interface UserProfile {
  userId: string;

  // Personal information
  dateOfBirth: Date;
  sex: 'male' | 'female' | 'other' | 'prefer_not_to_say';
  height: number;                // cm
  weight: number;                // kg (current)

  // Calculated fields
  age: number;
  bmi: number;

  // Fitness metrics
  restingHeartRate?: number;     // BPM
  maxHeartRate?: number;         // BPM (measured or estimated)
  vo2Max?: number;               // ml/kg/min
  ftp?: number;                  // Functional Threshold Power (watts)
  lthr?: number;                 // Lactate Threshold HR (BPM)

  // Activity level
  activityLevel: 'sedentary' | 'lightly_active' | 'moderately_active' |
                 'very_active' | 'extremely_active';

  // Goals
  primaryGoal?: 'weight_loss' | 'muscle_gain' | 'endurance' | 'strength' |
                'general_fitness' | 'performance';

  // Preferences
  units: 'metric' | 'imperial';
  timezone: string;              // IANA timezone
  language: string;              // ISO 639-1 code

  // Privacy
  privacySettings: PrivacySettings;

  // Metadata
  createdAt: Date;
  updatedAt: Date;
}

interface PrivacySettings {
  shareActivities: boolean;
  shareLocation: boolean;
  shareProfile: boolean;
  allowFriendRequests: boolean;
  activityVisibility: 'public' | 'friends' | 'private';
}
```

---

## 4.8 Data Source Metadata

```typescript
interface DataSource {
  device?: {
    manufacturer: string;        // "Apple", "Garmin", "Fitbit", etc.
    model: string;               // "Apple Watch Series 9"
    firmwareVersion?: string;
    serialNumber?: string;       // Hashed for privacy
  };

  app?: {
    name: string;                // "WIA Fitness Tracker"
    version: string;             // "2.1.0"
    platform: 'ios' | 'android' | 'web';
  };

  sensors?: string[];            // ["gps", "heart_rate", "accelerometer"]
}
```

---

## Key Takeaways

✓ All data types use standardized JSON schemas with TypeScript interfaces

✓ Activity format supports both simple tracking and detailed workouts

✓ GPS data includes accuracy and elevation for precise route tracking

✓ Heart rate data supports real-time streaming and HRV analysis

✓ Body composition schema covers basic and advanced metrics

✓ Sleep tracking includes stages, quality scores, and physiological data

✓ Goals and achievements provide gamification and motivation

✓ Data source metadata ensures transparency and traceability

✓ All timestamps use ISO 8601 format for consistency

✓ Optional fields support varying device capabilities

---

**Next:** [Chapter 5: API Interface →](05-api-interface.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
