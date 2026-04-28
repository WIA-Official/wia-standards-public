# WIA-IND-014 — Phase 2: API

> Member-management, automated-workout, facility-IoT, and virtual-training API surface — each presented as a worked endpoint specification.

## 4. Member Management

### 4.1 Member Profile Structure

```typescript
interface MemberProfile {
  // Basic Information
  memberId: string;
  personalInfo: {
    name: string;
    email: string;
    phone: string;
    dateOfBirth: Date;
    gender: 'male' | 'female' | 'other' | 'prefer-not-to-say';
    emergencyContact: {
      name: string;
      phone: string;
      relationship: string;
    };
  };

  // Membership Details
  membership: {
    type: 'basic' | 'premium' | 'vip' | 'corporate';
    status: 'active' | 'frozen' | 'expired' | 'cancelled';
    startDate: Date;
    expiryDate: Date;
    billingCycle: 'monthly' | 'quarterly' | 'annual';
    autoRenew: boolean;
    accessHours: string; // e.g., "24/7" or "06:00-22:00"
    includedFeatures: string[];
  };

  // Physical Metrics
  physicalProfile: {
    height: number; // cm
    weight: number; // kg
    bodyFatPercentage?: number;
    muscleMass?: number;
    bmi: number;
    restingHeartRate?: number;
    bloodPressure?: { systolic: number; diastolic: number };
    vo2Max?: number;
    measurementHistory: Array<{
      date: Date;
      weight: number;
      bodyFat?: number;
      muscleMass?: number;
    }>;
  };

  // Fitness Profile
  fitnessProfile: {
    experience: 'beginner' | 'intermediate' | 'advanced' | 'athlete';
    primaryGoals: Array<'weight-loss' | 'muscle-gain' | 'strength' |
                        'endurance' | 'flexibility' | 'general-fitness' |
                        'sport-specific' | 'rehabilitation'>;
    currentFitnessLevel: {
      cardiovascular: 1 | 2 | 3 | 4 | 5;
      strength: 1 | 2 | 3 | 4 | 5;
      flexibility: 1 | 2 | 3 | 4 | 5;
      overall: number;
    };
    preferredActivities: string[];
    dislikedActivities: string[];
  };

  // Medical & Safety
  medicalInfo: {
    medicalClearance: boolean;
    clearanceDate?: Date;
    clearanceExpiry?: Date;
    conditions: string[];
    injuries: Array<{
      type: string;
      area: string;
      date: Date;
      severity: 'minor' | 'moderate' | 'severe';
      restrictions: string[];
      recoveryStatus: 'acute' | 'recovery' | 'chronic' | 'healed';
    }>;
    medications: string[];
    allergies: string[];
    restrictions: string[];
  };

  // Training Preferences
  preferences: {
    workoutFrequency: number; // days per week
    sessionDuration: number; // minutes
    preferredTime: 'morning' | 'afternoon' | 'evening' | 'flexible';
    trainingStyle: 'solo' | 'group-class' | 'personal-trainer' | 'mixed';
    equipment: {
      cardio: boolean;
      freeWeights: boolean;
      machines: boolean;
      functional: boolean;
      groupClasses: string[];
    };
    notifications: {
      workoutReminders: boolean;
      classBookings: boolean;
      achievements: boolean;
      challenges: boolean;
      marketing: boolean;
    };
  };

  // Activity Tracking
  activity: {
    totalWorkouts: number;
    totalMinutes: number;
    currentStreak: number;
    longestStreak: number;
    lastVisit: Date;
    avgWorkoutsPerWeek: number;
    favoriteEquipment: string[];
    peakWorkoutTime: string;
  };
}
```

### 4.2 Access Control Systems

#### 4.2.1 Check-in Methods

**1. RFID Card/Fob:**
```typescript
interface RFIDCheckIn {
  method: 'rfid';
  cardId: string;
  readerId: string;
  location: 'main-entrance' | 'side-door' | 'locker-room';
  timestamp: Date;
  validationTime: number; // ms
}
```

**2. Mobile App (QR Code):**
```typescript
interface MobileCheckIn {
  method: 'mobile-app';
  qrCode: string;
  appVersion: string;
  deviceInfo: {
    platform: 'ios' | 'android';
    model: string;
    osVersion: string;
  };
  location: string;
  timestamp: Date;
}
```

**3. Biometric (Fingerprint/Face):**
```typescript
interface BiometricCheckIn {
  method: 'biometric';
  type: 'fingerprint' | 'facial-recognition';
  biometricId: string; // hashed
  confidence: number; // 0-1
  location: string;
  timestamp: Date;
  fallbackUsed: boolean;
}
```

#### 4.2.2 Occupancy Management

**Real-time Capacity Tracking:**
```typescript
interface OccupancyStatus {
  current: number;
  maximum: number;
  percentage: number;
  trend: 'increasing' | 'stable' | 'decreasing';
  forecast: {
    nextHour: number;
    peakTimeToday: { time: string; expected: number };
  };
  zoneOccupancy: {
    cardio: { current: number; max: number };
    weights: { current: number; max: number };
    functional: { current: number; max: number };
    groupClasses: { current: number; max: number };
    lockerRooms: { current: number; max: number };
  };
}
```

---


## 5. Automated Workout Programs

### 5.1 AI Workout Generation

#### 5.1.1 Progressive Overload Algorithm

**Base Formula:**
```
Training Load = Volume × Intensity × Frequency

Volume = Σ(Weight × Reps × Sets) for all exercises
Intensity = % of 1RM or RPE scale
Frequency = Workouts per week

Progressive Increase Per Week:
  Beginner: 5-10% volume increase
  Intermediate: 2-5% volume increase
  Advanced: 1-3% volume increase

Deload Week (every 4-6 weeks):
  Volume: 40-60% of peak week
  Intensity: Maintain or slight decrease
```

#### 5.1.2 Exercise Selection Matrix

**Compound Movement Priority:**
```typescript
interface ExerciseDatabase {
  id: string;
  name: string;
  category: 'compound' | 'isolation' | 'cardio' | 'flexibility';
  primaryMuscles: string[];
  secondaryMuscles: string[];
  equipment: string[];
  difficulty: 1 | 2 | 3 | 4 | 5;
  injuryRisk: 'low' | 'medium' | 'high';
  learningCurve: 'easy' | 'moderate' | 'difficult';
  alternatives: string[]; // substitute exercises
  contraindications: string[]; // injuries/conditions to avoid
  biomechanics: {
    movementPlane: 'sagittal' | 'frontal' | 'transverse' | 'multi-plane';
    movementPattern: 'push' | 'pull' | 'squat' | 'hinge' | 'carry' | 'rotate';
    rangeOfMotion: 'full' | 'partial';
  };
}
```

**Weekly Split Patterns:**

```typescript
// Push/Pull/Legs (PPL) - 6 days
const pplSplit = {
  day1: { focus: 'push-upper', exercises: ['bench-press', 'overhead-press', 'triceps'] },
  day2: { focus: 'pull-upper', exercises: ['deadlift', 'rows', 'biceps'] },
  day3: { focus: 'legs', exercises: ['squat', 'leg-press', 'calves'] },
  day4: { focus: 'push-upper', exercises: ['incline-press', 'dips', 'lateral-raises'] },
  day5: { focus: 'pull-upper', exercises: ['pull-ups', 'cable-rows', 'face-pulls'] },
  day6: { focus: 'legs', exercises: ['front-squat', 'romanian-deadlift', 'lunges'] },
  day7: { focus: 'rest' }
};

// Upper/Lower - 4 days
const upperLowerSplit = {
  day1: { focus: 'upper-strength', exercises: ['bench-press', 'rows', 'overhead-press'] },
  day2: { focus: 'lower-strength', exercises: ['squat', 'deadlift', 'leg-curl'] },
  day3: { focus: 'rest' },
  day4: { focus: 'upper-hypertrophy', exercises: ['incline-db-press', 'pull-ups', 'curls'] },
  day5: { focus: 'lower-hypertrophy', exercises: ['leg-press', 'rdl', 'leg-extension'] },
  day6: { focus: 'rest' },
  day7: { focus: 'rest' }
};

// Full Body - 3 days
const fullBodySplit = {
  day1: { focus: 'full-body-a', exercises: ['squat', 'bench-press', 'rows', 'abs'] },
  day2: { focus: 'rest' },
  day3: { focus: 'full-body-b', exercises: ['deadlift', 'overhead-press', 'pull-ups', 'core'] },
  day4: { focus: 'rest' },
  day5: { focus: 'full-body-c', exercises: ['front-squat', 'incline-press', 'cable-rows'] },
  day6: { focus: 'rest' },
  day7: { focus: 'rest' }
};
```

### 5.2 Periodization Models

#### 5.2.1 Linear Periodization (12 weeks)

```
Phase 1: Hypertrophy (Weeks 1-4)
  Sets: 3-4
  Reps: 8-12
  Intensity: 65-75% 1RM
  Rest: 60-90 seconds
  Volume: High

Phase 2: Strength (Weeks 5-8)
  Sets: 4-5
  Reps: 4-6
  Intensity: 80-85% 1RM
  Rest: 2-3 minutes
  Volume: Medium-High

Phase 3: Power (Weeks 9-11)
  Sets: 3-5
  Reps: 1-3
  Intensity: 85-95% 1RM
  Rest: 3-5 minutes
  Volume: Medium

Week 12: Deload/Test
  Test 1RM on key lifts
  Active recovery
  Plan next cycle
```

#### 5.2.2 Undulating Periodization (Weekly)

```
Monday: Power Day
  Focus: Explosive movements
  Intensity: 75-85% 1RM
  Reps: 3-5
  Exercises: Olympic lifts, jump squats, plyometrics

Wednesday: Hypertrophy Day
  Focus: Muscle growth
  Intensity: 65-75% 1RM
  Reps: 8-12
  Exercises: Isolation work, volume training

Friday: Strength Day
  Focus: Maximum strength
  Intensity: 85-95% 1RM
  Reps: 2-4
  Exercises: Heavy compound lifts
```

### 5.3 Auto-Regulation Features

**RPE-Based Adjustment:**
```typescript
interface AutoRegulation {
  plannedLoad: {
    exercise: string;
    sets: number;
    reps: number;
    weight: number;
    targetRPE: number;
  };

  actualPerformance: {
    completedSets: number;
    completedReps: number[];
    actualRPE: number[];
    barVelocity?: number[]; // m/s if VBT sensors available
  };

  adjustment: {
    nextSession: {
      weight: number; // adjusted based on RPE
      reps: number; // adjusted if RPE too high/low
      sets: number;
    };
    reason: string;
    confidence: number; // AI confidence in adjustment
  };
}
```

**Fatigue Monitoring:**
```typescript
interface FatigueScore {
  readinessScore: number; // 1-10
  factors: {
    sleepQuality: number;
    sleepDuration: number;
    muscularSoreness: number;
    stressLevel: number;
    restingHeartRate: number;
    hrvScore: number;
  };
  recommendation: {
    action: 'proceed-as-planned' | 'reduce-volume' | 'deload' | 'rest-day';
    volumeAdjustment: number; // percentage
    intensityAdjustment: number; // percentage
  };
}
```

---


## 6. Facility IoT

### 6.1 Environmental Monitoring

#### 6.1.1 Air Quality Sensors

**Measurement Parameters:**
```typescript
interface AirQuality {
  timestamp: Date;
  location: string;

  temperature: {
    value: number; // Celsius
    ideal: { min: 18, max: 22 };
    status: 'too-cold' | 'optimal' | 'too-warm';
  };

  humidity: {
    value: number; // percentage
    ideal: { min: 40, max: 60 };
    status: 'too-dry' | 'optimal' | 'too-humid';
  };

  co2: {
    value: number; // ppm
    levels: {
      excellent: '< 600',
      good: '600-1000',
      acceptable: '1000-1500',
      poor: '1500-2000',
      bad: '> 2000'
    };
    status: 'excellent' | 'good' | 'acceptable' | 'poor' | 'bad';
    action: string; // "Increase ventilation" if poor/bad
  };

  pm25: {
    value: number; // μg/m³
    status: 'good' | 'moderate' | 'unhealthy';
  };

  voc: {
    value: number; // ppb (parts per billion)
    status: 'low' | 'medium' | 'high';
  };
}
```

**HVAC Integration:**
```typescript
interface HVACControl {
  zone: string;
  mode: 'auto' | 'cooling' | 'heating' | 'ventilation';
  targetTemperature: number;
  fanSpeed: 'low' | 'medium' | 'high' | 'auto';

  triggers: {
    co2Threshold: 1200; // ppm - increase ventilation
    temperatureDeviation: 2; // degrees - adjust HVAC
    occupancyFactor: true; // adjust based on crowd
    peakHourPreCool: true; // cool before peak hours
  };

  energyOptimization: {
    enabled: boolean;
    schedules: Array<{
      time: string;
      temperature: number;
      reason: string;
    }>;
  };
}
```

#### 6.1.2 Equipment Availability Sensors

**Real-time Equipment Status:**
```typescript
interface EquipmentAvailability {
  equipmentType: string;
  total: number;
  available: number;
  inUse: number;
  outOfService: number;
  reserved: number;

  currentUsers: Array<{
    equipmentId: string;
    memberId: string;
    startTime: Date;
    estimatedEndTime: Date;
    currentDuration: number;
  }>;

  waitlist: Array<{
    memberId: string;
    priority: number;
    waitingSince: Date;
    notified: boolean;
  }>;

  forecast: {
    availableIn5Min: number;
    availableIn15Min: number;
    peakWaitTime: number; // minutes
  };
}
```

### 6.2 Safety Systems

#### 6.2.1 Emergency Response

**Emergency Protocol:**
```typescript
interface EmergencySystem {
  emergencyButtons: Array<{
    id: string;
    location: string;
    type: 'panic' | 'medical' | 'fire';
    status: 'armed' | 'triggered' | 'acknowledged';
  }>;

  aedLocations: Array<{
    id: string;
    location: string;
    lastInspection: Date;
    batteryStatus: number;
    padsExpiry: Date;
  }>;

  staffAlerts: {
    onDutyStaff: string[];
    nearestStaff: Array<{
      staffId: string;
      location: string;
      distanceToIncident: number; // meters
      estimatedArrival: number; // seconds
    }>;
    externalEmergency: {
      policeNotified: boolean;
      ambulanceNotified: boolean;
      estimatedArrival: number; // minutes
    };
  };

  memberSafety: {
    evacuationRoutes: string[];
    assemblyPoints: string[];
    memberCount: {
      total: number;
      evacuated: number;
      unaccounted: number;
    };
  };
}
```

#### 6.2.2 Form Analysis & Injury Prevention

**Computer Vision Form Analysis:**
```typescript
interface FormAnalysis {
  exercise: string;
  cameraAngle: 'front' | 'side' | 'rear' | 'multi';

  poseKeypoints: {
    head: { x: number; y: number; confidence: number };
    shoulders: { left: Point; right: Point };
    elbows: { left: Point; right: Point };
    wrists: { left: Point; right: Point };
    hips: { left: Point; right: Point };
    knees: { left: Point; right: Point };
    ankles: { left: Point; right: Point };
  };

  angles: {
    kneeAngle: { left: number; right: number };
    hipAngle: { left: number; right: number };
    spineAngle: number;
    shoulderAngle: { left: number; right: number };
  };

  formScore: {
    overall: number; // 0-10
    components: {
      depth: number; // e.g., squat depth
      barPath: number; // vertical vs curved
      symmetry: number; // left/right balance
      tempo: number; // controlled vs jerky
      range: number; // full ROM vs partial
    };
  };

  warnings: Array<{
    severity: 'info' | 'warning' | 'danger';
    issue: string;
    description: string;
    recommendation: string;
  }>;

  injuryRisk: {
    overall: number; // 0-1
    areas: {
      lowerBack: number;
      knees: number;
      shoulders: number;
      wrists: number;
    };
    recommendation: 'continue' | 'reduce-weight' | 'stop-immediately';
  };
}
```

---


## 7. Virtual Training Systems

### 7.1 Live Streaming Classes

#### 7.1.1 WebRTC Architecture

**Streaming Configuration:**
```typescript
interface LiveClassStream {
  classId: string;
  instructor: {
    id: string;
    name: string;
    camera: {
      resolution: '1080p' | '4K';
      fps: 30 | 60;
      bitrate: number; // kbps
    };
    microphone: {
      sampleRate: 48000; // Hz
      bitrate: 128; // kbps
      noiseSuppression: boolean;
    };
  };

  participants: Array<{
    memberId: string;
    joinTime: Date;
    connectionQuality: 'excellent' | 'good' | 'fair' | 'poor';
    videoEnabled: boolean;
    audioEnabled: boolean;
    heartRateSharing: boolean;
    currentHeartRate?: number;
  }>;

  overlays: {
    instructorMetrics: boolean;
    leaderboard: boolean;
    personalStats: boolean;
    chatMessages: boolean;
  };

  interactivity: {
    participantVideo: boolean; // show participant cameras
    voiceInteraction: boolean;
    reactions: boolean; // emoji reactions
    polls: boolean;
  };
}
```

#### 7.1.2 Class Types

**Group Class Categories:**
```typescript
const classTypes = {
  spin: {
    duration: [30, 45, 60],
    intensity: ['beginner', 'intermediate', 'advanced'],
    style: ['endurance', 'intervals', 'climb', 'race'],
    equipment: ['stationary-bike'],
    maxParticipants: 30
  },

  hiit: {
    duration: [20, 30, 45],
    intensity: ['moderate', 'high', 'extreme'],
    style: ['bodyweight', 'kettlebell', 'mixed'],
    equipment: ['mat', 'dumbbells', 'kettlebells'],
    maxParticipants: 20
  },

  yoga: {
    duration: [30, 60, 90],
    intensity: ['gentle', 'moderate', 'power'],
    style: ['hatha', 'vinyasa', 'yin', 'restorative'],
    equipment: ['mat', 'blocks', 'straps'],
    maxParticipants: 25
  },

  strength: {
    duration: [45, 60],
    intensity: ['beginner', 'intermediate', 'advanced'],
    style: ['full-body', 'upper-body', 'lower-body', 'core'],
    equipment: ['barbell', 'dumbbells', 'resistance-bands'],
    maxParticipants: 15
  }
};
```

### 7.2 AR/VR Workouts

#### 7.2.1 Augmented Reality Features

**AR Overlay System:**
```typescript
interface ARWorkout {
  device: 'smartphone' | 'tablet' | 'ar-glasses';

  overlays: {
    exerciseDemo: {
      enabled: boolean;
      3dModel: string;
      animation: 'skeleton' | 'muscular' | 'full-body';
      opacity: number;
      position: 'mirror' | 'side-by-side';
    };

    formGuides: {
      enabled: boolean;
      alignmentLines: boolean;
      depthIndicators: boolean;
      angleMarkers: boolean;
      colorCoding: {
        good: 'green';
        warning: 'yellow';
        danger: 'red';
      };
    };

    liveMetrics: {
      heartRate: boolean;
      calories: boolean;
      repCount: boolean;
      setProgress: boolean;
      restTimer: boolean;
    };

    virtualTrainer: {
      enabled: boolean;
      avatar: string;
      voiceCoaching: boolean;
      motivationalMessages: boolean;
    };
  };

  gamification: {
    points: number;
    achievements: string[];
    challenges: Array<{
      name: string;
      progress: number;
      target: number;
      reward: string;
    }>;
  };
}
```

#### 7.2.2 Virtual Reality Environments

**VR Workout Scenarios:**
```typescript
interface VRWorkout {
  headset: 'meta-quest' | 'psvr' | 'htc-vive';

  environment: {
    scene: 'beach' | 'mountain' | 'forest' | 'gym' | 'space' | 'custom';
    weather: 'sunny' | 'rainy' | 'snowy' | 'foggy';
    timeOfDay: 'sunrise' | 'day' | 'sunset' | 'night';
    ambiance: {
      soundscape: boolean;
      music: string;
      visualEffects: boolean;
    };
  };

  workout: {
    type: 'boxing' | 'cycling' | 'rowing' | 'yoga' | 'dance';
    difficulty: 1 | 2 | 3 | 4 | 5;
    opponents?: Array<{ name: string; difficulty: number }>; // for boxing
    course?: { distance: number; terrain: string }; // for cycling
  };

  multiplayer: {
    enabled: boolean;
    participants: string[];
    mode: 'cooperative' | 'competitive';
    voiceChat: boolean;
  };

  biometrics: {
    heartRateMonitor: boolean;
    calorieTracking: boolean;
    motionTracking: '3dof' | '6dof';
  };
}
```

---


## 11. API Interface

### 11.1 RESTful Endpoints

**Base URL:** `https://api.smart-gym.wia.com/v1`

#### Member Management

```
POST   /members                     - Register new member
GET    /members/:id                 - Get member profile
PUT    /members/:id                 - Update member profile
DELETE /members/:id                 - Deactivate membership

POST   /members/:id/checkin         - Check-in member
POST   /members/:id/checkout        - Check-out member
GET    /members/:id/visits          - Get visit history
```

#### Workout Sessions

```
POST   /sessions                    - Start workout session
PUT    /sessions/:id                - Update session
POST   /sessions/:id/complete       - Complete session
GET    /sessions/:id                - Get session details
GET    /sessions/member/:memberId   - Get member's sessions

POST   /sessions/:id/exercises      - Log exercise
PUT    /sessions/:id/exercises/:exId - Update exercise
```

#### Equipment

```
GET    /equipment                   - List all equipment
GET    /equipment/:id               - Get equipment details
GET    /equipment/available         - Get available equipment
POST   /equipment/:id/reserve       - Reserve equipment
DELETE /equipment/:id/reserve       - Cancel reservation

GET    /equipment/:id/maintenance   - Get maintenance schedule
POST   /equipment/:id/maintenance   - Log maintenance
```

#### Workout Plans

```
POST   /plans/generate              - Generate AI workout plan
GET    /plans/:id                   - Get plan details
PUT    /plans/:id                   - Update plan
GET    /plans/member/:memberId      - Get member's plans
```

#### Analytics

```
GET    /analytics/member/:id        - Member analytics
GET    /analytics/facility          - Facility analytics
GET    /analytics/equipment/:id     - Equipment usage analytics
```

#### Virtual Training

```
GET    /classes                     - List scheduled classes
GET    /classes/:id                 - Get class details
POST   /classes/:id/join            - Join live class
POST   /classes/:id/book            - Book class
DELETE /classes/:id/booking         - Cancel booking
```

### 11.2 WebSocket Events

**Real-time Updates:**

```typescript
// Equipment status updates
socket.on('equipment:status', (data: {
  equipmentId: string;
  status: 'available' | 'in-use' | 'reserved' | 'maintenance';
  memberId?: string;
  estimatedEndTime?: Date;
}));

// Live workout metrics
socket.on('workout:metrics', (data: {
  sessionId: string;
  heartRate: number;
  calories: number;
  duration: number;
  currentExercise: string;
}));

// Form analysis warnings
socket.on('form:warning', (data: {
  sessionId: string;
  severity: 'info' | 'warning' | 'danger';
  message: string;
  recommendation: string;
}));

// Facility occupancy
socket.on('facility:occupancy', (data: {
  current: number;
  max: number;
  zones: Record<string, number>;
}));
```

---



## A.1 Endpoint reference

```http
POST /smartgym/v1/member/checkin       # member checkin at facility
POST /smartgym/v1/equipment/start      # session start on a piece of equipment
POST /smartgym/v1/equipment/end        # session end
POST /smartgym/v1/workout/program      # automated workout-program assignment
GET  /smartgym/v1/member/{id}/history   # member's session history (consented)
GET  /smartgym/v1/facility/health      # facility IoT health snapshot
```

Every endpoint follows the discovery convention at
`/.well-known/wia-smart-gym`.

## A.2 Member-management endpoint

The member-management API spans member registration, membership
upgrades, billing integration, and access-control (RFID / QR /
biometric checkin). Each operation emits a signed envelope; the
audit log preserves the chain for accounting and access reviews.

## A.3 Automated-workout-program endpoint

The workout-program API generates personalised programs from member
goals, fitness baseline, and past-session telemetry. The program is
a signed envelope the member receives in their app; the trainer (if
present) co-signs the program for liability and accountability.

## A.4 Facility-IoT endpoint

The facility-IoT endpoint exposes telemetry from connected sensors:
HVAC state, equipment occupancy, water consumption, energy
consumption. The endpoint is the basis for operational dashboards;
member-PII is never present in IoT telemetry.

## A.5 Virtual-training endpoint

Virtual-training sessions stream form-feedback to remote members
in real time. The endpoint accepts the trainer's signed exercise
prescription and the member's video stream; the standards latency
budget is documented so adaptation strategies (variable rate, lower
resolution) preserve the user experience under congestion.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-gym/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-gym-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-gym-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/smart-gym.sh` ships sample envelope generators with no dependencies
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
