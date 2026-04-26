# Chapter 3: WIA-IND-012 Standard Overview

## Overview

This chapter provides a comprehensive architectural overview of the WIA-IND-012 Fitness Tracking Standard, explaining how its various components work together to create a unified, interoperable fitness tracking ecosystem.

---

## 3.1 Architecture Overview

### System Components

```
┌─────────────────────────────────────────────────────────┐
│                 Fitness Tracking System                 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Activity   │  │  Heart Rate  │  │   Calorie    │  │
│  │  Tracker    │  │   Monitor    │  │  Calculator  │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Workout    │  │    Health    │  │     Goal     │  │
│  │   Logger    │  │   Metrics    │  │   Manager    │  │
│  └─────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │          Data Synchronization Layer             │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐  │
│  │          Privacy & Security Layer               │  │
│  └─────────────────────────────────────────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Layer Descriptions

**Component Layer** (Top)
- Specialized modules for different fitness tracking functions
- Modular design allows selective implementation
- Clear interfaces between components

**Data Synchronization Layer** (Middle)
- Handles multi-device consistency
- Conflict resolution
- Cloud and local storage orchestration

**Privacy & Security Layer** (Bottom)
- Encryption and access control
- Consent management
- Audit logging

---

## 3.2 Core Components

### 3.2.1 Activity Tracker

**Responsibilities:**
- Step counting and stride detection
- Distance measurement (GPS and estimated)
- Pace and speed calculation
- Elevation tracking
- Route recording

**Key Features:**
```typescript
interface ActivityTracker {
  // Real-time tracking
  startTracking(activityType: ActivityType): void;
  pauseTracking(): void;
  resumeTracking(): void;
  stopTracking(): ActivitySummary;

  // Step detection
  getStepCount(): number;
  getStrideLength(): number;
  getCadence(): number;

  // Distance and position
  getCurrentDistance(): number;
  getCurrentPosition(): GPSPoint;
  getRoute(): GPSPoint[];

  // Performance metrics
  getCurrentPace(): number;
  getCurrentSpeed(): number;
  getAveragePace(): number;
}
```

**Data Flow:**
```
Sensors (GPS, Accelerometer)
    → Raw Data Collection
    → Signal Processing & Filtering
    → Step/Movement Detection
    → Distance Calculation
    → Pace/Speed Derivation
    → Activity Summary
```

### 3.2.2 Heart Rate Monitor

**Responsibilities:**
- Real-time heart rate measurement
- Heart rate zone calculation
- HRV (Heart Rate Variability) analysis
- VO2 Max estimation
- Recovery metrics

**Key Features:**
```typescript
interface HeartRateMonitor {
  // Real-time monitoring
  getCurrentHeartRate(): number;
  getCurrentZone(): HeartRateZone;
  getHeartRateHistory(duration: number): HRData[];

  // Zone management
  calculateZones(maxHR: number, restingHR: number): HeartRateZones;
  getTimeInZones(): ZoneDistribution;

  // Advanced metrics
  calculateHRV(): HRVMetrics;
  estimateVO2Max(): number;
  getRecoveryHeartRate(): number;

  // Alerts
  setZoneAlerts(zones: ZoneAlert[]): void;
}
```

**Heart Rate Zone System:**
```
Zone 5 (Maximum)     ██████ 90-100% Max HR
Zone 4 (Threshold)   ██████ 80-90% Max HR
Zone 3 (Tempo)       ██████ 70-80% Max HR
Zone 2 (Aerobic)     ██████ 60-70% Max HR
Zone 1 (Recovery)    ██████ 50-60% Max HR
```

### 3.2.3 Calorie Calculator

**Responsibilities:**
- BMR (Basal Metabolic Rate) calculation
- TDEE (Total Daily Energy Expenditure) estimation
- Activity-based calorie calculation
- EPOC (afterburn) estimation

**Calculation Methods:**
```typescript
interface CalorieCalculator {
  // Baseline metabolism
  calculateBMR(profile: UserProfile): number;
  calculateTDEE(bmr: number, activityLevel: ActivityLevel): number;

  // Activity calories
  calculateActivityCalories(
    activity: Activity,
    duration: number,
    userWeight: number
  ): number;

  // Method selection
  useMETMethod(met: number, weight: number, duration: number): number;
  useHeartRateMethod(avgHR: number, profile: UserProfile): number;
  useVO2Method(vo2: number, weight: number, duration: number): number;

  // Advanced
  calculateEPOC(baseCalories: number, intensity: number): number;
}
```

**Formula Selection Logic:**
```
Priority Order:
1. VO2-based (if VO2 max known + HR available)
2. Heart rate-based (if HR monitor used)
3. MET-based (default for all activities)
4. User-adjusted (manual calibration factor)
```

### 3.2.4 Workout Logger

**Responsibilities:**
- Structured workout recording
- Interval and lap tracking
- Strength training set/rep logging
- Performance analysis
- Training plan management

**Data Structures:**
```typescript
interface WorkoutLogger {
  // Workout session management
  createWorkout(type: WorkoutType): Workout;
  saveWorkout(workout: Workout): void;
  getWorkout(id: string): Workout;
  listWorkouts(filters: WorkoutFilters): Workout[];

  // During workout
  addInterval(interval: Interval): void;
  recordLap(): void;
  addExercise(exercise: Exercise): void;
  addSet(exerciseId: string, set: Set): void;

  // Analysis
  analyzePerformance(workout: Workout): PerformanceMetrics;
  compareWorkouts(id1: string, id2: string): Comparison;
  calculateTrainingLoad(workout: Workout): number;
}
```

**Workout Types Supported:**
- Cardio: Running, Cycling, Swimming, Rowing
- Strength: Weight training, Bodyweight, CrossFit
- Sports: Basketball, Soccer, Tennis, Golf
- Mind-Body: Yoga, Pilates, Tai Chi, Meditation
- Other: HIIT, Circuit training, Walking, Hiking

### 3.2.5 Health Metrics Manager

**Responsibilities:**
- Body composition tracking
- Weight and BMI monitoring
- Sleep quality analysis
- Recovery score calculation

**Features:**
```typescript
interface HealthMetrics {
  // Body composition
  recordWeight(weight: number, date: Date): void;
  recordBodyComposition(data: BodyComposition): void;
  calculateBMI(weight: number, height: number): number;

  // Sleep tracking
  recordSleepSession(session: SleepSession): void;
  analyzeSleepQuality(session: SleepSession): SleepScore;

  // Recovery
  calculateRecoveryScore(): number;
  getReadinessToTrain(): number;

  // Trends
  getWeightTrend(days: number): TrendData;
  getHealthTrends(): HealthTrendReport;
}
```

**Body Composition Tracking:**
```
┌──────────────────────────────────────┐
│ Weight: 75.0 kg                      │
│ Body Fat: 15.2%                      │
│ Lean Mass: 63.6 kg                   │
│ Muscle Mass: 58.5 kg                 │
│ Bone Mass: 3.2 kg                    │
│ Water: 55.8%                         │
│ BMR: 1,685 kcal/day                  │
│ Metabolic Age: 28 years              │
└──────────────────────────────────────┘
```

### 3.2.6 Goal Manager

**Responsibilities:**
- Goal creation and tracking
- Progress monitoring
- Achievement system
- Reminders and notifications

**Goal Types:**
```typescript
interface GoalManager {
  // Goal CRUD
  createGoal(goal: FitnessGoal): string;
  updateGoal(id: string, updates: Partial<FitnessGoal>): void;
  deleteGoal(id: string): void;
  getGoal(id: string): FitnessGoal;

  // Progress
  updateProgress(id: string, value: number): void;
  getProgress(id: string): number;
  checkGoalCompletion(id: string): boolean;

  // Achievements
  unlockAchievement(achievement: Achievement): void;
  getAchievements(userId: string): Achievement[];
  getNextMilestone(userId: string): Achievement;
}
```

**Common Goal Types:**
- Daily steps target (e.g., 10,000 steps)
- Weekly distance (e.g., 25 km)
- Monthly workouts (e.g., 20 sessions)
- Weight loss/gain targets
- Race time goals
- Workout streaks

---

## 3.3 Data Flow Architecture

### 3.3.1 Real-Time Data Flow

```
┌──────────────┐
│   Sensors    │  (GPS, HR, Accelerometer, etc.)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│ Data Buffer  │  (Raw sensor readings)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│   Filters    │  (Noise reduction, smoothing)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│  Processors  │  (Step detection, HR zone calc)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│   Metrics    │  (Distance, pace, calories)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│ Local Store  │  (Encrypted local database)
└──────┬───────┘
       │
       ↓
┌──────────────┐
│  UI Update   │  (Real-time dashboard)
└──────────────┘
```

### 3.3.2 Synchronization Flow

```
┌──────────────┐         ┌──────────────┐
│   Device A   │         │   Device B   │
└──────┬───────┘         └──────┬───────┘
       │                        │
       │  Upload                │  Upload
       ↓                        ↓
┌─────────────────────────────────────┐
│          Cloud Storage              │
│  ┌─────────────────────────────┐   │
│  │   Conflict Resolution       │   │
│  │   Merge Strategy            │   │
│  │   Data Validation           │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
       │                        │
       │  Download              │  Download
       ↓                        ↓
┌──────────────┐         ┌──────────────┐
│   Device A   │         │   Device B   │
│  (Updated)   │         │  (Updated)   │
└──────────────┘         └──────────────┘
```

### 3.3.3 Analysis Pipeline

```
Raw Activity Data
    ↓
[Basic Metrics]
    ↓
[Advanced Analytics]
    ↓
[Machine Learning]
    ↓
[Insights & Recommendations]
    ↓
User Dashboard
```

**Processing Stages:**

1. **Basic Metrics** (Real-time)
   - Current pace, distance, HR
   - Elapsed time
   - Current zone

2. **Advanced Analytics** (Post-activity)
   - Training load calculation
   - Performance analysis
   - Comparative statistics

3. **Machine Learning** (Batch processing)
   - Pattern recognition
   - Anomaly detection
   - Predictive modeling

4. **Insights** (Periodic)
   - Trend analysis
   - Goal recommendations
   - Training optimization

---

## 3.4 Integration Points

### 3.4.1 Device Integration

**Wearable Devices:**
```
┌─────────────┐
│ Smartwatch  │ → Bluetooth LE → ┌──────────┐
└─────────────┘                   │          │
                                  │   App    │
┌─────────────┐                   │          │
│ HR Monitor  │ → ANT+ ────────→  │          │
└─────────────┘                   │          │
                                  └──────────┘
┌─────────────┐
│ Bike Sensor │ → Bluetooth ───→
└─────────────┘
```

**Connection Protocols:**
- Bluetooth Low Energy (BLE)
- ANT+
- Wi-Fi
- NFC (for pairing)

### 3.4.2 Platform Integration

**Health Platforms:**
```
┌──────────────┐
│  HealthKit   │ ←─┐
│   (Apple)    │   │
└──────────────┘   │
                   │
┌──────────────┐   │    ┌─────────────┐
│  Google Fit  │ ←─┼───→│ WIA-IND-012 │
└──────────────┘   │    │   Platform  │
                   │    └─────────────┘
┌──────────────┐   │
│ Samsung      │ ←─┘
│   Health     │
└──────────────┘
```

**Integration Methods:**
- OAuth 2.0 authentication
- REST API calls
- Webhook subscriptions
- Background sync

### 3.4.3 Third-Party Services

**Popular Integrations:**
- **Strava:** Social fitness network
- **MyFitnessPal:** Nutrition tracking
- **TrainingPeaks:** Training plans
- **Runkeeper:** Route sharing
- **MapMyRun:** Route discovery
- **Zwift:** Virtual training

**Data Exchange:**
```
WIA-IND-012 Standard Format
    ↓
[Adapter Layer]
    ↓
Third-Party Format
```

### 3.4.4 Healthcare Systems

**EHR Integration:**
```
WIA-IND-012 Data
    ↓
[FHIR Converter]
    ↓
FHIR Observation Resources
    ↓
EHR System
```

**FHIR Resource Mapping:**
- Activity → Observation (physical activity)
- Heart Rate → Observation (heart rate)
- Weight → Observation (body weight)
- Sleep → Observation (sleep)

---

## 3.5 Security Architecture

### 3.5.1 Defense in Depth

```
┌─────────────────────────────────────┐
│  Application Layer                  │
│  - Input validation                 │
│  - Output encoding                  │
│  - Business logic security          │
├─────────────────────────────────────┤
│  Authentication Layer               │
│  - OAuth 2.0                        │
│  - Multi-factor authentication      │
│  - Session management               │
├─────────────────────────────────────┤
│  Authorization Layer                │
│  - Role-based access control        │
│  - Resource-level permissions       │
│  - API rate limiting                │
├─────────────────────────────────────┤
│  Encryption Layer                   │
│  - TLS 1.3 in transit              │
│  - AES-256 at rest                 │
│  - Key management                   │
├─────────────────────────────────────┤
│  Network Layer                      │
│  - Firewall rules                   │
│  - DDoS protection                  │
│  - Intrusion detection              │
└─────────────────────────────────────┘
```

### 3.5.2 Data Privacy

**Privacy by Design Principles:**

1. **Data Minimization**
   - Collect only necessary data
   - Delete after retention period
   - Aggregate when possible

2. **User Control**
   - Granular consent management
   - Easy data export
   - Right to deletion

3. **Transparency**
   - Clear privacy policy
   - Data usage notifications
   - Access logs available

4. **Security**
   - Encryption everywhere
   - Access controls
   - Audit trails

### 3.5.3 Consent Management

```typescript
interface ConsentManager {
  // Consent collection
  requestConsent(purpose: ConsentPurpose): Promise<boolean>;
  updateConsent(purpose: ConsentPurpose, granted: boolean): void;
  getConsents(userId: string): ConsentRecord[];

  // Data access control
  canAccessData(userId: string, dataType: DataType): boolean;
  canShareData(userId: string, recipient: string): boolean;

  // Audit
  logDataAccess(userId: string, accessor: string, dataType: DataType): void;
  getAuditLog(userId: string): AuditEntry[];
}
```

---

## 3.6 Scalability and Performance

### 3.6.1 Horizontal Scaling

```
           ┌──────────────┐
           │ Load Balancer│
           └──────┬───────┘
                  │
      ┌───────────┼───────────┐
      │           │           │
┌─────▼────┐ ┌───▼──────┐ ┌─▼────────┐
│ Server 1 │ │ Server 2 │ │ Server 3 │
└─────┬────┘ └───┬──────┘ └─┬────────┘
      │          │          │
      └──────────┼──────────┘
                 │
         ┌───────▼────────┐
         │  Database      │
         │  Cluster       │
         └────────────────┘
```

**Scaling Strategies:**
- Stateless application servers
- Distributed caching (Redis)
- Database read replicas
- Microservices architecture
- Event-driven processing

### 3.6.2 Performance Optimization

**Caching Strategy:**
```
User Request
    ↓
[Browser Cache] (static assets)
    ↓
[CDN Cache] (API responses)
    ↓
[Application Cache] (computed metrics)
    ↓
[Database Cache] (query results)
    ↓
Database
```

**Latency Targets:**
- Real-time updates: < 100ms
- API responses: < 200ms
- Dashboard load: < 1s
- Report generation: < 5s
- Data sync: < 10s

---

## 3.7 Extensibility

### 3.7.1 Plugin Architecture

```typescript
interface FitnessPlugin {
  name: string;
  version: string;

  // Lifecycle
  initialize(context: PluginContext): void;
  shutdown(): void;

  // Hooks
  onActivityStart?(activity: Activity): void;
  onActivityEnd?(activity: Activity): void;
  onDataSync?(data: SyncData): void;

  // Custom processing
  processData?(data: any): any;

  // UI extensions
  renderWidget?(): JSX.Element;
}
```

**Plugin Examples:**
- Custom activity types (rock climbing, parkour)
- Advanced analytics (power curve analysis)
- Social features (group challenges)
- Equipment tracking (shoe mileage)
- Weather integration

### 3.7.2 Custom Metrics

```typescript
interface CustomMetric {
  id: string;
  name: string;
  unit: string;

  // Calculation
  calculate(activity: Activity): number;

  // Display
  format(value: number): string;
  getIcon(): string;

  // Aggregation
  aggregate(values: number[]): number;
}
```

---

## 3.8 Certification Levels

### Level 1: Basic
**Requirements:**
- ✓ Step counting
- ✓ Basic activity logging
- ✓ Manual calorie entry
- ✓ Simple goals

**Target:** Entry-level devices, basic apps

### Level 2: Standard
**Requirements:**
- ✓ All Level 1 features
- ✓ GPS tracking
- ✓ Heart rate monitoring
- ✓ Automatic calorie calculation
- ✓ Workout analysis

**Target:** Consumer smartwatches, fitness apps

### Level 3: Advanced
**Requirements:**
- ✓ All Level 2 features
- ✓ Multi-sport support
- ✓ HRV and VO2 Max
- ✓ Training load calculation
- ✓ Cross-platform sync

**Target:** Athlete-focused devices, pro apps

### Level 4: Professional
**Requirements:**
- ✓ All Level 3 features
- ✓ Medical-grade accuracy (±5%)
- ✓ Healthcare integration (FHIR)
- ✓ Research-grade export
- ✓ HIPAA/GDPR compliance

**Target:** Medical devices, clinical platforms

---

## Key Takeaways

✓ WIA-IND-012 uses a modular component architecture for flexibility

✓ Six core components handle activity, heart rate, calories, workouts, health, and goals

✓ Data flows through collection → processing → storage → analysis → presentation

✓ Multiple integration points support devices, platforms, and third-party services

✓ Security architecture implements defense in depth and privacy by design

✓ System designed for horizontal scalability and high performance

✓ Plugin architecture enables extensibility without core changes

✓ Four certification levels accommodate different use cases and requirements

---

**Next:** [Chapter 4: Data Format →](04-data-format.md)

---

© 2025 WIA Standards Committee. 弘익人間 (홍익인간) - Benefit All Humanity
