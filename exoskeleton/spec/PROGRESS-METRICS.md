# WIA Exoskeleton Progress Metrics Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Medical Device Classification:** IEC 62304 Class B

## 1. Overview

이 문서는 재활 외골격 훈련의 진척도를 측정하고 평가하기 위한 메트릭 체계를 정의합니다.
객관적 데이터와 주관적 평가를 통합하여 환자의 재활 진행 상황을 추적합니다.

## 2. Metric Categories

### 2.1 Category Overview

| Category | Description | Data Source | Frequency |
|----------|-------------|-------------|-----------|
| Functional | 기능적 수행 능력 | System sensors | Every session |
| Biomechanical | 생체역학적 품질 | Motion capture | Every session |
| Assistance | 보조 의존도 | Control system | Every session |
| Physiological | 생리학적 반응 | Vital monitors | Continuous |
| Patient-Reported | 환자 자기 보고 | Questionnaires | Per session |
| Standardized | 표준화된 평가 | Clinical tests | Weekly/Monthly |

## 3. Functional Metrics

### 3.1 Walking Performance

```typescript
interface WalkingMetrics {
  // Speed metrics
  speed: {
    current: number;                      // m/s (현재)
    average: number;                      // m/s (평균)
    peak: number;                         // m/s (최고)
    selfSelected: number;                 // m/s (자가 선택)
    maximal: number;                      // m/s (최대)
  };

  // Distance metrics
  distance: {
    sessionTotal: number;                 // m (세션 총거리)
    continuousMax: number;                // m (연속 최대)
    sixMinuteWalk: number;                // m (6분 보행)
    dailyTotal?: number;                  // m (일일 총거리)
  };

  // Step metrics
  steps: {
    sessionTotal: number;                 // 세션 총걸음
    perMinute: number;                    // 분당 걸음 (cadence)
    dailyTotal?: number;                  // 일일 총걸음
  };

  // Duration metrics
  duration: {
    totalWalking: number;                 // 분 (총 보행)
    continuousMax: number;                // 분 (연속 최대)
    standingTime: number;                 // 분 (기립 시간)
  };
}
```

### 3.2 Walking Performance Thresholds

| Metric | Poor | Fair | Good | Excellent | Community |
|--------|------|------|------|-----------|-----------|
| Speed (m/s) | < 0.4 | 0.4-0.6 | 0.6-0.8 | 0.8-1.0 | > 1.0 |
| 6MWT (m) | < 100 | 100-200 | 200-300 | 300-400 | > 400 |
| Cadence (steps/min) | < 60 | 60-80 | 80-100 | 100-120 | > 120 |

### 3.3 Functional Tasks

```typescript
interface FunctionalTaskMetrics {
  // Sit-to-stand
  sitToStand: {
    time: number;                         // 초 (완료 시간)
    fiveTimesTest: number;                // 초 (5회 반복)
    assistanceUsed: number;               // % (사용된 보조)
    handSupport: boolean;                 // 손 지지 사용
    successRate: number;                  // % (성공률)
  };

  // Stair climbing
  stairClimbing: {
    stepsCompleted: number;               // 완료한 계단 수
    timePerStep: number;                  // 초/계단
    pattern: 'step_by_step' | 'reciprocal';
    handrailUse: boolean;
    assistanceLevel: number;              // %
  };

  // Turning
  turning: {
    time180: number;                      // 초 (180° 회전)
    stepsFor180: number;                  // 180° 회전 걸음 수
    stability: number;                    // 0-100 (안정성)
  };

  // Obstacle crossing
  obstacleCrossing: {
    heightCleared: number;                // cm (통과 높이)
    successRate: number;                  // % (성공률)
    hesitation: boolean;                  // 망설임 여부
  };
}
```

## 4. Biomechanical Metrics

### 4.1 Gait Quality

```typescript
interface GaitQualityMetrics {
  // Symmetry
  symmetry: {
    temporal: number;                     // 0-1 (시간적 대칭)
    spatial: number;                      // 0-1 (공간적 대칭)
    overall: number;                      // 0-1 (전체 대칭)
  };

  // Temporal parameters
  temporal: {
    stanceTimeLeft: number;               // 초
    stanceTimeRight: number;              // 초
    swingTimeLeft: number;                // 초
    swingTimeRight: number;               // 초
    doubleSupport: number;                // % of cycle
    singleSupport: number;                // % of cycle
  };

  // Spatial parameters
  spatial: {
    stepLengthLeft: number;               // m
    stepLengthRight: number;              // m
    strideLength: number;                 // m
    stepWidth: number;                    // m
    footClearance: number;                // cm
  };

  // Deviation from normal
  deviation: {
    gaitDeviationIndex: number;           // GDI (0-100)
    movementDeviationProfile: number[];   // Per joint
    overallDeviation: number;             // %
  };
}
```

### 4.2 Symmetry Calculation

```typescript
// Temporal symmetry ratio
function calculateTemporalSymmetry(
  stanceTimeLeft: number,
  stanceTimeRight: number
): number {
  const ratio = Math.min(stanceTimeLeft, stanceTimeRight) /
                Math.max(stanceTimeLeft, stanceTimeRight);
  return ratio; // 1.0 = perfect symmetry
}

// Spatial symmetry ratio
function calculateSpatialSymmetry(
  stepLengthLeft: number,
  stepLengthRight: number
): number {
  const ratio = Math.min(stepLengthLeft, stepLengthRight) /
                Math.max(stepLengthLeft, stepLengthRight);
  return ratio; // 1.0 = perfect symmetry
}

// Symmetry thresholds
interface SymmetryThresholds {
  normal: { min: 0.9, max: 1.0 };         // Normal symmetry
  mild: { min: 0.8, max: 0.9 };           // Mild asymmetry
  moderate: { min: 0.6, max: 0.8 };       // Moderate asymmetry
  severe: { min: 0.0, max: 0.6 };         // Severe asymmetry
}
```

### 4.3 Joint Kinematics

```typescript
interface JointKinematicsMetrics {
  // Range of motion achieved
  rom: {
    hipFlexion: number;                   // degrees
    hipExtension: number;                 // degrees
    kneeFlexion: number;                  // degrees
    kneeExtension: number;                // degrees
    ankleDorisflexion: number;            // degrees
    anklePlantarflexion: number;          // degrees
  };

  // ROM compared to target
  romAchievement: {
    hipFlexion: number;                   // % of target
    hipExtension: number;                 // % of target
    kneeFlexion: number;                  // % of target
    kneeExtension: number;                // % of target
    ankleDorisflexion: number;            // % of target
    anklePlantarflexion: number;          // % of target
  };

  // Peak angles during gait
  peakAngles: {
    hipFlexionSwing: number;              // degrees
    hipExtensionStance: number;           // degrees
    kneeFlexionSwing: number;             // degrees
    kneeExtensionStance: number;          // degrees
    ankleDorisflexionSwing: number;       // degrees
    anklePlantarflexionPushoff: number;   // degrees
  };

  // Angular velocity
  peakVelocities: {
    hip: number;                          // deg/s
    knee: number;                         // deg/s
    ankle: number;                        // deg/s
  };
}
```

### 4.4 Balance Metrics

```typescript
interface BalanceMetrics {
  // Center of pressure
  cop: {
    swayArea: number;                     // cm² (동요 면적)
    swayVelocity: number;                 // cm/s (동요 속도)
    pathLength: number;                   // cm (경로 길이)
    mlRange: number;                      // cm (좌우 범위)
    apRange: number;                      // cm (전후 범위)
  };

  // Weight distribution
  weightDistribution: {
    leftPercent: number;                  // %
    rightPercent: number;                 // %
    asymmetryIndex: number;               // 0-100
  };

  // Dynamic balance
  dynamic: {
    stabilityLimit: number;               // degrees (안정성 한계)
    reactionTime: number;                 // ms (반응 시간)
    recoveryTime: number;                 // ms (회복 시간)
  };

  // Functional balance
  functional: {
    singleLegStanceTime: number;          // 초
    tandemStanceTime: number;             // 초
    reachDistance: number;                // cm (뻗기 거리)
  };
}
```

## 5. Assistance Metrics

### 5.1 Assistance Level Tracking

```typescript
interface AssistanceMetrics {
  // Current assistance
  current: {
    overall: number;                      // 0-100%
    hip: number;                          // 0-100%
    knee: number;                         // 0-100%
    ankle: number;                        // 0-100%
  };

  // Assistance over time
  history: TimeSeriesData;

  // Independence calculation
  independence: {
    score: number;                        // 0-100
    level: IndependenceLevel;
    trend: 'improving' | 'stable' | 'declining';
  };

  // Assistance components
  components: {
    weightSupport: number;                // % (체중 지지)
    propulsion: number;                   // % (추진력)
    legSwing: number;                     // % (하지 스윙)
    balance: number;                      // % (균형)
  };
}

interface TimeSeriesData {
  timestamps: number[];
  values: number[];
  smoothed: number[];
  trend: {
    slope: number;
    confidence: number;
  };
}

enum IndependenceLevel {
  DEPENDENT = 'dependent',                // > 75% assistance
  MAXIMAL_ASSIST = 'maximal_assist',      // 50-75% assistance
  MODERATE_ASSIST = 'moderate_assist',    // 25-50% assistance
  MINIMAL_ASSIST = 'minimal_assist',      // 5-25% assistance
  SUPERVISION = 'supervision',            // < 5% but needs oversight
  INDEPENDENT = 'independent',            // 0%, no oversight needed
}
```

### 5.2 Assistance Reduction Algorithm

```typescript
interface AssistanceProgression {
  // Reduction strategy
  strategy: ReductionStrategy;

  // Current parameters
  currentReduction: {
    weeklyRate: number;                   // % per week
    sessionRate: number;                  // % per session
    minimumLevel: number;                 // Floor percentage
  };

  // Progression rules
  rules: ProgressionRule[];

  // Safety constraints
  safetyConstraints: {
    maxReductionPerSession: number;       // %
    minSessionsAtLevel: number;
    requiredStability: number;            // 0-1
  };
}

enum ReductionStrategy {
  LINEAR = 'linear',                      // 일정한 감소
  STEPPED = 'stepped',                    // 단계적 감소
  ADAPTIVE = 'adaptive',                  // 적응적 감소
  PERFORMANCE_BASED = 'performance_based' // 수행 기반 감소
}

interface ProgressionRule {
  ruleId: string;
  condition: {
    metric: MetricType;
    threshold: number;
    comparison: 'greater_than' | 'less_than';
    duration: number;                     // Sessions or days
  };
  action: {
    reduceBy: number;                     // %
    targetJoints: ('hip' | 'knee' | 'ankle')[] | 'all';
  };
}
```

## 6. Physiological Metrics

### 6.1 Cardiovascular Response

```typescript
interface CardiovascularMetrics {
  // Heart rate
  heartRate: {
    resting: number;                      // bpm
    current: number;                      // bpm
    peak: number;                         // bpm
    average: number;                      // bpm
    reserve: number;                      // % HRR used
    recovery: number;                     // bpm drop in 1 min
  };

  // Blood pressure
  bloodPressure?: {
    systolicPre: number;                  // mmHg
    diastolicPre: number;                 // mmHg
    systolicPost: number;                 // mmHg
    diastolicPost: number;                // mmHg
  };

  // Oxygen saturation
  spo2?: {
    current: number;                      // %
    minimum: number;                      // %
    desaturationEvents: number;           // Count
  };

  // Metabolic
  metabolic?: {
    vo2?: number;                         // mL/kg/min
    energyExpenditure: number;            // kcal
    metabolicEquivalent: number;          // METs
  };
}
```

### 6.2 Exercise Intensity Zones

| Zone | % HRmax | % HRR | Borg Scale | Description |
|------|---------|-------|------------|-------------|
| 1 - Very Light | 50-60% | 20-40% | 9-11 | Warm-up/Recovery |
| 2 - Light | 60-70% | 40-50% | 11-13 | Base training |
| 3 - Moderate | 70-80% | 50-60% | 13-15 | Aerobic training |
| 4 - Hard | 80-90% | 60-70% | 15-17 | Threshold training |
| 5 - Maximum | 90-100% | 70-80% | 17-20 | High intensity |

### 6.3 Fatigue Assessment

```typescript
interface FatigueMetrics {
  // Subjective measures
  subjective: {
    borgScale: number;                    // 6-20 (RPE)
    fatigueScale: number;                 // 0-10
    perceivedExertion: 'light' | 'moderate' | 'hard' | 'very_hard';
  };

  // Objective indicators
  objective: {
    performanceDecline: number;           // % from baseline
    reactionTimeIncrease: number;         // % from baseline
    movementVariability: number;          // CV increase
    compensatoryMovements: number;        // Count
  };

  // EMG-based (if available)
  muscularFatigue?: {
    medianFrequencyShift: number;         // Hz
    amplitudeIncrease: number;            // %
    activeMuscles: Record<MuscleGroup, number>;
  };

  // Recovery needs
  recovery: {
    recommendedRest: number;              // minutes
    nextSessionReady: boolean;
    recoveryScore: number;                // 0-100
  };
}
```

## 7. Patient-Reported Outcomes

### 7.1 Session-Level Reporting

```typescript
interface PatientReportedOutcomes {
  // Pain assessment
  pain: {
    level: number;                        // 0-10 NRS
    location?: string[];                  // Body locations
    type?: PainType[];                    // Pain characteristics
    aggravatingFactors?: string[];
    relievingFactors?: string[];
  };

  // Comfort and satisfaction
  comfort: {
    deviceComfort: number;                // 0-10
    fitRating: number;                    // 0-10
    pressurePoints?: string[];
  };

  // Confidence and motivation
  psychological: {
    confidenceLevel: number;              // 0-10
    motivationLevel: number;              // 0-10
    anxietyLevel: number;                 // 0-10
    satisfactionScore: number;            // 0-10
  };

  // Open feedback
  feedback?: {
    positives: string[];
    concerns: string[];
    suggestions: string[];
  };
}

enum PainType {
  SHARP = 'sharp',
  DULL = 'dull',
  ACHING = 'aching',
  BURNING = 'burning',
  TINGLING = 'tingling',
  CRAMPING = 'cramping',
}
```

### 7.2 Periodic Questionnaires

```typescript
interface PeriodicQuestionnaires {
  // Quality of life
  qualityOfLife: {
    sf36?: SF36Scores;                    // SF-36
    eq5d?: EQ5DScores;                    // EQ-5D
    whoqol?: WHOQOLScores;                // WHOQOL-BREF
  };

  // Condition-specific
  conditionSpecific: {
    sci?: {
      scim: SCIMScores;                   // Spinal Cord Independence Measure
      wisci: number;                      // Walking Index for SCI
    };
    stroke?: {
      sisShort: SISShortScores;           // Stroke Impact Scale
      abcScale: number;                   // Activities-specific Balance Confidence
    };
  };

  // Goals
  goals: {
    goalAttainmentScale: GASScores;       // GAS
    patientGoals: PatientGoal[];
  };
}

interface PatientGoal {
  goalId: string;
  description: string;
  baseline: number;
  target: number;
  current: number;
  importance: number;                     // 0-10
  difficulty: number;                     // 0-10
  achievementLevel: GASLevel;
}

enum GASLevel {
  MUCH_LESS = -2,
  LESS = -1,
  EXPECTED = 0,
  MORE = 1,
  MUCH_MORE = 2,
}
```

## 8. Standardized Assessments

### 8.1 Functional Independence Measure (FIM)

```typescript
interface FIMScores {
  // Locomotion
  locomotion: {
    walkWheelchair: number;               // 1-7
    stairs: number;                       // 1-7
  };

  // Total scores
  motorScore: number;                     // 13-91
  cognitiveScore: number;                 // 5-35
  totalScore: number;                     // 18-126

  // Change from baseline
  changeFromBaseline: number;
  minimalClinicallyImportantDifference: boolean;  // MCID = 22 points
}

// FIM Levels
const FIM_LEVELS = {
  7: 'Complete Independence',
  6: 'Modified Independence',
  5: 'Supervision',
  4: 'Minimal Assistance (75%+)',
  3: 'Moderate Assistance (50%+)',
  2: 'Maximal Assistance (25%+)',
  1: 'Total Assistance (<25%)',
};
```

### 8.2 Berg Balance Scale

```typescript
interface BergBalanceScores {
  items: {
    sittingToStanding: number;            // 0-4
    standingUnsupported: number;          // 0-4
    sittingUnsupported: number;           // 0-4
    standingToSitting: number;            // 0-4
    transfers: number;                    // 0-4
    standingEyesClosed: number;           // 0-4
    standingFeetTogether: number;         // 0-4
    reachingForward: number;              // 0-4
    pickingUpObject: number;              // 0-4
    turningToLookBehind: number;          // 0-4
    turning360: number;                   // 0-4
    placingFootOnStool: number;           // 0-4
    standingOneLegFront: number;          // 0-4
    standingOnOneLeg: number;             // 0-4
  };

  totalScore: number;                     // 0-56
  fallRisk: 'low' | 'medium' | 'high';    // Based on score

  // Interpretation
  // 0-20: Wheelchair bound
  // 21-40: Walking with assistance
  // 41-56: Independent
}

function calculateFallRisk(bergScore: number): 'low' | 'medium' | 'high' {
  if (bergScore >= 41) return 'low';
  if (bergScore >= 21) return 'medium';
  return 'high';
}
```

### 8.3 10-Meter Walk Test

```typescript
interface TenMeterWalkTest {
  // Self-selected speed
  selfSelected: {
    time: number;                         // seconds
    speed: number;                        // m/s
    steps: number;
  };

  // Fast speed
  fast: {
    time: number;                         // seconds
    speed: number;                        // m/s
    steps: number;
  };

  // Assistive device used
  assistiveDevice: AssistiveDevice;

  // Physical assistance level
  physicalAssistance: AssistanceLevel;

  // Classification
  functionalAmbulationCategory: number;   // 0-5 (FAC)
  communityAmbulator: boolean;            // Speed >= 0.8 m/s
}

enum AssistiveDevice {
  NONE = 'none',
  CANE = 'cane',
  CRUTCHES = 'crutches',
  WALKER = 'walker',
  PARALLEL_BARS = 'parallel_bars',
  EXOSKELETON = 'exoskeleton',
}
```

### 8.4 Timed Up and Go (TUG)

```typescript
interface TimedUpAndGoTest {
  // Times
  time: number;                           // seconds
  average: number;                        // Average of 3 trials

  // Component times
  components?: {
    standUp: number;                      // seconds
    walk: number;                         // seconds
    turn: number;                         // seconds
    walkBack: number;                     // seconds
    sitDown: number;                      // seconds
  };

  // Dual task
  dualTask?: {
    cognitiveTask: string;
    time: number;                         // seconds
    dualTaskCost: number;                 // % increase
    errors: number;
  };

  // Fall risk prediction
  fallRisk: 'low' | 'moderate' | 'high';
  // < 10s: freely mobile
  // 10-20s: mostly independent
  // 20-30s: variable mobility
  // > 30s: impaired mobility, often requires assistance
}
```

### 8.5 6-Minute Walk Test

```typescript
interface SixMinuteWalkTest {
  // Distance
  distance: number;                       // meters
  predictedDistance: number;              // Based on age, height, weight
  percentPredicted: number;               // %

  // Vitals
  vitals: {
    hrPre: number;
    hrPost: number;
    hrRecovery: number;                   // At 1 minute
    bpPre: [number, number];              // [systolic, diastolic]
    bpPost: [number, number];
    spo2Pre: number;
    spo2Min: number;
    spo2Post: number;
    borgPre: number;
    borgPost: number;
  };

  // Stops
  stops: {
    count: number;
    totalRestTime: number;                // seconds
    reasons: string[];
  };

  // Gait aids
  assistiveDevice: AssistiveDevice;
  supplementalO2: boolean;

  // Interpretation
  communityAmbulation: boolean;           // > 300m
  minimallyDetectableChange: number;      // meters
}

// Predicted distance calculation (Enright & Sherrill equation)
function predictSixMinuteWalkDistance(
  gender: 'male' | 'female',
  age: number,
  height: number,  // cm
  weight: number   // kg
): number {
  if (gender === 'male') {
    return (7.57 * height) - (5.02 * age) - (1.76 * weight) - 309;
  } else {
    return (2.11 * height) - (2.29 * weight) - (5.78 * age) + 667;
  }
}
```

## 9. Progress Visualization

### 9.1 Dashboard Metrics

```typescript
interface ProgressDashboard {
  // Summary scores
  summary: {
    overallProgress: number;              // 0-100%
    weeklyChange: number;                 // +/- %
    trend: 'improving' | 'stable' | 'declining';
    projectedCompletion: Date | null;
  };

  // Category scores
  categories: {
    functional: CategoryScore;
    biomechanical: CategoryScore;
    independence: CategoryScore;
    quality: CategoryScore;
  };

  // Key metrics
  keyMetrics: KeyMetric[];

  // Milestones
  milestones: Milestone[];

  // Alerts
  alerts: ProgressAlert[];
}

interface CategoryScore {
  current: number;                        // 0-100
  baseline: number;                       // 0-100
  target: number;                         // 0-100
  change: number;                         // +/- points
  percentToTarget: number;                // 0-100%
}

interface KeyMetric {
  metricId: string;
  name: string;
  nameKorean: string;
  current: number;
  baseline: number;
  target: number;
  unit: string;
  trend: TrendData;
  priority: 'primary' | 'secondary';
}

interface Milestone {
  milestoneId: string;
  name: string;
  nameKorean: string;
  description: string;
  targetDate: Date;
  achieved: boolean;
  achievedDate?: Date;
  metric: MetricType;
  targetValue: number;
}

interface ProgressAlert {
  alertId: string;
  type: 'positive' | 'warning' | 'action_required';
  message: string;
  messageKorean: string;
  metric?: MetricType;
  value?: number;
  recommendation?: string;
}
```

### 9.2 Trend Analysis

```typescript
interface TrendAnalysis {
  // Linear regression
  linear: {
    slope: number;
    intercept: number;
    rSquared: number;
    projectedValue: number;               // At program end
  };

  // Moving average
  movingAverage: {
    period: number;                       // Sessions
    current: number;
    values: number[];
  };

  // Rate of change
  rateOfChange: {
    perSession: number;
    perWeek: number;
    acceleration: number;                 // Is progress speeding up?
  };

  // Plateau detection
  plateau: {
    detected: boolean;
    duration: number;                     // Sessions
    recommendation: string;
  };
}
```

## 10. Reporting

### 10.1 Progress Report Structure

```typescript
interface ProgressReport {
  // Report metadata
  metadata: {
    reportId: string;
    patientId: string;
    generatedAt: Date;
    period: {
      start: Date;
      end: Date;
    };
    generatedBy: string;
  };

  // Executive summary
  executiveSummary: {
    overallStatus: 'on_track' | 'ahead' | 'behind' | 'complete';
    keyAchievements: string[];
    areasOfConcern: string[];
    recommendations: string[];
  };

  // Session summary
  sessionSummary: {
    sessionsCompleted: number;
    sessionsScheduled: number;
    attendanceRate: number;
    totalTrainingTime: number;            // minutes
    averageSessionDuration: number;       // minutes
  };

  // Metric details
  metrics: {
    functional: FunctionalMetricsReport;
    biomechanical: BiomechanicalMetricsReport;
    assistance: AssistanceMetricsReport;
    physiological: PhysiologicalMetricsReport;
    patientReported: PatientReportedReport;
    standardized: StandardizedAssessmentsReport;
  };

  // Goals progress
  goalsProgress: GoalProgressReport[];

  // Next steps
  nextSteps: {
    immediateActions: string[];
    programModifications: string[];
    referrals: string[];
    nextAssessmentDue: Date;
  };

  // Signatures
  signatures: {
    therapist: Signature;
    supervisor?: Signature;
    patient?: Signature;
  };
}
```

### 10.2 Report Export Formats

```typescript
interface ReportExporter {
  // Export methods
  exportToPDF(report: ProgressReport): Blob;
  exportToCSV(data: MetricData[]): Blob;
  exportToJSON(report: ProgressReport): string;
  exportToHL7FHIR(report: ProgressReport): FHIRBundle;

  // Templates
  templates: {
    clinical: ReportTemplate;
    research: ReportTemplate;
    insurance: ReportTemplate;
    patient: ReportTemplate;
  };

  // Customization
  customize(template: ReportTemplate, options: CustomizationOptions): ReportTemplate;
}
```

## 11. API Interface

### 11.1 Metrics Collection API

```typescript
interface IMetricsCollector {
  // Session metrics
  startMetricsCollection(sessionId: string): void;
  stopMetricsCollection(sessionId: string): SessionMetrics;

  // Real-time updates
  updateMetric(sessionId: string, metric: MetricUpdate): void;
  batchUpdateMetrics(sessionId: string, metrics: MetricUpdate[]): void;

  // Patient-reported
  submitPatientReported(sessionId: string, pro: PatientReportedOutcomes): void;

  // Standardized assessments
  recordAssessment(assessment: StandardizedAssessment): void;

  // Retrieval
  getSessionMetrics(sessionId: string): SessionMetrics;
  getPatientMetrics(patientId: string, dateRange: DateRange): PatientMetrics;
  getMetricHistory(patientId: string, metric: MetricType): TimeSeriesData;
}
```

### 11.2 Progress Analysis API

```typescript
interface IProgressAnalyzer {
  // Progress calculation
  calculateProgress(patientId: string): ProgressSummary;
  calculateCategoryProgress(patientId: string, category: MetricCategory): CategoryProgress;

  // Trend analysis
  analyzeTrends(patientId: string, metric: MetricType): TrendAnalysis;
  detectPlateau(patientId: string, metric: MetricType): PlateauDetection;

  // Predictions
  predictOutcome(patientId: string, targetMetric: MetricType): Prediction;
  estimateCompletionDate(patientId: string): Date | null;

  // Comparisons
  compareToBaseline(patientId: string): BaselineComparison;
  compareToNorms(patientId: string, normGroup: NormGroup): NormComparison;
  compareToPeers(patientId: string, peerGroup: PeerGroup): PeerComparison;

  // Reports
  generateProgressReport(patientId: string, period: DateRange): ProgressReport;
}
```

## 12. Related Specifications

- [REHAB-PROGRAM-SPEC.md](./REHAB-PROGRAM-SPEC.md) - Rehabilitation program structure
- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - Joint state data format
- [GAIT-CYCLE-SPEC.md](./GAIT-CYCLE-SPEC.md) - Gait cycle definitions
- [SESSION-DATA-SPEC.md](./SESSION-DATA-SPEC.md) - Session data structure

## 13. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
