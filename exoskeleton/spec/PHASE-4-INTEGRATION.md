# WIA Exoskeleton Rehabilitation Program Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-14
**Medical Device Classification:** IEC 62304 Class B

## 1. Overview

이 문서는 재활 외골격을 활용한 훈련 프로그램의 표준 구조를 정의합니다.
다양한 신경학적 상태(척수손상, 뇌졸중 등)에 대한 맞춤형 재활 프로토콜을 제공합니다.

## 2. Target Conditions

### 2.1 Supported Conditions

| Condition | Code | Description | Typical Duration |
|-----------|------|-------------|------------------|
| Spinal Cord Injury | SCI | 척수 손상 (완전/불완전) | 12-24 weeks |
| Stroke | STROKE | 뇌졸중 후 편마비 | 8-16 weeks |
| Traumatic Brain Injury | TBI | 외상성 뇌손상 | 12-20 weeks |
| Multiple Sclerosis | MS | 다발성 경화증 | Ongoing |
| Parkinson's Disease | PD | 파킨슨병 | Ongoing |
| Cerebral Palsy | CP | 뇌성마비 | Ongoing |
| Guillain-Barré Syndrome | GBS | 길랭-바레 증후군 | 8-16 weeks |
| Post-Surgical | POST_OP | 수술 후 재활 | 4-12 weeks |

### 2.2 Condition Severity Levels

```typescript
enum SeverityLevel {
  MILD = 'mild',           // 독립 보행 가능, 보조 필요
  MODERATE = 'moderate',   // 보조 기구로 보행 가능
  SEVERE = 'severe',       // 독립 보행 불가
  COMPLETE = 'complete',   // 완전 마비 (SCI ASIA A)
}

enum SCILevel {
  CERVICAL = 'cervical',      // C1-C8
  THORACIC = 'thoracic',      // T1-T12
  LUMBAR = 'lumbar',          // L1-L5
  SACRAL = 'sacral',          // S1-S5
}

enum StrokeSide {
  LEFT = 'left',              // 좌측 편마비
  RIGHT = 'right',            // 우측 편마비
  BILATERAL = 'bilateral',    // 양측성
}
```

## 3. Rehabilitation Program Structure

### 3.1 Program Definition

```typescript
interface RehabProgram {
  // Program identification
  programId: string;                    // UUID
  version: string;                      // Semantic version

  // Program metadata
  metadata: ProgramMetadata;

  // Target patient profile
  targetProfile: TargetProfile;

  // Program phases
  phases: RehabPhase[];

  // Progression criteria
  progressionCriteria: ProgressionCriteria;

  // Contraindications
  contraindications: Contraindication[];

  // Safety parameters
  safetyParameters: SafetyParameters;
}

interface ProgramMetadata {
  name: string;                         // 프로그램 이름
  nameKorean: string;                   // 한글 이름
  description: string;                  // 설명
  targetCondition: Condition;           // 대상 질환
  difficulty: DifficultyLevel;          // 난이도
  totalWeeks: number;                   // 전체 기간 (주)
  sessionsPerWeek: number;              // 주당 세션 수
  sessionDuration: number;              // 세션당 시간 (분)
  createdBy: string;                    // 작성자
  createdAt: Date;                      // 작성일
  approvedBy?: string;                  // 승인자
  approvedAt?: Date;                    // 승인일
  evidenceLevel: EvidenceLevel;         // 근거 수준
  references: string[];                 // 참고 문헌
}

enum DifficultyLevel {
  BEGINNER = 'beginner',
  INTERMEDIATE = 'intermediate',
  ADVANCED = 'advanced',
  CUSTOM = 'custom',
}

enum EvidenceLevel {
  LEVEL_1 = 'level_1',    // RCT 기반
  LEVEL_2 = 'level_2',    // 비무작위 연구
  LEVEL_3 = 'level_3',    // 관찰 연구
  LEVEL_4 = 'level_4',    // 전문가 의견
  CUSTOM = 'custom',      // 맞춤형
}
```

### 3.2 Target Profile

```typescript
interface TargetProfile {
  // Medical condition
  condition: Condition;
  severityRange: [SeverityLevel, SeverityLevel];

  // Physical requirements
  physical: {
    ageRange: [number, number];           // 연령 범위
    heightRange: [number, number];        // 신장 범위 (cm)
    weightRange: [number, number];        // 체중 범위 (kg)
    bmiRange?: [number, number];          // BMI 범위
  };

  // Functional requirements
  functional: {
    minStandingBalance: number;           // 최소 기립 균형 (초)
    minTrunkControl: TrunkControlLevel;   // 최소 체간 조절
    minUpperExtremityFunction: boolean;   // 상지 기능 필요 여부
    minCognitive: CognitiveLevel;         // 최소 인지 수준
  };

  // Prerequisites
  prerequisites: string[];                // 선행 조건
}

enum TrunkControlLevel {
  NONE = 'none',
  MINIMAL = 'minimal',
  FAIR = 'fair',
  GOOD = 'good',
  NORMAL = 'normal',
}

enum CognitiveLevel {
  FOLLOWS_SIMPLE = 'follows_simple',      // 간단한 지시 수행
  FOLLOWS_COMPLEX = 'follows_complex',    // 복잡한 지시 수행
  INDEPENDENT = 'independent',            // 독립적 판단
}
```

### 3.3 Rehabilitation Phase

```typescript
interface RehabPhase {
  phaseId: string;
  name: string;
  nameKorean: string;
  description: string;

  // Timing
  weekStart: number;                      // 시작 주차
  weekEnd: number;                        // 종료 주차

  // Goals
  goals: PhaseGoal[];

  // Sessions in this phase
  sessions: RehabSession[];

  // Phase-specific parameters
  parameters: PhaseParameters;

  // Entry criteria
  entryCriteria: Criterion[];

  // Exit criteria
  exitCriteria: Criterion[];
}

interface PhaseGoal {
  goalId: string;
  description: string;
  descriptionKorean: string;
  targetMetric: MetricType;
  targetValue: number;
  unit: string;
  priority: 'primary' | 'secondary' | 'optional';
}

interface PhaseParameters {
  // Assistance levels
  assistanceRange: [number, number];      // Min-max assistance (0-100%)

  // Speed limits
  maxWalkingSpeed: number;                // m/s

  // Duration limits
  maxContinuousWalking: number;           // 분
  minRestBetweenActivities: number;       // 초

  // Progression
  weeklyAssistanceReduction: number;      // % per week
  adaptiveAdjustment: boolean;            // 자동 조정 여부
}
```

### 3.4 Rehabilitation Session

```typescript
interface RehabSession {
  sessionId: string;
  sessionNumber: number;

  // Session info
  title: string;
  titleKorean: string;
  description: string;

  // Timing
  totalDuration: number;                  // 총 시간 (분)

  // Session structure
  warmUp: WarmUpPhase;
  mainActivities: RehabActivity[];
  coolDown: CoolDownPhase;

  // Rest periods
  restPeriods: RestPeriod[];

  // Therapist notes
  therapistNotes: string;

  // Expected outcomes
  expectedOutcomes: ExpectedOutcome[];
}

interface WarmUpPhase {
  duration: number;                       // 분
  activities: {
    type: WarmUpType;
    duration: number;
    description: string;
  }[];
}

enum WarmUpType {
  PASSIVE_ROM = 'passive_rom',            // 수동 관절 가동
  ACTIVE_ASSISTED_ROM = 'active_assisted_rom',
  STRETCHING = 'stretching',              // 스트레칭
  WEIGHT_SHIFTING = 'weight_shifting',    // 체중 이동
}

interface CoolDownPhase {
  duration: number;                       // 분
  activities: {
    type: CoolDownType;
    duration: number;
    description: string;
  }[];
}

enum CoolDownType {
  GENTLE_MOVEMENT = 'gentle_movement',
  STRETCHING = 'stretching',
  BREATHING = 'breathing',
  RELAXATION = 'relaxation',
}

interface RestPeriod {
  afterActivity: number;                  // 활동 인덱스
  duration: number;                       // 초
  type: 'standing' | 'sitting' | 'lying';
  hydration: boolean;                     // 수분 섭취 필요
}
```

### 3.5 Rehabilitation Activity

```typescript
interface RehabActivity {
  activityId: string;

  // Activity type
  type: ActivityType;

  // Timing
  duration: number;                       // 분
  order: number;                          // 순서

  // Repetition structure
  repetitions?: number;                   // 반복 횟수
  sets?: number;                          // 세트 수
  restBetweenSets?: number;               // 세트 간 휴식 (초)

  // Assistance
  assistanceLevel: number;                // 0-100%
  assistanceMode: AssistanceMode;         // 보조 모드

  // Parameters
  parameters: ActivityParameters;

  // Target metrics
  targetMetrics: TargetMetrics;

  // Instructions
  instructions: Instruction[];

  // Modifications
  modifications: ActivityModification[];
}

enum ActivityType {
  // Basic activities
  STANDING = 'standing',
  SIT_TO_STAND = 'sit_to_stand',
  STAND_TO_SIT = 'stand_to_sit',
  WEIGHT_SHIFTING = 'weight_shifting',

  // Walking activities
  TREADMILL_WALKING = 'treadmill_walking',
  OVERGROUND_WALKING = 'overground_walking',
  OUTDOOR_WALKING = 'outdoor_walking',

  // Advanced activities
  STAIR_CLIMBING = 'stair_climbing',
  STAIR_DESCENDING = 'stair_descending',
  RAMP_WALKING = 'ramp_walking',
  UNEVEN_TERRAIN = 'uneven_terrain',

  // Training activities
  BALANCE_TRAINING = 'balance_training',
  STRENGTH_TRAINING = 'strength_training',
  ENDURANCE_TRAINING = 'endurance_training',
  COORDINATION_TRAINING = 'coordination_training',

  // Functional activities
  TURNING = 'turning',
  OBSTACLE_CROSSING = 'obstacle_crossing',
  DUAL_TASK = 'dual_task',
}

enum AssistanceMode {
  PASSIVE = 'passive',              // 완전 보조
  ACTIVE_ASSIST = 'active_assist',  // 능동 보조
  ACTIVE_RESIST = 'active_resist',  // 저항 훈련
  TRANSPARENT = 'transparent',      // 최소 개입
  CHALLENGE = 'challenge',          // 도전 모드
}

interface ActivityParameters {
  // Walking parameters
  walking?: {
    speed: number;                        // m/s
    stepLength: number;                   // m
    cadence: number;                      // steps/min
    supportSurface: SupportSurface;       // 지지면
  };

  // Standing parameters
  standing?: {
    targetDuration: number;               // 초
    weightDistribution: [number, number]; // Left-Right %
    armSupport: 'none' | 'light' | 'moderate' | 'full';
  };

  // Stair parameters
  stairs?: {
    stepHeight: number;                   // cm
    numberOfSteps: number;
    handrailUse: 'required' | 'optional' | 'none';
    pattern: 'step_by_step' | 'reciprocal';
  };

  // Balance parameters
  balance?: {
    baseOfSupport: 'wide' | 'normal' | 'narrow' | 'tandem';
    surface: 'stable' | 'foam' | 'rocker';
    eyesOpen: boolean;
    dualTask: boolean;
  };
}

enum SupportSurface {
  TREADMILL = 'treadmill',
  INDOOR_FLAT = 'indoor_flat',
  OUTDOOR_FLAT = 'outdoor_flat',
  CARPET = 'carpet',
  GRASS = 'grass',
  GRAVEL = 'gravel',
}
```

### 3.6 Target Metrics

```typescript
interface TargetMetrics {
  // Primary targets
  primary: {
    metric: MetricType;
    target: number;
    unit: string;
    tolerance: number;                    // Acceptable deviation %
  }[];

  // Secondary targets
  secondary: {
    metric: MetricType;
    target: number;
    unit: string;
  }[];

  // Safety limits
  safetyLimits: {
    metric: MetricType;
    maxValue: number;
    unit: string;
  }[];
}

enum MetricType {
  // Walking metrics
  WALKING_SPEED = 'walking_speed',
  WALKING_DISTANCE = 'walking_distance',
  STEP_COUNT = 'step_count',
  CADENCE = 'cadence',
  STEP_LENGTH = 'step_length',

  // Time metrics
  STANDING_TIME = 'standing_time',
  WALKING_TIME = 'walking_time',
  TOTAL_ACTIVE_TIME = 'total_active_time',

  // Quality metrics
  GAIT_SYMMETRY = 'gait_symmetry',
  GAIT_DEVIATION = 'gait_deviation',
  BALANCE_SCORE = 'balance_score',

  // Assistance metrics
  ASSISTANCE_LEVEL = 'assistance_level',
  INDEPENDENCE_SCORE = 'independence_score',

  // Physiological metrics
  HEART_RATE = 'heart_rate',
  FATIGUE_LEVEL = 'fatigue_level',
  PAIN_LEVEL = 'pain_level',

  // Performance metrics
  REPETITIONS_COMPLETED = 'repetitions_completed',
  SETS_COMPLETED = 'sets_completed',
  TASK_COMPLETION_RATE = 'task_completion_rate',
}
```

### 3.7 Instructions

```typescript
interface Instruction {
  step: number;
  type: InstructionType;
  content: string;
  contentKorean: string;

  // Cues
  verbal?: string;                        // 언어 지시
  visual?: string;                        // 시각 지시
  tactile?: string;                       // 촉각 지시

  // Timing
  timing?: 'before' | 'during' | 'after';
  duration?: number;                      // 초
}

enum InstructionType {
  PREPARATION = 'preparation',            // 준비
  EXECUTION = 'execution',                // 수행
  FEEDBACK = 'feedback',                  // 피드백
  CORRECTION = 'correction',              // 교정
  ENCOURAGEMENT = 'encouragement',        // 격려
  SAFETY = 'safety',                      // 안전
}
```

### 3.8 Activity Modifications

```typescript
interface ActivityModification {
  modificationId: string;

  // Condition for modification
  condition: {
    trigger: ModificationTrigger;
    threshold: number;
    comparison: 'less_than' | 'greater_than' | 'equals';
  };

  // Modification action
  action: ModificationAction;

  // Description
  description: string;
  descriptionKorean: string;
}

enum ModificationTrigger {
  FATIGUE_LEVEL = 'fatigue_level',
  PAIN_LEVEL = 'pain_level',
  HEART_RATE = 'heart_rate',
  GAIT_DEVIATION = 'gait_deviation',
  BALANCE_SCORE = 'balance_score',
  TIME_ELAPSED = 'time_elapsed',
  REPETITIONS_COMPLETED = 'repetitions_completed',
  PATIENT_REQUEST = 'patient_request',
  THERAPIST_OVERRIDE = 'therapist_override',
}

interface ModificationAction {
  type: ModificationActionType;
  parameter: string;
  newValue?: number;
  adjustment?: number;                    // + or - adjustment
}

enum ModificationActionType {
  REDUCE_DURATION = 'reduce_duration',
  REDUCE_INTENSITY = 'reduce_intensity',
  INCREASE_ASSISTANCE = 'increase_assistance',
  SKIP_ACTIVITY = 'skip_activity',
  ADD_REST = 'add_rest',
  END_SESSION = 'end_session',
  ALERT_THERAPIST = 'alert_therapist',
}
```

## 4. Progression Criteria

### 4.1 Progression System

```typescript
interface ProgressionCriteria {
  // Automatic progression
  automatic: AutomaticProgression;

  // Manual progression (therapist decision)
  manual: ManualProgression;

  // Regression criteria
  regression: RegressionCriteria;
}

interface AutomaticProgression {
  // Criteria for auto-progression
  criteria: ProgressionCriterion[];

  // How many criteria must be met
  requiredCriteriaCount: number | 'all';

  // Minimum sessions before progression
  minSessionsInPhase: number;

  // Consistency requirement
  consistencyRequirement: {
    consecutiveSessions: number;
    percentageOfSessions: number;
  };
}

interface ProgressionCriterion {
  criterionId: string;
  name: string;
  nameKorean: string;

  metric: MetricType;
  target: number;
  unit: string;
  comparison: 'greater_than' | 'less_than' | 'equals' | 'within_range';
  range?: [number, number];

  weight: number;                         // 가중치 (0-1)
  required: boolean;                      // 필수 여부
}

interface ManualProgression {
  // Therapist assessment required
  assessmentRequired: boolean;

  // Assessment types needed
  assessmentTypes: AssessmentType[];

  // Supervisor approval required
  supervisorApproval: boolean;

  // Documentation required
  documentationRequired: string[];
}

enum AssessmentType {
  CLINICAL_OBSERVATION = 'clinical_observation',
  STANDARDIZED_TEST = 'standardized_test',
  FUNCTIONAL_ASSESSMENT = 'functional_assessment',
  PATIENT_INTERVIEW = 'patient_interview',
  CAREGIVER_INTERVIEW = 'caregiver_interview',
}

interface RegressionCriteria {
  // Conditions that trigger regression
  triggers: {
    metric: MetricType;
    threshold: number;
    consecutiveSessions: number;
  }[];

  // Actions on regression
  actions: {
    reducePhase: boolean;
    reduceWithinPhase: boolean;
    alertTherapist: boolean;
    requireReassessment: boolean;
  };
}
```

## 5. Contraindications

### 5.1 Contraindication Types

```typescript
interface Contraindication {
  contraindicationId: string;
  type: ContraindicationType;

  // Description
  name: string;
  nameKorean: string;
  description: string;

  // Severity
  severity: 'absolute' | 'relative';

  // Affected activities
  affectedActivities: ActivityType[] | 'all';

  // Required actions
  requiredActions: ContraindicationAction[];

  // Medical review
  medicalReviewRequired: boolean;
}

enum ContraindicationType {
  // Cardiovascular
  UNSTABLE_CARDIAC = 'unstable_cardiac',
  SEVERE_HYPERTENSION = 'severe_hypertension',
  DVT_RISK = 'dvt_risk',

  // Musculoskeletal
  FRACTURE = 'fracture',
  SEVERE_OSTEOPOROSIS = 'severe_osteoporosis',
  JOINT_CONTRACTURE = 'joint_contracture',
  HETEROTOPIC_OSSIFICATION = 'heterotopic_ossification',

  // Neurological
  AUTONOMIC_DYSREFLEXIA = 'autonomic_dysreflexia',
  SEVERE_SPASTICITY = 'severe_spasticity',
  UNCONTROLLED_SEIZURES = 'uncontrolled_seizures',

  // Skin
  PRESSURE_ULCER = 'pressure_ulcer',
  SKIN_BREAKDOWN = 'skin_breakdown',

  // Other
  ACUTE_INFECTION = 'acute_infection',
  COGNITIVE_IMPAIRMENT = 'cognitive_impairment',
  NON_COMPLIANCE = 'non_compliance',
}

interface ContraindicationAction {
  action: 'stop' | 'modify' | 'monitor' | 'consult';
  description: string;
  urgency: 'immediate' | 'within_session' | 'before_next_session';
}
```

## 6. Safety Parameters

### 6.1 Program Safety Configuration

```typescript
interface SafetyParameters {
  // Vital sign limits
  vitalLimits: {
    heartRateMax: number;                 // bpm
    heartRateMin: number;                 // bpm
    bloodPressureSystolicMax: number;     // mmHg
    bloodPressureDiastolicMax: number;    // mmHg
    oxygenSaturationMin: number;          // %
  };

  // Fatigue limits
  fatigueLimits: {
    maxFatigueLevel: number;              // 0-10
    maxBorgScale: number;                 // 6-20
    mandatoryRestThreshold: number;       // 0-10
  };

  // Pain limits
  painLimits: {
    maxPainLevel: number;                 // 0-10
    stopThreshold: number;                // 0-10
    newPainProtocol: 'stop' | 'modify' | 'monitor';
  };

  // Environmental limits
  environmentalLimits: {
    maxAmbientTemperature: number;        // °C
    minAmbientTemperature: number;        // °C
    maxHumidity: number;                  // %
  };

  // Session limits
  sessionLimits: {
    maxSessionDuration: number;           // 분
    maxContinuousActivity: number;        // 분
    minRestBetweenActivities: number;     // 초
    maxSessionsPerDay: number;
  };

  // Supervision requirements
  supervision: {
    minTherapistRatio: string;            // e.g., "1:1", "1:2"
    continuousMonitoring: boolean;
    remoteMonitoringAllowed: boolean;
  };
}
```

## 7. Program Templates

### 7.1 SCI Beginner Template

```json
{
  "programId": "wia-exo-sci-beginner-v1",
  "metadata": {
    "name": "SCI Beginner Gait Training",
    "nameKorean": "척수손상 초급 보행훈련",
    "targetCondition": "SCI",
    "difficulty": "beginner",
    "totalWeeks": 12,
    "sessionsPerWeek": 3,
    "sessionDuration": 60
  },
  "phases": [
    {
      "name": "Standing Tolerance",
      "nameKorean": "기립 적응",
      "weekStart": 1,
      "weekEnd": 3,
      "goals": [
        {
          "description": "Stand for 30 minutes with support",
          "targetValue": 30,
          "unit": "minutes"
        }
      ]
    },
    {
      "name": "Weight Shifting",
      "nameKorean": "체중 이동",
      "weekStart": 4,
      "weekEnd": 6,
      "goals": [
        {
          "description": "Achieve 60:40 weight shift",
          "targetValue": 60,
          "unit": "percent"
        }
      ]
    },
    {
      "name": "Treadmill Walking",
      "nameKorean": "트레드밀 보행",
      "weekStart": 7,
      "weekEnd": 9,
      "goals": [
        {
          "description": "Walk continuously for 15 minutes",
          "targetValue": 15,
          "unit": "minutes"
        }
      ]
    },
    {
      "name": "Overground Walking",
      "nameKorean": "지상 보행",
      "weekStart": 10,
      "weekEnd": 12,
      "goals": [
        {
          "description": "Walk 100m overground",
          "targetValue": 100,
          "unit": "meters"
        }
      ]
    }
  ]
}
```

### 7.2 Stroke Intermediate Template

```json
{
  "programId": "wia-exo-stroke-intermediate-v1",
  "metadata": {
    "name": "Stroke Intermediate Gait Training",
    "nameKorean": "뇌졸중 중급 보행훈련",
    "targetCondition": "STROKE",
    "difficulty": "intermediate",
    "totalWeeks": 8,
    "sessionsPerWeek": 4,
    "sessionDuration": 45
  },
  "phases": [
    {
      "name": "Symmetry Training",
      "nameKorean": "대칭성 훈련",
      "weekStart": 1,
      "weekEnd": 2,
      "goals": [
        {
          "description": "Achieve 45:55 weight distribution",
          "targetValue": 45,
          "unit": "percent"
        }
      ]
    },
    {
      "name": "Speed Training",
      "nameKorean": "속도 훈련",
      "weekStart": 3,
      "weekEnd": 5,
      "goals": [
        {
          "description": "Achieve walking speed of 0.6 m/s",
          "targetValue": 0.6,
          "unit": "m/s"
        }
      ]
    },
    {
      "name": "Endurance and Community",
      "nameKorean": "지구력 및 지역사회 보행",
      "weekStart": 6,
      "weekEnd": 8,
      "goals": [
        {
          "description": "6-minute walk distance 200m",
          "targetValue": 200,
          "unit": "meters"
        }
      ]
    }
  ]
}
```

## 8. API Interface

### 8.1 Program Management API

```typescript
interface IProgramManager {
  // Program CRUD
  createProgram(template: ProgramTemplate): RehabProgram;
  getProgram(programId: string): RehabProgram | null;
  updateProgram(programId: string, updates: Partial<RehabProgram>): RehabProgram;
  deleteProgram(programId: string): boolean;

  // Program assignment
  assignToPatient(programId: string, patientId: string, startDate: Date): Assignment;
  unassignFromPatient(assignmentId: string): boolean;

  // Program customization
  customizeForPatient(programId: string, patientId: string, modifications: Modification[]): RehabProgram;

  // Template management
  saveAsTemplate(program: RehabProgram): ProgramTemplate;
  listTemplates(filter?: TemplateFilter): ProgramTemplate[];

  // Validation
  validateProgram(program: RehabProgram): ValidationResult;
  checkContraindications(program: RehabProgram, patientId: string): ContraindicationCheck;
}
```

### 8.2 Session Execution API

```typescript
interface ISessionExecutor {
  // Session lifecycle
  startSession(sessionId: string, patientId: string): SessionInstance;
  pauseSession(instanceId: string, reason: string): void;
  resumeSession(instanceId: string): void;
  endSession(instanceId: string, summary: SessionSummary): SessionResult;

  // Activity execution
  startActivity(instanceId: string, activityId: string): ActivityInstance;
  completeActivity(activityInstanceId: string, result: ActivityResult): void;
  skipActivity(activityInstanceId: string, reason: string): void;
  modifyActivity(activityInstanceId: string, modification: Modification): void;

  // Real-time monitoring
  getCurrentMetrics(instanceId: string): CurrentMetrics;
  getActivityProgress(activityInstanceId: string): ActivityProgress;

  // Safety
  triggerSafetyStop(instanceId: string, reason: string): void;
  acknowledgeWarning(instanceId: string, warningId: string): void;
}
```

## 9. Integration with WIA Ecosystem

### 9.1 BCI Integration

```typescript
interface BCIRehabIntegration {
  // Intent detection enhancement
  useForProgression: boolean;             // Use mental effort for progression
  motorImageryTraining: boolean;          // Include MI tasks

  // Feedback
  bciNeurofeedback: boolean;              // Provide brain state feedback

  // Adaptive control
  adaptAssistanceFromBCI: boolean;        // Adjust assistance based on brain signals
}
```

### 9.2 EMG Integration

```typescript
interface EMGRehabIntegration {
  // Muscle monitoring
  muscleTargets: MuscleGroup[];           // Muscles to monitor

  // EMG-triggered assistance
  emgTriggeredMovement: boolean;          // Start movement on EMG threshold
  emgModulatedAssistance: boolean;        // Modulate assistance with EMG

  // Biofeedback
  emgBiofeedback: boolean;                // Show EMG to patient
  targetActivation: Record<MuscleGroup, number>;  // Target activation levels
}

enum MuscleGroup {
  TIBIALIS_ANTERIOR = 'tibialis_anterior',
  GASTROCNEMIUS = 'gastrocnemius',
  QUADRICEPS = 'quadriceps',
  HAMSTRINGS = 'hamstrings',
  HIP_FLEXORS = 'hip_flexors',
  GLUTEUS = 'gluteus',
}
```

## 10. Related Specifications

- [PROGRESS-METRICS.md](./PROGRESS-METRICS.md) - Progress measurement metrics
- [JOINT-STATE-SPEC.md](./JOINT-STATE-SPEC.md) - Joint state data format
- [CONTROL-MODES.md](./CONTROL-MODES.md) - Control modes for assistance
- [SAFETY-CHECKLIST.md](./SAFETY-CHECKLIST.md) - Safety checklists

## 11. Revision History

| Version | Date | Author | Description |
|---------|------|--------|-------------|
| 1.0.0 | 2025-12-14 | WIA | Initial specification |
