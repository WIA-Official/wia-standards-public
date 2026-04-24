# WIA Bionic Eye - Phase 4: Rehabilitation Protocol

## 목표
인공 시각 시스템 사용자의 재활 및 적응 프로토콜을 표준화합니다.

## 4.1 재활 프로그램 구조

```typescript
interface VisionRehabProgram {
  programId: string;
  version: string;

  // 프로그램 정보
  metadata: {
    name: string;
    nameKorean: string;
    description: string;
    implantType: ImplantType;
    totalWeeks: number;           // 전체 기간 (보통 12-24주)
    sessionsPerWeek: number;
    sessionDuration: number;      // minutes
    createdBy: string;
    evidenceLevel: string;
  };

  // 대상 환자
  targetPatient: {
    ageRange: [number, number];
    blindnessDuration: string;    // 실명 기간 범위
    blindnessType: BlindnessType[];
    cognitiveRequirement: string;
    motivationLevel: string;
    supportSystem: boolean;       // 가족/보호자 지원
  };

  // 재활 단계
  phases: RehabPhase[];

  // 진행 기준
  progressionCriteria: ProgressionCriteria;
}

enum BlindnessType {
  RETINITIS_PIGMENTOSA = 'retinitis_pigmentosa',  // 망막색소변성증
  AMD = 'amd',                    // 황반변성
  DIABETIC_RETINOPATHY = 'diabetic_retinopathy',
  GLAUCOMA = 'glaucoma',
  OPTIC_NEUROPATHY = 'optic_neuropathy',
  CORTICAL_BLINDNESS = 'cortical_blindness',
  CONGENITAL = 'congenital',     // 선천성
  TRAUMA = 'trauma',             // 외상성
}
```

## 4.2 재활 단계

```typescript
interface RehabPhase {
  phaseId: string;
  name: string;
  nameKorean: string;
  weekStart: number;
  weekEnd: number;

  // 단계 목표
  goals: PhaseGoal[];

  // 훈련 세션
  sessions: TrainingSession[];

  // 단계별 파라미터
  parameters: {
    stimulationIntensity: [number, number];  // % range
    sessionComplexity: 'basic' | 'intermediate' | 'advanced';
    environmentType: 'controlled' | 'semi_controlled' | 'real_world';
  };

  // 진입/종료 기준
  entryCriteria: Criterion[];
  exitCriteria: Criterion[];
}

// 재활 단계 예시
const STANDARD_PHASES: RehabPhase[] = [
  {
    phaseId: 'phase-1',
    name: 'Phosphene Perception',
    nameKorean: '인광 지각 훈련',
    weekStart: 1,
    weekEnd: 4,
    goals: [
      { description: '개별 인광 인식', targetMetric: 'phosphene_detection', target: 90 },
      { description: '밝기 구분', targetMetric: 'brightness_discrimination', target: 4 },
    ],
  },
  {
    phaseId: 'phase-2',
    name: 'Pattern Recognition',
    nameKorean: '패턴 인식 훈련',
    weekStart: 5,
    weekEnd: 8,
    goals: [
      { description: '단순 형태 인식', targetMetric: 'shape_recognition', target: 80 },
      { description: '방향 판단', targetMetric: 'orientation_accuracy', target: 85 },
    ],
  },
  {
    phaseId: 'phase-3',
    name: 'Object Localization',
    nameKorean: '객체 위치 파악',
    weekStart: 9,
    weekEnd: 12,
    goals: [
      { description: '물체 위치 파악', targetMetric: 'localization_accuracy', target: 80 },
      { description: '거리 추정', targetMetric: 'distance_estimation', target: 70 },
    ],
  },
  {
    phaseId: 'phase-4',
    name: 'Mobility Training',
    nameKorean: '이동 훈련',
    weekStart: 13,
    weekEnd: 16,
    goals: [
      { description: '실내 이동', targetMetric: 'indoor_navigation', target: 85 },
      { description: '장애물 회피', targetMetric: 'obstacle_avoidance', target: 90 },
    ],
  },
  {
    phaseId: 'phase-5',
    name: 'Daily Living Activities',
    nameKorean: '일상생활 훈련',
    weekStart: 17,
    weekEnd: 20,
    goals: [
      { description: '물체 찾기', targetMetric: 'object_finding', target: 75 },
      { description: '사람 인식', targetMetric: 'person_detection', target: 80 },
    ],
  },
  {
    phaseId: 'phase-6',
    name: 'Community Integration',
    nameKorean: '사회 통합 훈련',
    weekStart: 21,
    weekEnd: 24,
    goals: [
      { description: '실외 이동', targetMetric: 'outdoor_navigation', target: 70 },
      { description: '사회적 상호작용', targetMetric: 'social_interaction', target: 60 },
    ],
  },
];
```

## 4.3 훈련 세션

```typescript
interface TrainingSession {
  sessionId: string;
  sessionNumber: number;
  title: string;
  titleKorean: string;
  duration: number;             // minutes

  // 세션 구조
  warmUp: {
    duration: number;
    activities: WarmUpActivity[];
  };

  mainActivities: TrainingActivity[];

  coolDown: {
    duration: number;
    activities: CoolDownActivity[];
  };

  // 평가
  assessments: SessionAssessment[];
}

interface TrainingActivity {
  activityId: string;
  type: TrainingType;
  duration: number;
  difficulty: 1 | 2 | 3 | 4 | 5;

  // 활동 파라미터
  parameters: ActivityParameters;

  // 목표 메트릭
  targetMetrics: TargetMetric[];

  // 피드백 설정
  feedback: {
    visual: boolean;             // 화면 피드백
    auditory: boolean;           // 소리 피드백
    haptic: boolean;             // 진동 피드백
    verbal: boolean;             // 언어 지시
  };

  // 적응 설정
  adaptation: {
    autoAdjustDifficulty: boolean;
    successThreshold: number;    // % for level up
    failureThreshold: number;    // % for level down
  };
}

enum TrainingType {
  // 기초 지각 훈련
  PHOSPHENE_DETECTION = 'phosphene_detection',
  PHOSPHENE_COUNTING = 'phosphene_counting',
  BRIGHTNESS_DISCRIMINATION = 'brightness_discrimination',
  LOCATION_IDENTIFICATION = 'location_identification',

  // 패턴 인식 훈련
  SHAPE_RECOGNITION = 'shape_recognition',
  LETTER_RECOGNITION = 'letter_recognition',
  ORIENTATION_DETECTION = 'orientation_detection',
  MOTION_DETECTION = 'motion_detection',

  // 객체 인식 훈련
  OBJECT_DETECTION = 'object_detection',
  OBJECT_LOCALIZATION = 'object_localization',
  OBJECT_TRACKING = 'object_tracking',
  FACE_DETECTION = 'face_detection',

  // 이동 훈련
  DOOR_FINDING = 'door_finding',
  OBSTACLE_AVOIDANCE = 'obstacle_avoidance',
  PATH_FOLLOWING = 'path_following',
  STAIR_NAVIGATION = 'stair_navigation',

  // 일상생활 훈련
  TABLE_SETTING = 'table_setting',
  OBJECT_REACHING = 'object_reaching',
  READING_LARGE_TEXT = 'reading_large_text',
  PERSON_RECOGNITION = 'person_recognition',

  // 실외 훈련
  CROSSWALK_NAVIGATION = 'crosswalk_navigation',
  SIDEWALK_WALKING = 'sidewalk_walking',
  PUBLIC_TRANSPORT = 'public_transport',
}
```

## 4.4 평가 메트릭

```typescript
interface AssessmentMetrics {
  // 지각 능력
  perception: {
    phospheneThreshold: number;      // μA
    spatialResolution: number;       // degrees
    temporalResolution: number;      // Hz
    contrastSensitivity: number;     // log units
    brightnessLevels: number;        // 구분 단계
  };

  // 인식 능력
  recognition: {
    shapeAccuracy: number;           // %
    letterAccuracy: number;          // %
    objectAccuracy: number;          // %
    faceDetection: number;           // %
    motionDetection: number;         // %
  };

  // 공간 능력
  spatial: {
    localizationError: number;       // degrees
    distanceEstimation: number;      // % error
    depthPerception: number;         // score
    motionDirection: number;         // % accuracy
  };

  // 기능적 능력
  functional: {
    indoorMobility: number;          // score
    outdoorMobility: number;         // score
    obstacleAvoidance: number;       // %
    navigationAccuracy: number;      // %
    taskCompletion: number;          // %
  };

  // 삶의 질
  qualityOfLife: {
    visualFunctionIndex: number;     // VFQ-25
    independenceScore: number;       // 0-100
    satisfactionScore: number;       // 0-10
    confidenceLevel: number;         // 0-10
    socialParticipation: number;     // score
  };
}

// 표준화된 평가
interface StandardizedAssessments {
  // 시각 기능 평가
  functionalVision: {
    bvat: BASICVisionTest;           // Basic Assessment of Vision
    flvt: FunctionalLowVisionTest;   // Functional Low Vision Test
    mlvt: MelbourneLowVisionTest;    // Melbourne Low Vision ADL Index
  };

  // 이동 능력 평가
  mobility: {
    tmt: TiledMazeTest;              // 타일 미로 테스트
    oat: ObstacleAvoidanceTest;
    ort: OrientationTest;
  };

  // 일상생활 평가
  adl: {
    iadl: InstrumentalADL;
    badl: BasicADL;
  };

  // 삶의 질 평가
  qol: {
    neivfq25: NEIVFQ25;              // NEI VFQ-25
    impact: ImpactOfVisionImpairment;
    whoqol: WHOQOLBREF;
  };
}
```

## 4.5 재활 전문가 대시보드

```typescript
interface RehabilitationDashboard {
  // 환자 관리
  patients: {
    list: PatientInfo[];
    getPatient(id: string): PatientDetail;
    createPatient(info: PatientInfo): Patient;
    updatePatient(id: string, updates: Partial<PatientInfo>): Patient;
  };

  // 프로그램 관리
  programs: {
    assignProgram(patientId: string, programId: string): Assignment;
    customizeProgram(assignment: Assignment, modifications: Modification[]): void;
    trackProgress(patientId: string): ProgressReport;
  };

  // 세션 관리
  sessions: {
    scheduleSession(patientId: string, session: SessionSchedule): void;
    startSession(sessionId: string): LiveSession;
    endSession(sessionId: string, summary: SessionSummary): void;
    reviewSession(sessionId: string): SessionReview;
  };

  // 평가 관리
  assessments: {
    conductAssessment(patientId: string, type: AssessmentType): Assessment;
    compareAssessments(patientId: string, dates: Date[]): Comparison;
    generateReport(patientId: string, period: DateRange): Report;
  };

  // 장치 관리
  devices: {
    calibrateDevice(patientId: string): CalibrationResult;
    updatePhospheneMap(patientId: string): PhospheneMap;
    adjustParameters(patientId: string, params: DeviceParameters): void;
    checkDeviceHealth(patientId: string): DeviceHealth;
  };

  // 리포트
  reporting: {
    generateProgressReport(patientId: string): ProgressReport;
    generateInsuranceReport(patientId: string): InsuranceReport;
    exportData(patientId: string, format: 'pdf' | 'csv' | 'json'): Blob;
  };
}
```

## 4.6 환자용 앱

```typescript
interface PatientApp {
  // 홈 화면
  home: {
    todaySchedule: SessionSchedule[];
    currentProgress: ProgressSummary;
    notifications: Notification[];
    quickSettings: QuickSettings;
  };

  // 훈련
  training: {
    startDailyTraining(): TrainingSession;
    practiceModes: PracticeMode[];       // 자유 연습
    gamifiedExercises: Game[];           // 게임형 훈련
    trackPerformance(): PerformanceData;
  };

  // 설정
  settings: {
    brightness: number;                  // 0-100
    contrast: number;                    // 0-100
    zoom: number;                        // 1-4x
    edgeEnhancement: boolean;
    objectHighlighting: boolean;
    audioDescriptions: boolean;
    colorMode: 'grayscale' | 'high_contrast' | 'custom';
  };

  // 도움
  help: {
    tutorials: Tutorial[];
    faq: FAQ[];
    contactSupport(): void;
    emergencyContacts: Contact[];
  };

  // 커뮤니티
  community: {
    peerSupport: Forum;
    successStories: Story[];
    localEvents: Event[];
    mentorConnection: MentorProgram;
  };
}
```

## 4.7 가족/보호자 가이드

```typescript
interface CaregiverGuide {
  // 기본 이해
  basics: {
    howItWorks: Tutorial;
    expectations: Document;
    timeline: RehabTimeline;
    roleOfCaregiver: Guidelines;
  };

  // 일상 지원
  dailySupport: {
    deviceHandling: Instructions;
    chargingMaintenance: Checklist;
    troubleshooting: FAQ;
    safetyPrecautions: SafetyGuide;
  };

  // 훈련 지원
  trainingSupport: {
    homeExercises: Exercise[];
    environmentSetup: Guidelines;
    progressMonitoring: Checklist;
    motivationTips: Tips;
  };

  // 긴급 상황
  emergency: {
    warningSign: WarningSign[];
    emergencyProcedures: Procedure[];
    contactNumbers: Contact[];
  };

  // 자원
  resources: {
    supportGroups: Group[];
    financialAssistance: Resource[];
    insuranceGuidance: Guide;
    legalRights: Document;
  };
}
```

---

## 산출물

```
bionic-eye/
├── programs/
│   ├── standard-rehab-24week.json
│   ├── accelerated-12week.json
│   └── templates/
│       ├── session-template.json
│       └── assessment-template.json
├── apps/
│   ├── rehab-dashboard/
│   │   ├── src/
│   │   └── package.json
│   └── patient-app/
│       ├── src/
│       └── package.json
├── spec/
│   ├── REHAB-PROGRAM-SPEC.md
│   ├── TRAINING-ACTIVITIES-SPEC.md
│   ├── ASSESSMENT-METRICS-SPEC.md
│   └── CAREGIVER-GUIDE.md
└── docs/
    ├── SPECIALIST-GUIDE.md
    ├── PATIENT-MANUAL.md
    └── CAREGIVER-MANUAL.md
```

---

## 홍익인간

**"빛을 되찾는 여정을 함께합니다"**

*WIA Bionic Eye Standard*
