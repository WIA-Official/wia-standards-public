# WIA Exoskeleton - Phase 4: Rehabilitation Protocol

## 목표
외골격 기반 재활 훈련 프로토콜을 표준화합니다.

## 4.1 훈련 프로그램 정의

```typescript
interface RehabProgram {
  // 프로그램 정보
  metadata: {
    programId: string;
    name: string;
    targetCondition: Condition;  // SCI, Stroke, etc.
    difficulty: 'beginner' | 'intermediate' | 'advanced';
    duration: number;            // 주 (weeks)
  };

  // 세션 구성
  sessions: RehabSession[];

  // 진행 조건
  progressionCriteria: ProgressionCriteria;
}

interface RehabSession {
  sessionNumber: number;
  duration: number;              // 분

  // 훈련 활동
  activities: RehabActivity[];

  // 휴식
  restPeriods: RestPeriod[];
}

interface RehabActivity {
  type: ActivityType;
  duration: number;              // 분
  repetitions?: number;
  sets?: number;
  assistanceLevel: number;       // 0-100%
  targetMetrics: TargetMetrics;
}

enum ActivityType {
  STANDING = 'standing',
  SIT_TO_STAND = 'sit_to_stand',
  TREADMILL_WALKING = 'treadmill_walking',
  OVERGROUND_WALKING = 'overground_walking',
  STAIR_CLIMBING = 'stair_climbing',
  BALANCE_TRAINING = 'balance_training',
  STRENGTH_TRAINING = 'strength_training',
}
```

## 4.2 진척도 측정 메트릭

```typescript
interface ProgressMetrics {
  // 기능적 지표
  functional: {
    walkingSpeed: number;        // m/s
    walkingDistance: number;     // 6분 보행 검사 (m)
    stepsPerDay: number;
    standingTime: number;        // 분/일
  };

  // 생체역학적 지표
  biomechanical: {
    symmetry: number;            // 0-1 (좌우 대칭)
    jointROM: Record<JointType, number>;
    gaitDeviation: number;       // 정상 보행과의 편차
  };

  // 보조 의존도
  assistance: {
    currentLevel: number;        // %
    levelOverTime: TimeSeriesData;
    independenceScore: number;   // 0-100
  };

  // 환자 보고 결과
  patientReported: {
    painLevel: number;           // 0-10
    fatigueLevel: number;        // 0-10
    confidenceLevel: number;     // 0-10
    satisfactionScore: number;   // 0-10
  };
}

// 표준 평가 척도 호환
interface StandardizedAssessments {
  // 기능적 독립성 척도
  fim: FunctionalIndependenceMeasure;
  // 버그 균형 척도
  bergBalance: BergBalanceScale;
  // 10미터 보행 검사
  tenMeterWalk: TenMeterWalkTest;
  // 타임업앤고
  tugTest: TimedUpAndGoTest;
}
```

## 4.3 치료사 대시보드

```typescript
interface TherapistDashboard {
  // 환자 목록
  patients: Patient[];

  // 개별 환자 상세
  patientDetail: {
    profile: PatientProfile;
    currentProgram: RehabProgram;
    progressHistory: ProgressMetrics[];
    sessionHistory: SessionSummary[];
  };

  // 프로그램 관리
  programManagement: {
    createProgram: (params: ProgramParams) => RehabProgram;
    modifyProgram: (id: string, changes: Partial<RehabProgram>) => void;
    assignProgram: (patientId: string, programId: string) => void;
  };

  // 리포트 생성
  reporting: {
    generateProgressReport: (patientId: string, period: DateRange) => Report;
    exportData: (format: 'pdf' | 'csv' | 'json') => Blob;
  };

  // 원격 모니터링
  remoteMonitoring: {
    liveSession: (sessionId: string) => LiveSessionView;
    alerts: Alert[];
  };
}
```

## 4.4 WIA 에코시스템 통합

```typescript
interface WiaExoIntegration {
  // BCI 연동 (의도 감지)
  bci: {
    useForIntentDetection: boolean;
    combinedControl: BCIExoController;
  };

  // EMG 연동 (근전도 피드백)
  myoelectric: {
    muscleActivityMonitoring: boolean;
    emgTriggeredAssistance: boolean;
  };

  // AAC 연동 (의사소통)
  aac: {
    voiceCommandControl: boolean;
    statusCommunication: boolean;
  };

  // 스마트 휠체어 연동
  wheelchair: {
    seamlessTransition: boolean;  // 휠체어 ↔ 외골격
    combinedMobility: boolean;
  };
}
```

---

## 산출물

```
exoskeleton/
├── programs/
│   ├── sci-beginner.json
│   ├── stroke-intermediate.json
│   └── templates/
├── apps/
│   └── therapist-dashboard/
│       ├── src/
│       └── package.json
├── spec/
│   ├── REHAB-PROGRAM-SPEC.md
│   └── PROGRESS-METRICS.md
└── docs/
    └── THERAPIST-GUIDE.md
```

---

## 완료 체크리스트

- [ ] 운동 데이터 표준 (Phase 1)
- [ ] 제어 명령 표준 (Phase 2)
- [ ] 안전 프로토콜 (Phase 3)
- [ ] 재활 프로토콜 (Phase 4)
- [ ] 치료사 대시보드

## 홍익인간

다시 걸을 수 있다는 희망을 더 많은 사람들에게.
