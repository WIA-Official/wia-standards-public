# WIA Bionic Eye - Phase 3: Safety Protocol

## 목표
인공 시각 시스템의 안전 프로토콜을 표준화합니다.

## 3.1 전기 안전

```typescript
interface ElectricalSafety {
  // 전하 밀도 제한 (Shannon 기준)
  chargeDensityLimits: {
    // Shannon 공식: log(Q/A) = k - log(Q)
    // Q: 전하 (μC), A: 면적 (cm²), k: 상수
    shannonK: number;             // 안전 k값 (1.5-1.85)
    maxChargeDensity: number;     // μC/cm² (보통 < 35)
    maxChargePerPhase: number;    // μC per electrode
  };

  // 전류 제한
  currentLimits: {
    perElectrodeMax: number;      // μA (보통 < 1000)
    totalArrayMax: number;        // mA (전체 동시 자극)
    peakCurrent: number;          // mA
    averageCurrent: number;       // mA
  };

  // DC 누설 방지 (필수!)
  dcProtection: {
    chargeBalancing: {
      enabled: boolean;
      method: 'passive' | 'active' | 'blocking_capacitor';
      balanceRatio: number;       // 0.99-1.01 (anodic/cathodic)
      maxDCOffset: number;        // nA (< 100nA)
    };
    blockingCapacitor: {
      present: boolean;
      capacitance: number;        // μF per electrode
      leakageCurrent: number;     // nA max
    };
  };

  // 임피던스 모니터링
  impedanceMonitoring: {
    measurementFrequency: number; // Hz (보통 1kHz)
    normalRange: [number, number]; // kΩ (예: 1-100)
    warningThreshold: number;     // kΩ
    criticalThreshold: number;    // kΩ (차단)
    checkInterval: 'continuous' | 'per_session' | 'daily';
    autoDisableOnAnomaly: boolean;
  };

  // 전압 제한
  voltageLimits: {
    compliance: number;           // V (최대 출력 전압)
    waterWindow: [number, number]; // V (전기분해 방지, 예: -0.6 to +0.8)
    monitoringEnabled: boolean;
  };
}
```

## 3.2 조직 안전

```typescript
interface TissueSafety {
  // 열 안전
  thermal: {
    maxTemperatureRise: number;   // °C (권장 < 1°C, 최대 2°C)
    maxPowerDissipation: number;  // mW
    thermalModel: 'simple' | 'finite_element';
    monitoring: {
      enabled: boolean;
      sensor: 'integrated' | 'external' | 'calculated';
      samplingRate: number;       // Hz
    };
    autoShutdown: {
      enabled: boolean;
      threshold: number;          // °C above baseline
      cooldownPeriod: number;     // seconds
    };
  };

  // 전기화학적 안전
  electrochemical: {
    electrodeMaterial: ElectrodeMaterial;
    chargeInjectionLimit: number; // mC/cm² (재료별 한계)
    potentialWindow: [number, number]; // V vs Ag/AgCl
    phMonitoring: boolean;        // 조직 pH 변화 감시
    gasEvolutionPrevention: boolean;
  };

  // 기계적 안전
  mechanical: {
    maxContactPressure: number;   // kPa (조직 압력)
    cableTension: {
      maxForce: number;           // mN
      strainRelief: boolean;
    };
    implantFixation: {
      method: 'tack' | 'suture' | 'adhesive' | 'self_retaining';
      migrationDetection: boolean;
    };
    flexibleInterconnects: boolean;
  };

  // 생체적합성
  biocompatibility: {
    encapsulation: string;        // 코팅 재료
    hermeticity: boolean;         // 밀봉성
    sterilizationMethod: string;
    shelfLife: number;            // months
    implantLifetime: number;      // years (예상)
  };
}
```

## 3.3 비상 정지 시스템

```typescript
interface EmergencyStopSystem {
  // 자동 트리거
  autoTriggers: {
    overCurrent: {
      enabled: boolean;
      threshold: number;          // μA
      responseTime: number;       // μs (< 100μs 권장)
    };
    overVoltage: {
      enabled: boolean;
      threshold: number;          // V
      responseTime: number;       // μs
    };
    impedanceAnomaly: {
      enabled: boolean;
      changeThreshold: number;    // % 급격한 변화
      absoluteThreshold: number;  // kΩ
      responseTime: number;       // ms
    };
    thermalOverload: {
      enabled: boolean;
      threshold: number;          // °C rise
      responseTime: number;       // ms
    };
    chargeImbalance: {
      enabled: boolean;
      threshold: number;          // % 불균형
      measurementWindow: number;  // ms
    };
  };

  // 수동 트리거
  manualTriggers: {
    patientButton: {
      enabled: boolean;
      location: 'glasses' | 'pendant' | 'wristband';
      confirmationRequired: boolean;
    };
    caregiverRemote: {
      enabled: boolean;
      range: number;              // meters
    };
    voiceCommand: {
      enabled: boolean;
      keywords: string[];         // ["stop", "emergency", "멈춰"]
    };
    appControl: {
      enabled: boolean;
      authentication: boolean;
    };
  };

  // 정지 시퀀스
  stopSequence: {
    // Phase 1: 즉시 (< 1ms)
    immediate: {
      disableAllOutputs: boolean;
      shortCircuitProtection: boolean;
      dischargeCapacitors: boolean;
    };
    // Phase 2: 빠름 (< 100ms)
    fast: {
      logEventData: boolean;
      notifySystem: boolean;
      safeStateTransition: boolean;
    };
    // Phase 3: 후속 (< 1s)
    followup: {
      alertCaregiver: boolean;
      alertMedicalTeam: boolean;
      saveSessionData: boolean;
    };
  };

  // 재시작 프로토콜
  restartProtocol: {
    autoRestartAllowed: boolean;
    manualOnly: boolean;
    cooldownPeriod: number;       // seconds
    checksRequired: {
      impedanceCheck: boolean;
      calibrationCheck: boolean;
      safetySystemsTest: boolean;
      patientConfirmation: boolean;
    };
    maxRestartAttempts: number;   // per session
    requireMedicalClearance: boolean;  // 특정 트리거 후
  };
}
```

## 3.4 지각 안전

```typescript
interface PerceptualSafety {
  // 광과민성 발작 방지 (PSE Prevention)
  photosensitivity: {
    maxFlashRate: number;         // Hz (< 3Hz 권장)
    maxBrightnessChange: number;  // % per frame
    forbiddenPatterns: string[];  // 금지 패턴
    screenTimeLimit: number;      // minutes per session
    breakReminder: {
      enabled: boolean;
      interval: number;           // minutes
      duration: number;           // minutes
    };
  };

  // 적응 과부하 방지
  adaptationLimits: {
    maxSessionDuration: number;   // minutes (권장 60-120)
    maxDailyUsage: number;        // hours
    mandatoryBreaks: {
      interval: number;           // minutes
      duration: number;           // minutes
      enforced: boolean;
    };
    fatigueDetection: {
      enabled: boolean;
      metrics: string[];          // 반응 시간, 정확도 등
      autoReduceIntensity: boolean;
    };
  };

  // 전정기관/균형 안전
  vestibularSafety: {
    motionCompensation: {
      enabled: boolean;
      imuIntegration: boolean;
      latencyCompensation: number; // ms
    };
    stabilizationMode: {
      enabled: boolean;
      transitionSmoothing: number; // ms
    };
    maxUpdateRate: number;        // Hz (지나친 변화 방지)
    nauseaPrevention: {
      enabled: boolean;
      conflictDetection: boolean;
    };
  };

  // 심리적 안전
  psychological: {
    gradualIntroduction: boolean; // 점진적 자극 증가
    comfortSettings: {
      brightness: [number, number]; // 편안한 범위
      contrast: [number, number];
    };
    distressDetection: {
      enabled: boolean;
      metrics: string[];
      autoReduce: boolean;
    };
    counselingReminder: boolean;  // 정기 상담 알림
  };
}
```

## 3.5 장기 모니터링

```typescript
interface LongTermMonitoring {
  // 정기 검진 일정
  checkupSchedule: {
    impedanceCheck: Frequency;
    thresholdRemapping: Frequency;
    visualFieldTest: Frequency;
    retinalImaging: Frequency;
    fullSystemCheck: Frequency;
    physicianReview: Frequency;
  };

  // 전극 열화 감지
  electrodeDegradation: {
    impedanceTrendAnalysis: {
      enabled: boolean;
      windowSize: number;         // days
      alertThreshold: number;     // % change
    };
    thresholdDrift: {
      enabled: boolean;
      significantChange: number;  // μA
    };
    electrodeFailure: {
      detection: boolean;
      autoDisable: boolean;
      remappingTrigger: boolean;
    };
    replacementPrediction: {
      enabled: boolean;
      model: string;
    };
  };

  // 조직 건강 평가
  tissueHealth: {
    retinalImaging: {
      oct: boolean;               // OCT 스캔
      fundusPhoto: boolean;       // 안저 사진
      fluoresceinAngio: boolean;  // 형광 조영
      frequency: Frequency;
    };
    electrophysiology: {
      erg: boolean;               // 망막전도검사
      vep: boolean;               // 시각유발전위
      frequency: Frequency;
    };
    inflammationMarkers: {
      monitoring: boolean;
      thresholds: Record<string, number>;
    };
  };

  // 데이터 수집 및 보고
  dataCollection: {
    usageStatistics: boolean;
    performanceMetrics: boolean;
    adverseEvents: boolean;
    patientFeedback: boolean;
    automaticUpload: boolean;
    privacyCompliance: string[];  // HIPAA, GDPR 등
  };

  // 경고 시스템
  alertSystem: {
    patientAlerts: {
      methods: ('app' | 'sms' | 'email' | 'call')[];
      urgencyLevels: AlertLevel[];
    };
    clinicianAlerts: {
      methods: ('app' | 'email' | 'pager')[];
      escalation: boolean;
    };
    manufacturerReporting: {
      enabled: boolean;
      adverseEventThreshold: string;
    };
  };
}

enum Frequency {
  CONTINUOUS = 'continuous',
  DAILY = 'daily',
  WEEKLY = 'weekly',
  MONTHLY = 'monthly',
  QUARTERLY = 'quarterly',
  YEARLY = 'yearly',
}

enum AlertLevel {
  INFO = 'info',
  ATTENTION = 'attention',
  WARNING = 'warning',
  URGENT = 'urgent',
  CRITICAL = 'critical',
}
```

---

## 산출물

```
bionic-eye/
├── spec/
│   ├── ELECTRICAL-SAFETY-SPEC.md
│   ├── TISSUE-SAFETY-SPEC.md
│   ├── EMERGENCY-STOP-SPEC.md
│   ├── PERCEPTUAL-SAFETY-SPEC.md
│   └── LONG-TERM-MONITORING-SPEC.md
├── api/
│   └── rust/
│       └── src/
│           └── safety/
│               ├── mod.rs
│               ├── electrical.rs
│               ├── tissue.rs
│               ├── emergency.rs
│               ├── perceptual.rs
│               └── monitoring.rs
└── prompts/
```
