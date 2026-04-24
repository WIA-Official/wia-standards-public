# WIA Bionic Eye - Phase 2: Stimulation Pattern Standard

## 목표
망막/시신경/피질 자극 패턴의 표준을 정의합니다.

## 2.1 자극 파라미터

```typescript
interface StimulationParameters {
  // 전기 자극 기본 파라미터
  electrical: {
    waveform: Waveform;
    amplitude: number;         // μA (마이크로암페어, 0-1000)
    pulseWidth: number;        // μs (마이크로초, 50-1000)
    frequency: number;         // Hz (1-1000)
    interphaseGap: number;     // μs (0-100)
    pulsesPerBurst: number;    // 버스트당 펄스 수 (1-20)
    burstDuration: number;     // ms
  };

  // 안전 한계
  safetyLimits: {
    maxChargeDensity: number;  // μC/cm² (< 35 권장)
    maxChargePerPhase: number; // nC (나노쿨롱)
    maxTotalCurrent: number;   // mA (전체 어레이)
    maxFrequency: number;      // Hz
    maxDutyCycle: number;      // % (< 50% 권장)
  };

  // 타이밍 제어
  timing: {
    frameRate: number;            // fps (자극 프레임율, 1-60)
    interElectrodeDelay: number;  // μs (전극 간 지연)
    refreshMode: 'synchronous' | 'sequential' | 'random';
  };
}

enum Waveform {
  BIPHASIC_SYMMETRIC = 'biphasic_symmetric',      // 가장 안전
  BIPHASIC_ASYMMETRIC = 'biphasic_asymmetric',    // 효율적
  BIPHASIC_CATHODIC_FIRST = 'biphasic_cathodic',  // 일반적
  BIPHASIC_ANODIC_FIRST = 'biphasic_anodic',
  TRIPHASIC = 'triphasic',                        // 전하 균형 최적
}
```

## 2.2 전극 어레이

```typescript
interface ElectrodeArray {
  // 어레이 식별
  arrayId: string;
  type: ImplantType;
  manufacturer: string;
  model: string;
  serialNumber: string;

  // 물리적 사양
  physical: {
    totalElectrodes: number;      // 총 전극 수
    activeElectrodes: number;     // 활성 전극 수
    rows: number;
    columns: number;
    electrodeDiameter: number;    // μm (100-500)
    electrodeSpacing: number;     // μm 중심 간 거리 (200-1000)
    electrodeArea: number;        // mm² (개별 전극)
    arrayDimensions: [number, number];  // mm x mm
    material: ElectrodeMaterial;
  };

  // 전극별 상태
  electrodes: ElectrodeInfo[];

  // 임피던스 측정
  impedance: {
    values: number[];             // kΩ per electrode
    measurementDate: Date;
    measurementFrequency: number; // Hz (보통 1kHz)
    normalRange: [number, number];
  };

  // 임플란트 위치
  placement: {
    location: ImplantLocation;
    eyeSide: 'left' | 'right' | 'bilateral';
    implantDate: Date;
    surgeon: string;
  };
}

enum ImplantType {
  EPIRETINAL = 'epiretinal',          // 망막 상부 (Argus II, IRIS)
  SUBRETINAL = 'subretinal',          // 망막 하부 (Alpha AMS, PRIMA)
  SUPRACHOROIDAL = 'suprachoroidal',  // 맥락막 상부
  INTRASCLERAL = 'intrascleral',      // 공막 내
  OPTIC_NERVE = 'optic_nerve',        // 시신경 커프
  LGN = 'lgn',                        // 외측슬상핵
  CORTICAL = 'cortical',              // 시각 피질 (Orion, ICVP)
}

enum ElectrodeMaterial {
  PLATINUM = 'platinum',
  PLATINUM_IRIDIUM = 'platinum_iridium',
  IRIDIUM_OXIDE = 'iridium_oxide',         // IrOx
  TITANIUM_NITRIDE = 'titanium_nitride',   // TiN
  PEDOT = 'pedot',                         // 전도성 고분자
  CARBON_NANOTUBE = 'carbon_nanotube',     // CNT
}

interface ElectrodeInfo {
  index: number;
  position: { row: number; col: number };
  functional: boolean;
  impedance: number;              // kΩ
  threshold: number;              // μA (지각 역치)
  maxSafeCurrent: number;         // μA
  chargeCapacity: number;         // mC/cm²
}
```

## 2.3 인광(Phosphene) 매핑

```typescript
interface PhospheneMap {
  patientId: string;
  arrayId: string;
  mappingDate: Date;
  version: number;

  // 개별 인광 정의
  phosphenes: PhospheneDefinition[];

  // 시야 커버리지
  visualField: {
    horizontalExtent: number;     // degrees (좌우 총 범위)
    verticalExtent: number;       // degrees (상하 총 범위)
    centerOffset: [number, number]; // 중심 오프셋
    blindSpots: Region[];         // 지각 불가 영역
    overlap: number;              // 인광 중첩도 (0-1)
  };

  // 전체 보정 파라미터
  globalCalibration: {
    brightnessScaling: number;    // 전체 밝기 스케일
    rotationCorrection: number;   // degrees
    mirrorHorizontal: boolean;
    mirrorVertical: boolean;
  };

  // 품질 메트릭
  quality: {
    totalPhosphenes: number;
    functionalPhosphenes: number;
    averageThreshold: number;     // μA
    dynamicRangeAvg: number;      // dB
    mappingConfidence: number;    // 0-1
  };
}

interface PhospheneDefinition {
  electrodeIndex: number;
  functional: boolean;

  // 지각 특성 (환자 보고)
  perception: {
    location: {
      x: number;                  // degrees from center (수평)
      y: number;                  // degrees from center (수직)
      uncertainty: number;        // degrees (위치 불확실성)
    };
    size: number;                 // degrees (지각 크기)
    shape: 'round' | 'oval' | 'elongated' | 'irregular' | 'streak';
    orientation?: number;         // degrees (elongated인 경우)
    color?: string;               // 지각되는 색상
  };

  // 자극-지각 반응 함수
  response: {
    thresholdCurrent: number;     // μA (최소 지각)
    comfortableCurrent: number;   // μA (편안한 자극)
    maxCurrent: number;           // μA (최대/포화)
    brightnessLevels: number;     // 구분 가능한 밝기 단계
    dynamicRange: number;         // dB
    responseLatency: number;      // ms (자극-지각 지연)
  };

  // 시간 특성
  temporal: {
    persistenceDuration: number;  // ms (인광 지속 시간)
    recoveryTime: number;         // ms (재자극 가능 시간)
    flickerFusionFreq: number;    // Hz (깜빡임 융합 주파수)
  };

  // 이웃 상호작용
  interactions: {
    neighbors: number[];          // 상호작용하는 전극 인덱스
    summationType: 'independent' | 'linear' | 'sublinear' | 'supralinear';
    crowdingEffect: number;       // 0-1 (밀집 효과 정도)
  };
}
```

## 2.4 자극 인코딩 전략

```typescript
interface EncodingStrategy {
  strategyId: string;
  name: string;
  description: string;
  version: string;

  // 공간 인코딩
  spatial: {
    type: SpatialEncodingType;
    downsampleMethod: 'nearest' | 'bilinear' | 'gaussian';
    edgeWeight: number;           // 엣지 강조 가중치 (0-2)
    contrastEnhancement: number;  // 대비 강화 (0-2)
  };

  // 시간 인코딩
  temporal: {
    type: TemporalEncodingType;
    scanPattern?: 'raster' | 'spiral' | 'random' | 'saliency';
    scanRate?: number;            // electrodes per frame
    motionPriority: boolean;      // 움직임 우선
  };

  // 밝기 인코딩
  intensity: {
    method: IntensityEncodingMethod;
    levels: number;               // 구분 단계 수 (4-16)
    mapping: 'linear' | 'logarithmic' | 'gamma';
    gamma?: number;               // gamma 값 (0.5-2.5)
    noiseFloor: number;           // 최소 임계값 (0-255)
  };

  // 적응 알고리즘
  adaptation: {
    autoGain: boolean;            // 자동 밝기 조절
    histogramEqualization: boolean;
    localContrastAdapt: boolean;
    darkAdaptation: boolean;      // 어둠 적응 모드
  };
}

enum SpatialEncodingType {
  DIRECT_MAPPING = 'direct',              // 단순 다운샘플링
  SCOREBOARD = 'scoreboard',              // 각 전극 독립 (Argus II)
  CENTER_SURROUND = 'center_surround',    // 망막 모방
  EDGE_PRIORITY = 'edge_priority',        // 엣지 우선
  SALIENCY_BASED = 'saliency',            // 주의 기반
  SEMANTIC = 'semantic',                  // 객체 기반
}

enum TemporalEncodingType {
  SYNCHRONOUS = 'synchronous',            // 모든 전극 동시
  SEQUENTIAL_ROW = 'sequential_row',      // 행 순차
  SEQUENTIAL_RANDOM = 'sequential_random', // 무작위 순차
  MULTIPLEXED = 'multiplexed',            // 시분할 다중화
  PERSISTENCE = 'persistence',            // 잔상 활용
}

enum IntensityEncodingMethod {
  AMPLITUDE = 'amplitude',                // 전류 크기
  PULSE_WIDTH = 'pulse_width',            // 펄스 폭
  FREQUENCY = 'frequency',                // 자극 주파수
  PULSE_COUNT = 'pulse_count',            // 펄스 수
  DUTY_CYCLE = 'duty_cycle',              // 듀티 사이클
}
```

## 2.5 실시간 제어 인터페이스

```typescript
interface StimulationController {
  // 상태
  state: ControllerState;

  // 현재 설정
  currentStrategy: EncodingStrategy;
  currentParameters: StimulationParameters;

  // 제어 메서드
  methods: {
    // 자극 시작/정지
    start(): void;
    stop(): void;
    pause(): void;
    resume(): void;

    // 프레임 자극
    stimulateFrame(frame: ProcessedVisual): StimulationResult;

    // 파라미터 조정
    setAmplitude(electrodeIndex: number, amplitude: number): void;
    setGlobalBrightness(level: number): void;
    setStrategy(strategy: EncodingStrategy): void;

    // 안전
    emergencyStop(): void;
    checkSafety(): SafetyStatus;
  };
}

enum ControllerState {
  IDLE = 'idle',
  RUNNING = 'running',
  PAUSED = 'paused',
  ERROR = 'error',
  EMERGENCY_STOP = 'emergency_stop',
}

interface StimulationResult {
  frameId: string;
  timestamp: number;
  electrodesStimulated: number[];
  amplitudes: number[];
  totalCharge: number;           // nC
  stimulationTime: number;       // ms
  success: boolean;
  warnings: string[];
}
```

---

## 산출물

```
bionic-eye/
├── spec/
│   ├── STIMULATION-PARAMS-SPEC.md
│   ├── ELECTRODE-ARRAY-SPEC.md
│   ├── PHOSPHENE-MAPPING-SPEC.md
│   └── ENCODING-STRATEGY-SPEC.md
├── api/
│   ├── typescript/
│   │   └── src/
│   │       ├── stimulator/
│   │       │   ├── types.ts
│   │       │   ├── controller.ts
│   │       │   └── index.ts
│   │       └── encoder/
│   │           ├── types.ts
│   │           ├── spatial.ts
│   │           ├── temporal.ts
│   │           └── index.ts
│   └── rust/
│       └── src/
│           ├── stimulator/
│           └── encoder/
└── prompts/
```
