# WIA Pet Emotion API Interface Standard
## Phase 2 사양 (Specification)

---

**버전 (Version)**: 1.0.0
**상태 (Status)**: Draft
**날짜 (Date)**: 2025-12-18
**작성자 (Authors)**: WIA Standards Committee
**라이선스 (License)**: MIT
**주요 색상 (Primary Color)**: #F59E0B (Amber)

---

## 목차 (Table of Contents)

1. [개요](#개요-overview)
2. [용어 정의](#용어-정의-terminology)
3. [핵심 인터페이스](#핵심-인터페이스-core-interfaces)
4. [감정 감지 API](#감정-감지-api-emotion-detection-api)
5. [센서 통합 API](#센서-통합-api-sensor-integration-api)
6. [분석 API](#분석-api-analysis-api)
7. [이벤트 시스템](#이벤트-시스템-event-system)
8. [설정](#설정-configuration)
9. [오류 처리](#오류-처리-error-handling)
10. [사용 예제](#사용-예제-usage-examples)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA Pet Emotion API Interface Standard는 반려동물 감정 상태를 감지, 기록, 분석하기 위한 프로그래밍 인터페이스를 정의합니다. 이 Phase 2 사양은 Phase 1 데이터 형식을 기반으로 실시간 감정 모니터링을 위한 언어 독립적 API를 제공합니다.

**핵심 목표 (Core Goals)**:
- 모든 감정 감지 소스를 위한 통합 API
- 실시간 감정 스트리밍
- 다중 모달 센서 통합
- 이벤트 기반 아키텍처
- TypeScript/Python/REST 구현 제공

### 1.2 적용 범위 (Scope)

| 컴포넌트 (Component) | 설명 (Description) |
|---------------------|-------------------|
| **Core API** | 메인 PetEmotion 클래스 인터페이스 |
| **Event System** | 실시간 감정 이벤트 스트리밍 |
| **Sensor Adapters** | Wearable, 카메라, 마이크 통합 |
| **Analysis Tools** | 시간적, 사회적, 패턴 분석 |
| **ML Integration** | 모델 배포 및 추론 |

### 1.3 Phase 1 호환성 (Phase 1 Compatibility)

Phase 2 API는 Phase 1 데이터 형식을 완전히 구현합니다:

```
Phase 1: Data Format (JSON 구조)
    ↓
Phase 2: API Interface (프로그래밍 인터페이스)
    ↓
Phase 3: Communication Protocol (네트워크 전송)
    ↓
Phase 4: Integration (생태계 연결)
```

---

## 2. 용어 정의 (Terminology)

### 2.1 핵심 용어 (Core Terms)

| 용어 (Term) | 정의 (Definition) |
|-----------|------------------|
| **PetEmotion** | 메인 API 클래스 (진입점) |
| **EmotionDetector** | 입력을 분석하여 감정을 감지하는 컴포넌트 |
| **SensorAdapter** | 물리적/가상 센서에 대한 인터페이스 |
| **EmotionEvent** | 실시간 감정 상태 변경 알림 |
| **EmotionStream** | 연속적인 감정 데이터 흐름 |
| **EmotionAnalyzer** | 패턴 및 추세 분석 컴포넌트 |

### 2.2 이벤트 유형 (Event Types)

| 이벤트 (Event) | 설명 (Description) | 데이터 (Data) |
|---------------|-------------------|--------------|
| `emotion_detected` | 새로운 감정 상태 감지됨 | EmotionDetectionOutput |
| `emotion_changed` | 감정 전환 발생 | EmotionTransition |
| `stress_alert` | 높은 스트레스 수준 감지됨 | StressAlert |
| `social_interaction` | 다중 반려동물 상호작용 이벤트 | SocialInteractionEvent |
| `pattern_detected` | 행동 패턴 인식됨 | BehavioralPattern |
| `sensor_data` | 원시 센서 데이터 수신 | WearableSensorData |
| `error` | 오류 발생 | WiaError |

---

## 3. 핵심 인터페이스 (Core Interfaces)

### 3.1 PetEmotion 클래스

감정 감지 및 모니터링을 위한 메인 API 진입점입니다.

#### TypeScript

```typescript
class PetEmotion {
  // 생성자 (Constructor)
  constructor(options?: PetEmotionOptions);

  // 생명주기 (Lifecycle)
  initialize(): Promise<void>;
  shutdown(): Promise<void>;
  isInitialized(): boolean;

  // 반려동물 관리 (Pet Management)
  registerPet(pet: PetProfile): Promise<string>;
  updatePet(petId: string, updates: Partial<PetProfile>): Promise<void>;
  removePet(petId: string): Promise<void>;
  getPet(petId: string): Promise<PetProfile | null>;
  listPets(): Promise<PetProfile[]>;

  // 감정 감지 (Emotion Detection)
  detectEmotion(petId: string, inputs: EmotionInputs): Promise<EmotionDetectionResult>;
  startContinuousDetection(petId: string, options?: ContinuousOptions): Promise<void>;
  stopContinuousDetection(petId: string): Promise<void>;

  // 상태 조회 (State Query)
  getCurrentEmotion(petId: string): Promise<EmotionState | null>;
  getEmotionHistory(petId: string, options: HistoryOptions): Promise<EmotionTimeline>;
  getEmotionSummary(petId: string, period: TimePeriod): Promise<EmotionSummary>;

  // 이벤트 시스템 (Event System)
  on<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;
  once<T extends EmotionEventType>(event: T, handler: EventHandler<T>): void;

  // 센서 관리 (Sensor Management)
  addSensor(petId: string, sensor: ISensorAdapter): Promise<void>;
  removeSensor(petId: string, sensorId: string): Promise<void>;
  listSensors(petId: string): Promise<SensorInfo[]>;

  // 분석 (Analysis)
  analyzePattern(petId: string, options: AnalysisOptions): Promise<PatternAnalysis>;
  detectAnomaly(petId: string, baseline: EmotionBaseline): Promise<AnomalyReport>;
  comparePets(petIds: string[]): Promise<ComparisonReport>;

  // 설정 (Configuration)
  configure(options: Partial<PetEmotionOptions>): void;
  getConfig(): PetEmotionOptions;
}
```

#### Python

```python
class PetEmotion:
    def __init__(self, options: Optional[PetEmotionOptions] = None):
        ...

    # 생명주기 (Lifecycle)
    async def initialize(self) -> None: ...
    async def shutdown(self) -> None: ...
    def is_initialized(self) -> bool: ...

    # 반려동물 관리 (Pet Management)
    async def register_pet(self, pet: PetProfile) -> str: ...
    async def update_pet(self, pet_id: str, updates: Dict[str, Any]) -> None: ...
    async def remove_pet(self, pet_id: str) -> None: ...
    async def get_pet(self, pet_id: str) -> Optional[PetProfile]: ...
    async def list_pets(self) -> List[PetProfile]: ...

    # 감정 감지 (Emotion Detection)
    async def detect_emotion(self, pet_id: str, inputs: EmotionInputs) -> EmotionDetectionResult: ...
    async def start_continuous_detection(self, pet_id: str, options: Optional[ContinuousOptions] = None) -> None: ...
    async def stop_continuous_detection(self, pet_id: str) -> None: ...

    # 상태 조회 (State Query)
    async def get_current_emotion(self, pet_id: str) -> Optional[EmotionState]: ...
    async def get_emotion_history(self, pet_id: str, options: HistoryOptions) -> EmotionTimeline: ...
    async def get_emotion_summary(self, pet_id: str, period: TimePeriod) -> EmotionSummary: ...

    # 이벤트 시스템 (Event System)
    def on(self, event: EmotionEventType, handler: EventHandler) -> None: ...
    def off(self, event: EmotionEventType, handler: EventHandler) -> None: ...

    # 센서 관리 (Sensor Management)
    async def add_sensor(self, pet_id: str, sensor: ISensorAdapter) -> None: ...
    async def remove_sensor(self, pet_id: str, sensor_id: str) -> None: ...

    # 분석 (Analysis)
    async def analyze_pattern(self, pet_id: str, options: AnalysisOptions) -> PatternAnalysis: ...
    async def detect_anomaly(self, pet_id: str, baseline: EmotionBaseline) -> AnomalyReport: ...
```

### 3.2 PetEmotionOptions

```typescript
interface PetEmotionOptions {
  // 감지 설정 (Detection Settings)
  detection: {
    mode: 'realtime' | 'batch' | 'on_demand';
    frequency: number;           // 연속 감지를 위한 Hz
    confidence_threshold: number; // 0.0-1.0
    multi_modal_fusion: boolean; // 다중 모달 융합
  };

  // 저장소 (Storage)
  storage: {
    enabled: boolean;
    backend: 'memory' | 'sqlite' | 'postgresql' | 'mongodb' | 'custom';
    retention_days: number;      // 보관 일수
    compression: boolean;        // 압축 사용
  };

  // AI/ML 모델 (AI/ML Models)
  models: {
    primary_model: string;       // 주 모델
    fallback_model?: string;     // 대체 모델
    ensemble: boolean;           // 앙상블 사용
    gpu_acceleration: boolean;   // GPU 가속
  };

  // 센서 (Sensors)
  sensors: {
    auto_discover: boolean;      // 자동 탐색
    preferred_types: SensorType[];
    sampling_rate: number;       // Hz
  };

  // 이벤트 (Events)
  events: {
    emit_all: boolean;           // 모든 이벤트 발생
    buffer_size: number;         // 버퍼 크기
    async_handlers: boolean;     // 비동기 핸들러
  };

  // 프라이버시 (Privacy)
  privacy: {
    anonymize: boolean;          // 익명화
    encrypt_storage: boolean;    // 저장소 암호화
    data_sharing: 'none' | 'aggregated' | 'full';
  };

  // 로깅 (Logging)
  logging: {
    level: 'debug' | 'info' | 'warn' | 'error';
    destination: 'console' | 'file' | 'remote';
  };
}
```

---

## 4. 감정 감지 API (Emotion Detection API)

### 4.1 단일 감지 (Single Detection)

단일 입력 세트에서 감정을 감지합니다.

```typescript
interface EmotionInputs {
  // 시각 입력 (Visual input)
  image?: {
    data: Buffer | Blob | string;  // raw, blob, 또는 base64
    format: 'jpeg' | 'png' | 'raw';
    width: number;
    height: number;
    timestamp: Date;
  };

  // 비디오 입력 (Video input)
  video?: {
    frames: ImageFrame[];
    fps: number;
    duration: number;
  };

  // 오디오 입력 (Audio input)
  audio?: {
    data: Buffer | Blob;
    format: 'wav' | 'mp3' | 'raw';
    sample_rate: number;
    channels: number;
    duration: number;
  };

  // 센서 데이터 (Sensor data)
  sensors?: {
    accelerometer?: AccelerometerData;
    heart_rate?: number;
    temperature?: number;
    gps?: GPSData;
  };

  // 수동 관찰 (Manual observation)
  observation?: {
    body_language: Partial<BodyLanguage>;
    behaviors: string[];
    notes: string;
  };

  // 맥락 (Context)
  context?: {
    location: string;
    social_setting: 'alone' | 'with_humans' | 'with_pets';
    activity: string;
  };
}

interface EmotionDetectionResult {
  petId: string;
  timestamp: Date;

  // 주요 결과 (Primary result)
  emotion: CoreEmotion;
  intensity: EmotionIntensity;
  confidence: number;

  // 차원 모델 (Dimensional model)
  dimensions: EmotionDimensions;

  // 모든 예측 (All predictions)
  predictions: {
    emotion: CoreEmotion;
    probability: number;
    confidence: number;
  }[];

  // 기여 요인 (Contributing factors)
  factors: {
    source: 'visual' | 'audio' | 'sensor' | 'observation';
    contribution: number;
    confidence: number;
  }[];

  // 모델 정보 (Model info)
  model: {
    id: string;
    version: string;
    processing_time: number;
  };

  // 품질 지표 (Quality metrics)
  quality: {
    input_quality: number;
    detection_reliability: number;
    data_completeness: number;
  };
}
```

### 4.2 연속 감지 (Continuous Detection)

감정 감지를 연속적으로 스트리밍합니다.

```typescript
interface ContinuousOptions {
  frequency: number;            // Hz (1-60)
  sources: ('camera' | 'microphone' | 'wearable')[];

  // 필터 (Filters)
  min_confidence: number;
  emotion_filter?: CoreEmotion[];

  // 최적화 (Optimization)
  low_power_mode: boolean;      // 저전력 모드
  batch_processing: boolean;    // 일괄 처리

  // 알림 (Alerts)
  alerts: {
    enabled: boolean;
    conditions: AlertCondition[];
  };
}

interface AlertCondition {
  type: 'emotion_change' | 'high_stress' | 'prolonged_state' | 'anomaly';

  // 감정별 설정 (Emotion-specific)
  target_emotion?: CoreEmotion;
  min_intensity?: number;
  min_duration?: number;       // milliseconds

  // 임계값 (Thresholds)
  stress_threshold?: number;
  arousal_threshold?: number;

  // 액션 (Action)
  callback?: (alert: EmotionAlert) => void;
  notification?: boolean;
}
```

---

## 5. 센서 통합 API (Sensor Integration API)

### 5.1 센서 어댑터 인터페이스 (Sensor Adapter Interface)

```typescript
interface ISensorAdapter {
  // 속성 (Properties)
  readonly sensorId: string;
  readonly sensorType: SensorType;
  readonly manufacturer: string;
  readonly model: string;
  readonly isConnected: boolean;

  // 생명주기 (Lifecycle)
  connect(config: SensorConfig): Promise<void>;
  disconnect(): Promise<void>;
  calibrate(): Promise<void>;              // 보정

  // 데이터 수집 (Data acquisition)
  startStreaming(options?: StreamOptions): Promise<void>;
  stopStreaming(): Promise<void>;
  getSample(): Promise<SensorSample>;      // 샘플 가져오기

  // 설정 (Configuration)
  configure(settings: SensorSettings): Promise<void>;
  getCapabilities(): SensorCapabilities;    // 기능 조회

  // 이벤트 (Events)
  on(event: SensorEvent, handler: SensorEventHandler): void;
  off(event: SensorEvent, handler: SensorEventHandler): void;

  // 상태 (Status)
  getStatus(): SensorStatus;
  getBatteryLevel?(): Promise<number>;
}
```

### 5.2 내장 센서 어댑터 (Built-in Sensor Adapters)

#### 카메라 어댑터 (Camera Adapter)

```typescript
class CameraAdapter implements ISensorAdapter {
  sensorType = SensorType.CAMERA;

  // 카메라 전용 메서드 (Camera-specific methods)
  captureFrame(): Promise<ImageFrame>;          // 프레임 캡처
  captureVideo(duration: number): Promise<VideoClip>;
  setResolution(width: number, height: number): void;
  setFrameRate(fps: number): void;
  enableFaceTracking(enabled: boolean): void;   // 얼굴 추적

  // 고급 기능 (Advanced features)
  detectBodyPose(): Promise<BodyPose>;          // 신체 자세 감지
  trackMovement(): AsyncIterator<MovementData>; // 움직임 추적
}
```

#### 마이크 어댑터 (Microphone Adapter)

```typescript
class MicrophoneAdapter implements ISensorAdapter {
  sensorType = SensorType.MICROPHONE;

  // 오디오 전용 메서드 (Audio-specific methods)
  recordAudio(duration: number): Promise<AudioBuffer>;
  detectVocalization(): Promise<VocalizationEvent | null>;
  analyzeAcoustics(audio: AudioBuffer): Promise<AcousticFeatures>;

  // 실시간 분석 (Real-time analysis)
  streamVocalizationDetection(): AsyncIterator<VocalizationEvent>;
  getNoiseLevel(): Promise<number>;             // 소음 수준
}
```

#### 웨어러블 어댑터 (Wearable Adapter)

```typescript
class WearableAdapter implements ISensorAdapter {
  sensorType = SensorType.WEARABLE;

  // 웨어러블 전용 메서드 (Wearable-specific methods)
  getHeartRate(): Promise<number>;              // 심박수
  getActivityLevel(): Promise<number>;          // 활동 수준
  getLocation(): Promise<GPSCoordinates>;       // 위치
  getTemperature(): Promise<number>;            // 체온

  // 연속 모니터링 (Continuous monitoring)
  streamVitalSigns(): AsyncIterator<VitalSigns>;
  streamActivityData(): AsyncIterator<ActivityData>;

  // 장치 관리 (Device management)
  syncData(): Promise<void>;                    // 데이터 동기화
  getBatteryLevel(): Promise<number>;           // 배터리 수준
}
```

---

## 6. 분석 API (Analysis API)

### 6.1 패턴 분석 (Pattern Analysis)

```typescript
interface PatternAnalyzer {
  // 패턴 감지 (Detect patterns)
  detectPatterns(
    petId: string,
    timeRange: TimeRange,
    options?: PatternOptions
  ): Promise<DetectedPattern[]>;

  // 특정 패턴 유형 (Specific pattern types)
  detectDailyRoutine(petId: string): Promise<DailyRoutine>;
  detectEmotionalTriggers(petId: string): Promise<TriggerAnalysis>;
  detectStressPatterns(petId: string): Promise<StressPattern[]>;

  // 이상 탐지 (Anomaly detection)
  detectAnomalies(
    petId: string,
    baseline: EmotionBaseline
  ): Promise<Anomaly[]>;
}

interface DetectedPattern {
  patternId: string;
  type: PatternType;
  confidence: number;

  // 시간적 정보 (Temporal)
  occurrence_times: Date[];
  frequency: string;
  duration: number;

  // 설명 (Description)
  description: string;
  triggers: string[];          // 유발 요인
  associated_emotions: CoreEmotion[];

  // 통계적 (Statistical)
  significance: number;        // 유의성
  correlation: number;         // 상관관계
}

interface DailyRoutine {
  petId: string;
  analyzed_period: TimeRange;

  // 시간 블록 (Time blocks)
  routine_blocks: {
    time_range: string;        // "08:00-10:00"
    typical_emotion: CoreEmotion;
    typical_activity: string;
    confidence: number;
  }[];

  // 변동성 (Variations)
  weekday_weekend_difference: number;
  consistency_score: number;   // 일관성 점수
}
```

### 6.2 시간적 분석 (Temporal Analysis)

```typescript
interface TemporalAnalyzer {
  // 추세 분석 (Trends)
  analyzeTrend(
    petId: string,
    emotion: CoreEmotion,
    period: TimePeriod
  ): Promise<EmotionTrend>;

  // 시계열 (Time series)
  getEmotionTimeSeries(
    petId: string,
    timeRange: TimeRange,
    resolution: number
  ): Promise<EmotionTimeSeries>;

  // 전환 분석 (Transitions)
  analyzeTransitions(
    petId: string,
    timeRange: TimeRange
  ): Promise<TransitionAnalysis>;

  // 예측 (Forecasting)
  forecastEmotion(
    petId: string,
    horizon: number
  ): Promise<EmotionForecast>;
}

interface EmotionTrend {
  emotion: CoreEmotion;
  direction: 'increasing' | 'decreasing' | 'stable';
  rate_of_change: number;    // 변화율
  confidence: number;

  // 데이터 포인트 (Data points)
  data_points: {
    timestamp: Date;
    value: number;
  }[];

  // 통계 (Statistics)
  mean: number;
  variance: number;
  trend_line: {
    slope: number;
    intercept: number;
    r_squared: number;
  };
}
```

---

## 7. 이벤트 시스템 (Event System)

### 7.1 이벤트 유형 (Event Types)

```typescript
// 감정 이벤트 (Emotion events)
interface EmotionDetectedEvent {
  type: 'emotion_detected';
  petId: string;
  timestamp: Date;
  result: EmotionDetectionResult;
}

interface EmotionChangedEvent {
  type: 'emotion_changed';
  petId: string;
  timestamp: Date;
  previous: EmotionState;
  current: EmotionState;
  transition_duration: number;  // 전환 지속시간
}

interface StressAlertEvent {
  type: 'stress_alert';
  petId: string;
  timestamp: Date;
  stress_level: number;
  duration: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  recommended_action: string;   // 권장 조치
}

// 사회적 이벤트 (Social events)
interface SocialInteractionEvent {
  type: 'social_interaction';
  timestamp: Date;
  participants: string[];
  interaction_type: InteractionType;
  quality: InteractionQuality;
  duration: number;
}

// 센서 이벤트 (Sensor events)
interface SensorDataEvent {
  type: 'sensor_data';
  petId: string;
  sensorId: string;
  timestamp: Date;
  data: SensorSample;
}
```

### 7.2 이벤트 핸들러 (Event Handlers)

```typescript
// 이벤트 핸들러 타입
type EventHandler<T extends EmotionEvent> = (event: T) => void | Promise<void>;

// 이벤트 구독 (Subscribe to events)
petEmotion.on('emotion_detected', (event: EmotionDetectedEvent) => {
  console.log(`${event.petId}: ${event.result.emotion}`);
});

petEmotion.on('stress_alert', (event: StressAlertEvent) => {
  if (event.severity === 'critical') {
    notifyOwner(event);  // 보호자에게 알림
  }
});

// 필터링된 구독 (Filtered subscriptions)
petEmotion.on('emotion_changed', (event: EmotionChangedEvent) => {
  // 특정 감정만 처리
  if (event.current.emotion === 'anxious') {
    handleAnxiety(event);
  }
});

// 일회성 핸들러 (One-time handlers)
petEmotion.once('pattern_detected', (event: PatternDetectedEvent) => {
  console.log('첫 번째 패턴 감지:', event.pattern);
});
```

---

## 8. 설정 (Configuration)

### 8.1 전역 설정 (Global Configuration)

```typescript
interface GlobalConfig {
  // API 설정 (API settings)
  api: {
    version: string;
    base_url?: string;
    api_key?: string;
    timeout: number;             // 타임아웃 (milliseconds)
  };

  // 모델 설정 (Model settings)
  models: {
    registry: string;            // 모델 레지스트리 URL
    cache_dir: string;           // 캐시 디렉토리
    auto_update: boolean;        // 자동 업데이트
    preferred_models: Record<PetSpecies, string>;
  };

  // 데이터 관리 (Data management)
  data: {
    storage_path: string;
    max_storage_size: number;    // bytes
    auto_cleanup: boolean;
    backup_enabled: boolean;
  };

  // 성능 (Performance)
  performance: {
    max_concurrent_detections: number;
    gpu_enabled: boolean;
    worker_threads: number;
    batch_size: number;
  };

  // 프라이버시 및 보안 (Privacy and security)
  security: {
    encryption_key?: string;
    ssl_verify: boolean;
    data_anonymization: boolean;
  };
}

// 설정 지정
PetEmotion.setGlobalConfig(config: Partial<GlobalConfig>): void;
PetEmotion.getGlobalConfig(): GlobalConfig;
```

---

## 9. 사용 예제 (Usage Examples)

### 9.1 기본 감정 감지 (Basic Emotion Detection)

```typescript
import { PetEmotion, CoreEmotion } from 'wia-pet-emotion';

// 초기화 (Initialize)
const petEmotion = new PetEmotion({
  detection: {
    mode: 'realtime',
    frequency: 30,
    confidence_threshold: 0.7
  }
});

await petEmotion.initialize();

// 반려동물 등록 (Register a pet)
const petId = await petEmotion.registerPet({
  name: '맥스',
  species: 'dog',
  breed: 'Golden Retriever',
  dateOfBirth: new Date('2020-03-15'),
  sex: 'neutered_male'
});

// 이미지에서 감정 감지 (Detect emotion from image)
const result = await petEmotion.detectEmotion(petId, {
  image: {
    data: imageBuffer,
    format: 'jpeg',
    width: 1920,
    height: 1080,
    timestamp: new Date()
  }
});

console.log(`감지된 감정: ${result.emotion}`);
console.log(`신뢰도: ${result.confidence}`);
console.log(`Valence: ${result.dimensions.valence}`);
console.log(`Arousal: ${result.dimensions.arousal}`);
```

### 9.2 연속 모니터링 (Continuous Monitoring)

```typescript
// 연속 감지 시작 (Start continuous detection)
await petEmotion.startContinuousDetection(petId, {
  frequency: 30,  // 30 Hz
  sources: ['camera', 'wearable'],
  alerts: {
    enabled: true,
    conditions: [
      {
        type: 'high_stress',
        stress_threshold: 0.8,
        min_duration: 60000,  // 1분
        callback: async (alert) => {
          console.log('높은 스트레스 감지!');
          await notifyOwner(alert);
        }
      }
    ]
  }
});

// 감정 변화 감지 (Listen for emotion changes)
petEmotion.on('emotion_changed', (event) => {
  console.log(`감정 변화: ${event.previous.emotion} → ${event.current.emotion}`);

  if (event.current.emotion === 'anxious') {
    console.log('반려동물이 불안해합니다. 환경을 확인하세요');
  }
});

// 스트레스 알림 감지 (Listen for stress alerts)
petEmotion.on('stress_alert', (event) => {
  console.log(`스트레스 알림! 수준: ${event.stress_level}`);
  console.log(`권장 조치: ${event.recommended_action}`);
});

// 완료 후 연속 감지 중지 (Stop continuous detection when done)
await petEmotion.stopContinuousDetection(petId);
```

### 9.3 다중 센서 통합 (Multi-Sensor Integration)

```typescript
import { CameraAdapter, WearableAdapter, SensorFusion } from 'wia-pet-emotion';

// 센서 융합 생성 (Create sensor fusion)
const fusion = new SensorFusion();
fusion.setFusionStrategy(FusionStrategy.HYBRID);

// 카메라 추가 (Add camera)
const camera = new CameraAdapter({
  device: '/dev/video0',
  resolution: [1920, 1080],
  fps: 30
});
await camera.connect();
fusion.addSensor(camera);

// 웨어러블 추가 (Add wearable)
const wearable = new WearableAdapter({
  device: 'FitBark-12345',
  bluetooth_address: '00:11:22:33:44:55'
});
await wearable.connect();
fusion.addSensor(wearable);

// 동기화된 데이터 캡처 (Capture synchronized data)
const multiModalData = await fusion.captureSynchronized();

// 융합을 통한 감정 감지 (Detect emotion with fusion)
const result = await fusion.detectEmotionFused(multiModalData);

console.log(`융합된 감정: ${result.emotion}`);
console.log(`기여 요인:`);
result.factors.forEach(factor => {
  console.log(`  ${factor.source}: ${(factor.contribution * 100).toFixed(1)}%`);
});
```

---

## 10. REST API 엔드포인트 (REST API Endpoints)

### 10.1 반려동물 관리 (Pet Management)

```
POST   /api/v1/pets                    # 반려동물 등록
GET    /api/v1/pets                    # 반려동물 목록 조회
GET    /api/v1/pets/:petId             # 반려동물 상세 조회
PUT    /api/v1/pets/:petId             # 반려동물 정보 수정
DELETE /api/v1/pets/:petId             # 반려동물 삭제
```

### 10.2 감정 감지 (Emotion Detection)

```
POST   /api/v1/pets/:petId/detect              # 감정 감지 요청
POST   /api/v1/pets/:petId/detect/batch        # 일괄 감지
POST   /api/v1/pets/:petId/detect/start        # 연속 감지 시작
POST   /api/v1/pets/:petId/detect/stop         # 연속 감지 중지
GET    /api/v1/pets/:petId/emotion/current     # 현재 감정 조회
GET    /api/v1/pets/:petId/emotion/history     # 감정 이력 조회
GET    /api/v1/pets/:petId/emotion/summary     # 감정 요약 조회
```

### 10.3 분석 (Analysis)

```
GET    /api/v1/pets/:petId/analysis/patterns   # 패턴 분석
GET    /api/v1/pets/:petId/analysis/trends     # 추세 분석
GET    /api/v1/pets/:petId/analysis/anomalies  # 이상 탐지
GET    /api/v1/pets/:petId/analysis/routine    # 일과 분석
POST   /api/v1/analysis/compare                # 반려동물 비교
POST   /api/v1/analysis/social                 # 사회적 분석
```

### 10.4 센서 (Sensors)

```
POST   /api/v1/pets/:petId/sensors                    # 센서 추가
GET    /api/v1/pets/:petId/sensors                    # 센서 목록
DELETE /api/v1/pets/:petId/sensors/:sensorId          # 센서 제거
GET    /api/v1/pets/:petId/sensors/:sensorId/status   # 센서 상태
POST   /api/v1/pets/:petId/sensors/:sensorId/calibrate # 센서 보정
```

---

## 11. 성능 지표 (Performance Metrics)

| 작업 (Operation) | 지연시간 p50 (Latency) | 지연시간 p99 | 처리량 (Throughput) |
|-----------------|----------------------|------------|-------------------|
| 단일 감지 | < 100ms | < 300ms | 100 req/s |
| 연속 감지 (30Hz) | < 33ms | < 50ms | - |
| 일괄 처리 (100개 이미지) | < 5s | < 10s | - |
| 패턴 분석 | < 2s | < 5s | 10 req/s |
| 이벤트 디스패치 | < 5ms | < 20ms | 10,000 events/s |

---

**문서 ID (Document ID)**: WIA-PET-EMOTION-PHASE2-001
**버전 (Version)**: 1.0.0
**최종 업데이트 (Last Updated)**: 2025-12-18
**저작권 (Copyright)**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
