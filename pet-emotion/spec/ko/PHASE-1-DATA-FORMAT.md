# Phase 1: Pet Emotion Data Format Specification
# 반려동물 감정 데이터 형식 표준

**버전 (Version)**: 1.0.0
**날짜 (Date)**: 2025-12-18
**상태 (Status)**: Draft
**표준 ID (Standard ID)**: WIA-PET-EMOTION-PHASE1-001
**주요 색상 (Primary Color)**: #F59E0B (Amber)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA-PET-EMOTION은 반려동물의 감정 상태를 감지, 기록, 분석하기 위한 포괄적인 표준입니다. 이 표준은 수의사, 반려동물 보호자, 연구자, AI 시스템이 다중 모달 감지 및 분석을 통해 반려동물의 감정을 이해하고 대응할 수 있도록 합니다.

**핵심 목표 (Core Objectives)**:
- 종(species) 전반의 감정 분류 표준화
- Wearable을 통한 실시간 감정 모니터링
- AI/ML 감정 감지 모델 지원
- 시간적 감정 패턴 분석 제공
- 다중 반려동물 상호작용 감정 추적 지원
- 수의학 및 반려동물 돌봄 시스템 간 상호운용성 보장

### 1.2 적용 범위 (Scope)

이 표준이 다루는 영역:

| Domain | 설명 (Description) |
|--------|-------------------|
| **Emotion Taxonomy** | 개, 고양이 및 기타 반려동물의 표준화된 감정 카테고리 |
| **Behavioral Indicators** | Body language, 자세, 움직임 패턴 |
| **Vocalization Analysis** | 소리 패턴 분류 및 음향 특징 |
| **Physiological Markers** | 심박수, cortisol, 체온, biomarker |
| **AI/ML Integration** | 모델 출력 및 신뢰도 점수 |
| **Temporal Patterns** | 시계열 감정 추적 및 추세 |
| **Multi-Pet Dynamics** | 사회적 상호작용 감정 감지 |

### 1.3 철학 (Philosophy)

**弘益人間 (홍익인간)** - 인류와 모든 생명체를 이롭게 하라

반려동물의 감정을 이해하는 것은 복지를 개선하고, 인간-동물 유대를 강화하며, 행동 문제를 예방합니다.

---

## 2. 감정 분류 체계 (Emotion Taxonomy)

### 2.1 핵심 감정 카테고리 (Core Emotion Categories)

```typescript
enum CoreEmotion {
  // 긍정적 감정 (Positive Emotions)
  HAPPY = 'happy',              // 기쁨
  CONTENT = 'content',          // 만족
  PLAYFUL = 'playful',          // 장난스러움
  EXCITED = 'excited',          // 흥분
  CURIOUS = 'curious',          // 호기심
  AFFECTIONATE = 'affectionate', // 애정

  // 부정적 감정 (Negative Emotions)
  FEARFUL = 'fearful',          // 두려움
  ANXIOUS = 'anxious',          // 불안
  STRESSED = 'stressed',        // 스트레스
  ANGRY = 'angry',              // 분노
  FRUSTRATED = 'frustrated',    // 좌절
  SAD = 'sad',                  // 슬픔
  LONELY = 'lonely',            // 외로움

  // 중립/기타 (Neutral/Other)
  CALM = 'calm',                // 차분함
  ALERT = 'alert',              // 경계
  TIRED = 'tired',              // 피곤함
  PAIN = 'pain',                // 통증
  UNKNOWN = 'unknown'           // 알 수 없음
}
```

### 2.2 감정 강도 척도 (Emotion Intensity Scale)

```typescript
interface EmotionIntensity {
  // 0.0에서 1.0까지의 척도
  intensity: number;

  // 단계별 수준
  level: 'minimal' | 'low' | 'moderate' | 'high' | 'extreme';

  // 평가 신뢰도 (0.0-1.0)
  confidence: number;
}
```

### 2.3 감정 차원: Valence와 Arousal

```typescript
interface EmotionDimensions {
  // Valence: -1.0 (부정적) ~ +1.0 (긍정적)
  valence: number;

  // Arousal: 0.0 (차분함) ~ 1.0 (고도 각성)
  arousal: number;

  // Dominance: 0.0 (복종적) ~ 1.0 (지배적)
  dominance: number;

  // Circumplex 모델 좌표
  circumplex: {
    x: number;  // valence 축
    y: number;  // arousal 축
  };
}
```

### 2.4 종별 감정 매핑 (Species-Specific Emotion Mapping)

```typescript
interface SpeciesEmotionProfile {
  species: PetSpecies;

  // 종별 감정 표현
  emotionExpressions: {
    [key in CoreEmotion]?: {
      typical: boolean;          // 이 종에서 전형적인가
      indicators: string[];       // 행동 지표들
      reliability: number;        // 신뢰도
    };
  };

  // 종별 행동 마커
  behavioralMarkers: BehavioralMarker[];
}

enum PetSpecies {
  DOG = 'dog',        // 개
  CAT = 'cat',        // 고양이
  RABBIT = 'rabbit',  // 토끼
  BIRD = 'bird',      // 새
  REPTILE = 'reptile', // 파충류
  RODENT = 'rodent',  // 설치류
  OTHER = 'other'     // 기타
}
```

---

## 3. 행동 지표 형식 (Behavioral Indicators Format)

### 3.1 Body Language 신호

```typescript
interface BodyLanguage {
  timestamp: ISO8601;

  // 꼬리 위치 및 움직임
  tail: {
    position: 'up' | 'down' | 'neutral' | 'tucked' | 'curved';
    movement: 'wagging' | 'still' | 'twitching' | 'thrashing';
    speed: number;           // 초당 움직임 횟수
    amplitude: number;       // 0.0-1.0 (미세함-격렬함)
  };

  // 귀 위치
  ears: {
    position: 'forward' | 'neutral' | 'back' | 'flat' | 'rotating';
    symmetry: 'symmetrical' | 'asymmetrical';
    angle: number;           // 중립에서의 각도 (degrees)
  };

  // 몸 자세
  posture: {
    stance: 'relaxed' | 'tense' | 'crouched' | 'play_bow' | 'defensive';
    height: 'raised' | 'normal' | 'lowered';
    weight_distribution: 'forward' | 'centered' | 'backward';
    muscle_tension: number;  // 0.0-1.0 (근육 긴장도)
  };

  // 머리 위치
  head: {
    tilt: number;            // 기울기 (degrees)
    direction: 'forward' | 'down' | 'up' | 'averted';
    stability: 'stable' | 'scanning' | 'bobbing';
  };

  // 얼굴 표정 (개/고양이)
  face: {
    eyes: {
      size: 'normal' | 'dilated' | 'constricted';  // 동공 크기
      gaze: 'direct' | 'averted' | 'soft' | 'hard'; // 시선
      blinking_rate: number; // 분당 깜빡임 횟수
      whale_eye: boolean;    // 흰자위 노출
    };
    mouth: {
      state: 'closed' | 'open' | 'panting' | 'yawning';
      lip_position: 'relaxed' | 'pulled_back' | 'tense';
      teeth_visible: boolean; // 이빨 노출
      tongue_out: boolean;    // 혀 내밀기
    };
    whiskers: {
      position: 'forward' | 'neutral' | 'back';  // 수염 위치
      spread: number;        // 벌림 정도 (degrees)
    };
  };

  // 전체 몸 움직임
  movement: {
    activity_level: number;  // 0.0-1.0 (활동 수준)
    coordination: 'smooth' | 'jerky' | 'trembling';
    direction: 'approaching' | 'retreating' | 'circling' | 'stationary';
    speed: number;           // m/s
  };
}
```

### 3.2 행동 패턴 인식 (Behavioral Pattern Recognition)

```typescript
interface BehavioralPattern {
  patternId: string;
  type: BehavioralPatternType;

  // 행동 시퀀스
  sequence: {
    behavior: string;        // 행동명
    duration: number;        // 지속 시간 (milliseconds)
    timestamp: ISO8601;
  }[];

  // 패턴 특성
  frequency: number;         // 시간당 발생 횟수
  duration: number;          // 전체 패턴 지속 시간 (ms)

  // 연관된 감정
  associatedEmotion: CoreEmotion;
  confidence: number;

  // 맥락
  context: {
    location: string;        // 위치
    socialContext: 'alone' | 'with_humans' | 'with_pets' | 'mixed';
    timeOfDay: string;       // 시간대
  };
}

enum BehavioralPatternType {
  // 긍정적 패턴
  PLAY_SOLICITATION = 'play_solicitation',    // 놀이 요청
  AFFECTION_SEEKING = 'affection_seeking',    // 애정 추구
  EXPLORATION = 'exploration',                // 탐색

  // 부정적 패턴
  DISPLACEMENT_BEHAVIOR = 'displacement_behavior', // 전위 행동
  AVOIDANCE = 'avoidance',                    // 회피
  AGGRESSION = 'aggression',                  // 공격성
  STEREOTYPY = 'stereotypy',                  // 상동 행동

  // 중립적
  RESTING = 'resting',        // 휴식
  GROOMING = 'grooming',      // 그루밍
  FEEDING = 'feeding'         // 식사
}
```

---

## 4. 음성 분석 형식 (Vocalization Analysis Format)

### 4.1 소리 분류 (Sound Classification)

```typescript
interface VocalizationEvent {
  eventId: string;
  timestamp: ISO8601;

  // 소리 유형
  type: VocalizationType;
  subtype?: string;

  // 음향 특징
  acoustics: {
    // 주파수 (Frequency)
    fundamental_frequency: number;  // Hz (기본 주파수)
    frequency_range: {
      min: number;
      max: number;
    };

    // 시간적 특성 (Temporal)
    duration: number;               // 지속시간 (milliseconds)
    syllable_count: number;         // 음절 수
    syllable_rate: number;          // 초당 음절 수

    // 진폭 (Amplitude)
    amplitude: number;              // dB (데시벨)
    amplitude_variation: number;    // 0.0-1.0 (진폭 변화)

    // 패턴 (Pattern)
    pattern: 'continuous' | 'pulsed' | 'ascending' | 'descending' | 'complex';
    rhythm: {
      regular: boolean;
      tempo: number;                // 분당 박자
    };

    // 음질 (Quality)
    harshness: number;              // 0.0-1.0 (거칠기)
    breathiness: number;            // 0.0-1.0 (숨소리)
    tonality: number;               // 0.0-1.0 (음조성 vs 잡음성)
  };

  // 맥락 (Context)
  context: {
    trigger: string;                // 유발 요인
    directed_at: 'human' | 'pet' | 'environment' | 'self';
    social_context: string;         // 사회적 맥락
  };

  // 감정 상관관계
  emotionIndicators: {
    emotion: CoreEmotion;
    confidence: number;
  }[];
}

enum VocalizationType {
  // 개 음성화 (Dog vocalizations)
  DOG_BARK = 'dog_bark',      // 짖기
  DOG_GROWL = 'dog_growl',    // 으르렁거림
  DOG_WHINE = 'dog_whine',    // 낑낑거림
  DOG_HOWL = 'dog_howl',      // 울부짖음
  DOG_YELP = 'dog_yelp',      // 비명
  DOG_PANTING = 'dog_panting', // 헐떡임
  DOG_SIGH = 'dog_sigh',      // 한숨

  // 고양이 음성화 (Cat vocalizations)
  CAT_MEOW = 'cat_meow',      // 야옹
  CAT_PURR = 'cat_purr',      // 가르랑거림
  CAT_HISS = 'cat_hiss',      // 쉿 소리
  CAT_GROWL = 'cat_growl',    // 으르렁거림
  CAT_CHIRP = 'cat_chirp',    // 짹짹거림
  CAT_TRILL = 'cat_trill',    // 트릴 소리
  CAT_YOWL = 'cat_yowl',      // 울부짖음
  CAT_CHATTER = 'cat_chatter', // 재잘거림

  // 기타
  OTHER = 'other',
  UNKNOWN = 'unknown'
}
```

### 4.2 짖음 패턴 분석 (Bark Pattern Analysis - Dogs)

```typescript
interface BarkPattern {
  patternId: string;
  timestamp: ISO8601;

  // 짖음 특성
  bark_count: number;         // 짖음 횟수
  bark_sequence: {
    index: number;
    duration: number;         // ms
    interval_after: number;   // 다음 짖음까지의 간격 (ms)
    pitch: number;            // Hz
    intensity: number;        // 0.0-1.0
  }[];

  // 패턴 분류
  pattern_type: 'alert' | 'alarm' | 'attention' | 'play' | 'frustration' | 'threat';

  // 감정적 내용
  urgency: number;            // 0.0-1.0 (긴급도)
  stress_level: number;       // 0.0-1.0 (스트레스 수준)

  // 해석
  likely_meaning: string;     // 의미 추정
  confidence: number;
}
```

### 4.3 가르랑거림 분석 (Purr Analysis - Cats)

```typescript
interface PurrPattern {
  patternId: string;
  timestamp: ISO8601;

  // 가르랑거림 특성
  frequency: number;          // Hz (일반적으로 25-150)
  amplitude: number;          // dB
  duration: number;           // milliseconds

  // 패턴
  continuous: boolean;        // 연속적인가
  interruptions: number;      // 중단 횟수

  // 유형 분류
  purr_type: 'contentment' | 'solicitation' | 'stress' | 'pain';

  // 감정 해석
  valence: number;            // -1.0 ~ 1.0
  confidence: number;
}
```

---

## 5. 생리학적 마커 형식 (Physiological Markers Format)

### 5.1 생체 징후 (Vital Signs)

```typescript
interface VitalSigns {
  timestamp: ISO8601;

  // 심박수
  heart_rate: {
    bpm: number;              // 분당 심박수
    variability: number;      // SDNN (milliseconds)
    rhythm: 'regular' | 'irregular';

    // HRV 지표
    hrv_metrics: {
      rmssd: number;          // 연속 RR 간격의 제곱근 평균
      pnn50: number;          // 50ms 초과 연속 RR 간격 비율
      lf_hf_ratio: number;    // 저주파/고주파 파워 비율
    };
  };

  // 호흡
  respiration: {
    rate: number;             // 분당 호흡 수
    pattern: 'normal' | 'rapid' | 'shallow' | 'labored';
    regularity: number;       // 0.0-1.0 (규칙성)
  };

  // 체온
  temperature: {
    value: number;            // 섭씨
    location: 'rectal' | 'ear' | 'axillary' | 'surface';
  };

  // 혈압 (가능한 경우)
  blood_pressure?: {
    systolic: number;         // mmHg (수축기)
    diastolic: number;        // mmHg (이완기)
  };

  // 감정 상관관계
  stress_index: number;       // 0.0-1.0 (생체신호 기반 스트레스 지수)
}
```

### 5.2 Biomarker 분석

```typescript
interface BiomarkerProfile {
  profileId: string;
  timestamp: ISO8601;

  // 샘플 정보
  sample: {
    type: 'blood' | 'saliva' | 'urine' | 'feces';
    collectedAt: ISO8601;     // 채취 시각
    analyzedAt: ISO8601;      // 분석 시각
  };

  // 스트레스 호르몬
  cortisol: {
    value: number;            // nmol/L
    baseline: number;         // 기준치
    percentChange: number;    // 변화율
    interpretation: 'low' | 'normal' | 'elevated' | 'high';
  };

  // 기타 마커들
  markers: {
    name: string;
    value: number;
    unit: string;
    referenceRange: {
      min: number;
      max: number;
    };
    status: 'low' | 'normal' | 'high';
  }[];

  // 감정 해석
  stress_level: number;       // 0.0-1.0
  anxiety_indicators: boolean;
}
```

### 5.3 Wearable 센서 데이터

```typescript
interface WearableSensorData {
  deviceId: string;
  deviceInfo: {
    manufacturer: string;     // 제조사
    model: string;            // 모델
    firmwareVersion: string;  // 펌웨어 버전
    sensorTypes: SensorType[];
  };

  timestamp: ISO8601;

  // 가속도계 (Accelerometer)
  accelerometer?: {
    x: number;                // m/s²
    y: number;
    z: number;
    magnitude: number;        // 크기
    activity_count: number;   // 활동 카운트
  };

  // 자이로스코프 (Gyroscope)
  gyroscope?: {
    pitch: number;            // degrees
    roll: number;
    yaw: number;
  };

  // 위치 (Location)
  gps?: {
    latitude: number;         // 위도
    longitude: number;        // 경도
    accuracy: number;         // 정확도 (meters)
    speed: number;            // 속도 (m/s)
  };

  // 환경 센서
  environment?: {
    temperature: number;      // 섭씨
    humidity: number;         // 습도 (percentage)
    light_level: number;      // 조도 (lux)
    noise_level: number;      // 소음 (dB)
  };

  // 유도된 지표
  activity_level: number;     // 0.0-1.0 (활동 수준)
  rest_quality: number;       // 0.0-1.0 (휴식 품질)
  stress_indicators: boolean; // 스트레스 지표
}
```

---

## 6. AI/ML 모델 출력 형식 (AI/ML Model Output Format)

### 6.1 감정 감지 모델 출력

```typescript
interface EmotionDetectionOutput {
  modelId: string;
  modelVersion: string;
  timestamp: ISO8601;
  processingTime: number;     // 처리 시간 (milliseconds)

  // 모델 정보
  model: {
    name: string;
    type: 'cnn' | 'lstm' | 'transformer' | 'ensemble' | 'rule_based';
    architecture: string;     // 아키텍처
    trainedOn: {
      species: PetSpecies[];
      datasetSize: number;    // 데이터셋 크기
      accuracy: number;       // 정확도
    };
  };

  // 입력 모달리티
  inputs: {
    video?: boolean;
    audio?: boolean;
    sensors?: boolean;
    text?: boolean;
  };

  // 감정 예측
  predictions: {
    emotion: CoreEmotion;
    probability: number;      // 확률
    confidence: number;       // 신뢰도
  }[];

  // 최우선 예측
  primary_emotion: CoreEmotion;
  primary_confidence: number;

  // 차원 모델 출력
  dimensions: EmotionDimensions;

  // Attention map (주목도 맵)
  attention_map?: {
    regions: {
      x: number;
      y: number;
      width: number;
      height: number;
      importance: number;     // 중요도
    }[];
  };

  // 불확실성
  uncertainty: {
    aleatoric: number;        // 데이터 불확실성
    epistemic: number;        // 모델 불확실성
    total: number;            // 전체 불확실성
  };
}
```

### 6.2 다중 모달 융합 (Multi-Modal Fusion)

```typescript
interface MultiModalFusion {
  fusionId: string;
  timestamp: ISO8601;

  // 개별 모달리티 결과
  modalities: {
    video: EmotionDetectionOutput;
    audio: EmotionDetectionOutput;
    sensors: EmotionDetectionOutput;
  };

  // 융합 전략
  fusion_method: 'early' | 'late' | 'hybrid' | 'attention';

  // 융합 결과
  fused_emotion: CoreEmotion;
  fused_confidence: number;
  fused_dimensions: EmotionDimensions;

  // 모달리티 가중치
  weights: {
    video: number;
    audio: number;
    sensors: number;
  };

  // 합의 지표
  inter_modality_agreement: number;  // 0.0-1.0 (모달리티 간 합의도)
  conflicts: {
    modality1: string;
    modality2: string;
    disagreement: number;   // 불일치 정도
  }[];
}
```

---

## 7. 시간적 패턴 형식 (Temporal Pattern Format)

### 7.1 감정 타임라인 (Emotion Timeline)

```typescript
interface EmotionTimeline {
  timelineId: string;
  petId: string;

  // 시간 범위
  startTime: ISO8601;
  endTime: ISO8601;
  duration: number;           // milliseconds

  // 감정 상태들
  states: {
    emotion: CoreEmotion;
    startTime: ISO8601;
    endTime: ISO8601;
    duration: number;
    intensity: EmotionIntensity;
    triggers: string[];       // 유발 요인
    confidence: number;
  }[];

  // 통계
  statistics: {
    dominant_emotion: CoreEmotion;  // 주요 감정
    emotion_distribution: {
      emotion: CoreEmotion;
      percentage: number;
      count: number;
    }[];

    transition_count: number;       // 전환 횟수
    average_state_duration: number; // 평균 상태 지속시간
    volatility: number;             // 0.0-1.0 (변동성)
  };
}
```

### 7.2 일일 감정 요약 (Daily Emotion Summary)

```typescript
interface DailyEmotionSummary {
  date: string;               // YYYY-MM-DD
  petId: string;

  // 전체 지표
  overall: {
    dominant_emotion: CoreEmotion;  // 주요 감정
    average_valence: number;        // 평균 valence (-1.0 ~ 1.0)
    average_arousal: number;        // 평균 arousal (0.0 ~ 1.0)
    emotional_stability: number;    // 감정 안정성 (0.0-1.0)
  };

  // 시간대별 패턴
  periods: {
    period: 'morning' | 'afternoon' | 'evening' | 'night';
    dominant_emotion: CoreEmotion;
    activity_level: number;
    stress_level: number;
  }[];

  // 주목할 만한 이벤트
  events: {
    timestamp: ISO8601;
    event_type: string;
    emotion: CoreEmotion;
    intensity: number;
    description: string;
  }[];

  // 추세
  trends: {
    compared_to_baseline: number;  // 기준선 대비 변화율
    stress_trend: 'improving' | 'stable' | 'worsening';
    activity_trend: 'increasing' | 'stable' | 'decreasing';
  };
}
```

---

## 8. 다중 반려동물 상호작용 형식 (Multi-Pet Interaction Format)

### 8.1 사회적 상호작용 이벤트

```typescript
interface SocialInteractionEvent {
  eventId: string;
  timestamp: ISO8601;
  duration: number;

  // 참가자들
  participants: {
    petId: string;
    species: PetSpecies;
    role: 'initiator' | 'responder' | 'observer';

    // 개별 감정
    emotion: CoreEmotion;
    intensity: number;
    arousal: number;
  }[];

  // 상호작용 유형
  interaction_type: InteractionType;

  // 상호작용 품질
  quality: {
    valence: 'positive' | 'neutral' | 'negative' | 'mixed';
    intensity: number;        // 0.0-1.0
    reciprocity: number;      // 0.0-1.0 (상호성)
    synchrony: number;        // 0.0-1.0 (동시성)
  };

  // 결과
  outcome: 'positive' | 'neutral' | 'negative' | 'escalated' | 'de-escalated';
}

enum InteractionType {
  PLAY = 'play',              // 놀이
  GROOMING = 'grooming',      // 그루밍
  AGGRESSION = 'aggression',  // 공격
  AVOIDANCE = 'avoidance',    // 회피
  AFFILIATION = 'affiliation', // 친밀
  COMPETITION = 'competition', // 경쟁
  COOPERATION = 'cooperation', // 협력
  MATING = 'mating'           // 짝짓기
}
```

---

## 9. 완전한 감정 기록 형식 (Complete Emotion Record Format)

### 9.1 반려동물 감정 기록

이것은 Phase 1 데이터 형식의 최상위 구조로, 모든 감정 관련 데이터를 포함합니다.

```typescript
interface PetEmotionRecord {
  // 헤더
  header: {
    magic: 'WIAPETE';
    version: [1, 0, 0];
    standard: 'WIA-PET-EMOTION';
    recordId: string;         // UUID v7
    createdAt: ISO8601;
    lastUpdated: ISO8601;
  };

  // 반려동물 식별
  pet: {
    petId: string;
    passportId?: string;      // WIA-PET-HEALTH-PASSPORT 링크
    species: PetSpecies;
    breed?: string;           // 품종
    name: string;             // 이름
    dateOfBirth: ISO8601;     // 생년월일
  };

  // 현재 감정 상태
  currentState: {
    timestamp: ISO8601;
    primary_emotion: CoreEmotion;
    intensity: EmotionIntensity;
    dimensions: EmotionDimensions;
    confidence: number;
  };

  // 데이터 소스
  dataSources: {
    behavioral: BodyLanguage;           // 행동 데이터
    vocalization?: VocalizationEvent;   // 음성 데이터
    physiological?: VitalSigns;         // 생리 데이터
    wearable?: WearableSensorData;      // 웨어러블 데이터
    biomarkers?: BiomarkerProfile;      // 바이오마커
    ai_model?: EmotionDetectionOutput;  // AI 모델 출력
  };

  // 시간적 데이터
  timeline: EmotionTimeline;
  dailySummary?: DailyEmotionSummary;

  // 사회적 맥락
  socialContext?: {
    alone: boolean;           // 혼자인가
    with_humans: boolean;     // 사람과 함께인가
    with_pets: boolean;       // 다른 반려동물과 함께인가
    interaction?: SocialInteractionEvent;
  };

  // 환경 맥락
  environment: {
    location: string;         // 위치
    temperature?: number;     // 온도
    noise_level?: number;     // 소음 수준
    light_level?: number;     // 조도
    weather?: string;         // 날씨
  };

  // 메타데이터
  metadata: {
    source: 'manual' | 'automated' | 'hybrid';
    observer?: {
      type: 'owner' | 'veterinarian' | 'trainer' | 'researcher' | 'ai';
      id: string;
      qualifications?: string[];
    };
    quality_score: number;    // 0.0-1.0 (품질 점수)
    data_completeness: number; // 0.0-1.0 (데이터 완전성)
  };
}
```

---

## 10. 성능 요구사항 (Performance Requirements)

| 지표 (Metric) | 요구사항 (Requirement) | 목표 (Target) |
|--------------|----------------------|--------------|
| 감정 감지 지연시간 | < 500ms | < 200ms |
| 연속 모니터링 | 1-60 Hz | 30 Hz |
| 모델 정확도 | > 85% | > 92% |
| False Positive 비율 | < 15% | < 8% |
| 배터리 수명 (wearable) | > 24시간 | > 72시간 |
| 일일 데이터 저장량 | < 100MB | < 50MB |

---

## 11. 호환성 (Compatibility)

### 11.1 다른 WIA 표준과의 통합

| 표준 (Standard) | 통합 지점 (Integration Point) |
|----------------|------------------------------|
| WIA-PET-HEALTH-PASSPORT | petId/passportId를 통한 링크 |
| WIA-PET-CARE-ROBOT | 감정 인식 기반 로봇 행동 |
| WIA-PET-GENOME | 불안 유전적 소인 분석 |
| WIA-AI | 모델 배포 및 버전 관리 |

---

**문서 ID (Document ID)**: WIA-PET-EMOTION-PHASE1-001
**버전 (Version)**: 1.0.0
**최종 업데이트 (Last Updated)**: 2025-12-18
**저작권 (Copyright)**: © 2025 WIA - MIT License

---
**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
