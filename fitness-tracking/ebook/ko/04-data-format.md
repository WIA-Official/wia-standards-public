# 4장: 데이터 형식 및 스키마

## 개요

이 장은 WIA-IND-012의 모든 피트니스 트래킹 데이터 유형에 대한 표준화된 JSON 스키마 및 TypeScript 인터페이스를 정의합니다.

---

## 4.1 활동 데이터 형식

### 4.1.1 기본 활동 스키마

```typescript
interface Activity {
  // 식별
  id: string;                    // UUID
  userId: string;                // 사용자 식별자
  type: ActivityType;            // 활동 카테고리

  // 타이밍
  startTime: Date;               // ISO 8601 타임스탬프
  endTime: Date;                 // ISO 8601 타임스탬프
  duration: number;              // 총 초

  // 거리
  distance?: number;             // 미터
  steps?: number;                // 걸음 수

  // 기본 지표
  calories: number;              // 소모 칼로리
  avgPace?: number;              // min/km
  avgSpeed?: number;             // km/h

  // 메타데이터
  source: DataSource;            // 기록한 기기/앱
  manual: boolean;               // 사용자 입력 vs. 자동 추적
  notes?: string;                // 사용자 노트
}
```

**예시 JSON:**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "userId": "user_12345",
  "type": "running",
  "startTime": "2025-12-27T07:00:00Z",
  "endTime": "2025-12-27T07:32:15Z",
  "duration": 1935,
  "distance": 8000,
  "steps": 9840,
  "calories": 680,
  "avgPace": 4.03,
  "avgSpeed": 14.88,
  "manual": false
}
```

### 4.1.2 활동 유형 열거형

```typescript
enum ActivityType {
  // 유산소
  RUNNING = 'running',
  WALKING = 'walking',
  CYCLING = 'cycling',
  SWIMMING = 'swimming',
  HIKING = 'hiking',

  // 근력
  WEIGHT_TRAINING = 'weight_training',
  BODYWEIGHT = 'bodyweight',
  CROSSFIT = 'crossfit',

  // 스포츠
  BASKETBALL = 'basketball',
  SOCCER = 'soccer',
  TENNIS = 'tennis',

  // 마음-몸
  YOGA = 'yoga',
  PILATES = 'pilates',

  // 기타
  HIIT = 'hiit',
  OTHER = 'other'
}
```

### 4.1.3 GPS 데이터 형식

```typescript
interface GPSPoint {
  latitude: number;              // 십진 도 (-90 ~ 90)
  longitude: number;             // 십진 도 (-180 ~ 180)
  elevation?: number;            // 해발 미터
  accuracy?: number;             // 수평 정확도 (미터)
  timestamp: number;             // Unix 타임스탬프 (ms)
  speed?: number;                // m/s
  heading?: number;              // 도 (0-360, 0=북쪽)
}
```

---

## 4.2 운동 데이터 형식

### 4.2.1 포괄적인 운동 스키마

```typescript
interface Workout {
  // 식별
  id: string;
  userId: string;
  type: WorkoutType;
  name?: string;                 // 맞춤 운동 이름

  // 타이밍
  startTime: Date;
  endTime: Date;
  duration: number;              // 총 초

  // 활동 지표
  distance?: number;             // 미터
  steps?: number;
  elevation?: {
    gain: number;                // 미터
    loss: number;                // 미터
  };

  // 심혈관 지표
  heartRate?: {
    avg: number;                 // BPM
    max: number;                 // BPM
    min: number;                 // BPM
    zones: HeartRateZones;
  };

  // 에너지 소비
  calories: number;

  // 성능 지표
  pace?: {
    avg: number;                 // min/km
    best: number;                // 최고 페이스
  };

  // GPS 데이터
  route?: GPSRoute;

  // 구조화된 인터벌
  intervals?: Interval[];

  // 사용자 피드백
  perceivedExertion?: number;    // RPE 1-10 척도
  notes?: string;

  // 트레이닝 지표
  trainingLoad?: number;         // TRIMP 또는 TSS
  tss?: number;                  // 트레이닝 스트레스 점수
}
```

### 4.2.2 심박수 구역 분포

```typescript
interface HeartRateZones {
  zone1: number;                 // 구역 1의 분 (50-60%)
  zone2: number;                 // 구역 2의 분 (60-70%)
  zone3: number;                 // 구역 3의 분 (70-80%)
  zone4: number;                 // 구역 4의 분 (80-90%)
  zone5: number;                 // 구역 5의 분 (90-100%)
}
```

### 4.2.3 인터벌 데이터 형식

```typescript
interface Interval {
  number: number;                // 인터벌 순서
  type: 'work' | 'rest' | 'warmup' | 'cooldown';
  duration: number;              // 초
  distance?: number;             // 미터

  // 목표 (계획된)
  targetPace?: number;           // min/km
  targetHeartRate?: number;      // BPM

  // 실제 (달성된)
  avgPace?: number;
  avgHeartRate?: number;
  calories?: number;

  // 성능
  completed: boolean;
}
```

### 4.2.4 근력 트레이닝 형식

```typescript
interface StrengthWorkout extends Workout {
  exercises: Exercise[];
  totalVolume: number;           // kg × 반복 총합
  totalSets: number;
  totalReps: number;
}

interface Exercise {
  name: string;                  // "벤치 프레스", "스쿼트" 등
  category: ExerciseCategory;
  equipment: string;             // "바벨", "덤벨" 등
  muscleGroups: MuscleGroup[];
  sets: Set[];
}

interface Set {
  setNumber: number;
  reps: number;
  weight?: number;               // kg 또는 lbs
  duration?: number;             // 초 (아이소메트릭 홀드용)
  restTime?: number;             // 다음 세트 전 초
  completed: boolean;
}
```

---

## 4.3 심박수 데이터 형식

### 4.3.1 실시간 심박수

```typescript
interface HeartRateReading {
  timestamp: Date;
  bpm: number;                   // 분당 박동수
  confidence?: number;           // 0-100% (센서 신뢰도)
  source: 'chest_strap' | 'wrist' | 'optical' | 'ecg';
}

interface HeartRateStream {
  userId: string;
  readings: HeartRateReading[];
  avgBpm: number;
  maxBpm: number;
  minBpm: number;
  startTime: Date;
  endTime: Date;
}
```

### 4.3.2 심박 변이도 (HRV)

```typescript
interface HRVMeasurement {
  timestamp: Date;
  userId: string;

  // 시간 영역 지표
  rmssd: number;                 // 연속 차이의 제곱평균제곱근 (ms)
  sdnn: number;                  // NN 간격의 표준편차 (ms)
  pnn50: number;                 // 연속 RR 간격 > 50ms의 %

  // 원시 데이터
  rrIntervals?: number[];        // ms 단위 R-R 간격

  // 컨텍스트
  measurementType: 'morning' | 'resting' | 'post_exercise' | 'sleeping';
  quality: 'excellent' | 'good' | 'fair' | 'poor';
}
```

---

## 4.4 신체 구성 데이터 형식

### 4.4.1 신체 지표 스키마

```typescript
interface BodyComposition {
  timestamp: Date;
  userId: string;

  // 기본 측정
  weight: number;                // kg
  height: number;                // cm
  bmi: number;                   // 계산됨

  // 체지방
  bodyFatPercentage?: number;    // %
  bodyFatMass?: number;          // kg
  leanMass?: number;             // kg

  // 고급 지표
  visceralFat?: number;          // 레벨 1-59
  muscleMass?: number;           // kg
  boneMass?: number;             // kg
  waterPercentage?: number;      // %

  // 대사
  basalMetabolicRate?: number;   // kcal/일
  metabolicAge?: number;         // 년

  // 신체 측정
  measurements?: {
    neck?: number;               // cm
    chest?: number;              // cm
    waist?: number;              // cm
    hips?: number;               // cm
    thigh?: number;              // cm
    arm?: number;                // cm
  };

  // 측정 방법
  method?: 'bioimpedance' | 'caliper' | 'dexa' | 'manual' | 'estimated';
}
```

---

## 4.5 수면 데이터 형식

### 4.5.1 수면 세션 스키마

```typescript
interface SleepSession {
  id: string;
  userId: string;

  // 타이밍
  startTime: Date;               // 잠자리에 들 때
  endTime: Date;                 // 일어날 때
  totalDuration: number;         // 침대에 있는 분

  // 수면 단계
  stages: {
    awake: number;               // 분
    light: number;               // 분
    deep: number;                // 분
    rem: number;                 // 분
  };

  // 품질 지표
  quality: {
    score: number;               // 0-100
    efficiency: number;          // % (수면 시간 / 침대 시간)
    interruptions: number;       // 각성 횟수
    restlessness: number;        // 움직임 횟수
    timeToSleep: number;         // 잠들기까지 분
  };

  // 생리학적 지표
  heartRate?: {
    avg: number;                 // 수면 중 BPM
    min: number;                 // 최저 BPM
    max: number;                 // 최고 BPM
  };

  hrv?: {
    avg: number;                 // 평균 RMSSD (ms)
  };

  respiratoryRate?: number;      // 분당 호흡수
  oxygenSaturation?: {
    avg: number;                 // SpO2 %
    min: number;                 // 최저 SpO2 %
  };

  // 사용자 피드백
  feeling?: 'refreshed' | 'good' | 'fair' | 'tired' | 'exhausted';
  notes?: string;
}
```

---

## 4.6 목표 및 업적 데이터 형식

### 4.6.1 피트니스 목표 스키마

```typescript
interface FitnessGoal {
  id: string;
  userId: string;

  // 목표 정의
  type: GoalType;
  name: string;
  description?: string;

  // 목표값
  target: number;
  current: number;
  unit: string;                  // "걸음", "km", "운동", "kg" 등

  // 타임라인
  period: 'daily' | 'weekly' | 'monthly' | 'yearly' | 'one-time';
  startDate: Date;
  endDate?: Date;

  // 진행 상황
  progress: number;              // 0-100%
  status: 'active' | 'completed' | 'abandoned' | 'paused';
}

enum GoalType {
  DAILY_STEPS = 'daily_steps',
  WEEKLY_DISTANCE = 'weekly_distance',
  MONTHLY_WORKOUTS = 'monthly_workouts',
  CALORIE_BURN = 'calorie_burn',
  WEIGHT_LOSS = 'weight_loss',
  WORKOUT_STREAK = 'workout_streak'
}
```

### 4.6.2 업적 스키마

```typescript
interface Achievement {
  id: string;
  name: string;
  description: string;
  category: 'distance' | 'duration' | 'frequency' | 'milestone';
  tier: 'bronze' | 'silver' | 'gold' | 'platinum';

  // 요구사항
  requirements: {
    metric: string;
    value: number;
    operator: '>=' | '<=' | '==' | '>';
  };

  // 사용자 진행 상황
  earnedDate?: Date;
  progress: number;              // 0-100%
  unlocked: boolean;
}
```

---

## 핵심 요점

✓ 모든 데이터 유형은 TypeScript 인터페이스로 표준화된 JSON 스키마 사용

✓ 활동 형식은 간단한 추적과 상세한 운동 모두 지원

✓ GPS 데이터는 정확한 경로 추적을 위한 정확도 및 고도 포함

✓ 심박수 데이터는 실시간 스트리밍 및 HRV 분석 지원

✓ 신체 구성 스키마는 기본 및 고급 지표 포괄

✓ 수면 추적은 단계, 품질 점수, 생리학적 데이터 포함

✓ 목표와 업적은 게임화 및 동기부여 제공

✓ 모든 타임스탬프는 일관성을 위해 ISO 8601 형식 사용

---

**다음:** [5장: API 인터페이스 →](05-api-interface.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
