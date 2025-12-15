# WIA Myoelectric - Phase 1: EMG Signal Standard

## 목표
근전도(EMG) 신호의 표준 데이터 포맷과 전처리 파이프라인을 정의합니다.

## 1.1 EMG 데이터 포맷

```typescript
interface EMGDataFrame {
  // 메타데이터
  metadata: {
    sampleRate: number;        // 1000-2000Hz 권장
    channelCount: number;      // 2-8채널
    resolution: number;        // bits (10-24)
    gain: number;              // 증폭률
    referenceType: 'monopolar' | 'bipolar' | 'differential';
  };

  // 채널별 데이터
  channels: EMGChannel[];

  // 타임스탬프
  startTime: number;           // Unix ms
  duration: number;            // ms
}

interface EMGChannel {
  id: number;
  name: string;                // 'flexor_carpi_radialis'
  placement: ElectrodePlacement;
  samples: Float32Array;       // 원시 샘플
}

interface ElectrodePlacement {
  muscle: string;              // 근육 이름
  location: {
    x: number;                 // 팔꿈치 기준 거리 (cm)
    y: number;                 // 근육 중심 기준 오프셋
    circumference: number;     // 팔둘레 위치 (도)
  };
}
```

## 1.2 신호 전처리 파이프라인

```typescript
interface EMGPreprocessor {
  // 1. 노이즈 필터링
  notchFilter(signal: Float32Array, freq: number): Float32Array;  // 50/60Hz
  bandpassFilter(signal: Float32Array, low: number, high: number): Float32Array;  // 20-450Hz

  // 2. 정류
  rectify(signal: Float32Array): Float32Array;

  // 3. 엔벨로프 추출
  envelope(signal: Float32Array, windowMs: number): Float32Array;

  // 4. 정규화
  normalize(signal: Float32Array, method: 'mvc' | 'zscore' | 'minmax'): Float32Array;
}

// MVC (Maximum Voluntary Contraction) 정규화
interface MVCCalibration {
  muscle: string;
  mvcValue: number;            // 최대 수축 시 EMG 값
  calibrationDate: Date;
}
```

## 1.3 특징 추출

```typescript
interface EMGFeatureExtractor {
  // 시간 영역 특징
  temporal: {
    mav: (signal: Float32Array) => number;      // Mean Absolute Value
    rms: (signal: Float32Array) => number;      // Root Mean Square
    wl: (signal: Float32Array) => number;       // Waveform Length
    zc: (signal: Float32Array) => number;       // Zero Crossings
    ssc: (signal: Float32Array) => number;      // Slope Sign Changes
  };

  // 주파수 영역 특징
  frequency: {
    meanFreq: (signal: Float32Array) => number;
    medianFreq: (signal: Float32Array) => number;
    powerSpectrum: (signal: Float32Array) => Float32Array;
  };

  // 특징 벡터 생성
  extractFeatures(
    signal: Float32Array,
    windowSize: number,
    overlap: number
  ): FeatureVector;
}

interface FeatureVector {
  timestamp: number;
  features: number[];          // [mav1, rms1, wl1, ..., mavN, rmsN, wlN]
  channelCount: number;
  featureNames: string[];
}
```

---

## 산출물

```
myoelectric/
├── spec/
│   ├── EMG-DATA-FORMAT.md
│   ├── PREPROCESSING-PIPELINE.md
│   ├── FEATURE-EXTRACTION.md
│   └── ELECTRODE-PLACEMENT-GUIDE.md
├── schemas/
│   ├── emg-data.schema.json
│   └── electrode-placement.schema.json
├── api/rust/src/
│   ├── signal/
│   │   ├── filter.rs
│   │   ├── rectify.rs
│   │   ├── envelope.rs
│   │   └── features.rs
```

---

## 다음: Phase 2 (제스처 인식 API)
