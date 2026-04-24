# WIA Bionic Eye - Phase 1: Visual Data Standard

## 목표
인공 시각 시스템의 시각 데이터 형식을 표준화합니다.

## 1.1 이미지 캡처 데이터

```typescript
interface CapturedFrame {
  // 프레임 메타데이터
  frameId: string;
  timestamp: number;           // Unix timestamp (ms)
  sequenceNumber: number;

  // 이미지 데이터
  image: {
    width: number;             // 픽셀
    height: number;            // 픽셀
    format: ImageFormat;       // GRAY8, RGB24, DEPTH16
    data: Uint8Array;          // Raw pixel data
    encoding: 'raw' | 'jpeg' | 'h264';
  };

  // 카메라 파라미터
  camera: {
    fov: number;               // 시야각 (degrees)
    exposure: number;          // 노출 시간 (ms)
    gain: number;              // ISO/게인
    whiteBalance: number;      // 색온도 (K)
  };

  // 센서 데이터
  sensors: {
    ambientLight: number;      // 주변 광량 (lux)
    proximity: number;         // 근접 센서 (cm)
    imu: IMUData;              // 머리 움직임
  };
}

interface IMUData {
  accelerometer: [number, number, number];  // m/s²
  gyroscope: [number, number, number];      // rad/s
  magnetometer: [number, number, number];   // μT
  orientation: Quaternion;
}

enum ImageFormat {
  GRAY8 = 'gray8',             // 8-bit grayscale
  GRAY16 = 'gray16',           // 16-bit grayscale
  RGB24 = 'rgb24',             // 24-bit RGB
  RGBD = 'rgbd',               // RGB + Depth
  DEPTH16 = 'depth16',         // 16-bit depth map
  IR = 'infrared',             // 적외선
}
```

## 1.2 처리된 시각 데이터

```typescript
interface ProcessedVisual {
  frameId: string;
  processingTime: number;      // ms

  // 엣지 검출 결과
  edges: {
    data: Uint8Array;          // Edge map
    algorithm: 'canny' | 'sobel' | 'laplacian';
    threshold: number;
  };

  // 객체 인식 결과
  objects: DetectedObject[];

  // 깊이 정보
  depth: {
    map: Float32Array;         // Depth values (meters)
    minDistance: number;
    maxDistance: number;
    confidenceMap: Uint8Array;
  };

  // 움직임 감지
  motion: {
    flowField: Float32Array;   // Optical flow
    movingRegions: Region[];
    velocity: number;          // 평균 이동 속도
  };

  // 텍스트 인식 (OCR)
  text: {
    regions: TextRegion[];
    language: string;
    confidence: number;
  };

  // 얼굴 인식
  faces: DetectedFace[];
}

interface DetectedObject {
  objectId: string;
  label: string;               // "person", "door", "stairs", "car"
  labelKorean: string;         // "사람", "문", "계단", "자동차"
  confidence: number;          // 0-1
  boundingBox: BoundingBox;
  distance: number;            // meters
  priority: ObjectPriority;    // 자극 우선순위
  threat: boolean;             // 위험 객체 여부
}

interface DetectedFace {
  faceId: string;
  boundingBox: BoundingBox;
  landmarks: FaceLandmarks;
  identity?: string;           // 알려진 사람 이름
  emotion?: string;            // 표정
  gazeDirection?: [number, number];
}

enum ObjectPriority {
  CRITICAL = 'critical',       // 장애물, 위험, 차량
  HIGH = 'high',               // 사람, 얼굴, 계단
  MEDIUM = 'medium',           // 물체, 가구, 문
  LOW = 'low',                 // 배경, 벽
}
```

## 1.3 자극 매핑 데이터

```typescript
interface StimulationMap {
  frameId: string;
  timestamp: number;

  // 전극 그리드 매핑
  electrodeGrid: {
    rows: number;              // 전극 행 수 (예: 6)
    columns: number;           // 전극 열 수 (예: 10)
    totalElectrodes: number;   // 총 전극 수 (예: 60)
    activeElectrodes: number[]; // 활성화할 전극 인덱스
  };

  // 픽셀-전극 매핑
  pixelMapping: {
    strategy: MappingStrategy;
    downsampledImage: Uint8Array;  // 다운샘플된 이미지
    intensityLevels: number;       // 밝기 단계 수 (예: 8)
    intensityMap: Uint8Array;      // 전극별 밝기 (0-255)
  };

  // 공간 필터링
  spatialFilter: {
    type: 'none' | 'center_surround' | 'edge_enhancement' | 'high_contrast';
    kernelSize: number;
    strength: number;          // 0-1
  };

  // 시간 필터링
  temporalFilter: {
    type: 'none' | 'motion_enhancement' | 'flicker_reduction';
    frameBuffer: number;       // 버퍼 프레임 수
  };
}

enum MappingStrategy {
  DIRECT = 'direct',           // 직접 픽셀-전극 매핑
  SCOREBOARD = 'scoreboard',   // 스코어보드 모델 (Argus II)
  AXON_MAP = 'axon_map',       // 축삭 기반 매핑
  PHOSPHENE = 'phosphene',     // 인광 위치 기반 매핑
  SALIENCY = 'saliency',       // 주의 기반 선택적 매핑
}
```

## 1.4 사용자 지각 피드백

```typescript
interface PerceptionFeedback {
  sessionId: string;
  timestamp: number;
  patientId: string;

  // 지각된 패턴
  perceivedPattern: {
    brightness: number;        // 0-10 (주관적 밝기)
    clarity: number;           // 0-10 (선명도)
    coverage: number;          // 0-10 (시야 범위)
    flickering: boolean;       // 깜빡임 여부
    afterImage: boolean;       // 잔상 여부
    color?: string;            // 지각된 색상 (있을 경우)
  };

  // 객체 인식 테스트
  recognition: {
    objectPresented: string;   // 제시된 객체
    objectRecognized: string;  // 인식한 객체
    correct: boolean;
    responseTime: number;      // ms
    confidenceLevel: number;   // 환자 확신도 0-10
  };

  // 방향/위치 지각
  spatial: {
    directionTest: {
      actualDirection: number;    // degrees (0-360)
      perceivedDirection: number;
      error: number;              // degrees
    };
    distanceTest: {
      actualDistance: number;     // cm
      perceivedDistance: number;
      error: number;              // cm
    };
    localizationScore: number;    // 0-100
  };

  // 이동 능력 평가
  mobility: {
    obstacleAvoidance: number;    // 0-100 (장애물 회피율)
    navigationAccuracy: number;   // 0-100
    walkingSpeed: number;         // m/s
    confidenceWalking: number;    // 0-10
  };

  // 불편감/부작용
  discomfort: {
    pain: number;              // 0-10
    dizziness: number;         // 0-10
    headache: number;          // 0-10
    nausea: number;            // 0-10
    eyeStrain: number;         // 0-10
  };
}
```

## 1.5 세션 데이터

```typescript
interface VisionSession {
  sessionId: string;
  patientId: string;
  deviceId: string;

  // 시간 정보
  startTime: Date;
  endTime: Date;
  duration: number;            // minutes

  // 환경 설정
  settings: {
    brightnessLevel: number;   // 0-100
    contrastLevel: number;     // 0-100
    zoomLevel: number;         // 1.0-4.0
    edgeEnhancement: boolean;
    objectHighlighting: boolean;
    audioFeedback: boolean;
  };

  // 프레임 통계
  frameStats: {
    totalFrames: number;
    droppedFrames: number;
    averageFps: number;
    averageLatency: number;    // ms
  };

  // 자극 통계
  stimulationStats: {
    totalStimulations: number;
    averageIntensity: number;
    peakIntensity: number;
    electrodesUsed: number[];
  };

  // 환자 피드백 요약
  feedbackSummary: {
    overallSatisfaction: number;  // 0-10
    mostUsefulFeature: string;
    difficulties: string[];
    suggestions: string[];
  };
}
```

---

## 산출물

```
bionic-eye/
├── spec/
│   ├── FRAME-DATA-SPEC.md
│   ├── PROCESSED-VISUAL-SPEC.md
│   ├── STIMULATION-MAP-SPEC.md
│   └── PERCEPTION-FEEDBACK-SPEC.md
├── schemas/
│   ├── captured-frame.schema.json
│   ├── processed-visual.schema.json
│   ├── stimulation-map.schema.json
│   └── perception-feedback.schema.json
└── prompts/
    └── PHASE-1-PROMPT.md
```
