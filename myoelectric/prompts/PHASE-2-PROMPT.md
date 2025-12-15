# WIA Myoelectric - Phase 2: Gesture Recognition API

## 목표
EMG 신호에서 손 제스처를 인식하는 API를 구현합니다.

## 2.1 제스처 분류 API

```typescript
interface GestureClassifier {
  // 모델 로드/저장
  loadModel(path: string): Promise<void>;
  saveModel(path: string): Promise<void>;

  // 실시간 분류
  classify(features: FeatureVector): ClassificationResult;
  classifyStream(stream: Observable<FeatureVector>): Observable<ClassificationResult>;

  // 학습
  train(dataset: TrainingDataset): TrainingResult;
  finetune(samples: LabeledSample[]): void;  // 사용자별 미세조정
}

interface ClassificationResult {
  gesture: Gesture;
  confidence: number;           // 0.0 - 1.0
  alternatives: {
    gesture: Gesture;
    confidence: number;
  }[];
  latency: number;              // ms
}

enum Gesture {
  REST = 'rest',
  HAND_OPEN = 'hand_open',
  HAND_CLOSE = 'hand_close',
  WRIST_FLEXION = 'wrist_flexion',
  WRIST_EXTENSION = 'wrist_extension',
  WRIST_PRONATION = 'wrist_pronation',
  WRIST_SUPINATION = 'wrist_supination',
  PINCH_THUMB_INDEX = 'pinch_thumb_index',
  PINCH_THUMB_MIDDLE = 'pinch_thumb_middle',
  POINT_INDEX = 'point_index',
  THUMBS_UP = 'thumbs_up',
}
```

## 2.2 분류 알고리즘

```typescript
// 알고리즘 추상화
interface ClassificationAlgorithm {
  name: string;
  train(X: number[][], y: number[]): void;
  predict(x: number[]): number;
  predictProba(x: number[]): number[];
}

// 지원 알고리즘
const ALGORITHMS = {
  // 전통적 ML
  svm: SVMClassifier,           // Support Vector Machine
  lda: LDAClassifier,           // Linear Discriminant Analysis
  knn: KNNClassifier,           // K-Nearest Neighbors
  rf: RandomForestClassifier,   // Random Forest

  // 딥러닝
  cnn: CNNClassifier,           // Convolutional Neural Network
  lstm: LSTMClassifier,         // Long Short-Term Memory
  tcn: TCNClassifier,           // Temporal Convolutional Network
  transformer: TransformerClassifier,
};
```

## 2.3 사용자 캘리브레이션

```typescript
interface CalibrationSession {
  // 캘리브레이션 시작
  start(): void;

  // 제스처별 샘플 수집
  collectSample(gesture: Gesture): Promise<void>;

  // 진행 상황
  getProgress(): CalibrationProgress;

  // 완료 및 모델 생성
  complete(): Promise<UserModel>;
}

interface CalibrationProgress {
  gesture: Gesture;
  samplesCollected: number;
  samplesRequired: number;
  quality: 'poor' | 'fair' | 'good' | 'excellent';
}

interface UserModel {
  userId: string;
  createdAt: Date;
  gestures: Gesture[];
  accuracy: number;
  calibrationQuality: number;
}
```

## 2.4 실시간 처리 요구사항

| 항목 | 요구사항 |
|------|----------|
| 분류 지연시간 | < 50ms |
| 정확도 | > 90% (5개 제스처) |
| 오탐률 | < 5% |
| 프레임 레이트 | > 20fps |

---

## 산출물

```
myoelectric/
├── api/
│   ├── typescript/
│   │   └── src/
│   │       ├── classifier/
│   │       ├── calibration/
│   │       └── streaming/
│   ├── python/
│   │   └── wia_myoelectric/
│   │       ├── ml/
│   │       │   ├── svm.py
│   │       │   ├── cnn.py
│   │       │   └── transformer.py
│   │       └── calibration/
│   └── rust/
│       └── src/
│           ├── classifier.rs
│           └── realtime.rs
├── models/
│   └── pretrained/
│       ├── 5gesture_cnn.onnx
│       └── 10gesture_transformer.onnx
```

---

## 다음: Phase 3 (저가 하드웨어)
