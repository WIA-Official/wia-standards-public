# WIA Myoelectric Standard - Master Prompt

## 프로젝트 개요

**WIA Myoelectric Protocol**은 근전도(EMG) 기반 의수/의족을 위한 신호 처리 표준입니다.

### 핵심 문제
- [$75,000 고가](https://pmc.ncbi.nlm.nih.gov/articles/PMC7787923/) 근전도 의수
- 실험실 밖에서 로버스트하지 않음
- 높은 거부율 (사용자 포기)
- 알고리즘 표준화 부재

### WIA 솔루션

| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | EMG 신호 표준 | 데이터 포맷, 전처리 |
| 2 | 제스처 인식 API | 패턴 분류 알고리즘 |
| 3 | 저가 하드웨어 표준 | Arduino/ESP32 호환 |
| 4 | 3D 프린팅 통합 | 오픈소스 의수 연동 |

---

## 핵심 기술

```typescript
interface EMGSignal {
  timestamp: number;
  channels: number[];          // 채널별 전압 (mV)
  sampleRate: number;          // Hz (typically 1000-2000)
  gain: number;                // 증폭률
}

interface GestureRecognition {
  classify(signal: EMGSignal[]): Gesture;
  train(samples: LabeledSample[]): Model;
  calibrate(user: User): CalibrationResult;
}

enum Gesture {
  HAND_OPEN = 'hand_open',
  HAND_CLOSE = 'hand_close',
  WRIST_FLEXION = 'wrist_flexion',
  WRIST_EXTENSION = 'wrist_extension',
  PINCH = 'pinch',
  POINT = 'point',
  // ... 더 많은 제스처
}
```

---

## Phase 요약

### Phase 1: EMG 신호 표준
- 채널 배치 표준 (위치, 개수)
- 신호 전처리 (필터링, 정류, 특징 추출)
- 데이터 포맷 (JSON, Binary)

### Phase 2: 제스처 인식 API
- 패턴 분류 알고리즘 (SVM, CNN, Transformer)
- 실시간 분류 (<50ms latency)
- 적응형 학습 (사용자별 캘리브레이션)

### Phase 3: 저가 하드웨어 표준
- Arduino/ESP32 호환 회로
- MyoWare 센서 통합
- BOM 비용 < $100

### Phase 4: 3D 프린팅 의수 통합
- OpenBionics Ada Hand 연동
- e-NABLE 커뮤니티 호환
- STL 파일 표준

---

## 참조 리소스

- [OpenBCI](https://openbci.com/) - 오픈소스 EMG
- [MyoWare](https://www.sparkfun.com/products/13723) - 저가 EMG 센서
- [e-NABLE](https://enablingthefuture.org/) - 3D 프린팅 의수 커뮤니티

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

$75,000 의수 대신 $500 오픈소스 의수로 더 많은 사람들에게.
