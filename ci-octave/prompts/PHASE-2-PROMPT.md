# Phase 2: Octave Detection Algorithm
## Claude Code 작업 프롬프트

---

**Phase**: 2 of 4
**목표**: 음향 신호에서 옥타브/피치 정보를 추출하는 알고리즘 개발
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + 알고리즘 구현 + 테스트

---

## 🎯 Phase 2 목표

### 핵심 질문
```
"Phase 1에서 CI 신호의 옥타브 손실 문제를 분석했다.

 이제 원본 음향 신호에서 옥타브/피치 정보를
 정확하게 추출할 수 있는 알고리즘을 만들 수 있을까?

 이 정보를 CI 신호에 '추가'하면
 사용자가 옥타브를 인식할 수 있을까?"
```

### 목표
```
1. 음향 신호에서 F0 (기본 주파수) 추출
2. 옥타브 정보 정확하게 판별
3. 실시간 처리 가능한 알고리즘 설계
4. CI 신호 인핸스먼트를 위한 데이터 출력
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 피치 검출 알고리즘 조사

| 알고리즘 | 설명 | 웹서치 키워드 |
|---------|------|--------------|
| **Autocorrelation** | 자기상관 기반 | "autocorrelation pitch detection algorithm" |
| **YIN** | 차분함수 기반 | "YIN pitch detection algorithm" |
| **PYIN** | 확률적 YIN | "pYIN probabilistic pitch detection" |
| **CREPE** | 딥러닝 기반 | "CREPE CNN pitch estimation" |
| **SPICE** | Google 모델 | "SPICE pitch estimation Google" |
| **HPS** | Harmonic Product Spectrum | "harmonic product spectrum pitch detection" |

### 2단계: 옥타브 결정 연구 조사

| 연구 주제 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **옥타브 모호성** | Octave error 문제 | "pitch detection octave error problem" |
| **Harmonic Matching** | 하모닉 구조 분석 | "harmonic structure octave determination" |
| **Spectral Analysis** | 스펙트럼 분석 | "spectral centroid octave detection" |
| **Machine Learning** | ML 기반 접근 | "machine learning pitch octave estimation" |

### 3단계: 실시간 처리 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **지연 시간** | 허용 가능 지연 | "real-time audio processing latency requirement" |
| **FFT 최적화** | 고속 FFT | "real-time FFT implementation" |
| **STFT** | Short-Time FFT | "STFT real-time pitch tracking" |
| **CI 지연** | 기존 CI 처리 지연 | "cochlear implant processing delay" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-2.md`에 다음을 정리:

```markdown
# Phase 2 사전 조사 결과

## 1. 피치 검출 알고리즘 비교

### Autocorrelation
- 원리: [조사 내용]
- 장점: [조사 내용]
- 단점: [조사 내용]
- 옥타브 오류율: [조사 내용]

### YIN/pYIN
- 원리: [조사 내용]
- 장점: [조사 내용]
- 단점: [조사 내용]
- 옥타브 오류율: [조사 내용]

### CREPE (딥러닝)
- 원리: [조사 내용]
- 장점: [조사 내용]
- 단점: [조사 내용]
- 옥타브 오류율: [조사 내용]

## 2. 옥타브 결정 문제

### 문제 정의
- 옥타브 오류란: [설명]
- 발생 원인: [분석]

### 기존 해결 방법
- [방법 1]: [설명]
- [방법 2]: [설명]

## 3. 실시간 처리 요구사항

### CI 처리 지연
- 현재 CI 지연: [조사 내용] ms
- 허용 가능 추가 지연: [분석] ms

### 알고리즘 복잡도
- YIN: O(n log n)
- CREPE: O(n) but GPU 필요

## 4. 결론

### 권장 알고리즘
- [선택 및 이유]

### 옥타브 결정 전략
- [제안]
```

---

## 🏗️ 알고리즘 설계

### 1. 옥타브 검출 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│                    원본 오디오 신호                           │
│                    (44.1kHz, 16bit)                         │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   전처리 (Preprocessing)                     │
│     - 다운샘플링 (16kHz)                                     │
│     - 프레임 분할 (20-50ms)                                  │
│     - 윈도우 함수 적용 (Hann)                                │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   F0 후보 추출 (F0 Candidates)               │
│     - YIN 알고리즘                                          │
│     - 다중 F0 후보 생성 (f, 2f, f/2 등)                      │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   하모닉 분석 (Harmonic Analysis)            │
│     - FFT 스펙트럼 분석                                      │
│     - 하모닉 피크 검출                                       │
│     - 하모닉 에너지 비율 계산                                 │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   옥타브 결정 (Octave Decision)              │
│     - 하모닉 일치도 계산                                     │
│     - Spectral Centroid 분석                                │
│     - 확률적 옥타브 선택                                     │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   출력 (Octave Info)                         │
│     - F0 (Hz)                                               │
│     - Octave (0-8)                                          │
│     - Note (C, C#, D, ...)                                  │
│     - Confidence (0.0-1.0)                                  │
└─────────────────────────────────────────────────────────────┘
```

### 2. 핵심 알고리즘: OctaveYIN

```typescript
interface OctaveYINConfig {
  sampleRate: number;         // 16000
  frameSize: number;          // 1024 (64ms at 16kHz)
  hopSize: number;            // 256 (16ms)
  minF0: number;              // 50 Hz
  maxF0: number;              // 2000 Hz
  threshold: number;          // 0.1 (YIN threshold)
  harmonicWeight: number;     // 0.7 (하모닉 분석 가중치)
}

interface OctaveResult {
  timestamp: number;          // ms
  f0: number;                 // Hz
  octave: number;             // 0-8
  note: string;               // "A", "B", "C#", etc.
  cents: number;              // -50 to +50
  confidence: number;         // 0.0-1.0
  harmonicStrength: number;   // 하모닉 일치도
  candidates: F0Candidate[];  // 다른 후보들
}

interface F0Candidate {
  f0: number;
  octave: number;
  score: number;              // 종합 점수
}
```

### 3. 하모닉 기반 옥타브 결정

```typescript
/**
 * 하모닉 구조 분석으로 옥타브 결정
 *
 * 원리:
 * - 진짜 F0의 하모닉(2f, 3f, 4f...)은 스펙트럼에 피크가 있어야 함
 * - 옥타브 오류(f/2 또는 2f)면 하모닉 패턴이 불일치
 */
function determineOctave(
  spectrum: Float32Array,
  f0Candidates: number[],
  sampleRate: number
): OctaveResult {

  const scores: Map<number, number> = new Map();

  for (const f0 of f0Candidates) {
    let harmonicScore = 0;

    // 1차-8차 하모닉 검사
    for (let h = 1; h <= 8; h++) {
      const harmonicFreq = f0 * h;
      const binIndex = Math.round(harmonicFreq * spectrum.length / sampleRate);

      if (binIndex < spectrum.length) {
        // 해당 주파수에 피크가 있는가?
        const peakStrength = findPeakStrength(spectrum, binIndex);
        harmonicScore += peakStrength / h;  // 고차 하모닉은 가중치 낮춤
      }
    }

    scores.set(f0, harmonicScore);
  }

  // 가장 높은 점수의 F0 선택
  const bestF0 = [...scores.entries()].sort((a, b) => b[1] - a[1])[0][0];

  return {
    f0: bestF0,
    octave: frequencyToOctave(bestF0),
    note: frequencyToNote(bestF0),
    cents: frequencyToCents(bestF0),
    confidence: scores.get(bestF0)! / getMaxPossibleScore(),
    // ...
  };
}
```

### 4. 실시간 처리 구조

```typescript
class RealTimeOctaveDetector {
  private buffer: Float32Array;
  private config: OctaveYINConfig;
  private yin: YINProcessor;
  private harmonicAnalyzer: HarmonicAnalyzer;

  constructor(config: OctaveYINConfig) {
    this.config = config;
    this.buffer = new Float32Array(config.frameSize);
    this.yin = new YINProcessor(config);
    this.harmonicAnalyzer = new HarmonicAnalyzer(config);
  }

  /**
   * 오디오 프레임 처리 (실시간)
   * @param frame 입력 오디오 프레임 (hopSize 샘플)
   * @returns 옥타브 정보 또는 null (무음/불확실)
   */
  processFrame(frame: Float32Array): OctaveResult | null {
    // 버퍼 갱신 (overlap-add)
    this.updateBuffer(frame);

    // 1. YIN으로 F0 후보 추출
    const f0Candidates = this.yin.getCandidates(this.buffer);

    if (f0Candidates.length === 0) {
      return null;  // 피치 검출 불가
    }

    // 2. FFT로 스펙트럼 분석
    const spectrum = this.getSpectrum(this.buffer);

    // 3. 하모닉 분석으로 옥타브 결정
    const result = this.harmonicAnalyzer.determineOctave(
      spectrum,
      f0Candidates
    );

    // 4. 신뢰도 필터링
    if (result.confidence < 0.5) {
      return null;
    }

    return result;
  }

  /**
   * 지연 시간 (latency)
   * @returns 알고리즘 지연 (ms)
   */
  getLatency(): number {
    // 프레임 크기 + 처리 시간
    return (this.config.frameSize / this.config.sampleRate) * 1000 + 2;
  }
}
```

---

## 📁 산출물 목록

Phase 2 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-2.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-2-OCTAVE-DETECTION.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 피치 검출 알고리즘 비교 (Pitch Detection Algorithms)
4. OctaveYIN 알고리즘 (Our Algorithm)
5. 하모닉 분석 (Harmonic Analysis)
6. 옥타브 결정 로직 (Octave Decision Logic)
7. 실시간 처리 (Real-time Processing)
8. API 인터페이스 (API Interface)
9. 정확도 평가 (Accuracy Evaluation)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. TypeScript 구현
```
/api/typescript/src/
├── octave/
│   ├── index.ts
│   ├── types.ts                    # 타입 정의
│   ├── OctaveYIN.ts                # 메인 알고리즘
│   ├── YINProcessor.ts             # YIN 구현
│   ├── HarmonicAnalyzer.ts         # 하모닉 분석
│   ├── FFTProcessor.ts             # FFT 처리
│   ├── RealTimeDetector.ts         # 실시간 처리
│   └── utils/
│       ├── frequency-utils.ts      # 주파수 변환 유틸
│       └── window-functions.ts     # 윈도우 함수
└── ...
```

### 4. Python 구현
```
/api/python/wia_ci/
├── octave/
│   ├── __init__.py
│   ├── types.py
│   ├── octave_yin.py
│   ├── yin_processor.py
│   ├── harmonic_analyzer.py
│   ├── fft_processor.py
│   ├── realtime_detector.py
│   └── utils/
│       ├── frequency_utils.py
│       └── window_functions.py
└── ...
```

### 5. 테스트 및 평가
```
/examples/octave-detection/
├── test-pure-tones.ts              # 순음 테스트
├── test-music.ts                   # 음악 테스트
├── test-speech.ts                  # 음성 테스트
├── accuracy-benchmark.ts           # 정확도 벤치마크
└── datasets/
    ├── pure-tones/                 # 테스트용 순음
    ├── music-samples/              # 음악 샘플
    └── ground-truth/               # 정답 레이블
```

---

## ✅ 완료 체크리스트

Phase 2 완료 전 확인:

```
□ 웹서치로 피치 검출 알고리즘 조사 완료
□ 웹서치로 옥타브 결정 연구 조사 완료
□ /spec/RESEARCH-PHASE-2.md 작성 완료
□ /spec/PHASE-2-OCTAVE-DETECTION.md 작성 완료
□ OctaveYIN 알고리즘 설계 완료
□ TypeScript 구현 완료
□ Python 구현 완료
□ 단위 테스트 작성 완료
□ 순음 정확도 > 95% 달성
□ 음악 정확도 > 85% 달성
□ 실시간 처리 지연 < 50ms 달성
□ README 업데이트 (Phase 2 완료 표시)
```

---

## 🎯 정확도 목표

| 테스트 유형 | 옥타브 정확도 목표 | 비고 |
|------------|------------------|------|
| **순음 (Pure Tone)** | > 99% | 단일 주파수 |
| **하모닉 (Harmonic)** | > 95% | 악기 소리 |
| **음악 (Melody)** | > 90% | 멜로디 라인 |
| **음성 (Speech)** | > 85% | 화자 피치 |
| **성조 (Tonal)** | > 90% | 중국어 등 |

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 옥타브 오류 방지에 집중 (핵심 목표)
✅ 실시간 처리 가능한 복잡도 유지
✅ 다양한 음원에서 테스트
✅ 신뢰도(confidence) 함께 출력
✅ 하모닉 분석 가중치 조절 가능하게
```

### DON'T (하지 말 것)

```
❌ 옥타브 오류 무시 (기존 알고리즘의 문제점)
❌ 오프라인 전용 알고리즘 (실시간 필수)
❌ 과도한 GPU 의존 (임베디드 고려)
❌ 단일 알고리즘만 사용 (앙상블 권장)
```

---

## 🚀 작업 시작

이제 Phase 2 작업을 시작하세요.

첫 번째 단계: **웹서치로 YIN 알고리즘 상세 조사**

```
검색 키워드: "YIN pitch detection algorithm octave error prevention"
```

화이팅! 🤟

---

<div align="center">

**Phase 2 of 4**

Octave Detection Algorithm

**"어떤 음인지 정확하게 아는 것"**

</div>
