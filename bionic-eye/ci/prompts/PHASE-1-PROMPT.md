# Phase 1: CI Signal Analysis Standard
## Claude Code 작업 프롬프트

---

**Phase**: 1 of 4
**목표**: 인공와우(Cochlear Implant) 신호 형식 분석 및 표준화
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 분석 도구

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"인공와우는 22개의 전극으로 소리를 전달한다.
 각 전극은 특정 주파수 대역을 담당한다.

 하지만 '옥타브'와 '피치' 정보가 손실된다.
 → 로봇 목소리처럼 들린다
 → 음악 감상이 어렵다
 → 성조 언어(중국어 등) 인식이 어렵다

 현재 CI 신호 형식을 분석하고,
 어떤 정보가 손실되는지 정확히 파악할 수 있을까?"
```

### 목표
```
1. 현재 인공와우 신호 처리 방식 분석
2. 손실되는 옥타브/피치 정보 정의
3. 분석을 위한 표준 신호 형식 정의
4. 옥타브 정보 보존을 위한 기반 마련
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 인공와우 신호 처리 조사

| 조사 대상 | 내용 | 웹서치 키워드 |
|----------|------|--------------|
| **CI 기본 원리** | 전극 배열, 주파수 매핑 | "cochlear implant electrode array frequency mapping" |
| **CIS 전략** | Continuous Interleaved Sampling | "CIS speech processing strategy cochlear implant" |
| **ACE 전략** | Advanced Combination Encoder | "ACE Cochlear Nucleus speech processor" |
| **FSP 전략** | Fine Structure Processing | "Fine Structure Processing Med-El" |
| **주파수 대역** | 22채널 주파수 분배 | "cochlear implant frequency allocation 22 channels" |

### 2단계: 주요 CI 제조사 기술 조사

| 제조사 | 조사 대상 | 웹서치 키워드 |
|-------|----------|--------------|
| **Cochlear Ltd** | Nucleus 시리즈 | "Cochlear Nucleus 8 signal processing" |
| **MED-EL** | OPUS, SONNET | "Med-El cochlear implant signal processing" |
| **Advanced Bionics** | HiRes 기술 | "Advanced Bionics HiRes Optima" |

### 3단계: 옥타브/피치 문제 조사

| 조사 대상 | 내용 | 웹서치 키워드 |
|----------|------|--------------|
| **피치 인식 한계** | CI 사용자 음높이 인식 | "cochlear implant pitch perception limitation" |
| **음악 청취** | CI 사용자 음악 경험 | "cochlear implant music perception research" |
| **성조 언어** | 중국어 등 톤 인식 | "cochlear implant tonal language Mandarin" |
| **Fine Structure** | 미세 구조 정보 손실 | "temporal fine structure cochlear implant" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. 인공와우 기본 원리

### 전극 배열 (Electrode Array)
- 전극 수: [조사 내용]
- 주파수 배치: [조사 내용]
- Tonotopic 매핑: [조사 내용]

### 신호 처리 전략
- CIS (Continuous Interleaved Sampling): [조사 내용]
- ACE (Advanced Combination Encoder): [조사 내용]
- FSP (Fine Structure Processing): [조사 내용]

## 2. 주파수 대역 분석

### 22채널 주파수 분배 (예시: Cochlear)
| 채널 | 주파수 범위 (Hz) | 담당 옥타브 |
|------|-----------------|------------|
| 1    | 188-313         | ~A3-D#4    |
| 2    | 313-438         | ~D#4-A4    |
| ...  | ...             | ...        |
| 22   | 6563-7938       | ~G#8-B8    |

## 3. 손실되는 정보 분석

### Temporal Fine Structure (TFS)
- 정의: [조사 내용]
- 손실 원인: [조사 내용]
- 영향: [조사 내용]

### Spectral Resolution
- 한계: [조사 내용]
- 22개 채널 vs 3,500개 유모세포

### Pitch Perception
- 현재 인식률: [조사 내용]
- 옥타브 오류율: [조사 내용]

## 4. 기존 개선 시도

### Fine Structure Processing (FSP)
- MED-EL 접근법: [조사 내용]
- 한계점: [조사 내용]

### Current Steering
- 가상 채널 생성: [조사 내용]
- 한계점: [조사 내용]

## 5. 결론

### 핵심 문제
- [분석 결과]

### 개선 가능성
- [제안]
```

---

## 🏗️ 신호 분석 표준 설계

### 1. CI 신호 표준 형식

```json
{
  "$schema": "https://wia.live/ci/signal/v1/schema.json",
  "version": "1.0.0",
  "type": "ci_electrode_signal",
  "device": {
    "manufacturer": "Cochlear",
    "model": "Nucleus 8",
    "strategy": "ACE",
    "channels": 22
  },
  "timestamp": "2025-12-14T12:00:00.000Z",
  "sampleRate": 17400,
  "data": {
    "electrodes": [
      {
        "id": 1,
        "frequency": {"min": 188, "max": 313, "center": 250},
        "amplitude": 0.75,
        "pulseRate": 900,
        "pulseWidth": 25
      },
      // ... 22개 채널
    ],
    "envelope": {
      "channel": [0.8, 0.6, 0.9, ...],
      "timestamp_us": 57
    }
  },
  "analysis": {
    "dominantFrequency": 440,
    "estimatedPitch": "A4",
    "octave": 4,
    "confidence": 0.85
  }
}
```

### 2. 옥타브 정보 보존 형식

```json
{
  "octaveAnalysis": {
    "inputSignal": {
      "fundamentalFrequency": 440.0,
      "harmonics": [880, 1320, 1760, 2200],
      "octave": 4,
      "note": "A",
      "cents": 0
    },
    "ciOutput": {
      "activatedChannels": [5, 8, 12, 15],
      "channelEnergies": [0.9, 0.7, 0.5, 0.3],
      "perceivedPitch": "ambiguous",
      "octaveConfidence": 0.45
    },
    "octaveLoss": {
      "expectedOctave": 4,
      "perceivedOctaveRange": [3, 5],
      "errorMargin": 1,
      "cause": "insufficient_tfs"
    },
    "enhancement": {
      "required": true,
      "suggestedMethod": "temporal_modulation",
      "parameters": {
        "modulationFrequency": 110,
        "modulationDepth": 0.3
      }
    }
  }
}
```

### 3. 음악 신호 분석 형식

```json
{
  "musicAnalysis": {
    "source": {
      "sampleRate": 44100,
      "duration_ms": 100,
      "type": "melody"
    },
    "pitchTrack": [
      {"time_ms": 0, "frequency": 440, "note": "A4", "confidence": 0.95},
      {"time_ms": 10, "frequency": 442, "note": "A4", "confidence": 0.93},
      // ...
    ],
    "harmonyStructure": {
      "fundamental": 440,
      "harmonics": [880, 1320, 1760],
      "harmonicRatios": [1.0, 0.8, 0.5, 0.3]
    },
    "ciSimulation": {
      "channelActivation": [
        {"channel": 7, "energy": 0.9, "frequency": 438},
        {"channel": 12, "energy": 0.7, "frequency": 875}
      ],
      "pitchAmbiguity": true,
      "octaveError": 1
    }
  }
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-CI-SIGNAL-ANALYSIS.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 인공와우 신호 처리 원리 (CI Signal Processing)
4. 주파수-전극 매핑 (Frequency-Electrode Mapping)
5. 옥타브/피치 정보 손실 분석 (Octave/Pitch Loss Analysis)
6. 표준 신호 형식 (Standard Signal Format)
7. 분석 데이터 구조 (Analysis Data Structure)
8. 음악 신호 분석 (Music Signal Analysis)
9. 확장성 (Extensibility)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── wia-ci-signal-v1.schema.json       (기본 CI 신호 스키마)
├── octave-analysis.schema.json        (옥타브 분석 스키마)
└── music-analysis.schema.json         (음악 분석 스키마)
```

### 4. 분석 도구
```
/api/
├── typescript/
│   └── src/
│       ├── signal-analyzer.ts          (신호 분석기)
│       ├── frequency-mapper.ts         (주파수 매핑)
│       └── octave-detector.ts          (옥타브 검출)
└── python/
    └── wia_ci/
        ├── signal_analyzer.py
        ├── frequency_mapper.py
        └── octave_detector.py
```

### 5. 예제 데이터
```
/examples/sample-data/
├── pure-tone-440hz.json               (순음 A4)
├── melody-sample.json                 (멜로디)
├── speech-sample.json                 (음성)
└── mandarin-tone-sample.json          (중국어 성조)
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 CI 신호 처리 원리 조사 완료
□ 웹서치로 주요 제조사 기술 조사 완료
□ 웹서치로 옥타브/피치 문제 연구 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-CI-SIGNAL-ANALYSIS.md 작성 완료
□ JSON Schema 파일 생성 완료
□ 신호 분석 도구 구현 완료 (TypeScript + Python)
□ 예제 데이터 파일 생성 완료
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 CI 기본 원리 조사
   ↓
2. 웹서치로 주파수-전극 매핑 조사
   ↓
3. 웹서치로 옥타브 손실 연구 조사
   ↓
4. /spec/RESEARCH-PHASE-1.md 작성
   ↓
5. 신호 분석 표준 형식 설계
   ↓
6. /spec/PHASE-1-CI-SIGNAL-ANALYSIS.md 작성
   ↓
7. JSON Schema 파일 생성
   ↓
8. 분석 도구 구현
   ↓
9. 예제 데이터 생성
   ↓
10. 완료 체크리스트 확인
   ↓
11. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 실제 CI 제조사 기술 문서/연구 논문 웹서치 필수
✅ 주파수-전극 매핑은 실제 CI 기준으로 정확하게
✅ 옥타브 손실 문제를 정량적으로 분석
✅ 확장 가능한 데이터 구조로 설계
✅ 기존 연구(FSP, Current Steering 등) 참조
```

### DON'T (하지 말 것)

```
❌ 의료기기 규제 관련 클레임 (표준만 정의)
❌ 특정 제조사 비하/비교
❌ 추측 기반 데이터 (반드시 조사 후)
❌ 기존 연구 무시 (관련 분야 자료 존중)
```

---

## 📞 핵심 개념 정리

### 왜 옥타브가 손실되는가?

```
정상 청력:
├── 달팽이관 3,500개 유모세포
├── 각 유모세포 = 특정 주파수 담당
├── 연속적인 주파수 해상도
└── 옥타브/피치 정보 완벽 보존

인공와우:
├── 22개 전극
├── 각 전극 = 넓은 주파수 대역 담당
├── 이산적(discrete) 주파수 해상도
├── Envelope 정보만 전달
└── Fine Structure(TFS) 손실 → 옥타브 손실
```

### Fine Structure가 중요한 이유

```
음악 A4 (440Hz):
├── Envelope: 볼륨의 시간적 변화 ✅ CI 전달됨
├── Fine Structure: 440Hz 자체의 파형 ❌ CI 손실됨
└── 결과: "뭔가 음이 들리지만, 무슨 음인지 모름"

성조 언어 (중국어):
├── "mā" (妈, 엄마) vs "mà" (骂, 욕하다)
├── 차이: 피치 변화 패턴
├── CI: 피치 변화 감지 어려움
└── 결과: 성조 언어 사용자에게 치명적
```

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 인공와우 신호 처리 원리 조사**

```
검색 키워드: "cochlear implant signal processing strategy CIS ACE FSP comparison"
```

화이팅! 🤟

---

<div align="center">

**Phase 1 of 4**

CI Signal Analysis Standard

**"로봇 목소리"에서 벗어나기 위한 첫 걸음**

</div>
