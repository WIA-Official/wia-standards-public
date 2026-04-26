# Phase 3: CI Signal Enhancement Protocol
## Claude Code 작업 프롬프트

---

**Phase**: 3 of 4
**목표**: 옥타브 정보를 CI 신호에 인코딩하는 프로토콜 개발
**난이도**: ★★★★★
**예상 작업량**: 스펙 문서 1개 + 인코딩 알고리즘 + 시뮬레이션

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 2에서 옥타브 정보를 정확하게 추출했다.

 이제 이 정보를 CI 신호에 어떻게 '추가'할 것인가?

 22개 전극의 한계 내에서
 옥타브 정보를 인코딩할 수 있는 방법은 무엇인가?"
```

### 목표
```
1. 옥타브 정보 인코딩 전략 설계
2. CI 전극 자극 패턴 수정 알고리즘
3. 청각 심리학 기반 인코딩
4. 시뮬레이션 및 검증
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: CI 자극 전략 조사

| 기술 | 설명 | 웹서치 키워드 |
|------|------|--------------|
| **Pulse Rate** | 자극 펄스 속도 | "cochlear implant pulse rate pitch perception" |
| **Current Steering** | 가상 채널 생성 | "current steering cochlear implant virtual channels" |
| **Temporal Modulation** | 시간적 변조 | "temporal modulation cochlear implant" |
| **F0 Modulation** | 기본주파수 변조 | "F0 modulation cochlear implant music" |
| **Fine Structure** | 미세구조 전달 | "fine structure processing MED-EL" |

### 2단계: 청각 심리학 조사

| 주제 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **Pitch Perception** | CI 사용자 피치 인식 | "cochlear implant pitch perception mechanisms" |
| **Place vs Rate** | 위치 vs 속도 코딩 | "place coding rate coding cochlear implant" |
| **Temporal Pitch** | 시간적 피치 인식 | "temporal pitch perception electric hearing" |
| **Bimodal** | EAS 연구 | "electroacoustic stimulation pitch" |

### 3단계: 기존 연구 조사

| 연구 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **FSP (MED-EL)** | Fine Structure Processing | "MED-EL FSP coding strategy" |
| **MP3000** | 음악용 맵 | "cochlear implant MP3000 music program" |
| **Current Focusing** | 전류 집중 | "current focusing cochlear implant" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-3.md`에 다음을 정리:

```markdown
# Phase 3 사전 조사 결과

## 1. CI에서 피치를 인식하는 메커니즘

### Place Coding (위치 코딩)
- 원리: [조사 내용]
- 한계: [조사 내용]

### Rate/Temporal Coding (속도/시간 코딩)
- 원리: [조사 내용]
- 한계: [조사 내용]
- 최대 주파수: ~300 Hz (청신경 한계)

## 2. 기존 인핸스먼트 전략

### Fine Structure Processing (FSP)
- 방식: [조사 내용]
- 장점: [조사 내용]
- 한계: [조사 내용]

### F0 Modulation
- 방식: [조사 내용]
- 연구 결과: [조사 내용]

### Current Steering
- 방식: [조사 내용]
- 가상 채널 수: [조사 내용]

## 3. 옥타브 인코딩 가능성

### 전략 1: Temporal Modulation
- [분석]

### 전략 2: Multi-channel Patterns
- [분석]

### 전략 3: Harmonic Enhancement
- [분석]

## 4. 결론

### 권장 인코딩 전략
- [선택 및 이유]
```

---

## 🏗️ 인핸스먼트 프로토콜 설계

### 1. 옥타브 인코딩 전략

```
┌─────────────────────────────────────────────────────────────┐
│                 Octave Enhancement Protocol                  │
│                                                             │
│   입력:                                                      │
│   ├── 원본 오디오 신호                                        │
│   └── Phase 2 옥타브 정보 (F0, octave, confidence)           │
│                                                             │
│   출력:                                                      │
│   └── 인핸스된 CI 자극 패턴                                   │
└─────────────────────────────────────────────────────────────┘

전략 1: Temporal Modulation
═══════════════════════════
원본:    ────────────────
변조:    ∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿
         ↑
         옥타브 정보가 변조 주파수에 인코딩

전략 2: Octave-specific Channel Activation
═══════════════════════════════════════════
옥타브 3: 채널 1-5 강조
옥타브 4: 채널 6-10 강조
옥타브 5: 채널 11-15 강조
→ 옥타브별 고유 활성화 패턴

전략 3: Harmonic Enhancement
════════════════════════════
F0 = 440 Hz (A4)
→ 440, 880, 1320 Hz 채널에 동기화된 펄스
→ 하모닉 관계 명확하게 전달
```

### 2. 인코딩 데이터 구조

```json
{
  "$schema": "https://wia.live/ci/enhancement/v1/schema.json",
  "version": "1.0.0",
  "timestamp": "2025-12-14T12:00:00.000Z",
  "source": {
    "f0": 440.0,
    "octave": 4,
    "note": "A",
    "confidence": 0.95
  },
  "enhancement": {
    "strategy": "temporal_modulation",
    "parameters": {
      "modulationFrequency": 110.0,
      "modulationDepth": 0.3,
      "modulationPhase": 0.0
    }
  },
  "electrodePattern": {
    "primary": [
      {"electrode": 7, "amplitude": 1.0, "phase": 0.0},
      {"electrode": 12, "amplitude": 0.7, "phase": 0.0}
    ],
    "octaveMarker": [
      {"electrode": 5, "amplitude": 0.3, "pulseRate": 110}
    ],
    "harmonicReinforcement": [
      {"electrode": 12, "amplitude": 0.5, "sync": true},
      {"electrode": 15, "amplitude": 0.3, "sync": true}
    ]
  },
  "timing": {
    "frameStart": 0,
    "frameDuration": 20,
    "pulseSequence": [
      {"time": 0, "electrodes": [7, 12]},
      {"time": 9, "electrodes": [5]},
      {"time": 18, "electrodes": [7, 12]}
    ]
  }
}
```

### 3. Temporal Modulation 알고리즘

```typescript
interface TemporalModulationConfig {
  baseModulationDepth: number;      // 0.0-1.0, 기본 0.3
  octaveModulationMap: Map<number, number>;  // 옥타브별 변조 주파수
  adaptiveDepth: boolean;           // 신뢰도 기반 깊이 조절
}

class TemporalModulationEncoder {
  private config: TemporalModulationConfig;

  // 옥타브별 변조 주파수 매핑
  // 원리: 낮은 옥타브 = 낮은 변조 주파수
  private readonly octaveToModFreq: Map<number, number> = new Map([
    [2, 55],    // C2-B2: 55Hz 변조
    [3, 110],   // C3-B3: 110Hz 변조
    [4, 110],   // C4-B4: 110Hz 변조 (A4 = 440Hz의 1/4)
    [5, 220],   // C5-B5: 220Hz 변조
    [6, 220],   // C6-B6: 220Hz 변조
  ]);

  /**
   * 옥타브 정보를 시간적 변조 신호로 인코딩
   */
  encodeOctave(
    originalSignal: Float32Array,
    octaveInfo: OctaveResult
  ): EnhancedSignal {

    const modFreq = this.octaveToModFreq.get(octaveInfo.octave) || 110;
    const modDepth = this.config.baseModulationDepth * octaveInfo.confidence;

    // 변조 신호 생성
    const modulator = this.generateModulator(
      originalSignal.length,
      modFreq,
      modDepth
    );

    // 원본 신호에 변조 적용
    const enhancedSignal = new Float32Array(originalSignal.length);
    for (let i = 0; i < originalSignal.length; i++) {
      enhancedSignal[i] = originalSignal[i] * (1 + modulator[i]);
    }

    return {
      signal: enhancedSignal,
      metadata: {
        strategy: 'temporal_modulation',
        modulationFrequency: modFreq,
        modulationDepth: modDepth,
        octave: octaveInfo.octave
      }
    };
  }

  private generateModulator(
    length: number,
    frequency: number,
    depth: number
  ): Float32Array {
    const modulator = new Float32Array(length);
    const sampleRate = 16000;  // 가정

    for (let i = 0; i < length; i++) {
      const t = i / sampleRate;
      modulator[i] = depth * Math.sin(2 * Math.PI * frequency * t);
    }

    return modulator;
  }
}
```

### 4. Electrode Pattern 알고리즘

```typescript
interface ElectrodePatternConfig {
  numElectrodes: number;            // 22
  frequencyMap: number[];           // 각 전극의 중심 주파수
  octaveChannelGroups: Map<number, number[]>;
}

class ElectrodePatternEncoder {
  /**
   * 옥타브 정보 기반 전극 패턴 생성
   */
  generatePattern(
    originalPattern: ElectrodeActivation[],
    octaveInfo: OctaveResult
  ): EnhancedElectrodePattern {

    // 1. 원본 패턴 유지
    const enhanced = [...originalPattern];

    // 2. 옥타브 마커 추가
    const octaveMarkerElectrode = this.getOctaveMarkerElectrode(octaveInfo.octave);
    enhanced.push({
      electrode: octaveMarkerElectrode,
      amplitude: 0.2,
      pulseRate: this.getOctaveModFreq(octaveInfo.octave)
    });

    // 3. 하모닉 강화
    if (octaveInfo.confidence > 0.8) {
      const harmonicElectrodes = this.getHarmonicElectrodes(octaveInfo.f0);
      for (const he of harmonicElectrodes) {
        const existing = enhanced.find(e => e.electrode === he.electrode);
        if (existing) {
          existing.amplitude *= 1.2;  // 20% 강화
        }
      }
    }

    return {
      electrodes: enhanced,
      octaveInfo: octaveInfo,
      enhancementType: 'octave_pattern'
    };
  }

  /**
   * 옥타브별 고유 마커 전극
   * 저주파 = 근위부(apex), 고주파 = 원위부(base)
   */
  private getOctaveMarkerElectrode(octave: number): number {
    // 옥타브가 높을수록 원위부(base) 전극
    const markerMap: Map<number, number> = new Map([
      [2, 2],   // 옥타브 2: 전극 2 (apex 쪽)
      [3, 5],
      [4, 8],
      [5, 12],
      [6, 16],
      [7, 20],  // 옥타브 7: 전극 20 (base 쪽)
    ]);
    return markerMap.get(octave) || 8;
  }
}
```

### 5. 시뮬레이션 구조

```typescript
class CISimulator {
  /**
   * CI 청취 시뮬레이션
   *
   * 인핸스먼트 적용 전후 비교를 위한 vocoder 시뮬레이션
   */
  simulate(
    originalAudio: Float32Array,
    enhancedPattern: EnhancedElectrodePattern
  ): SimulationResult {

    // 1. 원본 CI 시뮬레이션 (옥타브 손실)
    const baseline = this.vocoderSimulate(originalAudio, {
      octaveEnhancement: false
    });

    // 2. 인핸스된 CI 시뮬레이션
    const enhanced = this.vocoderSimulate(originalAudio, {
      octaveEnhancement: true,
      enhancementPattern: enhancedPattern
    });

    // 3. 비교 분석
    return {
      baseline: baseline,
      enhanced: enhanced,
      improvement: {
        pitchClarity: this.measurePitchClarity(baseline, enhanced),
        octaveAccuracy: this.measureOctaveAccuracy(baseline, enhanced),
        subjectiveRating: null  // 실제 사용자 테스트 필요
      }
    };
  }

  /**
   * Vocoder 기반 CI 시뮬레이션
   *
   * CI 사용자의 청취 경험을 정상 청력자가 체험할 수 있게 함
   */
  private vocoderSimulate(
    audio: Float32Array,
    options: SimulationOptions
  ): Float32Array {
    // 22채널 vocoder 구현
    // 각 채널: 밴드패스 필터 → 엔벨로프 추출 → 노이즈 캐리어
    // ...
  }
}
```

---

## 📁 산출물 목록

Phase 3 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-3.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-3-ENHANCEMENT-PROTOCOL.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
3. 옥타브 인코딩 원리 (Octave Encoding Principles)
4. Temporal Modulation 전략 (Strategy 1)
5. Electrode Pattern 전략 (Strategy 2)
6. Harmonic Enhancement 전략 (Strategy 3)
7. 인코딩 데이터 형식 (Encoding Data Format)
8. 시뮬레이션 방법론 (Simulation Methodology)
9. 평가 메트릭 (Evaluation Metrics)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. TypeScript 구현
```
/api/typescript/src/
├── enhancement/
│   ├── index.ts
│   ├── types.ts
│   ├── TemporalModulationEncoder.ts
│   ├── ElectrodePatternEncoder.ts
│   ├── HarmonicEnhancer.ts
│   ├── EnhancementOrchestrator.ts
│   └── CISimulator.ts
└── ...
```

### 4. Python 구현
```
/api/python/wia_ci/
├── enhancement/
│   ├── __init__.py
│   ├── types.py
│   ├── temporal_modulation_encoder.py
│   ├── electrode_pattern_encoder.py
│   ├── harmonic_enhancer.py
│   ├── enhancement_orchestrator.py
│   └── ci_simulator.py
└── ...
```

### 5. 시뮬레이션 도구
```
/examples/simulation/
├── vocoder-simulation.ts           # Vocoder 시뮬레이션
├── before-after-comparison.ts      # 전후 비교
├── generate-demo-audio.ts          # 데모 오디오 생성
└── audio-samples/
    ├── original/                   # 원본 오디오
    ├── baseline-ci/                # 기본 CI 시뮬레이션
    └── enhanced-ci/                # 인핸스된 CI 시뮬레이션
```

---

## ✅ 완료 체크리스트

Phase 3 완료 전 확인:

```
□ 웹서치로 CI 자극 전략 조사 완료
□ 웹서치로 청각 심리학 조사 완료
□ /spec/RESEARCH-PHASE-3.md 작성 완료
□ /spec/PHASE-3-ENHANCEMENT-PROTOCOL.md 작성 완료
□ 인코딩 데이터 형식 정의 완료
□ Temporal Modulation Encoder 구현 완료
□ Electrode Pattern Encoder 구현 완료
□ CI Simulator (vocoder) 구현 완료
□ 전후 비교 데모 오디오 생성 완료
□ 시뮬레이션에서 피치 명확도 향상 확인
□ README 업데이트 (Phase 3 완료 표시)
```

---

## 🎯 평가 메트릭

| 메트릭 | 측정 방법 | 목표 |
|-------|----------|------|
| **Pitch Clarity** | 스펙트럼 선명도 | > 20% 향상 |
| **Octave Accuracy** | 옥타브 인식 정확도 | > 30% 향상 |
| **Speech Intelligibility** | 기존 어음 명료도 유지 | 저하 없음 |
| **Latency** | 처리 지연 | < 10ms 추가 |

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 청각 심리학 원리 기반 설계
✅ 기존 CI 자극 전략과 호환되도록
✅ 어음 명료도 저하 방지
✅ 시뮬레이션으로 효과 검증
✅ 다양한 음원에서 테스트
```

### DON'T (하지 말 것)

```
❌ 어음 명료도 저하시키는 변조
❌ CI 하드웨어 한계 무시
❌ 과도한 자극 (청신경 피로)
❌ 검증 없는 임상 적용 주장
```

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **웹서치로 CI 피치 인코딩 연구 조사**

```
검색 키워드: "cochlear implant temporal modulation pitch enhancement research"
```

화이팅! 🤟

---

<div align="center">

**Phase 3 of 4**

CI Signal Enhancement Protocol

**"옥타브 정보를 전달하는 방법"**

</div>
