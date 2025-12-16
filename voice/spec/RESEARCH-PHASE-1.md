# WIA Voice-Sign Phase 1: Technology Research Report

## 1. Executive Summary

본 문서는 음성-수화 변환 기술의 현재 상태, 주요 기업, 기술 표준, 그리고 WIA 표준 개발을 위한 기술적 고려사항을 정리한 연구 보고서입니다.

**핵심 발견사항:**
- 실시간 음성→수화 변환은 AI/ML 발전으로 실용화 단계 진입
- 주요 표기 시스템: HamNoSys, SignWriting, SiGML
- 출력 방식: 3D 아바타, 2D 애니메이션, 비디오 합성
- 지역별 수화 차이(ASL, BSL, KSL 등)가 표준화의 핵심 과제

---

## 2. Market Players & Solutions

### 2.1 Major Companies

| Company | Product | Technology | Target Languages |
|---------|---------|------------|------------------|
| **Google** | SignGemma | LLM-based translation | ASL, BSL |
| **Signapse AI** | Real-time Translator | AI Avatar | BSL, ASL |
| **Hand Talk** | Hugo/Maya Avatars | 3D Animation | Libras (Brazilian) |
| **SiMAX** | Universal Translator | Avatar System | 50+ sign languages |
| **Kara Technologies** | Kotahi Platform | AI Translation | NZSL, Auslan |
| **Terp 360** | Video Relay | Human + AI Hybrid | ASL |

### 2.2 Research Institutions

- **Gallaudet University**: 수화 언어학 연구의 세계적 권위
- **Rochester Institute of Technology (RIT)**: ASL 데이터셋 및 인식 연구
- **University of Hamburg**: HamNoSys 표기 시스템 개발
- **KAIST (한국)**: 한국수화(KSL) 인식 연구

### 2.3 Open Source Projects

| Project | Description | License |
|---------|-------------|---------|
| MediaPipe | Google의 손/몸 추적 프레임워크 | Apache 2.0 |
| OpenPose | CMU의 자세 추정 라이브러리 | Custom (연구용) |
| SignLanguageNet | 수화 인식 신경망 | MIT |
| JASigning | HamNoSys→Avatar 렌더링 | LGPL |

---

## 3. Sign Language Notation Systems

### 3.1 HamNoSys (Hamburg Notation System)

**개요:**
- Hamburg 대학에서 개발한 음성학적 수화 표기 시스템
- 수화 동작을 구성 요소로 분해하여 표기
- 국제적으로 가장 널리 사용되는 학술 표기법

**구성 요소:**
```
1. 손 모양 (Handshape): 60+ 기본 형태
2. 손 방향 (Hand Orientation): 손바닥/손가락 방향
3. 위치 (Location): 신체 기준 공간 좌표
4. 동작 (Movement): 경로, 반복, 속도
5. 비수지 요소 (Non-manual): 표정, 입 모양, 시선
```

**예시:**
```
[ASL "HELLO"]
HamNoSys:
  handshape: flat-B
  orientation: palm-out
  location: forehead
  movement: wave-side-to-side
```

### 3.2 SignWriting

**개요:**
- Valerie Sutton이 개발한 시각적 표기 시스템
- 문자처럼 읽고 쓸 수 있는 그래픽 기호
- 80+ 국가에서 교육용으로 사용

**특징:**
- 직관적인 시각적 표현
- 컴퓨터 렌더링에 적합
- Unicode 지원 (U+1D800-1DAAF)

### 3.3 SiGML (Signing Gesture Markup Language)

**개요:**
- XML 기반 수화 기술 언어
- HamNoSys를 기계 판독 가능한 형식으로 변환
- 3D 아바타 애니메이션 생성에 최적화

**구조:**
```xml
<sigml>
  <hns_sign gloss="HELLO">
    <hamnosys_manual>
      <handconfig>
        <handshape shape="flat"/>
        <palmor ori="ul"/>
      </handconfig>
      <location_bodyarm location="forehead"/>
      <directedmotion direction="r" size="small"/>
    </hamnosys_manual>
  </hns_sign>
</sigml>
```

### 3.4 Comparison Matrix

| Feature | HamNoSys | SignWriting | SiGML |
|---------|----------|-------------|-------|
| 기계 판독성 | Medium | Low | High |
| 인간 가독성 | Low | High | Medium |
| 언어 독립성 | High | High | High |
| 3D 변환 용이성 | Medium | Low | High |
| 표준화 수준 | High | Medium | Medium |
| WIA 적합도 | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ |

---

## 4. Technical Architecture Patterns

### 4.1 Speech-to-Sign Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                    Speech-to-Sign Pipeline                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐  │
│  │  Audio   │───▶│  Speech  │───▶│   NLP    │───▶│   Sign   │  │
│  │  Input   │    │  Recog.  │    │ Process  │    │ Mapping  │  │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘  │
│       │               │               │               │         │
│       ▼               ▼               ▼               ▼         │
│   [WAV/PCM]      [Transcript]    [Semantics]     [SiGML/     │
│                                                  HamNoSys]     │
│                                                      │         │
│                                      ┌───────────────┘         │
│                                      ▼                          │
│                              ┌──────────────┐                   │
│                              │   Renderer   │                   │
│                              │  (3D Avatar) │                   │
│                              └──────────────┘                   │
│                                      │                          │
│                                      ▼                          │
│                              ┌──────────────┐                   │
│                              │    Output    │                   │
│                              │ (Video/Anim) │                   │
│                              └──────────────┘                   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 Core Components

#### 4.2.1 Speech Recognition Module
- **ASR Engine**: Whisper, DeepSpeech, Google Speech API
- **Language Detection**: 자동 언어 식별
- **Streaming**: 실시간 처리를 위한 청크 기반 인식

#### 4.2.2 NLP Processing Module
- **Tokenization**: 형태소 분석
- **Semantic Parsing**: 의미 구조 추출
- **Grammar Transformation**: 음성언어 → 수화 문법 변환
  - 수화는 SOV 어순이 일반적 (영어/한국어와 다름)
  - 시간 표현이 문장 앞에 위치

#### 4.2.3 Sign Mapping Module
- **Lexicon Database**: 단어 → 수화 사전
- **Gloss Generation**: 중간 표현 생성
- **Notation Encoding**: HamNoSys/SiGML 변환

#### 4.2.4 Rendering Module
- **Avatar System**: 3D 캐릭터 애니메이션
- **Blending**: 동작 간 자연스러운 전환
- **Facial Animation**: 비수지 요소 표현

### 4.3 Real-time Requirements

| Metric | Target | Acceptable |
|--------|--------|------------|
| ASR Latency | < 200ms | < 500ms |
| Translation Latency | < 300ms | < 800ms |
| Rendering Latency | < 100ms | < 200ms |
| **End-to-End** | **< 600ms** | **< 1500ms** |
| Frame Rate | 30 FPS | 24 FPS |

---

## 5. Sign Language Variations

### 5.1 Major Sign Languages

| Code | Name | Region | Users (M) | Linguistic Family |
|------|------|--------|-----------|-------------------|
| ASL | American Sign Language | USA, Canada | 0.5-2 | French SL family |
| BSL | British Sign Language | UK | 0.15 | BANZSL |
| LSF | Langue des Signes Française | France | 0.1 | French SL family |
| DGS | Deutsche Gebärdensprache | Germany | 0.2 | German SL family |
| JSL | 日本手話 | Japan | 0.3 | Isolate |
| KSL | 한국수화 | Korea | 0.3 | Isolate |
| CSL | 中国手语 | China | 20+ | Chinese SL |

### 5.2 Korean Sign Language (KSL) 특성

**문법적 특징:**
- 어순: 주어-목적어-서술어 (SOV)
- 시간 표현: 문장 시작 부분에 위치
- 지시 대명사: 공간 지시로 표현
- 부정: 머리 흔들기 + 부정 수화

**기술적 고려사항:**
- 한글 자모 지문자 시스템
- 한국 특유의 관용 표현
- 지역별 방언 차이 (서울/대구/부산)

### 5.3 Cross-Language Considerations

```
┌───────────────────────────────────────────────────────────┐
│              Language-Agnostic Layer                       │
├───────────────────────────────────────────────────────────┤
│                                                            │
│   ┌─────────┐     ┌─────────────┐     ┌─────────────┐    │
│   │  ASL    │     │  Universal  │     │    BSL      │    │
│   │ Module  │────▶│   Semantic  │◀────│   Module    │    │
│   └─────────┘     │    Layer    │     └─────────────┘    │
│                   └─────────────┘                         │
│        ▲                │                    ▲            │
│        │                ▼                    │            │
│   ┌─────────┐     ┌─────────────┐     ┌─────────────┐    │
│   │  KSL    │     │   Common    │     │    DGS      │    │
│   │ Module  │◀───▶│   Gloss DB  │◀───▶│   Module    │    │
│   └─────────┘     └─────────────┘     └─────────────┘    │
│                                                            │
└───────────────────────────────────────────────────────────┘
```

---

## 6. AI/ML Approaches

### 6.1 Speech Recognition Models

| Model | Type | Accuracy | Latency | License |
|-------|------|----------|---------|---------|
| Whisper | Transformer | 95%+ | 200ms | MIT |
| DeepSpeech | RNN | 90% | 150ms | MPL 2.0 |
| Wav2Vec 2.0 | Self-supervised | 94% | 180ms | MIT |
| Conformer | Hybrid | 96% | 250ms | Apache 2.0 |

### 6.2 Translation Models

**Sequence-to-Sequence:**
- Input: Text tokens
- Output: Sign gloss sequence
- Architecture: Transformer, LSTM

**Approaches:**
1. **Direct Translation**: Text → Sign Notation
2. **Pivot-based**: Text → Universal Gloss → Sign Notation
3. **Multi-task**: Joint ASR + Translation

### 6.3 Avatar Animation

**Skeletal Animation:**
- 손: 21 joints per hand (MediaPipe standard)
- 팔/상체: 33 body landmarks
- 얼굴: 468 facial landmarks

**Motion Synthesis:**
- **Data-driven**: 모션 캡처 데이터 기반
- **Procedural**: 규칙 기반 동작 생성
- **Hybrid**: 기본 동작 + 보간

---

## 7. Data Format Requirements

### 7.1 Input Formats

```yaml
Audio Input:
  formats: [WAV, PCM, FLAC, OGG]
  sample_rate: [16000, 44100, 48000]
  channels: [mono, stereo]
  bit_depth: [16, 24, 32]

Text Input:
  encodings: [UTF-8]
  formats: [plain, SSML]
  languages: [ISO 639-1 codes]
```

### 7.2 Intermediate Representation

```yaml
Sign Notation:
  primary: SiGML
  secondary: HamNoSys
  gloss:
    format: JSON
    schema: WIA-Gloss-v1

Semantic Structure:
  format: JSON-LD
  vocabulary: WIA-Sign-Vocab
```

### 7.3 Output Formats

```yaml
3D Animation:
  formats: [glTF, FBX, BVH]
  skeleton: WIA-Sign-Skeleton-v1

2D Video:
  codecs: [H.264, H.265, VP9, AV1]
  resolutions: [720p, 1080p, 4K]
  frame_rates: [24, 30, 60]

Streaming:
  protocols: [WebRTC, HLS, DASH]
  latency_target: < 1s
```

---

## 8. Accessibility Standards

### 8.1 Relevant Standards

| Standard | Scope | Relevance |
|----------|-------|-----------|
| WCAG 2.1 AAA | Web accessibility | Sign language for video |
| EN 301 549 | EU ICT accessibility | Mandatory for public sector |
| Section 508 | US federal accessibility | Government contracts |
| JIS X 8341 | Japanese accessibility | Asian market |
| 한국 웹접근성 지침 | Korean accessibility | Korean market |

### 8.2 WCAG Sign Language Requirements

**1.2.6 Sign Language (Prerecorded) - Level AAA:**
> Sign language interpretation is provided for all prerecorded audio content in synchronized media.

**Key Requirements:**
- 수화 통역 비디오 동기화
- 충분한 화면 크기 (최소 320x240)
- 배경 대비 명확한 수화자
- 일시정지/속도 조절 기능

---

## 9. WIA Standard Recommendations

### 9.1 Data Format Choices

| Component | Recommended | Rationale |
|-----------|-------------|-----------|
| Sign Notation | SiGML + HamNoSys | 기계 판독성 + 학술 표준 |
| Intermediate | JSON-LD | 확장성 + 의미론적 명확성 |
| Animation | glTF 2.0 | 웹 표준 + 경량 |
| Streaming | WebRTC | 저지연 + 브라우저 호환 |

### 9.2 Core Schema Entities

```
1. AudioInput
   - 음성 입력 메타데이터

2. TranscriptionResult
   - ASR 결과 및 타임스탬프

3. SignGloss
   - 수화 단어/구문 표현

4. SignNotation
   - HamNoSys/SiGML 인코딩

5. AvatarPose
   - 스켈레톤 데이터

6. RenderOutput
   - 최종 출력 정보
```

### 9.3 API Design Principles

1. **Language Agnostic**: 특정 수화에 종속되지 않는 설계
2. **Modular Pipeline**: 각 단계 독립적 교체 가능
3. **Real-time Ready**: 스트리밍 처리 기본 지원
4. **Extensible Notation**: 새로운 표기법 추가 용이

---

## 10. Challenges & Limitations

### 10.1 Technical Challenges

| Challenge | Severity | Mitigation |
|-----------|----------|------------|
| 실시간 지연 | High | 파이프라인 최적화, Edge 처리 |
| 문법 변환 정확도 | High | 대규모 병렬 말뭉치, Fine-tuning |
| 자연스러운 동작 | Medium | 모션 캡처 데이터, 보간 알고리즘 |
| 비수지 요소 | Medium | 얼굴 인식 + 규칙 기반 생성 |
| 방언/변이 | Low | 지역별 모듈화 |

### 10.2 Data Scarcity

**현재 상황:**
- 수화 병렬 말뭉치 부족 (ASL: ~10만 문장, KSL: ~1만 문장)
- 모션 캡처 데이터 수집 비용 높음
- 전문 수화 통역사 검증 필요

**해결 방안:**
- 합성 데이터 생성
- 크라우드소싱
- 전이 학습

### 10.3 Ethical Considerations

- **농인 커뮤니티 참여**: 당사자 의견 반영 필수
- **품질 보증**: 오역으로 인한 소통 장애 방지
- **문화적 존중**: 수화는 독립된 언어임을 인정

---

## 11. Conclusion

음성-수화 변환 기술은 AI/ML의 발전으로 실용화 단계에 진입했습니다. WIA 표준은 다음을 목표로 합니다:

1. **통합 데이터 형식**: SiGML 기반 + JSON-LD 확장
2. **모듈형 아키텍처**: 각 처리 단계 독립적 정의
3. **다국어 지원**: 언어 독립적 중간 표현
4. **실시간 최적화**: 저지연 파이프라인 표준

다음 단계로 PHASE-1-DATA-FORMAT.md에서 구체적인 데이터 형식을 정의하겠습니다.

---

## References

1. HamNoSys Documentation - University of Hamburg
2. SignWriting - Center for Sutton Movement Writing
3. SiGML Specification - ViSiCAST Project
4. WCAG 2.1 - W3C Web Accessibility Initiative
5. MediaPipe Hands - Google AI
6. Whisper - OpenAI
7. Google SignGemma - Google Research
8. Signapse AI Technical Documentation
