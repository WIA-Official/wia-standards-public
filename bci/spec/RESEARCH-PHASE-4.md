# Phase 4 사전 조사 결과

**WIA BCI Ecosystem Integration Research**

**Date**: 2025-01
**Phase**: 4 of 4

---

## 1. 개요

Phase 4는 BCI 신호 처리 결과를 WIA 생태계의 다양한 출력 시스템과 연동하는 것을 목표로 합니다.

### 통합 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│                     BCI 사용자                               │
│              (ALS, 사지마비, 의식장애 환자 등)               │
└─────────────────────────────────────────────────────────────┘
                              │
                         [뇌 신호 입력]
                    EEG/ECoG/Intracortical
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1-3: BCI Signal → Protocol                │
└─────────────────────────────────────────────────────────────┘
                              │
                         [의도 해석]
                    텍스트/명령/제어신호
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: WIA Ecosystem Integration              │
│                     OutputManager                            │
├─────────────┬─────────────┬─────────────┬───────────────────┤
│ TTSAdapter  │ SignAdapter │ BrailleAdapter │ NeuroFeedback  │
└──────┬──────┴──────┬──────┴───────┬───────┴────────┬────────┘
       ▼             ▼              ▼                ▼
    음성 출력    수어 아바타    점자 디스플레이    뇌파 시각화
```

---

## 2. BCI-Speech 통합 기술 현황

### 2.1 최신 연구 동향 (2024-2025)

#### UC Davis / BrainGate (2024.08)
- ALS 환자 대상 BCI → 음성 변환 시스템 개발
- **정확도 97%** 달성 (가장 정확한 시스템)
- 뇌 신호를 텍스트로 디코딩 후 환자 원래 목소리로 TTS 출력

#### NIH / UCSF (2025.03)
- 실시간 Brain-to-Voice 신경보철 개발
- **80ms 단위** 지연 없는 음성 스트리밍
- 분당 약 130단어 속도 달성

#### Stanford Inner Speech (2025)
- "내면 언어" 기반 BCI 연구
- 물리적 발화 시도 없이 상상만으로 텍스트 변환
- 수어 아바타 출력까지 연동 가능성 제시

#### Paradromics FDA 승인 (2025.11)
- 음성 복원용 BCI 임상 시험 FDA 승인
- 마비 환자 대상 텍스트/합성음성 출력 목표
- 시장 규모: Morgan Stanley 기준 **$400B** 전망

### 2.2 기술 아키텍처

#### 전통적 방식
```
뇌 신호 → LDA 분류 → 텍스트 디코딩 → TTS 합성
```

#### 최신 방식 (딥러닝)
```
뇌 신호 → CNN 직접 디코딩 → Mel-Spectrogram → Neural Vocoder
```

### 2.3 참고 자료

- [Brain-Computer Interface Breakthroughs in Speech Restoration](https://blog.unrealspeech.com/brain-computer-interface-breakthroughs-in-speech-restoration/)
- [NIH: BCI Restores Natural Speech](https://www.nih.gov/news-events/nih-research-matters/brain-computer-interface-restores-natural-speech-after-paralysis)
- [Brown University: BCI Speak Again](https://www.brown.edu/news/2024-08-14/bci-speak-again)
- [Paradromics FDA Approval](https://www.statnews.com/2025/11/20/fda-approves-paradromics-bci-trial-for-speech-restoration/)

---

## 3. TTS (Text-to-Speech) 서비스 비교

### 3.1 Web Speech API

| 항목 | 내용 |
|------|------|
| **장점** | 무료, 브라우저 내장, 설치 불필요 |
| **단점** | 음성 품질 제한, 브라우저 의존 |
| **BCI 적용** | 경량 응용, 프로토타입에 적합 |
| **최대 길이** | 32,767 characters/utterance |

#### 주요 인터페이스
```typescript
// SpeechSynthesis 컨트롤러
const synth = window.speechSynthesis;

// SpeechSynthesisUtterance 설정
const utterance = new SpeechSynthesisUtterance(text);
utterance.rate = 1.0;    // 0.1 ~ 10
utterance.pitch = 1.0;   // 0 ~ 2
utterance.volume = 1.0;  // 0 ~ 1
utterance.voice = synth.getVoices()[0];
```

### 3.2 Cloud TTS 서비스

| 서비스 | 장점 | 단점 | 가격 |
|--------|------|------|------|
| **Google Cloud TTS** | 고품질 WaveNet, 다국어 | 비용, 네트워크 의존 | $4/1M chars |
| **Amazon Polly** | Neural TTS, SSML 지원 | AWS 종속 | $4/1M chars |
| **Azure Speech** | 실시간 스트리밍, 감정 | 복잡한 설정 | $4/1M chars |

### 3.3 권장 구현 전략

```
1순위: Web Speech API (기본, 무료)
2순위: Cloud TTS (고품질 필요 시)
3순위: Custom Voice (환자 원래 목소리 복원)
```

### 3.4 참고 자료

- [MDN: Web Speech API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API/Using_the_Web_Speech_API)
- [DigitalOcean: TTS App Tutorial](https://www.digitalocean.com/community/tutorials/how-to-build-a-text-to-speech-app-with-web-speech-api)
- [EasySpeech Library](https://github.com/leaonline/easy-speech)

---

## 4. 수어 아바타 기술

### 4.1 현황

#### 3D 아바타 방식
- 텍스트 → 수어 코드 → 3D 애니메이션
- MediaPipe Holistic으로 포즈 추정
- WebGL/Three.js로 렌더링

#### ISP (International Sign Protocol) 기반
- WIA Talk 표준 활용
- 텍스트 → ISP 코드 변환
- ISP 코드 → 제스처 시퀀스

### 4.2 BCI 연동 시나리오

```
BCI 의도 해석 → 텍스트 생성 → ISP 변환 → 수어 아바타 출력
```

### 4.3 구현 방향

```typescript
interface ISignLanguageAdapter {
  // 텍스트 → ISP 코드
  textToISP(text: string): Promise<ISPCode[]>;

  // ISP 코드 → 아바타 애니메이션
  playGesture(code: ISPCode): Promise<void>;

  // 시퀀스 재생
  playSequence(codes: ISPCode[]): Promise<void>;
}
```

---

## 5. 점자 출력 기술

### 5.1 Refreshable Braille Display

| 항목 | 설명 |
|------|------|
| **작동 원리** | 전자기계적 핀을 통해 점자 문자 표시 |
| **셀 수** | 20셀 (휴대용) ~ 80셀 (데스크탑) |
| **가격대** | $500 ~ $10,000+ |

### 5.2 운영체제 API

| 플랫폼 | API/서비스 |
|--------|-----------|
| **Windows** | UI Automation (UIA), Narrator |
| **macOS/iOS** | VoiceOver, Apple Accessibility API |
| **Android** | BrailleBack, AT-SPI |
| **Linux** | BRLTTY, AT-SPI |

### 5.3 프로그래밍 접근

```typescript
interface IBrailleAdapter {
  // 텍스트 → IPA (발음 기호)
  textToIPA(text: string): Promise<string>;

  // IPA → 점자
  ipaToBraille(ipa: string): Promise<BrailleOutput>;

  // 점자 디스플레이 전송
  sendToDisplay(braille: BrailleOutput): Promise<void>;
}
```

### 5.4 오픈소스 라이브러리

- **liblouis**: 점자 변환 라이브러리
- **BRLTTY**: Linux 점자 디스플레이 드라이버
- **BrailleBack**: Android 점자 서비스

### 5.5 참고 자료

- [Apple: Braille Displays Documentation](https://developer.apple.com/documentation/accessibility/braille-displays)
- [Google BrailleBack](https://github.com/google/brailleback)
- [Wikipedia: Refreshable Braille Display](https://en.wikipedia.org/wiki/Refreshable_braille_display)

---

## 6. 뉴로피드백 시각화

### 6.1 개요

BCI 시스템의 고유 기능으로, 뇌 활동을 실시간으로 시각화하여 사용자에게 피드백을 제공합니다.

### 6.2 시각화 방식

| 방식 | 설명 | 용도 |
|------|------|------|
| **토포그래피** | 두피 위 활성화 맵 | 채널별 활성도 |
| **밴드파워 바** | 주파수 대역별 막대 | 알파/베타/감마 상태 |
| **3D 뇌 모델** | 실시간 뇌 활성화 | 연구용 |
| **게이지/미터** | 단순 지표 표시 | 집중도, 이완도 |

### 6.3 소프트웨어 도구

- **OpenBCI GUI**: 오픈소스 시각화
- **Turbo-Satori**: fNIRS 실시간 분석
- **Lab Streaming Layer (LSL)**: 데이터 스트리밍

### 6.4 BCI 피드백 통합

```typescript
interface INeurofeedbackAdapter {
  // 실시간 밴드파워 시각화
  updateBandPowers(powers: BandPowers): void;

  // 토포그래피 맵 업데이트
  updateTopography(channels: ChannelData[]): void;

  // 분류 결과 표시
  showClassification(result: ClassificationResult): void;

  // 커서 제어 피드백
  updateCursor(position: Position): void;
}
```

### 6.5 참고 자료

- [OpenBCI: Open Source Imaging](https://www.opensourceimaging.org/project/openbci/)
- [NIRx: fNIRS BCI Neurofeedback](https://nirx.net/fnirs-bci-neurofeedback)
- [PMC: BCI-Neurofeedback System](https://pmc.ncbi.nlm.nih.gov/articles/PMC11021665/)

---

## 7. 접근성 기술 통합 동향

### 7.1 AI 통합 보조기기

| 기술 | 설명 | 현황 |
|------|------|------|
| **스마트 의수** | AI 의도 해석, 실시간 조정 | 상용화 진행 |
| **외골격** | 뇌 신호 → 물리적 움직임 | 임상 시험 중 |
| **ThoughtSpeak** | BCI → 음성/텍스트 변환 | CES 2024 발표 |

### 7.2 Wimagine Brain Implant

- 프랑스 CEA 개발
- 마비 환자 보행 복원
- 뇌 신호 → 디지털 브릿지 → 외골격

### 7.3 참고 자료

- [CES 2024: Inclusive Tech Innovations](https://www.accessibility.com/blog/ces-2024-unveils-a-range-of-inclusive-tech-innovations-promising-a-new-era-in-accessibility)
- [AI and Assistive Technologies in Healthcare](https://pmc.ncbi.nlm.nih.gov/articles/PMC11898476/)
- [Machine Learning Neural Interfaces for Prosthetics](https://arxiv.org/html/2505.02516v1)

---

## 8. 결론 및 권장 사항

### 8.1 권장 TTS 방식

| 용도 | 권장 |
|------|------|
| **프로토타입/경량** | Web Speech API |
| **고품질 필요** | Google Cloud TTS |
| **오프라인 환경** | eSpeak/pico2wave |
| **환자 목소리 복원** | Custom Voice Model |

### 8.2 ISP 연동 설계

```
1. Phase 2 API에서 텍스트 출력 캡처
2. 텍스트 → ISP 코드 변환 (WIA Talk 연동)
3. ISP 코드 → 3D 아바타 애니메이션
4. WebGL 렌더링 출력
```

### 8.3 점자 연동 설계

```
1. Phase 2 API에서 텍스트 출력 캡처
2. 텍스트 → IPA 변환 (WIA Braille)
3. IPA → Grade 2 Braille 변환
4. 시리얼/Bluetooth로 디스플레이 전송
```

### 8.4 뉴로피드백 설계

```
1. Phase 3 Protocol에서 신호 스트림 수신
2. 실시간 밴드파워 계산
3. Canvas/WebGL로 시각화 렌더링
4. 분류 결과 오버레이 표시
```

### 8.5 통합 아키텍처

```typescript
class OutputManager {
  // 모든 출력 어댑터 관리
  private adapters: Map<OutputType, IOutputAdapter>;

  // 멀티모달 출력 (동시 출력)
  async broadcast(content: OutputContent): Promise<void>;

  // 특정 어댑터로 출력
  async outputTo(type: OutputType, content: OutputContent): Promise<void>;

  // 사용자 선호도 기반 출력
  async outputPreferred(content: OutputContent): Promise<void>;
}
```

---

## 9. 다음 단계

1. **PHASE-4-INTEGRATION.md** 스펙 문서 작성
2. TypeScript 출력 모듈 구현
   - IOutputAdapter 인터페이스
   - TTSAdapter (Web Speech API)
   - SignLanguageAdapter (Mock)
   - BrailleAdapter (Mock)
   - NeurofeedbackAdapter
   - OutputManager
3. Python 출력 모듈 구현
4. 예제 코드 작성
5. README 업데이트

---

弘益人間 🤟

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group
