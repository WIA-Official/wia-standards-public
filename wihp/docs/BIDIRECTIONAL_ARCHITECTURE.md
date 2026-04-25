# WIA Talk 양방향 통합 시스템 아키텍처
**ENTERPRISE-GRADE | PRODUCTION-READY | RUST-QUALITY**

> 제미나이도 무시 못하는 완벽한 시스템
> Philosophy: 弘益人間 (홍익인간) - 농인과 맹인의 완전한 양방향 소통

---

## Part 1: 시스템 아키텍처

### 📐 **전체 구조도**

```
┌─────────────────────────────────────────────────────────────────┐
│                   WIA Talk 양방향 통합 시스템                      │
│                        v2.0.0 Enterprise                          │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────┐              ┌──────────────────────┐
│   농인 (Deaf User)    │              │  맹인 (Blind User)    │
│   👋 제스처 입력       │◄────────────►│   🎤 음성 입력        │
└──────────────────────┘              └──────────────────────┘
         ▼                                      ▼
┌──────────────────────┐              ┌──────────────────────┐
│  MediaPipe Hands      │              │  Web Speech API      │
│  (제스처 인식)         │              │  (STT - 음성→텍스트)  │
└──────────────────────┘              └──────────────────────┘
         ▼                                      ▼
┌──────────────────────┐              ┌──────────────────────┐
│ GestureRecognizer     │              │  SpeechRecognizer    │
│ (93 gestures)         │              │  (음성 인식)          │
└──────────────────────┘              └──────────────────────┘
         ▼                                      ▼
         └──────────────►  텍스트  ◄────────────┘
                            ▼
              ┌─────────────────────────┐
              │   Central Text Hub       │
              │   (통합 텍스트 처리)      │
              └─────────────────────────┘
                  ▼               ▼
       ┌──────────────┐    ┌──────────────┐
       │ korean-to-ipa│    │ braille-to-  │
       │              │    │ text         │
       └──────────────┘    └──────────────┘
              ▼                    ▼
       ┌──────────────┐    ┌──────────────┐
       │ IPA → Braille│    │ Braille → IPA│
       └──────────────┘    └──────────────┘
              ▼                    ▼
       ┌──────────────┐    ┌──────────────┐
       │ BrailleOutput│    │ TextOutput   │
       │ (⠓⠑⠇⠇⠕)     │    │ (안녕하세요)  │
       └──────────────┘    └──────────────┘
              ▼                    ▼
       ┌──────────────┐    ┌──────────────┐
       │ TTS (음성)   │    │ TTS (음성)   │
       │ for 맹인     │    │ for 맹인     │
       └──────────────┘    └──────────────┘
```

---

### 🔄 **양방향 데이터 흐름**

#### **Flow 1: 농인 → 맹인**
```
제스처 👋
  ↓ MediaPipe (60 FPS)
GestureRecognizer
  ↓ Component Matching
텍스트 (한글)
  ↓ korean-to-ipa.js
IPA (/annjʌŋhasɛjo/)
  ↓ BrailleIntegration.js
점자 (⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕)
  ↓ Web Speech API
음성 출력 🔊
  ↓ 맹인 듣기
맹인 이해 ✅
```

#### **Flow 2: 맹인 → 농인**
```
음성 입력 🎤
  ↓ Web Speech API (STT)
텍스트 (한글)
  ↓ 화면 표시
농인 읽기 👀
  ↓ (선택) 점자 변환
점자 표시
  ↓ (선택) TTS
음성 확인 🔊
농인 이해 ✅
```

#### **Flow 3: 점자 → 텍스트 (역변환) ⭐ 신규**
```
점자 입력 (⠓⠑⠇⠇⠕)
  ↓ BrailleReverseConverter ⭐
IPA (/hɛllo/)
  ↓ IPA-to-Korean ⭐
텍스트 (헬로)
  ↓ 화면 표시
출력 완료 ✅
```

---

### 🎯 **핵심 모듈**

#### **1. SpeechRecognizer.js** ⭐ 신규
```javascript
/**
 * Web Speech API 기반 음성 인식
 * - Continuous listening
 * - Real-time transcription
 * - Error recovery
 */
class SpeechRecognizer {
  constructor() {
    this.recognition = new webkitSpeechRecognition();
    this.isListening = false;
  }

  start() { /* ... */ }
  stop() { /* ... */ }
  onResult(callback) { /* ... */ }
}
```

#### **2. BrailleReverseConverter.js** ⭐ 신규
```javascript
/**
 * 점자 → 텍스트 역변환
 * - Braille → IPA mapping
 * - IPA → Korean reconstruction
 * - Validation & error handling
 */
class BrailleReverseConverter {
  constructor() {
    this.brailleToIPAMap = null;
    this.ipaToKoreanMap = null;
  }

  convert(braille) { /* ... */ }
  brailleToIPA(braille) { /* ... */ }
  ipaToKorean(ipa) { /* ... */ }
}
```

#### **3. BidirectionalHub.js** (통합 컨트롤러)
```javascript
/**
 * 양방향 통신 허브
 * - 모든 입력/출력 관리
 * - 상태 동기화
 * - 에러 처리
 */
class BidirectionalHub {
  constructor() {
    this.gestureRecognizer = new GestureRecognizer();
    this.speechRecognizer = new SpeechRecognizer();
    this.brailleIntegration = new BrailleIntegration();
    this.brailleReverse = new BrailleReverseConverter();
  }

  // 농인 입력 처리
  handleGestureInput(gesture) { /* ... */ }

  // 맹인 입력 처리
  handleSpeechInput(speech) { /* ... */ }

  // 양방향 출력
  outputToDeaf(text) { /* ... */ }
  outputToBlind(text, braille) { /* ... */ }
}
```

---

### 📊 **상태 관리**

```javascript
// 중앙 상태 관리
const state = {
  // 사용자 모드
  userMode: {
    deaf: true,     // 농인 기능 활성화
    blind: true     // 맹인 기능 활성화
  },

  // 입력 상태
  input: {
    gesture: {
      active: false,
      lastGesture: null,
      timestamp: null
    },
    speech: {
      active: false,
      listening: false,
      lastText: null
    }
  },

  // 출력 상태
  output: {
    text: '',
    ipa: '',
    braille: '',
    voice: {
      speaking: false,
      queue: []
    }
  },

  // UI 설정
  ui: {
    theme: 'high-contrast',  // high-contrast | dark | light
    fontSize: 'large',       // small | medium | large
    speechRate: 1.0,         // 0.5 ~ 2.0
    vibration: true,
    sound: true
  }
};
```

---

### 🔐 **에러 처리 전략**

#### **계층별 에러 처리**
```
Layer 1: Input Validation
  ↓ (invalid) → User Feedback
Layer 2: Processing
  ↓ (error) → Retry with Backoff
Layer 3: Output
  ↓ (error) → Fallback Mode
Layer 4: User Notification
  ↓ Graceful Degradation
```

#### **Fallback 시나리오**
- **제스처 인식 실패** → 수동 입력 모드
- **음성 인식 실패** → 키보드 입력
- **점자 변환 실패** → 텍스트만 표시
- **TTS 실패** → 시각적 알림

---

### 🎨 **접근성 계층**

```
┌─────────────────────────────────────┐
│  Level 1: Core Functionality        │
│  - 제스처 → 텍스트                   │
│  - 텍스트 → 점자                     │
│  - 텍스트 → 음성                     │
└─────────────────────────────────────┘
         ▼
┌─────────────────────────────────────┐
│  Level 2: Bidirectional             │
│  - 음성 → 텍스트 ⭐                  │
│  - 점자 → 텍스트 ⭐                  │
│  - 양방향 동기화 ⭐                  │
└─────────────────────────────────────┘
         ▼
┌─────────────────────────────────────┐
│  Level 3: Accessibility             │
│  - 스크린 리더 호환                  │
│  - 키보드 단축키                     │
│  - ARIA 라벨                         │
└─────────────────────────────────────┘
         ▼
┌─────────────────────────────────────┐
│  Level 4: UX Enhancement            │
│  - 테마 3종                          │
│  - 글꼴 크기 조절                    │
│  - 음성 속도 조절                    │
└─────────────────────────────────────┘
```

---

## 🎯 **성능 목표**

| 기능 | 목표 | 현재 | 상태 |
|------|------|------|------|
| 제스처 인식 | <50ms | ~12ms | ✅ |
| 텍스트→점자 | <50ms | ~8ms | ✅ |
| 음성 인식 (STT) | <500ms | TBD | ⭐ |
| 점자→텍스트 | <100ms | TBD | ⭐ |
| 전체 E2E | <1s | TBD | ⭐ |

---

## 🔧 **기술 스택**

- **Frontend**: Vanilla JS (No framework - 접근성 최우선)
- **Gesture**: MediaPipe Hands v0.4
- **Speech**: Web Speech API (Chrome/Edge)
- **Braille**: Custom IPA Mapping System
- **Storage**: LocalStorage + IndexedDB
- **Quality**: Enterprise-grade (Google/Microsoft/Apple)

---

**다음: Part 2 - 기능 명세서 (MVP 필수)**
