# WIA Talk 구현 가이드
**Part 6: 구현 순서 (단계별)**

> 제미나이도 따라할 수 있는 체계적인 구현 절차

---

## 🎯 **Phase 1: 신규 모듈 구현** (우선순위: 🔴 High)

### **Step 1.1: BrailleReverseConverter.js 구현**
**목표:** 점자 → 텍스트 역변환 기능

**작업 내용:**
```javascript
// 📁 src/braille/BrailleReverseConverter.js
class BrailleReverseConverter {
  constructor() {
    // 역 매핑 테이블
    this.brailleToIPAMap = new Map();
    this.ipaToKoreanMap = new Map();
  }

  async initialize() {
    // 기존 mapping.json 로드
    // 역 매핑 생성
  }

  convert(brailleText) {
    // 점자 → IPA → 한글
    return { text, ipa, confidence };
  }
}
```

**테스트:**
```javascript
const converter = new BrailleReverseConverter();
await converter.initialize();

const result = converter.convert('⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕');
console.assert(result.text === '안녕하세요');
```

**예상 시간:** 2-3시간

---

### **Step 1.2: SpeechRecognizer.js 구현**
**목표:** Web Speech API 기반 STT

**작업 내용:**
```javascript
// 📁 src/speech/SpeechRecognizer.js
class SpeechRecognizer {
  constructor() {
    this.recognition = new webkitSpeechRecognition();
    this.recognition.lang = 'ko-KR';
    this.recognition.continuous = true;
  }

  start() { /* ... */ }
  stop() { /* ... */ }

  onFinalResult(callback) {
    // 최종 결과 콜백
  }
}
```

**테스트:**
```javascript
const recognizer = new SpeechRecognizer();

recognizer.onFinalResult = (result) => {
  console.log('인식:', result.text);
};

await recognizer.start();
// 말하기: "안녕하세요"
// 출력: 인식: 안녕하세요
```

**예상 시간:** 1-2시간

---

### **Step 1.3: TextToSpeech.js 구현**
**목표:** TTS 유틸리티

**작업 내용:**
```javascript
// 📁 src/speech/TextToSpeech.js
class TextToSpeech {
  constructor() {
    this.synth = window.speechSynthesis;
  }

  speak(text, options = {}) {
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.rate = options.rate || 1.0;
    this.synth.speak(utterance);
  }
}
```

**예상 시간:** 30분

---

## 🎨 **Phase 2: 양방향 통합 앱 구현** (우선순위: 🔴 High)

### **Step 2.1: BidirectionalApp 클래스 구현**

**작업 순서:**

1. **기본 구조 생성** (30분)
```javascript
class BidirectionalApp {
  constructor() {
    this.gestureRecognizer = new GestureRecognizer();
    this.speechRecognizer = new SpeechRecognizer();
    this.brailleIntegration = new BrailleIntegration();
    this.brailleReverse = new BrailleReverseConverter();

    this.state = { /* ... */ };
    this.bindEvents();
  }
}
```

2. **제스처 입력 통합** (1시간)
```javascript
async startGesture() {
  await this._startCamera();
  this.hands.onResults((results) => {
    const gesture = this.gestureRecognizer.processFrame(results);
    this._handleGesture(gesture);
  });
}
```

3. **음성 입력 통합** (1시간)
```javascript
async startSpeech() {
  this.speechRecognizer.onFinalResult = (result) => {
    this._handleSpeech(result);
  };
  await this.speechRecognizer.start();
}
```

4. **양방향 변환 로직** (2시간)
```javascript
async _convertTextToBraille(text) {
  const result = await this.brailleIntegration.convert(text);
  this._updateOutput(result);
  this._speakText(text);
}
```

5. **UI 업데이트 로직** (1시간)
```javascript
_updateStatus(type, message) {
  this.elements[`${type}Status`].textContent = message;
}

_updateOutput(result) {
  this.elements.textOutput.textContent = result.text;
  this.elements.ipaOutput.textContent = result.ipa;
  this.elements.brailleOutput.textContent = result.braille;
}
```

**예상 총 시간:** 5-6시간

---

### **Step 2.2: HTML/CSS 구현**

**작업 순서:**

1. **HTML 기본 구조** (1시간)
   - Header
   - Input panels (gesture + speech)
   - Output panel
   - Settings panel

2. **접근성 구현** (2시간)
   - ARIA 라벨 추가
   - Skip links
   - Keyboard navigation
   - Screen reader support

3. **CSS 스타일링** (3시간)
   - 테마 3종 (high-contrast, dark, light)
   - 글꼴 크기 3단계
   - 반응형 디자인
   - 애니메이션

**예상 총 시간:** 6시간

---

## 🧪 **Phase 3: 테스트 & 디버깅** (우선순위: 🟡 Medium)

### **Step 3.1: 단위 테스트**

**테스트 대상:**
- BrailleReverseConverter
- SpeechRecognizer
- TextToSpeech

**예상 시간:** 2시간

---

### **Step 3.2: 통합 테스트**

**시나리오:**
1. 농인 제스처 → 맹인 음성 출력
2. 맹인 음성 → 농인 텍스트 표시
3. 점자 ↔ 텍스트 양방향 변환

**예상 시간:** 3시간

---

### **Step 3.3: 접근성 테스트**

**도구:**
- Chrome Lighthouse
- WAVE (Web Accessibility Evaluation Tool)
- NVDA (Screen Reader)

**예상 시간:** 2시간

---

## 📦 **Phase 4: 배포 준비** (우선순위: 🟢 Low)

### **Step 4.1: 문서화**
- README.md 업데이트
- API 문서 작성
- 사용자 가이드

**예상 시간:** 2시간

---

### **Step 4.2: 성능 최적화**
- Bundle size 최적화
- Lazy loading
- Code splitting

**예상 시간:** 2시간

---

## ⏱️ **전체 일정**

| Phase | 작업 | 예상 시간 | 우선순위 |
|-------|------|----------|---------|
| 1 | 신규 모듈 구현 | 4-6시간 | 🔴 High |
| 2 | 양방향 통합 앱 | 11-12시간 | 🔴 High |
| 3 | 테스트 & 디버깅 | 7시간 | 🟡 Medium |
| 4 | 배포 준비 | 4시간 | 🟢 Low |
| **Total** | **26-29시간** | **~4일** |

---

## 🎯 **MVP (최소 기능 제품) 기준**

**Must Have (반드시 필요):**
- ✅ 제스처 → 텍스트 → 점자 → 음성
- ✅ 음성 → 텍스트 표시
- ✅ 기본 UI (high-contrast 테마)
- ✅ 키보드 단축키

**Should Have (있으면 좋음):**
- ✅ 점자 → 텍스트 역변환
- ✅ 테마 3종
- ✅ 글꼴 크기 조절
- ✅ 음성 속도 조절

**Could Have (선택 사항):**
- ⭐ 히스토리 저장
- ⭐ 오프라인 모드
- ⭐ PWA 지원

---

## 🚀 **빠른 시작 가이드**

### **개발 환경 설정**

1. **의존성 설치:**
```bash
# MediaPipe는 CDN 사용 (설치 불필요)
# Web Speech API는 브라우저 내장
```

2. **로컬 서버 실행:**
```bash
# Python 간단 서버
python -m http.server 8000

# 또는 Node.js
npx http-server -p 8000
```

3. **브라우저 접속:**
```
http://localhost:8000/bidirectional/
```

---

### **첫 실행 체크리스트**

- [ ] Chrome/Edge 브라우저 사용 (Web Speech API 필요)
- [ ] HTTPS 또는 localhost (카메라/마이크 권한)
- [ ] 카메라 연결 확인
- [ ] 마이크 연결 확인
- [ ] 점자 폰트 설치 (선택)

---

**다음: Part 7 - 테스트 시나리오**
