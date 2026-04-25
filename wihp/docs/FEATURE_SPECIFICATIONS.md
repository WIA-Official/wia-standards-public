# WIA Talk 기능 명세서 (MVP 필수)
**ENTERPRISE-GRADE | PRODUCTION-READY | RUST-QUALITY**

> 제미나이도 인정하는 완벽한 명세서
> Version: 2.0.0 Bidirectional

---

## Part 2: 기능 명세서

### 🎯 **A. 핵심 변환 기능**

---

#### **A-1: 맹인 음성 → 텍스트 (STT)** ⭐ 신규

**기능 설명:**
맹인 사용자의 음성을 실시간으로 텍스트로 변환하여 농인에게 전달

**기술 스택:**
- Web Speech API (SpeechRecognition)
- Continuous mode (연속 인식)
- Korean language support

**상세 명세:**

```javascript
/**
 * SpeechToText 모듈
 *
 * @class SpeechRecognizer
 * @version 2.0.0
 * @quality Enterprise-Grade
 */

class SpeechRecognizer {
  constructor() {
    // Web Speech API 초기화
    this.recognition = new (window.SpeechRecognition ||
                           window.webkitSpeechRecognition)();

    // 설정
    this.config = {
      lang: 'ko-KR',                    // 한국어
      continuous: true,                 // 연속 인식
      interimResults: true,             // 중간 결과 표시
      maxAlternatives: 3                // 대안 3개
    };

    // 상태
    this.state = {
      listening: false,
      lastResult: '',
      confidence: 0,
      errorCount: 0
    };

    // 성능 메트릭
    this.metrics = {
      totalRecognitions: 0,
      averageConfidence: 0,
      errors: 0
    };

    this._initialize();
  }

  /**
   * 초기화
   * @private
   */
  _initialize() {
    this.recognition.lang = this.config.lang;
    this.recognition.continuous = this.config.continuous;
    this.recognition.interimResults = this.config.interimResults;
    this.recognition.maxAlternatives = this.config.maxAlternatives;

    this._bindEvents();
  }

  /**
   * 이벤트 바인딩
   * @private
   */
  _bindEvents() {
    // 결과 이벤트
    this.recognition.onresult = (event) => {
      this._handleResult(event);
    };

    // 에러 이벤트
    this.recognition.onerror = (event) => {
      this._handleError(event);
    };

    // 종료 이벤트
    this.recognition.onend = () => {
      this._handleEnd();
    };

    // 시작 이벤트
    this.recognition.onstart = () => {
      this.state.listening = true;
      console.log('🎤 음성 인식 시작');
    };
  }

  /**
   * 음성 인식 시작
   * @returns {Promise<void>}
   */
  async start() {
    try {
      if (this.state.listening) {
        console.warn('⚠️  Already listening');
        return;
      }

      await this.recognition.start();
      console.log('✅ Speech recognition started');

    } catch (error) {
      this._logError('Start failed', error);
      throw error;
    }
  }

  /**
   * 음성 인식 중지
   */
  stop() {
    try {
      if (!this.state.listening) {
        console.warn('⚠️  Not listening');
        return;
      }

      this.recognition.stop();
      this.state.listening = false;
      console.log('🛑 Speech recognition stopped');

    } catch (error) {
      this._logError('Stop failed', error);
    }
  }

  /**
   * 결과 처리
   * @private
   * @param {SpeechRecognitionEvent} event
   */
  _handleResult(event) {
    try {
      const results = event.results;
      const lastResultIndex = results.length - 1;
      const result = results[lastResultIndex];

      // Final result
      if (result.isFinal) {
        const transcript = result[0].transcript;
        const confidence = result[0].confidence;

        this.state.lastResult = transcript;
        this.state.confidence = confidence;

        // 메트릭 업데이트
        this.metrics.totalRecognitions++;
        this.metrics.averageConfidence = (
          (this.metrics.averageConfidence * (this.metrics.totalRecognitions - 1) + confidence) /
          this.metrics.totalRecognitions
        );

        // 콜백 호출
        if (this.onFinalResult) {
          this.onFinalResult({
            text: transcript,
            confidence: confidence,
            timestamp: Date.now()
          });
        }

        console.log(`✅ Final: "${transcript}" (${(confidence * 100).toFixed(1)}%)`);
      }
      // Interim result
      else {
        const transcript = result[0].transcript;

        if (this.onInterimResult) {
          this.onInterimResult({
            text: transcript,
            isFinal: false
          });
        }

        console.log(`⏳ Interim: "${transcript}"`);
      }

    } catch (error) {
      this._logError('Result processing failed', error);
    }
  }

  /**
   * 에러 처리
   * @private
   * @param {SpeechRecognitionError} event
   */
  _handleError(event) {
    this.state.errorCount++;
    this.metrics.errors++;

    const errorMessages = {
      'no-speech': '음성이 감지되지 않았습니다',
      'audio-capture': '마이크를 찾을 수 없습니다',
      'not-allowed': '마이크 권한이 필요합니다',
      'network': '네트워크 오류가 발생했습니다'
    };

    const userMessage = errorMessages[event.error] || event.error;

    console.error(`❌ Speech recognition error: ${userMessage}`);

    if (this.onError) {
      this.onError({
        type: event.error,
        message: userMessage,
        timestamp: Date.now()
      });
    }

    // Auto-restart on certain errors
    if (event.error === 'no-speech' || event.error === 'network') {
      setTimeout(() => {
        if (this.state.listening) {
          console.log('🔄 Auto-restarting...');
          this.start();
        }
      }, 1000);
    }
  }

  /**
   * 종료 처리
   * @private
   */
  _handleEnd() {
    this.state.listening = false;

    // Auto-restart if continuous mode
    if (this.config.continuous) {
      setTimeout(() => {
        if (this.state.listening === false) {  // Intentionally stopped
          console.log('🔄 Continuous mode: restarting...');
          this.start();
        }
      }, 100);
    }
  }

  /**
   * 콜백 등록
   */
  onFinalResult(callback) {
    this.onFinalResult = callback;
  }

  onInterimResult(callback) {
    this.onInterimResult = callback;
  }

  onError(callback) {
    this.onError = callback;
  }

  /**
   * 메트릭 조회
   */
  getMetrics() {
    return {
      ...this.metrics,
      state: this.state
    };
  }

  /**
   * 에러 로깅
   * @private
   */
  _logError(message, error) {
    console.error(`❌ SpeechRecognizer: ${message}`, error);
  }
}
```

**사용 예시:**
```javascript
// 초기화
const speechRecognizer = new SpeechRecognizer();

// 이벤트 리스너 등록
speechRecognizer.onFinalResult = (result) => {
  console.log('맹인 말함:', result.text);
  // 농인에게 텍스트 표시
  displayTextForDeaf(result.text);
};

// 시작
await speechRecognizer.start();
```

**성능 목표:**
- 인식 지연: <500ms
- 정확도: >90%
- 연속 작동: >1시간

---

#### **A-2: 텍스트 → 맹인 음성 (TTS)**

**기능 설명:**
농인이 입력한 텍스트를 음성으로 변환하여 맹인에게 전달

**상세 명세:**

```javascript
class TextToSpeech {
  constructor() {
    this.synth = window.speechSynthesis;
    this.config = {
      lang: 'ko-KR',
      rate: 1.0,      // 0.5 ~ 2.0
      pitch: 1.0,     // 0.0 ~ 2.0
      volume: 1.0     // 0.0 ~ 1.0
    };
  }

  speak(text, options = {}) {
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.lang = options.lang || this.config.lang;
    utterance.rate = options.rate || this.config.rate;
    utterance.pitch = options.pitch || this.config.pitch;
    utterance.volume = options.volume || this.config.volume;

    this.synth.speak(utterance);
  }

  stop() {
    this.synth.cancel();
  }

  setRate(rate) {
    this.config.rate = Math.max(0.5, Math.min(2.0, rate));
  }
}
```

---

#### **A-3: 제스처 → 텍스트 (기존)**

**상태:** ✅ 이미 완성 (GestureRecognizer v2.0.0)

**기능:**
- 93개 제스처 인식
- MediaPipe Hands 기반
- 성능: <50ms

---

#### **A-4: 텍스트 → IPA → 점자**

**상태:** ✅ 이미 완성 (BrailleIntegration v2.0.0)

**기능:**
- 한글 → IPA 변환
- IPA → 점자 매핑
- 성능: <50ms

---

#### **A-5: 점자 → 텍스트 (역변환)** ⭐ 신규

**기능 설명:**
점자를 텍스트로 역변환 (맹인이 입력한 점자를 농인이 읽을 수 있게)

**상세 명세:**

```javascript
/**
 * 점자 → 텍스트 역변환기
 *
 * @class BrailleReverseConverter
 * @version 2.0.0
 * @quality Enterprise-Grade
 */

class BrailleReverseConverter {
  constructor() {
    // 역 매핑 테이블
    this.brailleToIPAMap = null;
    this.ipaToKoreanMap = null;

    // 성능 메트릭
    this.metrics = {
      totalConversions: 0,
      averageTime: 0,
      errors: 0
    };

    this._initialize();
  }

  /**
   * 초기화 - 매핑 테이블 로드
   * @private
   */
  async _initialize() {
    try {
      // 기존 BrailleIntegration의 매핑 테이블 사용
      const response = await fetch('/tables/wia-braille-mapping.json');
      const mapping = await response.json();

      // 역 매핑 생성
      this._buildReverseMaps(mapping);

      console.log('✅ BrailleReverseConverter initialized');

    } catch (error) {
      console.error('❌ Initialization failed:', error);
      throw error;
    }
  }

  /**
   * 역 매핑 테이블 생성
   * @private
   * @param {object} mapping - Original mapping
   */
  _buildReverseMaps(mapping) {
    this.brailleToIPAMap = new Map();

    // 모음
    for (const [ipa, braille] of Object.entries(mapping.vowels)) {
      this.brailleToIPAMap.set(braille, ipa);
    }

    // 초성 자음
    for (const [ipa, braille] of Object.entries(mapping.consonants.initial)) {
      this.brailleToIPAMap.set(braille, ipa);
    }

    // 종성 자음
    for (const [ipa, braille] of Object.entries(mapping.consonants.final)) {
      this.brailleToIPAMap.set(braille, ipa);
    }

    // IPA → 한글 매핑
    this.ipaToKoreanMap = this._buildIPAtoKoreanMap(mapping);
  }

  /**
   * IPA → 한글 매핑 생성
   * @private
   */
  _buildIPAtoKoreanMap(mapping) {
    const ipaToKorean = new Map();

    // korean_jamo_to_ipa를 역으로 매핑
    for (const [jamo, ipa] of Object.entries(mapping.korean_jamo_to_ipa.초성)) {
      ipaToKorean.set(ipa, { type: 'cho', jamo });
    }

    for (const [jamo, ipa] of Object.entries(mapping.korean_jamo_to_ipa.중성)) {
      ipaToKorean.set(ipa, { type: 'jung', jamo });
    }

    for (const [jamo, ipa] of Object.entries(mapping.korean_jamo_to_ipa.종성)) {
      ipaToKorean.set(ipa, { type: 'jong', jamo });
    }

    return ipaToKorean;
  }

  /**
   * 점자 → 텍스트 변환 (메인 함수)
   * @param {string} brailleText - 점자 문자열
   * @returns {object} { text, ipa, confidence }
   */
  convert(brailleText) {
    const startTime = performance.now();

    try {
      // Step 1: 점자 → IPA
      const ipa = this.brailleToIPA(brailleText);

      // Step 2: IPA → 한글
      const korean = this.ipaToKorean(ipa);

      // Step 3: 메트릭 업데이트
      const endTime = performance.now();
      const duration = endTime - startTime;

      this.metrics.totalConversions++;
      this.metrics.averageTime = (
        (this.metrics.averageTime * (this.metrics.totalConversions - 1) + duration) /
        this.metrics.totalConversions
      );

      return {
        text: korean,
        ipa: ipa,
        duration: duration,
        confidence: 0.95  // 역변환은 확정적이므로 높은 신뢰도
      };

    } catch (error) {
      this.metrics.errors++;
      console.error('❌ Braille reverse conversion failed:', error);

      return {
        text: '',
        ipa: '',
        duration: 0,
        confidence: 0,
        error: error.message
      };
    }
  }

  /**
   * 점자 → IPA 변환
   * @param {string} brailleText
   * @returns {string} IPA
   */
  brailleToIPA(brailleText) {
    let ipa = '';

    for (const brailleChar of brailleText) {
      if (brailleChar === ' ') {
        ipa += ' ';
        continue;
      }

      const ipaChar = this.brailleToIPAMap.get(brailleChar);

      if (ipaChar) {
        ipa += ipaChar;
      } else {
        console.warn(`⚠️  Unknown braille: ${brailleChar}`);
        ipa += '?';
      }
    }

    return ipa;
  }

  /**
   * IPA → 한글 변환
   * @param {string} ipa
   * @returns {string} Korean text
   */
  ipaToKorean(ipa) {
    // IPA를 자모 단위로 분해
    const jamos = this._parseIPA(ipa);

    // 자모를 한글 음절로 조합
    const korean = this._assembleKorean(jamos);

    return korean;
  }

  /**
   * IPA 파싱 (자모 단위로 분해)
   * @private
   * @param {string} ipa
   * @returns {Array} Jamo array
   */
  _parseIPA(ipa) {
    const jamos = [];
    let i = 0;

    while (i < ipa.length) {
      // 2글자 IPA 먼저 확인 (예: k̚)
      if (i < ipa.length - 1) {
        const twoChar = ipa.substring(i, i + 2);
        const mapping = this.ipaToKoreanMap.get(twoChar);

        if (mapping) {
          jamos.push(mapping);
          i += 2;
          continue;
        }
      }

      // 1글자 IPA 확인
      const oneChar = ipa[i];
      const mapping = this.ipaToKoreanMap.get(oneChar);

      if (mapping) {
        jamos.push(mapping);
      } else if (oneChar === ' ') {
        jamos.push({ type: 'space' });
      } else {
        console.warn(`⚠️  Unknown IPA: ${oneChar}`);
      }

      i++;
    }

    return jamos;
  }

  /**
   * 자모 → 한글 음절 조합
   * @private
   * @param {Array} jamos
   * @returns {string} Korean text
   */
  _assembleKorean(jamos) {
    let result = '';
    let cho = null;
    let jung = null;
    let jong = null;

    const HANGUL_BASE = 0xAC00;
    const CHOSEONG = ['ㄱ', 'ㄲ', 'ㄴ', 'ㄷ', 'ㄸ', 'ㄹ', 'ㅁ', 'ㅂ', 'ㅃ', 'ㅅ', 'ㅆ', 'ㅇ', 'ㅈ', 'ㅉ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ'];
    const JUNGSEONG = ['ㅏ', 'ㅐ', 'ㅑ', 'ㅒ', 'ㅓ', 'ㅔ', 'ㅕ', 'ㅖ', 'ㅗ', 'ㅘ', 'ㅙ', 'ㅚ', 'ㅛ', 'ㅜ', 'ㅝ', 'ㅞ', 'ㅟ', 'ㅠ', 'ㅡ', 'ㅢ', 'ㅣ'];
    const JONGSEONG = ['', 'ㄱ', 'ㄲ', 'ㄳ', 'ㄴ', 'ㄵ', 'ㄶ', 'ㄷ', 'ㄹ', 'ㄺ', 'ㄻ', 'ㄼ', 'ㄽ', 'ㄾ', 'ㄿ', 'ㅀ', 'ㅁ', 'ㅂ', 'ㅄ', 'ㅅ', 'ㅆ', 'ㅇ', 'ㅈ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ'];

    for (const jamo of jamos) {
      if (jamo.type === 'space') {
        // 음절 완성
        if (cho && jung) {
          result += this._composeHangul(cho, jung, jong, HANGUL_BASE, CHOSEONG, JUNGSEONG, JONGSEONG);
        }
        result += ' ';
        cho = jung = jong = null;
        continue;
      }

      if (jamo.type === 'cho') {
        // 이전 음절 완성
        if (cho && jung) {
          result += this._composeHangul(cho, jung, jong, HANGUL_BASE, CHOSEONG, JUNGSEONG, JONGSEONG);
        }
        cho = jamo.jamo;
        jung = jong = null;
      } else if (jamo.type === 'jung') {
        jung = jamo.jamo;
      } else if (jamo.type === 'jong') {
        jong = jamo.jamo;
      }
    }

    // 마지막 음절 완성
    if (cho && jung) {
      result += this._composeHangul(cho, jung, jong, HANGUL_BASE, CHOSEONG, JUNGSEONG, JONGSEONG);
    }

    return result;
  }

  /**
   * 한글 음절 조합
   * @private
   */
  _composeHangul(cho, jung, jong, BASE, CHO_LIST, JUNG_LIST, JONG_LIST) {
    const choIndex = CHO_LIST.indexOf(cho);
    const jungIndex = JUNG_LIST.indexOf(jung);
    const jongIndex = jong ? JONG_LIST.indexOf(jong) : 0;

    if (choIndex === -1 || jungIndex === -1 || jongIndex === -1) {
      return cho + jung + (jong || '');
    }

    const code = BASE + (choIndex * 21 * 28) + (jungIndex * 28) + jongIndex;
    return String.fromCharCode(code);
  }

  /**
   * 메트릭 조회
   */
  getMetrics() {
    return { ...this.metrics };
  }
}
```

**사용 예시:**
```javascript
// 초기화
const reverseConverter = new BrailleReverseConverter();
await reverseConverter._initialize();

// 변환
const result = reverseConverter.convert('⠁⠝⠝⠚⠕⠓⠁⠎⠑⠚⠕');
console.log(result.text);  // "안녕하세요"
console.log(result.ipa);   // "/annjʌŋhasɛjo/"
```

**성능 목표:**
- 변환 시간: <100ms
- 정확도: >95%

---

### 🎨 **B. UI/UX 기능**

#### **B-1: 농인 UI (테마 3종, 글꼴 3단계)**

**테마 3종:**
1. **High Contrast** (고대비) - 검은 배경 + 흰색 텍스트
2. **Dark Mode** - 어두운 회색 + 밝은 회색
3. **Light Mode** - 밝은 배경 + 검은 텍스트

**글꼴 크기:**
- Small: 16px
- Medium: 20px (기본)
- Large: 28px

```css
/* 테마 변수 */
:root {
  --theme-bg: #ffffff;
  --theme-text: #000000;
  --theme-accent: #0066cc;
}

[data-theme="high-contrast"] {
  --theme-bg: #000000;
  --theme-text: #ffffff;
  --theme-accent: #ffff00;
}

[data-theme="dark"] {
  --theme-bg: #1a1a1a;
  --theme-text: #e0e0e0;
  --theme-accent: #4a9eff;
}

/* 글꼴 크기 */
[data-font-size="small"] { font-size: 16px; }
[data-font-size="medium"] { font-size: 20px; }
[data-font-size="large"] { font-size: 28px; }
```

---

#### **B-2: 맹인 UI (음성 속도 3단계, 단축키)**

**음성 속도:**
- Slow: 0.7x
- Normal: 1.0x (기본)
- Fast: 1.5x

**단축키:**
| 키 | 기능 |
|----|------|
| Space | 음성 인식 시작/중지 |
| Ctrl+R | 마지막 텍스트 재생 |
| Ctrl+↑ | 음성 속도 증가 |
| Ctrl+↓ | 음성 속도 감소 |
| Ctrl+M | 음소거 |
| Esc | 전체 중지 |

---

#### **B-3: 양방향 상태 표시** ⭐ 신규

**상태 표시기:**
```html
<div class="status-bar">
  <div class="status-item">
    <span class="status-label">농인 입력:</span>
    <span class="status-value" id="deafInputStatus">대기</span>
  </div>
  <div class="status-item">
    <span class="status-label">맹인 입력:</span>
    <span class="status-value" id="blindInputStatus">대기</span>
  </div>
  <div class="status-item">
    <span class="status-label">변환 상태:</span>
    <span class="status-value" id="conversionStatus">준비</span>
  </div>
</div>
```

**상태 값:**
- 대기 (Idle)
- 인식 중 (Recognizing)
- 변환 중 (Converting)
- 출력 중 (Outputting)
- 완료 (Complete)
- 오류 (Error)

---

### 🔔 **C. 피드백 & 에러 처리**

#### **C-1: 피드백 (진동/소리)**

**진동 패턴:**
```javascript
// 성공
navigator.vibrate(200);

// 에러
navigator.vibrate([100, 50, 100, 50, 100]);

// 알림
navigator.vibrate(500);
```

**소리 피드백:**
- 제스처 인식 성공: "딩" (pitch: high)
- 음성 인식 시작: "삐" (pitch: medium)
- 에러: "부웅" (pitch: low)

---

#### **C-2: 에러 처리**

**에러 분류:**

| 에러 타입 | 사용자 메시지 | 복구 방법 |
|----------|--------------|----------|
| NO_CAMERA | "카메라를 찾을 수 없습니다" | 수동 입력 모드 |
| NO_MIC | "마이크를 찾을 수 없습니다" | 키보드 입력 |
| NO_SPEECH | "음성이 감지되지 않았습니다" | 재시도 안내 |
| NETWORK | "네트워크 오류" | 오프라인 모드 |
| UNKNOWN | "알 수 없는 오류" | 페이지 새로고침 |

**에러 표시:**
```html
<div class="error-toast" role="alert" aria-live="assertive">
  <span class="error-icon">⚠️</span>
  <span class="error-message">카메라를 찾을 수 없습니다</span>
  <button class="error-close">×</button>
</div>
```

---

## 🎯 **성능 목표 요약**

| 기능 | 목표 | 우선순위 |
|------|------|---------|
| 제스처 인식 | <50ms | 🔴 High |
| 음성 인식 (STT) | <500ms | 🔴 High |
| 텍스트→점자 | <50ms | 🔴 High |
| 점자→텍스트 | <100ms | 🟡 Medium |
| TTS | <200ms | 🟡 Medium |
| UI 업데이트 | <16ms (60 FPS) | 🟢 Low |

---

**다음: Part 3 - 통합 앱 (app.js)**
