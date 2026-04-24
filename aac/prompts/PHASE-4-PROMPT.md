# Phase 4: WIA Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Phase**: 4 of 4
**목표**: AAC 텍스트 출력을 WIA 생태계 (TTS, ISP, WIA Braille)와 연동
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + 연동 모듈 구현 + 예제

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1에서 Signal Format을 정의하고,
 Phase 2에서 API Interface를 만들고,
 Phase 3에서 Communication Protocol을 정의했다.

 이제 AAC 사용자가 입력한 텍스트를 어떻게 출력할 것인가?

 - 음성으로? (TTS)
 - 수어 아바타로? (ISP/WIA Talk)
 - 점자로? (WIA Braille)

 모든 출력 방식에서 동일한 인터페이스를 사용할 수 있을까?"
```

### 목표
```
AAC 센서 입력 → 텍스트 출력 → WIA 생태계 연동

출력 경로:
├─ TTS: 텍스트 → 음성 (비장애인과 소통)
├─ ISP: 텍스트 → 수어 아바타 (청각장애인과 소통)
└─ WIA Braille: 텍스트 → 점자 (시각장애인과 소통)

단일 API로 모든 출력 방식 지원
```

---

## 📋 Phase 1-3 결과물 활용

| 이전 Phase 산출물 | Phase 4 활용 |
|-----------------|-------------|
| Phase 1: Signal Format | 센서 입력 처리 |
| Phase 2: API Interface | 텍스트 생성 API |
| Phase 3: Protocol | 메시지 전송 |
| WIA 기반 문서 | 출력 연동 스펙 |

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: TTS API 조사

| 서비스 | 조사 대상 | 웹서치 키워드 |
|-------|----------|--------------|
| **Web Speech API** | 브라우저 내장 TTS | "Web Speech API SpeechSynthesis" |
| **Google Cloud TTS** | 클라우드 TTS | "Google Cloud Text-to-Speech API" |
| **Amazon Polly** | AWS TTS | "Amazon Polly API reference" |
| **Microsoft Azure** | Azure TTS | "Azure Speech Service TTS" |

### 2단계: 수어 아바타 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **3D 아바타** | 수어 애니메이션 | "sign language avatar animation" |
| **MediaPipe** | 포즈 추정 | "MediaPipe holistic pose estimation" |
| **ISP 구현** | 코드 to 제스처 | (내부 문서 참조) |

### 3단계: 점자 출력 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **점자 디스플레이** | 하드웨어 인터페이스 | "refreshable braille display API" |
| **BrailleBack** | Android 접근성 | "Android BrailleBack API" |
| **liblouis** | 점자 변환 라이브러리 | "liblouis braille translation" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. TTS 서비스 비교

### Web Speech API
- 장점: [조사 내용]
- 단점: [조사 내용]
- AAC 적용: [분석]

### Cloud TTS (Google/Amazon/Azure)
- 장점: [조사 내용]
- 단점: [조사 내용]
- AAC 적용: [분석]

## 2. 수어 아바타 기술

### 3D 애니메이션 방식
- 현황: [조사 내용]
- 기술: [조사 내용]

### ISP 기반 구현
- WIA Talk 문서 참조
- 구현 방향: [제안]

## 3. 점자 출력 기술

### 점자 디스플레이
- 현황: [조사 내용]
- API: [조사 내용]

### WIA Braille 연동
- IPA 변환 방식
- 구현 방향: [제안]

## 4. 결론
- 권장 TTS 방식: [제안]
- ISP 연동 설계: [제안]
- 점자 연동 설계: [제안]
```

---

## 🏗️ 출력 연동 설계

### 1. 출력 인터페이스 (Output Interface)

#### 기본 출력 인터페이스
```typescript
interface IOutputAdapter {
  readonly type: OutputType;
  readonly name: string;

  // 초기화
  initialize(options?: OutputOptions): Promise<void>;

  // 출력
  output(text: string, options?: OutputOptions): Promise<void>;

  // 상태 확인
  isAvailable(): boolean;

  // 정리
  dispose(): Promise<void>;
}

type OutputType = 'tts' | 'sign_language' | 'braille' | 'custom';

interface OutputOptions {
  language?: string;      // 언어 코드 (ko, en, ja, ...)
  voice?: string;         // TTS 음성 ID
  speed?: number;         // 출력 속도 (0.5 ~ 2.0)
  volume?: number;        // 볼륨 (0.0 ~ 1.0)
  [key: string]: any;     // 추가 옵션
}
```

### 2. TTS 출력 어댑터

```typescript
interface ITTSAdapter extends IOutputAdapter {
  type: 'tts';

  // TTS 전용 메서드
  getVoices(): Promise<Voice[]>;
  setVoice(voiceId: string): void;
  pause(): void;
  resume(): void;
  stop(): void;

  // 이벤트
  onStart(handler: () => void): void;
  onEnd(handler: () => void): void;
  onError(handler: (error: Error) => void): void;
}

interface Voice {
  id: string;
  name: string;
  language: string;
  gender?: 'male' | 'female' | 'neutral';
}
```

### 3. ISP/WIA Talk 출력 어댑터

```typescript
interface ISignLanguageAdapter extends IOutputAdapter {
  type: 'sign_language';

  // 수어 전용 메서드
  textToISP(text: string): Promise<ISPCode[]>;
  playGesture(ispCode: ISPCode): Promise<void>;
  playSequence(ispCodes: ISPCode[]): Promise<void>;

  // 아바타 설정
  setAvatar(avatarId: string): void;
  setSpeed(speed: number): void;

  // 이벤트
  onGestureStart(handler: (code: ISPCode) => void): void;
  onGestureEnd(handler: (code: ISPCode) => void): void;
}

interface ISPCode {
  code: string;           // "HS01-LC07-MV10-OR02-NM15"
  meaning?: string;       // "안녕" (선택)
  duration?: number;      // 밀리초
}
```

### 4. WIA Braille 출력 어댑터

```typescript
interface IBrailleAdapter extends IOutputAdapter {
  type: 'braille';

  // 점자 전용 메서드
  textToIPA(text: string): Promise<string>;
  textToBraille(text: string): Promise<BrailleOutput>;
  sendToDisplay(braille: BrailleOutput): Promise<void>;

  // 디스플레이 설정
  getConnectedDisplays(): Promise<BrailleDisplay[]>;
  setDisplay(displayId: string): void;
}

interface BrailleOutput {
  ipa: string;            // "/annjʌŋ/"
  braille: string;        // "⠁⠝⠚⠪⠝"
  unicode: string[];      // ["U+2801", "U+281D", ...]
}

interface BrailleDisplay {
  id: string;
  name: string;
  cells: number;          // 점자 셀 수 (40, 80 등)
  connected: boolean;
}
```

### 5. 통합 출력 매니저

```typescript
class OutputManager {
  private adapters: Map<OutputType, IOutputAdapter> = new Map();

  // 어댑터 등록
  register(adapter: IOutputAdapter): void;

  // 어댑터 제거
  unregister(type: OutputType): void;

  // 출력 (모든 활성 어댑터로)
  broadcast(text: string, options?: OutputOptions): Promise<void>;

  // 특정 어댑터로 출력
  outputTo(type: OutputType, text: string, options?: OutputOptions): Promise<void>;

  // 활성 어댑터 조회
  getActiveAdapters(): IOutputAdapter[];
}
```

---

## 📁 산출물 목록

Phase 4 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-4.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-4-INTEGRATION.md

내용:
1. 개요 (Overview)
2. 출력 계층 아키텍처 (Output Layer Architecture)
3. 출력 인터페이스 (Output Interface)
4. TTS 연동 (Text-to-Speech Integration)
5. ISP/WIA Talk 연동 (Sign Language Integration)
6. WIA Braille 연동 (Braille Integration)
7. 통합 출력 매니저 (Output Manager)
8. 이벤트 및 콜백 (Events and Callbacks)
9. 에러 처리 (Error Handling)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. TypeScript 출력 모듈
```
/api/typescript/src/
├── output/
│   ├── index.ts
│   ├── IOutputAdapter.ts       # 출력 인터페이스
│   ├── OutputManager.ts        # 통합 매니저
│   ├── TTSAdapter.ts           # TTS 어댑터 (Web Speech API)
│   ├── SignLanguageAdapter.ts  # ISP/WIA Talk 어댑터
│   ├── BrailleAdapter.ts       # WIA Braille 어댑터
│   └── MockOutputAdapter.ts    # 테스트용
└── ...
```

### 4. Python 출력 모듈
```
/api/python/wia_aac/
├── output/
│   ├── __init__.py
│   ├── base_output.py          # 출력 인터페이스
│   ├── output_manager.py       # 통합 매니저
│   ├── tts_adapter.py          # TTS 어댑터
│   ├── sign_language_adapter.py # ISP/WIA Talk 어댑터
│   ├── braille_adapter.py      # WIA Braille 어댑터
│   └── mock_output.py          # 테스트용
└── ...
```

### 5. 예제 코드
```
/examples/integration/
├── typescript/
│   ├── full-aac-demo.ts        # 센서 → 텍스트 → 출력 전체 예제
│   ├── tts-output.ts           # TTS 출력 예제
│   ├── sign-language-output.ts # 수어 출력 예제
│   └── braille-output.ts       # 점자 출력 예제
└── python/
    ├── full_aac_demo.py
    ├── tts_output.py
    ├── sign_language_output.py
    └── braille_output.py
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 TTS/수어/점자 기술 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ TypeScript output 모듈 구현 완료
□ Python output 모듈 구현 완료
□ TTS 어댑터 구현 완료
□ ISP/WIA Talk 어댑터 구현 완료 (Mock)
□ WIA Braille 어댑터 구현 완료 (Mock)
□ OutputManager 구현 완료
□ 단위 테스트 작성 완료
□ 테스트 통과
□ 전체 AAC 데모 예제 완료
□ README 업데이트 (Phase 4 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 TTS/수어/점자 기술 조사
   ↓
2. /spec/RESEARCH-PHASE-4.md 작성
   ↓
3. 출력 인터페이스 설계
   ↓
4. /spec/PHASE-4-INTEGRATION.md 작성
   ↓
5. TypeScript IOutputAdapter 정의
   ↓
6. TypeScript TTSAdapter 구현
   ↓
7. TypeScript SignLanguageAdapter 구현 (Mock)
   ↓
8. TypeScript BrailleAdapter 구현 (Mock)
   ↓
9. TypeScript OutputManager 구현
   ↓
10. Python 모듈 구현
   ↓
11. 테스트 작성 및 실행
   ↓
12. 전체 AAC 데모 예제 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. WIA AAC Standard 완료! 🎉
```

---

## 💡 설계 가이드라인

### DO (해야 할 것)

```
✅ Phase 1-3 결과물과 연동 가능하도록 설계
✅ 출력 어댑터 추상화 (새로운 출력 방식 쉽게 추가)
✅ Mock 구현으로 테스트 가능하게
✅ 비동기 처리 (async/await)
✅ 이벤트 기반 콜백 지원
✅ 에러 처리 포함
```

### DON'T (하지 말 것)

```
❌ 특정 TTS 서비스에만 종속
❌ 실제 하드웨어 필수 의존 (Mock 필요)
❌ 동기 블로킹 처리
❌ Phase 1-3 형식과 불일치
```

---

## 🔗 WIA 생태계 연동 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                     AAC 사용자                               │
│              (ALS, 뇌성마비, 사지마비 등)                    │
└─────────────────────────────────────────────────────────────┘
                              │
                         [센서 입력]
                    눈/뺨/뇌파/숨/스위치
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 1: Signal Format Standard                 │
│                   센서 신호 → 표준 JSON                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 2: API Interface Standard                 │
│                  표준 API → 텍스트 생성                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 3: Communication Protocol                 │
│                  메시지 전송 → 수신                          │
└─────────────────────────────────────────────────────────────┘
                              │
                         [텍스트 출력]
                        "안녕하세요"
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              Phase 4: WIA Ecosystem Integration              │
│                     OutputManager                            │
├─────────────────┬─────────────────┬─────────────────────────┤
│   TTSAdapter    │ SignLanguageAdapter │   BrailleAdapter     │
└────────┬────────┴────────┬────────┴──────────┬──────────────┘
         │                 │                   │
         ▼                 ▼                   ▼
    ┌─────────┐      ┌─────────┐         ┌─────────┐
    │   TTS   │      │   ISP   │         │   WIA   │
    │  음성   │      │  수어   │         │ Braille │
    │  출력   │      │ 아바타  │         │  점자   │
    └─────────┘      └─────────┘         └─────────┘
         │                 │                   │
         ▼                 ▼                   ▼
    비장애인            청각장애인           시각장애인
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 TTS 및 접근성 기술 조사**

```
검색 키워드: "Web Speech API SpeechSynthesis tutorial"
```

화이팅! 🤟

WIA AAC Standard의 마지막 Phase입니다.
완료되면 센서 입력부터 출력까지 전체 파이프라인이 완성됩니다!

---

<div align="center">

**Phase 4 of 4**

WIA Ecosystem Integration

🎯 최종 목표: 센서 → 텍스트 → 출력 (TTS/수어/점자)

</div>
