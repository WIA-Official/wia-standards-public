# WIA BCI Ecosystem Integration Specification

**Phase 4: WIA Ecosystem Integration Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA BCI Ecosystem Integration은 BCI 신호 처리 결과를 WIA 생태계의 다양한 출력 시스템과 연동하는 표준입니다. 이 표준은 BCI 사용자가 생성한 텍스트, 명령, 제어 신호를 음성, 수어, 점자, 뉴로피드백 등 다양한 방식으로 출력할 수 있게 합니다.

### 1.2 Design Goals

1. **Unified Interface**: 모든 출력 방식에 동일한 인터페이스 제공
2. **Multimodal Output**: 동시에 여러 출력 채널 지원
3. **Extensible**: 새로운 출력 방식 쉽게 추가
4. **Accessible**: 다양한 장애 유형에 맞는 출력 지원
5. **Real-time**: 저지연 실시간 출력

### 1.3 Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    BCI Signal Input                          │
│                 (Phase 1-3 Pipeline)                        │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Output Content                            │
│          (Text, Command, Classification, Signal)            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    OutputManager                             │
│                  (Central Controller)                        │
├─────────────┬─────────────┬─────────────┬───────────────────┤
│ TTSAdapter  │ SignAdapter │ BrailleAdptr│ NeurofeedbackAdptr│
├─────────────┼─────────────┼─────────────┼───────────────────┤
│  음성 출력  │  수어 아바타  │  점자 출력   │   뇌파 시각화    │
└─────────────┴─────────────┴─────────────┴───────────────────┘
```

---

## 2. Output Types

### 2.1 Output Type Enumeration

```typescript
type OutputType =
  | 'tts'             // Text-to-Speech
  | 'sign_language'   // Sign Language Avatar (ISP)
  | 'braille'         // Braille Display
  | 'neurofeedback'   // Neurofeedback Visualization
  | 'cursor'          // Cursor Control
  | 'custom';         // Custom Output
```

### 2.2 Output Content

```typescript
interface OutputContent {
  // 콘텐츠 유형
  type: 'text' | 'command' | 'classification' | 'signal';

  // 텍스트 콘텐츠
  text?: string;

  // 명령 콘텐츠
  command?: {
    action: string;
    params?: Record<string, unknown>;
  };

  // 분류 결과
  classification?: {
    classId: number;
    className: string;
    confidence: number;
  };

  // 신호 데이터 (뉴로피드백용)
  signal?: {
    bandPowers: BandPowers;
    channels: ChannelData[];
  };

  // 메타데이터
  metadata?: {
    timestamp: number;
    source?: string;
    priority?: 'low' | 'normal' | 'high';
  };
}
```

---

## 3. Output Adapter Interface

### 3.1 Base Interface

```typescript
interface IOutputAdapter {
  // 어댑터 정보
  readonly type: OutputType;
  readonly name: string;
  readonly version: string;

  // 초기화
  initialize(options?: OutputOptions): Promise<void>;

  // 출력
  output(content: OutputContent): Promise<void>;

  // 상태 확인
  isAvailable(): boolean;
  isReady(): boolean;

  // 이벤트
  on(event: OutputEvent, handler: OutputEventHandler): void;
  off(event: OutputEvent, handler: OutputEventHandler): void;

  // 정리
  dispose(): Promise<void>;
}

interface OutputOptions {
  language?: string;      // 언어 코드 (ko, en, ja, ...)
  autoStart?: boolean;    // 자동 시작
  [key: string]: unknown; // 어댑터별 추가 옵션
}

type OutputEvent = 'start' | 'end' | 'error' | 'ready' | 'busy';
type OutputEventHandler = (event: OutputEventData) => void;
```

### 3.2 Adapter Registry

```typescript
interface IAdapterRegistry {
  // 어댑터 등록
  register(adapter: IOutputAdapter): void;

  // 어댑터 제거
  unregister(type: OutputType): void;

  // 어댑터 조회
  get(type: OutputType): IOutputAdapter | undefined;

  // 모든 어댑터 조회
  getAll(): IOutputAdapter[];

  // 사용 가능한 어댑터 조회
  getAvailable(): IOutputAdapter[];
}
```

---

## 4. TTS Adapter

### 4.1 TTS Interface

```typescript
interface ITTSAdapter extends IOutputAdapter {
  type: 'tts';

  // TTS 전용 메서드
  getVoices(): Promise<Voice[]>;
  setVoice(voiceId: string): void;
  getVoice(): Voice | undefined;

  // 재생 제어
  speak(text: string, options?: TTSOptions): Promise<void>;
  pause(): void;
  resume(): void;
  stop(): void;

  // 상태
  isSpeaking(): boolean;
  isPaused(): boolean;

  // 속성
  setRate(rate: number): void;     // 0.1 ~ 10
  setPitch(pitch: number): void;   // 0 ~ 2
  setVolume(volume: number): void; // 0 ~ 1
}

interface Voice {
  id: string;
  name: string;
  language: string;
  gender?: 'male' | 'female' | 'neutral';
  localService?: boolean;
  default?: boolean;
}

interface TTSOptions {
  voice?: string;
  rate?: number;
  pitch?: number;
  volume?: number;
  language?: string;
}
```

### 4.2 Web Speech API Implementation

```typescript
class WebSpeechTTSAdapter implements ITTSAdapter {
  type = 'tts' as const;
  name = 'Web Speech TTS';
  version = '1.0.0';

  private synth: SpeechSynthesis;
  private voices: Voice[] = [];
  private currentVoice?: SpeechSynthesisVoice;
  private rate = 1.0;
  private pitch = 1.0;
  private volume = 1.0;

  async initialize(): Promise<void> {
    this.synth = window.speechSynthesis;
    await this.loadVoices();
  }

  async speak(text: string, options?: TTSOptions): Promise<void> {
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.voice = this.currentVoice ?? null;
    utterance.rate = options?.rate ?? this.rate;
    utterance.pitch = options?.pitch ?? this.pitch;
    utterance.volume = options?.volume ?? this.volume;

    return new Promise((resolve, reject) => {
      utterance.onend = () => resolve();
      utterance.onerror = (e) => reject(e);
      this.synth.speak(utterance);
    });
  }

  // ... 기타 메서드
}
```

---

## 5. Sign Language Adapter

### 5.1 Sign Language Interface

```typescript
interface ISignLanguageAdapter extends IOutputAdapter {
  type: 'sign_language';

  // 텍스트 → ISP 변환
  textToISP(text: string): Promise<ISPCode[]>;

  // 제스처 재생
  playGesture(code: ISPCode): Promise<void>;
  playSequence(codes: ISPCode[]): Promise<void>;

  // 아바타 설정
  getAvatars(): Avatar[];
  setAvatar(avatarId: string): void;
  getAvatar(): Avatar | undefined;

  // 재생 제어
  pause(): void;
  resume(): void;
  stop(): void;

  // 속도 설정
  setSpeed(speed: number): void; // 0.5 ~ 2.0
}

interface ISPCode {
  code: string;           // "HS01-LC07-MV10-OR02-NM15"
  meaning?: string;       // "안녕" (선택)
  duration?: number;      // 밀리초
  metadata?: {
    handshape?: string;
    location?: string;
    movement?: string;
    orientation?: string;
    nonManual?: string;
  };
}

interface Avatar {
  id: string;
  name: string;
  style: 'realistic' | 'cartoon' | 'simple';
  preview?: string;
}
```

### 5.2 ISP Code Format (WIA Talk Compatible)

```
ISP Code: HS01-LC07-MV10-OR02-NM15

HS: Handshape (손 모양)
LC: Location (위치)
MV: Movement (움직임)
OR: Orientation (방향)
NM: Non-Manual (비수지 신호)
```

---

## 6. Braille Adapter

### 6.1 Braille Interface

```typescript
interface IBrailleAdapter extends IOutputAdapter {
  type: 'braille';

  // 텍스트 → 점자 변환
  textToIPA(text: string, language?: string): Promise<string>;
  textToBraille(text: string, language?: string): Promise<BrailleOutput>;

  // 디스플레이 관리
  getDisplays(): Promise<BrailleDisplay[]>;
  setDisplay(displayId: string): void;
  getDisplay(): BrailleDisplay | undefined;

  // 점자 전송
  sendToDisplay(braille: BrailleOutput): Promise<void>;

  // 페이징
  nextPage(): void;
  previousPage(): void;
  setPage(page: number): void;
}

interface BrailleOutput {
  original: string;       // 원본 텍스트
  ipa: string;            // IPA 발음 기호
  braille: string;        // 점자 문자 (유니코드)
  unicode: string[];      // 유니코드 코드포인트
  cells: number;          // 점자 셀 수
  grade: 1 | 2;           // 점자 등급
}

interface BrailleDisplay {
  id: string;
  name: string;
  manufacturer?: string;
  cells: number;          // 점자 셀 수 (20, 40, 80)
  rows: number;           // 행 수 (보통 1)
  connected: boolean;
  battery?: number;       // 배터리 (0-100)
}
```

### 6.2 Braille Grade

| Grade | 설명 | 예시 |
|-------|------|------|
| **Grade 1** | 1:1 문자 매핑 | "hello" → ⠓⠑⠇⠇⠕ |
| **Grade 2** | 축약형 사용 | "the" → ⠮ |

---

## 7. Neurofeedback Adapter

### 7.1 Neurofeedback Interface

```typescript
interface INeurofeedbackAdapter extends IOutputAdapter {
  type: 'neurofeedback';

  // 시각화 모드 설정
  setVisualization(mode: VisualizationMode): void;
  getVisualization(): VisualizationMode;

  // 실시간 업데이트
  updateBandPowers(powers: BandPowers): void;
  updateTopography(channels: ChannelData[]): void;
  updateClassification(result: ClassificationResult): void;

  // 커서 제어
  updateCursor(position: CursorPosition): void;

  // 캔버스/렌더러 설정
  setCanvas(canvas: HTMLCanvasElement): void;
  setRenderer(renderer: 'canvas' | 'webgl' | 'svg'): void;

  // 테마
  setTheme(theme: NeurofeedbackTheme): void;
}

type VisualizationMode =
  | 'band_powers'     // 주파수 대역 바 차트
  | 'topography'      // 두피 활성화 맵
  | 'time_series'     // 시계열 그래프
  | 'spectrogram'     // 스펙트로그램
  | 'classification'  // 분류 결과
  | 'cursor'          // 커서 제어
  | 'combined';       // 복합 뷰

interface ChannelData {
  channel: string;    // "Fp1", "C3", etc.
  value: number;      // 정규화된 값 (0-1)
  quality?: number;   // 신호 품질
}

interface CursorPosition {
  x: number;          // -1 ~ 1
  y: number;          // -1 ~ 1
  click?: boolean;    // 클릭 상태
}

interface NeurofeedbackTheme {
  background: string;
  foreground: string;
  positive: string;   // 좋은 상태
  negative: string;   // 나쁜 상태
  neutral: string;    // 중립 상태
}
```

### 7.2 Band Power Visualization

```typescript
interface BandPowerVisualization {
  // 막대 차트 설정
  bars: {
    delta: BarConfig;
    theta: BarConfig;
    alpha: BarConfig;
    beta: BarConfig;
    gamma: BarConfig;
  };

  // 임계값 표시
  thresholds?: {
    relaxed: number;    // 알파 임계값
    focused: number;    // 베타 임계값
  };
}

interface BarConfig {
  color: string;
  label: string;
  min: number;
  max: number;
}
```

---

## 8. Output Manager

### 8.1 Manager Interface

```typescript
interface IOutputManager {
  // 어댑터 관리
  register(adapter: IOutputAdapter): void;
  unregister(type: OutputType): void;
  get(type: OutputType): IOutputAdapter | undefined;
  getAll(): IOutputAdapter[];

  // 초기화
  initialize(options?: ManagerOptions): Promise<void>;

  // 출력
  output(content: OutputContent, targets?: OutputType[]): Promise<void>;
  broadcast(content: OutputContent): Promise<void>;

  // 사용자 선호도
  setPreferences(prefs: OutputPreferences): void;
  getPreferences(): OutputPreferences;

  // 이벤트
  on(event: ManagerEvent, handler: ManagerEventHandler): void;
  off(event: ManagerEvent, handler: ManagerEventHandler): void;

  // 정리
  dispose(): Promise<void>;
}

interface ManagerOptions {
  autoInitialize?: boolean;
  defaultOutputs?: OutputType[];
  preferences?: OutputPreferences;
}

interface OutputPreferences {
  primaryOutput: OutputType;
  enabledOutputs: OutputType[];
  language: string;
  tts?: TTSPreferences;
  signLanguage?: SignLanguagePreferences;
  braille?: BraillePreferences;
  neurofeedback?: NeurofeedbackPreferences;
}
```

### 8.2 Manager Implementation

```typescript
class OutputManager implements IOutputManager {
  private adapters: Map<OutputType, IOutputAdapter> = new Map();
  private preferences: OutputPreferences;
  private eventEmitter: EventEmitter;

  async initialize(options?: ManagerOptions): Promise<void> {
    // 기본 어댑터 등록
    this.register(new WebSpeechTTSAdapter());
    this.register(new MockSignLanguageAdapter());
    this.register(new MockBrailleAdapter());
    this.register(new CanvasNeurofeedbackAdapter());

    // 모든 어댑터 초기화
    await Promise.all(
      Array.from(this.adapters.values())
        .map(adapter => adapter.initialize())
    );
  }

  async output(content: OutputContent, targets?: OutputType[]): Promise<void> {
    const outputTypes = targets ?? this.preferences.enabledOutputs;

    await Promise.all(
      outputTypes.map(async (type) => {
        const adapter = this.adapters.get(type);
        if (adapter?.isAvailable()) {
          await adapter.output(content);
        }
      })
    );
  }

  async broadcast(content: OutputContent): Promise<void> {
    await this.output(content, this.preferences.enabledOutputs);
  }
}
```

---

## 9. Event System

### 9.1 Output Events

```typescript
interface OutputEventData {
  type: OutputEvent;
  adapter: OutputType;
  timestamp: number;
  content?: OutputContent;
  error?: Error;
}

// 이벤트 유형
type OutputEvent =
  | 'output:start'    // 출력 시작
  | 'output:end'      // 출력 완료
  | 'output:error'    // 출력 에러
  | 'adapter:ready'   // 어댑터 준비 완료
  | 'adapter:busy'    // 어댑터 사용 중
  | 'adapter:error';  // 어댑터 에러
```

### 9.2 Event Flow

```
Content Input
     │
     ▼
OutputManager.output()
     │
     ├─────────────────────────────────────┐
     ▼                                     ▼
TTSAdapter.output()              SignAdapter.output()
     │                                     │
     ├── emit('output:start')              ├── emit('output:start')
     │                                     │
     ├── [Processing]                      ├── [Processing]
     │                                     │
     ├── emit('output:end')                ├── emit('output:end')
     │                                     │
     └─────────────────────────────────────┘
                    │
                    ▼
            Manager.emit('output:complete')
```

---

## 10. Error Handling

### 10.1 Error Types

```typescript
class OutputError extends Error {
  constructor(
    public readonly code: OutputErrorCode,
    message: string,
    public readonly adapter?: OutputType,
    public readonly cause?: Error
  ) {
    super(message);
    this.name = 'OutputError';
  }
}

enum OutputErrorCode {
  // 어댑터 에러 (1xxx)
  ADAPTER_NOT_FOUND = 1001,
  ADAPTER_NOT_READY = 1002,
  ADAPTER_BUSY = 1003,
  ADAPTER_INIT_FAILED = 1004,

  // TTS 에러 (2xxx)
  TTS_NOT_SUPPORTED = 2001,
  TTS_VOICE_NOT_FOUND = 2002,
  TTS_SYNTHESIS_FAILED = 2003,

  // 수어 에러 (3xxx)
  SIGN_CONVERSION_FAILED = 3001,
  SIGN_AVATAR_NOT_FOUND = 3002,
  SIGN_ANIMATION_FAILED = 3003,

  // 점자 에러 (4xxx)
  BRAILLE_DISPLAY_NOT_FOUND = 4001,
  BRAILLE_CONVERSION_FAILED = 4002,
  BRAILLE_SEND_FAILED = 4003,

  // 뉴로피드백 에러 (5xxx)
  NEURO_CANVAS_NOT_SET = 5001,
  NEURO_RENDER_FAILED = 5002,
}
```

### 10.2 Error Recovery

```typescript
interface ErrorRecoveryStrategy {
  // 재시도 가능 여부
  canRetry(error: OutputError): boolean;

  // 대체 출력 가능 여부
  canFallback(error: OutputError): boolean;

  // 대체 어댑터 반환
  getFallbackAdapter(error: OutputError): OutputType | undefined;
}
```

---

## 11. Configuration

### 11.1 Default Configuration

```typescript
const DEFAULT_CONFIG: OutputManagerConfig = {
  // 기본 어댑터
  adapters: {
    tts: {
      enabled: true,
      implementation: 'web-speech',
      options: {
        rate: 1.0,
        pitch: 1.0,
        volume: 1.0,
      },
    },
    sign_language: {
      enabled: false,
      implementation: 'mock',
      options: {
        speed: 1.0,
        avatar: 'default',
      },
    },
    braille: {
      enabled: false,
      implementation: 'mock',
      options: {
        grade: 2,
      },
    },
    neurofeedback: {
      enabled: true,
      implementation: 'canvas',
      options: {
        visualization: 'band_powers',
        theme: 'dark',
      },
    },
  },

  // 전역 설정
  language: 'ko',
  autoInitialize: true,
  errorRecovery: true,
};
```

---

## 12. Examples

### 12.1 Basic Usage

```typescript
import { OutputManager, OutputContent } from 'wia-bci';

// 매니저 생성 및 초기화
const manager = new OutputManager();
await manager.initialize();

// 텍스트 출력 (TTS로)
await manager.output({
  type: 'text',
  text: '안녕하세요',
});

// 분류 결과 출력 (뉴로피드백으로)
await manager.output({
  type: 'classification',
  classification: {
    classId: 1,
    className: 'left_hand',
    confidence: 0.85,
  },
});
```

### 12.2 Multimodal Output

```typescript
// 멀티모달 출력 (TTS + 수어 동시)
await manager.output(
  {
    type: 'text',
    text: '물을 주세요',
  },
  ['tts', 'sign_language']
);
```

### 12.3 Full BCI Pipeline

```typescript
import { WiaBci, OutputManager } from 'wia-bci';

const bci = new WiaBci();
const output = new OutputManager();

await bci.connect({ type: 'simulator' });
await output.initialize();

// 분류 결과를 출력으로 연결
bci.on('classification', async (event) => {
  // 텍스트 출력
  if (event.className === 'yes') {
    await output.output({ type: 'text', text: '네' });
  } else if (event.className === 'no') {
    await output.output({ type: 'text', text: '아니오' });
  }

  // 뉴로피드백 표시
  await output.output({
    type: 'classification',
    classification: event,
  });
});

// 밴드파워를 실시간 시각화
bci.on('signal', async (event) => {
  const powers = SignalProcessor.allBandPowers(event.data, 250);
  await output.output({
    type: 'signal',
    signal: { bandPowers: powers, channels: [] },
  });
});

await bci.startStream();
```

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA BCI Working Group

---

弘益人間 - *Benefit All Humanity*
