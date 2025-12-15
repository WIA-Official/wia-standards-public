# Phase 4: WIA Ecosystem Integration
# WIA 생태계 연동 표준

---

**버전**: 1.0.0
**상태**: Draft
**작성일**: 2025-12-13
**작성자**: WIA / SmileStory Inc.

---

## 목차

1. [개요](#1-개요)
2. [출력 계층 아키텍처](#2-출력-계층-아키텍처)
3. [출력 인터페이스](#3-출력-인터페이스)
4. [TTS 연동](#4-tts-연동)
5. [ISP/WIA Talk 연동](#5-ispwia-talk-연동)
6. [WIA Braille 연동](#6-wia-braille-연동)
7. [통합 출력 매니저](#7-통합-출력-매니저)
8. [이벤트 및 콜백](#8-이벤트-및-콜백)
9. [에러 처리](#9-에러-처리)
10. [예제](#10-예제)
11. [참고문헌](#11-참고문헌)

---

## 1. 개요

### 1.1 목적

Phase 4는 WIA AAC Standard의 마지막 단계로, AAC 센서 입력을 통해 생성된 텍스트를
다양한 출력 방식으로 변환하여 WIA 생태계와 연동하는 표준을 정의합니다.

### 1.2 범위

```
입력: Phase 1-3을 통해 생성된 텍스트
출력:
├─ TTS (Text-to-Speech) → 음성
├─ ISP/WIA Talk → 수어 아바타
└─ WIA Braille → 점자
```

### 1.3 이전 Phase와의 관계

| Phase | 역할 | Phase 4 연동 |
|-------|------|-------------|
| Phase 1 | Signal Format | 센서 입력 표준화 |
| Phase 2 | API Interface | 텍스트 생성 API |
| Phase 3 | Protocol | 메시지 전송 |
| **Phase 4** | **Integration** | **출력 연동** |

---

## 2. 출력 계층 아키텍처

### 2.1 전체 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│                     AAC 사용자                               │
│              (ALS, 뇌성마비, 사지마비 등)                    │
└─────────────────────────────────────────────────────────────┘
                              │
                         [센서 입력]
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│     Phase 1: Signal Format → Phase 2: API → Phase 3: Protocol│
└─────────────────────────────────────────────────────────────┘
                              │
                         [텍스트 출력]
                        "안녕하세요"
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      OutputManager                           │
│                   (Phase 4: Integration)                     │
├──────────────────┬──────────────────┬───────────────────────┤
│   TTSAdapter     │ SignLanguageAdapter │   BrailleAdapter    │
│   (음성 출력)     │    (수어 아바타)    │    (점자 출력)      │
└────────┬─────────┴────────┬─────────┴──────────┬────────────┘
         │                  │                    │
         ▼                  ▼                    ▼
    ┌─────────┐       ┌─────────┐          ┌─────────┐
    │ 스피커  │       │ 디스플레이│          │점자 디스플레이│
    │         │       │ (아바타) │          │         │
    └─────────┘       └─────────┘          └─────────┘
         │                  │                    │
         ▼                  ▼                    ▼
    비장애인            청각장애인            시각장애인
```

### 2.2 계층 구조

```
Layer 0: 입력 (Input)
         └─ 텍스트 문자열

Layer 1: 출력 관리 (Output Management)
         └─ OutputManager

Layer 2: 출력 어댑터 (Output Adapters)
         ├─ TTSAdapter
         ├─ SignLanguageAdapter
         └─ BrailleAdapter

Layer 3: 출력 장치 (Output Devices)
         ├─ Web Speech API / Cloud TTS
         ├─ 3D Avatar / ISP Engine
         └─ Braille Display / BrlAPI
```

---

## 3. 출력 인터페이스

### 3.1 기본 인터페이스

#### TypeScript 정의

```typescript
/**
 * 출력 유형
 */
type OutputType = 'tts' | 'sign_language' | 'braille' | 'custom';

/**
 * 출력 상태
 */
type OutputState = 'idle' | 'outputting' | 'paused' | 'error';

/**
 * 출력 옵션
 */
interface OutputOptions {
  /** 언어 코드 (ko, en, ja, ...) */
  language?: string;

  /** 음성 ID (TTS용) */
  voice?: string;

  /** 출력 속도 (0.5 ~ 2.0) */
  speed?: number;

  /** 볼륨 (0.0 ~ 1.0) */
  volume?: number;

  /** 추가 옵션 */
  [key: string]: unknown;
}

/**
 * 출력 어댑터 인터페이스
 */
interface IOutputAdapter {
  /** 출력 유형 */
  readonly type: OutputType;

  /** 어댑터 이름 */
  readonly name: string;

  /** 현재 상태 */
  readonly state: OutputState;

  /**
   * 어댑터 초기화
   */
  initialize(options?: OutputOptions): Promise<void>;

  /**
   * 텍스트 출력
   */
  output(text: string, options?: OutputOptions): Promise<void>;

  /**
   * 출력 중지
   */
  stop(): void;

  /**
   * 사용 가능 여부
   */
  isAvailable(): boolean;

  /**
   * 리소스 정리
   */
  dispose(): Promise<void>;

  /**
   * 이벤트 핸들러 등록
   */
  on(event: OutputEventType, handler: OutputEventHandler): void;

  /**
   * 이벤트 핸들러 해제
   */
  off(event: OutputEventType, handler: OutputEventHandler): void;
}
```

#### Python 정의

```python
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Any, Callable, Dict

class OutputType(str, Enum):
    TTS = "tts"
    SIGN_LANGUAGE = "sign_language"
    BRAILLE = "braille"
    CUSTOM = "custom"

class OutputState(str, Enum):
    IDLE = "idle"
    OUTPUTTING = "outputting"
    PAUSED = "paused"
    ERROR = "error"

class OutputOptions:
    def __init__(
        self,
        language: Optional[str] = None,
        voice: Optional[str] = None,
        speed: float = 1.0,
        volume: float = 1.0,
        **kwargs
    ):
        self.language = language
        self.voice = voice
        self.speed = speed
        self.volume = volume
        self.extra = kwargs

class IOutputAdapter(ABC):
    """출력 어댑터 추상 클래스"""

    @property
    @abstractmethod
    def type(self) -> OutputType:
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @property
    @abstractmethod
    def state(self) -> OutputState:
        pass

    @abstractmethod
    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        pass

    @abstractmethod
    async def output(self, text: str, options: Optional[OutputOptions] = None) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass

    @abstractmethod
    def is_available(self) -> bool:
        pass

    @abstractmethod
    async def dispose(self) -> None:
        pass
```

---

## 4. TTS 연동

### 4.1 개요

TTS (Text-to-Speech) 어댑터는 텍스트를 음성으로 변환합니다.

지원 백엔드:
- **Web Speech API**: 브라우저 내장 (권장)
- **Cloud TTS**: Google/Amazon/Azure (선택적)

### 4.2 인터페이스

```typescript
interface ITTSAdapter extends IOutputAdapter {
  type: 'tts';

  /**
   * 사용 가능한 음성 목록 조회
   */
  getVoices(): Promise<Voice[]>;

  /**
   * 음성 설정
   */
  setVoice(voiceId: string): void;

  /**
   * 일시 정지
   */
  pause(): void;

  /**
   * 재개
   */
  resume(): void;
}

interface Voice {
  /** 음성 ID */
  id: string;

  /** 음성 이름 */
  name: string;

  /** 언어 코드 */
  language: string;

  /** 성별 */
  gender?: 'male' | 'female' | 'neutral';

  /** 로컬 음성 여부 */
  local?: boolean;
}
```

### 4.3 Web Speech API 구현

```typescript
class WebSpeechTTSAdapter implements ITTSAdapter {
  readonly type = 'tts';
  readonly name = 'WebSpeechTTS';

  private synth: SpeechSynthesis;
  private currentUtterance: SpeechSynthesisUtterance | null = null;
  private _state: OutputState = 'idle';
  private selectedVoice: SpeechSynthesisVoice | null = null;

  get state(): OutputState {
    return this._state;
  }

  async initialize(): Promise<void> {
    if (typeof window === 'undefined' || !window.speechSynthesis) {
      throw new Error('Web Speech API not available');
    }
    this.synth = window.speechSynthesis;
  }

  async getVoices(): Promise<Voice[]> {
    const voices = this.synth.getVoices();
    return voices.map(v => ({
      id: v.voiceURI,
      name: v.name,
      language: v.lang,
      local: v.localService
    }));
  }

  async output(text: string, options?: OutputOptions): Promise<void> {
    return new Promise((resolve, reject) => {
      const utterance = new SpeechSynthesisUtterance(text);

      if (options?.language) utterance.lang = options.language;
      if (options?.speed) utterance.rate = options.speed;
      if (options?.volume) utterance.volume = options.volume;
      if (this.selectedVoice) utterance.voice = this.selectedVoice;

      utterance.onstart = () => { this._state = 'outputting'; };
      utterance.onend = () => { this._state = 'idle'; resolve(); };
      utterance.onerror = (e) => { this._state = 'error'; reject(e); };

      this.currentUtterance = utterance;
      this.synth.speak(utterance);
    });
  }

  stop(): void {
    this.synth.cancel();
    this._state = 'idle';
  }

  pause(): void {
    this.synth.pause();
    this._state = 'paused';
  }

  resume(): void {
    this.synth.resume();
    this._state = 'outputting';
  }

  isAvailable(): boolean {
    return typeof window !== 'undefined' && !!window.speechSynthesis;
  }

  async dispose(): Promise<void> {
    this.stop();
  }
}
```

---

## 5. ISP/WIA Talk 연동

### 5.1 개요

ISP (International Sign Phonology) 어댑터는 텍스트를 수어 제스처로 변환합니다.

변환 과정:
```
텍스트 → 단어 분리 → ISP 코드 매핑 → 제스처 파라미터 → 아바타 애니메이션
```

### 5.2 ISP 코드 체계

```
코드 형식: HS##-LC##-MV##-OR##-NM##

HS (Handshape): 손 모양 (01-99)
LC (Location): 위치 (01-50)
MV (Movement): 움직임 (01-99)
OR (Orientation): 방향 (01-20)
NM (Non-manual): 비수지 신호 (01-50)

예시:
"안녕" → HS01-LC01-MV01-OR01-NM01
"사랑" → HS09-LC07-MV10-OR02-NM01
```

### 5.3 인터페이스

```typescript
interface ISignLanguageAdapter extends IOutputAdapter {
  type: 'sign_language';

  /**
   * 텍스트를 ISP 코드로 변환
   */
  textToISP(text: string): Promise<ISPCode[]>;

  /**
   * 단일 제스처 재생
   */
  playGesture(ispCode: ISPCode): Promise<void>;

  /**
   * 연속 제스처 재생
   */
  playSequence(ispCodes: ISPCode[]): Promise<void>;

  /**
   * 아바타 설정
   */
  setAvatar(avatarId: string): void;

  /**
   * 재생 속도 설정
   */
  setSpeed(speed: number): void;
}

interface ISPCode {
  /** ISP 코드 (예: "HS01-LC07-MV10-OR02-NM15") */
  code: string;

  /** 의미 (선택) */
  meaning?: string;

  /** 지속 시간 (밀리초) */
  duration?: number;

  /** 구성 요소 */
  components?: {
    handshape: string;    // HS##
    location: string;     // LC##
    movement: string;     // MV##
    orientation: string;  // OR##
    nonManual: string;    // NM##
  };
}
```

### 5.4 Mock 구현

```typescript
class MockSignLanguageAdapter implements ISignLanguageAdapter {
  readonly type = 'sign_language';
  readonly name = 'MockSignLanguage';

  private _state: OutputState = 'idle';
  private ispDictionary: Map<string, ISPCode> = new Map();
  private speed: number = 1.0;

  get state(): OutputState {
    return this._state;
  }

  async initialize(): Promise<void> {
    // 기본 ISP 사전 로드
    this.loadDefaultDictionary();
  }

  private loadDefaultDictionary(): void {
    // 기본 수어 매핑
    this.ispDictionary.set('안녕', {
      code: 'HS01-LC01-MV01-OR01-NM01',
      meaning: '안녕',
      duration: 1000
    });
    this.ispDictionary.set('감사', {
      code: 'HS02-LC07-MV02-OR02-NM02',
      meaning: '감사',
      duration: 1200
    });
    // ... 추가 매핑
  }

  async textToISP(text: string): Promise<ISPCode[]> {
    const words = text.split(/\s+/);
    const codes: ISPCode[] = [];

    for (const word of words) {
      const code = this.ispDictionary.get(word);
      if (code) {
        codes.push(code);
      } else {
        // 사전에 없는 단어는 기본 코드
        codes.push({
          code: 'HS00-LC00-MV00-OR00-NM00',
          meaning: word,
          duration: 800
        });
      }
    }

    return codes;
  }

  async output(text: string): Promise<void> {
    const codes = await this.textToISP(text);
    await this.playSequence(codes);
  }

  async playGesture(ispCode: ISPCode): Promise<void> {
    this._state = 'outputting';
    const duration = (ispCode.duration || 1000) / this.speed;

    // Mock: 실제로는 아바타 애니메이션
    console.log(`Playing: ${ispCode.code} (${ispCode.meaning})`);

    await new Promise(resolve => setTimeout(resolve, duration));
    this._state = 'idle';
  }

  async playSequence(ispCodes: ISPCode[]): Promise<void> {
    for (const code of ispCodes) {
      await this.playGesture(code);
    }
  }

  stop(): void {
    this._state = 'idle';
  }

  isAvailable(): boolean {
    return true;
  }

  async dispose(): Promise<void> {
    this.ispDictionary.clear();
  }
}
```

---

## 6. WIA Braille 연동

### 6.1 개요

WIA Braille 어댑터는 텍스트를 점자로 변환합니다.

변환 과정:
```
텍스트 → IPA 변환 → 점자 매핑 → 점자 디스플레이 출력
```

### 6.2 IPA 기반 점자 체계

```
입력: "안녕하세요"
IPA: /annjʌŋhasejo/
점자: ⠁⠝⠚⠪⠝⠓⠁⠎⠑⠚⠕
```

### 6.3 인터페이스

```typescript
interface IBrailleAdapter extends IOutputAdapter {
  type: 'braille';

  /**
   * 텍스트를 IPA로 변환
   */
  textToIPA(text: string): Promise<string>;

  /**
   * 텍스트를 점자로 변환
   */
  textToBraille(text: string): Promise<BrailleOutput>;

  /**
   * 점자 디스플레이로 전송
   */
  sendToDisplay(braille: BrailleOutput): Promise<void>;

  /**
   * 연결된 디스플레이 목록
   */
  getConnectedDisplays(): Promise<BrailleDisplay[]>;

  /**
   * 디스플레이 선택
   */
  setDisplay(displayId: string): void;
}

interface BrailleOutput {
  /** 원본 텍스트 */
  text: string;

  /** IPA 표기 */
  ipa: string;

  /** 점자 유니코드 문자열 */
  braille: string;

  /** 점자 유니코드 배열 */
  unicode: string[];

  /** 점자 도트 패턴 (8점) */
  dots: number[];
}

interface BrailleDisplay {
  /** 디스플레이 ID */
  id: string;

  /** 디스플레이 이름 */
  name: string;

  /** 점자 셀 수 */
  cells: number;

  /** 연결 상태 */
  connected: boolean;
}
```

### 6.4 Mock 구현

```typescript
class MockBrailleAdapter implements IBrailleAdapter {
  readonly type = 'braille';
  readonly name = 'MockBraille';

  private _state: OutputState = 'idle';
  private ipaMap: Map<string, string> = new Map();
  private brailleMap: Map<string, string> = new Map();

  get state(): OutputState {
    return this._state;
  }

  async initialize(): Promise<void> {
    this.loadIPAMap();
    this.loadBrailleMap();
  }

  private loadIPAMap(): void {
    // 간단한 한글 → IPA 매핑 (실제로는 G2P 라이브러리 사용)
    this.ipaMap.set('안녕', '/annjʌŋ/');
    this.ipaMap.set('감사', '/kamsa/');
    this.ipaMap.set('사랑', '/saraŋ/');
  }

  private loadBrailleMap(): void {
    // IPA → 점자 매핑 (WIA Braille 체계)
    this.brailleMap.set('a', '⠁');
    this.brailleMap.set('n', '⠝');
    this.brailleMap.set('j', '⠚');
    this.brailleMap.set('ʌ', '⠪');
    this.brailleMap.set('ŋ', '⠻');
    this.brailleMap.set('k', '⠅');
    this.brailleMap.set('m', '⠍');
    this.brailleMap.set('s', '⠎');
    this.brailleMap.set('r', '⠗');
    // ... 추가 매핑
  }

  async textToIPA(text: string): Promise<string> {
    return this.ipaMap.get(text) || `/${text}/`;
  }

  async textToBraille(text: string): Promise<BrailleOutput> {
    const ipa = await this.textToIPA(text);
    const ipaChars = ipa.replace(/[\/]/g, '').split('');

    let braille = '';
    const unicode: string[] = [];
    const dots: number[] = [];

    for (const char of ipaChars) {
      const b = this.brailleMap.get(char) || '⠀';
      braille += b;
      unicode.push(`U+${b.charCodeAt(0).toString(16).toUpperCase()}`);
      dots.push(b.charCodeAt(0) - 0x2800);
    }

    return {
      text,
      ipa,
      braille,
      unicode,
      dots
    };
  }

  async output(text: string): Promise<void> {
    this._state = 'outputting';
    const brailleOutput = await this.textToBraille(text);
    await this.sendToDisplay(brailleOutput);
    this._state = 'idle';
  }

  async sendToDisplay(braille: BrailleOutput): Promise<void> {
    // Mock: 실제로는 BrlAPI 또는 하드웨어 연동
    console.log(`Braille: ${braille.braille}`);
    console.log(`IPA: ${braille.ipa}`);
  }

  async getConnectedDisplays(): Promise<BrailleDisplay[]> {
    // Mock: 가상 디스플레이
    return [{
      id: 'mock-display-1',
      name: 'Mock Braille Display 40',
      cells: 40,
      connected: true
    }];
  }

  stop(): void {
    this._state = 'idle';
  }

  isAvailable(): boolean {
    return true;
  }

  async dispose(): Promise<void> {
    this.ipaMap.clear();
    this.brailleMap.clear();
  }
}
```

---

## 7. 통합 출력 매니저

### 7.1 개요

OutputManager는 여러 출력 어댑터를 통합 관리합니다.

### 7.2 인터페이스

```typescript
interface IOutputManager {
  /**
   * 어댑터 등록
   */
  register(adapter: IOutputAdapter): void;

  /**
   * 어댑터 제거
   */
  unregister(type: OutputType): void;

  /**
   * 특정 어댑터 조회
   */
  getAdapter<T extends IOutputAdapter>(type: OutputType): T | undefined;

  /**
   * 활성 어댑터 목록
   */
  getActiveAdapters(): IOutputAdapter[];

  /**
   * 모든 활성 어댑터로 출력 (브로드캐스트)
   */
  broadcast(text: string, options?: OutputOptions): Promise<void>;

  /**
   * 특정 어댑터로 출력
   */
  outputTo(type: OutputType, text: string, options?: OutputOptions): Promise<void>;

  /**
   * 모든 출력 중지
   */
  stopAll(): void;

  /**
   * 리소스 정리
   */
  dispose(): Promise<void>;
}
```

### 7.3 구현

```typescript
class OutputManager implements IOutputManager {
  private adapters: Map<OutputType, IOutputAdapter> = new Map();
  private emitter: EventEmitter;

  constructor() {
    this.emitter = new EventEmitter();
  }

  register(adapter: IOutputAdapter): void {
    this.adapters.set(adapter.type, adapter);
    this.emitter.emit('adapterRegistered', { type: adapter.type });
  }

  unregister(type: OutputType): void {
    const adapter = this.adapters.get(type);
    if (adapter) {
      adapter.dispose();
      this.adapters.delete(type);
      this.emitter.emit('adapterUnregistered', { type });
    }
  }

  getAdapter<T extends IOutputAdapter>(type: OutputType): T | undefined {
    return this.adapters.get(type) as T | undefined;
  }

  getActiveAdapters(): IOutputAdapter[] {
    return Array.from(this.adapters.values())
      .filter(a => a.isAvailable());
  }

  async broadcast(text: string, options?: OutputOptions): Promise<void> {
    const activeAdapters = this.getActiveAdapters();

    await Promise.all(
      activeAdapters.map(adapter =>
        adapter.output(text, options).catch(err => {
          this.emitter.emit('error', { adapter: adapter.type, error: err });
        })
      )
    );
  }

  async outputTo(
    type: OutputType,
    text: string,
    options?: OutputOptions
  ): Promise<void> {
    const adapter = this.adapters.get(type);

    if (!adapter) {
      throw new Error(`Adapter not found: ${type}`);
    }

    if (!adapter.isAvailable()) {
      throw new Error(`Adapter not available: ${type}`);
    }

    await adapter.output(text, options);
  }

  stopAll(): void {
    for (const adapter of this.adapters.values()) {
      adapter.stop();
    }
  }

  async dispose(): Promise<void> {
    for (const adapter of this.adapters.values()) {
      await adapter.dispose();
    }
    this.adapters.clear();
  }

  on(event: string, handler: (...args: any[]) => void): void {
    this.emitter.on(event, handler);
  }

  off(event: string, handler: (...args: any[]) => void): void {
    this.emitter.off(event, handler);
  }
}
```

---

## 8. 이벤트 및 콜백

### 8.1 이벤트 유형

```typescript
type OutputEventType =
  | 'start'          // 출력 시작
  | 'end'            // 출력 완료
  | 'pause'          // 일시 정지
  | 'resume'         // 재개
  | 'progress'       // 진행 상황
  | 'error';         // 에러

interface OutputEvent {
  /** 이벤트 유형 */
  type: OutputEventType;

  /** 어댑터 유형 */
  adapter: OutputType;

  /** 타임스탬프 */
  timestamp: number;

  /** 추가 데이터 */
  data?: any;
}

type OutputEventHandler = (event: OutputEvent) => void;
```

### 8.2 이벤트 사용 예시

```typescript
const manager = new OutputManager();
const tts = new WebSpeechTTSAdapter();

await tts.initialize();
manager.register(tts);

// 이벤트 리스너 등록
tts.on('start', (e) => console.log('TTS started'));
tts.on('end', (e) => console.log('TTS completed'));
tts.on('error', (e) => console.error('TTS error:', e.data));

// 출력
await manager.outputTo('tts', '안녕하세요');
```

---

## 9. 에러 처리

### 9.1 에러 코드

```typescript
enum OutputErrorCode {
  // 초기화 에러 (1xxx)
  INIT_FAILED = 1001,
  NOT_AVAILABLE = 1002,
  ALREADY_INITIALIZED = 1003,

  // 출력 에러 (2xxx)
  OUTPUT_FAILED = 2001,
  OUTPUT_CANCELLED = 2002,
  OUTPUT_TIMEOUT = 2003,

  // 어댑터 에러 (3xxx)
  ADAPTER_NOT_FOUND = 3001,
  ADAPTER_NOT_READY = 3002,
  ADAPTER_BUSY = 3003,

  // 변환 에러 (4xxx)
  CONVERSION_FAILED = 4001,
  INVALID_INPUT = 4002,
  MAPPING_NOT_FOUND = 4003,

  // 장치 에러 (5xxx)
  DEVICE_NOT_CONNECTED = 5001,
  DEVICE_ERROR = 5002
}

class OutputError extends Error {
  constructor(
    public code: OutputErrorCode,
    message: string,
    public recoverable: boolean = true,
    public details?: any
  ) {
    super(message);
    this.name = 'OutputError';
  }
}
```

### 9.2 에러 처리 예시

```typescript
try {
  await manager.outputTo('tts', text);
} catch (error) {
  if (error instanceof OutputError) {
    switch (error.code) {
      case OutputErrorCode.NOT_AVAILABLE:
        // 대체 어댑터 시도
        await manager.outputTo('braille', text);
        break;
      case OutputErrorCode.OUTPUT_TIMEOUT:
        // 재시도
        await manager.outputTo('tts', text);
        break;
      default:
        console.error('Output error:', error.message);
    }
  }
}
```

---

## 10. 예제

### 10.1 전체 AAC 파이프라인 예제

```typescript
import { WiaAac, MockAdapter } from 'wia-aac';
import {
  OutputManager,
  WebSpeechTTSAdapter,
  MockSignLanguageAdapter,
  MockBrailleAdapter
} from 'wia-aac/output';

async function main() {
  // 1. AAC 초기화 (Phase 1-3)
  const aac = new WiaAac({
    sensorType: 'eye_tracker',
    autoConnect: true
  });

  // 2. 출력 매니저 초기화 (Phase 4)
  const output = new OutputManager();

  // TTS 어댑터
  const tts = new WebSpeechTTSAdapter();
  await tts.initialize();
  output.register(tts);

  // 수어 어댑터
  const signLanguage = new MockSignLanguageAdapter();
  await signLanguage.initialize();
  output.register(signLanguage);

  // 점자 어댑터
  const braille = new MockBrailleAdapter();
  await braille.initialize();
  output.register(braille);

  // 3. 센서 입력 → 텍스트 → 출력
  aac.on('text', async (text: string) => {
    console.log('Generated text:', text);

    // 모든 어댑터로 브로드캐스트
    await output.broadcast(text, { language: 'ko' });

    // 또는 특정 어댑터만
    // await output.outputTo('tts', text);
  });

  // 4. 테스트: Mock 입력
  aac.emit('text', '안녕하세요');

  // 5. 정리
  await aac.disconnect();
  await output.dispose();
}

main().catch(console.error);
```

### 10.2 TTS 단독 사용 예제

```typescript
const tts = new WebSpeechTTSAdapter();
await tts.initialize();

// 음성 목록 조회
const voices = await tts.getVoices();
console.log('Available voices:', voices);

// 한국어 음성 선택
const koreanVoice = voices.find(v => v.language.startsWith('ko'));
if (koreanVoice) {
  tts.setVoice(koreanVoice.id);
}

// 출력
await tts.output('안녕하세요, WIA AAC 표준입니다.', {
  speed: 0.9,
  volume: 1.0
});

await tts.dispose();
```

### 10.3 ISP 변환 예제

```typescript
const signLanguage = new MockSignLanguageAdapter();
await signLanguage.initialize();

// 텍스트 → ISP 코드 변환
const ispCodes = await signLanguage.textToISP('안녕 감사 사랑');
console.log('ISP Codes:', ispCodes);
// [
//   { code: 'HS01-LC01-MV01-OR01-NM01', meaning: '안녕' },
//   { code: 'HS02-LC07-MV02-OR02-NM02', meaning: '감사' },
//   { code: 'HS09-LC07-MV10-OR02-NM01', meaning: '사랑' }
// ]

// 제스처 재생
await signLanguage.playSequence(ispCodes);

await signLanguage.dispose();
```

### 10.4 점자 변환 예제

```typescript
const braille = new MockBrailleAdapter();
await braille.initialize();

// 텍스트 → 점자 변환
const output = await braille.textToBraille('안녕');
console.log('Braille output:', output);
// {
//   text: '안녕',
//   ipa: '/annjʌŋ/',
//   braille: '⠁⠝⠚⠪⠻',
//   unicode: ['U+2801', 'U+281D', 'U+281A', 'U+282A', 'U+283B'],
//   dots: [1, 29, 26, 42, 59]
// }

// 점자 디스플레이로 전송
await braille.sendToDisplay(output);

await braille.dispose();
```

---

## 11. 참고문헌

### TTS
1. MDN Web Docs: Web Speech API
   - https://developer.mozilla.org/en-US/docs/Web/API/Web_Speech_API
2. W3C Web Speech API Specification
   - https://wicg.github.io/speech-api/

### 수어
3. ISP Scientific Foundation - 내부 문서
4. WIA Talk Scientific Foundation - 내부 문서
5. MMS Player: Open Source Sign Language Animation
   - https://github.com/DFKI-SignLanguage/MMS-Player

### 점자
6. WIA Braille Scientific Foundation - 내부 문서
7. Liblouis: Open Source Braille Translator
   - https://liblouis.io/
8. BRLTTY: Braille TTY
   - https://brltty.app/

---

<div align="center">

**WIA AAC Standard - Phase 4**

WIA Ecosystem Integration

버전 1.0.0 | 2025-12-13

© 2025 SmileStory Inc. / WIA

**弘益人間** - 널리 인간을 이롭게

</div>
