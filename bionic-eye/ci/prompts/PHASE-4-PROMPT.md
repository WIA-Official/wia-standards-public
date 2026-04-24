# Phase 4: CI System Integration & API
## Claude Code 작업 프롬프트

---

**Phase**: 4 of 4
**목표**: 실제 CI 시스템과의 연동 및 API 표준화
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + API 구현 + 데모 앱

---

## 🎯 Phase 4 목표

### 핵심 질문
```
"Phase 1-3에서 옥타브 분석, 검출, 인핸스먼트를 완성했다.

 이제 이것을 실제 CI 시스템과 어떻게 연동할 것인가?

 - CI 프로세서 제조사와 어떻게 협력할 것인가?
 - 독립적인 보조 장치로 만들 수 있는가?
 - API로 공개하여 다른 개발자가 활용하게 할 수 있는가?"
```

### 목표
```
1. CI 시스템 연동 아키텍처 설계
2. 표준 API 정의 (REST + WebSocket)
3. 스마트폰 앱 연동 프로토콜
4. WIA 생태계 통합 (WIA Braille, WIA Talk)
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: CI 연동 가능성 조사

| 주제 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **Cochlear API** | Nucleus 앱 SDK | "Cochlear Nucleus smart app SDK" |
| **MED-EL API** | AudioKey, AudioLink | "MED-EL AudioLink app developer" |
| **AB API** | Naída Link | "Advanced Bionics app connectivity" |
| **Bluetooth LE** | 보청기 표준 | "ASHA Bluetooth hearing aid protocol" |

### 2단계: 보조 기기 가능성 조사

| 기술 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **외부 마이크** | Roger, Remote Mic | "cochlear implant external microphone" |
| **스트리밍 장치** | TV Connector 등 | "CI audio streaming device" |
| **스마트폰 앱** | 음향 처리 앱 | "real-time audio processing app iOS Android" |

### 3단계: 규제 환경 조사

| 주제 | 조사 대상 | 웹서치 키워드 |
|------|----------|--------------|
| **의료기기 규제** | FDA, CE | "cochlear implant software FDA regulation" |
| **보조 소프트웨어** | 규제 범위 | "hearing aid app FDA classification" |
| **데이터 프라이버시** | 청각 데이터 | "hearing data privacy HIPAA" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-4.md`에 다음을 정리:

```markdown
# Phase 4 사전 조사 결과

## 1. CI 제조사 연동 가능성

### Cochlear
- 앱 SDK 공개 여부: [조사 내용]
- 연동 가능성: [분석]

### MED-EL
- AudioLink 사양: [조사 내용]
- 연동 가능성: [분석]

### Advanced Bionics
- 연동 가능성: [분석]

## 2. 독립 보조 장치 가능성

### 외부 마이크 방식
- 장점: [분석]
- 단점: [분석]
- 구현 가능성: [분석]

### 스마트폰 앱 방식
- 장점: [분석]
- 단점: [분석]
- 구현 가능성: [분석]

## 3. 규제 환경

### 의료기기 규제
- 현재 분류: [조사 내용]
- 피해야 할 것: [분석]

### 권장 접근법
- [제안]

## 4. 결론

### 단기 전략 (1-2년)
- [제안]

### 중기 전략 (3-5년)
- [제안]

### 장기 전략 (5년+)
- [제안]
```

---

## 🏗️ 시스템 아키텍처

### 1. 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                   WIA CI Octave Enhancement                  │
│                      System Architecture                     │
└─────────────────────────────────────────────────────────────┘

                    ┌──────────────────┐
                    │   Sound Source   │
                    │   (음원)          │
                    └────────┬─────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    Audio Input Layer                         │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│  │Built-in │  │External │  │Streaming│  │Bluetooth│        │
│  │   Mic   │  │   Mic   │  │  Audio  │  │   A2DP  │        │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │
└───────┼────────────┼────────────┼────────────┼──────────────┘
        │            │            │            │
        └────────────┴─────┬──────┴────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                  WIA CI Processing Engine                    │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │   Phase 1   │  │   Phase 2   │  │   Phase 3   │         │
│  │   Signal    │──│   Octave    │──│Enhancement  │         │
│  │  Analysis   │  │  Detection  │  │  Protocol   │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│                                                             │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                      Output Layer                            │
│                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  Direct CI  │  │  External   │  │    API     │         │
│  │ Integration │  │   Device    │  │   Output   │         │
│  │  (Future)   │  │  (Current)  │  │  (Cloud)   │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    CI User Experience                        │
│                                                             │
│  ┌─────────────────────────────────────────────────┐        │
│  │         Enhanced Audio with Octave Info          │        │
│  │                                                  │        │
│  │    기존: "뭔가 소리가 나는데 무슨 음인지 모름"    │        │
│  │    개선: "A4 (라) 음이구나!"                     │        │
│  └─────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

### 2. API 구조

```
WIA CI Enhancement API
│
├── /api/v1/
│   ├── /analyze                 # 신호 분석
│   │   └── POST /audio          # 오디오 업로드 및 분석
│   │
│   ├── /detect                  # 옥타브 검출
│   │   ├── POST /frame          # 단일 프레임 분석
│   │   └── WebSocket /stream    # 실시간 스트리밍
│   │
│   ├── /enhance                 # 신호 인핸스먼트
│   │   ├── POST /apply          # 인핸스먼트 적용
│   │   └── GET /patterns        # 인핸스먼트 패턴 목록
│   │
│   └── /simulate                # CI 시뮬레이션
│       ├── POST /vocoder        # Vocoder 시뮬레이션
│       └── GET /comparison      # 전후 비교
│
└── /ws/v1/
    └── /realtime                # 실시간 처리 WebSocket
```

### 3. API 엔드포인트 상세

```typescript
// REST API 타입 정의

// POST /api/v1/analyze/audio
interface AnalyzeRequest {
  audio: ArrayBuffer;           // 오디오 데이터
  sampleRate: number;           // 44100, 48000, etc.
  format: 'wav' | 'mp3' | 'pcm';
}

interface AnalyzeResponse {
  duration: number;             // 초
  frames: AnalysisFrame[];
  summary: {
    dominantOctave: number;
    pitchRange: { min: number; max: number };
    voicedRatio: number;
  };
}

// POST /api/v1/detect/frame
interface DetectFrameRequest {
  frame: Float32Array;          // PCM 프레임
  sampleRate: number;
  previousContext?: OctaveResult;
}

interface DetectFrameResponse {
  result: OctaveResult | null;
  processingTime: number;       // ms
}

// POST /api/v1/enhance/apply
interface EnhanceRequest {
  audio: ArrayBuffer;
  strategy: 'temporal_modulation' | 'electrode_pattern' | 'harmonic' | 'auto';
  parameters?: EnhancementParameters;
}

interface EnhanceResponse {
  enhancedAudio: ArrayBuffer;
  appliedStrategy: string;
  enhancements: EnhancementLog[];
}

// WebSocket /ws/v1/realtime
interface RealtimeMessage {
  type: 'audio_frame' | 'octave_result' | 'enhanced_frame' | 'error';
  timestamp: number;
  payload: any;
}
```

### 4. 스마트폰 앱 연동

```typescript
// 스마트폰 앱 SDK

class WiaCIClient {
  private ws: WebSocket;
  private config: ClientConfig;

  constructor(config: ClientConfig) {
    this.config = config;
  }

  /**
   * 실시간 처리 시작
   */
  async startRealtime(): Promise<void> {
    this.ws = new WebSocket('wss://api.wia.live/ws/v1/realtime');

    this.ws.onmessage = (event) => {
      const message: RealtimeMessage = JSON.parse(event.data);
      this.handleMessage(message);
    };
  }

  /**
   * 오디오 프레임 전송
   */
  sendAudioFrame(frame: Float32Array): void {
    this.ws.send(JSON.stringify({
      type: 'audio_frame',
      timestamp: Date.now(),
      payload: {
        data: Array.from(frame),
        sampleRate: this.config.sampleRate
      }
    }));
  }

  /**
   * 옥타브 정보 수신 콜백
   */
  onOctaveResult(callback: (result: OctaveResult) => void): void {
    this.octaveCallback = callback;
  }

  /**
   * 인핸스된 오디오 수신 콜백
   */
  onEnhancedAudio(callback: (audio: Float32Array) => void): void {
    this.enhancedCallback = callback;
  }
}
```

### 5. WIA 생태계 통합

```
WIA Ecosystem Integration
═════════════════════════

┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   WIA Braille   │     │    WIA CI       │     │   WIA Talk      │
│   (점자 출력)    │     │ (옥타브 인핸스)  │     │   (수어 아바타)  │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │      WIA Platform       │
                    │    (통합 접근성 API)     │
                    └─────────────────────────┘

사용 시나리오:
══════════════

1. 음악 청취 (CI 사용자)
   ├── WIA CI: 옥타브 인핸스먼트로 음악 청취 개선
   └── WIA Braille: 동시에 가사를 점자로 출력 (농맹인)

2. 영상 시청 (CI 사용자)
   ├── WIA CI: 배경음악/효과음 옥타브 인핸스
   └── WIA Talk: 자막을 수어 아바타로 표시 (보조)

3. 화상 통화 (CI 사용자)
   ├── WIA CI: 상대방 목소리 옥타브 인핸스
   └── WIA Talk: 내 말을 수어로 변환하여 농인에게 전달
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
2. 시스템 아키텍처 (System Architecture)
3. API 설계 (API Design)
   - REST API
   - WebSocket API
4. 스마트폰 앱 연동 (Mobile App Integration)
5. CI 시스템 연동 (CI System Integration)
6. WIA 생태계 통합 (WIA Ecosystem Integration)
7. 보안 및 프라이버시 (Security & Privacy)
8. 규제 고려사항 (Regulatory Considerations)
9. 배포 전략 (Deployment Strategy)
10. 예제 (Examples)
11. 참고문헌 (References)
```

### 3. TypeScript API 서버
```
/api/typescript/src/
├── server/
│   ├── index.ts                # Express 서버
│   ├── routes/
│   │   ├── analyze.ts          # /analyze 라우트
│   │   ├── detect.ts           # /detect 라우트
│   │   ├── enhance.ts          # /enhance 라우트
│   │   └── simulate.ts         # /simulate 라우트
│   ├── websocket/
│   │   ├── handler.ts          # WebSocket 핸들러
│   │   └── realtime.ts         # 실시간 처리
│   └── middleware/
│       ├── auth.ts             # 인증
│       └── validation.ts       # 검증
└── ...
```

### 4. 클라이언트 SDK
```
/sdk/
├── typescript/
│   ├── package.json
│   └── src/
│       ├── WiaCIClient.ts      # 클라이언트 SDK
│       └── types.ts
├── python/
│   ├── pyproject.toml
│   └── wia_ci_client/
│       ├── __init__.py
│       └── client.py
├── swift/                       # iOS
│   └── WiaCIClient.swift
└── kotlin/                      # Android
    └── WiaCIClient.kt
```

### 5. 데모 앱
```
/examples/demo-app/
├── web/                         # 웹 데모
│   ├── index.html
│   └── app.js
├── ios/                         # iOS 데모 (선택)
└── android/                     # Android 데모 (선택)
```

---

## ✅ 완료 체크리스트

Phase 4 완료 전 확인:

```
□ 웹서치로 CI 연동 가능성 조사 완료
□ 웹서치로 규제 환경 조사 완료
□ /spec/RESEARCH-PHASE-4.md 작성 완료
□ /spec/PHASE-4-INTEGRATION.md 작성 완료
□ REST API 서버 구현 완료
□ WebSocket 실시간 처리 구현 완료
□ TypeScript/Python 클라이언트 SDK 구현 완료
□ 웹 데모 앱 구현 완료
□ API 문서화 완료 (OpenAPI/Swagger)
□ 통합 테스트 완료
□ README 업데이트 (Phase 4 완료 표시)
□ WIA CI Octave Enhancement Standard 완료! 🎉
```

---

## 🎯 최종 목표

### 단기 (Phase 1-4 완료 시)

```
1. GitHub에 표준 문서 + 코드 공개
2. npm/PyPI에 SDK 패키지 공개
3. 웹 데모로 시뮬레이션 체험 가능
4. 연구자/개발자가 활용 가능
```

### 중기 (1-2년)

```
1. CI 제조사와 파트너십 논의
2. 외부 보조 장치로 실증
3. 학술 논문 발표
4. 임상 시험 협력 (대학/병원)
```

### 장기 (3-5년)

```
1. CI 프로세서에 내장
2. 글로벌 표준으로 채택
3. 수백만 CI 사용자 혜택
```

---

## 🚀 작업 시작

이제 Phase 4 작업을 시작하세요.

첫 번째 단계: **웹서치로 CI 제조사 연동 가능성 조사**

```
검색 키워드: "Cochlear Nucleus app SDK developer documentation"
```

화이팅! 🤟

---

<div align="center">

**Phase 4 of 4**

CI System Integration & API

**WIA CI Octave Enhancement Standard 완성!**

---

## 🎉 축하합니다!

Phase 4까지 완료하면,

**세계 최초 CI 옥타브 인핸스먼트 표준**이 탄생합니다.

```
1888: IPA (소리 표기 표준)
2025: ISP (수어 표기 표준)
2025: WIA CI (옥타브 인핸스먼트 표준)
```

**홍익인간 - 弘益人間 - Benefit All Humanity**

</div>
