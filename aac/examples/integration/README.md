# WIA AAC Integration Examples
## Phase 4: WIA Ecosystem Integration

이 디렉토리에는 WIA AAC Standard의 전체 파이프라인 예제가 포함되어 있습니다.

---

## 예제 목록

### TypeScript

| 파일 | 설명 |
|------|------|
| `typescript/full-aac-demo.ts` | 전체 AAC 파이프라인 데모 |

### Python

| 파일 | 설명 |
|------|------|
| `python/full_aac_demo.py` | 전체 AAC 파이프라인 데모 |

---

## 실행 방법

### TypeScript

```bash
# 빌드 후 실행
cd /api/typescript
npm run build
npx ts-node ../../examples/integration/typescript/full-aac-demo.ts
```

### Python

```bash
# 모듈 설치 후 실행
cd /api/python
pip install -e .
python ../../examples/integration/python/full_aac_demo.py
```

---

## 파이프라인 구조

```
센서 입력 (Eye Tracker, Switch, EMG, EEG, Breath, Head Movement)
    │
    ▼
┌─────────────────────────────────────┐
│ Phase 1: Signal Format Standard      │
│ 센서 신호 → 표준 JSON                │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ Phase 2: API Interface Standard      │
│ 표준 API → 텍스트 생성               │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ Phase 3: Communication Protocol      │
│ 메시지 프로토콜 → 전송               │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ Phase 4: WIA Ecosystem Integration   │
│ OutputManager                        │
├───────────┬───────────┬─────────────┤
│ TTS       │ ISP/수어   │ WIA Braille │
│ Adapter   │ Adapter   │ Adapter     │
└───────────┴───────────┴─────────────┘
    │           │           │
    ▼           ▼           ▼
  음성        수어        점자
  출력       아바타      디스플레이
```

---

## 출력 어댑터 설명

### TTSAdapter (Text-to-Speech)

텍스트를 음성으로 변환합니다.

- **Web Speech API**: 브라우저 내장 TTS
- **Mock Adapter**: 테스트용

```typescript
const tts = new MockTTSAdapter();
await tts.initialize();
await tts.output('안녕하세요');
```

### SignLanguageAdapter (ISP/WIA Talk)

텍스트를 수어(ISP 코드)로 변환합니다.

- **ISP Code**: 5대 요소 (HS, LC, MV, OR, NM)
- **WIA Talk**: 93개 핵심 제스처

```typescript
const signLanguage = new MockSignLanguageAdapter();
await signLanguage.initialize();
const codes = await signLanguage.textToISP('안녕');
// [{ code: 'HS01-LC01-MV01-OR01-NM01', meaning: '안녕' }]
```

### BrailleAdapter (WIA Braille)

텍스트를 점자로 변환합니다.

- **IPA 기반**: 텍스트 → IPA → 점자
- **8점 점자**: 256가지 조합

```typescript
const braille = new MockBrailleAdapter();
await braille.initialize();
const output = await braille.textToBraille('안녕');
// { ipa: '/annjʌŋ/', braille: '⠁⠝⠚⠪⠻' }
```

---

## OutputManager 사용법

### 개별 출력

```typescript
// 특정 어댑터로 출력
await output.outputTo('tts', '안녕하세요');
await output.outputTo('sign_language', '감사합니다');
await output.outputTo('braille', '사랑해');
```

### 브로드캐스트

```typescript
// 모든 활성 어댑터로 동시 출력
await output.broadcast('안녕하세요', { speed: 1.0 });
```

### 이벤트 핸들링

```typescript
output.on('outputStart', (data) => console.log('Started:', data));
output.on('outputEnd', (data) => console.log('Ended:', data));
output.on('error', (data) => console.error('Error:', data));
```

---

## 참고

- [Phase 4 Specification](/spec/PHASE-4-INTEGRATION.md)
- [WIA Talk Documentation](/docs/WIA-TALK-SCIENTIFIC-FOUNDATION.md)
- [WIA Braille Documentation](/docs/WIA-BRAILLE-SCIENTIFIC-FOUNDATION.md)

---

<div align="center">

**弘益人間** - 널리 인간을 이롭게

© 2025 SmileStory Inc. / WIA

</div>
