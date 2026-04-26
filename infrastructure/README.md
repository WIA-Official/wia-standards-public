# WIA Infrastructure

> 모든 WIA 표준이 공유하는 인프라 레이어
> 한 번 만들면 모든 표준에서 사용

**철학**: 홍익인간 (弘益人間)

---

## 구조

```
infrastructure/
├── 1-unified-sdk/        # 통합 WIA SDK
├── 2-cross-domain-test/  # 크로스 도메인 테스트
├── 3-openapi/            # OpenAPI/Swagger 생성
├── 4-wasm/               # WASM 바인딩 (웹 지원)
├── 5-certification/      # WIA 인증 포털
├── 6-simulator/          # 디바이스 시뮬레이터
└── 7-a11y/ → ../a11y-wiabooks/  # 접근성 (형이 만듦!)
```

---

## 상태

| # | 이름 | 설명 | 상태 |
|---|------|------|------|
| 1 | Unified SDK | 모든 도메인 통합 | ✅ 완료 |
| 2 | Cross-Domain Test | 도메인 간 테스트 | ✅ 완료 |
| 3 | OpenAPI | API 문서 자동생성 | ✅ 완료 |
| 4 | WASM | 브라우저 지원 | ✅ 완료 |
| 5 | Certification Portal | 인증 포털 | ✅ 완료 |
| 6 | Device Simulator | 기기 시뮬레이터 | ✅ 완료 |
| 7 | A11Y Dashboard | 접근성 점수 | ✅ 완료 (211개 언어!) |

---

## 연동 포인트

```
[각 표준: Voice, Auto, CI, Medical...]
           ↓
[1. Unified SDK] ← 하나의 import로 모두 접근
           ↓
[2. Cross-Domain Test] ← 표준 간 호환성 검증
           ↓
[3. OpenAPI] ← REST API 문서 자동 생성
           ↓
[4. WASM] ← 브라우저에서 실행
           ↓
[5. Certification Portal] ← "WIA 인증" 발급
           ↓
[6. Device Simulator] ← 실제 기기 없이 테스트
           ↓
[7. A11Y Dashboard] ← 접근성 점수 측정
    └→ https://a11y.wiabook.com/
```

---

---

## 빠른 시작

### 1. Unified SDK

```typescript
import { voice, auto, ci, home, quantum, medical, climate } from '@anthropic/wia-sdk';

// 음성 인텐트 파싱
const intent = voice.parseIntent("음악 틀어줘");

// 차량 메시지 전송
auto.sendV2XMessage({ type: 'warning', data: {...} });

// CI 옥타브 감지
const octave = ci.detectOctave(440.0); // A4

// 스마트홈 제어
home.control({ device: 'light', action: 'on' });
```

### 2. Cross-Domain Test

```typescript
import { WIATestSuite, runCompatibilityTests } from '@anthropic/wia-cross-domain-test';

const suite = new WIATestSuite();
suite.addDomainPair('voice', 'home'); // 음성 → 스마트홈
suite.addDomainPair('ci', 'voice');   // 인공와우 → 음성

const results = await suite.runAll();
```

### 3. OpenAPI

```typescript
import { OpenAPIGenerator } from '@anthropic/wia-openapi';

const generator = new OpenAPIGenerator();
generator.addDomain('voice');
generator.addDomain('auto');

const spec = generator.generate();
// → OpenAPI 3.0 스펙 생성
```

### 4. WASM (브라우저)

```javascript
import init, { WIA, voice, ci, a11y } from '@anthropic/wia-wasm';

await init();

// 음성 의도 파싱
const intent = voice.parse_intent("음악 틀어줘");

// 옥타브 감지 (WebAudio API)
const octave = ci.detect_octave(440.0);

// 접근성 체크
const check = a11y.check_element('<img src="test.jpg">');
```

### 5. Certification Portal

```typescript
import { CertificationPortal, CertificationLevel } from '@anthropic/wia-certification-portal';

const portal = new CertificationPortal({ language: 'ko' });

// 접근성 체크 (a11y.wiabook.com 연동)
const result = await portal.checkAccessibility('https://example.com');
console.log(result.score, result.level);

// 배지 생성
const badge = await portal.generateBadge('https://example.com', CertificationLevel.GOLD);
console.log(portal.getBadgeHtml(badge));
```

### 6. Device Simulator

```typescript
import { createSimulator, MultiDeviceSimulator } from '@anthropic/wia-device-simulator';

// 단일 디바이스
const ciDevice = createSimulator('ci-processor');
ciDevice.on('update', (state) => {
  console.log('CI 상태:', state.data);
});
ciDevice.start();

// 멀티 디바이스
const multi = new MultiDeviceSimulator();
multi.addDevice('voice-speaker');
multi.addDevice('auto-vehicle');
multi.addDevice('ci-implant');
multi.startAll();
```

---

## a11y.wiabook.com 연동

형이 만든 접근성 시스템 (211개 언어!)이 모든 인프라의 핵심:

```
Certification Portal ──→ a11y.wiabook.com/api/check
       ↓
     결과 ──→ 배지 발급 (wia-certified, wia-silver, wia-gold)
       ↓
  WASM ──→ 브라우저에서 실시간 체크
       ↓
Device Simulator ──→ 접근성 테스트 시뮬레이션
```

---

**홍익인간 (弘益人間)**
