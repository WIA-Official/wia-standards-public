# WIA-CITY-009: 스마트 조명 표준 💡

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--009-red.svg)](https://wia.org/standards/city-009)

## 개요

WIA-CITY-009 스마트 조명 표준은 도시 및 건물 내 조명 시스템의 지능화, 에너지 효율화, 사용자 경험 최적화를 위한 국제 표준입니다.

### 주요 기능

- 💡 **다양한 제어 프로토콜**: DALI, DMX, Zigbee, BLE, WiFi 지원
- 🌈 **색온도 제어 (CCT)**: Tunable White LED, RGB/RGBW 조명
- ⏰ **일주기 조명 (Circadian Lighting)**: 생체 리듬 지원
- 👁️ **스마트 센서**: 점유 감지, 조도 센서, 주광 이용
- ⚡ **에너지 관리**: 실시간 모니터링, 예산 관리, 이상 감지
- 📅 **스케줄링**: 시간 기반, 천문학적 스케줄
- 🎬 **씬 관리**: 사전 정의 및 동적 씬
- 🤖 **자동화**: IFTTT 스타일 자동화 룰
- 🌐 **개방형 표준**: 다양한 제조사와 시스템 간 호환

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/smart-lighting

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```bash
# npm으로 설치
npm install @wia/city-009

# 또는 yarn
yarn add @wia/city-009
```

```typescript
import { SmartLightingSDK } from '@wia/city-009';

const sdk = new SmartLightingSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-009/v1'
});

// 기구 제어
await sdk.fixtures.turnOn('fixture-001', 2000);
await sdk.fixtures.dim('fixture-001', 80);
await sdk.fixtures.setColorTemperature('fixture-001', 4000);

// 씬 활성화
await sdk.scenes.activate('relax', { fadeTime: 3000 });

// 센서 데이터 조회
const sensors = await sdk.sensors.list();

// 에너지 대시보드
const dashboard = await sdk.energy.getDashboard({
  start: '2025-12-01T00:00:00Z',
  end: '2025-12-25T23:59:59Z'
});
```

### 3. CLI 도구 사용

```bash
# API 키 설정
export WIA_SMART_LIGHTING_API_KEY="your-api-key"

# 기구 목록 조회
./cli/smart-lighting.sh list-fixtures

# 조명 켜기
./cli/smart-lighting.sh turn-on fixture-001 2000

# 밝기 조절 (50%)
./cli/smart-lighting.sh dim fixture-001 50

# 색온도 설정 (따뜻한 백색 2700K)
./cli/smart-lighting.sh set-cct fixture-001 2700

# 씬 활성화
./cli/smart-lighting.sh activate-scene relax

# 에너지 대시보드
./cli/smart-lighting.sh energy-dashboard
```

## 주요 기능 상세

### 1. 조명 기구 유형

#### LED 조명
- 일반 LED (80-150 lm/W)
- Tunable White LED (2700K-6500K)
- RGB/RGBW LED (1,600만 색상)

#### OLED 조명
- 면광원 (균일한 발광)
- 플렉시블 형태
- 우수한 연색지수 (CRI > 90)

#### 스마트 전구
- WiFi/Zigbee/BLE 내장
- 모바일 앱 제어
- 음성 제어 (Alexa, Google Home)

### 2. 제어 프로토콜

| 프로토콜 | 거리 | 노드 수 | 속도 | 전력 | 용도 |
|---------|------|---------|------|------|------|
| DALI | 300m | 64 (라인당) | 1.2 kbps | 저 | 상업용 조명 |
| DMX512 | 400m | 512 (유니버스당) | 250 kbps | 중 | 무대 조명 |
| Zigbee | 100m | 65,000 | 250 kbps | 저 | 스마트홈 |
| BLE Mesh | 30m | 32,767 | 1 Mbps | 초저 | 스마트홈 |
| WiFi | 50m | 제한없음 | 54+ Mbps | 고 | 범용 |

### 3. 일주기 조명 (Circadian Lighting)

인간의 생체 시계에 맞춰 색온도와 밝기를 자동으로 조절:

| 시간대 | EML (lux) | 목적 | 권장 CCT |
|--------|-----------|------|----------|
| 아침 (6-9시) | 200-300 | 각성 촉진 | 5000-6500K |
| 오전 (9-12시) | 300-500 | 최대 각성 | 5500-6500K |
| 정오 (12-15시) | 400-600 | 지속적 각성 | 5000-6000K |
| 오후 (15-18시) | 250-400 | 활동 유지 | 4500-5500K |
| 저녁 (18-21시) | 100-200 | 휴식 전환 | 3500-4500K |
| 밤 (21-23시) | 10-50 | 수면 준비 | 2700-3000K |

**효과:**
- 수면 질 개선
- 각성 상태 향상
- 기분 개선
- 인지 기능 향상

### 4. 센서 통합

#### 점유 감지 센서
- PIR (Passive Infrared): 움직임 감지
- 마이크로파: 벽 투과 감지
- 카메라 기반: 인원 수 카운팅

#### 조도 센서
- Photocell: 아날로그 센서
- 디지털 조도 센서: 정확도 ±5%

#### 센서 융합
- 다중 센서 통합
- 베이지안 확률 계산
- 높은 신뢰도 (95%+)

### 5. 주광 이용 (Daylight Harvesting)

자연광을 최대한 활용하여 에너지 절약:

**제어 전략:**
- 오픈 루프 (Open-loop): 외부 센서 기반
- 클로즈드 루프 (Closed-loop): 실내 센서 피드백
- 하이브리드: 외부 + 실내 센서

**절감 효과:**
- 사무실 (창가): 50-70%
- 학교: 40-60%
- 상업 시설: 30-50%

### 6. 에너지 관리

#### 실시간 모니터링
- 전압, 전류, 전력, 역률 측정
- 누적 에너지 소비량 (kWh)
- 운영 시간 추적

#### 에너지 절감 전략
1. **Task Tuning**: 작업 영역만 밝게 (30-50% 절약)
2. **개인 제어**: 사용자 맞춤 설정 (15-30% 절약)
3. **비점유 시간 조광**: 소등 또는 조광 (40-60% 절약)

#### 이상 감지
- 베이스라인 대비 편차 분석
- 램프 고장, 드라이버 오작동 감지
- 자동 알림

### 7. 스케줄링 및 씬 관리

#### 시간 기반 스케줄
- 요일별 시간대 설정
- 예외 날짜 처리
- 페이드 타임 설정

#### 천문학적 스케줄
- 일출/일몰 자동 동기화
- 위치 기반 자동 계산
- 오프셋 설정 가능

#### 사전 정의 씬
- 집중 (Work/Focus): 밝고 시원한 조명 (5000K, 100%)
- 휴식 (Relax): 따뜻한 조명 (2700K, 40%)
- 영화 (Movie): 어두운 분위기 (10%)
- 식사 (Dining): 아늑한 조명 (3000K, 50%)

### 8. 자동화

IFTTT 스타일의 자동화 룰:

```typescript
// 예시: 저녁 7시에 점유 중이면 휴식 모드
{
  trigger: { type: 'time', time: { at: '19:00' } },
  conditions: [
    { type: 'sensor', sensor: { sensorId: 'pir-1', parameter: 'occupancy', value: true } }
  ],
  actions: [
    { type: 'scene', scene: { sceneId: 'relax' } }
  ]
}

// 예시: 조도가 100 lux 이하면 조명 켜기
{
  trigger: { type: 'sensor', sensor: { sensorId: 'light-1', parameter: 'illuminance', value: 100, operator: '<' } },
  conditions: [
    { type: 'time_range', timeRange: { start: '08:00', end: '18:00' } }
  ],
  actions: [
    { type: 'fixture', fixture: { fixtureId: 'group:office', command: 'dim', parameters: { brightness: 80 } } }
  ]
}
```

## API 엔드포인트

### 기구 관리
```
GET    /api/v1/fixtures              # 기구 목록
GET    /api/v1/fixtures/{id}         # 기구 조회
POST   /api/v1/fixtures              # 기구 등록
PUT    /api/v1/fixtures/{id}         # 기구 업데이트
DELETE /api/v1/fixtures/{id}         # 기구 삭제
PUT    /api/v1/fixtures/{id}/state   # 상태 변경
```

### 씬 관리
```
GET    /api/v1/scenes                # 씬 목록
GET    /api/v1/scenes/{id}           # 씬 조회
POST   /api/v1/scenes                # 씬 생성
POST   /api/v1/scenes/{id}/activate  # 씬 활성화
```

### 스케줄 관리
```
GET    /api/v1/schedules             # 스케줄 목록
POST   /api/v1/schedules             # 스케줄 생성
PUT    /api/v1/schedules/{id}/enabled # 활성화/비활성화
```

### 센서 데이터
```
GET    /api/v1/sensors               # 센서 목록
GET    /api/v1/sensors/{id}          # 센서 조회
GET    /api/v1/sensors/{id}/measurements # 측정값 조회
```

### 에너지 분석
```
GET    /api/v1/energy/dashboard      # 대시보드
GET    /api/v1/energy/report         # 리포트
GET    /api/v1/energy/budget/{zone}  # 예산 조회
```

## WebSocket API (실시간)

```javascript
const ws = new WebSocket('wss://api.wia.org/city-009/ws');

// 구독
ws.send(JSON.stringify({
  type: 'subscribe',
  channels: [
    'fixtures:state',
    'sensors:sensor-001',
    'alerts'
  ]
}));

// 메시지 수신
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log(message);
};
```

## 디렉토리 구조

```
smart-lighting/
├── spec/
│   └── WIA-CITY-009-v1.0.md          # 상세 스펙
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts              # 타입 정의
│       │   └── index.ts              # SDK
│       └── package.json
├── cli/
│   └── smart-lighting.sh             # CLI 도구
├── README.md
└── install.sh
```

## 통합 예제

### 1. Node.js 백엔드

```typescript
import { SmartLightingSDK } from '@wia/city-009';

const sdk = new SmartLightingSDK({
  apiKey: process.env.WIA_API_KEY,
  endpoint: 'https://api.wia.org/city-009/v1'
});

// Express 라우트
app.post('/api/fixtures/:id/on', async (req, res) => {
  const { id } = req.params;
  const result = await sdk.fixtures.turnOn(id);
  res.json(result);
});
```

### 2. React 프론트엔드

```tsx
import { SmartLightingSDK } from '@wia/city-009';
import { useState, useEffect } from 'react';

function LightControl() {
  const [fixtures, setFixtures] = useState([]);
  const sdk = new SmartLightingSDK({ apiKey: 'your-key', endpoint: 'https://api.wia.org/city-009/v1' });

  useEffect(() => {
    sdk.fixtures.list().then((res) => {
      setFixtures(res.data.items);
    });
  }, []);

  const handleToggle = async (id, on) => {
    if (on) {
      await sdk.fixtures.turnOn(id);
    } else {
      await sdk.fixtures.turnOff(id);
    }
  };

  return (
    <div>
      {fixtures.map((fixture) => (
        <div key={fixture.fixtureId}>
          <span>{fixture.name}</span>
          <button onClick={() => handleToggle(fixture.fixtureId, !fixture.state.on)}>
            {fixture.state.on ? 'OFF' : 'ON'}
          </button>
        </div>
      ))}
    </div>
  );
}
```

### 3. Home Assistant 통합

```yaml
# configuration.yaml
rest_command:
  turn_on_light:
    url: "https://api.wia.org/city-009/v1/fixtures/{{ fixture_id }}/state"
    method: PUT
    headers:
      Authorization: "Bearer YOUR_API_KEY"
    payload: '{"on": true}'
```

## 성과 지표 (KPI)

### 에너지 효율
- 에너지 절감률: 30-50%
- 조명 효율: > 100 lm/W
- 에너지 원단위: < 10 kWh/m²/년

### 제어 성능
- 응답 시간: < 500 ms
- 시스템 가용성: > 99.5%
- 센서 정확도: > 95%

### 사용자 경험
- 사용자 만족도: > 8/10
- 수동 조정 빈도: < 3회/일
- 앱 사용률: > 60%

## 보안 및 개인정보보호

### 인증 및 권한
- OAuth 2.0 인증
- 역할 기반 접근 제어 (RBAC)
- API 키 관리

### 데이터 보안
- 전송 중 암호화: TLS 1.3
- 저장 암호화: AES-256
- 접근 로그 기록

### 개인정보보호
- GDPR 준수
- 데이터 최소화
- 카메라 센서 익명화
- 사용자 동의 기반 수집

## 문서

- [상세 스펙](./spec/WIA-CITY-009-v1.0.md): 전체 기술 사양
- [API 레퍼런스](https://api.wia.org/city-009/docs): OpenAPI 3.0 문서
- [SDK 문서](./api/typescript/README.md): TypeScript SDK 가이드
- [CLI 가이드](./cli/README.md): 명령줄 도구 사용법

## 관련 표준

- **WIA-ENE-029 (Light Pollution)**: 빛 공해 관리 표준
- **WIA-ENE-007 (Smart Grid)**: 스마트 그리드 연계
- **WIA-SOCIAL**: 사용자 참여 및 교육
- **WIA-BLOCKCHAIN**: 조명 기구 이력 추적

## 라이선스

© 2025 WIA (World Certification Industry Association)

이 표준은 MIT License 하에 배포됩니다.

## 기여

기여를 환영합니다! Pull Request를 보내주세요.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 연락처

**WIA 표준 사무국**
- 웹사이트: https://wiastandards.com
- 이메일: standards@wia-official.org
- GitHub: https://github.com/WIA-Official/wia-standards

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

스마트 조명은 단순히 에너지를 절약하는 기술을 넘어, 인간의 건강과 웰빙을 증진하고, 환경을 보호하며, 모두가 더 나은 삶을 살 수 있도록 돕는 기술입니다.

WIA-CITY-009 표준은 개방형 표준, 투명한 프로토콜, 협력적 개발을 통해 스마트 조명 기술이 인류 전체의 공동선에 기여하도록 보장합니다.

**함께, 우리는 더 밝고 건강한 미래를 만듭니다.** 💡

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
