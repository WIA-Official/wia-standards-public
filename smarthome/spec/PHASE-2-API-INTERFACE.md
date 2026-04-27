# WIA Smart Home - Phase 2: API Interface

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Smart Home 접근성 표준의 API 인터페이스를 정의합니다.

## Rust SDK

완전한 Rust 구현이 제공됩니다:

**위치**: `api/rust/`

### 주요 컴포넌트

#### 1. 디바이스 제어
- **조명 제어**: 밝기, 색온도, RGB
- **도어락**: 잠금/해제, 상태 확인
- **온도 조절**: 난방/냉방, 자동 제어
- **블라인드**: 개폐, 각도 조절

#### 2. 센서 데이터
- **모션 센서**: 움직임 감지
- **도어 센서**: 개폐 감지
- **온습도 센서**: 환경 모니터링
- **조도 센서**: 밝기 측정

#### 3. 접근성 지원
- **음성 명령**: 한국어, 영어, 일본어
- **시각 피드백**: 조명 패턴, 색상 신호
- **촉각 피드백**: 진동 알림
- **긴급 알림**: 화재, 침입, 낙상

#### 4. WIA 통합
- **BCI 연동**: 생각으로 제어
- **Voice-Sign**: 음성/수어 명령
- **Smart Wheelchair**: 자동 도어 개방
- **AAC**: 다중 입력 방식

## API 구조

```rust
// 디바이스 제어
pub trait SmartHomeDevice {
    fn control(&mut self, command: Command) -> Result<Status>;
    fn status(&self) -> Result<DeviceStatus>;
}

// 센서 데이터
pub trait Sensor {
    fn read(&self) -> Result<SensorData>;
}

// 접근성 인터페이스
pub trait AccessibilitySupport {
    fn voice_command(&mut self, audio: &[u8]) -> Result<Action>;
    fn visual_feedback(&mut self, pattern: Pattern) -> Result<()>;
    fn haptic_feedback(&mut self, intensity: u8) -> Result<()>;
}
```

## 통신 프로토콜

### Matter Protocol
```
- Thread: 저전력 메시 네트워크
- WiFi: 고속 데이터 전송
- Bluetooth LE: 근거리 통신
```

### REST API
```
GET    /devices
GET    /devices/{id}
POST   /devices/{id}/control
GET    /sensors/{id}/data
POST   /accessibility/voice
```

### WebSocket
```
ws://hub.local/events
- 실시간 센서 데이터
- 디바이스 상태 변경
- 긴급 알림
```

## 인증 & 보안

### OAuth 2.0
- 사용자 인증
- 디바이스 인가
- 토큰 기반 접근 제어

### 암호화
- TLS 1.3 통신
- End-to-End 암호화
- 데이터 무결성 보장

## 에러 처리

```rust
pub enum SmartHomeError {
    DeviceNotFound,
    ConnectionFailed,
    CommandFailed,
    UnauthorizedAccess,
    TimeoutError,
}
```

## 예제

```rust
use wia_smarthome::*;

// 조명 제어
let mut light = Light::new("living_room_light");
light.control(Command::SetBrightness(80))?;

// 음성 명령
let mut accessibility = AccessibilityInterface::new();
let action = accessibility.voice_command(audio_data)?;

// 센서 읽기
let motion = MotionSensor::new("entrance");
let detected = motion.read()?;
```

## 디바이스 등록 및 페어링

스마트 홈 허브는 새 디바이스 등록 시 두 단계 핸드셰이크를 수행합니다.

```
1) Discovery: 허브가 mDNS / Matter Commissionable Discovery 로
   주변의 commissionable 디바이스 목록을 수집한다.
2) Commissioning: 사용자가 PIN(11자리, 디바이스 후면 라벨)을
   입력하면 PASE(Password-Authenticated Session Establishment)
   세션을 통해 fabric 인증서가 디바이스로 발급된다.
```

PASE 절차에서 페어링 PIN 은 1회용이 아닌 영구 자격으로 동작하므로,
디바이스 폐기 시 fabric 에서 명시적으로 제거(`removeFabric`) 되어야
합니다. 접근성 사용자가 PIN 을 시각적으로 읽기 어려운 경우 허브는
NFC 탭 또는 QR 코드 스캔의 대체 경로를 제공해야 합니다.

## 그룹 제어

`Group` 엔드포인트는 시각/청각 장애인이 한 번의 명령으로 다수
디바이스를 동시에 제어할 수 있도록 합니다.

```rust
pub struct Group {
    pub group_id: String,
    pub name: String,
    pub members: Vec<DeviceId>,
    pub fallback_strategy: FallbackStrategy,
}

pub enum FallbackStrategy {
    AllOrNothing,           // 한 디바이스라도 실패 시 전체 롤백
    BestEffort,             // 가능한 디바이스만 적용
    PriorityOrder(Vec<u8>), // 디바이스별 우선순위 정렬 후 순차 적용
}
```

`AllOrNothing` 은 의료 환경(예: 인공호흡기 보조 조명 시퀀스)에서,
`BestEffort` 는 일반 거주 시나리오(예: 취침 모드)에서 권장됩니다.

## 이벤트 스트림 및 백프레셔

WebSocket 이벤트 채널은 디바이스 수가 100 개를 초과하면 burst
조건에서 메시지가 폭주할 수 있습니다. 표준은 다음 백프레셔 정책을
요구합니다.

- 클라이언트가 ack 를 송신하지 않은 미처리 이벤트가 1024 건을
  초과하면 허브는 새 이벤트를 일시 큐잉하고 `congestion_warning`
  플래그를 송신한다.
- 미처리 이벤트가 8192 건을 초과하면 허브는 `low priority` 이벤트
  (예: 정기적인 조도 변화 보고)를 폐기하고 `urgent` 이벤트(화재,
  침입, 낙상)만 보존한다.
- 클라이언트가 30 초 이상 응답하지 않으면 연결이 종료되고 재연결
  시 `since=` 커서로 누락 이벤트를 재수신한다.

## 권한 모델

세 단계 권한 계층을 정의합니다.

| 역할 | 가능한 조작 | 비고 |
|------|-------------|------|
| `owner` | 모든 조작 + fabric 관리 + 타 사용자 초대 | 일반적으로 가구주 |
| `member` | 디바이스 제어 + 그룹 생성 + 시나리오 트리거 | 동거인 |
| `guest` | 읽기 전용 + 화이트리스트된 시나리오 호출 | 단기 방문자, 돌봄 인력 |

접근성 사용자에게는 별도의 `assistive` 역할이 부여될 수 있으며,
이 역할은 `owner` 의 위임을 통해 디바이스의 비상 제어 기능
(예: 모든 조명 켜기, 비상 호출)에 한해 별도 권한을 부여받습니다.

## 오프라인 폴백

허브-클라우드 연결이 끊어진 경우에도 로컬 LAN 내 디바이스 제어가
가능해야 합니다. 표준은 모든 conformant 허브가 다음 기능을 인터넷
없이도 제공할 것을 요구합니다.

- 디바이스 상태 조회 및 제어
- 그룹 명령 송출
- 음성 명령(로컬 NLU 모델 탑재 시)
- 비상 알림(LAN 내 모바일 디바이스로 푸시)

원격 접근, 음성 어시스턴트 연동, 사용자 초대 등은 인터넷 복구
시 자동 재개됩니다.

## 응답 시간 SLA

접근성 핵심 명령의 응답 시간 SLA 는 다음과 같습니다.

| 명령 종류 | 목표 (P50) | 상한 (P99) |
|-----------|------------|------------|
| 조명 켜기/끄기 | 200 ms | 1.0 s |
| 도어락 잠금 | 500 ms | 2.0 s |
| 비상 호출 트리거 | 50 ms | 500 ms |
| 음성 명령 인식+실행 | 800 ms | 3.0 s |

비상 호출이 1 초를 초과하는 경우 사용자는 시스템이 응답하지
않는 것으로 인지하므로, 표준은 비상 경로에 별도 우선순위 큐를
요구합니다.

## 멱등성 및 재시도

모든 mutating 호출은 클라이언트가 `Idempotency-Key` 헤더를
지정해야 하며, 허브는 같은 키의 재요청을 처음 응답과 동일한
바이트로 반복합니다. 권장 보존 시간은 24 시간입니다.

이는 모바일 클라이언트가 네트워크 단절 시 무차별 재시도하더라도
디바이스가 두 번 작동(예: 도어락 두 번 열림)하지 않도록 보장합니다.

## 에러 모델

표준 에러 응답은 RFC 7807 Problem Details 형식을 따릅니다.

```json
{
  "type": "https://wiastandards.com/smarthome/errors/device-offline",
  "title": "Device offline",
  "status": 503,
  "detail": "Device 'living_room_light' has not responded for 45s",
  "instance": "/devices/living_room_light/control",
  "device_id": "living_room_light",
  "last_seen": "2026-04-27T09:14:33Z"
}
```

확장 필드(`device_id`, `last_seen`)는 표준이 명시한 것에 한해
추가됩니다. 클라이언트는 알 수 없는 확장 필드를 무시하고 표준
필드만으로 동작 가능해야 합니다.

## 비상 알림 채널

비상 알림은 일반 이벤트 스트림과 분리된 별도 채널로 송출됩니다.
이는 일반 이벤트 폭주 상황에서도 비상 알림이 묻히지 않도록
보장합니다.

```
주 채널: ws://hub.local/events
비상 채널: ws://hub.local/emergency
```

비상 채널 구독자는 다음 카테고리 알림을 즉시 수신합니다.

- `fire` — 화재 감지기 트리거
- `intrusion` — 침입 감지기 트리거
- `fall` — 낙상 감지기 트리거(고령자 거주 시나리오)
- `medical` — 의료 비상 호출(웨어러블 또는 푸시 버튼)
- `co` — 일산화탄소 감지

비상 채널 메시지는 자동 재전송되며, 클라이언트가 30 초 내 ack 를
송신하지 않으면 사전 등록된 외부 연락처(가족, 119/911, 돌봄 인력)
로 SMS / 음성 통화가 자동 발송됩니다.

## API 버전 관리

URL prefix 로 메이저 버전을 표현합니다(`/api/v1/...`). 마이너 버전
은 응답 헤더 `API-Version: 1.2.0` 으로 표시됩니다. 호환성 보증
범위:

- v1.x 내에서는 기존 엔드포인트의 status code 와 응답 shape 가
  보존된다.
- 새 필드는 추가될 수 있으나 제거되지 않는다.
- 깨지는 변경은 v2 prefix 로만 도입되며, v1 은 deprecation 발표
  후 12 개월간 유지된다.

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
