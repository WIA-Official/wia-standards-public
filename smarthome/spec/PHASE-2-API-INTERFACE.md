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

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
