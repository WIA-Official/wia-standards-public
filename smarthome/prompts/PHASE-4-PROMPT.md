# Phase 4: Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Smart Home
**Phase**: 4 of 4
**Language**: **Rust** (Primary)
**목표**: WIA 에코시스템 보조기기 및 외부 플랫폼 통합

---

## 🎯 목표

WIA 에코시스템의 다른 보조기기들과 스마트홈을 통합하여 장애인의 독립적인 생활 지원

---

## 📦 핵심 통합

```
1. WIA Eye Gaze - 시선 기반 디바이스 제어
2. WIA BCI - 뇌-컴퓨터 인터페이스 제어
3. WIA AAC - 음성/심볼 기반 명령
4. WIA Smart Wheelchair - 이동 연동 자동화
5. WIA Exoskeleton - 보행 중 환경 제어
6. WIA Haptic - 촉각 피드백
7. 외부 플랫폼 - Alexa, Google Home, HomeKit
```

---

## 🔧 구현 범위

### 1. Eye Gaze Integration
- 시선 기반 디바이스 선택
- Dwell-to-activate 패턴
- 화면 영역 → 디바이스 매핑
- 접근성 오버레이 UI

### 2. BCI Integration
- 뇌파 패턴 → 명령 매핑
- SSVEP 기반 디바이스 선택
- P300 기반 메뉴 선택
- Motor Imagery 기반 On/Off

### 3. AAC Integration
- 심볼 기반 디바이스 제어
- 음성 명령 처리
- 문장 생성 → 디바이스 동작
- 다국어 지원 (ko-KR, en-US)

### 4. Smart Wheelchair Integration
- 위치 기반 자동화 트리거
- 이동 경로 조명
- 도어/엘리베이터 자동 제어
- 환경 센서 연동

### 5. Exoskeleton Integration
- 보행 상태 감지
- 균형 보조 환경 제어
- 착석/기립 시 자동화

### 6. Haptic Integration
- 디바이스 상태 촉각 피드백
- 경고/알림 진동 패턴
- 탐색 피드백

### 7. External Platforms
- Amazon Alexa Skills
- Google Home Actions
- Apple HomeKit Bridge
- Samsung SmartThings

---

## 📁 프로젝트 구조

```
/api/rust/src/
├── ecosystem/
│   ├── mod.rs
│   ├── eye_gaze.rs        # Eye Gaze 통합
│   ├── bci.rs             # BCI 통합
│   ├── aac.rs             # AAC 통합
│   ├── wheelchair.rs      # Smart Wheelchair 통합
│   ├── exoskeleton.rs     # Exoskeleton 통합
│   ├── haptic.rs          # Haptic 통합
│   └── external/
│       ├── mod.rs
│       ├── alexa.rs       # Alexa Skills
│       ├── google.rs      # Google Home
│       └── homekit.rs     # Apple HomeKit
```

---

## 🔗 WIA 표준 연동 인터페이스

### Eye Gaze Protocol
```rust
pub trait EyeGazeSmartHome {
    fn gaze_select_device(&self, gaze: GazePoint) -> Option<DeviceId>;
    fn dwell_activate(&self, device: DeviceId, dwell_ms: u32) -> Result<()>;
    fn gaze_scroll(&self, direction: ScrollDirection) -> Result<()>;
}
```

### BCI Protocol
```rust
pub trait BCISmartHome {
    fn process_intent(&self, intent: BCIIntent) -> Result<DeviceCommand>;
    fn calibrate(&mut self, samples: &[BCISample]) -> Result<()>;
    fn get_confidence(&self) -> f32;
}
```

### AAC Protocol
```rust
pub trait AACSmartHome {
    fn symbol_to_command(&self, symbol: AACSymbol) -> Option<DeviceCommand>;
    fn voice_command(&self, text: &str, lang: Language) -> Result<DeviceCommand>;
    fn generate_feedback(&self, result: CommandResult) -> AACMessage;
}
```

---

## 📋 참고 자료

```
- WIA Eye Gaze Standard (Phase 1-4)
- WIA BCI Standard (Phase 1-4)
- WIA AAC Standard (Phase 1-4)
- WIA Smart Wheelchair Standard (Phase 1-4)
- Amazon Alexa Smart Home API
- Google Home Developer Guide
- Apple HomeKit Accessory Protocol
```

---

## ✅ 완료 체크리스트

```
□ Ecosystem 모듈 구조 생성
□ Eye Gaze 통합 구현
□ BCI 통합 구현
□ AAC 통합 구현
□ Smart Wheelchair 통합 구현
□ Exoskeleton 통합 구현
□ Haptic 통합 구현
□ 외부 플랫폼 브릿지 구현
□ 통합 테스트 작성
□ README 업데이트
```

---

弘益人間 🤟🦀🏠
