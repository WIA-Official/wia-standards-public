# WIA Game Phase 3: Communication Protocol
## 입력 장치 통신 프로토콜 표준

**Version**: 1.0.0
**Date**: 2025-01-15
**Status**: Complete

---

## 1. 개요

게임 접근성 입력 장치와의 표준화된 통신 프로토콜을 정의합니다.

### 1.1 지원 장치

| 카테고리 | 장치 |
|---------|------|
| **적응형 컨트롤러** | Xbox Adaptive Controller, PlayStation Access, HORI Flex |
| **스위치 접근** | Button Arrays, Sip-and-Puff, Switch Interfaces |
| **아이 트래커** | Tobii Eye Tracker 5, Irisbond Duo, EyeWare Beam |
| **헤드 트래커** | TrackIR, Tobii Head Tracking, Webcam-based |
| **보이스 컨트롤** | Voice Attack, Windows Speech Recognition |
| **기타** | QuadStick, Mouth Controllers, Foot Pedals |

---

## 2. HID 프로토콜 계층

### 2.1 USB HID Gamepad

Xbox Adaptive Controller는 USB HID 표준을 따릅니다:

```
USB HID Report Descriptor:
- Usage Page: Generic Desktop (0x01)
- Usage: Gamepad (0x05)
- Input Reports: Buttons, Axes, D-Pad
```

### 2.2 버튼 매핑 (Xbox Adaptive Controller)

| USB 포트 | 버튼 1-8 매핑 |
|----------|---------------|
| **Left Port** | X1, X2, ThumbBtnL, A, B, View, Menu, (ignored) |
| **Right Port** | View+Menu, ThumbBtnR, X, Y, X1, X2, (ignored) |

### 2.3 축 매핑

```rust
pub struct GamepadAxes {
    pub left_stick_x: i16,   // -32768 to 32767
    pub left_stick_y: i16,
    pub right_stick_x: i16,
    pub right_stick_y: i16,
    pub left_trigger: u8,    // 0 to 255
    pub right_trigger: u8,
}
```

---

## 3. 이벤트 시스템

### 3.1 입력 이벤트 타입

```rust
pub enum InputEvent {
    ButtonPressed { button: Button, timestamp: u64 },
    ButtonReleased { button: Button, timestamp: u64 },
    AxisMoved { axis: Axis, value: f32, timestamp: u64 },
    GazePoint { x: f32, y: f32, timestamp: u64 },
    HeadPose { yaw: f32, pitch: f32, roll: f32, timestamp: u64 },
    VoiceCommand { command: String, confidence: f32, timestamp: u64 },
}
```

### 3.2 이벤트 필터링

| 필터 | 설명 | 기본값 |
|------|------|--------|
| **Debounce** | 버튼 바운스 제거 | 50ms |
| **Dead Zone** | 스틱 데드존 | 0.15 |
| **Smoothing** | 아이트래킹 스무딩 | 0.3 |
| **Threshold** | 트리거 활성화 임계값 | 0.1 |

---

## 4. 아이 트래커 통합

### 4.1 Tobii Game Integration

[Tobii Developer Zone](https://developer.tobii.com/)에서 제공하는 TGI SDK 기반:

```rust
pub struct GazeData {
    pub gaze_point: Point2D,      // 정규화된 화면 좌표 (0.0-1.0)
    pub gaze_origin: Point3D,     // 눈 위치 (mm)
    pub head_pose: HeadPose,      // 머리 위치/회전
    pub validity: GazeValidity,   // 유효성 플래그
    pub timestamp: u64,           // 마이크로초
}
```

### 4.2 드웰 선택 (Dwell Selection)

```rust
pub struct DwellConfig {
    pub dwell_time_ms: u32,       // 선택까지 응시 시간 (기본 800ms)
    pub tolerance_radius: f32,    // 허용 반경 (기본 0.05)
    pub progressive_feedback: bool, // 진행률 표시
}
```

---

## 5. 스위치 접근

### 5.1 스캐닝 모드

| 모드 | 설명 |
|------|------|
| **Auto Scan** | 자동으로 옵션 순환 |
| **Step Scan** | 스위치로 다음 항목 이동 |
| **Row-Column** | 행 선택 → 열 선택 |
| **Group Scan** | 그룹 → 항목 선택 |

### 5.2 스위치 설정

```rust
pub struct SwitchConfig {
    pub scan_speed_ms: u32,       // 스캔 속도 (기본 1000ms)
    pub scan_mode: ScanMode,
    pub auto_restart: bool,       // 끝에서 재시작
    pub loops_before_exit: u8,    // 종료까지 반복 횟수
    pub audio_feedback: bool,     // 오디오 피드백
    pub visual_highlight: bool,   // 시각적 강조
}
```

---

## 6. 매크로 시스템

### 6.1 매크로 정의

```rust
pub struct Macro {
    pub name: String,
    pub trigger: MacroTrigger,
    pub actions: Vec<MacroAction>,
    pub repeat_mode: RepeatMode,
    pub enabled: bool,
}

pub enum MacroAction {
    PressButton { button: Button },
    ReleaseButton { button: Button },
    HoldButton { button: Button, duration_ms: u32 },
    MoveAxis { axis: Axis, value: f32 },
    Delay { ms: u32 },
    PlaySound { sound_id: String },
}
```

### 6.2 트리거 타입

```rust
pub enum MacroTrigger {
    Button { button: Button },
    ButtonCombo { buttons: Vec<Button> },
    VoiceCommand { phrase: String },
    GazeRegion { region: Rect },
    Timer { interval_ms: u32 },
}
```

---

## 7. 코파일럿 모드

두 컨트롤러가 하나처럼 동작:

```rust
pub struct CopilotConfig {
    pub enabled: bool,
    pub primary_device: DeviceId,
    pub secondary_device: DeviceId,
    pub merge_mode: MergeMode,
}

pub enum MergeMode {
    FirstInput,      // 먼저 입력된 값 사용
    LastInput,       // 마지막 입력 값 사용
    Sum,             // 합산 (축)
    Override,        // 보조가 주 컨트롤러 덮어쓰기
}
```

---

## 8. 디바이스 핫플러그

```rust
pub enum DeviceEvent {
    Connected { device_id: DeviceId, info: DeviceInfo },
    Disconnected { device_id: DeviceId },
    ConfigChanged { device_id: DeviceId },
    BatteryLow { device_id: DeviceId, level: u8 },
    Error { device_id: DeviceId, error: DeviceError },
}
```

---

## 9. 참고 자료

- [Xbox Adaptive Controller](https://www.xbox.com/en-US/accessories/controllers/xbox-adaptive-controller)
- [Xbox Adaptive Controller HID Descriptor](https://gist.github.com/darthcloud/0e6bb1f14a805f9d32075d2812a06abc)
- [ControllersInfo HID Dumps](https://github.com/DJm00n/ControllersInfo)
- [Tobii Developer Zone](https://developer.tobii.com/)
- [Tobii Game Integration](https://developer.tobii.com/pc-gaming/develop/tobii-game-integration/)

---

弘益人間 - Gaming for Everyone 🎮
