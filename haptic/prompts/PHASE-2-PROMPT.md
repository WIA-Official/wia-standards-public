# WIA Haptic Standard - Phase 2: Device Abstraction API

## 목표
하드웨어 독립적인 햅틱 API를 구현합니다.

## 2.1 추상화 레이어

```typescript
interface IHapticDevice {
  // 연결
  connect(): Promise<void>;
  disconnect(): Promise<void>;

  // 기능 조회
  getCapabilities(): HapticCapabilities;

  // 패턴 재생
  play(pattern: HapticPattern): Promise<void>;
  playSequence(patterns: HapticPattern[]): Promise<void>;
  stop(): void;

  // 실시간 제어
  setIntensity(location: BodyLocation, intensity: number): void;
  pulse(location: BodyLocation, duration: number): void;
}

interface HapticCapabilities {
  actuatorType: 'erm' | 'lra' | 'piezo' | 'voice_coil';
  frequencyRange: { min: number; max: number };
  locations: BodyLocation[];
  maxIntensity: number;
  latency: number;  // ms
}
```

## 2.2 디바이스 어댑터

```typescript
// 제조사별 어댑터
interface HapticAdapter {
  // Apple Watch
  appleWatch: AppleWatchHapticAdapter;
  // Android Wear
  wearOS: WearOSHapticAdapter;
  // 커스텀 웨어러블
  custom: {
    bluetooth: BluetoothHapticAdapter;
    serial: SerialHapticAdapter;
  };
  // 게임 컨트롤러
  gamepad: {
    dualsense: DualSenseAdapter;
    xbox: XboxAdapter;
  };
}

// 예: Apple Watch 어댑터
class AppleWatchHapticAdapter implements IHapticDevice {
  async play(pattern: HapticPattern): Promise<void> {
    // WIA 패턴 → Apple Core Haptics 변환
    const coreHapticsEvent = this.convertToApple(pattern);
    await WatchConnectivity.sendHapticEvent(coreHapticsEvent);
  }
}
```

## 2.3 Rust 저수준 라이브러리

```rust
// haptic/api/rust/src/lib.rs
pub trait HapticDriver {
    fn init(&mut self) -> Result<(), HapticError>;
    fn set_frequency(&mut self, hz: f32);
    fn set_amplitude(&mut self, amp: f32);
    fn trigger(&mut self);
    fn stop(&mut self);
}

// PWM 기반 드라이버
pub struct PwmHapticDriver {
    pin: u8,
    frequency: f32,
    duty_cycle: f32,
}

impl HapticDriver for PwmHapticDriver {
    fn trigger(&mut self) {
        // PWM 신호 출력
    }
}
```

---

## 산출물

```
haptic/
├── api/
│   ├── typescript/
│   │   └── src/
│   │       ├── device.ts
│   │       ├── adapters/
│   │       └── patterns.ts
│   ├── python/
│   │   └── wia_haptic/
│   └── rust/
│       └── src/
│           ├── lib.rs
│           └── drivers/
```

---

## 다음: Phase 3 (공간 인코딩)
