# WIA Myoelectric - Phase 4: 3D Printed Prosthetic Integration

## 목표
오픈소스 3D 프린팅 의수와 WIA EMG 시스템을 통합합니다.

## 4.1 호환 의수 목록

```typescript
interface CompatibleProsthetic {
  name: string;
  community: string;
  license: string;
  actuatorType: 'cable' | 'servo' | 'linear';
  fingerCount: number;
  wristDOF: number;           // 손목 자유도
  stlUrl: string;
  documentationUrl: string;
}

const COMPATIBLE_PROSTHETICS: CompatibleProsthetic[] = [
  {
    name: 'Ada Hand',
    community: 'OpenBionics',
    license: 'CC BY-SA',
    actuatorType: 'cable',
    fingerCount: 5,
    wristDOF: 1,
    stlUrl: 'https://openbionics.com/ada',
  },
  {
    name: 'Raptor Reloaded',
    community: 'e-NABLE',
    license: 'CC BY-NC',
    actuatorType: 'cable',
    fingerCount: 5,
    wristDOF: 0,
    stlUrl: 'https://hub.e-nable.org/raptor',
  },
  {
    name: 'Brunel Hand',
    community: 'OpenBionics',
    license: 'CC BY-SA',
    actuatorType: 'servo',
    fingerCount: 5,
    wristDOF: 1,
    stlUrl: 'https://openbionics.com/brunel',
  },
];
```

## 4.2 제스처 → 모터 명령 매핑

```typescript
interface ProstheticController {
  // 제스처 → 동작 매핑
  gestureMapping: Map<Gesture, ProstheticAction>;

  // 비례 제어 (EMG 강도 → 그립력)
  proportionalControl: {
    enabled: boolean;
    emgToForce: (emgLevel: number) => number;  // 0-1 → 0-100%
    speedMapping: (emgLevel: number) => number;
  };

  // 안전
  safety: {
    maxGripForce: number;      // 뉴턴
    autoRelease: boolean;      // 과부하 시 자동 해제
    temperatureLimit: number;  // 모터 온도 제한
  };
}

interface ProstheticAction {
  fingers: FingerPosition[];   // 각 손가락 위치 (0-100%)
  wrist?: WristPosition;
  speed: number;               // 0-100%
  holdDuration?: number;       // 유지 시간 (ms)
}

interface FingerPosition {
  finger: 'thumb' | 'index' | 'middle' | 'ring' | 'pinky';
  position: number;            // 0=펴짐, 100=굽힘
}
```

## 4.3 서보 모터 제어

```cpp
// Arduino 서보 제어 코드
#include <Servo.h>

class WiaProstheticController {
private:
    Servo thumbServo;
    Servo indexServo;
    Servo middleServo;
    Servo ringServo;
    Servo pinkyServo;

public:
    void executeGesture(Gesture gesture) {
        switch (gesture) {
            case HAND_OPEN:
                setAllFingers(0);  // 모두 펴기
                break;
            case HAND_CLOSE:
                setAllFingers(100);  // 모두 굽히기
                break;
            case PINCH:
                thumbServo.write(map(70, 0, 100, 0, 180));
                indexServo.write(map(70, 0, 100, 0, 180));
                // 나머지는 펴진 상태
                break;
        }
    }

    void setAllFingers(int position) {
        int angle = map(position, 0, 100, 0, 180);
        thumbServo.write(angle);
        indexServo.write(angle);
        middleServo.write(angle);
        ringServo.write(angle);
        pinkyServo.write(angle);
    }
};
```

## 4.4 통합 시스템 아키텍처

```
┌──────────────────────────────────────────────────────────┐
│                   WIA Myoelectric System                  │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  [EMG 센서] → [ESP32] → [BLE] → [스마트폰 앱]            │
│       │                              │                   │
│       └─ [WIA Classifier] ─────────┘                    │
│                  │                                       │
│                  ↓                                       │
│  [제스처 인식] → [ProstheticController] → [서보 모터]    │
│                                              │          │
│                                   [3D 프린팅 의수] ←────┘ │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

---

## 산출물

```
myoelectric/
├── integrations/
│   ├── openbionics/
│   │   ├── ada-hand-controller.ino
│   │   └── brunel-hand-controller.ino
│   ├── enable/
│   │   └── raptor-controller.ino
│   └── generic/
│       └── servo-controller.ino
├── mobile-app/
│   ├── android/
│   └── ios/
├── examples/
│   ├── basic-grip/
│   ├── proportional-control/
│   └── multi-gesture/
└── docs/
    ├── ASSEMBLY-GUIDE.md
    ├── CALIBRATION-GUIDE.md
    └── TROUBLESHOOTING.md
```

---

## 전체 비용 요약

| 구성 요소 | 비용 |
|-----------|------|
| EMG 하드웨어 (Phase 3) | $48 |
| 서보 모터 (5개) | $25 |
| 3D 프린팅 재료 | $15 |
| 기타 (배선, 나사 등) | $12 |
| **합계** | **$100** |

vs. 상업용 근전도 의수: $75,000

## 홍익인간

$75,000 → $100. 더 많은 사람들이 손을 되찾을 수 있도록.
