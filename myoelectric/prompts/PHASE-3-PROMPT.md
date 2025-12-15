# WIA Myoelectric - Phase 3: Low-cost Hardware Standard

## 목표
$100 미만으로 구축할 수 있는 EMG 하드웨어 표준을 정의합니다.

## 3.1 저가 EMG 회로 설계

```
┌─────────────────────────────────────────────────────────────┐
│  WIA Myoelectric Low-Cost Reference Design                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [전극] → [증폭] → [필터] → [ADC] → [MCU] → [BLE/USB]        │
│                                                             │
│  부품:                                                       │
│  • 전극: 3M Red Dot (~$0.50/ea)                             │
│  • 증폭: INA128 or AD620 (~$5)                              │
│  • 필터: 2nd order Butterworth (RC ~$2)                     │
│  • MCU: ESP32-S3 (~$5) or Arduino Nano 33 BLE (~$20)        │
│                                                             │
│  총 BOM: < $50 (2채널 기준)                                  │
└─────────────────────────────────────────────────────────────┘
```

## 3.2 하드웨어 인터페이스

```typescript
interface EMGHardwareInterface {
  // 연결
  connect(transport: 'ble' | 'usb' | 'wifi'): Promise<void>;
  disconnect(): Promise<void>;

  // 스트리밍
  startStreaming(sampleRate: number): void;
  stopStreaming(): void;
  onData(callback: (frame: EMGDataFrame) => void): void;

  // 설정
  setGain(channel: number, gain: number): void;
  setFilter(type: 'notch' | 'bandpass', params: FilterParams): void;

  // 상태
  getBatteryLevel(): number;
  getConnectionQuality(): number;
}
```

## 3.3 Arduino/ESP32 펌웨어

```cpp
// ESP32 펌웨어 예시
#include <BLEDevice.h>

#define EMG_CHANNEL_1 34
#define EMG_CHANNEL_2 35
#define SAMPLE_RATE 1000

class WiaMyoelectricFirmware {
public:
    void setup() {
        analogReadResolution(12);
        setupBLE();
        setupTimer(SAMPLE_RATE);
    }

    void loop() {
        if (sampleReady) {
            uint16_t ch1 = analogRead(EMG_CHANNEL_1);
            uint16_t ch2 = analogRead(EMG_CHANNEL_2);
            sendOverBLE(ch1, ch2);
        }
    }

private:
    void setupBLE();
    void setupTimer(int hz);
    void sendOverBLE(uint16_t ch1, uint16_t ch2);
};
```

## 3.4 BOM (Bill of Materials)

| 부품 | 수량 | 단가 | 소계 |
|------|------|------|------|
| ESP32-S3-DevKitC | 1 | $7 | $7 |
| INA128 (계측 증폭기) | 2 | $4 | $8 |
| 전극 (3M Red Dot) | 6 | $0.5 | $3 |
| 저항/커패시터 키트 | 1 | $5 | $5 |
| 전극 케이블 | 2 | $3 | $6 |
| PCB 제작 (JLCPCB) | 5 | $2 | $10 |
| 3D 프린팅 케이스 | 1 | $5 | $5 |
| 배터리 (LiPo 500mAh) | 1 | $4 | $4 |
| **합계** | | | **$48** |

---

## 산출물

```
myoelectric/
├── hardware/
│   ├── schematics/
│   │   ├── wia-emg-2ch.kicad_sch
│   │   └── wia-emg-2ch.pdf
│   ├── pcb/
│   │   ├── wia-emg-2ch.kicad_pcb
│   │   └── gerbers/
│   ├── firmware/
│   │   ├── esp32/
│   │   │   └── wia_myoelectric/
│   │   └── arduino/
│   │       └── wia_myoelectric/
│   ├── enclosure/
│   │   └── case.stl
│   └── BOM.csv
```

---

## 다음: Phase 4 (3D 프린팅 의수 통합)
