# Phase 1 사전 조사 결과
# Phase 1 Research Findings

---

**작성일**: 2025년 12월 13일
**작성**: Claude Code (Opus 4.5)
**목적**: AAC 센서별 데이터 형식 조사 및 표준 설계 방향 도출

---

## 목차 (Table of Contents)

1. [Eye Tracker (시선 추적기)](#1-eye-tracker-시선-추적기)
2. [Switch (스위치)](#2-switch-스위치)
3. [Muscle Sensor / EMG (근육 센서)](#3-muscle-sensor--emg-근육-센서)
4. [Brain Interface / EEG/BCI (뇌파 인터페이스)](#4-brain-interface--eegbci-뇌파-인터페이스)
5. [Breath / Sip-and-Puff (호흡 센서)](#5-breath--sip-and-puff-호흡-센서)
6. [Head Movement (머리 움직임)](#6-head-movement-머리-움직임)
7. [기존 표준 조사](#7-기존-표준-조사)
8. [공통점 분석](#8-공통점-분석)
9. [결론 및 설계 방향](#9-결론-및-설계-방향)

---

## 1. Eye Tracker (시선 추적기)

### 1.1 Tobii Pro SDK

**제조사**: Tobii (스웨덴)
**SDK 버전**: tobii-research 2.1.0 (2024년 11월 기준)
**지원 언어**: Python, .NET, C, MATLAB/Octave, Unity

#### 데이터 형식

Tobii Pro SDK는 `GazeData` 객체로 시선 데이터를 제공하며, Python에서 딕셔너리로 변환 가능:

```python
# Tobii Pro SDK Python - GazeData 구조
gaze_data = {
    'device_time_stamp': 123456789,      # 장치 타임스탬프 (microseconds)
    'system_time_stamp': 123456789,      # 시스템 타임스탬프 (microseconds)
    'left_eye': {
        'gaze_point': {
            'position_on_display_area': (0.45, 0.32),  # (x, y) 정규화 0.0~1.0
            'validity': 'Valid'  # 또는 'Invalid'
        },
        'gaze_origin': {
            'position_in_user_coordinates': (x, y, z),  # mm 단위 3D 좌표
            'validity': 'Valid'
        },
        'pupil': {
            'diameter': 3.5,  # mm 단위
            'validity': 'Valid'
        }
    },
    'right_eye': {
        # left_eye와 동일한 구조
    }
}
```

#### 좌표계

- **Active Display Coordinate System**: 화면 좌상단이 원점 (0,0)
- **정규화**: 0.0 ~ 1.0 범위 (화면 비율)
- **3D 좌표**: User Coordinate System (mm 단위)

#### 주요 특징

- 양안(좌/우) 개별 데이터 제공
- 마이크로초 단위 타임스탬프
- 유효성(validity) 플래그 포함
- 동공 크기(pupil diameter) 제공

### 1.2 EyeTech (QuickLINK 2 API)

**제조사**: EyeTech Digital Systems (미국)
**API**: QuickLINK 2 API (Windows)

#### 데이터 형식

- QuickLINK 2 API를 통해 시선 좌표, 유효성, 타임스탬프 제공
- Lab Streaming Layer (LSL)를 통한 데이터 스트리밍 지원
- 실제 FPS: VT-3 Mini 기준 약 6-7 Hz (full resolution)

#### 특징

- Windows HID 표준 마우스로 동작 가능
- Microsoft Gaze Interaction Library와 호환
- 오픈소스 LSL 통합 라이브러리 존재

### 1.3 공통 Eye Tracker 데이터 필드

| 필드 | 설명 | 단위/범위 |
|------|------|----------|
| gaze_x, gaze_y | 시선 좌표 | 0.0 ~ 1.0 (정규화) |
| timestamp | 타임스탬프 | microseconds |
| validity | 데이터 유효성 | boolean |
| pupil_diameter | 동공 크기 | mm |
| fixation | 응시 여부 | boolean |
| dwell_time | 응시 시간 | ms |

---

## 2. Switch (스위치)

### 2.1 AbleNet Hitch 2

**제조사**: AbleNet (미국)
**인터페이스**: USB HID

#### 동작 방식

- 최대 5개 스위치 또는 1개 조이스틱 입력
- USB HID 키보드/마우스로 에뮬레이션
- 1-4개 키스트로크 프로그래밍 가능

#### 데이터 형식

```
USB HID Report:
- 키보드 모드: 키 누름 → HID Keyboard Report (8 bytes)
  - Modifier keys + Reserved + Key codes (6 keys max)
- 마우스 모드: 조이스틱 → HID Mouse Report
  - Buttons + X delta + Y delta
```

### 2.2 AbleNet Hook Plus (iOS)

**인터페이스**: Apple MFi Lightning (HID Assistive Switch Control)

#### 특징

- iOS 7+ Switch Control 전용
- Apple 전용 HID Assistive Switch Control 프로토콜
- 키스트로크가 아닌 스위치 클릭으로 동작

### 2.3 일반 스위치 인터페이스 특성

| 데이터 | 설명 | 값 |
|--------|------|-----|
| switch_id | 스위치 식별자 | 1, 2, 3... |
| state | 상태 | pressed / released / held |
| duration | 누름 지속 시간 | ms |
| channel | 채널 | primary / secondary |

---

## 3. Muscle Sensor / EMG (근육 센서)

### 3.1 MyoWare 2.0

**제조사**: Advancer Technologies / SparkFun
**인터페이스**: 아날로그 출력 (Arduino ADC)

#### 출력 유형

1. **Envelope (ENV)**: 근육 활성도 크기 (기본 출력)
2. **Raw (RAW)**: 증폭/필터링된 원본 EMG 신호
3. **Rectified (REC)**: 전파 정류된 신호

#### 데이터 형식

```
Arduino ADC 읽기:
- 범위: 0 ~ 1023 (10-bit ADC)
- 전압: 0V ~ 5V (또는 3.3V)
- 중심 전압: 약 2.5V (5V 공급 시, Raw 출력)

변환:
- 정규화: ADC_value / 1023.0 → 0.0 ~ 1.0
- 밀리볼트: (ADC_value / 1023.0) * 5000 → 0 ~ 5000 mV
```

#### 특징

- 아날로그 신호 → 디지털 변환 필요
- 임계값 기반 제스처 인식 가능
- 실시간 스트리밍 시 노이즈 고려 필요

### 3.2 OpenBCI (EMG 모드)

**제조사**: OpenBCI (미국)
**인터페이스**: Bluetooth (RFduino) / USB

#### 데이터 형식

- 8채널 또는 16채널 (Daisy 모듈)
- 24-bit 해상도
- 샘플 레이트: 250 Hz (기본)
- 단위: 마이크로볼트 (µV)

```
스케일 팩터: 0.02235 µV/count (24x 게인)
실제 해상도: ~0.1 µV (EEG/EMG에 충분)
노이즈: ~0.16 µV RMS
```

---

## 4. Brain Interface / EEG/BCI (뇌파 인터페이스)

### 4.1 OpenBCI Cyton

**채널 수**: 8채널 (Daisy로 16채널 확장)
**샘플 레이트**: 250 Hz
**해상도**: 24-bit

#### 데이터 패킷 형식

```
Binary Packet (33 bytes):
├── Header (1 byte): 0xA0
├── Sample Index (1 byte): 0-255
├── Channel Data (24 bytes): 8 channels × 3 bytes
├── Accelerometer/Aux (6 bytes): X, Y, Z axes
└── Footer (1 byte): 0xC0

Channel Data 변환:
raw_count (24-bit signed int) × 0.02235 = microvolts
```

#### BrainFlow 라이브러리 형식

```python
# BrainFlow 2D 배열 구조
data = [
    [ch1_t1, ch1_t2, ...],  # EEG 채널 1 (µV)
    [ch2_t1, ch2_t2, ...],  # EEG 채널 2 (µV)
    ...
    [acc_x_t1, acc_x_t2, ...],  # 가속도 X
    [acc_y_t1, acc_y_t2, ...],  # 가속도 Y
    [acc_z_t1, acc_z_t2, ...],  # 가속도 Z
    [timestamp_t1, timestamp_t2, ...],  # UNIX timestamp (µs)
]

# 보드 설명 (board description)
{
    "eeg_channels": [1, 2, 3, 4, 5, 6, 7, 8],
    "eeg_names": "Fp1,Fp2,C3,C4,P3,P4,O1,O2",
    "accel_channels": [17, 18, 19],
    "timestamp_channel": 30,
    "sampling_rate": 250,
    "num_rows": 32
}
```

### 4.2 NeuroSky MindWave

**채널 수**: 1채널
**인터페이스**: Bluetooth / ThinkGear Connector

#### JSON 데이터 형식

```json
{
    "eSense": {
        "attention": 61,
        "meditation": 56
    },
    "eegPower": {
        "delta": 75352,
        "theta": 47262,
        "lowAlpha": 3073,
        "highAlpha": 4947,
        "lowBeta": 2618,
        "highBeta": 9062,
        "lowGamma": 4357,
        "highGamma": 2634
    },
    "poorSignalLevel": 0
}
```

### 4.3 Emotiv (Cortex API)

**인터페이스**: WebSocket + JSON-RPC
**인증**: Client ID / Client Secret

#### 데이터 형식

```json
// Emotiv Cortex 2 API - EEG 데이터 스트림
{
    "id": 1,
    "jsonrpc": "2.0",
    "result": {
        "eeg": [timestamp, ch1, ch2, ch3, ..., marker],
        "mot": [timestamp, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z],
        "dev": [battery, signal1, signal2, ...]
    }
}
```

### 4.4 공통 EEG/BCI 데이터 필드

| 필드 | 설명 | 단위 |
|------|------|------|
| channel_id | 채널 식별자 | Fp1, Fp2, C3, C4... |
| value | EEG 값 | µV (마이크로볼트) |
| timestamp | 타임스탬프 | UNIX timestamp (µs) |
| bands | 주파수 대역 파워 | delta, theta, alpha, beta, gamma |
| sample_rate | 샘플 레이트 | Hz |

---

## 5. Breath / Sip-and-Puff (호흡 센서)

### 5.1 Origin Instruments Sip/Puff Breeze

**인터페이스**: USB HID (마우스/키보드/조이스틱)

#### 동작 방식

- 압력 센서로 흡입(sip)/불기(puff) 감지
- 표준 압력: ±3 인치 수주 (water column)
- 고감도 설정: ±2 인치 수주

#### 모드

1. **Switch 모드**: 마우스 버튼 클릭, 조이스틱 버튼, 키보드 키
2. **Joystick Plus 모드**: 실시간 게이지 압력 (±4 kPa)

### 5.2 DIY Sip-and-Puff (Adafruit LPS33HW)

**센서**: STMicroelectronics LPS33HW MEMS 압력 센서

#### 데이터 형식

```python
# 압력 데이터
{
    "pressure_hpa": 1013.25,  # hPa (헥토파스칼)
    "baseline": 1013.0,       # 기준 압력
    "delta": 0.25,            # 압력 변화
    "action": "sip"           # sip / puff / neutral
}

# 변환
delta_kPa = (pressure - baseline) / 10
```

### 5.3 LipSync (오픈소스)

**개발**: Makers Making Change / Neil Squire Society
**인터페이스**: USB HID / Bluetooth

#### 특징

- 압력 감도 조절 가능
- 마우스 커서 제어 또는 스위치 모드
- GitHub에서 오픈소스로 공개

---

## 6. Head Movement (머리 움직임)

### 6.1 HeadMouse Nano

**제조사**: Origin Instruments
**인터페이스**: USB HID (표준 마우스)

#### 동작 방식

- 광학 센서로 반사 타겟 추적
- 머리 움직임 → 마우스 커서 이동
- 별도 API 없음 (표준 USB HID 마우스)

#### 특징

- 크로스 플랫폼 (Windows, macOS, Linux, iOS, Android)
- 추가 소프트웨어 불필요
- 원시 추적 데이터 접근 불가

### 6.2 TrackerPro

**제조사**: AbleNet/Madentec
**인터페이스**: USB HID (표준 마우스)

#### 동작 방식

- HeadMouse Nano와 유사
- 표준 USB 마우스로 동작
- 별도 API 미제공

### 6.3 소프트웨어 기반 Head Tracking

**오픈소스 예시**: OpenCV 기반 headmouse

#### 데이터 형식

```python
# 소프트웨어 기반 head tracking
{
    "position": {
        "x": 0.55,  # 정규화 0.0 ~ 1.0
        "y": 0.48
    },
    "rotation": {
        "pitch": 5.2,   # 도 (degrees)
        "yaw": -3.1,
        "roll": 0.5
    },
    "face_detected": true,
    "confidence": 0.92
}
```

---

## 7. 기존 표준 조사

### 7.1 Intel ACAT (Assistive Context-Aware Toolkit)

**라이선스**: Apache 2.0
**언어**: C# (.NET 4.5)
**GitHub**: https://github.com/intel/acat

#### 아키텍처

- 플러그인 기반 프레임워크
- 런타임에 확장 모듈 동적 로드
- ActuatorBase, ActuatorSwitchBase 클래스 상속으로 센서 추가

#### 주요 컴포넌트

```
ACAT 구조:
├── ACATCore.dll          # 핵심 라이브러리
├── ACATExtension.dll     # 확장 라이브러리
└── Extensions/           # 플러그인 폴더
    ├── Actuators/        # 센서/입력 장치
    ├── TTS/              # 음성 합성
    └── WordPrediction/   # 단어 예측 (Presage)
```

#### 역사적 의의

- 스티븐 호킹 박사를 위해 개발
- 3년간의 반복적 설계 과정
- 실제 ALS 환자의 피드백 반영

### 7.2 USB HID (Human Interface Device)

**스펙**: Device Class Definition for HID 1.11
**HUT**: HID Usage Tables 1.6

#### Report Descriptor 구조

```
HID Report Descriptor 구성요소:
- Usage Page: 디바이스 기능 분류 (Generic Desktop, Button, etc.)
- Usage: 구체적 용도 (Mouse, Keyboard, Joystick)
- Collection: 관련 항목 그룹화
- Logical Minimum/Maximum: 값 범위
- Report Size: 필드당 비트 수
- Report Count: 필드 개수
- Input/Output/Feature: 데이터 방향
```

#### AAC 관련성

- 대부분의 AAC 스위치 인터페이스가 USB HID 사용
- 키보드/마우스/조이스틱으로 에뮬레이션
- 표준화된 드라이버로 크로스 플랫폼 호환

### 7.3 W3C Pointer Events

**스펙**: https://www.w3.org/TR/pointerevents/
**레벨**: Level 3

#### 좌표 속성

```javascript
// W3C Pointer Events 좌표 속성
{
    screenX: 1920,      // 화면 좌표
    screenY: 1080,
    clientX: 500,       // 뷰포트 좌표
    clientY: 300,
    pageX: 500,         // 페이지 좌표 (스크롤 포함)
    pageY: 1300,
    offsetX: 50,        // 타겟 요소 내 좌표
    offsetY: 25,

    // 추가 속성
    pressure: 0.5,      // 압력 0.0 ~ 1.0
    tiltX: 15,          // 기울기 (도)
    tiltY: -10,
    twist: 0,           // 회전 (도)
    width: 20,          // 접촉 영역 크기
    height: 20
}
```

#### AAC 관련성

- 시선 추적, 머리 추적을 Pointer로 추상화 가능
- 표준화된 좌표계와 이벤트 모델 참조
- coalesced events로 고해상도 추적 지원

---

## 8. 공통점 분석

### 8.1 모든 센서에 공통으로 필요한 필드

| 필드 | 설명 | 필수 여부 |
|------|------|----------|
| `type` | 센서 유형 | 필수 |
| `timestamp` | 타임스탬프 | 필수 |
| `device` | 장치 정보 | 필수 |
| `data` | 센서별 데이터 | 필수 |
| `confidence` | 신뢰도 | 선택 (권장) |
| `sequence` | 시퀀스 번호 | 선택 |

### 8.2 타임스탬프 형식

| 표준 | 형식 | 사용처 |
|------|------|--------|
| UNIX timestamp | 정수 (ms 또는 µs) | OpenBCI, BrainFlow |
| ISO 8601 | 문자열 | JSON 직렬화 |
| Device timestamp | 장치 고유 | Tobii Pro SDK |

**권장**: UNIX timestamp (밀리초) + ISO 8601 문자열 병행

### 8.3 좌표계

| 유형 | 범위 | 사용처 |
|------|------|--------|
| 정규화 | 0.0 ~ 1.0 | 시선 추적, 머리 추적 |
| 픽셀 | 0 ~ 화면크기 | 디스플레이 기반 |
| 물리 단위 | mm, cm | 3D 위치 |

**권장**: 정규화 (0.0 ~ 1.0)를 기본으로, 픽셀/물리 단위는 메타데이터로 제공

### 8.4 센서별 고유 필드

| 센서 유형 | 고유 필드 |
|----------|----------|
| Eye Tracker | gaze_point, pupil, fixation, blink |
| Switch | switch_id, state, duration |
| EMG | channel_id, activation, gesture |
| EEG/BCI | channels[], bands{}, classification |
| Breath | action, pressure, intensity |
| Head | position, rotation, gesture |

---

## 9. 결론 및 설계 방향

### 9.1 표준 형식 설계 원칙

1. **확장성 (Extensibility)**
   - 새로운 센서 유형 추가 용이
   - 커스텀 필드 허용

2. **상호운용성 (Interoperability)**
   - JSON 기반 (모든 플랫폼 지원)
   - 명확한 타입 정의

3. **정확성 (Precision)**
   - 마이크로초 단위 타임스탬프
   - 부동소수점 좌표

4. **유효성 검증 (Validation)**
   - JSON Schema로 형식 검증
   - 필수/선택 필드 명확히 구분

### 9.2 권장 기본 구조

```json
{
    "$schema": "https://wia.live/aac/signal/v1/schema.json",
    "version": "1.0.0",
    "type": "sensor_type",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-13T12:00:00.000Z"
    },
    "sequence": 12345,
    "device": {
        "manufacturer": "Tobii",
        "model": "Pro Fusion",
        "firmware": "1.2.3",
        "serial": "ABC123"
    },
    "data": {
        // 센서별 고유 데이터
    },
    "meta": {
        "confidence": 0.95,
        "validity": true,
        "raw": {}  // 선택: 원본 데이터
    }
}
```

### 9.3 다음 단계

1. **PHASE-1-SIGNAL-FORMAT.md** 작성
   - 위 조사 결과를 바탕으로 정식 스펙 문서 작성

2. **JSON Schema 생성**
   - 기본 스키마 + 센서별 스키마 7개

3. **예제 데이터 생성**
   - 각 센서 유형별 실제 사용 시나리오 반영

4. **검증 스크립트**
   - TypeScript 및 Python으로 스키마 검증 도구 구현

---

## 참고 문헌 (References)

### 공식 문서

- [Tobii Pro SDK Documentation](https://developer.tobiipro.com/)
- [OpenBCI Documentation](https://docs.openbci.com/)
- [BrainFlow Documentation](https://brainflow.readthedocs.io/)
- [Emotiv Cortex API](https://emotiv.gitbook.io/cortex-api)
- [NeuroSky Developer Docs](https://developer.neurosky.com/)
- [USB HID Specification](https://www.usb.org/hid)
- [W3C Pointer Events](https://www.w3.org/TR/pointerevents/)

### 오픈소스 프로젝트

- [Intel ACAT (GitHub)](https://github.com/intel/acat)
- [LipSync by Makers Making Change](https://github.com/makersmakingchange/LipSync)
- [OpenBCI GUI](https://github.com/OpenBCI/OpenBCI_GUI)
- [Lab Streaming Layer](https://github.com/sccn/labstreaminglayer)

### 제조사 웹사이트

- [AbleNet](https://www.ablenetinc.com/)
- [Origin Instruments](https://www.orin.com/)
- [MyoWare/SparkFun](https://learn.sparkfun.com/tutorials/getting-started-with-the-myoware-20-muscle-sensor-ecosystem)

---

<div align="center">

**Phase 1 Research Complete**

🔬

</div>
