# WIA-CITY-009: 스마트 조명 표준 v1.0 💡

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-CITY-009
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 스마트 시티 (CITY)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [조명 기구 유형](#4-조명-기구-유형)
5. [제어 프로토콜](#5-제어-프로토콜)
6. [조광 및 색온도 제어](#6-조광-및-색온도-제어)
7. [일주기 조명 (Circadian Lighting)](#7-일주기-조명-circadian-lighting)
8. [센서 통합](#8-센서-통합)
9. [주광 이용 (Daylight Harvesting)](#9-주광-이용-daylight-harvesting)
10. [에너지 관리](#10-에너지-관리)
11. [스케줄링 및 씬 관리](#11-스케줄링-및-씬-관리)
12. [데이터 모델](#12-데이터-모델)
13. [API 명세](#13-api-명세)
14. [보안 및 개인정보보호](#14-보안-및-개인정보보호)
15. [성과 지표 (KPI)](#15-성과-지표-kpi)

---

## 1. 개요

### 1.1 목적

WIA-CITY-009 스마트 조명 표준은 도시 및 건물 내 조명 시스템의 지능화, 에너지 효율화, 사용자 경험 최적화를 위한 국제 표준입니다.

스마트 조명은 IoT 센서, 무선 통신, AI 제어를 통해 조명을 자동으로 조절하여 에너지를 절약하고, 사용자의 생체 리듬을 지원하며, 공간의 활용도를 높입니다.

### 1.2 핵심 원칙

- **적응형 조명 (Adaptive Lighting)**: 환경과 사용자 요구에 따라 자동 조절
- **에너지 효율 (Energy Efficiency)**: 최소 에너지로 최적의 조명 제공
- **인간 중심 (Human-Centric)**: 생체 리듬과 건강을 고려한 조명 설계
- **상호운용성 (Interoperability)**: 다양한 제조사와 프로토콜 간 호환
- **지능형 제어 (Intelligent Control)**: AI 기반 예측 및 최적화
- **개방형 표준 (Open Standards)**: 공개된 프로토콜과 API 제공
- **확장성 (Scalability)**: 소규모부터 대규모까지 유연한 확장

### 1.3 적용 대상

- 스마트 빌딩 및 오피스
- 스마트 홈
- 도시 가로등 및 공공 조명
- 상업 시설 (쇼핑몰, 레스토랑 등)
- 산업 시설 (공장, 창고 등)
- 의료 시설 (병원, 요양원 등)
- 교육 시설 (학교, 도서관 등)
- 문화 시설 (박물관, 극장 등)

---

## 2. 적용 범위

### 2.1 조명 시스템 구성요소

본 표준은 다음 구성요소에 적용됩니다:

- **조명 기구 (Fixtures)**: LED, OLED, 스마트 전구
- **센서 (Sensors)**: 점유 감지, 조도, 색온도, 모션 센서
- **제어 장치 (Controllers)**: 조광기, 스위치, 게이트웨이
- **통신 네트워크 (Communication)**: 유무선 프로토콜
- **관리 플랫폼 (Platform)**: 클라우드 기반 관리 시스템
- **사용자 인터페이스 (UI)**: 모바일 앱, 웹 대시보드, 음성 제어

### 2.2 조명 제어 유형

- **개별 제어 (Individual Control)**: 각 기구 독립 제어
- **그룹 제어 (Group Control)**: 여러 기구를 그룹으로 제어
- **씬 제어 (Scene Control)**: 사전 설정된 조명 상태
- **자동 제어 (Automatic Control)**: 센서 기반 자동 조절
- **스케줄 제어 (Scheduled Control)**: 시간 기반 자동화
- **원격 제어 (Remote Control)**: 클라우드를 통한 원격 관리

### 2.3 관리 단계

- 시스템 설계 및 계획
- 기구 선정 및 설치
- 네트워크 구축 및 연결
- 센서 배치 및 캘리브레이션
- 제어 로직 설정
- 모니터링 및 최적화
- 유지보수 및 업그레이드

---

## 3. 용어 정의

### 3.1 조명 측정 단위

- **조도 (Illuminance)**: 표면에 도달하는 빛의 양, 단위: lux (lx)
- **휘도 (Luminance)**: 발광 표면의 밝기, 단위: cd/m²
- **광속 (Luminous Flux)**: 광원이 방출하는 총 빛의 양, 단위: lm (루멘)
- **광도 (Luminous Intensity)**: 특정 방향의 빛의 세기, 단위: cd (칸델라)
- **색온도 (Color Temperature)**: 빛의 색상 특성, 단위: K (켈빈)
- **연색지수 (CRI)**: 색재현성 지수, 범위: 0-100

### 3.2 스마트 조명 기술 용어

- **CCT (Correlated Color Temperature)**: 상관 색온도
- **Tunable White**: 색온도 조절 가능 조명
- **RGB/RGBW**: 다색 LED (Red, Green, Blue, White)
- **PWM (Pulse Width Modulation)**: 펄스 폭 변조 (조광 방식)
- **0-10V Dimming**: 아날로그 조광 프로토콜
- **DALI (Digital Addressable Lighting Interface)**: 디지털 조명 제어 프로토콜
- **DMX512**: 무대 조명 제어 프로토콜
- **Zigbee**: 저전력 메시 네트워크 프로토콜
- **BLE (Bluetooth Low Energy)**: 저전력 블루투스
- **PoE (Power over Ethernet)**: 이더넷을 통한 전력 공급

### 3.3 센서 및 제어

- **Occupancy Sensor**: 점유 감지 센서
- **Daylight Sensor**: 주광 센서
- **PIR (Passive Infrared)**: 수동 적외선 센서
- **Microwave Sensor**: 마이크로파 센서
- **Photocell**: 광전 센서
- **Lux Meter**: 조도계
- **Smart Switch**: 스마트 스위치
- **Gateway**: 게이트웨이 (프로토콜 변환 장치)

### 3.4 생체 리듬 관련 용어

- **일주기 리듬 (Circadian Rhythm)**: 24시간 주기의 생체 리듬
- **멜라토닌 (Melatonin)**: 수면을 조절하는 호르몬
- **멜라놉신 (Melanopsin)**: 청색광에 민감한 망막 광수용체
- **EML (Equivalent Melanopic Lux)**: 멜라놉신 등가 조도
- **CS (Circadian Stimulus)**: 일주기 자극 지수

---

## 4. 조명 기구 유형

### 4.1 LED 조명

#### 4.1.1 일반 LED

**특징:**
- 에너지 효율: 80-150 lm/W
- 수명: 50,000-100,000 시간 (L70)
- 즉시 점등 (순간 점등)
- 조광 가능 (0-100%)
- 색온도 범위: 2700K-6500K

**적용 분야:**
- 실내 조명 (다운라이트, 패널등)
- 외부 조명 (가로등, 투광등)
- 장식 조명

#### 4.1.2 Tunable White LED

**특징:**
- 색온도 조절: 2700K-6500K
- 이중 LED 칩 (따뜻한 백색 + 차가운 백색)
- 일주기 조명 지원
- 동적 색온도 변화 가능

**데이터 모델:**
```typescript
interface TunableWhiteLED {
  minColorTemperature: number;  // K (예: 2700)
  maxColorTemperature: number;  // K (예: 6500)
  currentCCT: number;            // K
  warmLEDPower: number;          // % (0-100)
  coolLEDPower: number;          // % (0-100)
  transitionTime: number;        // ms
}
```

#### 4.1.3 RGB/RGBW LED

**특징:**
- 1,600만 색상 표현 가능
- 동적 색상 변화
- 특수 효과 (페이드, 스트로브 등)
- RGBW: 백색 LED 추가로 순백색 표현 향상

**적용 분야:**
- 장식 조명
- 무대 조명
- 광고판 조명
- 건축물 외관 조명

### 4.2 OLED 조명

#### 4.2.1 특징

- 면광원 (발광면 전체가 균일하게 발광)
- 얇고 유연한 형태 (플렉시블 OLED)
- 낮은 휘도로 눈부심 최소화
- 우수한 연색지수 (CRI > 90)
- 넓은 시야각

#### 4.2.2 한계점

- 현재 광효율: 40-80 lm/W (LED보다 낮음)
- 수명: 20,000-50,000 시간 (LED보다 짧음)
- 높은 제조 비용
- 대면적 제작의 어려움

#### 4.2.3 적용 분야

- 고급 실내 조명 (천장등, 벽등)
- 장식 조명
- 자동차 내부 조명
- 웨어러블 조명

### 4.3 스마트 전구

#### 4.3.1 기능

- WiFi/Zigbee/BLE 내장
- 모바일 앱 제어
- 음성 제어 (Alexa, Google Home 등)
- 색온도/색상 조절
- 스케줄링
- 씬 저장

#### 4.3.2 데이터 모델

```typescript
interface SmartBulb {
  bulbId: string;
  manufacturer: string;
  model: string;

  // 하드웨어 사양
  socketType: 'E26' | 'E27' | 'GU10' | 'B22';
  wattage: number;              // W
  lumens: number;               // lm
  efficacy: number;             // lm/W

  // 기능
  capabilities: {
    dimming: boolean;
    tunableWhite: boolean;
    rgbColor: boolean;
    powerMeasurement: boolean;
    firmware_ota: boolean;      // 무선 펌웨어 업데이트
  };

  // 통신
  protocols: ('wifi' | 'zigbee' | 'ble' | 'thread')[];

  // 현재 상태
  state: {
    on: boolean;
    brightness: number;         // % (0-100)
    colorTemperature?: number;  // K
    color?: {
      hue: number;              // 0-360
      saturation: number;       // % (0-100)
    };
    powerConsumption?: number;  // W
  };
}
```

---

## 5. 제어 프로토콜

### 5.1 DALI (Digital Addressable Lighting Interface)

#### 5.1.1 개요

- **표준:** IEC 62386
- **전송 방식:** 2선식 디지털 통신
- **주소 지정:** 최대 64개 기구 (단일 라인)
- **그룹:** 최대 16개 그룹
- **씬:** 최대 16개 씬
- **조광 해상도:** 254 레벨 (0-254)

#### 5.1.2 명령 구조

```
DALI 명령 프레임 (16비트):
┌────────┬────────────────┐
│ 주소   │ 데이터/명령     │
│ 8비트  │ 8비트          │
└────────┴────────────────┘

주소 유형:
- 단일 주소: 0-63 (개별 기구)
- 그룹 주소: 64-79 (그룹 0-15)
- 브로드캐스트: 255 (모든 기구)

명령 예시:
- 직접 조광: 0-254 (밝기 레벨)
- OFF: 0
- ON (이전 레벨): RECALL MAX LEVEL
- 씬 호출: GO TO SCENE 0-15
```

#### 5.1.3 DALI-2 확장

- 색온도 제어 (DALI Device Type 8)
- 센서 통합 (조도, 점유 센서)
- 에너지 모니터링
- 진단 정보

#### 5.1.4 데이터 모델

```typescript
interface DALIDevice {
  daliAddress: number;          // 0-63
  groups: number[];             // 0-15
  scenes: DALIScene[];          // 최대 16개

  capabilities: {
    dimming: boolean;
    colorControl: boolean;
    powerMeasurement: boolean;
    deviceType: number;         // DALI Device Type
  };

  status: {
    lampFailure: boolean;
    lampOn: boolean;
    limitError: boolean;
    fadeRunning: boolean;
    actualLevel: number;        // 0-254
    powerOn: boolean;
  };
}

interface DALIScene {
  sceneNumber: number;          // 0-15
  level: number;                // 0-254
  fadeTime: number;             // ms
}
```

### 5.2 DMX512

#### 5.2.1 개요

- **표준:** ANSI E1.11, USITT DMX512-A
- **용도:** 무대/엔터테인먼트 조명 제어
- **채널:** 512 채널 (유니버스당)
- **전송 방식:** RS-485
- **업데이트 속도:** 최대 44Hz

#### 5.2.2 채널 할당

```
DMX512 채널 구조 (RGB 기구 예시):

채널 1: Red (0-255)
채널 2: Green (0-255)
채널 3: Blue (0-255)
채널 4: White (0-255) - RGBW의 경우
채널 5: Dimmer (0-255)
채널 6: Strobe (0-255)
채널 7: Mode (색상 모드 선택)

다채널 기구는 여러 채널을 연속으로 사용
```

#### 5.2.3 데이터 모델

```typescript
interface DMXUniverse {
  universeId: number;
  channels: number[];           // 512개 (0-255 각각)
  updateRate: number;           // Hz
  fixtures: DMXFixture[];
}

interface DMXFixture {
  fixtureId: string;
  startChannel: number;         // 1-512
  channelCount: number;         // 기구가 사용하는 채널 수
  channelMap: {
    [channel: number]: 'red' | 'green' | 'blue' | 'white' | 'dimmer' | 'strobe' | 'pan' | 'tilt' | 'mode';
  };
}
```

### 5.3 Zigbee

#### 5.3.1 개요

- **표준:** IEEE 802.15.4
- **주파수:** 2.4 GHz
- **범위:** 10-100m (실내)
- **토폴로지:** 메시 네트워크
- **노드:** 최대 65,000개 (네트워크당)
- **전력:** 저전력 (배터리 동작 가능)

#### 5.3.2 Zigbee 조명 프로필

- **ZLL (Zigbee Light Link)**: 가정용 스마트 조명
- **ZHA (Zigbee Home Automation)**: 홈 오토메이션
- **Zigbee 3.0**: 통합 프로필 (ZLL + ZHA)

#### 5.3.3 클러스터

```
On/Off 클러스터 (0x0006):
- On, Off, Toggle

Level Control 클러스터 (0x0008):
- Move to Level (조광)
- Move, Step (연속 조광)
- Stop

Color Control 클러스터 (0x0300):
- Move to Hue (색상 변경)
- Move to Saturation (채도 변경)
- Move to Color Temperature (색온도 변경)
```

#### 5.3.4 데이터 모델

```typescript
interface ZigbeeLight {
  ieeeAddress: string;          // 64비트 MAC 주소
  networkAddress: number;       // 16비트 네트워크 주소
  endpoint: number;

  clusters: {
    onOff: boolean;
    levelControl: boolean;
    colorControl: boolean;
    scenes: boolean;
    groups: boolean;
  };

  state: {
    on: boolean;
    brightness: number;         // 0-254
    colorMode: 'hs' | 'xy' | 'ct';
    hue?: number;               // 0-254
    saturation?: number;        // 0-254
    colorTemp?: number;         // mireds (1,000,000/K)
    x?: number;                 // CIE x (0-1)
    y?: number;                 // CIE y (0-1)
  };
}
```

### 5.4 BLE (Bluetooth Low Energy)

#### 5.4.1 Bluetooth Mesh

- **표준:** Bluetooth Mesh Profile 1.0
- **토폴로지:** 메시 네트워크
- **노드:** 최대 32,767개
- **범위:** 10-30m (노드간), 메시로 확장
- **프로비저닝:** 스마트폰을 통한 쉬운 설정

#### 5.4.2 메시지 유형

```
Generic OnOff Model:
- Get, Set, Set Unacknowledged

Generic Level Model:
- Get, Set, Delta Set, Move Set

Light Lightness Model:
- Get, Set (밝기 제어)

Light CTL Model:
- Get, Set (색온도 제어)
```

#### 5.4.3 데이터 모델

```typescript
interface BLEMeshLight {
  uuid: string;                 // 128비트 UUID
  unicastAddress: number;       // 16비트 유니캐스트 주소

  elements: {
    elementIndex: number;
    models: ('generic_onoff' | 'generic_level' | 'light_lightness' | 'light_ctl')[];
  }[];

  subscriptions: number[];      // 그룹 주소
  publications: {
    address: number;
    period: number;             // ms
  };
}
```

### 5.5 프로토콜 비교

| 프로토콜 | 거리 | 노드 수 | 속도 | 전력 | 용도 |
|---------|------|---------|------|------|------|
| DALI | 300m | 64 (라인당) | 1.2 kbps | 저 | 상업용 조명 |
| DMX512 | 400m | 512 (유니버스당) | 250 kbps | 중 | 무대 조명 |
| Zigbee | 100m | 65,000 | 250 kbps | 저 | 스마트홈 |
| BLE Mesh | 30m | 32,767 | 1 Mbps | 초저 | 스마트홈 |
| WiFi | 50m | 제한없음 | 54+ Mbps | 고 | 범용 |

---

## 6. 조광 및 색온도 제어

### 6.1 조광 기술

#### 6.1.1 PWM (Pulse Width Modulation)

**원리:**
- 고속 온/오프 스위칭 (수백 Hz ~ 수십 kHz)
- 듀티 사이클로 밝기 조절
- 0-100% 무단계 조광

**장점:**
- 전 범위 조광 가능 (0-100%)
- 색온도 변화 없음
- 효율 우수

**단점:**
- 플리커 발생 가능 (저주파 PWM)
- EMI (전자파 간섭) 발생 가능

**권장 주파수:**
- 최소 1 kHz (플리커 방지)
- 권장 5 kHz 이상

```typescript
interface PWMDimming {
  frequency: number;            // Hz (1000-50000 권장)
  dutyCycle: number;            // % (0-100)
  resolution: number;           // bits (8-16)
  brightness: number;           // % (0-100)
}
```

#### 6.1.2 0-10V 조광

**원리:**
- 0-10V DC 신호로 밝기 제어
- 0V = 최소 밝기 (보통 10%)
- 10V = 최대 밝기 (100%)

**장점:**
- 단순하고 안정적
- 긴 케이블 거리 (수백 미터)
- EMI 없음

**단점:**
- 완전 소등 불가 (별도 스위치 필요)
- 양방향 통신 불가

#### 6.1.3 Phase-Cut 조광 (트라이악 조광)

**원리:**
- AC 파형의 일부를 잘라내어 조광
- Forward Phase (선행 위상): AC 파형 앞부분 차단
- Reverse Phase (후행 위상): AC 파형 뒷부분 차단

**장점:**
- 기존 인프라 사용 가능
- 저렴한 비용

**단점:**
- LED와 호환성 문제
- 플리커, 버즈음 발생 가능
- 조광 범위 제한적

### 6.2 조광 곡선 (Dimming Curves)

#### 6.2.1 선형 (Linear)

```
밝기(%) = 입력(%) × 1

예:
입력 50% → 50% 밝기
입력 25% → 25% 밝기
```

**특징:**
- 단순하고 예측 가능
- 낮은 레벨에서 변화 감지 어려움

#### 6.2.2 로그 (Logarithmic)

```
밝기(%) = log(입력) × K

인간의 시각 인지에 맞춘 곡선
낮은 레벨에서 더 세밀한 제어
```

**특징:**
- 인간 눈의 인지 특성 반영
- 전 범위에서 균일한 변화 인지

#### 6.2.3 S-Curve

```
초기: 천천히 증가
중간: 빠르게 증가
말기: 천천히 증가
```

**특징:**
- 부드러운 조광 경험
- 자연스러운 페이드 효과

#### 6.2.4 데이터 모델

```typescript
enum DimmingCurve {
  LINEAR = 'linear',
  LOGARITHMIC = 'logarithmic',
  S_CURVE = 's_curve',
  CUSTOM = 'custom',
}

interface DimmingProfile {
  curve: DimmingCurve;
  minLevel: number;             // % (최소 조광 레벨, 예: 1%)
  maxLevel: number;             // % (최대 조광 레벨, 예: 100%)
  fadeTime: number;             // ms (변화 속도)
  customCurve?: number[];       // 커스텀 곡선 (0-100, 256 포인트)
}
```

### 6.3 색온도 제어 (CCT)

#### 6.3.1 색온도 범위

| 색온도 | 색상 | 용도 |
|--------|------|------|
| 2700K | 따뜻한 백색 | 저녁 조명, 휴식 공간 |
| 3000K | 따뜻한 중성 백색 | 거실, 침실 |
| 3500K | 중성 백색 | 주방, 욕실 |
| 4000K | 차가운 백색 | 사무실, 작업 공간 |
| 5000K | 주광색 | 독서, 상업 공간 |
| 6500K | 한낮 햇빛 | 스튜디오, 정밀 작업 |

#### 6.3.2 Tunable White 제어

```typescript
interface TunableWhiteControl {
  targetCCT: number;            // K (목표 색온도)
  warmLEDChannel: number;       // 0-255 (2700K LED)
  coolLEDChannel: number;       // 0-255 (6500K LED)
  brightness: number;           // % (0-100)
  transitionTime: number;       // ms

  // 계산 공식
  calculateChannels(targetCCT: number, brightness: number): {
    warm: number;
    cool: number;
  };
}

// 구현 예시
function calculateChannels(targetCCT: number, brightness: number) {
  const warmCCT = 2700;
  const coolCCT = 6500;

  // 색온도 비율 계산
  const ratio = (targetCCT - warmCCT) / (coolCCT - warmCCT);
  const coolRatio = Math.max(0, Math.min(1, ratio));
  const warmRatio = 1 - coolRatio;

  // 밝기 적용
  const warm = warmRatio * (brightness / 100) * 255;
  const cool = coolRatio * (brightness / 100) * 255;

  return { warm, cool };
}
```

#### 6.3.3 일주기 색온도 스케줄

```typescript
interface CircadianCCTSchedule {
  timePoints: {
    time: string;               // HH:MM
    cct: number;                // K
    brightness: number;         // %
  }[];
}

// 예시: 자연스러운 일주기 리듬
const circadianSchedule: CircadianCCTSchedule = {
  timePoints: [
    { time: '06:00', cct: 2700, brightness: 20 },   // 아침 - 따뜻한 시작
    { time: '09:00', cct: 4500, brightness: 100 },  // 오전 - 각성
    { time: '12:00', cct: 5500, brightness: 100 },  // 정오 - 최대 각성
    { time: '15:00', cct: 5000, brightness: 90 },   // 오후
    { time: '18:00', cct: 4000, brightness: 80 },   // 저녁
    { time: '21:00', cct: 3000, brightness: 50 },   // 밤 - 휴식
    { time: '23:00', cct: 2700, brightness: 10 },   // 취침 전
  ]
};
```

---

## 7. 일주기 조명 (Circadian Lighting)

### 7.1 개요

일주기 조명은 인간의 생체 시계에 맞춰 색온도와 밝기를 자동으로 조절하여 각성, 수면, 건강을 최적화하는 조명 시스템입니다.

### 7.2 멜라놉신과 청색광

#### 7.2.1 멜라놉신 (Melanopsin)

- 망막의 ipRGC (intrinsically photosensitive Retinal Ganglion Cells)에 존재
- 청색광 (460-480nm)에 가장 민감
- 일주기 리듬과 각성을 조절
- 멜라토닌 분비를 억제

#### 7.2.2 청색광 영향

**낮 시간 (고청색광 노출):**
- 멜라토닌 억제 → 각성 상태 유지
- 집중력, 생산성 향상
- 기분 개선

**밤 시간 (고청색광 노출):**
- 멜라토닌 억제 → 수면 방해
- 수면 시작 지연
- 수면 질 저하
- 건강 악영향 (장기적)

### 7.3 EML (Equivalent Melanopic Lux)

#### 7.3.1 정의

EML은 빛의 멜라놉신 자극 효과를 정량화한 지표입니다.

```
EML = ∫ S(λ) × M(λ) × dλ

S(λ): 스펙트럼 분포
M(λ): 멜라놉신 감도 함수
```

#### 7.3.2 권장 EML 레벨

| 시간대 | EML (lux) | 목적 | 권장 CCT |
|--------|-----------|------|----------|
| 아침 (6-9시) | 200-300 | 각성 촉진 | 5000-6500K |
| 오전 (9-12시) | 300-500 | 최대 각성 | 5500-6500K |
| 정오 (12-15시) | 400-600 | 지속적 각성 | 5000-6000K |
| 오후 (15-18시) | 250-400 | 활동 유지 | 4500-5500K |
| 저녁 (18-21시) | 100-200 | 휴식 전환 | 3500-4500K |
| 밤 (21-23시) | 10-50 | 수면 준비 | 2700-3000K |
| 취침 전 (23-6시) | < 10 | 멜라토닌 보호 | 2200-2700K |

### 7.4 일주기 조명 구현

#### 7.4.1 자동 스케줄

```typescript
interface CircadianLightingSchedule {
  enabled: boolean;
  latitude: number;             // 일출/일몰 계산용
  longitude: number;
  timezone: string;

  // 동적 조정
  sunriseSyncEnabled: boolean;  // 일출 시간에 동기화
  sunsetSyncEnabled: boolean;   // 일몰 시간에 동기화

  // 사용자 맞춤 설정
  wakeTime: string;             // HH:MM (기상 시간)
  sleepTime: string;            // HH:MM (취침 시간)

  // 조명 프로파일
  profiles: {
    morning: LightingProfile;
    day: LightingProfile;
    evening: LightingProfile;
    night: LightingProfile;
  };
}

interface LightingProfile {
  brightness: number;           // %
  colorTemperature: number;     // K
  eml: number;                  // Equivalent Melanopic Lux
  transitionDuration: number;   // minutes
}
```

#### 7.4.2 실시간 계산

```typescript
function calculateCircadianLighting(
  currentTime: Date,
  location: { lat: number; lon: number },
  userPreferences: CircadianLightingSchedule
): LightingState {
  // 1. 일출/일몰 시간 계산
  const sunTimes = calculateSunTimes(currentTime, location);

  // 2. 현재 시간대 결정
  const phase = determineCircadianPhase(currentTime, sunTimes, userPreferences);

  // 3. 목표 조명 상태 계산
  const target = interpolateLightingProfile(phase, userPreferences.profiles);

  return target;
}

enum CircadianPhase {
  WAKE = 'wake',                // 기상
  MORNING = 'morning',          // 아침
  DAY = 'day',                  // 낮
  EVENING = 'evening',          // 저녁
  NIGHT = 'night',              // 밤
  SLEEP = 'sleep',              // 취침
}
```

### 7.5 건강 및 웰빙 효과

#### 7.5.1 입증된 효과

- **수면 질 개선**: 적절한 멜라토닌 리듬 유지
- **각성 상태 향상**: 낮 시간 생산성 증가
- **기분 개선**: 계절성 우울증 (SAD) 완화
- **일주기 리듬 안정화**: 교대 근무자, 제트랙 회복
- **인지 기능 향상**: 집중력, 기억력 개선

#### 7.5.2 데이터 수집 및 피드백

```typescript
interface CircadianHealthMetrics {
  userId: string;
  period: {
    start: Date;
    end: Date;
  };

  // 조명 노출 데이터
  exposure: {
    averageDayEML: number;      // lux (낮 시간 평균)
    averageNightEML: number;    // lux (밤 시간 평균)
    blueExposureDuration: number; // minutes (청색광 노출 시간)
  };

  // 주관적 평가 (선택)
  subjective?: {
    sleepQuality: number;       // 1-10
    daytimeAlertness: number;   // 1-10
    mood: number;               // 1-10
    eyeStrain: number;          // 1-10
  };

  // 권장 사항
  recommendations: string[];
}
```

---

## 8. 센서 통합

### 8.1 점유 감지 센서 (Occupancy Sensors)

#### 8.1.1 PIR (Passive Infrared) 센서

**원리:**
- 적외선 복사열 감지
- 움직임 있을 때만 감지

**특징:**
- 감지 범위: 5-15m
- 감지 각도: 90-360°
- 전력: 저전력
- 비용: 저렴

**한계:**
- 정지 상태 미감지 (타임아웃 후 소등 문제)
- 온도 영향 (여름철 감지 저하)

```typescript
interface PIRSensor {
  sensorId: string;
  location: Location;

  specifications: {
    detectionRange: number;     // m
    detectionAngle: number;     // degrees
    sensitivity: number;        // 1-10
    timeDelay: number;          // seconds (감지 후 유지 시간)
  };

  state: {
    occupied: boolean;
    lastMotionDetected: Date;
    temperature?: number;       // °C
  };
}
```

#### 8.1.2 마이크로파 센서

**원리:**
- 마이크로파 발사 및 반사파 감지
- 도플러 효과로 움직임 감지

**특징:**
- 감지 범위: 10-30m
- 벽 투과 가능 (단점이 될 수도 있음)
- 정지 상태에서도 미세 움직임 감지 가능

**한계:**
- 전력 소모 높음
- 오감지 가능 (벽 너머 움직임)
- 비용 높음

#### 8.1.3 듀얼 테크놀로지 센서

**원리:**
- PIR + 마이크로파 조합
- AND 로직: 두 센서 모두 감지 시 점유

**장점:**
- 오감지 최소화
- 높은 정확도

**단점:**
- 높은 비용
- 복잡한 설정

#### 8.1.4 카메라 기반 점유 감지

**원리:**
- 영상 분석 (Computer Vision)
- AI 기반 사람 감지 및 카운팅

**장점:**
- 정확한 인원 수 파악
- 위치 추적 가능
- 행동 분석 가능 (앉아 있음, 서 있음 등)

**단점:**
- 프라이버시 문제
- 높은 비용
- 연산 부하

```typescript
interface VisionOccupancySensor {
  sensorId: string;
  location: Location;
  cameraId: string;

  capabilities: {
    peopleCount: boolean;
    positionTracking: boolean;
    activityRecognition: boolean;
    anonymization: boolean;     // 프라이버시 보호 (얼굴 블러 등)
  };

  state: {
    occupantCount: number;
    positions?: { x: number; y: number }[];
    activityLevel: 'low' | 'medium' | 'high';
  };
}
```

### 8.2 조도 센서 (Light Sensors)

#### 8.2.1 Photocell (광전 센서)

**원리:**
- 빛의 강도에 따라 저항 변화
- 간단한 아날로그 센서

**특징:**
- 측정 범위: 1-100,000 lux
- 정확도: ±10-20%
- 저렴한 비용

#### 8.2.2 디지털 조도 센서

**원리:**
- 포토다이오드 + ADC
- 디지털 출력 (I2C, SPI 등)

**특징:**
- 정확도: ±5%
- 자동 범위 조정
- 캘리브레이션 기능

**예시:**
- BH1750 (0.11-100,000 lux)
- VEML7700 (0-120,000 lux)
- TSL2591 (188 μlux - 88,000 lux, 고감도)

```typescript
interface LightSensor {
  sensorId: string;
  location: Location;

  specifications: {
    range: { min: number; max: number }; // lux
    accuracy: number;           // %
    samplingRate: number;       // Hz
    spectralResponse: string;   // 'human_eye' | 'broad_spectrum'
  };

  state: {
    illuminance: number;        // lux
    timestamp: Date;
    calibrationDate?: Date;
  };
}
```

### 8.3 센서 융합 (Sensor Fusion)

#### 8.3.1 다중 센서 통합

```typescript
interface MultiSensorZone {
  zoneId: string;
  zoneName: string;
  area: number;                 // m²

  sensors: {
    occupancy: PIRSensor[];
    light: LightSensor[];
    temperature?: TemperatureSensor[];
  };

  // 융합된 상태
  aggregatedState: {
    occupied: boolean;
    occupantCount?: number;
    averageIlluminance: number;  // lux
    temperature?: number;        // °C
    confidence: number;          // 0-1 (센서 융합 신뢰도)
  };

  // 제어 로직
  control: {
    targetIlluminance: number;   // lux
    occupiedBrightness: number;  // %
    vacantBrightness: number;    // %
    timeDelay: number;           // seconds (점유→비점유 전환 지연)
  };
}
```

#### 8.3.2 베이지안 융합

```typescript
function fuseSensorReadings(
  pirOccupancy: boolean,
  lightLevel: number,
  previousState: boolean,
  priorProbability: number = 0.5
): { occupied: boolean; confidence: number } {
  // 각 센서의 신뢰도
  const pirConfidence = 0.85;   // PIR 센서 정확도
  const lightConfidence = 0.70; // 조도 기반 추론 정확도

  // 베이지안 확률 계산
  let probability = priorProbability;

  if (pirOccupancy) {
    probability *= pirConfidence;
  } else {
    probability *= (1 - pirConfidence);
  }

  // 조도 변화로 점유 추론
  const lightThreshold = 50; // lux
  if (lightLevel > lightThreshold) {
    probability *= lightConfidence;
  }

  // 정규화
  const normalizedProb = probability / (probability + (1 - probability));

  return {
    occupied: normalizedProb > 0.5,
    confidence: normalizedProb
  };
}
```

---

## 9. 주광 이용 (Daylight Harvesting)

### 9.1 개요

주광 이용은 자연광을 최대한 활용하여 인공 조명을 줄이고 에너지를 절약하는 기술입니다.

### 9.2 제어 전략

#### 9.2.1 오픈 루프 (Open-loop)

**원리:**
- 외부 조도 센서 측정
- 실내 조명 조도 사전 계산
- 피드백 없이 제어

**장점:**
- 단순한 구조
- 저비용

**단점:**
- 부정확 (창문 더러움, 커튼 등 변수 미반영)

#### 9.2.2 클로즈드 루프 (Closed-loop)

**원리:**
- 실내 조도 센서 측정
- 목표 조도와 비교
- 피드백 제어

**장점:**
- 정확한 조도 유지
- 자동 보정

**단점:**
- 센서 위치 중요
- 불안정성 가능 (발진)

#### 9.2.3 하이브리드

**원리:**
- 외부 + 실내 조도 센서
- 지능형 알고리즘

**장점:**
- 최적의 정확도와 안정성

```typescript
interface DaylightHarvestingControl {
  zoneId: string;

  sensors: {
    outdoor: LightSensor;       // 외부 조도
    indoor: LightSensor[];      // 실내 조도 (여러 위치)
  };

  target: {
    illuminance: number;        // lux (목표 조도)
    uniformity: number;         // 0-1 (균일도)
  };

  control: {
    mode: 'open_loop' | 'closed_loop' | 'hybrid';
    algorithm: 'proportional' | 'pid' | 'fuzzy' | 'ml';
    updateInterval: number;     // seconds

    // PID 파라미터
    pid?: {
      kp: number;               // 비례 게인
      ki: number;               // 적분 게인
      kd: number;               // 미분 게인
    };
  };

  state: {
    outdoorIlluminance: number; // lux
    indoorIlluminance: number;  // lux (평균)
    artificialContribution: number; // % (인공 조명 기여도)
    daylightContribution: number;   // % (자연광 기여도)
    energySaved: number;        // kWh (누적)
  };
}
```

### 9.3 PID 제어

```typescript
class PIDController {
  private kp: number;
  private ki: number;
  private kd: number;
  private integral: number = 0;
  private previousError: number = 0;

  constructor(kp: number, ki: number, kd: number) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  update(targetLux: number, measuredLux: number, dt: number): number {
    const error = targetLux - measuredLux;

    // 적분항
    this.integral += error * dt;

    // 미분항
    const derivative = (error - this.previousError) / dt;

    // PID 출력
    const output =
      this.kp * error +
      this.ki * this.integral +
      this.kd * derivative;

    this.previousError = error;

    // 0-100% 범위로 제한
    return Math.max(0, Math.min(100, output));
  }
}
```

### 9.4 에너지 절감 효과

#### 9.4.1 예상 절감률

| 건물 유형 | 창문 면적 비율 | 절감률 |
|----------|---------------|--------|
| 사무실 (창가) | 40-60% | 50-70% |
| 사무실 (중앙) | 10-20% | 10-30% |
| 학교 | 30-50% | 40-60% |
| 상업 시설 | 20-40% | 30-50% |
| 공장 (천창) | 10-30% | 20-40% |

#### 9.4.2 측정 및 검증

```typescript
interface DaylightSavingsReport {
  period: {
    start: Date;
    end: Date;
  };

  baseline: {
    totalEnergyConsumption: number;     // kWh (주광 이용 전)
    averageDailyConsumption: number;    // kWh/day
  };

  withDaylight: {
    totalEnergyConsumption: number;     // kWh (주광 이용 후)
    averageDailyConsumption: number;    // kWh/day
    daylightContribution: number;       // % (자연광 기여도)
  };

  savings: {
    energySaved: number;                // kWh
    percentageSaved: number;            // %
    costSaved: number;                  // 원
    co2Reduced: number;                 // kg CO2
  };
}
```

---

## 10. 에너지 관리

### 10.1 전력 측정

#### 10.1.1 실시간 전력 모니터링

```typescript
interface PowerMeasurement {
  fixtureId: string;
  timestamp: Date;

  instantaneous: {
    voltage: number;            // V
    current: number;            // A
    power: number;              // W (유효 전력)
    apparentPower: number;      // VA (피상 전력)
    powerFactor: number;        // 0-1
  };

  cumulative: {
    energyConsumed: number;     // kWh (누적)
    operatingHours: number;     // hours
  };
}
```

#### 10.1.2 전력 예산 (Power Budget)

```typescript
interface PowerBudget {
  zoneId: string;
  period: 'daily' | 'weekly' | 'monthly';

  budget: {
    target: number;             // kWh (목표 소비량)
    current: number;            // kWh (현재 소비량)
    remaining: number;          // kWh (남은 예산)
    percentUsed: number;        // %
  };

  forecast: {
    expectedConsumption: number; // kWh (예상 총 소비량)
    exceedanceRisk: number;     // % (초과 위험도)
  };

  actions: {
    dimmingLevel: number;       // % (자동 조광 레벨)
    alertsEnabled: boolean;
    autoAdjustEnabled: boolean;
  };
}
```

### 10.2 에너지 절감 전략

#### 10.2.1 Task Tuning

**원리:**
- 작업 영역만 밝게, 주변은 어둡게
- 작업 조명 (Task Lighting) + 주변 조명 (Ambient Lighting)

**절감 효과:**
- 30-50% 에너지 절약

```typescript
interface TaskTuningZone {
  zoneId: string;

  areas: {
    task: {
      illuminance: number;      // lux (예: 500 lux)
      fixtures: string[];
    };
    ambient: {
      illuminance: number;      // lux (예: 200 lux)
      fixtures: string[];
    };
  };

  // 작업 영역 자동 감지
  taskDetection: {
    enabled: boolean;
    method: 'occupancy' | 'vision' | 'manual';
    adaptiveAdjustment: boolean;
  };
}
```

#### 10.2.2 Personal Control

**원리:**
- 사용자 개인 제어 가능
- 만족도 향상 + 에너지 절약

**연구 결과:**
- 개인 제어 가능 시 평균 15-30% 낮은 조도 선택
- 만족도는 오히려 증가

```typescript
interface PersonalLightingControl {
  userId: string;
  fixtureId: string;

  preferences: {
    defaultBrightness: number;  // %
    defaultCCT: number;         // K
    autoAdjust: boolean;        // 일주기 자동 조절
  };

  overrides: {
    manualOverride: boolean;    // 수동 조정 중
    overrideUntil?: Date;       // 수동 조정 종료 시간
  };

  usage: {
    averageBrightness: number;  // % (평균 사용 밝기)
    energySaved: number;        // kWh (표준 대비 절감)
  };
}
```

#### 10.2.3 Dimming During Unoccupied Hours

**원리:**
- 비점유 시간 조광 또는 소등
- 심야 시간대 조광 (가로등 등)

**절감 효과:**
- 40-60% 에너지 절약 (비점유 시간)

```typescript
interface UnoccupiedDimmingSchedule {
  zoneId: string;

  schedule: {
    occupiedHours: {
      start: string;            // HH:MM
      end: string;              // HH:MM
      brightness: number;       // %
    };
    unoccupiedHours: {
      brightness: number;       // % (예: 20%)
      offDelay: number;         // minutes (소등까지 지연)
    };
  };

  exceptions: {
    date: string;               // YYYY-MM-DD
    occupied: boolean;          // 예외적으로 점유
  }[];
}
```

### 10.3 에너지 분석 및 리포팅

#### 10.3.1 에너지 대시보드

```typescript
interface EnergyDashboard {
  overview: {
    totalPower: number;         // W (현재 총 전력)
    totalEnergy: number;        // kWh (기간 총 에너지)
    averagePowerFactor: number; // 평균 역률
    peakDemand: number;         // W (피크 전력)
    peakDemandTime: Date;
  };

  byZone: {
    zoneId: string;
    zoneName: string;
    power: number;              // W
    energy: number;             // kWh
    efficiency: number;         // lm/W (평균 효율)
  }[];

  trends: {
    hourly: { hour: number; energy: number }[];
    daily: { date: string; energy: number }[];
    monthly: { month: string; energy: number }[];
  };

  savings: {
    compared_to_baseline: number;      // % (베이스라인 대비)
    daylight_harvesting: number;       // kWh
    occupancy_control: number;         // kWh
    dimming: number;                   // kWh
    total: number;                     // kWh
  };
}
```

#### 10.3.2 이상 감지 (Anomaly Detection)

```typescript
interface EnergyAnomalyDetection {
  fixtureId: string;

  baseline: {
    averagePower: number;       // W (정상 평균)
    stdDeviation: number;       // W (표준 편차)
  };

  anomalies: {
    timestamp: Date;
    measuredPower: number;      // W
    expectedPower: number;      // W
    deviation: number;          // % (편차)
    severity: 'low' | 'medium' | 'high';
    possibleCauses: string[];   // ['lamp_failure', 'driver_malfunction', ...]
  }[];

  alerts: {
    enabled: boolean;
    threshold: number;          // % (이상 판정 임계값)
    notification: 'email' | 'sms' | 'push';
  };
}
```

---

## 11. 스케줄링 및 씬 관리

### 11.1 스케줄링

#### 11.1.1 시간 기반 스케줄

```typescript
interface TimeBasedSchedule {
  scheduleId: string;
  name: string;
  enabled: boolean;

  timeSlots: {
    daysOfWeek: number[];       // 0=일요일, 1=월요일, ..., 6=토요일
    startTime: string;          // HH:MM
    endTime: string;            // HH:MM
    action: ScheduleAction;
  }[];

  exceptions: {
    date: string;               // YYYY-MM-DD
    action: ScheduleAction | 'skip';
  }[];
}

interface ScheduleAction {
  type: 'on' | 'off' | 'dim' | 'scene' | 'auto';
  brightness?: number;          // % (type='dim'일 때)
  sceneId?: string;             // type='scene'일 때
  fadeTime?: number;            // ms (전환 시간)
}
```

#### 11.1.2 천문학적 스케줄 (Astronomical Schedule)

```typescript
interface AstronomicalSchedule {
  scheduleId: string;
  location: {
    latitude: number;
    longitude: number;
    timezone: string;
  };

  events: {
    event: 'sunrise' | 'sunset' | 'dawn' | 'dusk' | 'solar_noon';
    offset: number;             // minutes (이벤트로부터 오프셋)
    action: ScheduleAction;
  }[];

  // 계산된 오늘의 이벤트 시간
  todayEvents: {
    sunrise: Date;
    sunset: Date;
    dawn: Date;                 // 시민박명 시작
    dusk: Date;                 // 시민박명 종료
    solarNoon: Date;
  };
}

// 예시: 일몰 30분 후 켜기
const sunsetSchedule: AstronomicalSchedule = {
  scheduleId: 'sunset-on',
  location: { latitude: 37.5665, longitude: 126.9780, timezone: 'Asia/Seoul' },
  events: [
    {
      event: 'sunset',
      offset: 30,               // 일몰 후 30분
      action: { type: 'on', brightness: 100, fadeTime: 5000 }
    },
    {
      event: 'sunrise',
      offset: -30,              // 일출 30분 전
      action: { type: 'off', fadeTime: 10000 }
    }
  ],
  todayEvents: {
    sunrise: new Date('2025-12-25T07:45:00+09:00'),
    sunset: new Date('2025-12-25T17:20:00+09:00'),
    dawn: new Date('2025-12-25T07:15:00+09:00'),
    dusk: new Date('2025-12-25T17:50:00+09:00'),
    solarNoon: new Date('2025-12-25T12:32:00+09:00'),
  }
};
```

### 11.2 씬 관리 (Scene Management)

#### 11.2.1 씬 정의

씬은 여러 조명 기구의 상태를 하나의 프리셋으로 저장한 것입니다.

```typescript
interface LightingScene {
  sceneId: string;
  name: string;
  description?: string;
  icon?: string;                // 아이콘 식별자

  fixtureStates: {
    fixtureId: string;
    on: boolean;
    brightness?: number;        // %
    colorTemperature?: number;  // K
    color?: {
      hue: number;              // 0-360
      saturation: number;       // 0-100
    };
  }[];

  transition: {
    duration: number;           // ms
    curve: 'linear' | 'ease-in' | 'ease-out' | 'ease-in-out';
  };

  metadata: {
    category: 'work' | 'relax' | 'entertainment' | 'dining' | 'custom';
    createdBy?: string;
    createdAt: Date;
    tags?: string[];
  };
}
```

#### 11.2.2 사전 정의 씬

```typescript
// 집중 (Work/Focus)
const focusScene: LightingScene = {
  sceneId: 'focus',
  name: '집중',
  description: '생산적인 작업을 위한 밝고 시원한 조명',
  icon: 'lightbulb',
  fixtureStates: [
    { fixtureId: 'ceiling-1', on: true, brightness: 100, colorTemperature: 5000 },
    { fixtureId: 'desk-lamp', on: true, brightness: 100, colorTemperature: 5500 },
  ],
  transition: { duration: 2000, curve: 'ease-in-out' },
  metadata: { category: 'work', tags: ['productive', 'bright'] }
};

// 휴식 (Relax)
const relaxScene: LightingScene = {
  sceneId: 'relax',
  name: '휴식',
  description: '편안한 분위기의 따뜻한 조명',
  fixtureStates: [
    { fixtureId: 'ceiling-1', on: true, brightness: 40, colorTemperature: 2700 },
    { fixtureId: 'floor-lamp', on: true, brightness: 60, colorTemperature: 2700 },
  ],
  transition: { duration: 3000, curve: 'ease-out' },
  metadata: { category: 'relax', tags: ['cozy', 'warm'] }
};

// 영화 감상 (Movie)
const movieScene: LightingScene = {
  sceneId: 'movie',
  name: '영화',
  description: '영화 감상을 위한 어두운 조명',
  fixtureStates: [
    { fixtureId: 'ceiling-1', on: false },
    { fixtureId: 'strip-light', on: true, brightness: 10, color: { hue: 240, saturation: 80 } },
  ],
  transition: { duration: 5000, curve: 'ease-in-out' },
  metadata: { category: 'entertainment', tags: ['dim', 'ambient'] }
};

// 식사 (Dining)
const diningScene: LightingScene = {
  sceneId: 'dining',
  name: '식사',
  description: '식사를 위한 따뜻하고 아늑한 조명',
  fixtureStates: [
    { fixtureId: 'ceiling-1', on: true, brightness: 50, colorTemperature: 3000 },
    { fixtureId: 'table-pendant', on: true, brightness: 80, colorTemperature: 2700 },
  ],
  transition: { duration: 2000, curve: 'linear' },
  metadata: { category: 'dining', tags: ['warm', 'cozy'] }
};
```

#### 11.2.3 동적 씬 (Dynamic Scenes)

```typescript
interface DynamicScene extends LightingScene {
  dynamic: {
    enabled: boolean;
    type: 'fade' | 'pulse' | 'color_cycle' | 'random';
    parameters: {
      speed?: number;           // 1-10 (변화 속도)
      colorRange?: {
        minHue: number;
        maxHue: number;
      };
      brightnessRange?: {
        min: number;
        max: number;
      };
    };
  };
}

// 예시: 파티 모드 (색상 순환)
const partyScene: DynamicScene = {
  sceneId: 'party',
  name: '파티',
  description: '활기찬 파티를 위한 동적 조명',
  fixtureStates: [],            // 동적으로 변경됨
  transition: { duration: 500, curve: 'linear' },
  metadata: { category: 'entertainment', tags: ['dynamic', 'party'] },
  dynamic: {
    enabled: true,
    type: 'color_cycle',
    parameters: {
      speed: 5,
      colorRange: { minHue: 0, maxHue: 360 },
      brightnessRange: { min: 60, max: 100 }
    }
  }
};
```

### 11.3 조건부 자동화 (Conditional Automation)

#### 11.3.1 If-This-Then-That (IFTTT) 룰

```typescript
interface AutomationRule {
  ruleId: string;
  name: string;
  enabled: boolean;

  trigger: Trigger;
  conditions: Condition[];      // AND 조건
  actions: Action[];

  metadata: {
    createdBy?: string;
    createdAt: Date;
    lastTriggered?: Date;
    triggerCount: number;
  };
}

interface Trigger {
  type: 'time' | 'sensor' | 'state' | 'manual';

  // type='time'
  time?: {
    at: string;                 // HH:MM
    daysOfWeek?: number[];
  };

  // type='sensor'
  sensor?: {
    sensorId: string;
    parameter: 'occupancy' | 'illuminance' | 'temperature';
    operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
    value: number | boolean;
  };

  // type='state'
  state?: {
    fixtureId: string;
    parameter: 'on' | 'brightness' | 'cct';
    operator: '>' | '<' | '==' | '!=' | '>=' | '<=';
    value: number | boolean;
  };
}

interface Condition {
  type: 'time_range' | 'sensor' | 'state';

  timeRange?: {
    start: string;              // HH:MM
    end: string;                // HH:MM
  };

  sensor?: {
    sensorId: string;
    parameter: string;
    operator: string;
    value: number | boolean;
  };

  state?: {
    fixtureId: string;
    parameter: string;
    operator: string;
    value: number | boolean;
  };
}

interface Action {
  type: 'fixture' | 'scene' | 'notification' | 'api_call';

  fixture?: {
    fixtureId: string | 'all' | 'group:GROUP_ID';
    command: 'on' | 'off' | 'dim' | 'set_cct' | 'set_color';
    parameters?: {
      brightness?: number;
      colorTemperature?: number;
      color?: { hue: number; saturation: number };
      fadeTime?: number;
    };
  };

  scene?: {
    sceneId: string;
  };

  notification?: {
    message: string;
    channels: ('email' | 'sms' | 'push')[];
  };

  apiCall?: {
    url: string;
    method: 'GET' | 'POST' | 'PUT';
    body?: any;
  };
}
```

#### 11.3.2 예시 자동화 룰

```typescript
// 예시 1: 저녁 7시에 점유 중이면 휴식 모드
const eveningRelaxRule: AutomationRule = {
  ruleId: 'evening-relax',
  name: '저녁 휴식 모드',
  enabled: true,
  trigger: {
    type: 'time',
    time: { at: '19:00', daysOfWeek: [1, 2, 3, 4, 5] } // 평일
  },
  conditions: [
    {
      type: 'sensor',
      sensor: {
        sensorId: 'living-room-pir',
        parameter: 'occupancy',
        operator: '==',
        value: true
      }
    }
  ],
  actions: [
    {
      type: 'scene',
      scene: { sceneId: 'relax' }
    }
  ],
  metadata: {
    createdAt: new Date('2025-12-01'),
    triggerCount: 0
  }
};

// 예시 2: 조도가 100 lux 이하면 조명 켜기
const lowLightRule: AutomationRule = {
  ruleId: 'low-light-on',
  name: '어두우면 조명 켜기',
  enabled: true,
  trigger: {
    type: 'sensor',
    sensor: {
      sensorId: 'office-light-sensor',
      parameter: 'illuminance',
      operator: '<',
      value: 100
    }
  },
  conditions: [
    {
      type: 'time_range',
      timeRange: { start: '08:00', end: '18:00' } // 근무 시간만
    }
  ],
  actions: [
    {
      type: 'fixture',
      fixture: {
        fixtureId: 'group:office-lights',
        command: 'dim',
        parameters: { brightness: 80, fadeTime: 2000 }
      }
    }
  ],
  metadata: {
    createdAt: new Date('2025-12-15'),
    triggerCount: 0
  }
};
```

---

## 12. 데이터 모델

### 12.1 코어 엔티티

#### 12.1.1 조명 기구 (Lighting Fixture)

```typescript
interface LightingFixture {
  // 식별자
  fixtureId: string;
  name: string;
  description?: string;

  // 위치
  location: {
    building?: string;
    floor?: string;
    room?: string;
    zone?: string;
    coordinates?: {
      x: number;
      y: number;
      z?: number;
    };
    gps?: {
      latitude: number;
      longitude: number;
    };
  };

  // 하드웨어 사양
  hardware: {
    manufacturer: string;
    model: string;
    type: 'led' | 'oled' | 'fluorescent' | 'halogen' | 'incandescent';
    wattage: number;            // W (정격 전력)
    lumens: number;             // lm (정격 광속)
    efficacy: number;           // lm/W
    colorTemperature: number | { min: number; max: number }; // K
    cri: number;                // 0-100
    beamAngle?: number;         // degrees
    ipRating?: string;          // IP65 등
    dimming: boolean;
    tunableWhite: boolean;
    rgbColor: boolean;
  };

  // 통신 및 제어
  communication: {
    protocols: ('dali' | 'dmx' | 'zigbee' | 'ble' | 'wifi' | '0-10v')[];
    address?: {
      dali?: number;            // 0-63
      dmx?: number;             // 1-512
      zigbee?: string;          // IEEE 주소
      ble?: string;             // UUID
      ip?: string;              // IP 주소
    };
    gateway?: string;           // 게이트웨이 ID
  };

  // 현재 상태
  state: {
    on: boolean;
    reachable: boolean;         // 통신 가능 여부
    brightness: number;         // % (0-100)
    colorTemperature?: number;  // K
    color?: {
      hue: number;              // 0-360
      saturation: number;       // 0-100
    };
    power?: number;             // W (현재 전력)
    energy?: number;            // kWh (누적 에너지)
    lastUpdate: Date;
  };

  // 그룹 및 씬
  groups: string[];             // 그룹 ID 목록
  scenes: string[];             // 씬 ID 목록

  // 설치 및 유지보수
  installation: {
    installDate: Date;
    warrantyUntil?: Date;
    lastMaintenance?: Date;
    nextMaintenance?: Date;
  };

  // 메타데이터
  metadata: {
    tags?: string[];
    customFields?: { [key: string]: any };
  };
}
```

#### 12.1.2 센서 (Sensor)

```typescript
interface Sensor {
  sensorId: string;
  name: string;
  type: 'occupancy' | 'light' | 'temperature' | 'humidity' | 'co2';

  location: {
    building?: string;
    floor?: string;
    room?: string;
    zone?: string;
    coordinates?: { x: number; y: number; z?: number };
  };

  specifications: {
    manufacturer: string;
    model: string;
    range?: { min: number; max: number };
    accuracy?: number;          // %
    resolution?: number;
    samplingRate?: number;      // Hz
  };

  communication: {
    protocol: 'zigbee' | 'ble' | 'wifi' | 'wired';
    address?: string;
    gateway?: string;
  };

  state: {
    reachable: boolean;
    batteryLevel?: number;      // %
    lastUpdate: Date;

    // 센서별 측정값
    occupancy?: boolean;
    illuminance?: number;       // lux
    temperature?: number;       // °C
    humidity?: number;          // %
    co2?: number;               // ppm
  };

  configuration: {
    updateInterval: number;     // seconds
    threshold?: number;
    calibrationDate?: Date;
  };
}
```

#### 12.1.3 존/그룹 (Zone/Group)

```typescript
interface Zone {
  zoneId: string;
  name: string;
  description?: string;

  location: {
    building?: string;
    floor?: string;
    area?: number;              // m²
    capacity?: number;          // 최대 인원
  };

  fixtures: string[];           // 기구 ID 목록
  sensors: string[];            // 센서 ID 목록

  control: {
    mode: 'manual' | 'auto' | 'schedule' | 'sensor';
    schedule?: string;          // 스케줄 ID
    automationRules?: string[]; // 자동화 룰 ID 목록

    // 기본 설정
    defaults: {
      occupiedBrightness: number;     // %
      vacantBrightness: number;       // %
      targetIlluminance?: number;     // lux
      colorTemperature?: number;      // K
      fadeTime: number;               // ms
    };

    // 센서 기반 제어
    sensorControl?: {
      enabled: boolean;
      occupancyBased: boolean;
      daylightHarvesting: boolean;
      timeDelay: number;        // seconds (비점유 → 소등 지연)
    };
  };

  energyBudget?: {
    daily: number;              // kWh
    monthly: number;            // kWh
  };
}
```

### 12.2 이벤트 및 로그

#### 12.2.1 이벤트

```typescript
interface LightingEvent {
  eventId: string;
  timestamp: Date;
  type: 'state_change' | 'sensor_trigger' | 'schedule_execute' | 'automation_run' | 'alert' | 'maintenance';

  source: {
    type: 'user' | 'sensor' | 'schedule' | 'automation' | 'system';
    id?: string;
    name?: string;
  };

  target: {
    type: 'fixture' | 'zone' | 'scene';
    id: string;
    name?: string;
  };

  details: {
    action?: string;
    previousState?: any;
    newState?: any;
    reason?: string;
  };

  metadata?: {
    userId?: string;
    sessionId?: string;
    ipAddress?: string;
  };
}
```

#### 12.2.2 알림 (Alert)

```typescript
interface LightingAlert {
  alertId: string;
  timestamp: Date;
  severity: 'info' | 'warning' | 'error' | 'critical';
  type: 'fixture_offline' | 'sensor_offline' | 'energy_exceeded' | 'maintenance_due' | 'anomaly_detected';

  source: {
    type: 'fixture' | 'sensor' | 'zone' | 'system';
    id: string;
    name?: string;
  };

  message: string;
  description?: string;

  status: 'active' | 'acknowledged' | 'resolved' | 'closed';
  acknowledgedBy?: string;
  acknowledgedAt?: Date;
  resolvedAt?: Date;

  actions?: {
    label: string;
    action: string;             // API 호출 또는 명령
  }[];
}
```

---

## 13. API 명세

### 13.1 RESTful API 엔드포인트

#### 13.1.1 기구 관리

```
# 기구 목록 조회
GET /api/v1/fixtures
Query Parameters:
  - zone: string (존 ID)
  - floor: string
  - type: string (led, oled, etc.)
  - page: number
  - limit: number

Response: {
  fixtures: LightingFixture[],
  total: number,
  page: number,
  limit: number
}

# 기구 상세 조회
GET /api/v1/fixtures/{fixtureId}

Response: LightingFixture

# 기구 상태 업데이트
PUT /api/v1/fixtures/{fixtureId}/state
Body: {
  on?: boolean,
  brightness?: number,        // 0-100
  colorTemperature?: number,  // K
  color?: { hue: number, saturation: number },
  fadeTime?: number           // ms
}

Response: {
  success: boolean,
  state: FixtureState
}

# 기구 등록
POST /api/v1/fixtures
Body: LightingFixture

Response: {
  success: boolean,
  fixtureId: string
}

# 기구 삭제
DELETE /api/v1/fixtures/{fixtureId}
```

#### 13.1.2 씬 관리

```
# 씬 목록 조회
GET /api/v1/scenes

Response: {
  scenes: LightingScene[]
}

# 씬 조회
GET /api/v1/scenes/{sceneId}

Response: LightingScene

# 씬 활성화
POST /api/v1/scenes/{sceneId}/activate
Body: {
  fadeTime?: number           // ms (선택)
}

Response: {
  success: boolean,
  activatedAt: Date
}

# 씬 생성
POST /api/v1/scenes
Body: LightingScene

Response: {
  success: boolean,
  sceneId: string
}

# 씬 업데이트
PUT /api/v1/scenes/{sceneId}
Body: LightingScene

# 씬 삭제
DELETE /api/v1/scenes/{sceneId}
```

#### 13.1.3 스케줄 관리

```
# 스케줄 목록 조회
GET /api/v1/schedules

Response: {
  schedules: (TimeBasedSchedule | AstronomicalSchedule)[]
}

# 스케줄 조회
GET /api/v1/schedules/{scheduleId}

# 스케줄 생성
POST /api/v1/schedules
Body: TimeBasedSchedule | AstronomicalSchedule

# 스케줄 활성화/비활성화
PUT /api/v1/schedules/{scheduleId}/enabled
Body: {
  enabled: boolean
}

# 스케줄 삭제
DELETE /api/v1/schedules/{scheduleId}
```

#### 13.1.4 센서 데이터

```
# 센서 목록 조회
GET /api/v1/sensors

# 센서 상태 조회
GET /api/v1/sensors/{sensorId}

Response: Sensor

# 센서 측정값 조회 (시계열)
GET /api/v1/sensors/{sensorId}/measurements
Query Parameters:
  - start: ISO 8601 timestamp
  - end: ISO 8601 timestamp
  - interval: '1m' | '5m' | '15m' | '1h' | '1d'

Response: {
  measurements: {
    timestamp: Date,
    value: number
  }[]
}
```

#### 13.1.5 에너지 분석

```
# 에너지 대시보드
GET /api/v1/energy/dashboard
Query Parameters:
  - start: ISO 8601 timestamp
  - end: ISO 8601 timestamp
  - zone?: string

Response: EnergyDashboard

# 에너지 리포트
GET /api/v1/energy/report
Query Parameters:
  - type: 'daily' | 'weekly' | 'monthly'
  - date: YYYY-MM-DD

Response: {
  report: EnergyReport
}

# 에너지 예산 조회
GET /api/v1/energy/budget/{zoneId}

Response: PowerBudget
```

### 13.2 WebSocket API (실시간)

```typescript
// 연결
ws://api.example.com/ws

// 구독 메시지
{
  "type": "subscribe",
  "channels": [
    "fixtures:state",         // 모든 기구 상태 변화
    "fixtures:{fixtureId}",   // 특정 기구
    "sensors:{sensorId}",     // 특정 센서
    "zones:{zoneId}",         // 특정 존
    "alerts",                 // 알림
    "energy"                  // 에너지 데이터
  ]
}

// 상태 변화 푸시
{
  "type": "fixture_state_update",
  "fixtureId": "fixture-001",
  "state": {
    "on": true,
    "brightness": 80,
    "colorTemperature": 4000
  },
  "timestamp": "2025-12-25T10:30:00Z"
}

// 센서 데이터 푸시
{
  "type": "sensor_update",
  "sensorId": "sensor-pir-01",
  "state": {
    "occupancy": true,
    "lastMotionDetected": "2025-12-25T10:30:00Z"
  },
  "timestamp": "2025-12-25T10:30:00Z"
}

// 알림 푸시
{
  "type": "alert",
  "alert": {
    "alertId": "alert-123",
    "severity": "warning",
    "type": "fixture_offline",
    "message": "Fixture 'Office Light 1' is offline",
    "timestamp": "2025-12-25T10:30:00Z"
  }
}
```

---

## 14. 보안 및 개인정보보호

### 14.1 인증 및 권한

#### 14.1.1 OAuth 2.0

```
# 인증 플로우
1. 사용자 로그인 → 인증 서버
2. 인증 서버 → Access Token 발급
3. API 요청 시 Bearer Token 포함

Authorization: Bearer <access_token>
```

#### 14.1.2 역할 기반 접근 제어 (RBAC)

```typescript
enum Role {
  ADMIN = 'admin',              // 모든 권한
  MANAGER = 'manager',          // 설정, 스케줄 관리
  USER = 'user',                // 기본 제어
  VIEWER = 'viewer',            // 읽기 전용
}

interface Permission {
  resource: 'fixtures' | 'scenes' | 'schedules' | 'zones' | 'sensors' | 'analytics';
  action: 'read' | 'write' | 'delete' | 'control';
  scope?: string;               // 특정 존/그룹만
}

const rolePermissions: { [role: string]: Permission[] } = {
  admin: [
    { resource: 'fixtures', action: 'read' },
    { resource: 'fixtures', action: 'write' },
    { resource: 'fixtures', action: 'delete' },
    { resource: 'fixtures', action: 'control' },
    // ... 모든 리소스
  ],
  user: [
    { resource: 'fixtures', action: 'read' },
    { resource: 'fixtures', action: 'control' },
    { resource: 'scenes', action: 'read' },
    { resource: 'scenes', action: 'control' },
  ],
  viewer: [
    { resource: 'fixtures', action: 'read' },
    { resource: 'sensors', action: 'read' },
    { resource: 'analytics', action: 'read' },
  ]
};
```

### 14.2 데이터 보안

#### 14.2.1 암호화

- **전송 중 암호화:** TLS 1.3
- **저장 암호화:** AES-256 (민감 데이터)
- **비밀번호:** bcrypt (해싱)

#### 14.2.2 데이터 분류

| 데이터 유형 | 보안 등급 | 암호화 | 접근 제한 |
|------------|----------|--------|----------|
| 기구 상태 | Public | 불필요 | 전체 |
| 센서 데이터 | Public | 불필요 | 전체 |
| 사용자 정보 | Confidential | 필수 | 인증 필요 |
| 에너지 데이터 | Internal | 권장 | 역할 기반 |
| 설정 정보 | Internal | 권장 | MANAGER 이상 |

### 14.3 개인정보보호

#### 14.3.1 GDPR 준수

- 데이터 최소화: 필요한 데이터만 수집
- 명시적 동의: 사용자 동의 후 수집
- 삭제 권리: 사용자 요청 시 데이터 삭제
- 이동권: 데이터 내보내기 기능
- 익명화: 개인 식별 정보 제거

#### 14.3.2 카메라 센서 프라이버시

```typescript
interface CameraPrivacySettings {
  sensorId: string;

  privacyMode: {
    enabled: boolean;
    anonymization: 'blur_faces' | 'blur_all' | 'silhouette' | 'bounding_box';
    dataRetention: number;      // days (영상 보관 기간)
    accessLog: boolean;         // 접근 로그 기록
  };

  consent: {
    signageDisplayed: boolean;  // "CCTV 촬영 중" 안내판
    notificationSent: boolean;  // 사용자 알림
    optOut: boolean;            // 개인 옵트아웃 가능
  };
}
```

---

## 15. 성과 지표 (KPI)

### 15.1 에너지 효율 지표

| KPI | 측정 단위 | 목표값 | 산정 방식 |
|-----|----------|--------|----------|
| 에너지 절감률 | % | 30-50% | (베이스라인 - 현재) / 베이스라인 × 100 |
| 조명 효율 | lm/W | > 100 | 총 광속 / 총 전력 |
| 에너지 원단위 | kWh/m²/년 | < 10 | 연간 에너지 / 면적 |
| 피크 수요 감소 | % | 20-30% | (피크 베이스라인 - 현재 피크) / 피크 베이스라인 × 100 |
| 에너지 비용 절감 | 원/년 | - | (베이스라인 비용 - 현재 비용) |

### 15.2 제어 성능 지표

| KPI | 측정 단위 | 목표값 |
|-----|----------|--------|
| 응답 시간 | ms | < 500 |
| 시스템 가용성 | % | > 99.5% |
| 센서 정확도 | % | > 95% |
| 자동화 성공률 | % | > 98% |
| 기구 온라인율 | % | > 99% |

### 15.3 사용자 경험 지표

| KPI | 측정 단위 | 목표값 |
|-----|----------|--------|
| 사용자 만족도 | 1-10 | > 8 |
| 수동 조정 빈도 | 회/일 | < 3 |
| 불만 건수 | 건/월 | < 5 |
| 앱 사용률 | % | > 60% |

### 15.4 지속가능성 지표

| KPI | 측정 단위 | 목표값 |
|-----|----------|--------|
| CO2 감축량 | kg CO2/년 | - |
| 조명 수명 | 시간 | > 50,000 |
| 재활용률 | % | > 80% |
| 재생 에너지 사용 | % | > 30% |

---

## 부록

### A. 프로토콜 상세 비교표

| 기능 | DALI | DMX512 | Zigbee | BLE Mesh | WiFi |
|------|------|--------|--------|----------|------|
| 양방향 통신 | ✓ | ✗ | ✓ | ✓ | ✓ |
| 개별 주소 지정 | ✓ (64) | ✓ (512) | ✓ (65K) | ✓ (32K) | ✓ (무제한) |
| 그룹 제어 | ✓ (16) | ✗ | ✓ | ✓ | ✓ |
| 씬 저장 | ✓ (16) | ✗ | ✓ | ✓ | ✓ |
| 상태 피드백 | ✓ | ✗ | ✓ | ✓ | ✓ |
| 전력 측정 | ✓ (DALI-2) | ✗ | 선택 | 선택 | ✓ |
| 메시 네트워크 | ✗ | ✗ | ✓ | ✓ | ✓ |
| 설치 복잡도 | 중 | 중 | 낮음 | 낮음 | 낮음 |
| 비용 | 중 | 저 | 저 | 저 | 중 |

### B. 색온도 및 조도 권장표

#### B.1 공간별 권장 조도

| 공간 | 권장 조도 (lux) | 색온도 (K) |
|------|----------------|-----------|
| 복도 | 100-200 | 3000-4000 |
| 로비 | 200-300 | 3000-4000 |
| 사무실 (일반) | 500 | 4000-5000 |
| 사무실 (정밀) | 750-1000 | 5000-6500 |
| 회의실 | 300-500 | 3500-5000 |
| 교실 | 300-500 | 4000-5000 |
| 도서관 (열람실) | 500-750 | 4000-5000 |
| 병원 (병동) | 100-300 | 3000-4000 |
| 병원 (수술실) | 1000-10000 | 4000-5000 |
| 상점 | 300-500 | 3000-5000 |
| 레스토랑 | 100-300 | 2700-3000 |
| 주방 | 500 | 4000-5000 |
| 욕실 | 200-300 | 3000-4000 |
| 침실 | 50-150 | 2700-3000 |
| 거실 | 100-300 | 2700-4000 |

### C. API 예제 코드

```typescript
// TypeScript 예제
import { SmartLightingSDK } from '@wia/city-009';

const sdk = new SmartLightingSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-009/v1'
});

// 기구 제어
await sdk.fixtures.setState('fixture-001', {
  on: true,
  brightness: 80,
  colorTemperature: 4000,
  fadeTime: 2000
});

// 씬 활성화
await sdk.scenes.activate('relax', { fadeTime: 3000 });

// 센서 데이터 조회
const sensorData = await sdk.sensors.getMeasurements('sensor-001', {
  start: '2025-12-24T00:00:00Z',
  end: '2025-12-25T00:00:00Z',
  interval: '15m'
});

// 에너지 대시보드
const dashboard = await sdk.energy.getDashboard({
  start: '2025-12-01T00:00:00Z',
  end: '2025-12-25T23:59:59Z'
});
```

### D. 용어 사전

[150개 이상의 전문 용어 정의]

---

## 개정 이력

| 버전 | 날짜 | 주요 변경사항 |
|------|------|--------------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

## 라이선스

© 2025 WIA (World Certification Industry Association)
이 표준은 Creative Commons Attribution 4.0 International License 하에 배포됩니다.

---

## 연락처

**WIA 표준 사무국**
웹사이트: https://wiastandards.com
이메일: standards@wia-official.org
GitHub: https://github.com/WIA-Official/wia-standards

---

## 弘익人間 (홍익인간) · 널리 인간을 이롭게 하라

스마트 조명은 단순히 에너지를 절약하는 기술을 넘어, 인간의 건강과 웰빙을 증진하고, 환경을 보호하며, 모두가 더 나은 삶을 살 수 있도록 돕는 기술입니다. WIA-CITY-009 표준은 개방형 표준, 투명한 프로토콜, 협력적 개발을 통해 스마트 조명 기술이 인류 전체의 공동선에 기여하도록 보장합니다.

**함께, 우리는 더 밝고 건강한 미래를 만듭니다.**

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간) · Benefit All Humanity**
