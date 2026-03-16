# WIA-ENE-035: 환경 센서 네트워크 표준 기술 사양서

**버전**: 1.0.0
**발행일**: 2025년 12월 25일
**상태**: 정식 표준 (Ratified Standard)
**분류**: 환경/에너지 (Environment & Energy)

---

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

---

## 목차

1. [서론](#1-서론)
2. [범위](#2-범위)
3. [용어 정의](#3-용어-정의)
4. [센서 유형 및 사양](#4-센서-유형-및-사양)
5. [IoT 통신 프로토콜](#5-iot-통신-프로토콜)
6. [데이터 구조 및 포맷](#6-데이터-구조-및-포맷)
7. [센서 보정 및 품질 관리](#7-센서-보정-및-품질-관리)
8. [유지보수 및 운영](#8-유지보수-및-운영)
9. [알림 및 임계값 관리](#9-알림-및-임계값-관리)
10. [네트워크 아키텍처](#10-네트워크-아키텍처)
11. [엣지 컴퓨팅 및 데이터 처리](#11-엣지-컴퓨팅-및-데이터-처리)
12. [보안 및 프라이버시](#12-보안-및-프라이버시)
13. [성능 요구사항](#13-성능-요구사항)
14. [API 사양](#14-api-사양)
15. [준수 및 인증](#15-준수-및-인증)

---

## 1. 서론

### 1.1 목적

WIA-ENE-035 환경 센서 네트워크 표준은 다양한 환경 센서의 통합 관리, IoT 프로토콜 기반 데이터 수집, 실시간 모니터링, 데이터 품질 관리를 위한 국제 표준을 정의합니다.

### 1.2 배경

환경 모니터링은 대기질, 수질, 토양, 소음, 방사선 등 다양한 환경 요소를 측정하고 관리하는 중요한 분야입니다. 기존의 환경 모니터링 시스템은 센서 제조사별 독자 포맷, 비표준 통신 프로토콜, 데이터 품질 관리 부재 등의 문제를 안고 있었습니다.

본 표준은 이러한 문제를 해결하기 위해:
- 센서 유형별 표준 데이터 포맷 정의
- LoRaWAN, NB-IoT, Sigfox 등 표준 IoT 프로토콜 지원
- 센서 보정 및 품질 관리 절차 표준화
- 엣지 컴퓨팅 및 실시간 데이터 처리 지원
- 개방형 API 및 상호운용성 보장

### 1.3 적용 범위

본 표준은 다음 분야에 적용됩니다:

- **스마트시티**: 도시 환경 모니터링 및 관리
- **산업 안전**: 공장 및 산업단지 환경 모니터링
- **농업**: 스마트팜, 정밀 농업
- **환경 보호**: 국가 환경 모니터링 시스템
- **연구**: 환경 연구 및 데이터 분석

### 1.4 준수 표준

본 표준은 다음 국제 표준을 준수합니다:

- ISO/IEC 30141:2018 - IoT Reference Architecture
- LoRaWAN 1.1 Specification
- 3GPP NB-IoT Specification
- MQTT 5.0
- IEEE 802.15.4 (ZigBee)
- ISO 17025 - 시험 및 교정 기관의 적격성
- ISO/IEC 27001 - 정보 보안 관리

---

## 2. 범위

### 2.1 표준 적용 대상

#### 2.1.1 센서 유형

본 표준은 다음 환경 센서를 지원합니다:

1. **대기질 센서**
   - PM1.0, PM2.5, PM10 (미세먼지)
   - CO, CO₂, NO₂, SO₂, O₃ (가스)
   - VOC (휘발성 유기화합물)

2. **수질 센서**
   - pH, 탁도, 용존산소
   - 전기전도도, TDS
   - BOD, COD
   - ORP, 염도

3. **토양 센서**
   - 토양 수분, 온도
   - pH, EC
   - NPK (질소, 인, 칼륨)
   - 유기물 함량

4. **소음 센서**
   - 데시벨 (dB)
   - 주파수 분석
   - Leq, Lmax, Lmin

5. **방사선 센서**
   - 알파선, 베타선, 감마선, 중성자선
   - 선량률, 누적 선량

6. **기상 센서**
   - 온도, 습도, 기압
   - 강우량, 풍속, 풍향
   - 조도, 자외선 지수

#### 2.1.2 통신 프로토콜

- LPWAN: LoRaWAN, NB-IoT, Sigfox, LTE-M
- 단거리: ZigBee, Bluetooth LE, Wi-Fi
- 유선: Ethernet, Modbus, RS-485
- 애플리케이션: MQTT, CoAP, HTTP/HTTPS, WebSocket

### 2.2 표준 제외 대상

다음 항목은 본 표준의 범위에 포함되지 않습니다:

- 센서 하드웨어 제조 표준
- 센서 설치 물리적 위치 기준
- 국가별 환경 규제 기준
- 센서 데이터의 법적 효력

---

## 3. 용어 정의

### 3.1 일반 용어

- **센서 (Sensor)**: 환경 변수를 측정하여 전기 신호로 변환하는 장치
- **게이트웨이 (Gateway)**: 센서 데이터를 수집하여 서버로 전송하는 중계 장치
- **센서 네트워크 (Sensor Network)**: 다수의 센서와 게이트웨이로 구성된 통합 시스템
- **엣지 컴퓨팅 (Edge Computing)**: 게이트웨이에서 데이터를 전처리하는 기술

### 3.2 성능 지표

- **정확도 (Accuracy)**: 측정값과 실제값의 차이 (%)
- **정밀도 (Precision)**: 반복 측정값의 일관성 (%)
- **응답 시간 (Response Time)**: 측정 시작부터 결과 출력까지 시간 (초)
- **분해능 (Resolution)**: 측정 가능한 최소 단위
- **드리프트 (Drift)**: 시간 경과에 따른 측정값 변화

### 3.3 데이터 품질

- **우수 (Excellent)**: 오차 < 5%
- **양호 (Good)**: 오차 < 10%
- **보통 (Fair)**: 오차 < 20%
- **불량 (Poor)**: 오차 > 20%
- **유효하지 않음 (Invalid)**: 측정 실패 또는 범위 초과

### 3.4 IoT 프로토콜

- **LoRaWAN**: Long Range Wide Area Network, 장거리 저전력 통신
- **NB-IoT**: Narrowband IoT, 셀룰러 기반 IoT 통신
- **Sigfox**: 초장거리 저전력 통신 프로토콜
- **MQTT**: Message Queuing Telemetry Transport, 경량 메시징 프로토콜

---

## 4. 센서 유형 및 사양

### 4.1 대기질 센서

#### 4.1.1 미세먼지 센서 (PM)

**측정 항목**:
- PM1.0: 1.0 μm 이하 입자
- PM2.5: 2.5 μm 이하 입자
- PM10: 10 μm 이하 입자

**성능 요구사항**:
```
측정 범위: 0 - 1000 μg/m³
정확도: ±10% (0-100 μg/m³), ±15% (100-500 μg/m³)
분해능: 1 μg/m³
응답 시간: < 60초
```

**보정 주기**: 6개월

**데이터 포맷**:
```json
{
  "pm1_0": 15.2,
  "pm2_5": 35.5,
  "pm10": 55.0,
  "unit": "μg/m³",
  "timestamp": "2025-12-25T10:30:00Z"
}
```

#### 4.1.2 가스 센서

**CO (일산화탄소)**:
```
측정 범위: 0 - 50 ppm
정확도: ±5%
분해능: 0.1 ppm
응답 시간: < 30초
```

**CO₂ (이산화탄소)**:
```
측정 범위: 0 - 10,000 ppm
정확도: ±50 ppm (0-1000 ppm), ±5% (1000-10000 ppm)
분해능: 1 ppm
응답 시간: < 60초
```

**NO₂ (이산화질소)**:
```
측정 범위: 0 - 500 ppb
정확도: ±10%
분해능: 1 ppb
응답 시간: < 45초
```

### 4.2 수질 센서

#### 4.2.1 pH 센서

```
측정 범위: 0 - 14 pH
정확도: ±0.1 pH
분해능: 0.01 pH
응답 시간: < 30초
보정 주기: 1개월
보정 방법: 3점 보정 (pH 4.0, 7.0, 10.0)
```

#### 4.2.2 용존산소 센서 (DO)

```
측정 범위: 0 - 20 mg/L
정확도: ±0.2 mg/L
분해능: 0.01 mg/L
응답 시간: < 60초
보정 주기: 2개월
보정 방법: 대기 중 산소 포화도 보정
```

#### 4.2.3 탁도 센서

```
측정 범위: 0 - 1000 NTU
정확도: ±5% (0-100 NTU), ±10% (100-1000 NTU)
분해능: 0.1 NTU
응답 시간: < 10초
보정 주기: 3개월
보정 방법: 표준 탁도 용액
```

### 4.3 토양 센서

#### 4.3.1 토양 수분 센서

```
측정 범위: 0 - 100%
정확도: ±3%
분해능: 0.1%
응답 시간: < 5초
보정 주기: 6개월
보정 방법: 건조 토양 및 포화 토양 기준
```

#### 4.3.2 토양 온도 센서

```
측정 범위: -20 - 80°C
정확도: ±0.5°C
분해능: 0.1°C
응답 시간: < 10초
보정 주기: 12개월
```

### 4.4 소음 센서

```
측정 범위: 30 - 130 dB
정확도: ±1.5 dB
분해능: 0.1 dB
주파수 범위: 20 Hz - 20 kHz
응답 시간: < 1초
보정 주기: 6개월
보정 방법: 음향 교정기 (94 dB @ 1 kHz)
```

**측정 지표**:
- Leq: 등가 소음도
- Lmax: 최대 소음도
- Lmin: 최소 소음도
- L10, L50, L90: 백분위 소음도

### 4.5 방사선 센서

#### 4.5.1 감마선 센서

```
측정 범위: 0.01 - 10 μSv/h
정확도: ±15%
분해능: 0.01 μSv/h
응답 시간: < 60초
보정 주기: 12개월
보정 방법: 표준 선원 (Cs-137, Co-60)
```

---

## 5. IoT 통신 프로토콜

### 5.1 LoRaWAN

#### 5.1.1 프로토콜 버전
- LoRaWAN 1.1 Specification

#### 5.1.2 주파수 대역
- EU868: 863-870 MHz (유럽)
- US915: 902-928 MHz (미주)
- AS923: 915-928 MHz (아시아)
- KR920: 920-923 MHz (한국)

#### 5.1.3 확산 인자 (Spreading Factor)
```
SF7:  5,470 bps (최대 전송 속도, 최단 거리)
SF8:  3,125 bps
SF9:  1,760 bps
SF10:   980 bps
SF11:   440 bps
SF12:   250 bps (최저 전송 속도, 최장 거리)
```

#### 5.1.4 전송 출력
```
최소: 2 dBm
권장: 14 dBm
최대: 20 dBm (지역 규제 준수)
```

#### 5.1.5 Duty Cycle
- EU: 1% (36초/시간)
- US: No limit (FHSS)

#### 5.1.6 메시지 구조

```json
{
  "header": {
    "deviceEUI": "0000000000000001",
    "appEUI": "0000000000000001",
    "counter": 1234,
    "port": 1
  },
  "payload": {
    "sensorId": "SENSOR-001",
    "timestamp": "2025-12-25T10:30:00Z",
    "data": {
      "pm2_5": 35.5,
      "temperature": 22.5,
      "humidity": 60
    },
    "battery": 95,
    "rssi": -75
  }
}
```

### 5.2 NB-IoT

#### 5.2.1 프로토콜 버전
- 3GPP Release 13 이상

#### 5.2.2 주파수 대역
```
한국: Band 5 (850 MHz), Band 8 (900 MHz)
전 세계: Band 1, 3, 5, 8, 20, 28
```

#### 5.2.3 데이터 전송률
```
Downlink: 최대 250 kbps
Uplink: 최대 250 kbps (Single-tone)
        최대 1 Mbps (Multi-tone)
```

#### 5.2.4 전력 소비
```
PSM (Power Saving Mode): < 5 μA
eDRX (Extended Discontinuous Reception): < 100 μA
Idle: < 1 mA
Active: 100-300 mA
```

### 5.3 Sigfox

#### 5.3.1 주파수 대역
```
EU: 868 MHz
US: 902 MHz
Asia: 923 MHz
```

#### 5.3.2 데이터 전송률
```
Uplink: 100 bps
Downlink: 600 bps
```

#### 5.3.3 메시지 제한
```
Uplink: 140 messages/day, 12 bytes/message
Downlink: 4 messages/day, 8 bytes/message
```

### 5.4 MQTT

#### 5.4.1 프로토콜 버전
- MQTT 5.0

#### 5.4.2 QoS (Quality of Service)
```
QoS 0: At most once (최대 1회 전달, 손실 가능)
QoS 1: At least once (최소 1회 전달, 중복 가능)
QoS 2: Exactly once (정확히 1회 전달, 권장)
```

#### 5.4.3 토픽 구조
```
wia/ene-035/{networkId}/{sensorId}/data
wia/ene-035/{networkId}/{sensorId}/status
wia/ene-035/{networkId}/{sensorId}/alert
wia/ene-035/{networkId}/gateway/{gatewayId}/status
```

#### 5.4.4 Payload 포맷
- JSON (권장)
- Protocol Buffers
- MessagePack

---

## 6. 데이터 구조 및 포맷

### 6.1 센서 등록 데이터

```json
{
  "sensorId": "SENSOR-AIR-001",
  "sensorType": "AIR_QUALITY",
  "location": {
    "sensorId": "SENSOR-AIR-001",
    "siteName": "서울시청",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude": 50
    },
    "zone": "Seoul-Jung-gu",
    "description": "서울시청 옥상"
  },
  "specs": {
    "manufacturer": "WIA Sensors",
    "model": "WIA-AQ-2025",
    "serialNumber": "SN-001234",
    "firmwareVersion": "1.0.0",
    "measuringRange": {
      "min": 0,
      "max": 1000,
      "unit": "μg/m³"
    },
    "accuracy": 5,
    "resolution": 1,
    "responseTime": 60,
    "operatingTemperature": {
      "min": -20,
      "max": 50
    },
    "ipRating": "IP67",
    "certifications": ["CE", "FCC", "KCC"]
  },
  "power": {
    "powerSource": "solar",
    "batteryType": "Li-ion",
    "batteryCapacity": 10000,
    "batteryLevel": 100,
    "solarPanelWattage": 5,
    "powerConsumption": 50,
    "expectedBatteryLife": 200
  },
  "communication": {
    "protocol": "LORAWAN",
    "frequency": 920,
    "bandwidth": 125,
    "transmitPower": 14,
    "dataRate": 5000,
    "deviceEUI": "0000000000000001",
    "appKey": "00000000000000000000000000000001"
  },
  "sampling": {
    "samplingInterval": 300,
    "transmissionInterval": 900,
    "averagingPeriod": 60,
    "samplesPerTransmission": 3,
    "enabledParameters": ["pm2_5", "pm10", "temperature", "humidity"]
  },
  "status": "ONLINE",
  "installationDate": "2025-12-01T09:00:00Z",
  "lastMaintenanceDate": "2025-12-15T10:00:00Z",
  "nextMaintenanceDate": "2026-06-15T10:00:00Z"
}
```

### 6.2 센서 데이터 (Reading)

```json
{
  "readingId": "READ-2025-001234",
  "sensorId": "SENSOR-AIR-001",
  "sensorType": "AIR_QUALITY",
  "timestamp": "2025-12-25T10:30:00Z",
  "airQuality": {
    "pm1_0": 15.2,
    "pm2_5": 35.5,
    "pm10": 55.0,
    "co": 0.8,
    "co2": 450,
    "no2": 25,
    "so2": 10,
    "o3": 45,
    "voc": 120,
    "aqi": 100
  },
  "dataQuality": "GOOD",
  "batteryLevel": 95,
  "signalStrength": -75,
  "flags": [],
  "notes": ""
}
```

### 6.3 보정 데이터 (Calibration)

```json
{
  "calibrationId": "CAL-2025-001",
  "sensorId": "SENSOR-AIR-001",
  "calibrationType": "FIELD",
  "calibrationDate": "2025-12-25T09:00:00Z",
  "performedBy": "tech-001",
  "calibrationPoints": [
    {
      "referenceValue": 0,
      "measuredValue": 0.5,
      "unit": "μg/m³",
      "deviation": 0.5
    },
    {
      "referenceValue": 50,
      "measuredValue": 51.5,
      "unit": "μg/m³",
      "deviation": 3
    },
    {
      "referenceValue": 100,
      "measuredValue": 102,
      "unit": "μg/m³",
      "deviation": 2
    }
  ],
  "preCalibrationDrift": 5,
  "postCalibrationAccuracy": 98,
  "referenceEquipment": {
    "manufacturer": "Thermo Scientific",
    "model": "TEOM 1405",
    "serialNumber": "REF-001",
    "certificationDate": "2025-01-01T00:00:00Z"
  },
  "passed": true,
  "nextCalibrationDue": "2026-06-25T00:00:00Z",
  "notes": "현장 보정 완료. 정상 작동 확인."
}
```

---

## 7. 센서 보정 및 품질 관리

### 7.1 보정 유형

#### 7.1.1 공장 보정 (Factory Calibration)
- 센서 제조 시 수행
- 표준 환경 조건에서 수행
- 유효 기간: 12개월

#### 7.1.2 현장 보정 (Field Calibration)
- 설치 현장에서 수행
- 실제 운영 환경에서 수행
- 유효 기간: 6개월

#### 7.1.3 자동 보정 (Automatic Calibration)
- 센서 자체적으로 수행
- 기준 센서 또는 알고리즘 기반
- 주기: 매일 또는 매주

#### 7.1.4 수동 보정 (Manual Calibration)
- 전문가가 직접 수행
- 정밀 기준 장비 사용
- 주기: 필요 시

### 7.2 보정 절차

#### 7.2.1 사전 준비
1. 보정 장비 준비 및 점검
2. 환경 조건 확인 (온도, 습도, 기압)
3. 센서 안정화 시간 확보 (최소 30분)

#### 7.2.2 보정 수행
1. 영점 보정 (Zero calibration)
   - 측정 범위 최소값에서 수행
   - 드리프트 확인 및 보정

2. 스팬 보정 (Span calibration)
   - 측정 범위 중간값 및 최대값에서 수행
   - 기울기 보정

3. 다점 보정 (Multi-point calibration)
   - 3점 이상에서 수행 (권장)
   - 선형성 확인

#### 7.2.3 보정 검증
1. 보정 후 측정값 확인
2. 정확도 및 정밀도 검증
3. 보정 성공 여부 판정

#### 7.2.4 보정 기록
1. 보정 데이터 저장
2. 보정 증명서 발급
3. 다음 보정 일정 계획

### 7.3 데이터 품질 관리

#### 7.3.1 실시간 품질 검사

**범위 검사 (Range Check)**:
```
if (value < min || value > max) {
  quality = "INVALID";
  flag.add("OUT_OF_RANGE");
}
```

**변화율 검사 (Rate of Change Check)**:
```
changeRate = (currentValue - previousValue) / timeInterval;
if (abs(changeRate) > maxChangeRate) {
  quality = "POOR";
  flag.add("EXCESSIVE_CHANGE_RATE");
}
```

**고정값 검사 (Stuck Value Check)**:
```
if (value == previousValue for N consecutive readings) {
  quality = "POOR";
  flag.add("STUCK_VALUE");
}
```

**스파이크 검사 (Spike Check)**:
```
if (abs(value - movingAverage) > 3 * stdDev) {
  quality = "POOR";
  flag.add("SPIKE_DETECTED");
}
```

#### 7.3.2 품질 등급

```
EXCELLENT: 오차 < 5%, 완전성 > 95%
GOOD: 오차 < 10%, 완전성 > 90%
FAIR: 오차 < 20%, 완전성 > 80%
POOR: 오차 > 20%, 완전성 < 80%
INVALID: 측정 불가능
```

---

## 8. 유지보수 및 운영

### 8.1 유지보수 유형

#### 8.1.1 예방 유지보수 (Preventive Maintenance)

**점검 (Inspection)**:
- 주기: 월 1회
- 내용: 센서 상태 육안 점검, 연결 상태 확인

**청소 (Cleaning)**:
- 주기: 월 1회 (환경에 따라 조정)
- 내용: 센서 표면 청소, 필터 교체

**보정 (Calibration)**:
- 주기: 6개월
- 내용: 센서 보정, 정확도 검증

#### 8.1.2 교정 유지보수 (Corrective Maintenance)

**수리 (Repair)**:
- 발생 시: 센서 고장 또는 성능 저하
- 내용: 부품 교체, 회로 수리

**펌웨어 업데이트 (Firmware Update)**:
- 주기: 필요 시
- 내용: 보안 패치, 기능 개선

**배터리 교체 (Battery Replacement)**:
- 주기: 배터리 수명 또는 성능 저하 시
- 내용: 배터리 교체, 전원 시스템 점검

### 8.2 유지보수 기록

```json
{
  "maintenanceId": "MAINT-2025-001",
  "sensorId": "SENSOR-AIR-001",
  "maintenanceType": "BATTERY_REPLACEMENT",
  "scheduledDate": "2025-12-30T09:00:00Z",
  "performedDate": "2025-12-30T10:30:00Z",
  "performedBy": "tech-001",
  "description": "배터리 교체 및 센서 청소",
  "partsReplaced": ["battery", "filter"],
  "costsIncurred": 50000,
  "status": "completed",
  "preMaintenanceStatus": "LOW_BATTERY",
  "postMaintenanceStatus": "ONLINE",
  "issues": [],
  "recommendations": ["다음 점검 시 필터 추가 교체 권장"],
  "notes": "정상 작동 확인"
}
```

### 8.3 배터리 관리

#### 8.3.1 배터리 수명 계산

```
Battery Life (days) = Battery Capacity (mAh) / Current Draw (mA) / 24
```

#### 8.3.2 전력 소비 최적화

**샘플링 간격 조정**:
```
일일 전송 횟수 = 86400 / transmission_interval
일일 전력 소비 = (active_current × active_time + sleep_current × sleep_time) × 전송 횟수
```

**동적 전송 간격**:
- 정상 상태: 15분
- 이상 감지: 5분
- 긴급 상태: 1분

#### 8.3.3 태양광 패널 설계

```
Required Solar Panel = Daily Power Consumption / (Sunlight Hours × Solar Efficiency × 0.7)
```

---

## 9. 알림 및 임계값 관리

### 9.1 알림 등급

#### 9.1.1 정보 (INFO)
- 센서 온라인/오프라인
- 정기 보고

#### 9.1.2 경고 (WARNING)
- 임계값 1차 초과
- 배터리 부족 (< 20%)
- 데이터 품질 저하

#### 9.1.3 위험 (CRITICAL)
- 임계값 2차 초과
- 센서 고장
- 배터리 심각 부족 (< 10%)

#### 9.1.4 긴급 (EMERGENCY)
- 임계값 3차 초과
- 즉각적인 대응 필요
- 공중 보건 위협

### 9.2 임계값 설정

#### 9.2.1 대기질 (PM2.5)

```json
{
  "parameter": "pm2_5",
  "unit": "μg/m³",
  "warningMax": 50,      // 경고: PM2.5 > 50
  "criticalMax": 100,    // 위험: PM2.5 > 100
  "emergencyMax": 200    // 긴급: PM2.5 > 200
}
```

#### 9.2.2 수질 (pH)

```json
{
  "parameter": "ph",
  "unit": "pH",
  "warningMin": 6.0,
  "warningMax": 9.0,
  "criticalMin": 5.5,
  "criticalMax": 9.5,
  "emergencyMin": 5.0,
  "emergencyMax": 10.0
}
```

#### 9.2.3 소음

```json
{
  "parameter": "noise",
  "unit": "dB",
  "warningMax": 65,      // 일반 지역
  "criticalMax": 70,
  "emergencyMax": 80
}
```

### 9.3 알림 전송

#### 9.3.1 전송 채널
- 이메일
- SMS
- 모바일 푸시 알림
- Webhook

#### 9.3.2 알림 포맷

```json
{
  "eventId": "ALERT-2025-001",
  "sensorId": "SENSOR-AIR-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "severity": "CRITICAL",
  "parameter": "pm2_5",
  "value": 105.5,
  "threshold": 100,
  "message": "PM2.5 농도가 위험 수준 (105.5 μg/m³)을 초과했습니다.",
  "acknowledged": false,
  "resolved": false
}
```

#### 9.3.3 알림 중복 방지

**쿨다운 기간 (Cooldown Period)**:
```
동일 센서, 동일 파라미터에 대해 30분 이내 중복 알림 방지
```

---

## 10. 네트워크 아키텍처

### 10.1 네트워크 토폴로지

#### 10.1.1 Star Topology (LoRaWAN)
```
Sensors → Gateway → Network Server → Application Server
```

#### 10.1.2 Mesh Topology (ZigBee)
```
Sensors ⇄ Sensors ⇄ Gateway → Server
```

#### 10.1.3 Cellular (NB-IoT)
```
Sensors → Base Station → Core Network → Server
```

### 10.2 게이트웨이 사양

```json
{
  "gatewayId": "GW-SEOUL-001",
  "location": {
    "siteName": "서울시청",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    }
  },
  "protocol": "LORAWAN",
  "ipAddress": "192.168.1.100",
  "macAddress": "00:11:22:33:44:55",
  "connectionType": "ethernet",
  "uplink": {
    "bandwidth": 100,
    "latency": 10
  },
  "status": "online",
  "connectedSensors": 150,
  "maxSensors": 1000,
  "edgeComputing": {
    "enabled": true,
    "processing": ["filtering", "aggregation", "anomaly-detection"],
    "cpuUsage": 25,
    "memoryUsage": 40,
    "storageUsage": 30
  },
  "lastSeen": "2025-12-25T10:30:00Z"
}
```

### 10.3 네트워크 성능 지표

```json
{
  "networkHealth": {
    "uptime": 99.9,
    "avgSignalStrength": -70,
    "packetLossRate": 0.5,
    "avgLatency": 150
  }
}
```

---

## 11. 엣지 컴퓨팅 및 데이터 처리

### 11.1 엣지 처리 기능

#### 11.1.1 데이터 필터링 (Filtering)
- 이상치 제거
- 범위 초과 데이터 필터링

#### 11.1.2 데이터 집계 (Aggregation)
- 평균, 최소, 최대 계산
- 시간대별 집계

#### 11.1.3 이상 탐지 (Anomaly Detection)
- 통계 기반 이상 탐지
- 머신러닝 기반 이상 탐지

#### 11.1.4 ML 추론 (ML Inference)
- 센서 상태 예측
- 환경 지수 예측

### 11.2 엣지 처리 규칙

```json
{
  "ruleId": "RULE-001",
  "ruleName": "PM2.5 이상치 필터링",
  "enabled": true,
  "sensorTypes": ["AIR_QUALITY"],
  "conditions": [
    {
      "parameter": "pm2_5",
      "operator": ">",
      "value": 500
    }
  ],
  "actions": [
    {
      "type": "filter",
      "config": { "action": "reject" }
    },
    {
      "type": "alert",
      "config": {
        "severity": "warning",
        "message": "PM2.5 이상치 감지"
      }
    }
  ],
  "priority": 1,
  "executionOrder": 1
}
```

---

## 12. 보안 및 프라이버시

### 12.1 전송 보안

#### 12.1.1 암호화
- TLS 1.3 (HTTPS, MQTTS, WSS)
- AES-256 (LoRaWAN 페이로드)

#### 12.1.2 인증
- API Key
- OAuth 2.0
- JWT (JSON Web Token)

#### 12.1.3 무결성
- HMAC-SHA256

### 12.2 데이터 보안

#### 12.2.1 저장 암호화
- Database: AES-256
- Backup: AES-256

#### 12.2.2 접근 제어
- RBAC (Role-Based Access Control)
- 최소 권한 원칙

### 12.3 프라이버시

#### 12.3.1 위치 정보
- 정밀 좌표 익명화 (공개 데이터)
- 구역 단위 공개 (시/구/동)

#### 12.3.2 개인정보 보호
- GDPR 준수
- 데이터 보관 기간 제한 (기본 3년)

---

## 13. 성능 요구사항

### 13.1 네트워크 성능

```
네트워크 가동률: ≥ 99.5%
센서 가용성: ≥ 95%
데이터 완전성: ≥ 90%
데이터 정확도: ≥ 95%
평균 응답 시간: ≤ 5초
패킷 손실률: ≤ 1%
```

### 13.2 센서 성능

```
샘플링 간격: 1분 - 1시간 (조정 가능)
전송 간격: 5분 - 24시간 (조정 가능)
배터리 수명: ≥ 1년 (solar: unlimited)
신호 강도: RSSI ≥ -120 dBm
```

### 13.3 API 성능

```
최대 동시 요청: 10,000 req/s
평균 응답 시간: ≤ 200ms (GET), ≤ 500ms (POST)
처리량: ≥ 1,000,000 readings/hour
가용성: 99.9%
```

---

## 14. API 사양

### 14.1 인증

```http
Authorization: Bearer {API_KEY}
X-WIA-Standard: ENE-035
X-WIA-Version: 1.0.0
```

### 14.2 엔드포인트

#### 14.2.1 센서 관리

**센서 등록**:
```http
POST /api/v1/sensor/register
Content-Type: application/json

{
  "sensorId": "SENSOR-001",
  "sensorType": "AIR_QUALITY",
  ...
}
```

**센서 조회**:
```http
GET /api/v1/sensor/{sensorId}
```

**센서 목록**:
```http
GET /api/v1/sensors?page=1&limit=10&type=AIR_QUALITY&status=ONLINE
```

#### 14.2.2 데이터 수집

**데이터 제출**:
```http
POST /api/v1/reading/submit
Content-Type: application/json

{
  "sensorId": "SENSOR-001",
  "airQuality": {
    "pm2_5": 35.5,
    "pm10": 55.0
  },
  "dataQuality": "GOOD",
  "batteryLevel": 95
}
```

**최신 데이터 조회**:
```http
GET /api/v1/sensor/{sensorId}/latest
```

**집계 데이터 조회**:
```http
GET /api/v1/data/aggregate?sensorId=SENSOR-001&parameter=pm2_5&interval=hour&startDate=2025-12-25T00:00:00Z&endDate=2025-12-25T23:59:59Z
```

#### 14.2.3 보정 및 유지보수

**보정 기록 제출**:
```http
POST /api/v1/calibration/submit
```

**유지보수 일정 등록**:
```http
POST /api/v1/maintenance/schedule
```

#### 14.2.4 알림 관리

**알림 설정**:
```http
POST /api/v1/alert/configure
```

**활성 알림 조회**:
```http
GET /api/v1/alerts/active?sensorId=SENSOR-001&severity=CRITICAL
```

### 14.3 오류 코드

```
200 OK: 성공
201 Created: 생성 성공
400 Bad Request: 잘못된 요청
401 Unauthorized: 인증 실패
403 Forbidden: 권한 없음
404 Not Found: 리소스 없음
429 Too Many Requests: 요청 제한 초과
500 Internal Server Error: 서버 오류
503 Service Unavailable: 서비스 불가
```

---

## 15. 준수 및 인증

### 15.1 인증 프로그램

#### 15.1.1 WIA-ENE-035 인증

**Level 1: Basic Compliance**
- 센서 데이터 포맷 준수
- 기본 API 지원
- 데이터 품질 Level 2 (GOOD)

**Level 2: Standard Compliance**
- 모든 API 지원
- IoT 프로토콜 지원 (LoRaWAN/NB-IoT/MQTT)
- 데이터 품질 Level 1 (EXCELLENT)
- 보정 및 유지보수 관리

**Level 3: Advanced Compliance**
- 엣지 컴퓨팅 지원
- 이상 탐지 및 ML 기능
- 고가용성 (99.9%)
- 보안 강화 (TLS 1.3, AES-256)

### 15.2 인증 절차

1. **신청**: 인증 신청서 제출
2. **문서 검토**: 기술 문서 및 구현 사양 검토
3. **기술 시험**: 실제 센서 및 시스템 테스트
4. **보안 감사**: 보안 및 프라이버시 검증
5. **인증 발급**: 인증서 발급 (유효 기간 2년)

### 15.3 준수 체크리스트

- [ ] 센서 데이터 포맷 준수
- [ ] IoT 프로토콜 구현
- [ ] API 엔드포인트 구현
- [ ] 보정 절차 수립
- [ ] 데이터 품질 관리
- [ ] 보안 및 암호화
- [ ] 성능 요구사항 충족
- [ ] 문서화 완료

---

## 부록 A: 예제 코드

### A.1 TypeScript SDK

```typescript
import { EnvironmentalSensorSDK } from '@wia/ene-035';

const sdk = new EnvironmentalSensorSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-035/v1'
});

// 센서 등록
const sensor = await sdk.registerSensor({
  sensorId: 'SENSOR-001',
  sensorType: SensorType.AIR_QUALITY,
  ...
});

// 데이터 제출
const reading = await sdk.submitReading({
  sensorId: 'SENSOR-001',
  airQuality: { pm2_5: 35.5 }
});
```

### A.2 Python SDK

```python
from wia_ene035 import EnvironmentalSensorClient

client = EnvironmentalSensorClient(
    api_key='your-api-key'
)

# 센서 등록
sensor = client.register_sensor(
    sensor_id='SENSOR-001',
    sensor_type='AIR_QUALITY'
)

# 데이터 제출
reading = client.submit_reading(
    sensor_id='SENSOR-001',
    pm25=35.5
)
```

---

## 부록 B: 용어집

- **AQI**: Air Quality Index (대기질 지수)
- **LoRaWAN**: Long Range Wide Area Network
- **NB-IoT**: Narrowband Internet of Things
- **MQTT**: Message Queuing Telemetry Transport
- **PM2.5**: Particulate Matter 2.5μm (미세먼지)
- **TDS**: Total Dissolved Solids (총 용존 고형물)
- **BOD**: Biochemical Oxygen Demand (생화학적 산소 요구량)
- **COD**: Chemical Oxygen Demand (화학적 산소 요구량)

---

## 부록 C: 참고 문헌

1. LoRa Alliance. (2017). LoRaWAN 1.1 Specification.
2. 3GPP. (2016). NB-IoT Technical Specification.
3. OASIS. (2019). MQTT Version 5.0 Specification.
4. ISO/IEC 30141:2018 - IoT Reference Architecture
5. US EPA. (2024). Air Quality Index Technical Guide.
6. WHO. (2021). Air Quality Guidelines.

---

## 변경 이력

### Version 1.0.0 (2025-12-25)
- 초판 발행
- 센서 유형 및 사양 정의
- IoT 프로토콜 표준화
- API 사양 정의
- 보안 및 성능 요구사항 정의

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
