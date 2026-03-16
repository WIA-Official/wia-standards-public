# WIA-ENE-032: 산불 감지 표준 v1.0 🔥

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-032
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [산불 위험도 분류](#4-산불-위험도-분류)
5. [데이터 모델](#5-데이터-모델)
6. [위성 영상 감지](#6-위성-영상-감지)
7. [열 감지 시스템](#7-열-감지-시스템)
8. [연기 감지 시스템](#8-연기-감지-시스템)
9. [기상 조건 모니터링](#9-기상-조건-모니터링)
10. [연료 수분 측정](#10-연료-수분-측정)
11. [화재 확산 예측](#11-화재-확산-예측)
12. [대피 구역 관리](#12-대피-구역-관리)
13. [소방 자원 관리](#13-소방-자원-관리)
14. [경보 시스템](#14-경보-시스템)
15. [통합 및 상호운용성](#15-통합-및-상호운용성)
16. [보안 및 개인정보보호](#16-보안-및-개인정보보호)
17. [인증 요구사항](#17-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-032 산불 감지 표준은 산림 화재의 조기 감지, 실시간 모니터링, 신속한 대응을 위한 통합 시스템 표준입니다. 본 표준은 위성 관측, 지상 센서, AI 분석을 결합하여 산불로 인한 인명 및 재산 피해를 최소화하고 산림 생태계를 보호하는 것을 목표로 합니다.

### 1.2 핵심 원칙

- **조기 감지 (Early Detection)**: 산불 발생 즉시 감지 (목표: 10분 이내)
- **정확성 (Accuracy)**: 오탐지율 5% 미만, 미탐지율 1% 미만
- **실시간성 (Real-time)**: 데이터 수집-분석-경보 지연시간 3분 이내
- **다중 센서 융합 (Multi-sensor Fusion)**: 위성, 지상, 드론 데이터 통합
- **AI 예측 (AI Prediction)**: 화재 확산 경로 및 위험도 자동 예측
- **공개 표준 (Open Standard)**: 국제적 상호운용성 보장
- **인명 우선 (Life First)**: 대피 및 구조 최우선

### 1.3 적용 대상

- 산림청 및 지방자치단체 산불 방지 부서
- 국가산불종합상황실 및 지역 상황실
- 위성 관측 기관 (MODIS, VIIRS, Sentinel)
- 기상청 및 기상 예보 기관
- 소방서 및 긴급구조대
- 드론 및 항공 소방 운영자
- 스마트시티 재난 관리 시스템
- 산림 보호 NGO 및 연구 기관

---

## 2. 적용 범위

### 2.1 감지 대상

본 표준은 다음 산불 유형에 적용됩니다:

- **지표화 (Surface Fire)**: 낙엽, 관목 등 지표면 연소
- **수관화 (Crown Fire)**: 나무 수관부 연소 (가장 위험)
- **지중화 (Ground Fire)**: 토탄층, 부식토 연소 (장기 지속)
- **잔불 (Smoldering Fire)**: 불씨 상태로 장시간 연소
- **비화 (Spot Fire)**: 바람에 날린 불씨로 인한 2차 발화

### 2.2 감지 단계

- **예방 단계**: 산불 위험도 예측 및 경보
- **발화 감지**: 최초 화재 지점 식별 (골든타임: 30분)
- **확산 모니터링**: 실시간 화재 진행 추적
- **진화 지원**: 소방 자원 배치 최적화
- **사후 모니터링**: 잔불 감시 및 재발화 방지

### 2.3 지리적 범위

- **산림 지역**: 국립공원, 도립공원, 사유림
- **도시 경계 (WUI)**: 산림과 인접한 주거지역
- **농촌 지역**: 논밭과 인접한 산림
- **자연보호구역**: 생태계 보전 지역

---

## 3. 용어 정의

### 3.1 기본 용어

- **산불 (Wildfire)**: 산림에서 발생하는 통제되지 않은 화재
- **화점 (Hot Spot)**: 위성 또는 센서가 감지한 고온 지점
- **화선 (Fire Line)**: 불이 확산되는 최전선
- **완전 진화 (Containment)**: 100% 화선 통제 완료
- **통제선 (Control Line)**: 화재 확산 방지를 위한 방화선
- **골든타임 (Golden Time)**: 초기 진화 가능한 시간 (통상 30분)

### 3.2 기술 용어

- **MODIS (Moderate Resolution Imaging Spectroradiometer)**: NASA의 중해상도 위성 센서 (Terra, Aqua 위성 탑재)
- **VIIRS (Visible Infrared Imaging Radiometer Suite)**: 가시광선-적외선 영상 관측 장비 (Suomi NPP, NOAA-20 위성)
- **Sentinel**: 유럽우주국(ESA)의 지구관측 위성 시리즈
- **FRP (Fire Radiative Power)**: 화재 복사 에너지 (MW 단위)
- **FWI (Fire Weather Index)**: 캐나다 산불 기상 지수
- **NFDRS (National Fire Danger Rating System)**: 미국 국가 화재위험도 평가 시스템
- **EFFIS (European Forest Fire Information System)**: 유럽 산불 정보 시스템
- **DFMC (Dead Fuel Moisture Content)**: 고사 연료 수분 함량
- **LFMC (Live Fuel Moisture Content)**: 생체 연료 수분 함량

### 3.3 센서 용어

- **열화상 카메라 (Thermal Camera)**: 적외선으로 온도 분포 촬영
- **연기 감지기 (Smoke Detector)**: 입자 또는 이온화 방식 연기 센서
- **기상 센서 (Weather Sensor)**: 온도, 습도, 풍속, 풍향 측정
- **수분 센서 (Moisture Sensor)**: 토양 및 연료 수분 측정

---

## 4. 산불 위험도 분류

### 4.1 위험도 등급

| 등급 | 명칭 | 색상 | FWI 범위 | 발화 확률 | 대응 조치 |
|------|------|------|----------|-----------|-----------|
| 1 | 낮음 (Low) | 🟢 녹색 | 0-5 | <5% | 정상 감시 |
| 2 | 보통 (Moderate) | 🟡 황색 | 6-12 | 5-15% | 주의 관찰 |
| 3 | 높음 (High) | 🟠 주황 | 13-22 | 15-35% | 경계 태세 |
| 4 | 매우 높음 (Very High) | 🔴 적색 | 23-38 | 35-60% | 통제 조치 (입산 통제) |
| 5 | 극도 (Extreme) | 🟣 보라 | 39-50+ | >60% | 비상 태세 (산림 폐쇄) |

### 4.2 위험도 계산 요소

```typescript
interface FireDangerCalculation {
  // 기상 조건 (가중치 40%)
  temperature: number;        // 온도 (°C)
  humidity: number;           // 상대습도 (%)
  windSpeed: number;          // 풍속 (m/s)
  precipitation: number;      // 강수량 (mm, 최근 7일)

  // 연료 상태 (가중치 30%)
  deadFuelMoisture: number;   // 고사 연료 수분 (%)
  liveFuelMoisture: number;   // 생체 연료 수분 (%)
  fuelLoad: number;           // 연료량 (ton/ha)

  // 지형 조건 (가중치 20%)
  slope: number;              // 경사도 (°)
  aspect: number;             // 사면 방향 (°)
  elevation: number;          // 고도 (m)

  // 인간 활동 (가중치 10%)
  humanActivity: number;      // 활동 빈도 (0-1)
  proximity: number;          // 인구 밀집지 거리 (km)
}
```

---

## 5. 데이터 모델

### 5.1 산불 감지 이벤트

```json
{
  "eventId": "FIRE-2025-KR-001234",
  "timestamp": "2025-04-15T14:23:00Z",
  "detectionMethod": "VIIRS",
  "confidenceLevel": 95,

  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "elevation": 450,
    "address": "강원도 속초시 설악산",
    "forestType": "침엽수림",
    "administrativeArea": "강원도"
  },

  "fireCharacteristics": {
    "frp": 125.5,
    "brightness": 345.2,
    "area": 1500,
    "perimeter": 450,
    "fireLineIntensity": 3500,
    "rateOfSpread": 2.5
  },

  "weatherConditions": {
    "temperature": 28.5,
    "humidity": 25,
    "windSpeed": 12.5,
    "windDirection": 225,
    "precipitation24h": 0
  },

  "riskAssessment": {
    "dangerLevel": 4,
    "fwi": 35.2,
    "threatToLife": "high",
    "threatToProperty": "high",
    "evacuationRequired": true
  },

  "metadata": {
    "satellite": "NOAA-20",
    "sensor": "VIIRS-I4",
    "resolution": 375,
    "quality": "high",
    "validated": true
  }
}
```

### 5.2 화재 확산 예측

```json
{
  "predictionId": "PRED-2025-001234",
  "fireEventId": "FIRE-2025-KR-001234",
  "timestamp": "2025-04-15T14:30:00Z",
  "forecastHorizon": 24,

  "spreadPrediction": {
    "currentArea": 1500,
    "predicted6h": 5000,
    "predicted12h": 12000,
    "predicted24h": 30000,
    "confidence": 85
  },

  "firePerimeter": {
    "type": "Polygon",
    "coordinates": [
      [[126.978, 37.566], [126.982, 37.568], [126.980, 37.564], [126.978, 37.566]]
    ]
  },

  "threatenedAssets": [
    {
      "type": "residential",
      "name": "속초시 외곽 주택가",
      "population": 1500,
      "distance": 2.5,
      "arrivalTime": "2025-04-15T18:00:00Z",
      "priority": "critical"
    },
    {
      "type": "infrastructure",
      "name": "고압 송전선",
      "distance": 1.2,
      "arrivalTime": "2025-04-15T16:30:00Z",
      "priority": "high"
    }
  ],

  "model": {
    "name": "FlamMap-FARSITE",
    "version": "5.4",
    "accuracy": 82.5
  }
}
```

---

## 6. 위성 영상 감지

### 6.1 MODIS 위성

**사양:**
- **위성**: Terra (오전 통과), Aqua (오후 통과)
- **공간 해상도**: 1km (열적외선)
- **시간 해상도**: 하루 4회 관측
- **열 밴드**: Band 21 (3.96 μm), Band 31 (11.03 μm)
- **탐지 임계값**: 밝기 온도 > 310K

**데이터 포맷:**

```json
{
  "satellite": "Terra-MODIS",
  "acquisitionTime": "2025-04-15T02:30:00Z",
  "tileId": "h28v05",
  "hotspots": [
    {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "brightness": 345.2,
      "frp": 125.5,
      "confidence": 95,
      "dayNight": "D"
    }
  ]
}
```

### 6.2 VIIRS 위성

**사양:**
- **위성**: Suomi NPP, NOAA-20, NOAA-21
- **공간 해상도**: 375m (I-band), 750m (M-band)
- **시간 해상도**: 하루 2-4회 관측
- **열 밴드**: I4 (3.74 μm), I5 (11.45 μm)
- **탐지 임계값**: 밝기 온도 > 325K

**장점:**
- MODIS 대비 4배 높은 공간 해상도
- 소형 화재 감지 능력 우수 (면적 50m² 이상)
- 야간 감지 성능 향상

### 6.3 Sentinel 위성

**사양:**
- **위성**: Sentinel-2 (광학), Sentinel-3 (열적외선)
- **공간 해상도**: 10-60m (Sentinel-2), 1km (Sentinel-3)
- **시간 해상도**: 5일 (Sentinel-2), 1일 (Sentinel-3)
- **활용**: 화재 피해 면적 평가, 식생 지수 모니터링

---

## 7. 열 감지 시스템

### 7.1 지상 열화상 카메라

**사양:**
- **탐지 범위**: 5-15km (기종별 상이)
- **회전 속도**: 360° / 60초
- **온도 감지 범위**: -40°C ~ 500°C
- **해상도**: 640×480 이상
- **프레임률**: 30fps
- **오탐지 필터**: AI 기반 동물/차량 제외

**설치 기준:**
- 산정상 또는 전망대 (가시거리 최대화)
- 10-15km 간격으로 배치
- 전원 공급 (태양광 패널 + 배터리)
- 통신: LTE/5G 또는 위성통신

### 7.2 드론 열화상 감지

**운용 시나리오:**
- 산불 초기 정밀 위치 확인
- 연기로 인한 가시성 부족 시 투입
- 잔불 수색 (진화 후)
- 인명 구조 지원 (열 신호로 실종자 탐색)

**드론 사양:**
- **비행 시간**: 30분 이상
- **탑재 센서**: 열화상 + 가시광 카메라
- **전송**: 실시간 영상 스트리밍 (5G)
- **자율 비행**: GPS + 장애물 회피

---

## 8. 연기 감지 시스템

### 8.1 광학 연기 감지기

**원리:**
- 레이저 산란 방식 (Laser Scattering)
- 연기 입자에 의한 빛 산란 측정
- 입자 농도 임계값: 0.05 mg/m³

**설치 위치:**
- 산림 내 감시탑
- 등산로 주요 지점
- 휴게소 및 야영장

### 8.2 AI 기반 영상 연기 감지

**기술:**
- CNN (Convolutional Neural Network) 모델
- 연기 패턴 학습 (10만+ 이미지)
- 구름, 안개, 먼지 구분
- 탐지 정확도: 92%+

**입력 데이터:**
- CCTV 실시간 영상
- 드론 영상
- 위성 영상 (저해상도 보조)

---

## 9. 기상 조건 모니터링

### 9.1 필수 기상 요소

| 요소 | 측정 단위 | 업데이트 주기 | 영향도 |
|------|-----------|---------------|--------|
| 온도 (Temperature) | °C | 1분 | 높음 |
| 상대습도 (Humidity) | % | 1분 | 매우 높음 |
| 풍속 (Wind Speed) | m/s | 1분 | 매우 높음 |
| 풍향 (Wind Direction) | ° | 1분 | 높음 |
| 강수량 (Precipitation) | mm | 1시간 | 높음 |
| 기압 (Pressure) | hPa | 10분 | 보통 |
| 일사량 (Solar Radiation) | W/m² | 1분 | 보통 |

### 9.2 자동 기상 관측소 (AWS)

**배치 밀도:**
- 산림 내: 50km² 당 1개소
- 고위험 지역: 20km² 당 1개소

**데이터 전송:**
- 실시간: 1분 간격
- 백업: LoRaWAN, 위성통신

---

## 10. 연료 수분 측정

### 10.1 고사 연료 수분 (DFMC)

**측정 방법:**
- **직접 측정**: 10시간 연료봉 중량법
- **간접 추정**: 기상 데이터 기반 모델링

**위험 임계값:**
- DFMC < 10%: 극도 위험
- DFMC 10-15%: 높은 위험
- DFMC 15-25%: 보통 위험
- DFMC > 25%: 낮은 위험

### 10.2 생체 연료 수분 (LFMC)

**측정 방법:**
- **실험실 분석**: 잎 샘플 채취 후 건조 중량 비교
- **원격 탐사**: NDVI (Normalized Difference Vegetation Index)

**계절별 패턴:**
- 봄철 (3-5월): LFMC 최저 (60-80%) → 산불 다발
- 여름철 (6-8월): LFMC 상승 (80-120%)
- 가을철 (9-11월): LFMC 하강 (70-100%)
- 겨울철 (12-2월): LFMC 최저 (50-70%)

---

## 11. 화재 확산 예측

### 11.1 Rothermel 확산 모델

**기본 방정식:**

```
R = (IR × ξ × (1 + Φw + Φs)) / (ρb × ε × Qig)

여기서:
R = 확산 속도 (m/min)
IR = 반응 강도 (kJ/m²/min)
ξ = 전파 유동비
Φw = 풍속 인자
Φs = 경사도 인자
ρb = 연료 밀도 (kg/m³)
ε = 유효 발열량 비율
Qig = 발화 열량 (kJ/kg)
```

### 11.2 FARSITE 시뮬레이션

**입력 데이터:**
- 지형도 (DEM, Slope, Aspect)
- 연료 유형 맵
- 기상 예보 (72시간)
- 초기 화재 경계

**출력 결과:**
- 시간대별 화재 경계 (GeoJSON)
- 화선 강도 히트맵
- 위협 지역 우선순위

### 11.3 AI 기반 예측

**딥러닝 모델:**
- **아키텍처**: ConvLSTM (시공간 데이터)
- **학습 데이터**: 과거 10년 산불 1,000건
- **정확도**: 85% (24시간 예측)

---

## 12. 대피 구역 관리

### 12.1 대피 구역 등급

| 등급 | 명칭 | 도착 예상 시간 | 조치 |
|------|------|----------------|------|
| RED | 즉시 대피 | 0-2시간 | 강제 대피 명령 |
| ORANGE | 대피 준비 | 2-6시간 | 대피 권고 |
| YELLOW | 주의 | 6-12시간 | 상황 감시 |
| GREEN | 안전 | >12시간 | 정상 활동 |

### 12.2 대피 경로 최적화

**알고리즘:**
- Dijkstra 최단 경로
- 도로 용량 고려
- 실시간 교통 정보 반영
- 장애인/노약자 우선 차량 배정

---

## 13. 소방 자원 관리

### 13.1 자원 유형

| 유형 | 수량 | 배치 기준 |
|------|------|-----------|
| 소방 헬기 | 전국 50대 | 30분 내 도달 |
| 소방차 | 전국 500대 | 20분 내 도달 |
| 소방 인력 | 10,000명 | 권역별 배치 |
| 진화 장비 | - | 전진 기지 비축 |

### 13.2 실시간 추적

```json
{
  "resourceId": "HELI-02",
  "type": "helicopter",
  "status": "en_route",
  "location": {
    "latitude": 37.5700,
    "longitude": 126.9850,
    "altitude": 500
  },
  "capacity": {
    "waterTank": 2000,
    "currentLoad": 1800
  },
  "eta": "2025-04-15T15:10:00Z"
}
```

---

## 14. 경보 시스템

### 14.1 경보 등급

| 등급 | 수신 대상 | 발송 채널 | 응답 시간 |
|------|-----------|-----------|-----------|
| Level 1 - 관심 | 관련 기관 | 이메일 | 1시간 |
| Level 2 - 주의 | 지역 소방서 | SMS + 이메일 | 30분 |
| Level 3 - 경계 | 전체 소방 | 긴급 문자 + 전화 | 10분 |
| Level 4 - 심각 | 국가 재난망 | 재난 문자 (CBS) | 즉시 |

### 14.2 다국어 지원

- 한국어, 영어, 중국어, 일본어
- 자동 번역 API 연동
- 음성 안내 (TTS)

---

## 15. 통합 및 상호운용성

### 15.1 국제 표준 준수

- **WMO (World Meteorological Organization)**: 기상 데이터
- **OGC (Open Geospatial Consortium)**: 지리 데이터
- **EDXL (Emergency Data Exchange Language)**: 재난 정보 교환
- **CAP (Common Alerting Protocol)**: 경보 프로토콜

### 15.2 API 엔드포인트

- `POST /api/v1/fire/detect` - 화재 감지 이벤트 등록
- `GET /api/v1/fire/{id}` - 화재 정보 조회
- `GET /api/v1/fire/active` - 진행 중 화재 목록
- `POST /api/v1/prediction/spread` - 확산 예측 요청
- `GET /api/v1/evacuation/zones` - 대피 구역 조회
- `POST /api/v1/resources/dispatch` - 자원 출동 지시
- `POST /api/v1/alert/send` - 경보 발송

---

## 16. 보안 및 개인정보보호

### 16.1 보안 요구사항

- **전송 암호화**: TLS 1.3
- **인증**: OAuth 2.0 + JWT
- **접근 제어**: RBAC (Role-Based Access Control)
- **감사 로그**: 모든 API 호출 기록 (90일 보관)

### 16.2 개인정보 보호

- 위치 정보: 일반 대중에게는 행정구역 수준까지만 공개
- 대피자 명단: 암호화 저장, 권한자만 접근
- GDPR/개인정보보호법 준수

---

## 17. 인증 요구사항

### 17.1 시스템 인증

**Tier 1 (기본):**
- 위성 데이터 수신 및 처리
- 기본 경보 발송
- 데이터 정확도 90% 이상

**Tier 2 (고급):**
- 다중 센서 융합
- AI 확산 예측
- 실시간 자원 추적
- 데이터 정확도 95% 이상

**Tier 3 (전문):**
- 전국 규모 통합 운영
- 국제 협력 체계
- 연구 개발 기능
- 데이터 정확도 98% 이상

### 17.2 연간 인증 심사

- 시스템 가동률 99.5% 이상
- 평균 탐지 시간 10분 이내
- 오탐지율 5% 미만
- 미탐지율 1% 미만

---

## 부록 A: 화재 행동 연료 모델 (FBFM)

### 13가지 표준 연료 모델 (Anderson, 1982)

| 모델 | 명칭 | 특성 | 확산 속도 |
|------|------|------|-----------|
| 1 | 짧은 초본 | 건조 초지 | 빠름 |
| 2 | 낮은 관목 + 초본 | 사바나 | 중간 |
| 3 | 높은 초본 | 습지 건조 시 | 매우 빠름 |
| 4 | 높은 관목 (2m) | 차파랄 | 빠름 |
| 5 | 짧은 관목 (0.6m) | 어린 산림 | 중간 |
| 6 | 휴면 관목 | 침엽수림 하층 | 중간 |
| 7 | 남부 거친 관목 | 2.5m 관목 | 느림 |
| 8 | 밀집 침엽수 | 낙엽층 깊음 | 느림-중간 |
| 9 | 활엽수 낙엽 | 낙엽층 | 느림 |
| 10 | 목재 낙하 | 베기 후 잔재 (가벼움) | 중간 |
| 11 | 목재 낙하 | 베기 후 잔재 (중간) | 중간 |
| 12 | 목재 낙하 | 베기 후 잔재 (무거움) | 빠름 |
| 13 | 목재 낙하 | 베기 후 잔재 (매우 무거움) | 매우 빠름 |

---

## 부록 B: 위성 데이터 접근 정보

### MODIS

- **NASA FIRMS**: https://firms.modaps.eosdis.nasa.gov/
- **API**: https://firms.modaps.eosdis.nasa.gov/api/
- **갱신 주기**: 3시간
- **무료 제공**: 예 (등록 필요)

### VIIRS

- **NOAA**: https://www.star.nesdis.noaa.gov/jpss/VIIRS.php
- **API**: https://firms.modaps.eosdis.nasa.gov/api/
- **갱신 주기**: 3시간
- **무료 제공**: 예 (등록 필요)

### Sentinel

- **Copernicus Hub**: https://scihub.copernicus.eu/
- **API**: https://scihub.copernicus.eu/twiki/do/view/SciHubWebPortal/APIHubDescription
- **갱신 주기**: 5일 (Sentinel-2), 1일 (Sentinel-3)
- **무료 제공**: 예 (등록 필요)

---

## 부록 C: 참고 문헌

1. Anderson, H. E. (1982). "Aids to determining fuel models for estimating fire behavior." USDA Forest Service.
2. Rothermel, R. C. (1972). "A mathematical model for predicting fire spread in wildland fuels." USDA Forest Service Research Paper INT-115.
3. Finney, M. A. (1998). "FARSITE: Fire Area Simulator - Model development and evaluation." USDA Forest Service Research Paper RMRS-RP-4.
4. NASA FIRMS. (2024). "Fire Information for Resource Management System User Guide."
5. Van Wagner, C. E. (1987). "Development and structure of the Canadian Forest Fire Weather Index System." Canadian Forestry Service Forestry Technical Report 35.

---

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 기술 사양
- 위성 감지 프로토콜
- AI 예측 모델 정의
- 국제 표준 준수

---

## 弘益人間 (홍익인간) · 널리 인간을 이롭게 하라

WIA-ENE-032 산불 감지 표준은 弘益人間(홍익인간)의 정신을 구현합니다. 첨단 기술과 국제 협력을 통해 산림 화재로부터 인명과 재산을 보호하고, 소중한 자연 생태계를 지켜냅니다.

개방형 표준, 투명한 데이터, 협력적 대응을 통해 산불 감지 및 대응 기술이 인류 전체와 지구 환경의 공동선에 기여하도록 보장합니다.

**함께, 우리는 산불로부터 안전한 세상을 만들어갑니다.**

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간) · Benefit All Humanity**
