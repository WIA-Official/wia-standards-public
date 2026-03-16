# WIA-ENE-034: 가뭄 모니터링 표준 v1.0

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 번호**: WIA-ENE-034
**버전**: 1.0.0
**발행일**: 2025-12-25
**상태**: 정식 표준
**카테고리**: 환경 및 에너지 (ENE)
**분류**: 가뭄 모니터링, 수자원 관리, 기후 적응

---

## 목차

1. [서론](#1-서론)
2. [적용 범위](#2-적용-범위)
3. [참조 표준](#3-참조-표준)
4. [용어 정의](#4-용어-정의)
5. [가뭄 지수](#5-가뭄-지수)
6. [모니터링 시스템](#6-모니터링-시스템)
7. [데이터 표준](#7-데이터-표준)
8. [API 사양](#8-api-사양)
9. [조기 경보 시스템](#9-조기-경보-시스템)
10. [가뭄 대응 체계](#10-가뭄-대응-체계)
11. [구현 가이드](#11-구현-가이드)
12. [보안 및 품질](#12-보안-및-품질)

---

## 1. 서론

### 1.1 배경

가뭄은 전 세계적으로 가장 파괴적인 자연재해 중 하나로, 매년 수십억 명의 인구와 수조 달러의 경제적 손실을 초래합니다. 기후 변화로 인해 가뭄의 빈도와 강도가 증가하고 있으며, 이에 대한 과학적이고 통합적인 모니터링이 필수적입니다.

### 1.2 목적

WIA-ENE-034 표준은 다음을 목적으로 합니다:

1. **표준화된 가뭄 지수**: SPI, PDSI, SPEI 등 국제적으로 인정된 가뭄 지수의 일관된 계산 및 해석
2. **통합 모니터링**: 기상, 수문, 농업, 사회경제적 가뭄을 통합적으로 모니터링
3. **조기 경보**: 가뭄 발생을 사전에 예측하여 선제적 대응
4. **데이터 상호운용성**: 다양한 기관 및 시스템 간 가뭄 데이터 공유 및 통합
5. **의사결정 지원**: 정책 결정자, 농민, 수자원 관리자에게 신뢰할 수 있는 정보 제공

### 1.3 원칙

- **과학적 근거**: 검증된 가뭄 지수 및 모니터링 기법 사용
- **다중 지표**: 단일 지표가 아닌 여러 지표의 종합적 평가
- **실시간성**: 신속한 데이터 수집 및 분석
- **접근성**: 모든 이해관계자가 쉽게 접근할 수 있는 정보 제공
- **확장성**: 새로운 기술 및 데이터 소스 통합 가능

---

## 2. 적용 범위

### 2.1 지리적 범위

- 전 세계 모든 기후대 및 지역
- 국가, 지역, 유역, 지방 수준의 모니터링
- 도시 및 농촌 지역

### 2.2 가뭄 유형

#### 2.2.1 기상학적 가뭄 (Meteorological Drought)
- 강수량 부족으로 인한 가뭄
- SPI, SPEI 등으로 평가
- 리드 타임: 가장 빠름

#### 2.2.2 농업 가뭄 (Agricultural Drought)
- 토양 수분 부족으로 인한 작물 스트레스
- 토양 수분, 식생 지수로 평가
- 리드 타임: 중간

#### 2.2.3 수문학적 가뭄 (Hydrological Drought)
- 지표수 및 지하수 부족
- 하천 유량, 저수지 저수율, 지하수위로 평가
- 리드 타임: 느림

#### 2.2.4 사회경제적 가뭄 (Socioeconomic Drought)
- 물 수요 초과로 인한 공급 부족
- 용수 제한, 경제적 손실로 평가
- 리드 타임: 가장 느림

### 2.3 대상 사용자

- 국가 및 지방 정부 (가뭄 정책 및 대응)
- 수자원 관리 기관 (저수지, 상수도)
- 농업 부문 (농민, 농업 협동조합)
- 기상청 및 연구 기관
- 보험사 및 금융 기관
- 일반 대중

---

## 3. 참조 표준

### 3.1 국제 표준

- **WMO-485**: Guide to Meteorological Instruments and Methods of Observation
- **ISO 19115**: Geographic information - Metadata
- **OGC SensorML**: Sensor Model Language
- **IPCC AR6**: Climate Change 2021 - Physical Science Basis

### 3.2 가뭄 지수 기준

- **McKee et al. (1993)**: Standardized Precipitation Index (SPI)
- **Palmer (1965)**: Palmer Drought Severity Index (PDSI)
- **Vicente-Serrano et al. (2010)**: SPEI
- **Kogan (1995)**: Vegetation Health Index (VHI)

### 3.3 WIA 관련 표준

- **WIA-ENE-017**: 기후 변화 관리
- **WIA-ENE-018**: 수질 모니터링
- **WIA-ENE-033**: 생태계 모니터링

---

## 4. 용어 정의

### 4.1 가뭄 관련 용어

#### 가뭄 (Drought)
장기간에 걸쳐 평년보다 현저히 적은 강수량으로 인해 물 부족이 발생하는 상태.

#### 가뭄 심도 (Drought Severity)
가뭄의 강도를 나타내는 척도. D0(이상 건조)부터 D4(예외적 가뭄)까지 5단계.

#### 가뭄 지속 기간 (Drought Duration)
가뭄이 시작되어 종료될 때까지의 기간.

#### 가뭄 발생 (Drought Onset)
가뭄 지수가 가뭄 임계값 이하로 떨어지는 시점.

#### 가뭄 종료 (Drought Termination)
가뭄 지수가 정상 수준으로 회복되는 시점.

### 4.2 가뭄 지수

#### SPI (Standardized Precipitation Index)
표준화된 강수 지수. 강수량의 확률 분포를 기반으로 계산.

#### PDSI (Palmer Drought Severity Index)
Palmer 가뭄 심도 지수. 물 수지(강수, 증발산, 토양 수분)를 기반으로 계산.

#### SPEI (Standardized Precipitation Evapotranspiration Index)
표준화된 강수-증발산 지수. 강수량과 잠재 증발산량의 차이를 기반으로 계산.

### 4.3 모니터링 용어

#### 토양 수분 (Soil Moisture)
토양에 포함된 물의 양. 부피 함수비(VWC, %)로 표현.

#### 지하수위 (Groundwater Level)
지표면으로부터 지하수면까지의 깊이.

#### 저수율 (Reservoir Storage Percentage)
저수지의 현재 저수량을 총 용량으로 나눈 백분율.

#### 식생 지수 (Vegetation Index)
위성 영상을 이용하여 계산한 식생의 건강도 지표 (NDVI, EVI 등).

---

## 5. 가뭄 지수

### 5.1 SPI (Standardized Precipitation Index)

#### 5.1.1 개요

SPI는 강수량의 확률 분포를 기반으로 한 표준화된 가뭄 지수입니다. 다양한 시간 척도(1, 3, 6, 12, 24개월)로 계산 가능하여 서로 다른 가뭄 유형을 평가할 수 있습니다.

#### 5.1.2 계산 방법

1. **강수량 데이터 수집**: 30년 이상의 장기 강수량 데이터
2. **확률 분포 적합**: 감마(Gamma) 또는 피어슨(Pearson) 분포 적합
3. **누적 확률 계산**: 현재 강수량의 누적 확률 계산
4. **표준 정규 분포 변환**: 누적 확률을 표준 정규 분포로 변환

**수식**:
```
SPI = Φ^(-1)(F(x))
```
여기서:
- Φ^(-1): 표준 정규 분포의 역함수
- F(x): 강수량 x의 누적 확률

#### 5.1.3 시간 척도

| 시간 척도 | 용도 | 가뭄 유형 |
|---------|------|---------|
| SPI-1 | 단기 강수량 이상, 기상학적 가뭄 | 기상학적 |
| SPI-3 | 계절 강수량 평가, 농업 가뭄 | 농업 |
| SPI-6 | 반기 강수량, 토양 수분 | 농업 |
| SPI-12 | 수문학적 가뭄, 저수지 | 수문학적 |
| SPI-24 | 장기 가뭄, 지하수 | 수문학적 |

#### 5.1.4 해석

| SPI 값 | 분류 | 확률 (%) |
|--------|------|---------|
| ≥ 2.00 | 극심한 습윤 | 2.3 |
| 1.50 ~ 1.99 | 심한 습윤 | 4.4 |
| 1.00 ~ 1.49 | 보통 습윤 | 9.2 |
| -0.99 ~ 0.99 | 정상 | 68.2 |
| -1.00 ~ -1.49 | 보통 가뭄 | 9.2 |
| -1.50 ~ -1.99 | 심한 가뭄 | 4.4 |
| ≤ -2.00 | 극심한 가뭄 | 2.3 |

### 5.2 PDSI (Palmer Drought Severity Index)

#### 5.2.1 개요

PDSI는 Palmer(1965)가 개발한 지수로, 물 수지 모형을 기반으로 가뭄을 평가합니다. 강수량, 온도, 토양 특성을 고려하여 계산됩니다.

#### 5.2.2 계산 방법

1. **수분 수지 계산**:
   - P (Precipitation): 강수량
   - PET (Potential Evapotranspiration): 잠재 증발산량
   - PR (Potential Recharge): 잠재 재충전량
   - RO (Runoff): 유출
   - L (Loss): 손실

2. **수분 이상치 계산**:
   ```
   d = P - P̂
   ```
   여기서 P̂는 기후학적 적정 강수량(CAFEC)

3. **Z-지수 계산**:
   ```
   Z = K × d
   ```
   여기서 K는 기후 특성 계수

4. **PDSI 계산**:
   ```
   PDSI_t = 0.897 × PDSI_{t-1} + (1/3) × Z_t
   ```

#### 5.2.3 해석

| PDSI 값 | 분류 |
|---------|------|
| ≥ 4.00 | 극심한 습윤 |
| 3.00 ~ 3.99 | 매우 습윤 |
| 2.00 ~ 2.99 | 보통 습윤 |
| 1.00 ~ 1.99 | 약간 습윤 |
| 0.50 ~ 0.99 | 정상 범위 |
| -0.49 ~ 0.49 | 정상 |
| -0.99 ~ -0.50 | 정상 범위 |
| -1.99 ~ -1.00 | 경미한 가뭄 |
| -2.99 ~ -2.00 | 보통 가뭄 |
| -3.99 ~ -3.00 | 심한 가뭄 |
| ≤ -4.00 | 극심한 가뭄 |

### 5.3 SPEI (Standardized Precipitation Evapotranspiration Index)

#### 5.3.1 개요

SPEI는 SPI의 확장으로, 강수량뿐만 아니라 잠재 증발산량(PET)도 고려합니다. 기후 변화에 따른 온도 상승의 영향을 반영할 수 있습니다.

#### 5.3.2 계산 방법

1. **기후 수지 계산**:
   ```
   D_i = P_i - PET_i
   ```
   여기서:
   - D_i: i월의 기후 수지
   - P_i: i월의 강수량
   - PET_i: i월의 잠재 증발산량

2. **누적 기후 수지**:
   시간 척도 k에 대해:
   ```
   D_n^k = Σ(D_{n-i+1})  (i=0 to k-1)
   ```

3. **확률 분포 적합**: 3-매개변수 log-logistic 분포
4. **표준화**: SPI와 동일한 방식으로 표준 정규 분포로 변환

#### 5.3.3 PET 계산 방법

**Thornthwaite 방법** (기본):
```
PET = 16 × (L/12) × (N/30) × (10T/I)^a
```
여기서:
- L: 월평균 일조 시간
- N: 해당 월의 일수
- T: 월평균 기온 (°C)
- I: 연간 열 지수
- a: 경험적 지수

**Penman-Monteith 방법** (권장):
FAO-56 기준 참조 증발산량 계산

#### 5.3.4 해석

SPEI의 해석은 SPI와 동일합니다.

### 5.4 종합 가뭄 지수 (Composite Drought Index)

#### 5.4.1 개요

여러 가뭄 지수를 가중 평균하여 종합적인 가뭄 상황을 평가합니다.

#### 5.4.2 계산 방법

```
CDI = w1×SPI_norm + w2×PDSI_norm + w3×SM_norm + w4×VHI
```

여기서:
- SPI_norm: 표준화된 SPI (0-100 척도)
- PDSI_norm: 표준화된 PDSI (0-100 척도)
- SM_norm: 표준화된 토양 수분 백분위수
- VHI: 식생 건강 지수 (0-100)
- 가중치: w1=0.3, w2=0.3, w3=0.25, w4=0.15

#### 5.4.3 해석

| CDI 값 | 가뭄 심도 |
|--------|---------|
| 80-100 | 정상 |
| 60-79 | 이상 건조 (D0) |
| 40-59 | 보통 가뭄 (D1) |
| 20-39 | 심한 가뭄 (D2) |
| 10-19 | 극심한 가뭄 (D3) |
| 0-9 | 예외적 가뭄 (D4) |

---

## 6. 모니터링 시스템

### 6.1 토양 수분 모니터링

#### 6.1.1 현장 센서

**센서 유형**:
- TDR (Time Domain Reflectometry): 높은 정확도
- 정전용량 센서 (Capacitance): 비용 효율적
- 텐시오미터 (Tensiometer): 토양 수분 장력 측정

**설치 깊이** (cm):
- 0-10: 표층 수분
- 10-30: 뿌리대 상부
- 30-60: 뿌리대 중부
- 60-100: 뿌리대 하부

**측정 빈도**: 매 15분 ~ 1시간

#### 6.1.2 위성 원격탐사

**SMAP (Soil Moisture Active Passive)**:
- 해상도: 9 km
- 깊이: 표층 5 cm
- 재방문: 2-3일
- 정확도: ±0.04 m³/m³

**SMOS (Soil Moisture and Ocean Salinity)**:
- 해상도: 40 km
- 깊이: 표층 5 cm
- 재방문: 3일

**Sentinel-1 (SAR)**:
- 해상도: 10 m
- 깊이: 표층 ~10 cm
- 재방문: 6일 (A+B)

### 6.2 지하수 모니터링

#### 6.2.1 관정 네트워크

**관정 유형**:
- 국가 기준 관정
- 지역 모니터링 관정
- 농업용 관정

**측정 항목**:
- 지하수위 (m, 지표면 기준)
- 수온 (°C)
- 전기전도도 (μS/cm)
- pH

**측정 빈도**:
- 자동 측정: 매 1시간
- 수동 측정: 주 1회 ~ 월 1회

#### 6.2.2 데이터 품질 관리

- 이상치 탐지 및 제거
- 기압 보정
- 계절 변동 고려
- 장기 추세 분석

### 6.3 저수지 및 댐 모니터링

#### 6.3.1 측정 항목

**저수량**:
- 현재 저수량 (million m³)
- 총 용량 대비 비율 (%)
- 유효 저수량
- 사수량 (dead storage)

**수위**:
- 현재 수위 (EL.m)
- 만수위
- 저수위
- 최저 운영 수위

**유입/방류**:
- 유입량 (m³/s)
- 방류량 (m³/s)
- 순 변화량 (m³/day)

#### 6.3.2 통계 분석

- 평년 동기 대비
- 역사적 백분위수 (0-100)
- 동일 날짜 최저 기록
- 30일 추세 (증가/감소/안정)

### 6.4 강수량 모니터링

#### 6.4.1 데이터 소스

**지상 관측소**:
- 우량계 (tipping bucket, weighing)
- 밀도: 1개소 / 100-500 km² (지역 특성에 따라)
- 측정 빈도: 1분 ~ 1시간

**레이더**:
- 기상 레이더 (S-band, C-band)
- 해상도: 1 km × 1 km
- 갱신 빈도: 5-10분

**위성**:
- GPM (Global Precipitation Measurement)
- TRMM (Tropical Rainfall Measuring Mission)
- IMERG (Integrated Multi-satellitE Retrievals for GPM)

#### 6.4.2 품질 관리

- 레이더-우량계 보정
- 위성-지상 검증
- 공간 내삽 (IDW, Kriging)

### 6.5 식생 모니터링

#### 6.5.1 위성 식생 지수

**NDVI (Normalized Difference Vegetation Index)**:
```
NDVI = (NIR - Red) / (NIR + Red)
```
- 범위: -1 ~ +1
- 건강한 식생: 0.6 ~ 0.9
- 스트레스 식생: 0.2 ~ 0.5

**EVI (Enhanced Vegetation Index)**:
```
EVI = 2.5 × (NIR - Red) / (NIR + 6×Red - 7.5×Blue + 1)
```
- 대기 보정 개선
- 높은 바이오매스에서 포화 감소

**VCI (Vegetation Condition Index)**:
```
VCI = 100 × (NDVI - NDVI_min) / (NDVI_max - NDVI_min)
```
- 범위: 0-100
- 0-10: 극심한 가뭄
- 10-20: 심한 가뭄
- 20-30: 보통 가뭄
- 30-40: 이상 건조
- 40-100: 정상

**VHI (Vegetation Health Index)**:
```
VHI = 0.5 × VCI + 0.5 × TCI
```
여기서 TCI는 Temperature Condition Index

#### 6.5.2 위성 플랫폼

| 위성 | 센서 | 해상도 | 재방문 | 밴드 |
|------|------|--------|--------|------|
| MODIS | Terra/Aqua | 250-1000m | 1-2일 | 36 |
| Sentinel-2 | MSI | 10-20m | 5일 | 13 |
| Landsat-8/9 | OLI | 30m | 16일 | 9 |
| VIIRS | NOAA-20 | 375m | 1일 | 22 |

---

## 7. 데이터 표준

### 7.1 데이터 포맷

#### 7.1.1 JSON 스키마

모든 데이터 교환은 JSON 포맷을 사용합니다.

**공통 필드**:
```json
{
  "monitoringId": "string",
  "timestamp": "ISO 8601 datetime",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "elevation": "number (optional)"
  }
}
```

#### 7.1.2 시간 포맷

ISO 8601 형식을 사용합니다:
- `2025-12-25T10:00:00Z` (UTC)
- `2025-12-25T19:00:00+09:00` (KST)

#### 7.1.3 공간 좌표

- 좌표계: WGS84 (EPSG:4326)
- 위도: -90 ~ +90 (십진법 도)
- 경도: -180 ~ +180 (십진법 도)
- 고도: 해수면 기준 (m)

### 7.2 메타데이터

모든 데이터셋은 다음 메타데이터를 포함해야 합니다:

```json
{
  "metadata": {
    "dataQuality": "number (0-100)",
    "source": "string",
    "method": "string",
    "uncertainty": "number (%)",
    "lastUpdate": "ISO 8601 datetime",
    "version": "string"
  }
}
```

### 7.3 단위 표준

| 항목 | 단위 | 기호 |
|------|------|------|
| 강수량 | 밀리미터 | mm |
| 온도 | 섭씨 | °C |
| 증발산 | 밀리미터 | mm |
| 토양 수분 | 부피 함수비 | % VWC |
| 지하수위 | 미터 (지표 하) | m |
| 저수량 | 백만 세제곱미터 | million m³ |
| 유량 | 세제곱미터/초 | m³/s |

---

## 8. API 사양

### 8.1 인증

모든 API 요청은 Bearer 토큰 인증을 사용합니다:

```http
Authorization: Bearer YOUR_API_KEY
X-WIA-Standard: ENE-034
X-WIA-Version: 1.0.0
```

### 8.2 공통 응답 형식

```json
{
  "success": "boolean",
  "data": "object or array",
  "error": {
    "code": "string",
    "message": "string",
    "details": "object (optional)"
  },
  "metadata": {
    "timestamp": "ISO 8601 datetime",
    "version": "string"
  }
}
```

### 8.3 주요 엔드포인트

#### 8.3.1 가뭄 지수

**GET /api/v1/indices/spi**

Query Parameters:
- `latitude` (required): number
- `longitude` (required): number
- `startDate` (optional): ISO 8601 datetime
- `endDate` (optional): ISO 8601 datetime

Response:
```json
{
  "success": true,
  "data": [
    {
      "monitoringId": "SPI-20251225-001",
      "location": {...},
      "timestamp": "2025-12-25T10:00:00Z",
      "spi1Month": -1.25,
      "spi3Month": -1.58,
      "spi6Month": -1.92,
      "spi12Month": -2.15,
      "spi24Month": -1.85,
      "interpretation": {...}
    }
  ]
}
```

#### 8.3.2 종합 보고서

**GET /api/v1/monitoring/report**

Query Parameters:
- `latitude` (required): number
- `longitude` (required): number
- `date` (optional): YYYY-MM-DD

Response:
```json
{
  "success": true,
  "data": {
    "reportId": "REPORT-20251225-001",
    "location": {...},
    "compositeDroughtIndex": {
      "value": 35.2,
      "severity": "severe_drought",
      "confidence": 88
    },
    "components": {...},
    "trend": {...},
    "recommendations": {...}
  }
}
```

### 8.4 오류 코드

| 코드 | HTTP | 설명 |
|------|------|------|
| INVALID_COORDINATES | 400 | 유효하지 않은 좌표 |
| UNAUTHORIZED | 401 | 인증 실패 |
| FORBIDDEN | 403 | 권한 없음 |
| NOT_FOUND | 404 | 데이터 없음 |
| RATE_LIMIT_EXCEEDED | 429 | 요청 한도 초과 |
| INTERNAL_ERROR | 500 | 서버 오류 |

---

## 9. 조기 경보 시스템

### 9.1 경보 단계

| 단계 | 명칭 | 조건 | 조치 |
|------|------|------|------|
| 정상 | Normal | SPI > -0.5, 저수율 > 60% | 일상 모니터링 |
| 주의 | Watch | SPI -0.5 ~ -1.0, 저수율 40-60% | 모니터링 강화 |
| 경고 | Warning | SPI -1.0 ~ -1.5, 저수율 20-40% | 용수 절약 권고 |
| 비상 | Emergency | SPI < -1.5, 저수율 < 20% | 용수 제한 시행 |

### 9.2 예측 모델

#### 9.2.1 단기 예측 (7-30일)

- 기상 예보 모델 활용 (GFS, ECMWF)
- 앙상블 예측
- 확률론적 접근

#### 9.2.2 중기 예측 (1-3개월)

- 계절 예측 모델
- SST (해수면 온도) 패턴 분석
- ENSO (엘니뇨-남방진동) 영향

#### 9.2.3 장기 전망 (3-12개월)

- 기후 모델 (CMIP6)
- 통계적 다운스케일링
- 기계 학습 모델

### 9.3 경보 발령 기준

가뭄 경보는 다음 조건 중 2개 이상을 만족할 때 발령:

1. SPI-6 < -1.0 (3개월 연속)
2. PDSI < -2.0
3. 주요 저수지 저수율 < 40% (평년 대비 -30%p)
4. 토양 수분 < 30백분위수
5. VHI < 30

---

## 10. 가뭄 대응 체계

### 10.1 용수 제한 단계

#### 10.1.1 1단계 (자발적 절수)

**조건**:
- SPI-6: -0.5 ~ -1.0
- 저수율: 50-60%

**조치**:
- 옥외 살수 자제
- 절수 홍보 캠페인
- 목표: 10% 절감

#### 10.1.2 2단계 (권고적 제한)

**조건**:
- SPI-6: -1.0 ~ -1.5
- 저수율: 40-50%

**조치**:
- 옥외 살수 주 2-3회 제한
- 세차 자제
- 분수 가동 중단
- 목표: 20% 절감

#### 10.1.3 3단계 (의무적 제한)

**조건**:
- SPI-6: -1.5 ~ -2.0
- 저수율: 30-40%

**조치**:
- 옥외 살수 금지
- 세차 금지
- 농업 관개 30% 감축
- 벌금 부과
- 목표: 30% 절감

#### 10.1.4 4단계 (비상 제한)

**조건**:
- SPI-6: < -2.0
- 저수율: < 30%

**조치**:
- 모든 비필수 용수 사용 금지
- 급수 제한 (시간대별)
- 농업 관개 50% 감축
- 비상 급수
- 목표: 40% 이상 절감

### 10.2 가뭄 선포

#### 10.2.1 선포 절차

1. 가뭄 모니터링 위원회 소집
2. 가뭄 지수 및 영향 평가
3. 가뭄 선포 건의
4. 정부 승인 및 공표
5. 대응 계획 시행

#### 10.2.2 선포 내용

- 가뭄 유형 (기상학적, 농업, 수문학적)
- 가뭄 심도 (D0-D4)
- 영향 지역 및 인구
- 예상 지속 기간
- 대응 조치

### 10.3 농업 지원

#### 10.3.1 긴급 지원

- 가뭄 구호 자금
- 농작물 재해보험
- 사료 공급 지원
- 가축 용수 지원

#### 10.3.2 기술 지원

- 가뭄 저항성 품종 보급
- 점적 관개 시스템 설치 지원
- 멀칭 기술 교육
- 토양 수분 센서 보급

---

## 11. 구현 가이드

### 11.1 최소 요구사항

#### 11.1.1 데이터 인프라

- 강수량 관측소: 1개소 / 500 km²
- 토양 수분 센서: 주요 농경지
- 지하수 관정: 1개소 / 1,000 km²
- 저수지 모니터링: 100% (주요 댐)

#### 11.1.2 기술 요구사항

- 데이터베이스: PostgreSQL 13+ with PostGIS
- API 서버: Node.js 18+ or Python 3.9+
- 프론트엔드: React 18+ or Vue 3+
- 지도: Leaflet or OpenLayers

### 11.2 데이터 흐름

```
[센서/위성] → [데이터 수집] → [품질 관리] → [데이터베이스]
                                                    ↓
[사용자] ← [시각화/API] ← [가뭄 지수 계산] ← [분석 엔진]
```

### 11.3 배포 아키텍처

```
[로드 밸런서]
      ↓
[API 게이트웨이]
      ↓
[마이크로서비스]
  - 가뭄 지수 서비스
  - 모니터링 서비스
  - 예보 서비스
  - 알림 서비스
      ↓
[데이터 레이어]
  - PostgreSQL (관계형 데이터)
  - TimescaleDB (시계열 데이터)
  - Redis (캐시)
```

---

## 12. 보안 및 품질

### 12.1 데이터 보안

#### 12.1.1 전송 보안

- TLS 1.3 암호화
- HTTPS only
- API 키 암호화 저장

#### 12.1.2 접근 제어

- 역할 기반 접근 제어 (RBAC)
- 사용자 역할: 관리자, 분석가, 공공
- API 요청 속도 제한

### 12.2 데이터 품질

#### 12.2.1 품질 지표

- 완전성: > 95%
- 정확도: ±5% (강수량), ±0.04 m³/m³ (토양 수분)
- 적시성: < 24시간 (데이터 갱신)
- 일관성: 100% (포맷 준수)

#### 12.2.2 품질 관리 절차

1. 실시간 이상치 탐지
2. 자동 플래깅
3. 전문가 검증
4. 보정 및 수정
5. 문서화

### 12.3 성능

- API 응답 시간: < 500ms (95th percentile)
- 가용성: 99.9% uptime
- 데이터 처리 지연: < 1시간

---

## 부록 A: 참고 문헌

1. McKee, T. B., Doesken, N. J., & Kleist, J. (1993). The relationship of drought frequency and duration to time scales. In Proceedings of the 8th Conference on Applied Climatology (Vol. 17, No. 22, pp. 179-183).

2. Palmer, W. C. (1965). Meteorological drought (Vol. 30). US Department of Commerce, Weather Bureau.

3. Vicente-Serrano, S. M., Beguería, S., & López-Moreno, J. I. (2010). A multiscalar drought index sensitive to global warming: the standardized precipitation evapotranspiration index. Journal of climate, 23(7), 1696-1718.

4. Kogan, F. N. (1995). Application of vegetation index and brightness temperature for drought detection. Advances in Space Research, 15(11), 91-100.

5. World Meteorological Organization (2012). Standardized Precipitation Index User Guide. WMO-No. 1090.

---

## 부록 B: 용어집

**가용 토양 수분 (Available Water Capacity, AWC)**: 포장용수량과 영구위조점 사이의 수분량.

**기후 수지 (Climatic Water Balance)**: 강수량과 잠재 증발산량의 차이.

**영구위조점 (Permanent Wilting Point, PWP)**: 식물이 더 이상 물을 흡수할 수 없는 토양 수분 함량.

**잠재 증발산량 (Potential Evapotranspiration, PET)**: 물 공급이 충분할 때 증발 및 증산으로 대기로 이동하는 최대 수분량.

**포장용수량 (Field Capacity, FC)**: 중력에 의한 배수가 끝난 후 토양이 보유하는 수분량.

---

## 부록 C: 예제 코드

### TypeScript SDK 사용 예제

```typescript
import { DroughtMonitoringSDK } from '@wia/ene-034';

const sdk = new DroughtMonitoringSDK({
  apiKey: process.env.WIA_API_KEY!,
  endpoint: 'https://api.wia.org/ene-034/v1'
});

async function main() {
  // SPI 데이터 조회
  const spi = await sdk.getSPI(
    { latitude: 37.5665, longitude: 126.9780 },
    {
      startDate: '2025-01-01T00:00:00Z',
      endDate: '2025-12-31T23:59:59Z'
    }
  );

  if (spi.success && spi.data) {
    console.log('SPI-12:', spi.data[0].spi12Month);
    console.log('Severity:', spi.data[0].interpretation.severity);
  }

  // 종합 가뭄 보고서
  const report = await sdk.getDroughtMonitoringReport(
    { latitude: 37.5665, longitude: 126.9780 }
  );

  if (report.success && report.data) {
    console.log('Composite Index:', report.data.compositeDroughtIndex.value);
    console.log('Trend:', report.data.trend.direction);
  }
}

main();
```

---

## 부록 D: 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- SPI, PDSI, SPEI 표준 정의
- 토양 수분, 지하수, 저수지 모니터링 사양
- 식생 스트레스 및 농업 영향 평가
- 조기 경보 시스템 및 가뭄 대응 체계
- REST API 사양
- TypeScript SDK

---

## 저작권 및 라이선스

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)

이 문서는 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

**弘益人間 (홍익인간) · Benefit All Humanity**

---

**문서 끝**
