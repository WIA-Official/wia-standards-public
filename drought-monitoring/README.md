# WIA-ENE-034: 가뭄 모니터링 표준 🏜️

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--ENE--034-red.svg)](https://wia.org/standards/ene-034)

## 개요

WIA-ENE-034 가뭄 모니터링 표준은 가뭄의 발생, 심화, 지속, 종료를 과학적으로 모니터링하고 평가하기 위한 국제 표준입니다. 이 표준은 다양한 가뭄 지수(SPI, PDSI, SPEI), 토양 수분, 지하수위, 저수지 저수율, 식생 스트레스, 농업 영향을 통합하여 종합적인 가뭄 상황을 제공합니다.

### 주요 기능

- 📊 **가뭄 지수 모니터링**: SPI, PDSI, SPEI 등 표준화된 가뭄 지수
- 💧 **수자원 모니터링**: 토양 수분, 지하수위, 저수지 저수율, 강수량
- 🌿 **식생 스트레스**: NDVI, VHI, VCI 등 위성 기반 식생 건강도
- 🌾 **농업 영향 평가**: 작물 수분 스트레스, 수량 감소, 경제적 손실
- 🚰 **용수 제한 관리**: 단계별 절수 조치 및 집행
- 📢 **가뭄 선포 추적**: 공식 가뭄 선포 및 비상 대응
- 🔮 **예보 및 조기 경보**: 가뭄 발생 예측 및 사전 대응
- 📈 **추세 분석**: 장기 가뭄 추세 및 기후 변화 영향

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/drought-monitoring

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { DroughtMonitoringSDK } from '@wia/ene-034';

const sdk = new DroughtMonitoringSDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-034/v1'
});

// SPI 데이터 조회
const spi = await sdk.getSPI(
  { latitude: 37.5665, longitude: 126.9780 },
  {
    startDate: '2025-01-01T00:00:00Z',
    endDate: '2025-12-31T23:59:59Z'
  }
);

console.log('SPI 12-month:', spi.data?.[0]?.spi12Month);
console.log('Drought Severity:', spi.data?.[0]?.interpretation.severity);

// 종합 가뭄 모니터링 보고서
const report = await sdk.getDroughtMonitoringReport(
  { latitude: 37.5665, longitude: 126.9780 }
);

console.log('Composite Drought Index:', report.data?.compositeDroughtIndex.value);
console.log('Severity:', report.data?.compositeDroughtIndex.severity);

// 저수지 저수율 조회
const reservoirs = await sdk.listReservoirs({ region: 'Seoul' });
const storage = await sdk.getReservoirStorage('RES-001');

console.log('Reservoir Storage:', storage.data?.[0]?.storage.percentFull + '%');
console.log('Alert Level:', storage.data?.[0]?.droughtStatus.alertLevel);

// 가뭄 예보
const forecast = await sdk.getDroughtForecast(
  { latitude: 37.5665, longitude: 126.9780 }
);

console.log('Onset Probability:', forecast.data?.droughtPrediction.onset.probability + '%');
console.log('Alert Level:', forecast.data?.earlyWarning.alertLevel);
```

### 3. CLI 도구 사용

```bash
# SPI 데이터 조회
./cli/drought-monitoring.sh spi 37.5665 126.9780 2025-01-01 2025-12-31

# PDSI 데이터 조회
./cli/drought-monitoring.sh pdsi 37.5665 126.9780

# 토양 수분 조회
./cli/drought-monitoring.sh soil-moisture 37.5665 126.9780

# 저수지 목록 조회
./cli/drought-monitoring.sh reservoir list

# 저수지 저수율 조회
./cli/drought-monitoring.sh reservoir get RES-001

# 식생 스트레스 조회
./cli/drought-monitoring.sh vegetation 37.5665 126.9780

# 종합 보고서 조회
./cli/drought-monitoring.sh report 37.5665 126.9780

# 가뭄 예보 조회
./cli/drought-monitoring.sh forecast 37.5665 126.9780

# 용수 제한 조회
./cli/drought-monitoring.sh restrictions Seoul

# 가뭄 선포 목록 조회
./cli/drought-monitoring.sh declarations list
```

### 4. 상세 사양 확인

- **스펙 문서**: [`spec/WIA-ENE-034-v1.0.md`](spec/WIA-ENE-034-v1.0.md)

## 저장소 구조

```
drought-monitoring/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-ENE-034-v1.0.md  # 상세 스펙
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # 타입 정의
│       │   └── index.ts    # SDK 구현
│       └── package.json    # npm 패키지 설정
└── cli/
    └── drought-monitoring.sh  # CLI 도구
```

## 기술 범위

### 가뭄 지수

#### 1. SPI (Standardized Precipitation Index)
- **정의**: 강수량 확률 분포 기반 표준화 지수
- **시간 척도**: 1, 3, 6, 12, 24개월
- **범위**: -3.0 (극심한 가뭄) ~ +3.0 (극심한 습윤)
- **특징**: 계산 간단, 비교 용이, 다중 시간 척도
- **용도**: 기상학적 가뭄 탐지

#### 2. PDSI (Palmer Drought Severity Index)
- **정의**: 물 수지 기반 가뭄 지수
- **범위**: -10 (극심한 가뭄) ~ +10 (극심한 습윤)
- **입력**: 강수량, 온도, 가용 토양 수분
- **특징**: 토양 수분 변화 반영, 자기상관 높음
- **용도**: 농업 및 수문학적 가뭄

#### 3. SPEI (Standardized Precipitation Evapotranspiration Index)
- **정의**: 강수량과 증발산을 결합한 지수
- **시간 척도**: 1, 3, 6, 12, 24개월
- **범위**: -3.0 ~ +3.0
- **특징**: 온도 영향 반영, 기후 변화 민감
- **용도**: 농업 및 수자원 관리

### 모니터링 구성 요소

#### 토양 수분 (Soil Moisture)
- **깊이**: 0-10cm, 10-30cm, 30-60cm, 60-100cm
- **측정**: 부피 함수비 (VWC, %)
- **지표**: 식물 이용 가능 수분, 토양 수분 결핍, 이상치
- **출처**: 현장 센서 (TDR, capacitance), 위성 (SMAP, SMOS, Sentinel-1)

#### 지하수 (Groundwater)
- **측정**: 지표 하 수위 깊이 (m)
- **변화**: 일별, 주별, 월별, 연별 변동
- **이상**: 평년 대비 편차 및 백분위수
- **영향**: 가뭄 심도, 회복 시간 예측

#### 저수지 (Reservoirs)
- **저수율**: 현재 저수량 / 총 용량 (%)
- **통계**: 평년 대비, 역사적 백분위수, 최저치
- **유입/방류**: 유입량, 방류량, 순 변화
- **공급**: 생활, 농업, 공업 용수 공급 일수

#### 강수량 결핍 (Precipitation Deficit)
- **기간**: 30일, 90일, 6개월, 12개월, 물 해(water year)
- **결핍량**: 절대 결핍량 (mm), 평년 대비 비율 (%)
- **무강우**: 연속 무강우 일수

#### 식생 스트레스 (Vegetation Stress)
- **NDVI**: Normalized Difference Vegetation Index (-1 ~ +1)
- **VCI**: Vegetation Condition Index (0-100)
- **VHI**: Vegetation Health Index (0-100)
- **NDWI**: Normalized Difference Water Index (-1 ~ +1)
- **위성**: MODIS, Sentinel-2, Landsat-8/9

#### 농업 영향 (Agricultural Impact)
- **작물 상태**: 건강 상태, 스트레스 수준, 수관 피복
- **수분 스트레스**: CWSI, 토양 수분 적정도, 관개 필요량
- **수량 영향**: 예상 수량, 수량 감소율, 경제적 손실

### 가뭄 심도 분류

| 등급 | 명칭 | SPI | PDSI | 영향 |
|------|------|-----|------|------|
| D0 | 이상 건조 | -0.5 ~ -0.79 | -1.0 ~ -1.99 | 농작물 성장 지연, 산불 위험 |
| D1 | 보통 가뭄 | -0.8 ~ -1.29 | -2.0 ~ -2.99 | 일부 작물 피해, 하천 수위 저하 |
| D2 | 심한 가뭄 | -1.3 ~ -1.59 | -3.0 ~ -3.99 | 작물 피해 확대, 용수 제한 |
| D3 | 극심한 가뭄 | -1.6 ~ -1.99 | -4.0 ~ -4.99 | 광범위한 피해, 물 부족 |
| D4 | 예외적 가뭄 | -2.0 이하 | -5.0 이하 | 재난 수준, 비상 급수 |

## 데이터 포맷 표준

### SPI 데이터

```json
{
  "monitoringId": "SPI-20251225-001",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "timestamp": "2025-12-25T10:00:00Z",
  "spi1Month": -1.25,
  "spi3Month": -1.58,
  "spi6Month": -1.92,
  "spi12Month": -2.15,
  "spi24Month": -1.85,
  "interpretation": {
    "severity": "severe_drought",
    "description": "심한 가뭄 (6개월 기준 극심한 가뭄)",
    "probability": 2.3
  },
  "metadata": {
    "baselinePeriod": "1981-2010",
    "distribution": "gamma",
    "dataQuality": 95
  }
}
```

### 저수지 저수율 데이터

```json
{
  "monitoringId": "RES-20251225-001",
  "reservoirId": "RES-001",
  "reservoirName": "소양강댐",
  "location": {
    "latitude": 38.0,
    "longitude": 127.8
  },
  "timestamp": "2025-12-25T10:00:00Z",
  "storage": {
    "currentVolume": 1450.5,
    "capacity": 2900.0,
    "percentFull": 50.0,
    "deadStorage": 200.0,
    "activeStorage": 1250.5
  },
  "statistics": {
    "percentileRank": 25,
    "averageForDate": 75.0,
    "deviation": -25.0,
    "lowestEverForDate": 48.2,
    "trend30Days": "decreasing"
  },
  "droughtStatus": {
    "severity": "moderate_drought",
    "alertLevel": "warning",
    "restrictionsInPlace": true
  }
}
```

### 종합 가뭄 모니터링 보고서

```json
{
  "reportId": "REPORT-20251225-001",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "region": "경기도 수원시",
  "timestamp": "2025-12-25T10:00:00Z",
  "compositeDroughtIndex": {
    "value": 35.2,
    "severity": "severe_drought",
    "confidence": 88
  },
  "components": {
    "spi": { "spi12Month": -2.15, "severity": "extreme_drought" },
    "pdsi": { "pdsi": -3.5, "severity": "severe_drought" },
    "soilMoisture": { "relativeSoilMoisture": 25, "severity": "severe_drought" },
    "reservoirStorage": { "percentFull": 45, "alertLevel": "warning" },
    "vegetationStress": { "vhi": 32, "severity": "severe_drought" }
  },
  "trend": {
    "direction": "worsening",
    "changeRate": -2.5,
    "forecast7Day": "severe_drought",
    "forecast30Day": "extreme_drought"
  },
  "recommendations": {
    "waterConservation": [
      "용수 사용량 30% 절감",
      "옥외 살수 금지",
      "세차 금지"
    ],
    "agriculturalActions": [
      "가뭄 저항성 작물 재배",
      "점적 관개 시스템 도입",
      "멀칭을 통한 수분 보존"
    ]
  }
}
```

## API 엔드포인트

### 가뭄 지수

- `GET /api/v1/indices/spi` - SPI 데이터 조회
- `GET /api/v1/indices/pdsi` - PDSI 데이터 조회
- `GET /api/v1/indices/spei` - SPEI 데이터 조회

### 수자원 모니터링

- `GET /api/v1/soil-moisture` - 토양 수분 조회
- `POST /api/v1/soil-moisture` - 토양 수분 제출
- `GET /api/v1/groundwater/wells` - 지하수 관정 목록
- `GET /api/v1/groundwater/wells/{id}/levels` - 지하수위 조회
- `POST /api/v1/groundwater/levels` - 지하수위 제출
- `GET /api/v1/reservoirs` - 저수지 목록
- `GET /api/v1/reservoirs/{id}/storage` - 저수지 저수율 조회
- `POST /api/v1/reservoirs/storage` - 저수지 저수율 제출

### 식생 및 농업

- `GET /api/v1/precipitation/deficit` - 강수량 결핍 분석
- `GET /api/v1/vegetation/stress` - 식생 스트레스 조회
- `GET /api/v1/agriculture/impact` - 농업 영향 조회
- `POST /api/v1/agriculture/impact` - 농업 영향 제출

### 용수 제한 및 선포

- `GET /api/v1/restrictions` - 용수 제한 조회
- `PUT /api/v1/restrictions/{id}` - 용수 제한 업데이트
- `GET /api/v1/declarations` - 가뭄 선포 목록
- `GET /api/v1/declarations/{id}` - 가뭄 선포 상세
- `POST /api/v1/declarations` - 가뭄 선포 등록
- `PUT /api/v1/declarations/{id}` - 가뭄 선포 업데이트

### 통합 모니터링 및 분석

- `GET /api/v1/monitoring/report` - 종합 가뭄 모니터링 보고서
- `GET /api/v1/monitoring/regional` - 지역별 가뭄 현황
- `GET /api/v1/forecast` - 가뭄 예보
- `GET /api/v1/alerts/early-warning` - 조기 경보
- `GET /api/v1/analytics/trends` - 가뭄 추세 분석
- `POST /api/v1/analytics/impact-report` - 영향 보고서 생성

## 주요 성과 지표 (KPI)

### 모니터링 범위

- **모니터링 지점**: 전국 1,000+ 지점
- **실시간 센서**: 토양 수분 500개, 지하수 300개
- **저수지**: 주요 댐 및 저수지 100개
- **위성 영상**: 매일 갱신 (MODIS), 5일 (Sentinel-2)

### 조기 경보

- **예측 리드 타임**: 30-90일
- **정확도**: 75% 이상
- **알림 시간**: 12시간 이내

### 대응 시간

- **가뭄 선포**: 48시간 이내
- **용수 제한 시행**: 24시간 이내
- **긴급 급수**: 6시간 이내

## 사용 예제

### Python

```python
from wia_ene034 import DroughtMonitoringClient

client = DroughtMonitoringClient(
    api_key='your-api-key',
    endpoint='https://api.wia.org/ene-034/v1'
)

# 종합 가뭄 보고서
report = client.get_drought_monitoring_report(37.5665, 126.9780)
print(f'Composite Index: {report.composite_drought_index.value}')
print(f'Severity: {report.composite_drought_index.severity}')

# SPI 데이터
spi = client.get_spi(37.5665, 126.9780)
print(f'SPI-12: {spi[0].spi12_month}')
```

### REST API (cURL)

```bash
# SPI 데이터 조회
curl -X GET "https://api.wia.org/ene-034/v1/indices/spi?latitude=37.5665&longitude=126.9780" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -H "X-WIA-Standard: ENE-034" \
  -H "X-WIA-Version: 1.0.0"

# 종합 보고서 조회
curl -X GET "https://api.wia.org/ene-034/v1/monitoring/report?latitude=37.5665&longitude=126.9780" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json"
```

## 글로벌 현황

### 현재 상황 (2024)

- **모니터링 국가**: 100+ 개국
- **가뭄 영향 인구**: 20억 명
- **연간 경제 손실**: $80억 USD
- **영향 받는 농경지**: 3억 ha

### 2030 목표

- **조기 경보 시스템**: 모든 가뭄 취약 지역
- **리드 타임**: 90일 이상
- **정확도**: 85% 이상
- **경제 손실 감소**: 50%

### 2050 비전

- **기후 적응**: 가뭄 복원력 있는 농업 및 도시
- **물 안보**: 모든 인구의 물 접근성 보장
- **생태계 보호**: 가뭄에도 건강한 생태계 유지

## 기여하기

WIA-ENE-034 표준 개선에 기여해 주세요:

1. **기술 피드백**: GitHub 이슈로 제안사항 제출
2. **사례 연구**: 구현 경험 공유
3. **번역**: 다른 언어로 문서 번역
4. **프로토콜 개선**: 데이터 포맷 또는 API 개선 제안

자세한 내용은 [CONTRIBUTING.md](../CONTRIBUTING.md)를 참조하세요.

## 커뮤니티 및 지원

- **웹사이트**: [wia.org/standards/ene-034](https://wia.org/standards/ene-034)
- **문서**: [docs.wia.org/ene-034](https://docs.wia.org/ene-034)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **이메일**: standards@wia.org

## 라이선스

이 표준은 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

## 인용

```bibtex
@standard{wia-ene-034,
  title = {WIA-ENE-034: Drought Monitoring Standard},
  author = {{World Certification Industry Association}},
  year = {2025},
  version = {1.0},
  url = {https://github.com/WIA-Official/wia-standards/drought-monitoring}
}
```

## 관련 표준

- **WIA-ENE-017**: 기후 변화 관리
- **WIA-ENE-018**: 수질 모니터링
- **WIA-ENE-033**: 생태계 모니터링
- **WIA-ENE-011**: 대기질 모니터링
- **WIA-BLOCKCHAIN**: 가뭄 구호 자금 추적

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 문서 패키지
- TypeScript SDK
- CLI 도구
- API 스펙

---

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

WIA-ENE-034 표준은 홍익인간 (弘益人間)(홍익인간)의 정신을 구현합니다. 과학적 가뭄 모니터링을 통해 물 부족에 대비하고, 농업과 생태계를 보호하며, 인류가 가뭄에 복원력 있는 미래를 만들어갑니다.

**함께, 우리는 가뭄에 강한 사회와 지속가능한 물 관리를 실현합니다.**

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
