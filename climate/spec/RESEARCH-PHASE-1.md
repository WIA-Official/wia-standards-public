# Phase 1 사전 조사 결과
# Phase 1 Research Findings

---

**작성일**: 2025년 12월 14일
**작성**: Claude Code (Opus 4.5)
**목적**: Climate & Environment 기술 조사 및 데이터 형식 표준화 방향 도출

---

## 목차 (Table of Contents)

1. [Carbon Capture (탄소 포집)](#1-carbon-capture-탄소-포집)
2. [Weather Control (기상 제어)](#2-weather-control-기상-제어)
3. [Geoengineering (지구공학)](#3-geoengineering-지구공학)
4. [Vertical Farming (수직 농장)](#4-vertical-farming-수직-농장)
5. [Ocean Cleanup (해양 정화)](#5-ocean-cleanup-해양-정화)
6. [Climate Modeling (기후 모델링)](#6-climate-modeling-기후-모델링)
7. [기존 표준 조사](#7-기존-표준-조사)
8. [공통점 분석](#8-공통점-분석)
9. [결론 및 설계 방향](#9-결론-및-설계-방향)

---

## 1. Carbon Capture (탄소 포집)

### 1.1 기술 개요

Carbon Capture, Utilization, and Storage (CCUS)는 산업 시설이나 대기에서 CO2를 포집하여 저장하거나 활용하는 기술입니다.

**주요 기술 분류**:
- **Post-combustion**: 연소 후 배기가스에서 CO2 분리
- **Pre-combustion**: 연소 전 연료에서 CO2 분리
- **Oxy-fuel combustion**: 순수 산소 사용 연소로 고농도 CO2 생성
- **Direct Air Capture (DAC)**: 대기 중 CO2 직접 포집

### 1.2 산업 현황 (2025)

| 지표 | 수치 |
|------|------|
| 글로벌 CCUS 운영 용량 | 50+ Mt CO2/년 |
| DAC 운영 시설 | 93개 (2030년 예상) |
| DAC 용량 | 6.4-11.4 Mt CO2/년 (2030년) |
| 투자액 | $6.4 billion (2024) |
| 파이프라인 프로젝트 | 628개 |

### 1.3 주요 기업

| 기업 | 기술 | 특징 |
|------|------|------|
| **Climeworks** | DAC | 아이슬란드 Orca 플랜트 (36 kt/년) |
| **Carbon Engineering** | DAC | 미국 대규모 플랜트 (500 kt/년) |
| **Global Thermostat** | DAC | 저비용 DAC 기술 |
| **Equinor** | CCS | 노르웨이 Northern Lights 프로젝트 |
| **Shell** | CCS | Quest CCS (캐나다) |

### 1.4 데이터 요소

```json
{
    "capture_rate_kg_per_hour": 1000,
    "co2_concentration_ppm": 400,
    "energy_consumption_kwh": 2500,
    "storage_pressure_mpa": 10.5,
    "injection_depth_m": 2000,
    "purity_percentage": 99.5,
    "temperature_celsius": 25.0,
    "flow_rate_m3_per_hour": 500
}
```

### 1.5 관련 표준/프로토콜

- **ISO 27914:2017**: Carbon dioxide capture, transportation and geological storage
- **IPCC Guidelines**: National Greenhouse Gas Inventories
- **45Q Tax Credit** (미국): $85/ton (산업), $180/ton (DAC)
- **EU ETS**: Emission Trading System

---

## 2. Weather Control (기상 제어)

### 2.1 기술 개요

Weather Modification은 인위적으로 기상 현상을 변경하는 기술로, 주로 강수 증가나 우박 억제에 사용됩니다.

**주요 기술**:
- **Cloud Seeding**: 구름에 응결핵(Silver Iodide, AgI) 살포
- **Fog Dispersal**: 안개 제거 (공항 등)
- **Hail Suppression**: 우박 피해 감소
- **Rain Enhancement**: 강수량 증가

### 2.2 산업 현황 (2025)

| 국가/지역 | 현황 |
|----------|------|
| 미국 | 9개 주 운영, 10개 주 금지/검토 중 |
| 중국 | 세계 최대 규모 프로그램 |
| UAE | 드론 기반 Cloud Seeding |
| 유타 | 세계 최대 원격 제어 프로그램 |

**규제 동향**:
- **플로리다** (2025): Solar geoengineering 및 weather modification 금지법 통과
- **테네시** (2024): Solar geoengineering 금지법 통과

### 2.3 주요 기업/기관

| 기관 | 역할 |
|------|------|
| **Weather Modification International** | Cloud seeding 장비/서비스 |
| **North American Weather Consultants** | 컨설팅/운영 |
| **NOAA** | 모니터링/보고 수집 |
| **WMO** | 국제 표준/가이드라인 |

### 2.4 데이터 요소

```json
{
    "seeding_agent": "silver_iodide",
    "agent_mass_grams": 50,
    "dispersion_altitude_m": 3000,
    "cloud_type": "cumulus",
    "cloud_base_m": 2000,
    "cloud_top_m": 5000,
    "temperature_celsius": -5,
    "humidity_percentage": 85,
    "wind_speed_m_per_s": 15,
    "precipitation_mm": 12.5
}
```

### 2.5 관련 표준/프로토콜

- **NOAA Weather Modification Reporting Act**: 미국 연방 보고 요건
- **WMO Guidelines**: 기상 수정 프로그램 가이드라인
- **ENMOD Convention**: 환경 수정 기술 군사적 사용 금지 조약

---

## 3. Geoengineering (지구공학)

### 3.1 기술 개요

Geoengineering은 지구 기후 시스템에 대규모로 개입하는 기술로, 크게 두 가지로 분류됩니다:

**Solar Radiation Management (SRM)**:
- **Stratospheric Aerosol Injection (SAI)**: 성층권에 에어로졸 주입
- **Marine Cloud Brightening (MCB)**: 해양 구름 반사율 증가
- **Space-based Reflectors**: 우주 반사경

**Carbon Dioxide Removal (CDR)**:
- Direct Air Capture (DAC) - 섹션 1 참조
- Ocean Fertilization
- Biochar
- Enhanced Weathering

### 3.2 산업 현황 (2025)

| 지표 | 수치 |
|------|------|
| SRM 연구 누적 투자 | ~$200 million (2024년까지) |
| 2025년 연간 투자 | $30+ million |
| 2025-2029 약정 투자 | $164 million |
| UK 투자 | £60+ million (2025) |

**규제 동향**:
- 16+ 미국 주에서 SRM 금지 법안 발의 (2025년 1분기)
- EU 연구 모라토리엄 검토 중
- EPA: Make Sunsets 등 스타트업 조사

### 3.3 거버넌스 프레임워크

| 프레임워크 | 설명 |
|-----------|------|
| **Oxford Principles** | SRM 연구 윤리 원칙 |
| **AGU Ethical Framework** | 기후 개입 연구 윤리 |
| **ENMOD Convention** | 환경 수정 기술 국제 협약 |

### 3.4 데이터 요소

```json
{
    "intervention_type": "stratospheric_aerosol_injection",
    "aerosol_type": "sulfur_dioxide",
    "injection_altitude_km": 20,
    "injection_rate_kg_per_day": 100000,
    "coverage_area_km2": 1000000,
    "radiative_forcing_w_per_m2": -1.5,
    "temperature_change_celsius": -0.5,
    "precipitation_change_percentage": -2.0
}
```

---

## 4. Vertical Farming (수직 농장)

### 4.1 기술 개요

Vertical Farming은 수직으로 쌓인 층에서 작물을 재배하는 Controlled Environment Agriculture (CEA) 기술입니다.

**핵심 기술**:
- **Hydroponics**: 수경재배 (토양 없이 영양액)
- **Aeroponics**: 분무경재배 (뿌리에 영양액 분무)
- **Aquaponics**: 물고기+식물 통합 시스템
- **LED Lighting**: 최적화된 광원 제어
- **HVAC Systems**: 온습도 제어

### 4.2 산업 현황 (2025)

| 지표 | 수치 |
|------|------|
| 시장 규모 | $7.3 billion (2025 예상) |
| 주요 시장 | 미국, 싱가포르, 네덜란드, 독일, 핀란드 |

**주요 과제**:
- 높은 초기 투자 비용
- 에너지 비용
- 표준화 부재
- 데이터 공유 부족

### 4.3 주요 기업

| 기업 | 국가 | 특징 |
|------|------|------|
| **AeroFarms** | 미국 | 대규모 수직 농장 |
| **Plenty** | 미국 | AI 기반 자동화 |
| **Bowery Farming** | 미국 | 도심 농업 |
| **Infarm** | 독일 | 분산형 모듈 농장 |
| **AppHarvest** | 미국 | 대규모 온실 |

### 4.4 데이터 요소

```json
{
    "environment": {
        "temperature_celsius": 22.5,
        "humidity_percentage": 65,
        "co2_ppm": 800,
        "light_intensity_ppfd": 450,
        "light_spectrum": "full_spectrum_led",
        "photoperiod_hours": 16
    },
    "nutrient": {
        "ph": 6.0,
        "ec_ms_per_cm": 1.8,
        "nitrogen_ppm": 150,
        "phosphorus_ppm": 50,
        "potassium_ppm": 200
    },
    "crop": {
        "species": "lettuce",
        "variety": "butterhead",
        "growth_stage": "vegetative",
        "days_since_planting": 21,
        "yield_kg_per_m2": 8.5
    }
}
```

### 4.5 관련 표준

- **UL 8800**: Horticultural Lighting Equipment
- **UL 508A**: Industrial Control Panels
- **ASHRAE Guidelines**: CEA HVAC
- **USDA Organic**: 유기농 인증

---

## 5. Ocean Cleanup (해양 정화)

### 5.1 기술 개요

Ocean Cleanup은 해양 플라스틱 및 오염물질을 제거하는 기술입니다.

**주요 접근법**:
- **Passive Systems**: 해류를 이용한 수동 수거
- **Active Systems**: 동력을 사용한 능동 수거
- **River Interception**: 하천에서 유입 차단
- **Microplastic Removal**: 미세플라스틱 제거

### 5.2 산업 현황 (2025)

| 지표 | 수치 |
|------|------|
| 해양 플라스틱 총량 | 75-199 million tonnes |
| 연간 유입량 | 11 million tonnes |
| The Ocean Cleanup 수거량 | 30,000+ tonnes (누적) |
| 태평양 쓰레기 지대 청소 비용 (예상) | $7.5 billion |

### 5.3 주요 기업/기관

| 기관 | 기술 |
|------|------|
| **The Ocean Cleanup** | 부유식 수거 시스템, 하천 인터셉터 |
| **4ocean** | 브레이슬릿 수익 기반 수거 |
| **Seabin Project** | 항만용 쓰레기통 |
| **Clear Blue Sea** | 자율 청소 로봇 |

### 5.4 데이터 요소

```json
{
    "collection": {
        "plastic_kg": 250.5,
        "fishing_nets_kg": 45.2,
        "microplastic_count": 150000,
        "collection_area_km2": 10.5,
        "duration_hours": 8
    },
    "location": {
        "latitude": 35.5,
        "longitude": -145.2,
        "depth_m": 0,
        "zone": "great_pacific_garbage_patch"
    },
    "water_quality": {
        "temperature_celsius": 18.5,
        "salinity_ppt": 35,
        "ph": 8.1,
        "dissolved_oxygen_mg_per_l": 6.5
    }
}
```

### 5.5 관련 표준/협약

- **UN Global Plastic Treaty**: 2025년 최종 협상 예정
- **MARPOL Annex V**: 해양 오염 방지 협약
- **EU Single-Use Plastics Directive**: 일회용 플라스틱 규제

---

## 6. Climate Modeling (기후 모델링)

### 6.1 기술 개요

Climate Modeling은 지구 기후 시스템을 시뮬레이션하는 컴퓨터 모델입니다.

**주요 모델 유형**:
- **GCM (General Circulation Model)**: 대기/해양 순환 모델
- **ESM (Earth System Model)**: 탄소 순환 포함
- **RCM (Regional Climate Model)**: 지역 고해상도 모델

### 6.2 CMIP6 (Coupled Model Intercomparison Project Phase 6)

CMIP6는 기후 모델 비교를 위한 국제 프로젝트로, IPCC 6차 평가보고서의 기반이 됩니다.

| 지표 | 수치 |
|------|------|
| 참여 모델 | 134개 |
| 모델링 센터 | 53개 |
| 시나리오 | SSP1-2.6 ~ SSP5-8.5 |
| 해상도 | 100km ~ 1km (다운스케일링) |

### 6.3 데이터 형식 표준

**CMIP6 데이터 표준**:
- **파일 형식**: NetCDF (Network Common Data Form)
- **메타데이터**: CF Conventions (Climate and Forecast)
- **변수 명명**: CMIP6 Data Request
- **인프라**: ESGF (Earth System Grid Federation)

```
CMIP6 파일 구조:
├── 파일명 규칙: variable_table_model_experiment_ensemble_grid_timerange.nc
├── 전역 속성: source_id, experiment_id, variant_label, further_info_url
├── 변수 속성: units, long_name, standard_name
└── 좌표 변수: time, lat, lon, plev (pressure level)
```

### 6.4 데이터 요소

```json
{
    "model": {
        "source_id": "CESM2",
        "experiment_id": "ssp245",
        "variant_label": "r1i1p1f1",
        "grid_label": "gn"
    },
    "variable": {
        "name": "tas",
        "long_name": "Near-Surface Air Temperature",
        "units": "K",
        "standard_name": "air_temperature"
    },
    "grid": {
        "latitude_deg": 35.5,
        "longitude_deg": 127.0,
        "resolution_km": 100
    },
    "time": {
        "start_year": 2015,
        "end_year": 2100,
        "frequency": "monthly"
    }
}
```

### 6.5 주요 도구/라이브러리

| 도구 | 용도 |
|------|------|
| **CMOR** | Climate Model Output Rewriter |
| **CDO** | Climate Data Operators |
| **NCO** | NetCDF Operators |
| **xarray** | Python NetCDF 분석 |
| **Iris** | Python 기후 데이터 분석 |

---

## 7. 기존 표준 조사

### 7.1 기후 데이터 표준

| 표준 | 설명 | 적용 분야 |
|------|------|----------|
| **CF Conventions** | NetCDF 기후 데이터 메타데이터 | 기후 모델링 |
| **CMIP6 Data Request** | 변수/실험 정의 | 기후 시뮬레이션 |
| **WMO BUFR/GRIB** | 기상 데이터 인코딩 | 기상 관측/예보 |
| **ISO 19115** | 지리공간 메타데이터 | GIS/지구과학 |

### 7.2 환경 모니터링 표준

| 표준 | 설명 |
|------|------|
| **EPA Methods** | 대기/수질 측정 방법 |
| **ISO 14001** | 환경 관리 시스템 |
| **GHG Protocol** | 온실가스 배출 산정 |
| **CDP** | 탄소 공개 프로젝트 |

### 7.3 데이터 교환 프로토콜

| 프로토콜 | 설명 |
|----------|------|
| **OGC SensorThings API** | IoT 센서 데이터 교환 |
| **MQTT** | 경량 메시징 프로토콜 |
| **OPC UA** | 산업 자동화 통신 |
| **Modbus** | 산업 장비 통신 |

---

## 8. 공통점 분석

### 8.1 모든 영역에 공통으로 필요한 필드

| 필드 | 설명 | 필수 여부 |
|------|------|----------|
| `type` | 데이터 유형 | 필수 |
| `timestamp` | 타임스탬프 | 필수 |
| `location` | 위치 정보 | 필수 |
| `device` | 장치/센서 정보 | 필수 |
| `data` | 영역별 고유 데이터 | 필수 |
| `quality` | 데이터 품질 지표 | 선택 (권장) |
| `units` | 측정 단위 | 필수 |

### 8.2 타임스탬프 형식

| 표준 | 형식 | 사용처 |
|------|------|--------|
| ISO 8601 | 문자열 | JSON 직렬화 |
| UNIX timestamp | 정수 (ms) | 고속 스트리밍 |
| Julian Day | 부동소수점 | 기후 모델 |

**권장**: ISO 8601 문자열 + UNIX timestamp (ms) 병행

### 8.3 지리 좌표계

| 시스템 | 설명 |
|--------|------|
| WGS84 | GPS 표준 좌표계 |
| EPSG:4326 | WGS84 위경도 |
| CF Conventions | 기후 모델 좌표 |

### 8.4 영역별 고유 필드

| 영역 | 고유 필드 |
|------|----------|
| Carbon Capture | capture_rate, co2_concentration, storage_pressure |
| Weather Control | seeding_agent, cloud_type, precipitation |
| Geoengineering | intervention_type, radiative_forcing, coverage_area |
| Vertical Farming | environment, nutrient, crop_data |
| Ocean Cleanup | plastic_mass, collection_area, water_quality |
| Climate Modeling | model_id, experiment, variable, grid |

---

## 9. 결론 및 설계 방향

### 9.1 표준 형식 설계 원칙

1. **확장성 (Extensibility)**
   - 6개 주요 영역 + 커스텀 유형 지원
   - 새로운 기술/센서 추가 용이

2. **상호운용성 (Interoperability)**
   - JSON 기반 (모든 플랫폼 지원)
   - 기존 표준(CF Conventions, OGC)과 호환

3. **정확성 (Precision)**
   - 고해상도 타임스탬프
   - 명확한 단위 표기
   - 데이터 품질 지표 포함

4. **검증 가능성 (Validation)**
   - JSON Schema로 형식 검증
   - 필수/선택 필드 명확히 구분

### 9.2 권장 기본 구조

```json
{
    "$schema": "https://wia.live/climate/data/v1/schema.json",
    "version": "1.0.0",
    "type": "<data_type>",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
        "latitude": 35.5,
        "longitude": 127.0,
        "altitude_m": 100,
        "crs": "EPSG:4326"
    },
    "device": {
        "manufacturer": "string",
        "model": "string",
        "serial": "string"
    },
    "data": {
        // 영역별 고유 데이터
    },
    "meta": {
        "quality_score": 0.95,
        "uncertainty": 0.05,
        "source": "sensor"
    }
}
```

### 9.3 다음 단계

1. **PHASE-1-DATA-FORMAT.md** 작성
   - 위 조사 결과를 바탕으로 정식 스펙 문서 작성

2. **JSON Schema 생성**
   - 기본 스키마 + 영역별 스키마 6개

3. **예제 데이터 생성**
   - 각 영역별 실제 사용 시나리오 반영

4. **검증 스크립트**
   - TypeScript 및 Python으로 스키마 검증 도구 구현

---

## 참고 문헌 (References)

### 공식 문서/기관

- [IEA - Carbon Capture, Utilisation and Storage](https://www.iea.org/energy-system/carbon-capture-utilisation-and-storage)
- [IEA - Direct Air Capture](https://www.iea.org/energy-system/carbon-capture-utilisation-and-storage/direct-air-capture)
- [U.S. GAO - Cloud Seeding Technology Assessment](https://www.gao.gov/products/gao-25-107328)
- [NOAA - Weather Modification Project Reports](https://library.noaa.gov/weather-climate/weather-modification-project-reports)
- [EPA - Geoengineering Government Action](https://www.epa.gov/geoengineering/government-action)
- [UN Scientific Advisory Board - Solar Radiation Modification](https://www.un.org/scientific-advisory-board)
- [WCRP - CMIP6 Guide](https://www.wcrp-climate.org/wgcm-cmip)
- [The Ocean Cleanup](https://theoceancleanup.com/)
- [USDA - Vertical Farming for the Future](https://www.usda.gov/about-usda/news/blog/vertical-farming-future)

### 학술 자료

- [MDPI - Advancements in CCUS Technologies](https://www.mdpi.com/2571-8797/7/4/109)
- [Wiley - Sociotechnical Review of CCUS](https://onlinelibrary.wiley.com/doi/10.1155/ijce/7195300)
- [Taylor & Francis - Future of Vertical Farming](https://www.tandfonline.com/doi/full/10.1080/14620316.2025.2513702)
- [Nature - CMIP6 High-Resolution Dataset](https://www.nature.com/articles/s41597-025-05987-6)
- [ACS EST - Plastic Cleanup Technologies Regulation](https://pubs.acs.org/doi/10.1021/acs.est.3c01885)

### 데이터 포털

- [Copernicus Climate Data Store - CMIP6](https://cds.climate.copernicus.eu/datasets/projections-cmip6)
- [PCMDI - CMIP6 Guidance](https://pcmdi.llnl.gov/CMIP6/Guide/dataUsers.html)
- [ESGF - Earth System Grid Federation](https://esgf.llnl.gov/)

---

<div align="center">

**Phase 1 Research Complete**

🌍

</div>
