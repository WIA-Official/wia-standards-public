# WIA Climate Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-14
**Authors**: WIA (World Industry Authentication Association) / SmileStory Inc.
**License**: MIT

---

## 목차 (Table of Contents)

1. [개요 (Overview)](#1-개요-overview)
2. [용어 정의 (Terminology)](#2-용어-정의-terminology)
3. [기본 구조 (Base Structure)](#3-기본-구조-base-structure)
4. [영역별 데이터 형식 (Domain-Specific Data)](#4-영역별-데이터-형식-domain-specific-data)
5. [확장성 (Extensibility)](#5-확장성-extensibility)
6. [버전 관리 (Versioning)](#6-버전-관리-versioning)
7. [예제 (Examples)](#7-예제-examples)
8. [참고문헌 (References)](#8-참고문헌-references)

---

## 1. 개요 (Overview)

### 1.1 목적 (Purpose)

WIA Climate Data Format Standard는 다양한 기후 및 환경 관련 데이터의 형식을 표준화하기 위한 규격입니다.

**핵심 목표**:
- 모든 기후/환경 데이터가 동일한 JSON 형식으로 교환
- 다양한 시스템 간 상호운용성 제공
- 기존 표준(CF Conventions, OGC)과의 호환성

### 1.2 적용 범위 (Scope)

본 표준은 다음 영역을 포함합니다:

| 영역 | 영문명 | 설명 |
|------|--------|------|
| 탄소 포집 | Carbon Capture | CO2 포집, 저장, 활용 |
| 기상 제어 | Weather Control | 인공 강우, 구름 씨뿌리기 |
| 지구공학 | Geoengineering | 태양복사 관리, 탄소 제거 |
| 수직 농장 | Vertical Farming | 실내 농업, 환경 제어 |
| 해양 정화 | Ocean Cleanup | 해양 플라스틱 수거 |
| 기후 모델링 | Climate Modeling | 기후 시뮬레이션 데이터 |

### 1.3 설계 원칙 (Design Principles)

1. **단순성 (Simplicity)**: JSON 기반의 명확한 구조
2. **확장성 (Extensibility)**: 새로운 영역 추가 용이
3. **상호운용성 (Interoperability)**: 모든 플랫폼에서 파싱 가능
4. **정확성 (Precision)**: 고해상도 타임스탬프와 좌표
5. **검증 가능성 (Validation)**: JSON Schema로 형식 검증

---

## 2. 용어 정의 (Terminology)

### 2.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **Data Point** | 특정 시간/위치에서 측정된 하나의 데이터 |
| **Message** | 표준 형식을 따르는 하나의 JSON 객체 |
| **Stream** | 시간 순서로 정렬된 Message의 연속 |
| **Device** | 물리적 센서 또는 데이터 소스 |
| **Quality Score** | 데이터의 품질 (0.0 ~ 1.0) |
| **Uncertainty** | 측정 불확실성 |

### 2.2 데이터 타입

| 타입 | 설명 | 예시 |
|------|------|------|
| `string` | UTF-8 문자열 | `"carbon_capture"` |
| `number` | 64-bit IEEE 754 부동소수점 | `0.45`, `1702468800000` |
| `integer` | 정수 | `1`, `255` |
| `boolean` | 불리언 | `true`, `false` |
| `null` | 널 값 | `null` |
| `object` | JSON 객체 | `{"lat": 35.5, "lon": 127.0}` |
| `array` | JSON 배열 | `[1, 2, 3]` |

### 2.3 필드 요구사항

| 표기 | 의미 |
|------|------|
| **REQUIRED** | 반드시 포함해야 함 |
| **OPTIONAL** | 선택적으로 포함 가능 |
| **CONDITIONAL** | 특정 조건에서 필수 |

---

## 3. 기본 구조 (Base Structure)

### 3.1 메시지 형식 (Message Format)

모든 WIA Climate Data Message는 다음 기본 구조를 따릅니다:

```json
{
    "$schema": "https://wia.live/climate/data/v1/schema.json",
    "version": "1.0.0",
    "type": "<data_type>",
    "timestamp": {
        "unix_ms": <milliseconds>,
        "iso8601": "<ISO 8601 string>"
    },
    "location": {
        "latitude": <degrees>,
        "longitude": <degrees>,
        "altitude_m": <meters>,
        "crs": "EPSG:4326"
    },
    "device": {
        "manufacturer": "<string>",
        "model": "<string>",
        "serial": "<string>"
    },
    "data": {
        // 영역별 고유 데이터
    },
    "meta": {
        "quality_score": <0.0-1.0>,
        "uncertainty": <number>,
        "source": "<string>"
    }
}
```

### 3.2 필드 상세

#### 3.2.1 `$schema` (OPTIONAL)

```
타입: string
형식: URI
설명: JSON Schema 위치
예시: "https://wia.live/climate/data/v1/schema.json"
```

#### 3.2.2 `version` (REQUIRED)

```
타입: string
형식: Semantic Versioning (MAJOR.MINOR.PATCH)
설명: 스펙 버전
예시: "1.0.0"
```

#### 3.2.3 `type` (REQUIRED)

```
타입: string
설명: 데이터 유형 식별자
유효값:
  - "carbon_capture"     : 탄소 포집 데이터
  - "weather_control"    : 기상 제어 데이터
  - "geoengineering"     : 지구공학 데이터
  - "vertical_farming"   : 수직 농장 데이터
  - "ocean_cleanup"      : 해양 정화 데이터
  - "climate_model"      : 기후 모델 데이터
  - "custom"             : 사용자 정의 (확장용)
```

#### 3.2.4 `timestamp` (REQUIRED)

```
타입: object
설명: 데이터 생성/측정 시간

하위 필드:
  - unix_ms (REQUIRED): number
    설명: UNIX 타임스탬프 (밀리초)
    예시: 1702468800000

  - iso8601 (OPTIONAL): string
    설명: ISO 8601 형식 문자열 (UTC)
    예시: "2024-12-14T12:00:00.000Z"
```

#### 3.2.5 `location` (REQUIRED)

```
타입: object
설명: 지리적 위치 정보

하위 필드:
  - latitude (REQUIRED): number
    설명: 위도 (WGS84)
    범위: -90.0 ~ 90.0
    단위: 도 (degrees)

  - longitude (REQUIRED): number
    설명: 경도 (WGS84)
    범위: -180.0 ~ 180.0
    단위: 도 (degrees)

  - altitude_m (OPTIONAL): number
    설명: 고도 (해수면 기준)
    단위: 미터 (m)

  - crs (OPTIONAL): string
    설명: 좌표 참조 시스템
    기본값: "EPSG:4326"
```

#### 3.2.6 `device` (REQUIRED)

```
타입: object
설명: 데이터 소스/센서 정보

하위 필드:
  - manufacturer (REQUIRED): string
    설명: 제조사명 또는 데이터 제공자
    예시: "Climeworks", "NOAA", "OpenBCI"

  - model (REQUIRED): string
    설명: 모델명 또는 시스템명
    예시: "Orca DAC", "CESM2", "Interceptor"

  - serial (OPTIONAL): string
    설명: 시리얼 번호 또는 식별자
    예시: "DAC-2024-001"

  - firmware (OPTIONAL): string
    설명: 펌웨어/소프트웨어 버전
```

#### 3.2.7 `data` (REQUIRED)

```
타입: object
설명: 영역별 고유 데이터 (섹션 4 참조)
```

#### 3.2.8 `meta` (OPTIONAL)

```
타입: object
설명: 메타데이터

하위 필드:
  - quality_score (OPTIONAL): number
    설명: 데이터 품질 점수
    범위: 0.0 (최저) ~ 1.0 (최고)

  - uncertainty (OPTIONAL): number
    설명: 측정 불확실성 (동일 단위)

  - source (OPTIONAL): string
    설명: 데이터 출처
    예시: "sensor", "model", "manual", "satellite"

  - processing_level (OPTIONAL): string
    설명: 데이터 처리 수준
    예시: "raw", "calibrated", "validated"
```

---

## 4. 영역별 데이터 형식 (Domain-Specific Data)

### 4.1 Carbon Capture (탄소 포집)

`type: "carbon_capture"`

#### data 구조

```json
{
    "data": {
        "technology": "dac",
        "capture_rate_kg_per_hour": 125.5,
        "co2_concentration_ppm": 415,
        "co2_purity_percentage": 99.2,
        "energy_consumption_kwh": 2500,
        "sorbent_status": {
            "type": "solid_amine",
            "efficiency_percentage": 85.5,
            "cycles_completed": 1250
        },
        "storage": {
            "method": "geological",
            "pressure_mpa": 10.5,
            "depth_m": 2000,
            "formation_type": "saline_aquifer"
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 | 단위 |
|------|------|------|------|------|
| `technology` | string | REQUIRED | 포집 기술 | - |
| `capture_rate_kg_per_hour` | number | REQUIRED | 포집 속도 | kg/h |
| `co2_concentration_ppm` | number | OPTIONAL | CO2 농도 | ppm |
| `co2_purity_percentage` | number | OPTIONAL | 포집 CO2 순도 | % |
| `energy_consumption_kwh` | number | OPTIONAL | 에너지 소비 | kWh |
| `sorbent_status.type` | string | OPTIONAL | 흡착제 유형 | - |
| `sorbent_status.efficiency_percentage` | number | OPTIONAL | 흡착 효율 | % |
| `storage.method` | string | CONDITIONAL | 저장 방법 | - |
| `storage.pressure_mpa` | number | CONDITIONAL | 저장 압력 | MPa |
| `storage.depth_m` | number | CONDITIONAL | 저장 깊이 | m |

#### technology 값 정의

| 값 | 설명 |
|----|------|
| `"dac"` | Direct Air Capture |
| `"post_combustion"` | 연소 후 포집 |
| `"pre_combustion"` | 연소 전 포집 |
| `"oxy_fuel"` | 산소 연소 |
| `"bioenergy_ccs"` | BECCS |

---

### 4.2 Weather Control (기상 제어)

`type: "weather_control"`

#### data 구조

```json
{
    "data": {
        "operation_type": "cloud_seeding",
        "seeding_agent": {
            "type": "silver_iodide",
            "mass_grams": 50,
            "concentration_g_per_m3": 0.01
        },
        "delivery_method": "aircraft",
        "target_cloud": {
            "type": "cumulus",
            "base_altitude_m": 2000,
            "top_altitude_m": 5000,
            "coverage_km2": 100
        },
        "atmospheric_conditions": {
            "temperature_celsius": -5,
            "humidity_percentage": 85,
            "wind_speed_m_per_s": 15,
            "wind_direction_deg": 270
        },
        "result": {
            "precipitation_mm": 12.5,
            "duration_hours": 3,
            "effectiveness_percentage": 15
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 | 단위 |
|------|------|------|------|------|
| `operation_type` | string | REQUIRED | 작업 유형 | - |
| `seeding_agent.type` | string | CONDITIONAL | 씨뿌리기 물질 | - |
| `seeding_agent.mass_grams` | number | CONDITIONAL | 투입량 | g |
| `delivery_method` | string | OPTIONAL | 전달 방법 | - |
| `target_cloud.type` | string | OPTIONAL | 구름 유형 | - |
| `target_cloud.base_altitude_m` | number | OPTIONAL | 구름 저면 고도 | m |
| `atmospheric_conditions.temperature_celsius` | number | REQUIRED | 기온 | °C |
| `atmospheric_conditions.humidity_percentage` | number | REQUIRED | 습도 | % |
| `result.precipitation_mm` | number | OPTIONAL | 강수량 | mm |

#### operation_type 값 정의

| 값 | 설명 |
|----|------|
| `"cloud_seeding"` | 구름 씨뿌리기 |
| `"fog_dispersal"` | 안개 제거 |
| `"hail_suppression"` | 우박 억제 |
| `"rain_enhancement"` | 강수 증가 |

#### seeding_agent.type 값 정의

| 값 | 설명 |
|----|------|
| `"silver_iodide"` | 요오드화은 (AgI) |
| `"potassium_iodide"` | 요오드화칼륨 (KI) |
| `"dry_ice"` | 드라이아이스 |
| `"liquid_propane"` | 액화 프로판 |
| `"salt"` | 소금 (해양성 구름) |

---

### 4.3 Geoengineering (지구공학)

`type: "geoengineering"`

#### data 구조

```json
{
    "data": {
        "intervention_type": "stratospheric_aerosol_injection",
        "category": "solar_radiation_management",
        "deployment": {
            "aerosol_type": "sulfur_dioxide",
            "injection_altitude_km": 20,
            "injection_rate_kg_per_day": 100000,
            "particle_size_um": 0.5
        },
        "coverage": {
            "area_km2": 1000000,
            "latitude_range": [-60, 60],
            "longitude_range": [-180, 180]
        },
        "effects": {
            "radiative_forcing_w_per_m2": -1.5,
            "temperature_change_celsius": -0.5,
            "precipitation_change_percentage": -2.0
        },
        "monitoring": {
            "ozone_impact_percentage": -0.5,
            "acid_deposition_increase_percentage": 1.2
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 | 단위 |
|------|------|------|------|------|
| `intervention_type` | string | REQUIRED | 개입 유형 | - |
| `category` | string | REQUIRED | 분류 (SRM/CDR) | - |
| `deployment.aerosol_type` | string | CONDITIONAL | 에어로졸 유형 | - |
| `deployment.injection_altitude_km` | number | CONDITIONAL | 주입 고도 | km |
| `deployment.injection_rate_kg_per_day` | number | CONDITIONAL | 주입 속도 | kg/day |
| `coverage.area_km2` | number | OPTIONAL | 영향 면적 | km² |
| `effects.radiative_forcing_w_per_m2` | number | OPTIONAL | 복사강제력 | W/m² |
| `effects.temperature_change_celsius` | number | OPTIONAL | 온도 변화 | °C |

#### intervention_type 값 정의

| 값 | 설명 |
|----|------|
| `"stratospheric_aerosol_injection"` | 성층권 에어로졸 주입 (SAI) |
| `"marine_cloud_brightening"` | 해양 구름 밝기 증가 (MCB) |
| `"ocean_fertilization"` | 해양 비옥화 |
| `"enhanced_weathering"` | 강화된 풍화작용 |
| `"space_reflector"` | 우주 반사경 |
| `"direct_air_capture"` | 직접 공기 포집 |

#### category 값 정의

| 값 | 설명 |
|----|------|
| `"solar_radiation_management"` | 태양복사 관리 (SRM) |
| `"carbon_dioxide_removal"` | 이산화탄소 제거 (CDR) |

---

### 4.4 Vertical Farming (수직 농장)

`type: "vertical_farming"`

#### data 구조

```json
{
    "data": {
        "system_type": "hydroponics",
        "environment": {
            "temperature_celsius": 22.5,
            "humidity_percentage": 65,
            "co2_ppm": 800,
            "vpd_kpa": 0.85
        },
        "lighting": {
            "type": "led",
            "ppfd_umol_per_m2_s": 450,
            "dli_mol_per_m2_day": 25.9,
            "spectrum": "full_spectrum",
            "photoperiod_hours": 16
        },
        "nutrient_solution": {
            "ph": 6.0,
            "ec_ms_per_cm": 1.8,
            "temperature_celsius": 20,
            "dissolved_oxygen_mg_per_l": 8.0,
            "elements": {
                "nitrogen_ppm": 150,
                "phosphorus_ppm": 50,
                "potassium_ppm": 200,
                "calcium_ppm": 180,
                "magnesium_ppm": 50
            }
        },
        "crop": {
            "species": "lactuca_sativa",
            "variety": "butterhead",
            "growth_stage": "vegetative",
            "days_after_planting": 21,
            "plant_count": 500,
            "density_plants_per_m2": 25
        },
        "yield": {
            "fresh_weight_kg": 42.5,
            "dry_weight_kg": 2.1,
            "area_m2": 5,
            "kg_per_m2": 8.5
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 | 단위 |
|------|------|------|------|------|
| `system_type` | string | REQUIRED | 재배 시스템 | - |
| `environment.temperature_celsius` | number | REQUIRED | 기온 | °C |
| `environment.humidity_percentage` | number | REQUIRED | 습도 | % |
| `environment.co2_ppm` | number | OPTIONAL | CO2 농도 | ppm |
| `environment.vpd_kpa` | number | OPTIONAL | 수증기압차 | kPa |
| `lighting.ppfd_umol_per_m2_s` | number | REQUIRED | 광합성광량자속밀도 | µmol/m²/s |
| `lighting.photoperiod_hours` | number | REQUIRED | 광주기 | h |
| `nutrient_solution.ph` | number | REQUIRED | pH | - |
| `nutrient_solution.ec_ms_per_cm` | number | REQUIRED | 전기전도도 | mS/cm |
| `crop.species` | string | REQUIRED | 학명 | - |
| `crop.growth_stage` | string | OPTIONAL | 생장 단계 | - |
| `yield.kg_per_m2` | number | OPTIONAL | 면적당 수확량 | kg/m² |

#### system_type 값 정의

| 값 | 설명 |
|----|------|
| `"hydroponics"` | 수경재배 |
| `"aeroponics"` | 분무경재배 |
| `"aquaponics"` | 아쿠아포닉스 |
| `"substrate"` | 배지 재배 |

#### growth_stage 값 정의

| 값 | 설명 |
|----|------|
| `"germination"` | 발아 |
| `"seedling"` | 유묘 |
| `"vegetative"` | 영양 생장 |
| `"flowering"` | 개화 |
| `"fruiting"` | 결실 |
| `"harvest"` | 수확 |

---

### 4.5 Ocean Cleanup (해양 정화)

`type: "ocean_cleanup"`

#### data 구조

```json
{
    "data": {
        "operation_type": "floating_barrier",
        "collection": {
            "total_mass_kg": 250.5,
            "plastic_mass_kg": 180.2,
            "fishing_gear_kg": 45.3,
            "other_debris_kg": 25.0,
            "microplastic_count": 150000,
            "microplastic_size_range_mm": [0.1, 5.0]
        },
        "area": {
            "swept_km2": 10.5,
            "duration_hours": 8,
            "efficiency_kg_per_km2": 23.86
        },
        "water_conditions": {
            "temperature_celsius": 18.5,
            "salinity_ppt": 35,
            "ph": 8.1,
            "current_speed_m_per_s": 0.5,
            "wave_height_m": 1.2
        },
        "vessel": {
            "type": "support_vessel",
            "name": "Ocean Warrior",
            "fuel_consumption_l_per_hour": 50
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 | 단위 |
|------|------|------|------|------|
| `operation_type` | string | REQUIRED | 작업 유형 | - |
| `collection.total_mass_kg` | number | REQUIRED | 총 수거량 | kg |
| `collection.plastic_mass_kg` | number | OPTIONAL | 플라스틱 질량 | kg |
| `collection.microplastic_count` | integer | OPTIONAL | 미세플라스틱 개수 | - |
| `area.swept_km2` | number | REQUIRED | 작업 면적 | km² |
| `area.duration_hours` | number | REQUIRED | 작업 시간 | h |
| `water_conditions.temperature_celsius` | number | OPTIONAL | 수온 | °C |
| `water_conditions.salinity_ppt` | number | OPTIONAL | 염분 | ppt |

#### operation_type 값 정의

| 값 | 설명 |
|----|------|
| `"floating_barrier"` | 부유식 방벽 |
| `"river_interceptor"` | 하천 인터셉터 |
| `"beach_cleanup"` | 해변 청소 |
| `"autonomous_vessel"` | 자율 청소선 |
| `"drone_collection"` | 드론 수거 |

---

### 4.6 Climate Model (기후 모델)

`type: "climate_model"`

#### data 구조

```json
{
    "data": {
        "model": {
            "source_id": "CESM2",
            "institution_id": "NCAR",
            "experiment_id": "ssp245",
            "variant_label": "r1i1p1f1",
            "grid_label": "gn"
        },
        "variable": {
            "name": "tas",
            "long_name": "Near-Surface Air Temperature",
            "standard_name": "air_temperature",
            "units": "K",
            "cell_methods": "time: mean"
        },
        "grid": {
            "resolution_deg": 1.0,
            "resolution_km": 100,
            "nlat": 180,
            "nlon": 360
        },
        "time": {
            "start_date": "2015-01-01",
            "end_date": "2100-12-31",
            "frequency": "mon",
            "calendar": "gregorian"
        },
        "value": {
            "data": 288.5,
            "anomaly": 1.2,
            "climatology": 287.3,
            "percentile_5": 285.0,
            "percentile_95": 292.0
        },
        "scenario": {
            "ssp": "SSP2-4.5",
            "forcing_level_w_per_m2": 4.5,
            "description": "Middle of the road"
        }
    }
}
```

#### 필드 상세

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `model.source_id` | string | REQUIRED | 모델 ID (CMIP6) |
| `model.experiment_id` | string | REQUIRED | 실험 ID |
| `model.variant_label` | string | REQUIRED | 앙상블 멤버 |
| `variable.name` | string | REQUIRED | 변수명 |
| `variable.units` | string | REQUIRED | 단위 |
| `grid.resolution_deg` | number | OPTIONAL | 격자 해상도 (도) |
| `time.frequency` | string | REQUIRED | 시간 빈도 |
| `value.data` | number | REQUIRED | 측정/모델 값 |
| `scenario.ssp` | string | CONDITIONAL | SSP 시나리오 |

#### frequency 값 정의 (CF Conventions)

| 값 | 설명 |
|----|------|
| `"fx"` | 고정값 |
| `"yr"` | 연간 |
| `"mon"` | 월간 |
| `"day"` | 일간 |
| `"6hr"` | 6시간 |
| `"3hr"` | 3시간 |
| `"1hr"` | 1시간 |

#### 주요 변수 (CMIP6 Data Request)

| 변수명 | 설명 | 단위 |
|--------|------|------|
| `tas` | 지표면 기온 | K |
| `pr` | 강수량 | kg m-2 s-1 |
| `psl` | 해면 기압 | Pa |
| `uas`, `vas` | 지표면 풍속 | m s-1 |
| `hurs` | 상대습도 | % |
| `rsds` | 지표면 하향 단파복사 | W m-2 |
| `co2` | CO2 농도 | ppm |

---

## 5. 확장성 (Extensibility)

### 5.1 사용자 정의 데이터 (Custom Data)

`type: "custom"` 을 사용하여 표준에 정의되지 않은 데이터를 지원합니다:

```json
{
    "version": "1.0.0",
    "type": "custom",
    "timestamp": { "unix_ms": 1702468800000 },
    "location": { "latitude": 35.5, "longitude": 127.0 },
    "device": { "manufacturer": "CustomCorp", "model": "MySensor" },
    "data": {
        "custom_type": "soil_carbon",
        "custom_data": {
            "soil_organic_carbon_percentage": 2.5,
            "depth_cm": 30,
            "bulk_density_g_per_cm3": 1.2
        }
    }
}
```

### 5.2 필드 확장

영역별 `data` 객체 내에 추가 필드를 포함할 수 있습니다. 표준에 정의되지 않은 필드는 `x_` 접두사를 권장합니다:

```json
{
    "data": {
        "capture_rate_kg_per_hour": 125.5,
        "x_proprietary_metric": 0.85,
        "x_calibration_date": "2024-12-01"
    }
}
```

### 5.3 하위 호환성 (Backward Compatibility)

- MAJOR 버전 변경 시에만 필드 삭제 가능
- MINOR 버전에서는 새 필드 추가만 가능
- 파서는 알 수 없는 필드를 무시해야 함 (MUST ignore)

---

## 6. 버전 관리 (Versioning)

### 6.1 버전 형식

Semantic Versioning 2.0.0을 따릅니다:

```
MAJOR.MINOR.PATCH

예: 1.2.3
    │ │ └─ PATCH: 버그 수정, 문서 개선
    │ └─── MINOR: 하위 호환 가능한 기능 추가
    └───── MAJOR: 호환되지 않는 변경
```

### 6.2 버전 협상

클라이언트와 서버 간 버전 협상:

```json
{
    "supported_versions": ["1.0.0", "1.1.0", "1.2.0"],
    "selected_version": "1.2.0"
}
```

---

## 7. 예제 (Examples)

### 7.1 Carbon Capture 전체 예제

```json
{
    "$schema": "https://wia.live/climate/data/v1/schema.json",
    "version": "1.0.0",
    "type": "carbon_capture",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
        "latitude": 64.0,
        "longitude": -21.0,
        "altitude_m": 100,
        "crs": "EPSG:4326"
    },
    "device": {
        "manufacturer": "Climeworks",
        "model": "Orca DAC",
        "serial": "ORCA-2024-001",
        "firmware": "2.1.0"
    },
    "data": {
        "technology": "dac",
        "capture_rate_kg_per_hour": 125.5,
        "co2_concentration_ppm": 415,
        "co2_purity_percentage": 99.2,
        "energy_consumption_kwh": 2500,
        "sorbent_status": {
            "type": "solid_amine",
            "efficiency_percentage": 85.5,
            "cycles_completed": 1250
        },
        "storage": {
            "method": "geological",
            "pressure_mpa": 10.5,
            "depth_m": 2000,
            "formation_type": "basalt"
        }
    },
    "meta": {
        "quality_score": 0.98,
        "uncertainty": 0.02,
        "source": "sensor",
        "processing_level": "validated"
    }
}
```

### 7.2 Vertical Farming 전체 예제

```json
{
    "version": "1.0.0",
    "type": "vertical_farming",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "altitude_m": 50
    },
    "device": {
        "manufacturer": "SmartFarm Inc",
        "model": "VerticalPro 5000"
    },
    "data": {
        "system_type": "hydroponics",
        "environment": {
            "temperature_celsius": 22.5,
            "humidity_percentage": 65,
            "co2_ppm": 800,
            "vpd_kpa": 0.85
        },
        "lighting": {
            "type": "led",
            "ppfd_umol_per_m2_s": 450,
            "dli_mol_per_m2_day": 25.9,
            "spectrum": "full_spectrum",
            "photoperiod_hours": 16
        },
        "nutrient_solution": {
            "ph": 6.0,
            "ec_ms_per_cm": 1.8,
            "temperature_celsius": 20,
            "elements": {
                "nitrogen_ppm": 150,
                "phosphorus_ppm": 50,
                "potassium_ppm": 200
            }
        },
        "crop": {
            "species": "lactuca_sativa",
            "variety": "butterhead",
            "growth_stage": "vegetative",
            "days_after_planting": 21
        },
        "yield": {
            "kg_per_m2": 8.5
        }
    },
    "meta": {
        "quality_score": 0.95,
        "source": "sensor"
    }
}
```

### 7.3 Climate Model 전체 예제

```json
{
    "version": "1.0.0",
    "type": "climate_model",
    "timestamp": {
        "unix_ms": 1702468800000,
        "iso8601": "2024-12-14T12:00:00.000Z"
    },
    "location": {
        "latitude": 35.5,
        "longitude": 127.0
    },
    "device": {
        "manufacturer": "NCAR",
        "model": "CESM2"
    },
    "data": {
        "model": {
            "source_id": "CESM2",
            "institution_id": "NCAR",
            "experiment_id": "ssp245",
            "variant_label": "r1i1p1f1",
            "grid_label": "gn"
        },
        "variable": {
            "name": "tas",
            "long_name": "Near-Surface Air Temperature",
            "standard_name": "air_temperature",
            "units": "K"
        },
        "time": {
            "start_date": "2050-01-01",
            "end_date": "2050-12-31",
            "frequency": "mon"
        },
        "value": {
            "data": 289.5,
            "anomaly": 2.2
        },
        "scenario": {
            "ssp": "SSP2-4.5"
        }
    },
    "meta": {
        "source": "model",
        "processing_level": "validated"
    }
}
```

---

## 8. 참고문헌 (References)

### 표준 문서

- [JSON (ECMA-404)](https://www.ecma-international.org/publications-and-standards/standards/ecma-404/)
- [JSON Schema draft-07](https://json-schema.org/specification-links.html#draft-7)
- [Semantic Versioning 2.0.0](https://semver.org/)
- [ISO 8601 Date/Time Format](https://www.iso.org/iso-8601-date-and-time-format.html)
- [CF Conventions](https://cfconventions.org/)
- [CMIP6 Data Request](https://pcmdi.llnl.gov/CMIP6/Guide/)
- [OGC SensorThings API](https://www.ogc.org/standards/sensorthings)

### 관련 표준

- [ISO 27914:2017 - Carbon dioxide capture](https://www.iso.org/standard/64148.html)
- [ISO 19115 - Geographic information Metadata](https://www.iso.org/standard/53798.html)
- [GHG Protocol](https://ghgprotocol.org/)
- [WMO Codes](https://library.wmo.int/index.php?lvl=notice_display&id=10684)

---

## 부록 A: JSON Schema 파일 목록

| 파일명 | 설명 |
|--------|------|
| `wia-climate-data-v1.schema.json` | 기본 스키마 |
| `carbon-capture.schema.json` | Carbon Capture data 스키마 |
| `weather-control.schema.json` | Weather Control data 스키마 |
| `geoengineering.schema.json` | Geoengineering data 스키마 |
| `vertical-farming.schema.json` | Vertical Farming data 스키마 |
| `ocean-cleanup.schema.json` | Ocean Cleanup data 스키마 |
| `climate-model.schema.json` | Climate Model data 스키마 |

---

## 부록 B: 단위 규약

### B.1 SI 기본 단위

| 물리량 | 단위 | 기호 |
|--------|------|------|
| 길이 | 미터 | m |
| 질량 | 킬로그램 | kg |
| 시간 | 초 | s |
| 온도 | 켈빈 | K |
| 물질량 | 몰 | mol |

### B.2 파생 단위

| 물리량 | 단위 | 표기 |
|--------|------|------|
| 면적 | 제곱킬로미터 | km² |
| 부피 | 세제곱미터 | m³ |
| 속도 | 미터/초 | m/s |
| 압력 | 메가파스칼 | MPa |
| 에너지 | 킬로와트시 | kWh |
| 복사강제력 | 와트/제곱미터 | W/m² |

### B.3 농도 단위

| 물리량 | 단위 | 표기 |
|--------|------|------|
| 기체 농도 | ppm | ppm |
| 염분 | ‰ (천분율) | ppt |
| 전기전도도 | mS/cm | mS/cm |

---

<div align="center">

**WIA Climate Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

🌍

---

**© 2025 SmileStory Inc. / WIA**

**MIT License**

</div>
