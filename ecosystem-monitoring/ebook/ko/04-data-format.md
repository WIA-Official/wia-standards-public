# 제4장: 데이터 형식 사양

## 학습 목표

이 장을 마친 후 다음을 할 수 있게 됩니다:

1. 관찰 및 측정을 위한 WIA JSON 스키마 구현
2. 데이터 품질을 보장하기 위한 검증 규칙 적용
3. 범주형 데이터에 제어 어휘 사용
4. WIA 형식과 호환되는 데이터베이스 설계
5. 기존 데이터를 WIA 형식으로 변환

---

## 4.1 기본 스키마 구조

### 4.1.1 공통 필드

모든 WIA 생태계 모니터링 레코드는 공통 기본 스키마를 공유합니다:

```json
{
  "wia_version": "1.0",
  "schema_type": "string (필수)",
  "record_id": "UUID 또는 고유 식별자 (필수)",
  "timestamp": "ISO 8601 날짜/시간 (필수)",

  "location": {
    "latitude": "number (필수)",
    "longitude": "number (필수)",
    "elevation": "number (선택)",
    "datum": "string (기본값: WGS84)",
    "precision": "number, 미터 단위 (선택)",
    "location_name": "string (선택)"
  },

  "observer": {
    "id": "string (필수)",
    "name": "string (선택)",
    "organization": "string (선택)",
    "email": "string (선택)"
  },

  "quality": {
    "validation_status": "enum (필수)",
    "quality_flags": "array of strings (선택)",
    "confidence_level": "number 0-1 (선택)"
  }
}
```

### 4.1.2 필수 필드 vs. 선택적 필드

**필수 필드** (존재하고 null이 아니어야 함):
- `wia_version`: 표준 버전 ("1.0")
- `schema_type`: 레코드 유형
- `record_id`: 전역 고유 식별자
- `timestamp`: 시간대가 있는 ISO 8601 날짜/시간
- `location.latitude`: -90 ~ 90 십진도
- `location.longitude`: -180 ~ 180 십진도
- `observer.id`: 고유 관찰자/센서 식별자
- `quality.validation_status`: 제어 어휘 값 중 하나

**선택적 필드** (생략하거나 null일 수 있음):
- `location.elevation`: 해발 미터
- `location.datum`: 기본값 WGS84
- `location.precision`: 미터 단위 위치 정확도
- `observer.name`, `observer.organization`, `observer.email`
- `quality.confidence_level`: 0.0 ~ 1.0

### 4.1.3 검증 상태 값

```typescript
enum ValidationStatus {
  unvalidated = "새로 제출됨, 아직 검토되지 않음",
  in_review = "전문가 검토 중",
  validated = "자동 검사 및 기본 검토 통과",
  expert_verified = "분류학 전문가 또는 보정된 센서에 의해 확인됨",
  questionable = "플래그 발생, 주의하여 사용",
  invalid = "검증 실패, 분석에서 제외"
}
```

---

## 4.2 종 관찰 스키마

### 4.2.1 완전한 스키마

```json
{
  "wia_version": "1.0",
  "schema_type": "species-observation",
  "record_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-06-15T14:30:00-07:00",

  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "elevation": 15.5,
    "datum": "WGS84",
    "precision": 10,
    "location_name": "Discovery Park, Seattle, WA"
  },

  "observer": {
    "id": "orcid:0000-0002-1825-0097",
    "name": "Jane Smith",
    "organization": "Seattle Audubon Society",
    "email": "jane.smith@seattleaudubon.org"
  },

  "taxon": {
    "scientific_name": "Haliaeetus leucocephalus",
    "common_name": "대머리독수리",
    "kingdom": "Animalia",
    "phylum": "Chordata",
    "class": "Aves",
    "order": "Accipitriformes",
    "family": "Accipitridae",
    "genus": "Haliaeetus",
    "species": "leucocephalus",
    "subspecies": null,
    "taxon_authority": "Catalogue of Life 2025",
    "taxon_id": "COL:7MDXZ"
  },

  "detection_method": "visual_survey",
  "occurrence_status": "present",
  "abundance": 2,
  "life_stage": "adult",
  "sex": "unknown",
  "behavior": "Perched in snag, vocalizing",
  "reproductive_condition": null,
  "habitat_type": "ENVO:00000447",

  "associated_taxa": [
    {
      "scientific_name": "Corvus corax",
      "relationship": "Mobbing eagle"
    }
  ],

  "environmental_conditions": {
    "temperature_celsius": 18.5,
    "weather": "Partly cloudy",
    "wind_speed_ms": 3.2
  },

  "quality": {
    "validation_status": "expert_verified",
    "quality_flags": [],
    "confidence_level": 1.0
  }
}
```

### 4.2.2 탐지 방법

`detection_method`에 대한 제어 어휘:

| 값 | 설명 | 일반적인 용도 |
|----|-----|-------------|
| `visual_survey` | 관찰자에 의한 직접 관찰 | 조류, 포유류, 양서파충류, 식물 |
| `camera_trap` | 동작 활성화 카메라 | 육상 포유류, 지상 조류 |
| `acoustic_monitoring` | 오디오 녹음 및 분석 | 조류, 박쥐, 양서류, 곤충 |
| `edna` | 환경 DNA 샘플링 | 수생 유기체, 희귀종 |
| `telemetry` | 라디오/GPS 추적 장치 | 이동 생태학, 서식지 범위 |
| `mark_recapture` | 포획-표시-재포획 연구 | 개체군 추정 |
| `remote_sensing` | 위성/항공 이미지 분석 | 대형 동물, 경관 규모 |
| `specimen` | 물리적 표본 수집 | 분류학적 검증 |

### 4.2.3 출현 상태

```typescript
enum OccurrenceStatus {
  present = "위치에서 유기체 감지됨",
  absent = "대상 조사 수행, 유기체 발견 안 됨",
  trace = "유기체의 증거 (발자국, 배설물, 굴) 있으나 직접 관찰은 없음"
}
```

**중요:** `absent`는 대상 조사가 수행된 경우에만 유효합니다. 기회 관찰은 `present` 또는 `trace`만 기록해야 합니다.

### 4.2.4 생애 단계 값

```typescript
enum LifeStage {
  // 동물
  egg = "알 단계",
  larva = "유충 단계 (곤충, 양서류)",
  juvenile = "유년/미성숙",
  adult = "성체/성숙",

  // 식물
  seed = "씨앗",
  seedling = "묘목 (< 1년)",
  sapling = "어린 나무",
  mature = "성숙한 식물",

  // 일반
  unknown = "생애 단계를 결정할 수 없음"
}
```

---

## 4.3 환경 센서 데이터 스키마

### 4.3.1 센서 시계열 스키마

```json
{
  "wia_version": "1.0",
  "schema_type": "sensor-timeseries",
  "record_id": "650e8400-e29b-41d4-a716-446655440111",
  "timestamp": "2025-06-15T00:00:00Z",

  "sensor_id": "WEATHER-STATION-042",

  "sensor_metadata": {
    "sensor_type": "Temperature",
    "manufacturer": "Campbell Scientific",
    "model": "CS107",
    "serial_number": "SN12345678",
    "measurement_parameter": "Air temperature",
    "measurement_unit": "celsius",
    "precision": 0.1,
    "accuracy": 0.2,
    "detection_limit": null,
    "calibration_date": "2025-01-15",
    "calibration_certificate": "https://example.org/certs/CAL-2025-042.pdf"
  },

  "deployment": {
    "deployment_date": "2025-01-20T10:00:00Z",
    "location": {
      "latitude": 47.6815,
      "longitude": -121.7453,
      "elevation": 850,
      "datum": "WGS84"
    },
    "height_above_ground": 2.0,
    "depth_below_surface": null,
    "environment": "Forest canopy gap"
  },

  "data": {
    "start_time": "2025-06-15T00:00:00Z",
    "end_time": "2025-06-15T23:59:00Z",
    "interval_seconds": 900,
    "readings": [
      {
        "timestamp": "2025-06-15T00:00:00Z",
        "value": 12.3,
        "qc_flag": "good",
        "qc_notes": null
      },
      {
        "timestamp": "2025-06-15T00:15:00Z",
        "value": 12.1,
        "qc_flag": "good",
        "qc_notes": null
      },
      {
        "timestamp": "2025-06-15T00:30:00Z",
        "value": 11.9,
        "qc_flag": "good",
        "qc_notes": null
      }
      // ... 15분 간격으로 하루 96개 판독값
    ]
  },

  "observer": {
    "id": "org:neon",
    "name": "National Ecological Observatory Network",
    "organization": "Battelle"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.98
  }
}
```

### 4.3.2 QC 플래그 값

```typescript
enum QCFlag {
  good = "모든 QC 검사 통과, 모든 분석에 적합",
  questionable = "한계 허용 가능, 주의하여 사용",
  bad = "QC 검사 실패, 분석에서 제외",
  missing = "예상된 판독값이지만 수집되지 않음 (센서 오프라인, 전원 고장)",
  estimated = "보간 또는 모델링된 값, 직접 측정되지 않음"
}
```

### 4.3.3 측정 단위 (QUDT)

모든 단위는 QUDT 온톨로지를 참조해야 합니다:

| 측정 | 단위 | QUDT 코드 |
|------|-----|----------|
| 온도 | celsius | qudt:DEG_C |
| 온도 | fahrenheit | qudt:DEG_F |
| 압력 | millibar | qudt:MilliBAR |
| 풍속 | meters per second | qudt:M-PER-SEC |
| 강수량 | millimeter | qudt:MilliM |
| 습도 | percent | qudt:PERCENT |

---

## 4.4 수질 스키마

### 4.4.1 완전한 스키마

```json
{
  "wia_version": "1.0",
  "schema_type": "water-quality-sample",
  "record_id": "750e8400-e29b-41d4-a716-446655440222",
  "timestamp": "2025-06-15T10:30:00-07:00",

  "location": {
    "latitude": 47.6201,
    "longitude": -122.3493,
    "elevation": 5.0,
    "datum": "WGS84",
    "precision": 5,
    "location_name": "Green River at Auburn"
  },

  "sample_id": "GR-2025-06-15-001",
  "waterbody_name": "Green River",
  "site_id": "USGS-12113000",
  "sampling_depth_meters": 0.5,
  "sampling_method": "grab_sample",

  "parameters": {
    "temperature_c": 16.5,
    "ph": 7.2,
    "dissolved_oxygen_mgl": 9.8,
    "turbidity_ntu": 12.3,
    "conductivity_uscm": 145,
    "total_nitrogen_mgl": 0.85,
    "total_phosphorus_mgl": 0.042,
    "chlorophyll_a_ugl": 3.2,
    "salinity_ppt": 0.1
  },

  "laboratory": {
    "lab_name": "King County Environmental Lab",
    "analysis_date": "2025-06-16",
    "methods": {
      "total_nitrogen": "EPA 351.2",
      "total_phosphorus": "EPA 365.1"
    },
    "detection_limits": {
      "total_nitrogen_mgl": 0.01,
      "total_phosphorus_mgl": 0.002
    }
  },

  "observer": {
    "id": "kcenvlab:tech042",
    "name": "Robert Johnson",
    "organization": "King County Environmental Laboratory"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.95
  }
}
```

### 4.4.2 샘플링 방법

```typescript
enum SamplingMethod {
  grab_sample = "특정 시점의 단일 이산 샘플",
  composite = "여러 샘플 결합 (시간 또는 공간 복합)",
  in_situ = "센서로 현장에서 측정 (제거되지 않음)",
  continuous = "자동 연속 모니터링 시스템"
}
```

### 4.4.3 표준 매개변수

| 매개변수 | 단위 | 일반적인 범위 | 방법 |
|---------|-----|-------------|-----|
| 온도 | °C | 0-40 | 현장 온도계/센서 |
| pH | - | 4-10 | 유리 전극 |
| 용존 산소 | mg/L | 0-20 | 광학 또는 전극 센서 |
| 탁도 | NTU | 0-1000 | 네펠로미터 |
| 전도도 | μS/cm | 0-5000 | 전도도 프로브 |
| 총 질소 | mg/L | 0-10 | EPA 351.2 또는 동등 |
| 총 인 | mg/L | 0-1 | EPA 365.1 또는 동등 |
| 엽록소-a | μg/L | 0-100 | 형광법 또는 추출 |
| 염분 | ppt | 0-40 | 전도도로부터 계산 |

---

## 4.5 대기질 스키마

### 4.5.1 완전한 스키마

```json
{
  "wia_version": "1.0",
  "schema_type": "air-quality-measurement",
  "record_id": "850e8400-e29b-41d4-a716-446655440333",
  "timestamp": "2025-06-15T14:00:00-07:00",

  "location": {
    "latitude": 47.6062,
    "longitude": -122.3321,
    "elevation": 50,
    "datum": "WGS84",
    "precision": 10,
    "location_name": "Seattle - Duwamish Valley"
  },

  "station_id": "EPA-AQS-530330030",

  "parameters": {
    "pm25_ugm3": 8.5,
    "pm10_ugm3": 15.2,
    "o3_ppb": 42,
    "no2_ppb": 18,
    "so2_ppb": 2,
    "co_ppm": 0.4,
    "co2_ppm": 415,
    "temperature_c": 18.5,
    "relative_humidity_percent": 65,
    "pressure_mb": 1013.25,
    "wind_speed_ms": 3.2,
    "wind_direction_degrees": 225
  },

  "aqi_value": 35,
  "aqi_category": "good",

  "observer": {
    "id": "epa:pscaa",
    "name": "Puget Sound Clean Air Agency",
    "organization": "EPA AQS Network"
  },

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 0.98
  }
}
```

### 4.5.2 AQI 범주

```typescript
enum AQICategory {
  good = "0-50: 대기질 양호",
  moderate = "51-100: 대부분에게 허용 가능, 민감 그룹은 영향받을 수 있음",
  unhealthy_sensitive = "101-150: 민감 그룹이 건강 영향을 경험할 수 있음",
  unhealthy = "151-200: 모두가 건강 영향을 경험할 수 있음",
  very_unhealthy = "201-300: 건강 경보, 모두가 심각한 영향을 경험할 수 있음",
  hazardous = "301+: 비상 상황의 건강 경고"
}
```

### 4.5.3 표준 오염물질

| 오염물질 | 단위 | 건강 우려 | EPA 표준 |
|---------|-----|---------|---------|
| PM2.5 | μg/m³ | 호흡기, 심혈관 | 연간 12 μg/m³ |
| PM10 | μg/m³ | 호흡기 | 24시간 150 μg/m³ |
| 오존 (O3) | ppb | 호흡기 자극 | 8시간 70 ppb |
| NO2 | ppb | 호흡기 | 1시간 100 ppb |
| SO2 | ppb | 호흡기 | 1시간 75 ppb |
| CO | ppm | 심혈관 | 8시간 9 ppm |

---

## 4.6 메타데이터 표준

### 4.6.1 데이터셋 수준 메타데이터

모든 데이터셋(관찰 모음)은 포괄적인 메타데이터를 포함해야 합니다:

```json
{
  "title": "Green River Watershed Biodiversity Monitoring 2020-2025",
  "abstract": "그린 리버 유역의 수생 및 육상 생물다양성 장기 모니터링, 연어 회복 지표 및 강변 서식지 품질에 초점.",
  "keywords": ["생물다양성", "연어", "강변", "Green River", "모니터링"],

  "authors": [
    {
      "name": "Jane Smith",
      "orcid": "0000-0002-1825-0097",
      "affiliation": "University of Washington",
      "role": "Principal Investigator"
    },
    {
      "name": "Robert Johnson",
      "orcid": "0000-0003-9876-5432",
      "affiliation": "King County",
      "role": "Data Manager"
    }
  ],

  "contacts": [
    {
      "name": "Jane Smith",
      "email": "jsmith@uw.edu",
      "role": "Data custodian"
    }
  ],

  "funding_sources": [
    "EPA Puget Sound Partnership Grant #PS-2020-042",
    "Washington Department of Fish and Wildlife"
  ],

  "temporal_coverage": {
    "start_date": "2020-01-01",
    "end_date": "2025-12-31"
  },

  "geographic_coverage": {
    "bounding_box": [-122.5, 47.3, -121.8, 47.8],
    "description": "캐스케이드 산맥 상류에서 엘리엇 베이 하구까지의 그린 리버 유역"
  },

  "taxonomic_coverage": [
    {
      "scientific_name": "Oncorhynchus",
      "common_name": "태평양 연어",
      "rank": "genus"
    },
    {
      "scientific_name": "Aves",
      "common_name": "조류",
      "rank": "class"
    }
  ],

  "methods": {
    "description": "지점 계수 조사(조류), 전기 어업(어류), 함정 트랩(무척추동물), 수질 샘플 채취",
    "protocol_url": "https://example.org/protocols/green-river-2020.pdf",
    "sampling_design": "계층화된 무작위, 20개 사이트 매월 재방문"
  },

  "quality_assurance": {
    "description": "모든 종 ID는 분류학 전문가에 의해 검증됨. 수질 샘플은 인증된 실험실에서 분석됨. 10% 현장 복제본.",
    "validation_procedures": "자동 범위 검사, 플래그된 레코드의 전문가 검토"
  },

  "access": {
    "license": "CC BY 4.0",
    "restrictions": "민감한 종 위치 (독수리, 박쥐)는 1km 정밀도로 모호화됨",
  },

  "related_resources": {
    "publications": [
      "https://doi.org/10.1234/paper1",
      "https://doi.org/10.1234/paper2"
    ],
    "related_datasets": [
      "https://doi.org/10.1234/related-dataset"
    ]
  }
}
```

---

## 4.7 검증 규칙

### 4.7.1 자동화된 검증 검사

모든 WIA 준수 데이터는 다음 검증 검사를 통과해야 합니다:

```typescript
interface ValidationRules {
  schemaValidation: {
    check: "데이터 구조가 JSON 스키마와 일치";
    errorExample: "필드 'latitude'가 필수이지만 누락됨";
  };

  dataTypes: {
    check: "값이 지정된 유형과 일치";
    errorExample: "필드 'abundance'는 숫자를 예상하지만 문자열 '5'를 받음";
  };

  rangeChecks: {
    latitude: "-90과 90 사이여야 함",
    longitude: "-180과 180 사이여야 함",
    temperature_c: "-100과 100 사이여야 함",
    ph: "0과 14 사이여야 함",
    confidence_level: "0과 1 사이여야 함"
  };

  temporalValidation: {
    futureDate: "타임스탬프는 미래일 수 없음",
    endAfterStart: "종료 시간은 시작 시간 >= 여야 함"
  };

  taxonomicValidation: {
    scientificName: "권위에서 인정된 분류군으로 해결되어야 함",
    authority: "다음 중 하나여야 함: Catalogue of Life, GBIF Backbone, ITIS, WoRMS"
  };

  spatialValidation: {
    landOcean: "육상 종의 경우 위도/경도는 육지에 있어야 함",
    precision: "제공된 경우 정밀도는 > 0이어야 함"
  };

  qualityFlags: {
    validationStatus: "다음 중 하나여야 함: unvalidated, in_review, validated, expert_verified, questionable, invalid",
    qcFlag: "다음 중 하나여야 함: good, questionable, bad, missing, estimated"
  };
}
```

### 4.7.2 교차 필드 검증

```typescript
interface CrossFieldValidation {
  detectionMethodVsOccurrence: {
    rule: "occurrence_status = 'absent'인 경우, detection_method는 'remote_sensing' 또는 'camera_trap'일 수 없음",
    reason: "이러한 방법은 부재를 확인할 수 없음"
  };

  abundanceVsOccurrence: {
    rule: "occurrence_status = 'absent'인 경우, abundance는 0 또는 null이어야 함",
    reason: "부재 종에 대해 풍부도를 가질 수 없음"
  };

  elevationVsLocation: {
    rule: "고도는 위도/경도와 일치해야 함 (DEM 조회)",
    warning: "고도가 예상보다 >100m 차이나면 플래그 지정"
  };

  temperatureVsLocation: {
    rule: "온도는 위치 및 계절에 대해 그럴듯해야 함",
    warning: "위치/날짜에 대한 예상 범위를 벗어나면 플래그 지정"
  };
}
```

---

## 4.8 파일 형식 규칙

### 4.8.1 단일 레코드 파일

**형식:**
- 인코딩: UTF-8
- 형식: JSON
- 확장자: `.json`
- 압축: gzip 선택적 (`.json.gz`)

**예제 파일명:** `observation-550e8400-e29b-41d4-a716-446655440000.json`

### 4.8.2 다중 레코드 파일

**줄바꿈으로 구분된 JSON (NDJSON):**
```
{"wia_version":"1.0","schema_type":"species-observation",...}
{"wia_version":"1.0","schema_type":"species-observation",...}
{"wia_version":"1.0","schema_type":"species-observation",...}
```

- 한 줄에 하나의 완전한 JSON 객체
- 확장자: `.ndjson` 또는 `.jsonl`
- 압축: 대용량 파일에는 gzip 권장 (`.ndjson.gz`)

**장점:**
- 스트리밍 가능 (한 번에 한 줄씩 처리)
- 추가 가능 (전체 파일을 파싱하지 않고 레코드 추가)
- 병렬 처리 (줄로 파일 분할)

### 4.8.3 일괄 제출

**JSON 배열:**
```json
[
  {"wia_version":"1.0", "schema_type":"species-observation", ...},
  {"wia_version":"1.0", "schema_type":"species-observation", ...},
  {"wia_version":"1.0", "schema_type":"species-observation", ...}
]
```

- 파일당 최대 10,000개 레코드 권장
- 더 큰 데이터셋에는 NDJSON 사용

---

## 4.9 복습 질문

### 질문 1
산악 하천의 온도 판독값 35°C가 오류일 가능성이 있는지 감지하는 검증 알고리즘을 설계하시오. 어떤 필드를 확인하겠는가?

### 질문 2
연구자가 일반 이름만 있는 조류 관찰을 가지고 있습니다 ("로빈", "참새"). WIA 형식으로 변환하는 방법은? 어떤 도전이 발생할 수 있는가?

### 질문 3
`occurrence_status = "absent"`와 관찰 레코드가 없는 것의 차이를 설명하시오. 각각 언제 사용해야 하는가?

### 질문 4
센서가 하루에 96개의 판독값(15분 간격)을 생성합니다. 이것들을 96개의 별도 WIA 레코드로 저장해야 하는가, 아니면 96개의 판독값이 있는 하나의 레코드로 저장해야 하는가? 답을 정당화하시오.

### 질문 5
이 레거시 수질 레코드를 WIA 형식으로 변환하시오:
```
사이트: Green River
날짜: 6/15/25 10:30 AM
온도: 61.7 F
DO: 9.8
pH: 7.2
참고: 약간 탁함
```

---

## 4.10 주요 요점

| 구성요소 | 핵심 사항 |
|---------|---------|
| **기본 스키마** | 모든 레코드 유형이 공유하는 공통 필드 |
| **종 관찰** | 분류학적 정보, 탐지 방법, 풍부도 |
| **센서 데이터** | QC 플래그가 있는 메타데이터가 풍부한 시계열 |
| **수질** | 실험실 방법, 검출 한계, 매개변수 |
| **대기질** | 오염물질, 기상학, AQI 계산 |
| **메타데이터** | 포괄적인 데이터셋 문서화 |
| **검증** | 자동화된 검사, 범위 검증, 교차 필드 규칙 |

### 설계 결정
- **JSON**: 사람이 읽을 수 있고, 널리 지원됨
- **ISO 8601**: 시간대가 있는 명확한 날짜/시간
- **WGS84**: 글로벌 상호운용성을 위한 표준 데이텀
- **QUDT**: 표준 단위 어휘
- **제어 어휘**: 철자 오류 방지, 필터링 가능

### 다음 장 미리보기

5장은 Phase 2(API 인터페이스)를 자세히 설명하며, REST, WebSocket 및 MQTT 프로토콜을 사용하여 프로그래밍 방식으로 관찰을 제출하고, 데이터를 쿼리하며, 실시간 센서 판독값을 스트리밍하는 방법을 보여줍니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
