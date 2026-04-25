# 제3장: 데이터 형식

## 1단계: 표준화된 생물다양성 데이터 스키마

### 구조화된 데이터를 통한 글로벌 상호운용성 실현

---

## 개요

WIA 생물다양성 지수 표준의 1단계는 생물다양성 정보를 위한 표준화된 데이터 형식을 정의하여 전 세계 시스템과 기관 간의 상호운용성을 가능하게 합니다. 이 장에서는 종 출현, 환경DNA 샘플, 서식지 분류 및 다양성 지수를 위한 JSON 스키마, 검증 규칙 및 데이터 교환 프로토콜을 다룹니다.

---

## 핵심 스키마 아키텍처

### 설계 원칙

WIA 생물다양성 지수 데이터 형식은 다음 핵심 원칙을 따릅니다:

1. **JSON 네이티브**: 현대 API 통합을 위한 JSON이 주요 형식
2. **Darwin Core 호환**: 확립된 GBIF 표준과의 매핑
3. **확장 가능**: 선택적 확장이 있는 핵심 스키마
4. **검증됨**: 기계 판독 가능한 검증 규칙
5. **버전 관리됨**: 폐기 정책이 있는 시맨틱 버전 관리

### 스키마 레지스트리

**스키마 기본 URL:** `https://wia.org/schemas/`

| 스키마 | ID | 버전 | 상태 |
|--------|-----|------|------|
| 종 출현 | `occurrence/v1.0` | 1.0 | 활성 |
| 환경DNA 샘플 | `edna/v1.0` | 1.0 | 활성 |
| 서식지 분류 | `habitat/v1.0` | 1.0 | 활성 |
| 다양성 지수 | `diversity-index/v1.0` | 1.0 | 활성 |
| 분류학 참조 | `taxonomy/v1.0` | 1.0 | 활성 |
| 조사 이벤트 | `survey-event/v1.0` | 1.0 | 활성 |

---

## 종 출현 스키마

### 스키마 정의

**스키마 ID:** `https://wia.org/schemas/occurrence/v1.0`

종 출현 스키마는 종이 언제, 어디서, 어떻게 감지되었는지에 대한 표준화된 기록을 캡처합니다.

### 필드 요구사항

**필수 필드:**

| 필드 | 유형 | 설명 | 검증 |
|------|------|------|------|
| `occurrence_id` | 문자열 | 고유 식별자 | 패턴: `^OCC-\d{4}-\d{6}$` |
| `species.scientific_name` | 문자열 | 이명법 학명 | 대문자로 시작하는 속명 |
| `location.latitude` | 숫자 | 십진도 위도 | -90에서 90 |
| `location.longitude` | 숫자 | 십진도 경도 | -180에서 180 |
| `temporal.observation_date` | 문자열 | ISO 8601 타임스탬프 | 미래가 아님 |

**권장 필드:**

| 필드 | 유형 | 설명 | 기본값 |
|------|------|------|--------|
| `observation.individual_count` | 정수 | 관찰된 개체 수 | 1 |
| `location.habitat_type` | 문자열 | 서식지 분류 | null |
| `observation.observer_id` | 문자열 | 관찰자 식별자 | null |
| `quality.quality_flag` | 열거형 | 검증 상태 | "unvalidated" |

### 완전한 스키마 예제

```json
{
  "$schema": "https://wia.org/schemas/occurrence/v1.0",
  "occurrence_id": "OCC-2025-123456",
  "species": {
    "scientific_name": "Panthera tigris",
    "common_name": "벵골 호랑이",
    "taxonomy": {
      "kingdom": "Animalia",
      "phylum": "Chordata",
      "class": "Mammalia",
      "order": "Carnivora",
      "family": "Felidae",
      "genus": "Panthera",
      "species": "tigris"
    },
    "iucn_status": "EN",
    "taxon_id": "GBIF:9694"
  },
  "location": {
    "latitude": 27.5142,
    "longitude": 88.7597,
    "coordinate_uncertainty_m": 50,
    "elevation_m": 350,
    "habitat_type": "tropical_moist_forest",
    "country": "India",
    "country_code": "IN",
    "protected_area": {
      "name": "순다르반스 국립공원",
      "wdpa_id": 9164,
      "iucn_category": "II"
    }
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z",
    "date_precision": "day"
  },
  "observation": {
    "individual_count": 1,
    "sex": "male",
    "life_stage": "adult",
    "behavior": "foraging",
    "basis_of_record": "human_observation",
    "detection_method": "visual",
    "observer_id": "OBS-2025-042"
  },
  "quality": {
    "quality_flag": "expert_verified",
    "quality_score": 0.95
  }
}
```

### Darwin Core 매핑

WIA 출현 스키마는 Darwin Core 용어에 매핑됩니다:

| WIA 필드 | Darwin Core 용어 | 비고 |
|----------|-----------------|------|
| `occurrence_id` | `occurrenceID` | 직접 매핑 |
| `species.scientific_name` | `scientificName` | 직접 매핑 |
| `location.latitude` | `decimalLatitude` | 직접 매핑 |
| `location.longitude` | `decimalLongitude` | 직접 매핑 |
| `temporal.observation_date` | `eventDate` | ISO 8601 형식 |
| `observation.individual_count` | `individualCount` | 직접 매핑 |
| `observation.basis_of_record` | `basisOfRecord` | 통제 어휘 |
| `location.country_code` | `countryCode` | ISO 3166-1 alpha-2 |

---

## 환경DNA 샘플 스키마

### 스키마 정의

**스키마 ID:** `https://wia.org/schemas/edna/v1.0`

환경DNA 스키마는 환경 DNA 샘플 수집, 처리 및 탐지 결과를 캡처합니다.

### 샘플 수집 필드

```json
{
  "$schema": "https://wia.org/schemas/edna/v1.0",
  "sample_id": "EDNA-2025-KR-089234",
  "collection": {
    "date": "2025-10-22T11:15:00Z",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "water_body_name": "한강",
      "water_body_type": "river",
      "country_code": "KR"
    },
    "sampling_method": "filtration",
    "volume_liters": 2.0,
    "filter_pore_size_um": 0.45,
    "replicates": 3,
    "negative_control": true,
    "field_conditions": {
      "water_temperature_c": 14.2,
      "water_ph": 7.8,
      "turbidity_ntu": 12.5
    }
  },
  "processing": {
    "extraction_date": "2025-10-23T09:00:00Z",
    "extraction_method": "qiagen_dneasy_powerwater",
    "sequencing_platform": "illumina_novaseq_6000",
    "target_gene": "COI"
  },
  "results": {
    "total_reads": 2847562,
    "detected_taxa": [
      {
        "scientific_name": "Zacco platypus",
        "common_name": "피라미",
        "read_count": 15432,
        "confidence": 0.97,
        "iucn_status": "LC"
      }
    ],
    "species_richness": 24
  }
}
```

### 환경DNA 품질 관리

**필수 QC 검사:**

| 검사 | 요구사항 | 실패 시 조치 |
|------|----------|--------------|
| 음성 대조군 | 대상 DNA 미검출 | 샘플 플래그, 오염 조사 |
| 양성 대조군 | 예상 종 검출 | 배치 플래그, 재추출 |
| 판독 깊이 | 샘플당 최소 10,000 리드 | 재시퀀싱 플래그 |
| 키메라율 | 전체 리드의 <5% | 생물정보학 재실행 |

---

## 서식지 분류 스키마

### IUCN 기반 계층

WIA 서식지 분류는 상세한 생태적 특성화를 위한 확장이 포함된 IUCN 서식지 분류 체계를 따릅니다.

### 레벨 1 범주

| 코드 | 범주 | 설명 |
|------|------|------|
| 1 | 산림 | 나무가 지배하는 생태계 |
| 2 | 사바나 | 흩어진 나무가 있는 초원 |
| 3 | 관목지 | 관목이 지배하는 생태계 |
| 4 | 초원 | 풀이 지배하는 생태계 |
| 5 | 습지 (내륙) | 담수 습지 |
| 6 | 암석 지역 | 동굴, 절벽, 암석 노두 |
| 7 | 동굴 및 지하 | 지하 서식지 |
| 8 | 사막 | 건조 생태계 |
| 9 | 해양 연안 | 얕은 바다 (0-200m) |
| 10 | 해양 원양 | 외해 (>200m) |
| 11 | 해양 심해 | 저서대 |
| 12 | 해양 조간대 | 조간대 |
| 13 | 해양 연안 | 연안 생태계 |
| 14 | 인공 - 육상 | 인간 변형 토지 |
| 15 | 인공 - 수생 | 인간 변형 수역 |

### 서식지 스키마 예제

```json
{
  "$schema": "https://wia.org/schemas/habitat/v1.0",
  "habitat_id": "HAB-2025-KR-0042",
  "classification": {
    "level_1": {
      "code": 1,
      "name": "산림"
    },
    "level_2": {
      "code": "1.4",
      "name": "온대림"
    }
  },
  "characteristics": {
    "canopy_height_m": 25,
    "canopy_cover_percent": 85,
    "dominant_species": ["Pinus densiflora", "Quercus mongolica"]
  },
  "condition": {
    "disturbance_level": "minimal",
    "fragmentation": "low",
    "connectivity": "high"
  }
}
```

---

## 다양성 지수 결과 스키마

### 스키마 정의

**스키마 ID:** `https://wia.org/schemas/diversity-index/v1.0`

다양성 지수 스키마는 통계적 신뢰 측정과 함께 계산된 생물다양성 지표를 캡처합니다.

### 지원되는 지수

| 지수 | 공식 | 범위 | 해석 |
|------|------|------|------|
| 종 풍부도 (S) | 종 수 | 0에서 ∞ | 높을수록 = 더 많은 종 |
| Shannon 다양성 (H') | -Σ(pi × ln(pi)) | 0에서 ln(S) | 높을수록 = 더 다양 |
| Simpson 지수 (D) | Σ(pi²) | 0에서 1 | 낮을수록 = 더 다양 |
| Simpson 다양성 (1-D) | 1 - Σ(pi²) | 0에서 1 | 높을수록 = 더 다양 |
| 역 Simpson (1/D) | 1 / Σ(pi²) | 1에서 S | 높을수록 = 더 다양 |
| Pielou 균등도 (J') | H' / ln(S) | 0에서 1 | 높을수록 = 더 균등 |
| Chao1 추정기 | S + (f₁²/2f₂) | S에서 ∞ | 추정 실제 풍부도 |

### 결과 스키마 예제

```json
{
  "$schema": "https://wia.org/schemas/diversity-index/v1.0",
  "calculation_id": "CALC-2025-KOREA-001",
  "input_summary": {
    "dataset_id": "DS-KOREA-2025",
    "occurrence_count": 15847,
    "species_count": 156
  },
  "spatial_extent": {
    "type": "polygon",
    "area_km2": 125.8,
    "country_code": "KR",
    "protected_area": "지리산 국립공원"
  },
  "indices": {
    "species_richness": {
      "observed": 156,
      "rarefied_value": 142.7,
      "confidence_interval_95": {
        "lower": 138.2,
        "upper": 147.3
      }
    },
    "shannon_diversity": {
      "value": 4.127,
      "variance": 0.0052,
      "confidence_interval_95": {
        "lower": 3.982,
        "upper": 4.268
      }
    },
    "simpson_index": {
      "dominance_d": 0.0234,
      "diversity_1_minus_d": 0.9766
    }
  }
}
```

---

## 검증 규칙

### 지리적 검증

**좌표 유효성:**
```python
def validate_coordinates(lat, lon):
    errors = []

    # 범위 검사
    if not (-90 <= lat <= 90):
        errors.append("위도가 범위 [-90, 90] 벗어남")
    if not (-180 <= lon <= 180):
        errors.append("경도가 범위 [-180, 180] 벗어남")

    # 영점 검사 (종종 누락된 데이터를 나타냄)
    if lat == 0 and lon == 0:
        errors.append("경고: (0,0) 좌표 - 정확한지 확인")

    return errors
```

### 시간적 검증

**날짜 규칙:**
- 관찰 날짜는 미래일 수 없음
- 역사적 한계: 일반적으로 1600년 이전 아님
- 정밀도 일관성: 날짜 정밀도가 명시된 해상도와 일치

### 분류학적 검증

**명명법 규칙:**
1. 학명은 이명법/삼명법 형식을 따름
2. 속은 대문자, 종/아종은 소문자
3. 복합 상피에서 하이픈 외 특수 문자 없음
4. 저자 인용은 선택사항이지만 있으면 표준화

---

## 직렬화 형식

### JSON (표준)

**API 통신을 위한 주요 형식:**
- UTF-8 인코딩 필수
- 개발을 위한 예쁜 인쇄
- 프로덕션을 위한 축소
- Content-Type: `application/json`

### GeoJSON

**공간 데이터 표현:**
- RFC 7946 사양 준수
- 여러 출현을 위한 Feature 컬렉션
- 속성에 전체 출현 메타데이터 포함
- Content-Type: `application/geo+json`

### CSV

**테이블 형식 데이터 내보내기:**
- BOM이 있는 UTF-8 인코딩
- 헤더 행 필수
- 중첩 필드는 점 표기법으로 평탄화
- 누락된 값은 빈 문자열
- Content-Type: `text/csv`

### Apache Parquet

**빅 데이터 분석:**
- Apache Parquet 2.0+ 형식
- 효율적인 쿼리를 위한 열 기반 저장
- 파일에 스키마 포함
- 압축: Snappy (기본) 또는 Gzip

---

## 핵심 내용

1. **JSON 네이티브 스키마**가 Darwin Core 호환성을 유지하면서 현대 API 통합 제공
2. **출현 스키마**가 표준화된 검증 규칙으로 누가, 무엇을, 언제, 어디서 캡처
3. **환경DNA 스키마**가 수집에서 생물정보학, 탐지 결과까지 전체 워크플로우 커버
4. **서식지 분류**가 조건 및 기후 확장이 있는 IUCN 계층 준수
5. **다양성 지수 스키마**가 통계적 신뢰 측정 및 풍부도 추정기 포함

## 복습 문제

1. WIA 종 출현 스키마의 5개 필수 필드는 무엇입니까?
2. WIA 스키마는 Darwin Core 용어에 어떻게 매핑됩니까?
3. 환경DNA 샘플에 필요한 품질 관리 검사는 무엇입니까?
4. 다양성 지수 스키마에 포함된 세 가지 생물다양성 지수를 나열하세요.
5. 지리적 좌표에 적용되는 검증 규칙은 무엇입니까?

---

**다음 장 미리보기:** 4장에서는 WIA 생물다양성 지수 API 인터페이스 (2단계)를 탐구하며, RESTful 엔드포인트, GraphQL 쿼리, 인증 및 SDK 지원을 다룹니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 모든 생명을 보존
