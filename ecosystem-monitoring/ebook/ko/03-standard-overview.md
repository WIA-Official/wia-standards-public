# 제3장: WIA 생태계 모니터링 표준 개요

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:

1. WIA 4단계 아키텍처 및 근거 설명
2. 각 단계가 특정 과제를 해결하는 방법 이해
3. 표준의 핵심 구성요소 식별
4. WIA 접근방식을 기존 표준과 비교
5. 다양한 사용 사례에 대한 채택 경로 평가

---

## 3.1 WIA 4단계 아키텍처

### 3.1.1 단계 개요

WIA 생태계 모니터링 표준은 모든 WIA 표준에서 사용되는 검증된 4단계 아키텍처를 따릅니다:

```
WIA 4단계 아키텍처:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  PHASE 1: 데이터 형식                                                │
│  ├─ 무엇: JSON 스키마, 제어 어휘, 검증 규칙                           │
│  ├─ 왜: 일관되고 기계 판독 가능한 데이터 구조 보장                     │
│  ├─ 누구를 위해: 데이터 수집자, 데이터베이스 설계자                    │
│  └─ 출력: 표준화된 관찰 및 측정 레코드                                │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 2: API 인터페이스                                             │
│  ├─ 무엇: RESTful 엔드포인트, WebSocket 스트리밍, 대량 액세스         │
│  ├─ 왜: 프로그래밍 방식의 데이터 제출 및 검색 가능                     │
│  ├─ 누구를 위해: 소프트웨어 개발자, 데이터 관리자                      │
│  └─ 출력: 시스템 전반의 상호운용 가능한 데이터 액세스                  │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 3: 프로토콜                                                   │
│  ├─ 무엇: QA/QC 절차, 보정 표준, 현장 방법                           │
│  ├─ 왜: 데이터 품질 및 과학적 엄격성 보장                             │
│  ├─ 누구를 위해: 현장 연구자, 실험실 기술자                           │
│  └─ 출력: 출처가 있는 출판 품질 데이터셋                              │
│                                                                     │
│                        ⬇                                           │
│                                                                     │
│  PHASE 4: 시스템 통합                                                │
│  ├─ 무엇: GIS 연결, 클라우드 플랫폼, 보존 데이터베이스                │
│  ├─ 왜: WIA 시스템을 기존 인프라에 연결                              │
│  ├─ 누구를 위해: IT 전문가, 시스템 관리자                            │
│  └─ 출력: 조직 경계를 넘는 원활한 데이터 흐름                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.1.2 4단계가 필요한 이유?

```typescript
// 단계별 접근방식의 근거
interface PhasedApproachRationale {
  separation_of_concerns: {
    benefit: "각 단계가 고유한 기술 영역을 다룸";
    example: "데이터 과학자는 형식에 집중, DevOps는 통합에 집중";
  };

  incremental_adoption: {
    benefit: "조직이 단계적으로 단계를 구현할 수 있음";
    example: "Phase 1(데이터 형식)로 시작, 나중에 API 추가";
  };

  expertise_matching: {
    benefit: "다른 단계에 다른 기술이 필요";
    example: "생태학자가 프로토콜 설계, 엔지니어가 API 구축";
  };

  versioning_independence: {
    benefit: "단계가 다른 속도로 발전할 수 있음";
    example: "API 변경 없이 새 데이터 유형 추가";
  };

  clear_responsibilities: {
    benefit: "이해관계자가 자신에게 적용되는 것을 알고 있음";
    example: "현장 팀은 Phase 3를 따르고, Phase 2와 4는 무시";
  };
}
```

---

## 3.2 Phase 1: 데이터 형식 사양

### 3.2.1 핵심 구성요소

**종 관찰 스키마**
- 권위가 있는 분류학적 정보
- 탐지 방법 및 발생 상태
- 생애 단계, 성별, 풍부도
- 관련 환경 조건

**센서 시계열 스키마**
- 센서 메타데이터 (유형, 보정, 배치)
- 타임스탬프 및 QC 플래그가 있는 판독값
- 배터리 전압 및 신호 강도
- 측정 단위 (QUDT 표준)

**수질 스키마**
- 샘플 ID 및 사이트 정보
- 물리적 매개변수 (온도, pH, DO, 탁도)
- 화학적 매개변수 (영양소, 염도)
- 실험실 방법 및 검출 한계

**대기질 스키마**
- 스테이션 ID 및 위치
- 오염물질 농도 (PM2.5, O3, NO2 등)
- 기상 조건
- AQI 값 및 범주

**기본 스키마 (모두 공통)**
- WIA 버전 식별자
- 고유 레코드 ID (UUID)
- 타임스탬프 (ISO 8601)
- 위치 (위도/경도, 고도, 기준)
- 관찰자/센서 식별
- 품질 메타데이터 (검증 상태, 플래그, 신뢰도)

### 3.2.2 설계 원칙

```
데이터 형식 설계 원칙:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  1. JSON 기반                                                        │
│     ├─ 사람이 읽을 수 있는 텍스트 형식                                │
│     ├─ 모든 프로그래밍 언어에서 광범위하게 지원됨                      │
│     ├─ JSON 스키마로 쉽게 검증                                       │
│     └─ 웹 API 친화적                                                │
│                                                                     │
│  2. 자기 설명적                                                      │
│     ├─ 필드 이름이 명시적 ("temp"가 아닌 "temperature_celsius")      │
│     ├─ 단위가 필드 이름이나 메타데이터에 포함됨                        │
│     ├─ 각 레코드에 스키마 버전 포함                                   │
│     └─ 외부 표준 참조 (ENVO, QUDT)                                   │
│                                                                     │
│  3. 확장 가능                                                        │
│     ├─ 추가 필드 허용 ("선택적"으로 표시)                             │
│     ├─ 새 스키마 유형 추가 가능                                       │
│     ├─ 하위 호환 버전 관리                                           │
│     └─ 네임스페이스를 통한 사용자 정의 확장                            │
│                                                                     │
│  4. 검증 준비됨                                                      │
│     ├─ 필수 vs. 선택적 필드 명시적                                   │
│     ├─ 데이터 유형 지정 (숫자, 문자열, 열거형)                        │
│     ├─ 숫자 값에 대한 범위 제약                                       │
│     ├─ 범주형 데이터에 대한 제어 어휘                                 │
│     └─ 교차 필드 검증 규칙                                           │
│                                                                     │
│  5. 상호운용성 우선                                                   │
│     ├─ Darwin Core에 대한 매핑 제공됨                                │
│     ├─ 공간 데이터를 위한 GeoJSON과 호환                             │
│     ├─ ISO 표준 참조 (8601, 19115)                                  │
│     └─ 도메인 온톨로지와 정렬 (ENVO, QUDT)                           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2.3 예시: 종 관찰

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
    "location_name": "Discovery Park, Seattle"
  },

  "observer": {
    "id": "orcid:0000-0002-1825-0097",
    "name": "Jane Smith",
    "organization": "Seattle Audubon Society"
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
    "taxon_authority": "Catalogue of Life 2025"
  },

  "detection_method": "visual_survey",
  "occurrence_status": "present",
  "abundance": 2,
  "life_stage": "adult",
  "behavior": "나무 그루터기에 앉아 울고 있음",

  "quality": {
    "validation_status": "validated",
    "quality_flags": [],
    "confidence_level": 1.0
  }
}
```

---

## 3.3 Phase 2: API 인터페이스 사양

### 3.3.1 RESTful API 설계

**핵심 엔드포인트:**

```
GET    /v1/observations          # 관찰 쿼리
POST   /v1/observations          # 새 관찰 제출
GET    /v1/observations/{id}     # 특정 관찰 검색
GET    /v1/sensors               # 센서 나열
GET    /v1/sensors/{id}/data     # 센서 시계열 검색
GET    /v1/sites                 # 모니터링 사이트 쿼리
GET    /v1/datasets              # 데이터셋 발견
POST   /v1/bulk-query            # 비동기 대량 쿼리
GET    /v1/jobs/{id}             # 작업 상태 확인
```

**쿼리 매개변수:**
- `taxon`: 학명으로 필터링
- `start_date`, `end_date`: 날짜 범위
- `bbox`: 경계 상자 (minLon,minLat,maxLon,maxLat)
- `limit`, `offset`: 페이지네이션
- `format`: 응답 형식 (json, csv, geojson)

**인증:**
- 간단한 액세스를 위한 API 키
- 사용자 위임 액세스를 위한 OAuth 2.0
- 서비스 간 JWT 토큰

**속도 제한:**
- 익명: 100 요청/시간
- 인증됨: 1,000 요청/시간
- 프리미엄: 10,000 요청/시간

### 3.3.2 실시간 스트리밍

**WebSocket 프로토콜:**
```javascript
// 실시간 센서 스트림에 연결
const ws = new WebSocket('wss://api.example.org/stream');

ws.send(JSON.stringify({
  action: 'subscribe',
  sensors: ['WEATHER-001', 'WATER-QUALITY-042'],
  filters: { quality_min: 0.8 }
}));

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log(`${data.sensor_id}: ${data.value} ${data.unit}`);
};
```

**MQTT 토픽:**
```
sensors/SENSOR-001/data       # 센서 판독값
sensors/SENSOR-001/status     # 온라인/오프라인, 배터리
sensors/SENSOR-001/alerts     # 임계값 초과
observations/SITE-042/species # 새로운 종 탐지
```

### 3.3.3 대량 데이터 액세스

대용량 데이터셋(수백만 레코드)의 경우 비동기 쿼리:

```typescript
// 대량 쿼리 제출
const response = await fetch('/v1/bulk-query', {
  method: 'POST',
  body: JSON.stringify({
    type: 'observations',
    filters: {
      taxon: 'Ursus arctos',
      start_date: '2020-01-01',
      end_date: '2024-12-31',
      bbox: [-125, 40, -110, 50]
    },
    format: 'csv'
  })
});

const job = await response.json();
// 반환: { job_id: "abc123", status: "processing" }

// 완료를 위한 폴링
const result = await fetch(`/v1/jobs/${job.job_id}`);
// 완료 시: { status: "complete", download_url: "..." }
```

---

## 3.4 Phase 3: 프로토콜 사양

### 3.4.1 품질 보증 요구사항

**현장 QA/QC:**
- 현장 블랭크: 샘플의 5%
- 장비 블랭크: 이벤트 전후
- 현장 복제본: 샘플의 10%
- 양성 대조군: 탐지 방법용

**실험실 QA/QC:**
- 방법 블랭크: 각 분석 배치
- 보정 검증: 샘플 10개마다
- 스파이크 회수: 10% (수용: 75-125%)
- 중복 분석: 10% (RPD < 20%)
- 인증 표준 물질: 각 배치

**자동화된 검증:**
- 범위 검사 (물리적 한계)
- 변화율 감지
- 플랫라인 감지
- 이상치/스파이크 감지
- 교차 매개변수 일관성

### 3.4.2 보정 표준

센서 유형별 최소 보정 빈도:

| 센서 유형 | 간격 | 방법 |
|----------|-----|-----|
| 온도 | 6개월 | 빙점 검사 (0°C) |
| pH | 2주 | 2점 완충액 보정 |
| 용존 산소 | 1개월 | 수포화 공기 |
| 전도도 | 3개월 | 표준 용액 |
| 탁도 | 1개월 | 포르마진 표준 |
| 대기질 (PM) | 6개월 | 참조와 동일 위치 배치 |
| 유량계 | 1년 | 알려진 부피 방법 |

### 3.4.3 현장 샘플링 프로토콜

**지점 계수 프로토콜 (조류):**
1. 5-10분 동안 고정 위치
2. 지정된 반경 내의 모든 탐지 기록 (종종 50m)
3. 탐지 가능성 보정을 위한 거리 밴드 주목
4. 적절한 계절 (번식) 및 시간 (새벽)에 수행
5. 탐지 확률 추정을 위한 여러 방문 (3-5회)
6. 날씨, 소음, 관찰자 문서화

**수질 샘플링 표준 절차:**
1. 하류에서 샘플링 위치에 접근
2. 주변 물로 병을 3회 헹굼
3. 중간 하천, 중간 깊이에서 수집 (표면 및 퇴적물 피함)
4. 완전히 채움 (DO 샘플의 경우 헤드스페이스 없음)
5. 필요시 즉시 보존 (금속용 산, 유기물용 냉동)
6. 방수 마커로 라벨링 (사이트, 날짜, 시간, 이니셜)
7. 현장 매개변수 문서화 (현장에서 온도, pH, DO)
8. 보관 연속성 유지
9. 얼음 위에서 운송, 보관 시간 내에 처리

---

## 3.5 Phase 4: 시스템 통합 사양

### 3.5.1 GIS 플랫폼 통합

**QGIS 플러그인:**
- QGIS에 직접 WIA 데이터 로드
- 공간 범위로 쿼리 (현재 맵 뷰)
- 분류군, 품질, 날짜로 관찰 스타일링
- Shapefile, GeoPackage로 내보내기

**ArcGIS 통합:**
- WIA REST Feature Service 커넥터
- 실시간 대시보드 위젯
- ArcGIS Field Maps 데이터 수집
- 가져오기/내보내기를 위한 지오프로세싱 도구

**OGC 웹 서비스:**
- 벡터 데이터를 위한 WFS (Web Feature Service)
- 시각화를 위한 WMS (Web Map Service)
- 메타데이터 발견을 위한 CSW (Catalog Service)

### 3.5.2 보존 데이터베이스 통합

**GBIF (Global Biodiversity Information Facility):**
```typescript
// WIA에서 Darwin Core Archive로 내보내기
function exportToGBIF(wiaObservations) {
  return {
    'meta.xml': generateMetaXML(),
    'occurrence.txt': wiaObservations.map(obs => ({
      occurrenceID: obs.record_id,
      eventDate: obs.timestamp,
      decimalLatitude: obs.location.latitude,
      decimalLongitude: obs.location.longitude,
      scientificName: obs.taxon.scientific_name,
      individualCount: obs.abundance,
      samplingProtocol: obs.detection_method,
      // ... 추가 매핑
    })),
    'eml.xml': generateEML(metadata)
  };
}
```

**iNaturalist 양방향 동기화:**
- API를 통해 연구 등급 관찰 가져오기
- WIA 관찰을 iNaturalist로 내보내기
- 커뮤니티 ID 및 AI 제안 활용
- 분류학적 업데이트 동기화

**eBird 통합:**
- eBird 체크리스트를 WIA 관찰로 가져오기
- 관찰자 노력 메타데이터 유지
- 완전 vs. 불완전 체크리스트 상태 보존

### 3.5.3 클라우드 플랫폼 배포

**AWS 아키텍처:**
```
AWS의 WIA 생태계 모니터링:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐         ┌─────────────┐ │
│  │ IoT Core     │────────►│ Lambda       │────────►│ S3          │ │
│  │ (MQTT)       │         │ (검증)       │         │ (원시 데이터) │ │
│  └──────────────┘         └──────────────┘         └─────────────┘ │
│         │                         │                                │
│         │                         ▼                                │
│         │                 ┌──────────────┐                         │
│         │                 │ RDS/PostGIS  │                         │
│         │                 │ (검증됨)     │                         │
│         │                 └──────────────┘                         │
│         │                         │                                │
│         ▼                         ▼                                │
│  ┌──────────────┐         ┌──────────────┐                         │
│  │ SageMaker    │         │ API Gateway  │                         │
│  │ (ML 모델)    │         │ (REST/WS)    │                         │
│  └──────────────┘         └──────────────┘                         │
│                                   │                                │
│                                   ▼                                │
│                           ┌──────────────┐                         │
│                           │ CloudFront   │                         │
│                           │ (CDN)        │                         │
│                           └──────────────┘                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3.6 기존 표준과의 비교

### 3.6.1 WIA의 차별점

| 측면 | Darwin Core | NetCDF-CF | WIA Ecosystem |
|------|------------|-----------|--------------|
| **주요 초점** | 생물다양성 표본/관찰 | 기후/센서 데이터 | 통합 생태계 |
| **데이터 모델** | 플랫 테이블 | 다차원 배열 | JSON 계층 구조 |
| **품질 메타데이터** | 제한적 | 광범위 | 포괄적 |
| **API 사양** | 없음 (데이터 형식만) | 없음 | RESTful + 스트리밍 |
| **실시간 지원** | 아니요 | 아니요 | 예 (WebSocket, MQTT) |
| **센서 통합** | 아니요 | 예 | 예 |
| **종 관찰** | 예 | 아니요 | 예 |
| **수질/대기질** | 아니요 | 제한적 | 예 |
| **채택 장벽** | 낮음 | 중간 | 낮음-중간 |

### 3.6.2 보완적 관계

WIA는 기존 표준을 대체하지 않고 연결합니다:

```
통합 계층으로서의 WIA:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  레거시 및 특수 표준                                                  │
│  ├─ Darwin Core (GBIF, 박물관 컬렉션)                               │
│  ├─ NetCDF (기후 모델, 해양 데이터)                                  │
│  ├─ EML (LTER 메타데이터)                                           │
│  ├─ NEON 데이터 제품                                                 │
│  └─ eBird, iNaturalist 형식                                         │
│                                                                     │
│                        ⬇ (변환기 제공됨)                            │
│                                                                     │
│  ┌────────────────────────────────────────────────────────────┐    │
│  │                                                            │    │
│  │          WIA 생태계 모니터링 표준                            │    │
│  │                                                            │    │
│  │  통합 API · 공통 JSON 형식 · QA/QC 프로토콜                 │    │
│  │                                                            │    │
│  └────────────────────────────────────────────────────────────┘    │
│                                                                     │
│                        ⬇ (표준 인터페이스)                         │
│                                                                     │
│  현대 분석 및 시각화 도구                                             │
│  ├─ GIS 플랫폼 (ArcGIS, QGIS)                                      │
│  ├─ 통계 소프트웨어 (R, Python)                                     │
│  ├─ 클라우드 플랫폼 (AWS, Google Earth Engine)                     │
│  ├─ 의사 결정 지원 (Marxan, InVEST)                                │
│  └─ 사용자 정의 애플리케이션                                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3.7 채택 경로

### 3.7.1 최소 구현

**개별 연구자 또는 소규모 조직의 경우:**

```typescript
interface MinimalImplementation {
  phase1: {
    action: "WIA JSON 형식으로 데이터 내보내기";
    effort: "제공된 도구로 1-2일";
    benefit: "데이터가 발견 가능하고 상호운용 가능함";
  };

  phase2: {
    action: "선택적 - 실시간 공유하지 않으면 건너뛸 수 있음";
    effort: "해당 없음";
  };

  phase3: {
    action: "WIA 템플릿에 기존 QA/QC 문서화";
    effort: "반나절";
    benefit: "사용자에게 데이터 품질이 투명함";
  };

  phase4: {
    action: "선택적 - 시스템 통합하지 않으면 건너뛸 수 있음";
    effort: "해당 없음";
  };

  totalEffort: "2-3일";
  outcome: "출판 준비가 된 WIA 준수 데이터셋";
}
```

### 3.7.2 전체 구현

**모니터링 인프라를 구축하는 조직의 경우:**

```typescript
interface FullImplementation {
  phase1: {
    action: "WIA 형식 기반 데이터베이스 스키마 설계";
    effort: "1주";
    benefit: "처음부터 기본 WIA 지원";
  };

  phase2: {
    action: "API 엔드포인트, 인증, 속도 제한 구현";
    effort: "2-4주";
    benefit: "파트너를 위한 프로그래밍 방식 데이터 액세스";
  };

  phase3: {
    action: "QA/QC 파이프라인, 보정 추적 개발";
    effort: "2-3주";
    benefit: "자동화된 품질 관리, 수동 검토 감소";
  };

  phase4: {
    action: "GIS, 클라우드 플랫폼, 저장소에 대한 커넥터 구축";
    effort: "3-4주";
    benefit: "원활한 워크플로우, 수동 내보내기 감소";
  };

  totalEffort: "8-12주";
  outcome: "엔터프라이즈급 WIA 모니터링 플랫폼";
}
```

### 3.7.3 점진적 채택

조직은 단계를 점진적으로 채택할 수 있습니다:

```
1년차: Phase 1 구현 (데이터 형식)
  → 이득: 표준화된 내부 데이터, 더 쉬운 분석

2년차: Phase 3 추가 (프로토콜)
  → 이득: 향상된 데이터 품질, 문서화된 방법

3년차: Phase 2 구현 (API)
  → 이득: 파트너 액세스 가능, 수동 요청 감소

4년차: Phase 4 완료 (통합)
  → 이득: 전체 생태계 연결성
```

---

## 3.8 복습 질문

### 질문 1
WIA가 단일 모놀리식 사양 대신 4단계 아키텍처를 사용하는 이유를 설명하시오. 다양한 유형의 채택자에 대한 장점은 무엇인가?

### 질문 2
WIA 생태계 모니터링 표준을 Darwin Core와 비교하시오. 각각을 언제 사용하겠는가? 함께 사용할 수 있는가?

### 질문 3
연구 기지에 10년간의 조류 조사 데이터가 Excel 스프레드시트에 있습니다. 최소한의 노력으로 이 데이터를 WIA 준수하게 만드는 채택 계획을 설계하시오.

### 질문 4
RESTful API (Phase 2)와 MQTT 스트리밍 프로토콜의 차이를 설명하시오. 각각을 언제 사용하겠는가?

### 질문 5
조직이 Phase 1과 3만 구현하고 Phase 2와 4를 건너뛰고자 합니다. 이것이 유효한 접근방식인가? 무엇을 얻고 무엇을 잃게 되는가?

---

## 3.9 주요 요점

| 구성요소 | 목적 | 주요 기능 |
|---------|-----|---------|
| **Phase 1: 데이터 형식** | 데이터 구조 표준화 | 검증이 있는 JSON 스키마 |
| **Phase 2: API 인터페이스** | 프로그래밍 방식 액세스 활성화 | REST + WebSocket + MQTT |
| **Phase 3: 프로토콜** | 데이터 품질 보장 | QA/QC, 보정, 샘플링 방법 |
| **Phase 4: 통합** | 생태계 연결 | GIS, 데이터베이스, 클라우드 플랫폼 |

### 설계 원칙
- **JSON 기반**: 사람이 읽을 수 있고, 광범위하게 지원됨
- **자기 설명적**: 단위 및 메타데이터 포함
- **확장 가능**: 향후 추가를 위한 공간
- **검증 준비됨**: 자동화된 품질 검사
- **상호운용 가능**: 기존 표준과 호환

### 채택 유연성
- **최소**: 2-3일 (기존 데이터 내보내기)
- **전체**: 8-12주 (새 인프라)
- **점진적**: 단계적으로 단계 채택

### 다음 장 미리보기

제4장에서는 모든 관찰 및 측정 유형에 대한 완전한 JSON 스키마, 검증 규칙 및 예제를 포함하여 Phase 1(데이터 형식)에 대한 자세한 내용을 다룹니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
