# 제4장: API 인터페이스

## 2단계: RESTful API, GraphQL 및 통합 서비스

### 연결된 생물다양성 데이터 시스템 구축

---

## 개요

WIA 생물다양성 지수 표준의 2단계는 생물다양성 데이터 분석을 위한 계산 서비스 및 API를 제공합니다. 1단계의 표준화된 데이터 형식을 기반으로, 이 장에서는 다양성 계산, 검증, 통합 및 분석을 위한 RESTful 및 GraphQL 인터페이스를 정의합니다.

---

## API 아키텍처

### 설계 원칙

WIA 생물다양성 API는 현대 API 설계 원칙을 따릅니다:

1. **RESTful**: 표준 HTTP 메서드를 사용하는 리소스 지향 URL
2. **GraphQL**: 복잡한 데이터 관계를 위한 유연한 쿼리
3. **실시간**: 라이브 데이터 피드를 위한 WebSocket 스트림
4. **버전 관리됨**: URL 기반 버전 관리 (v1, v2)
5. **문서화됨**: OpenAPI 3.0 사양

### 기본 구성

**기본 URL:** `https://api.biodiversity.wia.org/v1/`

**프로덕션 엔드포인트:**
| 서비스 | URL | 목적 |
|--------|-----|------|
| REST API | `api.biodiversity.wia.org` | CRUD 작업 |
| GraphQL | `api.biodiversity.wia.org/graphql` | 복잡한 쿼리 |
| WebSocket | `stream.biodiversity.wia.org` | 실시간 피드 |
| 인증 | `auth.biodiversity.wia.org` | 인증 |

---

## 인증

### OAuth 2.0 + JWT

WIA API는 인증을 위해 JWT 토큰과 함께 OAuth 2.0을 사용합니다.

**지원되는 부여 유형:**
- 클라이언트 자격 증명 (기계 대 기계)
- 인증 코드 (사용자 응용프로그램)
- 리프레시 토큰 (토큰 갱신)

### 클라이언트 자격 증명 흐름

```http
POST https://auth.biodiversity.wia.org/oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "read:occurrences write:occurrences calculate:indices"
}
```

**응답:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read:occurrences write:occurrences calculate:indices"
}
```

### 액세스 토큰 사용

Authorization 헤더에 액세스 토큰 포함:

```http
GET /v1/occurrences
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

---

## 속도 제한

### 티어 구조

| 티어 | 요청/시간 | 버스트/분 | 월간 한도 | 용도 |
|------|-----------|-----------|-----------|------|
| 무료 | 1,000 | 50 | 25,000 | 개인 연구자 |
| 연구 | 10,000 | 200 | 250,000 | 학술 기관 |
| 기업 | 100,000 | 1,000 | 무제한 | 정부 기관 |
| 커스텀 | 협상 가능 | 협상 가능 | 협상 가능 | 글로벌 이니셔티브 |

### 속도 제한 헤더

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1640995200
X-RateLimit-Tier: free
```

---

## RESTful 엔드포인트

### 출현 관리

**출현 목록 조회:**
```http
GET /v1/occurrences
```

**쿼리 매개변수:**

| 매개변수 | 유형 | 설명 | 예시 |
|----------|------|------|------|
| `species` | 문자열 | 학명 | `Panthera tigris` |
| `country` | 문자열 | ISO 3166-1 alpha-2 | `KR` |
| `start_date` | 문자열 | ISO 8601 날짜 | `2025-01-01` |
| `end_date` | 문자열 | ISO 8601 날짜 | `2025-12-31` |
| `bbox` | 문자열 | 경계 상자 | `-180,-90,180,90` |
| `limit` | 정수 | 페이지당 결과 (최대 1000) | `100` |
| `offset` | 정수 | 페이지네이션 오프셋 | `0` |

**예시 요청:**
```http
GET /v1/occurrences?species=Naemorhedus+caudatus&country=KR&year=2025&limit=50
Authorization: Bearer eyJhbG...
```

**예시 응답:**
```json
{
  "count": 45,
  "total": 45,
  "results": [
    {
      "occurrence_id": "OCC-2025-KR-001",
      "species": {
        "scientific_name": "Naemorhedus caudatus",
        "common_name": "산양",
        "iucn_status": "VU"
      },
      "location": {
        "latitude": 38.1195,
        "longitude": 128.4656,
        "country": "대한민국",
        "protected_area": "설악산 국립공원"
      },
      "temporal": {
        "observation_date": "2025-06-15T08:30:00Z"
      }
    }
  ]
}
```

### 다양성 지수 계산

**지수 계산:**
```http
POST /v1/indices/calculate
Content-Type: application/json
Authorization: Bearer eyJhbG...

{
  "dataset_id": "DS-KOREA-2025",
  "spatial_filter": {
    "type": "polygon",
    "coordinates": [[[127.5, 35.5], [128.5, 35.5], [128.5, 36.5], [127.5, 36.5], [127.5, 35.5]]]
  },
  "temporal_filter": {
    "start_date": "2025-01-01",
    "end_date": "2025-12-31"
  },
  "indices": ["shannon_diversity", "simpson_index", "species_richness", "chao1"],
  "bootstrap": {
    "enabled": true,
    "iterations": 1000,
    "confidence_level": 0.95
  }
}
```

**응답:**
```json
{
  "calculation_id": "CALC-2025-KR-001",
  "status": "completed",
  "execution_time_ms": 2847,
  "results": {
    "species_richness": {
      "observed": 156,
      "rarefied": 142.7,
      "ci_lower": 138.2,
      "ci_upper": 147.3
    },
    "shannon_diversity": {
      "value": 4.127,
      "ci_lower": 3.982,
      "ci_upper": 4.268
    },
    "simpson_index": {
      "dominance_d": 0.0234,
      "diversity_1_minus_d": 0.9766
    }
  }
}
```

### 데이터 검증

**단일 출현 검증:**
```http
POST /v1/validate/occurrence
Content-Type: application/json

{
  "occurrence_id": "TEST-001",
  "species": {
    "scientific_name": "Prionailurus bengalensis"
  },
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "country": "대한민국"
  },
  "temporal": {
    "observation_date": "2025-11-15T09:30:00Z"
  }
}
```

**검증 응답:**
```json
{
  "valid": true,
  "quality_score": 0.94,
  "checks": [
    {
      "check": "coordinate_validity",
      "status": "passed",
      "details": "좌표가 유효한 범위 내에 있음"
    },
    {
      "check": "country_match",
      "status": "passed",
      "details": "좌표가 대한민국 경계 내에 있음"
    },
    {
      "check": "species_range",
      "status": "passed",
      "details": "위치가 삵의 알려진 분포 범위 내에 있음"
    }
  ],
  "warnings": [],
  "errors": []
}
```

---

## GraphQL 인터페이스

### 엔드포인트

**URL:** `https://api.biodiversity.wia.org/graphql`
**메서드:** POST
**Content-Type:** `application/json`

### 예시 쿼리

**출현과 함께 종 조회:**
```graphql
query GetSpeciesWithOccurrences {
  species(scientificName: "Naemorhedus caudatus") {
    taxonId
    scientificName
    commonNames {
      language
      name
    }
    conservationStatus {
      iucnCategory
      populationTrend
    }
    occurrences(limit: 10, year: 2025) {
      totalCount
      edges {
        node {
          occurrenceId
          location {
            latitude
            longitude
            protectedArea {
              name
              iucnCategory
            }
          }
          observationDate
          individualCount
        }
      }
    }
  }
}
```

---

## WebSocket 스트림

### 연결

```javascript
const ws = new WebSocket('wss://stream.biodiversity.wia.org/v1/live');

ws.onopen = () => {
  // 인증
  ws.send(JSON.stringify({
    type: 'authenticate',
    token: 'eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('수신:', message);
};
```

### 채널 구독

**멸종위기종 탐지:**
```javascript
ws.send(JSON.stringify({
  action: 'subscribe',
  channels: ['endangered_detections'],
  filters: {
    region: 'East_Asia',
    iucn_categories: ['CR', 'EN']
  }
}));
```

---

## SDK 지원

### Python SDK

**설치:**
```bash
pip install wia-biodiversity
```

**사용:**
```python
from wia_biodiversity import BiodiversityAPI

# 클라이언트 초기화
client = BiodiversityAPI(api_key='your_api_key')

# 출현 쿼리
occurrences = client.occurrences.list(
    species='Naemorhedus caudatus',
    country='KR',
    year=2025,
    limit=100
)

for occ in occurrences:
    print(f"{occ.occurrence_id}: {occ.location.protected_area}")

# 다양성 지수 계산
results = client.indices.calculate(
    dataset_id='DS-KOREA-2025',
    indices=['shannon', 'simpson', 'chao1'],
    bootstrap_iterations=1000
)

print(f"Shannon 지수: {results.shannon_diversity.value}")
```

### TypeScript/JavaScript SDK

**설치:**
```bash
npm install @wia/biodiversity-sdk
```

**사용:**
```typescript
import { BiodiversityAPI } from '@wia/biodiversity-sdk';

const client = new BiodiversityAPI({ apiKey: 'your_api_key' });

// 출현 쿼리
const occurrences = await client.occurrences.list({
  species: 'Naemorhedus caudatus',
  country: 'KR',
  year: 2025
});

// 지수 계산
const indices = await client.indices.calculate({
  datasetId: 'DS-KOREA-2025',
  indices: ['shannon', 'simpson'],
  bootstrap: { iterations: 1000 }
});

console.log(`Shannon: ${indices.shannonDiversity.value}`);
```

### R 패키지

**설치:**
```r
install.packages("wiabiodiversity")
```

**사용:**
```r
library(wiabiodiversity)

# API 키 설정
wia_set_key("your_api_key")

# 출현 쿼리
occurrences <- wia_occurrences(
  species = "Naemorhedus caudatus",
  country = "KR",
  year = 2025
)

# 다양성 지수 계산
indices <- wia_diversity(
  dataset_id = "DS-KOREA-2025",
  indices = c("shannon", "simpson", "chao1"),
  bootstrap = 1000
)

# 희박화 곡선 플롯
wia_rarefaction_plot(indices)
```

---

## 오류 처리

### HTTP 상태 코드

| 코드 | 의미 | 설명 |
|------|------|------|
| 200 | OK | 성공 |
| 201 | Created | 리소스 생성됨 |
| 400 | Bad Request | 잘못된 요청 형식 |
| 401 | Unauthorized | 누락/유효하지 않은 토큰 |
| 403 | Forbidden | 권한 부족 |
| 404 | Not Found | 리소스를 찾을 수 없음 |
| 422 | Unprocessable Entity | 검증 실패 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 오류 |

### 오류 응답 형식

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "잘못된 학명 형식",
    "details": {
      "field": "species.scientific_name",
      "value": "panthera tigris",
      "expected": "대문자로 시작하는 속명 (예: 'Panthera tigris')"
    },
    "documentation_url": "https://docs.biodiversity.wia.org/errors/VALIDATION_ERROR"
  }
}
```

---

## 핵심 내용

1. **RESTful API**가 출현, 종 및 데이터셋에 대한 CRUD 작업 제공
2. **GraphQL 인터페이스**가 복잡한 데이터 관계를 위한 유연한 쿼리 가능
3. **WebSocket 스트림**이 멸종위기종 및 임계값 위반에 대한 실시간 경고 전달
4. **인증**이 OAuth 2.0과 JWT 토큰 및 범위 기반 권한 사용
5. **SDK**가 Python, TypeScript/JavaScript 및 R에서 사용 가능

## 복습 문제

1. WIA API는 어떤 인증 방법을 사용합니까?
2. 무료와 기업 간 속도 제한 티어는 어떻게 다릅니까?
3. 생물다양성 쿼리에 REST보다 GraphQL을 사용하는 장점은 무엇입니까?
4. 실시간 멸종위기종 탐지 경고를 어떻게 구독할 수 있습니까?
5. sf 패키지 통합을 위해 어떤 SDK를 사용하시겠습니까?

---

**다음 장 미리보기:** 5장에서는 3단계: 현장 프로토콜을 탐구하며, 조류, 포유류, 식생 및 환경DNA 수집을 위한 표준화된 조사 방법을 다룹니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 · 모든 생명을 보존
