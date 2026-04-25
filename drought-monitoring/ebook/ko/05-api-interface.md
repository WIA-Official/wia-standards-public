# 제5장: API 인터페이스 및 서비스

## 가뭄 데이터 접근 및 관리를 위한 RESTful 서비스

---

## 5.1 RESTful API 아키텍처 설계

### 설계 원칙

WIA-ENV-003 API는 지리공간 및 시간 데이터에 적합한 REST 아키텍처 원칙을 따릅니다:

| 원칙 | 구현 | 이점 |
|------|------|------|
| 리소스 지향 | 가뭄 데이터 엔티티에 매핑된 리소스 | 직관적인 URL 구조 |
| 무상태 | 서버 측 세션 상태 없음 | 확장성, 신뢰성 |
| 캐시 가능 | HTTP 캐싱 헤더 | 성능, 부하 감소 |
| 균일한 인터페이스 | 엔드포인트 전반에 일관된 패턴 | 사용 용이성 |
| 계층화 시스템 | 로드 밸런서, CDN 지원 | 확장성 |

### API 기본 URL 구조

```
프로덕션:    https://api.drought-monitor.kr/v1
스테이징:    https://staging-api.drought-monitor.kr/v1
개발:        https://dev-api.drought-monitor.kr/v1
```

### 리소스 계층

```
/v1
├── /indices
│   ├── /pdsi
│   ├── /spi
│   ├── /spei
│   ├── /soil-moisture
│   └── /ndvi
├── /locations
│   ├── /points
│   ├── /regions
│   └── /polygons
├── /timeseries
├── /alerts
│   ├── /subscriptions
│   └── /notifications
├── /forecasts
├── /maps
└── /admin
    ├── /health
    └── /status
```

### HTTP 메서드 사용

| 메서드 | 목적 | 멱등성 | 안전 |
|-------|------|-------|------|
| GET | 리소스 검색 | 예 | 예 |
| POST | 리소스 생성, 복잡한 쿼리 | 아니오 | 아니오 |
| PUT | 전체 리소스 업데이트 | 예 | 아니오 |
| PATCH | 부분 리소스 업데이트 | 아니오 | 아니오 |
| DELETE | 리소스 제거 | 예 | 아니오 |

### 응답 형식 표준

모든 API 응답은 일관된 엔벨로프 구조를 따릅니다:

```json
{
  "meta": {
    "request_id": "uuid-문자열",
    "timestamp": "2025-01-15T12:00:00Z",
    "api_version": "1.0.0",
    "standard_id": "WIA-ENV-003",
    "processing_time_ms": 145
  },
  "data": {
    // 응답 페이로드
  },
  "pagination": {
    "total_count": 1250,
    "page": 1,
    "page_size": 100,
    "total_pages": 13,
    "next": "https://api.../indices/pdsi?page=2",
    "prev": null
  },
  "links": {
    "self": "https://api.../indices/pdsi?location=KR-CB-001",
    "related": {
      "spi": "https://api.../indices/spi?location=KR-CB-001",
      "soil_moisture": "https://api.../indices/soil-moisture?location=KR-CB-001"
    }
  }
}
```

### 오류 응답 형식

```json
{
  "meta": {
    "request_id": "uuid-문자열",
    "timestamp": "2025-01-15T12:00:00Z"
  },
  "error": {
    "code": "INVALID_LOCATION",
    "message": "지정된 위치 ID가 존재하지 않습니다",
    "details": {
      "provided_value": "KR-XX-999",
      "valid_format": "KR-{시도}-{번호}"
    },
    "documentation_url": "https://docs.../errors/INVALID_LOCATION"
  }
}
```

---

## 5.2 가뭄 지수 쿼리 엔드포인트

### PDSI 엔드포인트

**엔드포인트:** `GET /v1/indices/pdsi`

**쿼리 매개변수:**

| 매개변수 | 타입 | 필수 | 설명 |
|---------|------|------|------|
| location | string | 예* | 위치 ID 또는 좌표 |
| lat | number | 예* | 위도 (-90 ~ 90) |
| lon | number | 예* | 경도 (-180 ~ 180) |
| date | string | 아니오 | 특정 날짜 (YYYY-MM-DD) |
| start_date | string | 아니오 | 기간 시작 날짜 |
| end_date | string | 아니오 | 기간 종료 날짜 |
| variant | string | 아니오 | PDSI, SC-PDSI, PHDI, PMDI |
| format | string | 아니오 | json, geojson, csv |

*location 또는 lat/lon 중 하나 필수

**요청 예시:**
```http
GET /v1/indices/pdsi?location=KR-CB-001&start_date=2024-01-01&end_date=2025-01-31
Accept: application/json
Authorization: Bearer {token}
```

**응답 예시:**
```json
{
  "meta": {
    "request_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
    "timestamp": "2025-01-15T12:00:00Z",
    "api_version": "1.0.0"
  },
  "data": {
    "index_type": "SC-PDSI",
    "location": {
      "location_id": "KR-CB-001",
      "name": "충청북도",
      "coordinates": {"latitude": 36.6, "longitude": 127.5}
    },
    "period": {
      "start_date": "2024-01-01",
      "end_date": "2025-01-31"
    },
    "values": [
      {
        "date": "2024-01-31",
        "value": -1.15,
        "classification": {"category": "D1", "label": "중간 가뭄"},
        "quality": {"flag": 0, "confidence": 0.94}
      },
      {
        "date": "2024-02-29",
        "value": -1.78,
        "classification": {"category": "D1", "label": "중간 가뭄"},
        "quality": {"flag": 0, "confidence": 0.93}
      }
    ],
    "statistics": {
      "mean": -2.05,
      "min": -3.25,
      "max": -0.75,
      "std_dev": 0.72
    }
  }
}
```

### SPI 엔드포인트

**엔드포인트:** `GET /v1/indices/spi`

**추가 쿼리 매개변수:**

| 매개변수 | 타입 | 필수 | 설명 |
|---------|------|------|------|
| time_scale | integer | 아니오 | 개월 수 (1-48), 기본값: 3 |
| distribution | string | 아니오 | gamma, pearson-III |
| baseline_start | integer | 아니오 | 기준선 시작 연도 |
| baseline_end | integer | 아니오 | 기준선 종료 연도 |

**다중 척도 쿼리 예시:**
```http
GET /v1/indices/spi?location=KR-CB-001&time_scale=1,3,6,12&date=2025-01-31
```

**응답:**
```json
{
  "data": {
    "index_type": "SPI",
    "location": {"location_id": "KR-CB-001"},
    "date": "2025-01-31",
    "multi_scale": [
      {"time_scale_months": 1, "value": 0.35, "classification": "거의 정상"},
      {"time_scale_months": 3, "value": -1.15, "classification": "적당히 건조"},
      {"time_scale_months": 6, "value": -1.72, "classification": "심하게 건조"},
      {"time_scale_months": 12, "value": -2.05, "classification": "극심하게 건조"}
    ]
  }
}
```

### 토양 수분 엔드포인트

**엔드포인트:** `GET /v1/indices/soil-moisture`

**추가 매개변수:**

| 매개변수 | 타입 | 설명 |
|---------|------|------|
| depth | string | 깊이 범위 (예: "0-10cm", "0-100cm") |
| variable | string | volumetric, percentile, anomaly |
| source | string | in_situ, satellite, model, blended |

---

## 5.3 지리공간 데이터 서비스

### 포인트 쿼리 서비스

**엔드포인트:** `GET /v1/locations/points/{point_id}`

**엔드포인트:** `POST /v1/locations/points/query`

```json
// 다중 포인트 쿼리를 위한 POST 본문
{
  "points": [
    {"id": "P1", "lat": 36.6, "lon": 127.5},
    {"id": "P2", "lat": 35.9, "lon": 128.6},
    {"id": "P3", "lat": 37.5, "lon": 127.0}
  ],
  "indices": ["pdsi", "spi-3", "soil_moisture"],
  "date": "2025-01-15"
}
```

### 지역 쿼리 서비스

**엔드포인트:** `GET /v1/locations/regions`

**사용 가능한 지역 유형:**

| 유형 | 예시 | 설명 |
|------|------|------|
| province | KR-CB | 시도 코드 |
| county | KR-CB-001 | 시군구 코드 |
| watershed | HAN-001 | 수계 유역 코드 |
| agricultural_district | AGR-CB-01 | 농업 지구 |
| custom | USER-001 | 사용자 정의 지역 |

**지역 통계 요청:**
```http
GET /v1/locations/regions/KR-CB/indices/pdsi?date=2025-01-31&include_statistics=true
```

**응답:**
```json
{
  "data": {
    "region": {
      "region_id": "KR-CB",
      "name": "충청북도",
      "type": "province",
      "area_km2": 7407
    },
    "index_type": "PDSI",
    "date": "2025-01-31",
    "spatial_statistics": {
      "mean": -2.25,
      "median": -2.08,
      "min": -3.85,
      "max": -0.55,
      "std_dev": 0.85,
      "area_in_drought": {
        "D0_percent": 92.5,
        "D1_percent": 75.2,
        "D2_percent": 42.3,
        "D3_percent": 10.5,
        "D4_percent": 0.0
      }
    }
  }
}
```

### 폴리곤 쿼리 서비스

**엔드포인트:** `POST /v1/locations/polygons/query`

```json
// 사용자 정의 폴리곤 쿼리를 위한 POST 본문
{
  "geometry": {
    "type": "Polygon",
    "coordinates": [[
      [127.0, 36.0],
      [127.0, 37.0],
      [128.0, 37.0],
      [128.0, 36.0],
      [127.0, 36.0]
    ]]
  },
  "indices": ["pdsi", "ndvi_anomaly"],
  "date": "2025-01-15",
  "statistics": ["mean", "min", "max", "percentiles"]
}
```

---

## 5.4 시계열 분석 API

### 역사적 시계열

**엔드포인트:** `GET /v1/timeseries`

**쿼리 매개변수:**

| 매개변수 | 타입 | 설명 |
|---------|------|------|
| location | string | 위치 식별자 |
| index | string | 지수 유형 |
| start_date | string | 시리즈 시작 날짜 |
| end_date | string | 시리즈 종료 날짜 |
| interval | string | daily, weekly, monthly |
| include_statistics | boolean | 요약 통계 포함 |
| include_trend | boolean | 추세 분석 포함 |

**응답:**
```json
{
  "data": {
    "timeseries": {
      "location_id": "KR-CB-001",
      "index_type": "PDSI",
      "interval": "monthly",
      "start_date": "2020-01-01",
      "end_date": "2025-01-31",
      "values": [
        {"date": "2020-01-31", "value": -0.75, "quality": 0},
        {"date": "2020-02-29", "value": -1.12, "quality": 0}
      ]
    },
    "statistics": {
      "count": 61,
      "mean": -1.68,
      "median": -1.55,
      "std_dev": 1.02,
      "min": {"value": -3.95, "date": "2022-08-31"},
      "max": {"value": 1.65, "date": "2020-05-31"}
    },
    "trend": {
      "method": "Mann-Kendall",
      "slope": -0.022,
      "p_value": 0.035,
      "significance": "유의함",
      "interpretation": "통계적으로 유의한 건조 추세"
    }
  }
}
```

### 이상 감지

**엔드포인트:** `GET /v1/timeseries/anomalies`

**응답:**
```json
{
  "data": {
    "anomalies_detected": [
      {
        "period": {"start": "2022-06-01", "end": "2022-09-30"},
        "type": "급속_가뭄",
        "severity": "극심",
        "metrics": {
          "pdsi_change": -3.0,
          "duration_months": 4,
          "percentile_rank": 2.5
        }
      }
    ],
    "drought_periods": [
      {
        "start_date": "2021-03-01",
        "end_date": "2023-02-28",
        "duration_months": 24,
        "peak_severity": -3.95,
        "peak_date": "2022-08-31",
        "classification": "다년_가뭄"
      }
    ]
  }
}
```

---

## 5.5 경보 및 알림 서비스

### 경보 임계값 구성

**엔드포인트:** `POST /v1/alerts/thresholds`

```json
{
  "threshold_id": "user-threshold-001",
  "name": "충청북도 심한 가뭄 경보",
  "location": {
    "type": "region",
    "region_id": "KR-CB"
  },
  "conditions": [
    {
      "index": "pdsi",
      "operator": "less_than",
      "value": -3.0,
      "duration_months": 2
    },
    {
      "index": "soil_moisture_percentile",
      "operator": "less_than",
      "value": 10,
      "conjunction": "AND"
    }
  ],
  "severity": "high",
  "active": true
}
```

### 경보 구독 관리

**엔드포인트:** `POST /v1/alerts/subscriptions`

```json
{
  "subscription_id": "sub-001",
  "threshold_ids": ["user-threshold-001", "user-threshold-002"],
  "notification_channels": [
    {
      "type": "email",
      "address": "alerts@example.com",
      "frequency": "immediate"
    },
    {
      "type": "webhook",
      "url": "https://example.com/drought-alerts",
      "method": "POST",
      "headers": {"Authorization": "Bearer xxx"}
    },
    {
      "type": "sms",
      "phone": "+82-10-1234-5678",
      "frequency": "daily_digest"
    }
  ],
  "active": true
}
```

### 경보 알림 형식

```json
{
  "notification": {
    "notification_id": "notif-abc123",
    "timestamp": "2025-01-15T14:30:00Z",
    "alert": {
      "threshold_id": "user-threshold-001",
      "threshold_name": "충청북도 심한 가뭄 경보",
      "triggered_conditions": [
        {
          "index": "pdsi",
          "current_value": -3.25,
          "threshold_value": -3.0,
          "condition_met": true
        }
      ]
    },
    "location": {
      "region_id": "KR-CB",
      "name": "충청북도"
    },
    "current_status": {
      "pdsi": -3.25,
      "spi_3": -1.75,
      "soil_moisture_percentile": 8,
      "drought_category": "D2"
    },
    "trend": {
      "direction": "악화",
      "rate": -0.12,
      "forecast": "지속적인 악화 예상"
    }
  }
}
```

---

## 5.6 인증 및 권한 부여

### 인증 방법

API는 여러 인증 메커니즘을 지원합니다:

| 방법 | 사용 사례 | 보안 수준 |
|------|----------|----------|
| API 키 | 간단한 통합, 개발 | 기본 |
| OAuth 2.0 | 사용자 대면 애플리케이션 | 표준 |
| JWT Bearer | 기계 간, 마이크로서비스 | 높음 |
| mTLS | 기업, 정부 | 매우 높음 |

### API 키 인증

```http
GET /v1/indices/pdsi?location=KR-CB-001
X-API-Key: your-api-key-here
```

### OAuth 2.0 흐름

```
인증 엔드포인트: https://auth.drought-monitor.kr/authorize
토큰 엔드포인트: https://auth.drought-monitor.kr/token
스코프: read:indices, read:alerts, write:alerts, admin
```

### JWT Bearer 토큰

```http
GET /v1/indices/pdsi?location=KR-CB-001
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 권한 부여 수준

| 계층 | 속도 제한 | 기능 | 비용 |
|------|---------|------|------|
| 공개 | 100 요청/일 | 기본 지수, 제한된 지역 | 무료 |
| 개발자 | 10,000 요청/일 | 모든 지수, 모든 지역 | 무료 |
| 전문가 | 100,000 요청/일 | + 경보, + 역사적 | 월 10만원 |
| 기업 | 무제한 | + SLA, + 지원, + 맞춤 | 문의 |

---

## 5.7 속도 제한 및 할당량 관리

### 속도 제한 헤더

모든 API 응답에는 속도 제한 정보가 포함됩니다:

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9523
X-RateLimit-Reset: 1705363200
X-RateLimit-Tier: developer
```

### 속도 제한 응답 (429)

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "API 속도 제한을 초과했습니다",
    "details": {
      "limit": 10000,
      "window": "day",
      "reset_at": "2025-01-16T00:00:00Z",
      "upgrade_url": "https://drought-monitor.kr/pricing"
    }
  }
}
```

### 할당량 관리 엔드포인트

**현재 사용량 확인:**
```http
GET /v1/admin/usage
Authorization: Bearer {token}
```

**응답:**
```json
{
  "data": {
    "tier": "developer",
    "period": {
      "start": "2025-01-01T00:00:00Z",
      "end": "2025-01-31T23:59:59Z"
    },
    "usage": {
      "api_calls": {
        "limit": 10000,
        "used": 4523,
        "remaining": 5477
      },
      "data_export_mb": {
        "limit": 1000,
        "used": 234.5,
        "remaining": 765.5
      },
      "alert_subscriptions": {
        "limit": 10,
        "used": 3,
        "remaining": 7
      }
    }
  }
}
```

---

## 5.8 SDK 구현 가이드라인

### 공식 SDK 구조

```
drought-monitoring-sdk/
├── src/
│   ├── client.ts           # 메인 클라이언트 클래스
│   ├── indices/
│   │   ├── pdsi.ts
│   │   ├── spi.ts
│   │   └── soil-moisture.ts
│   ├── locations/
│   │   ├── points.ts
│   │   ├── regions.ts
│   │   └── polygons.ts
│   ├── alerts/
│   │   └── subscriptions.ts
│   ├── types/
│   │   └── index.ts
│   └── utils/
│       ├── auth.ts
│       └── errors.ts
├── package.json
└── README.md
```

### TypeScript SDK 예시

```typescript
import { DroughtMonitorClient } from '@wia/drought-monitoring-sdk';

// 클라이언트 초기화
const client = new DroughtMonitorClient({
  apiKey: process.env.DROUGHT_API_KEY,
  baseUrl: 'https://api.drought-monitor.kr/v1',
  timeout: 30000,
  retryConfig: {
    maxRetries: 3,
    backoffMs: 1000
  }
});

// PDSI 쿼리
async function getPDSI() {
  try {
    const response = await client.indices.pdsi.get({
      location: 'KR-CB-001',
      startDate: '2024-01-01',
      endDate: '2025-01-31'
    });

    console.log(`평균 PDSI: ${response.statistics.mean}`);
    console.log(`분류: ${response.values[0].classification.label}`);

    return response;
  } catch (error) {
    if (error.code === 'RATE_LIMIT_EXCEEDED') {
      console.log(`속도 제한. 재시도 시간: ${error.details.reset_at}`);
    }
    throw error;
  }
}

// 경보 구독
async function subscribeToAlerts() {
  const subscription = await client.alerts.subscriptions.create({
    thresholdIds: ['threshold-001'],
    channels: [{
      type: 'webhook',
      url: 'https://myapp.com/drought-webhook',
      method: 'POST'
    }]
  });

  return subscription;
}
```

### Python SDK 예시

```python
from drought_monitoring import DroughtMonitorClient
from datetime import date

# 클라이언트 초기화
client = DroughtMonitorClient(
    api_key=os.environ['DROUGHT_API_KEY'],
    base_url='https://api.drought-monitor.kr/v1'
)

# PDSI 시계열 쿼리
pdsi_data = client.indices.pdsi.get(
    location='KR-CB-001',
    start_date=date(2024, 1, 1),
    end_date=date(2025, 1, 31)
)

# 데이터 접근
for value in pdsi_data.values:
    print(f"{value.date}: {value.value} ({value.classification.label})")

# 다중 척도 SPI 쿼리
spi_data = client.indices.spi.get(
    location='KR-CB-001',
    time_scales=[1, 3, 6, 12],
    date=date(2025, 1, 31)
)

# 폴리곤 쿼리
from shapely.geometry import Polygon
polygon = Polygon([
    (127.0, 36.0), (127.0, 37.0),
    (128.0, 37.0), (128.0, 36.0)
])

polygon_results = client.locations.polygons.query(
    geometry=polygon,
    indices=['pdsi', 'soil_moisture'],
    date=date(2025, 1, 15)
)
```

---

## 5.9 복습 문제 및 핵심 요점

### 복습 문제

1. **API 설계**: WIA-ENV-003 API가 RPC 스타일 엔드포인트 대신 리소스 지향 REST 설계를 사용하는 이유를 설명하세요. 가뭄 모니터링 애플리케이션에 어떤 이점이 있습니까?

2. **다중 척도 쿼리**: 사용자가 최근 강수량 회복(1개월 SPI)과 장기 가뭄 상태(12개월 SPI)를 모두 이해해야 합니다. 둘 다 효율적으로 검색하는 API 쿼리를 설계하세요.

3. **지리공간 쿼리**: 포인트 쿼리, 지역 쿼리 및 폴리곤 쿼리의 사용 사례를 비교하세요. 각각 언제 가장 적합합니까?

4. **경보 구성**: 급속 가뭄(급격한 발생 가뭄)을 감지하기 위한 경보 임계값 구성을 설계하세요. 어떤 지수, 임계값 및 시간 조건을 지정하시겠습니까?

5. **인증 선택**: 농장 관리 SaaS 애플리케이션이 수천 명의 농민을 대신하여 가뭄 데이터에 접근해야 합니다. 어떤 인증 방법이 가장 적합하며 그 이유는?

6. **속도 제한 전략**: 연구팀이 100개 위치에 대해 10년간의 일별 PDSI 데이터를 다운로드해야 합니다. 속도 제한을 피하면서 다운로드를 효율적으로 완료하기 위해 API 호출을 어떻게 구조화해야 합니까?

7. **오류 처리**: 일반적인 오류 시나리오(속도 제한, 인증 실패, 잘못된 위치, 네트워크 타임아웃)를 처리하는 SDK 메서드의 의사 코드를 작성하세요.

8. **SDK 설계**: 가뭄 모니터링 API용 SDK를 설계할 때 핵심 고려사항은 무엇입니까? SDK가 페이지네이션, 속도 제한 및 오류 복구를 어떻게 처리해야 합니까?

### 핵심 요점

1. **RESTful 설계**: API는 리소스 지향 URL, 표준 HTTP 메서드 및 일관된 응답 형식을 가진 REST 원칙을 따라 직관적인 통합을 가능하게 합니다.

2. **포괄적인 지수 커버리지**: 모든 주요 가뭄 지수(PDSI, SPI, SPEI, 토양 수분, NDVI)에 대한 엔드포인트가 있으며 사용자 정의를 위한 유연한 쿼리 매개변수가 있습니다.

3. **지리공간 유연성**: 포인트, 지역 및 폴리곤 쿼리 서비스는 필드 규모에서 대륙까지 다양한 공간 분석 요구를 지원합니다.

4. **시계열 분석**: 전용 엔드포인트가 가뭄 모니터링에 필수적인 역사적 분석, 추세 감지 및 이상 식별을 지원합니다.

5. **경보 인프라**: 임계값 구성, 구독 관리 및 다중 채널 알림은 사전 예방적 가뭄 대응을 가능하게 합니다.

6. **보안 계층**: 여러 인증 방법(API 키, OAuth, JWT, mTLS)이 다양한 보안 요구사항과 사용 사례를 수용합니다.

7. **공정한 사용 관리**: 계층화된 할당량과 속도 제한이 다양한 사용자 유형에 적절한 접근을 제공하면서 시스템 안정성을 보장합니다.

8. **SDK 지원**: TypeScript와 Python의 공식 SDK가 통합을 단순화하고 일반적인 패턴을 처리하며 API 복잡성을 추상화합니다.

9. **응답 일관성**: 메타데이터, 페이지네이션 및 링크가 포함된 표준 엔벨로프 구조가 모든 엔드포인트에서 예측 가능한 응답을 제공합니다.

10. **오류 투명성**: 코드, 메시지 및 문서 링크가 포함된 상세한 오류 응답이 효율적인 디버깅과 통합을 지원합니다.

---

## 장 요약

이 장은 WIA-ENV-003 가뭄 모니터링 표준의 API 인터페이스 사양을 상세히 설명했습니다. RESTful API 설계는 가뭄 지수, 지리공간 서비스, 시계열 분석 및 경보 기능에 대한 포괄적인 접근을 제공합니다.

API 아키텍처는 확립된 REST 원칙—리소스 지향, 무상태, 캐시 가능—을 지리공간 및 시간적 가뭄 데이터의 특정 요구사항에 맞게 적용합니다. 일관된 URL 패턴, 쿼리 매개변수 및 응답 형식이 직관적인 통합을 가능하게 합니다.

지수 쿼리 엔드포인트는 위치, 시간 및 지수 변형별로 유연한 필터링과 함께 모든 주요 가뭄 지수를 지원합니다. 지리공간 서비스는 단일 포인트에서 복잡한 폴리곤까지의 쿼리를 가능하게 하여 다양한 애플리케이션 요구를 지원합니다. 시계열 엔드포인트는 통계 분석 및 추세 감지와 함께 역사적 데이터 접근을 제공합니다.

경보 시스템은 구성 가능한 임계값, 구독 관리 및 다중 채널 알림을 통해 사전 예방적 가뭄 모니터링을 가능하게 합니다. 조직은 여러 지수와 시간적 요구사항을 결합하는 복잡한 경보 조건을 정의할 수 있습니다.

개발용 간단한 API 키에서 민감한 애플리케이션을 위한 기업급 mTLS까지 인증 및 권한 부여 지원이 다양합니다. 계층화된 할당량과 속도 제한이 시스템 성능을 유지하면서 공정한 접근을 보장합니다.

공식 SDK가 인증, 페이지네이션, 속도 제한 및 오류 복구를 처리하여 API 복잡성을 추상화합니다. TypeScript와 Python 예시가 일반적인 통합 패턴을 보여줍니다.

이러한 API 사양은 상호 운용 가능한 가뭄 모니터링 애플리케이션의 기반을 제공하여 전 세계 구현에서 일관된 데이터 접근을 가능하게 합니다.

---

**다음 장: [제6장: 프로토콜 및 알고리즘](06-protocol.md)**
