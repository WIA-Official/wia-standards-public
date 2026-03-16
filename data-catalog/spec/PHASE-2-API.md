# PHASE 2: API Specification

**Standard:** WIA-DATA-007 (Data Catalog)
**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

이 문서는 Data Catalog RESTful API의 완전한 사양을 정의합니다. 데이터 자산, 메타데이터, 계보, 비즈니스 용어 등을 관리하는 API 엔드포인트를 제공합니다.

## Base URL

```
https://api.datacatalog.example.com/v1
```

## Authentication

모든 API 요청은 **Bearer Token** 인증을 사용합니다:

```http
Authorization: Bearer <access_token>
```

### OAuth 2.0 Token Endpoint

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=<client_id>&
client_secret=<client_secret>
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "catalog:read catalog:write"
}
```

## API Endpoints

### 1. Dataset Management

#### 1.1 List Datasets

```http
GET /datasets
```

**Query Parameters:**
- `page` (integer): 페이지 번호 (default: 1)
- `page_size` (integer): 페이지당 항목 수 (default: 20, max: 100)
- `type` (string): 데이터셋 타입 (TABLE, VIEW, FILE, STREAM)
- `platform` (string): 플랫폼 필터 (PostgreSQL, MySQL, BigQuery, etc.)
- `owner` (string): 소유자 이메일
- `tags` (string): 쉼표로 구분된 태그 목록
- `search` (string): 검색 쿼리
- `sort` (string): 정렬 필드 (name, created_at, updated_at)
- `order` (string): 정렬 순서 (asc, desc)

**Response:**
```json
{
  "data": [
    {
      "dataset_id": "550e8400-e29b-41d4-a716-446655440000",
      "name": "customers",
      "qualified_name": "prod_db.public.customers",
      "type": "TABLE",
      "description": "고객 정보를 저장하는 메인 테이블",
      "owner": {
        "email": "data-team@company.com",
        "name": "Data Team"
      },
      "platform": "PostgreSQL",
      "tags": ["customer-domain", "crm-team"],
      "created_at": "2025-01-15T09:00:00Z",
      "updated_at": "2025-12-20T14:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "page_size": 20,
    "total_items": 1247,
    "total_pages": 63
  }
}
```

#### 1.2 Get Dataset by ID

```http
GET /datasets/{dataset_id}
```

**Response:**
```json
{
  "dataset_id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "customers",
  "qualified_name": "prod_db.public.customers",
  "type": "TABLE",
  "description": "고객 정보를 저장하는 메인 테이블",
  "owner": {
    "type": "USER",
    "email": "data-team@company.com",
    "name": "Data Team"
  },
  "location": {
    "platform": "PostgreSQL",
    "host": "db.prod.example.com",
    "port": 5432,
    "database": "prod_db",
    "schema": "public",
    "table": "customers"
  },
  "properties": {
    "row_count": 500000,
    "size_bytes": 52428800,
    "partition_type": "RANGE",
    "partition_column": "registration_date"
  },
  "schema": {
    "columns": [
      {
        "column_id": "uuid",
        "name": "customer_id",
        "data_type": "BIGINT",
        "nullable": false,
        "description": "고객 고유 식별자",
        "classifications": ["IDENTIFIER"],
        "constraints": {
          "primary_key": true
        }
      },
      {
        "column_id": "uuid",
        "name": "email",
        "data_type": "VARCHAR(255)",
        "nullable": false,
        "description": "고객 이메일 주소",
        "classifications": ["PII"],
        "constraints": {
          "unique": true
        }
      }
    ]
  },
  "tags": ["customer-domain", "crm-team", "high-criticality"],
  "classifications": ["PII", "BUSINESS"],
  "quality_score": 0.95,
  "created_at": "2025-01-15T09:00:00Z",
  "updated_at": "2025-12-20T14:30:00Z",
  "last_accessed_at": "2025-12-26T08:15:00Z"
}
```

#### 1.3 Create Dataset

```http
POST /datasets
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "new_dataset",
  "qualified_name": "prod_db.public.new_dataset",
  "type": "TABLE",
  "description": "새로운 데이터셋",
  "owner": {
    "email": "data-team@company.com"
  },
  "location": {
    "platform": "PostgreSQL",
    "database": "prod_db",
    "schema": "public",
    "table": "new_dataset"
  },
  "tags": ["new-domain"]
}
```

**Response:**
```json
{
  "dataset_id": "new-uuid",
  "name": "new_dataset",
  "created_at": "2025-12-26T10:00:00Z",
  "message": "Dataset created successfully"
}
```

#### 1.4 Update Dataset

```http
PATCH /datasets/{dataset_id}
Content-Type: application/json
```

**Request Body:**
```json
{
  "description": "업데이트된 설명",
  "tags": ["updated-tag"],
  "owner": {
    "email": "new-owner@company.com"
  }
}
```

**Response:**
```json
{
  "dataset_id": "550e8400-e29b-41d4-a716-446655440000",
  "updated_at": "2025-12-26T10:30:00Z",
  "message": "Dataset updated successfully"
}
```

#### 1.5 Delete Dataset

```http
DELETE /datasets/{dataset_id}
```

**Response:**
```json
{
  "dataset_id": "550e8400-e29b-41d4-a716-446655440000",
  "deleted_at": "2025-12-26T11:00:00Z",
  "message": "Dataset deleted successfully"
}
```

### 2. Search API

#### 2.1 Global Search

```http
POST /search
Content-Type: application/json
```

**Request Body:**
```json
{
  "query": "customer email",
  "filters": {
    "types": ["TABLE", "COLUMN"],
    "platforms": ["PostgreSQL", "BigQuery"],
    "tags": ["customer-domain"],
    "classifications": ["PII"]
  },
  "page": 1,
  "page_size": 20,
  "highlight": true
}
```

**Response:**
```json
{
  "query": "customer email",
  "results": [
    {
      "type": "COLUMN",
      "dataset_id": "uuid",
      "column_id": "uuid",
      "qualified_name": "prod_db.public.customers.email",
      "name": "email",
      "description": "고객 <em>이메일</em> 주소",
      "score": 0.95,
      "highlights": {
        "description": "고객 <em>이메일</em> 주소",
        "name": "<em>email</em>"
      }
    },
    {
      "type": "TABLE",
      "dataset_id": "uuid",
      "qualified_name": "prod_db.public.customers",
      "name": "customers",
      "description": "<em>고객</em> 정보를 저장하는 메인 테이블",
      "score": 0.87,
      "highlights": {
        "description": "<em>고객</em> 정보를 저장하는 메인 테이블"
      }
    }
  ],
  "total": 45,
  "page": 1,
  "page_size": 20,
  "took_ms": 23
}
```

#### 2.2 Autocomplete

```http
GET /search/autocomplete?q=cust&limit=10
```

**Response:**
```json
{
  "suggestions": [
    {
      "text": "customers",
      "type": "TABLE",
      "qualified_name": "prod_db.public.customers"
    },
    {
      "text": "customer_id",
      "type": "COLUMN",
      "qualified_name": "prod_db.public.orders.customer_id"
    },
    {
      "text": "customer_segment",
      "type": "COLUMN",
      "qualified_name": "prod_db.public.customers.customer_segment"
    }
  ]
}
```

### 3. Data Lineage API

#### 3.1 Get Upstream Lineage

```http
GET /datasets/{dataset_id}/lineage/upstream?depth=3
```

**Response:**
```json
{
  "dataset_id": "target-uuid",
  "qualified_name": "analytics_db.marts.customer_analytics",
  "lineage": {
    "nodes": [
      {
        "dataset_id": "source-uuid-1",
        "qualified_name": "prod_db.public.customers",
        "type": "TABLE",
        "level": 1
      },
      {
        "dataset_id": "source-uuid-2",
        "qualified_name": "prod_db.sales.orders",
        "type": "TABLE",
        "level": 1
      },
      {
        "dataset_id": "staging-uuid",
        "qualified_name": "staging_db.stg_customer_orders",
        "type": "TABLE",
        "level": 2
      }
    ],
    "edges": [
      {
        "from": "source-uuid-1",
        "to": "staging-uuid",
        "process": {
          "name": "customer_etl",
          "type": "SPARK_JOB"
        }
      },
      {
        "from": "source-uuid-2",
        "to": "staging-uuid",
        "process": {
          "name": "orders_etl",
          "type": "SPARK_JOB"
        }
      },
      {
        "from": "staging-uuid",
        "to": "target-uuid",
        "process": {
          "name": "analytics_pipeline",
          "type": "DBT_MODEL"
        }
      }
    ],
    "depth": 2,
    "total_upstream": 3
  }
}
```

#### 3.2 Get Downstream Lineage

```http
GET /datasets/{dataset_id}/lineage/downstream?depth=3
```

**Response:**
```json
{
  "dataset_id": "source-uuid",
  "qualified_name": "prod_db.public.customers",
  "lineage": {
    "nodes": [
      {
        "dataset_id": "target-uuid-1",
        "qualified_name": "analytics_db.marts.customer_analytics",
        "type": "TABLE",
        "level": 1
      },
      {
        "dataset_id": "target-uuid-2",
        "qualified_name": "reporting_db.customer_reports",
        "type": "VIEW",
        "level": 2
      }
    ],
    "edges": [
      {
        "from": "source-uuid",
        "to": "target-uuid-1",
        "process": {
          "name": "analytics_pipeline",
          "type": "DBT_MODEL"
        }
      },
      {
        "from": "target-uuid-1",
        "to": "target-uuid-2",
        "process": {
          "name": "reporting_view",
          "type": "SQL_VIEW"
        }
      }
    ],
    "depth": 2,
    "total_downstream": 5
  }
}
```

#### 3.3 Get Column-Level Lineage

```http
GET /datasets/{dataset_id}/columns/{column_name}/lineage
```

**Response:**
```json
{
  "column": {
    "column_id": "uuid",
    "qualified_name": "analytics_db.marts.customer_analytics.total_spent",
    "data_type": "DECIMAL(10,2)"
  },
  "upstream": [
    {
      "column_id": "uuid",
      "qualified_name": "prod_db.sales.orders.total_amount",
      "data_type": "DECIMAL(10,2)",
      "transformation": "SUM(total_amount)"
    },
    {
      "column_id": "uuid",
      "qualified_name": "prod_db.sales.orders.discount_amount",
      "data_type": "DECIMAL(10,2)",
      "transformation": "total_amount - discount_amount"
    }
  ]
}
```

### 4. Business Glossary API

#### 4.1 List Glossary Terms

```http
GET /glossary/terms?domain=Customer&status=APPROVED
```

**Response:**
```json
{
  "data": [
    {
      "term_id": "uuid",
      "name": "Customer Lifetime Value",
      "abbreviation": "LTV",
      "definition": "고객이 기업과의 관계 동안 창출할 것으로 예상되는 총 수익",
      "domain": "Customer Analytics",
      "status": "APPROVED",
      "created_at": "2025-11-01T10:00:00Z"
    }
  ],
  "total": 156
}
```

#### 4.2 Get Glossary Term

```http
GET /glossary/terms/{term_id}
```

**Response:**
```json
{
  "term_id": "uuid",
  "name": "Customer Lifetime Value",
  "abbreviation": "LTV",
  "qualified_name": "business.customer.ltv",
  "definition": "고객이 기업과의 관계 동안 창출할 것으로 예상되는 총 수익",
  "long_description": "Customer Lifetime Value는 마케팅 투자 결정과 고객 세그먼테이션의 핵심 지표입니다.",
  "formula": "(평균 주문 금액) × (연간 구매 빈도) × (고객 유지 기간)",
  "business_purpose": [
    "마케팅 ROI 측정",
    "고객 세그먼테이션"
  ],
  "related_terms": [
    {
      "term_id": "uuid",
      "name": "Customer Acquisition Cost"
    }
  ],
  "related_datasets": [
    {
      "dataset_id": "uuid",
      "qualified_name": "prod_db.public.customers",
      "column": "lifetime_value"
    }
  ],
  "domain": "Customer Analytics",
  "owner": {
    "email": "marketing-analytics@company.com",
    "name": "Marketing Analytics Team"
  },
  "status": "APPROVED",
  "created_at": "2025-11-01T10:00:00Z",
  "approved_at": "2025-11-05T14:30:00Z"
}
```

#### 4.3 Create Glossary Term

```http
POST /glossary/terms
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Monthly Recurring Revenue",
  "abbreviation": "MRR",
  "definition": "매월 반복적으로 발생하는 예측 가능한 수익",
  "domain": "Finance",
  "owner": {
    "email": "finance@company.com"
  }
}
```

### 5. Data Quality API

#### 5.1 Get Quality Metrics

```http
GET /datasets/{dataset_id}/quality
```

**Response:**
```json
{
  "dataset_id": "uuid",
  "overall_score": 0.95,
  "metrics": {
    "completeness": 0.98,
    "accuracy": 0.94,
    "consistency": 0.96,
    "timeliness": 0.92
  },
  "column_quality": [
    {
      "column_name": "email",
      "score": 0.95,
      "issues": [
        {
          "type": "FORMAT_ERROR",
          "count": 25000,
          "percentage": 5.0
        }
      ]
    }
  ],
  "last_assessed": "2025-12-26T06:00:00Z"
}
```

#### 5.2 Run Quality Check

```http
POST /datasets/{dataset_id}/quality/check
Content-Type: application/json
```

**Request Body:**
```json
{
  "rules": [
    "email_format_validation",
    "null_check",
    "uniqueness_check"
  ]
}
```

**Response:**
```json
{
  "check_id": "uuid",
  "status": "RUNNING",
  "started_at": "2025-12-26T10:00:00Z",
  "estimated_completion": "2025-12-26T10:05:00Z"
}
```

### 6. Tags & Classification API

#### 6.1 Add Tags

```http
POST /datasets/{dataset_id}/tags
Content-Type: application/json
```

**Request Body:**
```json
{
  "tags": ["new-tag", "important"]
}
```

#### 6.2 Add Classification

```http
POST /datasets/{dataset_id}/classifications
Content-Type: application/json
```

**Request Body:**
```json
{
  "classifications": ["PII", "FINANCIAL"]
}
```

#### 6.3 Auto-Classify

```http
POST /datasets/{dataset_id}/classifications/auto
```

**Response:**
```json
{
  "dataset_id": "uuid",
  "detected_classifications": [
    {
      "classification": "PII",
      "confidence": 0.95,
      "columns": ["email", "phone_number", "full_name"]
    },
    {
      "classification": "FINANCIAL",
      "confidence": 0.88,
      "columns": ["lifetime_value"]
    }
  ]
}
```

### 7. Statistics API

#### 7.1 Get Catalog Statistics

```http
GET /statistics
```

**Response:**
```json
{
  "total_datasets": 1247,
  "total_tables": 8934,
  "total_columns": 124567,
  "total_glossary_terms": 156,
  "platforms": {
    "PostgreSQL": 456,
    "MySQL": 234,
    "BigQuery": 178,
    "Snowflake": 123
  },
  "documentation_coverage": 0.89,
  "quality_score_avg": 0.87,
  "last_updated": "2025-12-26T10:00:00Z"
}
```

## Error Handling

### Error Response Format

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Dataset not found",
    "details": {
      "dataset_id": "invalid-uuid"
    },
    "timestamp": "2025-12-26T10:00:00Z",
    "request_id": "req-uuid"
  }
}
```

### Error Codes

| HTTP Status | Error Code | Description |
|------------|------------|-------------|
| 400 | INVALID_REQUEST | 잘못된 요청 파라미터 |
| 401 | UNAUTHORIZED | 인증 실패 |
| 403 | FORBIDDEN | 권한 없음 |
| 404 | RESOURCE_NOT_FOUND | 리소스를 찾을 수 없음 |
| 409 | CONFLICT | 리소스 충돌 |
| 422 | VALIDATION_ERROR | 유효성 검사 실패 |
| 429 | RATE_LIMIT_EXCEEDED | 요청 한도 초과 |
| 500 | INTERNAL_ERROR | 내부 서버 오류 |
| 503 | SERVICE_UNAVAILABLE | 서비스 이용 불가 |

## Rate Limiting

- **Rate Limit:** 1000 requests per hour
- **Burst:** 100 requests per minute

Response Headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1640520000
```

## Pagination

모든 리스트 API는 커서 기반 페이지네이션을 지원합니다:

```http
GET /datasets?cursor=eyJpZCI6MTIzfQ&limit=20
```

**Response:**
```json
{
  "data": [...],
  "pagination": {
    "next_cursor": "eyJpZCI6MTQzfQ",
    "has_more": true
  }
}
```

## Webhooks

### Register Webhook

```http
POST /webhooks
Content-Type: application/json
```

**Request Body:**
```json
{
  "url": "https://example.com/webhook",
  "events": [
    "dataset.created",
    "dataset.updated",
    "dataset.deleted",
    "lineage.updated"
  ],
  "secret": "webhook-secret"
}
```

### Webhook Payload

```json
{
  "event": "dataset.updated",
  "timestamp": "2025-12-26T10:00:00Z",
  "data": {
    "dataset_id": "uuid",
    "changes": {
      "description": {
        "old": "Old description",
        "new": "New description"
      }
    }
  },
  "signature": "sha256=..."
}
```

---

**Next Phase:** [PHASE-3-PROTOCOL.md](./PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
