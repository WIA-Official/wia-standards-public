# PHASE 1: Data Format Specification

**Standard:** WIA-DATA-007 (Data Catalog)
**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

이 문서는 Data Catalog 시스템에서 사용되는 데이터 포맷을 정의합니다. 메타데이터, 데이터 자산, 계보, 비즈니스 용어 등의 표준 포맷을 제공합니다.

## Core Data Models

### 1. Dataset Metadata

```json
{
  "dataset_id": "uuid-v4",
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
  "tags": [
    "customer-domain",
    "crm-team",
    "high-criticality",
    "gdpr-compliant",
    "gold-quality"
  ],
  "classifications": [
    "PII",
    "BUSINESS"
  ],
  "created_at": "2025-01-15T09:00:00Z",
  "updated_at": "2025-12-20T14:30:00Z",
  "last_accessed_at": "2025-12-26T08:15:00Z"
}
```

### 2. Column Schema

```json
{
  "column_id": "uuid-v4",
  "name": "email",
  "qualified_name": "prod_db.public.customers.email",
  "dataset_id": "parent-dataset-uuid",
  "data_type": "VARCHAR(255)",
  "nullable": false,
  "description": "고객 이메일 주소",
  "classifications": ["PII", "CONTACT_INFO"],
  "tags": ["gdpr", "required"],
  "constraints": {
    "primary_key": false,
    "unique": true,
    "foreign_key": null,
    "default_value": null
  },
  "statistics": {
    "distinct_count": 498500,
    "null_count": 0,
    "null_percentage": 0.0,
    "min_length": 8,
    "max_length": 120,
    "avg_length": 24.5
  },
  "quality_score": 0.95,
  "sample_values": [
    "john.doe@example.com",
    "jane.smith@company.com",
    "alice.jones@domain.com"
  ]
}
```

### 3. Data Lineage

```json
{
  "lineage_id": "uuid-v4",
  "source": {
    "dataset_id": "source-uuid",
    "qualified_name": "prod_db.public.customers",
    "type": "TABLE"
  },
  "target": {
    "dataset_id": "target-uuid",
    "qualified_name": "analytics_db.marts.customer_analytics",
    "type": "TABLE"
  },
  "process": {
    "process_id": "process-uuid",
    "name": "customer_etl_pipeline",
    "type": "SPARK_JOB",
    "description": "고객 데이터 변환 및 적재 파이프라인",
    "schedule": "0 2 * * *",
    "owner": "data-engineering@company.com"
  },
  "transformations": [
    {
      "step": 1,
      "type": "FILTER",
      "description": "활성 고객만 필터링",
      "logic": "WHERE status = 'ACTIVE'"
    },
    {
      "step": 2,
      "type": "TRANSFORM",
      "description": "고객 세그먼트 계산",
      "logic": "CASE WHEN lifetime_value > 10000 THEN 'VIP' ELSE 'Regular' END"
    },
    {
      "step": 3,
      "type": "JOIN",
      "description": "주문 데이터와 조인",
      "logic": "LEFT JOIN orders ON customers.customer_id = orders.customer_id"
    },
    {
      "step": 4,
      "type": "AGGREGATE",
      "description": "고객별 통계 계산",
      "logic": "GROUP BY customer_id, SUM(order_amount) as total_spent"
    }
  ],
  "dependency_depth": 3,
  "upstream_count": 2,
  "downstream_count": 5,
  "created_at": "2025-12-26T10:00:00Z"
}
```

### 4. Business Glossary Term

```json
{
  "term_id": "uuid-v4",
  "name": "Customer Lifetime Value",
  "abbreviation": "LTV",
  "qualified_name": "business.customer.ltv",
  "definition": "고객이 기업과의 관계 동안 창출할 것으로 예상되는 총 수익",
  "long_description": "Customer Lifetime Value는 마케팅 투자 결정과 고객 세그먼테이션의 핵심 지표입니다. 평균 주문 금액, 구매 빈도, 고객 유지 기간을 기반으로 계산됩니다.",
  "formula": "(평균 주문 금액) × (연간 구매 빈도) × (고객 유지 기간)",
  "business_purpose": [
    "마케팅 ROI 측정",
    "고객 세그먼테이션",
    "CAC(고객 획득 비용) 대비 효율성 평가"
  ],
  "related_terms": [
    "customer_acquisition_cost",
    "customer_retention_rate",
    "average_order_value"
  ],
  "related_datasets": [
    {
      "dataset_id": "uuid",
      "qualified_name": "prod_db.public.customers",
      "column": "lifetime_value"
    },
    {
      "dataset_id": "uuid",
      "qualified_name": "prod_db.sales.orders",
      "column": "total_amount"
    }
  ],
  "domain": "Customer Analytics",
  "owner": {
    "email": "marketing-analytics@company.com",
    "name": "Marketing Analytics Team"
  },
  "status": "APPROVED",
  "examples": [
    {
      "scenario": "VIP 고객",
      "calculation": "$500 × 12 × 5 = $30,000"
    },
    {
      "scenario": "일반 고객",
      "calculation": "$100 × 6 × 3 = $1,800"
    }
  ],
  "created_at": "2025-11-01T10:00:00Z",
  "approved_at": "2025-11-05T14:30:00Z",
  "updated_at": "2025-12-15T09:00:00Z"
}
```

### 5. Data Classification

```json
{
  "classification_id": "uuid-v4",
  "name": "PII",
  "full_name": "Personally Identifiable Information",
  "description": "개인 식별 가능 정보",
  "category": "DATA_PRIVACY",
  "security_level": "HIGH",
  "compliance_frameworks": [
    "GDPR",
    "CCPA",
    "PIPEDA"
  ],
  "attributes": {
    "encryption_required": true,
    "access_logging_required": true,
    "retention_period_days": 2555,
    "anonymization_required": true
  },
  "detection_rules": [
    {
      "rule_type": "COLUMN_NAME_PATTERN",
      "pattern": "(email|phone|ssn|address|name|birth_date)",
      "confidence": 0.9
    },
    {
      "rule_type": "DATA_PATTERN",
      "pattern": "\\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Z|a-z]{2,}\\b",
      "description": "Email pattern",
      "confidence": 0.95
    }
  ],
  "actions": [
    {
      "action_type": "ENCRYPT",
      "description": "데이터 암호화 적용"
    },
    {
      "action_type": "MASK",
      "description": "비프로덕션 환경에서 마스킹"
    },
    {
      "action_type": "AUDIT",
      "description": "접근 로그 기록"
    }
  ],
  "created_at": "2025-01-01T00:00:00Z",
  "updated_at": "2025-12-01T10:00:00Z"
}
```

### 6. Data Quality Rule

```json
{
  "rule_id": "uuid-v4",
  "name": "email_format_validation",
  "type": "FORMAT_CHECK",
  "description": "이메일 주소 형식 유효성 검증",
  "dataset_id": "dataset-uuid",
  "column_name": "email",
  "rule_definition": {
    "validation_type": "REGEX",
    "pattern": "^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\\.[a-zA-Z]{2,}$",
    "error_threshold": 0.05,
    "warning_threshold": 0.02
  },
  "severity": "HIGH",
  "enabled": true,
  "schedule": "0 */6 * * *",
  "last_run": {
    "timestamp": "2025-12-26T06:00:00Z",
    "total_records": 500000,
    "passed": 475000,
    "failed": 25000,
    "pass_rate": 0.95,
    "status": "PASSED"
  },
  "created_by": "data-quality@company.com",
  "created_at": "2025-10-01T10:00:00Z"
}
```

### 7. Access Control

```json
{
  "policy_id": "uuid-v4",
  "dataset_id": "dataset-uuid",
  "resource_type": "TABLE",
  "qualified_name": "prod_db.public.customers",
  "permissions": [
    {
      "principal_type": "USER",
      "principal_id": "user@company.com",
      "access_level": "READ",
      "granted_at": "2025-11-01T10:00:00Z",
      "granted_by": "admin@company.com",
      "expires_at": null
    },
    {
      "principal_type": "GROUP",
      "principal_id": "data-analysts",
      "access_level": "READ",
      "granted_at": "2025-01-01T00:00:00Z",
      "granted_by": "data-governance@company.com",
      "expires_at": null
    },
    {
      "principal_type": "GROUP",
      "principal_id": "data-engineers",
      "access_level": "WRITE",
      "granted_at": "2025-01-01T00:00:00Z",
      "granted_by": "data-governance@company.com",
      "expires_at": null
    }
  ],
  "row_level_filters": [
    {
      "principal_id": "regional-managers",
      "filter": "region = current_user_region()"
    }
  ],
  "column_masking": [
    {
      "column_name": "email",
      "principal_id": "data-analysts",
      "masking_type": "PARTIAL",
      "mask_pattern": "***@***"
    }
  ]
}
```

## Data Format Standards

### Timestamp Format

모든 타임스탬프는 **ISO 8601** 형식을 따릅니다:
- Format: `YYYY-MM-DDTHH:mm:ss.sssZ`
- Timezone: UTC
- Example: `2025-12-26T10:30:45.123Z`

### UUID Format

모든 ID는 **UUID v4** 형식을 사용합니다:
- Format: `xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx`
- Example: `550e8400-e29b-41d4-a716-446655440000`

### Qualified Name Convention

데이터 자산의 전체 경로명:
- Pattern: `{platform}.{database}.{schema}.{table}.{column}`
- Example: `postgresql.prod_db.public.customers.email`

## Validation Rules

### Required Fields

모든 데이터셋은 다음 필드를 필수로 포함해야 합니다:
- `dataset_id`
- `name`
- `qualified_name`
- `type`
- `owner`
- `created_at`
- `updated_at`

### Data Type Mapping

| Source Type | Catalog Type | Description |
|------------|--------------|-------------|
| VARCHAR, TEXT | STRING | 문자열 |
| INT, BIGINT | INTEGER | 정수 |
| DECIMAL, NUMERIC | DECIMAL | 고정소수점 |
| FLOAT, DOUBLE | FLOAT | 부동소수점 |
| DATE | DATE | 날짜 |
| TIMESTAMP | TIMESTAMP | 타임스탬프 |
| BOOLEAN | BOOLEAN | 불린 |
| JSON, JSONB | JSON | JSON 객체 |
| ARRAY | ARRAY | 배열 |

## Storage Format

### File Format: JSON Lines (.jsonl)

메타데이터 내보내기는 JSON Lines 형식을 사용합니다:

```jsonl
{"dataset_id":"uuid-1","name":"customers","type":"TABLE"}
{"dataset_id":"uuid-2","name":"orders","type":"TABLE"}
{"dataset_id":"uuid-3","name":"products","type":"TABLE"}
```

### Compression

대용량 데이터 전송 시 **gzip** 압축을 사용합니다:
- Extension: `.jsonl.gz`
- Compression level: 6 (default)

## Versioning

메타데이터 버전 관리:

```json
{
  "version_id": "uuid-v4",
  "dataset_id": "dataset-uuid",
  "version": 3,
  "changes": [
    {
      "field": "description",
      "old_value": "Customer data",
      "new_value": "고객 정보를 저장하는 메인 테이블",
      "changed_by": "user@company.com",
      "changed_at": "2025-12-26T10:00:00Z"
    }
  ],
  "created_at": "2025-12-26T10:00:00Z"
}
```

## Best Practices

### 1. Naming Conventions

- **Dataset names**: lowercase, underscores (e.g., `customer_orders`)
- **Column names**: lowercase, underscores (e.g., `email_address`)
- **Tag names**: kebab-case (e.g., `customer-domain`)
- **Classification names**: UPPERCASE (e.g., `PII`, `FINANCIAL`)

### 2. Description Guidelines

- **Minimum length**: 20 characters
- **Language**: 한글 또는 영어
- **Include**: 목적, 데이터 출처, 업데이트 주기

### 3. Tag Management

- **Domain tags**: 비즈니스 도메인 (e.g., `customer-domain`, `sales-domain`)
- **Team tags**: 소유 팀 (e.g., `data-team`, `analytics-team`)
- **Quality tags**: 데이터 품질 등급 (e.g., `gold`, `silver`, `bronze`)
- **Compliance tags**: 규제 준수 (e.g., `gdpr`, `ccpa`, `pci-dss`)

### 4. Quality Scores

Quality score는 0.0 ~ 1.0 범위:
- **Gold (0.9 - 1.0)**: 완벽한 문서화, 높은 정확도
- **Silver (0.7 - 0.89)**: 양호한 품질, 일부 개선 필요
- **Bronze (0.5 - 0.69)**: 기본 품질, 개선 권장
- **Low (< 0.5)**: 품질 개선 필수

---

## References

- [Apache Atlas Data Model](https://atlas.apache.org/0.8.4/TypeSystem.html)
- [DataHub Metadata Model](https://datahubproject.io/docs/metadata-modeling/metadata-model)
- [Collibra Operating Model](https://www.collibra.com/us/en/operating-model)

---

**Next Phase:** [PHASE-2-API.md](./PHASE-2-API.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
