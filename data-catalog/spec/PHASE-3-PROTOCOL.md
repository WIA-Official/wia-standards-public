# PHASE 3: Protocol Specification

**Standard:** WIA-DATA-007 (Data Catalog)
**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

이 문서는 Data Catalog 시스템 간 상호 운용을 위한 프로토콜을 정의합니다. 메타데이터 교환, 동기화, 이벤트 스트리밍 등을 위한 표준 프로토콜을 제공합니다.

## 1. Metadata Exchange Protocol (MEP)

### 1.1 Protocol Overview

**MEP**는 서로 다른 Data Catalog 시스템 간 메타데이터를 교환하기 위한 표준 프로토콜입니다.

**Transport:** HTTP/2, gRPC
**Serialization:** Protocol Buffers, JSON
**Authentication:** OAuth 2.0, mTLS

### 1.2 Message Format

#### Metadata Export Request

```json
{
  "request_id": "uuid-v4",
  "timestamp": "2025-12-26T10:00:00Z",
  "source": {
    "system_id": "catalog-system-a",
    "version": "1.0"
  },
  "export_spec": {
    "entities": ["DATASET", "COLUMN", "LINEAGE", "GLOSSARY"],
    "filters": {
      "platforms": ["PostgreSQL", "BigQuery"],
      "updated_since": "2025-12-01T00:00:00Z"
    },
    "format": "JSONL",
    "compression": "GZIP"
  }
}
```

#### Metadata Export Response

```json
{
  "request_id": "uuid-v4",
  "status": "SUCCESS",
  "timestamp": "2025-12-26T10:05:00Z",
  "export": {
    "url": "https://catalog-a.example.com/exports/export-uuid.jsonl.gz",
    "size_bytes": 15728640,
    "record_count": 8934,
    "checksum": "sha256:abc123...",
    "expires_at": "2025-12-27T10:00:00Z"
  }
}
```

### 1.3 Protocol Buffers Definition

```protobuf
syntax = "proto3";

package datacatalog.v1;

service MetadataExchange {
  rpc ExportMetadata(ExportRequest) returns (ExportResponse);
  rpc ImportMetadata(ImportRequest) returns (ImportResponse);
  rpc GetExportStatus(StatusRequest) returns (StatusResponse);
}

message ExportRequest {
  string request_id = 1;
  int64 timestamp = 2;
  SystemInfo source = 3;
  ExportSpec spec = 4;
}

message SystemInfo {
  string system_id = 1;
  string version = 2;
  string endpoint = 3;
}

message ExportSpec {
  repeated EntityType entities = 1;
  Filters filters = 2;
  string format = 3;
  string compression = 4;
}

enum EntityType {
  ENTITY_TYPE_UNSPECIFIED = 0;
  ENTITY_TYPE_DATASET = 1;
  ENTITY_TYPE_COLUMN = 2;
  ENTITY_TYPE_LINEAGE = 3;
  ENTITY_TYPE_GLOSSARY = 4;
}

message Filters {
  repeated string platforms = 1;
  int64 updated_since = 2;
  repeated string tags = 3;
}

message ExportResponse {
  string request_id = 1;
  Status status = 2;
  int64 timestamp = 3;
  ExportInfo export = 4;
}

enum Status {
  STATUS_UNSPECIFIED = 0;
  STATUS_SUCCESS = 1;
  STATUS_PENDING = 2;
  STATUS_FAILED = 3;
}

message ExportInfo {
  string url = 1;
  int64 size_bytes = 2;
  int64 record_count = 3;
  string checksum = 4;
  int64 expires_at = 5;
}
```

## 2. Real-Time Event Streaming Protocol

### 2.1 Event Stream Format

Data Catalog의 변경 사항을 실시간으로 스트리밍하기 위한 프로토콜입니다.

**Transport:** WebSocket, Server-Sent Events (SSE), Apache Kafka
**Format:** JSON, Avro

### 2.2 Event Types

| Event Type | Description |
|-----------|-------------|
| `dataset.created` | 새 데이터셋 생성 |
| `dataset.updated` | 데이터셋 업데이트 |
| `dataset.deleted` | 데이터셋 삭제 |
| `schema.changed` | 스키마 변경 |
| `lineage.created` | 계보 관계 생성 |
| `lineage.updated` | 계보 관계 업데이트 |
| `glossary.created` | 용어 생성 |
| `glossary.updated` | 용어 업데이트 |
| `classification.applied` | 분류 적용 |
| `quality.assessed` | 품질 평가 완료 |

### 2.3 Event Message Format

```json
{
  "event_id": "uuid-v4",
  "event_type": "dataset.updated",
  "timestamp": "2025-12-26T10:00:00Z",
  "source": {
    "system_id": "catalog-prod",
    "environment": "production"
  },
  "subject": {
    "entity_type": "DATASET",
    "entity_id": "550e8400-e29b-41d4-a716-446655440000",
    "qualified_name": "prod_db.public.customers"
  },
  "data": {
    "before": {
      "description": "Old description"
    },
    "after": {
      "description": "고객 정보를 저장하는 메인 테이블"
    },
    "changed_fields": ["description"],
    "changed_by": {
      "user_id": "user@company.com",
      "user_name": "John Doe"
    }
  },
  "metadata": {
    "correlation_id": "correlation-uuid",
    "trace_id": "trace-uuid"
  }
}
```

### 2.4 WebSocket Protocol

#### Connection

```javascript
const ws = new WebSocket('wss://catalog.example.com/v1/events');

ws.onopen = () => {
  // Subscribe to events
  ws.send(JSON.stringify({
    action: 'subscribe',
    channels: ['dataset.*', 'lineage.*'],
    filters: {
      platforms: ['PostgreSQL'],
      tags: ['customer-domain']
    }
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Event received:', message);
};
```

#### Subscription Message

```json
{
  "action": "subscribe",
  "channels": [
    "dataset.*",
    "lineage.created",
    "glossary.*"
  ],
  "filters": {
    "platforms": ["PostgreSQL", "BigQuery"],
    "tags": ["customer-domain"]
  }
}
```

### 2.5 Kafka Topic Structure

```
datacatalog.events.dataset.created
datacatalog.events.dataset.updated
datacatalog.events.dataset.deleted
datacatalog.events.schema.changed
datacatalog.events.lineage.created
datacatalog.events.lineage.updated
datacatalog.events.glossary.created
datacatalog.events.glossary.updated
```

**Partition Key:** `entity_id`
**Message Format:** Avro

#### Avro Schema

```json
{
  "type": "record",
  "name": "CatalogEvent",
  "namespace": "com.wia.datacatalog.events",
  "fields": [
    {"name": "event_id", "type": "string"},
    {"name": "event_type", "type": "string"},
    {"name": "timestamp", "type": "long", "logicalType": "timestamp-millis"},
    {"name": "entity_type", "type": "string"},
    {"name": "entity_id", "type": "string"},
    {"name": "qualified_name", "type": "string"},
    {"name": "data", "type": "string"}
  ]
}
```

## 3. Synchronization Protocol

### 3.1 Change Data Capture (CDC)

메타데이터 변경 사항을 추적하고 동기화하는 프로토콜입니다.

#### Change Log Format

```json
{
  "change_id": "uuid-v4",
  "timestamp": "2025-12-26T10:00:00Z",
  "operation": "UPDATE",
  "entity_type": "DATASET",
  "entity_id": "550e8400-e29b-41d4-a716-446655440000",
  "changes": [
    {
      "field_path": "description",
      "old_value": "Old description",
      "new_value": "New description"
    },
    {
      "field_path": "tags",
      "operation": "ADD",
      "value": "new-tag"
    }
  ],
  "sequence_number": 12345,
  "committed_at": "2025-12-26T10:00:01Z"
}
```

### 3.2 Incremental Sync

```http
POST /sync/incremental
Content-Type: application/json
```

**Request:**
```json
{
  "last_sync_timestamp": "2025-12-25T10:00:00Z",
  "last_sequence_number": 12000,
  "entity_types": ["DATASET", "LINEAGE"]
}
```

**Response:**
```json
{
  "sync_id": "uuid-v4",
  "changes": [
    {
      "change_id": "uuid",
      "sequence_number": 12001,
      "timestamp": "2025-12-25T11:00:00Z",
      "operation": "CREATE",
      "entity_type": "DATASET",
      "entity_data": {...}
    }
  ],
  "next_sequence_number": 12345,
  "has_more": false
}
```

### 3.3 Conflict Resolution

#### Conflict Detection

```json
{
  "conflict_id": "uuid-v4",
  "detected_at": "2025-12-26T10:00:00Z",
  "entity_type": "DATASET",
  "entity_id": "uuid",
  "conflict_type": "UPDATE_UPDATE",
  "local_version": {
    "version": 5,
    "timestamp": "2025-12-26T09:55:00Z",
    "description": "Local description"
  },
  "remote_version": {
    "version": 5,
    "timestamp": "2025-12-26T09:56:00Z",
    "description": "Remote description"
  }
}
```

#### Resolution Strategies

1. **Last Write Wins (LWW):** 가장 최근 타임스탬프 선택
2. **Version Vector:** 벡터 클럭 기반 충돌 해결
3. **Manual Resolution:** 사용자가 직접 선택
4. **Merge:** 변경 사항 병합

## 4. Query Protocol

### 4.1 GraphQL API

```graphql
type Query {
  dataset(id: ID!): Dataset
  datasets(
    filter: DatasetFilter
    page: Int
    pageSize: Int
  ): DatasetConnection

  searchDatasets(
    query: String!
    filters: SearchFilter
  ): SearchResult

  lineage(
    datasetId: ID!
    direction: LineageDirection!
    depth: Int
  ): LineageGraph

  glossaryTerm(id: ID!): GlossaryTerm
}

type Dataset {
  id: ID!
  name: String!
  qualifiedName: String!
  type: DatasetType!
  description: String
  owner: Owner
  schema: Schema
  tags: [String!]!
  classifications: [Classification!]!
  upstreamLineage(depth: Int): [Dataset!]!
  downstreamLineage(depth: Int): [Dataset!]!
}

type Schema {
  columns: [Column!]!
}

type Column {
  id: ID!
  name: String!
  dataType: String!
  description: String
  classifications: [Classification!]!
  statistics: ColumnStatistics
}

type LineageGraph {
  nodes: [LineageNode!]!
  edges: [LineageEdge!]!
}

type LineageNode {
  dataset: Dataset!
  level: Int!
}

type LineageEdge {
  from: ID!
  to: ID!
  process: Process
}

enum LineageDirection {
  UPSTREAM
  DOWNSTREAM
  BOTH
}

enum DatasetType {
  TABLE
  VIEW
  FILE
  STREAM
}
```

### 4.2 Sample GraphQL Query

```graphql
query GetDatasetWithLineage {
  dataset(id: "550e8400-e29b-41d4-a716-446655440000") {
    id
    name
    qualifiedName
    description
    owner {
      email
      name
    }
    schema {
      columns {
        name
        dataType
        description
        classifications {
          name
        }
      }
    }
    upstreamLineage(depth: 2) {
      id
      name
      qualifiedName
    }
    downstreamLineage(depth: 2) {
      id
      name
      qualifiedName
    }
  }
}
```

## 5. Notification Protocol

### 5.1 Email Notification

```json
{
  "notification_id": "uuid-v4",
  "type": "EMAIL",
  "trigger": "dataset.quality.degraded",
  "recipients": [
    "data-owner@company.com",
    "data-team@company.com"
  ],
  "subject": "Data Quality Alert: customers table",
  "body": {
    "template": "quality_alert",
    "variables": {
      "dataset_name": "customers",
      "quality_score": 0.65,
      "threshold": 0.80,
      "issues": [
        "Email format validation failed: 35%",
        "Null values in required field: phone_number"
      ]
    }
  },
  "priority": "HIGH",
  "sent_at": "2025-12-26T10:00:00Z"
}
```

### 5.2 Slack Notification

```json
{
  "notification_id": "uuid-v4",
  "type": "SLACK",
  "trigger": "dataset.created",
  "channel": "#data-catalog",
  "message": {
    "blocks": [
      {
        "type": "header",
        "text": {
          "type": "plain_text",
          "text": "🎉 New Dataset Created"
        }
      },
      {
        "type": "section",
        "fields": [
          {
            "type": "mrkdwn",
            "text": "*Dataset:* customers"
          },
          {
            "type": "mrkdwn",
            "text": "*Owner:* data-team@company.com"
          },
          {
            "type": "mrkdwn",
            "text": "*Platform:* PostgreSQL"
          }
        ]
      },
      {
        "type": "actions",
        "elements": [
          {
            "type": "button",
            "text": {
              "type": "plain_text",
              "text": "View in Catalog"
            },
            "url": "https://catalog.example.com/datasets/uuid"
          }
        ]
      }
    ]
  }
}
```

## 6. Security Protocol

### 6.1 mTLS (Mutual TLS)

서버와 클라이언트 양방향 인증:

```bash
# Client certificate
openssl req -new -x509 -days 365 -key client-key.pem -out client-cert.pem

# Server verification
curl --cert client-cert.pem --key client-key.pem \
     --cacert server-ca.pem \
     https://catalog.example.com/v1/datasets
```

### 6.2 API Key Rotation

```http
POST /auth/rotate-key
Authorization: Bearer <current_token>
```

**Response:**
```json
{
  "new_api_key": "new-key-abc123",
  "old_api_key_expires_at": "2025-12-27T10:00:00Z",
  "rotation_id": "uuid"
}
```

### 6.3 Audit Log Format

```json
{
  "audit_id": "uuid-v4",
  "timestamp": "2025-12-26T10:00:00Z",
  "actor": {
    "user_id": "user@company.com",
    "ip_address": "192.168.1.100",
    "user_agent": "Mozilla/5.0..."
  },
  "action": "DATASET_UPDATE",
  "resource": {
    "type": "DATASET",
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "qualified_name": "prod_db.public.customers"
  },
  "changes": {
    "description": {
      "old": "Old",
      "new": "New"
    }
  },
  "result": "SUCCESS",
  "request_id": "req-uuid"
}
```

## 7. Health Check Protocol

### 7.1 Health Endpoint

```http
GET /health
```

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T10:00:00Z",
  "version": "1.0.0",
  "uptime_seconds": 864000,
  "components": {
    "database": {
      "status": "healthy",
      "response_time_ms": 5
    },
    "search_index": {
      "status": "healthy",
      "response_time_ms": 12
    },
    "cache": {
      "status": "healthy",
      "hit_rate": 0.87
    }
  }
}
```

### 7.2 Readiness Check

```http
GET /ready
```

**Response:**
```json
{
  "ready": true,
  "checks": {
    "database_connection": true,
    "search_index_available": true,
    "cache_available": true
  }
}
```

---

**Next Phase:** [PHASE-4-INTEGRATION.md](./PHASE-4-INTEGRATION.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
