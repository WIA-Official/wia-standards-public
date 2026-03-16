# WIA-DATA-010: PHASE 3 - Protocol Specification

**Version:** 1.0.0  
**Status:** Complete  
**Last Updated:** 2025-01-15

---

## Overview

This document specifies the communication protocols, security standards, and data transfer mechanisms for WIA-DATA-010 Data Integration Standard.

## 1. Network Protocols

### 1.1 HTTP/HTTPS
- **Version:** HTTP/1.1 minimum, HTTP/2 recommended, HTTP/3 optional
- **Encryption:** TLS 1.2 minimum, TLS 1.3 recommended
- **Certificates:** Valid SSL/TLS certificates from trusted CA
- **HSTS:** Strict-Transport-Security header required for production

**Required Headers:**
```http
Strict-Transport-Security: max-age=31536000; includeSubDomains
Content-Security-Policy: default-src 'self'
X-Content-Type-Options: nosniff
X-Frame-Options: DENY
```

### 1.2 WebSocket
- **Use Case:** Real-time data streaming, bi-directional communication
- **Protocol:** WSS (WebSocket Secure) over TLS
- **Heartbeat:** Ping/Pong every 30 seconds

### 1.3 gRPC
- **Transport:** HTTP/2
- **Encryption:** TLS 1.2+
- **Serialization:** Protocol Buffers
- **Streaming:** Unary, server streaming, client streaming, bidirectional streaming

## 2. Security Protocols

### 2.1 Authentication

#### OAuth 2.0
- **Grant Types:**
  - `authorization_code`: For user-delegated access
  - `client_credentials`: For service-to-service
  - `refresh_token`: For token renewal

**Token Endpoint:**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=CLIENT_ID&
client_secret=CLIENT_SECRET&
scope=pipelines:read pipelines:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "pipelines:read pipelines:write"
}
```

#### API Keys
- **Format:** `wia_live_` prefix for production, `wia_test_` for testing
- **Length:** 32 characters minimum (hex or base62)
- **Storage:** Never log or display full API keys
- **Rotation:** Support key rotation without service interruption

#### JWT (JSON Web Tokens)
- **Algorithm:** RS256 (RSA with SHA-256)
- **Expiration:** 1 hour maximum
- **Claims:**
  - `sub`: Subject (user/service ID)
  - `iss`: Issuer
  - `aud`: Audience
  - `exp`: Expiration
  - `iat`: Issued at
  - `scope`: Permissions

**Example JWT:**
```json
{
  "sub": "user_123",
  "iss": "https://auth.wia-standards.io",
  "aud": "https://api.data-integration.wia-standards.io",
  "exp": 1610727600,
  "iat": 1610724000,
  "scope": "pipelines:read pipelines:write runs:trigger"
}
```

### 2.2 Authorization

#### Role-Based Access Control (RBAC)
| Role | Permissions |
|------|-------------|
| `viewer` | Read pipelines, runs, logs |
| `editor` | viewer + Create/update pipelines, trigger runs |
| `admin` | editor + Delete pipelines, manage users, view audit logs |
| `service` | Automated service account with specific scopes |

#### Scopes
- `pipelines:read` - Read pipeline configurations
- `pipelines:write` - Create/update pipelines
- `pipelines:delete` - Delete pipelines
- `runs:read` - View run status and logs
- `runs:trigger` - Manually trigger pipeline runs
- `connectors:read` - List available connectors
- `metrics:read` - View pipeline metrics

### 2.3 Encryption

#### Data in Transit
- **Protocol:** TLS 1.2 minimum, TLS 1.3 recommended
- **Cipher Suites (TLS 1.2):**
  - `TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256`
  - `TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384`
  - `TLS_DHE_RSA_WITH_AES_128_GCM_SHA256`
- **Cipher Suites (TLS 1.3):**
  - `TLS_AES_128_GCM_SHA256`
  - `TLS_AES_256_GCM_SHA384`
  - `TLS_CHACHA20_POLY1305_SHA256`

#### Data at Rest
- **Algorithm:** AES-256-GCM
- **Key Management:** AWS KMS, Azure Key Vault, Google Cloud KMS, or HashiCorp Vault
- **Key Rotation:** Automatic rotation every 90 days

#### Field-Level Encryption
For PII/PHI fields:
```json
{
  "customer_id": "cust_123",
  "name": "John Doe",
  "email_encrypted": "AES256:base64encodedciphertext",
  "ssn_encrypted": "AES256:base64encodedciphertext"
}
```

### 2.4 mTLS (Mutual TLS)
For service-to-service communication:
- Client presents certificate to server
- Server validates client certificate
- Both parties verify each other

## 3. Data Transfer Protocols

### 3.1 Batch Transfer

#### SFTP (SSH File Transfer Protocol)
- **Port:** 22
- **Authentication:** SSH keys (RSA 2048-bit minimum, ED25519 recommended)
- **Directory Structure:**
  ```
  /inbound/YYYY/MM/DD/
  /outbound/YYYY/MM/DD/
  /archive/YYYY/MM/DD/
  ```

#### S3 Protocol
- **Authentication:** AWS Signature Version 4
- **Encryption:** SSE-S3, SSE-KMS, or SSE-C
- **Multipart Upload:** For files > 100MB
- **Lifecycle:** Archive to Glacier after 30 days, delete after 365 days

#### HTTP/HTTPS Upload
- **Method:** POST or PUT
- **Content-Type:** `multipart/form-data` for file uploads
- **Max Size:** 5GB per request (use multipart for larger)

### 3.2 Streaming Transfer

#### Apache Kafka
- **Protocol:** SASL_SSL
- **SASL Mechanism:** SCRAM-SHA-256 or SCRAM-SHA-512
- **Compression:** lz4, snappy, or zstd
- **Producer ACK:** `all` (wait for all in-sync replicas)

**Producer Configuration:**
```properties
security.protocol=SASL_SSL
sasl.mechanism=SCRAM-SHA-256
sasl.jaas.config=org.apache.kafka.common.security.scram.ScramLoginModule required \
  username="user" \
  password="password";
compression.type=lz4
acks=all
retries=3
max.in.flight.requests.per.connection=5
enable.idempotence=true
```

#### AWS Kinesis
- **Authentication:** AWS IAM
- **Encryption:** Server-side encryption with AWS KMS
- **Shard Iterator Type:** `TRIM_HORIZON`, `LATEST`, `AT_TIMESTAMP`

#### Google Pub/Sub
- **Authentication:** Service Account JSON key or OAuth 2.0
- **Ordering:** Enabled for ordered message delivery
- **Exactly-Once Delivery:** Supported with unique message IDs

### 3.3 Real-Time Sync

#### CDC (Change Data Capture)
**Debezium Configuration:**
```json
{
  "connector.class": "io.debezium.connector.postgresql.PostgresConnector",
  "database.hostname": "postgres.example.com",
  "database.port": 5432,
  "database.user": "debezium",
  "database.password": "${env:DB_PASSWORD}",
  "plugin.name": "pgoutput",
  "publication.autocreate.mode": "filtered",
  "slot.name": "debezium_slot",
  "heartbeat.interval.ms": 60000
}
```

## 4. Error Handling and Retry

### 4.1 Retry Strategy
**Exponential Backoff:**
```
retry_delay = base_delay * (2 ^ retry_attempt) + random_jitter
```

**Example:**
- Attempt 1: 1s + jitter
- Attempt 2: 2s + jitter
- Attempt 3: 4s + jitter
- Attempt 4: 8s + jitter
- Max retries: 5

### 4.2 Circuit Breaker
- **Failure Threshold:** 5 consecutive failures
- **Timeout:** 30 seconds
- **Half-Open State:** After 60 seconds, allow 1 test request
- **Success Threshold:** 2 consecutive successes to close circuit

### 4.3 Dead Letter Queue
Failed messages moved to DLQ after max retries:
```
original_topic: orders
dlq_topic: orders_dlq
```

## 5. Idempotency

### 5.1 Idempotency Keys
**Header:**
```http
Idempotency-Key: uuid-v4-here
```

**Behavior:**
- Same key within 24 hours returns same result
- Prevents duplicate processing
- Returns cached response if key reused

### 5.2 At-Least-Once vs Exactly-Once
| Guarantee | Use Case | Implementation |
|-----------|----------|----------------|
| At-Least-Once | Most systems | Retries without deduplication |
| Exactly-Once | Financial transactions | Idempotency keys + transactions |

## 6. Monitoring and Observability

### 6.1 Health Checks
**Endpoint:** `/health`

**Response:**
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "checks": {
    "database": "healthy",
    "kafka": "healthy",
    "s3": "healthy"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Status Codes:**
- `200 OK`: All systems healthy
- `503 Service Unavailable`: One or more systems unhealthy

### 6.2 Metrics (Prometheus Format)
```
# HELP pipeline_runs_total Total number of pipeline runs
# TYPE pipeline_runs_total counter
pipeline_runs_total{pipeline_id="pip_123",status="success"} 1250

# HELP pipeline_duration_seconds Pipeline execution duration
# TYPE pipeline_duration_seconds histogram
pipeline_duration_seconds_bucket{pipeline_id="pip_123",le="60"} 450
pipeline_duration_seconds_bucket{pipeline_id="pip_123",le="300"} 1100
pipeline_duration_seconds_sum{pipeline_id="pip_123"} 93000
pipeline_duration_seconds_count{pipeline_id="pip_123"} 1250
```

### 6.3 Distributed Tracing
- **Standard:** OpenTelemetry
- **Trace ID:** Propagated via `X-Trace-ID` header
- **Span Context:** Inject into all downstream calls

## 7. Data Quality Protocol

### 7.1 Checksums
- **Algorithm:** SHA-256
- **Location:** File metadata or sidecar file
- **Validation:** Verify on both sides of transfer

**Example:**
```
customers_20250115.csv
customers_20250115.csv.sha256
```

### 7.2 Record Counts
- **Source Count:** Included in metadata
- **Destination Count:** Verified after load
- **Reconciliation:** Alert if counts don't match

### 7.3 Schema Validation
- **On Extract:** Validate against source schema
- **On Load:** Validate against destination schema
- **On Transform:** Validate intermediate schemas

## 8. Compliance

### 8.1 Audit Logging
Required audit events:
- Pipeline created/updated/deleted
- Run triggered
- Configuration changed
- Access granted/revoked
- Data accessed/exported

**Log Format:**
```json
{
  "event_id": "evt_123",
  "event_type": "pipeline.created",
  "timestamp": "2025-01-15T10:30:00Z",
  "actor": {
    "user_id": "user_123",
    "ip_address": "192.0.2.1"
  },
  "resource": {
    "type": "pipeline",
    "id": "pip_abc123"
  },
  "action": "create",
  "result": "success"
}
```

### 8.2 Data Retention
- **Audit Logs:** 7 years
- **Pipeline Logs:** 90 days
- **Metrics:** 13 months

### 8.3 GDPR Compliance
- **Right to Erasure:** Support DELETE requests
- **Data Portability:** Export in machine-readable format
- **Consent Tracking:** Log consent status with data

## 9. Performance Standards

### 9.1 Latency Targets
| Operation | p50 | p95 | p99 |
|-----------|-----|-----|-----|
| GET request | <100ms | <200ms | <500ms |
| POST request | <200ms | <500ms | <1s |
| Batch transfer (1GB) | <5min | <10min | <15min |
| Stream latency | <1s | <5s | <10s |

### 9.2 Throughput
- **REST API:** 1000 requests/sec per instance
- **Streaming:** 100k events/sec per partition
- **Batch:** 1TB/hour minimum

### 9.3 Availability
- **SLA:** 99.9% uptime (8.76 hours downtime/year)
- **RTO:** 4 hours
- **RPO:** 1 hour

---

**Previous Phase:** [PHASE 2 - API Interface](PHASE2-API.md)  
**Next Phase:** [PHASE 4 - Integration](PHASE4-INTEGRATION.md)

**Status:** ✅ Complete  
**Compliance:** WIA-DATA-010 v1.0.0
