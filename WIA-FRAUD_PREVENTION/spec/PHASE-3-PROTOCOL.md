# WIA-FRAUD_PREVENTION v1.0
## PHASE 3: PROTOCOL SPECIFICATION

**Status:** FULL Specification
**Version:** 1.0.0
**Last Updated:** 2026-01-12
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Executive Summary

Phase 3 defines the comprehensive protocol specifications for WIA-FRAUD_PREVENTION systems, including communication protocols, security protocols, data exchange protocols, and integration patterns. This specification ensures secure, reliable, and efficient fraud prevention operations across distributed systems.

### Key Protocol Categories
- **Communication Protocols**: HTTP/REST, WebSocket, gRPC
- **Security Protocols**: Authentication, encryption, data protection
- **Data Exchange Protocols**: Real-time streaming, batch processing
- **Integration Protocols**: System-to-system communication
- **Monitoring Protocols**: Observability and health checks

---

## 2. Communication Protocols

### 2.1 HTTP/REST Protocol

**Protocol Version:** HTTP/2 (primary), HTTP/1.1 (fallback)

**Standard Headers:**
```http
GET /v1/detect/transaction HTTP/2
Host: api.fraud-prevention.wia.org
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
X-API-Key: wia_fp_1234567890abcdef
X-Request-ID: req_550e8400-e29b-41d4-a716-446655440000
X-Client-Version: wia-fraud-sdk/1.0.0
Content-Type: application/json
Accept: application/json
Accept-Encoding: gzip, br
User-Agent: WIA-FraudPrevention-Client/1.0
```

**Response Headers:**
```http
HTTP/2 200 OK
Content-Type: application/json; charset=utf-8
Content-Encoding: gzip
X-Request-ID: req_550e8400-e29b-41d4-a716-446655440000
X-Response-Time: 87ms
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1736705400
X-Content-Type-Options: nosniff
X-Frame-Options: DENY
X-XSS-Protection: 1; mode=block
Strict-Transport-Security: max-age=31536000; includeSubDomains
Cache-Control: no-store, no-cache, must-revalidate
```

**Request Flow:**
```
Client → API Gateway → Load Balancer → Fraud Detection Service → ML Engine
   ↓                                              ↓                    ↓
Response ← API Gateway ← Load Balancer ← Result Processing ← Model Inference
```

### 2.2 WebSocket Protocol

**Protocol:** RFC 6455 (WebSocket Protocol)

**Connection Establishment:**
```http
GET /v1/monitor/stream HTTP/1.1
Host: stream.fraud-prevention.wia.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Sec-WebSocket-Protocol: fraud-prevention-v1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Server Response:**
```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
Sec-WebSocket-Protocol: fraud-prevention-v1
```

**Message Format:**
```json
{
  "type": "message",
  "id": "msg_1234567890",
  "timestamp": "2026-01-12T10:00:00.000Z",
  "channel": "fraud_events",
  "data": {
    "eventType": "fraud_detected",
    "payload": { /* event data */ }
  }
}
```

**Control Messages:**
```json
// Ping
{
  "type": "ping",
  "timestamp": "2026-01-12T10:00:00.000Z"
}

// Pong
{
  "type": "pong",
  "timestamp": "2026-01-12T10:00:00.001Z"
}

// Subscribe
{
  "type": "subscribe",
  "channels": ["fraud_events", "high_risk_transactions"],
  "filters": {
    "minRiskScore": 70,
    "severity": ["high", "critical"]
  }
}

// Unsubscribe
{
  "type": "unsubscribe",
  "channels": ["high_risk_transactions"]
}
```

**Heartbeat Protocol:**
- **Interval:** 30 seconds
- **Timeout:** 60 seconds (if no pong received)
- **Reconnection:** Exponential backoff (1s, 2s, 4s, 8s, 16s, max 60s)

### 2.3 gRPC Protocol

**Protocol Version:** gRPC over HTTP/2

**Service Definition (Protocol Buffers):**
```protobuf
syntax = "proto3";

package wia.fraud_prevention.v1;

service FraudDetectionService {
  // Analyze single transaction
  rpc AnalyzeTransaction(TransactionRequest) returns (FraudAnalysis);

  // Stream real-time fraud events
  rpc StreamFraudEvents(StreamRequest) returns (stream FraudEvent);

  // Bidirectional streaming for batch analysis
  rpc AnalyzeBatch(stream TransactionRequest) returns (stream FraudAnalysis);
}

message TransactionRequest {
  string transaction_id = 1;
  double amount = 2;
  string currency = 3;
  string merchant_id = 4;
  string user_id = 5;
  Location location = 6;
  int64 timestamp = 7;
}

message FraudAnalysis {
  string analysis_id = 1;
  bool is_fraudulent = 2;
  double confidence = 3;
  int32 risk_score = 4;
  string risk_level = 5;
  string recommended_action = 6;
  repeated RiskFactor risk_factors = 7;
  int64 processing_time_ms = 8;
}

message FraudEvent {
  string event_id = 1;
  string event_type = 2;
  int64 timestamp = 3;
  string severity = 4;
  FraudAnalysis analysis = 5;
}

message Location {
  string country = 1;
  string city = 2;
  double latitude = 3;
  double longitude = 4;
  string ip_address = 5;
}

message RiskFactor {
  string factor = 1;
  string severity = 2;
  double contribution = 3;
  string description = 4;
}
```

**Client Implementation:**
```go
package main

import (
    "context"
    "time"

    pb "wia.org/fraud-prevention/v1"
    "google.golang.org/grpc"
    "google.golang.org/grpc/credentials"
)

func main() {
    // Setup TLS credentials
    creds, _ := credentials.NewClientTLSFromFile("ca.pem", "")

    // Connect to gRPC server
    conn, _ := grpc.Dial(
        "grpc.fraud-prevention.wia.org:443",
        grpc.WithTransportCredentials(creds),
        grpc.WithTimeout(10*time.Second),
    )
    defer conn.Close()

    client := pb.NewFraudDetectionServiceClient(conn)

    // Analyze transaction
    ctx, cancel := context.WithTimeout(context.Background(), time.Second)
    defer cancel()

    result, _ := client.AnalyzeTransaction(ctx, &pb.TransactionRequest{
        TransactionId: "txn_001",
        Amount: 150.00,
        Currency: "USD",
        MerchantId: "mch_123",
        UserId: "usr_456",
    })

    // Handle result
    if result.IsFraudulent {
        // Take action
    }
}
```

---

## 3. Security Protocols

### 3.1 Authentication Protocol

**Supported Methods:**

#### 3.1.1 API Key Authentication

**Format:** `wia_fp_{environment}_{random_32_chars}`

**Example:** `wia_fp_prod_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6`

**Usage:**
```http
GET /v1/detect/transaction HTTP/2
X-API-Key: wia_fp_prod_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6
```

**Security Requirements:**
- API keys must be stored securely (encrypted at rest)
- Rotation required every 90 days
- Rate limiting per API key
- Audit logging of all API key usage

#### 3.1.2 OAuth 2.0 Protocol

**Grant Types Supported:**
- Authorization Code Flow (for user-facing applications)
- Client Credentials Flow (for service-to-service)

**Token Endpoint:**
```http
POST /oauth/token HTTP/2
Host: auth.fraud-prevention.wia.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=wia_client_123
&client_secret=secret_abc456
&scope=fraud_detection:read fraud_detection:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "fraud_detection:read fraud_detection:write",
  "refresh_token": "refresh_xyz789"
}
```

**Token Usage:**
```http
GET /v1/detect/transaction HTTP/2
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

#### 3.1.3 JWT (JSON Web Token)

**Header:**
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "key_id_123"
}
```

**Payload:**
```json
{
  "iss": "https://auth.fraud-prevention.wia.org",
  "sub": "client_123",
  "aud": "https://api.fraud-prevention.wia.org",
  "exp": 1736705400,
  "iat": 1736701800,
  "jti": "jwt_1234567890",
  "scope": "fraud_detection:read fraud_detection:write",
  "client_id": "wia_client_123"
}
```

**Token Validation:**
1. Verify signature using public key
2. Check expiration time
3. Validate issuer and audience
4. Verify scope/permissions
5. Check token revocation status

### 3.2 Encryption Protocol

**Data in Transit:**
- **Protocol:** TLS 1.3 (mandatory)
- **Cipher Suites:**
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

**Certificate Requirements:**
- RSA 2048-bit minimum (4096-bit recommended)
- ECDSA P-256 minimum (P-384 recommended)
- Valid certificate chain from trusted CA
- OCSP stapling enabled

**Data at Rest:**
- **Algorithm:** AES-256-GCM
- **Key Management:** AWS KMS, Azure Key Vault, or HashiCorp Vault
- **Key Rotation:** Every 90 days

**PII Encryption:**
```json
{
  "user": {
    "userId": "usr_123",
    "email": "encrypted:AES256:base64encodeddata",
    "phone": "encrypted:AES256:base64encodeddata",
    "cardLast4": "4242",
    "fullCardNumber": "encrypted:AES256:base64encodeddata"
  }
}
```

### 3.3 Request Signing Protocol

**HMAC-SHA256 Signature:**

```python
import hmac
import hashlib
import base64
import time

def sign_request(method, path, body, secret_key):
    timestamp = str(int(time.time()))

    # Create signature payload
    payload = f"{method}\n{path}\n{timestamp}\n{body}"

    # Generate HMAC-SHA256 signature
    signature = hmac.new(
        secret_key.encode(),
        payload.encode(),
        hashlib.sha256
    ).digest()

    # Base64 encode
    signature_b64 = base64.b64encode(signature).decode()

    return {
        "X-Signature": signature_b64,
        "X-Timestamp": timestamp,
        "X-Signature-Version": "v1"
    }

# Usage
headers = sign_request(
    "POST",
    "/v1/detect/transaction",
    '{"amount": 100.00}',
    "your_secret_key"
)
```

**Server Verification:**
1. Extract timestamp from `X-Timestamp` header
2. Verify timestamp is within 5 minutes of current time
3. Reconstruct signature payload
4. Compute HMAC-SHA256 with server's copy of secret key
5. Compare computed signature with provided signature (constant-time comparison)
6. Reject request if signatures don't match

---

## 4. Data Exchange Protocols

### 4.1 Real-Time Streaming Protocol

**Message Queue System:** Apache Kafka / RabbitMQ / AWS Kinesis

**Topic Structure:**
```
fraud-prevention.events.fraud-detected.{severity}
fraud-prevention.transactions.analyzed
fraud-prevention.alerts.{priority}
fraud-prevention.models.predictions
```

**Message Format:**
```json
{
  "messageId": "msg_1234567890",
  "timestamp": "2026-01-12T10:00:00.000Z",
  "version": "1.0",
  "type": "fraud_detected",
  "source": "fraud-detection-service-01",

  "data": {
    "eventId": "evt_001",
    "transactionId": "txn_001",
    "severity": "critical",
    "riskScore": 94
  },

  "metadata": {
    "partition": 3,
    "offset": 12345,
    "correlationId": "corr_abc123"
  }
}
```

**Consumer Group Protocol:**
```yaml
consumer_group: fraud-prevention-consumers
topics:
  - fraud-prevention.events.fraud-detected.critical
  - fraud-prevention.events.fraud-detected.high
auto_offset_reset: earliest
enable_auto_commit: false
max_poll_records: 100
session_timeout_ms: 30000
```

**Delivery Guarantees:**
- **At-least-once:** Default for critical fraud events
- **Exactly-once:** For financial transactions (requires idempotent processing)

**Ordering Guarantees:**
- Messages with same `userId` go to same partition
- Within-partition ordering guaranteed
- Cross-partition ordering not guaranteed

### 4.2 Batch Processing Protocol

**Batch File Format:** JSONL (JSON Lines)

```jsonl
{"transactionId":"txn_001","amount":50.00,"userId":"usr_123","timestamp":"2026-01-12T10:00:00Z"}
{"transactionId":"txn_002","amount":75.50,"userId":"usr_456","timestamp":"2026-01-12T10:01:00Z"}
{"transactionId":"txn_003","amount":120.00,"userId":"usr_789","timestamp":"2026-01-12T10:02:00Z"}
```

**Batch Upload Protocol:**
```http
POST /v1/batch/upload HTTP/2
Host: api.fraud-prevention.wia.org
Content-Type: application/x-ndjson
Content-Encoding: gzip
X-Batch-ID: batch_1234567890
X-Record-Count: 10000

[gzipped JSONL data]
```

**Batch Status Check:**
```http
GET /v1/batch/batch_1234567890/status HTTP/2

Response:
{
  "batchId": "batch_1234567890",
  "status": "processing",
  "uploadedAt": "2026-01-12T10:00:00.000Z",
  "totalRecords": 10000,
  "processedRecords": 7342,
  "progress": 73.42,
  "estimatedCompletion": "2026-01-12T10:15:00.000Z"
}
```

**Results Retrieval:**
```http
GET /v1/batch/batch_1234567890/results HTTP/2

Response:
{
  "batchId": "batch_1234567890",
  "completedAt": "2026-01-12T10:12:34.000Z",
  "totalRecords": 10000,
  "fraudulentCount": 87,
  "resultsUrl": "https://results.fraud-prevention.wia.org/batch_1234567890.jsonl.gz",
  "expiresAt": "2026-01-19T10:12:34.000Z"
}
```

### 4.3 Event-Driven Architecture Protocol

**Event Schema:**
```json
{
  "eventId": "evt_1234567890",
  "eventType": "fraud_detected | transaction_analyzed | alert_triggered | model_updated",
  "eventVersion": "1.0",
  "eventSource": "fraud-detection-service",
  "eventTime": "2026-01-12T10:00:00.000Z",

  "aggregateId": "txn_001",
  "aggregateType": "transaction",

  "payload": {
    // Event-specific data
  },

  "metadata": {
    "correlationId": "corr_abc123",
    "causationId": "evt_previous",
    "userId": "usr_123",
    "tenantId": "tenant_xyz"
  }
}
```

**Event Bus Topics:**
```
# Domain Events
fraud-prevention.domain.transaction.created
fraud-prevention.domain.transaction.analyzed
fraud-prevention.domain.fraud.detected
fraud-prevention.domain.alert.triggered

# Integration Events
fraud-prevention.integration.notification.sent
fraud-prevention.integration.webhook.delivered
fraud-prevention.integration.report.generated
```

---

## 5. Integration Protocols

### 5.1 Webhook Protocol

**Webhook Registration:**
```http
POST /v1/webhooks HTTP/2
{
  "url": "https://your-domain.com/fraud-webhook",
  "events": ["fraud_detected", "alert_created"],
  "secret": "whsec_your_secret_key",
  "version": "1.0"
}
```

**Webhook Delivery:**
```http
POST /fraud-webhook HTTP/2
Host: your-domain.com
Content-Type: application/json
X-WIA-Event: fraud_detected
X-WIA-Delivery: del_1234567890
X-WIA-Signature: t=1736705400,v1=5257a869e7ecebeda32affa62cdca3fa51cad7e77a0e56ff536d0ce8e108d8bd

{
  "id": "evt_webhook_123",
  "type": "fraud_detected",
  "timestamp": "2026-01-12T10:00:00.000Z",
  "data": {
    "eventId": "evt_001",
    "transactionId": "txn_001",
    "riskScore": 94
  }
}
```

**Signature Verification:**
```python
import hmac
import hashlib

def verify_webhook_signature(payload, signature_header, secret):
    # Extract timestamp and signature
    parts = signature_header.split(',')
    timestamp = parts[0].split('=')[1]
    signature = parts[1].split('=')[1]

    # Create signed payload
    signed_payload = f"{timestamp}.{payload}"

    # Compute expected signature
    expected_signature = hmac.new(
        secret.encode(),
        signed_payload.encode(),
        hashlib.sha256
    ).hexdigest()

    # Compare signatures (constant-time)
    return hmac.compare_digest(expected_signature, signature)
```

**Retry Policy:**
- **Attempts:** 5 retries with exponential backoff
- **Backoff:** 1s, 2s, 4s, 8s, 16s
- **Timeout:** 10 seconds per attempt
- **Circuit Breaker:** After 10 consecutive failures, webhook disabled for 1 hour

### 5.2 System-to-System Integration

**Service Discovery Protocol:**
```yaml
service:
  name: fraud-detection-service
  version: 1.0.0
  endpoints:
    - protocol: grpc
      host: grpc.fraud-prevention.wia.org
      port: 443
      health: /health
    - protocol: http
      host: api.fraud-prevention.wia.org
      port: 443
      health: /v1/health

  capabilities:
    - fraud_detection
    - behavioral_analysis
    - anomaly_detection
    - ml_inference

  metadata:
    region: us-east-1
    environment: production
    deployment: blue-green
```

**Health Check Protocol:**
```http
GET /v1/health HTTP/2

Response:
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2026-01-12T10:00:00.000Z",
  "checks": {
    "database": {
      "status": "healthy",
      "responseTime": 12
    },
    "ml_engine": {
      "status": "healthy",
      "responseTime": 45
    },
    "cache": {
      "status": "healthy",
      "responseTime": 3
    },
    "message_queue": {
      "status": "healthy",
      "responseTime": 8
    }
  },
  "metrics": {
    "requestsPerSecond": 347,
    "averageLatency": 76,
    "errorRate": 0.002
  }
}
```

### 5.3 Third-Party Integration Protocol

**Supported Integrations:**
- Payment Processors (Stripe, PayPal, Square)
- Identity Verification (Onfido, Jumio)
- Credit Bureaus (Experian, Equifax, TransUnion)
- Threat Intelligence (IBM X-Force, AlienVault)

**Integration Authentication:**
```json
{
  "integration": {
    "provider": "stripe",
    "authMethod": "oauth2",
    "credentials": {
      "accessToken": "encrypted:AES256:...",
      "refreshToken": "encrypted:AES256:...",
      "expiresAt": "2026-01-12T11:00:00.000Z"
    },
    "webhookSecret": "encrypted:AES256:...",
    "environment": "production"
  }
}
```

**Data Mapping Protocol:**
```yaml
# Stripe → WIA-FRAUD_PREVENTION mapping
mapping:
  source: stripe.charge
  target: wia.transaction

  fields:
    - source: id
      target: externalId
      transform: prefix("stripe_")

    - source: amount
      target: amount
      transform: divide(100)  # Convert cents to dollars

    - source: currency
      target: currency
      transform: uppercase()

    - source: customer
      target: userId
      transform: lookup("stripe_customer_mapping")

    - source: metadata.ip_address
      target: location.ipAddress
      transform: identity()
```

---

## 6. Monitoring and Observability Protocols

### 6.1 Metrics Protocol

**Metrics Format:** Prometheus / OpenMetrics

```prometheus
# Transaction metrics
fraud_prevention_transactions_total{status="analyzed"} 1547892
fraud_prevention_transactions_total{status="fraudulent"} 4187
fraud_prevention_fraud_rate{} 0.0027

# Detection metrics
fraud_prevention_detection_latency_seconds{quantile="0.5"} 0.065
fraud_prevention_detection_latency_seconds{quantile="0.95"} 0.142
fraud_prevention_detection_latency_seconds{quantile="0.99"} 0.287

# Model performance
fraud_prevention_model_accuracy{model="model_fraud_v3.2"} 0.962
fraud_prevention_model_false_positive_rate{model="model_fraud_v3.2"} 0.048

# Alert metrics
fraud_prevention_alerts_total{priority="critical"} 342
fraud_prevention_alerts_total{priority="high"} 987
```

**Metrics Endpoint:**
```http
GET /metrics HTTP/1.1
Host: api.fraud-prevention.wia.org

Response:
# HELP fraud_prevention_transactions_total Total number of transactions analyzed
# TYPE fraud_prevention_transactions_total counter
fraud_prevention_transactions_total{status="analyzed"} 1547892
...
```

### 6.2 Logging Protocol

**Log Format:** Structured JSON

```json
{
  "timestamp": "2026-01-12T10:00:00.000Z",
  "level": "INFO",
  "service": "fraud-detection-service",
  "version": "1.0.0",
  "traceId": "trace_abc123",
  "spanId": "span_def456",
  "message": "Transaction analyzed",

  "context": {
    "transactionId": "txn_001",
    "userId": "usr_123",
    "riskScore": 23,
    "processingTime": 87
  },

  "metadata": {
    "hostname": "fraud-service-01",
    "environment": "production",
    "region": "us-east-1"
  }
}
```

**Log Levels:**
- **TRACE:** Detailed diagnostic information
- **DEBUG:** Debug information
- **INFO:** Informational messages
- **WARN:** Warning messages
- **ERROR:** Error messages
- **FATAL:** Critical failures

**Sensitive Data Masking:**
```json
{
  "level": "INFO",
  "message": "Payment processed",
  "context": {
    "cardNumber": "****-****-****-4242",
    "cvv": "***",
    "email": "u***r@example.com",
    "fullCardNumber": "[REDACTED]"
  }
}
```

### 6.3 Distributed Tracing Protocol

**Format:** OpenTelemetry / Jaeger

**Trace Context Propagation:**
```http
GET /v1/detect/transaction HTTP/2
traceparent: 00-4bf92f3577b34da6a3ce929d0e0e4736-00f067aa0ba902b7-01
tracestate: wia=fraud-detection-service
```

**Trace Format:**
```json
{
  "traceId": "4bf92f3577b34da6a3ce929d0e0e4736",
  "spanId": "00f067aa0ba902b7",
  "parentSpanId": null,
  "name": "POST /v1/detect/transaction",
  "kind": "SERVER",
  "startTime": "2026-01-12T10:00:00.000000Z",
  "endTime": "2026-01-12T10:00:00.087000Z",
  "duration": 87000,

  "attributes": {
    "http.method": "POST",
    "http.url": "/v1/detect/transaction",
    "http.status_code": 200,
    "user.id": "usr_123",
    "transaction.id": "txn_001",
    "fraud.risk_score": 23
  },

  "events": [
    {
      "time": "2026-01-12T10:00:00.010000Z",
      "name": "transaction_validated"
    },
    {
      "time": "2026-01-12T10:00:00.045000Z",
      "name": "ml_model_invoked"
    },
    {
      "time": "2026-01-12T10:00:00.082000Z",
      "name": "result_processed"
    }
  ]
}
```

---

## 7. Protocol Versioning

### 7.1 API Versioning Strategy

**URL Versioning:**
```
https://api.fraud-prevention.wia.org/v1/detect/transaction
https://api.fraud-prevention.wia.org/v2/detect/transaction
```

**Header Versioning:**
```http
GET /detect/transaction HTTP/2
X-API-Version: 2.0
Accept-Version: 2.0
```

**Version Lifecycle:**
- **Beta:** New features, may change
- **Stable:** Production-ready, backwards compatible
- **Deprecated:** Still available but discouraged
- **Sunset:** No longer available

**Deprecation Notice:**
```http
HTTP/2 200 OK
Deprecation: version="v1", date="2026-12-31T23:59:59Z"
Sunset: Sat, 31 Dec 2026 23:59:59 GMT
Link: <https://api.fraud-prevention.wia.org/v2/detect/transaction>; rel="successor-version"
```

### 7.2 Breaking Changes Protocol

**Notification Timeline:**
- **6 months before:** Announce deprecation
- **3 months before:** Begin sunset warnings
- **1 month before:** Increase warning frequency
- **Sunset date:** Remove deprecated version

**Migration Support:**
```json
{
  "migration": {
    "fromVersion": "1.0",
    "toVersion": "2.0",
    "guide": "https://docs.fraud-prevention.wia.org/migration/v1-to-v2",
    "breakingChanges": [
      {
        "type": "field_renamed",
        "oldField": "isFraud",
        "newField": "isFraudulent",
        "description": "Renamed for clarity"
      },
      {
        "type": "field_removed",
        "field": "legacyScore",
        "replacement": "riskScore",
        "description": "Use new risk scoring system"
      }
    ],
    "tooling": {
      "migrationScript": "https://tools.fraud-prevention.wia.org/migrate-v1-to-v2.sh",
      "validator": "https://tools.fraud-prevention.wia.org/validate-v2-compatibility"
    }
  }
}
```

---

## 8. Error Handling Protocol

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "FRAUD_DETECTION_ERROR",
    "message": "Unable to analyze transaction",
    "details": "ML model inference failed due to missing features",
    "timestamp": "2026-01-12T10:00:00.000Z",
    "requestId": "req_1234567890",
    "traceId": "trace_abc123",

    "validationErrors": [
      {
        "field": "transaction.amount",
        "code": "INVALID_VALUE",
        "message": "Amount must be positive"
      }
    ],

    "retryable": false,
    "documentation": "https://docs.fraud-prevention.wia.org/errors/FRAUD_DETECTION_ERROR"
  }
}
```

### 8.2 Circuit Breaker Protocol

**States:**
- **Closed:** Normal operation, requests pass through
- **Open:** Failures exceed threshold, requests fail immediately
- **Half-Open:** Testing if service recovered

**Configuration:**
```yaml
circuit_breaker:
  failure_threshold: 5
  timeout_duration: 60s
  success_threshold: 3
  half_open_max_requests: 10

  failure_conditions:
    - status_code: 500
    - status_code: 503
    - timeout: true
```

**Circuit Breaker Headers:**
```http
HTTP/2 503 Service Unavailable
X-Circuit-Breaker-State: open
X-Circuit-Breaker-Reset: 1736705460
Retry-After: 60
```

---

## 9. Performance Protocols

### 9.1 Caching Protocol

**Cache-Control Headers:**
```http
# Cacheable response (user profile)
HTTP/2 200 OK
Cache-Control: private, max-age=300
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
Last-Modified: Wed, 12 Jan 2026 09:55:00 GMT

# Non-cacheable response (fraud detection)
HTTP/2 200 OK
Cache-Control: no-store, no-cache, must-revalidate
Pragma: no-cache
```

**Conditional Requests:**
```http
GET /v1/analyze/user/usr_123/profile HTTP/2
If-None-Match: "33a64df551425fcc55e4d42a148795d9f25f89d4"
If-Modified-Since: Wed, 12 Jan 2026 09:55:00 GMT

Response (Not Modified):
HTTP/2 304 Not Modified
ETag: "33a64df551425fcc55e4d42a148795d9f25f89d4"
```

### 9.2 Compression Protocol

**Supported Algorithms:**
- gzip (RFC 1952)
- brotli (RFC 7932)
- deflate (RFC 1951)

**Content Negotiation:**
```http
GET /v1/detect/transaction HTTP/2
Accept-Encoding: br, gzip, deflate

Response:
HTTP/2 200 OK
Content-Encoding: br
Content-Type: application/json
Content-Length: 1247
```

### 9.3 Connection Management

**HTTP/2 Features:**
- **Multiplexing:** Multiple requests over single connection
- **Server Push:** Proactively send resources
- **Header Compression:** HPACK compression

**Connection Pooling:**
```yaml
connection_pool:
  max_connections: 100
  max_idle_connections: 50
  idle_timeout: 90s
  connection_timeout: 10s
  keep_alive: true
  keep_alive_timeout: 30s
```

---

## 10. Compliance Protocols

### 10.1 GDPR Compliance

**Data Subject Rights:**
```http
# Right to Access
GET /v1/users/{userId}/data HTTP/2

# Right to Erasure
DELETE /v1/users/{userId}/data HTTP/2
X-GDPR-Request: right-to-be-forgotten

# Data Portability
GET /v1/users/{userId}/export HTTP/2
Accept: application/json
X-GDPR-Request: data-portability
```

**Consent Management:**
```json
{
  "consent": {
    "userId": "usr_123",
    "fraudPrevention": true,
    "dataProcessing": true,
    "thirdPartySharing": false,
    "timestamp": "2026-01-12T10:00:00.000Z",
    "ipAddress": "192.168.1.1",
    "userAgent": "Mozilla/5.0..."
  }
}
```

### 10.2 PCI DSS Compliance

**Secure Transmission:**
- TLS 1.3 mandatory
- Certificate pinning for mobile apps
- No storage of full card numbers in logs
- Tokenization for card data

**Audit Logging:**
```json
{
  "auditLog": {
    "timestamp": "2026-01-12T10:00:00.000Z",
    "userId": "usr_123",
    "action": "access_card_data",
    "resource": "payment_method_pm_123",
    "result": "success",
    "ipAddress": "192.168.1.1",
    "userAgent": "WIA-Client/1.0"
  }
}
```

---

## 11. Implementation Checklist

- [ ] Implement HTTP/2 with TLS 1.3
- [ ] Configure WebSocket with heartbeat
- [ ] Set up gRPC services with Protocol Buffers
- [ ] Implement OAuth 2.0 authentication
- [ ] Configure JWT validation
- [ ] Enable request signing with HMAC-SHA256
- [ ] Set up encryption for data at rest (AES-256-GCM)
- [ ] Implement webhook delivery with retry logic
- [ ] Configure message queues (Kafka/RabbitMQ)
- [ ] Set up distributed tracing (OpenTelemetry)
- [ ] Configure metrics collection (Prometheus)
- [ ] Implement structured logging
- [ ] Set up circuit breakers
- [ ] Configure caching with ETags
- [ ] Enable compression (gzip/brotli)
- [ ] Implement rate limiting
- [ ] Set up API versioning
- [ ] Configure health check endpoints
- [ ] Implement GDPR compliance endpoints
- [ ] Configure PCI DSS compliant logging

---

## 12. Performance Benchmarks

| Protocol | Target Latency | Target Throughput |
|----------|---------------|-------------------|
| HTTP/REST | <100ms (p95) | 10,000 req/sec |
| WebSocket | <50ms (message delivery) | 50,000 msg/sec |
| gRPC | <50ms (p95) | 20,000 req/sec |
| Batch Processing | <1hr (10M records) | 3,000 records/sec |
| Message Queue | <10ms (publish) | 100,000 msg/sec |

---

**Document Classification:** Public Standard
**License:** Open Standard (Implementable without royalties)

© 2026 WIA (World Industry Association)
弘益人間 (홍익인간) - Benefit All Humanity
