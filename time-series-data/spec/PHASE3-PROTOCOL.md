# WIA-DATA-014: PHASE 3 - Protocol Specification

**Version:** 1.0.0
**Status:** Complete
**Date:** 2025-12-26

---

## 1. Overview

PHASE 3 defines communication protocols, security, and transport mechanisms for time-series data.

---

## 2. Transport Protocols

### 2.1 HTTP/HTTPS

**Primary protocol for API communications**

- **TLS 1.2+** required for production
- **HTTP/2** recommended for multiplexing
- **Keep-alive** connections for batch operations

### 2.2 WebSocket

**For real-time streaming data**

```javascript
const ws = new WebSocket('wss://api.example.com/v1/stream');

ws.onmessage = (event) => {
  const dataPoint = JSON.parse(event.data);
  console.log(dataPoint);
};
```

### 2.3 gRPC

**High-performance binary protocol**

```protobuf
service TimeSeriesService {
  rpc Write(WriteRequest) returns (WriteResponse);
  rpc Query(QueryRequest) returns (stream QueryResponse);
}

message DataPoint {
  int64 timestamp = 1;
  string measurement = 2;
  map<string, string> tags = 3;
  map<string, Value> fields = 4;
}
```

### 2.4 MQTT

**For IoT sensor networks**

**Topic Structure:**
```
timeseries/{database}/{measurement}/{sensor_id}
```

**Payload:**
```json
{
  "timestamp": 1735208400000,
  "fields": {"temperature": 23.5, "humidity": 65.2}
}
```

---

## 3. Security

### 3.1 Authentication Methods

#### API Key
```http
Authorization: Bearer wia_sk_abc123def456
```

#### OAuth 2.0
```http
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

#### mTLS (Mutual TLS)
Client certificates for machine-to-machine authentication

### 3.2 Encryption

- **In Transit:** TLS 1.2+ with strong cipher suites
- **At Rest:** AES-256 encryption for stored data
- **Field-level:** Optional encryption for sensitive fields

### 3.3 Authorization

**Role-Based Access Control (RBAC)**

| Role | Read | Write | Delete | Admin |
|------|------|-------|--------|-------|
| Reader | ✓ | ✗ | ✗ | ✗ |
| Writer | ✓ | ✓ | ✗ | ✗ |
| Admin | ✓ | ✓ | ✓ | ✓ |

**Resource-level permissions:**
```json
{
  "user": "user@example.com",
  "permissions": [
    {
      "database": "production",
      "measurement": "cpu",
      "actions": ["read", "write"]
    }
  ]
}
```

---

## 4. Data Compression

### 4.1 HTTP Compression

```http
Accept-Encoding: gzip, deflate, br
Content-Encoding: gzip
```

### 4.2 Time-Series Specific

- **Gorilla compression** for float values
- **Delta encoding** for timestamps
- **Dictionary compression** for tags

---

## 5. Protocol Negotiation

### 5.1 Content Types

| Content-Type | Use Case |
|-------------|----------|
| `application/json` | Human-readable, APIs |
| `application/x-influxdb-line` | Line protocol |
| `application/octet-stream` | Binary format |
| `application/grpc+proto` | gRPC |

### 5.2 Version Negotiation

```http
Accept: application/vnd.wia.timeseries.v1+json
```

---

## 6. Streaming Protocols

### 6.1 Server-Sent Events (SSE)

```http
GET /stream?measurement=cpu&host=server1
Accept: text/event-stream
```

```
data: {"timestamp":"2025-12-26T10:30:00Z","value":75.2}

data: {"timestamp":"2025-12-26T10:30:01Z","value":76.1}
```

### 6.2 WebSocket Streaming

```javascript
// Subscribe to real-time updates
ws.send(JSON.stringify({
  action: 'subscribe',
  measurement: 'temperature',
  tags: {location: 'warehouse'}
}));
```

---

## 7. Error Handling

### 7.1 Retry Policies

**Exponential Backoff:**
```
retry_delay = min(max_delay, base_delay * 2^attempt)
```

**Example:**
- Attempt 1: 1 second
- Attempt 2: 2 seconds
- Attempt 3: 4 seconds
- Attempt 4: 8 seconds
- Max: 60 seconds

### 7.2 Circuit Breaker

- **Threshold:** 50% error rate over 10 requests
- **Open duration:** 30 seconds
- **Half-open:** Test with single request

---

## 8. Quality of Service

### 8.1 QoS Levels

| Level | Guarantee | Use Case |
|-------|-----------|----------|
| 0 | At most once | Non-critical metrics |
| 1 | At least once | General monitoring |
| 2 | Exactly once | Financial data |

### 8.2 Delivery Semantics

**At-least-once:** Default for most operations
**Exactly-once:** Use idempotency keys

```json
{
  "idempotency_key": "unique-request-id",
  "timestamp": "2025-12-26T10:30:00Z",
  "measurement": "payment",
  "fields": {"amount": 100.00}
}
```

---

## 9. Connection Management

### 9.1 Connection Pooling

- **Min connections:** 2
- **Max connections:** 50
- **Idle timeout:** 300 seconds
- **Max lifetime:** 3600 seconds

### 9.2 Keep-Alive

```http
Connection: keep-alive
Keep-Alive: timeout=300, max=1000
```

---

## 10. Monitoring and Observability

### 10.1 Protocol Metrics

- **Request rate:** requests/second
- **Error rate:** errors/requests
- **Latency:** p50, p95, p99
- **Throughput:** bytes/second

### 10.2 Health Checks

```http
GET /health

{
  "status": "healthy",
  "version": "1.0.0",
  "uptime": 3600,
  "connections": 42
}
```

---

## 11. Compliance Requirements

WIA-DATA-014 PHASE 3 compliant implementations MUST:

1. Support HTTPS with TLS 1.2+
2. Implement API key authentication
3. Support JSON content type
4. Provide error responses in standard format
5. Implement rate limiting

Implementations SHOULD:

1. Support HTTP/2
2. Implement OAuth 2.0
3. Support WebSocket streaming
4. Implement compression
5. Provide health check endpoint

---

**License:** MIT
**Copyright:** © 2025 WIA Standards
**弘益人間 (Benefit All Humanity)**
