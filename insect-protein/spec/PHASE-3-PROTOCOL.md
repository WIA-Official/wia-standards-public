# WIA-AGRI-025: Insect Protein Standard
## PHASE 3: Protocol Specification

**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-12-26
**Standard ID:** WIA-AGRI-025

---

## 1. Overview

This specification defines communication protocols, security requirements, data synchronization mechanisms, and real-time update protocols for the WIA-AGRI-025 Insect Protein Standard.

### 1.1 Protocol Objectives

- Ensure secure data transmission
- Enable real-time updates and notifications
- Support offline-first applications
- Facilitate multi-party data sharing
- Maintain data integrity and authenticity

---

## 2. Communication Protocols

### 2.1 HTTP/HTTPS

**Required:** All API communications MUST use HTTPS (TLS 1.3 or higher)

**Headers:**
```http
Content-Type: application/json
Accept: application/json
Authorization: Bearer {API_KEY}
X-WIA-Version: 1.0
X-WIA-Request-ID: {UUID}
```

**Request ID:** Each request MUST include a unique request ID for tracing and debugging.

### 2.2 WebSocket

For real-time updates and bidirectional communication.

**Endpoint:** `wss://api.wia-agri.org/ws`

**Connection Initialization:**
```json
{
  "type": "connection",
  "apiKey": "wia_live_abc123xyz789",
  "subscriptions": [
    "products.*",
    "certifications.issued",
    "safety.alerts"
  ]
}
```

**Message Format:**
```json
{
  "type": "event",
  "event": "product.created",
  "timestamp": "2025-12-26T12:00:00Z",
  "data": {
    "productId": "PRODUCT-001",
    "species": "Acheta domesticus"
  }
}
```

### 2.3 gRPC

For high-performance, low-latency applications.

**Endpoint:** `grpc://api.wia-agri.org:443`

**Proto Definition:**
```protobuf
syntax = "proto3";

package wia.agri.v1;

service InsectProteinService {
  rpc GetProduct (GetProductRequest) returns (Product);
  rpc ListProducts (ListProductsRequest) returns (ListProductsResponse);
  rpc CreateProduct (CreateProductRequest) returns (Product);
  rpc StreamUpdates (StreamRequest) returns (stream ProductUpdate);
}

message Product {
  string id = 1;
  string version = 2;
  Species species = 3;
  ProductForm form = 4;
  NutritionProfile nutrition = 5;
}
```

---

## 3. Authentication & Authorization

### 3.1 API Key Authentication

**Format:** `wia_{environment}_{random_string}`
- `wia_test_...`: Test environment
- `wia_live_...`: Production environment

**Storage:** API keys MUST be stored securely (environment variables, secret management systems)

**Rotation:** API keys SHOULD be rotated every 90 days

### 3.2 OAuth 2.0 Flow

**Authorization Code Grant:**

1. **Authorization Request:**
```http
GET /oauth/authorize?
  response_type=code&
  client_id={CLIENT_ID}&
  redirect_uri={REDIRECT_URI}&
  scope=products:read products:write certifications:read&
  state={RANDOM_STATE}
```

2. **Token Exchange:**
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={AUTH_CODE}&
client_id={CLIENT_ID}&
client_secret={CLIENT_SECRET}&
redirect_uri={REDIRECT_URI}
```

3. **Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "scope": "products:read products:write certifications:read"
}
```

### 3.3 JWT (JSON Web Tokens)

**Header:**
```json
{
  "alg": "RS256",
  "typ": "JWT",
  "kid": "wia-key-001"
}
```

**Payload:**
```json
{
  "iss": "api.wia-agri.org",
  "sub": "FARM-KR-001",
  "aud": "wia-agri-api",
  "exp": 1640534400,
  "iat": 1640530800,
  "scope": "products:read products:write"
}
```

**Validation:**
- Verify signature using public key
- Check expiration time
- Validate issuer and audience
- Verify scope permissions

### 3.4 Permissions & Scopes

**Available Scopes:**
- `products:read`: Read product data
- `products:write`: Create/update products
- `products:delete`: Delete products
- `certifications:read`: Read certifications
- `certifications:write`: Request certifications
- `safety:read`: Read safety data
- `safety:write`: Report safety alerts
- `admin:*`: Full administrative access

**Role-Based Access Control (RBAC):**

| Role | Permissions |
|------|-------------|
| Farm Operator | products:write, safety:read |
| Processor | products:read, products:write |
| Certification Body | certifications:write, products:read |
| Retailer | products:read, certifications:read |
| Admin | admin:* |

---

## 4. Data Synchronization

### 4.1 Synchronization Strategies

**Push Synchronization:**
- Server pushes updates to clients via WebSocket or webhooks
- Real-time notification of changes

**Pull Synchronization:**
- Clients periodically poll for updates
- Using `Last-Modified` and `ETag` headers

**Hybrid Synchronization:**
- WebSocket for critical updates
- Periodic polling for comprehensive sync

### 4.2 Versioning & Conflict Resolution

**Optimistic Locking:**

```http
PUT /products/PRODUCT-001
If-Match: "version-123"

{
  "nutrition": { /* updated data */ }
}
```

**Response (Conflict):**
```http
HTTP/1.1 409 Conflict
ETag: "version-124"

{
  "error": {
    "code": "CONFLICT",
    "message": "Resource has been modified",
    "currentVersion": "version-124"
  }
}
```

**Last Write Wins (LWW):**
- Timestamp-based conflict resolution
- Server timestamp is authoritative

**Vector Clocks:**
- For distributed systems
- Track causal relationships between updates

### 4.3 Change Data Capture (CDC)

**Event Log:**
```json
{
  "eventId": "EVENT-001",
  "timestamp": "2025-12-26T12:00:00Z",
  "entityType": "product",
  "entityId": "PRODUCT-001",
  "operation": "UPDATE",
  "changes": [
    {
      "field": "nutrition.protein.value",
      "oldValue": 65.0,
      "newValue": 65.5
    }
  ],
  "userId": "USER-001"
}
```

**Subscription:**
```http
GET /events/stream
Accept: text/event-stream
Last-Event-ID: EVENT-100
```

**Server-Sent Events (SSE):**
```
event: product.updated
id: EVENT-101
data: {"productId": "PRODUCT-001", "field": "nutrition"}

event: certification.issued
id: EVENT-102
data: {"certificationId": "CERT-001", "productId": "PRODUCT-001"}
```

---

## 5. Webhook Protocol

### 5.1 Webhook Registration

**POST /webhooks**

```json
{
  "url": "https://your-domain.com/webhook",
  "events": [
    "product.created",
    "product.updated",
    "certification.issued",
    "safety.alert"
  ],
  "secret": "whsec_abc123xyz789",
  "active": true
}
```

**Response:**
```json
{
  "success": true,
  "data": {
    "webhookId": "WEBHOOK-001",
    "url": "https://your-domain.com/webhook",
    "events": ["product.created", "product.updated"],
    "createdAt": "2025-12-26T12:00:00Z"
  }
}
```

### 5.2 Webhook Payload

**Request:**
```http
POST https://your-domain.com/webhook
Content-Type: application/json
X-WIA-Event: product.created
X-WIA-Signature: sha256=abc123...
X-WIA-Delivery-ID: DELIVERY-001
X-WIA-Timestamp: 1640530800

{
  "event": "product.created",
  "timestamp": "2025-12-26T12:00:00Z",
  "data": {
    "productId": "PRODUCT-NEW-001",
    "batchNumber": "BATCH-2025-010",
    "species": "Acheta domesticus",
    "form": "powder"
  }
}
```

### 5.3 Webhook Security

**Signature Verification:**

```javascript
const crypto = require('crypto');

function verifyWebhookSignature(payload, signature, secret) {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  return `sha256=${expectedSignature}` === signature;
}
```

**Replay Protection:**
- Check `X-WIA-Timestamp` to reject old requests (>5 minutes)
- Track `X-WIA-Delivery-ID` to prevent duplicate processing

### 5.4 Webhook Retry Policy

**Retry Schedule:**
- Attempt 1: Immediate
- Attempt 2: 1 minute later
- Attempt 3: 5 minutes later
- Attempt 4: 15 minutes later
- Attempt 5: 1 hour later

**Success Criteria:**
- HTTP status codes: 200, 201, 202, 204

**Failure:**
- After 5 failed attempts, webhook is marked as failed
- Email notification sent to webhook owner

---

## 6. Real-Time Updates

### 6.1 WebSocket Protocol

**Connection:**
```javascript
const ws = new WebSocket('wss://api.wia-agri.org/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    apiKey: 'wia_live_abc123xyz789'
  }));

  ws.send(JSON.stringify({
    type: 'subscribe',
    channels: ['products.*', 'certifications.issued']
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Event:', message);
};
```

**Message Types:**

1. **Authentication:**
```json
{
  "type": "auth",
  "apiKey": "wia_live_abc123xyz789"
}
```

2. **Subscription:**
```json
{
  "type": "subscribe",
  "channels": ["products.PRODUCT-001", "safety.alerts"]
}
```

3. **Unsubscribe:**
```json
{
  "type": "unsubscribe",
  "channels": ["products.PRODUCT-001"]
}
```

4. **Event:**
```json
{
  "type": "event",
  "channel": "products.created",
  "timestamp": "2025-12-26T12:00:00Z",
  "data": { /* event data */ }
}
```

5. **Heartbeat:**
```json
{
  "type": "ping"
}
```

**Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-26T12:00:01Z"
}
```

### 6.2 Channel Patterns

**Wildcard Subscriptions:**
- `products.*`: All product events
- `products.PRODUCT-001.*`: All events for specific product
- `*.created`: All creation events
- `safety.alerts.*`: All safety alerts

### 6.3 Presence & Status

**User Presence:**
```json
{
  "type": "presence",
  "channel": "farm.FARM-KR-001",
  "users": [
    {
      "userId": "USER-001",
      "status": "online",
      "connectedAt": "2025-12-26T11:30:00Z"
    }
  ]
}
```

---

## 7. Caching & Performance

### 7.1 HTTP Caching Headers

**Cache-Control:**
```http
Cache-Control: public, max-age=3600
Cache-Control: private, no-cache
Cache-Control: no-store
```

**ETag:**
```http
ETag: "version-123"
```

**Last-Modified:**
```http
Last-Modified: Wed, 26 Dec 2025 12:00:00 GMT
```

**Conditional Requests:**
```http
GET /products/PRODUCT-001
If-None-Match: "version-123"
If-Modified-Since: Wed, 26 Dec 2025 12:00:00 GMT
```

**Response (Not Modified):**
```http
HTTP/1.1 304 Not Modified
ETag: "version-123"
```

### 7.2 CDN Integration

**CloudFlare, Fastly, Akamai:**
- Cache static resources (images, PDFs)
- Cache GET requests with appropriate headers
- Purge cache on updates

**Geo-Replication:**
- Regional API endpoints
- Reduce latency for global users

---

## 8. Data Compression

### 8.1 Request Compression

**Accept-Encoding:**
```http
GET /products
Accept-Encoding: gzip, deflate, br
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Encoding: gzip
Content-Type: application/json
```

### 8.2 Response Compression

**Automatic Compression:**
- Responses >1KB automatically compressed
- Brotli (br) preferred over gzip

**Compression Algorithms:**
1. Brotli (br): Best compression ratio
2. Gzip (gzip): Widely supported
3. Deflate (deflate): Legacy support

---

## 9. Monitoring & Observability

### 9.1 Logging

**Request Logging:**
```json
{
  "timestamp": "2025-12-26T12:00:00Z",
  "requestId": "REQ-001",
  "method": "GET",
  "path": "/products/PRODUCT-001",
  "statusCode": 200,
  "responseTime": 45,
  "userId": "USER-001",
  "ipAddress": "203.0.113.42"
}
```

**Error Logging:**
```json
{
  "timestamp": "2025-12-26T12:05:00Z",
  "requestId": "REQ-002",
  "level": "ERROR",
  "error": {
    "code": "INTERNAL_ERROR",
    "message": "Database connection failed",
    "stack": "..."
  }
}
```

### 9.2 Metrics

**Key Metrics:**
- Request rate (requests/second)
- Error rate (errors/total requests)
- Response time (p50, p95, p99)
- API key usage
- Webhook delivery success rate

**OpenTelemetry Integration:**
```javascript
const { trace } = require('@opentelemetry/api');

const span = trace.getTracer('wia-agri').startSpan('getProduct');
span.setAttribute('product.id', 'PRODUCT-001');
// ... perform operation
span.end();
```

### 9.3 Health Checks

**GET /health**

```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T12:00:00Z",
  "services": {
    "database": "healthy",
    "redis": "healthy",
    "storage": "healthy"
  },
  "version": "1.0.0"
}
```

**GET /health/ready**

```json
{
  "ready": true,
  "checks": {
    "database": true,
    "migrations": true,
    "config": true
  }
}
```

---

## 10. Security Best Practices

### 10.1 HTTPS/TLS

- TLS 1.3 required
- Strong cipher suites only
- HSTS (HTTP Strict Transport Security) enabled

### 10.2 API Key Management

- Rotate keys every 90 days
- Use separate keys for different environments
- Revoke compromised keys immediately

### 10.3 Rate Limiting

- Protect against DDoS attacks
- Implement per-user and per-IP limits
- Return `429 Too Many Requests` with `Retry-After` header

### 10.4 Input Validation

- Validate all input data
- Sanitize user-provided content
- Use parameterized queries (prevent SQL injection)

### 10.5 CORS (Cross-Origin Resource Sharing)

```http
Access-Control-Allow-Origin: https://app.example.com
Access-Control-Allow-Methods: GET, POST, PUT, DELETE
Access-Control-Allow-Headers: Authorization, Content-Type
Access-Control-Max-Age: 86400
```

---

**Next Phase:** [PHASE-4-INTEGRATION.md](PHASE-4-INTEGRATION.md)

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA
