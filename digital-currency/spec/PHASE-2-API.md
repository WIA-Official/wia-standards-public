# WIA-FIN-004 Digital Currency Standard
## Phase 2: API Specification

**Version:** 1.0  
**Status:** Production Ready  
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the RESTful API and WebSocket interfaces for digital currency operations. All WIA-FIN-004 compliant systems MUST provide these standard endpoints.

---

## 2. API Principles

- **RESTful Design**: Resources accessed via HTTP methods
- **JSON Payloads**: All requests and responses in JSON
- **HTTPS Only**: TLS 1.3+ required for all connections
- **Stateless**: Each request contains all necessary information
- **Versioned**: API version in URL path (`/api/v1/`)
- **Idempotent**: Support for idempotency keys on mutations
- **Rate Limited**: Protect against abuse

---

## 3. Authentication & Authorization

### 3.1 API Key Authentication

```http
GET /api/v1/accounts HTTP/1.1
Host: api.example.com
Authorization: Bearer YOUR_API_KEY
```

### 3.2 OAuth 2.0

```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read write"
}
```

---

## 4. Core Endpoints

### 4.1 Account Management

#### Get Account
```http
GET /api/v1/accounts/{accountId}
Authorization: Bearer {token}
```

Response:
```json
{
  "id": "ACC-123456",
  "type": "PERSONAL",
  "status": "ACTIVE",
  "owner": { /* ... */ },
  "balances": [
    {
      "currency": "USDC",
      "available": "1000.00",
      "pending": "50.00",
      "total": "1050.00"
    }
  ]
}
```

#### Create Account
```http
POST /api/v1/accounts
Content-Type: application/json
Authorization: Bearer {token}

{
  "type": "PERSONAL",
  "owner": {
    "type": "INDIVIDUAL",
    "name": "John Doe",
    "email": "john@example.com"
  },
  "currency": "USDC"
}
```

#### Get Balance
```http
GET /api/v1/accounts/{accountId}/balance?currency=USDC
```

### 4.2 Payments

#### Initiate Payment
```http
POST /api/v1/payments
Content-Type: application/json
Idempotency-Key: {unique-key}

{
  "from": {
    "type": "ACCOUNT",
    "identifier": "ACC-123456"
  },
  "to": {
    "type": "ACCOUNT",
    "identifier": "ACC-789012"
  },
  "amount": {
    "value": "100.00",
    "currency": "USDC"
  },
  "purpose": "Payment for services",
  "reference": "INV-001"
}
```

Response:
```json
{
  "paymentId": "PAY-abc123",
  "status": "PENDING",
  "estimatedCompletion": "2024-01-15T10:30:05Z",
  "fee": { "value": "0.50", "currency": "USDC" }
}
```

#### Get Payment Status
```http
GET /api/v1/payments/{paymentId}
```

#### List Payments
```http
GET /api/v1/payments?accountId=ACC-123456&limit=10&offset=0
```

### 4.3 Currency Exchange

#### Get Exchange Rates
```http
GET /api/v1/exchange/rates?from=USDC&to=USDT
```

Response:
```json
{
  "from": "USDC",
  "to": "USDT",
  "rate": "0.9998",
  "timestamp": "2024-01-15T10:30:00Z",
  "validUntil": "2024-01-15T10:31:00Z"
}
```

#### Execute Exchange
```http
POST /api/v1/exchange
{
  "from": {
    "currency": "USDC",
    "amount": "1000.00"
  },
  "to": {
    "currency": "USDT"
  },
  "accountId": "ACC-123456"
}
```

### 4.4 Compliance

#### Submit KYC
```http
POST /api/v1/compliance/kyc
Content-Type: multipart/form-data

{
  "accountId": "ACC-123456",
  "level": "STANDARD",
  "documents": [/* files */]
}
```

#### Check Transaction
```http
POST /api/v1/compliance/screening
{
  "transactionId": "TXN-abc123",
  "checks": ["AML", "SANCTIONS"]
}
```

---

## 5. WebSocket API

### 5.1 Connection

```javascript
const ws = new WebSocket('wss://api.example.com/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_API_KEY'
  }));
};
```

### 5.2 Subscribe to Events

```javascript
// Subscribe to balance updates
ws.send(JSON.stringify({
  type: 'subscribe',
  channel: 'balance',
  accountId: 'ACC-123456'
}));

// Receive updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  // { type: 'balance_update', accountId: '...', newBalance: '...' }
};
```

### 5.3 Event Types

- `balance_update`: Account balance changed
- `payment_received`: Incoming payment
- `payment_completed`: Outgoing payment completed
- `payment_failed`: Payment failed
- `compliance_alert`: Compliance flag raised

---

## 6. Webhooks

### 6.1 Register Webhook

```http
POST /api/v1/webhooks
{
  "url": "https://merchant.com/webhook",
  "events": ["payment.completed", "payment.failed"],
  "secret": "webhook_secret"
}
```

### 6.2 Webhook Payload

```json
{
  "event": "payment.completed",
  "timestamp": "2024-01-15T10:30:00Z",
  "data": {
    "paymentId": "PAY-123",
    "amount": "99.99",
    "currency": "USDC"
  },
  "signature": "sha256_hmac_signature"
}
```

### 6.3 Verify Webhook Signature

```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const computed = crypto
    .createHmac('sha256', secret)
    .update(JSON.stringify(payload))
    .digest('hex');
  
  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(computed)
  );
}
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "INSUFFICIENT_BALANCE",
    "message": "Account balance insufficient for transaction",
    "details": "Available: 50.00 USDC, Required: 100.00 USDC",
    "field": "amount",
    "timestamp": "2024-01-15T10:30:00Z",
    "requestId": "req-abc123"
  }
}
```

### 7.2 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication failed
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `409 Conflict`: Duplicate/conflict
- `429 Too Many Requests`: Rate limited
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary outage

---

## 8. Rate Limiting

### 8.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642248000
```

### 8.2 Default Limits

- **Standard**: 1000 requests/hour
- **Premium**: 10000 requests/hour
- **Enterprise**: Custom

---

## 9. Pagination

### 9.1 Request

```http
GET /api/v1/payments?limit=10&offset=20
```

### 9.2 Response

```json
{
  "data": [/* items */],
  "pagination": {
    "total": 100,
    "limit": 10,
    "offset": 20,
    "hasMore": true
  }
}
```

---

**End of Phase 2 Specification**

© 2025 SmileStory Inc. / WIA
