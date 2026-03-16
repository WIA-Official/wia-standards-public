# WIA-FIN-021: Financial Data Exchange Standard
## Specification Version 1.2

**Status:** Published  
**Date:** September 15, 2024  
**Authors:** WIA Technical Committee  
**Category:** Finance (FIN)

---

## Changes from v1.1

This version adds:
- Webhook support for event notifications
- Enhanced consent management
- Batch operations API
- Idempotency improvements

## 1. Webhook Support

### 1.1 Webhook Registration

Clients can register webhooks for event notifications:

```http
POST /api/v1/webhooks HTTP/1.1
Content-Type: application/json

{
  "url": "https://client.example.com/webhook",
  "events": ["transaction.created", "payment.completed"],
  "secret": "whsec_abc123xyz"
}
```

### 1.2 Webhook Delivery

**Event Payload:**
```json
{
  "id": "evt_abc123",
  "type": "transaction.created",
  "created": "2024-09-15T14:30:00Z",
  "data": {
    "transaction_id": "TXN-12345",
    "account_id": "ACC-67890",
    "amount": 150.00,
    "currency": "USD"
  }
}
```

**Signature Header:**
```http
X-WIA-Signature: t=1694787000,v1=5257a869e7ecebeda32affa62cdca3fa51cad7e77a0e56ff536d0ce8e108d8bd
```

### 1.3 Retry Policy

Failed webhook deliveries will be retried:
- Immediate retry
- 5 minutes
- 30 minutes
- 2 hours
- 6 hours
- 24 hours (final attempt)

## 2. Enhanced Consent Management

### 2.1 Granular Permissions

More granular permission scopes:

```
accounts:basic:read          # Basic account information
accounts:detail:read         # Detailed account information
accounts:balance:read        # Balance information
transactions:recent:read     # Recent transactions (30 days)
transactions:historical:read # Historical transactions
payments:initiate           # Initiate payments
```

### 2.2 Consent Dashboard

Provide users with consent management dashboard:
- View all active consents
- Granular permission control
- Revoke consents
- View access history

## 3. Batch Operations

### 3.1 Batch Request

Execute multiple operations in a single request:

```http
POST /api/v1/batch HTTP/1.1
Content-Type: application/json

{
  "requests": [
    {
      "id": "req1",
      "method": "GET",
      "url": "/accounts/ACC-001"
    },
    {
      "id": "req2",
      "method": "GET",
      "url": "/accounts/ACC-002"
    }
  ]
}
```

**Response:**
```json
{
  "responses": [
    {
      "id": "req1",
      "status": 200,
      "body": {...}
    },
    {
      "id": "req2",
      "status": 200,
      "body": {...}
    }
  ]
}
```

### 3.2 Batch Limits

- Maximum 50 requests per batch
- Total payload size < 1MB
- Individual request timeout: 30s
- Batch timeout: 60s

## 4. Idempotency Enhancements

### 4.1 Idempotency Keys

All mutation operations MUST support idempotency keys:

```http
POST /api/v1/payments HTTP/1.1
Idempotency-Key: payment-2024-09-15-abc123

{
  "amount": 1500.00,
  "currency": "EUR",
  ...
}
```

### 4.2 Idempotency Behavior

- Keys valid for 24 hours
- Identical requests return cached response
- Different payload with same key returns 409 Conflict

## 5. Rate Limiting Improvements

### 5.1 Tiered Rate Limits

Different limits based on client tier:

| Tier | Requests/Hour | Burst |
|------|--------------|-------|
| Basic | 1,000 | 50 |
| Pro | 10,000 | 200 |
| Enterprise | 100,000 | 1,000 |

### 5.2 Rate Limit Headers

Enhanced rate limit information:

```http
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9987
X-RateLimit-Reset: 1694790000
X-RateLimit-Tier: pro
X-RateLimit-Retry-After: 3600
```

## 6. Pagination Improvements

### 6.1 Cursor-based Pagination

Required for all list operations:

```http
GET /api/v1/transactions?limit=50&cursor=eyJpZCI6MTIzNDU2fQ

Response:
{
  "data": [...],
  "pagination": {
    "next_cursor": "eyJpZCI6MTIzNTA2fQ",
    "prev_cursor": "eyJpZCI6MTIzNDA2fQ",
    "has_more": true,
    "total_count": 5432  # Optional, expensive to compute
  }
}
```

## 7. Security Enhancements

### 7.1 Token Rotation

Refresh token rotation for enhanced security:
- Single-use refresh tokens
- Automatic rotation on refresh
- Refresh token families tracked
- Revoke family on suspicious activity

### 7.2 Strong Customer Authentication (SCA)

Enhanced SCA support for PSD2 compliance:

```json
{
  "sca_required": true,
  "sca_methods": [
    {
      "type": "redirect",
      "url": "https://bank.com/sca?id=PMT-001"
    },
    {
      "type": "decoupled",
      "message": "Confirm payment of €1,500 to ACME Corp"
    }
  ]
}
```

---

**Version:** 1.2  
**Effective Date:** September 15, 2024  
**Supersedes:** v1.1

© 2024 WIA (World Industry Association)  
弘益人間 (Hongik Ingan) - Benefit All Humanity
