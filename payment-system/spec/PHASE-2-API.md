# WIA-FIN-012 Phase 2: Payment API Specification

**Version:** 1.0.0  
**Status:** Final  
**Date:** 2025-01-15  
**Category:** Finance - Payment Systems

## Overview

This specification defines the RESTful API endpoints for payment processing, including authorization, capture, void, refund, tokenization, and recurring billing operations.

## Base URL

```
Production:  https://api.payment-gateway.com/v1
Sandbox:     https://sandbox.payment-gateway.com/v1
```

## Authentication

### API Keys
```
Authorization: Bearer sk_live_abc123xyz789def456
```

### OAuth 2.0
```
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

## Core Endpoints

### 1. Authorization

**POST /payments/authorize**

Authorizes a payment without capturing funds.

Request:
```json
{
  "amount": 10000,
  "currency": "USD",
  "payment_method": {
    "type": "card",
    "card": {
      "number": "4532123456789010",
      "exp_month": 12,
      "exp_year": 2025,
      "cvc": "123"
    }
  },
  "merchant_id": "merch_7h3k2m9p"
}
```

Response:
```json
{
  "id": "auth_9x7k3m2n",
  "status": "succeeded",
  "amount": 10000,
  "auth_code": "A7B9C3",
  "created": "2025-01-15T10:30:45Z"
}
```

### 2. Capture

**POST /payments/{id}/capture**

Captures authorized funds.

### 3. Refund

**POST /payments/{id}/refund**

Refunds a captured payment.

### 4. Void

**POST /payments/{id}/void**

Cancels an authorization.

## Security Requirements

- TLS 1.2+ mandatory
- API key rotation every 90 days
- Rate limiting: 100 req/sec
- IP whitelisting recommended
- Webhook signature verification required

## Error Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 400 | Bad Request |
| 401 | Unauthorized |
| 402 | Payment Required (Declined) |
| 429 | Too Many Requests |
| 500 | Internal Server Error |

---

**弘益人間 - Benefit All Humanity**
