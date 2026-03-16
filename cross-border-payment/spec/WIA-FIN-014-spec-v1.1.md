# WIA-FIN-014: Cross-Border Payment Standard v1.1

**Status:** Official Standard
**Version:** 1.1.0
**Date:** December 25, 2025
**Authors:** WIA Standards Committee

---

## Changes from v1.0

This version adds enhanced features while maintaining backward compatibility with v1.0.

### New Features

1. **Batch Payment Processing** - Process multiple payments in a single request
2. **FX Rate Locking** - Lock exchange rates for future payments
3. **Webhooks** - Real-time event notifications
4. **Enhanced Compliance** - Automated risk scoring and EDD workflows

---

## 1. Batch Payment API

### 1.1 Create Batch Payment

**Endpoint:** `POST /batch-payments`

**Request Body:**
```json
{
  "payments": [
    {
      "beneficiaryId": "string",
      "amount": "decimal",
      "currency": "ISO 4217 code",
      "purpose": "string",
      "reference": "string (optional)"
    }
  ],
  "method": "SWIFT | SEPA | BLOCKCHAIN",
  "scheduledDate": "ISO 8601 date (optional)"
}
```

**Response:** `201 Created`
```json
{
  "batchId": "string (UUID)",
  "status": "PENDING | PROCESSING | COMPLETED",
  "paymentCount": "integer",
  "totalAmount": "decimal",
  "currency": "ISO 4217 code",
  "successCount": "integer",
  "failedCount": "integer",
  "payments": [
    {
      "id": "string",
      "status": "PENDING | COMPLETED | FAILED"
    }
  ]
}
```

---

## 2. FX Rate Locking

### 2.1 Lock FX Rate

**Endpoint:** `POST /fx/lock-rate`

**Request Body:**
```json
{
  "from": "ISO 4217 code",
  "to": "ISO 4217 code",
  "amount": "decimal",
  "duration": "integer (seconds, max 86400)"
}
```

**Response:** `201 Created`
```json
{
  "rateLockId": "string (UUID)",
  "rate": "decimal",
  "expiresAt": "ISO 8601 timestamp",
  "reservedAmount": "decimal"
}
```

### 2.2 Use Locked Rate

When creating a payment, include the `rateLockId`:

```json
{
  "beneficiaryId": "string",
  "amount": "decimal",
  "currency": "ISO 4217 code",
  "rateLockId": "string (optional)"
}
```

---

## 3. Webhook Events

### 3.1 Configuration

**Endpoint:** `POST /webhooks`

**Request Body:**
```json
{
  "url": "https://your-domain.com/webhooks",
  "events": ["payment.created", "payment.completed", "payment.failed"],
  "secret": "string (for signature verification)"
}
```

### 3.2 Event Types

- `payment.created` - Payment initiated
- `payment.processing` - Payment submitted to network
- `payment.completed` - Payment successfully completed
- `payment.failed` - Payment failed
- `compliance.review_required` - Manual review needed
- `beneficiary.verified` - Beneficiary account verified

### 3.3 Event Payload

```json
{
  "id": "string (event ID)",
  "type": "payment.completed",
  "timestamp": "ISO 8601 timestamp",
  "data": {
    "id": "string (payment ID)",
    "status": "COMPLETED",
    "amount": "decimal",
    "currency": "ISO 4217 code"
  }
}
```

### 3.4 Signature Verification

Webhooks include `X-WIA-Signature` header with HMAC-SHA256 signature:

```
X-WIA-Signature: sha256=<signature>
```

Verify using your webhook secret.

---

## 4. Enhanced Compliance

### 4.1 Risk Scoring

**Endpoint:** `POST /compliance/risk-score`

**Request Body:**
```json
{
  "customerId": "string",
  "paymentId": "string (optional)",
  "amount": "decimal",
  "currency": "ISO 4217 code",
  "destinationCountry": "ISO 3166-1 alpha-2"
}
```

**Response:** `200 OK`
```json
{
  "riskScore": "integer (0-100)",
  "riskLevel": "LOW | MEDIUM | HIGH | CRITICAL",
  "factors": [
    {
      "type": "COUNTRY_RISK | AMOUNT_RISK | VELOCITY_RISK",
      "score": "integer",
      "description": "string"
    }
  ],
  "recommendedAction": "APPROVE | REVIEW | REJECT"
}
```

### 4.2 Enhanced Due Diligence

**Endpoint:** `POST /compliance/edd`

**Request Body:**
```json
{
  "customerId": "string",
  "documents": [
    {
      "type": "PROOF_OF_INCOME | BANK_STATEMENT | BUSINESS_LICENSE",
      "url": "string",
      "expiryDate": "ISO 8601 date"
    }
  ],
  "sourceOfFunds": "string",
  "purposeOfAccount": "string"
}
```

---

## 5. Performance Enhancements

### 5.1 Response Time Targets

- Payment creation: < 500ms (p95)
- Payment status: < 100ms (p95)
- FX rates: < 50ms (p95)

### 5.2 Throughput

- Minimum: 1,000 payments/second
- Burst capacity: 5,000 payments/second

---

## Appendix: Migration from v1.0

All v1.0 endpoints remain supported. New features are additive:

1. Batch payments use new `/batch-payments` endpoint
2. Rate locking is optional feature via `/fx/lock-rate`
3. Webhooks are opt-in via configuration
4. Enhanced compliance endpoints are optional

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
