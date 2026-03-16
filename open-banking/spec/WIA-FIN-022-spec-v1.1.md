# WIA-FIN-022: Open Banking Standard - Specification v1.1

**Status:** Stable
**Published:** 2025-06-01
**Category:** Finance/Economy
**Emoji:** 🏦
**Extends:** v1.0

## Changes from v1.0

This version adds Variable Recurring Payments (VRP) and enhanced security features while maintaining backward compatibility with v1.0.

## 1. Variable Recurring Payments (VRP)

### 1.1 Overview

VRP enables customers to authorize recurring payments with variable amounts within predefined limits, without requiring re-authorization for each payment.

### 1.2 VRP Consent

**Endpoint:** `POST /domestic-vrp-consents`

**Request:**
```json
{
  "Data": {
    "ControlParameters": {
      "ValidFromDateTime": "2025-01-01T00:00:00Z",
      "ValidToDateTime": "2026-12-31T23:59:59Z",
      "MaximumIndividualAmount": {
        "Amount": "100.00",
        "Currency": "GBP"
      },
      "PeriodicLimits": [{
        "PeriodType": "Month",
        "Amount": {
          "Amount": "500.00",
          "Currency": "GBP"
        }
      }]
    },
    "Initiation": {
      "CreditorAccount": {
        "SchemeName": "UK.OBIE.SortCodeAccountNumber",
        "Identification": "80200110203345",
        "Name": "Utility Company"
      }
    }
  }
}
```

### 1.3 VRP Payment Submission

**Endpoint:** `POST /domestic-vrps`

**Request:**
```json
{
  "Data": {
    "ConsentId": "vrp-consent-123",
    "Instruction": {
      "InstructionIdentification": "ACME-VRP-001",
      "InstructedAmount": {
        "Amount": "42.50",
        "Currency": "GBP"
      },
      "RemittanceInformation": {
        "Unstructured": "Utility bill payment"
      }
    }
  }
}
```

## 2. Enhanced Security Features

### 2.1 Request Signing

ALL payment and VRP endpoints MUST include JWS signature:

**Header:**
```
x-jws-signature: eyJhbGciOiJQUzI1NiIsImtpZCI6IjEyMzQ1Njc4...
```

**JWS Header:**
```json
{
  "alg": "PS256",
  "kid": "tpp-cert-123",
  "b64": false,
  "crit": ["b64", "http://openbanking.org.uk/iat"],
  "http://openbanking.org.uk/iat": 1735132800,
  "http://openbanking.org.uk/iss": "TPP Client ID"
}
```

### 2.2 Certificate Bound Access Tokens

Access tokens SHOULD be bound to client certificates using:
- RFC 8705 OAuth 2.0 Mutual-TLS Client Authentication
- Certificate thumbprint in token claims

## 3. Bulk Operations

### 3.1 Bulk Payments

**Endpoint:** `POST /bulk-payment-consents`

Supports multiple payments in single consent/submission:

**Request:**
```json
{
  "Data": {
    "Initiation": {
      "Payments": [
        {
          "InstructionIdentification": "PAY-001",
          "InstructedAmount": { "Amount": "100.00", "Currency": "GBP" },
          "CreditorAccount": { /* ... */ }
        },
        {
          "InstructionIdentification": "PAY-002",
          "InstructedAmount": { "Amount": "200.00", "Currency": "GBP" },
          "CreditorAccount": { /* ... */ }
        }
      ]
    }
  }
}
```

## 4. Real-Time Notifications

### 4.1 Webhooks

ASPSPs MAY support webhooks for real-time event notifications:

**Events:**
- `payment.completed`
- `payment.failed`
- `consent.revoked`
- `account.transaction`

**Webhook Payload:**
```json
{
  "event_type": "payment.completed",
  "timestamp": "2025-12-25T14:43:07Z",
  "resource_id": "payment-123",
  "data": {
    "payment_id": "payment-123",
    "status": "AcceptedSettlementCompleted",
    "amount": "100.00",
    "currency": "GBP"
  }
}
```

## 5. Extended Account Information

### 5.1 Standing Orders

**Endpoint:** `GET /accounts/{AccountId}/standing-orders`

**Response:**
```json
{
  "Data": {
    "StandingOrder": [
      {
        "StandingOrderId": "SO-123",
        "Frequency": "Monthly",
        "FirstPaymentDateTime": "2025-01-01T00:00:00Z",
        "FirstPaymentAmount": {
          "Amount": "50.00",
          "Currency": "GBP"
        },
        "CreditorAccount": { /* ... */ }
      }
    ]
  }
}
```

### 5.2 Direct Debits

**Endpoint:** `GET /accounts/{AccountId}/direct-debits`

**Response:**
```json
{
  "Data": {
    "DirectDebit": [
      {
        "DirectDebitId": "DD-456",
        "MandateIdentification": "MANDATE-789",
        "DirectDebitStatusCode": "Active",
        "Name": "Utility Provider",
        "PreviousPaymentDateTime": "2025-11-25T00:00:00Z",
        "PreviousPaymentAmount": {
          "Amount": "45.67",
          "Currency": "GBP"
        }
      }
    ]
  }
}
```

## 6. Performance Requirements

### 6.1 Response Times

ASPSPs SHOULD maintain:
- p50 < 200ms
- p95 < 500ms
- p99 < 1000ms

### 6.2 Availability

ASPSPs MUST provide:
- 99.5% uptime (monthly)
- Planned maintenance windows communicated 7 days in advance

## 7. Backward Compatibility

All v1.0 APIs remain supported. TPPs MAY use v1.0 or v1.1 endpoints independently.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
