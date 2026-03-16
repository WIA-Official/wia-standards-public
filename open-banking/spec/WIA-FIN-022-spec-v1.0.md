# WIA-FIN-022: Open Banking Standard - Specification v1.0

**Status:** Stable
**Published:** 2025-01-15
**Category:** Finance/Economy
**Emoji:** 🏦

## Abstract

This document specifies the WIA-FIN-022 Open Banking Standard version 1.0, defining APIs, security protocols, and implementation requirements for enabling secure third-party access to financial account information and payment initiation services.

## 1. Introduction

### 1.1 Purpose

The Open Banking Standard enables customers to share their financial data with regulated third-party providers through secure, standardized APIs. This promotes innovation, competition, and consumer choice in financial services while maintaining security and privacy.

### 1.2 Scope

This specification covers:
- Account Information Services (AIS)
- Payment Initiation Services (PIS)
- Confirmation of Funds (CoF)
- Security and authentication protocols
- API design and implementation requirements

### 1.3 Compliance

Implementations MUST comply with:
- PSD2 (Payment Services Directive 2) where applicable
- Open Banking UK Read/Write API Specifications
- GDPR (General Data Protection Regulation)
- Local financial regulations

## 2. Terminology

- **ASPSP**: Account Servicing Payment Service Provider (bank)
- **TPP**: Third Party Provider
- **AISP**: Account Information Service Provider
- **PISP**: Payment Initiation Service Provider
- **PSU**: Payment Service User (customer)
- **SCA**: Strong Customer Authentication
- **FAPI**: Financial-grade API

## 3. Architecture

### 3.1 System Components

```
┌──────────┐
│   TPP    │ Third-Party Provider
└────┬─────┘
     │ HTTPS/TLS 1.2+
     │ OAuth 2.0
     │ mTLS (optional)
┌────▼──────────────┐
│  Authorization    │ OAuth 2.0 Server
│  Server           │
├───────────────────┤
│  Resource Server  │ API Endpoints
│  - Accounts       │
│  - Payments       │
│  - Funds Check    │
└────┬──────────────┘
     │
┌────▼─────┐
│  Core    │ Banking System
│  Banking │
└──────────┘
```

### 3.2 API Layers

1. **Transport Layer**: TLS 1.2+, mTLS (recommended)
2. **Authentication Layer**: OAuth 2.0 with FAPI profile
3. **Authorization Layer**: Consent management
4. **Resource Layer**: RESTful APIs

## 4. Account Information Services

### 4.1 Accounts API

#### 4.1.1 Get Accounts

**Endpoint:** `GET /accounts`

**Authorization:** OAuth 2.0 Bearer token

**Required Scopes:** `accounts:read`

**Request Headers:**
```
Authorization: Bearer {access_token}
x-fapi-interaction-id: {uuid}
Accept: application/json
```

**Response:** 200 OK
```json
{
  "Data": {
    "Account": [
      {
        "AccountId": "22289",
        "Currency": "GBP",
        "AccountType": "Personal",
        "AccountSubType": "CurrentAccount",
        "Nickname": "Personal Account",
        "Account": [
          {
            "SchemeName": "UK.OBIE.SortCodeAccountNumber",
            "Identification": "80200110203345",
            "Name": "Mr John Smith"
          }
        ]
      }
    ]
  },
  "Links": {
    "Self": "https://api.bank.example.com/accounts"
  },
  "Meta": {
    "TotalPages": 1
  }
}
```

#### 4.1.2 Get Account Balances

**Endpoint:** `GET /accounts/{AccountId}/balances`

**Response:** 200 OK
```json
{
  "Data": {
    "Balance": [
      {
        "AccountId": "22289",
        "Amount": {
          "Amount": "1230.00",
          "Currency": "GBP"
        },
        "CreditDebitIndicator": "Credit",
        "Type": "InterimAvailable",
        "DateTime": "2025-12-25T14:43:07+00:00"
      }
    ]
  }
}
```

#### 4.1.3 Get Transactions

**Endpoint:** `GET /accounts/{AccountId}/transactions`

**Query Parameters:**
- `fromBookingDateTime` (optional): ISO 8601 datetime
- `toBookingDateTime` (optional): ISO 8601 datetime

**Response:** 200 OK
```json
{
  "Data": {
    "Transaction": [
      {
        "TransactionId": "123",
        "BookingDateTime": "2025-12-25T14:23:45+00:00",
        "ValueDateTime": "2025-12-25T14:23:45+00:00",
        "Amount": {
          "Amount": "10.00",
          "Currency": "GBP"
        },
        "CreditDebitIndicator": "Debit",
        "Status": "Booked",
        "MerchantDetails": {
          "MerchantName": "Coffee Shop"
        }
      }
    ]
  }
}
```

## 5. Payment Initiation Services

### 5.1 Domestic Payments

#### 5.1.1 Create Payment Consent

**Endpoint:** `POST /domestic-payment-consents`

**Request Body:**
```json
{
  "Data": {
    "Initiation": {
      "InstructionIdentification": "ACME-PAY-123",
      "EndToEndIdentification": "E2E-123",
      "InstructedAmount": {
        "Amount": "165.88",
        "Currency": "GBP"
      },
      "CreditorAccount": {
        "SchemeName": "UK.OBIE.SortCodeAccountNumber",
        "Identification": "08080021325698",
        "Name": "ACME Inc"
      },
      "RemittanceInformation": {
        "Unstructured": "Internal ops code 5120101",
        "Reference": "FRESCO-101"
      }
    }
  },
  "Risk": {}
}
```

**Response:** 201 Created
```json
{
  "Data": {
    "ConsentId": "58923",
    "Status": "AwaitingAuthorisation",
    "CreationDateTime": "2025-12-25T14:43:07+00:00",
    "Initiation": { /* same as request */ }
  }
}
```

#### 5.1.2 Submit Payment

**Endpoint:** `POST /domestic-payments`

**Headers:**
```
Authorization: Bearer {access_token}
x-idempotency-key: {unique-key}
x-jws-signature: {detached-jws}
```

## 6. Confirmation of Funds

**Endpoint:** `POST /funds-confirmations`

**Request:**
```json
{
  "Data": {
    "InstructedAmount": {
      "Amount": "20.00",
      "Currency": "GBP"
    },
    "DebtorAccount": {
      "SchemeName": "UK.OBIE.SortCodeAccountNumber",
      "Identification": "11280001234567"
    }
  }
}
```

**Response:** 200 OK
```json
{
  "Data": {
    "FundsConfirmationId": "FC123",
    "FundsAvailable": true
  }
}
```

## 7. Security Requirements

### 7.1 Transport Security

- MUST use TLS 1.2 or higher
- MUST support strong cipher suites
- SHOULD implement mutual TLS (mTLS)

### 7.2 Authentication

- MUST use OAuth 2.0 with FAPI security profile
- MUST implement Strong Customer Authentication (SCA)
- MUST support PKCE for authorization code flow

### 7.3 Message Security

- MUST sign critical requests with JWS
- MUST validate JWS signatures
- MUST use eIDAS qualified certificates

## 8. Error Handling

### 8.1 HTTP Status Codes

- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Invalid or missing credentials
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

### 8.2 Error Response Format

```json
{
  "Errors": [
    {
      "ErrorCode": "UK.OBIE.Field.Invalid",
      "Message": "The field 'Amount' is invalid",
      "Path": "Data.Initiation.InstructedAmount.Amount"
    }
  ]
}
```

## 9. Compliance and Conformance

### 9.1 API Versioning

APIs MUST support versioning through:
- URL path: `/v1/accounts`
- Custom header: `x-api-version: 1.0`

### 9.2 Rate Limiting

ASPSPs SHOULD implement rate limiting:
- Account Information: 10 requests/second
- Payment Initiation: 5 requests/second

### 9.3 Monitoring

ASPSPs MUST log:
- All API requests and responses
- Authentication attempts
- Consent creations and revocations

## 10. References

- PSD2 Directive (EU) 2015/2366
- Open Banking UK Read/Write API Specification v3.1
- OAuth 2.0 RFC 6749
- FAPI 1.0 Baseline Profile
- ISO 20022 Universal Financial Industry Message Scheme

## Appendix A: Example Flows

### A.1 Account Information Flow

1. TPP creates account access consent
2. TPP redirects PSU to ASPSP for authorization
3. PSU authenticates with SCA
4. PSU grants consent
5. ASPSP redirects to TPP with authorization code
6. TPP exchanges code for access token
7. TPP requests account data with access token

### A.2 Payment Initiation Flow

1. TPP creates payment consent
2. TPP redirects PSU to ASPSP
3. PSU authenticates with SCA and dynamic linking
4. PSU approves payment
5. ASPSP redirects to TPP with authorization code
6. TPP exchanges code for access token
7. TPP submits payment with access token
8. ASPSP processes payment

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
