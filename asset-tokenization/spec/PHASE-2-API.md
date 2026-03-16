# WIA-FIN-008 Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-20
**Authors:** WIA Standards Committee

## Table of Contents

1. [Introduction](#introduction)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Token Service API](#token-service-api)
5. [Transfer Service API](#transfer-service-api)
6. [Investor Service API](#investor-service-api)
7. [Compliance Service API](#compliance-service-api)
8. [Distribution Service API](#distribution-service-api)
9. [Valuation Service API](#valuation-service-api)
10. [Webhook Events](#webhook-events)
11. [Error Handling](#error-handling)
12. [Rate Limiting](#rate-limiting)

---

## 1. Introduction

Phase 2 of WIA-FIN-008 defines standardized REST and GraphQL APIs for asset tokenization platforms. These APIs enable token creation, compliant transfers, investor management, dividend distribution, and regulatory compliance checks.

### Design Principles

- **RESTful Architecture:** HTTP methods (GET, POST, PUT, DELETE) for CRUD operations
- **OAuth 2.0 Authentication:** Industry-standard security protocol
- **Idempotency:** Safe retry of failed requests via idempotency keys
- **Versioning:** URL-based versioning (/api/v1, /api/v2)
- **OpenAPI 3.0 Specification:** Machine-readable API documentation
- **GraphQL Alternative:** Optional GraphQL endpoint for complex queries

---

## 2. API Architecture

### Base URL

```
Production: https://api.your-platform.com/v1
Sandbox:    https://sandbox-api.your-platform.com/v1
```

### Core Services

| Service | Responsibility | Base Path |
|---------|----------------|-----------|
| **Token Service** | Token creation, metadata, lifecycle | `/tokens` |
| **Transfer Service** | Compliant token transfers | `/transfers` |
| **Investor Service** | Investor management, KYC, accreditation | `/investors` |
| **Compliance Service** | Transfer validation, restrictions | `/compliance` |
| **Distribution Service** | Dividend/income payments | `/distributions` |
| **Valuation Service** | Asset appraisals, NAV updates | `/valuations` |

---

## 3. Authentication & Authorization

### OAuth 2.0 Client Credentials Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=tokens:read tokens:write investors:read

Response 200 OK:
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "tokens:read tokens:write investors:read"
}
```

### Using Access Tokens

```http
GET /api/v1/tokens
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

### Permission Scopes

| Scope | Description |
|-------|-------------|
| `tokens:read` | View token metadata and balances |
| `tokens:write` | Create and manage tokens |
| `transfers:read` | View transfer history |
| `transfers:write` | Execute token transfers |
| `investors:read` | View investor profiles |
| `investors:write` | Create and update investors |
| `compliance:read` | Check compliance rules |
| `compliance:write` | Update compliance settings |
| `distributions:read` | View dividend history |
| `distributions:write` | Create and execute distributions |
| `valuations:read` | View asset valuations |
| `valuations:write` | Submit new valuations |

---

## 4. Token Service API

### Create Token

**Endpoint:** `POST /api/v1/tokens`

**Request:**

```json
{
  "name": "Manhattan Prime Office REIT Token",
  "symbol": "MPOR",
  "assetClass": "REAL_ESTATE",
  "totalSupply": 50000000,
  "decimals": 0,
  "regulatory": {
    "framework": "REG_D_506C",
    "accreditedOnly": true,
    "lockupPeriod": "P6M",
    "jurisdictions": ["USA", "CAN"]
  },
  "blockchain": {
    "network": "polygon-mainnet",
    "standard": "ERC-1400"
  },
  "asset": {
    "type": "REAL_ESTATE",
    "address": {
      "street": "123 Main Street",
      "city": "New York",
      "state": "NY",
      "zipCode": "10001",
      "country": "USA"
    },
    "purchasePrice": 50000000,
    "appraisedValue": 52500000
  }
}
```

**Response 201 Created:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "deploymentTxHash": "0xabc123...",
  "status": "DEPLOYED",
  "createdAt": "2025-06-15T14:30:00Z",
  "estimatedGasCost": "0.15 MATIC"
}
```

### Get Token Details

**Endpoint:** `GET /api/v1/tokens/{tokenId}`

**Response 200 OK:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "name": "Manhattan Prime Office REIT Token",
  "symbol": "MPOR",
  "assetClass": "REAL_ESTATE",
  "totalSupply": 50000000,
  "circulatingSupply": 48500000,
  "holders": 450,
  "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "blockchain": {
    "network": "polygon-mainnet",
    "standard": "ERC-1400"
  },
  "currentValue": 52500000,
  "pricePerToken": 1.05,
  "marketCap": 52500000,
  "status": "ACTIVE",
  "createdAt": "2025-06-15T14:30:00Z",
  "lastValuationDate": "2025-12-01"
}
```

### List Tokens

**Endpoint:** `GET /api/v1/tokens`

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `assetClass` | string | Filter by asset class (REAL_ESTATE, ART, etc.) |
| `status` | string | Filter by status (ACTIVE, PAUSED, MATURED) |
| `page` | integer | Page number (default: 1) |
| `limit` | integer | Results per page (default: 50, max: 100) |

**Response 200 OK:**

```json
{
  "data": [
    {
      "tokenId": "TOK-2025-RE-001",
      "name": "Manhattan Prime Office REIT Token",
      "symbol": "MPOR",
      "assetClass": "REAL_ESTATE",
      "totalValue": 52500000,
      "holders": 450,
      "status": "ACTIVE"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 127,
    "pages": 3
  }
}
```

### Update Token Metadata

**Endpoint:** `PATCH /api/v1/tokens/{tokenId}`

**Request:**

```json
{
  "asset": {
    "appraisedValue": 54000000
  }
}
```

**Response 200 OK:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "updatedFields": ["asset.appraisedValue"],
  "updatedAt": "2025-12-15T10:00:00Z"
}
```

---

## 5. Transfer Service API

### Execute Transfer

**Endpoint:** `POST /api/v1/transfers`

**Request:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x1234567890abcdef1234567890abcdef12345678",
  "amount": 5000,
  "memo": "Purchase via secondary market"
}
```

**Response 202 Accepted:**

```json
{
  "transferId": "TXN-2025-001234",
  "status": "PENDING_COMPLIANCE",
  "complianceChecks": [
    {
      "check": "KYC_RECIPIENT",
      "status": "PASSED"
    },
    {
      "check": "ACCREDITATION",
      "status": "PASSED"
    },
    {
      "check": "LOCKUP_PERIOD",
      "status": "PASSED"
    },
    {
      "check": "MAX_OWNERSHIP",
      "status": "PENDING"
    }
  ],
  "estimatedCompletion": "2025-06-20T10:00:00Z"
}
```

### Get Transfer Status

**Endpoint:** `GET /api/v1/transfers/{transferId}`

**Response 200 OK:**

```json
{
  "transferId": "TXN-2025-001234",
  "tokenId": "TOK-2025-RE-001",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x1234567890abcdef1234567890abcdef12345678",
  "amount": 5000,
  "status": "COMPLETED",
  "txHash": "0xdef456...",
  "initiatedAt": "2025-06-20T09:30:00Z",
  "completedAt": "2025-06-20T09:45:00Z",
  "gasUsed": 125000,
  "gasCost": "0.05 MATIC"
}
```

### Get Transfer History

**Endpoint:** `GET /api/v1/tokens/{tokenId}/transfers`

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `address` | string | Filter by sender or recipient address |
| `startDate` | string | ISO 8601 date (e.g., 2025-01-01) |
| `endDate` | string | ISO 8601 date |
| `status` | string | Filter by status (PENDING, COMPLETED, FAILED) |
| `page` | integer | Page number |
| `limit` | integer | Results per page |

**Response 200 OK:**

```json
{
  "data": [
    {
      "transferId": "TXN-2025-001234",
      "from": "0x742d35...",
      "to": "0x123456...",
      "amount": 5000,
      "status": "COMPLETED",
      "completedAt": "2025-06-20T09:45:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 1250
  }
}
```

---

## 6. Investor Service API

### Create Investor

**Endpoint:** `POST /api/v1/investors`

**Request:**

```json
{
  "email": "investor@example.com",
  "firstName": "John",
  "lastName": "Doe",
  "dateOfBirth": "1985-03-15",
  "nationality": "USA",
  "walletAddress": "0x1234567890abcdef1234567890abcdef12345678",
  "kyc": {
    "provider": "Onfido",
    "documentType": "PASSPORT",
    "documentNumber": "P12345678"
  },
  "accreditation": {
    "method": "NET_WORTH",
    "netWorth": 2500000,
    "verificationProvider": "VerifyInvestor"
  }
}
```

**Response 201 Created:**

```json
{
  "investorId": "INVESTOR-001",
  "status": "PENDING_KYC",
  "kycVerificationUrl": "https://onfido.com/verify?token=xyz...",
  "estimatedApproval": "2025-06-16T12:00:00Z"
}
```

### Get Investor Profile

**Endpoint:** `GET /api/v1/investors/{investorId}`

**Response 200 OK:**

```json
{
  "investorId": "INVESTOR-001",
  "email": "investor@example.com",
  "firstName": "John",
  "lastName": "Doe",
  "walletAddress": "0x1234567890abcdef1234567890abcdef12345678",
  "kycStatus": "VERIFIED",
  "accreditationStatus": "VERIFIED",
  "holdings": [
    {
      "tokenId": "TOK-2025-RE-001",
      "balance": 5000,
      "value": 5250
    }
  ],
  "totalPortfolioValue": 5250,
  "createdAt": "2025-06-15T10:00:00Z"
}
```

### Get KYC Status

**Endpoint:** `GET /api/v1/investors/{investorId}/kyc`

**Response 200 OK:**

```json
{
  "investorId": "INVESTOR-001",
  "kycStatus": "VERIFIED",
  "kycProvider": "Onfido",
  "kycDate": "2025-06-16T10:30:00Z",
  "kycExpiry": "2026-06-16",
  "amlStatus": "CLEAR",
  "amlDate": "2025-06-16T10:32:00Z",
  "accreditationStatus": "VERIFIED",
  "accreditationMethod": "NET_WORTH",
  "accreditationExpiry": "2026-06-16",
  "riskLevel": "LOW",
  "approved": true
}
```

---

## 7. Compliance Service API

### Check Transfer Compliance

**Endpoint:** `POST /api/v1/compliance/check`

**Request:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
  "to": "0x1234567890abcdef1234567890abcdef12345678",
  "amount": 5000
}
```

**Response 200 OK:**

```json
{
  "allowed": true,
  "checks": [
    {
      "rule": "KYC_VERIFIED",
      "status": "PASSED",
      "message": "Recipient KYC verified on 2025-06-16"
    },
    {
      "rule": "ACCREDITATION_REQUIRED",
      "status": "PASSED",
      "message": "Recipient accredited investor (net worth method)"
    },
    {
      "rule": "LOCKUP_PERIOD",
      "status": "PASSED",
      "message": "Sender lockup expired on 2025-12-15"
    },
    {
      "rule": "MAX_OWNERSHIP_PERCENT",
      "status": "PASSED",
      "message": "Recipient ownership: 0.12% (max: 5.00%)"
    }
  ],
  "estimatedGas": 125000,
  "estimatedFee": "0.05 MATIC"
}
```

### Get Compliance Rules

**Endpoint:** `GET /api/v1/compliance/rules/{tokenId}`

**Response 200 OK:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "rules": [
    {
      "ruleId": "RULE-001",
      "type": "KYC_REQUIRED",
      "enabled": true,
      "parameters": {
        "provider": "Onfido",
        "maxAge": "P365D"
      }
    },
    {
      "ruleId": "RULE-002",
      "type": "ACCREDITATION_REQUIRED",
      "enabled": true,
      "parameters": {
        "methods": ["NET_WORTH", "INCOME"]
      }
    },
    {
      "ruleId": "RULE-003",
      "type": "LOCKUP_PERIOD",
      "enabled": true,
      "parameters": {
        "duration": "P6M"
      }
    }
  ]
}
```

---

## 8. Distribution Service API

### Create Distribution

**Endpoint:** `POST /api/v1/distributions`

**Request:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "type": "RENTAL_INCOME",
  "totalAmount": 3650000,
  "currency": "USDC",
  "recordDate": "2025-12-20",
  "paymentDate": "2025-12-31",
  "description": "Q4 2025 Net Operating Income Distribution"
}
```

**Response 201 Created:**

```json
{
  "distributionId": "DIST-2025-Q4",
  "status": "SCHEDULED",
  "eligibleTokens": 48500000,
  "amountPerToken": 0.075,
  "totalRecipients": 450,
  "estimatedGasCost": "125 MATIC",
  "scheduledPayment": "2025-12-31T00:00:00Z"
}
```

### Execute Distribution

**Endpoint:** `POST /api/v1/distributions/{distributionId}/execute`

**Response 202 Accepted:**

```json
{
  "distributionId": "DIST-2025-Q4",
  "status": "IN_PROGRESS",
  "paymentsBatches": 5,
  "currentBatch": 1,
  "paymentsProcessed": 90,
  "paymentsRemaining": 360,
  "estimatedCompletion": "2025-12-31T00:45:00Z"
}
```

### Get Distribution Status

**Endpoint:** `GET /api/v1/distributions/{distributionId}`

**Response 200 OK:**

```json
{
  "distributionId": "DIST-2025-Q4",
  "tokenId": "TOK-2025-RE-001",
  "type": "RENTAL_INCOME",
  "totalAmount": 3650000,
  "amountPerToken": 0.075,
  "status": "COMPLETED",
  "paymentsProcessed": 450,
  "totalPaid": 3650000,
  "completedAt": "2025-12-31T00:42:00Z"
}
```

---

## 9. Valuation Service API

### Submit Valuation

**Endpoint:** `POST /api/v1/valuations`

**Request:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "assetValue": 450000000,
  "valuationDate": "2025-01-15",
  "valuationMethod": "INCOME_CAPITALIZATION",
  "valuedBy": {
    "firm": "CBRE Valuation Services",
    "appraiser": "John Smith, MAI",
    "license": "NY-12345"
  },
  "assumptions": {
    "capRate": 6.4,
    "occupancyRate": 92.5,
    "noi": 28800000
  },
  "reportUrl": "ipfs://QmXyz..."
}
```

**Response 201 Created:**

```json
{
  "valuationId": "VAL-2025-001",
  "tokenId": "TOK-2025-RE-001",
  "assetValue": 450000000,
  "pricePerToken": 1.05,
  "priceChange": "+2.0%",
  "navPerToken": 3.50,
  "status": "PUBLISHED",
  "effectiveDate": "2025-01-15"
}
```

### Get Current Valuation

**Endpoint:** `GET /api/v1/valuations/current/{tokenId}`

**Response 200 OK:**

```json
{
  "tokenId": "TOK-2025-RE-001",
  "currentValue": 450000000,
  "pricePerToken": 1.05,
  "lastValuationDate": "2025-01-15",
  "valuedBy": "CBRE Valuation Services",
  "valuationMethod": "INCOME_CAPITALIZATION"
}
```

---

## 10. Webhook Events

### Configure Webhook

**Endpoint:** `POST /api/v1/webhooks`

**Request:**

```json
{
  "url": "https://your-app.com/webhooks/tokenization",
  "events": [
    "token.created",
    "transfer.completed",
    "distribution.completed"
  ],
  "secret": "your-webhook-secret"
}
```

### Event Payload

```json
{
  "eventId": "evt_2025_001234",
  "type": "transfer.completed",
  "timestamp": "2025-06-20T09:45:00Z",
  "data": {
    "transferId": "TXN-2025-001234",
    "tokenId": "TOK-2025-RE-001",
    "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "to": "0x1234567890abcdef1234567890abcdef12345678",
    "amount": 5000,
    "txHash": "0xdef456..."
  }
}
```

### Webhook Signature Verification

```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(JSON.stringify(payload))
    .digest('hex');

  return signature === `sha256=${expectedSignature}`;
}
```

---

## 11. Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "TRANSFER_NOT_ALLOWED",
    "message": "Transfer violates lockup period restriction",
    "details": {
      "rule": "LOCKUP_PERIOD",
      "lockupExpiry": "2025-12-15",
      "attemptedDate": "2025-06-20"
    },
    "requestId": "req_abc123",
    "documentation": "https://docs.wia-fin-008.com/errors/lockup-period"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request body or parameters |
| `UNAUTHORIZED` | 401 | Missing or invalid authentication token |
| `FORBIDDEN` | 403 | Insufficient permissions for operation |
| `NOT_FOUND` | 404 | Resource not found |
| `TRANSFER_NOT_ALLOWED` | 422 | Transfer violates compliance rules |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Server error |

---

## 12. Rate Limiting

### Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 42
X-RateLimit-Reset: 1735689600
```

### Tiers

| Tier | Requests/min | Burst | Price |
|------|--------------|-------|-------|
| Free | 60 | 100 | $0 |
| Starter | 600 | 1000 | $99/mo |
| Professional | 6000 | 10000 | $499/mo |
| Enterprise | Unlimited | Unlimited | Custom |

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
