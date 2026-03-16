# WIA-FIN-014: Cross-Border Payment Standard v1.0

**Status:** Official Standard
**Version:** 1.0.0
**Date:** December 25, 2025
**Authors:** WIA Standards Committee

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Data Format Specification](#3-data-format-specification)
4. [API Specification](#4-api-specification)
5. [Security Requirements](#5-security-requirements)
6. [Compliance Requirements](#6-compliance-requirements)
7. [Implementation Guidelines](#7-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the WIA-FIN-014 standard for cross-border payment systems, providing a comprehensive framework for implementing international money transfers that are fast, cost-effective, secure, and compliant with global regulations.

### 1.2 Design Principles

- **Interoperability:** Compatible with SWIFT, SEPA, blockchain, and other payment networks
- **Security:** End-to-end encryption, fraud detection, and secure key management
- **Compliance:** Built-in AML/KYC, sanctions screening, and regulatory reporting
- **Performance:** Sub-second to real-time settlement capabilities
- **Scalability:** Horizontal scaling to handle millions of transactions per day

### 1.3 Normative References

- ISO 20022: Financial Services - Universal financial industry message scheme
- SWIFT MT/MX Messages: SWIFT message standards
- PCI DSS v4.0: Payment Card Industry Data Security Standard
- FATF Recommendations: Financial Action Task Force AML/CFT standards

---

## 2. Scope

### 2.1 In Scope

- Cross-border payment initiation and processing
- Multi-currency support and FX conversion
- Multiple payment rails (SWIFT, SEPA, blockchain, RTP)
- AML/KYC compliance automation
- Transaction monitoring and fraud detection
- Settlement and reconciliation

### 2.2 Out of Scope

- Domestic payment processing (covered by other standards)
- Card payment processing (see WIA-FIN-008)
- Cryptocurrency trading (see WIA-FIN-003)

---

## 3. Data Format Specification

### 3.1 Payment Message Format

Based on ISO 20022 with extensions for modern payment rails.

#### 3.1.1 Payment Initiation Message

```json
{
  "messageType": "pain.001.001.09",
  "messageId": "string (UUID)",
  "creationDateTime": "ISO 8601 timestamp",
  "numberOfTransactions": "integer",
  "controlSum": "decimal",
  "initiatingParty": {
    "name": "string (max 140 chars)",
    "identification": {
      "type": "PASSPORT | NATIONAL_ID | TAX_ID",
      "number": "string",
      "country": "ISO 3166-1 alpha-2"
    },
    "address": {
      "street": "string",
      "city": "string",
      "postalCode": "string",
      "country": "ISO 3166-1 alpha-2"
    }
  },
  "paymentInformation": {
    "paymentInformationId": "string",
    "paymentMethod": "SWIFT | SEPA | BLOCKCHAIN | RTP",
    "requestedExecutionDate": "ISO 8601 date",
    "debtorAccount": {
      "currency": "ISO 4217 currency code",
      "identification": "IBAN | Account Number"
    },
    "creditorAccount": {
      "currency": "ISO 4217 currency code",
      "identification": "IBAN | Account Number"
    },
    "amount": {
      "instructedAmount": "decimal",
      "currency": "ISO 4217 currency code",
      "equivalentAmount": "decimal (optional)",
      "exchangeRate": "decimal (optional)"
    },
    "chargeBearer": "DEBT | CRED | SHAR",
    "remittanceInformation": {
      "unstructured": "string (max 140 chars)",
      "structured": "object (optional)"
    }
  }
}
```

#### 3.1.2 Payment Status Message

```json
{
  "messageType": "pacs.002.001.10",
  "messageId": "string (UUID)",
  "originalMessageId": "string (UUID)",
  "creationDateTime": "ISO 8601 timestamp",
  "status": "PENDING | PROCESSING | COMPLETED | FAILED | REVERSED",
  "statusReason": "string (optional)",
  "transactionReference": "string",
  "settlementInformation": {
    "settlementMethod": "INDA | INGA | COVE",
    "settlementDate": "ISO 8601 date",
    "settlementTime": "ISO 8601 timestamp"
  },
  "charges": [
    {
      "type": "TRANSACTION_FEE | FX_MARKUP | NETWORK_FEE",
      "amount": "decimal",
      "currency": "ISO 4217 currency code"
    }
  ]
}
```

### 3.2 Currency Codes

All currency codes MUST conform to ISO 4217 (3-letter alphabetic codes).

### 3.3 Country Codes

All country codes MUST conform to ISO 3166-1 alpha-2 (2-letter codes).

---

## 4. API Specification

### 4.1 Base URL

```
Production: https://api.wia.org/fin-014/v1
Sandbox: https://sandbox-api.wia.org/fin-014/v1
```

### 4.2 Authentication

All API requests MUST include authentication via API key in the header:

```http
Authorization: Bearer {api_key}
```

### 4.3 Core Endpoints

#### 4.3.1 Create Payment

**Endpoint:** `POST /payments`

**Request Body:**
```json
{
  "beneficiaryId": "string",
  "amount": "decimal",
  "currency": "ISO 4217 code",
  "purpose": "string",
  "reference": "string (optional)",
  "method": "SWIFT | SEPA | BLOCKCHAIN | RTP",
  "metadata": "object (optional)"
}
```

**Response:** `201 Created`
```json
{
  "id": "string (UUID)",
  "status": "PENDING",
  "estimatedDelivery": "ISO 8601 timestamp",
  "fees": "decimal",
  "exchangeRate": "decimal",
  "recipientAmount": "decimal"
}
```

#### 4.3.2 Get Payment Status

**Endpoint:** `GET /payments/{id}`

**Response:** `200 OK`
```json
{
  "id": "string",
  "status": "PENDING | PROCESSING | COMPLETED | FAILED",
  "createdAt": "ISO 8601 timestamp",
  "updatedAt": "ISO 8601 timestamp",
  "completedAt": "ISO 8601 timestamp (optional)",
  "transactionReference": "string"
}
```

#### 4.3.3 Get FX Rate

**Endpoint:** `GET /fx/rates?from={currency}&to={currency}`

**Response:** `200 OK`
```json
{
  "from": "ISO 4217 code",
  "to": "ISO 4217 code",
  "rate": "decimal",
  "timestamp": "ISO 8601 timestamp",
  "expiresAt": "ISO 8601 timestamp"
}
```

#### 4.3.4 Create Beneficiary

**Endpoint:** `POST /beneficiaries`

**Request Body:**
```json
{
  "name": "string",
  "country": "ISO 3166-1 alpha-2",
  "currency": "ISO 4217 code",
  "accountNumber": "string",
  "bankCode": "string",
  "accountType": "SAVINGS | CHECKING",
  "address": {
    "street": "string",
    "city": "string",
    "postalCode": "string",
    "country": "ISO 3166-1 alpha-2"
  }
}
```

**Response:** `201 Created`
```json
{
  "id": "string (UUID)",
  "status": "VERIFIED | PENDING_VERIFICATION",
  "createdAt": "ISO 8601 timestamp"
}
```

### 4.4 Error Responses

All errors follow RFC 7807 Problem Details format:

```json
{
  "type": "https://api.wia.org/errors/insufficient-balance",
  "title": "Insufficient Balance",
  "status": 400,
  "detail": "Account balance insufficient for transaction",
  "instance": "/payments/abc123"
}
```

**Standard Error Codes:**
- `400` Bad Request
- `401` Unauthorized
- `403` Forbidden
- `404` Not Found
- `429` Too Many Requests
- `500` Internal Server Error
- `503` Service Unavailable

---

## 5. Security Requirements

### 5.1 Transport Security

- All API communication MUST use TLS 1.3 or higher
- Certificate pinning RECOMMENDED for mobile applications
- HTTP Strict Transport Security (HSTS) MUST be enabled

### 5.2 Data Encryption

- Sensitive data at rest MUST be encrypted using AES-256
- Payment credentials MUST be stored in HSM (Hardware Security Module)
- Database encryption MUST be enabled

### 5.3 API Security

- API keys MUST be at least 256 bits entropy
- Rate limiting MUST be implemented (default: 100 requests/minute)
- Request signing RECOMMENDED for sensitive operations

### 5.4 Compliance

- PCI DSS Level 1 compliance REQUIRED
- SOC 2 Type II audit RECOMMENDED
- ISO 27001 certification RECOMMENDED

---

## 6. Compliance Requirements

### 6.1 KYC Requirements

- Customer identity verification REQUIRED for all users
- Enhanced Due Diligence (EDD) for high-risk customers
- Beneficial ownership verification for entities

### 6.2 AML Requirements

- Transaction monitoring REQUIRED for all payments
- Suspicious Activity Reports (SAR) filing as per local regulations
- Record retention minimum 7 years

### 6.3 Sanctions Screening

- Real-time screening against OFAC, UN, EU sanctions lists
- Daily list updates REQUIRED
- False positive resolution procedures REQUIRED

---

## 7. Implementation Guidelines

### 7.1 Payment Lifecycle

1. **Initiation:** Client submits payment request
2. **Validation:** System validates request format and data
3. **Compliance Check:** AML/KYC and sanctions screening
4. **Routing:** Intelligent routing to optimal payment rail
5. **Processing:** Payment submitted to network
6. **Settlement:** Funds settled and confirmed
7. **Notification:** Client notified of completion

### 7.2 Testing Requirements

- Unit test coverage minimum 80%
- Integration tests for all API endpoints
- Load testing to 10x expected peak volume
- Security penetration testing annually

### 7.3 Monitoring

- 99.9% uptime SLA
- Real-time alerting for failures
- Transaction success rate monitoring
- Latency monitoring (p50, p95, p99)

---

## Appendix A: Sample Implementation

See `/examples/typescript` for complete SDK implementation example.

## Appendix B: Changelog

### Version 1.0.0 (2025-12-25)
- Initial release
- Core payment processing
- Multi-rail support
- AML/KYC compliance

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
