# WIA-UNI-013: Currency Integration Standard v1.0

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

This specification defines the WIA-UNI-013 standard for inter-Korean currency integration, providing a comprehensive framework for implementing exchange rate management, currency conversion, banking infrastructure integration, and digital currency systems that promote peaceful cooperation and economic prosperity on the Korean Peninsula.

### 1.2 Design Principles

- **Mutual Benefit:** Ensuring fair and equitable economic outcomes for all parties
- **Transparency:** Clear processes, real-time reporting, and auditable transactions
- **Security:** Multi-layered security protecting against fraud, counterfeiting, and cyber threats
- **Compliance:** Built-in AML/CFT, sanctions screening, and regulatory compliance
- **Gradualism:** Phased implementation from pilot to full-scale deployment
- **Interoperability:** Compatibility with international standards and systems

### 1.3 Normative References

- ISO 4217: Currency codes (KRW, KPW, USD, etc.)
- ISO 3166-1: Country codes
- ISO 20022: Financial messaging standard
- FATF Recommendations: AML/CFT standards
- BIS Principles: Central Bank Digital Currencies
- SWIFT MT/MX Standards: International payment messages

---

## 2. Scope

### 2.1 In Scope

- Exchange rate determination and management between KRW and KPW
- Currency conversion and cross-border payment systems
- Banking infrastructure integration and interoperability
- Central Bank Digital Currency (CBDC) frameworks
- Monetary policy coordination mechanisms
- Security, compliance, and risk management protocols

### 2.2 Out of Scope

- Fiscal policy and government budgeting
- Trade agreements and tariff regulations
- Political negotiations and diplomatic protocols
- Individual bank's internal operations (beyond integration requirements)

---

## 3. Data Format Specification

### 3.1 Exchange Rate Format

```json
{
  "rateId": "RATE-2025-001234",
  "effectiveDate": "2025-12-25T00:00:00Z",
  "expiryDate": "2025-12-26T00:00:00Z",
  "baseCurrency": "KRW",
  "targetCurrency": "KPW",
  "rate": 120.00,
  "rateType": "OFFICIAL" | "MARKET" | "MANAGED_FLOAT",
  "source": "JOINT_MONETARY_COMMITTEE",
  "confidence": "HIGH" | "MEDIUM" | "LOW",
  "volatilityIndex": 0.05
}
```

### 3.2 Currency Conversion Request Format

```json
{
  "conversionId": "CVT-2025-789012",
  "timestamp": "2025-12-25T10:30:00Z",
  "fromCurrency": "KRW",
  "toCurrency": "KPW",
  "amount": 1000000,
  "transactionType": "INDIVIDUAL_TRANSFER" | "BUSINESS_PAYMENT" | "TRADE_SETTLEMENT" | "REMITTANCE",
  "priority": "INSTANT" | "STANDARD" | "BULK" | "SCHEDULED",
  "sender": {
    "id": "USER-123456",
    "name": "John Kim",
    "country": "KR",
    "accountNumber": "110-123-456789"
  },
  "recipient": {
    "id": "USER-789012",
    "name": "Min Park",
    "country": "KP",
    "accountNumber": "NK-456-789012"
  }
}
```

### 3.3 Currency Conversion Response Format

```json
{
  "conversionId": "CVT-2025-789012",
  "status": "COMPLETED" | "PENDING" | "FAILED",
  "processedAt": "2025-12-25T10:30:02Z",
  "exchangeRate": 120.00,
  "convertedAmount": 120000000,
  "fees": {
    "conversionFee": 3000,
    "currency": "KRW",
    "feePercentage": 0.003
  },
  "settlement": {
    "method": "RTGS" | "DNS" | "BLOCKCHAIN",
    "settlementTime": "T+0",
    "confirmationNumber": "CONF-2025-111111"
  }
}
```

### 3.4 CBDC Transaction Format

```json
{
  "transactionId": "CBDC-TX-2025-999999",
  "timestamp": "2025-12-25T10:30:00Z",
  "cbdcType": "UNIFIED_KCBDC",
  "transactionType": "TRANSFER" | "PAYMENT" | "ISSUANCE" | "REDEMPTION",
  "amount": 50000,
  "from": {
    "walletId": "WALLET-KR-123456",
    "walletType": "PERSONAL" | "BUSINESS" | "GOVERNMENT",
    "balance": 150000
  },
  "to": {
    "walletId": "WALLET-KP-789012",
    "walletType": "PERSONAL" | "BUSINESS" | "GOVERNMENT"
  },
  "metadata": {
    "purpose": "Merchant payment",
    "merchantId": "MERCHANT-456789",
    "location": "Kaesong SEZ"
  },
  "signature": "0x1234567890abcdef...",
  "blockchainHash": "0xabcdef1234567890..."
}
```

---

## 4. API Specification

### 4.1 Base URL

```
Production: https://api.wia.org/uni-013/v1
Sandbox: https://sandbox.wia.org/uni-013/v1
```

### 4.2 Authentication

All API requests require OAuth 2.0 authentication with Bearer token:

```
Authorization: Bearer {access_token}
```

### 4.3 Exchange Rate API

#### 4.3.1 Get Current Exchange Rate

```
GET /exchange-rates/current
Query Parameters:
  - baseCurrency: string (required)
  - targetCurrency: string (required)

Response:
{
  "success": true,
  "data": {
    "rate": 120.00,
    "rateId": "RATE-2025-001234",
    "effectiveDate": "2025-12-25T00:00:00Z"
  },
  "timestamp": "2025-12-25T10:30:00Z"
}
```

#### 4.3.2 Get Historical Exchange Rates

```
GET /exchange-rates/historical
Query Parameters:
  - baseCurrency: string (required)
  - targetCurrency: string (required)
  - startDate: ISO 8601 date (required)
  - endDate: ISO 8601 date (required)

Response:
{
  "success": true,
  "data": {
    "rates": [
      {
        "date": "2025-12-24",
        "rate": 119.50,
        "rateId": "RATE-2025-001233"
      },
      // ...
    ]
  },
  "timestamp": "2025-12-25T10:30:00Z"
}
```

### 4.4 Currency Conversion API

#### 4.4.1 Convert Currency

```
POST /conversions
Content-Type: application/json

Request Body:
{
  "fromCurrency": "KRW",
  "toCurrency": "KPW",
  "amount": 1000000,
  "transactionType": "INDIVIDUAL_TRANSFER",
  "priority": "INSTANT",
  "sender": {...},
  "recipient": {...}
}

Response:
{
  "success": true,
  "data": {
    "conversionId": "CVT-2025-789012",
    "status": "COMPLETED",
    "convertedAmount": 120000000,
    "exchangeRate": 120.00,
    "fees": {...}
  },
  "timestamp": "2025-12-25T10:30:02Z"
}
```

#### 4.4.2 Get Conversion Status

```
GET /conversions/{conversionId}

Response:
{
  "success": true,
  "data": {
    "conversionId": "CVT-2025-789012",
    "status": "COMPLETED",
    "processedAt": "2025-12-25T10:30:02Z",
    "timeline": [...]
  },
  "timestamp": "2025-12-25T10:35:00Z"
}
```

### 4.5 CBDC API

#### 4.5.1 Create CBDC Wallet

```
POST /cbdc/wallets
Content-Type: application/json

Request Body:
{
  "walletType": "PERSONAL",
  "ownerInfo": {
    "name": "John Kim",
    "idNumber": "123456-1234567",
    "country": "KR"
  },
  "initialBalance": 0
}

Response:
{
  "success": true,
  "data": {
    "walletId": "WALLET-KR-123456",
    "walletType": "PERSONAL",
    "balance": 0,
    "createdAt": "2025-12-25T10:30:00Z"
  }
}
```

#### 4.5.2 Transfer CBDC

```
POST /cbdc/transfers
Content-Type: application/json

Request Body:
{
  "fromWalletId": "WALLET-KR-123456",
  "toWalletId": "WALLET-KP-789012",
  "amount": 50000,
  "purpose": "Payment for goods"
}

Response:
{
  "success": true,
  "data": {
    "transactionId": "CBDC-TX-2025-999999",
    "status": "COMPLETED",
    "blockchainHash": "0xabcdef1234567890...",
    "timestamp": "2025-12-25T10:30:01Z"
  }
}
```

---

## 5. Security Requirements

### 5.1 Data Encryption

- All data in transit MUST use TLS 1.3 or higher
- All sensitive data at rest MUST be encrypted with AES-256
- Cryptographic keys MUST be managed using Hardware Security Modules (HSM)
- Key rotation MUST occur at least annually

### 5.2 Authentication

- Multi-factor authentication required for all transactions > $10,000
- Biometric authentication recommended for digital wallet access
- OAuth 2.0 with JWT tokens for API access
- Session timeout maximum 30 minutes for inactive sessions

### 5.3 Transaction Security

- Digital signatures required for all CBDC transactions
- Transaction limits enforced at multiple levels (per-transaction, daily, monthly)
- Real-time fraud detection using machine learning
- Mandatory transaction confirmation for large amounts

---

## 6. Compliance Requirements

### 6.1 AML/CFT

- Customer Due Diligence (CDD) for all account holders
- Enhanced Due Diligence (EDD) for high-risk customers
- Transaction monitoring with automated suspicious activity detection
- Suspicious Activity Reports (SAR) filed within 24 hours
- Record retention minimum 7 years

### 6.2 Sanctions Compliance

- Real-time screening against UN, OFAC, EU sanctions lists
- Blocking of prohibited transactions
- Regular list updates (within 24 hours of changes)
- Manual review of potential matches

### 6.3 Regulatory Reporting

- Daily transaction volume reports
- Monthly compliance certification
- Quarterly system audits
- Annual independent third-party audits

---

## 7. Implementation Guidelines

### 7.1 Phase 1: Pilot (Months 1-12)

- Deploy in Special Economic Zones only
- Limited transaction volumes (< $1M per month per entity)
- CBDC pilot with 10,000-50,000 users
- Comprehensive monitoring and evaluation

### 7.2 Phase 2: Expansion (Months 13-36)

- Expand to major cities
- Increased transaction limits
- CBDC rollout to millions of users
- Integration with existing banking infrastructure

### 7.3 Phase 3: Full Deployment (Months 37+)

- Peninsula-wide availability
- Full integration with international payment systems
- Advanced features (smart contracts, programmable currency)
- Preparation for monetary unification

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
