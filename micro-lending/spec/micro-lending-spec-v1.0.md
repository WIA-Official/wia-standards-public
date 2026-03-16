# WIA-FIN-003: Micro-Lending Standard Specification v1.0

> **弘益人間 · Benefit All Humanity**

## Document Information

- **Standard ID**: WIA-FIN-003
- **Version**: 1.0.0
- **Status**: Draft
- **Last Updated**: 2025-01-15
- **Authors**: WIA Technical Committee
- **License**: MIT

## Abstract

This specification defines a standardized protocol for peer-to-peer (P2P) micro-lending platforms. It provides data formats, API interfaces, security protocols, and integration guidelines to enable interoperable, secure, and scalable micro-lending services worldwide.

## Table of Contents

1. [Introduction](#1-introduction)
2. [Terminology](#2-terminology)
3. [Data Models](#3-data-models)
4. [API Specification](#4-api-specification)
5. [Credit Scoring](#5-credit-scoring)
6. [Matching Algorithm](#6-matching-algorithm)
7. [Security & Compliance](#7-security--compliance)
8. [Payment Integration](#8-payment-integration)
9. [Error Handling](#9-error-handling)
10. [Implementation Guidelines](#10-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

The WIA-FIN-003 standard addresses the global challenge of financial exclusion by providing a comprehensive framework for micro-lending platforms. It enables:

- Standardized data exchange between platforms
- Consistent credit assessment methodologies
- Secure transaction processing
- Regulatory compliance
- Scalable infrastructure

### 1.2 Scope

This specification covers:

- **Data Formats**: JSON schemas for all entities
- **REST APIs**: Endpoints for all operations
- **Credit Scoring**: Alternative assessment algorithms
- **Matching Logic**: Borrower-lender pairing
- **Security**: Authentication, encryption, compliance
- **Payments**: Multi-channel integration

### 1.3 Design Principles

1. **Simplicity**: Easy to understand and implement
2. **Interoperability**: Works across different platforms
3. **Security**: Protection of sensitive data
4. **Scalability**: Supports millions of users
5. **Inclusivity**: Accessible to underserved populations

---

## 2. Terminology

### 2.1 Key Terms

- **Borrower**: Individual or entity requesting a loan
- **Lender**: Individual or entity providing capital
- **Loan**: Financial agreement between borrower and lender(s)
- **Credit Score**: Numerical assessment of borrower creditworthiness (0-1000)
- **Matching**: Process of connecting borrowers with suitable lenders
- **Disbursement**: Transfer of loan funds to borrower
- **Repayment**: Return of principal and interest to lender(s)
- **Default**: Failure to repay according to agreed terms
- **KYC**: Know Your Customer - identity verification
- **AML**: Anti-Money Laundering - fraud prevention

### 2.2 Abbreviations

- **P2P**: Peer-to-Peer
- **API**: Application Programming Interface
- **REST**: Representational State Transfer
- **JSON**: JavaScript Object Notation
- **UUID**: Universally Unique Identifier
- **ISO**: International Organization for Standardization

---

## 3. Data Models

### 3.1 Borrower

```json
{
  "id": "string (UUID)",
  "type": "individual | business",
  "profile": {
    "firstName": "string",
    "lastName": "string",
    "email": "string (email format)",
    "phone": "string (E.164 format)",
    "dateOfBirth": "string (ISO 8601 date)",
    "nationalId": "string (encrypted)",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "country": "string (ISO 3166-1 alpha-2)",
      "postalCode": "string"
    }
  },
  "creditScore": "number (0-1000)",
  "kycStatus": "pending | verified | rejected",
  "kycVerifiedAt": "string (ISO 8601 datetime) | null",
  "createdAt": "string (ISO 8601 datetime)",
  "updatedAt": "string (ISO 8601 datetime)"
}
```

### 3.2 Lender

```json
{
  "id": "string (UUID)",
  "type": "individual | institutional",
  "profile": {
    "name": "string",
    "email": "string (email format)",
    "phone": "string (E.164 format)",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "country": "string (ISO 3166-1 alpha-2)",
      "postalCode": "string"
    }
  },
  "investmentProfile": {
    "totalCapital": "number",
    "availableCapital": "number",
    "investedCapital": "number",
    "riskTolerance": "conservative | moderate | aggressive",
    "preferences": {
      "minCreditScore": "number (0-1000)",
      "maxLoanAmount": "number",
      "preferredSectors": ["string"],
      "preferredRegions": ["string (ISO 3166-1 alpha-2)"]
    }
  },
  "kycStatus": "pending | verified | rejected",
  "createdAt": "string (ISO 8601 datetime)",
  "updatedAt": "string (ISO 8601 datetime)"
}
```

### 3.3 Loan

```json
{
  "id": "string (UUID)",
  "borrowerId": "string (UUID)",
  "amount": "number",
  "currency": "string (ISO 4217)",
  "purpose": "business | education | agriculture | healthcare | emergency | other",
  "description": "string",
  "term": "number (months)",
  "interestRate": "number (decimal, e.g., 0.12 for 12%)",
  "status": "draft | pending | approved | active | completed | defaulted | cancelled",
  "requestedAt": "string (ISO 8601 datetime)",
  "approvedAt": "string (ISO 8601 datetime) | null",
  "disbursedAt": "string (ISO 8601 datetime) | null",
  "completedAt": "string (ISO 8601 datetime) | null",
  "fundingProgress": {
    "targetAmount": "number",
    "fundedAmount": "number",
    "percentFunded": "number (0-100)"
  },
  "repaymentSchedule": {
    "frequency": "weekly | bi-weekly | monthly",
    "installments": [
      {
        "number": "number",
        "dueDate": "string (ISO 8601 date)",
        "principalAmount": "number",
        "interestAmount": "number",
        "totalAmount": "number",
        "status": "pending | paid | overdue | defaulted",
        "paidAt": "string (ISO 8601 datetime) | null"
      }
    ]
  },
  "lenders": [
    {
      "lenderId": "string (UUID)",
      "amount": "number",
      "percentage": "number (0-100)"
    }
  ],
  "metadata": {
    "tags": ["string"],
    "customFields": {}
  },
  "createdAt": "string (ISO 8601 datetime)",
  "updatedAt": "string (ISO 8601 datetime)"
}
```

### 3.4 Credit Score

```json
{
  "borrowerId": "string (UUID)",
  "score": "number (0-1000)",
  "rating": "excellent | good | fair | poor | very_poor",
  "factors": {
    "paymentHistory": {
      "value": "number (0-100)",
      "weight": 0.35,
      "score": "number"
    },
    "financialStability": {
      "value": "number (0-100)",
      "weight": 0.25,
      "score": "number"
    },
    "networkTrust": {
      "value": "number (0-100)",
      "weight": 0.20,
      "score": "number"
    },
    "incomeVerification": {
      "value": "number (0-100)",
      "weight": 0.15,
      "score": "number"
    },
    "educationSkills": {
      "value": "number (0-100)",
      "weight": 0.05,
      "score": "number"
    }
  },
  "dataSources": [
    {
      "type": "mobile_usage | utility_payments | social_network | transaction_history | employment",
      "verified": "boolean",
      "lastUpdated": "string (ISO 8601 datetime)"
    }
  ],
  "calculatedAt": "string (ISO 8601 datetime)",
  "expiresAt": "string (ISO 8601 datetime)"
}
```

### 3.5 Payment

```json
{
  "id": "string (UUID)",
  "loanId": "string (UUID)",
  "payerId": "string (UUID)",
  "amount": "number",
  "currency": "string (ISO 4217)",
  "method": "bank_transfer | mobile_wallet | card | cryptocurrency",
  "status": "pending | processing | completed | failed | refunded",
  "transactionId": "string",
  "processedAt": "string (ISO 8601 datetime) | null",
  "metadata": {
    "gateway": "string",
    "reference": "string"
  },
  "createdAt": "string (ISO 8601 datetime)"
}
```

---

## 4. API Specification

### 4.1 Base URL

```
https://api.wia.org/v1/micro-lending
```

### 4.2 Authentication

All API requests require authentication using API keys:

```http
Authorization: Bearer {API_KEY}
```

### 4.3 Endpoints

#### 4.3.1 Borrowers

**Create Borrower**
```http
POST /borrowers
Content-Type: application/json

{
  "type": "individual",
  "profile": { ... }
}

Response: 201 Created
{
  "id": "uuid",
  "type": "individual",
  ...
}
```

**Get Borrower**
```http
GET /borrowers/{borrowerId}

Response: 200 OK
{
  "id": "uuid",
  ...
}
```

**Update Borrower**
```http
PUT /borrowers/{borrowerId}
Content-Type: application/json

{
  "profile": { ... }
}

Response: 200 OK
```

**Get Credit Score**
```http
GET /borrowers/{borrowerId}/credit-score

Response: 200 OK
{
  "borrowerId": "uuid",
  "score": 725,
  "rating": "good",
  ...
}
```

#### 4.3.2 Lenders

**Create Lender**
```http
POST /lenders
Content-Type: application/json

{
  "type": "individual",
  "profile": { ... },
  "investmentProfile": { ... }
}

Response: 201 Created
```

**Get Lender Portfolio**
```http
GET /lenders/{lenderId}/portfolio

Response: 200 OK
{
  "totalInvested": 50000,
  "activeLoans": 24,
  "averageROI": 12.5,
  "defaultRate": 2.1,
  "loans": [ ... ]
}
```

#### 4.3.3 Loans

**Create Loan**
```http
POST /loans
Content-Type: application/json

{
  "borrowerId": "uuid",
  "amount": 5000,
  "currency": "USD",
  "purpose": "business",
  "term": 12,
  "interestRate": 0.12
}

Response: 201 Created
```

**Get Loan**
```http
GET /loans/{loanId}

Response: 200 OK
```

**List Loans**
```http
GET /loans?status=active&limit=20&offset=0

Response: 200 OK
{
  "data": [ ... ],
  "pagination": {
    "total": 150,
    "limit": 20,
    "offset": 0
  }
}
```

**Fund Loan**
```http
POST /loans/{loanId}/fund
Content-Type: application/json

{
  "lenderId": "uuid",
  "amount": 1000
}

Response: 200 OK
```

#### 4.3.4 Payments

**Make Payment**
```http
POST /payments
Content-Type: application/json

{
  "loanId": "uuid",
  "amount": 450,
  "method": "bank_transfer",
  "accountId": "uuid"
}

Response: 201 Created
```

**Get Payment Status**
```http
GET /payments/{paymentId}

Response: 200 OK
```

#### 4.3.5 Matching

**Get Matches for Loan**
```http
GET /loans/{loanId}/matches

Response: 200 OK
{
  "loanId": "uuid",
  "matches": [
    {
      "lenderId": "uuid",
      "matchScore": 0.85,
      "recommendedAmount": 500,
      "reason": "Risk profile and preferences aligned"
    }
  ]
}
```

**Get Investment Opportunities**
```http
GET /lenders/{lenderId}/opportunities

Response: 200 OK
{
  "opportunities": [
    {
      "loanId": "uuid",
      "matchScore": 0.92,
      "recommendedAmount": 1000,
      "borrowerCreditScore": 750
    }
  ]
}
```

---

## 5. Credit Scoring

### 5.1 Algorithm

The credit score is calculated using a weighted formula:

```
Score = (
  PaymentHistory × 0.35 +
  FinancialStability × 0.25 +
  NetworkTrust × 0.20 +
  IncomeVerification × 0.15 +
  EducationSkills × 0.05
) × 10
```

Where each factor is a percentage (0-100).

### 5.2 Rating Bands

| Score Range | Rating | Description |
|------------|--------|-------------|
| 800-1000 | Excellent | Prime borrowers, lowest rates |
| 650-799 | Good | Standard terms |
| 500-649 | Fair | Higher interest rates |
| 300-499 | Poor | Requires co-signer or collateral |
| 0-299 | Very Poor | Not eligible for loans |

### 5.3 Data Sources

1. **Payment History (35%)**
   - Previous loan repayments
   - Utility bill payments
   - Mobile phone bill payments
   - Credit card payments (if available)

2. **Financial Stability (25%)**
   - Income consistency
   - Savings balance
   - Debt-to-income ratio
   - Employment duration

3. **Network Trust (20%)**
   - References from community
   - Social connections quality
   - Professional network
   - Group lending history

4. **Income Verification (15%)**
   - Bank statements
   - Employer verification
   - Tax returns
   - Alternative income sources

5. **Education & Skills (5%)**
   - Educational qualifications
   - Professional certifications
   - Skill assessments
   - Training completion

### 5.4 Score Updates

Credit scores are recalculated:
- Monthly for all borrowers
- Immediately after loan repayment
- Immediately after default or late payment
- When new data sources become available

---

## 6. Matching Algorithm

### 6.1 Objective

Match borrowers with lenders to:
- Maximize borrower access to capital
- Optimize lender returns
- Minimize default risk
- Ensure portfolio diversification

### 6.2 Matching Criteria

#### For Borrowers
- Credit score meets lender's minimum
- Loan amount within lender's range
- Loan purpose matches lender preferences
- Geographic location (if specified)

#### For Lenders
- Available capital sufficient
- Risk tolerance aligned with borrower score
- Diversification rules satisfied
- Investment limits not exceeded

### 6.3 Match Score Calculation

```
MatchScore = (
  CreditScoreMatch × 0.40 +
  RiskToleranceMatch × 0.30 +
  PreferenceMatch × 0.20 +
  DiversificationBenefit × 0.10
)
```

Where:
- `CreditScoreMatch`: How well borrower score fits lender requirements (0-1)
- `RiskToleranceMatch`: Alignment of risk profiles (0-1)
- `PreferenceMatch`: Sector, region, purpose alignment (0-1)
- `DiversificationBenefit`: How much this loan improves portfolio diversity (0-1)

### 6.4 Diversification Rules

Default rules (configurable):
- Maximum 5% of capital in any single loan
- Maximum 20% in any sector
- Maximum 30% in any geographic region
- Minimum 10 different borrowers

---

## 7. Security & Compliance

### 7.1 Data Encryption

**At Rest**
- AES-256 encryption for all sensitive data
- Encrypted database fields: nationalId, bankAccount, cryptoWallet
- Separate encryption keys per environment

**In Transit**
- TLS 1.3 for all API communications
- Certificate pinning for mobile apps
- Perfect forward secrecy enabled

### 7.2 Authentication & Authorization

**API Authentication**
- API keys for service-to-service
- OAuth 2.0 for user authentication
- JWT tokens with 1-hour expiration
- Refresh tokens with 30-day expiration

**Authorization**
- Role-based access control (RBAC)
- Roles: borrower, lender, admin, auditor
- Scoped permissions per API endpoint

### 7.3 KYC/AML Compliance

**KYC Requirements**
- Government-issued photo ID
- Proof of address (utility bill, bank statement)
- Selfie verification (liveness check)
- Phone number verification (OTP)

**AML Checks**
- Sanctions list screening
- Politically exposed persons (PEP) check
- Transaction monitoring for suspicious patterns
- Large transaction reporting (>$10,000)

### 7.4 Data Privacy

**GDPR Compliance**
- Right to access personal data
- Right to rectification
- Right to erasure ("right to be forgotten")
- Data portability
- Privacy by design

**Data Retention**
- Active accounts: Indefinite
- Closed accounts: 7 years (regulatory requirement)
- Deleted accounts: 30-day grace period
- Audit logs: 10 years

### 7.5 Fraud Detection

**Rules-Based**
- Multiple accounts from same device
- Rapid succession of loan applications
- Mismatched identity information
- Blacklisted email/phone/address

**ML-Based**
- Anomaly detection in application patterns
- Behavioral biometrics analysis
- Network analysis for fraud rings
- Synthetic identity detection

---

## 8. Payment Integration

### 8.1 Supported Methods

1. **Bank Transfer**
   - ACH (US)
   - SEPA (Europe)
   - Wire transfer (International)

2. **Mobile Wallets**
   - M-Pesa (Kenya)
   - PayTM (India)
   - GCash (Philippines)
   - Alipay, WeChat Pay (China)

3. **Cards**
   - Credit cards (Visa, Mastercard, Amex)
   - Debit cards
   - Prepaid cards

4. **Cryptocurrency**
   - Bitcoin (BTC)
   - Ethereum (ETH)
   - USDC, USDT (stablecoins)

### 8.2 Payment Flow

```
1. Payment Initiation
   ↓
2. Method Validation
   ↓
3. Gateway Processing
   ↓
4. Confirmation & Receipt
   ↓
5. Distribution to Lenders
   ↓
6. Credit Score Update
```

### 8.3 Webhook Events

```http
POST {webhook_url}
Content-Type: application/json
X-WIA-Signature: {hmac_signature}

{
  "event": "payment.completed",
  "data": {
    "paymentId": "uuid",
    "loanId": "uuid",
    "amount": 450,
    "status": "completed"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

Supported events:
- `payment.initiated`
- `payment.processing`
- `payment.completed`
- `payment.failed`
- `loan.funded`
- `loan.completed`
- `loan.defaulted`

---

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_CREDIT_SCORE",
    "message": "Credit score below minimum threshold",
    "details": {
      "minimumRequired": 500,
      "actualScore": 425
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "uuid"
  }
}
```

### 9.2 Error Codes

| Code | HTTP Status | Description |
|------|------------|-------------|
| INVALID_REQUEST | 400 | Malformed request |
| UNAUTHORIZED | 401 | Invalid API key |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| INVALID_CREDIT_SCORE | 400 | Score below threshold |
| INSUFFICIENT_FUNDS | 400 | Lender lacks capital |
| KYC_NOT_VERIFIED | 403 | KYC verification required |
| LOAN_ALREADY_FUNDED | 409 | Loan fully funded |
| PAYMENT_FAILED | 402 | Payment processing error |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |

---

## 10. Implementation Guidelines

### 10.1 Environment Setup

**Development**
```
API Endpoint: https://api-dev.wia.org/v1/micro-lending
Rate Limit: 1000 req/hour
Sandbox: Yes
```

**Production**
```
API Endpoint: https://api.wia.org/v1/micro-lending
Rate Limit: 10000 req/hour
Sandbox: No
```

### 10.2 Best Practices

1. **API Usage**
   - Cache credit scores (valid for 30 days)
   - Use pagination for list endpoints
   - Implement exponential backoff for retries
   - Validate webhooks using signatures

2. **Data Management**
   - Encrypt sensitive data before storage
   - Implement audit logging
   - Regular data backups
   - GDPR-compliant data handling

3. **Performance**
   - Index database queries
   - Use CDN for static assets
   - Implement caching layers
   - Monitor API latency

4. **Security**
   - Rotate API keys quarterly
   - Use environment variables for secrets
   - Implement rate limiting
   - Regular security audits

### 10.3 Testing

**Unit Tests**
- Credit score calculation
- Matching algorithm
- Payment processing
- Error handling

**Integration Tests**
- End-to-end loan lifecycle
- Multi-lender funding
- Payment gateway integration
- Webhook delivery

**Load Tests**
- 1000 concurrent users
- 10000 loans per day
- 100000 transactions per day

---

## Appendix A: JSON Schemas

Complete JSON schemas available at:
```
https://schemas.wia.org/micro-lending/v1/
```

## Appendix B: SDKs

Official SDKs:
- TypeScript/JavaScript: `@wia/micro-lending`
- Python: `wia-micro-lending`
- Java: `org.wia.microlending`
- Go: `github.com/wia-official/micro-lending-go`

## Appendix C: References

1. Basel III Framework for Credit Risk
2. ISO 20022 Financial Messaging Standard
3. PCI DSS Payment Card Industry Data Security Standard
4. GDPR General Data Protection Regulation
5. FATF Recommendations on AML/CFT

---

© 2025 SmileStory Inc. / WIA

**弘益人間 · Benefit All Humanity**

License: MIT
