# WIA-UNI-004: Economic Integration Standard v1.0

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

This specification defines the WIA-UNI-004 standard for inter-Korean economic integration, providing a comprehensive framework for implementing trade facilitation, investment coordination, joint ventures, and special economic zones that promote peaceful cooperation and mutual prosperity.

### 1.2 Design Principles

- **Mutual Benefit:** Ensuring win-win economic outcomes for all parties
- **Transparency:** Clear processes, monitoring, and reporting mechanisms
- **Compliance:** Built-in sanctions screening and regulatory compliance
- **Gradualism:** Phased implementation from pilot to full-scale operations
- **Sustainability:** Economically viable and environmentally responsible

### 1.3 Normative References

- UN Comtrade: International Trade Statistics Database
- WTO Trade Facilitation Agreement: World Trade Organization standards
- FATF Recommendations: Financial Action Task Force AML/CFT standards
- ISO 3166-1: Country codes
- ISO 4217: Currency codes

---

## 2. Scope

### 2.1 In Scope

- Cross-border trade documentation and customs processing
- Foreign direct investment protocols and capital flow management
- Joint venture establishment and governance frameworks
- Special economic zone management and operations
- Compliance monitoring and risk assessment
- Infrastructure development coordination

### 2.2 Out of Scope

- Domestic trade within South or North Korea separately
- Military or security-related cooperation
- Political negotiations and diplomatic protocols
- Humanitarian aid (covered by separate frameworks)

---

## 3. Data Format Specification

### 3.1 Trade Documentation Format

#### 3.1.1 Commercial Invoice

```json
{
  "documentType": "COMMERCIAL_INVOICE",
  "documentId": "INV-KOR-2025-001234",
  "issueDate": "2025-12-25",
  "exporter": {
    "name": "Company Name",
    "country": "KR",
    "address": "Seoul, South Korea",
    "taxId": "123-45-67890",
    "contactPerson": "John Kim"
  },
  "importer": {
    "name": "Importer Company",
    "country": "KP",
    "address": "Kaesong, North Korea",
    "registrationId": "NK-456-789"
  },
  "items": [
    {
      "description": "Electronic Components",
      "hsCode": "8542.31",
      "quantity": 1000,
      "unit": "pieces",
      "unitPrice": 5.50,
      "totalValue": 5500.00,
      "currency": "USD",
      "originCountry": "KR"
    }
  ],
  "totalValue": 5500.00,
  "currency": "USD",
  "paymentTerms": "NET30",
  "incoterms": "CIF"
}
```

### 3.2 Investment Application Format

```json
{
  "applicationType": "FOREIGN_DIRECT_INVESTMENT",
  "applicationId": "FDI-2025-567890",
  "submissionDate": "2025-12-25",
  "investor": {
    "name": "Investment Company Ltd.",
    "country": "KR",
    "legalForm": "CORPORATION",
    "registrationNumber": "110111-1234567",
    "capital": {
      "amount": 10000000,
      "currency": "USD"
    }
  },
  "project": {
    "name": "Electronics Manufacturing Facility",
    "location": "Kaesong Industrial Complex",
    "sector": "MANUFACTURING",
    "investmentAmount": {
      "total": 10000000,
      "currency": "USD",
      "cashContribution": 7000000,
      "inKindContribution": 3000000
    },
    "timeline": {
      "startDate": "2026-01-01",
      "duration": 10,
      "unit": "years"
    },
    "employment": {
      "estimatedJobs": 500,
      "localHires": 450,
      "expatriates": 50
    }
  },
  "requestedIncentives": [
    "TAX_HOLIDAY",
    "DUTY_EXEMPTION",
    "PROFIT_REPATRIATION_GUARANTEE"
  ]
}
```

### 3.3 Joint Venture Agreement Format

```json
{
  "agreementType": "EQUITY_JOINT_VENTURE",
  "agreementId": "JV-2025-KOR-001",
  "effectiveDate": "2026-01-01",
  "parties": [
    {
      "name": "South Korean Partner Co.",
      "country": "KR",
      "equityShare": 60,
      "capitalContribution": {
        "amount": 3000000,
        "currency": "USD",
        "type": "CASH"
      }
    },
    {
      "name": "North Korean Partner Enterprise",
      "country": "KP",
      "equityShare": 40,
      "capitalContribution": {
        "amount": 2000000,
        "currency": "USD",
        "type": "IN_KIND",
        "description": "Land use rights and buildings"
      }
    }
  ],
  "governance": {
    "boardSize": 5,
    "chairmanAppointment": "ROTATING",
    "majorDecisionThreshold": 75,
    "ordinaryDecisionThreshold": 51
  },
  "profitDistribution": {
    "method": "PROPORTIONAL_TO_EQUITY",
    "frequency": "ANNUAL",
    "retentionRatio": 20
  },
  "disputeResolution": {
    "method": "INTERNATIONAL_ARBITRATION",
    "venue": "Stockholm Chamber of Commerce",
    "governingLaw": "UNCITRAL"
  }
}
```

---

## 4. API Specification

### 4.1 Trade API Endpoints

**Base URL:** `https://api.wia.org/uni-004/v1`

#### 4.1.1 Submit Trade Declaration

```
POST /trade/declarations
Content-Type: application/json

Request Body:
{
  "declaration": { ... },
  "documents": [...]
}

Response:
{
  "declarationId": "TRD-2025-001234",
  "status": "PENDING_REVIEW",
  "submissionTime": "2025-12-25T10:00:00Z",
  "estimatedProcessingTime": "3-5 business days"
}
```

#### 4.1.2 Get Trade Status

```
GET /trade/declarations/{declarationId}

Response:
{
  "declarationId": "TRD-2025-001234",
  "status": "APPROVED",
  "currentStage": "CUSTOMS_CLEARED",
  "timeline": [...]
}
```

### 4.2 Investment API Endpoints

#### 4.2.1 Submit Investment Application

```
POST /investment/applications
Content-Type: application/json

Request Body:
{
  "application": { ... },
  "supportingDocuments": [...]
}

Response:
{
  "applicationId": "INV-2025-567890",
  "status": "UNDER_REVIEW",
  "reviewCommittee": "Joint Investment Committee",
  "estimatedDecision": "30-60 days"
}
```

---

## 5. Security Requirements

### 5.1 Data Encryption

- All data in transit MUST use TLS 1.3 or higher
- All sensitive data at rest MUST be encrypted with AES-256
- Cryptographic keys MUST be managed using HSM or equivalent

### 5.2 Authentication

- API access requires OAuth 2.0 authentication
- Multi-factor authentication for administrative access
- API keys with minimum 256-bit entropy

### 5.3 Access Control

- Role-based access control (RBAC) implementation
- Principle of least privilege
- Audit logging of all access and modifications

---

## 6. Compliance Requirements

### 6.1 Sanctions Compliance

- Real-time screening against UN, OFAC, EU sanctions lists
- Mandatory Enhanced Due Diligence for all transactions
- Suspicious activity reporting to relevant authorities

### 6.2 AML/CFT Requirements

- Customer Due Diligence (CDD) for all parties
- Source of funds verification
- Transaction monitoring and pattern analysis
- Record retention for minimum 7 years

### 6.3 Regulatory Reporting

- Quarterly reports to relevant authorities
- Annual compliance certification
- Real-time incident reporting for violations

---

## 7. Implementation Guidelines

### 7.1 Phase 1: Pilot (Months 1-6)

- Limited scope trade (< $1M per transaction)
- Single SEZ operation
- Small-scale joint ventures (< 50 employees)
- Comprehensive monitoring and evaluation

### 7.2 Phase 2: Expansion (Months 7-18)

- Increased trade volume caps
- Multiple SEZ locations
- Larger joint ventures
- Infrastructure development projects

### 7.3 Phase 3: Scale (Months 19+)

- Full commercial operations
- Regional economic integration
- Comprehensive regulatory harmonization

---

**© 2025 SmileStory Inc. / WIA**  
**弘益人間 (홍익인간) · Benefit All Humanity**
