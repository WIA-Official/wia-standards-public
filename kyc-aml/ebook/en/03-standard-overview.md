# Chapter 3: WIA KYC/AML Standard Overview

## Overview

The WIA KYC/AML Standard provides a comprehensive, interoperable framework for customer identity verification, risk assessment, and compliance monitoring. This chapter introduces the standard's architecture, core components, and design philosophy.

---

## Standard Vision

**Mission Statement:**
> Enable secure, efficient, and compliant customer due diligence through standardized data formats, APIs, and protocols that work across institutions, vendors, and jurisdictions.

### Guiding Principles

1. **🌐 Interoperability First** - Systems must seamlessly exchange data
2. **🔒 Privacy by Design** - Minimize data collection, maximize protection
3. **⚖️ Risk-Based Approach** - Tailor measures to actual risk
4. **🚀 Efficiency** - Reduce costs while improving effectiveness
5. **🤝 Collaboration** - Enable safe information sharing
6. **📱 Customer-Centric** - Fast, transparent, user-friendly experiences
7. **🌍 Global Applicability** - Work across jurisdictions with local customization

---

## Architecture Overview

### High-Level Components

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA KYC/AML STANDARD                     │
└─────────────────────────────────────────────────────────────┘

┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│   Identity     │  │      Risk      │  │  Transaction   │
│  Verification  │  │   Assessment   │  │   Monitoring   │
│     Layer      │  │     Engine     │  │     System     │
└───────┬────────┘  └───────┬────────┘  └───────┬────────┘
        │                   │                    │
        └───────────────────┼────────────────────┘
                            │
                  ┌─────────▼─────────┐
                  │   Screening &     │
                  │ Watchlist Module  │
                  └─────────┬─────────┘
                            │
                  ┌─────────▼─────────┐
                  │  Case Management  │
                  │    & Reporting    │
                  └───────────────────┘
```

### Component Interaction

```
Customer Onboarding Flow:

1. Customer Application
        ↓
2. Identity Verification Layer
   → Document verification
   → Biometric matching
   → Data validation
        ↓
3. Screening Module
   → Sanctions lists
   → PEP databases
   → Adverse media
        ↓
4. Risk Assessment Engine
   → Risk factors evaluation
   → Score calculation
   → Category assignment
        ↓
5. Approval Decision
        ↓
6. Ongoing Monitoring
   → Transaction monitoring
   → Periodic review
   → Alert generation
        ↓
7. Case Management (if needed)
   → Investigation
   → SAR filing
   → Reporting
```

---

## Core Components

### 1. Identity Verification Layer

**Purpose:** Confirm customer identity through multiple verification methods.

#### Capabilities

**Document Verification:**
```json
{
  "documentType": "passport",
  "verificationMethod": "automated",
  "checks": {
    "authenticity": {
      "status": "pass",
      "methods": ["MRZ_validation", "security_features", "template_matching"]
    },
    "dataExtraction": {
      "firstName": "John",
      "lastName": "Smith",
      "dateOfBirth": "1985-06-15",
      "documentNumber": "N1234567",
      "expiryDate": "2030-06-14",
      "confidence": 0.98
    },
    "tampering": {
      "status": "pass",
      "indicators": []
    }
  }
}
```

**Biometric Verification:**
```json
{
  "biometricType": "facial",
  "livenessCheck": {
    "status": "pass",
    "method": "active",
    "confidence": 0.95
  },
  "matching": {
    "status": "match",
    "similarity": 0.92,
    "threshold": 0.85
  }
}
```

**Data Verification:**
- Email validation (format, domain, deliverability)
- Phone number verification (SMS OTP, carrier lookup)
- Address verification (postal service validation)
- Database cross-checks (credit bureaus, public records)

#### Verification Levels

| Level | Methods | Use Case | Cost | Time |
|-------|---------|----------|------|------|
| **Basic** | Email + Phone | Low-risk, low-value | $0.10 | 2 min |
| **Standard** | + Government ID | General banking | $1-3 | 5 min |
| **Enhanced** | + Biometric | High-value accounts | $3-8 | 10 min |
| **Maximum** | + In-person / Video | PEPs, high-risk | $20-50 | 30+ min |

---

### 2. Risk Assessment Engine

**Purpose:** Evaluate and categorize customer risk based on multiple factors.

#### Risk Factors Framework

```
Risk Dimensions:

1. Geographic Risk
   ├── Customer residence country
   ├── Citizenship/nationality
   ├── Business operation locations
   └── Transaction destinations

2. Product/Service Risk
   ├── Account types
   ├── Transaction methods
   ├── Expected volumes
   └── Cross-border services

3. Customer Type Risk
   ├── Individual vs. business
   ├── Industry sector
   ├── Occupation
   └── Entity structure (for businesses)

4. Behavior Risk
   ├── Transaction patterns
   ├── Source of funds clarity
   ├── Business rationale
   └── Historical compliance

5. Relationship Risk
   ├── Ultimate beneficial owners
   ├── Connected parties
   ├── PEP relationships
   └── Adverse media associations
```

#### Risk Scoring Model

**Calculation:**
```
Total Risk Score =
  (Geographic × W1) +
  (Product × W2) +
  (Customer Type × W3) +
  (Behavior × W4) +
  (Relationship × W5)

Where W1...W5 are configurable weights
```

**Example:**
```json
{
  "customerId": "CUST-789012",
  "riskAssessment": {
    "overallScore": 42,
    "category": "medium",
    "factors": [
      {
        "dimension": "geographic",
        "score": 35,
        "weight": 0.25,
        "details": {
          "residence": "Singapore (Low Risk: 10)",
          "citizenship": "Malaysia (Low Risk: 10)",
          "transactionCountries": ["Thailand (Medium: 40)", "Vietnam (Medium: 45)"]
        }
      },
      {
        "dimension": "product",
        "score": 50,
        "weight": 0.20,
        "details": {
          "accountType": "Business checking with international wire capability"
        }
      },
      {
        "dimension": "customerType",
        "score": 40,
        "weight": 0.20,
        "details": {
          "type": "SME",
          "industry": "Import/Export (Medium Risk)"
        }
      },
      {
        "dimension": "behavior",
        "score": 25,
        "weight": 0.20,
        "details": {
          "expectedVolume": "Consistent with business model",
          "sourceOfFunds": "Well documented"
        }
      },
      {
        "dimension": "relationship",
        "score": 30,
        "weight": 0.15,
        "details": {
          "beneficialOwners": "Clearly identified, no PEP",
          "adverseMedia": "None found"
        }
      }
    ],
    "recommendedAction": "standard_due_diligence",
    "reviewFrequency": "annual",
    "calculatedAt": "2025-01-09T10:30:00Z"
  }
}
```

#### Risk Categories

| Category | Score Range | CDD Level | Review Frequency | Approval Authority |
|----------|-------------|-----------|------------------|-------------------|
| **Prohibited** | 90-100 | Decline | N/A | Auto-reject |
| **High** | 70-89 | EDD | Quarterly | Senior compliance |
| **Medium** | 40-69 | Standard CDD | Annual | Compliance officer |
| **Low** | 0-39 | Simplified CDD | Biennial | Automated/Junior |

---

### 3. Screening & Watchlist Module

**Purpose:** Check customers against sanctions lists, PEP databases, and adverse media sources.

#### Screening Types

**1. Sanctions Screening**

Check against:
- OFAC (US Office of Foreign Assets Control)
- UN Security Council Consolidated List
- EU Sanctions List
- UK HM Treasury Financial Sanctions
- Regional lists (DFAT Australia, SECO Switzerland, etc.)

**2. PEP Screening**

Categories:
- **Tier 1**: Heads of state, senior politicians
- **Tier 2**: Mid-level government officials
- **Tier 3**: Family members and close associates
- **RCA**: Relatives and Close Associates

**3. Adverse Media Screening**

Monitor for:
- Criminal convictions
- Financial crimes investigations
- Regulatory enforcement actions
- Corruption allegations
- Fraud reports
- Money laundering links

#### Matching Algorithm

```
Screening Process:

1. Name Normalization
   → Remove titles, punctuation
   → Standardize character encoding
   → Expand abbreviations

2. Fuzzy Matching
   → Levenshtein distance
   → Phonetic matching (Soundex, Metaphone)
   → Transliteration handling
   → Nickname/alias detection

3. Contextual Filtering
   → Date of birth matching
   → Nationality matching
   → Address matching
   → Known aliases

4. Score & Threshold
   → Match confidence: 0-100
   → Configurable threshold (typically 85+)
   → Manual review queue for 70-84
```

**Example Result:**
```json
{
  "screeningId": "SCR-2025-001234",
  "customerId": "CUST-789012",
  "screeningType": "comprehensive",
  "executedAt": "2025-01-09T10:35:00Z",
  "results": {
    "sanctions": {
      "status": "no_match",
      "listsChecked": ["OFAC_SDN", "UN_SC", "EU_SANCTIONS"],
      "totalRecordsChecked": 125000
    },
    "pep": {
      "status": "potential_match",
      "matches": [
        {
          "matchId": "PEP-2025-5678",
          "confidence": 0.78,
          "name": "Jon Smith",
          "position": "Deputy Minister of Finance",
          "country": "Hypothetica",
          "tier": 2,
          "dateOfBirth": "1985-06-10",
          "requiresReview": true,
          "reason": "Name similarity, DOB close but not exact"
        }
      ]
    },
    "adverseMedia": {
      "status": "clear",
      "articlesReviewed": 0
    },
    "overallRisk": "medium",
    "recommendedAction": "manual_review_required"
  }
}
```

---

### 4. Transaction Monitoring System

**Purpose:** Detect suspicious patterns and activities through continuous surveillance.

#### Monitoring Scenarios

**Typologies Covered:**

1. **Structuring / Smurfing**
   - Multiple transactions just below reporting thresholds
   - Split deposits across accounts/time periods

2. **Rapid Movement of Funds**
   - Funds in and out quickly (< 24-48 hours)
   - No economic rationale

3. **Round-Tripping**
   - Circular transfers between accounts
   - Same amounts going back and forth

4. **Geographic Red Flags**
   - Transactions to/from high-risk jurisdictions
   - Patterns inconsistent with customer profile

5. **Volume Anomalies**
   - Sudden increase in transaction frequency
   - Amounts significantly above normal patterns

6. **Shell Company Indicators**
   - Many payments to dormant accounts
   - Complex web of transfers with no clear purpose

#### Rule Engine

**Example Rules:**
```json
{
  "rules": [
    {
      "ruleId": "TM-001",
      "name": "High Value Cash Deposits",
      "type": "threshold",
      "condition": {
        "transactionType": "cash_deposit",
        "amount": {"greaterThan": 10000},
        "currency": "USD"
      },
      "action": "generate_alert",
      "severity": "medium"
    },
    {
      "ruleId": "TM-045",
      "name": "Rapid Funds Movement",
      "type": "behavioral",
      "condition": {
        "creditAmount": {"greaterThan": 5000},
        "debitWithin": "24 hours",
        "debitPercentage": {"greaterThan": 0.95}
      },
      "action": "generate_alert",
      "severity": "high"
    },
    {
      "ruleId": "TM-078",
      "name": "High-Risk Jurisdiction Wire",
      "type": "geographic",
      "condition": {
        "transactionType": "international_wire",
        "destinationCountry": {"in": ["FATF_HIGH_RISK_LIST"]},
        "amount": {"greaterThan": 1000}
      },
      "action": "generate_alert",
      "severity": "high"
    }
  ]
}
```

#### Machine Learning Enhancement

Beyond rules, ML models detect:
- **Peer group analysis**: Compare customer to similar profiles
- **Time-series anomalies**: Identify unusual patterns
- **Network analysis**: Detect hidden relationships
- **Typology recognition**: Learn new laundering patterns

**Benefits:**
- Reduce false positives by 50-70%
- Detect novel patterns not captured by rules
- Adapt to evolving customer behavior
- Improve investigation efficiency

---

### 5. Case Management & Reporting

**Purpose:** Manage investigations and generate regulatory reports.

#### Case Workflow

```
Alert Generated
    ↓
├─ Auto-Disposition (if clearly false positive)
└─ Manual Review Queue
    ↓
    Triage (5-10 min)
    ├─ Escalate to investigation
    └─ Close (false positive)
        ↓
        Investigation (30-60 min)
        ├─ Request additional information
        ├─ Review historical activity
        ├─ Check external databases
        └─ Document findings
            ↓
            Decision
            ├─ Clear (no suspicious activity)
            ├─ Continue monitoring
            └─ File SAR
                ↓
                SAR Preparation (2-4 hours)
                ├─ Narrative drafting
                ├─ Supporting documentation
                ├─ Supervisor review
                └─ Submit to authorities
```

#### SAR Generation

**Standard SAR Format:**
```json
{
  "sarId": "SAR-2025-001234",
  "filingInstitution": {
    "name": "Example Bank",
    "identifier": "12-3456789",
    "contactInfo": {...}
  },
  "subject": {
    "type": "individual",
    "name": "John Doe",
    "identifiers": {
      "customerId": "CUST-123456",
      "ssn": "XXX-XX-6789",
      "dob": "1980-03-15"
    },
    "address": {...}
  },
  "suspiciousActivity": {
    "type": ["structuring", "suspected_money_laundering"],
    "description": "Customer made 15 cash deposits over 30-day period, each between $9,000-$9,900, totaling $142,500. Pattern consistent with structuring to avoid CTR reporting threshold.",
    "dateRange": {
      "from": "2024-11-15",
      "to": "2024-12-15"
    },
    "totalAmount": 142500,
    "currency": "USD"
  },
  "transactions": [
    {
      "date": "2024-11-15",
      "type": "cash_deposit",
      "amount": 9850,
      "account": "XXXX-1234"
    }
    // ... additional transactions
  ],
  "narrative": "Detailed explanation of suspicious activity, investigation steps taken, and basis for filing...",
  "filedBy": {
    "name": "Jane Investigator",
    "title": "Senior AML Analyst",
    "date": "2025-01-09"
  }
}
```

#### Regulatory Reporting

**Supported Reports:**
- **SAR**: Suspicious Activity Report
- **CTR**: Currency Transaction Report
- **CMIR**: Report of International Transportation of Currency
- **FBAR**: Foreign Bank Account Report
- **DOEP**: Designation of Exempt Person
- **Custom**: Jurisdiction-specific reports

---

## Data Model

### Core Entities

#### 1. Customer Profile

```json
{
  "customerId": "CUST-789012",
  "type": "individual",
  "personalInfo": {
    "firstName": "John",
    "middleName": "Michael",
    "lastName": "Smith",
    "dateOfBirth": "1985-06-15",
    "nationality": ["USA"],
    "citizenship": "USA",
    "placeOfBirth": {
      "city": "New York",
      "country": "USA"
    }
  },
  "identityDocuments": [
    {
      "type": "passport",
      "number": "N1234567",
      "issuingCountry": "USA",
      "issueDate": "2020-06-14",
      "expiryDate": "2030-06-14",
      "verified": true,
      "verificationDate": "2025-01-09"
    }
  ],
  "contactInfo": {
    "email": "john.smith@example.com",
    "phone": "+1-555-0123",
    "address": {
      "street": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "postalCode": "94102",
      "country": "USA"
    }
  },
  "riskProfile": {
    "category": "low",
    "score": 25,
    "lastAssessment": "2025-01-09",
    "nextReview": "2027-01-09"
  },
  "pep": false,
  "onboardingDate": "2025-01-09",
  "status": "active"
}
```

#### 2. Corporate Customer (KYB)

```json
{
  "customerId": "CORP-456789",
  "type": "business",
  "companyInfo": {
    "legalName": "Example Corp Inc.",
    "tradingName": "Example Corp",
    "registrationNumber": "12-3456789",
    "registrationCountry": "USA",
    "registrationDate": "2010-03-15",
    "businessType": "Corporation",
    "industry": "Software Development",
    "naicsCode": "541511"
  },
  "beneficialOwners": [
    {
      "customerId": "CUST-111111",
      "name": "Jane Doe",
      "ownershipPercentage": 55,
      "controlType": ["ownership", "voting_rights"],
      "isPep": false
    },
    {
      "customerId": "CUST-222222",
      "name": "Robert Johnson",
      "ownershipPercentage": 45,
      "controlType": ["ownership", "voting_rights"],
      "isPep": false
    }
  ],
  "controlPersons": [
    {
      "customerId": "CUST-111111",
      "title": "CEO",
      "role": "executive_control"
    }
  ],
  "riskProfile": {
    "category": "medium",
    "score": 45,
    "lastAssessment": "2025-01-09"
  }
}
```

---

## API Architecture

### RESTful Design

**Base URL:** `https://api.wia-kyc.org/v1`

**Authentication:** OAuth 2.0 / API Key

**Standard Response Format:**
```json
{
  "status": "success",
  "data": {...},
  "metadata": {
    "requestId": "req-123456",
    "timestamp": "2025-01-09T10:30:00Z",
    "version": "1.0"
  }
}
```

### Key Endpoint Categories

| Category | Endpoints | Purpose |
|----------|-----------|---------|
| **Identity** | `/identity/verify`, `/identity/documents` | Verification operations |
| **Screening** | `/screening/sanctions`, `/screening/pep` | Watchlist checks |
| **Risk** | `/risk/assess`, `/risk/score` | Risk evaluation |
| **Monitoring** | `/monitoring/alerts`, `/monitoring/rules` | Transaction surveillance |
| **Cases** | `/cases`, `/cases/{id}/sar` | Investigation management |
| **Customers** | `/customers`, `/customers/{id}` | Customer data |

---

## Integration Patterns

### 1. Embedded Verification

Financial institution integrates verification directly into onboarding:

```
Customer Application Form
        ↓
WIA Identity Verification API
        ↓
Real-time Result → Continue or Request More Info
```

### 2. Background Screening

Batch screening of existing customers:

```
Export Customer List
        ↓
WIA Screening API (bulk)
        ↓
Receive Results → Investigate Matches
```

### 3. Continuous Monitoring

Real-time transaction monitoring:

```
Transaction Occurs
        ↓
Push to WIA Monitoring API
        ↓
Evaluate Against Rules/ML Models
        ↓
Alert if Suspicious → Case Management
```

---

## Deployment Models

### 1. Cloud SaaS
- Fully managed service
- Fastest implementation
- Per-transaction pricing
- Ideal for: Small/medium institutions, fintechs

### 2. On-Premise
- Deployed in institution's data center
- Full data control
- License-based pricing
- Ideal for: Large banks, regulatory requirements

### 3. Hybrid
- Core processing in cloud
- Sensitive data on-premise
- Flexible architecture
- Ideal for: Global banks, regulated industries

---

## Security & Privacy

### Data Protection

- **Encryption**: AES-256 at rest, TLS 1.3 in transit
- **Access Control**: Role-based (RBAC), least privilege
- **Audit Logging**: Immutable logs of all access
- **Data Minimization**: Collect only necessary fields
- **Retention**: Configurable (default: 5 years from account closure)

### Compliance Support

- **GDPR**: Right to access, erasure, portability
- **CCPA**: California Consumer Privacy Act
- **SOC 2**: Security, availability, confidentiality
- **ISO 27001**: Information security management
- **PCI DSS**: For payment-related data

---

## Key Takeaways

1. 🏗️ **Five core components**: Identity, Risk, Screening, Monitoring, Case Management
2. 📊 **Standardized data models** for interoperability
3. 🔌 **RESTful APIs** for easy integration
4. ⚖️ **Risk-based approach** aligned with FATF recommendations
5. 🤖 **ML-enhanced** detection reduces false positives
6. 🔒 **Security & privacy** built into design
7. 🌐 **Flexible deployment** options for any institution

---

**Previous**: [← Chapter 2 - Current Challenges](02-current-challenges.md) | **Next**: [Chapter 4 - Data Format →](04-data-format.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
