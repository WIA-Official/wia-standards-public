# WIA-SOC-003 Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for e-government services, including citizen identity representation, service request structures, document formats, and transaction logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 Citizen Identity

```json
{
  "@context": "https://wiastandards.com/soc-003/v1",
  "@type": "CitizenIdentity",
  "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "identityNumber": "encrypted-id-hash",
  "givenName": "MinJun",
  "familyName": "Kim",
  "dateOfBirth": "1990-05-15",
  "nationality": "KR",
  "residency": {
    "country": "KR",
    "region": "Seoul",
    "district": "Gangnam-gu",
    "postalCode": "06000"
  },
  "contact": {
    "email": "kim.minjun@example.com",
    "phone": "+82-10-1234-5678",
    "preferredLanguage": "ko"
  },
  "verification": {
    "level": "high",
    "methods": ["biometric", "certificate"],
    "lastVerified": "2025-12-26T10:30:00Z"
  },
  "privacySettings": {
    "dataSharing": "minimal",
    "thirdPartyAccess": false,
    "retentionPeriod": "P7Y"
  }
}
```

### 2.2 Service Request

```json
{
  "@type": "ServiceRequest",
  "requestId": "REQ-2025-001234",
  "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "serviceType": "tax_filing",
  "category": "financial",
  "priority": "normal",
  "status": "submitted",
  "createdAt": "2025-12-26T14:30:00Z",
  "updatedAt": "2025-12-26T14:30:00Z",
  "estimatedCompletion": "2025-12-30T17:00:00Z",
  "metadata": {
    "taxYear": 2024,
    "filingType": "individual",
    "returnType": "standard"
  },
  "documents": [
    {
      "documentId": "DOC-2025-5678",
      "type": "tax_form",
      "format": "application/pdf",
      "size": 2458624,
      "hash": "sha256:abc123...",
      "encrypted": true
    }
  ],
  "attachments": [],
  "notes": "First time filing, requesting assistance",
  "assignedTo": "dept-taxation-001",
  "tracking": {
    "steps": [
      {
        "step": "submission",
        "completedAt": "2025-12-26T14:30:00Z",
        "status": "completed"
      },
      {
        "step": "verification",
        "status": "in_progress"
      },
      {
        "step": "processing",
        "status": "pending"
      },
      {
        "step": "approval",
        "status": "pending"
      }
    ]
  }
}
```

### 2.3 Government Service

```json
{
  "@type": "GovernmentService",
  "serviceId": "SRV-TAX-001",
  "serviceName": {
    "en": "Individual Tax Filing",
    "ko": "개인 세금 신고",
    "zh": "个人纳税申报"
  },
  "description": {
    "en": "File your annual income tax returns online",
    "ko": "연간 소득세 신고를 온라인으로 제출하세요"
  },
  "category": "taxation",
  "department": "Ministry of Economy and Finance",
  "availability": {
    "hours": "24/7",
    "maintenanceWindow": "Sunday 02:00-04:00 KST"
  },
  "requirements": {
    "authentication": "high",
    "documents": [
      "income_statement",
      "tax_withholding_certificate",
      "expense_receipts"
    ],
    "eligibility": {
      "minAge": 19,
      "citizenshipRequired": true
    }
  },
  "processing": {
    "averageTime": "P3D",
    "maxTime": "P14D",
    "autoApproval": false
  },
  "fees": {
    "serviceFee": 0,
    "currency": "KRW",
    "paymentMethods": ["bank_transfer", "credit_card", "digital_wallet"]
  },
  "endpoints": {
    "submitRequest": "/api/v1/services/tax/submit",
    "checkStatus": "/api/v1/services/tax/status/{requestId}",
    "uploadDocument": "/api/v1/documents/upload"
  }
}
```

### 2.4 Document Format

```json
{
  "@type": "GovernmentDocument",
  "documentId": "DOC-2025-5678",
  "type": "certificate",
  "subtype": "birth_certificate",
  "issuedBy": {
    "authority": "Seoul Metropolitan Government",
    "department": "Civil Affairs Bureau",
    "officerName": "Park SuJin",
    "officerId": "OFC-2025-1234"
  },
  "issuedTo": {
    "citizenId": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "name": "Kim MinJun"
  },
  "issuedDate": "2025-12-26",
  "validFrom": "2025-12-26",
  "validUntil": "2035-12-26",
  "documentNumber": "CERT-2025-BC-1234567",
  "content": {
    "format": "application/pdf",
    "encoding": "base64",
    "data": "encrypted-content-blob",
    "hash": "sha256:def456..."
  },
  "verification": {
    "method": "digital_signature",
    "algorithm": "RSA-4096",
    "signature": "signature-data",
    "certificate": "X.509-certificate-chain"
  },
  "qrCode": {
    "data": "https://verify.gov.kr/doc/CERT-2025-BC-1234567",
    "format": "QR_CODE",
    "version": 10
  },
  "blockchain": {
    "network": "government-blockchain",
    "txHash": "0x123abc...",
    "blockNumber": 1234567
  }
}
```

### 2.5 Transaction Log

```json
{
  "@type": "TransactionLog",
  "transactionId": "TXN-2025-9012",
  "timestamp": "2025-12-26T14:30:00.123Z",
  "type": "service_request",
  "action": "submit",
  "actor": {
    "type": "citizen",
    "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "ip": "203.0.113.42",
    "userAgent": "WIA-Gov-App/1.2.3 (iOS 17.0)"
  },
  "target": {
    "type": "service",
    "id": "SRV-TAX-001",
    "requestId": "REQ-2025-001234"
  },
  "result": {
    "status": "success",
    "code": 200,
    "message": "Request submitted successfully"
  },
  "audit": {
    "dataAccessed": ["citizen_profile", "tax_history"],
    "dataModified": ["service_requests"],
    "permissionsUsed": ["service.tax.submit"],
    "complianceFlags": ["GDPR", "CCPA"]
  },
  "performance": {
    "responseTime": 245,
    "serverRegion": "ap-northeast-2",
    "requestSize": 15840,
    "responseSize": 1024
  }
}
```

## 3. Data Validation Rules

### 3.1 Citizen Identity

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| citizenId | UUID | Yes | Valid UUID v4 |
| identityNumber | String | Yes | Encrypted, country-specific format |
| givenName | String | Yes | 1-100 chars, Unicode |
| familyName | String | Yes | 1-100 chars, Unicode |
| dateOfBirth | Date | Yes | ISO 8601, age >= 0 |
| nationality | String | Yes | ISO 3166-1 alpha-2 |
| email | Email | No | RFC 5322 compliant |
| phone | String | No | E.164 format |

### 3.2 Service Request

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| requestId | String | Yes | Pattern: REQ-YYYY-NNNNNN |
| citizenId | UUID | Yes | Valid citizen in system |
| serviceType | String | Yes | Enum of valid services |
| priority | String | Yes | [normal, urgent, critical] |
| status | String | Yes | [submitted, in_progress, completed, rejected] |
| createdAt | DateTime | Yes | ISO 8601 with timezone |

### 3.3 Document

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| documentId | String | Yes | Pattern: DOC-YYYY-NNNN |
| type | String | Yes | Enum of document types |
| content.format | String | Yes | MIME type |
| content.size | Integer | Yes | 0 < size <= 50MB |
| content.hash | String | Yes | SHA-256 hash |
| verification.signature | String | Yes (for official docs) | Valid digital signature |

## 4. Semantic Vocabularies

### 4.1 Service Categories

```json
{
  "taxation": {
    "label": {"en": "Taxation", "ko": "세무"},
    "subcategories": ["income_tax", "corporate_tax", "property_tax", "vat"]
  },
  "healthcare": {
    "label": {"en": "Healthcare", "ko": "보건의료"},
    "subcategories": ["medical_records", "insurance", "prescriptions"]
  },
  "civil_affairs": {
    "label": {"en": "Civil Affairs", "ko": "민원"},
    "subcategories": ["certificates", "registration", "permits"]
  },
  "legal": {
    "label": {"en": "Legal Services", "ko": "법률 서비스"},
    "subcategories": ["business_registration", "licenses", "legal_aid"]
  },
  "social_welfare": {
    "label": {"en": "Social Welfare", "ko": "사회복지"},
    "subcategories": ["benefits", "pensions", "disability_support"]
  }
}
```

### 4.2 Status Codes

```json
{
  "service_request": {
    "submitted": {"code": 100, "label": "Submitted"},
    "verified": {"code": 200, "label": "Verified"},
    "in_progress": {"code": 300, "label": "In Progress"},
    "pending_approval": {"code": 400, "label": "Pending Approval"},
    "approved": {"code": 500, "label": "Approved"},
    "completed": {"code": 600, "label": "Completed"},
    "rejected": {"code": 700, "label": "Rejected"},
    "cancelled": {"code": 800, "label": "Cancelled"}
  }
}
```

## 5. Encryption and Security

### 5.1 Data at Rest

- **Encryption**: AES-256-GCM
- **Key Management**: HSM (Hardware Security Module)
- **Key Rotation**: Every 90 days
- **PII Fields**: Always encrypted

### 5.2 Data in Transit

- **Protocol**: TLS 1.3
- **Cipher Suites**: ECDHE-RSA-AES256-GCM-SHA384
- **Certificate**: X.509 with OCSP stapling
- **Perfect Forward Secrecy**: Required

### 5.3 Sensitive Data Handling

```json
{
  "identityNumber": {
    "storage": "encrypted",
    "algorithm": "AES-256-GCM",
    "keyDerivation": "PBKDF2-SHA256",
    "display": "masked (last 4 digits only)",
    "logging": "hashed (SHA-256)"
  },
  "biometricData": {
    "storage": "encrypted + salted hash",
    "comparison": "homomorphic encryption",
    "retention": "until citizen removal request",
    "backup": "air-gapped storage"
  }
}
```

## 6. Data Retention Policies

| Data Type | Retention Period | Deletion Method |
|-----------|------------------|-----------------|
| Citizen Profile | Active + 7 years after death | Secure wipe (DoD 5220.22-M) |
| Service Requests | 7 years | Archival then secure deletion |
| Transaction Logs | 10 years | Compliance archival |
| Documents | Varies by type | According to legal requirements |
| Audit Trails | 10 years | Immutable blockchain storage |

## 7. Cross-Border Data Exchange

### 7.1 Data Transfer Format

```json
{
  "@type": "CrossBorderDataTransfer",
  "sourceCountry": "KR",
  "targetCountry": "JP",
  "legalBasis": {
    "agreement": "KR-JP-DataSharing-2024",
    "articleNumber": "Article 7.2",
    "purpose": "background_check"
  },
  "data": {
    "type": "citizen_verification",
    "minimized": true,
    "encrypted": true,
    "fields": ["name", "dateOfBirth", "citizenship"]
  },
  "consent": {
    "obtained": true,
    "timestamp": "2025-12-26T10:00:00Z",
    "expiry": "2026-12-26T10:00:00Z"
  },
  "audit": {
    "requestId": "XBORDER-2025-1234",
    "requestedBy": "immigration-dept-kr",
    "approvedBy": "data-protection-officer-kr"
  }
}
```

## 8. Accessibility Requirements

### 8.1 Multi-Language Support

All text fields MUST support:
- **Primary Language**: Based on citizen preference
- **Fallback**: English (en)
- **RTL Support**: Arabic, Hebrew, Persian, Urdu
- **Character Sets**: Unicode (UTF-8)

### 8.2 Alternative Formats

```json
{
  "document": {
    "formats": ["pdf", "html", "txt", "audio", "braille"],
    "accessibility": {
      "screenReader": "ARIA labels",
      "highContrast": true,
      "fontSize": "scalable",
      "voiceOver": "available"
    }
  }
}
```

## 9. Versioning and Compatibility

### 9.1 API Version

```json
{
  "@context": "https://wiastandards.com/soc-003/v1",
  "@version": "1.0.0",
  "compatibleWith": ["1.0.x"],
  "deprecations": [],
  "migrations": {
    "from": "0.9.x",
    "guide": "https://wiastandards.com/soc-003/migration"
  }
}
```

### 9.2 Backward Compatibility

- **Minor versions**: Must be backward compatible
- **Major versions**: Breaking changes allowed with migration path
- **Deprecation**: 12 months notice required

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.
