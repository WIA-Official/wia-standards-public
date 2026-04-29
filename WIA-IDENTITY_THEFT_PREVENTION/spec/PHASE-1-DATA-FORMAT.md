# WIA-IDENTITY_THEFT_PREVENTION: PHASE 1 - DATA FORMAT SPECIFICATION

**Version:** 1.0.0
**Status:** APPROVED
**Last Updated:** 2026-01-12
**Authors:** WIA Technical Committee on Identity Security

---

## Table of Contents

1. [Introduction](#introduction)
2. [Core Data Structures](#core-data-structures)
3. [Identity Profile Format](#identity-profile-format)
4. [Alert and Threat Data](#alert-and-threat-data)
5. [Biometric Data Format](#biometric-data-format)
6. [Credit Monitoring Data](#credit-monitoring-data)
7. [Dark Web Scan Results](#dark-web-scan-results)
8. [Breach Notification Format](#breach-notification-format)
9. [Data Validation Rules](#data-validation-rules)
10. [Security and Privacy](#security-and-privacy)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the standardized data formats for the WIA-IDENTITY_THEFT_PREVENTION standard, enabling consistent representation of identity protection data, threat intelligence, monitoring alerts, and authentication credentials across all compliant systems.

### 1.2 Scope

This document covers:
- Identity profile data structures
- Alert and threat intelligence formats
- Biometric authentication data
- Credit monitoring information
- Dark web scan results
- Breach notification formats
- Data validation and integrity mechanisms

### 1.3 Design Principles

1. **Privacy First**: All data formats include encryption and anonymization requirements
2. **Interoperability**: JSON-based formats for maximum compatibility
3. **Real-Time Ready**: Optimized for streaming and instant alerts
4. **Compliance Built-In**: GDPR, CCPA, and PCI-DSS compliant by design
5. **Extensibility**: Version-aware schemas for future enhancements

---

## 2. Core Data Structures

### 2.1 Base Identity Object

```json
{
  "identity": {
    "id": "uuid-v4",
    "version": "1.0",
    "timestamp": "ISO-8601",
    "profile": {
      "firstName": "string",
      "lastName": "string",
      "dateOfBirth": "YYYY-MM-DD",
      "ssn": "encrypted-string",
      "email": "string",
      "phone": "E.164-format",
      "addresses": [
        {
          "type": "home|work|other",
          "street": "string",
          "city": "string",
          "state": "string",
          "postalCode": "string",
          "country": "ISO-3166-alpha-2"
        }
      ]
    },
    "verification": {
      "status": "verified|pending|unverified",
      "method": "biometric|document|knowledge-based",
      "verifiedAt": "ISO-8601",
      "confidence": 0.0-1.0
    },
    "metadata": {
      "createdAt": "ISO-8601",
      "updatedAt": "ISO-8601",
      "source": "string",
      "checksum": "sha256-hash"
    }
  }
}
```

### 2.2 Authentication Credential Format

```json
{
  "credential": {
    "id": "uuid-v4",
    "type": "password|biometric|token|certificate",
    "status": "active|suspended|revoked",
    "strength": {
      "score": 0-100,
      "factors": ["password", "biometric", "token"],
      "mfaEnabled": true,
      "riskLevel": "low|medium|high|critical"
    },
    "biometric": {
      "type": "fingerprint|face|iris|voice",
      "templateHash": "sha256-hash",
      "enrolledAt": "ISO-8601",
      "deviceId": "string"
    },
    "token": {
      "type": "totp|hotp|push|sms",
      "secret": "encrypted-string",
      "algorithm": "sha1|sha256|sha512",
      "digits": 6,
      "period": 30
    },
    "passwordPolicy": {
      "minLength": 12,
      "requireUppercase": true,
      "requireLowercase": true,
      "requireDigits": true,
      "requireSpecial": true,
      "maxAge": 90,
      "historySize": 10
    },
    "lastUsed": "ISO-8601",
    "expiresAt": "ISO-8601"
  }
}
```

### 2.3 Session and Activity Log

```json
{
  "session": {
    "id": "uuid-v4",
    "userId": "uuid-v4",
    "status": "active|expired|terminated",
    "authentication": {
      "method": "password|biometric|sso|oauth",
      "timestamp": "ISO-8601",
      "ipAddress": "IPv4|IPv6",
      "userAgent": "string",
      "location": {
        "country": "ISO-3166-alpha-2",
        "region": "string",
        "city": "string",
        "coordinates": {
          "lat": -90.0-90.0,
          "lon": -180.0-180.0
        }
      }
    },
    "riskScore": {
      "overall": 0-100,
      "factors": {
        "deviceTrust": 0-100,
        "locationAnomaly": 0-100,
        "behaviorPattern": 0-100,
        "velocityCheck": 0-100
      }
    },
    "activities": [
      {
        "action": "login|access|modify|download",
        "resource": "string",
        "timestamp": "ISO-8601",
        "result": "success|failure|blocked",
        "details": "object"
      }
    ],
    "createdAt": "ISO-8601",
    "expiresAt": "ISO-8601"
  }
}
```

---

## 3. Identity Profile Format

### 3.1 Complete Identity Record

```json
{
  "identityRecord": {
    "header": {
      "recordId": "uuid-v4",
      "version": "1.0.0",
      "schemaVersion": "wia-itp-v1",
      "createdAt": "ISO-8601",
      "updatedAt": "ISO-8601",
      "status": "active|frozen|locked|closed"
    },
    "personalInformation": {
      "legalName": {
        "prefix": "string",
        "firstName": "string",
        "middleName": "string",
        "lastName": "string",
        "suffix": "string",
        "maiden": "string"
      },
      "aliases": [
        {
          "name": "string",
          "type": "legal|nickname|business",
          "validFrom": "ISO-8601",
          "validTo": "ISO-8601"
        }
      ],
      "demographics": {
        "dateOfBirth": "YYYY-MM-DD",
        "placeOfBirth": "string",
        "gender": "string",
        "nationality": "ISO-3166-alpha-2",
        "citizenship": ["ISO-3166-alpha-2"]
      },
      "identifiers": {
        "ssn": {
          "value": "encrypted-string",
          "country": "US",
          "verifiedAt": "ISO-8601"
        },
        "driverLicense": {
          "number": "encrypted-string",
          "state": "string",
          "expiresAt": "YYYY-MM-DD"
        },
        "passport": {
          "number": "encrypted-string",
          "country": "ISO-3166-alpha-2",
          "expiresAt": "YYYY-MM-DD"
        },
        "nationalId": {
          "number": "encrypted-string",
          "country": "ISO-3166-alpha-2",
          "type": "string"
        }
      }
    },
    "contactInformation": {
      "emails": [
        {
          "address": "string",
          "type": "personal|work|other",
          "verified": true,
          "primary": true,
          "verifiedAt": "ISO-8601"
        }
      ],
      "phones": [
        {
          "number": "E.164-format",
          "type": "mobile|home|work",
          "verified": true,
          "primary": true,
          "verifiedAt": "ISO-8601"
        }
      ],
      "addresses": [
        {
          "type": "residential|mailing|work",
          "street1": "string",
          "street2": "string",
          "city": "string",
          "state": "string",
          "postalCode": "string",
          "country": "ISO-3166-alpha-2",
          "validFrom": "ISO-8601",
          "validTo": "ISO-8601",
          "verified": true
        }
      ]
    },
    "financialProfiles": [
      {
        "institution": "string",
        "accountType": "checking|savings|credit|loan|investment",
        "accountNumber": "encrypted-string",
        "status": "active|closed|frozen",
        "openedAt": "YYYY-MM-DD",
        "monitoringEnabled": true
      }
    ],
    "digitalFootprint": {
      "onlineAccounts": [
        {
          "platform": "string",
          "username": "string",
          "email": "string",
          "registeredAt": "ISO-8601",
          "status": "active|disabled|deleted"
        }
      ],
      "socialMedia": [
        {
          "platform": "string",
          "profileUrl": "url",
          "username": "string",
          "visibility": "public|private|restricted"
        }
      ]
    },
    "protectionSettings": {
      "monitoringLevel": "basic|standard|premium|enterprise",
      "alertPreferences": {
        "channels": ["email", "sms", "push", "voice"],
        "frequency": "realtime|hourly|daily|weekly",
        "threshold": "low|medium|high|critical"
      },
      "freezeStatus": {
        "credit": "frozen|unfrozen",
        "accounts": ["string"]
      }
    }
  }
}
```

---

## 4. Alert and Threat Data

### 4.1 Security Alert Format

```json
{
  "alert": {
    "id": "uuid-v4",
    "type": "identity_theft|fraud_attempt|data_breach|suspicious_activity|credential_compromise",
    "severity": "info|low|medium|high|critical",
    "status": "new|acknowledged|investigating|resolved|false_positive",
    "priority": 1-5,
    "timestamp": "ISO-8601",
    "detection": {
      "method": "ml_model|rule_based|manual|third_party",
      "confidence": 0.0-1.0,
      "source": "string",
      "ruleId": "string"
    },
    "subject": {
      "userId": "uuid-v4",
      "identityId": "uuid-v4",
      "affectedAccounts": ["string"],
      "affectedCredentials": ["string"]
    },
    "threat": {
      "category": "string",
      "description": "string",
      "indicators": [
        {
          "type": "ip|domain|email|phone|hash",
          "value": "string",
          "confidence": 0.0-1.0
        }
      ],
      "attackVector": "phishing|malware|social_engineering|data_breach|insider",
      "impactAssessment": {
        "financial": "low|medium|high",
        "reputation": "low|medium|high",
        "legal": "low|medium|high"
      }
    },
    "details": {
      "title": "string",
      "summary": "string",
      "fullDescription": "string",
      "evidence": [
        {
          "type": "log|screenshot|document|network_capture",
          "url": "string",
          "hash": "sha256",
          "timestamp": "ISO-8601"
        }
      ],
      "timeline": [
        {
          "timestamp": "ISO-8601",
          "event": "string",
          "actor": "string"
        }
      ]
    },
    "response": {
      "actions": [
        {
          "type": "notify|freeze|reset|investigate|escalate",
          "status": "pending|completed|failed",
          "performedBy": "system|user|analyst",
          "timestamp": "ISO-8601",
          "notes": "string"
        }
      ],
      "recommendations": [
        {
          "action": "string",
          "priority": "immediate|urgent|normal",
          "description": "string"
        }
      ]
    },
    "notifications": [
      {
        "channel": "email|sms|push|voice",
        "recipient": "string",
        "sentAt": "ISO-8601",
        "status": "sent|delivered|failed|bounced"
      }
    ],
    "metadata": {
      "assignedTo": "string",
      "tags": ["string"],
      "relatedAlerts": ["uuid-v4"],
      "externalReferences": ["string"]
    }
  }
}
```

### 4.2 Threat Intelligence Feed

```json
{
  "threatIntel": {
    "id": "uuid-v4",
    "feedId": "string",
    "version": "1.0",
    "publishedAt": "ISO-8601",
    "validUntil": "ISO-8601",
    "source": {
      "provider": "string",
      "reliability": "A|B|C|D|E",
      "credibility": 0.0-1.0
    },
    "indicators": [
      {
        "type": "ip|domain|url|email|hash|phone",
        "value": "string",
        "category": "malicious|suspicious|benign",
        "confidence": 0.0-1.0,
        "firstSeen": "ISO-8601",
        "lastSeen": "ISO-8601",
        "tags": ["string"],
        "context": {
          "malwareFamily": "string",
          "campaignId": "string",
          "threatActor": "string",
          "ttps": ["MITRE-ATT&CK-ID"]
        }
      }
    ],
    "patterns": [
      {
        "type": "behavioral|signature|anomaly",
        "description": "string",
        "detection": "string",
        "severity": "low|medium|high|critical"
      }
    ]
  }
}
```

---

## 5. Biometric Data Format

### 5.1 Biometric Template

```json
{
  "biometric": {
    "id": "uuid-v4",
    "userId": "uuid-v4",
    "modalit": "fingerprint|face|iris|voice|palm|behavioral",
    "version": "1.0",
    "template": {
      "format": "ISO-19794|ANSI-378|proprietary",
      "encoding": "base64",
      "data": "encrypted-base64-string",
      "hash": "sha256",
      "compression": "none|jpeg2000|png"
    },
    "quality": {
      "score": 0-100,
      "nfiqScore": 1-5,
      "resolution": "dpi",
      "captureMethod": "optical|capacitive|ultrasonic|camera"
    },
    "enrollment": {
      "deviceId": "string",
      "deviceType": "scanner|camera|sensor",
      "enrolledAt": "ISO-8601",
      "enrolledBy": "user|operator|system",
      "location": "string",
      "attempts": 1-10
    },
    "metadata": {
      "leftRight": "left|right|both|na",
      "fingerPosition": "thumb|index|middle|ring|pinky",
      "faceAngle": "frontal|profile|three-quarter",
      "irisEye": "left|right|both"
    },
    "security": {
      "encryptionAlgorithm": "AES-256-GCM",
      "keyId": "string",
      "antiSpoofing": {
        "liveness": "passed|failed",
        "method": "pulsation|3d_depth|challenge_response",
        "confidence": 0.0-1.0
      }
    },
    "lifecycle": {
      "status": "active|expired|revoked",
      "expiresAt": "ISO-8601",
      "lastVerified": "ISO-8601",
      "verificationCount": 0
    }
  }
}
```

### 5.2 Biometric Verification Result

```json
{
  "verification": {
    "id": "uuid-v4",
    "timestamp": "ISO-8601",
    "templateId": "uuid-v4",
    "userId": "uuid-v4",
    "modality": "fingerprint|face|iris|voice",
    "result": {
      "matched": true,
      "confidence": 0.0-1.0,
      "score": 0.0-1.0,
      "threshold": 0.0-1.0,
      "decision": "accept|reject|review"
    },
    "capture": {
      "quality": 0-100,
      "deviceId": "string",
      "attempt": 1,
      "livenessCheck": "passed|failed"
    },
    "context": {
      "ipAddress": "string",
      "location": "string",
      "userAgent": "string",
      "sessionId": "uuid-v4"
    },
    "failureReasons": [
      {
        "code": "string",
        "message": "string",
        "category": "quality|liveness|match|system"
      }
    ]
  }
}
```

---

## 6. Credit Monitoring Data

### 6.1 Credit Report Format

```json
{
  "creditReport": {
    "id": "uuid-v4",
    "userId": "uuid-v4",
    "bureau": "equifax|experian|transunion",
    "reportDate": "ISO-8601",
    "reportType": "full|summary|monitoring",
    "personalInfo": {
      "name": "string",
      "ssn": "encrypted-string",
      "dateOfBirth": "YYYY-MM-DD",
      "currentAddress": "object",
      "previousAddresses": ["object"]
    },
    "creditScore": {
      "score": 300-850,
      "model": "FICO-8|FICO-9|VantageScore-3|VantageScore-4",
      "factors": [
        {
          "code": "string",
          "description": "string",
          "impact": "positive|negative|neutral"
        }
      ],
      "scoreHistory": [
        {
          "date": "YYYY-MM-DD",
          "score": 300-850
        }
      ]
    },
    "accounts": [
      {
        "id": "string",
        "type": "credit_card|mortgage|auto_loan|student_loan|personal_loan",
        "status": "open|closed|paid|charged_off|collections",
        "creditor": "string",
        "accountNumber": "encrypted-string",
        "openedDate": "YYYY-MM-DD",
        "closedDate": "YYYY-MM-DD",
        "creditLimit": 0,
        "currentBalance": 0,
        "highBalance": 0,
        "paymentStatus": "current|30_days|60_days|90_days|120_days",
        "paymentHistory": [
          {
            "date": "YYYY-MM",
            "status": "OK|30|60|90|120"
          }
        ]
      }
    ],
    "inquiries": [
      {
        "date": "YYYY-MM-DD",
        "creditor": "string",
        "type": "hard|soft",
        "purpose": "string"
      }
    ],
    "publicRecords": [
      {
        "type": "bankruptcy|tax_lien|judgment|foreclosure",
        "filedDate": "YYYY-MM-DD",
        "status": "active|satisfied|discharged",
        "amount": 0,
        "court": "string"
      }
    ],
    "alerts": [
      {
        "type": "fraud_alert|credit_freeze|security_alert",
        "status": "active|expired",
        "placedDate": "YYYY-MM-DD",
        "expiresDate": "YYYY-MM-DD"
      }
    ]
  }
}
```

### 6.2 Credit Change Alert

```json
{
  "creditAlert": {
    "id": "uuid-v4",
    "userId": "uuid-v4",
    "timestamp": "ISO-8601",
    "type": "new_account|inquiry|score_change|address_change|derogatory",
    "severity": "info|warning|critical",
    "change": {
      "category": "string",
      "before": "object",
      "after": "object",
      "delta": "object"
    },
    "details": {
      "creditor": "string",
      "accountNumber": "encrypted-string",
      "amount": 0,
      "date": "YYYY-MM-DD"
    },
    "fraudIndicators": {
      "suspicious": true,
      "reasons": ["string"],
      "riskScore": 0-100
    },
    "recommendations": [
      {
        "action": "string",
        "urgency": "low|medium|high|critical",
        "description": "string"
      }
    ]
  }
}
```

---

## 7. Dark Web Scan Results

### 7.1 Dark Web Finding

```json
{
  "darkWebFinding": {
    "id": "uuid-v4",
    "scanId": "uuid-v4",
    "userId": "uuid-v4",
    "discoveredAt": "ISO-8601",
    "status": "new|confirmed|mitigated|false_positive",
    "severity": "low|medium|high|critical",
    "source": {
      "type": "marketplace|forum|paste|breach_database",
      "url": "tor-url|i2p-url",
      "name": "string",
      "discovered": "ISO-8601",
      "lastSeen": "ISO-8601"
    },
    "exposedData": {
      "categories": ["email", "password", "ssn", "credit_card", "address", "phone"],
      "items": [
        {
          "type": "email|password|ssn|credit_card|bank_account|personal_info",
          "value": "redacted-or-partial",
          "hash": "sha256",
          "confidence": 0.0-1.0,
          "context": "string"
        }
      ]
    },
    "breach": {
      "name": "string",
      "company": "string",
      "breachDate": "YYYY-MM-DD",
      "publicDisclosure": "YYYY-MM-DD",
      "recordCount": 0,
      "dataClasses": ["string"]
    },
    "threat": {
      "type": "credentials|pii|financial|medical|corporate",
      "potentialImpact": "low|medium|high|critical",
      "exploitability": "low|medium|high",
      "indicators": [
        {
          "type": "string",
          "value": "string"
        }
      ]
    },
    "verification": {
      "status": "pending|verified|invalid",
      "method": "automated|manual|user_confirmed",
      "verifiedAt": "ISO-8601",
      "verifiedBy": "string"
    },
    "mitigation": {
      "actions": [
        {
          "type": "notify|reset_password|freeze_credit|contact_institution",
          "status": "pending|completed|failed",
          "timestamp": "ISO-8601"
        }
      ],
      "recommendations": ["string"]
    }
  }
}
```

### 7.2 Dark Web Monitoring Report

```json
{
  "monitoringReport": {
    "id": "uuid-v4",
    "userId": "uuid-v4",
    "reportPeriod": {
      "start": "ISO-8601",
      "end": "ISO-8601"
    },
    "summary": {
      "totalFindings": 0,
      "newFindings": 0,
      "criticalFindings": 0,
      "mitigatedFindings": 0,
      "monitoredAssets": [
        {
          "type": "email|phone|ssn|credit_card|username",
          "count": 0
        }
      ]
    },
    "findings": ["darkWebFinding-id"],
    "trends": {
      "timeSeriesData": [
        {
          "date": "YYYY-MM-DD",
          "findings": 0,
          "severity": "low|medium|high|critical"
        }
      ],
      "topSources": [
        {
          "source": "string",
          "count": 0
        }
      ],
      "dataTypes": [
        {
          "type": "string",
          "count": 0
        }
      ]
    },
    "recommendations": [
      {
        "priority": "immediate|high|medium|low",
        "action": "string",
        "description": "string"
      }
    ]
  }
}
```

---

## 8. Breach Notification Format

### 8.1 Data Breach Notice

```json
{
  "breachNotification": {
    "id": "uuid-v4",
    "breachId": "string",
    "status": "investigating|confirmed|contained|resolved",
    "severity": "low|medium|high|critical",
    "reportedAt": "ISO-8601",
    "confirmedAt": "ISO-8601",
    "organization": {
      "name": "string",
      "industry": "string",
      "size": "small|medium|large|enterprise",
      "website": "url"
    },
    "incident": {
      "discoveredDate": "YYYY-MM-DD",
      "occurredDate": "YYYY-MM-DD",
      "publicDisclosure": "YYYY-MM-DD",
      "type": "hacking|malware|insider|physical|misconfiguration",
      "attackVector": "string",
      "description": "string"
    },
    "impact": {
      "recordsAffected": 0,
      "usersAffected": 0,
      "dataTypes": [
        {
          "category": "personal|financial|health|authentication|other",
          "fields": ["name", "email", "password", "ssn", "credit_card"],
          "sensitivity": "low|medium|high|critical"
        }
      ],
      "geographicScope": ["ISO-3166-alpha-2"],
      "estimatedCost": 0
    },
    "affectedUsers": [
      {
        "userId": "uuid-v4",
        "exposedData": ["string"],
        "riskLevel": "low|medium|high|critical",
        "notified": true,
        "notifiedAt": "ISO-8601"
      }
    ],
    "response": {
      "containmentActions": [
        {
          "action": "string",
          "timestamp": "ISO-8601",
          "status": "completed|in_progress|planned"
        }
      ],
      "remediationSteps": [
        {
          "step": "string",
          "responsible": "string",
          "deadline": "ISO-8601",
          "status": "completed|in_progress|not_started"
        }
      ],
      "userRecommendations": [
        {
          "action": "change_password|monitor_credit|freeze_accounts",
          "priority": "immediate|high|medium|low",
          "instructions": "string"
        }
      ]
    },
    "compliance": {
      "regulations": ["GDPR", "CCPA", "HIPAA", "PCI-DSS"],
      "notifications": [
        {
          "authority": "string",
          "notifiedAt": "ISO-8601",
          "referenceNumber": "string"
        }
      ],
      "legalActions": [
        {
          "type": "investigation|lawsuit|fine|settlement",
          "status": "string",
          "details": "string"
        }
      ]
    },
    "timeline": [
      {
        "timestamp": "ISO-8601",
        "event": "string",
        "actor": "string",
        "details": "string"
      }
    ],
    "metadata": {
      "source": "company|researcher|media|government",
      "verified": true,
      "references": ["url"],
      "tags": ["string"]
    }
  }
}
```

---

## 9. Data Validation Rules

### 9.1 Field Validation

All data must adhere to the following validation rules:

| Field Type | Validation Rule | Format | Example |
|------------|----------------|--------|---------|
| UUID | RFC 4122 v4 | `^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$` | `550e8400-e29b-41d4-a716-446655440000` |
| ISO-8601 Timestamp | UTC timezone | `YYYY-MM-DDTHH:mm:ss.sssZ` | `2026-01-12T14:30:00.000Z` |
| Email | RFC 5322 | Standard email format | `user@example.com` |
| Phone | E.164 | `+[country][number]` | `+12025551234` |
| SSN (US) | Encrypted 9-digit | Encrypted format | `(encrypted)` |
| Credit Card | Encrypted Luhn-valid | Encrypted format | `(encrypted)` |
| IP Address | IPv4 or IPv6 | Standard notation | `192.168.1.1` or `2001:db8::1` |
| Country Code | ISO 3166-1 alpha-2 | Two-letter code | `US`, `GB`, `DE` |
| Score (0-1) | Floating point | `0.0` to `1.0` | `0.85` |
| Score (0-100) | Integer | `0` to `100` | `85` |
| Credit Score | Integer | `300` to `850` | `720` |
| Hash (SHA-256) | Hexadecimal | 64 characters | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |

### 9.2 Data Integrity

All records must include:

```json
{
  "integrity": {
    "checksum": "sha256-hash-of-record",
    "signature": "digital-signature",
    "signedBy": "certificate-id",
    "signedAt": "ISO-8601",
    "algorithm": "RSA-2048|ECDSA-P256"
  }
}
```

### 9.3 Version Control

```json
{
  "versioning": {
    "schemaVersion": "1.0.0",
    "recordVersion": 1,
    "previousVersion": "uuid-v4",
    "changeLog": [
      {
        "version": 1,
        "timestamp": "ISO-8601",
        "changedBy": "user-id|system",
        "changes": [
          {
            "field": "string",
            "operation": "create|update|delete",
            "oldValue": "encrypted",
            "newValue": "encrypted"
          }
        ]
      }
    ]
  }
}
```

---

## 10. Security and Privacy

### 10.1 Encryption Requirements

All sensitive data must be encrypted:

| Data Type | Encryption Method | Key Length | Algorithm |
|-----------|------------------|------------|-----------|
| PII (at rest) | AES-GCM | 256-bit | AES-256-GCM |
| PII (in transit) | TLS | 256-bit | TLS 1.3 |
| Biometric templates | AES-GCM | 256-bit | AES-256-GCM |
| Authentication tokens | HMAC | 256-bit | HMAC-SHA256 |
| Passwords | Argon2id | N/A | Argon2id (m=64MB, t=3, p=4) |
| Credit card data | PCI-DSS compliant | 256-bit | AES-256-GCM |

### 10.2 Data Minimization

Systems must implement:
- **Purpose Limitation**: Collect only data necessary for identity protection
- **Retention Limits**: Automatically purge data after defined retention period
- **Access Controls**: Role-based access with least privilege principle
- **Anonymization**: Remove or hash PII when full data not required
- **Consent Management**: Track and enforce user consent preferences

### 10.3 Privacy Controls

```json
{
  "privacySettings": {
    "dataRetention": {
      "personalInfo": "365d",
      "activityLogs": "90d",
      "alertHistory": "730d",
      "biometricTemplates": "until_revoked"
    },
    "consent": {
      "processing": true,
      "monitoring": true,
      "thirdPartySharing": false,
      "marketing": false,
      "grantedAt": "ISO-8601",
      "expiresAt": "ISO-8601"
    },
    "rights": {
      "dataAccess": true,
      "dataPortability": true,
      "dataErasure": true,
      "objectionToProcessing": false
    }
  }
}
```

### 10.4 Audit Trail

```json
{
  "auditLog": {
    "id": "uuid-v4",
    "timestamp": "ISO-8601",
    "action": "create|read|update|delete|access|export",
    "resource": {
      "type": "identity|alert|report|credential",
      "id": "uuid-v4"
    },
    "actor": {
      "id": "uuid-v4",
      "type": "user|system|admin|api",
      "ipAddress": "string",
      "userAgent": "string"
    },
    "result": "success|failure|denied",
    "changes": "encrypted-diff",
    "reason": "string"
  }
}
```

---

## Appendix A: JSON Schema Definitions

Complete JSON Schema files are available at:
- `https://wia.org/schemas/identity-theft-prevention/v1/identity.json`
- `https://wia.org/schemas/identity-theft-prevention/v1/alert.json`
- `https://wia.org/schemas/identity-theft-prevention/v1/biometric.json`
- `https://wia.org/schemas/identity-theft-prevention/v1/credit.json`
- `https://wia.org/schemas/identity-theft-prevention/v1/darkweb.json`
- `https://wia.org/schemas/identity-theft-prevention/v1/breach.json`

---

## Appendix B: Change Log

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-01-12 | Initial specification release |

---

**Document Status:** APPROVED
**Next Review:** 2026-07-12
**Contact:** standards@wia.org

---

© 2026 World Certification Industry Association (WIA)
弘益人間 (홍익인간) · Benefit All Humanity
