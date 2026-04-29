# WIA-IDENTITY_THEFT_PREVENTION: PHASE 2 - API INTERFACE SPECIFICATION

**Version:** 1.0.0
**Status:** APPROVED
**Last Updated:** 2026-01-12
**Authors:** WIA Technical Committee on Identity Security

---

## Table of Contents

1. [Introduction](#introduction)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Identity Management APIs](#identity-management-apis)
5. [Monitoring & Alert APIs](#monitoring--alert-apis)
6. [Credit Monitoring APIs](#credit-monitoring-apis)
7. [Dark Web Scanning APIs](#dark-web-scanning-apis)
8. [Biometric Authentication APIs](#biometric-authentication-apis)
9. [Breach Intelligence APIs](#breach-intelligence-apis)
10. [Reporting & Analytics APIs](#reporting--analytics-apis)
11. [Webhook & Event APIs](#webhook--event-apis)
12. [Error Handling](#error-handling)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the RESTful API interfaces for the WIA-IDENTITY_THEFT_PREVENTION standard, enabling applications to integrate comprehensive identity protection, monitoring, and threat detection capabilities.

### 1.2 API Design Principles

1. **RESTful Design**: Resource-based URLs with standard HTTP methods
2. **Security First**: All endpoints require authentication and encryption
3. **Rate Limiting**: Prevents abuse and ensures fair usage
4. **Idempotency**: Safe retries for critical operations
5. **Versioning**: Backward compatibility through API versioning
6. **Real-Time**: WebSocket support for instant alerts
7. **OpenAPI 3.0**: Complete specification available

### 1.3 Base URL

```
Production: https://api.wia.org/identity-theft-prevention/v1
Sandbox: https://sandbox-api.wia.org/identity-theft-prevention/v1
```

### 1.4 Common Headers

```http
Authorization: Bearer {access_token}
Content-Type: application/json
Accept: application/json
X-API-Version: 1.0
X-Request-ID: {uuid-v4}
X-Client-ID: {client-id}
```

---

## 2. API Architecture

### 2.1 Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    API Gateway                          │
│  (Authentication, Rate Limiting, Load Balancing)        │
└─────────────────┬───────────────────────────────────────┘
                  │
    ┌─────────────┼─────────────┬──────────────┬──────────┐
    │             │             │              │          │
┌───▼───┐   ┌────▼────┐   ┌────▼────┐   ┌────▼────┐ ┌──▼──┐
│Identity│   │Monitoring│   │ Credit │   │Dark Web│ │Breach│
│Service │   │ Service  │   │Service │   │Service │ │Intel │
└────────┘   └──────────┘   └─────────┘   └────────┘ └─────┘
     │             │              │             │         │
     └─────────────┴──────────────┴─────────────┴─────────┘
                           │
                   ┌───────▼────────┐
                   │  Data Storage  │
                   │ (Encrypted DB) │
                   └────────────────┘
```

### 2.2 Service Components

| Service | Purpose | Base Path |
|---------|---------|-----------|
| Identity Service | Manage identity profiles and credentials | `/identities` |
| Monitoring Service | Real-time threat monitoring and alerts | `/monitoring` |
| Credit Service | Credit report access and monitoring | `/credit` |
| Dark Web Service | Dark web scanning and findings | `/darkweb` |
| Biometric Service | Biometric enrollment and verification | `/biometric` |
| Breach Intel | Data breach intelligence and notifications | `/breaches` |
| Analytics Service | Reports, trends, and insights | `/analytics` |

### 2.3 Rate Limits

| Tier | Requests/Minute | Requests/Day | Concurrent Connections |
|------|----------------|--------------|----------------------|
| Free | 10 | 1,000 | 2 |
| Basic | 60 | 10,000 | 5 |
| Standard | 300 | 100,000 | 20 |
| Premium | 1,000 | 1,000,000 | 100 |
| Enterprise | Custom | Custom | Custom |

Rate limit headers:
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1641945600
```

---

## 3. Authentication & Authorization

### 3.1 OAuth 2.0 Authentication

**Get Access Token**

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id={client_id}
&client_secret={client_secret}
&scope=identity:read identity:write monitoring:read alerts:manage
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "identity:read identity:write monitoring:read alerts:manage",
  "refresh_token": "refresh_token_here"
}
```

### 3.2 API Key Authentication

```http
GET /identities/{id}
X-API-Key: wia_live_sk_1234567890abcdef
```

### 3.3 Authorization Scopes

| Scope | Description |
|-------|-------------|
| `identity:read` | Read identity profiles |
| `identity:write` | Create/update identity profiles |
| `monitoring:read` | View monitoring status and alerts |
| `monitoring:write` | Configure monitoring settings |
| `alerts:manage` | Create, update, resolve alerts |
| `credit:read` | Access credit reports and scores |
| `darkweb:read` | View dark web scan results |
| `biometric:enroll` | Enroll biometric templates |
| `biometric:verify` | Verify biometric authentication |
| `breach:read` | Access breach intelligence |
| `reports:read` | Generate and view reports |
| `admin:all` | Full administrative access |

### 3.4 Multi-Factor Authentication

**Initiate MFA Challenge**

```http
POST /auth/mfa/challenge
Authorization: Bearer {access_token}

{
  "userId": "uuid-v4",
  "method": "totp|sms|push|biometric"
}
```

Response:
```json
{
  "challengeId": "uuid-v4",
  "method": "totp",
  "expiresIn": 300,
  "qrCode": "data:image/png;base64,..."
}
```

**Verify MFA**

```http
POST /auth/mfa/verify
Authorization: Bearer {access_token}

{
  "challengeId": "uuid-v4",
  "code": "123456"
}
```

---

## 4. Identity Management APIs

### 4.1 Create Identity Profile

```http
POST /identities
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "personalInformation": {
    "legalName": {
      "firstName": "John",
      "lastName": "Doe"
    },
    "demographics": {
      "dateOfBirth": "1990-01-15",
      "nationality": "US"
    },
    "identifiers": {
      "ssn": "123-45-6789"
    }
  },
  "contactInformation": {
    "emails": [
      {
        "address": "john.doe@example.com",
        "type": "personal",
        "primary": true
      }
    ],
    "phones": [
      {
        "number": "+12025551234",
        "type": "mobile",
        "primary": true
      }
    ]
  },
  "protectionSettings": {
    "monitoringLevel": "premium",
    "alertPreferences": {
      "channels": ["email", "sms", "push"],
      "frequency": "realtime",
      "threshold": "low"
    }
  }
}
```

Response:
```json
{
  "id": "uuid-v4",
  "status": "active",
  "createdAt": "2026-01-12T10:30:00Z",
  "monitoringStatus": "enabled",
  "protectionLevel": "premium"
}
```

### 4.2 Get Identity Profile

```http
GET /identities/{id}
Authorization: Bearer {access_token}
```

Response: Returns complete identity record (see Phase 1 data format)

### 4.3 Update Identity Profile

```http
PATCH /identities/{id}
Authorization: Bearer {access_token}

{
  "contactInformation": {
    "emails": [
      {
        "address": "newemail@example.com",
        "type": "personal",
        "primary": true
      }
    ]
  }
}
```

### 4.4 Delete Identity Profile

```http
DELETE /identities/{id}
Authorization: Bearer {access_token}

{
  "reason": "user_request",
  "deleteData": true,
  "confirmation": "DELETE-{id}"
}
```

Response:
```json
{
  "status": "deleted",
  "deletedAt": "2026-01-12T10:30:00Z",
  "dataRetentionUntil": "2026-02-12T10:30:00Z"
}
```

### 4.5 List Identities

```http
GET /identities?page=1&limit=50&status=active&sort=createdAt:desc
Authorization: Bearer {access_token}
```

Response:
```json
{
  "data": [
    {
      "id": "uuid-v4",
      "name": "John Doe",
      "status": "active",
      "monitoringLevel": "premium",
      "createdAt": "2026-01-12T10:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 150,
    "pages": 3
  }
}
```

### 4.6 Freeze Identity

```http
POST /identities/{id}/freeze
Authorization: Bearer {access_token}

{
  "reason": "suspected_fraud",
  "duration": "30d",
  "notifyUser": true
}
```

---

## 5. Monitoring & Alert APIs

### 5.1 Start Monitoring

```http
POST /monitoring/sessions
Authorization: Bearer {access_token}

{
  "identityId": "uuid-v4",
  "monitoringType": "comprehensive|credit|darkweb|identity",
  "duration": "continuous|30d|90d|365d",
  "alertThreshold": "low|medium|high|critical"
}
```

Response:
```json
{
  "sessionId": "uuid-v4",
  "status": "active",
  "startedAt": "2026-01-12T10:30:00Z",
  "monitoringScope": ["credit", "darkweb", "identity", "biometric"],
  "nextScanAt": "2026-01-12T11:00:00Z"
}
```

### 5.2 Stop Monitoring

```http
DELETE /monitoring/sessions/{sessionId}
Authorization: Bearer {access_token}
```

### 5.3 Get Monitoring Status

```http
GET /monitoring/sessions/{sessionId}
Authorization: Bearer {access_token}
```

Response:
```json
{
  "sessionId": "uuid-v4",
  "status": "active",
  "uptime": 86400,
  "lastScan": "2026-01-12T10:15:00Z",
  "nextScan": "2026-01-12T11:00:00Z",
  "statistics": {
    "alertsGenerated": 12,
    "threatsDetected": 2,
    "scansCompleted": 48,
    "lastThreatAt": "2026-01-11T14:22:00Z"
  }
}
```

### 5.4 List Alerts

```http
GET /monitoring/alerts?identityId={id}&severity=high&status=new&from=2026-01-01&to=2026-01-12
Authorization: Bearer {access_token}
```

Response:
```json
{
  "data": [
    {
      "id": "uuid-v4",
      "type": "identity_theft",
      "severity": "critical",
      "status": "new",
      "timestamp": "2026-01-12T09:45:00Z",
      "title": "New credit card account opened",
      "summary": "A new credit card account was detected on your credit report",
      "riskScore": 85,
      "actions": [
        {
          "type": "verify",
          "label": "Was this you?",
          "url": "/alerts/{id}/verify"
        }
      ]
    }
  ],
  "summary": {
    "total": 12,
    "critical": 2,
    "high": 5,
    "medium": 3,
    "low": 2
  }
}
```

### 5.5 Get Alert Details

```http
GET /monitoring/alerts/{id}
Authorization: Bearer {access_token}
```

Response: Returns complete alert object (see Phase 1 data format)

### 5.6 Update Alert Status

```http
PATCH /monitoring/alerts/{id}
Authorization: Bearer {access_token}

{
  "status": "acknowledged|investigating|resolved|false_positive",
  "notes": "Verified with bank - authorized transaction",
  "actions": [
    {
      "type": "notify",
      "status": "completed"
    }
  ]
}
```

### 5.7 Acknowledge Alert

```http
POST /monitoring/alerts/{id}/acknowledge
Authorization: Bearer {access_token}

{
  "acknowledgedBy": "user",
  "notes": "I am aware of this alert"
}
```

### 5.8 Resolve Alert

```http
POST /monitoring/alerts/{id}/resolve
Authorization: Bearer {access_token}

{
  "resolution": "legitimate|false_positive|mitigated",
  "notes": "Contacted bank and confirmed transaction",
  "actions": [
    {
      "type": "contact_institution",
      "status": "completed",
      "timestamp": "2026-01-12T10:30:00Z"
    }
  ]
}
```

---

## 6. Credit Monitoring APIs

### 6.1 Get Credit Report

```http
GET /credit/reports/{identityId}?bureau=equifax|experian|transunion|all
Authorization: Bearer {access_token}
```

Response: Returns credit report object (see Phase 1 data format)

### 6.2 Get Credit Score

```http
GET /credit/scores/{identityId}?model=fico8|fico9|vantagescore3
Authorization: Bearer {access_token}
```

Response:
```json
{
  "score": 750,
  "model": "FICO-8",
  "range": [300, 850],
  "category": "good",
  "reportDate": "2026-01-12",
  "factors": [
    {
      "code": "10",
      "description": "Level of delinquency on accounts",
      "impact": "negative"
    },
    {
      "code": "20",
      "description": "Amount owed on accounts",
      "impact": "positive"
    }
  ],
  "history": [
    {
      "date": "2025-12-12",
      "score": 745
    },
    {
      "date": "2025-11-12",
      "score": 742
    }
  ]
}
```

### 6.3 Enable Credit Monitoring

```http
POST /credit/monitoring/{identityId}/enable
Authorization: Bearer {access_token}

{
  "bureaus": ["equifax", "experian", "transunion"],
  "frequency": "daily|weekly|monthly",
  "alerts": {
    "newAccounts": true,
    "hardInquiries": true,
    "scoreChanges": true,
    "addressChanges": true,
    "derogatoryMarks": true
  }
}
```

### 6.4 Freeze Credit

```http
POST /credit/freeze/{identityId}
Authorization: Bearer {access_token}

{
  "bureaus": ["equifax", "experian", "transunion"],
  "duration": "indefinite|30d|90d|180d",
  "pin": "1234"
}
```

Response:
```json
{
  "status": "frozen",
  "bureaus": [
    {
      "bureau": "equifax",
      "status": "frozen",
      "frozenAt": "2026-01-12T10:30:00Z",
      "pin": "encrypted",
      "referenceNumber": "EQ-123456789"
    }
  ]
}
```

### 6.5 Unfreeze Credit

```http
POST /credit/unfreeze/{identityId}
Authorization: Bearer {access_token}

{
  "bureaus": ["equifax"],
  "pin": "1234",
  "duration": "temporary|permanent",
  "temporaryDuration": "24h"
}
```

### 6.6 Place Fraud Alert

```http
POST /credit/fraud-alert/{identityId}
Authorization: Bearer {access_token}

{
  "type": "initial|extended|active_military",
  "duration": "90d|7y|1y",
  "contactInfo": {
    "phone": "+12025551234",
    "instructions": "Please call to verify before extending credit"
  }
}
```

---

## 7. Dark Web Scanning APIs

### 7.1 Start Dark Web Scan

```http
POST /darkweb/scans
Authorization: Bearer {access_token}

{
  "identityId": "uuid-v4",
  "assets": [
    {
      "type": "email",
      "value": "john.doe@example.com"
    },
    {
      "type": "phone",
      "value": "+12025551234"
    },
    {
      "type": "ssn",
      "value": "***-**-6789"
    },
    {
      "type": "credit_card",
      "value": "****-****-****-1234"
    }
  ],
  "scope": "marketplaces|forums|paste_sites|breach_databases|all",
  "depth": "surface|deep|comprehensive"
}
```

Response:
```json
{
  "scanId": "uuid-v4",
  "status": "initiated",
  "estimatedDuration": 300,
  "assetsCount": 4,
  "startedAt": "2026-01-12T10:30:00Z"
}
```

### 7.2 Get Scan Status

```http
GET /darkweb/scans/{scanId}
Authorization: Bearer {access_token}
```

Response:
```json
{
  "scanId": "uuid-v4",
  "status": "in_progress|completed|failed",
  "progress": 67,
  "currentPhase": "scanning_marketplaces",
  "findings": {
    "total": 5,
    "critical": 2,
    "high": 1,
    "medium": 2,
    "low": 0
  },
  "startedAt": "2026-01-12T10:30:00Z",
  "estimatedCompletion": "2026-01-12T10:35:00Z"
}
```

### 7.3 Get Scan Results

```http
GET /darkweb/scans/{scanId}/results
Authorization: Bearer {access_token}
```

Response:
```json
{
  "scanId": "uuid-v4",
  "completedAt": "2026-01-12T10:35:00Z",
  "summary": {
    "totalFindings": 5,
    "newFindings": 3,
    "sourcesCovered": 1247,
    "assetsScanned": 4
  },
  "findings": [
    {
      "id": "uuid-v4",
      "severity": "critical",
      "exposedData": ["email", "password"],
      "source": {
        "type": "breach_database",
        "name": "Company XYZ Breach 2025"
      },
      "discoveredAt": "2026-01-12T10:32:00Z"
    }
  ]
}
```

### 7.4 Get Dark Web Finding Details

```http
GET /darkweb/findings/{id}
Authorization: Bearer {access_token}
```

Response: Returns complete dark web finding object (see Phase 1 data format)

### 7.5 Enable Continuous Monitoring

```http
POST /darkweb/monitoring/{identityId}/enable
Authorization: Bearer {access_token}

{
  "assets": [
    {
      "type": "email",
      "value": "john.doe@example.com"
    }
  ],
  "frequency": "continuous|daily|weekly",
  "alertOnNewFindings": true
}
```

### 7.6 Search Breaches

```http
GET /darkweb/breaches?search=company&from=2024-01-01&severity=high
Authorization: Bearer {access_token}
```

Response:
```json
{
  "data": [
    {
      "id": "uuid-v4",
      "name": "Company XYZ Breach",
      "company": "XYZ Corp",
      "breachDate": "2025-06-15",
      "disclosureDate": "2025-08-01",
      "recordsAffected": 50000000,
      "dataClasses": ["email", "password", "name", "address"],
      "severity": "critical"
    }
  ],
  "total": 1247
}
```

---

## 8. Biometric Authentication APIs

### 8.1 Enroll Biometric

```http
POST /biometric/enroll
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

{
  "userId": "uuid-v4",
  "modality": "fingerprint|face|iris|voice",
  "templateData": "base64-encoded-biometric",
  "deviceInfo": {
    "deviceId": "device-123",
    "deviceType": "scanner",
    "manufacturer": "BiometricCo"
  }
}
```

Response:
```json
{
  "templateId": "uuid-v4",
  "status": "enrolled",
  "quality": 92,
  "enrolledAt": "2026-01-12T10:30:00Z",
  "expiresAt": "2027-01-12T10:30:00Z"
}
```

### 8.2 Verify Biometric

```http
POST /biometric/verify
Authorization: Bearer {access_token}

{
  "userId": "uuid-v4",
  "templateId": "uuid-v4",
  "sampleData": "base64-encoded-biometric",
  "livenessCheck": true,
  "deviceInfo": {
    "deviceId": "device-123"
  }
}
```

Response:
```json
{
  "verificationId": "uuid-v4",
  "result": "matched",
  "confidence": 0.95,
  "score": 0.97,
  "threshold": 0.85,
  "decision": "accept",
  "livenessResult": {
    "passed": true,
    "confidence": 0.98
  },
  "timestamp": "2026-01-12T10:30:00Z"
}
```

### 8.3 List Biometric Templates

```http
GET /biometric/templates?userId={id}
Authorization: Bearer {access_token}
```

Response:
```json
{
  "data": [
    {
      "id": "uuid-v4",
      "modality": "fingerprint",
      "status": "active",
      "quality": 92,
      "enrolledAt": "2026-01-12T10:30:00Z",
      "lastVerified": "2026-01-12T14:22:00Z",
      "verificationCount": 47
    }
  ]
}
```

### 8.4 Delete Biometric Template

```http
DELETE /biometric/templates/{id}
Authorization: Bearer {access_token}

{
  "reason": "user_request",
  "confirmation": "DELETE-{id}"
}
```

---

## 9. Breach Intelligence APIs

### 9.1 Get Breach Details

```http
GET /breaches/{breachId}
Authorization: Bearer {access_token}
```

Response: Returns complete breach notification object (see Phase 1 data format)

### 9.2 Check If Affected

```http
POST /breaches/check-exposure
Authorization: Bearer {access_token}

{
  "email": "john.doe@example.com",
  "phone": "+12025551234",
  "username": "johndoe123"
}
```

Response:
```json
{
  "exposed": true,
  "breaches": [
    {
      "breachId": "uuid-v4",
      "breachName": "Company XYZ Breach",
      "breachDate": "2025-06-15",
      "exposedData": ["email", "password"],
      "severity": "high"
    }
  ],
  "totalBreaches": 3,
  "recommendations": [
    {
      "action": "change_password",
      "priority": "immediate",
      "description": "Change your password immediately on affected sites"
    }
  ]
}
```

### 9.3 Get Breach Timeline

```http
GET /breaches/{breachId}/timeline
Authorization: Bearer {access_token}
```

Response:
```json
{
  "breachId": "uuid-v4",
  "timeline": [
    {
      "date": "2025-03-15",
      "event": "Initial compromise",
      "description": "Attackers gained access to database"
    },
    {
      "date": "2025-06-15",
      "event": "Breach discovered",
      "description": "Security team detected unauthorized access"
    },
    {
      "date": "2025-08-01",
      "event": "Public disclosure",
      "description": "Company announced breach to public"
    }
  ]
}
```

### 9.4 Subscribe to Breach Notifications

```http
POST /breaches/notifications/subscribe
Authorization: Bearer {access_token}

{
  "identityId": "uuid-v4",
  "channels": ["email", "sms", "push"],
  "filters": {
    "severity": ["high", "critical"],
    "industries": ["finance", "healthcare", "technology"]
  }
}
```

---

## 10. Reporting & Analytics APIs

### 10.1 Generate Risk Report

```http
POST /analytics/reports/risk
Authorization: Bearer {access_token}

{
  "identityId": "uuid-v4",
  "period": {
    "start": "2025-01-01",
    "end": "2026-01-01"
  },
  "includeRecommendations": true
}
```

Response:
```json
{
  "reportId": "uuid-v4",
  "identityId": "uuid-v4",
  "generatedAt": "2026-01-12T10:30:00Z",
  "period": {
    "start": "2025-01-01",
    "end": "2026-01-01"
  },
  "overallRiskScore": 35,
  "riskLevel": "low",
  "categories": {
    "identityTheft": {
      "score": 25,
      "level": "low",
      "incidents": 0
    },
    "creditFraud": {
      "score": 40,
      "level": "medium",
      "incidents": 2
    },
    "dataBreaches": {
      "score": 50,
      "level": "medium",
      "exposures": 3
    }
  },
  "timeline": [
    {
      "date": "2025-06-15",
      "event": "Data breach exposure",
      "riskImpact": 15
    }
  ],
  "recommendations": [
    {
      "priority": "high",
      "action": "Enable MFA on all accounts",
      "impact": "Reduces risk by 30%"
    }
  ]
}
```

### 10.2 Get Activity Summary

```http
GET /analytics/activity/{identityId}?period=30d
Authorization: Bearer {access_token}
```

Response:
```json
{
  "identityId": "uuid-v4",
  "period": {
    "start": "2025-12-12",
    "end": "2026-01-12"
  },
  "summary": {
    "alertsGenerated": 12,
    "threatsDetected": 2,
    "scansCompleted": 48,
    "breachesFound": 1,
    "loginAttempts": 245,
    "suspiciousActivities": 3
  },
  "trends": {
    "alertTrend": "decreasing",
    "threatTrend": "stable",
    "activityTrend": "increasing"
  },
  "topThreats": [
    {
      "type": "credential_compromise",
      "count": 5,
      "severity": "high"
    }
  ]
}
```

### 10.3 Download Report

```http
GET /analytics/reports/{reportId}/download?format=pdf|csv|json
Authorization: Bearer {access_token}
```

Response: File download

---

## 11. Webhook & Event APIs

### 11.1 Configure Webhook

```http
POST /webhooks
Authorization: Bearer {access_token}

{
  "url": "https://your-app.com/webhook",
  "events": [
    "alert.created",
    "alert.resolved",
    "scan.completed",
    "breach.discovered",
    "credit.changed"
  ],
  "secret": "webhook-secret-key",
  "active": true
}
```

Response:
```json
{
  "webhookId": "uuid-v4",
  "url": "https://your-app.com/webhook",
  "events": ["alert.created", "alert.resolved"],
  "secret": "whsec_****",
  "status": "active",
  "createdAt": "2026-01-12T10:30:00Z"
}
```

### 11.2 Webhook Event Format

```json
{
  "id": "uuid-v4",
  "type": "alert.created",
  "timestamp": "2026-01-12T10:30:00Z",
  "data": {
    "alert": {
      "id": "uuid-v4",
      "severity": "high",
      "type": "identity_theft"
    }
  },
  "signature": "sha256-hmac-signature"
}
```

### 11.3 WebSocket Connection

```javascript
const ws = new WebSocket('wss://api.wia.org/identity-theft-prevention/v1/ws');

ws.send(JSON.stringify({
  type: 'authenticate',
  token: 'Bearer {access_token}'
}));

ws.send(JSON.stringify({
  type: 'subscribe',
  channels: ['alerts', 'scans', 'breaches']
}));

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  console.log('Real-time event:', message);
};
```

### 11.4 Event Types

| Event Type | Description | Payload |
|------------|-------------|---------|
| `alert.created` | New alert generated | Alert object |
| `alert.updated` | Alert status changed | Alert object |
| `alert.resolved` | Alert resolved | Alert ID |
| `scan.started` | Dark web scan initiated | Scan ID |
| `scan.completed` | Scan finished | Scan results |
| `breach.discovered` | New breach detected | Breach object |
| `credit.changed` | Credit report updated | Change summary |
| `biometric.verified` | Biometric verification | Verification result |
| `monitoring.status` | Monitoring status change | Status object |

---

## 12. Error Handling

### 12.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "The request is invalid",
    "details": "Missing required field: identityId",
    "requestId": "uuid-v4",
    "timestamp": "2026-01-12T10:30:00Z",
    "documentation": "https://docs.wia.org/errors/INVALID_REQUEST"
  }
}
```

### 12.2 HTTP Status Codes

| Status Code | Meaning | Usage |
|-------------|---------|-------|
| 200 OK | Success | Request successful |
| 201 Created | Created | Resource created |
| 204 No Content | Success | Successful deletion |
| 400 Bad Request | Invalid request | Validation error |
| 401 Unauthorized | Authentication required | Missing/invalid token |
| 403 Forbidden | Access denied | Insufficient permissions |
| 404 Not Found | Resource not found | Invalid ID |
| 409 Conflict | Resource conflict | Duplicate resource |
| 429 Too Many Requests | Rate limit exceeded | Too many requests |
| 500 Internal Server Error | Server error | Unexpected error |
| 503 Service Unavailable | Service down | Maintenance mode |

### 12.3 Error Codes

| Error Code | Description | Resolution |
|------------|-------------|------------|
| `INVALID_REQUEST` | Request validation failed | Check request parameters |
| `UNAUTHORIZED` | Authentication failed | Verify access token |
| `FORBIDDEN` | Insufficient permissions | Check API scopes |
| `NOT_FOUND` | Resource not found | Verify resource ID |
| `RATE_LIMIT_EXCEEDED` | Too many requests | Wait and retry |
| `INVALID_CREDENTIALS` | Invalid login credentials | Check credentials |
| `EXPIRED_TOKEN` | Access token expired | Refresh token |
| `INSUFFICIENT_BALANCE` | Insufficient API credits | Upgrade plan |
| `SERVICE_UNAVAILABLE` | Service temporarily unavailable | Retry later |
| `INTERNAL_ERROR` | Unexpected server error | Contact support |

### 12.4 Retry Logic

```javascript
async function retryRequest(fn, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    try {
      return await fn();
    } catch (error) {
      if (error.status === 429) {
        const retryAfter = error.headers['Retry-After'] || Math.pow(2, i);
        await sleep(retryAfter * 1000);
      } else if (error.status >= 500) {
        await sleep(Math.pow(2, i) * 1000);
      } else {
        throw error;
      }
    }
  }
  throw new Error('Max retries exceeded');
}
```

---

## Appendix A: OpenAPI Specification

Complete OpenAPI 3.0 specification available at:
```
https://api.wia.org/identity-theft-prevention/v1/openapi.json
```

## Appendix B: SDK Libraries

Official SDKs available for:
- JavaScript/TypeScript: `npm install @wia/identity-theft-prevention`
- Python: `pip install wia-identity-theft-prevention`
- Java: Maven Central
- Ruby: `gem install wia-identity-theft-prevention`
- Go: `go get github.com/wia-official/identity-theft-prevention-go`
- PHP: Composer

## Appendix C: Postman Collection

Import our Postman collection:
```
https://api.wia.org/identity-theft-prevention/v1/postman-collection.json
```

---

**Document Status:** APPROVED
**Next Review:** 2026-07-12
**Contact:** api-support@wia.org

---

© 2026 World Certification Industry Association (WIA)
弘益人間 (홍익인간) · Benefit All Humanity
