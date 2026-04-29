# WIA-IDENTITY_THEFT_PREVENTION: PHASE 3 - PROTOCOL SPECIFICATION

**Version:** 1.0.0
**Status:** APPROVED
**Last Updated:** 2026-01-12
**Authors:** WIA Technical Committee on Identity Security

---

## Table of Contents

1. [Introduction](#introduction)
2. [Protocol Architecture](#protocol-architecture)
3. [Communication Protocols](#communication-protocols)
4. [Identity Verification Protocol](#identity-verification-protocol)
5. [Threat Detection Protocol](#threat-detection-protocol)
6. [Alert Distribution Protocol](#alert-distribution-protocol)
7. [Credit Monitoring Protocol](#credit-monitoring-protocol)
8. [Dark Web Scanning Protocol](#dark-web-scanning-protocol)
9. [Biometric Authentication Protocol](#biometric-authentication-protocol)
10. [Breach Response Protocol](#breach-response-protocol)
11. [Inter-Service Communication](#inter-service-communication)
12. [Security Protocols](#security-protocols)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the communication protocols, workflows, and security mechanisms for the WIA-IDENTITY_THEFT_PREVENTION standard, ensuring secure, reliable, and efficient identity protection across distributed systems.

### 1.2 Protocol Stack

```
┌─────────────────────────────────────────┐
│     Application Layer (Business Logic)  │
├─────────────────────────────────────────┤
│     WIA-ITP Protocol Layer              │
│  (Identity Protection Specific Logic)   │
├─────────────────────────────────────────┤
│     Transport Layer                     │
│  (HTTPS, WSS, gRPC, MQTT)              │
├─────────────────────────────────────────┤
│     Security Layer                      │
│  (TLS 1.3, mTLS, End-to-End Encryption)│
├─────────────────────────────────────────┤
│     Network Layer (TCP/IP)              │
└─────────────────────────────────────────┘
```

### 1.3 Design Principles

1. **Zero Trust**: Never trust, always verify
2. **Defense in Depth**: Multiple layers of security
3. **Real-Time**: Sub-second alert propagation
4. **Fault Tolerance**: Graceful degradation and recovery
5. **Privacy by Design**: Minimal data exposure
6. **Auditability**: Complete audit trail
7. **Scalability**: Horizontal scaling support

---

## 2. Protocol Architecture

### 2.1 System Architecture

```
┌──────────────┐        ┌──────────────┐        ┌──────────────┐
│   Client     │◄──────►│  API Gateway │◄──────►│   Identity   │
│ Application  │        │   (TLS 1.3)  │        │   Service    │
└──────────────┘        └──────┬───────┘        └──────────────┘
                               │
                               ├──────────────►┌──────────────┐
                               │               │  Monitoring  │
                               │               │   Service    │
                               │               └──────────────┘
                               │
                               ├──────────────►┌──────────────┐
                               │               │    Credit    │
                               │               │   Service    │
                               │               └──────────────┘
                               │
                               ├──────────────►┌──────────────┐
                               │               │   Dark Web   │
                               │               │   Service    │
                               │               └──────────────┘
                               │
                               └──────────────►┌──────────────┐
                                               │  Biometric   │
                                               │   Service    │
                                               └──────────────┘
```

### 2.2 Communication Patterns

| Pattern | Use Case | Protocol | Example |
|---------|----------|----------|---------|
| Request-Response | API calls | HTTPS/REST | Get identity profile |
| Publish-Subscribe | Real-time alerts | WebSocket/MQTT | Alert notifications |
| Streaming | Continuous monitoring | gRPC Streaming | Dark web monitoring |
| RPC | Service-to-service | gRPC | Internal microservices |
| Event-Driven | Async processing | Message Queue | Breach notification |

### 2.3 Message Format

All protocol messages follow this structure:

```json
{
  "header": {
    "version": "1.0",
    "messageId": "uuid-v4",
    "timestamp": "ISO-8601",
    "correlationId": "uuid-v4",
    "source": "service-name",
    "destination": "service-name",
    "messageType": "request|response|event|notification",
    "priority": "low|normal|high|critical",
    "ttl": 300
  },
  "security": {
    "signature": "digital-signature",
    "encryption": "AES-256-GCM",
    "keyId": "key-identifier",
    "nonce": "random-nonce"
  },
  "payload": {
    "type": "string",
    "data": "encrypted-or-plain-object"
  },
  "metadata": {
    "retryCount": 0,
    "traceId": "distributed-trace-id"
  }
}
```

---

## 3. Communication Protocols

### 3.1 HTTPS/REST Protocol

**Request Flow:**

```
Client                    API Gateway              Service
  │                            │                      │
  ├──── HTTPS POST ───────────►│                      │
  │    (TLS 1.3 Handshake)     │                      │
  │                            ├──── Authenticate ────┤
  │                            │                      │
  │                            ├──── Authorize ───────┤
  │                            │                      │
  │                            ├──── Forward Request ─►
  │                            │                      │
  │                            │◄──── Response ───────┤
  │◄──── HTTPS Response ───────┤                      │
  │                            │                      │
```

**Headers:**
```http
POST /api/v1/identities HTTP/2
Host: api.wia.org
Authorization: Bearer eyJhbGc...
Content-Type: application/json
Accept: application/json
X-API-Version: 1.0
X-Request-ID: 550e8400-e29b-41d4-a716-446655440000
X-Client-ID: client-123
X-Idempotency-Key: idem-key-123
User-Agent: WIA-SDK/1.0.0
```

### 3.2 WebSocket Protocol

**Connection Establishment:**

```
Client                    WebSocket Server
  │                            │
  ├──── WS Upgrade Request ───►│
  │    Connection: Upgrade     │
  │    Upgrade: websocket      │
  │                            │
  │◄──── 101 Switching ────────┤
  │      Protocols             │
  │                            │
  ├──── Authentication ────────►
  │    {type: "auth",          │
  │     token: "..."}          │
  │                            │
  │◄──── Auth Success ─────────┤
  │    {type: "authenticated"} │
  │                            │
  ├──── Subscribe ─────────────►
  │    {type: "subscribe",     │
  │     channels: ["alerts"]}  │
  │                            │
  │◄──── Real-time Events ─────┤
  │                            │
```

**Message Types:**

```json
// Authentication
{
  "type": "authenticate",
  "token": "Bearer token",
  "clientId": "client-123"
}

// Subscribe to channels
{
  "type": "subscribe",
  "channels": ["alerts", "scans", "breaches"],
  "filters": {
    "severity": ["high", "critical"]
  }
}

// Real-time alert
{
  "type": "alert",
  "channel": "alerts",
  "data": {
    "id": "uuid-v4",
    "severity": "critical",
    "message": "New credit account detected"
  }
}

// Heartbeat/Ping
{
  "type": "ping",
  "timestamp": "2026-01-12T10:30:00Z"
}

// Pong response
{
  "type": "pong",
  "timestamp": "2026-01-12T10:30:00Z"
}
```

### 3.3 gRPC Protocol

**Service Definition:**

```protobuf
syntax = "proto3";

package wia.identity_theft_prevention.v1;

service IdentityProtectionService {
  // Unary RPC
  rpc GetIdentity(GetIdentityRequest) returns (Identity);

  // Server streaming
  rpc MonitorThreats(MonitorRequest) returns (stream ThreatEvent);

  // Client streaming
  rpc ReportActivities(stream Activity) returns (ActivitySummary);

  // Bidirectional streaming
  rpc LiveMonitoring(stream MonitorCommand) returns (stream MonitorEvent);
}

message GetIdentityRequest {
  string identity_id = 1;
  repeated string fields = 2;
}

message Identity {
  string id = 1;
  string status = 2;
  PersonalInfo personal_info = 3;
  repeated Alert alerts = 4;
}

message ThreatEvent {
  string event_id = 1;
  string threat_type = 2;
  int32 severity = 3;
  google.protobuf.Timestamp timestamp = 4;
  map<string, string> metadata = 5;
}
```

**Usage:**

```javascript
const client = new IdentityProtectionServiceClient(
  'api.wia.org:443',
  credentials.createSsl()
);

// Unary call
const identity = await client.GetIdentity({
  identity_id: 'uuid-v4',
  fields: ['personal_info', 'alerts']
});

// Server streaming
const stream = client.MonitorThreats({
  identity_id: 'uuid-v4',
  threat_types: ['identity_theft', 'fraud']
});

stream.on('data', (event) => {
  console.log('Threat detected:', event);
});
```

### 3.4 MQTT Protocol

**Connection:**

```
Client                    MQTT Broker
  │                            │
  ├──── CONNECT ──────────────►│
  │    clientId: "client-123"  │
  │    username: "api-key"     │
  │    password: "secret"      │
  │                            │
  │◄──── CONNACK ──────────────┤
  │    returnCode: 0           │
  │                            │
  ├──── SUBSCRIBE ────────────►│
  │    topic: "alerts/+/high"  │
  │                            │
  │◄──── SUBACK ───────────────┤
  │                            │
  │◄──── PUBLISH ──────────────┤
  │    topic: "alerts/user123/high"
  │    payload: {...}          │
  │                            │
```

**Topic Structure:**

```
wia/itp/v1/{service}/{entity}/{action}
wia/itp/v1/alerts/{userId}/{severity}
wia/itp/v1/scans/{scanId}/status
wia/itp/v1/breaches/{breachId}/notification
wia/itp/v1/monitoring/{identityId}/status
```

---

## 4. Identity Verification Protocol

### 4.1 Identity Verification Flow

```
Client              API Gateway         Identity Service      Verification Provider
  │                      │                      │                      │
  ├──1. Start Verify────►│                      │                      │
  │                      ├──2. Validate Request─►                      │
  │                      │                      ├──3. Request Docs────►│
  │◄─4. Upload Docs──────┤                      │                      │
  │                      ├──5. Submit Docs──────┼──6. Verify Docs─────►│
  │                      │                      │◄─7. Results──────────┤
  │                      │◄─8. Update Status────┤                      │
  │◄─9. Verification─────┤                      │                      │
  │     Complete         │                      │                      │
```

### 4.2 Verification Protocol Messages

**1. Initiate Verification:**

```json
{
  "action": "identity.verify.initiate",
  "identityId": "uuid-v4",
  "verificationType": "knowledge_based|document|biometric|multi_factor",
  "requiredDocuments": [
    "government_id",
    "proof_of_address",
    "selfie"
  ],
  "verificationLevel": "basic|standard|enhanced",
  "callbackUrl": "https://your-app.com/webhook"
}
```

**2. Document Submission:**

```json
{
  "action": "identity.verify.submit",
  "verificationId": "uuid-v4",
  "documents": [
    {
      "type": "government_id",
      "format": "image/jpeg",
      "data": "base64-encoded",
      "metadata": {
        "documentNumber": "encrypted",
        "issueDate": "2020-01-15",
        "expiryDate": "2030-01-15"
      }
    }
  ]
}
```

**3. Verification Result:**

```json
{
  "action": "identity.verify.result",
  "verificationId": "uuid-v4",
  "status": "verified|pending|rejected",
  "confidence": 0.95,
  "checks": [
    {
      "type": "document_authenticity",
      "result": "pass",
      "confidence": 0.98
    },
    {
      "type": "face_match",
      "result": "pass",
      "confidence": 0.92
    },
    {
      "type": "liveness",
      "result": "pass",
      "confidence": 0.96
    }
  ],
  "verifiedAt": "2026-01-12T10:30:00Z",
  "validUntil": "2027-01-12T10:30:00Z"
}
```

### 4.3 Challenge-Response Protocol

**For Knowledge-Based Authentication:**

```json
// Challenge
{
  "action": "identity.challenge",
  "challengeId": "uuid-v4",
  "questions": [
    {
      "id": "q1",
      "question": "What is your mother's maiden name?",
      "options": ["Smith", "Johnson", "Williams", "Brown"]
    }
  ],
  "expiresIn": 300
}

// Response
{
  "action": "identity.challenge.response",
  "challengeId": "uuid-v4",
  "answers": [
    {
      "id": "q1",
      "answer": "encrypted-answer"
    }
  ]
}

// Verification
{
  "action": "identity.challenge.result",
  "challengeId": "uuid-v4",
  "passed": true,
  "score": 100
}
```

---

## 5. Threat Detection Protocol

### 5.1 Threat Detection Pipeline

```
Data Sources ──► Collectors ──► Normalizers ──► Analyzers ──► Correlators ──► Alert Generator
     │               │              │               │              │                │
     │               │              │               │              │                │
   Logs          Raw Events    Structured      ML Models      Pattern         Alerts
   APIs          Network       Events          Rules          Matching        Webhooks
   Sensors       Traffic                       Anomaly                        SIEM
```

### 5.2 Threat Intelligence Sharing Protocol

**STIX 2.1 Format:**

```json
{
  "type": "bundle",
  "id": "bundle--uuid",
  "objects": [
    {
      "type": "indicator",
      "id": "indicator--uuid",
      "created": "2026-01-12T10:30:00Z",
      "modified": "2026-01-12T10:30:00Z",
      "name": "Malicious IP",
      "pattern": "[ipv4-addr:value = '192.0.2.1']",
      "pattern_type": "stix",
      "valid_from": "2026-01-12T10:30:00Z",
      "labels": ["malicious-activity", "identity-theft"]
    },
    {
      "type": "threat-actor",
      "id": "threat-actor--uuid",
      "created": "2026-01-12T10:30:00Z",
      "name": "APT-Identity-Theft-Group",
      "labels": ["hacker"],
      "sophistication": "advanced",
      "primary_motivation": "financial-gain"
    }
  ]
}
```

### 5.3 Anomaly Detection Protocol

**Real-Time Scoring:**

```json
{
  "action": "threat.detect.anomaly",
  "sessionId": "uuid-v4",
  "userId": "uuid-v4",
  "event": {
    "type": "login_attempt",
    "timestamp": "2026-01-12T10:30:00Z",
    "ipAddress": "192.0.2.1",
    "location": {
      "country": "RU",
      "city": "Moscow"
    },
    "device": {
      "fingerprint": "device-fingerprint",
      "type": "desktop",
      "os": "Windows 10"
    }
  },
  "baseline": {
    "normalLocations": ["US-CA", "US-NY"],
    "normalDevices": ["device-1", "device-2"],
    "normalHours": [8, 17]
  },
  "anomalyScore": {
    "overall": 85,
    "factors": {
      "locationDeviation": 95,
      "deviceRecognition": 0,
      "timeDeviation": 60,
      "velocityAnomaly": 90
    }
  },
  "recommendation": "challenge|block|allow"
}
```

### 5.4 Machine Learning Model Protocol

**Model Inference Request:**

```json
{
  "action": "ml.predict",
  "modelId": "identity-theft-detector-v2",
  "modelVersion": "2.1.0",
  "features": {
    "account_age_days": 365,
    "transaction_count_30d": 45,
    "failed_login_attempts": 3,
    "device_trust_score": 0.8,
    "location_risk_score": 0.3,
    "behavioral_score": 0.75
  }
}
```

**Model Response:**

```json
{
  "action": "ml.prediction",
  "modelId": "identity-theft-detector-v2",
  "prediction": {
    "class": "fraud",
    "probability": 0.87,
    "confidence": 0.92
  },
  "explanation": {
    "topFeatures": [
      {
        "feature": "failed_login_attempts",
        "importance": 0.35,
        "value": 3
      },
      {
        "feature": "location_risk_score",
        "importance": 0.28,
        "value": 0.3
      }
    ]
  },
  "threshold": 0.75,
  "decision": "flag_for_review"
}
```

---

## 6. Alert Distribution Protocol

### 6.1 Alert Lifecycle

```
Detection ──► Triage ──► Enrichment ──► Routing ──► Delivery ──► Acknowledgment ──► Resolution
    │            │           │            │           │              │                 │
  Sensor      Priority   Context Add   Channel     Send to       User Ack         Close
  Detect      Assign     Intel Feed    Selection   Multi-Ch                      Update
```

### 6.2 Alert Protocol Messages

**Alert Creation:**

```json
{
  "action": "alert.create",
  "alert": {
    "id": "uuid-v4",
    "type": "identity_theft",
    "severity": "critical",
    "priority": 1,
    "title": "Suspicious credit application detected",
    "description": "A credit application was submitted from an unfamiliar location",
    "detectedAt": "2026-01-12T10:30:00Z",
    "source": "credit_monitoring_service",
    "affectedEntities": [
      {
        "type": "identity",
        "id": "uuid-v4"
      }
    ],
    "indicators": [
      {
        "type": "location",
        "value": "RU-Moscow",
        "confidence": 0.9
      }
    ],
    "evidence": [
      {
        "type": "credit_report",
        "id": "uuid-v4"
      }
    ]
  }
}
```

**Alert Routing:**

```json
{
  "action": "alert.route",
  "alertId": "uuid-v4",
  "routing": {
    "channels": [
      {
        "type": "email",
        "recipient": "user@example.com",
        "priority": "immediate",
        "template": "identity-theft-alert"
      },
      {
        "type": "sms",
        "recipient": "+12025551234",
        "priority": "immediate"
      },
      {
        "type": "push",
        "deviceTokens": ["token1", "token2"],
        "priority": "high"
      },
      {
        "type": "webhook",
        "url": "https://app.com/webhook",
        "retry": true
      }
    ]
  }
}
```

**Alert Acknowledgment:**

```json
{
  "action": "alert.acknowledge",
  "alertId": "uuid-v4",
  "acknowledgedBy": "user-id",
  "acknowledgedAt": "2026-01-12T10:35:00Z",
  "notes": "I am aware of this alert and investigating"
}
```

### 6.3 Multi-Channel Delivery Protocol

**Email Delivery:**

```json
{
  "channel": "email",
  "to": "user@example.com",
  "from": "alerts@wia.org",
  "subject": "[CRITICAL] Identity Theft Alert",
  "templateId": "identity-theft-alert",
  "variables": {
    "userName": "John Doe",
    "alertType": "Credit Application",
    "timestamp": "2026-01-12T10:30:00Z",
    "actionUrl": "https://app.wia.org/alerts/uuid"
  },
  "priority": "high",
  "tracking": {
    "openTracking": true,
    "clickTracking": true
  }
}
```

**SMS Delivery:**

```json
{
  "channel": "sms",
  "to": "+12025551234",
  "message": "WIA Alert: Suspicious credit application detected. Visit wia.org/a/abc123 to review.",
  "priority": "high",
  "shortUrl": "wia.org/a/abc123"
}
```

**Push Notification:**

```json
{
  "channel": "push",
  "deviceTokens": ["token1"],
  "notification": {
    "title": "Identity Theft Alert",
    "body": "Suspicious credit application detected",
    "icon": "warning",
    "sound": "urgent.mp3",
    "badge": 1,
    "data": {
      "alertId": "uuid-v4",
      "type": "identity_theft",
      "action": "open_alert"
    }
  },
  "priority": "high",
  "ttl": 3600
}
```

---

## 7. Credit Monitoring Protocol

### 7.1 Credit Bureau Integration Protocol

**Credit Pull Request:**

```json
{
  "action": "credit.pull",
  "requestId": "uuid-v4",
  "bureau": "equifax|experian|transunion",
  "requestType": "hard_pull|soft_pull",
  "subject": {
    "firstName": "John",
    "lastName": "Doe",
    "ssn": "encrypted",
    "dateOfBirth": "1990-01-15",
    "address": {
      "street": "123 Main St",
      "city": "New York",
      "state": "NY",
      "zip": "10001"
    }
  },
  "permissiblePurpose": "account_review",
  "consent": {
    "obtained": true,
    "timestamp": "2026-01-12T10:30:00Z",
    "ipAddress": "192.0.2.1"
  }
}
```

**Credit Report Response:**

```json
{
  "action": "credit.report",
  "requestId": "uuid-v4",
  "bureau": "equifax",
  "reportDate": "2026-01-12",
  "fileNumber": "encrypted",
  "report": {
    "creditScore": 750,
    "scoreModel": "FICO-8",
    "accounts": [...],
    "inquiries": [...],
    "publicRecords": [...]
  },
  "signature": "bureau-signature",
  "expiresAt": "2026-01-13T10:30:00Z"
}
```

### 7.2 Credit Monitoring Event Protocol

**Change Detection:**

```json
{
  "action": "credit.change.detected",
  "identityId": "uuid-v4",
  "changeType": "new_account|inquiry|score_change|address_change",
  "bureau": "experian",
  "detectedAt": "2026-01-12T10:30:00Z",
  "change": {
    "type": "new_account",
    "before": null,
    "after": {
      "creditor": "ABC Bank",
      "accountType": "credit_card",
      "creditLimit": 5000,
      "openedDate": "2026-01-10"
    }
  },
  "riskAssessment": {
    "fraudProbability": 0.85,
    "reasons": [
      "Account opened from unfamiliar location",
      "No recent inquiry on file"
    ]
  }
}
```

### 7.3 Credit Freeze Protocol

**Freeze Request:**

```json
{
  "action": "credit.freeze",
  "identityId": "uuid-v4",
  "bureaus": ["equifax", "experian", "transunion"],
  "freezeType": "security_freeze",
  "duration": "indefinite",
  "pin": "encrypted-pin",
  "authentication": {
    "method": "knowledge_based",
    "verified": true
  }
}
```

**Freeze Response:**

```json
{
  "action": "credit.freeze.result",
  "requestId": "uuid-v4",
  "results": [
    {
      "bureau": "equifax",
      "status": "frozen",
      "pin": "encrypted-pin",
      "referenceNumber": "EQ-123456789",
      "frozenAt": "2026-01-12T10:30:00Z",
      "liftInstructions": "Call 1-800-xxx-xxxx or visit equifax.com"
    }
  ]
}
```

---

## 8. Dark Web Scanning Protocol

### 8.1 Dark Web Crawler Protocol

**Scan Initiation:**

```json
{
  "action": "darkweb.scan.start",
  "scanId": "uuid-v4",
  "targets": [
    {
      "type": "email",
      "value": "john.doe@example.com",
      "hash": "sha256-hash"
    },
    {
      "type": "username",
      "value": "johndoe123",
      "hash": "sha256-hash"
    }
  ],
  "scope": {
    "sources": ["marketplaces", "forums", "paste_sites", "breach_databases"],
    "depth": "comprehensive",
    "timeRange": "all|last_year|last_month"
  },
  "configuration": {
    "concurrent_crawlers": 10,
    "rate_limit": 100,
    "timeout": 300,
    "retry_attempts": 3
  }
}
```

**Crawler Status:**

```json
{
  "action": "darkweb.scan.status",
  "scanId": "uuid-v4",
  "status": "running",
  "progress": {
    "percent": 45,
    "sourcesScanned": 156,
    "totalSources": 347,
    "findingsCount": 3
  },
  "currentPhase": "scanning_marketplaces",
  "eta": 420
}
```

**Finding Report:**

```json
{
  "action": "darkweb.finding.report",
  "findingId": "uuid-v4",
  "scanId": "uuid-v4",
  "discoveredAt": "2026-01-12T10:30:00Z",
  "source": {
    "type": "marketplace",
    "url": "tor://darkmarket.onion/listing/123",
    "name": "DarkMarket",
    "reliability": "high"
  },
  "exposedData": {
    "email": "partial-match",
    "password": "hash-found",
    "creditCard": "last-4-digits",
    "ssn": "not-found"
  },
  "context": {
    "listing": "Database dump - Company XYZ",
    "price": "$50 BTC",
    "seller": "darkuser123",
    "postedDate": "2026-01-10"
  },
  "verification": {
    "status": "pending",
    "confidence": 0.85
  }
}
```

### 8.2 Tor Network Protocol

**Tor Connection:**

```python
# Establish Tor connection
{
  "action": "tor.connect",
  "socksProxy": "127.0.0.1:9050",
  "controlPort": 9051,
  "circuitId": "circuit-123",
  "entryNode": "node1",
  "middleNode": "node2",
  "exitNode": "node3"
}

# Tor request
{
  "action": "tor.request",
  "method": "GET",
  "url": "http://darkmarket.onion/api/search",
  "headers": {
    "User-Agent": "Tor Browser 11.0"
  },
  "circuit": "circuit-123",
  "timeout": 30
}
```

### 8.3 Breach Database Query Protocol

**Database Query:**

```json
{
  "action": "breach.database.query",
  "queryId": "uuid-v4",
  "databases": ["haveibeenpwned", "dehashed", "leakcheck"],
  "searchTerms": [
    {
      "type": "email",
      "value": "sha256-hash"
    }
  ],
  "includeDetails": true
}
```

**Query Response:**

```json
{
  "action": "breach.database.result",
  "queryId": "uuid-v4",
  "matches": [
    {
      "database": "haveibeenpwned",
      "breachName": "Company XYZ",
      "breachDate": "2025-06-15",
      "dataClasses": ["email", "password", "name"],
      "recordCount": 50000000,
      "verified": true
    }
  ],
  "totalMatches": 3
}
```

---

## 9. Biometric Authentication Protocol

### 9.1 Biometric Enrollment Protocol

**Enrollment Flow:**

```
Client              Biometric Service       Template DB       Liveness Service
  │                        │                      │                  │
  ├──1. Start Enroll──────►│                      │                  │
  │◄──2. Liveness Check────┤                      │                  │
  ├──3. Capture Sample─────►                      │                  │
  │                        ├──4. Verify Liveness─►│                  │
  │                        │◄─5. Liveness OK──────┤                  │
  │                        ├──6. Extract Template─┤                  │
  │                        ├──7. Encrypt Template─┤                  │
  │                        ├──8. Store Template──►│                  │
  │◄──9. Enrollment Success┤                      │                  │
```

**Enrollment Request:**

```json
{
  "action": "biometric.enroll",
  "userId": "uuid-v4",
  "modality": "fingerprint",
  "sample": {
    "format": "ISO-19794-2",
    "encoding": "base64",
    "data": "base64-encoded-image",
    "quality": 85,
    "resolution": 500
  },
  "device": {
    "id": "scanner-123",
    "type": "optical",
    "manufacturer": "BiometricCo",
    "certified": true
  },
  "livenessCheck": true
}
```

**Enrollment Response:**

```json
{
  "action": "biometric.enrolled",
  "templateId": "uuid-v4",
  "status": "success",
  "quality": {
    "score": 92,
    "nfiqScore": 2
  },
  "liveness": {
    "passed": true,
    "confidence": 0.98,
    "method": "pulsation_detection"
  },
  "template": {
    "hash": "sha256-hash",
    "encrypted": true,
    "keyId": "key-123"
  },
  "expiresAt": "2027-01-12T10:30:00Z"
}
```

### 9.2 Biometric Verification Protocol

**Verification Request:**

```json
{
  "action": "biometric.verify",
  "userId": "uuid-v4",
  "templateId": "uuid-v4",
  "sample": {
    "format": "ISO-19794-2",
    "encoding": "base64",
    "data": "base64-encoded-image"
  },
  "livenessCheck": true,
  "context": {
    "sessionId": "uuid-v4",
    "ipAddress": "192.0.2.1",
    "timestamp": "2026-01-12T10:30:00Z"
  }
}
```

**Verification Response:**

```json
{
  "action": "biometric.verified",
  "verificationId": "uuid-v4",
  "result": "matched",
  "confidence": 0.95,
  "matchScore": 0.97,
  "threshold": 0.85,
  "decision": "accept",
  "liveness": {
    "passed": true,
    "confidence": 0.96
  },
  "qualityCheck": {
    "passed": true,
    "score": 88
  },
  "timestamp": "2026-01-12T10:30:00Z"
}
```

### 9.3 Anti-Spoofing Protocol

**Liveness Detection:**

```json
{
  "action": "biometric.liveness.detect",
  "sample": "base64-encoded",
  "method": "pulsation|3d_depth|challenge_response",
  "parameters": {
    "frames": 30,
    "duration": 3000,
    "challenges": ["blink", "turn_head", "smile"]
  }
}
```

**Liveness Result:**

```json
{
  "action": "biometric.liveness.result",
  "passed": true,
  "confidence": 0.98,
  "indicators": {
    "pulsation": 0.95,
    "depth": 0.99,
    "texture": 0.97,
    "motion": 0.96
  },
  "spoofingAttempt": false
}
```

---

## 10. Breach Response Protocol

### 10.1 Breach Notification Protocol

**Breach Discovery:**

```json
{
  "action": "breach.discovered",
  "breachId": "uuid-v4",
  "organization": "Company XYZ",
  "discoveredAt": "2026-01-12T10:30:00Z",
  "severity": "critical",
  "preliminary": {
    "recordsAffected": "unknown",
    "dataTypes": ["email", "password", "personal_info"],
    "attackVector": "sql_injection"
  },
  "source": "security_researcher"
}
```

**Impact Assessment:**

```json
{
  "action": "breach.assess",
  "breachId": "uuid-v4",
  "affectedUsers": [
    {
      "userId": "uuid-v4",
      "exposedData": ["email", "password_hash", "name"],
      "riskLevel": "high",
      "lastLoginDate": "2026-01-10T14:30:00Z"
    }
  ],
  "totalAffected": 150,
  "riskDistribution": {
    "critical": 20,
    "high": 50,
    "medium": 60,
    "low": 20
  }
}
```

**User Notification:**

```json
{
  "action": "breach.notify",
  "breachId": "uuid-v4",
  "userId": "uuid-v4",
  "notification": {
    "channels": ["email", "sms", "push"],
    "priority": "immediate",
    "template": "breach-notification",
    "variables": {
      "company": "Company XYZ",
      "breachDate": "2025-06-15",
      "dataTypes": ["email", "password"],
      "recommendations": [
        "Change your password immediately",
        "Enable two-factor authentication",
        "Monitor your accounts for suspicious activity"
      ]
    }
  }
}
```

### 10.2 Incident Response Protocol

**Incident Declaration:**

```json
{
  "action": "incident.declare",
  "incidentId": "uuid-v4",
  "type": "data_breach",
  "severity": "critical",
  "declaredAt": "2026-01-12T10:30:00Z",
  "declaredBy": "security-team",
  "scope": {
    "affectedSystems": ["database-prod-01", "api-gateway"],
    "affectedUsers": 150000,
    "dataExposed": ["email", "password", "personal_info"]
  }
}
```

**Response Actions:**

```json
{
  "action": "incident.response",
  "incidentId": "uuid-v4",
  "actions": [
    {
      "id": "action-1",
      "type": "containment",
      "action": "isolate_affected_systems",
      "status": "completed",
      "timestamp": "2026-01-12T10:35:00Z"
    },
    {
      "id": "action-2",
      "type": "eradication",
      "action": "patch_vulnerability",
      "status": "in_progress",
      "timestamp": "2026-01-12T10:40:00Z"
    },
    {
      "id": "action-3",
      "type": "recovery",
      "action": "restore_services",
      "status": "pending",
      "timestamp": null
    },
    {
      "id": "action-4",
      "type": "notification",
      "action": "notify_affected_users",
      "status": "pending",
      "timestamp": null
    }
  ]
}
```

---

## 11. Inter-Service Communication

### 11.1 Service Mesh Protocol

**Service Discovery:**

```json
{
  "action": "service.register",
  "serviceId": "identity-service-01",
  "serviceName": "identity-service",
  "version": "1.0.0",
  "endpoints": [
    {
      "protocol": "grpc",
      "host": "identity-service.internal",
      "port": 50051
    }
  ],
  "healthCheck": {
    "protocol": "http",
    "path": "/health",
    "interval": 10,
    "timeout": 5
  }
}
```

**Service-to-Service Call:**

```json
{
  "action": "service.call",
  "sourceService": "monitoring-service",
  "targetService": "identity-service",
  "method": "GetIdentity",
  "requestId": "uuid-v4",
  "correlationId": "uuid-v4",
  "timeout": 5000,
  "retryPolicy": {
    "maxRetries": 3,
    "backoff": "exponential"
  },
  "circuitBreaker": {
    "enabled": true,
    "threshold": 5,
    "timeout": 30000
  }
}
```

### 11.2 Message Queue Protocol

**Message Publication:**

```json
{
  "action": "queue.publish",
  "exchange": "wia.identity-theft-prevention",
  "routingKey": "alerts.critical",
  "message": {
    "id": "uuid-v4",
    "type": "alert",
    "payload": {...}
  },
  "properties": {
    "persistent": true,
    "priority": 9,
    "expiration": "3600000",
    "contentType": "application/json"
  }
}
```

**Message Consumption:**

```json
{
  "action": "queue.consume",
  "queue": "alert-processing-queue",
  "consumerTag": "consumer-123",
  "autoAck": false,
  "prefetchCount": 10
}

// Acknowledge
{
  "action": "queue.ack",
  "deliveryTag": 12345,
  "multiple": false
}

// Reject
{
  "action": "queue.nack",
  "deliveryTag": 12345,
  "requeue": true
}
```

---

## 12. Security Protocols

### 12.1 End-to-End Encryption Protocol

**Key Exchange (ECDH):**

```json
{
  "action": "crypto.key_exchange",
  "algorithm": "ECDH-P256",
  "publicKey": "base64-encoded-public-key",
  "ephemeral": true
}
```

**Message Encryption:**

```json
{
  "action": "crypto.encrypt",
  "algorithm": "AES-256-GCM",
  "keyId": "key-uuid",
  "nonce": "base64-nonce",
  "ciphertext": "base64-ciphertext",
  "authTag": "base64-auth-tag"
}
```

### 12.2 Digital Signature Protocol

**Signature Creation:**

```json
{
  "action": "crypto.sign",
  "algorithm": "ECDSA-P256-SHA256",
  "keyId": "signing-key-uuid",
  "data": "base64-data-to-sign",
  "signature": "base64-signature"
}
```

**Signature Verification:**

```json
{
  "action": "crypto.verify",
  "algorithm": "ECDSA-P256-SHA256",
  "publicKey": "base64-public-key",
  "data": "base64-data",
  "signature": "base64-signature",
  "valid": true
}
```

### 12.3 Zero-Knowledge Proof Protocol

**ZKP Challenge:**

```json
{
  "action": "zkp.challenge",
  "protocol": "schnorr",
  "challengeId": "uuid-v4",
  "publicParameters": {
    "generator": "g",
    "prime": "p",
    "commitment": "C"
  }
}
```

**ZKP Response:**

```json
{
  "action": "zkp.response",
  "challengeId": "uuid-v4",
  "proof": {
    "response": "r",
    "challenge": "c"
  }
}
```

---

## Appendix A: Protocol Buffer Definitions

Complete `.proto` files available at:
```
https://github.com/WIA-Official/wia-standards/tree/main/standards/WIA-IDENTITY_THEFT_PREVENTION/proto
```

## Appendix B: Sequence Diagrams

Complete sequence diagrams for all protocols available at:
```
https://docs.wia.org/identity-theft-prevention/protocols/diagrams
```

## Appendix C: Protocol Compliance Testing

Test suite available at:
```
https://github.com/WIA-Official/wia-itp-protocol-tests
```

---

**Document Status:** APPROVED
**Next Review:** 2026-07-12
**Contact:** protocols@wia.org

---

© 2026 World Certification Industry Association (WIA)
弘益人間 (홍익인간) · Benefit All Humanity
