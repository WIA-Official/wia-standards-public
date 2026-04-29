# WIA-IOT_SECURITY: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the RESTful API interfaces for IoT Security management. All implementations MUST provide these endpoints to ensure interoperability.

## 2. Base URL and Versioning

### 2.1 Base URL Structure
```
https://api.example.com/wia/iot-security/v1
```

### 2.2 Versioning
- API version in URL path: `/v1`, `/v2`
- Header versioning: `API-Version: 1.0`
- Breaking changes require major version increment

## 3. Authentication Endpoints

### 3.1 Device Registration

**POST** `/devices/register`

Register a new IoT device in the security system.

**Request:**
```json
{
  "manufacturer": "SecureIoT Corp",
  "model": "SI-2000",
  "serialNumber": "SI2000-2025-001234",
  "macAddress": "00:1A:2B:3C:4D:5E",
  "publicKey": "-----BEGIN PUBLIC KEY-----\n...\n-----END PUBLIC KEY-----",
  "attestationData": "base64-encoded-data"
}
```

**Response:** `201 Created`
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "status": "registered",
  "certificateId": "cert-550e8400",
  "certificate": "-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----",
  "validUntil": "2026-01-12T00:00:00Z",
  "registeredAt": "2025-01-12T10:30:00Z"
}
```

### 3.2 Device Authentication

**POST** `/devices/authenticate`

Authenticate a device and obtain access tokens.

**Request:**
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "challenge": "base64-challenge-data",
  "signature": "base64-signature",
  "timestamp": "2025-01-12T10:30:00Z"
}
```

**Response:** `200 OK`
```json
{
  "accessToken": "eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refreshToken": "eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...",
  "tokenType": "Bearer",
  "expiresIn": 3600,
  "scope": ["read", "write"],
  "deviceStatus": "active"
}
```

### 3.3 Token Refresh

**POST** `/auth/refresh`

Refresh an expired access token.

**Request:**
```json
{
  "refreshToken": "eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**Response:** `200 OK`
```json
{
  "accessToken": "eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...",
  "expiresIn": 3600
}
```

## 4. Device Management Endpoints

### 4.1 Get Device Details

**GET** `/devices/{deviceId}`

Retrieve device information and security status.

**Response:** `200 OK`
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "identity": {
    "manufacturer": "SecureIoT Corp",
    "model": "SI-2000",
    "serialNumber": "SI2000-2025-001234",
    "firmwareVersion": "2.5.1",
    "hardwareVersion": "Rev B"
  },
  "security": {
    "status": "secure",
    "lastAudit": "2025-01-12T09:00:00Z",
    "certificateExpiry": "2026-01-12T00:00:00Z",
    "complianceScore": 95,
    "vulnerabilities": []
  },
  "lastSeen": "2025-01-12T10:30:00Z"
}
```

### 4.2 Update Device

**PATCH** `/devices/{deviceId}`

Update device configuration and metadata.

**Request:**
```json
{
  "firmwareVersion": "2.5.2",
  "configuration": {
    "updateChannel": "stable",
    "telemetryEnabled": true,
    "securityLevel": 4
  }
}
```

**Response:** `200 OK`
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "updated": true,
  "updatedFields": ["firmwareVersion", "configuration"],
  "timestamp": "2025-01-12T10:35:00Z"
}
```

### 4.3 Revoke Device

**POST** `/devices/{deviceId}/revoke`

Revoke device credentials and block access.

**Request:**
```json
{
  "reason": "compromised",
  "immediate": true
}
```

**Response:** `200 OK`
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "status": "revoked",
  "revokedAt": "2025-01-12T10:40:00Z",
  "certificateRevoked": true
}
```

## 5. Firmware Management Endpoints

### 5.1 Check Firmware Updates

**GET** `/firmware/updates?deviceId={deviceId}&currentVersion={version}`

Check for available firmware updates.

**Response:** `200 OK`
```json
{
  "updateAvailable": true,
  "latestVersion": "2.5.2",
  "currentVersion": "2.5.1",
  "releaseDate": "2025-01-10T00:00:00Z",
  "isCritical": false,
  "securityFixes": ["CVE-2025-0001", "CVE-2025-0002"],
  "downloadUrl": "https://updates.example.com/firmware/2.5.2",
  "checksum": "sha256:abc123...",
  "size": 10485760,
  "signature": "base64-signature"
}
```

### 5.2 Download Firmware

**GET** `/firmware/download/{version}?deviceId={deviceId}`

Download firmware image with authentication.

**Response:** `200 OK`
- Content-Type: `application/octet-stream`
- Headers:
  - `X-Firmware-Version: 2.5.2`
  - `X-Checksum-SHA256: abc123...`
  - `X-Signature: base64-signature`

### 5.3 Report Firmware Installation

**POST** `/firmware/installations`

Report successful firmware installation.

**Request:**
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "fromVersion": "2.5.1",
  "toVersion": "2.5.2",
  "installStatus": "success",
  "installDuration": 120,
  "timestamp": "2025-01-12T11:00:00Z",
  "verificationHash": "sha256:xyz789..."
}
```

**Response:** `201 Created`
```json
{
  "installationId": "inst-123456",
  "verified": true,
  "timestamp": "2025-01-12T11:00:00Z"
}
```

## 6. Security Policy Endpoints

### 6.1 Get Security Policies

**GET** `/policies?deviceId={deviceId}`

Retrieve security policies for a device.

**Response:** `200 OK`
```json
{
  "policies": [
    {
      "policyId": "pol-001",
      "name": "Authentication Policy",
      "version": "1.0",
      "rules": {
        "requireMutualTLS": true,
        "minimumTLSVersion": "1.3",
        "allowedCipherSuites": ["TLS_AES_256_GCM_SHA384"],
        "certificateValidation": "strict"
      },
      "appliedAt": "2025-01-12T00:00:00Z"
    },
    {
      "policyId": "pol-002",
      "name": "Encryption Policy",
      "version": "1.0",
      "rules": {
        "encryptionAtRest": true,
        "encryptionAlgorithm": "AES-256-GCM",
        "keyRotationDays": 90
      },
      "appliedAt": "2025-01-12T00:00:00Z"
    }
  ]
}
```

### 6.2 Apply Security Policy

**POST** `/policies/apply`

Apply a security policy to devices.

**Request:**
```json
{
  "policyId": "pol-003",
  "targetDevices": ["550e8400-e29b-41d4-a716-446655440000"],
  "enforceImmediately": true
}
```

**Response:** `200 OK`
```json
{
  "applied": true,
  "affectedDevices": 1,
  "timestamp": "2025-01-12T11:05:00Z"
}
```

## 7. Security Audit Endpoints

### 7.1 Run Security Audit

**POST** `/audits/run`

Initiate a security audit on devices.

**Request:**
```json
{
  "deviceIds": ["550e8400-e29b-41d4-a716-446655440000"],
  "auditType": "comprehensive",
  "checks": ["certificates", "encryption", "authentication", "vulnerabilities"]
}
```

**Response:** `202 Accepted`
```json
{
  "auditId": "audit-789",
  "status": "running",
  "startedAt": "2025-01-12T11:10:00Z",
  "estimatedCompletion": "2025-01-12T11:15:00Z"
}
```

### 7.2 Get Audit Results

**GET** `/audits/{auditId}`

Retrieve audit results.

**Response:** `200 OK`
```json
{
  "auditId": "audit-789",
  "status": "completed",
  "startedAt": "2025-01-12T11:10:00Z",
  "completedAt": "2025-01-12T11:14:30Z",
  "results": {
    "overallScore": 92,
    "findings": [
      {
        "severity": "medium",
        "category": "encryption",
        "issue": "Weak cipher suite detected",
        "recommendation": "Update to TLS 1.3 with AES-256-GCM",
        "affectedDevices": 1
      }
    ],
    "passed": 18,
    "failed": 2,
    "warnings": 1
  }
}
```

## 8. Incident Response Endpoints

### 8.1 Report Security Incident

**POST** `/incidents`

Report a security incident.

**Request:**
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "severity": "high",
  "category": "authentication",
  "description": "Multiple failed authentication attempts",
  "detectedAt": "2025-01-12T11:20:00Z",
  "evidence": {
    "sourceIP": "192.168.1.100",
    "attemptCount": 5,
    "timeWindow": "60s"
  }
}
```

**Response:** `201 Created`
```json
{
  "incidentId": "inc-456",
  "status": "open",
  "assignedTo": "security-team",
  "priority": "high",
  "createdAt": "2025-01-12T11:20:00Z",
  "ticketUrl": "https://security.example.com/incidents/inc-456"
}
```

### 8.2 Get Incident Details

**GET** `/incidents/{incidentId}`

Retrieve incident details and response status.

**Response:** `200 OK`
```json
{
  "incidentId": "inc-456",
  "status": "investigating",
  "timeline": [
    {
      "timestamp": "2025-01-12T11:20:00Z",
      "action": "incident_reported",
      "actor": "device-550e8400"
    },
    {
      "timestamp": "2025-01-12T11:21:00Z",
      "action": "assigned_to_team",
      "actor": "system"
    },
    {
      "timestamp": "2025-01-12T11:25:00Z",
      "action": "investigation_started",
      "actor": "security-analyst"
    }
  ],
  "resolution": null
}
```

## 9. Error Responses

### 9.1 Standard Error Format
```json
{
  "error": {
    "code": "DEVICE_NOT_FOUND",
    "message": "Device with ID 550e8400-e29b-41d4-a716-446655440000 not found",
    "timestamp": "2025-01-12T11:30:00Z",
    "requestId": "req-abc123"
  }
}
```

### 9.2 Common Error Codes

| HTTP Status | Error Code | Description |
|-------------|------------|-------------|
| 400 | INVALID_REQUEST | Malformed request body |
| 401 | UNAUTHORIZED | Missing or invalid authentication |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | DEVICE_NOT_FOUND | Device does not exist |
| 409 | DEVICE_ALREADY_EXISTS | Device already registered |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily unavailable |

## 10. Rate Limiting

- Authentication: 10 requests/minute per device
- Device Management: 100 requests/minute per API key
- Firmware Download: 5 downloads/hour per device
- Security Audits: 10 audits/hour per account

**Rate Limit Headers:**
- `X-RateLimit-Limit: 100`
- `X-RateLimit-Remaining: 95`
- `X-RateLimit-Reset: 1673520000`

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
