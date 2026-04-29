# WIA-IOT_SECURITY: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the data format requirements for IoT Security. All implementations MUST follow these specifications to ensure interoperability across IoT security systems and devices.

## 2. Data Structures

### 2.1 Device Identity Record Format

```json
{
  "type": "WIA-IOT_SECURITY:DeviceIdentity",
  "version": "1.0",
  "deviceId": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "identity": {
    "manufacturer": "string",
    "model": "string",
    "serialNumber": "string",
    "firmwareVersion": "string",
    "hardwareVersion": "string",
    "macAddress": "string",
    "certificateId": "string (X.509 cert ID)"
  },
  "signature": "string (Ed25519)"
}
```

### 2.2 Security Credential Format

```json
{
  "type": "WIA-IOT_SECURITY:Credential",
  "version": "1.0",
  "credentialId": "string (UUID v4)",
  "deviceId": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "credential": {
    "type": "certificate|token|key",
    "algorithm": "RSA-2048|ED25519|ECDSA-P256",
    "publicKey": "string (PEM format)",
    "certificateChain": ["string"],
    "issuedBy": "string",
    "validFrom": "ISO 8601",
    "validUntil": "ISO 8601",
    "status": "active|revoked|expired"
  },
  "signature": "string"
}
```

### 2.3 Authentication Token Format

```json
{
  "type": "WIA-IOT_SECURITY:AuthToken",
  "version": "1.0",
  "tokenId": "string (UUID v4)",
  "deviceId": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "token": {
    "jwt": "string (JWT token)",
    "scope": ["read", "write", "admin"],
    "issuedAt": "ISO 8601",
    "expiresAt": "ISO 8601",
    "refreshToken": "string (optional)"
  },
  "signature": "string"
}
```

### 2.4 Security Event Record

```json
{
  "type": "WIA-IOT_SECURITY:SecurityEvent",
  "version": "1.0",
  "eventId": "string (UUID v4)",
  "deviceId": "string (UUID v4)",
  "timestamp": "ISO 8601",
  "event": {
    "severity": "critical|high|medium|low|info",
    "category": "authentication|authorization|encryption|network|firmware|physical",
    "eventType": "string",
    "description": "string",
    "source": "string",
    "destination": "string (optional)",
    "metadata": {}
  },
  "signature": "string"
}
```

## 3. Field Definitions

### 3.1 Device Identity Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| deviceId | UUID | Yes | Unique device identifier |
| manufacturer | string | Yes | Device manufacturer name |
| model | string | Yes | Device model number |
| serialNumber | string | Yes | Unique serial number |
| firmwareVersion | string | Yes | Current firmware version |
| hardwareVersion | string | Yes | Hardware revision |
| macAddress | string | Yes | Network MAC address |
| certificateId | string | Yes | X.509 certificate ID |

### 3.2 Credential Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| credentialId | UUID | Yes | Unique credential identifier |
| type | enum | Yes | certificate, token, or key |
| algorithm | enum | Yes | Cryptographic algorithm used |
| publicKey | string | Yes | Public key in PEM format |
| certificateChain | array | No | Certificate chain for validation |
| validFrom | ISO 8601 | Yes | Credential validity start |
| validUntil | ISO 8601 | Yes | Credential expiration date |
| status | enum | Yes | Current credential status |

## 4. Data Types

### 4.1 Core Types

| Type | Format | Example |
|------|--------|---------|
| String | UTF-8 | "IoT-Device-001" |
| Number | IEEE 754 | 443 |
| Boolean | true/false | true |
| Date | ISO 8601 | "2025-01-12T00:00:00Z" |
| UUID | RFC 4122 | "550e8400-e29b-41d4-a716-446655440000" |
| MAC Address | IEEE 802 | "00:1A:2B:3C:4D:5E" |

### 4.2 Security Types

```typescript
interface DeviceCredential {
  credentialType: 'X509' | 'JWT' | 'HMAC' | 'PSK';
  keyMaterial: ArrayBuffer;
  storageType: 'TPM' | 'TEE' | 'HSM' | 'Software';
  rotationPolicy: RotationPolicy;
}

interface RotationPolicy {
  enabled: boolean;
  intervalDays: number;
  warningDays: number;
  autoRotate: boolean;
}

interface SecurityLevel {
  level: 1 | 2 | 3 | 4 | 5;  // 1=minimal, 5=maximum
  requiresHardwareSecurity: boolean;
  requiresTLS: boolean;
  requiresMutualAuth: boolean;
  minimumKeyLength: number;
}
```

## 5. Encoding Requirements

### 5.1 Character Encoding
- All text MUST be UTF-8 encoded
- No BOM (Byte Order Mark) allowed
- Line endings: LF (Unix style)
- JSON format for all data interchange

### 5.2 Binary Encoding
- Use Base64 for binary data in JSON
- Use DER encoding for certificates
- Big-endian byte order for network transmission
- CBOR for constrained devices (optional)

### 5.3 Cryptographic Requirements
- All signatures MUST use Ed25519 or ECDSA P-256
- All certificates MUST be X.509 v3 compliant
- All hashes MUST use SHA-256 or better
- All symmetric encryption MUST use AES-256-GCM or ChaCha20-Poly1305

## 6. Validation Rules

### 6.1 Device Identity Validation
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["type", "version", "deviceId", "timestamp", "identity", "signature"],
  "properties": {
    "type": { "type": "string", "const": "WIA-IOT_SECURITY:DeviceIdentity" },
    "version": { "type": "string", "pattern": "^\\d+\\.\\d+$" },
    "deviceId": { "type": "string", "format": "uuid" },
    "timestamp": { "type": "string", "format": "date-time" },
    "identity": {
      "type": "object",
      "required": ["manufacturer", "model", "serialNumber"],
      "properties": {
        "manufacturer": { "type": "string", "minLength": 1 },
        "model": { "type": "string", "minLength": 1 },
        "serialNumber": { "type": "string", "minLength": 1 }
      }
    }
  }
}
```

### 6.2 Validation Errors

| Code | Message | Resolution |
|------|---------|------------|
| E001 | Invalid device ID format | Use valid UUID v4 |
| E002 | Missing required field | Add all required fields |
| E003 | Invalid certificate format | Use X.509 v3 format |
| E004 | Signature verification failed | Re-sign with valid key |
| E005 | Expired credential | Renew credential |
| E006 | Invalid MAC address | Use IEEE 802 format |
| E007 | Unsupported algorithm | Use approved algorithm |

## 7. Security Event Categories

### 7.1 Event Severity Levels

| Level | Description | Response Time | Example |
|-------|-------------|---------------|---------|
| critical | Immediate threat | < 5 minutes | Device compromise |
| high | Serious concern | < 30 minutes | Authentication failure |
| medium | Potential issue | < 2 hours | Unusual traffic pattern |
| low | Minor anomaly | < 24 hours | Configuration change |
| info | Informational | As needed | Routine event |

### 7.2 Event Categories

- **authentication**: Login attempts, credential validation
- **authorization**: Access control, permission checks
- **encryption**: TLS handshakes, key exchanges
- **network**: Network connections, traffic anomalies
- **firmware**: Updates, integrity checks
- **physical**: Tamper detection, hardware events

## 8. Examples

### 8.1 Complete Device Identity Record
```json
{
  "type": "WIA-IOT_SECURITY:DeviceIdentity",
  "version": "1.0",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-12T10:30:00Z",
  "identity": {
    "manufacturer": "SecureIoT Corp",
    "model": "SI-2000",
    "serialNumber": "SI2000-2025-001234",
    "firmwareVersion": "2.5.1",
    "hardwareVersion": "Rev B",
    "macAddress": "00:1A:2B:3C:4D:5E",
    "certificateId": "cert-550e8400"
  },
  "signature": "MEUCIQDx1y2z3a4b5c6d7e8f9g0h1i2j3k4l5m6n7o8p9q0r1s2tUV3WX4Y5Z6A7B8C9D0E1F2G3H4I5J6K7L8M9N0O1P2Q3R4S5T6U7V8W9X0Y1Z2"
}
```

### 8.2 Security Event Example
```json
{
  "type": "WIA-IOT_SECURITY:SecurityEvent",
  "version": "1.0",
  "eventId": "650e8400-e29b-41d4-a716-446655440001",
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-12T10:32:15Z",
  "event": {
    "severity": "high",
    "category": "authentication",
    "eventType": "authentication_failure",
    "description": "Failed authentication attempt from unknown IP",
    "source": "192.168.1.100",
    "destination": "device-gateway.example.com",
    "metadata": {
      "attemptCount": 3,
      "lastAttempt": "2025-01-12T10:32:15Z",
      "protocol": "MQTT"
    }
  },
  "signature": "MEUCIQCa1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7r8s9t0u1v2w3x4y5z6A7B8C9D0E1F2G3H4I5J6K7L8M9N0O1P2Q3R4S5T6U7V8W9X0Y1Z2A3B4C5D6"
}
```

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
