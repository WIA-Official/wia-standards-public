# WIA-AUTO-023 PHASE 1: Security Data Format Specification

**Version:** 1.0
**Status:** Final
**Date:** 2025-12-27
**Category:** Automotive Cybersecurity

---

## 1. Introduction

Phase 1 of the WIA-AUTO-023 Vehicle Cybersecurity Standard defines standardized data formats for security-critical information in automotive systems. These formats enable consistent representation and exchange of security data across diverse components, manufacturers, and organizations.

### 1.1 Purpose

The purpose of this specification is to:

- Establish standard data formats for threat signatures, security events, audit logs, encrypted messages, and credentials
- Enable interoperability between security components from different suppliers
- Facilitate threat intelligence sharing across the automotive industry
- Support regulatory compliance through standardized audit and reporting formats
- Provide foundation for higher-level security functions defined in Phases 2-4

### 1.2 Scope

This specification covers:

- **Threat Signature Format**: Standardized descriptions of known cybersecurity threats
- **Security Event Format**: Real-time security event logging and reporting
- **Audit Log Format**: Comprehensive audit trails for compliance and forensics
- **Encrypted Message Format**: Secure communication message structures
- **Security Credential Format**: Certificates, keys, and authentication tokens

### 1.3 Conformance

Implementations claiming conformance to WIA-AUTO-023 Phase 1 MUST:

- Support all mandatory data formats specified in this document
- Pass all validation tests in the WIA-AUTO-023 Phase 1 test suite
- Correctly parse and generate data formats according to provided JSON schemas
- Handle version compatibility as specified in Section 8

---

## 2. Threat Signature Format

### 2.1 Overview

Threat signatures provide standardized descriptions of known cybersecurity threats, enabling systems to recognize and respond to attacks consistently.

### 2.2 Signature Structure

```json
{
  "signatureId": "WIA-THR-AUTO-023-YYYY-NNNN",
  "version": "1.0",
  "timestamp": "ISO-8601-datetime",
  "threatCategory": "string",
  "severity": "CRITICAL|HIGH|MEDIUM|LOW|INFORMATIONAL",
  "confidence": 0-100,
  "affectedSystems": ["string"],
  "indicators": {
    "type-specific-fields": "value"
  },
  "description": "string",
  "mitigation": "string",
  "references": ["string"]
}
```

### 2.3 Field Definitions

#### 2.3.1 signatureId (REQUIRED)

Unique identifier for the threat signature.

- Format: `WIA-THR-AUTO-023-YYYY-NNNN`
- YYYY: Year of signature creation
- NNNN: Sequential number (0001-9999)
- Example: `WIA-THR-AUTO-023-2025-0042`

#### 2.3.2 version (REQUIRED)

Signature format version using semantic versioning.

- Type: String
- Format: `major.minor.patch`
- Example: `"1.0.0"`

#### 2.3.3 timestamp (REQUIRED)

Timestamp when signature was created or last updated.

- Type: String
- Format: ISO 8601 with UTC timezone
- Example: `"2025-12-27T10:00:00Z"`

#### 2.3.4 threatCategory (REQUIRED)

Category of the threat.

Allowed values:
- `CAN_BUS_INJECTION`
- `CAN_BUS_FLOODING`
- `ECU_IMPERSONATION`
- `TELEMATICS_COMPROMISE`
- `KEYLESS_ENTRY_ATTACK`
- `GPS_SPOOFING`
- `FIRMWARE_TAMPERING`
- `DIAGNOSTIC_PORT_ABUSE`
- `WIRELESS_EXPLOITATION`
- `SENSOR_SPOOFING`
- `OTA_UPDATE_ATTACK`
- `PRIVACY_VIOLATION`
- `SUPPLY_CHAIN_COMPROMISE`

#### 2.3.5 severity (REQUIRED)

Severity level based on potential impact and exploitability.

| Level | Impact | Exploitability | Response Time |
|-------|--------|----------------|---------------|
| CRITICAL | Safety systems compromise, remote control | Readily exploitable remotely | < 24 hours |
| HIGH | Data breach, vehicle theft, privacy violation | Exploitable with moderate skill | < 7 days |
| MEDIUM | Service degradation, limited data access | Requires physical access or specialized knowledge | < 30 days |
| LOW | Minor inconvenience, information disclosure | Difficult to exploit | < 90 days |
| INFORMATIONAL | No direct impact | Not directly exploitable | Next update cycle |

#### 2.3.6 confidence (REQUIRED)

Confidence level that the signature accurately identifies the threat (0-100).

- Type: Integer
- Range: 0-100
- 90-100: Very high confidence, tested and verified
- 70-89: High confidence, strong evidence
- 50-69: Medium confidence, theoretical or limited evidence
- 30-49: Low confidence, speculative
- 0-29: Very low confidence, informational only

#### 2.3.7 affectedSystems (REQUIRED)

List of vehicle systems potentially affected by the threat.

- Type: Array of strings
- Examples: `"ECU_GATEWAY"`, `"BRAKE_CONTROLLER"`, `"INFOTAINMENT"`, `"TELEMATICS"`

#### 2.3.8 indicators (REQUIRED)

Threat-specific technical indicators for detection.

Type: Object with threat-category-specific fields

Examples:
```json
// CAN_BUS_INJECTION indicators
"indicators": {
  "canMessageId": "0x123",
  "payloadPattern": "regex",
  "frequency": ">100/sec"
}

// TELEMATICS_COMPROMISE indicators
"indicators": {
  "networkSignature": "pattern",
  "unexpectedEndpoints": ["IP:port"],
  "abnormalDataExfiltration": "size/time"
}
```

#### 2.3.9 description (REQUIRED)

Human-readable description of the threat.

- Type: String
- Length: 50-500 characters
- Must clearly explain the threat, attack vector, and potential impact

#### 2.3.10 mitigation (REQUIRED)

Recommended mitigation or countermeasure.

- Type: String
- Length: 50-500 characters
- Must provide actionable guidance for preventing or detecting the threat

#### 2.3.11 references (OPTIONAL)

External references for additional information.

- Type: Array of strings
- May include CVE IDs, URLs, research papers, etc.
- Examples: `"CVE-2024-XXXXX"`, `"https://wia.org/threats/AUTO-023-0001"`

### 2.4 Example Threat Signature

```json
{
  "signatureId": "WIA-THR-AUTO-023-2025-0001",
  "version": "1.0",
  "timestamp": "2025-12-27T10:00:00Z",
  "threatCategory": "CAN_BUS_INJECTION",
  "severity": "CRITICAL",
  "confidence": 95,
  "affectedSystems": ["ECU_GATEWAY", "BRAKE_CONTROLLER"],
  "indicators": {
    "canMessageId": "0x123",
    "payloadPattern": "^FF00.*AB$",
    "frequency": ">100/sec"
  },
  "description": "Malicious high-frequency messages on CAN ID 0x123 attempting to override brake commands",
  "mitigation": "Enable CAN message authentication (MAC) and rate limiting for safety-critical message IDs",
  "references": [
    "CVE-2024-XXXXX",
    "https://wia.org/threats/AUTO-023-0001",
    "ISO-SAE-21434-2021"
  ]
}
```

---

## 3. Security Event Format

### 3.1 Overview

Security events represent real-time occurrences that may indicate attacks, anomalies, or other security-relevant activities.

### 3.2 Event Structure

```json
{
  "eventId": "EVT-{vehicleId}-{timestamp}-{sequence}",
  "timestamp": "ISO-8601-datetime",
  "vehicleId": "string",
  "eventType": "string",
  "severity": "CRITICAL|HIGH|MEDIUM|LOW|INFORMATIONAL",
  "sourceSystem": "string",
  "sourceIP": "string (optional)",
  "targetSystem": "string",
  "action": "string",
  "outcome": "SUCCESS|FAILURE|DENIED|TIMEOUT",
  "details": {},
  "mitigationTaken": "string (optional)",
  "correlationId": "string (optional)"
}
```

### 3.3 Event Categories

| Category | Description | Common Triggers |
|----------|-------------|-----------------|
| AUTHENTICATION | Authentication and authorization events | Login attempts, credential validation, token generation |
| NETWORK_ANOMALY | Unusual network traffic patterns | Port scans, flooding, unusual protocols |
| CAN_BUS_ANOMALY | Anomalous CAN bus activity | Unexpected messages, rate anomalies, invalid IDs |
| FIRMWARE_INTEGRITY | Firmware and code integrity checks | Checksum failures, unauthorized modifications |
| PRIVACY_VIOLATION | Unauthorized data access or exfiltration | Location tracking, PII access, data leaks |
| CONFIGURATION_CHANGE | Security configuration changes | Policy updates, key rotation, access control changes |
| INTRUSION_ATTEMPT | Detected intrusion attempts | Failed authentication, port scanning, exploit attempts |
| ANOMALY_DETECTED | Behavior deviating from baseline | Unusual patterns, unexpected state transitions |

---

## 4. Audit Log Format

### 4.1 Overview

Audit logs provide comprehensive, tamper-evident records of security-relevant activities for compliance, forensic analysis, and incident investigation.

### 4.2 Requirements

Audit logs MUST be:

- **Comprehensive**: All security-relevant activities logged
- **Tamper-Evident**: Cryptographic signing or hash chains
- **Time-Synchronized**: Accurate timestamps across systems
- **Searchable**: Structured format for efficient querying
- **Retained**: Preserved according to regulatory requirements

### 4.3 Audit Entry Structure

```json
{
  "logId": "LOG-{date}-{vehicleId}-{sequence}",
  "timestamp": "ISO-8601-datetime",
  "sequenceNumber": "integer",
  "previousHash": "SHA256:hex-string",
  "currentHash": "SHA256:hex-string",
  "vehicleId": "string",
  "actor": {
    "type": "HUMAN|SYSTEM|SERVICE",
    "identity": "string",
    "authMethod": "string"
  },
  "action": "string",
  "targetSystem": "string",
  "outcome": "SUCCESS|FAILURE|PARTIAL",
  "details": {},
  "location": {
    "facility": "string (optional)",
    "geoLocation": "lat,long (optional)"
  }
}
```

---

## 5. Encrypted Message Format

### 5.1 Overview

Encrypted messages protect confidentiality and integrity of communications. The format supports algorithm agility.

### 5.2 Message Envelope

```json
{
  "messageId": "MSG-{timestamp}-{sequence}",
  "version": "1.0",
  "timestamp": "ISO-8601-datetime",
  "sender": "did:wia:entity:{id}",
  "recipient": "did:wia:entity:{id}",
  "cryptoConfig": {
    "algorithm": "string",
    "keyDerivation": "string",
    "keyId": "string"
  },
  "encryptedData": "base64-string",
  "iv": "base64-string",
  "authTag": "base64-string",
  "signature": {
    "algorithm": "string",
    "value": "base64-string",
    "certificate": "base64-string (optional)"
  }
}
```

### 5.3 Supported Algorithms

| Purpose | Required | Recommended | Deprecated |
|---------|----------|-------------|------------|
| Symmetric Encryption | AES-256-GCM | AES-256-GCM, ChaCha20-Poly1305 | AES-128-CBC, 3DES |
| Asymmetric Encryption | RSA-2048-OAEP | RSA-3072-OAEP, ECIES-P256 | RSA-1024 |
| Digital Signatures | ECDSA-P256-SHA256 | ECDSA-P384-SHA384, EdDSA | RSA-SHA1 |
| Hash Functions | SHA-256 | SHA-256, SHA-384, SHA-512 | MD5, SHA-1 |
| Key Agreement | ECDH-P256 | ECDH-P384, X25519 | DH-1024 |

---

## 6. Security Credential Format

### 6.1 Verifiable Credential Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/auto-023/v1"
  ],
  "type": ["VerifiableCredential", "WIASecurityCertificate"],
  "issuer": "did:wia:issuer:{id}",
  "issuanceDate": "ISO-8601-datetime",
  "expirationDate": "ISO-8601-datetime",
  "credentialSubject": {
    "id": "did:wia:vehicle:{VIN}",
    "vehicleId": "VIN",
    "securityLevel": "WIA-AUTO-023-LEVEL-{1-5}",
    "certifiedPhases": [1, 2, 3, 4],
    "complianceStandards": ["string"]
  },
  "proof": {
    "type": "EcdsaSecp256k1Signature2019",
    "created": "ISO-8601-datetime",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:issuer:{id}#keys-1",
    "jws": "base64-string"
  }
}
```

---

## 7. Validation and Compliance

### 7.1 Schema Validation

All data formats MUST validate against provided JSON schemas available at:
- https://wia.org/schemas/auto-023/v1/threat-signature.json
- https://wia.org/schemas/auto-023/v1/security-event.json
- https://wia.org/schemas/auto-023/v1/audit-log.json
- https://wia.org/schemas/auto-023/v1/encrypted-message.json
- https://wia.org/schemas/auto-023/v1/credential.json

### 7.2 Test Vectors

Reference test vectors provided in WIA-AUTO-023 Phase 1 test suite include:

- 100+ valid examples for each format type
- 50+ invalid examples demonstrating common errors
- Edge cases and boundary conditions
- Cross-format integration scenarios

### 7.3 Certification Requirements

Phase 1 certification requires:

1. **Format Generation**: Correctly generate all data format types
2. **Format Parsing**: Correctly parse all valid formats
3. **Validation**: Reject all invalid formats with appropriate error messages
4. **Interoperability**: Successfully exchange data with reference implementation
5. **Performance**: Meet specified performance targets for the platform

---

## 8. Versioning and Compatibility

### 8.1 Version Format

Data formats use semantic versioning: `major.minor.patch`

- **Major**: Incompatible changes requiring parser updates
- **Minor**: Backward-compatible additions (new optional fields)
- **Patch**: Bug fixes, clarifications, no format changes

### 8.2 Compatibility Requirements

Implementations MUST:

- Support current major version
- Support previous major version for minimum 2 years after new major release
- Gracefully handle unknown optional fields (ignore, do not error)
- Validate version field and reject unsupported versions with clear error message

---

## 9. Security Considerations

### 9.1 Data Protection

- Audit logs MUST use hash chaining or cryptographic signing for tamper evidence
- Encrypted messages MUST use authenticated encryption (AEAD) modes
- Credentials MUST be protected at rest and in transit
- Sensitive data in logs MUST be masked or encrypted

### 9.2 Privacy

- Personal data in security events and audit logs MUST comply with applicable privacy regulations (GDPR, CCPA, etc.)
- Anonymization or pseudonymization MUST be used where appropriate
- Data retention policies MUST be defined and enforced

---

## 10. References

- **ISO/IEC 27001**: Information Security Management
- **ISO/SAE 21434**: Road vehicles — Cybersecurity engineering
- **UNECE R155**: Uniform provisions concerning the approval of vehicles with regards to cybersecurity
- **NIST Cybersecurity Framework**: Framework for Improving Critical Infrastructure Cybersecurity
- **W3C Verifiable Credentials**: Verifiable Credentials Data Model 1.0

---

**Document Control**

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-12-27 | Initial release | WIA Technical Committee |

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association
