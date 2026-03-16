# WIA-LEG-010: Digital Identity After Death Standard - Specification v1.0

**Status:** Official Release
**Version:** 1.0.0
**Date:** 2025-12-25
**Category:** Legal & Identity (LEG)

## Abstract

This specification defines the WIA-LEG-010 Digital Identity After Death Standard, providing comprehensive guidelines for managing digital identities, credentials, and assets after an individual's death. The standard covers death verification protocols, identity revocation mechanisms, posthumous authentication, digital legacy management, AI persona services, and the legal/ethical frameworks necessary for implementation.

## 1. Introduction

### 1.1 Purpose

The purpose of this standard is to:
- Establish protocols for verifying death events and integrating with government registries
- Define systematic identity revocation mechanisms for deceased individuals
- Specify posthumous authentication and access control for authorized parties (executors, beneficiaries)
- Provide frameworks for digital legacy preservation and memorial services
- Address ethical considerations around AI personas of deceased individuals
- Enable legal compliance across multiple jurisdictions

### 1.2 Scope

This standard applies to:
- Digital identity providers and platforms
- Social media networks and online services
- Financial institutions managing digital assets
- Healthcare providers with patient data
- Cryptocurrency and blockchain services
- Digital estate planning services
- Memorial and legacy platforms
- Legal and regulatory authorities

### 1.3 Terminology

- **Death Verification**: Process of confirming and authenticating that a death has occurred
- **Proof-of-Death**: Cryptographically verifiable, immutable evidence of death anchored to blockchain
- **Credential Revocation**: Systematic invalidation of authentication mechanisms associated with a deceased individual
- **Posthumous Authentication**: Controlled access granted to authorized parties after death verification
- **Digital Legacy**: Curated collection of digital content, relationships, and achievements preserved after death
- **AI Persona**: Artificial intelligence system trained on deceased individual's data to simulate their communication patterns
- **Executor**: Legal representative authorized to manage a deceased individual's estate and digital assets
- **DID**: Decentralized Identifier - self-sovereign identity credential

## 2. Death Verification Protocol

### 2.1 Integration Tiers

The standard defines three integration tiers for death verification:

#### Tier 1: Direct API Integration
- Real-time integration with government death registries via RESTful APIs
- Automatic death event notifications via webhooks
- OAuth 2.0 or similar authentication
- Recommended for jurisdictions with advanced digital infrastructure

#### Tier 2: Certificate Verification
- Digital or physical death certificate submission
- Multi-source verification against government databases
- Cryptographic signature validation
- Applicable for most jurisdictions

#### Tier 3: Manual Verification
- Physical death certificate review by trusted third parties
- Notarized attestations from legal representatives
- Fallback option for jurisdictions with limited digital infrastructure

### 2.2 Death Certificate Data Schema

All death certificates must conform to the following JSON schema:

```json
{
  "certificateId": "string (UUID)",
  "jurisdiction": "string (ISO 3166-1 alpha-2 + subdivision)",
  "issuingAuthority": "string",
  "deceasedIdentity": {
    "legalName": "string",
    "dateOfBirth": "string (ISO 8601)",
    "nationalId": "string (encrypted)",
    "digitalIdentifiers": [
      {
        "type": "enum [DID, email, phoneNumber, governmentID]",
        "value": "string (encrypted)",
        "verified": "boolean"
      }
    ]
  },
  "deathEvent": {
    "dateOfDeath": "string (ISO 8601 datetime)",
    "placeOfDeath": {
      "jurisdiction": "string",
      "facility": "string (optional)",
      "coordinates": "object (optional, encrypted)"
    }
  },
  "certification": {
    "certifierType": "enum [physician, medical_examiner, coroner]",
    "certifierName": "string",
    "certificationDate": "string (ISO 8601 datetime)",
    "digitalSignature": "string"
  },
  "verification": {
    "verificationMethod": "enum [api, certificate, manual]",
    "verificationSources": ["array"],
    "consensusLevel": "integer",
    "verificationTimestamp": "string (ISO 8601 datetime)",
    "blockchainAnchor": {
      "chain": "string",
      "transactionHash": "string",
      "blockNumber": "integer",
      "timestamp": "string (ISO 8601 datetime)"
    }
  }
}
```

### 2.3 Multi-Source Verification

Death verification MUST achieve consensus across multiple independent sources:

- **Primary Sources** (weight: 1.0): Government death registries, vital statistics offices
- **Medical Sources** (weight: 0.8): Hospital records, medical examiner databases
- **Legal Sources** (weight: 0.7): Court records, probate documents
- **Commercial Sources** (weight: 0.4): Obituary databases, funeral home records
- **Social Sources** (weight: 0.2): Verified family attestations

**Consensus Threshold**: Minimum 2.0 weighted consensus score required for death verification.

### 2.4 Blockchain Anchoring

Upon successful verification, a proof-of-death MUST be generated and anchored to a public blockchain:

```json
{
  "proofType": "death-verification-v1",
  "subjectDID": "did:wia:deceased:identifier",
  "certificateHash": "sha256:...",
  "verificationHash": "sha256:...",
  "consensusScore": "float",
  "timestamp": "string (ISO 8601 datetime)",
  "merkleRoot": "string (0x-prefixed hex)",
  "signature": "string (0x-prefixed hex)",
  "blockchain": {
    "chain": "string",
    "contract": "string (0x-prefixed address)",
    "transactionHash": "string (0x-prefixed hex)",
    "blockNumber": "integer"
  }
}
```

## 3. Identity Revocation

### 3.1 Cascading Revocation Algorithm

Upon death verification, credential revocation MUST follow a breadth-first search algorithm:

1. Enumerate all credentials associated with deceased identity
2. Identify dependency relationships between credentials
3. Revoke credentials in dependency order
4. Notify all relying parties
5. Log all revocation actions with timestamps

### 3.2 Credential Types

The following credential types MUST be supported:

- Password-based credentials
- Session tokens (cookies, local storage)
- OAuth/OpenID tokens
- API keys and access tokens
- Biometric templates (fingerprints, facial recognition)
- Hardware security keys
- Blockchain credentials (DIDs, verifiable credentials)
- Derived credentials

### 3.3 Revocation Notification

All relying parties MUST be notified of credential revocation via webhook:

```json
POST /webhooks/identity-events
{
  "eventType": "user.deceased",
  "userId": "string",
  "deathVerification": {
    "verified": true,
    "verificationTimestamp": "string (ISO 8601)",
    "blockchainProof": "string",
    "consensusScore": "float"
  },
  "revokedTokens": ["array of token identifiers"],
  "recommendedAction": "enum [memorialize, delete, transfer]"
}
```

### 3.4 Biometric Purging

Biometric templates MUST be:
- Securely overwritten (DOD 5220.22-M standard or equivalent)
- Deleted from all systems within 24 hours of death verification
- Logged in immutable audit trail

## 4. Posthumous Authentication

### 4.1 Executor Authorization

Executors may be granted controlled access after verifying legal authority. Authorization requires:

- Valid probate order, will, or court appointment
- Multi-party verification (minimum 2 sources)
- Time-limited credentials (default: 1 year)
- Comprehensive audit logging

### 4.2 Access Control Schema

```json
{
  "credentialId": "UUID",
  "credentialType": "executor_access",
  "executorId": "string",
  "deceasedId": "string",
  "issuedAt": "string (ISO 8601)",
  "expiresAt": "string (ISO 8601)",
  "permissions": ["enum array: read, export, delete, modify]",
  "requiresMultiSig": "boolean",
  "multiSigConfig": {
    "requiredSignatures": "integer",
    "signers": ["array of executor IDs"]
  },
  "legalBasis": {
    "documentType": "string",
    "documentId": "string",
    "issuingCourt": "string",
    "verificationHash": "string"
  }
}
```

### 4.3 Audit Requirements

All posthumous access MUST be logged with:
- Accessor identity and role
- Resource accessed
- Timestamp and duration
- Actions performed
- Legal authorization basis
- Blockchain anchor (every 100 access events)

## 5. Digital Legacy Management

### 5.1 Legacy Curation

Content curation MUST consider:
- User pre-death instructions (highest priority)
- Significance scoring (engagement, life events, relationships)
- Privacy filters (exclude sensitive content)
- Quality thresholds
- Storage and preservation requirements

### 5.2 Memorial Platform Requirements

Memorial platforms MUST provide:
- Access control (public, private, family-only)
- Tribute submission with moderation
- Content preservation in archival formats (PDF/A, TIFF, FLAC)
- Multi-location redundant storage
- Format migration plans (minimum every 5 years)

## 6. AI Persona Framework

### 6.1 Consent Requirement

AI persona creation REQUIRES explicit consent:
- Pre-death user authorization (preferred)
- OR posthumous decision by authorized family with evidence of likely wishes
- Ethical review for all implementations

### 6.2 Persona Types

The standard recognizes five persona complexity levels:
1. Static Chatbot (pre-scripted responses)
2. Retrieval-Based (actual quotes only)
3. Generative Text (AI-created text in person's style)
4. Multimodal (text + voice + image)
5. Full Avatar (real-time audiovisual interaction)

### 6.3 Prohibited Uses

AI personas MUST NOT be used for:
- Commercial endorsements
- Political statements
- False evidence or testimony
- Fraud or impersonation
- Interactions with minors without guardian consent
- Unregulated therapeutic purposes

### 6.4 Attribution Requirement

All AI persona interactions MUST clearly state:
- This is an AI simulation, not the actual person
- What training data was used
- Limitations and inaccuracies
- Access to grief support resources

## 7. Security Requirements

### 7.1 Encryption

All sensitive data MUST be encrypted:
- At rest: AES-256 or equivalent
- In transit: TLS 1.3 or higher
- Field-level encryption for PII

### 7.2 Access Control

Implementation MUST include:
- Multi-factor authentication for executor access
- Time-locked credentials
- Least privilege access
- Regular access reviews and revocation

### 7.3 Fraud Prevention

Systems MUST implement:
- Anomaly detection for suspicious death claims
- Rate limiting on verification attempts
- Multi-source verification requirements
- Manual review for high-risk scenarios

## 8. Privacy and Compliance

### 8.1 GDPR Compliance

For EU users:
- Default to privacy protection
- Honor explicit user instructions
- Provide limited executor access for legitimate interests
- Document legal basis for all processing

### 8.2 Jurisdiction-Specific Requirements

Implementations MUST:
- Map applicable laws in all operational jurisdictions
- Implement jurisdiction-specific handling
- Maintain conflict resolution protocols
- Provide user choice where laws conflict

## 9. Implementation Requirements

### 9.1 Minimum Requirements

All implementations MUST include:
- Death verification system (minimum Tier 2)
- Credential revocation capability
- Executor access portal
- Comprehensive audit logging
- Privacy controls

### 9.2 Recommended Features

Implementations SHOULD include:
- Blockchain proof anchoring
- Digital legacy tools
- Memorial services
- API integrations
- User education materials

## 10. Versioning and Updates

This specification follows semantic versioning:
- MAJOR: Incompatible changes
- MINOR: Backward-compatible functionality
- PATCH: Backward-compatible bug fixes

## 11. References

- RUFADAA (Revised Uniform Fiduciary Access to Digital Assets Act)
- GDPR (General Data Protection Regulation)
- ISO 8601 (Date and time format)
- ISO 3166-1 (Country codes)
- DID Core Specification (W3C)
- Verifiable Credentials Data Model (W3C)

## 12. Acknowledgments

This standard was developed with input from:
- Legal experts in digital estate law
- Privacy and security professionals
- Platform and service providers
- Digital rights organizations
- Grieving families and executors

---

**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 WIA (World Certification Industry Association)
SmileStory Inc. / WIA Official

Licensed under CC BY-SA 4.0
