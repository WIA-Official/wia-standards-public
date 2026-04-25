# WIA-CORE-001: Phase 1 - Data Format Standards

> **Phase:** 1 of 4  
> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27

---

## Table of Contents

1. [Overview](#overview)
2. [Decentralized Identifiers (DIDs)](#decentralized-identifiers-dids)
3. [DID Methods](#did-methods)
4. [DID Documents](#did-documents)
5. [Verifiable Credentials](#verifiable-credentials)
6. [Credential Schemas](#credential-schemas)
7. [Verifiable Presentations](#verifiable-presentations)
8. [Data Validation](#data-validation)
9. [Interoperability](#interoperability)
10. [Examples](#examples)

---

## Overview

Phase 1 establishes the foundational data formats for universal identity. Without standardized data structures, interoperability is impossible. This phase defines how identities, credentials, and related data are represented, stored, and exchanged.

### Core Components

- **Decentralized Identifiers (DIDs):** Globally unique, user-controlled identifiers
- **DID Documents:** Machine-readable documents describing how to verify and interact with a DID
- **Verifiable Credentials (VCs):** Cryptographically-signed claims about identity attributes
- **Credential Schemas:** Standard formats for common credential types
- **Verifiable Presentations (VPs):** Collections of credentials presented for verification

### Design Principles

1. **Standards Compliance:** Build on W3C DID Core and VC Data Model specifications
2. **Format Flexibility:** Support both JSON and JSON-LD representations
3. **Cryptographic Agility:** Allow multiple signature algorithms
4. **Schema Extensibility:** Enable custom credential types while maintaining core interoperability
5. **Human Readability:** Ensure data can be understood by both machines and humans

---

## Decentralized Identifiers (DIDs)

### DID Syntax

Per W3C DID Core specification:

```
did:<method>:<method-specific-identifier>
```

**Components:**
- `did:` - Required scheme identifier
- `<method>` - Method name (e.g., "web", "key", "ethr")
- `<method-specific-identifier>` - Unique identifier within the method

### Examples

```
did:web:example.com:users:alice
did:key:z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH
did:ethr:0x3f7a8F93A837a3a2b0e2F7E5D8cA3f2e9B0dF5c6
did:ion:EiDyOQbbZAa3aiRzeCkV7LOx3SERjjH93EXoIM3UoN4oWg
```

### DID Resolution

DID resolution is the process of obtaining a DID Document from a DID:

```python
def resolve_did(did: str) -> DIDDocument:
    """
    Resolve DID to DID Document
    Returns: DIDDocument object or raises ResolutionError
    """
    method = extract_method(did)
    resolver = get_resolver_for_method(method)
    return resolver.resolve(did)
```

---

## DID Methods

### Supported Methods

#### did:web (Recommended for Production)

Web-based DID method using HTTPS for resolution.

**Format:** `did:web:domain.com:path`

**Resolution:** 
- Convert DID to HTTPS URL
- Fetch DID Document from `https://domain.com/path/did.json`

**Advantages:**
- Easy to implement
- Leverages existing web infrastructure
- No blockchain or ledger required
- Cost-effective

**Example:**
```
DID: did:web:example.com:users:alice
URL: https://example.com/users/alice/did.json
```

#### did:key (Recommended for Offline/Simple Use)

Self-contained, key-based DID method.

**Format:** `did:key:z6Mk...` (multibase-encoded public key)

**Resolution:** Derive DID Document from the key itself

**Advantages:**
- No external dependencies
- Works offline
- Cryptographically self-contained
- Immediate creation

**Example:**
```
did:key:z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH
```

#### did:ethr (Blockchain-Based)

Ethereum-based DID method.

**Format:** `did:ethr:0x...` (Ethereum address)

**Resolution:** Query Ethereum DID Registry smart contract

**Advantages:**
- Decentralized
- Censorship-resistant
- Publicly verifiable

#### did:ion (Bitcoin-Anchored)

ION (Identity Overlay Network) on Bitcoin.

**Format:** `did:ion:EiD...`

**Advantages:**
- Highly decentralized
- Bitcoin security
- Scalable (Sidetree protocol)

### Method Selection Guidelines

| Use Case | Recommended Method | Rationale |
|----------|-------------------|-----------|
| Web applications | did:web | Easy integration, no blockchain |
| Offline/air-gapped | did:key | No network required |
| Maximum decentralization | did:ion | Bitcoin security |
| Ethereum ecosystem | did:ethr | Native integration |

---

## DID Documents

### Structure

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://w3id.org/security/suites/ed25519-2020/v1"
  ],
  "id": "did:web:example.com:users:alice",
  "controller": "did:web:example.com:users:alice",
  
  "verificationMethod": [{
    "id": "did:web:example.com:users:alice#keys-1",
    "type": "Ed25519VerificationKey2020",
    "controller": "did:web:example.com:users:alice",
    "publicKeyMultibase": "zH3C2AVvLMv6gmMNam3uVAjZpfkcJCwDwnZn6z3wXmqPV"
  }],
  
  "authentication": [
    "did:web:example.com:users:alice#keys-1"
  ],
  
  "assertionMethod": [
    "did:web:example.com:users:alice#keys-1"
  ],
  
  "keyAgreement": [{
    "id": "did:web:example.com:users:alice#keys-2",
    "type": "X25519KeyAgreementKey2020",
    "controller": "did:web:example.com:users:alice",
    "publicKeyMultibase": "z6LSbysY2xFMRpGMhb7tFTLMpeuPRaqaWM1yECx2AtzE3KCc"
  }],
  
  "service": [{
    "id": "did:web:example.com:users:alice#profile",
    "type": "IdentityHub",
    "serviceEndpoint": "https://hub.example.com/alice"
  }]
}
```

### Required Properties

- `@context`: JSON-LD context (must include DID v1 context)
- `id`: The DID this document describes
- `verificationMethod`: Array of verification methods (public keys)

### Optional Properties

- `controller`: DID(s) that control this DID Document
- `authentication`: Verification methods for authentication
- `assertionMethod`: Verification methods for assertions/claims
- `keyAgreement`: Key agreement methods for encryption
- `capabilityInvocation`: Methods for invoking capabilities
- `capabilityDelegation`: Methods for delegating capabilities
- `service`: Service endpoints associated with the DID

---

## Verifiable Credentials

### Credential Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://www.w3.org/2018/credentials/examples/v1"
  ],
  "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "type": ["VerifiableCredential", "EmailCredential"],
  "issuer": "did:web:example.com",
  "issuanceDate": "2025-01-15T19:23:24Z",
  "expirationDate": "2026-01-15T19:23:24Z",
  
  "credentialSubject": {
    "id": "did:web:example.com:users:alice",
    "email": "alice@example.com",
    "emailVerified": true,
    "verifiedAt": "2025-01-15T19:23:24Z"
  },
  
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T19:23:24Z",
    "verificationMethod": "did:web:example.com#keys-1",
    "proofPurpose": "assertionMethod",
    "jws": "eyJhbGciOiJFZERTQSIsImI2NCI6ZmFsc2UsImNyaXQiOlsiYjY0Il19..BhWew0x"
  }
}
```

### Required Properties

- `@context`: Must include credentials/v1
- `type`: Must include "VerifiableCredential"
- `credentialSubject`: Claims about the subject
- `issuer`: DID of the credential issuer
- `issuanceDate`: When the credential was issued
- `proof`: Cryptographic proof (signature)

### Optional Properties

- `id`: Unique identifier for the credential
- `expirationDate`: When the credential expires
- `credentialStatus`: Revocation status information

---

## Credential Schemas

### Email Credential

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": ["VerifiableCredential", "EmailCredential"],
  "credentialSubject": {
    "email": "user@example.com",
    "emailVerified": true,
    "verifiedAt": "2025-01-15T19:23:24Z"
  }
}
```

### Phone Credential

```json
{
  "type": ["VerifiableCredential", "PhoneCredential"],
  "credentialSubject": {
    "phone": "+1-555-0100",
    "phoneVerified": true,
    "verifiedAt": "2025-01-15T19:23:24Z"
  }
}
```

### Identity Credential

```json
{
  "type": ["VerifiableCredential", "IdentityCredential"],
  "credentialSubject": {
    "fullName": "Alice Smith",
    "dateOfBirth": "1990-01-01",
    "nationality": "US",
    "idType": "passport",
    "idNumberHash": "sha256:abc123...",
    "verificationMethod": "government_id_check"
  }
}
```

---

## Verifiable Presentations

### Presentation Structure

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": ["VerifiablePresentation"],
  "holder": "did:web:example.com:users:alice",
  "verifiableCredential": [
    { /* Email Credential */ },
    { /* Phone Credential */ }
  ],
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T20:00:00Z",
    "verificationMethod": "did:web:example.com:users:alice#keys-1",
    "proofPurpose": "authentication",
    "challenge": "1f44d-6f6f61-71-72",
    "jws": "eyJhbGciOiJFZERTQSJ9..GF5NCy"
  }
}
```

### Challenge-Response

Verifiers include a `challenge` to prevent replay attacks:

```json
{
  "challenge": "1f44d-6f6f61-71-72",
  "domain": "verifier.example.com"
}
```

Holder includes this challenge in the presentation proof.

---

## Data Validation

### DID Validation

```python
def validate_did(did: str) -> bool:
    """Validate DID syntax"""
    pattern = r'^did:[a-z0-9]+:[a-zA-Z0-9._-]+$'
    return re.match(pattern, did) is not None
```

### Credential Validation

```python
def validate_credential(credential: dict) -> ValidationResult:
    """Validate credential structure and cryptographic proof"""
    errors = []
    
    # Check required fields
    if '@context' not in credential:
        errors.append('Missing @context')
    if 'type' not in credential or 'VerifiableCredential' not in credential['type']:
        errors.append('Missing or invalid type')
    
    # Verify proof
    if 'proof' in credential:
        if not verify_proof(credential):
            errors.append('Invalid cryptographic proof')
    
    # Check expiration
    if 'expirationDate' in credential:
        if parse_date(credential['expirationDate']) < datetime.now():
            errors.append('Credential expired')
    
    return ValidationResult(valid=len(errors) == 0, errors=errors)
```

---

## Interoperability

### Canonical Serialization

Use JSON Canonicalization Scheme (JCS) for consistent signing:

```python
import json

def canonicalize(data: dict) -> str:
    """Canonicalize JSON for signing"""
    return json.dumps(data, separators=(',', ':'), sort_keys=True)
```

### Format Conversion

Support both JSON and JSON-LD:

```python
def to_json_ld(credential: dict) -> dict:
    """Add JSON-LD context if missing"""
    if '@context' not in credential:
        credential['@context'] = ['https://www.w3.org/2018/credentials/v1']
    return credential
```

---

## Examples

### Complete Example: Email Credential Issuance

```python
from datetime import datetime, timedelta
import uuid

def issue_email_credential(issuer_did: str, subject_did: str, email: str):
    credential = {
        '@context': [
            'https://www.w3.org/2018/credentials/v1',
            'https://www.w3.org/2018/credentials/examples/v1'
        ],
        'id': f'urn:uuid:{uuid.uuid4()}',
        'type': ['VerifiableCredential', 'EmailCredential'],
        'issuer': issuer_did,
        'issuanceDate': datetime.utcnow().isoformat() + 'Z',
        'expirationDate': (datetime.utcnow() + timedelta(days=365)).isoformat() + 'Z',
        'credentialSubject': {
            'id': subject_did,
            'email': email,
            'emailVerified': True,
            'verifiedAt': datetime.utcnow().isoformat() + 'Z'
        }
    }
    
    # Sign credential
    proof = sign_credential(credential, issuer_did)
    credential['proof'] = proof
    
    return credential
```

---

**Phase 1 Complete.** Proceed to [Phase 2: API Interface](PHASE-2-API-INTERFACE.md).

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of universal-identity so that conformance claims at any
Phase remain unambiguous.*

