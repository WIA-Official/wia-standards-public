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
