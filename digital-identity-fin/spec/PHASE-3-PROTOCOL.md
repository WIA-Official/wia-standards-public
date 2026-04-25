# WIA-FIN-010: Digital Identity Standard
## PHASE 3: Protocol Specification

**Version:** 1.0.0  
**Status:** Official Standard  
**Last Updated:** 2025-12-25

---

## 1. Overview

This document specifies the protocols for the WIA Digital Identity Standard (WIA-FIN-010), including DID resolution, credential exchange, presentation protocols, trust frameworks, and communication protocols.

---

## 2. DID Resolution Protocol

### 2.1 Universal Resolver

```
Request: GET https://resolver.identity.wia.live/1.0/identifiers/{did}

Response:
{
  "@context": "https://w3id.org/did-resolution/v1",
  "didDocument": { /* DID document */ },
  "didResolutionMetadata": {
    "contentType": "application/did+ld+json",
    "retrieved": "2024-12-25T10:00:00Z"
  },
  "didDocumentMetadata": {
    "created": "2024-01-01T00:00:00Z",
    "updated": "2024-12-25T09:00:00Z"
  }
}
```

### 2.2 Method-Specific Resolution

#### did:ethr Resolution
1. Extract Ethereum address from DID
2. Query EthrDIDRegistry smart contract
3. Retrieve DID document from contract events
4. Return resolved document

#### did:web Resolution
1. Extract domain and path from DID
2. Construct URL: `https://{domain}/.well-known/did.json` or `https://{domain}/{path}/did.json`
3. Fetch via HTTPS
4. Validate HTTPS certificate
5. Return DID document

---

## 3. Credential Exchange Protocol

### 3.1 Issuance Flow (DIDComm)

```
1. Holder → Issuer: Request credential offer
   {
     "type": "https://didcomm.org/issue-credential/2.0/request-credential",
     "body": {
       "credential_type": "UniversityDegreeCredential"
     }
   }

2. Issuer → Holder: Send credential offer
   {
     "type": "https://didcomm.org/issue-credential/2.0/offer-credential",
     "body": {
       "credential_preview": { /* preview */ },
       "offers~attach": [ /* offer details */ ]
     }
   }

3. Holder → Issuer: Accept offer
   {
     "type": "https://didcomm.org/issue-credential/2.0/request-credential"
   }

4. Issuer → Holder: Issue credential
   {
     "type": "https://didcomm.org/issue-credential/2.0/issue-credential",
     "credentials~attach": [ /* signed VC */ ]
   }
```

### 3.2 Presentation Flow (DIDComm)

```
1. Verifier → Holder: Request presentation
   {
     "type": "https://didcomm.org/present-proof/2.0/request-presentation",
     "body": {
       "proof_request": {
         "requested_credentials": [
           {
             "credential_type": "KYCCredential",
             "constraints": { "kycLevel": { "$gte": 2 } }
           }
         ]
       },
       "challenge": "random_nonce",
       "domain": "verifier.example.com"
     }
   }

2. Holder → Verifier: Present credentials
   {
     "type": "https://didcomm.org/present-proof/2.0/presentation",
     "presentations~attach": [ /* VP */ ]
   }

3. Verifier: Verify presentation
   - Check signatures
   - Validate challenge and domain
   - Verify credential validity
   - Check revocation status

4. Verifier → Holder: Acknowledge
   {
     "type": "https://didcomm.org/present-proof/2.0/ack",
     "body": { "status": "accepted" }
   }
```

---

## 4. DIDComm Messaging Protocol

### 4.1 Message Structure

```json
{
  "id": "1234567890",
  "type": "https://didcomm.org/protocol/1.0/message_type",
  "from": "did:example:alice",
  "to": ["did:example:bob"],
  "created_time": 1735123200,
  "expires_time": 1735209600,
  "body": { /* message payload */ },
  "attachments": [ /* optional attachments */ ]
}
```

### 4.2 Encryption

**JWE (JSON Web Encryption) Format:**
```json
{
  "protected": "base64url(header)",
  "recipients": [
    {
      "encrypted_key": "base64url(encrypted_CEK)",
      "header": { "kid": "did:example:bob#key-agreement-1" }
    }
  ],
  "iv": "base64url(initialization_vector)",
  "ciphertext": "base64url(encrypted_content)",
  "tag": "base64url(authentication_tag)"
}
```

### 4.3 Signing

**JWS (JSON Web Signature) Format:**
```json
{
  "payload": "base64url(message)",
  "signatures": [
    {
      "protected": "base64url(header)",
      "signature": "base64url(signature)"
    }
  ]
}
```

---

## 5. Trust Framework Protocol

### 5.1 Trust Registry

**Registry Query:**
```
GET https://trust-registry.wia.live/issuers?type=UniversityDegreeCredential

Response:
{
  "issuers": [
    {
      "did": "did:example:mit",
      "name": "Massachusetts Institute of Technology",
      "types": ["UniversityDegreeCredential"],
      "accreditation": "US_DOE",
      "status": "active",
      "validFrom": "2020-01-01T00:00:00Z"
    }
  ]
}
```

### 5.2 Issuer Registration

```
POST https://trust-registry.wia.live/register

Request:
{
  "did": "did:example:university",
  "name": "University Name",
  "credentialTypes": ["UniversityDegreeCredential"],
  "accreditation": {
    "body": "US_DOE",
    "certificateUrl": "https://accreditation.gov/cert/123"
  },
  "proof": { /* signature from DID */ }
}
```

---

## 6. Revocation Protocol

### 6.1 Status List Publication

```
1. Issuer creates bitstring (1 = revoked, 0 = valid)
2. Compress bitstring (GZIP)
3. Encode as base64
4. Publish as StatusList2021Credential
5. Update at regular intervals (e.g., every hour)
```

### 6.2 Status Check

```
1. Extract credentialStatus from VC
2. Fetch StatusList2021Credential from statusListCredential URL
3. Decode and decompress encodedList
4. Check bit at statusListIndex
5. If bit = 1, credential is revoked
```

---

## 7. Cross-Border Identity Protocol

### 7.1 International Credential Recognition

```
1. Issuer issues credential with international context
   {
     "@context": [
       "https://www.w3.org/2018/credentials/v1",
       "https://identity.wia.live/contexts/international/v1"
     ],
     "type": ["VerifiableCredential", "InternationalPassport"],
     "credentialSubject": {
       "passportNumber": "X12345678",
       "nationality": "US",
       "icaoCompliant": true
     }
   }

2. Verifier checks trust registry for international issuers
3. Validates credential against international standards
4. Checks bilateral recognition agreements
```

### 7.2 Mutual Recognition Protocol

```
Country A ↔ Trust Agreement ↔ Country B

1. Establish trust framework between jurisdictions
2. Define recognized credential types
3. Set minimum assurance levels
4. Enable cross-border verification
5. Monitor compliance and revocation
```

---

## 8. Biometric Verification Protocol

### 8.1 Enrollment Protocol

```
1. Capture biometric data (photo, fingerprint, etc.)
2. Perform liveness detection
3. Extract biometric features
4. Generate protected template (cancelable/irreversible)
5. Store template with DID association
6. Issue biometric credential
```

### 8.2 Verification Protocol

```
1. User presents DID
2. System requests biometric sample
3. Perform liveness detection
4. Extract features from sample
5. Retrieve protected template from storage
6. Compare sample features to template
7. Return match/no-match result
```

---

## 9. Zero-Knowledge Proof Protocol

### 9.1 Age Verification Protocol

```
1. Prover inputs:
   - Private: birthdate
   - Public: minimumAge, currentDate

2. Generate proof:
   - Circuit verifies: (currentDate - birthdate) / 365.25 >= minimumAge
   - Generate zk-SNARK proof

3. Verifier:
   - Receives proof + public inputs
   - Verifies proof mathematically
   - Learns only: "age >= minimumAge" (yes/no)
   - Never learns actual birthdate
```

### 9.2 Selective Disclosure Protocol (BBS+)

```
1. Issuer signs credential with BBS+ signature
2. Holder receives credential with all attributes
3. When presenting:
   - Holder selects attributes to reveal
   - Generates derived proof for selected attributes
   - Sends proof + revealed attributes to verifier
4. Verifier validates derived proof
5. Verifier learns only revealed attributes
```

---

## 10. Security Protocols

### 10.1 Key Rotation Protocol

```
1. Generate new key pair
2. Add new key to DID document
3. Begin signing with new key
4. Maintain old key for verification of existing credentials
5. After transition period, remove old key
6. Update all new credentials to reference new key
```

### 10.2 DID Deactivation Protocol

```
1. Controller signs deactivation message
2. Update DID document:
   {
     "id": "did:example:123",
     "deactivated": true
   }
3. Publish deactivation to respective DID method
4. All future resolution returns deactivated status
5. Prevent reactivation (permanent)
```

---

## 11. Privacy Protocols

### 11.1 Pairwise DID Protocol

```
For each relationship:
1. Generate unique DID
2. Never reuse across relationships
3. Enable unlinkability

Example:
Alice ↔ Bank: did:peer:alice-bank-123
Alice ↔ Employer: did:peer:alice-employer-456
Alice ↔ Hospital: did:peer:alice-hospital-789
```

### 11.2 Minimal Disclosure Protocol

```
1. Verifier specifies exactly what's needed
2. Holder provides minimal credential set
3. Use selective disclosure when possible
4. Provide zero-knowledge proofs for range/threshold checks
5. Avoid oversharing
```

---

## 12. Interoperability Protocols

### 12.1 WIA Standard Integration

```
Digital Identity (WIA-FIN-010)
    ↓
    ├→ Blockchain Finance (WIA-FIN-002): DID anchoring
    ├→ Cryptocurrency (WIA-FIN-003): Wallet identity
    ├→ DeFi (WIA-FIN-004): Reputation credentials
    └→ Smart Contracts (WIA-FIN-007): Identity-gated execution
```

### 12.2 Legacy System Bridge Protocol

```
1. Legacy System uses traditional auth (username/password)
2. Bridge Service maintains DID ↔ Legacy ID mapping
3. User authenticates with DID
4. Bridge translates to legacy credentials
5. Gradual migration path to full SSI
```

---

## 13. Compliance Protocols

### 13.1 GDPR Compliance Protocol

```
1. Right to Access: Holder can export all credentials
2. Right to Erasure: Delete credentials from wallet
3. Right to Portability: Export in standard JSON-LD format
4. Consent: Every credential share requires explicit consent
5. Purpose Limitation: Credentials used only for stated purpose
```

### 13.2 AML/KYC Protocol

```
1. Customer initiates KYC
2. Identity proofing (document + biometric)
3. Sanctions screening
4. Risk assessment
5. Issue KYC credential with level
6. Ongoing monitoring
7. Periodic re-verification
```

---

**Document Version:** 1.0.0  
**Status:** Official Standard  
© 2025 WIA - World Interoperability Alliance

---

## Annex A — Conformance Tier Matrix

WIA conformance for digital-identity-fin is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/digital-identity-fin/api/` — TypeScript SDK skeleton
- `wia-standards/standards/digital-identity-fin/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/digital-identity-fin/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
