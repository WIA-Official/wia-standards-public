# WIA-FIN-010: Digital Identity Standard
## PHASE 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Official Standard  
**Last Updated:** 2025-12-25

---

## 1. Overview

This document specifies the data formats for the WIA Digital Identity Standard (WIA-FIN-010), including W3C Decentralized Identifiers (DIDs), Verifiable Credentials (VCs), Verifiable Presentations (VPs), biometric templates, and zero-knowledge proofs.

### 1.1 Design Principles

- **W3C Compliance:** Full compatibility with W3C DID and VC specifications
- **Interoperability:** Work seamlessly across platforms and implementations
- **Privacy:** Privacy-preserving by default
- **Extensibility:** Support future enhancements without breaking changes
- **Security:** Cryptographically secure data structures

---

## 2. Decentralized Identifiers (DIDs)

### 2.1 DID Syntax

```
did                = "did:" method-name ":" method-specific-id
method-name        = 1*method-char
method-char        = %x61-7A / DIGIT
method-specific-id = *( *idchar ":" ) 1*idchar
idchar             = ALPHA / DIGIT / "." / "-" / "_"
```

**Examples:**
```
did:ethr:0x3b0BC51Ab9De1e5B7B6E34E5b960285805C41736
did:key:z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH
did:web:example.com
did:ion:EiClkZMDxPKqC9c-umQfTkR8vvZ9JPhl_xLDI9Nfk38w5w
```

### 2.2 DID Document Structure

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://w3id.org/security/suites/ed25519-2020/v1"
  ],
  "id": "did:example:123456789abcdefghi",
  "controller": "did:example:123456789abcdefghi",
  "authentication": [
    {
      "id": "did:example:123456789abcdefghi#keys-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:example:123456789abcdefghi",
      "publicKeyMultibase": "zH3C2AVvLMv6gmMNam3uVAjZpfkcJCwDwnZn6z3wXmqPV"
    }
  ],
  "assertionMethod": ["did:example:123456789abcdefghi#keys-1"],
  "keyAgreement": [
    {
      "id": "did:example:123456789abcdefghi#keys-2",
      "type": "X25519KeyAgreementKey2020",
      "controller": "did:example:123456789abcdefghi",
      "publicKeyMultibase": "z9hFgmPVfmBZwRvFEyniQDBkz9LmV7gDEqytWyGZLmDXE"
    }
  ],
  "service": [
    {
      "id": "did:example:123456789abcdefghi#messaging",
      "type": "MessagingService",
      "serviceEndpoint": "https://example.com/messaging"
    }
  ]
}
```

### 2.3 Supported Verification Methods

| Type | Algorithm | Use Case | Key Size |
|------|-----------|----------|----------|
| Ed25519VerificationKey2020 | Ed25519 | Signatures | 32 bytes |
| EcdsaSecp256k1VerificationKey2019 | secp256k1 | Ethereum compatibility | 33 bytes |
| RsaVerificationKey2018 | RSA | Legacy systems | 2048/4096 bits |
| X25519KeyAgreementKey2020 | X25519 | Encryption | 32 bytes |

---

## 3. Verifiable Credentials

### 3.1 Basic Credential Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://www.w3.org/2018/credentials/examples/v1"
  ],
  "id": "http://example.edu/credentials/3732",
  "type": ["VerifiableCredential", "UniversityDegreeCredential"],
  "issuer": "did:example:university123",
  "issuanceDate": "2023-05-15T00:00:00Z",
  "expirationDate": "2028-05-14T23:59:59Z",
  "credentialSubject": {
    "id": "did:example:alice456",
    "degree": {
      "type": "BachelorDegree",
      "name": "Bachelor of Science in Computer Science"
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2023-05-15T12:00:00Z",
    "verificationMethod": "did:example:university123#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQjecWh...signature...gZpfkcJCwDw"
  }
}
```

### 3.2 Credential Schema

```json
{
  "id": "https://example.com/schemas/KYCCredential",
  "type": "JsonSchema",
  "version": "1.0",
  "name": "KYC Credential Schema",
  "author": "did:example:issuer",
  "schema": {
    "$schema": "http://json-schema.org/draft-07/schema#",
    "type": "object",
    "properties": {
      "credentialSubject": {
        "type": "object",
        "properties": {
          "id": {"type": "string"},
          "kycLevel": {"type": "integer", "minimum": 1, "maximum": 4},
          "verificationDate": {"type": "string", "format": "date-time"},
          "documentType": {"type": "string", "enum": ["passport", "national_id", "drivers_license"]},
          "countryCode": {"type": "string", "pattern": "^[A-Z]{2}$"}
        },
        "required": ["id", "kycLevel", "verificationDate"]
      }
    }
  }
}
```

### 3.3 Proof Types

#### 3.3.1 Ed25519Signature2020

```json
{
  "type": "Ed25519Signature2020",
  "created": "2023-05-15T12:00:00Z",
  "verificationMethod": "did:example:issuer#key-1",
  "proofPurpose": "assertionMethod",
  "proofValue": "z58DAdFfa9Sk...Ed25519_signature...2sDe2"
}
```

#### 3.3.2 BbsBlsSignature2020 (Selective Disclosure)

```json
{
  "type": "BbsBlsSignature2020",
  "created": "2023-05-15T12:00:00Z",
  "verificationMethod": "did:example:issuer#bbs-key-1",
  "proofPurpose": "assertionMethod",
  "proofValue": "lrx...BBS+_signature...kd2"
}
```

---

## 4. Verifiable Presentations

### 4.1 Presentation Structure

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": "VerifiablePresentation",
  "holder": "did:example:holder",
  "verifiableCredential": [
    { /* VC 1 */ },
    { /* VC 2 */ }
  ],
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2024-12-25T10:30:00Z",
    "verificationMethod": "did:example:holder#key-1",
    "proofPurpose": "authentication",
    "challenge": "9f86d081884c7d659a2feaa0c55ad015",
    "domain": "verifier.example.com",
    "proofValue": "z3sj...holder_signature...9dk"
  }
}
```

### 4.2 Presentation Request

```json
{
  "id": "request-123",
  "type": "PresentationRequest",
  "from": "did:example:verifier",
  "created": "2024-12-25T10:00:00Z",
  "challenge": "9f86d081884c7d659a2feaa0c55ad015",
  "domain": "verifier.example.com",
  "query": [
    {
      "type": "QueryByExample",
      "credentialQuery": {
        "reason": "We need to verify your KYC status",
        "example": {
          "@context": ["https://www.w3.org/2018/credentials/v1"],
          "type": "KYCCredential",
          "credentialSubject": {
            "kycLevel": {
              "$gte": 2
            }
          }
        }
      }
    }
  ]
}
```

---

## 5. Biometric Data Format

### 5.1 Biometric Template

```json
{
  "type": "BiometricTemplate",
  "modality": "facial",
  "version": "1.0",
  "created": "2024-12-25T10:00:00Z",
  "template": {
    "algorithm": "FaceNet",
    "vector": [0.123, -0.456, 0.789, ...], // 128-512 dimensions
    "quality": 0.95,
    "metadata": {
      "captureDevice": "smartphone_camera",
      "resolution": "1920x1080",
      "lighting": "natural"
    }
  },
  "protection": {
    "method": "cancelable",
    "key": "user_specific_key_id",
    "irreversible": true
  }
}
```

### 5.2 Liveness Proof

```json
{
  "type": "LivenessProof",
  "method": "passive",
  "timestamp": "2024-12-25T10:30:00Z",
  "confidence": 0.98,
  "indicators": {
    "textureAnalysis": true,
    "depthSensing": true,
    "microExpressions": true,
    "bloodFlow": true
  },
  "videoHash": "sha256:a3d2b1c4e5f6...",
  "deviceAttestation": {
    "manufacturer": "Apple",
    "model": "iPhone 15 Pro",
    "attestation": "hardware_backed_keystore"
  }
}
```

---

## 6. Zero-Knowledge Proofs

### 6.1 Age Proof (zk-SNARK)

```json
{
  "type": "ZeroKnowledgeProof",
  "proofType": "AgeVerification",
  "algorithm": "groth16",
  "curve": "bn128",
  "claim": "age >= 18",
  "proof": {
    "pi_a": ["0x123...", "0x456..."],
    "pi_b": [["0x789...", "0xabc..."], ["0xdef...", "0x012..."]],
    "pi_c": ["0x345...", "0x678..."]
  },
  "publicInputs": {
    "minimumAge": 18,
    "currentTimestamp": 1735123200
  },
  "verificationKey": {
    "protocol": "groth16",
    "vk_alpha_1": ["0x...", "0x..."],
    "vk_beta_2": [["0x...", "0x..."], ["0x...", "0x..."]],
    "vk_gamma_2": [["0x...", "0x..."], ["0x...", "0x..."]],
    "vk_delta_2": [["0x...", "0x..."], ["0x...", "0x..."]],
    "IC": [["0x...", "0x..."], ["0x...", "0x..."]]
  }
}
```

### 6.2 Selective Disclosure Proof

```json
{
  "type": "SelectiveDisclosureProof",
  "algorithm": "BBS+",
  "disclosedAttributes": ["name", "dateOfBirth"],
  "proof": {
    "proofValue": "lrx...BBS+_derived_proof...kd2",
    "nonce": "random_nonce_from_verifier"
  },
  "revealedMessages": {
    "name": "Alice Johnson",
    "dateOfBirth": "1995-06-15"
  }
}
```

---

## 7. Revocation Status

### 7.1 Status List 2021

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/vc/status-list/2021/v1"
  ],
  "id": "https://example.com/status/1",
  "type": ["VerifiableCredential", "StatusList2021Credential"],
  "issuer": "did:example:issuer",
  "issuanceDate": "2024-01-01T00:00:00Z",
  "credentialSubject": {
    "id": "https://example.com/status/1#list",
    "type": "StatusList2021",
    "statusPurpose": "revocation",
    "encodedList": "H4sIAAAAAAAA...compressed_bitstring...AAAAAA=="
  }
}
```

### 7.2 Credential Status Reference

```json
{
  "credentialStatus": {
    "id": "https://example.com/status/1#94567",
    "type": "StatusList2021Entry",
    "statusPurpose": "revocation",
    "statusListIndex": "94567",
    "statusListCredential": "https://example.com/status/1"
  }
}
```

---

## 8. Data Encoding and Serialization

### 8.1 Character Encoding

- **UTF-8** for all text data
- **Base64url** for binary data in JSON
- **Multibase** for cryptographic material (with `z` prefix for base58btc)

### 8.2 Date and Time Format

- **ISO 8601** format: `YYYY-MM-DDThh:mm:ssZ`
- Always use UTC timezone (Z suffix)
- Milliseconds optional: `2024-12-25T10:30:45.123Z`

### 8.3 Cryptographic Material Encoding

```
// Public keys
Ed25519: multibase encoded (z + base58btc)
Example: z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH

// Signatures
Base64url encoded or multibase encoded depending on proof type

// Hashes
Multihash format: hash_function_id + length + hash_value
Example (SHA-256): 1220 + hash_bytes
```

---

## 9. JSON-LD Context

### 9.1 Required Contexts

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/security/suites/ed25519-2020/v1"
  ]
}
```

### 9.2 WIA Custom Context

```json
{
  "@context": {
    "@version": 1.1,
    "wia": "https://standards.wia.live/ns/identity/v1#",
    "KYCCredential": "wia:KYCCredential",
    "BiometricTemplate": "wia:BiometricTemplate",
    "ZeroKnowledgeProof": "wia:ZeroKnowledgeProof",
    "kycLevel": {"@id": "wia:kycLevel", "@type": "xsd:integer"},
    "biometricModality": "wia:biometricModality",
    "zkProofType": "wia:zkProofType"
  }
}
```

---

## 10. Validation Rules

### 10.1 DID Validation

1. **Syntax:** Must match DID syntax規則
2. **Method:** Method name must be registered or recognized
3. **Length:** Total length ≤ 2048 characters
4. **Characters:** Only allowed characters in each component

### 10.2 Credential Validation

1. **Required Fields:** @context, type, issuer, issuanceDate, credentialSubject, proof
2. **Dates:** issuanceDate ≤ current time; if expirationDate exists, expirationDate > current time
3. **Proof:** Valid cryptographic signature from issuer's verification method
4. **Revocation:** Check credential status if specified
5. **Schema:** Validate against credential schema if referenced

### 10.3 Presentation Validation

1. **Challenge:** Must match verifier's challenge
2. **Domain:** Must match verifier's domain
3. **Holder Binding:** Proof signed by holder's key
4. **Credential Validity:** All included credentials are valid
5. **Freshness:** Created timestamp within acceptable window (e.g., 5 minutes)

---

## 11. Security Considerations

### 11.1 Key Management

- Minimum key sizes: Ed25519 (256-bit), secp256k1 (256-bit), RSA (2048-bit)
- Key rotation: Support key rotation without changing DID
- Key revocation: Immediate revocation capability

### 11.2 Privacy Protection

- Minimize data disclosure
- Support selective disclosure
- Use pairwise DIDs when appropriate
- Implement unlinkability where possible

### 11.3 Data Integrity

- All credentials must be cryptographically signed
- Use collision-resistant hash functions (SHA-256 minimum)
- Implement tamper detection mechanisms

---

## 12. Compliance and Standards

### 12.1 W3C Standards

- DID Core 1.0 (July 2022)
- Verifiable Credentials Data Model 1.1 (March 2022)
- JSON-LD 1.1
- Security Vocabulary

### 12.2 Cryptographic Standards

- NIST FIPS 186-4 (Digital Signature Standard)
- RFC 8032 (Ed25519)
- RFC 7518 (JSON Web Algorithms)

### 12.3 Data Protection

- GDPR compliant
- CCPA compliant
- Support data minimization
- Enable right to erasure

---

**Document Version:** 1.0.0  
**Status:** Official Standard  
**Effective Date:** 2025-01-01  
**Maintained by:** WIA Technical Committee

© 2025 WIA - World Interoperability Alliance
