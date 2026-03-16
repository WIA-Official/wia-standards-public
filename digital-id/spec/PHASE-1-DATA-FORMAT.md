# WIA-SOC-002 PHASE 1: Data Format Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

This document specifies the data formats for digital identity in the WIA-SOC-002 standard, including Decentralized Identifiers (DIDs), DID Documents, Verifiable Credentials, and Verifiable Presentations.

### 1.1 Design Principles

- **W3C Compliance**: Full compatibility with W3C DID Core and VC Data Model
- **JSON-LD Based**: Semantic interoperability through linked data
- **Extensibility**: Support for custom claims and credential types
- **Privacy by Design**: Minimal disclosure, selective revelation
- **Blockchain Agnostic**: Support multiple blockchain networks

---

## 2. Decentralized Identifiers (DIDs)

### 2.1 DID Syntax

```
did:<method>:<method-specific-id>
```

**Examples:**
```
did:wia:0x7a3f9b2c4e8d1a5f6b9c3e2d7a1f4b8c9e2d5a1f
did:ethr:0x1234567890abcdef1234567890abcdef12345678
did:polygon:0xabcdef1234567890abcdef1234567890abcdef12
did:web:identity.example.com
```

### 2.2 DID Methods

#### WIA Method (`did:wia`)

Primary method for WIA digital identities.

**Format:**
```
did:wia:<network>:<address>
```

**Network Options:**
- `mainnet` - Production Ethereum mainnet
- `polygon` - Polygon PoS network
- `testnet` - Test networks (goerli, sepolia)

**Example:**
```
did:wia:mainnet:0x7a3f9b2c4e8d1a5f6b9c3e2d7a1f4b8c9e2d5a1f
```

---

## 3. DID Document

### 3.1 Structure

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://w3id.org/security/suites/ed25519-2020/v1"
  ],
  "id": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f6b9c3e2d7a1f4b8c9e2d5a1f",
  "controller": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f6b9c3e2d7a1f4b8c9e2d5a1f",
  "verificationMethod": [
    {
      "id": "did:wia:mainnet:0x7a3f...#key-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:wia:mainnet:0x7a3f...",
      "publicKeyMultibase": "z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH"
    }
  ],
  "authentication": [
    "did:wia:mainnet:0x7a3f...#key-1"
  ],
  "assertionMethod": [
    "did:wia:mainnet:0x7a3f...#key-1"
  ],
  "service": [
    {
      "id": "did:wia:mainnet:0x7a3f...#hub",
      "type": "IdentityHub",
      "serviceEndpoint": "https://hub.wiastandards.com/identity"
    }
  ],
  "created": "2025-12-26T10:00:00Z",
  "updated": "2025-12-26T10:00:00Z"
}
```

### 3.2 Verification Methods

Supported key types:

1. **Ed25519VerificationKey2020**
   - Algorithm: EdDSA with Ed25519 curve
   - Use: General purpose signing
   - Key size: 256 bits

2. **EcdsaSecp256k1VerificationKey2019**
   - Algorithm: ECDSA with secp256k1 curve
   - Use: Ethereum-compatible signatures
   - Key size: 256 bits

3. **JsonWebKey2020**
   - Format: JWK (RFC 7517)
   - Use: Web interoperability
   - Algorithms: ES256, ES384, ES512

### 3.3 Service Endpoints

```json
{
  "service": [
    {
      "id": "did:wia:mainnet:0x7a3f...#hub",
      "type": "IdentityHub",
      "serviceEndpoint": "https://hub.wiastandards.com/identity"
    },
    {
      "id": "did:wia:mainnet:0x7a3f...#messaging",
      "type": "MessagingService",
      "serviceEndpoint": "https://msg.wiastandards.com/v1"
    },
    {
      "id": "did:wia:mainnet:0x7a3f...#storage",
      "type": "IPFSStorage",
      "serviceEndpoint": "ipfs://QmY7Yh4UquoXHLPFo2XbhXkhBvFoPwmQUSa92pxnxjQuPU"
    }
  ]
}
```

---

## 4. Verifiable Credentials

### 4.1 Base Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wiastandards.com/credentials/v1"
  ],
  "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "type": ["VerifiableCredential", "IdentityCredential"],
  "issuer": {
    "id": "did:wia:mainnet:0x1234567890abcdef",
    "name": "WIA Identity Authority"
  },
  "issuanceDate": "2025-12-26T10:00:00Z",
  "expirationDate": "2027-12-26T10:00:00Z",
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
    "type": "Person",
    "name": "John Doe",
    "dateOfBirth": "1990-01-01",
    "nationality": "US"
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-26T10:00:00Z",
    "verificationMethod": "did:wia:mainnet:0x1234...#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z58DAdFfa9SkqZMVPxAQpic7ndSayn1PzZs6ZjWp1CktyGesjuTSwRdoWhAfGFCF5bppETSTojQCrfFPP2oumHKtz"
  }
}
```

### 4.2 Credential Types

#### Identity Credential

```json
{
  "type": ["VerifiableCredential", "IdentityCredential"],
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "type": "Person",
    "givenName": "John",
    "familyName": "Doe",
    "email": "john.doe@example.com",
    "image": "https://example.com/photo.jpg"
  }
}
```

#### Educational Credential

```json
{
  "type": ["VerifiableCredential", "EducationalCredential"],
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "degree": {
      "type": "BachelorDegree",
      "name": "Bachelor of Science in Computer Science",
      "institution": "Massachusetts Institute of Technology",
      "graduationDate": "2020-05-15"
    }
  }
}
```

#### Professional License

```json
{
  "type": ["VerifiableCredential", "ProfessionalLicense"],
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "license": {
      "type": "DriverLicense",
      "number": "D1234567",
      "issuingAuthority": "California DMV",
      "class": "C",
      "validFrom": "2020-01-01",
      "validUntil": "2028-01-01"
    }
  }
}
```

---

## 5. Verifiable Presentations

### 5.1 Structure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1"
  ],
  "type": "VerifiablePresentation",
  "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "holder": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "verifiableCredential": [
    {
      "@context": ["https://www.w3.org/2018/credentials/v1"],
      "id": "urn:uuid:12345678-1234-1234-1234-123456789012",
      "type": ["VerifiableCredential", "IdentityCredential"],
      "issuer": "did:wia:mainnet:0x1234567890abcdef",
      "issuanceDate": "2025-12-26T10:00:00Z",
      "credentialSubject": {
        "id": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
        "name": "John Doe"
      },
      "proof": { }
    }
  ],
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-26T11:00:00Z",
    "verificationMethod": "did:wia:mainnet:0x7a3f...#key-1",
    "proofPurpose": "authentication",
    "challenge": "1f44d",
    "domain": "example.com",
    "proofValue": "z58DAdFfa9SkqZMVPxAQpic..."
  }
}
```

### 5.2 Selective Disclosure

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wiastandards.com/credentials/sd-v1"
  ],
  "type": "VerifiablePresentation",
  "selectiveDisclosure": {
    "revealed": ["name", "nationality"],
    "hidden": ["dateOfBirth", "address", "ssn"],
    "hashAlgorithm": "SHA-256",
    "salt": "a1b2c3d4e5f6"
  },
  "verifiableCredential": [ ]
}
```

---

## 6. Cryptographic Proofs

### 6.1 Signature Formats

#### Ed25519Signature2020

```json
{
  "type": "Ed25519Signature2020",
  "created": "2025-12-26T10:00:00Z",
  "verificationMethod": "did:wia:mainnet:0x7a3f...#key-1",
  "proofPurpose": "assertionMethod",
  "proofValue": "z58DAdFfa9SkqZMVPxAQpic7ndSayn1PzZs6ZjWp1CktyGesjuTSwRdoWhAfGFCF5bppETSTojQCrfFPP2oumHKtz"
}
```

#### EcdsaSecp256k1Signature2019

```json
{
  "type": "EcdsaSecp256k1Signature2019",
  "created": "2025-12-26T10:00:00Z",
  "verificationMethod": "did:ethr:0x1234...#controller",
  "proofPurpose": "assertionMethod",
  "jws": "eyJhbGciOiJFUzI1NksiLCJiNjQiOmZhbHNlLCJjcml0IjpbImI2NCJdfQ..MEYCIQDkKqCL0Ym9QE5rEslQCCKZ7VlsKn8L8RK2jXJ6_GQyBQIhAKpJ4J8vvSQlQKQvGkJHZJxFvKqVfC8CGBUxJ8KGQ1fS"
}
```

---

## 7. Revocation

### 7.1 Revocation List

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/vc/status-list/2021/v1"
  ],
  "id": "https://wiastandards.com/credentials/status/1",
  "type": ["VerifiableCredential", "StatusList2021Credential"],
  "issuer": "did:wia:mainnet:0x1234567890abcdef",
  "issuanceDate": "2025-12-26T10:00:00Z",
  "credentialSubject": {
    "id": "https://wiastandards.com/credentials/status/1#list",
    "type": "StatusList2021",
    "statusPurpose": "revocation",
    "encodedList": "H4sIAAAAAAAAA-3BMQEAAADCoPVPbQwfoAAAAAAAAAAAAAAAAAAAAIC3AYbSVKsAQAAA"
  }
}
```

### 7.2 Status in Credential

```json
{
  "credentialStatus": {
    "id": "https://wiastandards.com/credentials/status/1#94567",
    "type": "StatusList2021Entry",
    "statusPurpose": "revocation",
    "statusListIndex": "94567",
    "statusListCredential": "https://wiastandards.com/credentials/status/1"
  }
}
```

---

## 8. Zero-Knowledge Proofs

### 8.1 ZK Proof Structure

```json
{
  "type": "ZKProof",
  "proofSystem": "groth16",
  "curve": "bn254",
  "statement": {
    "type": "RangeProof",
    "claim": "age >= 18",
    "publicInputs": []
  },
  "proof": {
    "pi_a": ["0x1234...", "0x5678..."],
    "pi_b": [["0xabcd...", "0xef01..."], ["0x2345...", "0x6789..."]],
    "pi_c": ["0x9abc...", "0xdef0..."]
  },
  "verificationKey": {
    "protocol": "groth16",
    "curve": "bn254",
    "nPublic": 1,
    "vk_alpha_1": ["0x1234...", "0x5678..."],
    "vk_beta_2": [["0xabcd...", "0xef01..."], ["0x2345...", "0x6789..."]],
    "vk_gamma_2": [["0x9abc...", "0xdef0..."], ["0x1234...", "0x5678..."]],
    "vk_delta_2": [["0xabcd...", "0xef01..."], ["0x2345...", "0x6789..."]],
    "IC": [["0x1234...", "0x5678..."], ["0xabcd...", "0xef01..."]]
  }
}
```

---

## 9. Schema Definitions

### 9.1 JSON Schema for DID Document

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DID Document",
  "type": "object",
  "required": ["@context", "id"],
  "properties": {
    "@context": {
      "type": ["string", "array"]
    },
    "id": {
      "type": "string",
      "pattern": "^did:"
    },
    "controller": {
      "type": ["string", "array"]
    },
    "verificationMethod": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["id", "type", "controller"],
        "properties": {
          "id": { "type": "string" },
          "type": { "type": "string" },
          "controller": { "type": "string" }
        }
      }
    }
  }
}
```

---

## 10. Data Privacy

### 10.1 Minimal Disclosure

Credentials SHOULD include only necessary information:

```json
{
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "ageOver18": true
  }
}
```

Instead of:

```json
{
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "dateOfBirth": "1990-01-01",
    "name": "John Doe",
    "address": "123 Main St"
  }
}
```

---

## 11. Interoperability

### 11.1 Cross-Chain DIDs

Support for multiple blockchains:

```
did:wia:ethereum:0x7a3f...
did:wia:polygon:0x7a3f...
did:wia:solana:0x7a3f...
did:wia:binance:0x7a3f...
```

### 11.2 Legacy System Integration

Bridge to traditional identity systems:

```json
{
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f...",
    "sameAs": [
      "urn:uuid:12345678-1234-1234-1234-123456789012",
      "https://legacy-system.com/users/johndoe"
    ]
  }
}
```

---

## 12. Compliance

### 12.1 GDPR Compliance

- **Right to Erasure**: DIDs can be deactivated
- **Data Portability**: Export in JSON-LD format
- **Minimal Data**: Only essential claims
- **Consent Management**: Explicit consent in presentations

### 12.2 eIDAS Compliance

- **Qualified Electronic Signatures**: Support for QES
- **Trust Service Providers**: Integration with eIDAS TSPs
- **Cross-Border Recognition**: EU-wide validity

---

**Document Status:** Complete ✅
**Next Phase:** [PHASE-2-API.md](PHASE-2-API.md)

弘益人間 · Benefit All Humanity
