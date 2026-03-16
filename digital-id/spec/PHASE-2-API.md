# WIA-SOC-002 PHASE 2: API Specification

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

This document specifies the RESTful API and SDK interfaces for WIA-SOC-002 digital identity operations, including DID management, credential issuance/verification, and zero-knowledge proof generation.

### 1.1 Design Principles

- **RESTful Architecture**: Standard HTTP methods and status codes
- **JSON-LD Format**: All payloads in JSON-LD format
- **Stateless**: No server-side session management
- **Secure**: TLS 1.3+ required, JWT authentication
- **Rate Limited**: Protection against abuse
- **Versioned**: API version in URL path

### 1.2 Base URL

```
Production: https://api.wiastandards.com/v1/identity
Testnet:    https://testnet-api.wiastandards.com/v1/identity
```

---

## 2. Authentication

### 2.1 JWT Tokens

All API requests require a valid JWT token in the `Authorization` header:

```http
Authorization: Bearer eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...
```

### 2.2 Obtaining Access Token

**Endpoint:** `POST /auth/token`

**Request:**
```json
{
  "did": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "challenge": "sign-this-nonce",
  "signature": "0x1234567890abcdef...",
  "publicKey": "0x04abcdef1234567890..."
}
```

**Response:**
```json
{
  "accessToken": "eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...",
  "tokenType": "Bearer",
  "expiresIn": 3600,
  "scope": "did:read did:write credential:issue credential:verify"
}
```

---

## 3. DID Operations

### 3.1 Create DID

**Endpoint:** `POST /did/create`

**Request:**
```json
{
  "method": "wia",
  "network": "mainnet",
  "options": {
    "keyType": "Ed25519VerificationKey2020",
    "controller": "did:wia:mainnet:0x1234...",
    "service": [
      {
        "type": "IdentityHub",
        "serviceEndpoint": "https://hub.example.com"
      }
    ]
  }
}
```

**Response:**
```json
{
  "did": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f6b9c3e2d7a1f4b8c9e2d5a1f",
  "didDocument": {
    "@context": ["https://www.w3.org/ns/did/v1"],
    "id": "did:wia:mainnet:0x7a3f...",
    "verificationMethod": [...],
    "authentication": [...],
    "assertionMethod": [...]
  },
  "keys": {
    "privateKey": "0x1234567890abcdef...",
    "publicKey": "0x04abcdef1234567890...",
    "keyId": "did:wia:mainnet:0x7a3f...#key-1"
  },
  "transactionHash": "0xabc123def456...",
  "blockNumber": 12345678
}
```

### 3.2 Resolve DID

**Endpoint:** `GET /did/{did}`

**Example:** `GET /did/did:wia:mainnet:0x7a3f9b2c4e8d1a5f`

**Response:**
```json
{
  "@context": ["https://www.w3.org/ns/did/v1"],
  "id": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "controller": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "verificationMethod": [
    {
      "id": "did:wia:mainnet:0x7a3f...#key-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:wia:mainnet:0x7a3f...",
      "publicKeyMultibase": "z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH"
    }
  ],
  "authentication": ["did:wia:mainnet:0x7a3f...#key-1"],
  "assertionMethod": ["did:wia:mainnet:0x7a3f...#key-1"],
  "created": "2025-12-26T10:00:00Z",
  "updated": "2025-12-26T10:00:00Z"
}
```

### 3.3 Update DID Document

**Endpoint:** `PUT /did/{did}`

**Request:**
```json
{
  "didDocument": {
    "@context": ["https://www.w3.org/ns/did/v1"],
    "id": "did:wia:mainnet:0x7a3f...",
    "service": [
      {
        "id": "did:wia:mainnet:0x7a3f...#new-service",
        "type": "MessagingService",
        "serviceEndpoint": "https://msg.example.com"
      }
    ]
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "verificationMethod": "did:wia:mainnet:0x7a3f...#key-1",
    "proofValue": "z58DAdFfa9SkqZ..."
  }
}
```

**Response:**
```json
{
  "success": true,
  "didDocument": {...},
  "transactionHash": "0xdef789abc456...",
  "updated": "2025-12-26T12:00:00Z"
}
```

### 3.4 Deactivate DID

**Endpoint:** `DELETE /did/{did}`

**Request Headers:**
```http
Authorization: Bearer eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9...
X-Signature: 0x1234567890abcdef...
```

**Response:**
```json
{
  "success": true,
  "did": "did:wia:mainnet:0x7a3f...",
  "deactivated": true,
  "timestamp": "2025-12-26T15:00:00Z",
  "transactionHash": "0xfed321cba987..."
}
```

---

## 4. Credential Operations

### 4.1 Issue Credential

**Endpoint:** `POST /credentials/issue`

**Request:**
```json
{
  "type": ["VerifiableCredential", "IdentityCredential"],
  "issuer": {
    "id": "did:wia:mainnet:0x1234567890abcdef",
    "name": "WIA Identity Authority"
  },
  "credentialSubject": {
    "id": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
    "name": "John Doe",
    "dateOfBirth": "1990-01-01",
    "nationality": "US"
  },
  "expirationDate": "2027-12-26T10:00:00Z",
  "options": {
    "proofType": "Ed25519Signature2020",
    "revocable": true
  }
}
```

**Response:**
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
  "credentialSubject": {...},
  "credentialStatus": {
    "id": "https://wiastandards.com/credentials/status/1#94567",
    "type": "StatusList2021Entry",
    "statusPurpose": "revocation",
    "statusListIndex": "94567"
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

### 4.2 Verify Credential

**Endpoint:** `POST /credentials/verify`

**Request:**
```json
{
  "verifiableCredential": {
    "@context": [...],
    "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
    "type": ["VerifiableCredential", "IdentityCredential"],
    "issuer": {...},
    "credentialSubject": {...},
    "proof": {...}
  },
  "options": {
    "checkRevocation": true,
    "checkExpiration": true,
    "verifySignature": true
  }
}
```

**Response:**
```json
{
  "verified": true,
  "checks": {
    "signature": {
      "valid": true,
      "issuer": "did:wia:mainnet:0x1234567890abcdef",
      "verificationMethod": "did:wia:mainnet:0x1234...#key-1"
    },
    "expiration": {
      "valid": true,
      "expirationDate": "2027-12-26T10:00:00Z"
    },
    "revocation": {
      "valid": true,
      "revoked": false,
      "statusListChecked": "https://wiastandards.com/credentials/status/1"
    }
  },
  "warnings": [],
  "timestamp": "2025-12-26T11:00:00Z"
}
```

### 4.3 Get Credential

**Endpoint:** `GET /credentials/{id}`

**Example:** `GET /credentials/urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5`

**Response:**
```json
{
  "@context": [...],
  "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "type": ["VerifiableCredential", "IdentityCredential"],
  "issuer": {...},
  "credentialSubject": {...},
  "proof": {...}
}
```

### 4.4 Revoke Credential

**Endpoint:** `POST /credentials/revoke`

**Request:**
```json
{
  "credentialId": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "reason": "credential-compromised",
  "proof": {
    "type": "Ed25519Signature2020",
    "verificationMethod": "did:wia:mainnet:0x1234...#key-1",
    "proofValue": "z58DAdFfa9SkqZ..."
  }
}
```

**Response:**
```json
{
  "success": true,
  "credentialId": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "revoked": true,
  "revokedAt": "2025-12-26T14:00:00Z",
  "reason": "credential-compromised",
  "statusListUpdated": true
}
```

---

## 5. Presentation Operations

### 5.1 Create Presentation

**Endpoint:** `POST /presentations/create`

**Request:**
```json
{
  "holder": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "verifiableCredential": [
    {
      "@context": [...],
      "id": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
      "type": ["VerifiableCredential", "IdentityCredential"],
      "issuer": {...},
      "credentialSubject": {...},
      "proof": {...}
    }
  ],
  "challenge": "1f44d",
  "domain": "example.com",
  "selectiveDisclosure": {
    "revealed": ["name", "nationality"],
    "hidden": ["dateOfBirth", "address"]
  }
}
```

**Response:**
```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": "VerifiablePresentation",
  "id": "urn:uuid:98765432-4321-4321-4321-987654321098",
  "holder": "did:wia:mainnet:0x7a3f9b2c4e8d1a5f",
  "verifiableCredential": [...],
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-26T11:00:00Z",
    "verificationMethod": "did:wia:mainnet:0x7a3f...#key-1",
    "proofPurpose": "authentication",
    "challenge": "1f44d",
    "domain": "example.com",
    "proofValue": "z3MvGcVxzRzzpKF..."
  }
}
```

### 5.2 Verify Presentation

**Endpoint:** `POST /presentations/verify`

**Request:**
```json
{
  "verifiablePresentation": {
    "@context": [...],
    "type": "VerifiablePresentation",
    "holder": "did:wia:mainnet:0x7a3f...",
    "verifiableCredential": [...],
    "proof": {...}
  },
  "challenge": "1f44d",
  "domain": "example.com"
}
```

**Response:**
```json
{
  "verified": true,
  "presentationVerified": true,
  "credentialsVerified": 1,
  "credentialChecks": [
    {
      "credentialId": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
      "verified": true,
      "issuer": "did:wia:mainnet:0x1234567890abcdef"
    }
  ],
  "holderVerified": true,
  "challengeMatched": true,
  "domainMatched": true
}
```

---

## 6. Zero-Knowledge Proof Operations

### 6.1 Create ZK Proof

**Endpoint:** `POST /proofs/create`

**Request:**
```json
{
  "credentialId": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
  "statement": {
    "type": "RangeProof",
    "claim": "age >= 18"
  },
  "reveal": [],
  "proofSystem": "groth16"
}
```

**Response:**
```json
{
  "zkProof": {
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
    }
  },
  "created": "2025-12-26T12:00:00Z"
}
```

### 6.2 Verify ZK Proof

**Endpoint:** `POST /proofs/verify`

**Request:**
```json
{
  "zkProof": {
    "type": "ZKProof",
    "proofSystem": "groth16",
    "curve": "bn254",
    "statement": {...},
    "proof": {...}
  },
  "verificationKey": {...}
}
```

**Response:**
```json
{
  "verified": true,
  "proofValid": true,
  "statementProven": "age >= 18",
  "timestamp": "2025-12-26T12:00:00Z"
}
```

---

## 7. SDK Reference

### 7.1 TypeScript SDK

```typescript
import { WiaDigitalID, VerifiableCredential, ZKProof } from 'wia-soc-002';

// Initialize client
const client = new WiaDigitalID({
  apiUrl: 'https://api.wiastandards.com/v1/identity',
  apiKey: 'your-api-key'
});

// Create DID
const did = await client.createDID({
  method: 'wia',
  network: 'mainnet',
  keyType: 'Ed25519VerificationKey2020'
});

// Issue credential
const credential = await client.issueCredential({
  type: ['VerifiableCredential', 'IdentityCredential'],
  issuer: did.id,
  credentialSubject: {
    id: 'did:wia:mainnet:0x7a3f...',
    name: 'John Doe'
  }
});

// Verify credential
const isValid = await client.verifyCredential(credential);

// Create ZK proof
const zkProof = await client.createZKProof({
  credentialId: credential.id,
  statement: { type: 'RangeProof', claim: 'age >= 18' }
});
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_DID",
    "message": "The provided DID format is invalid",
    "details": {
      "field": "did",
      "value": "invalid-did-format",
      "expected": "did:<method>:<method-specific-id>"
    },
    "timestamp": "2025-12-26T10:00:00Z",
    "requestId": "req-12345-67890"
  }
}
```

### 8.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_DID` | 400 | Invalid DID format |
| `DID_NOT_FOUND` | 404 | DID does not exist |
| `UNAUTHORIZED` | 401 | Invalid or missing authentication |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `CREDENTIAL_EXPIRED` | 400 | Credential has expired |
| `CREDENTIAL_REVOKED` | 400 | Credential has been revoked |
| `INVALID_SIGNATURE` | 400 | Signature verification failed |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Internal server error |

---

## 9. Rate Limiting

### 9.1 Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| Free | 60 | 1,000 |
| Basic | 300 | 10,000 |
| Pro | 1,000 | 100,000 |
| Enterprise | Custom | Custom |

### 9.2 Headers

```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1703592000
```

---

## 10. Webhooks

### 10.1 Register Webhook

**Endpoint:** `POST /webhooks/register`

**Request:**
```json
{
  "url": "https://example.com/webhook",
  "events": ["credential.issued", "credential.revoked", "did.updated"],
  "secret": "webhook-secret-key"
}
```

### 10.2 Webhook Payload

```json
{
  "event": "credential.issued",
  "timestamp": "2025-12-26T10:00:00Z",
  "data": {
    "credentialId": "urn:uuid:3978344f-8596-4c3a-a978-8fcaba3903c5",
    "issuer": "did:wia:mainnet:0x1234...",
    "subject": "did:wia:mainnet:0x7a3f..."
  },
  "signature": "sha256=5b7d8a..."
}
```

---

**Document Status:** Complete ✅
**Next Phase:** [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)

弘益人間 · Benefit All Humanity
