# WIA-FIN-010: Digital Identity Standard
## PHASE 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Official Standard  
**Last Updated:** 2025-12-25

---

## 1. Overview

This document specifies the API interfaces for the WIA Digital Identity Standard (WIA-FIN-010), including DID operations, credential issuance/verification, presentation flows, and wallet management.

---

## 2. DID Management API

### 2.1 Create DID

**Endpoint:** `POST /did/create`

**Request:**
```json
{
  "method": "ethr",
  "network": "mainnet",
  "keyType": "Ed25519",
  "serviceEndpoints": [
    {
      "type": "MessagingService",
      "endpoint": "https://example.com/messaging"
    }
  ]
}
```

**Response:**
```json
{
  "did": "did:ethr:0x3b0BC51Ab9De1e5B7B6E34E5b960285805C41736",
  "didDocument": { /* Full DID document */ },
  "keys": {
    "authentication": { /* key material */ },
    "keyAgreement": { /* key material */ }
  },
  "recoveryPhrase": "word1 word2 ... word12"
}
```

### 2.2 Resolve DID

**Endpoint:** `GET /did/resolve/{did}`

**Response:**
```json
{
  "didDocument": { /* DID document */ },
  "didDocumentMetadata": {
    "created": "2024-01-01T00:00:00Z",
    "updated": "2024-12-25T10:00:00Z",
    "deactivated": false
  }
}
```

### 2.3 Update DID Document

**Endpoint:** `PUT /did/{did}`

**Request:**
```json
{
  "action": "addKey",
  "keyId": "keys-3",
  "keyType": "Ed25519VerificationKey2020",
  "purpose": ["authentication"],
  "publicKeyMultibase": "z6Mk..."
}
```

---

## 3. Credential Issuance API

### 3.1 Issue Credential

**Endpoint:** `POST /credentials/issue`

**Request:**
```json
{
  "issuer": "did:example:university",
  "credentialSubject": {
    "id": "did:example:student",
    "degree": {
      "type": "BachelorDegree",
      "name": "Bachelor of Science"
    }
  },
  "type": ["VerifiableCredential", "UniversityDegreeCredential"],
  "expirationDate": "2028-12-31T23:59:59Z"
}
```

**Response:**
```json
{
  "verifiableCredential": { /* Full VC */ },
  "credentialId": "urn:uuid:12345...",
  "statusListEntry": {
    "statusListIndex": "94567",
    "statusListCredential": "https://example.com/status/1"
  }
}
```

### 3.2 Verify Credential

**Endpoint:** `POST /credentials/verify`

**Request:**
```json
{
  "verifiableCredential": { /* VC to verify */ },
  "options": {
    "checkStatus": true,
    "checkExpiration": true,
    "checkIssuerTrust": true
  }
}
```

**Response:**
```json
{
  "verified": true,
  "checks": {
    "signature": "valid",
    "expiration": "valid",
    "revocation": "not_revoked",
    "issuerTrust": "trusted"
  },
  "warnings": []
}
```

---

## 4. Presentation API

### 4.1 Create Presentation

**Endpoint:** `POST /presentations/create`

**Request:**
```json
{
  "holder": "did:example:holder",
  "verifiableCredentials": [
    { /* VC 1 */ },
    { /* VC 2 */ }
  ],
  "challenge": "nonce_from_verifier",
  "domain": "verifier.example.com"
}
```

**Response:**
```json
{
  "verifiablePresentation": { /* Full VP */ }
}
```

### 4.2 Verify Presentation

**Endpoint:** `POST /presentations/verify`

**Request:**
```json
{
  "verifiablePresentation": { /* VP to verify */ },
  "challenge": "expected_nonce",
  "domain": "verifier.example.com"
}
```

**Response:**
```json
{
  "verified": true,
  "holder": "did:example:holder",
  "credentials": [
    {"id": "urn:uuid:123", "verified": true},
    {"id": "urn:uuid:456", "verified": true}
  ]
}
```

---

## 5. Biometric API

### 5.1 Enroll Biometric

**Endpoint:** `POST /biometrics/enroll`

**Request:**
```json
{
  "did": "did:example:user",
  "modality": "facial",
  "biometricData": "base64_encoded_image",
  "livenessProof": { /* Liveness proof */ }
}
```

**Response:**
```json
{
  "templateId": "tmpl_123456",
  "quality": 0.95,
  "template": { /* Protected template */ }
}
```

### 5.2 Verify Biometric

**Endpoint:** `POST /biometrics/verify`

**Request:**
```json
{
  "did": "did:example:user",
  "modality": "facial",
  "biometricData": "base64_encoded_image",
  "livenessProof": { /* Liveness proof */ }
}
```

**Response:**
```json
{
  "verified": true,
  "confidence": 0.98,
  "matchScore": 0.95
}
```

---

## 6. Zero-Knowledge Proof API

### 6.1 Generate Age Proof

**Endpoint:** `POST /zkp/age/generate`

**Request:**
```json
{
  "birthdate": "1995-06-15",
  "minimumAge": 18,
  "credentialId": "urn:uuid:credential_with_birthdate"
}
```

**Response:**
```json
{
  "proof": { /* zk-SNARK proof */ },
  "claim": "age >= 18",
  "verified": true
}
```

### 6.2 Verify ZK Proof

**Endpoint:** `POST /zkp/verify`

**Request:**
```json
{
  "proof": { /* ZK proof */ },
  "publicInputs": {
    "minimumAge": 18
  }
}
```

**Response:**
```json
{
  "verified": true,
  "claim": "age >= 18"
}
```

---

## 7. eKYC API

### 7.1 Initiate KYC

**Endpoint:** `POST /kyc/initiate`

**Request:**
```json
{
  "did": "did:example:user",
  "level": 2,
  "documentType": "passport",
  "countryCode": "US"
}
```

**Response:**
```json
{
  "sessionId": "kyc_session_123",
  "uploadUrl": "https://api.example.com/kyc/upload/123",
  "expiresAt": "2024-12-25T12:00:00Z"
}
```

### 7.2 Submit KYC Documents

**Endpoint:** `POST /kyc/{sessionId}/documents`

**Request:**
```multipart/form-data
document: <file>
selfie: <file>
```

**Response:**
```json
{
  "status": "processing",
  "estimatedCompletion": "2024-12-25T10:15:00Z"
}
```

### 7.3 Get KYC Status

**Endpoint:** `GET /kyc/{sessionId}/status`

**Response:**
```json
{
  "status": "approved",
  "level": 2,
  "credential": { /* KYC VC */ },
  "verificationDetails": {
    "documentVerified": true,
    "biometricMatched": true,
    "livenessDetected": true,
    "sanctionsCleared": true
  }
}
```

---

## 8. Wallet API

### 8.1 Create Wallet

**Endpoint:** `POST /wallet/create`

**Request:**
```json
{
  "ownerDid": "did:example:owner",
  "backupMethod": "encrypted_cloud"
}
```

**Response:**
```json
{
  "walletId": "wallet_123",
  "recoveryPhrase": "word1 word2 ... word12",
  "backupUrl": "https://backup.example.com/wallet_123"
}
```

### 8.2 Store Credential

**Endpoint:** `POST /wallet/{walletId}/credentials`

**Request:**
```json
{
  "credential": { /* VC */ },
  "tags": ["education", "university", "degree"]
}
```

**Response:**
```json
{
  "credentialId": "cred_456",
  "stored": true
}
```

### 8.3 List Credentials

**Endpoint:** `GET /wallet/{walletId}/credentials`

**Query Parameters:**
- `type`: Filter by credential type
- `issuer`: Filter by issuer DID
- `tags`: Filter by tags

**Response:**
```json
{
  "credentials": [
    {
      "id": "cred_456",
      "type": ["VerifiableCredential", "UniversityDegreeCredential"],
      "issuer": "did:example:university",
      "issuanceDate": "2023-05-15T00:00:00Z",
      "tags": ["education"]
    }
  ],
  "total": 1
}
```

---

## 9. Authentication & Authorization

### 9.1 OAuth 2.0 with DID Authentication

**Token Endpoint:** `POST /oauth/token`

**Request:**
```json
{
  "grant_type": "did_authentication",
  "did": "did:example:user",
  "signature": "base64_encoded_signature",
  "challenge": "nonce_from_authorization_server"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGc...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "did": "did:example:user"
}
```

---

## 10. Error Responses

### Standard Error Format

```json
{
  "error": "invalid_credential",
  "error_description": "Credential signature verification failed",
  "error_code": "E1001",
  "timestamp": "2024-12-25T10:30:00Z"
}
```

### Error Codes

| Code | Error | Description |
|------|-------|-------------|
| E1001 | invalid_credential | Credential signature invalid |
| E1002 | expired_credential | Credential has expired |
| E1003 | revoked_credential | Credential has been revoked |
| E2001 | did_not_found | DID cannot be resolved |
| E2002 | invalid_did_format | DID syntax incorrect |
| E3001 | biometric_mismatch | Biometric verification failed |
| E3002 | liveness_failed | Liveness detection failed |
| E4001 | proof_verification_failed | ZK proof invalid |
| E5001 | kyc_rejected | KYC verification rejected |

---

## 11. Rate Limiting

- **DID Resolution:** 1000 requests/minute
- **Credential Issuance:** 100 requests/minute
- **Credential Verification:** 500 requests/minute
- **Biometric Operations:** 50 requests/minute

---

## 12. SDK Examples

### TypeScript SDK

```typescript
import { WIAIdentity } from '@wia/digital-identity';

const identity = new WIAIdentity({
  apiKey: 'your_api_key',
  baseUrl: 'https://api.identity.wia.live'
});

// Create DID
const did = await identity.createDID({
  method: 'ethr',
  network: 'mainnet'
});

// Issue credential
const credential = await identity.issueCredential({
  issuer: 'did:example:issuer',
  subject: 'did:example:subject',
  type: 'UniversityDegreeCredential',
  claims: { degree: 'Bachelor of Science' }
});

// Verify credential
const verified = await identity.verifyCredential(credential);
```

---

**Document Version:** 1.0.0  
**Status:** Official Standard  
© 2025 WIA - World Interoperability Alliance
