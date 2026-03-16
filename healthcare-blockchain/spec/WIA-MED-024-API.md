# WIA-MED-024: Healthcare Blockchain API Specification
## RESTful API & Smart Contract Interfaces v1.0.0

**Status:** Published
**Published:** 2025-01-26
**Organization:** WIA (World Certification Industry Association)

---

## 1. API Overview

The WIA-MED-024 API provides standardized interfaces for:
- Medical record management with blockchain verification
- Patient consent management
- Decentralized identity (DID) operations
- FHIR resource integration with blockchain
- Smart contract interactions

**Base URL:** `https://api.example.com/wia/med/v1`

**Authentication:** OAuth 2.0 + DID

---

## 2. Medical Record API

### 2.1 Create Medical Record

**Endpoint:** `POST /records`

**Request:**
```json
{
  "patientDID": "did:wia:med:0x9f8e...",
  "providerDID": "did:wia:med:provider:0xabc...",
  "fhirResource": {
    "resourceType": "Observation",
    "status": "final",
    "code": {
      "coding": [{
        "system": "http://loinc.org",
        "code": "85354-9",
        "display": "Blood pressure"
      }]
    },
    "valueQuantity": {
      "value": 120,
      "unit": "mmHg"
    }
  }
}
```

**Response:**
```json
{
  "recordId": "rec_abc123",
  "blockchainTx": "0xdef456...",
  "dataHash": "0x789abc...",
  "timestamp": "2025-01-26T10:30:00Z",
  "ipfsHash": "QmXyz...",
  "_links": {
    "self": "/records/rec_abc123",
    "blockchain": "https://etherscan.io/tx/0xdef456...",
    "fhir": "/fhir/Observation/rec_abc123"
  }
}
```

### 2.2 Get Medical Record

**Endpoint:** `GET /records/{recordId}`

**Headers:**
```
Authorization: Bearer {jwt_token}
X-Patient-Consent: {consent_proof}
```

**Response:**
```json
{
  "recordId": "rec_abc123",
  "patientDID": "did:wia:med:0x9f8e...",
  "fhirResource": { ... },
  "blockchain": {
    "transactionHash": "0xdef456...",
    "blockNumber": 12345678,
    "timestamp": "2025-01-26T10:30:00Z",
    "dataHash": "0x789abc...",
    "verified": true
  },
  "accessLog": [
    {
      "accessor": "did:wia:med:doctor:0x111...",
      "timestamp": "2025-01-26T14:00:00Z",
      "purpose": "treatment"
    }
  ]
}
```

### 2.3 Verify Record Integrity

**Endpoint:** `POST /records/{recordId}/verify`

**Response:**
```json
{
  "recordId": "rec_abc123",
  "integrity": "verified",
  "checks": {
    "hashMatch": true,
    "blockchainConfirmed": true,
    "timestampValid": true,
    "signatureValid": true
  },
  "verifiedAt": "2025-01-26T15:00:00Z"
}
```

---

## 3. Consent Management API

### 3.1 Grant Consent

**Endpoint:** `POST /consent`

**Request:**
```json
{
  "patientDID": "did:wia:med:0x9f8e...",
  "grantedTo": "did:wia:med:provider:hospital-a",
  "dataCategories": ["diagnosis", "lab_results", "medications"],
  "purpose": "treatment",
  "validFrom": "2025-01-26T00:00:00Z",
  "validUntil": "2025-04-26T23:59:59Z",
  "conditions": {
    "emergencyOverride": true,
    "thirdPartySharing": false,
    "researchUse": false
  }
}
```

**Response:**
```json
{
  "consentId": "consent_xyz789",
  "blockchainTx": "0xabc123...",
  "status": "active",
  "smartContractAddress": "0xdef456...",
  "createdAt": "2025-01-26T10:00:00Z"
}
```

### 3.2 Check Access Permission

**Endpoint:** `POST /consent/check`

**Request:**
```json
{
  "patientDID": "did:wia:med:0x9f8e...",
  "requesterDID": "did:wia:med:provider:hospital-a",
  "dataCategory": "lab_results",
  "purpose": "treatment"
}
```

**Response:**
```json
{
  "allowed": true,
  "consentId": "consent_xyz789",
  "expiresAt": "2025-04-26T23:59:59Z",
  "conditions": {
    "thirdPartySharing": false
  }
}
```

### 3.3 Revoke Consent

**Endpoint:** `DELETE /consent/{consentId}`

**Response:**
```json
{
  "consentId": "consent_xyz789",
  "status": "revoked",
  "blockchainTx": "0x999888...",
  "revokedAt": "2025-01-26T16:00:00Z"
}
```

---

## 4. Decentralized Identity (DID) API

### 4.1 Create DID

**Endpoint:** `POST /did`

**Request:**
```json
{
  "type": "patient",
  "publicKey": "0x04b97c30de767f084ce3080168ee293053ba33b235d7116a3263d29f1450936b71...",
  "biometricHash": "0xabc123..." (optional)
}
```

**Response:**
```json
{
  "did": "did:wia:med:ethereum:0x9f8e7d6c5b4a3210",
  "didDocument": {
    "@context": "https://www.w3.org/ns/did/v1",
    "id": "did:wia:med:ethereum:0x9f8e7d6c5b4a3210",
    "authentication": [{ ... }],
    "service": [{ ... }]
  },
  "blockchainTx": "0xdef456...",
  "recoveryKey": "0x777888..." (encrypted)
}
```

### 4.2 Resolve DID

**Endpoint:** `GET /did/{did}`

**Response:**
```json
{
  "didDocument": {
    "@context": "https://www.w3.org/ns/did/v1",
    "id": "did:wia:med:ethereum:0x9f8e...",
    "authentication": [{
      "id": "did:wia:med:ethereum:0x9f8e...#keys-1",
      "type": "EcdsaSecp256k1VerificationKey2019",
      "controller": "did:wia:med:ethereum:0x9f8e...",
      "publicKeyHex": "0x04b97c..."
    }],
    "service": [{
      "id": "did:wia:med:ethereum:0x9f8e...#healthData",
      "type": "HealthDataService",
      "serviceEndpoint": "https://vault.example.com"
    }]
  },
  "metadata": {
    "created": "2025-01-15T00:00:00Z",
    "updated": "2025-01-26T10:00:00Z"
  }
}
```

### 4.3 Issue Verifiable Credential

**Endpoint:** `POST /credentials/issue`

**Request:**
```json
{
  "issuerDID": "did:wia:med:provider:hospital-a",
  "subjectDID": "did:wia:med:0x9f8e...",
  "type": "VaccinationCredential",
  "claims": {
    "vaccineName": "Pfizer-BioNTech COVID-19",
    "doseNumber": 2,
    "vaccinationDate": "2025-01-15",
    "lotNumber": "EK9231"
  }
}
```

**Response:**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "VaccinationCredential"],
    "issuer": "did:wia:med:provider:hospital-a",
    "issuanceDate": "2025-01-26T10:00:00Z",
    "credentialSubject": {
      "id": "did:wia:med:0x9f8e...",
      "vaccineName": "Pfizer-BioNTech COVID-19",
      "doseNumber": 2,
      "vaccinationDate": "2025-01-15"
    },
    "proof": {
      "type": "EcdsaSecp256k1Signature2019",
      "created": "2025-01-26T10:00:00Z",
      "proofPurpose": "assertionMethod",
      "verificationMethod": "did:wia:med:provider:hospital-a#keys-1",
      "jws": "eyJhbGc..."
    }
  },
  "qrCode": "data:image/png;base64,iVBORw0KG..." (optional)
}
```

---

## 5. FHIR Integration API

### 5.1 FHIR Read with Blockchain Verification

**Endpoint:** `GET /fhir/{resourceType}/{id}`

**Example:** `GET /fhir/Patient/did:wia:med:0x9f8e...`

**Response:**
```json
{
  "resourceType": "Patient",
  "id": "did:wia:med:0x9f8e...",
  "identifier": [{
    "system": "blockchain:ethereum",
    "value": "0x9f8e7d6c5b4a3210"
  }],
  "name": [{
    "family": "Smith",
    "given": ["John"]
  }],
  "gender": "male",
  "birthDate": "1980-01-15",
  "extension": [{
    "url": "http://wia.org/fhir/blockchain",
    "extension": [
      {
        "url": "did",
        "valueString": "did:wia:med:0x9f8e..."
      },
      {
        "url": "blockchainTx",
        "valueString": "0xabc123..."
      },
      {
        "url": "dataHash",
        "valueString": "0xdef456..."
      },
      {
        "url": "verified",
        "valueBoolean": true
      }
    ]
  }]
}
```

### 5.2 FHIR Search with Consent Filtering

**Endpoint:** `GET /fhir/Observation?patient={patientId}&requester={requesterDID}`

**Headers:**
```
Authorization: Bearer {jwt_token}
X-Consent-Check: true
```

**Response:** Standard FHIR Bundle with only consented data

---

## 6. Smart Contract Interfaces

### 6.1 Patient Consent Contract

**Solidity Interface:**
```solidity
interface IPatientConsent {
    function grantConsent(
        address provider,
        string[] memory dataCategories,
        uint256 validUntil
    ) external returns (bytes32 consentId);

    function revokeConsent(bytes32 consentId) external;

    function checkAccess(
        address patient,
        address provider,
        string memory dataCategory
    ) external view returns (bool allowed);

    function getConsentDetails(bytes32 consentId)
        external
        view
        returns (
            address patient,
            address provider,
            string[] memory dataCategories,
            uint256 validUntil,
            bool isActive
        );

    event ConsentGranted(
        bytes32 indexed consentId,
        address indexed patient,
        address indexed provider,
        uint256 validUntil
    );

    event ConsentRevoked(
        bytes32 indexed consentId,
        address indexed patient,
        uint256 revokedAt
    );
}
```

### 6.2 Medical Record Registry Contract

**Solidity Interface:**
```solidity
interface IMedicalRecordRegistry {
    function registerRecord(
        bytes32 recordHash,
        string memory ipfsHash,
        address patient
    ) external returns (uint256 recordId);

    function verifyRecord(uint256 recordId, bytes32 computedHash)
        external
        view
        returns (bool verified);

    function getRecordMetadata(uint256 recordId)
        external
        view
        returns (
            bytes32 recordHash,
            string memory ipfsHash,
            address patient,
            uint256 timestamp,
            bool exists
        );

    event RecordRegistered(
        uint256 indexed recordId,
        bytes32 indexed recordHash,
        address indexed patient,
        uint256 timestamp
    );
}
```

### 6.3 Supply Chain Contract

**Solidity Interface:**
```solidity
interface IDrugSupplyChain {
    function manufactureDrug(
        string memory gtin,
        string memory batchNumber,
        uint256 quantity
    ) external returns (bytes32 shipmentId);

    function transferOwnership(
        bytes32 shipmentId,
        address newOwner,
        string memory location
    ) external;

    function recordTemperature(
        bytes32 shipmentId,
        int256 temperature,
        uint256 humidity
    ) external;

    function verifyAuthenticity(bytes32 shipmentId)
        external
        view
        returns (bool authentic, address[] memory chain);

    event DrugManufactured(
        bytes32 indexed shipmentId,
        string gtin,
        address indexed manufacturer
    );

    event OwnershipTransferred(
        bytes32 indexed shipmentId,
        address indexed from,
        address indexed to,
        string location,
        uint256 timestamp
    );

    event TemperatureAlert(
        bytes32 indexed shipmentId,
        int256 temperature,
        uint256 timestamp
    );
}
```

---

## 7. Webhook Events

### 7.1 Subscribe to Events

**Endpoint:** `POST /webhooks`

**Request:**
```json
{
  "url": "https://your-server.com/webhook",
  "events": ["consent.granted", "consent.revoked", "record.created"],
  "secret": "your_webhook_secret"
}
```

### 7.2 Event Payload Example

**Event:** `consent.granted`

```json
{
  "event": "consent.granted",
  "timestamp": "2025-01-26T10:00:00Z",
  "data": {
    "consentId": "consent_xyz789",
    "patientDID": "did:wia:med:0x9f8e...",
    "grantedTo": "did:wia:med:provider:hospital-a",
    "dataCategories": ["diagnosis", "lab_results"]
  },
  "signature": "sha256_hmac_signature"
}
```

---

## 8. Error Responses

### Standard Error Format

```json
{
  "error": {
    "code": "CONSENT_DENIED",
    "message": "Patient has not granted consent for this data access",
    "details": {
      "patientDID": "did:wia:med:0x9f8e...",
      "requesterDID": "did:wia:med:provider:hospital-b",
      "dataCategory": "diagnosis"
    },
    "timestamp": "2025-01-26T10:00:00Z",
    "requestId": "req_abc123"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| CONSENT_DENIED | 403 | No active consent found |
| INVALID_DID | 400 | DID format invalid |
| BLOCKCHAIN_ERROR | 500 | Blockchain transaction failed |
| RECORD_NOT_FOUND | 404 | Medical record not found |
| HASH_MISMATCH | 409 | Data integrity check failed |
| EXPIRED_CONSENT | 403 | Consent has expired |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |

---

## 9. Rate Limits

| Endpoint | Rate Limit |
|----------|------------|
| `/records/*` | 100 req/min |
| `/consent/*` | 50 req/min |
| `/did/*` | 20 req/min |
| `/fhir/*` | 200 req/min |

**Headers:**
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 87
X-RateLimit-Reset: 1706270400
```

---

## 10. SDK Examples

### JavaScript/TypeScript

```typescript
import { WIAHealthchain } from '@wia/med-024-sdk';

const client = new WIAHealthchain({
  apiUrl: 'https://api.example.com/wia/med/v1',
  blockchain: 'ethereum',
  privateKey: process.env.PRIVATE_KEY
});

// Create medical record
const record = await client.records.create({
  patientDID: 'did:wia:med:0x9f8e...',
  fhirResource: observationData
});

// Grant consent
const consent = await client.consent.grant({
  grantedTo: 'did:wia:med:provider:hospital-a',
  dataCategories: ['diagnosis', 'lab_results'],
  validUntil: '2025-04-26T23:59:59Z'
});

// Verify record integrity
const verification = await client.records.verify(record.recordId);
console.log('Verified:', verification.integrity === 'verified');
```

---

## 11. Testing & Sandbox

**Sandbox URL:** `https://sandbox-api.wiastandards.com/wia/med/v1`

**Test DIDs:**
- Patient: `did:wia:med:test:patient-001`
- Provider: `did:wia:med:test:provider-001`

**Test Blockchain:** Ethereum Goerli Testnet

---

**Document Version:** 1.0.0
**Last Updated:** 2025-01-26

**© 2025 SmileStory Inc. / WIA · MIT License**
**弘益人間 · Benefit All Humanity**
