# Phase 3: Pet Health Passport Communication Protocol Specification

## WIA-PET-HEALTH-PASSPORT Protocol Standard

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Standard ID**: WIA-PET-HEALTH-PASSPORT-PHASE3-001

---

## 1. 개요

WIA-PET-HEALTH-PASSPORT 프로토콜은 수의사, 검역소, 보호소, 반려동물 보호자 간의
안전하고 표준화된 건강 기록 교환을 정의합니다.

### 1.1 통신 구조

```
┌────────────────────────────────────────────────────────────────────┐
│                 WIA-PET-HEALTH-PASSPORT 통신 구조                  │
├────────────────────────────────────────────────────────────────────┤
│                                                                    │
│  [보호자 앱] ←→ [WIA Gateway] ←→ [수의사 시스템]                   │
│       ↓              ↓               ↓                            │
│  [마이크로칩     [분산 저장소]    [검역 시스템]                     │
│   리더기]            ↓               ↓                            │
│       ↓         [블록체인]      [정부 DB]                         │
│  [QR 스캐너]         ↓               ↓                            │
│                 [a11y.wiabooks.store]                             │
│                    ↓                                              │
│              [211개 언어 지원]                                     │
│                                                                    │
└────────────────────────────────────────────────────────────────────┘
```

### 1.2 프로토콜 레이어

| 레이어 | 프로토콜 | 역할 |
|--------|----------|------|
| Application | WIA-PET Protocol | 데이터 교환 |
| Security | WIA-PQ-CRYPTO | 양자 내성 암호화 |
| Session | OAuth 2.0 / DID | 인증/인가 |
| Transport | HTTPS / WebSocket | 전송 |
| Physical | Internet / NFC / BLE | 하드웨어 |

---

## 2. REST API Specification

### 2.1 Base URL

```
Production: https://api.pet.wia.world/v1
Staging:    https://api.pet.staging.wia.world/v1
```

### 2.2 인증

```http
Authorization: Bearer <WIA_ACCESS_TOKEN>
X-WIA-DID: did:wia:pet:guardian:1234567890
```

### 2.3 Passport Endpoints

#### 2.3.1 Create Passport

```http
POST /passports
Content-Type: application/json

Request:
{
  "identity": {
    "species": "dog",
    "breed": "Golden Retriever",
    "name": "Max",
    "dateOfBirth": "2022-03-15",
    "sex": "neutered_male",
    "color": "Golden",
    "markings": ["White chest patch"]
  },
  "microchip": {
    "chipNumber": "985141234567890",
    "implantedAt": "2022-05-01",
    "implantLocation": "left_neck"
  },
  "guardian": {
    "name": "홍길동",
    "contactPhone": "+82-10-1234-5678",
    "contactEmail": "hong@example.com",
    "address": {
      "country": "KR",
      "postalCode": "06164",
      "city": "Seoul",
      "addressLine1": "123 Gangnam-daero"
    }
  }
}

Response: 201 Created
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "identity": { ... },
  "microchip": { ... },
  "guardian": { ... },
  "qrCode": "data:image/png;base64,...",
  "createdAt": "2025-12-16T10:00:00Z"
}
```

#### 2.3.2 Get Passport

```http
GET /passports/{passportId}

Response: 200 OK
{
  "header": {
    "magic": "WIAPET",
    "version": [1, 0, 0],
    "standard": "WIA-PET-HEALTH-PASSPORT",
    "createdAt": "2025-12-16T10:00:00Z",
    "lastUpdated": "2025-12-16T15:30:00Z"
  },
  "identity": { ... },
  "microchip": { ... },
  "guardian": { ... },
  "medicalRecords": {
    "vaccinations": [...],
    "surgeries": [...],
    "diagnostics": [...],
    "allergies": [...],
    "conditions": [...],
    "medications": [...]
  },
  "genetics": { ... },
  "travel": { ... },
  "verification": {
    "issuingAuthority": "wia:authority:kr:mafra",
    "digitalSignature": "...",
    "certificateChain": [...]
  }
}
```

#### 2.3.3 Get Passport by Microchip

```http
GET /passports/microchip/{chipNumber}

Response: 200 OK
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "identity": { ... },
  ...
}
```

#### 2.3.4 Get Passport by QR Code

```http
POST /passports/qr
Content-Type: application/json

Request:
{
  "qrData": "WIAPET01H5XNQK..."
}

Response: 200 OK
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "mini": {
    "petName": "Max",
    "species": "dog",
    "rabiesValid": true,
    "rabiesExpiry": "2026-03-15",
    "emergencyPhone": "+82-10-1234-5678"
  },
  "fullDataUrl": "https://api.pet.wia.world/v1/passports/01H5XNQK..."
}
```

### 2.4 Medical Records Endpoints

#### 2.4.1 Add Vaccination

```http
POST /passports/{passportId}/vaccinations
Content-Type: application/json
X-WIA-Veterinarian-License: KR-VET-12345

Request:
{
  "vaccine": {
    "name": "Nobivac Rabies",
    "manufacturer": "MSD Animal Health",
    "lotNumber": "A123456",
    "serialNumber": "SN789012"
  },
  "targetDiseases": ["RABIES"],
  "administeredAt": "2025-12-16T10:00:00Z",
  "validFrom": "2025-12-16T10:00:00Z",
  "validUntil": "2026-12-16T10:00:00Z",
  "doseNumber": 1,
  "totalDoses": 1,
  "clinic": {
    "name": "Happy Pet Clinic",
    "licenseNumber": "KR-CLINIC-001",
    "address": "Seoul, Korea"
  }
}

Response: 201 Created
{
  "recordId": "vax_01H5XP123456",
  "vaccine": { ... },
  "administeredAt": "2025-12-16T10:00:00Z",
  "verified": true,
  "digitalSignature": "..."
}
```

#### 2.4.2 Get Vaccination History

```http
GET /passports/{passportId}/vaccinations

Query Parameters:
  - disease: Filter by disease (e.g., RABIES)
  - from: Start date
  - to: End date
  - valid_only: Only return valid vaccinations (true/false)

Response: 200 OK
{
  "vaccinations": [
    {
      "recordId": "vax_01H5XP123456",
      "vaccine": { ... },
      "targetDiseases": ["RABIES"],
      "administeredAt": "2025-12-16T10:00:00Z",
      "validUntil": "2026-12-16T10:00:00Z",
      "verified": true
    }
  ],
  "summary": {
    "total": 5,
    "valid": 4,
    "expired": 1
  }
}
```

#### 2.4.3 Add Allergy

```http
POST /passports/{passportId}/allergies
Content-Type: application/json

Request:
{
  "allergen": "Chicken",
  "allergenType": "FOOD",
  "reactionSeverity": "moderate",
  "symptoms": ["Itching", "Ear inflammation", "Digestive upset"],
  "diagnosedAt": "2024-06-15T00:00:00Z",
  "diagnosisMethod": "elimination_diet",
  "avoidanceInstructions": "Avoid all chicken-based foods and treats",
  "status": "active"
}

Response: 201 Created
{
  "recordId": "allergy_01H5XQ789012",
  ...
}
```

### 2.5 Quarantine Endpoints

#### 2.5.1 Check Travel Eligibility

```http
POST /quarantine/check
Content-Type: application/json

Request:
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "departureCountry": "KR",
  "arrivalCountry": "DE",
  "travelDate": "2025-06-01"
}

Response: 200 OK
{
  "eligible": true,
  "overallScore": 95,
  "requirements": [
    {
      "requirement": "Microchip",
      "status": "passed",
      "details": "ISO 11784/11785 compliant (985141234567890)"
    },
    {
      "requirement": "Rabies Vaccination",
      "status": "passed",
      "details": "Valid until 2026-12-16"
    },
    {
      "requirement": "Rabies Titer Test",
      "status": "passed",
      "details": "0.7 IU/ml (minimum: 0.5 IU/ml)"
    },
    {
      "requirement": "Tapeworm Treatment",
      "status": "pending",
      "details": "Required 24-120 hours before arrival",
      "actionRequired": "Get treatment from veterinarian",
      "deadline": "2025-05-31T00:00:00Z"
    }
  ],
  "missingItems": [],
  "recommendations": [
    "Schedule tapeworm treatment 1-5 days before departure"
  ],
  "estimatedProcessingDays": 0
}
```

#### 2.5.2 Get Country Requirements

```http
GET /quarantine/requirements/{countryCode}

Query Parameters:
  - species: dog, cat, etc.
  - fromCountry: Origin country

Response: 200 OK
{
  "country": "DE",
  "lastUpdated": "2025-12-01T00:00:00Z",
  "requirements": {
    "microchipRequired": true,
    "microchipStandard": "ISO11784",
    "rabiesVaccination": {
      "required": true,
      "minimumAge": 3,
      "waitPeriod": 21,
      "validityPeriod": 12,
      "titerTestRequired": true,
      "minimumTiter": 0.5
    },
    "otherVaccinations": [],
    "parasiteTreatment": {
      "type": "tapeworm",
      "required": true,
      "timingBeforeEntry": 120
    },
    "quarantine": null,
    "bannedBreeds": [],
    "importPermitRequired": false,
    "healthCertificateRequired": true,
    "healthCertificateValidDays": 10
  },
  "exemptCountries": ["FR", "NL", "BE", "AT"]
}
```

#### 2.5.3 Generate Health Certificate

```http
POST /quarantine/certificate
Content-Type: application/json

Request:
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "departureCountry": "KR",
  "arrivalCountry": "DE",
  "travelDate": "2025-06-01",
  "veterinarianLicense": "KR-VET-12345",
  "format": "PDF"
}

Response: 200 OK
{
  "certificateId": "cert_01H5XR345678",
  "issuedAt": "2025-12-16T12:00:00Z",
  "validUntil": "2025-12-26T12:00:00Z",
  "format": "PDF",
  "downloadUrl": "https://api.pet.wia.world/v1/certificates/cert_01H5XR345678/download",
  "digitalSignature": "...",
  "qrCode": "data:image/png;base64,..."
}
```

### 2.6 Emergency Endpoints

#### 2.6.1 Get Emergency Info

```http
GET /passports/{passportId}/emergency

Response: 200 OK
{
  "criticalAlerts": [
    {
      "type": "allergy",
      "severity": "high",
      "message": "ALLERGY: Penicillin - Severe reaction",
      "action": "Use alternative antibiotics"
    }
  ],
  "pet": {
    "name": "Max",
    "species": "dog",
    "breed": "Golden Retriever",
    "age": "3 years 9 months",
    "weight": 30.5,
    "sex": "Neutered Male"
  },
  "medical": {
    "severeAllergies": ["Penicillin"],
    "currentMedications": [
      {
        "name": "Apoquel",
        "dosage": "16mg BID",
        "route": "oral",
        "indication": "Allergic dermatitis"
      }
    ],
    "activeConditions": ["Allergic dermatitis"],
    "recentSurgeries": []
  },
  "contacts": [
    {
      "name": "홍길동",
      "relationship": "Owner",
      "phone": "+82-10-1234-5678"
    }
  ],
  "veterinaryNotes": [
    "Rabies: Current (until 2026-12-16)",
    "No known anesthesia complications"
  ]
}
```

#### 2.6.2 Emergency Access (No Auth)

```http
POST /emergency/access
Content-Type: application/json

Request:
{
  "microchipId": "985141234567890",
  "veterinarianLicense": "KR-VET-12345",
  "reason": "Emergency surgery",
  "location": {
    "lat": 37.5665,
    "lng": 126.9780
  }
}

Response: 200 OK
{
  "accessGranted": true,
  "accessToken": "emergency_01H5XS...",
  "expiresIn": 3600,
  "petInfo": {
    "name": "Max",
    "species": "dog",
    "criticalInfo": { ... }
  },
  "guardianNotified": true
}
```

### 2.7 Verification Endpoints

#### 2.7.1 Verify Passport

```http
POST /verify/passport
Content-Type: application/json

Request:
{
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW"
}

Response: 200 OK
{
  "valid": true,
  "signatureValid": true,
  "chainValid": true,
  "issuerTrusted": true,
  "issuer": {
    "id": "wia:authority:kr:mafra",
    "name": "Ministry of Agriculture, Food and Rural Affairs",
    "country": "KR"
  },
  "recordsVerified": 12,
  "recordsFailed": 0,
  "details": [
    {
      "component": "Passport Signature",
      "status": "valid",
      "message": "Digital signature verified"
    },
    {
      "component": "Certificate Chain",
      "status": "valid",
      "message": "Certificate chain valid"
    }
  ],
  "warnings": []
}
```

#### 2.7.2 Verify Vaccination Record

```http
POST /verify/vaccination
Content-Type: application/json

Request:
{
  "recordId": "vax_01H5XP123456",
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW"
}

Response: 200 OK
{
  "valid": true,
  "clinicVerified": true,
  "veterinarianVerified": true,
  "vaccineVerified": true,
  "blockchainAnchor": {
    "network": "wia-chain",
    "txHash": "0x1234...",
    "blockNumber": 12345678,
    "timestamp": "2025-12-16T10:01:00Z"
  }
}
```

---

## 3. WebSocket API

### 3.1 Connection

```javascript
const ws = new WebSocket('wss://api.pet.wia.world/v1/ws');

// 인증
ws.send(JSON.stringify({
  type: 'auth',
  token: 'YOUR_ACCESS_TOKEN'
}));
```

### 3.2 Message Types

#### 3.2.1 Subscribe to Passport Updates

```json
// Client → Server
{
  "type": "subscribe",
  "channel": "passport",
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW"
}

// Server → Client (on update)
{
  "type": "passport_updated",
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "updateType": "vaccination_added",
  "data": {
    "recordId": "vax_01H5XT456789",
    "vaccine": "Rabies",
    "administeredAt": "2025-12-16T14:00:00Z"
  },
  "timestamp": "2025-12-16T14:00:05Z"
}
```

#### 3.2.2 Real-time Location Tracking (Travel Mode)

```json
// Client → Server
{
  "type": "travel_mode",
  "action": "start",
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "journey": {
    "departure": "KR",
    "arrival": "DE",
    "estimatedArrival": "2025-06-01T10:00:00Z"
  }
}

// Server → Client (status updates)
{
  "type": "travel_status",
  "status": "in_transit",
  "currentLocation": {
    "country": "transit",
    "description": "In flight"
  },
  "nextCheckpoint": "DE border control",
  "estimatedArrival": "2025-06-01T10:00:00Z"
}
```

#### 3.2.3 Emergency Alert

```json
// Server → Client (emergency access notification)
{
  "type": "emergency_access",
  "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
  "accessedBy": {
    "type": "veterinarian",
    "license": "KR-VET-12345",
    "clinic": "Emergency Animal Hospital"
  },
  "reason": "Emergency surgery",
  "location": {
    "lat": 37.5665,
    "lng": 126.9780,
    "address": "Seoul, Korea"
  },
  "timestamp": "2025-12-16T15:00:00Z"
}
```

---

## 4. NFC/RFID Protocol

### 4.1 Microchip Reading

```typescript
interface MicrochipReadProtocol {
  // ISO 11784/11785 표준
  frequency: 134.2;              // kHz (FDX-B)
  modulation: 'FSK';

  // 데이터 구조
  data: {
    chipNumber: string;          // 15자리
    countryCode: string;         // 3자리
    manufacturer: string;
  };

  // 읽기 프로토콜
  readSequence: [
    'activate_field',            // RF 필드 활성화
    'detect_transponder',        // 트랜스폰더 감지
    'read_id',                   // ID 읽기
    'verify_checksum',           // 체크섬 검증
    'deactivate_field'           // RF 필드 비활성화
  ];
}
```

### 4.2 NFC Pet Tag (확장)

```typescript
interface NFCPetTag {
  // NDEF 메시지
  ndefMessage: {
    // Record 1: 여권 URL
    record1: {
      type: 'U';                 // URI
      payload: 'https://pet.wia.world/p/01H5XNQK...'
    };

    // Record 2: 응급 정보 (오프라인)
    record2: {
      type: 'T';                 // Text
      payload: {
        name: 'Max',
        species: 'dog',
        allergies: ['Penicillin'],
        emergencyPhone: '+82-10-1234-5678'
      }
    };

    // Record 3: 서명
    record3: {
      type: 'application/wia-signature';
      payload: '...'
    };
  };
}
```

---

## 5. Error Handling

### 5.1 Error Codes

```typescript
enum PetPassportErrorCode {
  // 인증 에러 (1xxx)
  UNAUTHORIZED = 1001,
  TOKEN_EXPIRED = 1002,
  INSUFFICIENT_PERMISSIONS = 1003,

  // 여권 에러 (2xxx)
  PASSPORT_NOT_FOUND = 2001,
  PASSPORT_ALREADY_EXISTS = 2002,
  PASSPORT_INVALID = 2003,
  MICROCHIP_NOT_FOUND = 2004,
  MICROCHIP_ALREADY_REGISTERED = 2005,

  // 의료 기록 에러 (3xxx)
  RECORD_NOT_FOUND = 3001,
  INVALID_VACCINATION = 3002,
  DUPLICATE_RECORD = 3003,
  VETERINARIAN_NOT_VERIFIED = 3004,

  // 검역 에러 (4xxx)
  COUNTRY_NOT_SUPPORTED = 4001,
  REQUIREMENTS_NOT_MET = 4002,
  BREED_BANNED = 4003,
  CERTIFICATE_EXPIRED = 4004,

  // 검증 에러 (5xxx)
  SIGNATURE_INVALID = 5001,
  CERTIFICATE_CHAIN_INVALID = 5002,
  ISSUER_NOT_TRUSTED = 5003,

  // 서버 에러 (9xxx)
  INTERNAL_ERROR = 9001,
  SERVICE_UNAVAILABLE = 9002,
  RATE_LIMIT_EXCEEDED = 9003
}
```

### 5.2 Error Response Format

```json
{
  "error": {
    "code": 2001,
    "message": "Passport not found",
    "details": "No passport exists with ID: 01H5XNQK...",
    "suggestions": [
      "Verify the passport ID is correct",
      "The passport may have been deleted",
      "Try searching by microchip number instead"
    ],
    "documentationUrl": "https://docs.pet.wia.world/errors/2001"
  },
  "requestId": "req_01H5XU123456",
  "timestamp": "2025-12-16T12:00:00Z"
}
```

### 5.3 Retry Strategy

```typescript
interface RetryConfig {
  maxRetries: 3;
  backoffMs: [1000, 2000, 4000];  // 지수 백오프
  retryableErrors: [
    9001,  // Internal Error
    9002,  // Service Unavailable
    9003   // Rate Limit Exceeded
  ];
}
```

---

## 6. Rate Limiting

### 6.1 Rate Limits

| Endpoint Type | Limit | Window |
|--------------|-------|--------|
| Read (GET) | 1000 req | 1 minute |
| Write (POST/PUT) | 100 req | 1 minute |
| Emergency | Unlimited | - |
| Verification | 500 req | 1 minute |
| Certificate Gen | 10 req | 1 hour |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1702728000
X-RateLimit-RetryAfter: 60
```

---

## 7. Security

### 7.1 Authentication

```typescript
interface AuthenticationMethods {
  // OAuth 2.0 + PKCE
  oauth2: {
    authorizationEndpoint: 'https://auth.wia.world/oauth/authorize';
    tokenEndpoint: 'https://auth.wia.world/oauth/token';
    scopes: [
      'passport:read',
      'passport:write',
      'medical:read',
      'medical:write',
      'emergency:access',
      'quarantine:check'
    ];
  };

  // DID 인증 (분산 신원)
  did: {
    method: 'did:wia';
    resolver: 'https://resolver.wia.world';
  };

  // 수의사 라이선스 인증
  veterinarian: {
    licenseVerification: 'https://verify.wia.world/veterinarian';
    requiredForOperations: ['add_vaccination', 'add_surgery', 'issue_certificate'];
  };
}
```

### 7.2 Encryption

```typescript
interface EncryptionConfig {
  // 전송 중 암호화
  transport: {
    protocol: 'TLS 1.3';
    cipherSuites: [
      'TLS_AES_256_GCM_SHA384',
      'TLS_CHACHA20_POLY1305_SHA256'
    ];
  };

  // 저장 시 암호화
  atRest: {
    algorithm: 'AES-256-GCM';
    keyManagement: 'WIA-KMS';
  };

  // 양자 내성 암호화 (WIA-PQ-CRYPTO)
  postQuantum: {
    keyEncapsulation: 'CRYSTALS-Kyber';
    signature: 'CRYSTALS-Dilithium';
  };
}
```

### 7.3 Access Control

```typescript
interface AccessControlPolicy {
  // 역할 기반 접근 제어
  roles: {
    guardian: {
      permissions: ['read:own', 'write:own', 'share'];
      restrictions: ['cannot_delete_verified_records'];
    };

    veterinarian: {
      permissions: ['read:all', 'write:medical', 'verify'];
      requires: ['valid_license', 'clinic_registration'];
    };

    quarantine_officer: {
      permissions: ['read:travel', 'verify', 'approve_entry'];
      requires: ['government_credential'];
    };

    shelter: {
      permissions: ['read:basic', 'transfer:ownership'];
      requires: ['shelter_registration'];
    };

    emergency_responder: {
      permissions: ['read:emergency'];
      requires: ['emergency_credentials', 'location_verification'];
    };
  };

  // 속성 기반 접근 제어
  abac: {
    rules: [
      {
        resource: 'genetic_data',
        action: 'read',
        conditions: [
          'subject.role == guardian AND resource.owner == subject.id',
          'subject.role == veterinarian AND subject.treating == true'
        ]
      }
    ];
  };
}
```

### 7.4 Privacy Protection

```typescript
interface PrivacyConfig {
  // 데이터 최소화
  dataMinimization: {
    emergencyAccess: ['name', 'species', 'allergies', 'medications', 'emergency_contact'];
    publicProfile: ['species', 'breed', 'microchip_prefix'];
    quarantineCheck: ['vaccination_status', 'travel_documents'];
  };

  // 익명화
  anonymization: {
    guardianInfo: 'hash_after_transfer';
    locationData: 'coarse_location_only';
    medicalHistory: 'aggregate_after_5_years';
  };

  // 동의 관리
  consent: {
    required: ['data_sharing', 'location_tracking', 'analytics'];
    optional: ['research_participation', 'breed_database'];
    withdrawable: true;
  };

  // 데이터 보존
  retention: {
    activeRecords: 'indefinite';
    deletedPassports: '30_days';
    accessLogs: '1_year';
    analytics: '5_years_anonymized';
  };
}
```

---

## 8. Webhook Events

### 8.1 Event Types

```typescript
enum WebhookEventType {
  // 여권 이벤트
  PASSPORT_CREATED = 'passport.created',
  PASSPORT_UPDATED = 'passport.updated',
  PASSPORT_TRANSFERRED = 'passport.transferred',

  // 의료 이벤트
  VACCINATION_ADDED = 'vaccination.added',
  VACCINATION_EXPIRING = 'vaccination.expiring',
  ALLERGY_ADDED = 'allergy.added',
  CONDITION_UPDATED = 'condition.updated',

  // 검역 이벤트
  TRAVEL_APPROVED = 'travel.approved',
  TRAVEL_DENIED = 'travel.denied',
  CERTIFICATE_ISSUED = 'certificate.issued',

  // 응급 이벤트
  EMERGENCY_ACCESS = 'emergency.access',

  // 검증 이벤트
  VERIFICATION_FAILED = 'verification.failed'
}
```

### 8.2 Webhook Payload

```json
{
  "id": "evt_01H5XV789012",
  "type": "vaccination.added",
  "apiVersion": "2025-12-16",
  "created": "2025-12-16T14:00:00Z",
  "data": {
    "passportId": "01H5XNQK4PJXV8RQFY7D6KTHNW",
    "recordId": "vax_01H5XT456789",
    "vaccine": "Rabies",
    "administeredAt": "2025-12-16T14:00:00Z",
    "validUntil": "2026-12-16T14:00:00Z"
  }
}
```

### 8.3 Webhook Signature

```typescript
// HMAC-SHA256 서명 검증
function verifyWebhookSignature(
  payload: string,
  signature: string,
  secret: string
): boolean {
  const expectedSignature = crypto
    .createHmac('sha256', secret)
    .update(payload)
    .digest('hex');

  return crypto.timingSafeEqual(
    Buffer.from(signature),
    Buffer.from(`sha256=${expectedSignature}`)
  );
}
```

---

## 9. Internationalization

### 9.1 Language Support

```http
Accept-Language: ko-KR, ko;q=0.9, en;q=0.8
```

### 9.2 a11y.wiabooks.store Integration

```typescript
interface AccessibilityIntegration {
  // 211개 언어 지원
  languages: 211;

  // 수어 비디오
  signLanguage: {
    supported: ['KSL', 'ASL', 'BSL', 'JSL', /* ... */];
    contentTypes: ['emergency_info', 'instructions', 'alerts'];
  };

  // 접근성 형식
  formats: {
    screenReader: true;
    braille: true;
    largeText: true;
    highContrast: true;
  };
}
```

### 9.3 Localized Response Example

```json
{
  "message": "백신 기록이 추가되었습니다",
  "message_en": "Vaccination record added",
  "details": {
    "vaccine": {
      "name": "광견병 백신",
      "name_en": "Rabies Vaccine"
    }
  }
}
```

---

## 10. SDK Examples

### 10.1 JavaScript/TypeScript

```typescript
import { WIAPetPassport } from '@wia/pet-passport';

const client = new WIAPetPassport({
  apiKey: 'your_api_key',
  environment: 'production'
});

// 여권 조회
const passport = await client.passports.get('01H5XNQK...');

// 검역 체크
const eligibility = await client.quarantine.check({
  passportId: passport.id,
  departureCountry: 'KR',
  arrivalCountry: 'DE',
  travelDate: '2025-06-01'
});

if (eligibility.eligible) {
  console.log('Travel approved!');
} else {
  console.log('Missing:', eligibility.missingItems);
}
```

### 10.2 Python

```python
from wia_pet_passport import WIAPetPassport

client = WIAPetPassport(api_key="your_api_key")

# 여권 생성
passport = client.passports.create(
    species="dog",
    breed="Golden Retriever",
    name="Max",
    date_of_birth="2022-03-15"
)

# 백신 기록 추가
vaccination = client.vaccinations.add(
    passport_id=passport.id,
    vaccine_name="Nobivac Rabies",
    target_diseases=["RABIES"],
    administered_at="2025-12-16T10:00:00Z",
    valid_until="2026-12-16T10:00:00Z"
)
```

### 10.3 Rust

```rust
use wia_pet_passport::{Client, Passport, QuarantineCheck};

#[tokio::main]
async fn main() -> Result<()> {
    let client = Client::new("your_api_key")?;

    // 여권 조회
    let passport = client
        .passports()
        .get("01H5XNQK...")
        .await?;

    // 응급 정보 조회
    let emergency = client
        .emergency()
        .get(&passport.id)
        .await?;

    for alert in emergency.critical_alerts {
        println!("ALERT: {}", alert.message);
    }

    Ok(())
}
```

---

## 11. Compliance

### 11.1 국제 규정 준수

| 규정 | 요구사항 | WIA 구현 |
|------|----------|---------|
| EU Regulation 576/2013 | EU 펫 여권 | 완전 호환 |
| USDA APHIS | 미국 수입 규정 | 인증서 생성 |
| OIE Standards | 국제 동물 건강 | 질병 코드 매핑 |
| GDPR | 개인정보 보호 | 데이터 최소화 |
| KPIPA | 한국 개인정보보호법 | 동의 관리 |

### 11.2 Audit Logging

```typescript
interface AuditLog {
  logId: string;
  timestamp: ISO8601;
  actor: {
    type: 'user' | 'veterinarian' | 'system' | 'emergency';
    id: string;
    ip?: string;
  };
  action: string;
  resource: {
    type: string;
    id: string;
  };
  outcome: 'success' | 'failure';
  details?: object;
}
```

---

**Document ID**: WIA-PET-HEALTH-PASSPORT-PHASE3-001
**Version**: 1.0.0
**Last Updated**: 2025-12-16
**Copyright**: © 2025 WIA - MIT License

**홍익인간 (弘益人間)** - 인류와 동물 모두를 이롭게 하라
