# WIA Food Allergy Passport API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Core Interfaces](#core-interfaces)
4. [API Endpoints](#api-endpoints)
5. [Passport Management](#passport-management)
6. [Verification Services](#verification-services)
7. [QR Code Services](#qr-code-services)
8. [Translation Services](#translation-services)
9. [Integration APIs](#integration-apis)
10. [Authentication](#authentication)
11. [Error Handling](#error-handling)
12. [Usage Examples](#usage-examples)

---

## Overview

### 1.1 Purpose

The WIA Food Allergy Passport API Interface Standard defines a comprehensive programmatic interface for managing food allergy passports, enabling seamless integration with restaurants, airlines, hospitals, and other food service providers. This Phase 2 specification builds upon the Phase 1 Data Format, providing developers with standardized APIs for creating, sharing, and verifying allergy information.

**Core Objectives**:
- Provide unified API for all allergy passport operations
- Enable secure sharing of allergy data via QR codes
- Support real-time allergy verification for food service providers
- Facilitate multilingual translation of allergy information
- Enable emergency medical access to critical allergy data

### 1.2 Scope

This standard defines:

| Component | Description |
|-----------|-------------|
| **Core API** | Main AllergyPassport class interface |
| **REST Endpoints** | HTTP API for passport operations |
| **QR Code API** | Generation and scanning of allergy QR codes |
| **Translation API** | Multilingual allergy information |
| **Integration APIs** | Restaurant POS, airline systems, healthcare |

### 1.3 Authentication Methods

| Method | Use Case | Security Level |
|--------|----------|----------------|
| **OAuth 2.0** | User applications, mobile apps | High |
| **API Key** | Restaurant systems, backend services | Medium |
| **QR Code** | Public scanning, one-time access | Low (read-only) |
| **Medical Access** | Emergency medical personnel | High (special permissions) |

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **AllergyPassport** | Main API class for passport operations |
| **PassportHolder** | Individual with food allergies |
| **VerificationLevel** | Confidence level of allergy verification |
| **ShareToken** | Temporary token for sharing passport data |
| **QRPayload** | Encoded allergy data in QR code |
| **TranslationEngine** | Service for multilingual translations |

### 2.2 API Response Types

| Type | Description | HTTP Status |
|------|-------------|-------------|
| `Success` | Operation completed successfully | 200, 201 |
| `ValidationError` | Invalid input data | 400 |
| `AuthenticationError` | Invalid credentials | 401 |
| `AuthorizationError` | Insufficient permissions | 403 |
| `NotFoundError` | Resource not found | 404 |
| `ServerError` | Internal server error | 500 |

---

## Core Interfaces

### 3.1 AllergyPassport Class

Main API entry point for allergy passport operations.

#### TypeScript

```typescript
class AllergyPassport {
  // Constructor
  constructor(options?: AllergyPassportOptions);

  // Passport Management
  createPassport(data: PassportCreationData): Promise<PassportRecord>;
  getPassport(passportId: string): Promise<PassportRecord | null>;
  updatePassport(passportId: string, updates: Partial<PassportRecord>): Promise<PassportRecord>;
  deletePassport(passportId: string): Promise<void>;
  listPassports(userId: string): Promise<PassportRecord[]>;
  renewPassport(passportId: string): Promise<PassportRecord>;

  // Allergy Management
  addAllergy(passportId: string, allergy: AllergyEntry): Promise<void>;
  updateAllergy(passportId: string, allergyId: string, updates: Partial<AllergyEntry>): Promise<void>;
  removeAllergy(passportId: string, allergyId: string): Promise<void>;
  getAllergies(passportId: string): Promise<AllergyEntry[]>;

  // QR Code Operations
  generateQRCode(passportId: string, options?: QROptions): Promise<QRCodeData>;
  scanQRCode(qrData: string): Promise<PassportSummary>;
  validateQRCode(qrData: string): Promise<boolean>;
  getQRPayload(passportId: string): Promise<string>;

  // Translation Services
  translatePassport(passportId: string, targetLanguage: string): Promise<TranslatedPassport>;
  getSupportedLanguages(): Promise<string[]>;
  getTranslation(passportId: string, language: string, field: string): Promise<string>;
  addCustomTranslation(passportId: string, language: string, text: string): Promise<void>;

  // Verification Services
  verifyPassport(passportId: string, verificationData: VerificationData): Promise<VerificationResult>;
  getMedicalVerification(passportId: string): Promise<MedicalVerification | null>;
  requestMedicalVerification(passportId: string, providerId: string): Promise<void>;

  // Sharing & Access
  createShareToken(passportId: string, permissions: SharePermissions): Promise<ShareToken>;
  revokeShareToken(tokenId: string): Promise<void>;
  getShareHistory(passportId: string): Promise<ShareEvent[]>;
  grantEmergencyAccess(passportId: string, contactInfo: EmergencyContact): Promise<string>;

  // Restaurant Integration
  notifyRestaurant(passportId: string, restaurantId: string): Promise<NotificationResult>;
  getRestaurantMenu(restaurantId: string, passportId: string): Promise<SafeMenuItem[]>;
  checkMenuItemSafety(passportId: string, menuItemId: string): Promise<SafetyCheck>;

  // Airline Integration
  submitToAirline(passportId: string, flightInfo: FlightInfo): Promise<AirlineSubmission>;
  getAirlineMealOptions(passportId: string, flightId: string): Promise<MealOption[]>;

  // Event Handling
  on<T extends EventType>(event: T, handler: EventHandler<T>): void;
  off<T extends EventType>(event: T, handler: EventHandler<T>): void;

  // Analytics
  getStatistics(passportId: string): Promise<PassportStatistics>;
  getUsageReport(passportId: string, period: TimePeriod): Promise<UsageReport>;
}
```

#### Python

```python
class AllergyPassport:
    def __init__(self, options: Optional[AllergyPassportOptions] = None):
        ...

    # Passport Management
    async def create_passport(self, data: PassportCreationData) -> PassportRecord: ...
    async def get_passport(self, passport_id: str) -> Optional[PassportRecord]: ...
    async def update_passport(self, passport_id: str, updates: dict) -> PassportRecord: ...
    async def delete_passport(self, passport_id: str) -> None: ...
    async def list_passports(self, user_id: str) -> List[PassportRecord]: ...
    async def renew_passport(self, passport_id: str) -> PassportRecord: ...

    # Allergy Management
    async def add_allergy(self, passport_id: str, allergy: AllergyEntry) -> None: ...
    async def update_allergy(self, passport_id: str, allergy_id: str, updates: dict) -> None: ...
    async def remove_allergy(self, passport_id: str, allergy_id: str) -> None: ...
    async def get_allergies(self, passport_id: str) -> List[AllergyEntry]: ...

    # QR Code Operations
    async def generate_qr_code(self, passport_id: str, options: Optional[QROptions] = None) -> QRCodeData: ...
    async def scan_qr_code(self, qr_data: str) -> PassportSummary: ...
    async def validate_qr_code(self, qr_data: str) -> bool: ...
    async def get_qr_payload(self, passport_id: str) -> str: ...

    # Translation Services
    async def translate_passport(self, passport_id: str, target_language: str) -> TranslatedPassport: ...
    async def get_supported_languages(self) -> List[str]: ...
    async def get_translation(self, passport_id: str, language: str, field: str) -> str: ...

    # Verification Services
    async def verify_passport(self, passport_id: str, verification_data: VerificationData) -> VerificationResult: ...
    async def get_medical_verification(self, passport_id: str) -> Optional[MedicalVerification]: ...

    # Sharing & Access
    async def create_share_token(self, passport_id: str, permissions: SharePermissions) -> ShareToken: ...
    async def revoke_share_token(self, token_id: str) -> None: ...
    async def get_share_history(self, passport_id: str) -> List[ShareEvent]: ...

    # Restaurant Integration
    async def notify_restaurant(self, passport_id: str, restaurant_id: str) -> NotificationResult: ...
    async def get_restaurant_menu(self, restaurant_id: str, passport_id: str) -> List[SafeMenuItem]: ...
    async def check_menu_item_safety(self, passport_id: str, menu_item_id: str) -> SafetyCheck: ...

    # Airline Integration
    async def submit_to_airline(self, passport_id: str, flight_info: FlightInfo) -> AirlineSubmission: ...
    async def get_airline_meal_options(self, passport_id: str, flight_id: str) -> List[MealOption]: ...
```

### 3.2 AllergyPassportOptions

```typescript
interface AllergyPassportOptions {
  // API Configuration
  apiUrl?: string;
  apiKey?: string;
  oauth?: {
    clientId: string;
    clientSecret: string;
    redirectUri: string;
  };

  // Storage Configuration
  storage?: 'cloud' | 'local' | 'hybrid';
  encryption?: {
    algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
    keyDerivation: 'PBKDF2' | 'Argon2';
  };

  // QR Code Configuration
  qrOptions?: {
    errorCorrection: 'L' | 'M' | 'Q' | 'H';
    version?: number;
    size?: number;
  };

  // Translation Configuration
  translationProvider?: 'google' | 'aws' | 'azure' | 'custom';
  fallbackLanguage?: string;

  // Integration Configuration
  integrations?: {
    restaurants?: RestaurantConfig;
    airlines?: AirlineConfig;
    healthcare?: HealthcareConfig;
  };
}
```

---

## API Endpoints

### 4.1 Base URL

```
Production:  https://api.wia.live/food-allergy-passport/v1
Staging:     https://api-staging.wia.live/food-allergy-passport/v1
Development: http://localhost:3000/api/v1
```

### 4.2 Passport Endpoints

#### POST /passports

Create a new allergy passport.

**Request:**
```http
POST /passports
Authorization: Bearer {token}
Content-Type: application/json

{
  "holder": {
    "userId": "USER-2025-001",
    "displayName": "John Doe",
    "dateOfBirth": "1990-05-15"
  },
  "allergies": [
    {
      "allergen": "peanuts",
      "allergenCode": "FDA-PEANUT",
      "severity": "anaphylaxis",
      "symptoms": [
        {"type": "respiratory", "description": "difficulty breathing"}
      ]
    }
  ],
  "medications": {
    "epipen": {
      "brand": "EpiPen",
      "dosage": "0.3mg"
    }
  }
}
```

**Response:**
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "success": true,
  "data": {
    "passportId": "FAP-2025-000001",
    "status": "active",
    "created": "2025-01-15T10:00:00Z",
    "expiresAt": "2026-01-15T10:00:00Z",
    "qrCode": "https://api.wia.live/qr/FAP-2025-000001"
  }
}
```

#### GET /passports/{passportId}

Retrieve passport details.

**Request:**
```http
GET /passports/FAP-2025-000001
Authorization: Bearer {token}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "passportId": "FAP-2025-000001",
    "status": "active",
    "holder": { ... },
    "allergies": [ ... ],
    "medications": { ... }
  }
}
```

#### PATCH /passports/{passportId}

Update passport information.

**Request:**
```http
PATCH /passports/FAP-2025-000001
Authorization: Bearer {token}
Content-Type: application/json

{
  "allergies": [
    {
      "allergen": "shellfish",
      "allergenCode": "FDA-SHELLFISH",
      "severity": "severe"
    }
  ]
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "passportId": "FAP-2025-000001",
    "lastUpdated": "2025-01-15T14:30:00Z",
    "version": 2
  }
}
```

#### DELETE /passports/{passportId}

Delete a passport.

**Request:**
```http
DELETE /passports/FAP-2025-000001
Authorization: Bearer {token}
```

**Response:**
```http
HTTP/1.1 204 No Content
```

### 4.3 QR Code Endpoints

#### POST /passports/{passportId}/qr

Generate QR code for passport.

**Request:**
```http
POST /passports/FAP-2025-000001/qr
Authorization: Bearer {token}
Content-Type: application/json

{
  "format": "png",
  "size": 512,
  "errorCorrection": "H",
  "includeFields": ["allergies", "severity", "medications"]
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "qrCode": "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAA...",
    "payload": "FAP://v1/eyJwYXNzcG9ydElkIjoi...",
    "expiresAt": "2025-01-15T22:00:00Z"
  }
}
```

#### POST /qr/scan

Scan and decode QR code.

**Request:**
```http
POST /qr/scan
Content-Type: application/json

{
  "qrData": "FAP://v1/eyJwYXNzcG9ydElkIjoi..."
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "passportId": "FAP-2025-000001",
    "allergies": [
      {
        "allergen": "peanuts",
        "severity": "anaphylaxis",
        "emergencyProtocol": "Administer EpiPen, call 911"
      }
    ],
    "emergencyContact": "encrypted:..."
  }
}
```

### 4.4 Translation Endpoints

#### GET /passports/{passportId}/translate/{language}

Get translated passport.

**Request:**
```http
GET /passports/FAP-2025-000001/translate/ko
Authorization: Bearer {token}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "language": "ko",
    "allergies": [
      {
        "allergen": "땅콩",
        "severity": "아나필락시스 (생명 위협)",
        "symptoms": ["호흡 곤란", "두드러기", "혈압 저하"],
        "emergencyAction": "에피펜 투여 후 즉시 119에 전화하세요"
      }
    ],
    "warning": "이 사람은 심각한 땅콩 알레르기가 있습니다. 미량의 땅콩도 생명을 위협할 수 있습니다."
  }
}
```

#### GET /languages

Get supported languages.

**Request:**
```http
GET /languages
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "languages": [
      {"code": "en", "name": "English", "nativeName": "English"},
      {"code": "ko", "name": "Korean", "nativeName": "한국어"},
      {"code": "ja", "name": "Japanese", "nativeName": "日本語"},
      {"code": "zh", "name": "Chinese", "nativeName": "中文"}
    ],
    "total": 52
  }
}
```

### 4.5 Restaurant Integration Endpoints

#### POST /restaurants/{restaurantId}/notify

Notify restaurant of allergy passport.

**Request:**
```http
POST /restaurants/REST-12345/notify
Authorization: Bearer {token}
Content-Type: application/json

{
  "passportId": "FAP-2025-000001",
  "tableNumber": "15",
  "message": "Customer with severe peanut allergy"
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "notificationId": "NOTIF-001",
    "restaurant": "Seoul Bistro",
    "acknowledged": true,
    "acknowledgedBy": "Chef Kim",
    "timestamp": "2025-01-15T18:30:00Z"
  }
}
```

#### GET /restaurants/{restaurantId}/menu/safe

Get safe menu items for passport holder.

**Request:**
```http
GET /restaurants/REST-12345/menu/safe?passportId=FAP-2025-000001
Authorization: Bearer {token}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "safeItems": [
      {
        "itemId": "MENU-101",
        "name": "Grilled Salmon",
        "description": "Wild salmon with vegetables",
        "allergenFree": ["peanuts", "tree nuts", "shellfish"],
        "safetyScore": 95
      },
      {
        "itemId": "MENU-205",
        "name": "Vegetable Stir Fry",
        "safetyScore": 98
      }
    ],
    "unsafeItems": [
      {
        "itemId": "MENU-150",
        "name": "Thai Curry",
        "reason": "Contains peanuts",
        "allergens": ["peanuts"]
      }
    ]
  }
}
```

### 4.6 Airline Integration Endpoints

#### POST /airlines/submit

Submit passport to airline for flight.

**Request:**
```http
POST /airlines/submit
Authorization: Bearer {token}
Content-Type: application/json

{
  "passportId": "FAP-2025-000001",
  "airline": "Korean Air",
  "flightNumber": "KE123",
  "departureDate": "2025-02-01",
  "seatNumber": "12A",
  "mealPreferences": ["no peanuts", "no shellfish"]
}
```

**Response:**
```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "success": true,
  "data": {
    "submissionId": "AIR-SUB-001",
    "airline": "Korean Air",
    "flightNumber": "KE123",
    "status": "confirmed",
    "mealOptions": [
      {
        "code": "VGML",
        "name": "Vegetarian Meal",
        "safe": true
      },
      {
        "code": "SFML",
        "name": "Seafood Meal",
        "safe": false,
        "reason": "May contain shellfish"
      }
    ],
    "confirmationNumber": "MEAL-12345"
  }
}
```

### 4.7 Verification Endpoints

#### POST /passports/{passportId}/verify

Request medical verification.

**Request:**
```http
POST /passports/FAP-2025-000001/verify
Authorization: Bearer {token}
Content-Type: application/json

{
  "providerId": "HOSP-SNH-001",
  "providerName": "Seoul National Hospital",
  "verificationMethod": "medical records + allergy testing",
  "testResults": {
    "allergen": "peanuts",
    "testType": "skin prick test",
    "result": "positive",
    "date": "2025-01-10"
  }
}
```

**Response:**
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "data": {
    "verificationId": "VER-001",
    "status": "verified",
    "verifiedBy": "Seoul National Hospital",
    "verifiedAt": "2025-01-15T10:00:00Z",
    "validUntil": "2026-01-15T10:00:00Z",
    "certificate": "https://api.wia.live/certificates/VER-001"
  }
}
```

---

## Authentication

### 5.1 OAuth 2.0 Flow

#### Authorization Request

```http
GET /oauth/authorize?
  client_id=YOUR_CLIENT_ID&
  redirect_uri=https://yourapp.com/callback&
  response_type=code&
  scope=passport:read passport:write
```

#### Token Exchange

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTHORIZATION_CODE&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
redirect_uri=https://yourapp.com/callback
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "eyJhbGciOiJIUzI1NiIs...",
  "scope": "passport:read passport:write"
}
```

### 5.2 API Key Authentication

```http
GET /passports/FAP-2025-000001
X-API-Key: your-api-key-here
```

### 5.3 Scopes

| Scope | Description |
|-------|-------------|
| `passport:read` | Read passport data |
| `passport:write` | Create and update passports |
| `passport:delete` | Delete passports |
| `qr:generate` | Generate QR codes |
| `translation:access` | Access translation services |
| `restaurant:notify` | Send restaurant notifications |
| `airline:submit` | Submit to airlines |
| `medical:verify` | Access medical verification |
| `emergency:access` | Emergency medical access (restricted) |

---

## Error Handling

### 6.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "ERR_INVALID_PASSPORT",
    "message": "Invalid passport format",
    "details": {
      "field": "passportId",
      "reason": "Must match pattern FAP-YYYY-NNNNNN"
    },
    "timestamp": "2025-01-15T10:00:00Z",
    "requestId": "req_abc123"
  }
}
```

### 6.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ERR_INVALID_PASSPORT` | 400 | Invalid passport data |
| `ERR_MISSING_EPIPEN` | 400 | EpiPen required for anaphylaxis |
| `ERR_UNAUTHORIZED` | 401 | Invalid or missing authentication |
| `ERR_FORBIDDEN` | 403 | Insufficient permissions |
| `ERR_PASSPORT_NOT_FOUND` | 404 | Passport does not exist |
| `ERR_EXPIRED_PASSPORT` | 400 | Passport has expired |
| `ERR_INVALID_QR` | 400 | Invalid QR code data |
| `ERR_TRANSLATION_FAILED` | 500 | Translation service error |
| `ERR_RATE_LIMIT` | 429 | Too many requests |

---

## Usage Examples

### 7.1 TypeScript Example - Create Passport

```typescript
import { AllergyPassport, AllergyEntry, SeverityLevel } from '@wia/food-allergy-passport';

const passport = new AllergyPassport({
  apiUrl: 'https://api.wia.live/food-allergy-passport/v1',
  oauth: {
    clientId: 'your-client-id',
    clientSecret: 'your-client-secret',
    redirectUri: 'https://yourapp.com/callback'
  }
});

// Create new passport
const newPassport = await passport.createPassport({
  holder: {
    userId: 'USER-2025-001',
    displayName: 'John Doe',
    dateOfBirth: '1990-05-15'
  },
  allergies: [
    {
      allergen: 'peanuts',
      allergenCode: 'FDA-PEANUT',
      icdCode: 'Z91.010',
      severity: 'anaphylaxis',
      symptoms: [
        {
          type: 'respiratory',
          description: 'difficulty breathing',
          onsetTime: '5-10 minutes'
        }
      ],
      threshold: 'trace amounts'
    }
  ],
  medications: {
    epipen: {
      brand: 'EpiPen',
      dosage: '0.3mg',
      location: 'always carried'
    }
  }
});

console.log(`Passport created: ${newPassport.passportId}`);

// Generate QR code
const qrCode = await passport.generateQRCode(newPassport.passportId, {
  size: 512,
  errorCorrection: 'H'
});

console.log(`QR Code: ${qrCode.data}`);
```

### 7.2 Python Example - Restaurant Integration

```python
from wia_food_allergy_passport import AllergyPassport, RestaurantConfig

passport = AllergyPassport(
    api_url='https://api.wia.live/food-allergy-passport/v1',
    api_key='your-api-key'
)

# Notify restaurant
result = await passport.notify_restaurant(
    passport_id='FAP-2025-000001',
    restaurant_id='REST-12345',
    table_number='15',
    message='Customer with severe peanut allergy'
)

print(f"Notification acknowledged by: {result['acknowledgedBy']}")

# Get safe menu items
safe_items = await passport.get_restaurant_menu(
    restaurant_id='REST-12345',
    passport_id='FAP-2025-000001'
)

for item in safe_items:
    print(f"Safe: {item['name']} (score: {item['safetyScore']})")
```

### 7.3 JavaScript Example - QR Code Scanning

```javascript
const { AllergyPassport } = require('@wia/food-allergy-passport');

const scanner = new AllergyPassport({
  apiUrl: 'https://api.wia.live/food-allergy-passport/v1'
});

// Scan QR code (no authentication required for read-only)
async function scanPassportQR(qrData) {
  try {
    const summary = await scanner.scanQRCode(qrData);

    console.log('Allergy Information:');
    summary.allergies.forEach(allergy => {
      console.log(`- ${allergy.allergen}: ${allergy.severity}`);
      console.log(`  Emergency: ${allergy.emergencyProtocol}`);
    });

    if (summary.medications.epipen) {
      console.log('\n⚠️ CARRIES EPIPEN');
    }
  } catch (error) {
    console.error('Error scanning QR code:', error);
  }
}

// Usage
scanPassportQR('FAP://v1/eyJwYXNzcG9ydElkIjoi...');
```

### 7.4 Python Example - Translation

```python
# Translate passport to Korean
translated = await passport.translate_passport(
    passport_id='FAP-2025-000001',
    target_language='ko'
)

print(translated['allergies'][0]['allergen'])  # "땅콩"
print(translated['allergies'][0]['severity'])  # "아나필락시스"

# Get all supported languages
languages = await passport.get_supported_languages()
for lang in languages:
    print(f"{lang['code']}: {lang['nativeName']}")
```

### 7.5 TypeScript Example - Airline Submission

```typescript
// Submit passport to airline
const submission = await passport.submitToAirline('FAP-2025-000001', {
  airline: 'Korean Air',
  flightNumber: 'KE123',
  departureDate: '2025-02-01',
  seatNumber: '12A',
  specialRequests: 'Peanut-free meal, no shellfish'
});

console.log(`Confirmation: ${submission.confirmationNumber}`);

// Get safe meal options
const mealOptions = await passport.getAirlineMealOptions(
  'FAP-2025-000001',
  submission.flightId
);

mealOptions.forEach(meal => {
  if (meal.safe) {
    console.log(`✅ ${meal.name} (${meal.code})`);
  } else {
    console.log(`❌ ${meal.name} - ${meal.reason}`);
  }
});
```

### 7.6 Python Example - Emergency Access

```python
# Emergency medical access (special permissions required)
emergency_data = await passport.get_passport(
    passport_id='FAP-2025-000001',
    access_type='emergency',
    requestor='Paramedic John Smith',
    incident_id='EMG-2025-001'
)

print("EMERGENCY ALLERGY INFORMATION")
print("=" * 50)
for allergy in emergency_data['allergies']:
    print(f"\nAllergen: {allergy['allergen']}")
    print(f"Severity: {allergy['severity']}")
    print(f"Symptoms: {', '.join([s['description'] for s in allergy['symptoms']])}")

if emergency_data['medications']['epipen']:
    epipen = emergency_data['medications']['epipen']
    print(f"\n⚠️ EPIPEN: {epipen['dosage']}")
    print(f"Location: {epipen['location']}")
    print(f"Instructions: {epipen['instructions']}")

# Emergency contact
contact = emergency_data['holder']['emergencyContacts'][0]
print(f"\nEmergency Contact: {contact['name']}")
print(f"Phone: {contact['phone']}")
```

---

<div align="center">

**WIA Food Allergy Passport Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
