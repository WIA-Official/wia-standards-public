# WIA-CRYO-ASSET Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #06B6D4 (Cyan)

## 1. Introduction

### 1.1 Purpose

This Phase 2 specification defines the Application Programming Interfaces (APIs) for the WIA Cryogenic Asset Management Standard. These APIs enable secure, standardized interactions between cryonics facilities, financial institutions, custodians, legal representatives, and revival verification systems.

### 1.2 API Design Principles

- **Security First**: All endpoints require authentication and authorization
- **Idempotency**: Write operations support idempotent requests
- **Versioning**: API versioning through URL path and headers
- **Rate Limiting**: Protection against abuse and DoS attacks
- **Audit Logging**: Complete audit trail of all operations
- **HATEOAS**: Hypermedia-driven navigation where appropriate

### 1.3 Base URL Structure

```
Production:  https://api.wia.dev/cryo-asset/v1
Staging:     https://api-staging.wia.dev/cryo-asset/v1
Sandbox:     https://api-sandbox.wia.dev/cryo-asset/v1
```

## 2. Authentication and Authorization

### 2.1 Authentication Methods

The API supports multiple authentication methods based on client type and security requirements:

| Method | Use Case | Security Level | Token Lifetime |
|--------|----------|----------------|----------------|
| OAuth 2.0 | Web applications, trusted clients | High | 1 hour (access), 30 days (refresh) |
| JWT Bearer | Service-to-service | High | 24 hours |
| API Key + Secret | Legacy systems | Medium | No expiration (rotatable) |
| Multi-Sig | High-value transactions | Critical | Single use |
| Biometric + JWT | Revival verification | Critical | 15 minutes |

### 2.2 OAuth 2.0 Authentication Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "cryo_custodian_12345",
  "client_secret": "sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER",
  "scope": "asset:read asset:write custodian:manage"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "rt_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6",
  "scope": "asset:read asset:write custodian:manage"
}
```

### 2.3 Authorization Scopes

| Scope | Description | Required Role |
|-------|-------------|---------------|
| `asset:read` | View asset information | Any authenticated user |
| `asset:write` | Create or update assets | Custodian, Legal Rep |
| `asset:delete` | Archive or remove assets | Primary Custodian + Legal Approval |
| `custodian:read` | View custodian information | Custodian, Legal Rep, Family |
| `custodian:manage` | Add/remove custodians | Primary Custodian + Court Order |
| `transaction:create` | Initiate asset transactions | Custodian |
| `transaction:approve` | Approve pending transactions | Authorized Signers |
| `revival:verify` | Submit revival verification | Medical Director, Legal Authority |
| `revival:execute` | Execute asset transfer upon revival | All Required Authorities |
| `audit:read` | Access audit logs | Custodian, Legal Rep |

## 3. Core API Endpoints

### 3.1 Asset Registry Endpoints

#### 3.1.1 Create Asset Registry

```http
POST /registries
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "John Michael Anderson",
    "dateOfBirth": "1985-06-15",
    "nationality": ["USA"],
    "identityProof": {
      "type": "biometric_hash",
      "value": "a7f3c9e2b8d4f1a6e9c3b7d2f5a8e1c4b9d6f3a7e2c8b5d1f4a9e6c3b8d7f2a5"
    }
  },
  "preservationDetails": {
    "facility": "Alcor Life Extension Foundation",
    "facilityId": "ALCOR-AZ-001",
    "preservationDate": "2025-12-20T14:00:00Z",
    "preservationType": "vitrification"
  },
  "custodians": [
    {
      "type": "primary",
      "name": "Trust Company Alpha",
      "contact": "custodian@trustalpha.com",
      "publicKey": "-----BEGIN PUBLIC KEY-----\nMIICIjANBgkq..."
    }
  ]
}
```

**Response (201 Created):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "version": "1.0.0",
  "status": "active",
  "created": "2025-12-18T10:30:00Z",
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "John Michael Anderson"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9",
    "assets": "/registries/AR-2025-1734519000-A7F3C9/assets",
    "custodians": "/registries/AR-2025-1734519000-A7F3C9/custodians"
  }
}
```

#### 3.1.2 Get Asset Registry

```http
GET /registries/{registryId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "version": "1.0.0",
  "status": "active",
  "created": "2025-12-18T10:30:00Z",
  "lastModified": "2025-12-18T12:45:00Z",
  "subject": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "John Michael Anderson",
    "dateOfBirth": "1985-06-15",
    "preservationStatus": "preserved",
    "preservationDate": "2025-12-20T14:00:00Z"
  },
  "assets": {
    "financial": 12,
    "realEstate": 3,
    "intellectual": 2,
    "digital": 45,
    "personal": 8,
    "business": 1
  },
  "totalValue": {
    "amount": 5750000.00,
    "currency": "USD",
    "lastAssessed": "2025-12-18T00:00:00Z"
  },
  "custodians": [
    {
      "id": "CUST-001",
      "type": "primary",
      "name": "Trust Company Alpha",
      "status": "active"
    }
  ],
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9",
    "assets": "/registries/AR-2025-1734519000-A7F3C9/assets",
    "transactions": "/registries/AR-2025-1734519000-A7F3C9/transactions"
  }
}
```

#### 3.1.3 Update Asset Registry

```http
PATCH /registries/{registryId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "operations": [
    {
      "op": "add",
      "path": "/custodians/-",
      "value": {
        "type": "backup",
        "name": "Estate Attorney Services LLC",
        "contact": "attorney@estatellc.com"
      }
    },
    {
      "op": "replace",
      "path": "/subject/preservationStatus",
      "value": "preserved"
    }
  ]
}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "version": "1.0.1",
  "lastModified": "2025-12-18T14:22:00Z",
  "changesApplied": 2,
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9"
  }
}
```

### 3.2 Asset Management Endpoints

#### 3.2.1 Add Financial Asset

```http
POST /registries/{registryId}/assets/financial
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "category": "cryptocurrency",
  "asset": {
    "name": "Bitcoin Holdings",
    "cryptocurrency": {
      "symbol": "BTC",
      "network": "bitcoin",
      "wallets": [
        {
          "type": "cold_storage",
          "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
          "balance": {
            "amount": "12.45678901",
            "unit": "BTC"
          },
          "accessMethod": "multisig",
          "requiredSignatures": 3,
          "totalSignatures": 5
        }
      ]
    }
  },
  "custodian": {
    "primary": "Trust Company Alpha",
    "backup": ["Estate Attorney"]
  }
}
```

**Response (201 Created):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "assetType": "financial",
  "category": "cryptocurrency",
  "status": "active",
  "registered": "2025-12-18T10:35:00Z",
  "valuation": {
    "totalValue": 523456.78,
    "currency": "USD",
    "lastAssessed": "2025-12-18T10:35:00Z"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/assets/FA-2025-BTC-001",
    "registry": "/registries/AR-2025-1734519000-A7F3C9",
    "transactions": "/assets/FA-2025-BTC-001/transactions"
  }
}
```

#### 3.2.2 Get Asset Details

```http
GET /assets/{assetId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "assetType": "financial",
  "category": "cryptocurrency",
  "status": "active",
  "registered": "2025-12-18T10:35:00Z",
  "asset": {
    "name": "Bitcoin Holdings",
    "cryptocurrency": {
      "symbol": "BTC",
      "network": "bitcoin",
      "totalBalance": {
        "amount": "12.45678901",
        "unit": "BTC",
        "usdValue": 523456.78
      },
      "wallets": [
        {
          "walletId": "WALLET-BTC-001",
          "type": "cold_storage",
          "address": "bc1qxy2k...fjhx0wlh",
          "balance": {
            "amount": "12.45678901",
            "unit": "BTC"
          },
          "securityLevel": "multisig_3_of_5"
        }
      ]
    }
  },
  "valuation": {
    "totalValue": 523456.78,
    "currency": "USD",
    "lastAssessed": "2025-12-18T10:35:00Z",
    "volatilityRating": "high"
  },
  "management": {
    "strategy": "hold",
    "custodian": "Trust Company Alpha"
  },
  "links": {
    "self": "/assets/FA-2025-BTC-001",
    "registry": "/registries/AR-2025-1734519000-A7F3C9",
    "transactions": "/assets/FA-2025-BTC-001/transactions",
    "valuation_history": "/assets/FA-2025-BTC-001/valuations"
  }
}
```

#### 3.2.3 List Assets

```http
GET /registries/{registryId}/assets?type=financial&category=cryptocurrency&limit=20&offset=0
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "total": 3,
  "limit": 20,
  "offset": 0,
  "assets": [
    {
      "assetId": "FA-2025-BTC-001",
      "assetType": "financial",
      "category": "cryptocurrency",
      "name": "Bitcoin Holdings",
      "value": 523456.78,
      "currency": "USD",
      "status": "active"
    },
    {
      "assetId": "FA-2025-ETH-001",
      "assetType": "financial",
      "category": "cryptocurrency",
      "name": "Ethereum Holdings",
      "value": 367890.12,
      "currency": "USD",
      "status": "active"
    },
    {
      "assetId": "FA-2025-USDC-001",
      "assetType": "financial",
      "category": "cryptocurrency",
      "name": "USDC Stablecoin Reserve",
      "value": 100000.00,
      "currency": "USD",
      "status": "active"
    }
  ],
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/assets?type=financial&category=cryptocurrency&limit=20&offset=0",
    "first": "/registries/AR-2025-1734519000-A7F3C9/assets?type=financial&category=cryptocurrency&limit=20&offset=0",
    "last": "/registries/AR-2025-1734519000-A7F3C9/assets?type=financial&category=cryptocurrency&limit=20&offset=0"
  }
}
```

#### 3.2.4 Update Asset

```http
PATCH /assets/{assetId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "operations": [
    {
      "op": "replace",
      "path": "/asset/cryptocurrency/wallets/0/balance/amount",
      "value": "12.56789012"
    },
    {
      "op": "replace",
      "path": "/valuation/totalValue",
      "value": 528234.50
    },
    {
      "op": "replace",
      "path": "/valuation/lastAssessed",
      "value": "2025-12-18T15:00:00Z"
    }
  ]
}
```

**Response (200 OK):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "version": "1.0.1",
  "lastModified": "2025-12-18T15:00:00Z",
  "changesApplied": 3,
  "links": {
    "self": "/assets/FA-2025-BTC-001"
  }
}
```

#### 3.2.5 Delete/Archive Asset

```http
DELETE /assets/{assetId}
Authorization: Bearer {access_token}
X-Reason: "Asset liquidated to cover preservation expenses"
X-Authorization-Signatures: 3
```

**Response (200 OK):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "status": "archived",
  "archivedAt": "2025-12-18T16:30:00Z",
  "reason": "Asset liquidated to cover preservation expenses",
  "approvedBy": [
    "primary_custodian",
    "backup_custodian",
    "legal_representative"
  ],
  "links": {
    "audit_log": "/assets/FA-2025-BTC-001/audit"
  }
}
```

### 3.3 Transaction Endpoints

#### 3.3.1 Create Transaction

```http
POST /transactions
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "assetId": "FA-2025-BTC-001",
  "type": "transfer",
  "action": "sell_partial",
  "amount": {
    "value": "2.0",
    "unit": "BTC"
  },
  "purpose": "preservation_expenses",
  "description": "Liquidate 2 BTC to cover annual preservation and custodian fees",
  "requiredSignatures": 3,
  "recipient": {
    "type": "bank_account",
    "accountId": "BANK-2025-001",
    "accountName": "Preservation Expense Account"
  }
}
```

**Response (201 Created):**

```json
{
  "transactionId": "TX-1734519600-A7B8C9D0E1F2",
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "transfer",
  "action": "sell_partial",
  "status": "pending_signatures",
  "created": "2025-12-18T11:00:00Z",
  "amount": {
    "value": "2.0",
    "unit": "BTC",
    "estimatedUSD": 84000.00
  },
  "requiredSignatures": 3,
  "currentSignatures": 0,
  "signingDeadline": "2025-12-21T11:00:00Z",
  "links": {
    "self": "/transactions/TX-1734519600-A7B8C9D0E1F2",
    "sign": "/transactions/TX-1734519600-A7B8C9D0E1F2/sign",
    "cancel": "/transactions/TX-1734519600-A7B8C9D0E1F2/cancel"
  }
}
```

#### 3.3.2 Sign Transaction

```http
POST /transactions/{transactionId}/sign
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "signerId": "CUST-001",
  "signerRole": "primary_custodian",
  "signature": "3045022100f7a9c8b6d4e2f0a8c6b4d2e0f9a7c5b3d1e9f7a5c3b1d9e7f5a3c1b9e7f5a3c10220a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2",
  "publicKey": "04a7b1c3d5e7f9a2b4c6d8e0f1a3b5c7d9e1f3a5b7c9d1e3f5a7b9c1d3e5f7a9b1c3d5e7f9a2b4c6d8e0f1a3b5c7d9e1f3a5b7c9d1e3f5a7b9c1d3"
}
```

**Response (200 OK):**

```json
{
  "transactionId": "TX-1734519600-A7B8C9D0E1F2",
  "status": "pending_signatures",
  "currentSignatures": 1,
  "requiredSignatures": 3,
  "signatures": [
    {
      "signerId": "CUST-001",
      "signerRole": "primary_custodian",
      "signerName": "Trust Company Alpha",
      "signedAt": "2025-12-18T11:15:00Z",
      "verified": true
    }
  ],
  "links": {
    "self": "/transactions/TX-1734519600-A7B8C9D0E1F2"
  }
}
```

#### 3.3.3 Get Transaction Status

```http
GET /transactions/{transactionId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "transactionId": "TX-1734519600-A7B8C9D0E1F2",
  "assetId": "FA-2025-BTC-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "transfer",
  "action": "sell_partial",
  "status": "executed",
  "created": "2025-12-18T11:00:00Z",
  "executed": "2025-12-18T14:30:00Z",
  "amount": {
    "value": "2.0",
    "unit": "BTC",
    "actualUSD": 84234.56
  },
  "recipient": {
    "type": "bank_account",
    "accountId": "BANK-2025-001",
    "accountName": "Preservation Expense Account",
    "confirmationNumber": "CONF-987654321"
  },
  "signatures": [
    {
      "signerId": "CUST-001",
      "signerRole": "primary_custodian",
      "signedAt": "2025-12-18T11:15:00Z"
    },
    {
      "signerId": "CUST-002",
      "signerRole": "backup_custodian",
      "signedAt": "2025-12-18T12:30:00Z"
    },
    {
      "signerId": "LEGAL-001",
      "signerRole": "legal_representative",
      "signedAt": "2025-12-18T14:00:00Z"
    }
  ],
  "links": {
    "self": "/transactions/TX-1734519600-A7B8C9D0E1F2",
    "asset": "/assets/FA-2025-BTC-001",
    "receipt": "/transactions/TX-1734519600-A7B8C9D0E1F2/receipt"
  }
}
```

#### 3.3.4 List Transactions

```http
GET /registries/{registryId}/transactions?status=executed&limit=10&offset=0
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "total": 47,
  "limit": 10,
  "offset": 0,
  "transactions": [
    {
      "transactionId": "TX-1734519600-A7B8C9D0E1F2",
      "assetId": "FA-2025-BTC-001",
      "type": "transfer",
      "action": "sell_partial",
      "status": "executed",
      "amount": "84234.56 USD",
      "executed": "2025-12-18T14:30:00Z"
    },
    {
      "transactionId": "TX-1734433200-B8C9D0E1F2A3",
      "assetId": "RE-2025-001",
      "type": "income",
      "action": "rental_payment_received",
      "status": "executed",
      "amount": "3500.00 USD",
      "executed": "2025-12-17T09:00:00Z"
    }
  ],
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/transactions?status=executed&limit=10&offset=0",
    "next": "/registries/AR-2025-1734519000-A7F3C9/transactions?status=executed&limit=10&offset=10"
  }
}
```

### 3.4 Custodian Management Endpoints

#### 3.4.1 Add Custodian

```http
POST /registries/{registryId}/custodians
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "type": "backup",
  "name": "Family Trust Services LLC",
  "contact": {
    "email": "trustees@familytrust.com",
    "phone": "+1-555-0188",
    "address": {
      "street": "456 Trust Avenue",
      "city": "Phoenix",
      "state": "Arizona",
      "zipCode": "85002",
      "country": "USA"
    }
  },
  "credentials": {
    "licenseNumber": "AZ-TRUST-567890",
    "licenseState": "Arizona",
    "bondAmount": 1000000.00,
    "insurancePolicy": "ERRORS-OMISSIONS-2025-12345"
  },
  "publicKey": "-----BEGIN PUBLIC KEY-----\nMIICIjANBgkqhkiG9w0BAQ..."
}
```

**Response (201 Created):**

```json
{
  "custodianId": "CUST-003",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "backup",
  "name": "Family Trust Services LLC",
  "status": "pending_approval",
  "added": "2025-12-18T15:00:00Z",
  "requiredApprovals": 4,
  "currentApprovals": 0,
  "approvalDeadline": "2025-12-28T15:00:00Z",
  "links": {
    "self": "/custodians/CUST-003",
    "approve": "/custodians/CUST-003/approve",
    "registry": "/registries/AR-2025-1734519000-A7F3C9"
  }
}
```

#### 3.4.2 Get Custodian Details

```http
GET /custodians/{custodianId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "custodianId": "CUST-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "primary",
  "name": "Trust Company Alpha",
  "status": "active",
  "added": "2025-12-18T10:30:00Z",
  "contact": {
    "email": "custodian@trustalpha.com",
    "phone": "+1-555-0100",
    "address": {
      "street": "789 Financial Plaza",
      "city": "Phoenix",
      "state": "Arizona",
      "zipCode": "85003",
      "country": "USA"
    }
  },
  "credentials": {
    "licenseNumber": "AZ-TRUST-123456",
    "licenseState": "Arizona",
    "bondAmount": 5000000.00,
    "insurancePolicy": "FIDUCIARY-BOND-2025-67890"
  },
  "performanceMetrics": {
    "assetsUnderManagement": 5750000.00,
    "transactionsProcessed": 47,
    "averageResponseTime": "4.2 hours",
    "complianceScore": 98.5
  },
  "links": {
    "self": "/custodians/CUST-001",
    "registry": "/registries/AR-2025-1734519000-A7F3C9",
    "transactions": "/custodians/CUST-001/transactions"
  }
}
```

#### 3.4.3 List Custodians

```http
GET /registries/{registryId}/custodians
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "total": 3,
  "custodians": [
    {
      "custodianId": "CUST-001",
      "type": "primary",
      "name": "Trust Company Alpha",
      "status": "active"
    },
    {
      "custodianId": "CUST-002",
      "type": "backup",
      "name": "Estate Attorney Services LLC",
      "status": "active"
    },
    {
      "custodianId": "CUST-003",
      "type": "family",
      "name": "Family Trust Services LLC",
      "status": "pending_approval"
    }
  ],
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/custodians"
  }
}
```

### 3.5 Revival Verification Endpoints

#### 3.5.1 Submit Medical Verification

```http
POST /registries/{registryId}/revival/medical-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "medicalDirector": {
    "name": "Dr. Sarah Chen, MD",
    "license": "AZ-MED-123456",
    "facility": "Alcor Life Extension Foundation"
  },
  "revivalDate": "2075-06-15T08:30:00Z",
  "vitalSigns": {
    "heartRate": 72,
    "bloodPressure": "120/80",
    "temperature": 36.8,
    "respiratoryRate": 16,
    "oxygenSaturation": 98
  },
  "neurologicalAssessment": {
    "consciousness": "alert_and_oriented",
    "glasgowComaScale": 15,
    "pupilResponse": "normal",
    "motorFunction": "intact",
    "sensoryFunction": "intact"
  },
  "cognitiveAssessment": {
    "memoryRecall": "partial",
    "languageFunction": "intact",
    "executiveFunction": "moderately_impaired",
    "overallScore": 78
  },
  "medicalCertification": {
    "statement": "Patient successfully revived from cryogenic preservation with stable vital signs and recovering neurological function",
    "signature": "-----BEGIN SIGNATURE-----\n...",
    "timestamp": "2075-06-15T12:00:00Z"
  },
  "attachedDocuments": [
    "medical_certification.pdf",
    "neurological_assessment.pdf",
    "vital_signs_report.pdf"
  ]
}
```

**Response (201 Created):**

```json
{
  "verificationId": "VERIFY-MED-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "medical",
  "status": "verified",
  "submittedBy": "Dr. Sarah Chen, MD",
  "submittedAt": "2075-06-15T12:00:00Z",
  "verifiedAt": "2075-06-15T12:00:00Z",
  "links": {
    "self": "/verifications/VERIFY-MED-2075-001",
    "documents": "/verifications/VERIFY-MED-2075-001/documents",
    "registry": "/registries/AR-2025-1734519000-A7F3C9"
  }
}
```

#### 3.5.2 Submit Legal Verification

```http
POST /registries/{registryId}/revival/legal-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "court": {
    "name": "Superior Court of Arizona, Maricopa County",
    "jurisdiction": "Arizona, USA",
    "caseNumber": "CV-2075-012345"
  },
  "judge": {
    "name": "Honorable Michael Johnson",
    "title": "Superior Court Judge"
  },
  "orderDetails": {
    "orderType": "restoration_of_legal_status",
    "orderDate": "2075-06-20T10:00:00Z",
    "findings": [
      "Individual successfully revived from cryogenic preservation",
      "Medical evidence confirms stable condition",
      "Identity verified through biometric comparison",
      "Mental competency evaluation shows capacity to manage affairs"
    ],
    "orders": [
      "Legal status restored effective immediately",
      "Death certificate vacated",
      "Rights to property and assets reinstated",
      "Authorization to access and control previously registered assets"
    ]
  },
  "courtOrder": {
    "documentHash": "sha256:c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1b2c3d4",
    "signature": "-----BEGIN COURT ORDER-----\n...",
    "seal": "data:image/png;base64,..."
  },
  "attachedDocuments": [
    "court_order.pdf",
    "legal_findings.pdf",
    "death_certificate_vacation.pdf"
  ]
}
```

**Response (201 Created):**

```json
{
  "verificationId": "VERIFY-LEGAL-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "legal",
  "status": "verified",
  "court": "Superior Court of Arizona, Maricopa County",
  "caseNumber": "CV-2075-012345",
  "submittedAt": "2075-06-20T14:00:00Z",
  "verifiedAt": "2075-06-20T14:00:00Z",
  "links": {
    "self": "/verifications/VERIFY-LEGAL-2075-001",
    "documents": "/verifications/VERIFY-LEGAL-2075-001/documents",
    "registry": "/registries/AR-2025-1734519000-A7F3C9"
  }
}
```

#### 3.5.3 Submit Biometric Verification

```http
POST /registries/{registryId}/revival/biometric-verification
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "verificationAuthority": {
    "name": "WIA Biometric Certification Center",
    "certificationId": "WIA-BIO-CERT-001",
    "technician": "Dr. Robert Martinez"
  },
  "biometricSamples": [
    {
      "type": "fingerprint",
      "samples": 10,
      "quality": 94,
      "hash": "sha3-512:a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6...",
      "matchScore": 98.7,
      "matchThreshold": 95.0,
      "matched": true
    },
    {
      "type": "iris",
      "samples": 4,
      "quality": 96,
      "hash": "sha3-512:b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7...",
      "matchScore": 99.2,
      "matchThreshold": 95.0,
      "matched": true
    },
    {
      "type": "dna",
      "samples": 1,
      "quality": 99,
      "hash": "sha3-512:c3d4e5f6a7b8c9d0e1f2a3b4c5d6e7f8...",
      "matchScore": 99.99,
      "matchThreshold": 99.9,
      "matched": true
    }
  ],
  "overallMatchScore": 99.3,
  "verificationResult": "positive_match",
  "certification": {
    "statement": "Biometric comparison confirms positive identity match with pre-preservation records",
    "signature": "-----BEGIN SIGNATURE-----\n...",
    "timestamp": "2075-06-18T10:00:00Z"
  }
}
```

**Response (201 Created):**

```json
{
  "verificationId": "VERIFY-BIO-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "type": "biometric",
  "status": "verified",
  "matchScore": 99.3,
  "verificationResult": "positive_match",
  "submittedAt": "2075-06-18T10:00:00Z",
  "verifiedAt": "2075-06-18T10:00:00Z",
  "links": {
    "self": "/verifications/VERIFY-BIO-2075-001",
    "registry": "/registries/AR-2025-1734519000-A7F3C9"
  }
}
```

#### 3.5.4 Get Revival Status

```http
GET /registries/{registryId}/revival/status
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "revivalStatus": "verified_pending_transfer",
  "verifications": {
    "medical": {
      "status": "verified",
      "verificationId": "VERIFY-MED-2075-001",
      "verifiedAt": "2075-06-15T12:00:00Z"
    },
    "legal": {
      "status": "verified",
      "verificationId": "VERIFY-LEGAL-2075-001",
      "verifiedAt": "2075-06-20T14:00:00Z"
    },
    "biometric": {
      "status": "verified",
      "verificationId": "VERIFY-BIO-2075-001",
      "verifiedAt": "2075-06-18T10:00:00Z"
    },
    "mentalCompetency": {
      "status": "verified",
      "verificationId": "VERIFY-COMP-2075-001",
      "verifiedAt": "2075-06-22T09:00:00Z"
    }
  },
  "transferEligibility": {
    "eligible": true,
    "requiredVerifications": 4,
    "completedVerifications": 4,
    "pendingRequirements": []
  },
  "assetTransferPlan": {
    "totalAssets": 71,
    "immediateTransfer": 15,
    "standardTransfer": 42,
    "complexTransfer": 14,
    "estimatedCompletionDate": "2075-09-20"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/revival/status",
    "initiateTransfer": "/registries/AR-2025-1734519000-A7F3C9/revival/transfer"
  }
}
```

#### 3.5.5 Initiate Asset Transfer

```http
POST /registries/{registryId}/revival/transfer
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "transferType": "full_revival_transfer",
  "recipient": {
    "individualId": "CRYO-IND-2025-987654",
    "legalName": "John Michael Anderson",
    "verifiedIdentity": true
  },
  "newAccounts": {
    "bankAccount": {
      "bank": "Chase Bank",
      "accountType": "checking",
      "accountNumber": "****1234"
    },
    "brokerageAccount": {
      "brokerage": "Fidelity Investments",
      "accountNumber": "Z12345678"
    },
    "cryptoWallets": {
      "bitcoin": "bc1q...",
      "ethereum": "0x..."
    }
  },
  "authorization": {
    "medicalDirector": "VERIFY-MED-2075-001",
    "legalAuthority": "VERIFY-LEGAL-2075-001",
    "biometricAuthority": "VERIFY-BIO-2075-001",
    "primaryCustodian": "CUST-001",
    "courtOrder": "CV-2075-012345"
  }
}
```

**Response (202 Accepted):**

```json
{
  "transferId": "TRANSFER-2075-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "status": "processing",
  "initiated": "2075-06-25T10:00:00Z",
  "estimatedCompletion": "2075-09-20T23:59:59Z",
  "assets": {
    "total": 71,
    "transferred": 0,
    "pending": 71,
    "failed": 0
  },
  "phases": [
    {
      "phase": "immediate_transfer",
      "assets": 15,
      "status": "in_progress",
      "estimatedCompletion": "2075-06-27T23:59:59Z"
    },
    {
      "phase": "standard_transfer",
      "assets": 42,
      "status": "pending",
      "estimatedCompletion": "2075-07-25T23:59:59Z"
    },
    {
      "phase": "complex_transfer",
      "assets": 14,
      "status": "pending",
      "estimatedCompletion": "2075-09-20T23:59:59Z"
    }
  ],
  "links": {
    "self": "/transfers/TRANSFER-2075-001",
    "status": "/transfers/TRANSFER-2075-001/status",
    "assets": "/transfers/TRANSFER-2075-001/assets"
  }
}
```

### 3.6 Valuation Endpoints

#### 3.6.1 Get Asset Valuation

```http
GET /assets/{assetId}/valuation
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "assetId": "FA-2025-BTC-001",
  "currentValuation": {
    "amount": 523456.78,
    "currency": "USD",
    "timestamp": "2025-12-18T10:35:00Z",
    "source": "market_rate",
    "exchange": "Coinbase Pro"
  },
  "historicalValuation": [
    {
      "date": "2025-12-01",
      "value": 498234.50,
      "currency": "USD"
    },
    {
      "date": "2025-11-01",
      "value": 467890.25,
      "currency": "USD"
    },
    {
      "date": "2025-10-01",
      "value": 512456.75,
      "currency": "USD"
    }
  ],
  "metrics": {
    "30dayChange": 5.1,
    "90dayChange": 2.2,
    "yearToDateChange": 15.8,
    "volatility": "high"
  },
  "links": {
    "self": "/assets/FA-2025-BTC-001/valuation",
    "asset": "/assets/FA-2025-BTC-001"
  }
}
```

#### 3.6.2 Get Portfolio Valuation

```http
GET /registries/{registryId}/valuation
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "totalValuation": {
    "amount": 5750000.00,
    "currency": "USD",
    "timestamp": "2025-12-18T00:00:00Z"
  },
  "assetBreakdown": {
    "financial": {
      "value": 2145000.00,
      "percentage": 37.3,
      "count": 12
    },
    "realEstate": {
      "value": 2850000.00,
      "percentage": 49.6,
      "count": 3
    },
    "intellectual": {
      "value": 550000.00,
      "percentage": 9.6,
      "count": 2
    },
    "digital": {
      "value": 125000.00,
      "percentage": 2.2,
      "count": 45
    },
    "personal": {
      "value": 65000.00,
      "percentage": 1.1,
      "count": 8
    },
    "business": {
      "value": 15000.00,
      "percentage": 0.3,
      "count": 1
    }
  },
  "performanceMetrics": {
    "yearToDateReturn": 8.5,
    "inceptionReturn": 23.4,
    "preservationExpensesCovered": "18 years"
  },
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/valuation",
    "assets": "/registries/AR-2025-1734519000-A7F3C9/assets"
  }
}
```

### 3.7 Audit and Reporting Endpoints

#### 3.7.1 Get Audit Log

```http
GET /registries/{registryId}/audit?startDate=2025-12-01&endDate=2025-12-18&limit=50
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "registryId": "AR-2025-1734519000-A7F3C9",
  "startDate": "2025-12-01T00:00:00Z",
  "endDate": "2025-12-18T23:59:59Z",
  "total": 127,
  "limit": 50,
  "offset": 0,
  "entries": [
    {
      "eventId": "AUDIT-2025-001234",
      "timestamp": "2025-12-18T14:30:00Z",
      "eventType": "transaction_executed",
      "actor": "CUST-001",
      "actorName": "Trust Company Alpha",
      "resource": "TX-1734519600-A7B8C9D0E1F2",
      "action": "execute_transaction",
      "details": {
        "transactionType": "sell_partial",
        "assetId": "FA-2025-BTC-001",
        "amount": "2.0 BTC",
        "usdValue": 84234.56
      },
      "ipAddress": "203.0.113.45",
      "userAgent": "WIA-Custodian-Client/1.0.0"
    },
    {
      "eventId": "AUDIT-2025-001233",
      "timestamp": "2025-12-18T14:00:00Z",
      "eventType": "transaction_signed",
      "actor": "LEGAL-001",
      "actorName": "Robert Martinez, Esq.",
      "resource": "TX-1734519600-A7B8C9D0E1F2",
      "action": "sign_transaction",
      "details": {
        "signatureNumber": "3 of 3",
        "role": "legal_representative"
      },
      "ipAddress": "198.51.100.78",
      "userAgent": "Mozilla/5.0..."
    }
  ],
  "links": {
    "self": "/registries/AR-2025-1734519000-A7F3C9/audit?startDate=2025-12-01&endDate=2025-12-18&limit=50",
    "next": "/registries/AR-2025-1734519000-A7F3C9/audit?startDate=2025-12-01&endDate=2025-12-18&limit=50&offset=50"
  }
}
```

#### 3.7.2 Generate Report

```http
POST /registries/{registryId}/reports
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "reportType": "annual_custodian_report",
  "period": {
    "startDate": "2025-01-01",
    "endDate": "2025-12-31"
  },
  "sections": [
    "asset_summary",
    "valuation_changes",
    "transactions",
    "income_and_expenses",
    "custodian_fees",
    "tax_information"
  ],
  "format": "pdf"
}
```

**Response (202 Accepted):**

```json
{
  "reportId": "REPORT-2025-001",
  "registryId": "AR-2025-1734519000-A7F3C9",
  "reportType": "annual_custodian_report",
  "status": "generating",
  "requestedAt": "2025-12-18T16:00:00Z",
  "estimatedCompletion": "2025-12-18T16:05:00Z",
  "links": {
    "self": "/reports/REPORT-2025-001",
    "status": "/reports/REPORT-2025-001/status",
    "download": "/reports/REPORT-2025-001/download"
  }
}
```

## 4. Webhooks

### 4.1 Webhook Events

| Event Type | Description | Payload |
|-----------|-------------|---------|
| `registry.created` | New asset registry created | Registry object |
| `registry.updated` | Asset registry modified | Registry object + changes |
| `asset.created` | New asset added | Asset object |
| `asset.updated` | Asset modified | Asset object + changes |
| `asset.valuation_changed` | Significant valuation change (>10%) | Asset + old/new values |
| `transaction.created` | New transaction initiated | Transaction object |
| `transaction.signed` | Transaction signature added | Transaction + signature |
| `transaction.executed` | Transaction completed | Transaction + result |
| `transaction.failed` | Transaction failed | Transaction + error |
| `custodian.added` | New custodian added | Custodian object |
| `custodian.removed` | Custodian removed | Custodian object + reason |
| `revival.medical_verified` | Medical verification submitted | Verification object |
| `revival.legal_verified` | Legal verification submitted | Verification object |
| `revival.biometric_verified` | Biometric verification submitted | Verification object |
| `revival.transfer_initiated` | Asset transfer started | Transfer object |
| `revival.transfer_completed` | Asset transfer finished | Transfer + summary |

### 4.2 Webhook Configuration

```http
POST /webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://custodian.example.com/webhooks/wia-cryo-asset",
  "events": [
    "transaction.created",
    "transaction.executed",
    "asset.valuation_changed",
    "revival.medical_verified"
  ],
  "secret": "whsec_a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6"
}
```

**Response (201 Created):**

```json
{
  "webhookId": "WEBHOOK-001",
  "url": "https://custodian.example.com/webhooks/wia-cryo-asset",
  "events": [
    "transaction.created",
    "transaction.executed",
    "asset.valuation_changed",
    "revival.medical_verified"
  ],
  "status": "active",
  "created": "2025-12-18T17:00:00Z",
  "links": {
    "self": "/webhooks/WEBHOOK-001",
    "test": "/webhooks/WEBHOOK-001/test"
  }
}
```

### 4.3 Webhook Payload Example

```json
{
  "id": "evt_1234567890abcdef",
  "type": "transaction.executed",
  "created": "2025-12-18T14:30:00Z",
  "data": {
    "object": {
      "transactionId": "TX-1734519600-A7B8C9D0E1F2",
      "assetId": "FA-2025-BTC-001",
      "registryId": "AR-2025-1734519000-A7F3C9",
      "type": "transfer",
      "action": "sell_partial",
      "status": "executed",
      "amount": {
        "value": "2.0",
        "unit": "BTC",
        "usdValue": 84234.56
      },
      "executedAt": "2025-12-18T14:30:00Z"
    }
  },
  "webhookId": "WEBHOOK-001"
}
```

## 5. Error Handling

### 5.1 Error Response Format

```json
{
  "error": {
    "code": "insufficient_signatures",
    "message": "Transaction requires 3 signatures but only 2 provided",
    "details": {
      "requiredSignatures": 3,
      "currentSignatures": 2,
      "missingRoles": ["legal_representative"]
    },
    "requestId": "req_a1b2c3d4e5f6",
    "timestamp": "2025-12-18T15:00:00Z",
    "documentation": "https://docs.wia.dev/errors/insufficient_signatures"
  }
}
```

### 5.2 Error Codes

| HTTP Status | Error Code | Description |
|------------|-----------|-------------|
| 400 | `invalid_request` | Malformed request body or parameters |
| 400 | `validation_error` | Request validation failed |
| 401 | `unauthorized` | Missing or invalid authentication |
| 403 | `forbidden` | Insufficient permissions |
| 403 | `insufficient_signatures` | Not enough signatures for operation |
| 404 | `resource_not_found` | Requested resource doesn't exist |
| 409 | `conflict` | Resource state conflict |
| 409 | `duplicate_resource` | Resource already exists |
| 422 | `unprocessable_entity` | Request understood but cannot be processed |
| 429 | `rate_limit_exceeded` | Too many requests |
| 500 | `internal_error` | Server error |
| 503 | `service_unavailable` | Temporary service disruption |

## 6. Rate Limiting

### 6.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1734523200
```

### 6.2 Rate Limit Tiers

| Tier | Requests/Hour | Burst Limit | Use Case |
|------|--------------|-------------|----------|
| Free | 100 | 10 | Testing, development |
| Standard | 1,000 | 50 | Small custodians |
| Professional | 10,000 | 200 | Medium custodians |
| Enterprise | 100,000 | 1,000 | Large institutions |
| Unlimited | No limit | 5,000 | Critical infrastructure |

## 7. Code Examples

### 7.1 Python Client

```python
import requests
import json
from datetime import datetime

class WIACryoAssetClient:
    def __init__(self, api_key, api_secret, base_url="https://api.wia.dev/cryo-asset/v1"):
        self.api_key = api_key
        self.api_secret = api_secret
        self.base_url = base_url
        self.access_token = None

    def authenticate(self):
        response = requests.post(
            f"{self.base_url}/oauth/token",
            json={
                "grant_type": "client_credentials",
                "client_id": self.api_key,
                "client_secret": self.api_secret,
                "scope": "asset:read asset:write transaction:create"
            }
        )
        response.raise_for_status()
        data = response.json()
        self.access_token = data['access_token']
        return self.access_token

    def _headers(self):
        if not self.access_token:
            self.authenticate()
        return {
            "Authorization": f"Bearer {self.access_token}",
            "Content-Type": "application/json"
        }

    def create_registry(self, subject_data, preservation_details):
        response = requests.post(
            f"{self.base_url}/registries",
            headers=self._headers(),
            json={
                "subject": subject_data,
                "preservationDetails": preservation_details
            }
        )
        response.raise_for_status()
        return response.json()

    def add_crypto_asset(self, registry_id, crypto_data):
        response = requests.post(
            f"{self.base_url}/registries/{registry_id}/assets/financial",
            headers=self._headers(),
            json={
                "category": "cryptocurrency",
                "asset": crypto_data
            }
        )
        response.raise_for_status()
        return response.json()

    def create_transaction(self, asset_id, transaction_data):
        response = requests.post(
            f"{self.base_url}/transactions",
            headers=self._headers(),
            json={
                "assetId": asset_id,
                **transaction_data
            }
        )
        response.raise_for_status()
        return response.json()

    def get_registry(self, registry_id):
        response = requests.get(
            f"{self.base_url}/registries/{registry_id}",
            headers=self._headers()
        )
        response.raise_for_status()
        return response.json()

# Example usage
client = WIACryoAssetClient(
    api_key="cryo_custodian_12345",
    api_secret="sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER"
)

# Create registry
registry = client.create_registry(
    subject_data={
        "individualId": "CRYO-IND-2025-987654",
        "legalName": "John Michael Anderson",
        "dateOfBirth": "1985-06-15"
    },
    preservation_details={
        "facility": "Alcor Life Extension Foundation",
        "facilityId": "ALCOR-AZ-001",
        "preservationDate": "2025-12-20T14:00:00Z"
    }
)

print(f"Created registry: {registry['registryId']}")

# Add Bitcoin asset
btc_asset = client.add_crypto_asset(
    registry_id=registry['registryId'],
    crypto_data={
        "name": "Bitcoin Holdings",
        "cryptocurrency": {
            "symbol": "BTC",
            "network": "bitcoin",
            "wallets": [{
                "type": "cold_storage",
                "address": "bc1qxy2kgdygjrsqtzq2n0yrf2493p83kkfjhx0wlh",
                "balance": {"amount": "12.45678901", "unit": "BTC"}
            }]
        }
    }
)

print(f"Added Bitcoin asset: {btc_asset['assetId']}")
```

### 7.2 JavaScript/Node.js Client

```javascript
const axios = require('axios');

class WIACryoAssetClient {
    constructor(apiKey, apiSecret, baseURL = 'https://api.wia.dev/cryo-asset/v1') {
        this.apiKey = apiKey;
        this.apiSecret = apiSecret;
        this.baseURL = baseURL;
        this.accessToken = null;
    }

    async authenticate() {
        const response = await axios.post(`${this.baseURL}/oauth/token`, {
            grant_type: 'client_credentials',
            client_id: this.apiKey,
            client_secret: this.apiSecret,
            scope: 'asset:read asset:write transaction:create'
        });

        this.accessToken = response.data.access_token;
        return this.accessToken;
    }

    async _getHeaders() {
        if (!this.accessToken) {
            await this.authenticate();
        }
        return {
            'Authorization': `Bearer ${this.accessToken}`,
            'Content-Type': 'application/json'
        };
    }

    async createRegistry(subjectData, preservationDetails) {
        const headers = await this._getHeaders();
        const response = await axios.post(
            `${this.baseURL}/registries`,
            {
                subject: subjectData,
                preservationDetails: preservationDetails
            },
            { headers }
        );
        return response.data;
    }

    async getAsset(assetId) {
        const headers = await this._getHeaders();
        const response = await axios.get(
            `${this.baseURL}/assets/${assetId}`,
            { headers }
        );
        return response.data;
    }

    async createTransaction(assetId, transactionData) {
        const headers = await this._getHeaders();
        const response = await axios.post(
            `${this.baseURL}/transactions`,
            {
                assetId: assetId,
                ...transactionData
            },
            { headers }
        );
        return response.data;
    }

    async signTransaction(transactionId, signerId, signature, publicKey) {
        const headers = await this._getHeaders();
        const response = await axios.post(
            `${this.baseURL}/transactions/${transactionId}/sign`,
            {
                signerId: signerId,
                signature: signature,
                publicKey: publicKey
            },
            { headers }
        );
        return response.data;
    }

    async getRevivalStatus(registryId) {
        const headers = await this._getHeaders();
        const response = await axios.get(
            `${this.baseURL}/registries/${registryId}/revival/status`,
            { headers }
        );
        return response.data;
    }
}

// Example usage
(async () => {
    const client = new WIACryoAssetClient(
        'cryo_custodian_12345',
        'sk_REPLACE_WITH_YOUR_API_KEY_PLACEHOLDER'
    );

    // Get asset details
    const asset = await client.getAsset('FA-2025-BTC-001');
    console.log(`Asset: ${asset.asset.name}`);
    console.log(`Value: $${asset.valuation.totalValue}`);

    // Create transaction
    const transaction = await client.createTransaction('FA-2025-BTC-001', {
        type: 'transfer',
        action: 'sell_partial',
        amount: { value: '0.5', unit: 'BTC' },
        purpose: 'preservation_expenses'
    });

    console.log(`Created transaction: ${transaction.transactionId}`);
})();
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
