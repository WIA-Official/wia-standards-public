# WIA Digital Will Standard - Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-18
**Primary Color:** #64748B (Slate)

---

## 1. Introduction

### 1.1 Overview

The WIA Digital Will Standard establishes a comprehensive framework for digital estate planning and will management. Phase 1 defines the core data formats and structures required to represent digital wills, asset inventories, beneficiary designations, and inheritance rules in a standardized, interoperable manner.

### 1.2 Scope

This specification covers:
- Digital asset inventory formats
- Will document structures
- Beneficiary and executor data models
- Access credential secure storage formats
- Conditional trigger and time-lock mechanisms
- Legal document templates
- Multi-jurisdictional compliance metadata

### 1.3 Design Principles

- **Security First**: All sensitive data must support encryption at rest and in transit
- **Legal Validity**: Formats must support legally binding digital signatures and notarization
- **Interoperability**: JSON-based formats for maximum compatibility
- **Privacy**: Support for granular access controls and data minimization
- **Extensibility**: Versioned schemas to accommodate future requirements

---

## 2. Core Data Structures

### 2.1 Digital Will Document

The Digital Will document is the primary container for all estate planning information.

#### 2.1.1 Will Document Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.dev/schemas/digital-will/v1/will-document.json",
  "type": "object",
  "required": [
    "willId",
    "version",
    "testatorInfo",
    "createdAt",
    "lastModified",
    "status"
  ],
  "properties": {
    "willId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the will document"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Semantic version of the will document"
    },
    "testatorInfo": {
      "$ref": "#/$defs/PersonIdentity",
      "description": "Information about the person creating the will"
    },
    "createdAt": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of will creation"
    },
    "lastModified": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp of last modification"
    },
    "status": {
      "type": "string",
      "enum": ["draft", "active", "revoked", "executed", "superseded"],
      "description": "Current status of the will"
    },
    "executionTriggers": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/ExecutionTrigger"
      },
      "description": "Conditions that trigger will execution"
    },
    "digitalAssets": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/DigitalAsset"
      },
      "description": "Inventory of digital assets"
    },
    "beneficiaries": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/Beneficiary"
      },
      "minItems": 1,
      "description": "List of beneficiaries"
    },
    "executors": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/Executor"
      },
      "minItems": 1,
      "description": "Appointed executors"
    },
    "inheritanceRules": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/InheritanceRule"
      },
      "description": "Rules governing asset distribution"
    },
    "legalDocuments": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/LegalDocument"
      },
      "description": "Supporting legal documents"
    },
    "jurisdiction": {
      "$ref": "#/$defs/JurisdictionInfo",
      "description": "Legal jurisdiction information"
    },
    "notarization": {
      "$ref": "#/$defs/NotarizationRecord",
      "description": "Digital notarization information"
    },
    "encryption": {
      "$ref": "#/$defs/EncryptionMetadata",
      "description": "Encryption configuration"
    },
    "smartContracts": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/SmartContractReference"
      },
      "description": "Associated blockchain smart contracts"
    },
    "metadata": {
      "type": "object",
      "description": "Additional metadata"
    }
  },
  "$defs": {
    "PersonIdentity": {
      "type": "object",
      "required": ["fullName", "dateOfBirth"],
      "properties": {
        "fullName": {
          "type": "string",
          "minLength": 1
        },
        "legalName": {
          "type": "string"
        },
        "dateOfBirth": {
          "type": "string",
          "format": "date"
        },
        "nationalId": {
          "type": "string"
        },
        "email": {
          "type": "string",
          "format": "email"
        },
        "phone": {
          "type": "string"
        },
        "address": {
          "$ref": "#/$defs/Address"
        },
        "biometricHash": {
          "type": "string",
          "description": "Hash of biometric identifier"
        }
      }
    },
    "Address": {
      "type": "object",
      "properties": {
        "street": {"type": "string"},
        "city": {"type": "string"},
        "state": {"type": "string"},
        "postalCode": {"type": "string"},
        "country": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        }
      }
    }
  }
}
```

### 2.2 Digital Asset Inventory

Digital assets represent all online accounts, files, cryptocurrency, and digital property.

#### 2.2.1 Digital Asset Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.dev/schemas/digital-will/v1/digital-asset.json",
  "type": "object",
  "required": ["assetId", "assetType", "name"],
  "properties": {
    "assetId": {
      "type": "string",
      "format": "uuid"
    },
    "assetType": {
      "type": "string",
      "enum": [
        "online-account",
        "cryptocurrency",
        "nft",
        "cloud-storage",
        "domain-name",
        "digital-media",
        "social-media",
        "email-account",
        "intellectual-property",
        "gaming-asset",
        "subscription-service",
        "business-asset"
      ]
    },
    "name": {
      "type": "string",
      "description": "Human-readable asset name"
    },
    "description": {
      "type": "string"
    },
    "provider": {
      "type": "string",
      "description": "Service provider or platform name"
    },
    "accountIdentifier": {
      "type": "string",
      "description": "Username, email, or account ID"
    },
    "accessCredentials": {
      "$ref": "#/$defs/EncryptedCredentials",
      "description": "Encrypted access credentials"
    },
    "estimatedValue": {
      "$ref": "#/$defs/MonetaryValue",
      "description": "Estimated monetary value"
    },
    "location": {
      "type": "object",
      "properties": {
        "url": {"type": "string", "format": "uri"},
        "storagePath": {"type": "string"},
        "blockchainAddress": {"type": "string"}
      }
    },
    "recoveryOptions": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/RecoveryOption"
      }
    },
    "transferInstructions": {
      "type": "string",
      "description": "Specific instructions for asset transfer"
    },
    "restrictions": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Legal or technical restrictions on transfer"
    },
    "dependentAssets": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      },
      "description": "References to related assets"
    },
    "metadata": {
      "type": "object",
      "additionalProperties": true
    }
  },
  "$defs": {
    "EncryptedCredentials": {
      "type": "object",
      "required": ["encryptedData", "encryptionMethod"],
      "properties": {
        "encryptedData": {
          "type": "string",
          "description": "Base64-encoded encrypted credential data"
        },
        "encryptionMethod": {
          "type": "string",
          "enum": ["AES-256-GCM", "RSA-4096", "ChaCha20-Poly1305"]
        },
        "keyDerivation": {
          "type": "string",
          "enum": ["PBKDF2", "Argon2id", "scrypt"]
        },
        "salt": {
          "type": "string",
          "description": "Base64-encoded salt"
        },
        "iv": {
          "type": "string",
          "description": "Base64-encoded initialization vector"
        },
        "authTag": {
          "type": "string",
          "description": "Base64-encoded authentication tag"
        }
      }
    },
    "MonetaryValue": {
      "type": "object",
      "required": ["amount", "currency"],
      "properties": {
        "amount": {
          "type": "number",
          "minimum": 0
        },
        "currency": {
          "type": "string",
          "pattern": "^[A-Z]{3}$"
        },
        "asOfDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "RecoveryOption": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["email-recovery", "phone-recovery", "backup-codes", "recovery-key", "social-recovery", "hardware-key"]
        },
        "encryptedDetails": {
          "type": "string"
        }
      }
    }
  }
}
```

### 2.3 Beneficiary Data Model

#### 2.3.1 Beneficiary Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.dev/schemas/digital-will/v1/beneficiary.json",
  "type": "object",
  "required": ["beneficiaryId", "identity", "relationship"],
  "properties": {
    "beneficiaryId": {
      "type": "string",
      "format": "uuid"
    },
    "identity": {
      "$ref": "will-document.json#/$defs/PersonIdentity"
    },
    "relationship": {
      "type": "string",
      "enum": [
        "spouse",
        "child",
        "parent",
        "sibling",
        "grandchild",
        "other-family",
        "friend",
        "charity",
        "organization",
        "trust",
        "other"
      ]
    },
    "isPrimary": {
      "type": "boolean",
      "default": false
    },
    "contingentFor": {
      "type": "string",
      "format": "uuid",
      "description": "Beneficiary ID this is contingent for"
    },
    "allocationPercentage": {
      "type": "number",
      "minimum": 0,
      "maximum": 100
    },
    "specificAssets": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      },
      "description": "Specific asset IDs allocated to this beneficiary"
    },
    "conditions": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/InheritanceCondition"
      }
    },
    "trustConfiguration": {
      "$ref": "#/$defs/TrustConfiguration",
      "description": "Trust settings if assets held in trust"
    },
    "notificationPreferences": {
      "$ref": "#/$defs/NotificationPreferences"
    },
    "verificationStatus": {
      "type": "string",
      "enum": ["unverified", "pending", "verified", "rejected"]
    },
    "verificationMethod": {
      "type": "string",
      "enum": ["document-verification", "biometric", "multi-factor", "legal-attestation"]
    }
  },
  "$defs": {
    "InheritanceCondition": {
      "type": "object",
      "required": ["conditionType"],
      "properties": {
        "conditionType": {
          "type": "string",
          "enum": ["age-requirement", "educational-milestone", "marriage-status", "substance-free", "employment-status", "custom"]
        },
        "parameters": {
          "type": "object"
        },
        "verificationMethod": {
          "type": "string"
        }
      }
    },
    "TrustConfiguration": {
      "type": "object",
      "properties": {
        "trustType": {
          "type": "string",
          "enum": ["discretionary", "fixed-interest", "bare", "mixed"]
        },
        "trusteeId": {
          "type": "string",
          "format": "uuid"
        },
        "distributionSchedule": {
          "$ref": "#/$defs/DistributionSchedule"
        },
        "terminationConditions": {
          "type": "array",
          "items": {
            "type": "string"
          }
        }
      }
    },
    "DistributionSchedule": {
      "type": "object",
      "properties": {
        "scheduleType": {
          "type": "string",
          "enum": ["immediate", "periodic", "milestone-based", "age-based"]
        },
        "intervals": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "triggerDate": {"type": "string", "format": "date"},
              "triggerAge": {"type": "integer"},
              "percentage": {"type": "number"}
            }
          }
        }
      }
    },
    "NotificationPreferences": {
      "type": "object",
      "properties": {
        "email": {"type": "string", "format": "email"},
        "phone": {"type": "string"},
        "encryptedMessage": {"type": "boolean"},
        "preferredLanguage": {"type": "string", "pattern": "^[a-z]{2}$"}
      }
    }
  }
}
```

### 2.4 Executor Data Model

#### 2.4.1 Executor Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.dev/schemas/digital-will/v1/executor.json",
  "type": "object",
  "required": ["executorId", "identity", "role"],
  "properties": {
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "identity": {
      "$ref": "will-document.json#/$defs/PersonIdentity"
    },
    "role": {
      "type": "string",
      "enum": ["primary-executor", "co-executor", "successor-executor", "digital-executor", "trustee"]
    },
    "permissions": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": [
          "access-all-assets",
          "distribute-assets",
          "close-accounts",
          "access-credentials",
          "modify-beneficiaries",
          "execute-smart-contracts",
          "legal-representation",
          "financial-transactions"
        ]
      }
    },
    "scope": {
      "type": "string",
      "enum": ["all-assets", "digital-only", "specific-assets"],
      "default": "all-assets"
    },
    "specificAssets": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "activationConditions": {
      "type": "array",
      "items": {
        "$ref": "#/$defs/ActivationCondition"
      }
    },
    "compensation": {
      "$ref": "digital-asset.json#/$defs/MonetaryValue"
    },
    "bondRequirement": {
      "type": "boolean",
      "default": false
    },
    "reportingRequirements": {
      "type": "object",
      "properties": {
        "frequency": {
          "type": "string",
          "enum": ["weekly", "monthly", "quarterly", "annually", "on-demand"]
        },
        "recipients": {
          "type": "array",
          "items": {
            "type": "string",
            "format": "uuid"
          }
        }
      }
    },
    "successorExecutor": {
      "type": "string",
      "format": "uuid"
    },
    "acceptanceStatus": {
      "type": "string",
      "enum": ["pending", "accepted", "declined", "revoked"]
    },
    "acceptanceDate": {
      "type": "string",
      "format": "date-time"
    },
    "digitalSignature": {
      "$ref": "#/$defs/DigitalSignature"
    }
  },
  "$defs": {
    "ActivationCondition": {
      "type": "object",
      "properties": {
        "conditionType": {
          "type": "string",
          "enum": ["death-certificate", "incapacitation", "time-based", "co-executor-approval", "court-order"]
        },
        "verificationMethod": {
          "type": "string"
        },
        "requiredDocuments": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "DigitalSignature": {
      "type": "object",
      "required": ["signature", "algorithm", "timestamp"],
      "properties": {
        "signature": {
          "type": "string",
          "description": "Base64-encoded signature"
        },
        "algorithm": {
          "type": "string",
          "enum": ["RSA-SHA256", "ECDSA-SHA256", "EdDSA"]
        },
        "publicKey": {
          "type": "string"
        },
        "timestamp": {
          "type": "string",
          "format": "date-time"
        },
        "certificateChain": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    }
  }
}
```

### 2.5 Execution Triggers and Time-Locks

#### 2.5.1 Execution Trigger Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wia.dev/schemas/digital-will/v1/execution-trigger.json",
  "type": "object",
  "required": ["triggerId", "triggerType"],
  "properties": {
    "triggerId": {
      "type": "string",
      "format": "uuid"
    },
    "triggerType": {
      "type": "string",
      "enum": [
        "death-verification",
        "incapacitation",
        "time-lock",
        "inactivity-period",
        "manual-activation",
        "smart-contract-event",
        "multi-signature",
        "oracle-verification"
      ]
    },
    "configuration": {
      "oneOf": [
        {"$ref": "#/$defs/DeathVerificationConfig"},
        {"$ref": "#/$defs/TimeLockConfig"},
        {"$ref": "#/$defs/InactivityConfig"},
        {"$ref": "#/$defs/SmartContractConfig"}
      ]
    },
    "verificationRequirements": {
      "type": "object",
      "properties": {
        "minimumProofs": {
          "type": "integer",
          "minimum": 1
        },
        "proofTypes": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["death-certificate", "medical-report", "court-order", "executor-attestation", "oracle-data", "blockchain-proof"]
          }
        },
        "verificationPeriod": {
          "type": "integer",
          "description": "Hours to verify before execution"
        }
      }
    },
    "delayPeriod": {
      "type": "integer",
      "description": "Hours to delay after trigger before execution"
    },
    "notificationList": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      },
      "description": "Beneficiary/executor IDs to notify"
    },
    "revocationPeriod": {
      "type": "integer",
      "description": "Hours during which trigger can be revoked"
    },
    "priority": {
      "type": "integer",
      "minimum": 1,
      "maximum": 10,
      "default": 5
    }
  },
  "$defs": {
    "DeathVerificationConfig": {
      "type": "object",
      "properties": {
        "acceptedSources": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["government-registry", "medical-examiner", "funeral-home", "trusted-oracle", "blockchain-oracle"]
          }
        },
        "jurisdictionRequirements": {
          "type": "object"
        }
      }
    },
    "TimeLockConfig": {
      "type": "object",
      "required": ["unlockDate"],
      "properties": {
        "unlockDate": {
          "type": "string",
          "format": "date-time"
        },
        "allowEarlyUnlock": {
          "type": "boolean",
          "default": false
        },
        "earlyUnlockConditions": {
          "type": "array",
          "items": {"type": "string"}
        }
      }
    },
    "InactivityConfig": {
      "type": "object",
      "required": ["inactivityPeriodDays"],
      "properties": {
        "inactivityPeriodDays": {
          "type": "integer",
          "minimum": 1
        },
        "checkInMethods": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["email-response", "app-check-in", "biometric-verification", "blockchain-transaction"]
          }
        },
        "warningPeriodDays": {
          "type": "integer",
          "description": "Days before trigger to send warning"
        },
        "escalationContacts": {
          "type": "array",
          "items": {
            "type": "string",
            "format": "uuid"
          }
        }
      }
    },
    "SmartContractConfig": {
      "type": "object",
      "required": ["contractAddress", "network"],
      "properties": {
        "contractAddress": {
          "type": "string"
        },
        "network": {
          "type": "string",
          "enum": ["ethereum", "polygon", "binance-smart-chain", "solana", "cardano", "avalanche"]
        },
        "eventName": {
          "type": "string"
        },
        "eventParameters": {
          "type": "object"
        }
      }
    }
  }
}
```

---

## 3. Data Format Tables

### 3.1 Asset Type Classification

| Asset Type | Description | Transferability | Special Considerations |
|------------|-------------|-----------------|------------------------|
| online-account | General online service accounts | Platform-dependent | Check Terms of Service for inheritance clauses |
| cryptocurrency | Digital currencies (BTC, ETH, etc.) | High | Requires private key management |
| nft | Non-fungible tokens | High | Smart contract-based transfer |
| cloud-storage | Files in cloud storage services | Medium | May require platform cooperation |
| domain-name | Registered domain names | High | Transfer through registrar |
| digital-media | Music, videos, ebooks | Low | Often non-transferable due to DRM |
| social-media | Social media accounts | Low | Most platforms prohibit transfer |
| email-account | Email service accounts | Low | Platform-specific policies |
| intellectual-property | Digital IP rights | High | Requires legal documentation |
| gaming-asset | In-game items and accounts | Low | Usually against TOS to transfer |
| subscription-service | Ongoing subscription services | Low | Typically non-transferable |
| business-asset | Digital business assets | High | May require business succession planning |

### 3.2 Encryption Methods Comparison

| Method | Key Size | Performance | Security Level | Use Case |
|--------|----------|-------------|----------------|----------|
| AES-256-GCM | 256-bit | Very Fast | Very High | General credential encryption |
| RSA-4096 | 4096-bit | Slow | Very High | Asymmetric key encryption |
| ChaCha20-Poly1305 | 256-bit | Very Fast | Very High | Mobile/embedded systems |
| PBKDF2 | Variable | Medium | High | Key derivation |
| Argon2id | Variable | Slow | Very High | Password hashing (memory-hard) |
| scrypt | Variable | Slow | High | Legacy key derivation |

### 3.3 Trigger Verification Requirements

| Trigger Type | Minimum Proofs | Verification Period | Revocation Period | Priority |
|--------------|----------------|---------------------|-------------------|----------|
| death-verification | 2 | 72 hours | 30 days | 10 |
| incapacitation | 1 | 24 hours | 15 days | 8 |
| time-lock | 0 | Immediate | 7 days | 5 |
| inactivity-period | 0 | 24 hours | 14 days | 6 |
| manual-activation | 1 | 1 hour | 3 days | 3 |
| smart-contract-event | 1 | Immediate | 7 days | 7 |
| multi-signature | 2+ | 48 hours | 10 days | 9 |
| oracle-verification | 2 | 48 hours | 14 days | 8 |

### 3.4 Jurisdiction Compliance Matrix

| Region | Digital Signature Recognition | Notarization Requirement | Witness Requirement | Asset Transfer Laws |
|--------|------------------------------|--------------------------|---------------------|---------------------|
| United States | State-dependent | Recommended | 2 witnesses | State-specific probate |
| European Union | eIDAS compliant | Optional | 1-2 witnesses (country-dependent) | EU Succession Regulation |
| United Kingdom | Electronic Communications Act | Optional | 2 witnesses | Wills Act 1837 (digital amendments) |
| Canada | Provincial variation | Recommended | 2 witnesses | Provincial estate laws |
| Australia | Yes (Electronic Transactions Act) | Optional | 2 witnesses | State-based succession |
| Singapore | Electronic Transactions Act | Optional | 2 witnesses | Wills Act (Cap. 352) |
| Japan | Limited recognition | Required | 2 witnesses | Civil Code provisions |
| South Korea | Digital Signature Act | Required | 2 witnesses | Civil Act Article 1060 |

---

## 4. Code Examples

### 4.1 Creating a Digital Will Document

```javascript
const crypto = require('crypto');
const { v4: uuidv4 } = require('uuid');

class DigitalWillCreator {
  constructor() {
    this.willDocument = null;
  }

  createNewWill(testatorInfo) {
    this.willDocument = {
      willId: uuidv4(),
      version: "1.0.0",
      testatorInfo: {
        fullName: testatorInfo.fullName,
        legalName: testatorInfo.legalName,
        dateOfBirth: testatorInfo.dateOfBirth,
        nationalId: this.hashSensitiveData(testatorInfo.nationalId),
        email: testatorInfo.email,
        phone: testatorInfo.phone,
        address: testatorInfo.address
      },
      createdAt: new Date().toISOString(),
      lastModified: new Date().toISOString(),
      status: "draft",
      executionTriggers: [],
      digitalAssets: [],
      beneficiaries: [],
      executors: [],
      inheritanceRules: [],
      legalDocuments: [],
      jurisdiction: testatorInfo.jurisdiction,
      metadata: {}
    };

    return this.willDocument;
  }

  addDigitalAsset(assetData, credentials) {
    const asset = {
      assetId: uuidv4(),
      assetType: assetData.assetType,
      name: assetData.name,
      description: assetData.description,
      provider: assetData.provider,
      accountIdentifier: assetData.accountIdentifier,
      accessCredentials: this.encryptCredentials(credentials),
      estimatedValue: assetData.estimatedValue,
      location: assetData.location,
      transferInstructions: assetData.transferInstructions,
      metadata: assetData.metadata || {}
    };

    this.willDocument.digitalAssets.push(asset);
    this.updateLastModified();
    return asset;
  }

  encryptCredentials(credentials) {
    const algorithm = 'aes-256-gcm';
    const key = crypto.randomBytes(32);
    const iv = crypto.randomBytes(16);
    const salt = crypto.randomBytes(32);

    // In production, derive key from master password
    const derivedKey = crypto.pbkdf2Sync(
      key,
      salt,
      100000,
      32,
      'sha256'
    );

    const cipher = crypto.createCipheriv(algorithm, derivedKey, iv);

    let encrypted = cipher.update(
      JSON.stringify(credentials),
      'utf8',
      'base64'
    );
    encrypted += cipher.final('base64');

    const authTag = cipher.getAuthTag();

    return {
      encryptedData: encrypted,
      encryptionMethod: 'AES-256-GCM',
      keyDerivation: 'PBKDF2',
      salt: salt.toString('base64'),
      iv: iv.toString('base64'),
      authTag: authTag.toString('base64')
    };
  }

  hashSensitiveData(data) {
    return crypto
      .createHash('sha256')
      .update(data)
      .digest('hex');
  }

  updateLastModified() {
    this.willDocument.lastModified = new Date().toISOString();
  }

  exportWill() {
    return JSON.stringify(this.willDocument, null, 2);
  }
}

// Usage example
const creator = new DigitalWillCreator();
const will = creator.createNewWill({
  fullName: "John Smith",
  legalName: "John Michael Smith",
  dateOfBirth: "1980-05-15",
  nationalId: "123-45-6789",
  email: "john.smith@example.com",
  phone: "+1-555-0123",
  address: {
    street: "123 Main St",
    city: "Springfield",
    state: "IL",
    postalCode: "62701",
    country: "US"
  },
  jurisdiction: {
    country: "US",
    state: "IL"
  }
});

creator.addDigitalAsset({
  assetType: "cryptocurrency",
  name: "Bitcoin Wallet",
  description: "Primary BTC wallet",
  provider: "Hardware Wallet",
  accountIdentifier: "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa",
  estimatedValue: {
    amount: 50000,
    currency: "USD",
    asOfDate: "2025-12-18"
  },
  location: {
    blockchainAddress: "1A1zP1eP5QGefi2DMPTfTL5SLmv7DivfNa"
  },
  transferInstructions: "Use seed phrase to recover wallet"
}, {
  seedPhrase: "witch collapse practice feed shame open despair creek road again ice least",
  pin: "1234"
});

console.log(creator.exportWill());
```

### 4.2 Adding Beneficiaries with Inheritance Rules

```python
import uuid
from datetime import datetime
from typing import List, Dict, Optional

class BeneficiaryManager:
    def __init__(self, will_document: Dict):
        self.will_document = will_document

    def add_beneficiary(
        self,
        identity: Dict,
        relationship: str,
        allocation_percentage: Optional[float] = None,
        specific_assets: Optional[List[str]] = None,
        conditions: Optional[List[Dict]] = None
    ) -> Dict:
        """Add a beneficiary to the will"""

        beneficiary = {
            "beneficiaryId": str(uuid.uuid4()),
            "identity": identity,
            "relationship": relationship,
            "isPrimary": True,
            "contingentFor": None,
            "allocationPercentage": allocation_percentage,
            "specificAssets": specific_assets or [],
            "conditions": conditions or [],
            "verificationStatus": "unverified",
            "notificationPreferences": {
                "email": identity.get("email"),
                "encryptedMessage": True,
                "preferredLanguage": "en"
            }
        }

        self.will_document["beneficiaries"].append(beneficiary)
        self._update_last_modified()

        return beneficiary

    def add_contingent_beneficiary(
        self,
        primary_beneficiary_id: str,
        identity: Dict,
        relationship: str
    ) -> Dict:
        """Add a contingent (backup) beneficiary"""

        beneficiary = self.add_beneficiary(
            identity=identity,
            relationship=relationship
        )

        beneficiary["isPrimary"] = False
        beneficiary["contingentFor"] = primary_beneficiary_id

        return beneficiary

    def create_age_based_distribution(
        self,
        beneficiary_id: str,
        distribution_schedule: List[Dict]
    ) -> Dict:
        """Create age-based distribution trust"""

        trust_config = {
            "trustType": "age-based",
            "trusteeId": None,  # To be assigned
            "distributionSchedule": {
                "scheduleType": "age-based",
                "intervals": distribution_schedule
            },
            "terminationConditions": [
                "All funds distributed",
                "Beneficiary reaches final age milestone"
            ]
        }

        # Find and update beneficiary
        for beneficiary in self.will_document["beneficiaries"]:
            if beneficiary["beneficiaryId"] == beneficiary_id:
                beneficiary["trustConfiguration"] = trust_config
                break

        self._update_last_modified()
        return trust_config

    def add_inheritance_condition(
        self,
        beneficiary_id: str,
        condition_type: str,
        parameters: Dict
    ) -> Dict:
        """Add conditional requirements for inheritance"""

        condition = {
            "conditionType": condition_type,
            "parameters": parameters,
            "verificationMethod": self._get_verification_method(condition_type)
        }

        for beneficiary in self.will_document["beneficiaries"]:
            if beneficiary["beneficiaryId"] == beneficiary_id:
                beneficiary["conditions"].append(condition)
                break

        self._update_last_modified()
        return condition

    def _get_verification_method(self, condition_type: str) -> str:
        """Determine verification method based on condition type"""
        verification_map = {
            "age-requirement": "government-id-verification",
            "educational-milestone": "diploma-certificate-verification",
            "marriage-status": "legal-document-verification",
            "substance-free": "testing-verification",
            "employment-status": "employer-verification"
        }
        return verification_map.get(condition_type, "manual-verification")

    def _update_last_modified(self):
        self.will_document["lastModified"] = datetime.utcnow().isoformat()

# Usage example
will_doc = {
    "willId": str(uuid.uuid4()),
    "beneficiaries": [],
    "lastModified": datetime.utcnow().isoformat()
}

manager = BeneficiaryManager(will_doc)

# Add primary beneficiary (spouse)
spouse = manager.add_beneficiary(
    identity={
        "fullName": "Jane Smith",
        "dateOfBirth": "1982-08-20",
        "email": "jane.smith@example.com",
        "address": {
            "street": "123 Main St",
            "city": "Springfield",
            "state": "IL",
            "postalCode": "62701",
            "country": "US"
        }
    },
    relationship="spouse",
    allocation_percentage=50.0
)

# Add child beneficiary with age-based trust
child = manager.add_beneficiary(
    identity={
        "fullName": "Emily Smith",
        "dateOfBirth": "2010-03-12",
        "email": "emily.guardian@example.com"
    },
    relationship="child",
    allocation_percentage=50.0
)

# Set up age-based distribution
manager.create_age_based_distribution(
    beneficiary_id=child["beneficiaryId"],
    distribution_schedule=[
        {"triggerAge": 18, "percentage": 25.0},
        {"triggerAge": 25, "percentage": 35.0},
        {"triggerAge": 30, "percentage": 40.0}
    ]
)

# Add educational condition
manager.add_inheritance_condition(
    beneficiary_id=child["beneficiaryId"],
    condition_type="educational-milestone",
    parameters={
        "minimumDegree": "bachelor",
        "acceptableFields": ["any"]
    }
)

# Add contingent beneficiary
manager.add_contingent_beneficiary(
    primary_beneficiary_id=spouse["beneficiaryId"],
    identity={
        "fullName": "Robert Smith",
        "dateOfBirth": "1955-12-05",
        "relationship": "parent"
    },
    relationship="parent"
)

print(f"Added {len(will_doc['beneficiaries'])} beneficiaries")
```

### 4.3 Implementing Execution Triggers

```typescript
import { EventEmitter } from 'events';
import * as crypto from 'crypto';

interface ExecutionTrigger {
  triggerId: string;
  triggerType: string;
  configuration: any;
  verificationRequirements: {
    minimumProofs: number;
    proofTypes: string[];
    verificationPeriod: number;
  };
  delayPeriod: number;
  notificationList: string[];
  revocationPeriod: number;
  priority: number;
  status?: 'pending' | 'active' | 'verified' | 'executed' | 'revoked';
  activatedAt?: string;
  verifications?: Verification[];
}

interface Verification {
  verificationId: string;
  proofType: string;
  providedBy: string;
  timestamp: string;
  verificationData: any;
  digitalSignature: string;
}

class ExecutionTriggerManager extends EventEmitter {
  private triggers: Map<string, ExecutionTrigger>;
  private verifications: Map<string, Verification[]>;

  constructor() {
    super();
    this.triggers = new Map();
    this.verifications = new Map();
  }

  createDeathVerificationTrigger(
    notificationList: string[],
    acceptedSources: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'death-verification',
      configuration: {
        acceptedSources: acceptedSources,
        jurisdictionRequirements: {}
      },
      verificationRequirements: {
        minimumProofs: 2,
        proofTypes: ['death-certificate', 'medical-examiner'],
        verificationPeriod: 72 // hours
      },
      delayPeriod: 168, // 7 days
      notificationList: notificationList,
      revocationPeriod: 720, // 30 days
      priority: 10,
      status: 'pending'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    return trigger;
  }

  createInactivityTrigger(
    inactivityPeriodDays: number,
    warningPeriodDays: number,
    checkInMethods: string[],
    escalationContacts: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'inactivity-period',
      configuration: {
        inactivityPeriodDays: inactivityPeriodDays,
        checkInMethods: checkInMethods,
        warningPeriodDays: warningPeriodDays,
        escalationContacts: escalationContacts,
        lastCheckIn: new Date().toISOString()
      },
      verificationRequirements: {
        minimumProofs: 0,
        proofTypes: [],
        verificationPeriod: 24
      },
      delayPeriod: 336, // 14 days
      notificationList: escalationContacts,
      revocationPeriod: 336,
      priority: 6,
      status: 'active'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    // Start monitoring inactivity
    this.startInactivityMonitoring(trigger);

    return trigger;
  }

  createTimeLockTrigger(
    unlockDate: Date,
    notificationList: string[]
  ): ExecutionTrigger {
    const trigger: ExecutionTrigger = {
      triggerId: this.generateId(),
      triggerType: 'time-lock',
      configuration: {
        unlockDate: unlockDate.toISOString(),
        allowEarlyUnlock: false,
        earlyUnlockConditions: []
      },
      verificationRequirements: {
        minimumProofs: 0,
        proofTypes: [],
        verificationPeriod: 0
      },
      delayPeriod: 0,
      notificationList: notificationList,
      revocationPeriod: 168, // 7 days
      priority: 5,
      status: 'active'
    };

    this.triggers.set(trigger.triggerId, trigger);
    this.verifications.set(trigger.triggerId, []);

    // Schedule automatic execution
    this.scheduleTimeLockExecution(trigger);

    return trigger;
  }

  async submitVerification(
    triggerId: string,
    proofType: string,
    verificationData: any,
    providedBy: string,
    privateKey: string
  ): Promise<boolean> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) {
      throw new Error('Trigger not found');
    }

    // Verify proof type is accepted
    if (!trigger.verificationRequirements.proofTypes.includes(proofType)) {
      throw new Error('Proof type not accepted for this trigger');
    }

    // Create verification record
    const verification: Verification = {
      verificationId: this.generateId(),
      proofType: proofType,
      providedBy: providedBy,
      timestamp: new Date().toISOString(),
      verificationData: verificationData,
      digitalSignature: this.signVerification(verificationData, privateKey)
    };

    // Add verification
    const verifications = this.verifications.get(triggerId) || [];
    verifications.push(verification);
    this.verifications.set(triggerId, verifications);

    // Check if verification threshold met
    if (verifications.length >= trigger.verificationRequirements.minimumProofs) {
      await this.activateTrigger(triggerId);
      return true;
    }

    this.emit('verification-submitted', {
      triggerId,
      verification,
      remaining: trigger.verificationRequirements.minimumProofs - verifications.length
    });

    return false;
  }

  private async activateTrigger(triggerId: string): Promise<void> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return;

    trigger.status = 'verified';
    trigger.activatedAt = new Date().toISOString();

    // Send notifications
    await this.sendNotifications(trigger);

    // Wait for revocation period
    setTimeout(() => {
      if (trigger.status === 'verified') {
        this.executeTrigger(triggerId);
      }
    }, trigger.revocationPeriod * 60 * 60 * 1000);

    this.emit('trigger-activated', { trigger });
  }

  private async executeTrigger(triggerId: string): Promise<void> {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return;

    // Apply delay period
    setTimeout(() => {
      trigger.status = 'executed';
      this.emit('trigger-executed', { trigger });

      // Initiate will execution process
      this.initiateWillExecution(trigger);
    }, trigger.delayPeriod * 60 * 60 * 1000);
  }

  private startInactivityMonitoring(trigger: ExecutionTrigger): void {
    const checkInterval = 24 * 60 * 60 * 1000; // Daily check

    setInterval(() => {
      const lastCheckIn = new Date(trigger.configuration.lastCheckIn);
      const daysSinceCheckIn =
        (Date.now() - lastCheckIn.getTime()) / (24 * 60 * 60 * 1000);

      if (daysSinceCheckIn >= trigger.configuration.warningPeriodDays) {
        this.emit('inactivity-warning', {
          triggerId: trigger.triggerId,
          daysSinceCheckIn: daysSinceCheckIn,
          daysUntilTrigger:
            trigger.configuration.inactivityPeriodDays - daysSinceCheckIn
        });
      }

      if (daysSinceCheckIn >= trigger.configuration.inactivityPeriodDays) {
        this.activateTrigger(trigger.triggerId);
      }
    }, checkInterval);
  }

  private scheduleTimeLockExecution(trigger: ExecutionTrigger): void {
    const unlockDate = new Date(trigger.configuration.unlockDate);
    const delay = unlockDate.getTime() - Date.now();

    if (delay > 0) {
      setTimeout(() => {
        this.activateTrigger(trigger.triggerId);
      }, delay);
    }
  }

  private async sendNotifications(trigger: ExecutionTrigger): Promise<void> {
    // Implementation would send notifications to all parties
    this.emit('notifications-sent', {
      triggerId: trigger.triggerId,
      recipients: trigger.notificationList
    });
  }

  private initiateWillExecution(trigger: ExecutionTrigger): void {
    // Implementation would begin the will execution process
    this.emit('will-execution-initiated', { trigger });
  }

  private signVerification(data: any, privateKey: string): string {
    const sign = crypto.createSign('RSA-SHA256');
    sign.update(JSON.stringify(data));
    return sign.sign(privateKey, 'base64');
  }

  private generateId(): string {
    return crypto.randomBytes(16).toString('hex');
  }

  recordCheckIn(triggerId: string): void {
    const trigger = this.triggers.get(triggerId);
    if (trigger && trigger.triggerType === 'inactivity-period') {
      trigger.configuration.lastCheckIn = new Date().toISOString();
      this.emit('check-in-recorded', { triggerId });
    }
  }

  revokeTrigger(triggerId: string): boolean {
    const trigger = this.triggers.get(triggerId);
    if (!trigger) return false;

    if (trigger.status === 'verified' || trigger.status === 'active') {
      trigger.status = 'revoked';
      this.emit('trigger-revoked', { triggerId });
      return true;
    }

    return false;
  }
}

// Usage example
const triggerManager = new ExecutionTriggerManager();

// Create death verification trigger
const deathTrigger = triggerManager.createDeathVerificationTrigger(
  ['executor-id-1', 'beneficiary-id-1', 'beneficiary-id-2'],
  ['government-registry', 'medical-examiner', 'funeral-home']
);

// Create inactivity trigger
const inactivityTrigger = triggerManager.createInactivityTrigger(
  180, // 6 months
  150, // Warning at 5 months
  ['email-response', 'app-check-in'],
  ['trusted-contact-id-1', 'executor-id-1']
);

// Create time-lock trigger
const timeLockTrigger = triggerManager.createTimeLockTrigger(
  new Date('2030-01-01'),
  ['beneficiary-id-3']
);

// Listen for events
triggerManager.on('trigger-activated', (event) => {
  console.log('Trigger activated:', event.trigger.triggerId);
});

triggerManager.on('will-execution-initiated', (event) => {
  console.log('Will execution initiated by trigger:', event.trigger.triggerId);
});

console.log('Triggers created successfully');
```

### 4.4 Smart Contract Integration

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.20;

import "@openzeppelin/contracts/access/AccessControl.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";

/**
 * @title DigitalWillExecutor
 * @dev Smart contract for managing digital will execution on blockchain
 */
contract DigitalWillExecutor is AccessControl, ReentrancyGuard {
    using ECDSA for bytes32;

    bytes32 public constant EXECUTOR_ROLE = keccak256("EXECUTOR_ROLE");
    bytes32 public constant ORACLE_ROLE = keccak256("ORACLE_ROLE");

    struct Will {
        bytes32 willId;
        address testator;
        uint256 createdAt;
        uint256 lastModified;
        WillStatus status;
        bytes32 contentHash; // IPFS hash of will document
        uint256 executionDelay;
        uint256 revocationPeriod;
    }

    struct Beneficiary {
        address beneficiaryAddress;
        uint256 allocationPercentage; // Basis points (10000 = 100%)
        BeneficiaryType beneficiaryType;
        bool verified;
    }

    struct ExecutionTrigger {
        bytes32 triggerId;
        TriggerType triggerType;
        uint256 activatedAt;
        uint256 verificationCount;
        uint256 requiredVerifications;
        TriggerStatus status;
    }

    enum WillStatus { Draft, Active, Triggered, Executed, Revoked }
    enum TriggerStatus { Pending, Active, Verified, Executed, Revoked }
    enum TriggerType { DeathVerification, Inactivity, TimeLock, Manual }
    enum BeneficiaryType { Primary, Contingent, Trust }

    mapping(bytes32 => Will) public wills;
    mapping(bytes32 => Beneficiary[]) public willBeneficiaries;
    mapping(bytes32 => ExecutionTrigger) public executionTriggers;
    mapping(bytes32 => mapping(address => bool)) public hasVerified;
    mapping(address => bytes32[]) public testatorWills;

    event WillCreated(bytes32 indexed willId, address indexed testator);
    event WillUpdated(bytes32 indexed willId, bytes32 newContentHash);
    event BeneficiaryAdded(bytes32 indexed willId, address indexed beneficiary);
    event TriggerActivated(bytes32 indexed willId, bytes32 indexed triggerId);
    event VerificationSubmitted(bytes32 indexed triggerId, address indexed verifier);
    event WillExecuted(bytes32 indexed willId, uint256 timestamp);
    event WillRevoked(bytes32 indexed willId, uint256 timestamp);

    constructor() {
        _grantRole(DEFAULT_ADMIN_ROLE, msg.sender);
    }

    /**
     * @dev Create a new digital will
     */
    function createWill(
        bytes32 _willId,
        bytes32 _contentHash,
        uint256 _executionDelay,
        uint256 _revocationPeriod
    ) external {
        require(wills[_willId].testator == address(0), "Will already exists");

        wills[_willId] = Will({
            willId: _willId,
            testator: msg.sender,
            createdAt: block.timestamp,
            lastModified: block.timestamp,
            status: WillStatus.Draft,
            contentHash: _contentHash,
            executionDelay: _executionDelay,
            revocationPeriod: _revocationPeriod
        });

        testatorWills[msg.sender].push(_willId);
        emit WillCreated(_willId, msg.sender);
    }

    /**
     * @dev Add beneficiary to will
     */
    function addBeneficiary(
        bytes32 _willId,
        address _beneficiary,
        uint256 _allocationPercentage,
        BeneficiaryType _beneficiaryType
    ) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(
            wills[_willId].status == WillStatus.Draft ||
            wills[_willId].status == WillStatus.Active,
            "Cannot modify will"
        );

        willBeneficiaries[_willId].push(Beneficiary({
            beneficiaryAddress: _beneficiary,
            allocationPercentage: _allocationPercentage,
            beneficiaryType: _beneficiaryType,
            verified: false
        }));

        wills[_willId].lastModified = block.timestamp;
        emit BeneficiaryAdded(_willId, _beneficiary);
    }

    /**
     * @dev Activate will (make it legally binding)
     */
    function activateWill(bytes32 _willId) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(wills[_willId].status == WillStatus.Draft, "Will not in draft");
        require(validateBeneficiaries(_willId), "Invalid beneficiary allocation");

        wills[_willId].status = WillStatus.Active;
        wills[_willId].lastModified = block.timestamp;
    }

    /**
     * @dev Create execution trigger
     */
    function createExecutionTrigger(
        bytes32 _willId,
        bytes32 _triggerId,
        TriggerType _triggerType,
        uint256 _requiredVerifications
    ) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(wills[_willId].status == WillStatus.Active, "Will not active");

        executionTriggers[_triggerId] = ExecutionTrigger({
            triggerId: _triggerId,
            triggerType: _triggerType,
            activatedAt: 0,
            verificationCount: 0,
            requiredVerifications: _requiredVerifications,
            status: TriggerStatus.Pending
        });
    }

    /**
     * @dev Submit verification for trigger (e.g., death certificate)
     */
    function submitVerification(
        bytes32 _triggerId,
        bytes32 _willId,
        bytes calldata _proof
    ) external onlyRole(ORACLE_ROLE) {
        ExecutionTrigger storage trigger = executionTriggers[_triggerId];
        require(trigger.status == TriggerStatus.Pending, "Trigger not pending");
        require(!hasVerified[_triggerId][msg.sender], "Already verified");

        // Verify proof (implementation depends on proof type)
        require(_proof.length > 0, "Invalid proof");

        hasVerified[_triggerId][msg.sender] = true;
        trigger.verificationCount++;

        emit VerificationSubmitted(_triggerId, msg.sender);

        // Check if threshold met
        if (trigger.verificationCount >= trigger.requiredVerifications) {
            activateTrigger(_triggerId, _willId);
        }
    }

    /**
     * @dev Activate trigger after verification threshold met
     */
    function activateTrigger(bytes32 _triggerId, bytes32 _willId) internal {
        ExecutionTrigger storage trigger = executionTriggers[_triggerId];
        Will storage will = wills[_willId];

        trigger.status = TriggerStatus.Active;
        trigger.activatedAt = block.timestamp;
        will.status = WillStatus.Triggered;

        emit TriggerActivated(_willId, _triggerId);

        // Schedule execution after revocation period
        scheduleExecution(_willId, _triggerId);
    }

    /**
     * @dev Schedule will execution (in practice, would use oracle or keeper)
     */
    function scheduleExecution(bytes32 _willId, bytes32 _triggerId) internal {
        // In production, this would integrate with Chainlink Keepers or similar
        // For now, we just mark it as ready for execution
    }

    /**
     * @dev Execute will and distribute assets
     */
    function executeWill(bytes32 _willId) external onlyRole(EXECUTOR_ROLE) nonReentrant {
        Will storage will = wills[_willId];
        require(will.status == WillStatus.Triggered, "Will not triggered");

        // Check if revocation period has passed
        ExecutionTrigger memory trigger = findActiveTrigger(_willId);
        require(
            block.timestamp >= trigger.activatedAt + will.revocationPeriod,
            "Revocation period not passed"
        );

        // Check if execution delay has passed
        require(
            block.timestamp >= trigger.activatedAt + will.executionDelay,
            "Execution delay not passed"
        );

        will.status = WillStatus.Executed;

        // Distribute assets to beneficiaries
        distributeAssets(_willId);

        emit WillExecuted(_willId, block.timestamp);
    }

    /**
     * @dev Distribute assets to beneficiaries
     */
    function distributeAssets(bytes32 _willId) internal {
        Beneficiary[] memory beneficiaries = willBeneficiaries[_willId];
        uint256 totalBalance = address(this).balance;

        for (uint256 i = 0; i < beneficiaries.length; i++) {
            if (beneficiaries[i].beneficiaryType == BeneficiaryType.Primary) {
                uint256 amount = (totalBalance * beneficiaries[i].allocationPercentage) / 10000;
                payable(beneficiaries[i].beneficiaryAddress).transfer(amount);
            }
        }
    }

    /**
     * @dev Revoke will
     */
    function revokeWill(bytes32 _willId) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(
            wills[_willId].status != WillStatus.Executed,
            "Cannot revoke executed will"
        );

        wills[_willId].status = WillStatus.Revoked;
        emit WillRevoked(_willId, block.timestamp);
    }

    /**
     * @dev Validate beneficiary allocations sum to 100%
     */
    function validateBeneficiaries(bytes32 _willId) internal view returns (bool) {
        Beneficiary[] memory beneficiaries = willBeneficiaries[_willId];
        uint256 totalAllocation = 0;

        for (uint256 i = 0; i < beneficiaries.length; i++) {
            if (beneficiaries[i].beneficiaryType == BeneficiaryType.Primary) {
                totalAllocation += beneficiaries[i].allocationPercentage;
            }
        }

        return totalAllocation == 10000; // 100% in basis points
    }

    /**
     * @dev Find active trigger for will
     */
    function findActiveTrigger(bytes32 _willId) internal view returns (ExecutionTrigger memory) {
        // Implementation would search for active trigger
        // Simplified for example
        return ExecutionTrigger({
            triggerId: bytes32(0),
            triggerType: TriggerType.DeathVerification,
            activatedAt: block.timestamp,
            verificationCount: 0,
            requiredVerifications: 0,
            status: TriggerStatus.Active
        });
    }

    /**
     * @dev Allow testator to update will content
     */
    function updateWillContent(
        bytes32 _willId,
        bytes32 _newContentHash
    ) external {
        require(wills[_willId].testator == msg.sender, "Not testator");
        require(
            wills[_willId].status == WillStatus.Draft ||
            wills[_willId].status == WillStatus.Active,
            "Cannot modify will"
        );

        wills[_willId].contentHash = _newContentHash;
        wills[_willId].lastModified = block.timestamp;

        emit WillUpdated(_willId, _newContentHash);
    }

    /**
     * @dev Get beneficiaries for a will
     */
    function getBeneficiaries(bytes32 _willId)
        external
        view
        returns (Beneficiary[] memory)
    {
        return willBeneficiaries[_willId];
    }

    /**
     * @dev Get wills created by testator
     */
    function getTestatorWills(address _testator)
        external
        view
        returns (bytes32[] memory)
    {
        return testatorWills[_testator];
    }

    receive() external payable {}
}
```

### 4.5 Legal Document Template Generator

```go
package digitalwill

import (
    "bytes"
    "crypto/sha256"
    "encoding/hex"
    "encoding/json"
    "fmt"
    "text/template"
    "time"
)

// LegalDocument represents a legal document template
type LegalDocument struct {
    DocumentID       string                 `json:"documentId"`
    DocumentType     string                 `json:"documentType"`
    Title            string                 `json:"title"`
    Content          string                 `json:"content"`
    Jurisdiction     JurisdictionInfo       `json:"jurisdiction"`
    RequiredFields   []string               `json:"requiredFields"`
    NotarizationReq  bool                   `json:"notarizationRequired"`
    WitnessReq       int                    `json:"witnessRequired"`
    DigitalSignature *DigitalSignature      `json:"digitalSignature,omitempty"`
    Metadata         map[string]interface{} `json:"metadata"`
}

// JurisdictionInfo contains legal jurisdiction details
type JurisdictionInfo struct {
    Country      string            `json:"country"`
    State        string            `json:"state,omitempty"`
    LegalSystem  string            `json:"legalSystem"`
    Requirements map[string]string `json:"requirements"`
}

// DigitalSignature represents a digital signature
type DigitalSignature struct {
    Signature        string    `json:"signature"`
    Algorithm        string    `json:"algorithm"`
    PublicKey        string    `json:"publicKey"`
    Timestamp        time.Time `json:"timestamp"`
    CertificateChain []string  `json:"certificateChain"`
}

// TemplateGenerator generates legal documents from templates
type TemplateGenerator struct {
    templates map[string]*template.Template
}

// NewTemplateGenerator creates a new template generator
func NewTemplateGenerator() *TemplateGenerator {
    return &TemplateGenerator{
        templates: make(map[string]*template.Template),
    }
}

// RegisterTemplate registers a new document template
func (tg *TemplateGenerator) RegisterTemplate(
    docType string,
    templateContent string,
) error {
    tmpl, err := template.New(docType).Parse(templateContent)
    if err != nil {
        return fmt.Errorf("failed to parse template: %w", err)
    }

    tg.templates[docType] = tmpl
    return nil
}

// GenerateLastWillAndTestament generates a last will and testament
func (tg *TemplateGenerator) GenerateLastWillAndTestament(
    testator PersonIdentity,
    beneficiaries []Beneficiary,
    executors []Executor,
    digitalAssets []DigitalAsset,
    jurisdiction JurisdictionInfo,
) (*LegalDocument, error) {
    templateData := map[string]interface{}{
        "Testator":         testator,
        "Beneficiaries":    beneficiaries,
        "Executors":        executors,
        "DigitalAssets":    digitalAssets,
        "CreationDate":     time.Now().Format("January 2, 2006"),
        "Jurisdiction":     jurisdiction,
        "RevocationClause": tg.getRevocationClause(jurisdiction),
    }

    var buffer bytes.Buffer
    tmpl := tg.getWillTemplate(jurisdiction)

    if err := tmpl.Execute(&buffer, templateData); err != nil {
        return nil, fmt.Errorf("failed to execute template: %w", err)
    }

    doc := &LegalDocument{
        DocumentID:      generateDocumentID(),
        DocumentType:    "last-will-testament",
        Title:           fmt.Sprintf("Last Will and Testament of %s", testator.FullName),
        Content:         buffer.String(),
        Jurisdiction:    jurisdiction,
        RequiredFields:  []string{"testator", "executors", "beneficiaries", "date", "signature"},
        NotarizationReq: jurisdiction.Requirements["notarization"] == "required",
        WitnessReq:      2,
        Metadata: map[string]interface{}{
            "createdAt": time.Now().UTC(),
            "version":   "1.0.0",
        },
    }

    return doc, nil
}

// getWillTemplate returns the appropriate will template for jurisdiction
func (tg *TemplateGenerator) getWillTemplate(
    jurisdiction JurisdictionInfo,
) *template.Template {
    // Load jurisdiction-specific template
    templateKey := fmt.Sprintf("will-%s-%s", jurisdiction.Country, jurisdiction.State)

    if tmpl, exists := tg.templates[templateKey]; exists {
        return tmpl
    }

    // Fall back to generic template
    return tg.getGenericWillTemplate()
}

// getGenericWillTemplate returns a generic will template
func (tg *TemplateGenerator) getGenericWillTemplate() *template.Template {
    templateStr := `
LAST WILL AND TESTAMENT

I, {{.Testator.FullName}}, being of sound mind and disposing memory, do hereby make,
publish and declare this to be my Last Will and Testament, hereby revoking all wills
and codicils heretofore made by me.

ARTICLE I - IDENTIFICATION
I am a resident of {{.Testator.Address.City}}, {{.Testator.Address.State}},
{{.Testator.Address.Country}}.
Date of Birth: {{.Testator.DateOfBirth}}

ARTICLE II - APPOINTMENT OF EXECUTOR
{{range $index, $executor := .Executors}}
{{if eq $executor.Role "primary-executor"}}
I hereby nominate and appoint {{$executor.Identity.FullName}} as the Executor of this Will.
{{end}}
{{end}}

If the named Executor is unable or unwilling to serve, I nominate the following
successor Executor(s):
{{range $index, $executor := .Executors}}
{{if eq $executor.Role "successor-executor"}}
- {{$executor.Identity.FullName}}
{{end}}
{{end}}

ARTICLE III - DIGITAL ASSETS
I give, devise, and bequeath my digital assets as follows:

{{range $index, $asset := .DigitalAssets}}
{{$index | inc}}. {{$asset.Name}} ({{$asset.AssetType}})
   Provider: {{$asset.Provider}}
   Account: {{$asset.AccountIdentifier}}
   Instructions: {{$asset.TransferInstructions}}
{{end}}

ARTICLE IV - BENEFICIARIES
I give, devise, and bequeath my estate to the following beneficiaries:

{{range $index, $beneficiary := .Beneficiaries}}
{{$index | inc}}. {{$beneficiary.Identity.FullName}}
   Relationship: {{$beneficiary.Relationship}}
   Allocation: {{$beneficiary.AllocationPercentage}}%
   {{if $beneficiary.Conditions}}
   Conditions: Subject to fulfillment of specified conditions
   {{end}}
{{end}}

ARTICLE V - REVOCATION
{{.RevocationClause}}

IN WITNESS WHEREOF, I have hereunto set my hand this {{.CreationDate}}.

_________________________________
{{.Testator.FullName}}, Testator

WITNESSES:
We, the undersigned, certify that the testator signed this Will in our presence,
and that we signed as witnesses in the testator's presence and in the presence
of each other.

Witness 1:
Name: _______________________
Signature: __________________
Address: ____________________
Date: _______________________

Witness 2:
Name: _______________________
Signature: __________________
Address: ____________________
Date: _______________________

{{if .Jurisdiction.Requirements.notarization}}
NOTARY ACKNOWLEDGMENT:

State of ____________________
County of ___________________

On this _____ day of ________, 20___, before me personally appeared
{{.Testator.FullName}}, known to me to be the person described in and who
executed the foregoing instrument, and acknowledged that they executed the
same as their free act and deed.

_________________________________
Notary Public
My Commission Expires: __________
{{end}}
`

    funcMap := template.FuncMap{
        "inc": func(i int) int { return i + 1 },
    }

    tmpl, _ := template.New("generic-will").Funcs(funcMap).Parse(templateStr)
    return tmpl
}

// getRevocationClause returns jurisdiction-specific revocation clause
func (tg *TemplateGenerator) getRevocationClause(
    jurisdiction JurisdictionInfo,
) string {
    switch jurisdiction.Country {
    case "US":
        return "I hereby revoke all prior wills and codicils made by me."
    case "GB":
        return "I revoke all former wills and testamentary dispositions made by me."
    case "CA":
        return "I revoke all wills and codicils previously made by me."
    default:
        return "I revoke all previous wills and testamentary documents."
    }
}

// GeneratePowerOfAttorney generates a power of attorney document
func (tg *TemplateGenerator) GeneratePowerOfAttorney(
    principal PersonIdentity,
    attorney PersonIdentity,
    powers []string,
    effectiveDate time.Time,
    expirationDate *time.Time,
    jurisdiction JurisdictionInfo,
) (*LegalDocument, error) {
    templateData := map[string]interface{}{
        "Principal":      principal,
        "Attorney":       attorney,
        "Powers":         powers,
        "EffectiveDate":  effectiveDate.Format("January 2, 2006"),
        "ExpirationDate": expirationDate,
        "CreationDate":   time.Now().Format("January 2, 2006"),
    }

    var buffer bytes.Buffer
    tmpl := tg.getPowerOfAttorneyTemplate()

    if err := tmpl.Execute(&buffer, templateData); err != nil {
        return nil, fmt.Errorf("failed to execute template: %w", err)
    }

    doc := &LegalDocument{
        DocumentID:      generateDocumentID(),
        DocumentType:    "power-of-attorney",
        Title:           "Durable Power of Attorney for Financial and Digital Assets",
        Content:         buffer.String(),
        Jurisdiction:    jurisdiction,
        RequiredFields:  []string{"principal", "attorney", "powers", "date", "signature"},
        NotarizationReq: true,
        WitnessReq:      1,
        Metadata: map[string]interface{}{
            "createdAt": time.Now().UTC(),
            "version":   "1.0.0",
        },
    }

    return doc, nil
}

// getPowerOfAttorneyTemplate returns power of attorney template
func (tg *TemplateGenerator) getPowerOfAttorneyTemplate() *template.Template {
    templateStr := `
DURABLE POWER OF ATTORNEY
For Financial and Digital Asset Management

KNOW ALL MEN BY THESE PRESENTS:

I, {{.Principal.FullName}}, of {{.Principal.Address.City}}, {{.Principal.Address.State}},
being of sound mind, do hereby make, constitute and appoint {{.Attorney.FullName}} as my
true and lawful Attorney-in-Fact to act in my name, place and stead in any way which I
myself could do with respect to the following matters:

POWERS GRANTED:
{{range $index, $power := .Powers}}
{{$index | inc}}. {{$power}}
{{end}}

This Power of Attorney shall become effective on {{.EffectiveDate}}
{{if .ExpirationDate}}and shall expire on {{.ExpirationDate}}.{{else}}and shall
remain in effect until revoked by me.{{end}}

This Power of Attorney shall not be affected by my subsequent disability or incapacity.

IN WITNESS WHEREOF, I have executed this Durable Power of Attorney on {{.CreationDate}}.

_________________________________
{{.Principal.FullName}}, Principal

ACCEPTANCE BY ATTORNEY-IN-FACT:
I, {{.Attorney.FullName}}, hereby accept the appointment as Attorney-in-Fact and
agree to act in the best interests of the Principal.

_________________________________
{{.Attorney.FullName}}, Attorney-in-Fact
Date: _______________________
`

    funcMap := template.FuncMap{
        "inc": func(i int) int { return i + 1 },
    }

    tmpl, _ := template.New("power-of-attorney").Funcs(funcMap).Parse(templateStr)
    return tmpl
}

// HashDocument creates a cryptographic hash of the document
func (ld *LegalDocument) HashDocument() string {
    hasher := sha256.New()
    hasher.Write([]byte(ld.Content))
    hasher.Write([]byte(ld.DocumentID))
    hasher.Write([]byte(ld.Title))
    return hex.EncodeToString(hasher.Sum(nil))
}

// ToJSON converts the document to JSON
func (ld *LegalDocument) ToJSON() (string, error) {
    jsonData, err := json.MarshalIndent(ld, "", "  ")
    if err != nil {
        return "", err
    }
    return string(jsonData), nil
}

// generateDocumentID generates a unique document ID
func generateDocumentID() string {
    hash := sha256.Sum256([]byte(fmt.Sprintf("%d", time.Now().UnixNano())))
    return hex.EncodeToString(hash[:])
}

// PersonIdentity represents a person's identity
type PersonIdentity struct {
    FullName    string  `json:"fullName"`
    DateOfBirth string  `json:"dateOfBirth"`
    Email       string  `json:"email"`
    Phone       string  `json:"phone"`
    Address     Address `json:"address"`
}

// Address represents a physical address
type Address struct {
    Street     string `json:"street"`
    City       string `json:"city"`
    State      string `json:"state"`
    PostalCode string `json:"postalCode"`
    Country    string `json:"country"`
}

// Beneficiary represents a will beneficiary
type Beneficiary struct {
    Identity             PersonIdentity `json:"identity"`
    Relationship         string         `json:"relationship"`
    AllocationPercentage float64        `json:"allocationPercentage"`
    Conditions           []string       `json:"conditions,omitempty"`
}

// Executor represents a will executor
type Executor struct {
    Identity PersonIdentity `json:"identity"`
    Role     string         `json:"role"`
}

// DigitalAsset represents a digital asset
type DigitalAsset struct {
    Name                  string `json:"name"`
    AssetType             string `json:"assetType"`
    Provider              string `json:"provider"`
    AccountIdentifier     string `json:"accountIdentifier"`
    TransferInstructions  string `json:"transferInstructions"`
}
```

---

## 5. Validation and Compliance

### 5.1 Data Validation Rules

All implementations MUST validate:

1. **Required Fields**: All required fields per JSON schema must be present
2. **Format Validation**: Dates, UUIDs, emails must match specified formats
3. **Value Ranges**: Percentages (0-100), amounts (≥0), etc.
4. **Referential Integrity**: All UUID references must point to valid entities
5. **Allocation Totals**: Primary beneficiary allocations must sum to 100%
6. **Encryption Standards**: Only approved encryption methods allowed
7. **Signature Validity**: Digital signatures must verify successfully

### 5.2 Security Requirements

All implementations MUST:

1. Encrypt all access credentials using approved methods (AES-256-GCM, RSA-4096, ChaCha20-Poly1305)
2. Use secure key derivation (Argon2id recommended, PBKDF2 minimum)
3. Implement proper salt and IV generation (cryptographically random)
4. Store authentication tags for authenticated encryption
5. Support secure key escrow for executor access
6. Implement audit logging for all access to sensitive data
7. Support multi-factor authentication for will modifications

### 5.3 Legal Compliance Checklist

- [ ] Digital signatures comply with local e-signature laws
- [ ] Notarization meets jurisdiction requirements
- [ ] Witness requirements satisfied (number and qualifications)
- [ ] Testator capacity verification implemented
- [ ] Beneficiary identity verification available
- [ ] Asset transfer restrictions checked
- [ ] Privacy regulations compliance (GDPR, CCPA, etc.)
- [ ] Data retention policies implemented
- [ ] Cross-border transfer restrictions handled
- [ ] Probate court integration supported

---

## 6. Implementation Guidelines

### 6.1 Storage Recommendations

- **Primary Storage**: Encrypted database with at-rest encryption
- **Backup Storage**: Redundant encrypted backups in multiple locations
- **Cold Storage**: Offline encrypted backups for disaster recovery
- **Blockchain Storage**: Content hashes and proofs on immutable ledger
- **Distributed Storage**: IPFS or similar for document content
- **Access Control**: Role-based access with multi-factor authentication

### 6.2 Performance Considerations

- Index frequently queried fields (willId, testatorInfo, beneficiaryId)
- Cache encrypted credentials with short TTL
- Use pagination for asset and beneficiary lists
- Implement lazy loading for large documents
- Optimize JSON schema validation with compiled schemas
- Use streaming for large file encryption/decryption

### 6.3 Testing Requirements

All implementations must include:

- Unit tests for all data structure validation
- Integration tests for encryption/decryption
- Security tests for access control
- Compliance tests for jurisdiction requirements
- Load tests for concurrent access
- Disaster recovery tests
- Legal validity tests with sample documents

---

## 7. Future Considerations

### 7.1 Planned Extensions

- Multi-signature smart contract integration
- Quantum-resistant encryption algorithms
- AI-assisted estate planning recommendations
- Augmented reality will presentation
- Voice-activated will creation
- Biometric authentication integration
- Cross-chain asset management
- Decentralized identity integration (DID)

### 7.2 Version Compatibility

This specification follows semantic versioning. Implementations must:

- Support backward compatibility for data formats
- Provide migration tools for version upgrades
- Document breaking changes clearly
- Maintain deprecated features for minimum 2 major versions
- Include version in all document headers

---

## 8. References

### 8.1 Standards and Specifications

- [RFC 3339](https://tools.ietf.org/html/rfc3339) - Date and Time on the Internet
- [JSON Schema Draft 2020-12](https://json-schema.org/draft/2020-12/release-notes.html)
- [RFC 7519](https://tools.ietf.org/html/rfc7519) - JSON Web Token (JWT)
- [eIDAS Regulation](https://ec.europa.eu/digital-building-blocks/wikis/display/DIGITAL/eIDAS) - EU Electronic Identification
- [NIST FIPS 197](https://csrc.nist.gov/publications/detail/fips/197/final) - Advanced Encryption Standard

### 8.2 Legal References

- Uniform Electronic Transactions Act (UETA)
- Electronic Signatures in Global and National Commerce Act (E-SIGN)
- Wills Act 1837 (UK)
- EU Succession Regulation 650/2012
- Uniform Probate Code (UPC)

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
