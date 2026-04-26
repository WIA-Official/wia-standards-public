# WIA-DIGITAL_WILL PHASE 1 — Data Format Specification

**Standard:** WIA-DIGITAL_WILL
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

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
