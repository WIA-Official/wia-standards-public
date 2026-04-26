# WIA-DIGITAL_WILL PHASE 2 — API Interface Specification

**Standard:** WIA-DIGITAL_WILL
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

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
