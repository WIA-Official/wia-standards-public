# WIA-FIN-004 Digital Currency Standard
## Phase 1: Data Format Specification

**Version:** 1.0  
**Status:** Production Ready  
**Last Updated:** 2025-01-15

---

## 1. Introduction

This specification defines standardized data formats for digital currency transactions, wallets, accounts, and related metadata. All WIA-FIN-004 compliant systems MUST implement these data structures to ensure interoperability.

### 1.1 Design Principles

- **Interoperability**: Compatible across all digital currency types
- **Extensibility**: Support for future currency types and features
- **Clarity**: Self-documenting and human-readable
- **Security**: Built-in validation and security metadata
- **Compliance**: Integrated regulatory and compliance fields

### 1.2 Notation

- **MUST**: Absolute requirement
- **SHOULD**: Strong recommendation
- **MAY**: Optional
- **ISO_8601**: DateTime format (e.g., `2024-01-15T10:30:00Z`)
- **ISO_4217**: Currency code (e.g., `USD`, `EUR`)
- **ISO_3166**: Country code (e.g., `US`, `GB`)
- **UUID**: RFC 4122 UUID

---

## 2. Currency Definition

### 2.1 Currency Type Schema

```json
{
  "currency": {
    "code": "string",              // ISO 4217 or custom identifier
    "name": "string",              // Full currency name
    "type": "CURRENCY_TYPE",       // See Currency Types enum
    "symbol": "string",            // Currency symbol (e.g., $, €, ₿)
    "decimals": integer,           // Number of decimal places
    "minAmount": "decimal",        // Minimum transaction amount
    "maxAmount": "decimal",        // Maximum transaction amount (optional)
    "blockchain": {
      "network": "string",         // e.g., "ethereum", "bitcoin", "polygon"
      "chainId": integer,          // Blockchain chain ID
      "contractAddress": "string", // Smart contract address (if applicable)
      "standard": "string"         // e.g., "ERC-20", "BEP-20", "native"
    },
    "issuer": {
      "entity": "string",          // Issuing organization
      "jurisdiction": "ISO_3166",  // Issuing jurisdiction
      "website": "url",            // Official website
      "regulators": ["string"]     // Regulatory bodies
    },
    "metadata": {
      "created": "ISO_8601",
      "lastUpdated": "ISO_8601",
      "status": "ACTIVE | DEPRECATED | SUSPENDED"
    }
  }
}
```

### 2.2 Currency Types Enum

```
FIAT              // Traditional fiat currency (USD, EUR, etc.)
CBDC              // Central Bank Digital Currency
STABLECOIN_FIAT   // Fiat-collateralized stablecoin
STABLECOIN_CRYPTO // Crypto-collateralized stablecoin
STABLECOIN_COMMODITY // Commodity-backed stablecoin
STABLECOIN_ALGORITHMIC // Algorithmic stablecoin
CRYPTOCURRENCY    // Decentralized cryptocurrency
VIRTUAL           // Virtual/gaming currency
CORPORATE         // Corporate-issued digital currency
E_MONEY           // Electronic money
```

---

## 3. Account Structure

### 3.1 Account Schema

```json
{
  "account": {
    "id": "UUID",                  // Unique account identifier
    "type": "ACCOUNT_TYPE",        // See Account Types enum
    "status": "ACCOUNT_STATUS",    // See Account Status enum
    "owner": {
      "type": "INDIVIDUAL | BUSINESS | INSTITUTION",
      "id": "string",              // KYC identifier
      "name": "string",
      "email": "email",
      "phone": "phone",
      "address": {
        "street": "string",
        "city": "string",
        "state": "string",
        "postalCode": "string",
        "country": "ISO_3166"
      }
    },
    "balances": [
      {
        "currency": "string",      // Currency code
        "available": "decimal",    // Available balance
        "pending": "decimal",      // Pending transactions
        "locked": "decimal",       // Locked/frozen amount
        "total": "decimal"         // Total balance
      }
    ],
    "limits": {
      "dailyTransactionLimit": "decimal",
      "monthlyTransactionLimit": "decimal",
      "singleTransactionLimit": "decimal",
      "currency": "string"
    },
    "compliance": {
      "kycLevel": "BASIC | STANDARD | ENHANCED",
      "kycStatus": "PENDING | VERIFIED | REJECTED | EXPIRED",
      "kycDate": "ISO_8601",
      "amlRating": "LOW | MEDIUM | HIGH",
      "pepStatus": boolean,
      "sanctionsScreening": {
        "status": "CLEAR | FLAGGED | BLOCKED",
        "lastChecked": "ISO_8601"
      }
    },
    "security": {
      "mfaEnabled": boolean,
      "mfaMethods": ["SMS", "TOTP", "HARDWARE_KEY"],
      "lastLogin": "ISO_8601",
      "ipWhitelist": ["ip_address"]
    },
    "metadata": {
      "created": "ISO_8601",
      "lastUpdated": "ISO_8601",
      "tags": ["string"]
    }
  }
}
```

### 3.2 Account Types Enum

```
PERSONAL          // Individual personal account
BUSINESS          // Business/corporate account
SAVINGS           // Savings account
CHECKING          // Checking account
MERCHANT          // Merchant payment account
EXCHANGE          // Exchange trading account
CUSTODIAL         // Third-party custodial account
NON_CUSTODIAL     // Self-custody wallet
```

### 3.3 Account Status Enum

```
ACTIVE            // Fully operational
PENDING           // Awaiting verification
SUSPENDED         // Temporarily suspended
FROZEN            // Frozen by compliance/legal
CLOSED            // Permanently closed
```

---

## 4. Transaction Structure

### 4.1 Transaction Schema

```json
{
  "transaction": {
    "id": "UUID",                  // Unique transaction identifier
    "type": "TRANSACTION_TYPE",    // See Transaction Types enum
    "status": "TRANSACTION_STATUS", // See Transaction Status enum
    "timestamp": "ISO_8601",       // Transaction creation time
    "sender": {
      "type": "ACCOUNT | WALLET | ADDRESS",
      "identifier": "string",
      "name": "string"             // Optional display name
    },
    "recipient": {
      "type": "ACCOUNT | WALLET | ADDRESS",
      "identifier": "string",
      "name": "string"
    },
    "amount": {
      "value": "decimal",          // Amount as decimal string
      "currency": "string",        // Currency code
      "precision": integer         // Decimal precision
    },
    "fee": {
      "value": "decimal",
      "currency": "string",
      "paidBy": "SENDER | RECIPIENT | SPLIT"
    },
    "exchange": {                  // For currency exchanges
      "fromCurrency": "string",
      "toCurrency": "string",
      "rate": "decimal",
      "receivedAmount": "decimal"
    },
    "purpose": "string",           // Transaction description/memo
    "reference": "string",         // External reference (invoice, order ID)
    "blockchain": {
      "network": "string",
      "txHash": "string",          // On-chain transaction hash
      "blockNumber": integer,
      "confirmations": integer,
      "gasUsed": integer,
      "gasPrice": "decimal"
    },
    "settlement": {
      "method": "INSTANT | BATCH | DEFERRED",
      "settledAt": "ISO_8601",
      "settlementReference": "string"
    },
    "compliance": {
      "amlCheck": "PASSED | FLAGGED | FAILED",
      "amlScore": integer,         // 0-100 risk score
      "kycVerified": boolean,
      "sanctionsCheck": "PASSED | FLAGGED | FAILED",
      "travelRuleData": {          // For cross-border >$1000
        "originatorName": "string",
        "originatorAddress": "string",
        "beneficiaryName": "string",
        "beneficiaryAddress": "string"
      },
      "sarFiled": boolean,         // Suspicious Activity Report
      "flags": ["string"]          // Compliance flags
    },
    "security": {
      "signature": "string",       // Cryptographic signature
      "signingKey": "string",      // Public key used for signing
      "nonce": integer,            // Transaction nonce
      "ipAddress": "string",       // Originating IP
      "deviceId": "string",        // Device identifier
      "geolocation": {
        "latitude": decimal,
        "longitude": decimal,
        "country": "ISO_3166"
      }
    },
    "metadata": {
      "created": "ISO_8601",
      "lastUpdated": "ISO_8601",
      "completedAt": "ISO_8601",
      "failureReason": "string",
      "retryCount": integer,
      "tags": ["string"],
      "extensionData": {}          // Custom fields
    }
  }
}
```

### 4.2 Transaction Types Enum

```
TRANSFER          // Standard peer-to-peer transfer
PAYMENT           // Payment to merchant/service
EXCHANGE          // Currency exchange
WITHDRAWAL        // Withdrawal to external system
DEPOSIT           // Deposit from external system
REFUND            // Refund of previous transaction
FEE               // Fee payment
MINT              // Stablecoin/token minting
BURN              // Stablecoin/token burning
STAKE             // Staking operation
UNSTAKE           // Unstaking operation
REWARD            // Staking/yield reward
LOAN              // Loan disbursement
REPAYMENT         // Loan repayment
```

### 4.3 Transaction Status Enum

```
INITIATED         // Transaction created but not submitted
PENDING           // Awaiting processing
PROCESSING        // Currently being processed
CONFIRMING        // Awaiting blockchain confirmations
COMPLETED         // Successfully completed
FAILED            // Failed permanently
CANCELLED         // Cancelled by user or system
REVERSED          // Reversed/chargebacked
FLAGGED           // Flagged for compliance review
BLOCKED           // Blocked by compliance
```

---

## 5. Wallet Structure

### 5.1 Wallet Schema

```json
{
  "wallet": {
    "id": "UUID",
    "type": "WALLET_TYPE",         // See Wallet Types enum
    "status": "ACTIVE | SUSPENDED | LOCKED",
    "name": "string",              // User-defined wallet name
    "description": "string",
    "addresses": [
      {
        "currency": "string",
        "address": "string",       // Blockchain address
        "derivationPath": "string", // BIP-44 path
        "balance": "decimal",
        "lastActivity": "ISO_8601"
      }
    ],
    "security": {
      "encryptionMethod": "AES-256-GCM | CHACHA20-POLY1305",
      "encryptionSalt": "string",
      "keyDerivation": "PBKDF2 | ARGON2",
      "backupMethod": "SEED_PHRASE | KEYSTORE | HARDWARE",
      "seedPhraseBackedUp": boolean,
      "multiSig": {
        "enabled": boolean,
        "required": integer,       // M in M-of-N
        "total": integer,          // N in M-of-N
        "signers": ["publicKey"]
      }
    },
    "recovery": {
      "seedPhraseWordCount": 12 | 24,
      "recoveryQuestions": boolean,
      "socialRecovery": {
        "enabled": boolean,
        "guardians": ["identifier"],
        "threshold": integer
      }
    },
    "metadata": {
      "created": "ISO_8601",
      "lastAccessed": "ISO_8601",
      "deviceId": "string",
      "appVersion": "string"
    }
  }
}
```

### 5.2 Wallet Types Enum

```
HOT_WALLET        // Online/connected wallet
COLD_WALLET       // Offline/air-gapped wallet
HARDWARE_WALLET   // Dedicated hardware device
PAPER_WALLET      // Paper backup only
CUSTODIAL         // Third-party custodied
NON_CUSTODIAL     // Self-custody
MULTI_SIG         // Multi-signature wallet
SMART_CONTRACT    // Smart contract wallet
```

---

## 6. Validation Rules

### 6.1 Amount Validation

- MUST be represented as decimal strings to avoid floating-point errors
- MUST NOT exceed currency's maximum decimal precision
- MUST be greater than currency's minimum amount
- MUST NOT be negative (except for specific debit entries)

Example: `"123.456789"` for precision 6

### 6.2 Currency Code Validation

- MUST be 3-character ISO 4217 for fiat currencies
- MAY be 3-8 characters for digital currencies
- MUST be uppercase
- MUST NOT contain special characters

### 6.3 Address Validation

- MUST include checksum validation where applicable
- MUST be network-appropriate (e.g., Bitcoin address for Bitcoin network)
- MUST validate format based on currency type

### 6.4 Timestamp Validation

- MUST be ISO 8601 format
- MUST include timezone (UTC preferred)
- SHOULD be within reasonable time bounds

---

## 7. Compliance Data

### 7.1 KYC/AML Data Schema

```json
{
  "compliance": {
    "kyc": {
      "level": "BASIC | STANDARD | ENHANCED",
      "individual": {
        "fullName": "string",
        "dateOfBirth": "ISO_8601",
        "nationality": "ISO_3166",
        "idType": "PASSPORT | DRIVERS_LICENSE | NATIONAL_ID",
        "idNumber": "string",
        "idIssueDate": "ISO_8601",
        "idExpiryDate": "ISO_8601",
        "idIssuingCountry": "ISO_3166"
      },
      "business": {
        "legalName": "string",
        "registrationNumber": "string",
        "registrationCountry": "ISO_3166",
        "businessType": "string",
        "incorporationDate": "ISO_8601",
        "beneficialOwners": [
          {
            "name": "string",
            "ownership": "percentage",
            "pepStatus": boolean
          }
        ]
      },
      "verification": {
        "status": "PENDING | VERIFIED | REJECTED | EXPIRED",
        "method": "MANUAL | AUTOMATED | THIRD_PARTY",
        "verifiedBy": "string",
        "verifiedAt": "ISO_8601",
        "documents": ["documentId"]
      }
    },
    "aml": {
      "riskScore": integer,        // 0-100
      "riskCategory": "LOW | MEDIUM | HIGH | CRITICAL",
      "screeningResults": [
        {
          "listType": "SANCTIONS | PEP | ADVERSE_MEDIA",
          "listName": "string",    // e.g., "OFAC SDN"
          "matchStatus": "NO_MATCH | POSSIBLE_MATCH | TRUE_MATCH",
          "matchDetails": "string"
        }
      ],
      "transactionMonitoring": {
        "enabled": boolean,
        "rules": ["ruleId"],
        "lastReview": "ISO_8601"
      }
    }
  }
}
```

---

## 8. Error Format

### 8.1 Error Response Schema

```json
{
  "error": {
    "code": "string",              // Machine-readable error code
    "message": "string",           // Human-readable message
    "details": "string",           // Additional details
    "field": "string",             // Field that caused error
    "timestamp": "ISO_8601",
    "requestId": "UUID",
    "documentation": "url"         // Link to error documentation
  }
}
```

### 8.2 Standard Error Codes

```
INVALID_AMOUNT        // Amount validation failed
INSUFFICIENT_BALANCE  // Not enough funds
INVALID_ADDRESS       // Address format invalid
INVALID_CURRENCY      // Currency not supported
DUPLICATE_TRANSACTION // Transaction already exists
RATE_LIMIT_EXCEEDED   // Too many requests
KYC_REQUIRED          // KYC verification needed
KYC_FAILED            // KYC verification failed
AML_FLAGGED           // Transaction flagged by AML
SANCTIONS_HIT         // Sanctions screening hit
ACCOUNT_SUSPENDED     // Account not active
TRANSACTION_TOO_LARGE // Exceeds limits
NETWORK_ERROR         // Blockchain network error
INVALID_SIGNATURE     // Cryptographic signature invalid
EXPIRED_REQUEST       // Request timestamp too old
```

---

## 9. Versioning

All data structures MUST include a `version` field:

```json
{
  "version": "WIA-FIN-004-v1.0",
  "data": { /* ... */ }
}
```

Backward compatibility MUST be maintained within major versions.

---

## 10. Examples

### 10.1 Complete Transaction Example

```json
{
  "version": "WIA-FIN-004-v1.0",
  "transaction": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "type": "TRANSFER",
    "status": "COMPLETED",
    "timestamp": "2024-01-15T10:30:00Z",
    "sender": {
      "type": "ACCOUNT",
      "identifier": "ACC-123456",
      "name": "Alice Smith"
    },
    "recipient": {
      "type": "ACCOUNT",
      "identifier": "ACC-789012",
      "name": "Bob Johnson"
    },
    "amount": {
      "value": "100.00",
      "currency": "USDC",
      "precision": 2
    },
    "fee": {
      "value": "0.50",
      "currency": "USDC",
      "paidBy": "SENDER"
    },
    "purpose": "Payment for services rendered",
    "reference": "INV-2024-001",
    "blockchain": {
      "network": "ethereum",
      "txHash": "0x742d35Cc6634C0532925a3b844Bc9e7595f0b3e4c1a7b3a2d8e9f1a2b3c4d5e6",
      "blockNumber": 18500000,
      "confirmations": 12,
      "gasUsed": 21000,
      "gasPrice": "25.5"
    },
    "compliance": {
      "amlCheck": "PASSED",
      "amlScore": 15,
      "kycVerified": true,
      "sanctionsCheck": "PASSED",
      "sarFiled": false,
      "flags": []
    },
    "metadata": {
      "created": "2024-01-15T10:30:00Z",
      "completedAt": "2024-01-15T10:30:45Z"
    }
  }
}
```

---

## Appendix A: JSON Schema

Complete JSON Schema definitions available at:
`/schemas/v1/`

---

**End of Phase 1 Specification**

© 2025 SmileStory Inc. / WIA  
弘益人間 (Benefit All Humanity)

---

## Annex A — Conformance Tier Matrix

WIA conformance for digital-currency is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/digital-currency/api/` — TypeScript SDK skeleton
- `wia-standards/standards/digital-currency/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/digital-currency/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
