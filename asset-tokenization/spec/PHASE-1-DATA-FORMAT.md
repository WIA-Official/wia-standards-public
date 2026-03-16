# WIA-FIN-008 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-01-20  
**Authors:** WIA Standards Committee

## Table of Contents

1. [Introduction](#introduction)
2. [Core Data Structures](#core-data-structures)
3. [Asset-Specific Schemas](#asset-specific-schemas)
4. [Compliance & Ownership](#compliance--ownership)
5. [Validation Rules](#validation-rules)
6. [Implementation Guide](#implementation-guide)

---

## 1. Introduction

Phase 1 of WIA-FIN-008 defines standardized JSON schemas for representing tokenized real-world assets. These schemas ensure interoperability across platforms, wallets, exchanges, and custody solutions.

### Design Principles

- **Extensibility:** Support for future asset classes via modular schema design
- **Compliance-First:** Built-in fields for KYC, accreditation, jurisdiction restrictions
- **Blockchain-Agnostic:** Works with Ethereum, Polygon, Avalanche, or private chains
- **JSON-LD Compatible:** Semantic web integration via @context and @type fields
- **Version-Controlled:** Semantic versioning for backward compatibility

---

## 2. Core Data Structures

### 2.1 Token Metadata Schema

The base schema for all tokenized assets:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wiastandards.com/schemas/fin-008/token-metadata-v1.json",
  "title": "WIA-FIN-008 Token Metadata",
  "type": "object",
  "required": ["@context", "@type", "version", "tokenId", "name", "symbol", "assetClass", "totalSupply", "issuer"],
  "properties": {
    "@context": {
      "type": "string",
      "format": "uri",
      "const": "https://wiastandards.com/asset-tokenization/v1"
    },
    "@type": {
      "type": "string",
      "const": "WIA-FIN-008/AssetToken"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Semantic version (e.g., 1.0.0)"
    },
    "tokenId": {
      "type": "string",
      "pattern": "^TOK-\\d{4}-[A-Z]+-\\d+$",
      "description": "Unique identifier (format: TOK-YYYY-CLASS-###)",
      "examples": ["TOK-2025-RE-001", "TOK-2025-ART-042"]
    },
    "name": {
      "type": "string",
      "minLength": 3,
      "maxLength": 100,
      "description": "Human-readable token name"
    },
    "symbol": {
      "type": "string",
      "minLength": 2,
      "maxLength": 10,
      "pattern": "^[A-Z0-9]+$",
      "description": "Ticker symbol (uppercase alphanumeric)"
    },
    "assetClass": {
      "type": "string",
      "enum": [
        "REAL_ESTATE",
        "ART",
        "COMMODITY",
        "PRIVATE_EQUITY",
        "DEBT_INSTRUMENT",
        "INTELLECTUAL_PROPERTY",
        "FUND",
        "OTHER"
      ]
    },
    "subClass": {
      "type": "string",
      "description": "Asset sub-category (e.g., COMMERCIAL_OFFICE for REAL_ESTATE)"
    },
    "totalSupply": {
      "type": "integer",
      "minimum": 1,
      "maximum": 1000000000000,
      "description": "Total number of tokens issued"
    },
    "decimals": {
      "type": "integer",
      "minimum": 0,
      "maximum": 18,
      "default": 0,
      "description": "Number of decimal places (0 for whole shares)"
    },
    "issuer": {
      "type": "object",
      "required": ["legalName", "jurisdiction"],
      "properties": {
        "legalName": {"type": "string"},
        "jurisdiction": {"type": "string"},
        "lei": {
          "type": "string",
          "pattern": "^[A-Z0-9]{20}$",
          "description": "Legal Entity Identifier (ISO 17442)"
        },
        "website": {"type": "string", "format": "uri"},
        "contact": {
          "type": "object",
          "properties": {
            "email": {"type": "string", "format": "email"},
            "phone": {"type": "string"}
          }
        }
      }
    },
    "issuanceDate": {
      "type": "string",
      "format": "date",
      "description": "Token creation date (ISO 8601)"
    },
    "maturityDate": {
      "type": ["string", "null"],
      "format": "date",
      "description": "For debt instruments, when principal is repaid"
    },
    "regulatory": {
      "type": "object",
      "required": ["framework"],
      "properties": {
        "framework": {
          "type": "string",
          "enum": [
            "REG_D_506B",
            "REG_D_506C",
            "REG_A_TIER1",
            "REG_A_TIER2",
            "REG_S",
            "REG_CF",
            "FULLY_REGISTERED",
            "EU_MiFID_II",
            "UK_FCA",
            "OTHER"
          ]
        },
        "filingNumber": {"type": "string"},
        "jurisdictions": {
          "type": "array",
          "items": {"type": "string"},
          "description": "ISO 3166-1 alpha-3 country codes"
        },
        "investorRestrictions": {
          "type": "object",
          "properties": {
            "accreditedOnly": {"type": "boolean", "default": true},
            "minInvestment": {
              "type": "object",
              "properties": {
                "amount": {"type": "number"},
                "currency": {"type": "string", "pattern": "^[A-Z]{3}$"}
              }
            },
            "maxInvestors": {"type": "integer"},
            "lockupPeriod": {
              "type": "string",
              "pattern": "^P(\\d+Y)?(\\d+M)?(\\d+D)?$",
              "description": "ISO 8601 duration (e.g., P6M for 6 months)"
            }
          }
        }
      }
    },
    "blockchain": {
      "type": "object",
      "required": ["network", "contractAddress", "standard"],
      "properties": {
        "network": {
          "type": "string",
          "enum": [
            "ethereum-mainnet",
            "polygon-mainnet",
            "bsc-mainnet",
            "avalanche-mainnet",
            "arbitrum-mainnet",
            "optimism-mainnet",
            "sepolia",
            "mumbai",
            "private"
          ]
        },
        "contractAddress": {
          "type": "string",
          "pattern": "^0x[a-fA-F0-9]{40}$",
          "description": "Ethereum-style address"
        },
        "standard": {
          "type": "string",
          "enum": ["ERC-1400", "ERC-3643", "ERC-20", "ERC-1155", "CUSTOM"]
        },
        "deployedAt": {
          "type": "string",
          "format": "date-time"
        },
        "txHash": {"type": "string"}
      }
    }
  }
}
```

---

## 3. Asset-Specific Schemas

### 3.1 Real Estate Schema

Extended fields for tokenized real estate:

- **Physical Attributes:** Address, coordinates, square footage, year built
- **Tenancy Data:** Occupancy rate, lease terms, rental income
- **Financials:** Purchase price, mortgage details, operating expenses, NOI
- **Valuation:** Cap rate, appraisal methodology, third-party reports

### 3.2 Art & Collectibles Schema

- **Provenance:** Complete ownership history with authentication
- **Authentication:** Expert opinions, technical analysis, catalogue raisonné
- **Condition:** Conservation reports, restoration history
- **Custody:** Storage location, insurance coverage, security measures

### 3.3 Commodity Schema

- **Physical Specifications:** Type, grade, quantity, purity, serial numbers
- **Storage:** Custodian, vault location, audit frequency
- **Pricing:** Spot price, pricing source, real-time updates
- **Redemption:** Physical delivery options, minimum redemption amounts

---

## 4. Compliance & Ownership

### 4.1 Ownership Records

Track all token holders with:

- **Identity Verification:** KYC status, provider, expiry dates
- **Accreditation:** Verification method, status, renewal dates  
- **Holdings:** Acquisition history, cost basis, current value
- **Restrictions:** Lockup periods, jurisdiction limits, max ownership

### 4.2 Compliance Rules

Define transfer restrictions:

- **Lockup Periods:** Duration, start/end dates, exceptions
- **KYC/AML:** Required providers, validity periods
- **Jurisdiction:** Allowed/blocked countries
- **Ownership Limits:** Maximum percentage per investor

---

## 5. Validation Rules

### 5.1 JSON Schema Validation

All WIA-FIN-008 data must validate against official JSON Schemas:

```bash
npm install ajv ajv-formats
```

```javascript
const Ajv = require('ajv');
const addFormats = require('ajv-formats');

const ajv = new Ajv();
addFormats(ajv);

const schema = require('./wia-fin-008-token-metadata-v1.json');
const validate = ajv.compile(schema);

const data = { /* your token data */ };
const valid = validate(data);

if (!valid) {
  console.error(validate.errors);
}
```

### 5.2 Business Logic Validation

Beyond schema validation, enforce:

- **totalSupply must match on-chain token supply**
- **Issuer LEI must be valid and registered**
- **Lockup dates must be consistent with regulatory framework**
- **Jurisdictions must use ISO 3166-1 alpha-3 codes**
- **Currency codes must use ISO 4217**

---

## 6. Implementation Guide

### 6.1 Getting Started

1. **Choose Asset Class:** Real estate, art, commodity, etc.
2. **Design Token Structure:** Total supply, decimals, regulatory framework
3. **Populate Metadata:** Use WIA-FIN-008 JSON templates
4. **Validate Data:** Run JSON Schema validator
5. **Store Immutably:** IPFS, Arweave, or on-chain storage

### 6.2 Best Practices

- **Version Control:** Use semantic versioning for all data updates
- **Immutable Storage:** Store on IPFS/Arweave with content-addressed hashes
- **Encryption:** Encrypt PII (investor names, addresses) at rest
- **Audit Trail:** Log all data modifications with timestamps and actors
- **Backup:** Maintain redundant copies across multiple storage providers

### 6.3 Integration Examples

```javascript
// TypeScript example
import { WIATokenMetadata } from '@wia/fin-008-types';

const token: WIATokenMetadata = {
  '@context': 'https://wiastandards.com/asset-tokenization/v1',
  '@type': 'WIA-FIN-008/AssetToken',
  version: '1.0.0',
  tokenId: 'TOK-2025-RE-001',
  name: 'Manhattan Prime Office REIT Token',
  symbol: 'MPOR',
  assetClass: 'REAL_ESTATE',
  totalSupply: 50000000,
  decimals: 0,
  // ... additional fields
};
```

---

## 7. Appendix

### A. Complete Schema Files

Available at:
- https://github.com/WIA-Official/wia-standards/tree/main/asset-tokenization/spec/schemas

### B. Validation Tools

- **Online Validator:** https://validator.wia-fin-008.com
- **CLI Tool:** `npm install -g @wia/fin-008-validator`
- **CI/CD Integration:** GitHub Actions, GitLab CI examples

### C. Change Log

**v1.0.0 (2025-01-20):**
- Initial release
- Core metadata, ownership, compliance schemas
- Support for 6 major asset classes

---

© 2025 SmileStory Inc. / WIA  
弘益人間 · Benefit All Humanity
