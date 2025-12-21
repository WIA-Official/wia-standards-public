# WIA Carbon Credit Micro Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Data Types](#data-types)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Carbon Credit Micro Data Format Standard defines a unified format for tracking, verifying, and trading individual carbon footprints and micro carbon credits. This standard enables individuals to monitor their daily carbon emissions, earn micro-credits through sustainable actions, and trade these credits in a blockchain-verified marketplace.

**Core Objectives**:
- Enable precise tracking of individual carbon footprints
- Standardize micro carbon credit data for global interoperability
- Support blockchain-based verification and transparency
- Facilitate seamless integration with corporate offset programs
- Enable peer-to-peer and marketplace carbon credit trading

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Carbon Footprint Tracking | Daily emissions from transportation, energy, consumption |
| Micro Credit Issuance | Credit generation from sustainable actions |
| Credit Tokenization | Blockchain-based credit tokens (ERC-20, NFT) |
| Trading Transactions | P2P and marketplace credit exchanges |
| Corporate Integration | Enterprise carbon offset purchase programs |

### 1.3 Design Principles

1. **Granularity**: Track emissions at daily/hourly resolution
2. **Verifiability**: All data cryptographically signed and blockchain-anchored
3. **Fungibility**: Credits standardized for easy trading
4. **Privacy**: Personal data encrypted, aggregated data public
5. **Interoperability**: Compatible with major carbon standards (GHG Protocol, ISO 14064)

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Carbon Footprint** | Total CO2 equivalent emissions from an individual's activities |
| **Micro Credit** | Small-denomination carbon credit (0.001 - 1.0 tCO2e) |
| **Carbon Token** | Blockchain token representing verified carbon credits |
| **Emission Activity** | Single event generating CO2 emissions |
| **Offset Action** | Activity reducing or sequestering carbon |
| **Credit Balance** | Total verified credits owned by an individual |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `co2_amount` | CO2 equivalent in kilograms | `12.5` (kg CO2e) |
| `credit_token` | Blockchain token identifier | `"CCM-2025-0001"` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |
| `transaction_hash` | Blockchain transaction hash | `"0xabc123..."` |
| `wallet_address` | Crypto wallet address | `"0x742d35Cc..."` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Carbon Credit Record Format

```json
{
  "$schema": "https://wia.live/carbon-credit-micro/v1/schema.json",
  "version": "1.0.0",
  "recordId": "CCR-2025-000001",
  "userId": "USER-2025-001",
  "recordType": "footprint",
  "created": "2025-01-15T10:00:00Z",
  "carbonFootprint": {
    "totalEmissions": 125.5,
    "period": "2025-01-15",
    "activities": [],
    "categories": {}
  },
  "carbonCredits": {
    "balance": 2.5,
    "earned": [],
    "purchased": [],
    "sold": [],
    "retired": []
  },
  "blockchain": {
    "tokenId": "CCM-2025-0001",
    "contractAddress": "0x...",
    "network": "ethereum",
    "transactions": []
  },
  "verification": {
    "status": "verified",
    "verifiedBy": "WIA-Verifier-001",
    "verifiedAt": "2025-01-15T11:00:00Z",
    "confidence": 0.98
  },
  "meta": {
    "hash": "sha256:...",
    "signature": "...",
    "previousHash": "..."
  }
}
```

### 3.2 Field Details

#### 3.2.1 `recordId` (REQUIRED)

```
Type: string
Format: CCR-YYYY-NNNNNN
Description: Unique identifier for this carbon record
Example: "CCR-2025-000001"
```

#### 3.2.2 `recordType` (REQUIRED)

```
Type: string
Valid values:
  - "footprint"   : Carbon footprint tracking record
  - "credit"      : Carbon credit issuance record
  - "transaction" : Credit trading transaction
  - "offset"      : Corporate offset purchase
  - "retirement"  : Credit retirement (permanent offset)
```

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/carbon-credit-micro/v1/schema.json",
  "title": "WIA Carbon Credit Micro Record",
  "type": "object",
  "required": ["version", "recordId", "userId", "recordType", "created"],
  "properties": {
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$",
      "description": "Schema version"
    },
    "recordId": {
      "type": "string",
      "pattern": "^CCR-\\d{4}-\\d{6}$",
      "description": "Unique record identifier"
    },
    "userId": {
      "type": "string",
      "description": "User identifier"
    },
    "recordType": {
      "type": "string",
      "enum": ["footprint", "credit", "transaction", "offset", "retirement"]
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "carbonFootprint": {
      "type": "object",
      "properties": {
        "totalEmissions": {
          "type": "number",
          "minimum": 0,
          "description": "Total CO2e in kg"
        },
        "period": {
          "type": "string",
          "format": "date"
        },
        "activities": {
          "type": "array",
          "items": {
            "$ref": "#/definitions/EmissionActivity"
          }
        },
        "categories": {
          "type": "object",
          "properties": {
            "transportation": { "type": "number" },
            "energy": { "type": "number" },
            "food": { "type": "number" },
            "consumption": { "type": "number" },
            "waste": { "type": "number" }
          }
        }
      }
    },
    "carbonCredits": {
      "type": "object",
      "properties": {
        "balance": {
          "type": "number",
          "minimum": 0,
          "description": "Current credit balance in tCO2e"
        },
        "earned": { "type": "array" },
        "purchased": { "type": "array" },
        "sold": { "type": "array" },
        "retired": { "type": "array" }
      }
    },
    "blockchain": {
      "type": "object",
      "properties": {
        "tokenId": { "type": "string" },
        "contractAddress": { "type": "string" },
        "network": { "type": "string" },
        "transactions": { "type": "array" }
      }
    }
  },
  "definitions": {
    "EmissionActivity": {
      "type": "object",
      "required": ["activityId", "category", "co2Amount", "timestamp"],
      "properties": {
        "activityId": { "type": "string" },
        "category": { "type": "string" },
        "subcategory": { "type": "string" },
        "description": { "type": "string" },
        "co2Amount": { "type": "number" },
        "unit": { "type": "string" },
        "timestamp": { "type": "string", "format": "date-time" },
        "location": { "type": "object" },
        "verified": { "type": "boolean" }
      }
    }
  }
}
```

### 4.2 Carbon Footprint Schema

```json
{
  "carbonFootprint": {
    "totalEmissions": 125.5,
    "period": "2025-01-15",
    "periodType": "daily",
    "activities": [
      {
        "activityId": "ACT-2025-0001",
        "category": "transportation",
        "subcategory": "car",
        "description": "Commute to work",
        "co2Amount": 5.2,
        "unit": "kg CO2e",
        "distance": 25,
        "distanceUnit": "km",
        "fuelType": "gasoline",
        "timestamp": "2025-01-15T08:30:00Z",
        "location": {
          "start": { "lat": 37.5665, "lng": 126.9780 },
          "end": { "lat": 37.5172, "lng": 127.0473 }
        },
        "verified": true,
        "verificationMethod": "gps_tracking"
      },
      {
        "activityId": "ACT-2025-0002",
        "category": "energy",
        "subcategory": "electricity",
        "description": "Home electricity usage",
        "co2Amount": 3.8,
        "unit": "kg CO2e",
        "consumption": 15,
        "consumptionUnit": "kWh",
        "timestamp": "2025-01-15T00:00:00Z",
        "verified": true,
        "verificationMethod": "smart_meter"
      },
      {
        "activityId": "ACT-2025-0003",
        "category": "food",
        "subcategory": "meat",
        "description": "Beef consumption",
        "co2Amount": 2.7,
        "unit": "kg CO2e",
        "quantity": 0.5,
        "quantityUnit": "kg",
        "timestamp": "2025-01-15T19:00:00Z",
        "verified": false,
        "verificationMethod": "self_reported"
      }
    ],
    "categories": {
      "transportation": 45.3,
      "energy": 28.5,
      "food": 32.7,
      "consumption": 15.0,
      "waste": 4.0
    },
    "comparison": {
      "previousPeriod": 135.2,
      "change": -9.7,
      "changePercent": -7.2,
      "nationalAverage": 150.0,
      "vsAverage": -16.3
    }
  }
}
```

### 4.3 Carbon Credits Schema

```json
{
  "carbonCredits": {
    "balance": 2.5,
    "balanceUnit": "tCO2e",
    "earned": [
      {
        "creditId": "CRD-2025-0001",
        "amount": 0.5,
        "unit": "tCO2e",
        "source": "tree_planting",
        "description": "Planted 10 trees",
        "earnedAt": "2025-01-10T10:00:00Z",
        "verifiedBy": "WIA-Verifier-001",
        "verificationProof": "ipfs://Qm...",
        "tokenId": "CCM-2025-0001",
        "status": "active"
      },
      {
        "creditId": "CRD-2025-0002",
        "amount": 0.3,
        "unit": "tCO2e",
        "source": "renewable_energy",
        "description": "Solar panel installation",
        "earnedAt": "2025-01-12T15:00:00Z",
        "verifiedBy": "WIA-Verifier-002",
        "tokenId": "CCM-2025-0002",
        "status": "active"
      }
    ],
    "purchased": [
      {
        "transactionId": "TXN-2025-0001",
        "amount": 1.0,
        "pricePerCredit": 25.5,
        "currency": "USD",
        "totalPrice": 25.5,
        "seller": "USER-2025-500",
        "purchasedAt": "2025-01-14T12:00:00Z",
        "blockchainTx": "0xabc123...",
        "status": "completed"
      }
    ],
    "sold": [
      {
        "transactionId": "TXN-2025-0050",
        "amount": 0.2,
        "pricePerCredit": 26.0,
        "currency": "USD",
        "totalPrice": 5.2,
        "buyer": "CORP-2025-100",
        "soldAt": "2025-01-13T14:30:00Z",
        "blockchainTx": "0xdef456...",
        "status": "completed"
      }
    ],
    "retired": [
      {
        "retirementId": "RET-2025-0001",
        "amount": 0.1,
        "reason": "personal_offset",
        "retiredAt": "2025-01-15T09:00:00Z",
        "blockchainTx": "0xghi789...",
        "permanent": true
      }
    ]
  }
}
```

### 4.4 Blockchain Integration Schema

```json
{
  "blockchain": {
    "tokenId": "CCM-2025-0001",
    "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
    "network": "ethereum",
    "standard": "ERC-20",
    "decimals": 3,
    "transactions": [
      {
        "transactionHash": "0xabc123def456...",
        "blockNumber": 18234567,
        "timestamp": "2025-01-15T10:00:00Z",
        "from": "0x0000000000000000000000000000000000000000",
        "to": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
        "type": "mint",
        "amount": 2.5,
        "gasUsed": 65000,
        "status": "confirmed"
      },
      {
        "transactionHash": "0xdef456ghi789...",
        "blockNumber": 18234890,
        "timestamp": "2025-01-15T14:00:00Z",
        "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
        "to": "0x8ba1f109551bD432803012645Ac136ddd64DBA72",
        "type": "transfer",
        "amount": 0.2,
        "gasUsed": 45000,
        "status": "confirmed"
      }
    ],
    "metadata": {
      "name": "WIA Carbon Credit Micro",
      "symbol": "WCCM",
      "decimals": 3,
      "totalSupply": 1000000,
      "chainId": 1
    }
  }
}
```

---

## Field Specifications

### 5.1 Carbon Footprint Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `totalEmissions` | number | REQUIRED | Total CO2e in kg | `125.5` |
| `period` | string | REQUIRED | Date or period | `"2025-01-15"` |
| `periodType` | string | REQUIRED | Period granularity | `"daily"` |
| `activities[]` | array | REQUIRED | Individual activities | `[{...}]` |
| `categories` | object | OPTIONAL | Emissions by category | `{...}` |

### 5.2 Emission Activity Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `activityId` | string | REQUIRED | Activity identifier | `"ACT-2025-0001"` |
| `category` | string | REQUIRED | Main category | `"transportation"` |
| `subcategory` | string | OPTIONAL | Sub-category | `"car"` |
| `co2Amount` | number | REQUIRED | CO2e amount in kg | `5.2` |
| `timestamp` | string | REQUIRED | Activity timestamp | `"2025-01-15T08:30:00Z"` |
| `verified` | boolean | REQUIRED | Verification status | `true` |

**Valid Categories:**

| Category | Description | Typical Range (kg CO2e/day) |
|----------|-------------|----------------------------|
| `transportation` | Vehicles, public transit, flights | 1.0 - 50.0 |
| `energy` | Electricity, heating, cooling | 2.0 - 20.0 |
| `food` | Meals, groceries, dining | 3.0 - 15.0 |
| `consumption` | Shopping, goods, services | 1.0 - 30.0 |
| `waste` | Trash, recycling | 0.5 - 5.0 |

### 5.3 Carbon Credit Fields

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `creditId` | string | REQUIRED | Credit identifier | `"CRD-2025-0001"` |
| `amount` | number | REQUIRED | Credit amount in tCO2e | `0.5` |
| `source` | string | REQUIRED | Credit source | `"tree_planting"` |
| `earnedAt` | string | REQUIRED | Issuance timestamp | `"2025-01-10T10:00:00Z"` |
| `verifiedBy` | string | REQUIRED | Verifier ID | `"WIA-Verifier-001"` |
| `tokenId` | string | OPTIONAL | Blockchain token ID | `"CCM-2025-0001"` |
| `status` | string | REQUIRED | Credit status | `"active"` |

**Valid Credit Sources:**

| Source | Description | Typical Amount (tCO2e) |
|--------|-------------|----------------------|
| `tree_planting` | Tree planting projects | 0.1 - 5.0 |
| `renewable_energy` | Solar, wind energy adoption | 0.5 - 10.0 |
| `energy_efficiency` | Home/office efficiency improvements | 0.2 - 3.0 |
| `public_transit` | Using public transportation | 0.01 - 0.5 |
| `cycling_walking` | Active transportation | 0.01 - 0.3 |
| `waste_reduction` | Recycling, composting | 0.05 - 1.0 |
| `carbon_capture` | Direct air capture participation | 1.0 - 100.0 |

### 5.4 CO2 Calculation Formulas

#### Transportation

```
CO2 (kg) = Distance (km) × Emission Factor (kg CO2e/km)

Emission Factors:
- Gasoline car: 0.21 kg CO2e/km
- Diesel car: 0.17 kg CO2e/km
- Electric car: 0.05 kg CO2e/km
- Bus: 0.10 kg CO2e/km
- Train: 0.04 kg CO2e/km
- Flight (short): 0.25 kg CO2e/km
- Flight (long): 0.18 kg CO2e/km
```

#### Energy

```
CO2 (kg) = Energy (kWh) × Grid Emission Factor (kg CO2e/kWh)

Grid Emission Factors (by region):
- USA: 0.385 kg CO2e/kWh
- EU: 0.295 kg CO2e/kWh
- China: 0.555 kg CO2e/kWh
- India: 0.708 kg CO2e/kWh
- Korea: 0.459 kg CO2e/kWh
```

#### Food

```
CO2 (kg) = Quantity (kg) × Food Emission Factor (kg CO2e/kg)

Food Emission Factors:
- Beef: 27.0 kg CO2e/kg
- Lamb: 39.2 kg CO2e/kg
- Pork: 12.1 kg CO2e/kg
- Chicken: 6.9 kg CO2e/kg
- Fish: 5.4 kg CO2e/kg
- Vegetables: 2.0 kg CO2e/kg
- Fruits: 1.1 kg CO2e/kg
```

---

## Data Types

### 6.1 Custom Types

```typescript
type RecordType =
  | 'footprint'
  | 'credit'
  | 'transaction'
  | 'offset'
  | 'retirement';

type EmissionCategory =
  | 'transportation'
  | 'energy'
  | 'food'
  | 'consumption'
  | 'waste';

type CreditSource =
  | 'tree_planting'
  | 'renewable_energy'
  | 'energy_efficiency'
  | 'public_transit'
  | 'cycling_walking'
  | 'waste_reduction'
  | 'carbon_capture';

type CreditStatus =
  | 'active'
  | 'locked'
  | 'sold'
  | 'retired';

type BlockchainNetwork =
  | 'ethereum'
  | 'polygon'
  | 'avalanche'
  | 'binance';

interface CO2Amount {
  value: number;
  unit: 'kg' | 'ton';
  co2Equivalent: number;
}

interface CarbonToken {
  tokenId: string;
  amount: number;
  network: BlockchainNetwork;
  contractAddress: string;
  owner: string;
}
```

### 6.2 Enum Values

#### Verification Methods

| Code | Description | Confidence |
|------|-------------|------------|
| `gps_tracking` | GPS-based location tracking | 95% |
| `smart_meter` | Smart meter IoT integration | 98% |
| `receipt_scan` | Receipt/invoice OCR | 80% |
| `api_integration` | Third-party API data | 90% |
| `self_reported` | User self-reported data | 60% |
| `ai_estimation` | AI-based estimation | 75% |

---

## Validation Rules

### 7.1 Required Field Validation

| Rule ID | Field | Validation |
|---------|-------|------------|
| VAL-001 | `recordId` | Must match `^CCR-\d{4}-\d{6}$` |
| VAL-002 | `totalEmissions` | Must be >= 0 |
| VAL-003 | `co2Amount` | Must be >= 0 and <= 10000 kg |
| VAL-004 | `creditAmount` | Must be >= 0.001 and <= 1000 tCO2e |
| VAL-005 | `timestamp` | Must be valid ISO 8601 format |

### 7.2 Business Logic Validation

| Rule ID | Description | Error Code |
|---------|-------------|------------|
| BUS-001 | Total emissions must equal sum of activities | `ERR_EMISSION_MISMATCH` |
| BUS-002 | Cannot sell more credits than balance | `ERR_INSUFFICIENT_CREDITS` |
| BUS-003 | Retirement is permanent and irreversible | `ERR_RETIRED_CREDIT` |
| BUS-004 | Blockchain transaction must be confirmed | `ERR_UNCONFIRMED_TX` |
| BUS-005 | Credit must be verified before trading | `ERR_UNVERIFIED_CREDIT` |

### 7.3 Error Codes

| Code | Message | Description |
|------|---------|-------------|
| `ERR_INVALID_RECORD` | Invalid record format | Record format violation |
| `ERR_EMISSION_MISMATCH` | Emission calculation error | Sum mismatch |
| `ERR_INSUFFICIENT_CREDITS` | Insufficient credit balance | Balance too low |
| `ERR_RETIRED_CREDIT` | Credit already retired | Cannot trade retired credits |
| `ERR_INVALID_TOKEN` | Invalid blockchain token | Token validation failed |

---

## Examples

### 8.1 Valid Daily Footprint Record

```json
{
  "$schema": "https://wia.live/carbon-credit-micro/v1/schema.json",
  "version": "1.0.0",
  "recordId": "CCR-2025-000001",
  "userId": "USER-2025-001",
  "recordType": "footprint",
  "created": "2025-01-15T23:59:59Z",
  "carbonFootprint": {
    "totalEmissions": 18.5,
    "period": "2025-01-15",
    "periodType": "daily",
    "activities": [
      {
        "activityId": "ACT-2025-0001",
        "category": "transportation",
        "subcategory": "car",
        "description": "Morning commute",
        "co2Amount": 5.2,
        "unit": "kg CO2e",
        "distance": 25,
        "distanceUnit": "km",
        "fuelType": "gasoline",
        "timestamp": "2025-01-15T08:30:00Z",
        "verified": true,
        "verificationMethod": "gps_tracking"
      },
      {
        "activityId": "ACT-2025-0002",
        "category": "energy",
        "subcategory": "electricity",
        "description": "Home electricity",
        "co2Amount": 6.9,
        "unit": "kg CO2e",
        "consumption": 15,
        "consumptionUnit": "kWh",
        "timestamp": "2025-01-15T00:00:00Z",
        "verified": true,
        "verificationMethod": "smart_meter"
      },
      {
        "activityId": "ACT-2025-0003",
        "category": "food",
        "subcategory": "chicken",
        "description": "Lunch meal",
        "co2Amount": 3.4,
        "unit": "kg CO2e",
        "quantity": 0.5,
        "quantityUnit": "kg",
        "timestamp": "2025-01-15T12:30:00Z",
        "verified": true,
        "verificationMethod": "receipt_scan"
      },
      {
        "activityId": "ACT-2025-0004",
        "category": "consumption",
        "subcategory": "clothing",
        "description": "Online shopping",
        "co2Amount": 2.5,
        "unit": "kg CO2e",
        "timestamp": "2025-01-15T19:00:00Z",
        "verified": true,
        "verificationMethod": "api_integration"
      },
      {
        "activityId": "ACT-2025-0005",
        "category": "waste",
        "subcategory": "trash",
        "description": "Daily waste",
        "co2Amount": 0.5,
        "unit": "kg CO2e",
        "timestamp": "2025-01-15T20:00:00Z",
        "verified": false,
        "verificationMethod": "ai_estimation"
      }
    ],
    "categories": {
      "transportation": 5.2,
      "energy": 6.9,
      "food": 3.4,
      "consumption": 2.5,
      "waste": 0.5
    }
  },
  "verification": {
    "status": "verified",
    "verifiedBy": "WIA-Verifier-001",
    "verifiedAt": "2025-01-16T00:00:00Z",
    "confidence": 0.92
  },
  "meta": {
    "hash": "sha256:abc123def456...",
    "signature": "ed25519:signature...",
    "version": 1
  }
}
```

### 8.2 Valid Credit Transaction Record

```json
{
  "$schema": "https://wia.live/carbon-credit-micro/v1/schema.json",
  "version": "1.0.0",
  "recordId": "CCR-2025-000050",
  "userId": "USER-2025-001",
  "recordType": "transaction",
  "created": "2025-01-15T14:30:00Z",
  "carbonCredits": {
    "balance": 2.3,
    "sold": [
      {
        "transactionId": "TXN-2025-0050",
        "amount": 0.5,
        "pricePerCredit": 26.0,
        "currency": "USD",
        "totalPrice": 13.0,
        "buyer": "CORP-2025-100",
        "buyerName": "GreenTech Corp",
        "soldAt": "2025-01-15T14:30:00Z",
        "blockchainTx": "0xdef456ghi789...",
        "status": "completed"
      }
    ]
  },
  "blockchain": {
    "tokenId": "CCM-2025-0001",
    "contractAddress": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
    "network": "ethereum",
    "transactions": [
      {
        "transactionHash": "0xdef456ghi789...",
        "blockNumber": 18234890,
        "timestamp": "2025-01-15T14:30:00Z",
        "from": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb1",
        "to": "0x8ba1f109551bD432803012645Ac136ddd64DBA72",
        "type": "transfer",
        "amount": 0.5,
        "gasUsed": 45000,
        "status": "confirmed"
      }
    ]
  }
}
```

### 8.3 Invalid Example - Negative Emissions

```json
{
  "version": "1.0.0",
  "recordId": "CCR-2025-000099",
  "userId": "USER-2025-999",
  "recordType": "footprint",
  "carbonFootprint": {
    "totalEmissions": -5.0
  }
}
```

**Error**: `ERR_VALIDATION_FAILED` - Total emissions cannot be negative

### 8.4 Invalid Example - Insufficient Credits

```json
{
  "version": "1.0.0",
  "recordId": "CCR-2025-000100",
  "userId": "USER-2025-999",
  "recordType": "transaction",
  "carbonCredits": {
    "balance": 0.5,
    "sold": [
      {
        "transactionId": "TXN-2025-0100",
        "amount": 1.0
      }
    ]
  }
}
```

**Error**: `ERR_INSUFFICIENT_CREDITS` - Cannot sell more credits than available balance

### 8.5 Valid Corporate Offset Purchase

```json
{
  "$schema": "https://wia.live/carbon-credit-micro/v1/schema.json",
  "version": "1.0.0",
  "recordId": "CCR-2025-001000",
  "userId": "CORP-2025-100",
  "recordType": "offset",
  "created": "2025-01-15T16:00:00Z",
  "carbonCredits": {
    "purchased": [
      {
        "transactionId": "TXN-2025-1000",
        "amount": 100.0,
        "pricePerCredit": 25.0,
        "currency": "USD",
        "totalPrice": 2500.0,
        "sellers": [
          {
            "userId": "USER-2025-001",
            "amount": 0.5,
            "price": 12.5
          },
          {
            "userId": "USER-2025-002",
            "amount": 1.5,
            "price": 37.5
          }
        ],
        "purchasedAt": "2025-01-15T16:00:00Z",
        "purpose": "corporate_offset_2025_q1",
        "blockchainTx": "0xabc123xyz789...",
        "status": "completed"
      }
    ]
  },
  "blockchain": {
    "network": "polygon",
    "contractAddress": "0x9876543210abcdef...",
    "transactions": [
      {
        "transactionHash": "0xabc123xyz789...",
        "blockNumber": 39876543,
        "timestamp": "2025-01-15T16:00:00Z",
        "type": "batch_transfer",
        "amount": 100.0,
        "gasUsed": 150000,
        "status": "confirmed"
      }
    ]
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA Carbon Credit Micro Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>
