# WIA-AGRI-015: Food Safety Standard
## Phase 1 - Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines standardized data formats for food safety management systems, HACCP compliance tracking, contamination detection, product traceability, and consumer protection.

### 1.1 Design Principles

- **Traceability**: Complete chain of custody from farm to table
- **Real-time Safety**: Immediate contamination alerts and recall triggers
- **Regulatory Compliance**: FDA FSMA, USDA FSIS, MFDS (Korea) integration
- **Consumer Protection**: Transparent allergen, expiry, and safety information
- **Global Standards**: HACCP, ISO 22000, GMP, GFSI compatible

---

## 2. Core Data Structures

### 2.1 Food Product Entity

```json
{
  "productId": "string (UUID)",
  "productName": "string",
  "category": "string (enum: fresh-produce, dairy, meat, seafood, processed, beverages)",
  "batchNumber": "string",
  "lotNumber": "string",
  "productionDate": "string (ISO 8601)",
  "expiryDate": "string (ISO 8601)",
  "manufacturer": {
    "id": "string (UUID)",
    "name": "string",
    "facility": "string",
    "location": {
      "address": "string",
      "city": "string",
      "country": "string (ISO 3166-1 alpha-2)",
      "gps": {"latitude": "number", "longitude": "number"}
    },
    "licenses": ["string (license IDs)"],
    "did": "string (W3C DID)"
  },
  "ingredients": [
    {
      "name": "string",
      "percentage": "number",
      "origin": "string",
      "allergen": "boolean"
    }
  ],
  "allergens": {
    "contains": ["string (allergen names)"],
    "mayContain": ["string (trace allergens)"],
    "free": ["string (allergen-free claims)"]
  },
  "certifications": ["HACCP", "ISO 22000", "Organic", "Halal", "Kosher"],
  "blockchain": {
    "enabled": "boolean",
    "network": "string",
    "contractAddress": "string",
    "transactionHash": "string"
  }
}
```

### 2.2 Safety Monitoring Data

```json
{
  "monitoringId": "string (UUID)",
  "productId": "string (UUID)",
  "timestamp": "string (ISO 8601 with timezone)",
  "location": "string (facility, transport, storage, retail)",
  "temperature": {
    "current": "number (Celsius)",
    "min": "number (Celsius)",
    "max": "number (Celsius)",
    "criticalLimit": "number (Celsius)",
    "status": "string (enum: safe, warning, critical)"
  },
  "humidity": {
    "current": "number (percentage 0-100)",
    "acceptable_range": {"min": "number", "max": "number"}
  },
  "contamination_tests": [
    {
      "testId": "string (UUID)",
      "testType": "string (enum: microbial, chemical, physical, allergen)",
      "parameter": "string (e.g., Salmonella, E.coli, Listeria, Heavy Metals)",
      "result": "number or string",
      "unit": "string",
      "threshold": "number",
      "status": "string (enum: pass, fail, pending)",
      "labId": "string",
      "testDate": "string (ISO 8601)",
      "certifiedBy": "string (DID)"
    }
  ],
  "haccp_ccp": [
    {
      "ccpNumber": "number (Critical Control Point)",
      "description": "string",
      "criticalLimit": "string",
      "monitoringProcedure": "string",
      "measuredValue": "number",
      "status": "string (enum: within-limit, deviation, corrective-action)",
      "correctiveAction": "string",
      "verifiedBy": "string (inspector DID)",
      "timestamp": "string (ISO 8601)"
    }
  ]
}
```

### 2.3 Recall Data Structure

```json
{
  "recallId": "string (UUID)",
  "timestamp": "string (ISO 8601)",
  "severity": "string (enum: class-I, class-II, class-III)",
  "reason": "string",
  "affectedProducts": [
    {
      "productId": "string (UUID)",
      "batchNumbers": ["string"],
      "distributionDates": {"start": "string", "end": "string"},
      "quantity": "number",
      "unit": "string"
    }
  ],
  "contamination": {
    "type": "string (e.g., Salmonella, Listeria, Foreign Object)",
    "source": "string",
    "detectionMethod": "string"
  },
  "geographicArea": ["string (countries/regions)"],
  "retailers": ["string (retailer IDs)"],
  "consumerActions": "string",
  "regulatoryAgencies": [
    {
      "agency": "string (FDA, USDA, MFDS)",
      "notificationDate": "string (ISO 8601)",
      "caseNumber": "string"
    }
  ],
  "status": "string (enum: active, completed, monitoring)",
  "mediaContacts": ["string"]
}
```

### 2.4 Traceability Chain

```json
{
  "traceId": "string (UUID)",
  "productId": "string (UUID)",
  "chainOfCustody": [
    {
      "stepNumber": "number",
      "entity": "string (company name)",
      "did": "string (W3C DID)",
      "role": "string (enum: producer, processor, distributor, retailer)",
      "location": {
        "facility": "string",
        "address": "string",
        "gps": {"latitude": "number", "longitude": "number"}
      },
      "timestamp": {
        "received": "string (ISO 8601)",
        "shipped": "string (ISO 8601)"
      },
      "temperature_log": [
        {"timestamp": "string", "temperature": "number"}
      ],
      "inspections": [
        {
          "inspector": "string (DID)",
          "result": "string (enum: pass, fail)",
          "notes": "string"
        }
      ],
      "signature": "string (cryptographic signature)"
    }
  ],
  "blockchain_proof": {
    "merkleRoot": "string",
    "blockNumber": "number",
    "timestamp": "string (ISO 8601)"
  }
}
```

---

## 3. Field Definitions

### 3.1 Product Categories

| Category | Description | Storage Temp Range | Typical Shelf Life |
|----------|-------------|-------------------|-------------------|
| `fresh-produce` | Fruits, vegetables | 0-10°C | 3-14 days |
| `dairy` | Milk, cheese, yogurt | 0-4°C | 7-30 days |
| `meat` | Beef, pork, poultry | -2 to 4°C | 3-7 days fresh, 3-12 months frozen |
| `seafood` | Fish, shellfish | -2 to 2°C | 1-3 days fresh, 3-6 months frozen |
| `processed` | Canned, packaged | Room temp | 6-24 months |
| `beverages` | Juice, milk, water | Varies | Varies by type |

### 3.2 Recall Severity Classification

| Class | Definition | Health Risk |
|-------|------------|-------------|
| `class-I` | Life-threatening or serious injury | High |
| `class-II` | Temporary or medically reversible | Medium |
| `class-III` | Unlikely to cause adverse health | Low |

### 3.3 HACCP Critical Control Points

| CCP # | Control Point | Critical Limit Example |
|-------|---------------|----------------------|
| 1 | Receiving | Temperature ≤ 4°C |
| 2 | Cooking | Internal temp ≥ 74°C |
| 3 | Cooling | 60°C to 21°C in 2 hours |
| 4 | Storage | Temperature 0-4°C |
| 5 | Reheating | Temperature ≥ 74°C |

---

## 4. Validation Rules

### 4.1 Temperature Monitoring
- Readings every 15 minutes for refrigerated storage
- Immediate alert if temperature exceeds critical limit for > 2 hours
- Automatic log retention for 3 years (FDA requirement)

### 4.2 Expiry Date Validation
```javascript
expiryDate > currentDate  // Product must be within shelf life
expiryDate - productionDate ≤ maximumShelfLife  // Shelf life reasonable
```

### 4.3 Batch Number Format
```regex
^[A-Z0-9]{4}-[0-9]{8}-[0-9]{3}$
// Example: FARM-20250115-001
```

### 4.4 Required Allergen Declarations
- Must declare: Milk, Eggs, Fish, Shellfish, Tree nuts, Peanuts, Wheat, Soybeans
- Additional (Korea): Buckwheat, Mackerel, Crab, Shrimp, Pork, Peach, Tomato
- Additional (EU): Celery, Mustard, Sesame, Sulphites, Lupin, Molluscs

---

## 5. Examples

### 5.1 Complete Food Safety Record

```json
{
  "productId": "FS-2025-STR-001",
  "productName": "Fresh Strawberries",
  "category": "fresh-produce",
  "batchNumber": "FARM-20250115-001",
  "productionDate": "2025-01-15T06:00:00Z",
  "expiryDate": "2025-01-22T23:59:59Z",
  "manufacturer": {
    "id": "MFR-12345",
    "name": "Sunshine Farms",
    "facility": "Farm A, California",
    "licenses": ["CA-FARM-2024-567"],
    "did": "did:wia:manufacturer:sunshine-farms"
  },
  "allergens": {
    "contains": [],
    "mayContain": [],
    "free": ["Gluten", "Dairy", "Nuts"]
  },
  "certifications": ["HACCP", "Organic-USDA", "GlobalGAP"],
  "safety_monitoring": {
    "temperature": {"current": 4.2, "status": "safe"},
    "contamination_tests": [
      {
        "testType": "microbial",
        "parameter": "E.coli",
        "result": "negative",
        "status": "pass",
        "testDate": "2025-01-15T14:00:00Z"
      }
    ]
  }
}
```

---

## 6. Integration Requirements

### 6.1 Regulatory Compliance
- **FDA FSMA**: Food Safety Modernization Act compliance
- **USDA FSIS**: Food Safety and Inspection Service reporting
- **MFDS (Korea)**: 식품의약품안전처 integration
- **EFSA (EU)**: European Food Safety Authority

### 6.2 Blockchain Integration
- Ethereum/Polygon for traceability records
- IPFS for document storage (test results, certificates)
- Verifiable Credentials (W3C VC) for certifications

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
