# WIA-IND-007 Phase 1: Data Format Specification
## Food Safety Traceability Standard v1.0

**弘益人間 (Hongik Ingan) - Benefit All Humanity**

---

## Overview

Phase 1 establishes standardized data formats for capturing, storing, and exchanging food traceability information throughout the supply chain. These formats ensure interoperability between diverse systems while accommodating product-specific requirements.

## 1. Product Identification Schema

### 1.1 Unique Product Identifiers

All food products must carry globally unique identifiers enabling precise tracking and recall management.

**Primary Identifiers:**
- **GTIN (Global Trade Item Number):** 14-digit standardized product identifier
- **Batch/Lot Number:** Alphanumeric code linking products to production runs
- **Serial Number:** Optional item-level identifier for high-value products

**JSON Schema:**
```json
{
  "product": {
    "gtin": "01234567890128",
    "productName": "Organic Lettuce",
    "batchNumber": "BATCH-2025-001",
    "serialNumber": "SN-2025-001-00001",
    "productionDate": "2025-12-20T08:00:00Z",
    "expirationDate": "2026-01-03T23:59:59Z",
    "standardVersion": "WIA-IND-007-v1.0"
  }
}
```

### 1.2 Product Classification

Categories enable appropriate handling and regulatory compliance:

**Category Codes:**
- `PRODUCE` - Fresh fruits and vegetables
- `MEAT` - Meat and poultry products
- `SEAFOOD` - Fish and shellfish
- `DAIRY` - Milk, cheese, yogurt products
- `PACKAGED` - Processed packaged foods
- `BEVERAGES` - Drinks and liquid products

## 2. Origin and Provenance Data

### 2.1 Source Location Schema

Detailed origin information enables source tracing during contamination events.

**Required Fields:**
- Producer/Farm identification
- Geographic coordinates (latitude/longitude)
- Country, region, and locality
- Production system type (organic, conventional, etc.)

**XML Format:**
```xml
<origin>
  <producerId>FARM-ABC-001</producerId>
  <producerName>ABC Organic Farms</producerName>
  <location>
    <address>123 Farm Road, Salinas, CA 93901, USA</address>
    <coordinates>
      <latitude>36.7783</latitude>
      <longitude>-119.4179</longitude>
    </coordinates>
  </location>
  <productionSystem>organic</productionSystem>
  <certifications>
    <certification>
      <type>USDA Organic</type>
      <certificationNumber>USDA-ORG-2024-12345</certificationNumber>
      <issueDate>2024-01-15</issueDate>
      <expiryDate>2025-01-14</expiryDate>
    </certification>
  </certifications>
</origin>
```

### 2.2 Harvest and Collection Data

Capture critical information at the point of origin:

- Harvest date and time
- Field/lot identification
- Weather conditions
- Equipment used
- Worker identifications (anonymized for privacy)

## 3. Batch Tracking Records

### 3.1 Batch Metadata

Each production batch requires comprehensive metadata:

**Batch Record Structure:**
```json
{
  "batchRecord": {
    "batchId": "BATCH-2025-001",
    "productGtin": "01234567890128",
    "productionFacility": "FACILITY-CA-001",
    "productionDate": "2025-12-20",
    "productionShift": "A",
    "quantity": {
      "value": 10000,
      "unit": "kg"
    },
    "ingredientLots": [
      {
        "ingredientName": "Organic Lettuce Seeds",
        "lotNumber": "SEED-LOT-2024-456",
        "supplier": "SUPPLIER-001",
        "quantity": 50,
        "unit": "kg"
      }
    ],
    "qualityTests": [
      {
        "testType": "Microbial Analysis",
        "testDate": "2025-12-20T14:00:00Z",
        "laboratory": "LAB-001",
        "results": {
          "salmonella": "not-detected",
          "ecoli": "not-detected",
          "listeria": "not-detected"
        },
        "certificationNumber": "CERT-2025-12345"
      }
    ]
  }
}
```

### 3.2 Lot Code Standards

Lot codes must be:
- Unique within organization
- Human-readable
- Machine-scannable (barcode/QR code)
- Include production date encoding
- Resist tampering or degradation

**Recommended Format:**
`[FACILITY]-[PRODUCT]-[YEAR][MONTH][DAY]-[SEQUENCE]`

Example: `CA01-LETTUCE-20251220-001`

## 4. Temperature and Environmental Logs

### 4.1 Cold Chain Monitoring

Temperature-sensitive products require continuous monitoring:

**Temperature Log Format:**
```json
{
  "temperatureLog": [
    {
      "timestamp": "2025-12-20T10:00:00Z",
      "temperature": 4.2,
      "unit": "celsius",
      "location": "Storage Facility A",
      "sensorId": "TEMP-SENSOR-001",
      "withinRange": true,
      "acceptableRange": {
        "min": 2.0,
        "max": 6.0
      }
    }
  ]
}
```

### 4.2 Environmental Conditions

Capture relevant environmental factors:

- Humidity levels
- Light exposure
- Atmospheric composition (for controlled atmosphere storage)
- Shock/vibration events during transport

## 5. Handling and Processing Records

### 5.1 Supply Chain Events

Document each custody transfer and processing step:

**Event Schema:**
```json
{
  "supplyChainEvent": {
    "eventId": "EVT-2025-001",
    "timestamp": "2025-12-20T12:00:00Z",
    "eventType": "PROCESSING",
    "location": {
      "facility": "PROCESSING-FACILITY-001",
      "address": "456 Processing St, Salinas, CA"
    },
    "batchNumber": "BATCH-2025-001",
    "previousCustodian": "FARM-ABC-001",
    "newCustodian": "PROCESSOR-001",
    "processingDetails": {
      "operations": ["washing", "trimming", "packaging"],
      "equipment": ["WASHER-001", "TRIMMER-002", "PACKAGER-003"],
      "duration": 120,
      "temperatureRange": "4-6C"
    },
    "qualityChecks": {
      "visualInspection": "pass",
      "foreignMatterCheck": "pass",
      "temperatureCheck": "pass"
    }
  }
}
```

### 5.2 Transformation Events

When products are transformed (e.g., ingredients into finished goods):

- Input batch numbers and quantities
- Output batch numbers and quantities
- Recipe/formulation identifier
- Processing parameters (time, temperature, pressure)
- Yield percentage

## 6. Certification and Compliance Data

### 6.1 Food Safety Certifications

Link products to relevant certifications:

**Certification Types:**
- Organic certifications (USDA, EU, JAS, etc.)
- Food safety certifications (HACCP, SQF, BRC, etc.)
- Quality standards (ISO 22000, FSSC 22000)
- Sustainability certifications (Fair Trade, Rainforest Alliance)
- Religious certifications (Kosher, Halal)

**Data Structure:**
```json
{
  "certifications": [
    {
      "certificationType": "USDA Organic",
      "certificationBody": "Quality Assurance International",
      "certificateNumber": "USDA-ORG-2024-12345",
      "issueDate": "2024-01-15",
      "expiryDate": "2025-01-14",
      "scope": "Production and handling of organic produce",
      "verificationMethod": "blockchain-hash",
      "verificationHash": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb"
    }
  ]
}
```

### 6.2 Regulatory Compliance Records

Document compliance with food safety regulations:

- FDA Food Facility Registration Number (FFR)
- HACCP plan documentation
- Preventive controls validation
- Foreign supplier verification program (FSVP) records

## 7. Allergen and Ingredient Declarations

### 7.1 Allergen Information

Critical for consumer safety and regulatory compliance:

**Major Allergens (FDA):**
- Milk
- Eggs
- Fish
- Shellfish
- Tree nuts
- Peanuts
- Wheat
- Soybeans
- Sesame

**Allergen Declaration Format:**
```json
{
  "allergenInfo": {
    "containsAllergens": ["wheat", "soy"],
    "mayContainAllergens": ["tree nuts"],
    "allergenFreeFrom": ["milk", "eggs", "fish", "shellfish", "peanuts", "sesame"],
    "crossContactRisk": "low",
    "allergenControlMeasures": [
      "Dedicated production line",
      "Thorough cleaning between runs",
      "Allergen testing verification"
    ]
  }
}
```

### 7.2 Ingredient Composition

Full ingredient traceability:

- Ingredient name (common and scientific)
- Percentage by weight
- Source supplier and lot number
- Origin country
- Processing aids and additives

## 8. Packaging and Labeling Data

### 8.1 Package Information

Link packaging to product contents:

**Package Data:**
```json
{
  "packaging": {
    "packageType": "clamshell",
    "packageSize": "500g",
    "packagingMaterial": "recyclable-PET",
    "packagingDate": "2025-12-20T16:00:00Z",
    "labelingCompliance": {
      "nutritionFacts": true,
      "allergenDeclaration": true,
      "countryOfOrigin": true,
      "traceabilityQRCode": true
    },
    "qrCodeData": "https://trace.wia.org/product/01234567890128/batch/BATCH-2025-001"
  }
}
```

### 8.2 Label Requirements

Ensure labels meet regulatory and standard requirements:

- Product name and description
- Net weight/volume
- Nutrition facts panel
- Allergen declarations
- Producer/packer/distributor information
- Country of origin
- Traceability QR code or URL

## 9. Data Validation and Integrity

### 9.1 Data Quality Rules

All traceability data must meet quality standards:

**Validation Requirements:**
- Required fields must be non-null
- Dates must be in ISO 8601 format
- Coordinates must use WGS84 datum
- Quantities must include units
- Temperature readings must include timestamps
- Batch numbers must be unique within scope

### 9.2 Cryptographic Verification

Ensure data integrity using cryptographic techniques:

**Hash Verification:**
```json
{
  "dataIntegrity": {
    "recordHash": "SHA256:a4b89f3c2e1d...",
    "previousRecordHash": "SHA256:e1f2a3b4c5d6...",
    "timestamp": "2025-12-20T10:00:00Z",
    "digitalSignature": "0x742d35Cc6634C0532925a3b844Bc9e7595f0bEb",
    "signingAuthority": "PROCESSOR-001",
    "blockchainAnchor": {
      "network": "Ethereum",
      "transactionHash": "0x1234567890abcdef...",
      "blockNumber": 15234567
    }
  }
}
```

## 10. Interoperability Standards

### 10.1 Data Exchange Formats

Support multiple serialization formats:

- **JSON:** Primary format for APIs and web services
- **XML:** Legacy system compatibility
- **CSV:** Bulk data exchange and reporting
- **EDI:** B2B transaction integration

### 10.2 Character Encoding

All textual data must use UTF-8 encoding to support international characters and multilingual content.

---

## Implementation Guidance

### Phased Adoption

Organizations may implement data format standards incrementally:

1. **Phase 1A:** Product identification and batch tracking
2. **Phase 1B:** Temperature logging and environmental monitoring
3. **Phase 1C:** Certification and compliance documentation
4. **Phase 1D:** Full integration with blockchain verification

### Data Storage Recommendations

- **Retention Period:** Minimum 2 years, or longer per regulatory requirements
- **Backup Frequency:** Daily incremental, weekly full backups
- **Access Controls:** Role-based access with audit logging
- **Encryption:** At-rest encryption using AES-256 or equivalent

---

## Compliance Checklist

- [ ] Product identifiers include GTIN and batch numbers
- [ ] Origin data captures producer and geographic location
- [ ] Temperature logs record continuous cold chain monitoring
- [ ] Supply chain events document all custody transfers
- [ ] Certifications are verified and linked to products
- [ ] Allergen information is complete and accurate
- [ ] Data validation rules are implemented
- [ ] Cryptographic verification ensures data integrity
- [ ] Multiple data exchange formats are supported
- [ ] Data retention and backup procedures are established

---

**Document Version:** 1.0
**Last Updated:** 2025-12-27
**Status:** Official Release
**弘益人間 - Benefit All Humanity through Safe Food Systems**

© 2025 SmileStory Inc. / WIA (World Certification Industry Association)
