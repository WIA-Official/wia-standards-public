# Chapter 2: GS1 Standards & Identification Systems

**WIA-AGRI-016 eBook Series**

---

## Introduction to GS1

GS1 is a global organization that develops and maintains standards for business communication. Their identification standards are the foundation of modern traceability systems, used by over 2 million companies in 150 countries.

### Why GS1 Standards Matter

**Universal Language:** GS1 standards create a common language for identifying products, locations, and assets across the entire supply chain, regardless of company, industry, or country.

**Interoperability:** Systems using GS1 standards can automatically exchange data without custom integration.

**Proven at Scale:** Used for over 40 years, processing billions of transactions daily.

---

## Core GS1 Identifiers

### 1. GTIN (Global Trade Item Number)

**Purpose:** Uniquely identifies products

**Format:** 8, 12, 13, or 14 digits

**Structure:**
```
GTIN-14: 01234567890128
         ││└─ Product Reference (7 digits)
         │└── Company Prefix (6 digits)
         └─── Indicator Digit (1 digit)
         └─── Check Digit (1 digit)
```

**Example:**
```
GTIN: 00012345678905
- Company Prefix: 0001234 (assigned by GS1)
- Item Reference: 56789 (assigned by company)
- Check Digit: 5 (calculated)
```

**When to Use:**
- Different product types (apple vs. orange)
- Different brands
- Different package sizes
- Different formulations

**NOT for:**
- Different batches of same product (use Lot Number)
- Individual items (use Serial Number)

### 2. GLN (Global Location Number)

**Purpose:** Uniquely identifies physical locations and legal entities

**Format:** 13 digits

**Examples:**
```
Farm Location:        1234567890123
Packing House:        1234567890124
Distribution Center:  1234567890125
Retail Store #456:    1234567890126
```

**Granularity Levels:**
- Legal Entity: ABC Organic Farm Inc.
- Function: ABC Organic Farm - Quality Control Dept
- Physical Location: ABC Organic Farm - Warehouse Building 2
- Sub-location: ABC Organic Farm - Cold Storage Room 3

**Benefits:**
- Precise location tracking
- Automated routing
- Regulatory compliance
- Invoice reconciliation

### 3. SSCC (Serial Shipping Container Code)

**Purpose:** Uniquely identifies logistics units (pallets, containers)

**Format:** 18 digits

**Structure:**
```
SSCC: 006543210987654321
      ││└── Serial Reference (9 digits)
      │└─── Company Prefix (7 digits)
      └──── Extension Digit (1 digit)
      └──── Check Digit (1 digit)
```

**Use Cases:**
- Pallet tracking in warehouse
- Shipment tracking during transport
- Receiving verification
- Inventory management

**Lifecycle:**
1. Pallet packed → SSCC assigned
2. Barcode label applied
3. Scanned at shipping dock
4. Tracked in transit
5. Scanned at receiving dock
6. Pallet broken down → SSCC retired

### 4. GIAI (Global Individual Asset Identifier)

**Purpose:** Uniquely identifies fixed assets

**Format:** Variable length (up to 30 characters)

**Examples:**
```
Pasteurization Unit #3: 0012345PASTEUR003
Refrigerated Truck #42: 0012345TRUCK042
Sorting Machine #5:     0012345SORT005
```

**Applications:**
- Equipment maintenance tracking
- Processing parameter logging
- Asset lifecycle management
- Calibration scheduling

---

## Check Digit Calculation

GS1 identifiers use a check digit to detect data entry errors.

### Algorithm (Modulo 10)

```python
def calculate_check_digit(identifier):
    """
    Calculate GS1 check digit using modulo 10 algorithm
    """
    # Remove existing check digit if present
    digits = identifier[:-1] if len(identifier) == 14 else identifier

    # Calculate weighted sum
    odd_sum = sum(int(d) for d in digits[-1::-2])   # positions 1,3,5... from right
    even_sum = sum(int(d) for d in digits[-2::-2])  # positions 2,4,6... from right

    total = (odd_sum * 3) + even_sum

    # Check digit
    check_digit = (10 - (total % 10)) % 10

    return str(check_digit)

# Example
gtin = "0123456789012"
check = calculate_check_digit(gtin)  # Returns: "8"
full_gtin = gtin + check  # "01234567890128"
```

---

## Barcodes

### 1. EAN/UPC Barcodes

**EAN-13:** Most common for retail products in Europe/Asia
**UPC-A:** Most common for retail products in North America

**Characteristics:**
- 1D linear barcode
- Encodes GTIN
- Omnidirectional scanning
- Print size: Minimum 37.29mm × 25.93mm

**When to Use:**
- Retail point-of-sale scanning
- Consumer-facing products
- High-volume scanning environments

### 2. GS1-128 Barcode

**Purpose:** Encodes GTIN plus additional data (batch, dates, serial numbers)

**Application Identifiers (AIs):**
```
(01) GTIN: 01234567890128
(10) Batch/Lot: LOT2025001
(17) Expiry Date: 260630 (June 30, 2026)
(21) Serial Number: 123456

Full Barcode Data:
(01)01234567890128(10)LOT2025001(17)260630(21)123456
```

**Common AIs for Food Traceability:**

| AI | Data | Format | Example |
|----|------|--------|---------|
| (01) | GTIN | N14 | 01234567890128 |
| (10) | Batch/Lot | X..20 | LOT2025001 |
| (11) | Production Date | N6 (YYMMDD) | 251201 |
| (13) | Packaging Date | N6 (YYMMDD) | 251202 |
| (15) | Best Before Date | N6 (YYMMDD) | 260601 |
| (17) | Expiry Date | N6 (YYMMDD) | 260630 |
| (21) | Serial Number | X..20 | 123456 |
| (3103) | Net Weight (kg) | N6 | 001000 (100.0 kg) |
| (414) | GLN (Ship To) | N13 | 1234567890123 |
| (420) | Ship To Postal Code | X..20 | 98801 |

**Benefits:**
- Multiple data elements in single barcode
- Machine-readable batch and date information
- Automated data capture
- Reduced manual entry errors

### 3. GS1 DataMatrix

**Purpose:** 2D barcode for small items and direct part marking

**Characteristics:**
- Can encode more data than 1D barcodes
- Smaller print size possible
- Error correction built-in
- Can be laser-etched or printed

**When to Use:**
- Small products (medical devices, cosmetics)
- Direct marking on products
- High data density requirements
- Harsh environments

### 4. GS1 QR Code

**Purpose:** Mobile-friendly 2D barcode for consumer engagement

**Data Structure:**
```
https://id.gs1.org/01/01234567890128/10/LOT2025001/21/123456

Decoded:
- GTIN: 01234567890128
- Batch: LOT2025001
- Serial: 123456
```

**Benefits:**
- Smartphone scanning (no special equipment)
- Link to digital product information
- Consumer traceability access
- Marketing and engagement

**GS1 Digital Link:**
```
https://brand.example.com/01/01234567890128/10/LOT2025001
?linkType=traceability
```

When scanned, redirects to product traceability page.

---

## Implementing GS1 Identifiers

### Step 1: Join GS1

1. Contact your local GS1 Member Organization
2. Purchase a company prefix
3. Receive your unique prefix (e.g., 0123456)

**Costs:**
- Initial fee: $250-$10,000 (varies by country and prefix size)
- Annual renewal: $150-$2,000

### Step 2: Assign GTINs

```javascript
// Example GTIN assignment system
class GTINManager {
  constructor(companyPrefix) {
    this.companyPrefix = companyPrefix; // e.g., "0123456"
    this.itemCounter = 0;
  }

  assignGTIN(productName) {
    this.itemCounter++;

    const indicatorDigit = "0";
    const itemReference = this.itemCounter.toString().padStart(5, '0');

    const baseGTIN = indicatorDigit + this.companyPrefix + itemReference;
    const checkDigit = this.calculateCheckDigit(baseGTIN);

    const gtin = baseGTIN + checkDigit;

    // Store in database
    this.saveGTIN({
      gtin: gtin,
      productName: productName,
      assignedDate: new Date().toISOString()
    });

    return gtin;
  }

  calculateCheckDigit(gtin13) {
    let sum = 0;
    for (let i = 0; i < 13; i++) {
      const digit = parseInt(gtin13[i]);
      sum += (i % 2 === 0) ? digit * 1 : digit * 3;
    }
    return ((10 - (sum % 10)) % 10).toString();
  }
}

// Usage
const gtinManager = new GTINManager("0123456");
const appleGTIN = gtinManager.assignGTIN("Organic Apples 1kg");
// Returns: "00123456000018"
```

### Step 3: Assign GLNs

```javascript
class GLNManager {
  constructor(companyPrefix) {
    this.companyPrefix = companyPrefix;
    this.locationCounter = 0;
  }

  assignGLN(locationName, address) {
    this.locationCounter++;

    const locationReference = this.locationCounter.toString().padStart(5, '0');
    const baseGLN = this.companyPrefix + locationReference;
    const checkDigit = this.calculateCheckDigit(baseGLN);

    const gln = baseGLN + checkDigit;

    this.saveGLN({
      gln: gln,
      locationName: locationName,
      address: address,
      assignedDate: new Date().toISOString()
    });

    return gln;
  }
}

// Usage
const glnManager = new GLNManager("0123456");
const warehouseGLN = glnManager.assignGLN(
  "Main Warehouse",
  "123 Farm Road, Wenatchee, WA 98801"
);
```

### Step 4: Generate Barcodes

```javascript
// Generate GS1-128 barcode with AI data
function generateGS1_128(data) {
  const barcode = {
    symbology: "GS1-128",
    data: `(01)${data.gtin}(10)${data.batch}(17)${data.expiryDate}`,
    humanReadable: true,
    barcodeHeight: 50,  // mm
    barcodeWidth: 150,  // mm
    quietZone: 10       // mm
  };

  // Use barcode generation library
  return generateBarcode(barcode);
}

// Generate GS1 Digital Link QR Code
function generateGS1QR(data) {
  const url = `https://id.gs1.org/01/${data.gtin}/10/${data.batch}/21/${data.serial}`;

  return generateQRCode({
    data: url,
    errorCorrection: 'M',
    version: 'auto',
    quietZone: 4
  });
}
```

---

## Best Practices

### 1. Standardize Your Naming

**Good:**
```
Product: Organic Apples 1kg
GTIN: 00123456000018
```

**Bad:**
```
Product: Apples-org-1000g
GTIN: (manually created, no standard)
```

### 2. Document Your GTINs

Maintain a master database:
```sql
CREATE TABLE product_catalog (
  gtin VARCHAR(14) PRIMARY KEY,
  product_name VARCHAR(255) NOT NULL,
  brand VARCHAR(100),
  category VARCHAR(100),
  net_content_value DECIMAL(10,3),
  net_content_unit VARCHAR(10),
  assigned_date DATE,
  status VARCHAR(20),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### 3. Validate All Identifiers

```javascript
function validateGTIN(gtin) {
  // Check length
  if (![8, 12, 13, 14].includes(gtin.length)) {
    return { valid: false, error: "Invalid length" };
  }

  // Check digits only
  if (!/^\d+$/.test(gtin)) {
    return { valid: false, error: "Must contain only digits" };
  }

  // Validate check digit
  const calculatedCheck = calculateCheckDigit(gtin);
  const providedCheck = gtin.slice(-1);

  if (calculatedCheck !== providedCheck) {
    return { valid: false, error: "Invalid check digit" };
  }

  return { valid: true };
}
```

### 4. Plan for Growth

Reserve capacity in your numbering:
- Small company (< 100 products): 6-digit prefix OK
- Medium company (100-1000 products): 7-digit prefix better
- Large company (1000+ products): Consider multiple prefixes

---

## Integration with Traceability

### Batch Identification Format

Combine GTIN with Lot Number:
```
Format: GTIN + Lot Number
Example: 01234567890128.LOT2025001

Database storage:
{
  "batchId": "01234567890128.LOT2025001",
  "gtin": "01234567890128",
  "lot": "LOT2025001"
}
```

### Event Location Identification

Use GLN for all location references:
```json
{
  "eventId": "evt_001",
  "eventType": "shipping",
  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.0",
    "gln": "0123456000010",
    "name": "Warehouse Dock 3"
  },
  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0",
    "gln": "0123456000010",
    "name": "Main Distribution Center"
  }
}
```

---

## Chapter Summary

GS1 standards provide the universal language for food traceability:

**Key Identifiers:**
- **GTIN:** Products
- **GLN:** Locations
- **SSCC:** Logistics units
- **GIAI:** Assets

**Implementation Steps:**
1. Join GS1 and get company prefix
2. Assign GTINs systematically
3. Assign GLNs to all locations
4. Generate compliant barcodes
5. Validate all identifiers

**Benefits:**
- Global interoperability
- Automated data capture
- Regulatory compliance
- Supply chain efficiency

---

## Next Chapter

**Chapter 3: EPCIS Event Architecture**

Learn how to implement GS1 EPCIS for capturing and sharing traceability events across the supply chain.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
