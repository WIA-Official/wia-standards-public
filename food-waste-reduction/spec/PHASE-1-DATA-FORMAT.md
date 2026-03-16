# WIA-AGRI-032: Food Waste Reduction Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This specification defines standardized data formats for tracking food waste, managing inventory expiration, coordinating donations, and measuring waste reduction impact across the food supply chain.

### 1.1 Design Principles

- **Interoperability**: JSON/XML formats compatible with all major retail and inventory systems
- **Real-time Tracking**: Immediate expiration and waste event recording
- **Donation Ready**: Built-in support for food bank coordination
- **Impact Measurement**: Comprehensive metrics for waste reduction and cost savings
- **Privacy**: GDPR-compliant data handling with anonymization support

---

## 2. Core Data Structures

### 2.1 Food Item Tracking Format

Standard format for tracking food items with expiration dates and waste status.

```json
{
  "foodItem": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "trackingId": "TRACK-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "facilityId": "FACILITY-2025-001",
    "location": {
      "name": "Downtown Grocery Store",
      "address": "123 Main St",
      "city": "San Francisco",
      "state": "CA",
      "country": "USA",
      "postalCode": "94102"
    },
    "product": {
      "name": "Fresh Milk",
      "sku": "DAIRY-MILK-001",
      "barcode": "012345678901",
      "category": "dairy",
      "subcategory": "milk",
      "brand": "Local Farm Co",
      "quantity": 100,
      "unit": "kg",
      "packaging": "1L cartons"
    },
    "dates": {
      "received": "2025-12-20T08:00:00.000Z",
      "manufactured": "2025-12-19T00:00:00.000Z",
      "expiration": "2025-12-31T23:59:59.000Z",
      "bestBefore": "2025-12-30T23:59:59.000Z",
      "daysUntilExpiry": 5
    },
    "status": {
      "current": "fresh",
      "previousStatus": "received",
      "statusHistory": [
        {
          "status": "received",
          "timestamp": "2025-12-20T08:00:00.000Z"
        },
        {
          "status": "fresh",
          "timestamp": "2025-12-20T09:00:00.000Z"
        }
      ]
    },
    "economics": {
      "costPerUnit": 3.50,
      "totalCost": 350.00,
      "retailPrice": 4.99,
      "potentialRevenue": 499.00,
      "currency": "USD"
    },
    "metadata": {
      "supplier": "Local Farm Distributors",
      "batchNumber": "BATCH-2025-1219",
      "temperatureZone": "refrigerated",
      "storageLocation": "Cooler-A-Shelf-3",
      "validated": true
    }
  }
}
```

**Field Descriptions:**
- `trackingId`: Unique identifier for this food item tracking record
- `timestamp`: ISO 8601 UTC timestamp of the tracking event
- `status.current`: RECEIVED, FRESH, NEAR_EXPIRY, EXPIRED, DONATED, SOLD, WASTED, COMPOSTED
- `daysUntilExpiry`: Calculated days remaining before expiration
- `temperatureZone`: AMBIENT, REFRIGERATED, FROZEN

### 2.2 Waste Event Format

Standardized format for recording food waste events.

```json
{
  "wasteEvent": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "eventId": "WASTE-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "facilityId": "FACILITY-2025-001",
    "wasteType": "expired",
    "products": [
      {
        "trackingId": "TRACK-20251226-001",
        "name": "Fresh Milk",
        "quantity": 20,
        "unit": "kg",
        "reason": "passed_expiration_date",
        "daysExpired": 2
      }
    ],
    "totals": {
      "totalWeight": 20,
      "weightUnit": "kg",
      "totalValue": 70.00,
      "currency": "USD",
      "itemCount": 20
    },
    "disposal": {
      "method": "landfill",
      "disposalDate": "2025-12-26T14:00:00.000Z",
      "hauler": "City Waste Management",
      "certificateNumber": "DISPOSE-2025-1226"
    },
    "environmental": {
      "co2Emissions": 40,
      "co2Unit": "kg",
      "waterWaste": 2000,
      "waterUnit": "liters"
    },
    "preventionAnalysis": {
      "preventable": true,
      "rootCause": "over_ordering",
      "recommendations": [
        "Reduce order quantity by 15%",
        "Improve demand forecasting",
        "Consider donation before expiration"
      ]
    },
    "metadata": {
      "recordedBy": "system_auto",
      "verified": true,
      "photos": [
        "https://storage.example.com/waste-photos/waste-001.jpg"
      ]
    }
  }
}
```

**Waste Type Values:**
- `expired`: Passed expiration date
- `spoiled`: Quality degradation before expiration
- `damaged`: Physical damage to product or packaging
- `recalled`: Product recall
- `overproduction`: Excess production
- `customer_return`: Returned by customer

**Disposal Methods:**
- `landfill`: Sent to landfill
- `composted`: Organic composting
- `anaerobic_digestion`: Biogas production
- `animal_feed`: Converted to animal feed
- `recycled`: Packaging recycled

### 2.3 Donation Record Format

Format for coordinating food donations to food banks and charities.

```json
{
  "donation": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "donationId": "DONATE-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "donor": {
      "facilityId": "FACILITY-2025-001",
      "name": "Downtown Grocery Store",
      "contactName": "John Smith",
      "contactEmail": "john@grocery.com",
      "contactPhone": "+1-555-0123"
    },
    "recipient": {
      "organizationId": "FOODBANK-2025-001",
      "name": "City Food Bank",
      "contactName": "Jane Doe",
      "contactEmail": "jane@foodbank.org",
      "contactPhone": "+1-555-0456",
      "taxId": "12-3456789"
    },
    "products": [
      {
        "trackingId": "TRACK-20251226-002",
        "name": "Fresh Bread",
        "category": "bakery",
        "quantity": 50,
        "unit": "loaves",
        "expirationDate": "2025-12-27T23:59:59.000Z",
        "estimatedValue": 150.00
      }
    ],
    "totals": {
      "totalWeight": 25,
      "weightUnit": "kg",
      "totalValue": 150.00,
      "currency": "USD",
      "itemCount": 50,
      "estimatedMeals": 120
    },
    "logistics": {
      "pickupScheduled": "2025-12-26T16:00:00.000Z",
      "pickupCompleted": null,
      "transportMethod": "recipient_pickup",
      "driver": "Food Bank Driver #3",
      "vehicle": "VAN-2025-03"
    },
    "compliance": {
      "foodSafetyChecked": true,
      "temperatureLog": [
        {
          "timestamp": "2025-12-26T10:30:00.000Z",
          "temperature": 4.5,
          "unit": "celsius",
          "location": "donor_storage"
        }
      ],
      "donationReceipt": "RECEIPT-20251226-001",
      "taxDeductible": true
    },
    "impact": {
      "mealsProvided": 120,
      "peopleServed": 40,
      "co2Prevented": 50,
      "co2Unit": "kg"
    },
    "metadata": {
      "coordinatedVia": "WIA-AGRI-032",
      "verified": true,
      "photos": []
    }
  }
}
```

### 2.4 Inventory Status Format

Real-time inventory status with expiration tracking.

```json
{
  "inventoryStatus": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "snapshotId": "SNAPSHOT-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "facilityId": "FACILITY-2025-001",
    "summary": {
      "totalProducts": 1247,
      "totalValue": 45670.00,
      "currency": "USD",
      "categoriesCount": 12
    },
    "expirationBreakdown": {
      "fresh": {
        "productCount": 980,
        "totalValue": 38500.00,
        "percentOfInventory": 78.6
      },
      "nearExpiry": {
        "productCount": 215,
        "totalValue": 6250.00,
        "percentOfInventory": 17.2,
        "daysRange": "1-3 days"
      },
      "expired": {
        "productCount": 52,
        "totalValue": 920.00,
        "percentOfInventory": 4.2,
        "actionRequired": true
      }
    },
    "categories": [
      {
        "category": "dairy",
        "productCount": 245,
        "totalValue": 8750.00,
        "nearExpiryCount": 45,
        "expiredCount": 12,
        "wasteRisk": "high"
      },
      {
        "category": "produce",
        "productCount": 412,
        "totalValue": 12300.00,
        "nearExpiryCount": 89,
        "expiredCount": 23,
        "wasteRisk": "high"
      }
    ],
    "alerts": [
      {
        "severity": "high",
        "message": "52 products expired - immediate action required",
        "affectedProducts": 52,
        "potentialLoss": 920.00
      },
      {
        "severity": "medium",
        "message": "215 products expiring within 3 days",
        "affectedProducts": 215,
        "potentialLoss": 6250.00,
        "recommendations": [
          "Mark down pricing on near-expiry items",
          "Contact food banks for donation coordination"
        ]
      }
    ],
    "metadata": {
      "generatedBy": "WIA-AGRI-032-System",
      "refreshInterval": 3600,
      "nextUpdate": "2025-12-26T11:30:00.000Z"
    }
  }
}
```

---

## 3. Data Validation Rules

### 3.1 Required Fields

**Minimum required fields for food item tracking:**
- `standardVersion`
- `trackingId`
- `timestamp`
- `facilityId`
- `product.name`
- `product.quantity`
- `product.unit`
- `dates.expiration`
- `status.current`

### 3.2 Data Types and Constraints

```javascript
{
  "trackingId": "string, format: TRACK-YYYYMMDD-NNN",
  "quantity": "number, positive, > 0",
  "costPerUnit": "number, positive, >= 0",
  "timestamp": "string, ISO 8601 UTC format",
  "status.current": "enum, see status values",
  "daysUntilExpiry": "integer, can be negative for expired items"
}
```

### 3.3 Business Logic Validation

- `daysUntilExpiry` must be calculated from `dates.expiration`
- `totalCost` must equal `quantity * costPerUnit`
- Status transitions must follow allowed paths:
  - RECEIVED → FRESH → NEAR_EXPIRY → EXPIRED
  - RECEIVED → FRESH → SOLD
  - RECEIVED → FRESH → DONATED
  - NEAR_EXPIRY → DONATED
  - EXPIRED → WASTED
  - EXPIRED → COMPOSTED

---

## 4. Integration Patterns

### 4.1 Batch Import

For bulk inventory uploads:

```json
{
  "batchImport": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "batchId": "BATCH-20251226-001",
    "timestamp": "2025-12-26T10:30:00.000Z",
    "facilityId": "FACILITY-2025-001",
    "itemCount": 500,
    "items": [
      { /* foodItem object */ },
      { /* foodItem object */ }
    ],
    "metadata": {
      "importSource": "POS_SYSTEM",
      "importMethod": "API",
      "validationPassed": true
    }
  }
}
```

### 4.2 Real-time Updates

WebSocket/MQTT message format for real-time status updates:

```json
{
  "event": "status_change",
  "trackingId": "TRACK-20251226-001",
  "oldStatus": "fresh",
  "newStatus": "near_expiry",
  "timestamp": "2025-12-26T10:30:00.000Z",
  "daysUntilExpiry": 2
}
```

---

## 5. Extensibility

### 5.1 Custom Fields

Implementations may add custom fields under `customData`:

```json
{
  "foodItem": {
    "standardVersion": "WIA-AGRI-032-v1.0",
    "trackingId": "TRACK-20251226-001",
    /* ... standard fields ... */
    "customData": {
      "internalCode": "CUSTOM-001",
      "seasonalFlag": true,
      "promotionId": "PROMO-WINTER-2025"
    }
  }
}
```

### 5.2 Version Compatibility

- Parsers MUST accept unknown fields and preserve them
- Parsers MUST validate `standardVersion` field
- Breaking changes require major version increment

---

## 6. Example Use Cases

### 6.1 Restaurant Inventory Management

Track prepared foods with short shelf life:

```json
{
  "product": {
    "name": "Prepared Salad",
    "category": "prepared",
    "quantity": 15,
    "unit": "containers",
    "customData": {
      "preparedDate": "2025-12-26T06:00:00.000Z",
      "shelfLife": 24,
      "shelfLifeUnit": "hours"
    }
  }
}
```

### 6.2 Grocery Store Donation Program

Coordinate daily donations:

```json
{
  "donation": {
    "donationId": "DONATE-DAILY-001",
    "frequency": "daily",
    "pickupTime": "18:00",
    "recurringSchedule": true
  }
}
```

---

**Next Phase:** [Phase 2: API Interface](./PHASE-2-API-INTERFACE.md)

**Related Standards:**
- WIA-AGRI-020: Agricultural Data Exchange
- WIA-SUPPLY-CHAIN: Supply Chain Transparency

---

© 2025 WIA Standards · MIT License
弘益人間 · Benefit All Humanity
