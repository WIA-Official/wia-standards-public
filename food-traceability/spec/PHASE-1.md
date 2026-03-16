# WIA-AGRI-016: Food Traceability Standard
## PHASE 1: Basic Batch Tracking & Origin Documentation

**Version:** 1.0
**Status:** Stable
**Last Updated:** 2025-12-26

---

## 1. Overview

PHASE 1 establishes the foundational infrastructure for food traceability, focusing on basic batch identification, origin documentation, and simple supply chain tracking. This phase enables organizations to implement essential traceability capabilities compliant with regulatory requirements.

### 1.1 Objectives

- Implement unique batch identification systems
- Document origin and sourcing information
- Track basic supply chain events
- Enable simple trace-back capabilities
- Establish data collection standards

### 1.2 Scope

PHASE 1 covers:
- Batch numbering and lot codes
- Origin documentation (farm, facility, geographic location)
- Harvest/production date tracking
- Basic event logging (receiving, shipping, storage)
- Simple QR code generation for batch lookup

---

## 2. Batch Identification System

### 2.1 Batch Number Format

Every product batch must have a unique identifier combining:

```
Format: [GTIN].[LOT].[SERIAL]

Example: 01234567890128.LOT2025001.0001

Components:
- GTIN (Global Trade Item Number): 14-digit product identifier
- LOT: Batch/Lot identifier (alphanumeric, 8-20 chars)
- SERIAL: Optional serial number for item-level tracking
```

### 2.2 GS1 Standards Compliance

**Required GS1 Identifiers:**

| Identifier | Purpose | Format |
|------------|---------|--------|
| GTIN | Product identification | 8, 12, 13, or 14 digits |
| GLN | Location identification | 13 digits |
| SSCC | Logistics unit | 18 digits |
| GIAI | Asset identification | Variable |

### 2.3 Batch Data Structure

```json
{
  "batchId": "01234567890128.LOT2025001",
  "gtin": "01234567890128",
  "lotNumber": "LOT2025001",
  "productName": "Organic Apples",
  "productCategory": "Fresh Produce",
  "quantity": {
    "value": 1000,
    "unit": "kg"
  },
  "createdDate": "2025-12-01T08:00:00Z",
  "expiryDate": "2025-12-31T23:59:59Z",
  "status": "active"
}
```

---

## 3. Origin Documentation

### 3.1 Source Location Data

Every batch must document its origin:

```json
{
  "origin": {
    "type": "farm",
    "name": "ABC Organic Farm",
    "gln": "1234567890123",
    "address": {
      "street": "123 Farm Road",
      "city": "Wenatchee",
      "state": "Washington",
      "country": "USA",
      "postalCode": "98801"
    },
    "geo": {
      "latitude": 47.7511,
      "longitude": -120.7401
    },
    "certifications": [
      {
        "type": "USDA Organic",
        "number": "ORG-123456",
        "issueDate": "2024-01-15",
        "expiryDate": "2026-01-15"
      }
    ]
  }
}
```

### 3.2 Harvest/Production Information

```json
{
  "production": {
    "harvestDate": "2025-12-01",
    "harvestMethod": "mechanical",
    "variety": "Gala",
    "field": "North Field #3",
    "weather": {
      "temperature": "18°C",
      "conditions": "sunny"
    },
    "operator": {
      "name": "John Smith",
      "license": "OP-98765"
    }
  }
}
```

---

## 4. Supply Chain Event Tracking

### 4.1 Event Types

PHASE 1 supports four basic event types:

1. **Production/Harvest Event**
2. **Shipping Event**
3. **Receiving Event**
4. **Storage Event**

### 4.2 Event Data Structure

```json
{
  "eventId": "evt_abc123def456",
  "eventType": "shipping",
  "timestamp": "2025-12-02T14:30:00Z",
  "batchId": "01234567890128.LOT2025001",
  "location": {
    "gln": "9876543210987",
    "name": "Distribution Center Alpha",
    "type": "warehouse"
  },
  "action": "dispatched",
  "quantity": {
    "value": 1000,
    "unit": "kg"
  },
  "destination": {
    "gln": "5555555555555",
    "name": "Retail Store #456"
  },
  "transportDetails": {
    "carrier": "ABC Logistics",
    "vehicleId": "TRUCK-789",
    "expectedArrival": "2025-12-03T08:00:00Z"
  },
  "recordedBy": {
    "userId": "user_123",
    "name": "Jane Doe"
  }
}
```

### 4.3 Event Storage

Events must be stored in chronological order with immutability guarantees:

- Append-only database structure
- Hash-chain linking for integrity verification
- Timestamps with timezone information
- Digital signatures for critical events

---

## 5. Data Collection Methods

### 5.1 Manual Data Entry

- Web-based forms for batch creation
- Mobile apps for field data collection
- Barcode/QR scanning for event logging

### 5.2 Automated Data Capture

- IoT sensors for temperature, humidity
- RFID readers for location tracking
- Scale integrations for weight measurement

### 5.3 Data Validation

```javascript
// Batch ID validation
function validateBatchId(batchId) {
  const pattern = /^\d{8,14}\.[A-Z0-9]{8,20}(\.\d+)?$/;
  return pattern.test(batchId);
}

// GLN validation (GS1 check digit)
function validateGLN(gln) {
  if (gln.length !== 13) return false;

  let sum = 0;
  for (let i = 0; i < 12; i++) {
    const digit = parseInt(gln[i]);
    sum += (i % 2 === 0) ? digit : digit * 3;
  }

  const checkDigit = (10 - (sum % 10)) % 10;
  return checkDigit === parseInt(gln[12]);
}
```

---

## 6. Simple Trace-Back System

### 6.1 Trace-Back Query

```javascript
// Trace batch history
async function traceBatch(batchId) {
  const events = await database.query({
    collection: 'events',
    filter: { batchId: batchId },
    sort: { timestamp: 1 }
  });

  return {
    batchId: batchId,
    totalEvents: events.length,
    timeline: events.map(e => ({
      date: e.timestamp,
      type: e.eventType,
      location: e.location.name,
      action: e.action
    })),
    origin: events[0]?.location,
    currentLocation: events[events.length - 1]?.location
  };
}
```

### 6.2 Trace-Back Report Format

```json
{
  "reportId": "trace_20251226_001",
  "generatedAt": "2025-12-26T10:00:00Z",
  "batchId": "01234567890128.LOT2025001",
  "product": "Organic Apples",
  "status": "in_transit",
  "path": [
    {
      "step": 1,
      "date": "2025-12-01",
      "event": "harvested",
      "location": "ABC Organic Farm, Washington, USA"
    },
    {
      "step": 2,
      "date": "2025-12-02",
      "event": "shipped",
      "location": "Distribution Center Alpha"
    },
    {
      "step": 3,
      "date": "2025-12-03",
      "event": "received",
      "location": "Retail Store #456"
    }
  ],
  "totalDistance": "250 km",
  "transitTime": "48 hours"
}
```

---

## 7. QR Code Implementation

### 7.1 QR Code Content

```json
{
  "format": "WIA-AGRI-016",
  "version": "1.0",
  "type": "batch_lookup",
  "url": "https://trace.wia.org/batch/01234567890128.LOT2025001",
  "batchId": "01234567890128.LOT2025001",
  "product": "Organic Apples",
  "origin": "ABC Organic Farm, WA, USA",
  "harvestDate": "2025-12-01"
}
```

### 7.2 Consumer-Facing Information

When consumers scan the QR code, they see:

- Product name and variety
- Origin farm/facility
- Harvest/production date
- Certifications (organic, fair trade, etc.)
- Basic supply chain path
- Contact information for questions

---

## 8. Database Schema

### 8.1 Batches Table

```sql
CREATE TABLE batches (
  id VARCHAR(100) PRIMARY KEY,
  gtin VARCHAR(14) NOT NULL,
  lot_number VARCHAR(50) NOT NULL,
  product_name VARCHAR(255) NOT NULL,
  quantity_value DECIMAL(10,2),
  quantity_unit VARCHAR(20),
  created_date TIMESTAMP NOT NULL,
  expiry_date TIMESTAMP,
  status VARCHAR(20),
  origin_gln VARCHAR(13),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
  INDEX idx_gtin (gtin),
  INDEX idx_lot (lot_number),
  INDEX idx_created (created_date)
);
```

### 8.2 Events Table

```sql
CREATE TABLE events (
  id VARCHAR(100) PRIMARY KEY,
  event_type VARCHAR(50) NOT NULL,
  batch_id VARCHAR(100) NOT NULL,
  timestamp TIMESTAMP NOT NULL,
  location_gln VARCHAR(13),
  location_name VARCHAR(255),
  action VARCHAR(50),
  quantity_value DECIMAL(10,2),
  quantity_unit VARCHAR(20),
  recorded_by VARCHAR(100),
  data JSON,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  FOREIGN KEY (batch_id) REFERENCES batches(id),
  INDEX idx_batch (batch_id),
  INDEX idx_timestamp (timestamp),
  INDEX idx_type (event_type)
);
```

---

## 9. API Endpoints

### 9.1 Batch Management

```
POST /api/v1/batches
GET /api/v1/batches/{batchId}
PUT /api/v1/batches/{batchId}
GET /api/v1/batches?gtin={gtin}&lot={lot}
```

### 9.2 Event Logging

```
POST /api/v1/events
GET /api/v1/events/{eventId}
GET /api/v1/events?batchId={batchId}
GET /api/v1/events?location={gln}&date={date}
```

### 9.3 Trace-Back

```
GET /api/v1/trace/{batchId}
GET /api/v1/trace/{batchId}/report
```

---

## 10. Compliance & Regulations

### 10.1 Supported Regulations

- **FDA FSMA** (Food Safety Modernization Act)
- **EU Regulation 178/2002** (General Food Law)
- **Korea's Agricultural Products Quality Control Act**
- **China's Food Safety Law**

### 10.2 Recordkeeping Requirements

- Minimum 2-year retention for batch records
- 5-year retention for food safety incidents
- Audit trail for all data modifications
- Backup and disaster recovery procedures

---

## 11. Implementation Checklist

- [ ] Set up batch numbering system with GS1 GTIN
- [ ] Create origin documentation templates
- [ ] Implement event logging system
- [ ] Deploy batch tracking database
- [ ] Develop QR code generation
- [ ] Create consumer-facing lookup portal
- [ ] Train staff on data entry procedures
- [ ] Establish data validation rules
- [ ] Configure backup systems
- [ ] Conduct pilot testing with sample batches

---

## 12. Next Steps to PHASE 2

PHASE 2 will introduce:
- Processing and transformation tracking
- Multi-ingredient batch associations
- Quality control data integration
- Enhanced EPCIS event types
- Temperature and condition monitoring

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Cycle:** Annual
- **Next Review:** 2026-12-26

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
