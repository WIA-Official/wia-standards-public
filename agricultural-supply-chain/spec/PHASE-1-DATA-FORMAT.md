# WIA-AGRI-014: Agricultural Supply Chain Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-01-01

---

## 1. Overview

This specification defines the data formats for agricultural supply chain operations, including shipment tracking, cold chain monitoring, provenance records, and quality control data.

### 1.1 Design Principles

- **Traceability**: Complete farm-to-fork tracking with immutable records
- **Interoperability**: Standard JSON/XML formats compatible with major logistics platforms
- **Real-time**: IoT sensor data with sub-second granularity for cold chain monitoring
- **Blockchain-ready**: Data structures optimized for distributed ledger recording

---

## 2. Core Data Structures

### 2.1 Shipment Data Format

Complete shipment information for agricultural products.

```json
{
  "shipment": {
    "shipmentId": "SHIP-2025-001",
    "timestamp": "2025-01-01T10:00:00.000Z",
    "productInfo": {
      "productId": "PROD-TOMATO-001",
      "productName": "Organic Tomatoes",
      "productType": "FRESH_PRODUCE",
      "variety": "Cherry Tomatoes",
      "quantity": 500,
      "unit": "kg",
      "batchNumber": "BATCH-2025-001",
      "harvestDate": "2025-01-01",
      "shelfLife": 7,
      "certifications": ["ORGANIC", "GAP", "HACCP"]
    },
    "origin": {
      "farmId": "FARM-KR-001",
      "farmName": "Green Valley Farm",
      "location": {
        "address": "123 Farm Road, Jeju, South Korea",
        "latitude": 33.4996,
        "longitude": 126.5312
      },
      "operator": "Kim Farmer",
      "contactPhone": "+82-10-1234-5678"
    },
    "destination": {
      "facilityId": "RETAIL-001",
      "facilityName": "Fresh Market Supermarket",
      "location": {
        "address": "456 Market St, Seoul, South Korea",
        "latitude": 37.5665,
        "longitude": 126.9780
      },
      "contactPhone": "+82-2-9876-5432"
    },
    "route": {
      "waypoints": [
        {
          "seq": 0,
          "location": "Green Valley Farm",
          "timestamp": "2025-01-01T10:00:00Z",
          "type": "ORIGIN"
        },
        {
          "seq": 1,
          "location": "Regional Distribution Center",
          "timestamp": "2025-01-01T14:00:00Z",
          "type": "TRANSFER"
        },
        {
          "seq": 2,
          "location": "Fresh Market Supermarket",
          "timestamp": "2025-01-01T18:00:00Z",
          "type": "DESTINATION"
        }
      ],
      "totalDistance": 250,
      "estimatedDuration": 480
    },
    "status": "IN_TRANSIT"
  }
}
```

**Field Descriptions:**
- `shipmentId`: Unique identifier for the shipment
- `productType`: One of: FRESH_PRODUCE, DAIRY, MEAT, SEAFOOD, FROZEN, GRAIN
- `status`: One of: PENDING, IN_TRANSIT, ARRIVED, DELIVERED, REJECTED

### 2.2 Cold Chain Monitoring Data

Real-time sensor data for temperature-sensitive products.

```json
{
  "coldChainData": {
    "shipmentId": "SHIP-2025-001",
    "deviceId": "SENSOR-TEMP-001",
    "timestamp": "2025-01-01T12:30:45.123Z",
    "location": {
      "latitude": 37.1234,
      "longitude": 127.5678,
      "accuracy": 5.0
    },
    "temperature": {
      "current": 4.2,
      "min": 2.0,
      "max": 8.0,
      "unit": "CELSIUS",
      "status": "NORMAL"
    },
    "humidity": {
      "current": 75,
      "min": 60,
      "max": 90,
      "unit": "PERCENT",
      "status": "NORMAL"
    },
    "door": {
      "status": "CLOSED",
      "lastOpened": "2025-01-01T12:00:00Z",
      "openDuration": 120
    },
    "shock": {
      "detected": false,
      "level": 0,
      "threshold": 3
    },
    "battery": {
      "level": 85,
      "voltage": 3.7,
      "estimatedLife": 48
    },
    "alerts": []
  }
}
```

**Alert Types:**
- `TEMP_HIGH`: Temperature above maximum threshold
- `TEMP_LOW`: Temperature below minimum threshold
- `HUMIDITY_HIGH`: Humidity above maximum
- `DOOR_OPEN`: Door left open beyond threshold
- `BATTERY_LOW`: Battery below 20%
- `SHOCK_DETECTED`: Excessive vibration or impact

### 2.3 Provenance Record

Blockchain-compatible provenance data.

```json
{
  "provenance": {
    "recordId": "PROV-2025-001",
    "productId": "PROD-TOMATO-001",
    "timestamp": "2025-01-01T10:00:00Z",
    "blockchainTx": {
      "network": "HYPERLEDGER_FABRIC",
      "channelId": "supply-chain",
      "transactionId": "TX-0x1234567890abcdef",
      "blockNumber": 12345,
      "blockHash": "0xabcdef1234567890",
      "verified": true
    },
    "traceability": {
      "farmToRetail": [
        {
          "stage": "HARVEST",
          "location": "Green Valley Farm",
          "timestamp": "2025-01-01T08:00:00Z",
          "operator": "Kim Farmer",
          "verified": true
        },
        {
          "stage": "PROCESSING",
          "location": "Farm Processing Facility",
          "timestamp": "2025-01-01T09:00:00Z",
          "operator": "Processing Team",
          "verified": true
        },
        {
          "stage": "PACKAGING",
          "location": "Farm Processing Facility",
          "timestamp": "2025-01-01T09:30:00Z",
          "operator": "Packaging Team",
          "verified": true
        },
        {
          "stage": "SHIPPING",
          "location": "Green Valley Farm",
          "timestamp": "2025-01-01T10:00:00Z",
          "operator": "Logistics Partner A",
          "verified": true
        }
      ]
    },
    "certifications": [
      {
        "type": "ORGANIC",
        "certifier": "Korea Organic Certification",
        "certificateNumber": "ORG-2025-001",
        "validFrom": "2024-01-01",
        "validUntil": "2025-12-31"
      }
    ],
    "qualityChecks": [
      {
        "checkId": "QC-001",
        "stage": "POST_HARVEST",
        "timestamp": "2025-01-01T08:30:00Z",
        "inspector": "Quality Team",
        "parameters": {
          "appearance": "EXCELLENT",
          "size": "UNIFORM",
          "color": "OPTIMAL",
          "defects": 0,
          "pesticides": "NEGATIVE"
        },
        "passed": true
      }
    ]
  }
}
```

### 2.4 Quality Control Data

Product quality assessment at various stages.

```json
{
  "qualityControl": {
    "qcId": "QC-2025-001",
    "shipmentId": "SHIP-2025-001",
    "timestamp": "2025-01-01T10:00:00Z",
    "stage": "PRE_SHIPMENT",
    "inspector": {
      "name": "Lee Inspector",
      "licenseNumber": "QC-KR-2025-001",
      "organization": "Quality Assurance Corp"
    },
    "visualInspection": {
      "appearance": "EXCELLENT",
      "color": "OPTIMAL",
      "size": "UNIFORM",
      "defects": {
        "count": 0,
        "types": [],
        "severity": "NONE"
      }
    },
    "laboratoryTests": [
      {
        "testType": "PESTICIDE_RESIDUE",
        "result": "NEGATIVE",
        "detectionLimit": 0.01,
        "method": "GC-MS",
        "labName": "Agricultural Testing Lab"
      },
      {
        "testType": "HEAVY_METALS",
        "result": "WITHIN_LIMITS",
        "values": {
          "lead": 0.005,
          "cadmium": 0.003
        },
        "unit": "mg/kg"
      }
    ],
    "microbiological": {
      "totalPlateCount": 100,
      "coliforms": "NEGATIVE",
      "salmonella": "NEGATIVE",
      "listeria": "NEGATIVE",
      "unit": "CFU/g"
    },
    "overallRating": "GRADE_A",
    "passed": true,
    "notes": "All quality parameters within acceptable limits"
  }
}
```

---

## 3. Data Exchange Formats

### 3.1 GS1 EPCIS Integration

```xml
<?xml version="1.0" encoding="UTF-8"?>
<epcis:EPCISDocument
    xmlns:epcis="urn:epcglobal:epcis:xsd:1"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    schemaVersion="1.2">
  <EPCISBody>
    <EventList>
      <ObjectEvent>
        <eventTime>2025-01-01T10:00:00.000Z</eventTime>
        <eventTimeZoneOffset>+09:00</eventTimeZoneOffset>
        <epcList>
          <epc>urn:epc:id:sgtin:0614141.107346.2025001</epc>
        </epcList>
        <action>OBSERVE</action>
        <bizStep>urn:epcglobal:cbv:bizstep:shipping</bizStep>
        <readPoint>
          <id>urn:epc:id:sgln:0614141.00777.0</id>
        </readPoint>
      </ObjectEvent>
    </EventList>
  </EPCISBody>
</epcis:EPCISDocument>
```

### 3.2 CSV Export Format

For legacy system compatibility:

```csv
ShipmentID,ProductName,Origin,Destination,HarvestDate,ShipDate,Temperature,Humidity,Status
SHIP-2025-001,Organic Tomatoes,Green Valley Farm,Fresh Market,2025-01-01,2025-01-01,4.2,75,IN_TRANSIT
```

---

## 4. Validation Rules

### 4.1 Required Fields

**Shipment Data:**
- shipmentId (unique, alphanumeric, max 50 chars)
- productInfo.productName (required, max 200 chars)
- origin.farmId (required)
- destination.facilityId (required)
- timestamp (ISO 8601 format)

**Cold Chain Data:**
- temperature.current (numeric, -50 to 50)
- humidity.current (numeric, 0 to 100)
- timestamp (ISO 8601, not future dated)

### 4.2 Data Integrity

- All timestamps must be in ISO 8601 format with timezone
- Geographic coordinates: latitude (-90 to 90), longitude (-180 to 180)
- Temperature values must include unit (CELSIUS or FAHRENHEIT)
- Shipment status transitions must follow valid state machine

### 4.3 Blockchain Hash

```javascript
// SHA-256 hash of shipment data for blockchain recording
function generateProvenanceHash(shipmentData) {
  const canonicalData = JSON.stringify(shipmentData, Object.keys(shipmentData).sort());
  return crypto.createHash('sha256').update(canonicalData).digest('hex');
}
```

---

## 5. API Response Envelope

Standard response format for all API calls:

```json
{
  "success": true,
  "timestamp": "2025-01-01T10:00:00Z",
  "data": {
    // Actual data payload
  },
  "metadata": {
    "version": "1.0.0",
    "requestId": "REQ-123456",
    "processingTime": 45
  },
  "errors": []
}
```

---

## 6. Examples

### 6.1 Complete Shipment Record

See Section 2.1 for full example.

### 6.2 Alert Generation

```json
{
  "alert": {
    "alertId": "ALERT-2025-001",
    "shipmentId": "SHIP-2025-001",
    "timestamp": "2025-01-01T12:35:00Z",
    "severity": "HIGH",
    "type": "TEMP_HIGH",
    "message": "Temperature exceeded maximum threshold",
    "data": {
      "currentTemp": 12.5,
      "maxTemp": 8.0,
      "duration": 300,
      "location": {
        "latitude": 37.1234,
        "longitude": 127.5678
      }
    },
    "actions": [
      "Notify logistics manager",
      "Activate backup cooling",
      "Reroute to nearest facility"
    ]
  }
}
```

---

**Next Phase:** [Phase 2: API Interface](PHASE-2-API-INTERFACE.md)

---

**弘益人間 (Benefit All Humanity)**
*WIA - World Certification Industry Association*
*© 2025 MIT License*
