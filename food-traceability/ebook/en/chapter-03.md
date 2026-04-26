# Chapter 3: EPCIS Event Architecture

**WIA-AGRI-016 eBook Series**

---

## Introduction to EPCIS

EPCIS (Electronic Product Code Information Services) is the GS1 standard for capturing and sharing visibility event data across the supply chain. It provides a common language for describing supply chain events in a standardized, interoperable format.

### What EPCIS Provides

- **Standard event format** for all supply chain activities
- **Interoperable data sharing** between trading partners
- **Complete visibility** from farm to consumer
- **Regulatory compliance** for traceability requirements

---

## The Five W's of EPCIS

Every EPCIS event answers:

1. **WHAT:** What objects/products are involved?
2. **WHEN:** When did the event occur?
3. **WHERE:** Where did it happen?
4. **WHY:** What business step was performed?
5. **HOW:** Under what conditions?

### Example Event

```
WHAT:  1,000 kg of Organic Apples (Batch LOT2025001)
WHEN:  December 1, 2025 at 08:30 AM
WHERE: ABC Organic Farm, Field #3
WHY:   Harvesting operation
HOW:   Mechanical harvest, 18°C ambient temperature
```

---

## EPCIS Event Types

### 1. ObjectEvent

**Purpose:** Tracks visibility of objects without parent-child relationships

**Use Cases:**
- Harvesting produce
- Receiving shipments
- Quality inspections
- Retail sales

**Example: Harvesting Event**

```xml
<ObjectEvent>
  <eventTime>2025-12-01T08:30:00Z</eventTime>
  <eventTimeZoneOffset>-08:00</eventTimeZoneOffset>

  <epcList>
    <epc>urn:epc:id:sgtin:0123456.789012.LOT2025001</epc>
  </epcList>

  <action>ADD</action>

  <bizStep>urn:epcglobal:cbv:bizstep:commissioning</bizStep>
  <disposition>urn:epcglobal:cbv:disp:active</disposition>

  <readPoint>
    <id>urn:epc:id:sgln:0123456.00001.FIELD3</id>
  </readPoint>

  <bizLocation>
    <id>urn:epc:id:sgln:0123456.00001.0</id>
  </bizLocation>

  <extension>
    <quantityList>
      <quantityElement>
        <epcClass>urn:epc:class:lgtin:0123456.789012.LOT2025001</epcClass>
        <quantity>1000</quantity>
        <uom>KGM</uom>
      </quantityElement>
    </quantityList>

    <ilmd>
      <harvestDate>2025-12-01</harvestDate>
      <variety>Gala</variety>
      <field>Field #3</field>
      <temperature>18</temperature>
      <temperatureUnit>CEL</temperatureUnit>
    </ilmd>
  </extension>
</ObjectEvent>
```

**JSON Format (EPCIS 2.0):**

```json
{
  "type": "ObjectEvent",
  "eventTime": "2025-12-01T08:30:00Z",
  "eventTimeZoneOffset": "-08:00",
  "eventID": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",

  "epcList": [
    "urn:epc:id:sgtin:0123456.789012.LOT2025001"
  ],

  "action": "ADD",
  "bizStep": "commissioning",
  "disposition": "active",

  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.FIELD3"
  },

  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  },

  "quantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789012.LOT2025001",
      "quantity": 1000,
      "uom": "KGM"
    }
  ],

  "ilmd": {
    "harvestDate": "2025-12-01",
    "variety": "Gala",
    "field": "Field #3",
    "temperature": 18,
    "temperatureUnit": "CEL"
  }
}
```

### 2. AggregationEvent

**Purpose:** Records parent-child relationships (packaging hierarchy)

**Use Cases:**
- Packing individual items into cases
- Palletizing cases
- Container loading
- De-palletizing/unpacking

**Example: Palletization**

```json
{
  "type": "AggregationEvent",
  "eventTime": "2025-12-02T10:00:00Z",
  "eventTimeZoneOffset": "-08:00",

  "parentID": "urn:epc:id:sscc:0123456.1234567890",

  "childEPCs": [
    "urn:epc:id:sgtin:0123456.789012.CASE001",
    "urn:epc:id:sgtin:0123456.789012.CASE002",
    "urn:epc:id:sgtin:0123456.789012.CASE003"
  ],

  "action": "ADD",
  "bizStep": "packing",
  "disposition": "container_closed",

  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.PACK"
  },

  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  }
}
```

**Hierarchy:**
```
Pallet (SSCC)
  ├── Case 001
  ├── Case 002
  └── Case 003
```

### 3. TransformationEvent

**Purpose:** Tracks input materials transformed into output products

**Use Cases:**
- Juice pressing (apples → juice)
- Cooking/baking
- Blending ingredients
- Any processing operation

**Example: Apple Juice Production**

```json
{
  "type": "TransformationEvent",
  "eventTime": "2025-12-05T10:30:00Z",
  "eventTimeZoneOffset": "-08:00",
  "transformationID": "urn:uuid:transform-20251205-001",

  "inputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789012.LOT2025001",
      "quantity": 500,
      "uom": "KGM",
      "productName": "Organic Apples"
    },
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789013.LOT2025020",
      "quantity": 200,
      "uom": "KGM",
      "productName": "Organic Pears"
    },
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789014.LOT2024999",
      "quantity": 1,
      "uom": "KGM",
      "productName": "Ascorbic Acid"
    }
  ],

  "outputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789020.LOT2025100",
      "quantity": 650,
      "uom": "LTR",
      "productName": "Organic Apple-Pear Juice"
    }
  ],

  "bizStep": "commissioning",
  "disposition": "in_progress",

  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00002.PROCESSING"
  },

  "processParameters": {
    "temperature": 72,
    "temperatureUnit": "CEL",
    "duration": 15,
    "durationUnit": "SEC",
    "process": "pasteurization"
  }
}
```

### 4. TransactionEvent

**Purpose:** Links business transactions (POs, invoices) to physical goods

**Use Cases:**
- Shipping against purchase order
- Receiving against invoice
- Returns/credits
- Ownership transfer

**Example: Shipment with Purchase Order**

```json
{
  "type": "TransactionEvent",
  "eventTime": "2025-12-03T14:00:00Z",
  "eventTimeZoneOffset": "-08:00",

  "epcList": [
    "urn:epc:id:sscc:0123456.1234567890"
  ],

  "action": "ADD",
  "bizStep": "shipping",
  "disposition": "in_transit",

  "bizTransactionList": [
    {
      "type": "po",
      "bizTransaction": "urn:epdglobal:cbv:bt:po:PO-2025-12345"
    },
    {
      "type": "desadv",
      "bizTransaction": "urn:epdglobal:cbv:bt:desadv:ASN-2025-5678"
    }
  ],

  "sourceList": [
    {
      "type": "owning_party",
      "source": "urn:epc:id:sgln:0123456.00001.0"
    },
    {
      "type": "location",
      "source": "urn:epc:id:sgln:0123456.00001.SHIP"
    }
  ],

  "destinationList": [
    {
      "type": "owning_party",
      "destination": "urn:epc:id:sgln:9876543.00001.0"
    },
    {
      "type": "location",
      "destination": "urn:epc:id:sgln:9876543.00001.RECV"
    }
  ]
}
```

### 5. AssociationEvent (EPCIS 2.0)

**Purpose:** Manages associations between objects (e.g., products and RTIs)

**Use Cases:**
- Linking products to reusable containers
- Associating sensors with shipments
- Tracking returnable transport items (RTIs)

```json
{
  "type": "AssociationEvent",
  "eventTime": "2025-12-03T09:00:00Z",
  "eventTimeZoneOffset": "-08:00",

  "parentID": "urn:epc:id:grai:0123456.CRATE.C12345",

  "childEPCs": [
    "urn:epc:id:sgtin:0123456.789012.LOT2025001"
  ],

  "action": "ADD",
  "bizStep": "loading",

  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.LOADING"
  }
}
```

---

## Business Steps & Dispositions

### Common Business Steps (bizStep)

| Business Step | Description | Typical Event |
|---------------|-------------|---------------|
| commissioning | Creating new objects | Harvest, production |
| receiving | Accepting goods | Warehouse receipt |
| shipping | Dispatching goods | Outbound shipment |
| packing | Creating packaging hierarchy | Palletization |
| unpacking | Breaking down hierarchy | De-palletization |
| retail_selling | Point-of-sale | Consumer purchase |
| storing | Placing in storage | Warehouse storage |
| transporting | Moving between locations | In-transit |

### Common Dispositions

| Disposition | Description |
|-------------|-------------|
| active | Object is active/available |
| in_progress | Processing underway |
| in_transit | Being transported |
| container_closed | Packaged and sealed |
| damaged | Product damaged |
| expired | Past expiration date |
| recalled | Subject to recall |
| destroyed | Destroyed/disposed |

---

## Instance/Lot-Level Material (ILMD)

ILMD captures attributes at the time of object creation:

```json
{
  "ilmd": {
    "harvestDate": "2025-12-01",
    "variety": "Gala",
    "organicCertification": "USDA-ORG-123456",
    "field": "North Field #3",
    "packingDate": "2025-12-02",
    "expirationDate": "2025-12-31",
    "bestBeforeDate": "2025-12-20",
    "lotNumber": "LOT2025001",
    "countryOfOrigin": "US",
    "temperatureAtHarvest": 18,
    "operatorID": "OPR-456"
  }
}
```

**Key Characteristics:**
- Set once at object creation
- Immutable (doesn't change during lifecycle)
- Product-specific attributes
- Quality and compliance data

---

## Sensor Data Integration

EPCIS 2.0 supports sensor data:

```json
{
  "type": "ObjectEvent",
  "eventTime": "2025-12-03T20:00:00Z",

  "epcList": [
    "urn:epc:id:sscc:0123456.1234567890"
  ],

  "action": "OBSERVE",
  "bizStep": "transporting",
  "disposition": "in_transit",

  "sensorElementList": [
    {
      "sensorMetadata": {
        "time": "2025-12-03T20:00:00Z",
        "deviceID": "urn:epc:id:giai:0123456.SENSOR.T123",
        "deviceMetadata": "TempSensor Model XYZ"
      },
      "sensorReport": [
        {
          "type": "Temperature",
          "value": 4.2,
          "uom": "CEL",
          "minValue": 3.8,
          "maxValue": 4.5,
          "meanValue": 4.1
        },
        {
          "type": "Humidity",
          "value": 75,
          "uom": "P1",
          "minValue": 70,
          "maxValue": 80
        }
      ]
    }
  ]
}
```

---

## Implementing EPCIS

### Architecture

```
┌──────────────────┐
│  Applications    │  (ERP, WMS, QC systems)
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS Capture   │  (Event generation)
│  Interface       │
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS Repository│  (Event storage & query)
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS Query     │  (Event retrieval)
│  Interface       │
└────────┬─────────┘
         │
┌────────▼─────────┐
│  External        │  (Trading partners)
│  Systems         │
└──────────────────┘
```

### Capture Interface

```javascript
class EPCISCapture {
  constructor(repositoryUrl) {
    this.repositoryUrl = repositoryUrl;
  }

  async captureObjectEvent(event) {
    const epcisEvent = {
      "@context": [
        "https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld"
      ],
      "type": "EPCISDocument",
      "schemaVersion": "2.0",
      "creationDate": new Date().toISOString(),
      "epcisBody": {
        "eventList": [event]
      }
    };

    const response = await fetch(`${this.repositoryUrl}/capture`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(epcisEvent)
    });

    if (!response.ok) {
      throw new Error(`EPCIS capture failed: ${response.statusText}`);
    }

    return await response.json();
  }
}

// Usage
const capture = new EPCISCapture('https://epcis.example.com');

await capture.captureObjectEvent({
  type: "ObjectEvent",
  eventTime: "2025-12-01T08:30:00Z",
  eventTimeZoneOffset: "-08:00",
  epcList: ["urn:epc:id:sgtin:0123456.789012.LOT2025001"],
  action: "ADD",
  bizStep: "commissioning",
  disposition: "active",
  readPoint: {
    id: "urn:epc:id:sgln:0123456.00001.FIELD3"
  }
});
```

### Query Interface

```javascript
class EPCISQuery {
  constructor(repositoryUrl) {
    this.repositoryUrl = repositoryUrl;
  }

  async queryByBatch(batchId) {
    const query = {
      query: "SimpleEventQuery",
      EQ_bizLocation: null,
      EQ_epc: [`urn:epc:id:sgtin:${batchId}`],
      orderBy: "eventTime",
      orderDirection: "ASC"
    };

    const response = await fetch(`${this.repositoryUrl}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(query)
    });

    return await response.json();
  }

  async queryByLocation(gln, startDate, endDate) {
    const query = {
      query: "SimpleEventQuery",
      EQ_bizLocation: [`urn:epc:id:sgln:${gln}`],
      GE_eventTime: startDate,
      LE_eventTime: endDate,
      orderBy: "eventTime",
      orderDirection: "DESC"
    };

    const response = await fetch(`${this.repositoryUrl}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(query)
    });

    return await response.json();
  }
}
```

---

## Best Practices

### 1. Event Granularity

**Good:** Capture meaningful business events
```
- Harvesting completed
- Shipped to customer
- Quality test passed
```

**Bad:** Over-granular events
```
- Worker started picking
- Worker took break
- Worker resumed picking
- Worker finished picking
```

### 2. Consistent Timestamps

- Always use ISO 8601 format
- Include timezone offset
- Use UTC for international operations
- Synchronize clocks across facilities

### 3. Error Handling

```javascript
async function captureEventWithRetry(event, maxRetries = 3) {
  for (let attempt = 1; attempt <= maxRetries; attempt++) {
    try {
      return await capture.captureObjectEvent(event);
    } catch (error) {
      if (attempt === maxRetries) {
        // Store locally for later submission
        await storeOffline(event);
        throw error;
      }
      // Exponential backoff
      await sleep(Math.pow(2, attempt) * 1000);
    }
  }
}
```

---

## Chapter Summary

EPCIS provides the standard framework for capturing and sharing traceability events:

**Five Event Types:**
1. ObjectEvent - Object visibility
2. AggregationEvent - Packaging hierarchy
3. TransformationEvent - Processing
4. TransactionEvent - Business documents
5. AssociationEvent - Object associations

**Key Components:**
- What, When, Where, Why, How
- Business Steps & Dispositions
- ILMD for product attributes
- Sensor data integration

**Implementation:**
- Capture interface for event submission
- Repository for storage
- Query interface for retrieval
- Standards-based interoperability

---

## Next Chapter

**Chapter 4: Blockchain Integration for Immutable Records**

Learn how to combine EPCIS with blockchain technology for tamper-proof traceability records.

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
