# WIA-AGRI-016: Food Traceability Standard
## PHASE 2: Processing History & Multi-Ingredient Tracking

**Version:** 1.0
**Status:** Stable
**Last Updated:** 2025-12-26

---

## 1. Overview

PHASE 2 extends traceability to complex processing operations, multi-ingredient formulations, and transformation events. This phase enables tracking of how raw materials are processed, combined, and transformed into finished products.

### 1.1 Objectives

- Track processing and transformation events
- Manage multi-ingredient batch associations
- Document quality control data
- Implement GS1 EPCIS standards
- Enable ingredient-level trace-back

### 1.2 Scope

PHASE 2 covers:
- Transformation events (raw → processed)
- Recipe/formula management with batch associations
- Quality control and testing data
- Processing parameter logging (time, temperature, pressure)
- Allergen and nutritional tracking
- EPCIS 2.0 event implementation

---

## 2. Transformation Events

### 2.1 EPCIS TransformationEvent

Tracks when input materials are transformed into output products:

```json
{
  "type": "TransformationEvent",
  "eventTime": "2025-12-05T10:30:00Z",
  "eventTimeZoneOffset": "+00:00",
  "eventID": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "inputEPCList": [
    "urn:epc:id:sgtin:0123456.789012.LOT2025001",
    "urn:epc:id:sgtin:0123456.789013.LOT2025020"
  ],
  "inputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789012.LOT2025001",
      "quantity": 500,
      "uom": "KGM"
    },
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789013.LOT2025020",
      "quantity": 200,
      "uom": "KGM"
    }
  ],
  "outputEPCList": [
    "urn:epc:id:sgtin:0123456.789014.LOT2025100"
  ],
  "outputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789014.LOT2025100",
      "quantity": 650,
      "uom": "KGM"
    }
  ],
  "bizStep": "urn:epcglobal:cbv:bizstep:commissioning",
  "disposition": "urn:epcglobal:cbv:disp:in_progress",
  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  },
  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  },
  "transformationID": "urn:uuid:transform_20251205_001"
}
```

### 2.2 Processing Parameters

```json
{
  "transformationId": "transform_20251205_001",
  "processType": "pasteurization",
  "parameters": {
    "temperature": {
      "value": 72,
      "unit": "celsius",
      "duration": 15,
      "durationUnit": "seconds"
    },
    "pressure": {
      "value": 1.5,
      "unit": "bar"
    },
    "ph": {
      "value": 4.2,
      "measuredAt": "2025-12-05T10:35:00Z"
    }
  },
  "equipment": {
    "id": "PASTEURIZER-003",
    "name": "Pasteurization Unit #3",
    "lastCalibration": "2025-11-01",
    "calibrationDue": "2026-02-01"
  },
  "operator": {
    "id": "OPR-456",
    "name": "Maria Garcia",
    "certification": "FoodSafety-Level2"
  }
}
```

---

## 3. Multi-Ingredient Batch Management

### 3.1 Recipe/Formula Structure

```json
{
  "recipeId": "RECIPE-APPLEJUICE-001",
  "productName": "Organic Apple Juice",
  "version": "2.1",
  "effectiveDate": "2025-01-01",
  "ingredients": [
    {
      "ingredientId": "ING-001",
      "name": "Organic Apples",
      "gtin": "01234567890128",
      "quantity": {
        "value": 70,
        "unit": "percent"
      },
      "function": "primary_ingredient",
      "allergens": [],
      "origin": "domestic"
    },
    {
      "ingredientId": "ING-002",
      "name": "Organic Pear",
      "gtin": "01234567890129",
      "quantity": {
        "value": 25,
        "unit": "percent"
      },
      "function": "flavor_enhancement",
      "allergens": [],
      "origin": "domestic"
    },
    {
      "ingredientId": "ING-003",
      "name": "Ascorbic Acid (Vitamin C)",
      "gtin": "01234567890130",
      "quantity": {
        "value": 0.1,
        "unit": "percent"
      },
      "function": "preservative",
      "allergens": [],
      "cas": "50-81-7"
    }
  ],
  "allergenStatement": "Contains: None. Produced in facility that processes tree nuts.",
  "nutritionalProfile": {
    "servingSize": "250ml",
    "calories": 110,
    "sugar": "24g",
    "vitaminC": "120mg"
  }
}
```

### 3.2 Batch Production Record

```json
{
  "productionId": "PROD-20251205-001",
  "recipeId": "RECIPE-APPLEJUICE-001",
  "outputBatchId": "01234567890140.LOT2025200",
  "productionDate": "2025-12-05",
  "inputBatches": [
    {
      "ingredientId": "ING-001",
      "batchId": "01234567890128.LOT2025001",
      "quantityUsed": {
        "value": 700,
        "unit": "kg"
      },
      "supplier": "ABC Organic Farm",
      "receiptDate": "2025-12-03",
      "expiryDate": "2025-12-31"
    },
    {
      "ingredientId": "ING-002",
      "batchId": "01234567890129.LOT2025015",
      "quantityUsed": {
        "value": 250,
        "unit": "kg"
      },
      "supplier": "XYZ Orchards",
      "receiptDate": "2025-12-02",
      "expiryDate": "2025-12-28"
    },
    {
      "ingredientId": "ING-003",
      "batchId": "01234567890130.LOT2024999",
      "quantityUsed": {
        "value": 1,
        "unit": "kg"
      },
      "supplier": "ChemCo Suppliers",
      "receiptDate": "2025-11-15",
      "expiryDate": "2026-11-15"
    }
  ],
  "outputQuantity": {
    "value": 900,
    "unit": "liters"
  },
  "yield": {
    "actual": 95,
    "expected": 93,
    "unit": "percent"
  },
  "startTime": "2025-12-05T08:00:00Z",
  "endTime": "2025-12-05T12:30:00Z",
  "supervisedBy": "John Supervisor"
}
```

---

## 4. Quality Control Integration

### 4.1 QC Testing Data

```json
{
  "testId": "QC-20251205-100",
  "batchId": "01234567890140.LOT2025200",
  "testDate": "2025-12-05T13:00:00Z",
  "testType": "microbial_analysis",
  "laboratory": {
    "name": "FoodTest Lab Inc.",
    "accreditation": "ISO 17025",
    "certNumber": "LAB-12345"
  },
  "tests": [
    {
      "parameter": "Total Plate Count",
      "method": "AOAC 990.12",
      "result": 150,
      "unit": "CFU/ml",
      "specification": "<1000 CFU/ml",
      "status": "pass"
    },
    {
      "parameter": "E. coli",
      "method": "ISO 16649-2",
      "result": "Not Detected",
      "specification": "Not Detected in 25g",
      "status": "pass"
    },
    {
      "parameter": "Salmonella",
      "method": "ISO 6579-1",
      "result": "Not Detected",
      "specification": "Not Detected in 25g",
      "status": "pass"
    }
  ],
  "overallResult": "pass",
  "approvedBy": {
    "name": "Dr. Jane QC Manager",
    "license": "QC-98765",
    "approvalDate": "2025-12-05T15:00:00Z"
  },
  "certificateUrl": "https://lab.example.com/certs/QC-20251205-100.pdf"
}
```

### 4.2 In-Process Quality Checks

```json
{
  "checkId": "IPC-20251205-050",
  "batchId": "01234567890140.LOT2025200",
  "checkTime": "2025-12-05T10:15:00Z",
  "checkType": "in_process",
  "location": "Filling Line #2",
  "parameters": [
    {
      "name": "Brix (Sugar Content)",
      "value": 12.5,
      "unit": "degrees_brix",
      "spec": "12.0 - 13.0",
      "status": "pass"
    },
    {
      "name": "Fill Volume",
      "value": 1002,
      "unit": "ml",
      "spec": "1000 ± 10 ml",
      "status": "pass"
    },
    {
      "name": "pH",
      "value": 4.1,
      "unit": "pH",
      "spec": "3.8 - 4.4",
      "status": "pass"
    }
  ],
  "inspector": "QC Technician #7"
}
```

---

## 5. Ingredient-Level Trace-Back

### 5.1 Full Ingredient Trace

```javascript
async function traceIngredients(outputBatchId) {
  // Get production record
  const production = await getProductionRecord(outputBatchId);

  // Trace each input batch
  const ingredientTraces = await Promise.all(
    production.inputBatches.map(async (input) => {
      const events = await getEventHistory(input.batchId);
      return {
        ingredient: input.ingredientName,
        batchId: input.batchId,
        supplier: input.supplier,
        origin: events.find(e => e.eventType === 'harvest')?.location,
        path: events.map(e => ({
          date: e.timestamp,
          event: e.eventType,
          location: e.location.name
        }))
      };
    })
  );

  return {
    outputBatch: outputBatchId,
    productionDate: production.productionDate,
    ingredients: ingredientTraces,
    qualityTests: await getQualityTests(outputBatchId)
  };
}
```

### 5.2 Allergen Trace

```json
{
  "allergenTraceId": "ALLERG-20251205-001",
  "batchId": "01234567890140.LOT2025200",
  "product": "Organic Apple Juice",
  "declaredAllergens": [],
  "facilityAllergens": ["tree_nuts"],
  "crossContaminationRisk": "low",
  "ingredientAllergenMap": [
    {
      "ingredientId": "ING-001",
      "name": "Organic Apples",
      "batchId": "01234567890128.LOT2025001",
      "allergens": [],
      "supplierStatement": "Allergen-free farm"
    }
  ],
  "cleaningRecords": [
    {
      "lineId": "FILLING-LINE-02",
      "cleanedAt": "2025-12-05T06:00:00Z",
      "method": "CIP (Clean-in-Place)",
      "verifiedBy": "Sanitation Supervisor",
      "allergenSwabTest": "negative"
    }
  ]
}
```

---

## 6. EPCIS 2.0 Implementation

### 6.1 ObjectEvent for Processing

```xml
<ObjectEvent>
  <eventTime>2025-12-05T10:30:00.000Z</eventTime>
  <eventTimeZoneOffset>+00:00</eventTimeZoneOffset>
  <epcList>
    <epc>urn:epc:id:sgtin:0123456.789014.LOT2025200</epc>
  </epcList>
  <action>ADD</action>
  <bizStep>urn:epcglobal:cbv:bizstep:commissioning</bizStep>
  <disposition>urn:epcglobal:cbv:disp:active</disposition>
  <readPoint>
    <id>urn:epc:id:sgln:0123456.00001.FILLING</id>
  </readPoint>
  <bizLocation>
    <id>urn:epc:id:sgln:0123456.00001.0</id>
  </bizLocation>
  <extension>
    <quantityList>
      <quantityElement>
        <epcClass>urn:epc:class:lgtin:0123456.789014.LOT2025200</epcClass>
        <quantity>900</quantity>
        <uom>LTR</uom>
      </quantityElement>
    </quantityList>
    <ilmd>
      <productionDate>2025-12-05</productionDate>
      <expirationDate>2026-06-05</expirationDate>
      <bestBeforeDate>2026-03-05</bestBeforeDate>
      <lotNumber>LOT2025200</lotNumber>
    </ilmd>
  </extension>
</ObjectEvent>
```

### 6.2 AggregationEvent for Packaging

```json
{
  "type": "AggregationEvent",
  "eventTime": "2025-12-05T14:00:00Z",
  "eventTimeZoneOffset": "+00:00",
  "parentID": "urn:epc:id:sscc:0123456.7890123456",
  "childEPCs": [
    "urn:epc:id:sgtin:0123456.789014.100001",
    "urn:epc:id:sgtin:0123456.789014.100002",
    "urn:epc:id:sgtin:0123456.789014.100003"
  ],
  "action": "ADD",
  "bizStep": "urn:epcglobal:cbv:bizstep:packing",
  "disposition": "urn:epcglobal:cbv:disp:container_closed",
  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.PACKING"
  },
  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  },
  "childQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789014.LOT2025200",
      "quantity": 24,
      "uom": "EA"
    }
  ]
}
```

---

## 7. Database Schema Extensions

### 7.1 Transformations Table

```sql
CREATE TABLE transformations (
  id VARCHAR(100) PRIMARY KEY,
  transformation_id VARCHAR(100) UNIQUE NOT NULL,
  event_time TIMESTAMP NOT NULL,
  recipe_id VARCHAR(100),
  process_type VARCHAR(50),
  parameters JSON,
  equipment_id VARCHAR(50),
  operator_id VARCHAR(50),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  INDEX idx_recipe (recipe_id),
  INDEX idx_time (event_time)
);
```

### 7.2 Transformation Inputs/Outputs

```sql
CREATE TABLE transformation_inputs (
  id BIGINT AUTO_INCREMENT PRIMARY KEY,
  transformation_id VARCHAR(100) NOT NULL,
  batch_id VARCHAR(100) NOT NULL,
  ingredient_id VARCHAR(100),
  quantity_value DECIMAL(10,3),
  quantity_unit VARCHAR(20),
  FOREIGN KEY (transformation_id) REFERENCES transformations(id),
  INDEX idx_transformation (transformation_id),
  INDEX idx_batch (batch_id)
);

CREATE TABLE transformation_outputs (
  id BIGINT AUTO_INCREMENT PRIMARY KEY,
  transformation_id VARCHAR(100) NOT NULL,
  batch_id VARCHAR(100) NOT NULL,
  quantity_value DECIMAL(10,3),
  quantity_unit VARCHAR(20),
  FOREIGN KEY (transformation_id) REFERENCES transformations(id),
  INDEX idx_transformation (transformation_id),
  INDEX idx_batch (batch_id)
);
```

### 7.3 Quality Control Table

```sql
CREATE TABLE quality_tests (
  id VARCHAR(100) PRIMARY KEY,
  batch_id VARCHAR(100) NOT NULL,
  test_date TIMESTAMP NOT NULL,
  test_type VARCHAR(50),
  laboratory VARCHAR(255),
  overall_result VARCHAR(20),
  approved_by VARCHAR(100),
  approval_date TIMESTAMP,
  test_data JSON,
  certificate_url VARCHAR(500),
  FOREIGN KEY (batch_id) REFERENCES batches(id),
  INDEX idx_batch (batch_id),
  INDEX idx_date (test_date),
  INDEX idx_result (overall_result)
);
```

---

## 8. API Endpoints

### 8.1 Transformation Management

```
POST /api/v1/transformations
GET /api/v1/transformations/{transformationId}
GET /api/v1/transformations?recipeId={recipeId}
GET /api/v1/batches/{batchId}/inputs
GET /api/v1/batches/{batchId}/outputs
```

### 8.2 Quality Control

```
POST /api/v1/quality-tests
GET /api/v1/quality-tests/{testId}
GET /api/v1/batches/{batchId}/quality
PUT /api/v1/quality-tests/{testId}/approve
```

### 8.3 Ingredient Trace

```
GET /api/v1/trace/ingredients/{outputBatchId}
GET /api/v1/trace/allergens/{batchId}
GET /api/v1/batches/{batchId}/recipe
```

---

## 9. Implementation Checklist

- [ ] Deploy EPCIS 2.0 event repository
- [ ] Create recipe/formula management system
- [ ] Implement transformation event tracking
- [ ] Integrate quality control systems
- [ ] Develop ingredient trace-back queries
- [ ] Set up allergen tracking workflows
- [ ] Configure processing parameter logging
- [ ] Train production staff on batch recording
- [ ] Establish QC approval workflows
- [ ] Conduct multi-ingredient trace testing

---

## 10. Next Steps to PHASE 3

PHASE 3 will introduce:
- Cold chain and temperature monitoring
- Real-time IoT sensor integration
- Predictive quality analytics
- Advanced recall management
- Cross-facility batch correlation

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Cycle:** Annual
- **Next Review:** 2026-12-26

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity
