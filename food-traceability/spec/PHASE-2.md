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


## Annex E — Implementation Notes for PHASE-2

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
