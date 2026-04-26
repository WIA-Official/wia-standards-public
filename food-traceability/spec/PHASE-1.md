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


## Annex E — Implementation Notes for PHASE-1

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1.

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
evidence for PHASE-1. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1.
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
for PHASE-1. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
