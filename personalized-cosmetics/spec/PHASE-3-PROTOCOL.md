# WIA-IND-006 Phase 3: Protocol Specification
## Personalized Cosmetics Standard - Workflows and Protocols

**Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Overview

Phase 3 defines standardized protocols and workflows for custom order processing, production scheduling, quality control, and fulfillment synchronization in personalized cosmetics systems.

## 1. Custom Order Protocol

### 1.1 Order Lifecycle Stages

```
1. Submitted → 2. Analyzing → 3. Formulating → 4. Pending Approval →
5. Approved → 6. Manufacturing → 7. Quality Control → 8. Packaging →
9. Shipped → 10. Delivered → 11. Feedback
```

### 1.2 Stage Definitions

**1. Submitted**
- Customer submits order with profile reference
- System validates profile completeness
- Payment authorization initiated
- Expected duration: 1-5 minutes

**2. Analyzing**
- Skin analysis data reviewed
- Profile updated with latest information
- AI assessment run if needed
- Expected duration: 5-30 minutes

**3. Formulating**
- Formulation algorithm generates custom formula
- Ingredient availability verified
- Cost and timeline estimated
- Expected duration: 10-60 minutes

**4. Pending Approval**
- Customer reviews proposed formulation
- Ingredient list and benefits explained
- Price and delivery estimate provided
- Expected duration: 1-48 hours (customer dependent)

**5. Approved**
- Customer confirms formulation
- Payment captured
- Manufacturing queue entry created
- Expected duration: Immediate

**6. Manufacturing**
- Production scheduled based on priority and capacity
- Ingredients dispensed and mixed
- Product filled and sealed
- Expected duration: 4-24 hours

**7. Quality Control**
- Automated and manual QC checks performed
- pH, viscosity, contamination tested
- Certificate of analysis generated
- Expected duration: 2-8 hours

**8. Packaging**
- Product labeled with custom information
- QR code generated and applied
- Secondary packaging and inserts added
- Expected duration: 1-4 hours

**9. Shipped**
- Carrier pickup and tracking initiated
- Customer notified with tracking information
- Expected duration: Varies by shipping method

**10. Delivered**
- Package confirmed delivered
- Customer prompted for feedback
- Post-delivery support activated
- Expected duration: 1-7 days from shipment

**11. Feedback**
- Customer provides product feedback
- Data used to refine future formulations
- Ongoing support and reorder prompts
- Expected duration: Ongoing

### 1.3 Order State Transitions

```json
{
  "orderId": "ord_abc123",
  "currentStage": "manufacturing",
  "timeline": [
    {
      "stage": "submitted",
      "timestamp": "2025-01-15T09:00:00Z",
      "actor": "customer",
      "notes": "Order placed via mobile app"
    },
    {
      "stage": "analyzing",
      "timestamp": "2025-01-15T09:05:00Z",
      "actor": "system",
      "notes": "Analysis completed, profile updated"
    },
    {
      "stage": "formulating",
      "timestamp": "2025-01-15T09:20:00Z",
      "actor": "formulation-engine",
      "notes": "Formula generated, match score: 0.94"
    },
    {
      "stage": "approved",
      "timestamp": "2025-01-15T14:30:00Z",
      "actor": "customer",
      "notes": "Customer approved formulation"
    },
    {
      "stage": "manufacturing",
      "timestamp": "2025-01-16T08:00:00Z",
      "actor": "production-line-3",
      "notes": "Production started",
      "current": true
    }
  ],
  "estimatedCompletion": "2025-01-16T16:00:00Z",
  "estimatedDelivery": "2025-01-20T17:00:00Z"
}
```

## 2. Production Scheduling Protocol

### 2.1 Priority Levels

```
Priority 1 (Emergency): 6-12 hours
Priority 2 (High): 12-24 hours
Priority 3 (Normal): 24-48 hours
Priority 4 (Low): 48-72 hours
Priority 5 (Batch): 72-96 hours
```

### 2.2 Batch Optimization

**Micro-Batch Strategy:**
- Group orders with similar formulations
- Minimize changeover time between batches
- Balance production efficiency with delivery deadlines
- Maximum batch size: 50 units
- Minimum batch size: 1 unit

**Optimization Criteria:**
```json
{
  "batchId": "batch_xyz789",
  "orders": ["ord_001", "ord_002", "ord_003"],
  "similarity": 0.85,
  "productionTime": "4 hours",
  "changeover": "minimal",
  "scheduledStart": "2025-01-16T08:00:00Z",
  "assignedLine": "line-3",
  "assignedOperator": "operator-007"
}
```

### 2.3 Resource Allocation

**Production Line Assignment:**
- Check line availability and capability
- Verify ingredient stock levels
- Confirm operator qualification
- Allocate equipment and tools

**Ingredient Reservation:**
```json
{
  "reservationId": "res_def456",
  "orderId": "ord_abc123",
  "ingredients": [
    {
      "ingredientId": "ing_001",
      "requiredAmount": 2.5,
      "unit": "g",
      "lotNumber": "LOT-2025-001",
      "expiryDate": "2026-12-31",
      "reserved": true
    }
  ],
  "validUntil": "2025-01-16T12:00:00Z"
}
```

### 2.4 Production Workflow

**Step-by-Step Process:**

1. **Ingredient Verification**
   - Scan lot numbers
   - Verify expiry dates
   - Confirm quantities
   - Record in system

2. **Dispensing**
   - Robotic dispensing for precision
   - Weight verification
   - Cross-check against formula
   - Adjust for density/temperature

3. **Mixing**
   - Temperature control
   - Mixing speed and duration
   - Homogenization
   - pH adjustment

4. **Quality Check (In-Process)**
   - Visual inspection
   - pH measurement
   - Viscosity test
   - Sample retention

5. **Filling**
   - Container sanitization
   - Precise fill volume
   - Weight verification
   - Seal integrity check

6. **Labeling**
   - Print custom label
   - Apply to container
   - QR code generation
   - Batch number recording

## 3. Quality Control Protocol

### 3.1 QC Checkpoints

**Pre-Production:**
- Ingredient verification
- Equipment calibration
- Environmental conditions
- Operator qualification

**In-Process:**
- Dispensing accuracy
- Mixing homogeneity
- Temperature compliance
- pH verification

**Post-Production:**
- Final pH test
- Viscosity measurement
- Contamination screening
- Stability assessment
- Packaging integrity

### 3.2 Testing Procedures

**Physical Tests:**
```json
{
  "productId": "prod_abc123",
  "physicalTests": {
    "appearance": {
      "color": "white",
      "clarity": "clear",
      "consistency": "uniform",
      "result": "pass"
    },
    "pH": {
      "measured": 5.5,
      "specification": "5.0-6.0",
      "result": "pass"
    },
    "viscosity": {
      "measured": 12500,
      "unit": "cP",
      "specification": "10000-15000",
      "result": "pass"
    },
    "specificGravity": {
      "measured": 1.02,
      "specification": "1.00-1.05",
      "result": "pass"
    }
  }
}
```

**Chemical Tests:**
```json
{
  "chemicalTests": {
    "activeIngredients": {
      "niacinamide": {
        "measured": 4.95,
        "specification": "4.75-5.25",
        "unit": "%",
        "result": "pass"
      },
      "hyaluronicAcid": {
        "measured": 2.01,
        "specification": "1.90-2.10",
        "unit": "%",
        "result": "pass"
      }
    },
    "preservativeEfficacy": {
      "tested": true,
      "result": "pass",
      "method": "challenge test"
    }
  }
}
```

**Microbiological Tests:**
```json
{
  "microbiologicalTests": {
    "totalAerobicCount": {
      "measured": "<10",
      "specification": "<100",
      "unit": "CFU/g",
      "result": "pass"
    },
    "yeastMold": {
      "measured": "<10",
      "specification": "<100",
      "unit": "CFU/g",
      "result": "pass"
    },
    "pathogens": {
      "detected": false,
      "result": "pass"
    }
  }
}
```

### 3.3 Certificate of Analysis

```json
{
  "certificateNumber": "COA-2025-001234",
  "productId": "prod_abc123",
  "batchNumber": "BATCH-2025-A123",
  "formulationId": "form_def456",
  "manufactureDate": "2025-01-16",
  "expiryDate": "2026-01-16",
  "testResults": {
    "physical": "pass",
    "chemical": "pass",
    "microbiological": "pass"
  },
  "overallResult": "APPROVED",
  "inspector": {
    "name": "Jane Smith",
    "id": "QC-007",
    "signature": "digital_signature",
    "date": "2025-01-16T14:00:00Z"
  },
  "approvedBy": {
    "name": "John Doe",
    "role": "QA Manager",
    "signature": "digital_signature",
    "date": "2025-01-16T15:00:00Z"
  }
}
```

## 4. Fulfillment Synchronization Protocol

### 4.1 Inventory Management

**Real-Time Stock Tracking:**
```json
{
  "ingredientId": "ing_001",
  "inciName": "Hyaluronic Acid",
  "currentStock": 5000,
  "unit": "g",
  "reserved": 1200,
  "available": 3800,
  "reorderPoint": 2000,
  "reorderQuantity": 5000,
  "supplier": "supplier_xyz",
  "leadTime": 14,
  "lastUpdated": "2025-01-16T10:00:00Z"
}
```

### 4.2 Shipping Integration

**Carrier Selection:**
- Delivery deadline requirements
- Package size and weight
- Destination and service area
- Cost optimization
- Customer shipping preferences

**Tracking Integration:**
```json
{
  "shipmentId": "ship_abc123",
  "orderId": "ord_abc123",
  "carrier": "UPS",
  "service": "Ground",
  "trackingNumber": "1Z999AA10123456784",
  "status": "in_transit",
  "events": [
    {
      "timestamp": "2025-01-17T09:00:00Z",
      "location": "New York, NY",
      "status": "Picked up",
      "description": "Package picked up by carrier"
    },
    {
      "timestamp": "2025-01-17T15:30:00Z",
      "location": "Philadelphia, PA",
      "status": "In transit",
      "description": "Package in transit"
    }
  ],
  "estimatedDelivery": "2025-01-20T17:00:00Z"
}
```

### 4.3 Customer Notification Protocol

**Notification Triggers:**
- Order confirmed
- Formulation ready for approval
- Manufacturing started
- Quality control passed
- Shipped with tracking
- Out for delivery
- Delivered
- Feedback request

**Notification Channels:**
```json
{
  "notificationId": "notif_xyz789",
  "orderId": "ord_abc123",
  "userId": "user_12345",
  "type": "order_shipped",
  "channels": {
    "email": {
      "sent": true,
      "timestamp": "2025-01-17T09:05:00Z"
    },
    "sms": {
      "sent": true,
      "timestamp": "2025-01-17T09:05:00Z"
    },
    "push": {
      "sent": true,
      "timestamp": "2025-01-17T09:05:00Z"
    }
  },
  "content": {
    "subject": "Your order has shipped!",
    "message": "Your personalized serum is on its way...",
    "trackingLink": "https://track.carrier.com/1Z999AA10123456784"
  }
}
```

## 5. Exception Handling Protocol

### 5.1 Common Exceptions

**Ingredient Out of Stock:**
- Notify customer of delay
- Offer alternative formulation
- Provide revised timeline
- Optional: upgrade to expedited shipping

**Quality Control Failure:**
- Discard failed product
- Investigate root cause
- Reformulate if needed
- Re-manufacture at no charge
- Notify customer of delay

**Shipping Delay:**
- Monitor carrier status
- Proactive customer notification
- Offer compensation if late
- Arrange re-shipment if lost

### 5.2 Escalation Procedures

```
Level 1: Automated system response (0-2 hours)
Level 2: Customer service team (2-24 hours)
Level 3: Production manager (24-48 hours)
Level 4: Quality assurance director (48+ hours)
```

## 6. Continuous Improvement Protocol

### 6.1 Feedback Collection

```json
{
  "feedbackId": "fb_abc123",
  "orderId": "ord_abc123",
  "userId": "user_12345",
  "timestamp": "2025-01-25T10:00:00Z",
  "ratings": {
    "overall": 5,
    "efficacy": 5,
    "texture": 4,
    "fragrance": 5,
    "packaging": 5
  },
  "comments": "Love the serum! Skin feels amazing.",
  "concerns": [],
  "repurchaseIntent": "yes",
  "recommendations": ["moisturizer", "sunscreen"]
}
```

### 6.2 Data-Driven Optimization

- Aggregate feedback across formulations
- Identify high-performing ingredients
- Detect common concerns or issues
- Refine formulation algorithms
- Improve production processes
- Enhance customer experience

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for personalized-cosmetics is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/personalized-cosmetics/api/` — TypeScript SDK skeleton
- `wia-standards/standards/personalized-cosmetics/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/personalized-cosmetics/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
