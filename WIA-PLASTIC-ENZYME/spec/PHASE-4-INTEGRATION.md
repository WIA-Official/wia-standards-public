# WIA-PLASTIC-ENZYME Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 of the WIA-PLASTIC-ENZYME standard defines the integration requirements for connecting enzymatic plastic degradation facilities with the broader circular economy ecosystem. This includes waste collection, chemical supply chains, regulatory systems, and carbon tracking.

### 1.1 Scope

This specification covers:

- Waste collection system integration
- Chemical industry supply chain
- Regulatory compliance and reporting
- Carbon footprint tracking
- Digital twin and IoT integration
- Partner ecosystem connectivity

### 1.2 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        WIA Integration Layer                        │
├─────────────┬─────────────┬─────────────┬─────────────┬────────────┤
│    Waste    │   Chemical  │  Regulatory │   Carbon    │   Digital  │
│  Collection │   Supply    │  Compliance │  Tracking   │    Twin    │
└─────────────┴─────────────┴─────────────┴─────────────┴────────────┘
```

---

## 2. Waste Collection Integration

### 2.1 Municipal Waste Systems

**Data Exchange Interface:**

```json
{
  "collection_manifest": {
    "manifest_id": "MWS-2025-001",
    "source": "municipal",
    "collection_date": "2025-01-15",
    "origin": {
      "region": "Lyon Metropolitan Area",
      "collection_points": 45
    },
    "materials": [
      {
        "type": "PET",
        "weight_kg": 5000,
        "contamination_estimate": "low",
        "sorting_accuracy": 0.95
      }
    ],
    "destination_facility": "urn:wia:facility:lyon-01"
  }
}
```

**Integration Requirements:**
- Support for EDI (Electronic Data Interchange) formats
- Real-time or batch data synchronization
- Material tracking from collection to processing

### 2.2 Industrial Waste Streams

| Source Type | Data Required | Integration Method |
|-------------|---------------|-------------------|
| Manufacturing off-spec | Grade, composition | API or EDI |
| Trim and scrap | Weight, source line | Barcode/RFID |
| End-of-life returns | Product ID, condition | Take-back portal |

### 2.3 Consumer Take-back Programs

**API Endpoint:**

```yaml
POST /api/v1/takeback/register
```

**Request:**
```json
{
  "program_id": "brand-takeback-001",
  "location": {
    "type": "retail",
    "store_id": "STORE-12345",
    "address": "123 Main St, City"
  },
  "collection": {
    "date": "2025-01-15",
    "items_count": 250,
    "estimated_weight_kg": 25,
    "plastic_types": ["PET"]
  }
}
```

---

## 3. Chemical Supply Chain Integration

### 3.1 Monomer Trading Platform

**Product Listing Schema:**

```json
{
  "product_listing": {
    "listing_id": "TPA-2025-001",
    "product": {
      "type": "TPA",
      "grade": "food-contact",
      "purity": 99.2,
      "quantity_kg": 10000,
      "batch_ids": ["BATCH-001", "BATCH-002"]
    },
    "pricing": {
      "price_per_kg_usd": 1.25,
      "incoterms": "FOB",
      "currency": "USD"
    },
    "availability": {
      "available_date": "2025-01-20",
      "pickup_location": "Lyon, France"
    },
    "certifications": [
      "WIA-FC-2025-001",
      "ISO-14001"
    ]
  }
}
```

### 3.2 Supply Chain Events

**Event Types:**

| Event | Trigger | Data |
|-------|---------|------|
| `batch.ready` | QC complete | Batch ID, quantity, grade |
| `shipment.dispatched` | Truck departure | Tracking #, ETA |
| `shipment.delivered` | Receipt confirmed | POD, actual weight |
| `quality.verified` | Customer QC | Pass/fail, deviations |

### 3.3 Polymer Manufacturer Integration

**Order Fulfillment API:**

```yaml
POST /api/v1/orders
```

```json
{
  "order": {
    "customer_id": "POLYMER-MFG-001",
    "items": [
      {
        "product": "TPA",
        "grade": "bottle-grade",
        "quantity_kg": 50000,
        "delivery_date": "2025-02-01"
      }
    ],
    "delivery": {
      "address": "456 Industrial Blvd",
      "contact": "John Smith",
      "special_instructions": "Bulk tanker only"
    }
  }
}
```

---

## 4. Regulatory Compliance Integration

### 4.1 Food Contact Certification

**Regulatory Bodies:**
- EU: EFSA (European Food Safety Authority)
- US: FDA (Food and Drug Administration)
- Japan: JHOSPA

**Certification Data Schema:**

```json
{
  "food_contact_certification": {
    "cert_id": "WIA-FC-2025-001",
    "facility_id": "urn:wia:facility:lyon-01",
    "product": "TPA",
    "grade": "food-contact",
    "regulations": [
      {"code": "EC 282/2008", "status": "compliant"},
      {"code": "FDA 21 CFR 177.1630", "status": "compliant"}
    ],
    "testing": {
      "lab": "Certified Testing Lab Inc.",
      "report_id": "LAB-2025-001",
      "date": "2025-01-10"
    },
    "validity": {
      "issued": "2025-01-15",
      "expires": "2026-01-15"
    }
  }
}
```

### 4.2 Extended Producer Responsibility (EPR)

**EPR Reporting Interface:**

```json
{
  "epr_report": {
    "reporting_period": "2025-Q1",
    "facility_id": "urn:wia:facility:lyon-01",
    "processing_summary": {
      "total_input_kg": 500000,
      "total_recycled_kg": 475000,
      "recycling_rate": 0.95
    },
    "producer_allocations": [
      {
        "producer_id": "COCA-COLA-EU",
        "allocated_kg": 150000,
        "credits_issued": 150
      }
    ],
    "verification": {
      "auditor": "Bureau Veritas",
      "audit_date": "2025-04-01"
    }
  }
}
```

### 4.3 Environmental Permits

**Permit Compliance API:**

```yaml
GET /api/v1/compliance/permits/{facility_id}
```

```json
{
  "permits": [
    {
      "permit_id": "ENV-2024-001",
      "type": "water_discharge",
      "issuing_authority": "Regional Environment Agency",
      "status": "active",
      "conditions": [
        {"parameter": "pH", "limit": "6-9", "frequency": "daily"},
        {"parameter": "TSS", "limit": "50 mg/L", "frequency": "weekly"}
      ],
      "next_renewal": "2026-12-31"
    }
  ]
}
```

---

## 5. Carbon Footprint Integration

### 5.1 Life Cycle Assessment Data

**LCA Data Exchange Format:**

```json
{
  "lca_data": {
    "facility_id": "urn:wia:facility:lyon-01",
    "reporting_period": "2025-01",
    "functional_unit": "1 ton PET processed",
    "emissions": {
      "scope_1": {
        "value": 15.2,
        "unit": "kg CO2e",
        "sources": ["natural_gas", "refrigerants"]
      },
      "scope_2": {
        "value": 45.8,
        "unit": "kg CO2e",
        "sources": ["electricity"],
        "method": "market-based"
      },
      "scope_3": {
        "value": 12.5,
        "unit": "kg CO2e",
        "categories": ["upstream_transport", "waste"]
      }
    },
    "total": 73.5,
    "benchmark_comparison": {
      "virgin_pet": 2100,
      "mechanical_recycling": 450,
      "avoided_emissions": 2026.5
    }
  }
}
```

### 5.2 Carbon Credit Generation

**Credit Issuance API:**

```yaml
POST /api/v1/carbon/credits/issue
```

```json
{
  "credit_request": {
    "facility_id": "urn:wia:facility:lyon-01",
    "project_id": "WIA-CARBON-2025-001",
    "period": "2025-Q1",
    "methodology": "VCS-0046",
    "baseline_emissions": 2100,
    "project_emissions": 73.5,
    "credits_requested": 2026.5,
    "verification": {
      "verifier": "Gold Standard",
      "report_id": "GS-2025-001"
    }
  }
}
```

### 5.3 Carbon Registry Integration

Supported carbon registries:
- Verra (VCS)
- Gold Standard
- American Carbon Registry
- Climate Action Reserve

---

## 6. Digital Twin Integration

### 6.1 Real-time Data Streaming

**IoT Data Format:**

```json
{
  "sensor_data": {
    "facility_id": "urn:wia:facility:lyon-01",
    "timestamp": "2025-01-15T10:30:00.000Z",
    "reactor_1": {
      "temperature_c": 54.8,
      "ph": 8.02,
      "agitation_rpm": 152,
      "substrate_level_percent": 85,
      "tpa_concentration_mm": 185.5
    },
    "utilities": {
      "electricity_kw": 245.8,
      "water_flow_lpm": 12.5,
      "steam_kg_hr": 85.2
    }
  }
}
```

### 6.2 Predictive Maintenance

**Equipment Health API:**

```json
{
  "equipment_health": {
    "equipment_id": "PUMP-001",
    "type": "centrifugal_pump",
    "current_status": "operational",
    "health_score": 0.87,
    "predictions": {
      "remaining_useful_life_days": 120,
      "failure_probability_30d": 0.05,
      "recommended_maintenance": "2025-03-15"
    },
    "alerts": [
      {
        "type": "vibration_anomaly",
        "severity": "low",
        "detected_at": "2025-01-14T15:00:00Z"
      }
    ]
  }
}
```

### 6.3 Process Optimization

**Optimization Recommendations:**

```json
{
  "optimization": {
    "timestamp": "2025-01-15T10:30:00Z",
    "recommendations": [
      {
        "action": "Reduce temperature by 2°C",
        "expected_benefit": "5% energy reduction",
        "confidence": 0.85
      },
      {
        "action": "Increase enzyme concentration by 0.5 mg/g",
        "expected_benefit": "8% faster degradation",
        "confidence": 0.78
      }
    ]
  }
}
```

---

## 7. Partner Ecosystem

### 7.1 Partner Categories

| Category | Role | Integration Level |
|----------|------|-------------------|
| Enzyme suppliers | Provide enzyme products | Order, inventory |
| Equipment vendors | Supply and maintain equipment | Maintenance, parts |
| Testing labs | Quality analysis | Results, certification |
| Logistics partners | Transportation | Tracking, scheduling |
| Brand partners | Off-take agreements | Orders, forecasts |

### 7.2 Partner API Authentication

Partners authenticate using OAuth 2.0 with partner-specific scopes:

```
partner:enzyme:read
partner:enzyme:order
partner:logistics:track
partner:quality:submit
```

### 7.3 Data Sharing Agreements

All partner integrations require:

1. Signed Data Processing Agreement
2. API Terms of Service acceptance
3. Security assessment completion
4. Compliance certification

---

## 8. Implementation Requirements

### 8.1 Technical Requirements

| Requirement | Specification |
|-------------|---------------|
| API Protocol | HTTPS (TLS 1.3) |
| Authentication | OAuth 2.0 / API Key |
| Data Format | JSON, JSON-LD |
| Availability | 99.9% uptime |
| Response Time | <500ms (p95) |
| Data Retention | Per regulatory requirements |

### 8.2 Security Requirements

- All data encrypted in transit (TLS 1.3)
- Sensitive data encrypted at rest (AES-256)
- API keys rotated every 90 days
- Audit logging for all data access
- Annual security assessment required

### 8.3 Compliance Requirements

| Standard | Requirement |
|----------|-------------|
| GDPR | Personal data handling for EU |
| SOC 2 | Security controls attestation |
| ISO 27001 | Information security management |
| ISO 14001 | Environmental management |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-PLASTIC-ENZYME is evaluated across three tiers:

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

- `wia-standards/standards/WIA-PLASTIC-ENZYME/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-PLASTIC-ENZYME/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-PLASTIC-ENZYME/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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

