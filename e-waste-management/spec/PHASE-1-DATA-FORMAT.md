# WIA E-Waste Management Standard
# Phase 1: Data Format Specification v1.0

## Document Information

- **Standard**: WIA E-Waste Management
- **Phase**: 1 - Data Format
- **Version**: 1.0.0
- **Status**: Published
- **Date**: 2025-01-15
- **Maintainer**: World Industry Association (WIA)
- **License**: Creative Commons BY-SA 4.0

## 1. Introduction

### 1.1 Purpose

This specification defines standardized data formats for electronic waste (e-waste) management, enabling consistent classification, tracking, and reporting across the entire electronics lifecycle. These formats provide the foundational vocabulary for all subsequent phases of the WIA E-Waste Management Standard.

### 1.2 Scope

Phase 1 data formats cover:
- Device classification and identification
- Material composition declarations
- Lifecycle event tracking
- Processing and recovery reporting
- Compliance documentation

### 1.3 Normative References

- ISO 8601: Date and time format
- RFC 4122: UUID specification
- JSON Schema Draft 2020-12
- WEEE Directive 2012/19/EU
- Basel Convention on Hazardous Wastes
- RoHS Directive 2011/65/EU

## 2. Device Classification Schema

### 2.1 WEEE Category Taxonomy

All electronic devices SHALL be classified using the six-category WEEE framework:

```json
{
  "weee_categories": {
    "WEEE-1": {
      "name": "Temperature Exchange Equipment",
      "description": "Refrigerators, air conditioners, heat pumps",
      "typical_weight_range_kg": [15, 100],
      "hazard_level": "HIGH"
    },
    "WEEE-2": {
      "name": "Screens and Monitors",
      "description": "TVs, computer monitors, laptops, tablets",
      "typical_weight_range_kg": [2, 30],
      "hazard_level": "MEDIUM_HIGH"
    },
    "WEEE-3": {
      "name": "Lamps",
      "description": "LED, fluorescent, CFL lamps",
      "typical_weight_range_kg": [0.05, 1],
      "hazard_level": "MEDIUM"
    },
    "WEEE-4": {
      "name": "Large Equipment",
      "description": "Washing machines, dryers, ovens, solar panels",
      "typical_weight_range_kg": [30, 150],
      "hazard_level": "MEDIUM"
    },
    "WEEE-5": {
      "name": "Small Equipment",
      "description": "Phones, cameras, toasters, vacuums",
      "typical_weight_range_kg": [0.1, 10],
      "hazard_level": "LOW_MEDIUM"
    },
    "WEEE-6": {
      "name": "Small IT and Telecom Equipment",
      "description": "Routers, keyboards, mice, GPS devices",
      "typical_weight_range_kg": [0.05, 2],
      "hazard_level": "LOW"
    }
  }
}
```

### 2.2 Device Identification Format

Each device MUST be assigned a unique identifier following this structure:

```json
{
  "$schema": "https://wia-standards.org/schemas/device-v1.0.json",
  "device_id": "550e8400-e29b-41d4-a716-446655440000",
  "weee_category": "WEEE-5",
  "weee_subcategory": "smartphone",
  "manufacturer": {
    "name": "Example Corp",
    "facility_id": "FC-CN-SZ-001",
    "country_code": "CN"
  },
  "model": {
    "name": "ExamplePhone Pro 15",
    "number": "EP15-256-BK",
    "variant": "Black, 256GB"
  },
  "manufacturing": {
    "date": "2024-03-15",
    "batch_number": "2024-Q1-B0342",
    "facility_id": "FC-CN-SZ-001"
  },
  "physical_properties": {
    "weight_kg": 0.189,
    "dimensions_mm": {
      "length": 146.7,
      "width": 71.5,
      "depth": 7.8
    }
  },
  "serial_number": "EX24031512345",
  "qr_code_url": "https://ewaste.example.com/qr/550e8400",
  "created_at": "2024-03-15T14:23:00Z",
  "schema_version": "1.0.0"
}
```

#### 2.2.1 Required Fields

- `device_id`: UUID v4 (RFC 4122)
- `weee_category`: One of WEEE-1 through WEEE-6
- `manufacturer.name`: Legal entity name
- `model.name`: Commercial product name
- `weight_kg`: Device weight in kilograms (precision: 3 decimal places)
- `created_at`: ISO 8601 timestamp

#### 2.2.2 Optional Fields

- `weee_subcategory`: Device-specific classification
- `manufacturing.date`: Production date
- `dimensions_mm`: Physical dimensions
- `serial_number`: Manufacturer serial number
- `qr_code_url`: Scannable QR code for tracking

## 3. Material Composition Declaration

### 3.1 Bill of Materials Format

Manufacturers SHALL declare material composition using hierarchical component structure:

```json
{
  "device_id": "550e8400-e29b-41d4-a716-446655440000",
  "bill_of_materials": {
    "version": "1.2.0",
    "last_updated": "2024-03-15T14:23:00Z",
    "components": [
      {
        "component_id": "COMP-001",
        "component_name": "Main Circuit Board",
        "component_type": "PCB",
        "weight_g": 18.5,
        "materials": [
          {
            "substance": "Copper",
            "cas_number": "7440-50-8",
            "weight_g": 5.2,
            "percentage": 28.1,
            "form": "traces_and_layers",
            "location": "Conductive traces throughout PCB"
          },
          {
            "substance": "Gold",
            "cas_number": "7440-57-5",
            "weight_g": 0.024,
            "percentage": 0.13,
            "form": "plating",
            "location": "Connectors and contact pads",
            "recovery_priority": "HIGH"
          }
        ],
        "hazardous_substances": [
          {
            "substance": "Lead",
            "cas_number": "7439-92-1",
            "weight_g": 0.15,
            "concentration_ppm": 8100,
            "location": "Legacy solder joints (pre-RoHS)",
            "rohs_exempt": true,
            "exemption_code": "7a",
            "handling_requirements": "Segregate for specialized processing"
          }
        ]
      }
    ],
    "total_weight_g": 189.0,
    "material_summary": {
      "precious_metals_g": 0.035,
      "base_metals_g": 45.2,
      "plastics_g": 82.3,
      "glass_g": 28.1,
      "other_g": 33.365
    }
  }
}
```

### 3.2 Hazardous Substance Registry

All hazardous substances MUST be identified by CAS number from the standard registry:

| Substance | CAS Number | Typical Sources | Handling Priority |
|-----------|------------|-----------------|-------------------|
| Mercury (Hg) | 7439-97-6 | Backlights, switches, batteries | CRITICAL |
| Lead (Pb) | 7439-92-1 | Solder, CRT glass, batteries | HIGH |
| Cadmium (Cd) | 7440-43-9 | NiCd batteries, pigments | HIGH |
| Hexavalent Chromium (Cr VI) | 18540-29-9 | Metal coatings, primers | HIGH |
| PBBs | Various | Flame retardants | MEDIUM |
| PBDEs | Various | Flame retardants | MEDIUM |

## 4. Lifecycle Event Tracking

### 4.1 Event Schema

Devices accumulate lifecycle events from manufacturing through end-of-life:

```json
{
  "device_id": "550e8400-e29b-41d4-a716-446655440000",
  "lifecycle_events": [
    {
      "event_id": "evt_mfg_001",
      "event_type": "manufacturing_complete",
      "timestamp": "2024-03-15T14:23:00Z",
      "location": {
        "facility_id": "FC-CN-SZ-001",
        "country_code": "CN",
        "coordinates": {
          "latitude": 22.5431,
          "longitude": 114.0579
        }
      },
      "actor": {
        "organization_id": "ORG-EXAMPLE-001",
        "role": "manufacturer"
      },
      "metadata": {
        "production_line": "Line-7",
        "quality_check": "passed",
        "batch_number": "2024-Q1-B0342"
      }
    },
    {
      "event_id": "evt_collect_001",
      "event_type": "collected_for_recycling",
      "timestamp": "2025-11-08T09:45:00Z",
      "location": {
        "facility_id": "COL-US-NY-018",
        "address": "Brooklyn, NY, USA",
        "country_code": "US"
      },
      "actor": {
        "organization_id": "ORG-COLLECTOR-042",
        "role": "collector"
      },
      "metadata": {
        "collection_method": "retail_takeback",
        "device_condition": "functional_obsolete",
        "data_wiped": true,
        "visual_damage": "minor_scratches"
      }
    }
  ]
}
```

### 4.2 Standard Event Types

| Event Type | Description | Required Metadata | Actor Role |
|------------|-------------|-------------------|------------|
| manufacturing_complete | Device production finished | batch_number, quality_check | manufacturer |
| first_sale | Initial consumer purchase | retailer, warranty_start | retailer |
| ownership_transfer | Change of ownership | previous_owner, new_owner | various |
| repair_service | Device repaired | components_replaced, service_provider | service_center |
| collected_for_recycling | Entered waste stream | collection_method, condition | collector |
| data_wiped | Secure data erasure | wipe_standard, certificate_id | processor |
| dismantled | Device disassembled | components_extracted | processor |
| material_recovered | Materials extracted | materials_list, quantities | processor |

## 5. Material Recovery Reporting

### 5.1 Recovery Report Format

Processing facilities document recovery outcomes:

```json
{
  "recovery_report_id": "RR-2025-11-08-001",
  "facility_id": "RC-US-NY-003",
  "processing_date": "2025-11-08",
  "certification": ["R2", "e-Stewards"],
  "input_devices": [
    {
      "device_id": "550e8400-e29b-41d4-a716-446655440000",
      "weee_category": "WEEE-5",
      "weight_kg": 0.189
    }
  ],
  "batch_summary": {
    "total_devices": 250,
    "total_weight_kg": 52.7,
    "processing_method": "manual_dismantling_with_mechanical_separation"
  },
  "recovered_materials": [
    {
      "material": "Copper",
      "cas_number": "7440-50-8",
      "weight_kg": 6.8,
      "purity_percentage": 94.5,
      "recovery_method": "mechanical_separation",
      "destination": {
        "facility_id": "SM-US-PA-001",
        "facility_type": "secondary_smelter",
        "certification": "ISO_14001"
      },
      "estimated_value_usd": 64.60
    }
  ],
  "residual_outputs": [
    {
      "material_type": "mixed_plastics",
      "weight_kg": 18.3,
      "disposition": "energy_recovery",
      "facility_id": "EF-US-NJ-002"
    }
  ],
  "recovery_efficiency": {
    "overall_recovery_rate_pct": 89.2,
    "material_recovery_pct": 72.5,
    "energy_recovery_pct": 16.7
  }
}
```

## 6. Data Validation Rules

### 6.1 Field Validation

All implementations MUST enforce:

- **device_id**: Valid UUID v4 format
- **weee_category**: Enum [WEEE-1, WEEE-2, WEEE-3, WEEE-4, WEEE-5, WEEE-6]
- **weight_kg**: Positive number, max 3 decimal places
- **timestamp**: ISO 8601 format with timezone
- **cas_number**: Valid CAS registry number format (XXXX-XX-X or XXXXX-XX-X or XXXXXX-XX-X)
- **percentage**: Number between 0 and 100
- **coordinates**: Latitude [-90, 90], Longitude [-180, 180]

### 6.2 Cross-Field Validation

- Total component weights MUST sum to total device weight (±2% tolerance)
- Material percentages within component MUST sum to ~100% (±5% tolerance)
- Event timestamps MUST be chronologically ordered
- Recovery report input weights MUST match device registration weights

## 7. Extensibility

### 7.1 Custom Fields

Organizations MAY add custom fields using namespaced extensions:

```json
{
  "device_id": "550e8400-e29b-41d4-a716-446655440000",
  "weee_category": "WEEE-5",
  "extensions": {
    "example_corp:internal": {
      "factory_batch": "FC7-2024-Q1",
      "qc_inspector_id": "QC-42"
    },
    "recycler_co:assessment": {
      "reuse_score": 8.5,
      "battery_health_pct": 87
    }
  }
}
```

Extension namespaces MUST use reverse-DNS notation to avoid conflicts.

## 8. Security and Privacy

### 8.1 Personal Data Protection

- Device tracking MUST NOT include personally identifiable information
- Location data SHOULD be generalized to city-level when consumer devices
- Data retention policies MUST comply with GDPR and applicable privacy laws
- Consumer right to deletion MUST be supported via device_id de-activation

### 8.2 Data Integrity

- All records SHOULD include cryptographic checksums
- Lifecycle events SHOULD be digitally signed by the creating actor
- Tampering detection mechanisms SHOULD be implemented

## 9. Compliance and Conformance

### 9.1 Conformance Levels

- **Level 1**: Required fields only, manual processes acceptable
- **Level 2**: Required + recommended fields, automated validation
- **Level 3**: Complete data including optional fields, real-time validation

### 9.2 Certification

Organizations claiming Phase 1 compliance MUST:
- Implement all required fields and validation rules
- Provide sample data demonstrating conformance
- Submit to independent verification (for Level 2+)

## 10. Change Log

### Version 1.0.0 (2025-01-15)
- Initial publication
- Core device classification schema
- Material composition framework
- Lifecycle event tracking
- Recovery reporting format

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
