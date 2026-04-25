# WIA-SOC-011: Gas Supply Standard
## PHASE 4: WIA Integration Specification

**Version:** 1.0  
**Status:** Complete  
**Last Updated:** 2025-12-26

---

## 1. Overview

Phase 4 defines integration patterns for WIA-SOC-011 compliant systems with each other and with other WIA standards, creating a comprehensive smart infrastructure ecosystem.

---

## 2. Inter-Operator Pipeline Integration

### 2.1 Nomination and Scheduling

**Data Exchange Format**:
```json
{
  "nominationId": "NOM-2025-001",
  "effectiveDate": "2025-12-27",
  "deliveryPoint": "INTERCONNECT-KR-JP-001",
  "shipper": "Korea Gas Corporation",
  "receiver": "Tokyo Gas",
  "quantity_m3_day": 5000000,
  "pressure_bar": 70,
  "qualitySpec": {
    "wobbeIndex_min": 49.0,
    "wobbeIndex_max": 52.0,
    "heatingValue_min_MJ_m3": 37.0
  },
  "status": "confirmed"
}
```

### 2.2 Balancing at Interconnects

Real-time balancing data exchange:
- Frequency: Every 5 minutes
- Parameters: Flow rate, pressure, temperature, composition
- Reconciliation: Hourly totals with ±2% tolerance

---

## 3. LNG Terminal Integration

### 3.1 Ship Schedule Integration

```json
{
  "vesselArrivalId": "VESSEL-2025-001",
  "vessel": {
    "name": "LNG Carrier Alpha",
    "imo": "IMO1234567",
    "capacity_m3": 174000
  },
  "arrival": {
    "eta": "2025-12-28T06:00:00Z",
    "berthAssignment": "BERTH-1",
    "cargoVolume_m3": 165000
  },
  "regasificationPlan": {
    "startTime": "2025-12-28T12:00:00Z",
    "duration_hours": 48,
    "sendoutRate_m3_h": 3500
  },
  "pipelineAllocation": [
    {"pipelineId": "PL-KR-001-2025", "allocation_percent": 60},
    {"pipelineId": "PL-KR-002-2025", "allocation_percent": 40}
  ]
}
```

### 3.2 Storage Management

Integration with pipeline demand forecasting:
- Minimum storage level: 20% capacity
- Maximum fill rate: 5000 m³/hour
- Regasification capacity: 15 MTPA

---

## 4. Smart Meter Integration (AMI)

### 4.1 Meter Data Collection

**Collection Frequency**:
- Residential: Hourly
- Commercial: 15-minute intervals
- Industrial: Real-time (1-minute)

**Data Format**:
```json
{
  "meterId": "GM-2025-KR-001234",
  "readings": [
    {
      "timestamp": "2025-12-26T14:00:00Z",
      "volume_m3": 2547.35,
      "flow_m3_h": 1.2,
      "pressure_bar": 0.025,
      "temperature_C": 15.3
    }
  ],
  "events": [
    {"type": "low_pressure", "timestamp": "2025-12-26T13:45:00Z"}
  ]
}
```

### 4.2 Leak Detection from Consumption Patterns

Machine learning models analyze consumption patterns:
- Baseline consumption profiles
- Anomaly detection algorithms
- Automated leak notifications
- Integration with field crews

---

## 5. Renewable Gas Interconnection

### 5.1 Biomethane Injection

**Interconnection Agreement Data**:
```json
{
  "interconnectionId": "INTERCONN-BIO-001",
  "producer": "Green Energy Farm",
  "location": {"pipelineId": "PL-KR-005-2025", "kilometer": 23.5},
  "capacity": {
    "max_m3_h": 500,
    "annual_m3": 4000000
  },
  "qualityRequirements": {
    "methane_min_percent": 95,
    "CO2_max_percent": 3,
    "H2S_max_ppm": 5,
    "moisture_max_mg_m3": 50
  },
  "meteringStandard": "WIA-SOC-011",
  "dataExchangeProtocol": "MQTT"
}
```

### 5.2 Hydrogen Blending Coordination

**Blending Control**:
- Real-time composition monitoring
- Maximum H2 concentration: 20% by volume
- Automatic injection rate adjustment
- End-user equipment compatibility tracking

---

## 6. Cross-Standard WIA Integration

### 6.1 Electric Power Integration (WIA-ENE-001)

**Combined Heat and Power (CHP)**:
```json
{
  "integrationPoint": "CHP-PLANT-001",
  "gasSupply": {
    "standard": "WIA-SOC-011",
    "pipelineId": "PL-KR-001-2025",
    "flowRate_m3_h": 2500
  },
  "powerOutput": {
    "standard": "WIA-ENE-001",
    "gridConnection": "GC-KR-SEOUL-01",
    "capacity_MW": 25
  },
  "efficiency": {
    "electrical": 0.38,
    "thermal": 0.45,
    "combined": 0.83
  }
}
```

### 6.2 Water Infrastructure (WIA-SOC-010)

LNG terminal cooling water integration:
- Water consumption: 150 m³/hour
- Return water temperature: +8°C delta
- Water quality monitoring
- Environmental compliance reporting

### 6.3 Smart City Integration (WIA-CITY-001)

**Infrastructure Coordination**:
- Joint excavation planning
- Utility corridor sharing
- Emergency response coordination
- Shared digital twin platform

---

## 7. Emergency Response Integration

### 7.1 Multi-Agency Coordination

```json
{
  "incidentId": "INC-2025-001",
  "type": "pipeline_leak",
  "severity": "high",
  "location": {
    "pipelineId": "PL-KR-001-2025",
    "gps": [37.5665, 126.9780],
    "address": "123 Main Street, Seoul"
  },
  "notifications": [
    {"agency": "fire_department", "notified": "2025-12-26T14:00:00Z"},
    {"agency": "police", "notified": "2025-12-26T14:00:05Z"},
    {"agency": "gas_emergency", "notified": "2025-12-26T14:00:01Z"}
  ],
  "actions": [
    {"action": "isolate_section", "status": "completed", "timestamp": "2025-12-26T14:02:00Z"},
    {"action": "evacuate_radius_100m", "status": "in_progress"}
  ]
}
```

---

## 8. Digital Twin Integration

### 8.1 Real-time System Modeling

**Data Synchronization**:
- Physical measurements → Digital twin: Real-time
- Digital twin simulations → Operations: On-demand
- Model calibration: Daily
- Scenario planning: Continuous

**Applications**:
- Hydraulic modeling and optimization
- Predictive maintenance scheduling
- Emergency scenario simulation
- Capacity planning

---

## 9. Blockchain for Gas Trading

### 9.1 Smart Contracts

**Use Cases**:
- Automated nominations and confirmations
- Real-time balancing settlements
- Renewable gas certificate trading
- Carbon credit tracking

**Integration Points**:
- Meter data → Blockchain: Automated
- Nominations → Smart contracts: API
- Settlements → Financial systems: Batch daily

---

## 10. Legacy System Integration

### 10.1 Protocol Translation Gateways

**Gateway Architecture**:
```
Legacy SCADA (proprietary) 
    ↔ 
Protocol Gateway (translation)
    ↔ 
WIA-SOC-011 Standard Interface
```

**Supported Legacy Protocols**:
- Proprietary SCADA protocols
- Old Modbus RTU (serial)
- Legacy database formats
- Custom CSV exports

---

## 11. Certification Requirements

### 11.1 Integration Certification

**Test Scenarios**:
1. Inter-operator data exchange
2. LNG terminal coordination
3. Smart meter aggregation
4. Renewable gas interconnection
5. Emergency response coordination

**Certification Criteria**:
- 100% data format compliance
- <5s latency for critical operations
- 99.9% message delivery success
- Security assessment passed
- Interoperability with 3+ certified systems

---

## 12. Migration Strategies

### 12.1 Phased Rollout

**Phase 1**: Data format standardization (6 months)
**Phase 2**: API implementation (6 months)
**Phase 3**: Protocol migration (12 months)
**Phase 4**: Full integration (6 months)

Total timeline: 30 months for complete migration

---

**Document Control**
- Version: 1.0
- Integration Patterns: https://github.com/WIA-Official/wia-standards/tree/main/gas-supply/integration
- Contact: standards@wiastandards.com

© 2025 World Certification Industry Association | MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for gas-supply is evaluated across three tiers:

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

- `wia-standards/standards/gas-supply/api/` — TypeScript SDK skeleton
- `wia-standards/standards/gas-supply/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/gas-supply/simulator/` — interactive browser-based simulator for the PHASE protocol

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

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
