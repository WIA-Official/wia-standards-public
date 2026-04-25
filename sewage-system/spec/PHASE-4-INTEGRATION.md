# WIA-SOC-009 Phase 4: Integration Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 4 defines integration patterns with external systems, including smart city platforms, GIS systems, regulatory reporting, and public transparency portals. Integration enables holistic urban management and regulatory compliance.

## 2. Smart City Platform Integration

### 2.1 Common Data Model

Smart city platforms aggregate data from multiple infrastructure systems. Sewage data MUST conform to common schemas for:

- Geographic Information (GeoJSON)
- Time Series Data (SensorThings API)
- Events and Alerts (Common Alerting Protocol - CAP)
- Assets and Equipment (ISO 15926)

### 2.2 Integration Architecture

```
┌──────────────────────────────────────┐
│     Smart City Platform (Hub)       │
│  - Unified Dashboard                │
│  - Cross-Domain Analytics           │
│  - Coordinated Response             │
└──────────────────────────────────────┘
         ↕ Open APIs (REST/GraphQL)
┌──────────────────────────────────────┐
│   Domain-Specific Systems            │
│ ┌─────────┐ ┌──────────┐ ┌────────┐ │
│ │ Sewage  │ │ Water    │ │ Energy │ │
│ │ (SOC-009│ │ Supply   │ │ Grid   │ │
│ └─────────┘ └──────────┘ └────────┘ │
└──────────────────────────────────────┘
```

### 2.3 API Gateway Pattern

**API Gateway Functions:**
- Authentication and authorization
- Rate limiting and throttling
- Request routing and load balancing
- Protocol translation
- Response caching
- Analytics and monitoring

**Example Integration:**
```yaml
apiVersion: v1
kind: Gateway
metadata:
  name: sewage-api-gateway
spec:
  routes:
    - path: /sewage/flow
      backend: flow-service
      methods: [GET]
      auth: oauth2
      rateLimit: 1000/hour
    - path: /sewage/quality
      backend: quality-service
      methods: [GET, POST]
      auth: api-key
      rateLimit: 500/hour
```

## 3. GIS Integration

### 3.1 Spatial Data Requirements

**Asset Mapping:**
- Pipes (location, diameter, material, age)
- Manholes (location, depth, type)
- Pump stations (location, capacity)
- Treatment facilities (location, processes)
- Outfalls (location, permit limits)

**Data Format:** GeoJSON FeatureCollection

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "LineString",
        "coordinates": [
          [-122.4194, 37.7749],
          [-122.4195, 37.7750]
        ]
      },
      "properties": {
        "assetType": "pipe",
        "diameter": 600,
        "material": "concrete",
        "installDate": "1985-03-15",
        "condition": "good",
        "flowCapacity": 8.5
      }
    }
  ]
}
```

### 3.2 Real-Time Overlay Data

**Live Data Layers:**
- Current flow rates (color-coded by capacity %)
- Water quality status (compliant/warning/violation)
- Equipment status (operational/maintenance/fault)
- Active alerts (severity-based symbology)
- Overflow risk predictions (probability heatmap)

**Update Frequency:**
- Flow data: Every 5 minutes
- Water quality: Every 15 minutes
- Equipment status: Every 1 minute
- Predictions: Every hour

### 3.3 GIS Service Standards

**Web Map Service (WMS):** For base mapping

**Web Feature Service (WFS):** For asset queries

**Web Processing Service (WPS):** For spatial analysis

**Example WFS Query:**
```xml
<GetFeature service="WFS" version="2.0.0">
  <Query typeNames="sewage:pipes">
    <Filter>
      <PropertyIsGreaterThan>
        <PropertyName>installDate</PropertyName>
        <Literal>1980-01-01</Literal>
      </PropertyIsGreaterThan>
    </Filter>
  </Query>
</GetFeature>
```

## 4. Regulatory Reporting Integration

### 4.1 Electronic Discharge Monitoring Reports (eDMR)

**EPA NetDMR Integration:**
- Automated data collection from sensors and lab systems
- Quality assurance/quality control checks
- Exceedance flagging
- Secure submission to EPA

**Data Flow:**
```
Sensors → LIMS → QA/QC → eDMR Generator → NetDMR Portal → EPA
```

**Reporting Frequency:**
- Monthly discharge monitoring reports
- Annual reports
- Event-based reports (overflows, bypasses)
- Immediate notification of permit violations

### 4.2 State and Local Reporting

**Customizable Templates:**
- State-specific report formats
- Local municipality requirements
- Watershed management organizations
- Public health departments

**Report Types:**
- Compliance summaries
- Overflow event reports
- Water quality trends
- System performance metrics
- Resource recovery statistics

## 5. Public Transparency Portal

### 5.1 Public Dashboard Requirements

**Data to Display:**
- Real-time system status (high-level overview)
- Water quality at key points (simplified parameters)
- Recent events (overflows, maintenance)
- Compliance record (past 12 months)
- System performance metrics

**Data Granularity:**
- Aggregated data (not raw sensor readings)
- 15-minute to 1-hour resolution
- Geographic summaries (by zone, not specific pipes)
- Privacy protection (no information revealing private properties)

**Example Public API:**
```
GET /public/v1/status
```

Response:
```json
{
  "lastUpdated": "2025-12-26T14:30:00Z",
  "systemStatus": "normal",
  "waterQuality": "compliant",
  "recentEvents": 0,
  "complianceRate": 100,
  "message": "All systems operating normally"
}
```

### 5.2 Public Notification System

**Alert Types:**
- Boil water advisories (if affecting water supply)
- Beach closures (after overflow to recreational waters)
- Odor complaints (proactive notification)
- Planned maintenance (service interruptions)

**Notification Channels:**
- Email subscription lists
- SMS alerts
- Mobile app push notifications
- Social media (Twitter, Facebook)
- Municipal website banners
- Local media (press releases)

## 6. Third-Party Application Integration

### 6.1 Developer API

**Public API Tier:**
- Read-only access to public data
- Rate limited (100 requests/hour)
- API key authentication
- Comprehensive documentation (OpenAPI/Swagger)

**Partner API Tier:**
- Extended data access
- Higher rate limits (1000 requests/hour)
- Priority support
- Early access to new features

**Example Use Cases:**
- Environmental monitoring apps
- Research data access
- Educational dashboards
- Community engagement tools

### 6.2 Webhooks for Event Notifications

**Webhook Configuration:**
```json
{
  "url": "https://partner.example.com/webhooks/sewage-events",
  "events": ["overflow", "quality_violation", "maintenance"],
  "secret": "shared_secret_for_verification",
  "active": true
}
```

**Webhook Payload:**
```json
{
  "eventType": "overflow",
  "timestamp": "2025-12-26T14:30:00Z",
  "location": "CSO-5",
  "severity": "high",
  "data": {
    "volume": 150,
    "duration": 1800,
    "receivingWater": "Example River"
  },
  "signature": "sha256=..."
}
```

## 7. Cross-Domain Integrations

### 7.1 Water Supply Integration

**Data Sharing:**
- Water production vs. sewage influent (leak detection)
- Water quality monitoring (contamination tracing)
- Demand forecasting (coordinated planning)

**Use Cases:**
- Water main break detection (via sewage flow anomalies)
- Cross-contamination prevention
- Integrated water resource management

### 7.2 Stormwater Management Integration

**Combined Sewer System Coordination:**
- Real-time capacity sharing
- Overflow prediction and prevention
- Storage optimization
- Green infrastructure integration

**Separated System Coordination:**
- Infiltration/inflow monitoring
- Illicit discharge detection
- Combined data analysis for watershed health

### 7.3 Energy Grid Integration

**Demand Response:**
- Shift pump operations to off-peak hours
- Battery storage for load leveling
- Renewable energy (solar, biogas) integration

**Data Exchange:**
- Real-time power consumption
- Energy production from biogas
- Demand forecasts
- Curtailment signals

### 7.4 Public Health Integration

**Wastewater-Based Epidemiology:**
- Pathogen monitoring (COVID-19, poliovirus, etc.)
- Drug use surveillance
- Antimicrobial resistance tracking
- Early outbreak detection

**Data Privacy:**
- Aggregated data only (neighborhood level minimum)
- No personally identifiable information
- Secure transmission to health authorities
- Controlled access (authorized personnel only)

## 8. Emergency Response Integration

### 8.1 Emergency Operations Center (EOC)

**Real-Time Situational Awareness:**
- Sewage system status during emergencies
- Hazmat incident response (chemical spills affecting sewage)
- Flood response (sewer capacity and overflow risks)
- Power outage impact (pump stations, treatment facilities)

**Data Feeds to EOC:**
- Critical alerts
- System capacity and status
- Geographic visualization
- Resource availability (crews, equipment)

### 8.2 First Responder Integration

**Manhole and Underground Safety:**
- Gas detection alerts (H2S, methane)
- Water level monitoring (confined space safety)
- Real-time notifications to dispatch

**Mutual Aid Coordination:**
- Resource sharing with neighboring jurisdictions
- Standardized data formats for interoperability
- Emergency contact directories

## 9. Research and Academia Integration

### 9.1 Data Access for Research

**Anonymized Data Sharing:**
- Long-term datasets for trend analysis
- Event data for modeling and simulation
- Performance metrics for benchmarking studies

**Data Use Agreements:**
- Clear terms of use
- Citation requirements
- Embargo periods for sensitive data
- Results sharing with municipality

### 9.2 Living Lab Partnerships

**Pilot Project Integration:**
- Dedicated test zones for new technologies
- Real-time data access for researchers
- Controlled experiments
- Knowledge transfer to operations

## 10. Cloud Platform Integration

### 10.1 Multi-Cloud Strategy

**Supported Platforms:**
- Amazon Web Services (AWS)
- Microsoft Azure
- Google Cloud Platform (GCP)
- On-premise private cloud

**Cloud Services:**
- Managed databases (RDS, Cosmos DB, Cloud SQL)
- Object storage (S3, Blob Storage, Cloud Storage)
- Serverless functions (Lambda, Azure Functions, Cloud Functions)
- Machine learning (SageMaker, Azure ML, Vertex AI)

### 10.2 Data Lake Architecture

**Layered Data Organization:**
- **Raw Layer**: Unprocessed sensor data
- **Curated Layer**: Cleaned and validated data
- **Analytics Layer**: Aggregated and enriched data
- **Serving Layer**: API-ready datasets

**Data Catalog:**
- Metadata management
- Data lineage tracking
- Access control policies
- Data quality metrics

---

© 2025 WIA · MIT License

## P.4 Integration Cross-References

This Phase describes how the data formats (Phase 1), API surface (Phase 2),
and protocol layer (Phase 3) compose with adjacent infrastructure to form a
production deployment.

### P.4.1 Deployment Topologies

| Topology | When to Use | Trade-off |
|----------|------------|-----------|
| Single-region active-passive | Predictable latency, single-region users | Cold standby cost |
| Multi-region active-active | Global users, regional sovereignty | Conflict resolution complexity |
| Edge fan-out | Low latency at the edge, central system of record | Cache coherence |
| Air-gapped enclave | Regulatory / national security domains | Manual reconciliation |

### P.4.2 Dependency Inventory

Every implementation MUST publish a Software Bill of Materials (SBOM) in
SPDX 2.3 or CycloneDX 1.5 format covering: (a) direct runtime dependencies,
(b) transitive dependencies pinned to specific versions, (c) base container
images, (d) cryptographic libraries.

### P.4.3 Operational Readiness Checklist

- [ ] Health check endpoint returns 200 within 1 s p99
- [ ] Metrics exposed in Prometheus or OTLP format
- [ ] Logs are structured JSON with correlation IDs
- [ ] Traces use W3C Trace Context headers end-to-end
- [ ] Backups verified by quarterly restore drill
- [ ] Runbook published and indexed
- [ ] Disaster recovery RTO / RPO documented
- [ ] On-call rotation defined and acknowledged

### P.4.4 Migration Pathways

Adopters migrating from legacy systems should follow the staged pattern:
(1) shadow read, (2) shadow write, (3) primary write with legacy fallback,
(4) primary read, (5) legacy decommission. Each stage runs for at least one
business cycle before the next.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of sewage-system so that conformance claims at any
Phase remain unambiguous.*

