# WIA-CRYO-010 PHASE 4: INTEGRATION

**Standard**: WIA-CRYO-010  
**Phase**: 4 - System Integration and Deployment  
**Version**: 1.0.0  
**Date**: January 2025  
**Status**: Active  

## Overview

Phase 4 provides guidance for integrating WIA-CRYO-010 into existing laboratory and research systems.

## 4.1 LIMS Integration

### Data Flow
```
LIMS → WIA-CRYO-010 Adapter → Standardized JSON-LD → Repository
```

### Mapping Table
Map LIMS-specific fields to WIA-CRYO-010 schema:
```json
{
  "lims.sample_id": "sampleId",
  "lims.operator_name": "operator",
  "lims.test_date": "assessmentTime"
}
```

### Example Adapters
- LabWare LIMS
- LabVantage
- STARLIMS
- Custom in-house systems

## 4.2 Electronic Lab Notebook (ELN) Integration

### ELN Export
Configure ELN to export entries in WIA-CRYO-010 format:
```python
# LabArchives plugin example
def export_to_wia(entry_id):
    entry = get_entry(entry_id)
    wia_data = {
        "@context": "https://wia.org/standards/cryo-research/v1",
        "@type": "CryoResearchExperiment",
        "experimentId": entry.id,
        "title": entry.title,
        ...
    }
    return wia_data
```

## 4.3 Equipment Integration

### Temperature Logger
Connect via serial, USB, or network:
```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)
while True:
    line = ser.readline().decode('utf-8')
    temp, timestamp = parse_reading(line)
    publish_to_wia({
        "type": "temperature_update",
        "value": temp,
        "timestamp": timestamp
    })
```

### Flow Cytometer
Export FCS files + metadata in WIA format:
```json
{
  "@type": "ViabilityAssessment",
  "method": "FLOW_CYTOMETRY",
  "rawDataFile": "sample001.fcs",
  "results": {...}
}
```

## 4.4 Database Schema

### PostgreSQL Example
```sql
CREATE TABLE experiments (
    experiment_id VARCHAR(255) PRIMARY KEY,
    json_ld JSONB NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    modified_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_experiments_type ON experiments ((json_ld->>'@type'));
CREATE INDEX idx_experiments_status ON experiments ((json_ld->'lifecycle'->>'status'));
```

### MongoDB Example
```javascript
db.createCollection("experiments", {
   validator: {
      $jsonSchema: {
         bsonType: "object",
         required: ["@context", "@type", "experimentId"],
         properties: {
            "@context": {
               bsonType: "string",
               pattern: "^https://wia.org/standards/cryo-research/"
            }
         }
      }
   }
})
```

## 4.5 Backup and Disaster Recovery

### 3-2-1 Rule
- 3 copies of data
- 2 different media types
- 1 off-site backup

### Automated Backup Script
```bash
#!/bin/bash
# Daily backup of WIA-CRYO-010 data
DATE=$(date +%Y%m%d)
pg_dump cryo_research > /backup/cryo_${DATE}.sql
gzip /backup/cryo_${DATE}.sql
aws s3 cp /backup/cryo_${DATE}.sql.gz s3://wia-backups/
```

## 4.6 Security Considerations

### Encryption
- Data at rest: AES-256
- Data in transit: TLS 1.3
- API keys: bcrypt hashed

### Access Control
```yaml
roles:
  - name: researcher
    permissions: [read, write:own]
  - name: data_manager
    permissions: [read, write:all, delete:own]
  - name: admin
    permissions: [read, write:all, delete:all, admin]
```

### Audit Logging
```json
{
  "action": "UPDATE",
  "resourceType": "Experiment",
  "resourceId": "exp-2025-001",
  "userId": "user-123",
  "timestamp": "2025-01-21T10:15:00Z",
  "changes": {...}
}
```

## 4.7 Performance Optimization

### Caching
- Redis for frequently accessed experiments
- CDN for static schemas
- Database query optimization with indexes

### Bulk Operations
Use batch endpoints for large data imports:
```http
POST /bulk-import
Content-Type: application/x-ndjson

{"@type": "CryoResearchExperiment", ...}
{"@type": "CryoResearchExperiment", ...}
```

## 4.8 Monitoring and Alerting

### Metrics to Track
- API response time (p50, p95, p99)
- Error rate
- Data ingestion rate
- Storage usage

### Alerting Rules
```yaml
alerts:
  - name: high_error_rate
    condition: error_rate > 5%
    notify: ops-team@wia.org
  - name: low_viability_detected
    condition: viability < 70%
    notify: researcher@wia.org
```

## 4.9 Deployment Patterns

### Containerization (Docker)
```dockerfile
FROM python:3.10
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["python", "app.py"]
```

### Kubernetes
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: cryo-research-api
spec:
  replicas: 3
  template:
    spec:
      containers:
      - name: api
        image: wia/cryo-research-api:1.0.0
        ports:
        - containerPort: 8080
```

## 4.10 Testing

### Unit Tests
```python
def test_viability_calculation():
    result = calculate_viability(87, 100)
    assert result['viability'] == 87.0
    assert len(result['ci_95']) == 2
```

### Integration Tests
```python
def test_api_submit_experiment():
    response = client.post('/experiments', json=experiment_data)
    assert response.status_code == 201
    assert 'experimentId' in response.json()
```

### Load Testing
```bash
# Using Apache Bench
ab -n 1000 -c 10 https://api.wia.org/cryo-research/v1/experiments
```

## References

- Docker: https://www.docker.com/
- Kubernetes: https://kubernetes.io/
- PostgreSQL: https://www.postgresql.org/
- MongoDB: https://www.mongodb.com/

---

**Previous**: [PHASE-3: Protocols](PHASE-3.md)  
**Complete**: All phases documented

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for cryo-research is evaluated across three tiers:

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

- `wia-standards/standards/cryo-research/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cryo-research/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cryo-research/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-4

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4.

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
