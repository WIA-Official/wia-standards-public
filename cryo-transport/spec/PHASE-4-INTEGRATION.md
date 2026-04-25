# WIA Cryo-Transport Integration Standard
## Phase 4 Specification

**Version**: 1.0.0  
**Status**: Complete  
**License**: MIT

---

## System Architecture

### Transport Management System (TMS)

Components:
- Real-time monitoring dashboard
- Route planning engine
- Alert management system
- Documentation repository
- Quality analytics platform

### Sensor Integration

Supported protocols:
- Modbus RTU/TCP for industrial sensors
- I2C/SPI for embedded sensors
- MQTT for IoT devices
- HTTP/REST for cloud services

### Database Schema

```sql
CREATE TABLE transports (
  transport_id VARCHAR(50) PRIMARY KEY,
  manifest_id VARCHAR(50),
  status VARCHAR(20),
  origin_facility_id VARCHAR(50),
  destination_facility_id VARCHAR(50),
  created_at TIMESTAMP,
  updated_at TIMESTAMP
);

CREATE TABLE monitoring_data (
  monitoring_id BIGSERIAL PRIMARY KEY,
  transport_id VARCHAR(50),
  timestamp TIMESTAMP,
  temperature FLOAT,
  ln2_level FLOAT,
  latitude FLOAT,
  longitude FLOAT,
  FOREIGN KEY (transport_id) REFERENCES transports(transport_id)
);
```

---

## Deployment Configurations

### Cloud Deployment
- Kubernetes cluster (3+ nodes)
- PostgreSQL database (replicated)
- Redis cache
- S3-compatible object storage
- CloudFront CDN

### On-Premise
- Docker Compose stack
- Local PostgreSQL
- File-based storage
- Nginx reverse proxy

---

## Regulatory Compliance Systems

Supported regulations:
- DOT 49 CFR Parts 100-185
- FDA 21 CFR Part 1271
- EU ADR
- IATA Dangerous Goods Regulations

---

## Integration Architecture (Detailed)

### Data Flow

The reference cryogenic-transport platform has four logical tiers:

1. **Edge tier** — Sensor controllers, GPS receivers, satellite backup modems, and the driver's mobile application.
2. **Field aggregation tier** — Optional regional gateways that aggregate sensor traffic from multiple containers in a single fleet.
3. **Platform tier** — TMS services (manifest, telemetry ingestion, custody, alerts, route planning, regulatory reporting).
4. **Integration tier** — Outbound webhooks, regulator submissions, customer portals, EDI integrations with logistics partners.

Each tier publishes its own service-level objectives and is independently observable.

### Sensor Integration

Sensor connectivity follows the published industrial-protocol stack:

| Protocol | Reference | Use case |
|----------|-----------|----------|
| Modbus TCP | Modbus Application Protocol Specification V1.1b3 | Temperature controllers, LN2 level indicators |
| Modbus RTU | Modbus over Serial Line Specification V1.02 | Legacy serial sensors |
| MQTT 5.0 | OASIS MQTT 5.0 Standard | IoT telemetry to cloud broker |
| MQTT-SN | OASIS MQTT-SN | Battery-constrained satellite telemetry |
| OPC UA | IEC 62541 series | Plant-floor sensor integration at facilities |
| BACnet | ASHRAE 135-2020 / ISO 16484-5 | Building-management integration at facilities |
| HTTP REST | RFC 9110 / RFC 9112 / RFC 9113 | Cloud platform APIs |
| WebSocket | RFC 6455 | Real-time dispatcher dashboards |

Sensor telemetry semantics follow the W3C SOSA/SSN ontology so that any sensor's observations are described in a uniform vocabulary.

### Database Schema (Extended)

```sql
CREATE TABLE transports (
  transport_id VARCHAR(50) PRIMARY KEY,
  manifest_id VARCHAR(50) NOT NULL REFERENCES manifests(manifest_id),
  status VARCHAR(20) NOT NULL CHECK (status IN ('SCHEDULED','PICKUP','IN_TRANSIT','DELIVERED','RETURNED','EMERGENCY')),
  origin_facility_id VARCHAR(50) REFERENCES facilities(facility_id),
  destination_facility_id VARCHAR(50) REFERENCES facilities(facility_id),
  scheduled_pickup TIMESTAMPTZ,
  scheduled_delivery TIMESTAMPTZ,
  actual_pickup TIMESTAMPTZ,
  actual_delivery TIMESTAMPTZ,
  created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT now()
);

CREATE TABLE telemetry (
  telemetry_id BIGSERIAL PRIMARY KEY,
  transport_id VARCHAR(50) NOT NULL REFERENCES transports(transport_id),
  sensor_id VARCHAR(50) NOT NULL,
  ts TIMESTAMPTZ NOT NULL,
  metric VARCHAR(40) NOT NULL,
  value DOUBLE PRECISION NOT NULL,
  uncertainty DOUBLE PRECISION,
  time_source VARCHAR(20) CHECK (time_source IN ('ptp_v2','ntp_v4','none'))
);
CREATE INDEX idx_telemetry_transport_ts ON telemetry (transport_id, ts);

CREATE TABLE custody_events (
  event_id VARCHAR(50) PRIMARY KEY,
  manifest_id VARCHAR(50) NOT NULL REFERENCES manifests(manifest_id),
  ts TIMESTAMPTZ NOT NULL,
  activity VARCHAR(40) NOT NULL CHECK (activity IN ('PICKUP','IN_TRANSIT_CHECK','HANDOFF','EMERGENCY_STOP','DELIVERY','RETURN')),
  prev_event_hash BYTEA,
  this_event_hash BYTEA NOT NULL UNIQUE,
  signed_payload JSONB NOT NULL
);
CREATE INDEX idx_custody_manifest ON custody_events (manifest_id, ts);
```

The database conforms to ACID semantics per ISO/IEC 9075 (SQL standard). Storage backups follow the 3-2-1 rule (three copies, two media, one off-site).

### Identity and Access Management

The reference IAM model has six roles, each backed by a distinct credential class:

| Role | Credential | Scope |
|------|------------|-------|
| Driver | Hazmat-endorsed CDL + WIA driver DID | Read manifest, append custody events |
| Dispatcher | WIA dispatcher DID + 2FA | Manage manifests, alerts, routes |
| Quality officer | WIA QO DID + 2FA | Override quality gates, certify post-mortems |
| Receiving officer | WIA RO DID + facility access badge | Append handoff events, certify receipt |
| Auditor | WIA auditor DID, read-only | Read manifests, custody, telemetry |
| Regulator | WIA regulator DID, read-only with audit log | Subject-class-scoped access |

Role assignments are governed by ISO/IEC 27001 Annex A.9 (Access control). Quarterly reviews are mandatory and documented in the operations runbook.

### Observability

Metrics, traces, and logs use OpenTelemetry semantics. The reference dashboards present:

- Active transports by status.
- Median and p95 sensor reporting latency.
- Active alerts by severity.
- Manifest completion rates.
- Driver hours-of-service compliance.

Alerting rules trigger on documented thresholds; alert delivery uses the webhook conformance described in Phase 2 API §7.

### Regulatory Reporting

Periodic regulatory submissions are supported for:

- **DOT 49 CFR §171.16** Hazardous-materials incident reports (US).
- **EU ADR §1.8.5** Reporting of occurrences during carriage.
- **IATA DGR §9.6** Discrepancies, accidents, incidents.
- **FDA 21 CFR §1271** Adverse-reaction reporting (when applicable to subject class).

Reports are generated from the underlying custody-event and telemetry records, signed under the operating organisation's regulatory key, and archived alongside the original records.

### Information Security

Information security follows ISO/IEC 27001:2022. Privacy follows ISO/IEC 27701:2019, GDPR (Regulation (EU) 2016/679), CCPA/CPRA, and PIPA (Korea Personal Information Protection Act). Cloud-deployed components additionally follow ISO/IEC 27017:2015 and ISO/IEC 27018:2019.

### Disaster Recovery

Recovery objectives align with ISO/IEC 27031:2011 ICT readiness for business continuity:

- **RTO** for telemetry ingestion: 5 minutes.
- **RTO** for manifest API: 15 minutes.
- **RPO** for custody events: 0 (synchronous replication).
- **RPO** for telemetry: 60 seconds.
- **Audit-log integrity** is reverified on every restore.

Quarterly disaster recovery drills are mandatory and reviewed by the quality officer.

### Conformance Statement

A reference cryogenic-transport integration is conformant with Phase 4 when:

1. The system architecture matches the four-tier model (or documents an equivalent).
2. Sensor integration uses one or more of the §Sensor Integration protocols.
3. The database schema preserves the constraints documented in §Database Schema.
4. IAM follows the six-role model or documents an equivalent role taxonomy.
5. Observability uses OpenTelemetry semantics.
6. Regulatory reporting endpoints exist for at least one of the §Regulatory Reporting frameworks.
7. Information security maps to a published ISO/IEC 27001 statement of applicability.
8. Disaster-recovery objectives are published and exercised at the documented cadence.

---

## Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Industrial sensor protocols | Modbus, OPC UA (IEC 62541), BACnet (ASHRAE 135 / ISO 16484-5) |
| IoT telemetry | OASIS MQTT 5.0, MQTT-SN |
| Sensor metadata | W3C SOSA/SSN |
| Database semantics | ISO/IEC 9075 (SQL) |
| Information security | ISO/IEC 27001:2022, ISO/IEC 27017:2015, ISO/IEC 27018:2019, ISO/IEC 27701:2019 |
| Business continuity | ISO/IEC 27031:2011 |
| Quality management for medical devices | ISO 13485:2016 |
| Cell and tissue transport | ISO 21973:2020 |
| Hazardous-materials regulations (US) | DOT 49 CFR Parts 100–185 |
| Hazardous-materials regulations (EU road) | ADR 2025 |
| Hazardous-materials regulations (air) | IATA DGR 2025 |
| Hazardous-materials regulations (sea) | IMDG Code |
| Privacy regulations | GDPR (EU), CCPA/CPRA (US), PIPA (KR) |

---

## Integration Appendix

### A. Reference Implementation Stack

The reference implementation uses:

- **Container runtime** — OCI-conformant runtimes (containerd / CRI-O) on Linux kernels supporting cgroup v2.
- **Orchestration** — Kubernetes 1.28+ with the operator pattern for stateful components.
- **Service mesh** — Optional. When deployed, follows the Istio or Linkerd reference for mTLS and observability.
- **Database** — PostgreSQL 15+ with logical replication for cross-region replicas.
- **Cache** — Redis 7+ in cluster mode.
- **Message bus** — Apache Kafka 3.5+ for high-volume telemetry; NATS JetStream is an acceptable alternative for lower-volume deployments.
- **Object storage** — S3-compatible storage with versioning and object-lock for the audit log.
- **Metrics** — Prometheus 2.x with PromQL queries; long-term storage via Mimir, Cortex, or VictoriaMetrics.
- **Tracing** — OpenTelemetry collectors with OTLP export.
- **Logs** — Structured logs in JSON with the W3C Trace Context fields embedded.

This stack is not normative; it is the reference against which interoperability is measured during the conformance test plan.

### B. Cross-Service Schemas

Inter-service messaging uses Apache Avro schemas (Apache Avro Specification 1.11+) registered in a schema registry. Schemas evolve under the standard Avro forward-, backward-, and full-compatibility rules.

For browser- and partner-facing payloads, JSON Schema draft 2020-12 is used and embedded in the OpenAPI 3.1 specification.

### C. Data Retention and Archival

Records are retained on a graduated schedule:

- **Operational data (active transport)** — High-resolution telemetry retained until 90 days post-delivery, then downsampled and archived.
- **Custody events** — Retained for the full regulatory retention window (typically 10 years for tissue products, longer when specific clinical protocols apply).
- **Audit log** — Retained per ISO/IEC 27001 Annex A.12.4 with append-only object-lock storage.
- **Training records** — Retained per DOT 49 CFR §172.704(d) (3 years from completion of training, plus the duration of employment).

Archival uses the WORM (write-once, read-many) profile of the chosen object store. Each archived block carries a SHA-256 fingerprint that is published to the audit log so tampering is detectable.

### D. Performance Targets

The reference deployment targets:

- **Telemetry ingestion** — 100,000 events/second sustained, 250,000 burst, p99 latency < 200 ms.
- **Manifest API** — 100 req/s, p99 latency < 500 ms.
- **Custody-event signing** — 50 events/s with HSM-backed Ed25519 signing.
- **Dashboard queries** — p99 latency < 1 s for the active-transports view.

These targets are exercised in the reference load test plan and revised as deployment scale evolves.

### E. Internationalisation

The driver's mobile app and dispatcher console are localised across the WIA core language set. The Korean, English, Japanese, Chinese (Simplified), Spanish, Portuguese, French, and German locales are mandatory; additional locales follow the WIA i18n governance process.

Locale-specific date/time formatting follows ISO 8601:2019 for machine-readable storage and the Unicode CLDR (Common Locale Data Repository) for display formatting. Number formatting follows the locale conventions; the underlying values remain canonical IEEE 754 binary64.

### F. Accessibility

Driver-facing and dispatcher-facing UIs conform to W3C WCAG 2.2 Level AA. Conformance is verified by automated checks (Axe Core) and manual review against the WCAG 2.2 success criteria. Conformance evidence is part of the reference deployment's compliance artefacts.

### G. Cybersecurity and Incident Response

Cybersecurity follows ISO/IEC 27001:2022 and the IEC 62443 series for industrial-control-system components (sensor controllers, cryogenic equipment with networked interfaces). Incident response uses the five-severity model defined in the WIA-AI-017 enterprise reference for consistency across WIA standards.

For security-relevant incidents involving regulated subject material, the operating organisation notifies the relevant regulator within the rule-defined window (e.g., 72 hours for GDPR personal-data breaches under Article 33; the rule-specific window for tissue-product incidents).

### H. Sustainability Reporting

Operating organisations may publish sustainability reports following ISO 14064-1:2018 (greenhouse-gas inventory) and ISO 50001:2018 (energy management). Telemetry from cryogenic transports contributes to the operational scope of the inventory; the reference implementation provides export hooks for ISO 14064-1 §8 quantification.

---

© 2025 WIA
