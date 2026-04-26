# WIA-IND-025 Quality Control — Phase 4: Integration

**Standard**: WIA-IND-025 (Quality Control)
**Phase**: 4 of 4 — Integration with Existing Systems
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA-IND-025 composes with the surrounding factory
stack and with neighbouring WIA standards:

1. **ERP / MES bridges** (SAP, Oracle, Microsoft Dynamics, Tulip,
   in-house MES).
2. **Measurement equipment** (CMM, vision systems, torque wrenches,
   data loggers) over OPC-UA or vendor protocols.
3. **External certification bodies** (DNV, BSI, TÜV, UL) and their
   audit portals.
4. **WIA family standards** (WIA-OMNI-API for credentials,
   WIA-AIR-SHIELD for transport hardening, WIA-INTENT for operator
   workflow lowering, WIA-ACCESSIBILITY for inspector ergonomics).

The aim is that one WIA-IND-025 service is the system of record while
every other system reads from or writes to it through bridges, never
holding canonical state for inspection / NCR / CAPA / audit.

---

## 2. ERP / MES Bridges

### 2.1 Bridge Architecture

A bridge translates ERP/MES native objects to WIA-IND-025 objects under
the site's host signature. Bridges hold no inspector or quality-manager
signing keys; signatures stay with the people doing the work.

```
       ┌────────────────────┐
       │  ERP / MES         │
       └────────┬───────────┘
                │ vendor API
                ▼
       ┌────────────────────┐
       │  WIA-IND-025 Bridge│
       └────────┬───────────┘
                │ Phase 2 API
                ▼
       ┌────────────────────┐
       │  WIA-IND-025 Site  │
       └────────────────────┘
```

### 2.2 Per-System Mapping

| ERP/MES object | WIA-IND-025 object |
|----------------|---------------------|
| Production order | (not modelled — bridge correlates lot_id) |
| Quality plan / inspection plan | inspection_plan |
| Inspection lot | inspection_result |
| Defect record | defect_record |
| Calibration master | calibration_record |
| Quality notification / non-conformance | ncr |
| 8D / corrective action | capa |
| Internal audit | audit_finding |

Bridges MUST emit a `delivery_receipt` for every ERP/MES event consumed
to keep the canonical record consistent.

---

## 3. Measurement Equipment Integration

### 3.1 OPC-UA Adapter

CMMs, vision systems and data loggers commonly expose data over OPC-UA.
The adapter:

* Subscribes to the device's measurement node.
* Translates each measurement into either a
  `inspection_result.observations[]` entry or an `spc_sample` envelope.
* Signs the envelope with the assigned inspector's key (the inspector
  badges in to authorise the device).

### 3.2 Vendor-Specific Adapters

For devices without OPC-UA, vendor-specific adapters live under
`spec/profiles/<vendor>.md`. Each profile documents the field-by-field
mapping, the polling cadence, and the credential management approach.

Adapters MUST timestamp every measurement at the moment of capture, not
the moment of upload, so SPC charts reflect actual process history when
network connectivity is intermittent.

---

## 4. External Certification Bodies

### 4.1 Auditor Portal Integration

Certification bodies traditionally operate their own audit portals. A
bridge syncs:

* Outbound: site's `audit_finding` envelopes → portal entries.
* Inbound: portal closure decisions → site's `audit_finding` updates.

The bridge MUST verify the portal's host signature and MUST NOT update
findings without a verified portal signature.

### 4.2 Certificate Lookup

External parties verifying a site's certification (e.g., a customer
checking ISO 9001 status) call:

```
GET https://qc.example/qc/certifications
Authorization: WIA-Sig <customer key>
```

Returns a list of active third-party certifications with issuer,
scope, valid_until, and a trust-anchor pointer to the issuer's
verifiable public key.

---

## 5. WIA Family Integration

### 5.1 WIA-OMNI-API

Inspector and auditor credentials (training records, qualification
attestations, identity proofs) are stored in WIA-OMNI-API. Sites fetch
them by DID rather than holding raw documents.

### 5.2 WIA-ACCESSIBILITY

Inspector accommodations profiles drive UI choices: large-text mode,
high-contrast inspection forms, voice input for hands-free measurement
capture. Sites MUST offer the inspector's preferred accommodation when
issuing the device-side UI.

### 5.3 WIA-AIR-SHIELD

Transport hardening (TLS configuration, peer reputation). Sites MAY
refuse handshakes from peers whose AIR-SHIELD score is below a
site-set threshold.

### 5.4 WIA-INTENT

A quality manager's "investigate the threading defect on lot
L-2026-04-A" intent is lowered to:

1. Fetch all `inspection_result` envelopes for the lot.
2. Fetch all `defect_record` envelopes for the lot.
3. Fetch the active `inspection_plan` for the part.
4. Open a draft `ncr` referencing the evidence above.
5. Suggest containment actions from a learned playbook.

The lowering keeps the manager focused on the decision rather than the
data plumbing.

### 5.5 WIA-SOCIAL

Sites that publish quality milestones (CAPA closure, certification
renewal) for stakeholder communication can route them through
WIA-SOCIAL bridges so a single internal publish fans out to internal
networks with appropriate `audience` controls.

---

## 6. Compliance Mappings

### 6.1 ISO 9001:2015

| Standard requirement | WIA-IND-025 element |
|----------------------|----------------------|
| §7.1.5 Monitoring and measuring resources | calibration_record |
| §8.5.1 Control of production | inspection_plan, inspection_result |
| §8.7 Control of nonconforming outputs | ncr |
| §10.2 Nonconformity and corrective action | capa |
| §9.2 Internal audit | audit_finding |

### 6.2 ISO 13485:2016

Adds:

* §7.5.4 Particular requirements for terminal sterilization → SPC
  record retention extends to 15 years.
* §8.2.6 Monitoring and measurement of product → every batch MUST have
  at least one inspection_result; the bridge enforces this by refusing
  shipments without one.

### 6.3 IATF 16949

Adds PPAP (Production Part Approval Process) flow, which reuses
`certificate_of_conformance` with extra metadata under `automotive`
namespace. Profile document at `spec/profiles/iatf.md`.

### 6.4 AS9100

Adds first-article inspection (FAI) flow using inspection_result with
`form` set to `first_article` and a fixed checkpoint set per AS9102.

---

## 7. Migration Paths

### 7.1 From a Spreadsheet-Based Quality System

Many small sites still run quality control on spreadsheets. The
migration:

1. Deploy a WIA-IND-025 reference container.
2. Run `qc-import --csv inspections.csv --map spreadsheet-v0.yaml`. The
   importer assigns each row a fresh ULID and records the source CSV
   row id in a `provenance` field for audit.
3. Backfill calibration records from the equipment spreadsheet.
4. Switch the day-to-day inspection capture to the WIA-IND-025 UI; keep
   the spreadsheet as a read-only archive.
5. Run a 14-day shadow period where both systems receive new data and
   the operator compares daily. After three consecutive zero-discrepancy
   days, retire the spreadsheets.

### 7.2 From a Vendor QMS

Vendor QMS migrations follow a per-vendor profile (Tulip, Honeywell
TPS, Siemens Opcenter) documented under `spec/profiles/<vendor>.md`.
Each profile includes the field-mapping, the credential cutover plan,
and the rollback procedure if the migration is paused.

---

## 8. Observability

Bridges and adapters SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_qc_inspections_total{verdict}` | counter | Per-verdict inspection counts |
| `wia_qc_spc_samples_total{chart_id}` | counter | SPC samples per chart |
| `wia_qc_ncr_total{severity,state}` | counter | NCR pipeline state |
| `wia_qc_capa_effectiveness_total{result}` | counter | CAPA outcomes |
| `wia_qc_calibration_overdue` | gauge | Equipment items overdue today |
| `wia_qc_audit_findings_total{severity,state}` | counter | Audit pipeline state |

Labels MUST NOT include personal identifiers; chart_id is acceptable
because it identifies a process characteristic, not a person.

---

## 9. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | At least one ERP/MES bridge, OPC-UA measurement adapter |
| **Core**    | Plus WIA-OMNI-API credentials, WIA-ACCESSIBILITY enforcement, certification body sync |
| **Full**    | Plus WIA-AIR-SHIELD scoring, WIA-INTENT lowering, vendor profile for at least one major QMS |

Sites publish their level in `bridge_profile` of the discovery document.

---

## 10. Worked Example — Threading Defect Investigation

```
Site       : did:wia:site:bonghwa-line-A
Lot        : lot_2026-04-A
Inspector  : did:wia:inspector:09…
QC manager : did:wia:qm:21…
Customer   : did:wia:customer:42…
```

1. Inspector signs `inspection_result(verdict=fail)` for cp-007.
2. Site auto-creates `ncr_01HZA` (severity=major).
3. WIA-INTENT lowers QC manager's "investigate" intent into a packet of
   evidence (results, defects, plan).
4. QC manager sets disposition=rework, signs the patch.
5. Site auto-spawns `capa_01HZA` referencing the NCR.
6. Engineering team records corrective and preventive actions.
7. Effectiveness check at T+30d records `effective`; CAPA closes.
8. Outgoing CoC for lot_2026-04-A is updated with the rework reference.
9. Customer receives the CoC and verifies via `qc/certifications`.
10. Annual audit at T+90d references the closed NCR/CAPA chain as
    evidence of an effective management system.

---

## 11. Security Considerations

* Bridges hold ERP/MES credentials; operators MUST use WIA-OMNI-API for
  credential storage and HSM-backed refresh tokens. A compromised bridge
  could publish forged inspections under the site's host signature, so
  bridges MUST sign every outbound envelope with a separate "bridge"
  delegation key whose scopes are restricted to inspect / spc_sample.
* Measurement equipment adapters often run on factory-floor PCs with
  weaker hardening; operators MUST treat captured measurements as
  attested only when the inspector's badge-in is fresh (≤ shift duration).
* Certificate lookup endpoints are publicly readable by design; sites
  MUST rate-limit anonymous access to prevent inventory scraping.

---

## 12. References

* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* IETF RFC 6749 — OAuth 2.0
* OPC Foundation OPC-UA
* ISO 9001:2015, ISO 13485:2016
* IATF 16949, AS9100
* AS9102 — First Article Inspection
* WIA-OMNI-API standard
* WIA-AIR-SHIELD standard
* WIA-INTENT standard
* WIA-ACCESSIBILITY standard
* WIA-SOCIAL standard

---

## Appendix A — Bridge Configuration File

A reference SAP QM bridge configuration uses TOML:

```toml
[bridge]
erp = "sap-qm"
sap_version = "S/4HANA 2026"
site_id = "did:wia:site:bonghwa-line-A"

[credentials]
provider = "wia-omni-api"
omni_endpoint = "https://omni.example"

[delivery]
poll_interval_seconds = 60
backoff = "exponential"
initial_delay_ms = 1000
max_delay_ms = 60000

[mapping]
inspection_lot_to_inspection_result = "by-id"
quality_notification_to_ncr = "by-number"
calibration_master_to_calibration_record = "by-equipment-number"

[telemetry]
prometheus_port = 9091
```

Operators MAY embed vendor-specific sections; conformant bridges MUST
ignore unknown sections rather than refuse to start.

## Appendix B — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| ERP/MES bridge for ≥1 vendor | ✓ | ✓ | ✓ |
| OPC-UA measurement adapter | ✓ | ✓ | ✓ |
| WIA-OMNI-API credential fetch | — | ✓ | ✓ |
| WIA-ACCESSIBILITY enforcement | — | ✓ | ✓ |
| Certification body sync | — | ✓ | ✓ |
| WIA-AIR-SHIELD scoring | — | — | ✓ |
| WIA-INTENT lowering | — | — | ✓ |
| Vendor profile for at least one major QMS | — | — | ✓ |
| Migration tooling for spreadsheets | optional | optional | ✓ |
| Trust anchor publication | ✓ | ✓ | ✓ |

A site publishing `bridge_profile=Full` MUST pass every line in the Full
column and SHOULD pass at least one optional line.

弘益人間 — Benefit All Humanity.
