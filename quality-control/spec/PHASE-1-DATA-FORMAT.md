# WIA-IND-025 Quality Control — Phase 1: Data Format

**Standard**: WIA-IND-025 (Quality Control)
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — protect consumers and lift manufacturing excellence everywhere.

---

## 1. Scope

Phase 1 fixes the JSON object families used across the Quality Control
domain so that two manufacturing sites running different ERP/MES systems
can exchange inspection plans, SPC samples, defect classifications,
calibration records, non-conformance reports (NCR), corrective and
preventive actions (CAPA) and audit findings without retyping.

| Family | Purpose |
|--------|---------|
| **Inspection Plan** | Sampling rule + checkpoint list bound to a part number |
| **Inspection Result** | Pass/fail per checkpoint, captured during a production run |
| **SPC Sample** | One subgroup sample with descriptive stats and chart cursor |
| **Defect Record** | Classified defect with severity, frequency and root-cause pointer |
| **Calibration Record** | Equipment id + calibration history + due date |
| **NCR** | Non-conformance report tying together one or more inspections |
| **CAPA** | Corrective / preventive action plan with effectiveness verification |
| **Audit Finding** | Internal or external audit observation with disposition |

Out of scope: HTTP surface (Phase 2), federation across sites (Phase 3),
ERP/MES bridges (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Timestamps RFC 3339 in UTC, `Z` suffix.
* Identifiers URI-shaped per IETF RFC 3986. Sites SHOULD use
  `did:wia:site:<slug>` for site identifiers and ULIDs for record ids.
* Measurements: SI base units (metres, kilograms, seconds, kelvin,
  amperes, candela, mole) with up to four decimal places. Derived units
  (Newton, Pascal, Hertz) are encoded as their SI form.
* Numerical fields fit in signed 64-bit integers; floats use IEEE 754
  double precision; rationals are not represented.
* Currency in records uses ISO 4217 codes; conversion is out of scope.

### 2.1 Versioning

```json
"wia_quality_control_version": "1.0.0"
```

A receiver MUST refuse a major version it does not implement and MUST
accept additive minor-version fields as non-fatal unknown extensions.

---

## 3. Inspection Plan

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "inspection_plan",
  "plan_id": "plan_01HZA…",
  "site_id": "did:wia:site:bonghwa-line-A",
  "part_id": "PART-A-014",
  "issued_at": "2026-04-01T00:00:00Z",
  "valid_until": "2027-04-01T00:00:00Z",
  "sampling": {
    "rule": "AQL 1.0 / level II",
    "lot_size_min": 1,
    "lot_size_max": null
  },
  "checkpoints": [
    {
      "checkpoint_id": "cp-001",
      "description": "Outer dimension within tolerance",
      "method": "calliper",
      "tolerance_kind": "bilateral",
      "nominal": 10.000,
      "tol_minus": -0.05,
      "tol_plus":  0.05,
      "unit": "mm"
    }
  ],
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

`sampling.rule` references published sampling standards (ISO 2859-1,
ISO 3951, MIL-STD 1916, etc.). Implementations MUST treat the string as
opaque; lookup is the operator's responsibility.

---

## 4. Inspection Result

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "inspection_result",
  "result_id": "res_01HZA…",
  "plan_id": "plan_01HZA…",
  "lot_id": "lot_2026-04-A",
  "started_at": "2026-04-01T09:30:00Z",
  "completed_at": "2026-04-01T10:05:00Z",
  "inspector_id": "did:wia:inspector:09…",
  "observations": [
    { "checkpoint_id": "cp-001", "value": 10.012, "unit": "mm", "verdict": "pass" },
    { "checkpoint_id": "cp-002", "value": 1.4,    "unit": "Ra",  "verdict": "pass" }
  ],
  "verdict": "pass",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

The overall `verdict` MUST equal `pass` only when every observation is
`pass`. A single `fail` observation requires the overall verdict to be
`fail` and triggers the NCR auto-creation rule (Phase 3 §6).

---

## 5. SPC Sample

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "spc_sample",
  "sample_id": "spc_01HZA…",
  "chart_id": "chart_A-014_outer-dim",
  "captured_at": "2026-04-01T10:05:00Z",
  "subgroup_n": 5,
  "values": [10.011, 10.013, 10.012, 10.010, 10.014],
  "stats": {
    "mean": 10.0120,
    "stdev": 0.00150,
    "range": 0.004
  },
  "control_limits": {
    "ucl_x": 10.020,
    "lcl_x":  9.980,
    "ucl_r":  0.020,
    "lcl_r":  0.000
  },
  "out_of_control_rules": [],
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

`out_of_control_rules` is a list of rule identifiers (`WE-1`, `WE-2`, …
for Western Electric rules; `N-1`, `N-2`, … for Nelson rules). An empty
list means the subgroup is in control.

---

## 6. Defect Record

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "defect_record",
  "defect_id": "def_01HZA…",
  "lot_id": "lot_2026-04-A",
  "captured_at": "2026-04-01T11:15:00Z",
  "category": "scratch",
  "severity": "minor",
  "frequency_ppm": 3000,
  "root_cause_id": "rc_tooling_wear_22",
  "image_urls": ["https://media.example/defects/def_01HZA-1.jpg"],
  "signature": { "alg": "Ed25519", "value": "Yz1p…" }
}
```

Severity ladder: `cosmetic`, `minor`, `major`, `critical`. Severity
maps to the workflow rules in Phase 3 §6.

---

## 7. Calibration Record

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "calibration_record",
  "equipment_id": "did:wia:equipment:cmm-007",
  "calibrated_at": "2026-01-15T14:00:00Z",
  "next_due_at":   "2026-07-15T14:00:00Z",
  "interval_months": 6,
  "calibrated_by": "did:wia:lab:trace-cal",
  "traceability": "NIST",
  "as_found_uncertainty_um": 1.2,
  "as_left_uncertainty_um": 0.8,
  "result": "in_tolerance",
  "signature": { "alg": "Ed25519", "value": "Mx7q…" }
}
```

`traceability` MUST reference the national metrology institute or
accreditation body that provides the chain of traceability.

---

## 8. Non-Conformance Report (NCR)

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "ncr",
  "ncr_id": "ncr_01HZA…",
  "opened_at": "2026-04-01T12:00:00Z",
  "detected_at": "in-process",
  "severity": "major",
  "description": "Threading depth out of spec on lot L-2026-04-A",
  "evidence_inspection_ids": ["res_01HZA…"],
  "evidence_defect_ids": ["def_01HZA…"],
  "containment_action": "Quarantine lot L-2026-04-A; pause line A.",
  "disposition": null,
  "closed_at": null,
  "capa_required": true,
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

Disposition is one of `use_as_is`, `rework`, `regrade`, `scrap`,
`return_to_supplier`. A null disposition indicates the NCR is open.

---

## 9. CAPA

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "capa",
  "capa_id": "capa_01HZA…",
  "opened_at": "2026-04-02T09:00:00Z",
  "for_ncr_ids": ["ncr_01HZA…"],
  "root_cause": "Cutting tool exceeded specified life by 12% before scheduled replacement.",
  "corrective_actions": [
    { "action": "Replace tool t-441 immediately.", "owner": "ops-mgr", "due": "2026-04-02" }
  ],
  "preventive_actions": [
    { "action": "Reduce tool life parameter from 5 000 to 4 500 cycles.", "owner": "engineering", "due": "2026-04-09" }
  ],
  "effectiveness_check_at": "2026-05-09",
  "effectiveness_result": null,
  "signature": { "alg": "Ed25519", "value": "Ku8x…" }
}
```

`effectiveness_result` is `effective`, `partial`, or `ineffective`. An
ineffective verification re-opens the CAPA chain with a fresh CAPA
referencing the prior via `supersedes`.

---

## 10. Audit Finding

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "audit_finding",
  "finding_id": "find_01HZA…",
  "audit_id": "audit_2026Q2",
  "auditor_id": "did:wia:auditor:09…",
  "scope": "Calibration management",
  "clause_reference": "ISO 9001:2015 §7.1.5",
  "severity": "major",
  "observation": "Three of fifteen sampled equipment items had no traceable calibration record within the last 6 months.",
  "evidence_calibration_ids": ["cal_01HZA…"],
  "due_date": "2026-05-15",
  "status": "open",
  "signature": { "alg": "Ed25519", "value": "Pa1y…" }
}
```

Audit findings tie back to clause references in the chosen QMS standard
(ISO 9001, ISO 13485, IATF 16949, AS9100, etc.).

---

## 11. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/quality-control/schemas/`. Implementations
SHOULD bundle local copies for offline validation.

---

## 12. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields per the JSON Schemas.
3. Treat unknown optional fields as non-fatal.
4. Compute the overall inspection verdict from observations per §4.
5. Auto-flag SPC samples whose stats violate any of the listed rules.

---

## 13. References

* IETF RFC 8259 — JSON
* IETF RFC 3339 — Date/Time
* IETF RFC 3986 — URI Generic Syntax
* IETF RFC 4648 — base64
* ISO 9001:2015 — QMS
* ISO 13485:2016 — Medical devices QMS
* ISO 2859-1 — Sampling procedures by attributes
* ISO 3951 — Sampling procedures by variables
* IATF 16949 — Automotive QMS
* AS9100 — Aerospace QMS
* JSON Schema Draft 2020-12

---

## Appendix A — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `severity` (defect/NCR/audit) | `cosmetic`, `minor`, `major`, `critical` |
| `verdict` | `pass`, `fail` |
| `disposition` | `use_as_is`, `rework`, `regrade`, `scrap`, `return_to_supplier` |
| `effectiveness_result` | `effective`, `partial`, `ineffective` |
| `result` (calibration) | `in_tolerance`, `out_of_tolerance` |
| `detected_at` (NCR) | `incoming`, `in-process`, `final`, `field` |
| `tolerance_kind` | `bilateral`, `unilateral_upper`, `unilateral_lower`, `attribute` |

Future minor versions add tokens but never remove them.

## Appendix B — Worked Calibration Record (out-of-tolerance escalation)

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "calibration_record",
  "equipment_id": "did:wia:equipment:torque-007",
  "calibrated_at": "2026-04-15T08:30:00Z",
  "next_due_at":   "2026-10-15T08:30:00Z",
  "interval_months": 6,
  "calibrated_by": "did:wia:lab:trace-cal",
  "traceability": "NIST",
  "as_found_uncertainty_um": 18.4,
  "as_left_uncertainty_um": 0.9,
  "result": "out_of_tolerance",
  "downstream_impact_assessment": {
    "since_last_calibration": "2025-10-15T08:30:00Z",
    "products_potentially_affected": ["lot_2026-01-A","lot_2026-02-A","lot_2026-03-A"],
    "ncr_id_opened": "ncr_01HZB…"
  },
  "signature": { "alg": "Ed25519", "value": "Mx7q…" }
}
```

When `result = out_of_tolerance` the record MUST include
`downstream_impact_assessment`; the impact assessment MUST list every
lot inspected since the prior in-tolerance calibration and MUST open a
matching NCR for traceability.

## Appendix C — Worked NCR with Containment & Disposition

```json
{
  "wia_quality_control_version": "1.0.0",
  "type": "ncr",
  "ncr_id": "ncr_01HZA…",
  "opened_at": "2026-04-01T12:00:00Z",
  "detected_at": "in-process",
  "severity": "major",
  "description": "Threading depth out of spec on lot L-2026-04-A",
  "evidence_inspection_ids": ["res_01HZA…"],
  "evidence_defect_ids": ["def_01HZA…"],
  "containment_action": "Quarantine lot L-2026-04-A; pause line A.",
  "disposition": "rework",
  "disposition_signed_by": "did:wia:qm:21…",
  "disposition_signed_at": "2026-04-02T09:00:00Z",
  "rework_summary": "Re-thread to nominal; re-inspect all 1 200 units in lot.",
  "closed_at": "2026-04-04T17:00:00Z",
  "capa_required": true,
  "capa_id": "capa_01HZA…",
  "history": [
    { "event": "opened",            "at": "2026-04-01T12:00:00Z", "by": "did:wia:inspector:09…" },
    { "event": "containment_added", "at": "2026-04-01T12:30:00Z", "by": "did:wia:supervisor:11…" },
    { "event": "disposition_set",   "at": "2026-04-02T09:00:00Z", "by": "did:wia:qm:21…" },
    { "event": "closed",            "at": "2026-04-04T17:00:00Z", "by": "did:wia:qm:21…" }
  ],
  "signature": { "alg": "Ed25519", "value": "Vr3w…" }
}
```

弘益人間 — Benefit All Humanity.
