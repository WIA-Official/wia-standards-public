# WIA-nbc-defense PHASE 1 — Data Format Specification

**Standard:** WIA-nbc-defense
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for nuclear-biological-
chemical (NBC) defence detection records: agent identification, sensor
event payloads, contamination plume modelling inputs, decontamination
work-orders, casualty triage records, and the cross-references that
bind these to the operational picture. The shape interoperates with
allied-nation NBC reporting standards so that a multi-national
deployment does not require parallel data models.

References (CITATION-POLICY ALLOW only):
- STANAG 2103 / AArtyP-1 — NBC reports (NBC-1 through NBC-6) for Allied land forces
- STANAG 4632 / AEP-66 — Deployable Nuclear, Biological and Chemical detection systems
- ATP-3.8.1 / AJP-3.8 — Allied Joint Doctrine for Comprehensive CBRN Defence
- ISO/IEC 19762:2008 — automatic identification and data capture vocabulary
- ISO 17025:2017 — testing and calibration laboratory competence
- WHO IHR (2005) — International Health Regulations (notification of public-health events)
- OPCW Verification Annex — Chemical Weapons Convention verification regime
- HL7 FHIR R5 — Observation, DiagnosticReport, Patient (for medical CBRN integration)
- IETF RFC 7515 (JWS), RFC 9162 (Certificate Transparency 2.0 pattern)

---

## §1 Scope

This PHASE applies to systems that detect, classify, track, and
respond to NBC threats: hand-held and stationary chemical detectors,
biological identifiers, radiation portal monitors, dosimetry networks,
plume-modelling pipelines, and the medical-treatment records arising
from exposure or suspected exposure. It defines the *shape* of the
records; the protocols for transport and access are addressed in
PHASE 3, integration with command-and-control in PHASE 4.

The standard is allied-doctrine-aware: a single implementation MUST
declare which national or coalition doctrine its records originate
under so that downstream consumers can apply the correct interpretation
of categorisation, release authority, and public-health notification.

In scope: chemical (CWA, TIC), biological (Category A/B/C agents),
radiological (sealed and unsealed sources, dispersal events), and
nuclear (criticality, fallout). Out of scope: directed-energy and
electromagnetic threats addressed by separate WIA standards.

## §2 Agent identifier model

Agents are identified by structured codes drawn from existing
inventories:

| Domain | Identifier source | Example |
|--------|-------------------|---------|
| Chemical | OPCW Schedule 1/2/3 + CAS number | Sarin (GB) — Schedule 1, CAS 107-44-8 |
| Biological | ICTV taxonomy + CDC category | *Bacillus anthracis* (Category A) |
| Radiological | IAEA radionuclide table | Cs-137, Co-60, Am-241 |
| Nuclear | IAEA fission-product inventory | I-131, Cs-137, Sr-90 from criticality |
| Toxic Industrial Chemical | UN Recommendations on the Transport of Dangerous Goods (UN number) | UN 1017 (Chlorine), UN 1005 (Anhydrous Ammonia) |

An agent record is a stable triple `(domain, source-vocabulary, code)`
plus a free-text `localName` for human display. Implementations MUST
NOT invent agent codes; an unrecognised agent surfaces as
`(domain, "unknown", "<sensor-raw>")` and is queued for analyst
review rather than auto-classified.

## §3 Sensor event record

A detection emits a sensor event record with the fields:

- `eventId` — URN of form `urn:wia:nbc:event:<sensor>:<timestamp>:<seq>`
- `sensorRef` — URN of the emitting sensor (PHASE 4 §3 fleet registry)
- `agentRef` — agent identifier per §2 (or `unknown`)
- `confidence` — declared band (`presumptive`, `confirmed`, `validated`)
- `timestamp` — RFC 3339 with offset; sensor-clock plus authoritative
  time-source skew (PHASE 3 §6)
- `position` — WGS-84 lat/lon/alt with horizontal/vertical uncertainty;
  for fixed sensors a static survey position is permitted
- `concentration` — value + unit + measurement uncertainty (per
  ISO 17025 reporting style); permitted units are restricted to
  the per-domain inventory below
- `metadata` — sensor model, firmware, calibration date, last
  field-check date

Permitted units (closed):

| Domain | Unit |
|--------|------|
| Chemical (vapour) | mg/m³, ppm |
| Chemical (liquid) | mg/cm² |
| Biological | particles/L of air, CFU/mL |
| Radiological (rate) | µSv/h, mSv/h, Sv/h |
| Radiological (activity) | Bq, kBq, MBq, GBq |
| Surface contamination | Bq/cm² |

Concentrations outside the unit list are rejected at ingest. If a
sensor emits a non-listed unit, integration is responsible for
conversion to a listed unit before submitting the record (PHASE 4 §2).

## §4 Confidence bands

Detection confidence follows allied doctrine:

- `presumptive` — single-technology hit, single sensor; suitable for
  alerting but not for releasing fire or commencing decontamination
- `confirmed` — multi-technology or multi-sensor hit; basis for
  immediate operational response within the doctrine
- `validated` — laboratory analysis from a sample submitted under
  ISO 17025-conformant chain-of-custody; basis for permanent record
  and external reporting

Promotion across bands requires evidence linkage: a `confirmed`
record references the contributing `presumptive` events; a
`validated` record references the sample chain-of-custody log
(§5) and the analysing laboratory's accreditation reference.

## §5 Sample chain of custody

When physical samples are taken (chemical wipe, biological swab,
radiological smear, environmental air-filter), the chain-of-custody
log captures:

- `sampleId` — URN with sealed-bag identifier
- `collector` — authenticated principal who took the sample
- `collectionPoint` — geo-coordinate + descriptive locality
- `collectionTime` — RFC 3339
- `transferEvents[]` — each entry: from-principal, to-principal,
  timestamp, integrity check (seal status, optional photograph hash)
- `analysisRequest` — destination laboratory, requested test panel,
  release authority

The log is hash-chained per PHASE 3 §4 so that any retrospective
edit is detectable. A sample without an unbroken custody chain
cannot promote a sensor event to `validated`.

## §6 Plume model input record

Forecasts of contamination spread require structured inputs:

- Source term — agent ID, release rate (kg/s for chemical;
  Bq/s for radiological; CFU/s for biological), release elevation,
  source geometry (point, line, area)
- Meteorology — wind vector at multiple altitudes, atmospheric
  stability class (Pasquill A–F), mixing height, surface roughness
- Terrain — digital elevation model reference, land-use grid
- Time horizon — start, duration, time-step

The record references the model implementation used (HPAC, ALOHA,
RIMPUFF, etc.) and the input-data lineage so that two different
model runs against the same inputs are reproducible.

## §7 Decontamination work-order

A decontamination action is recorded as a work-order with:

- `triggerEventId` — the sensor event(s) or sample finding that
  triggered the action
- `targetArea` — polygon of WGS-84 coordinates
- `methodCodes[]` — coded methods (water-with-detergent,
  bleach-solution-1.5%, hot-air, mechanical-removal,
  catalysed-decomposition)
- `personnel[]` — authenticated principals on the work-order with
  protective-posture (MOPP level for ground forces, equivalent for
  emergency services)
- `commencementTime`, `completionTime`
- `verificationEvent` — sensor event after completion confirming
  residual below the action threshold

Records are signed with the issuing organisation's signing key
(PHASE 3 §3) so that the work-order's integrity survives across
multi-organisation incident responses.

## §8 Casualty triage record

When personnel are exposed or suspected exposed, a triage record
binds the NBC event to a clinical record:

- `subjectRef` — pseudonymous subject identifier (interop with
  WIA-medical-data-privacy PHASE 1 §2 when the subject is a
  civilian; military deployments use the relevant force-protection
  identifier per the deployment policy)
- `exposureEventId` — sensor event(s) implicating the exposure
- `triageCategory` — coded triage code (T1 immediate, T2 delayed,
  T3 minimal, T4 expectant, deceased)
- `decontaminationStatus` — pre-decon, decon-complete, not-required
- `clinicalFindings` — FHIR Observation references when integrated
  with a medical record system
- `releaseAuthority` — for record release outside the operational
  chain (e.g., to a national poisons centre or epidemiology
  authority)

Triage records are read-only post-issue; subsequent clinical
updates create new linked records rather than overwriting.

## §9 Allied report shapes (NBC-1 through NBC-6)

STANAG 2103 defines six standard NBC-report shapes. This PHASE
preserves them as derived projections of the records above:

| Report | Trigger | Source records |
|--------|---------|----------------|
| NBC-1  | Initial detection | `sensor event` with `presumptive` band |
| NBC-2  | Updated information | refinements after multi-sensor fusion |
| NBC-3  | Plume forecast | `plume model input` + run |
| NBC-4  | Reconnaissance results | promoted `confirmed` events |
| NBC-5  | Contaminated area | aggregated polygon over `validated` events |
| NBC-6  | Detailed information | full chain-of-custody + lab analysis |

A deployment that needs to emit STANAG-conformant reports renders
them on demand from the underlying records; the canonical record
remains in the WIA shape, which is more general and lossless.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked example: NBC-1 record (informative)

The following is a fully populated sensor event record at
`presumptive` confidence, ready for derivation into a STANAG 2103
NBC-1 report:

```json
{
  "eventId": "urn:wia:nbc:event:cpd-91a7:2026-04-27T09:31:14+09:00:0001",
  "sensorRef": "urn:wia:nbc:sensor:fielded:cpd-91a7",
  "agentRef": {"domain": "chemical", "vocab": "OPCW", "code": "GB", "localName": "Sarin"},
  "confidence": "presumptive",
  "timestamp": "2026-04-27T09:31:14+09:00",
  "position": {"lat": 37.4516, "lon": 126.6531, "alt": 12, "h_uncert_m": 2.5, "v_uncert_m": 1.0},
  "concentration": {"value": 0.18, "unit": "mg/m3", "uncert": 0.03},
  "metadata": {
    "model": "vendor-X.IMS-portable",
    "firmware": "fw-3.4.2",
    "calibration": {"date": "2026-03-15", "lab": "urn:wia:org:lab:nat-cal-A"},
    "fieldCheckDate": "2026-04-26"
  },
  "doctrine": "NATO-AJP-3.8"
}
```

## Annex B — Conformance disclosure

Implementations declare per-section conformance in their published
capability document. Sections marked `partial` reference the
deployment policy explaining the gap; sections marked `excluded`
carry a justification citing the controlling doctrine's allowance.
A deployment that is `partial` or `excluded` on §3 (Sensor event
record), §4 (Confidence bands), §5 (Sample chain-of-custody), or
§9 (Allied report shapes) is non-conformant overall.

## Annex C — Confidence promotion worked example (informative)

A typical chemical-agent confirmation sequence:

1. **Presumptive** — handheld IMS detector at outpost reports
   `(chemical, OPCW, GB)` at 0.18 mg/m³, h_uncert 2.5 m, evening 27 April. Single technology, single sensor — bandwidth `presumptive`.
2. **Cross-cuing** — fixed FPA detector 200 m downwind reports
   `(chemical, OPCW, GB)` at 0.04 mg/m³ five minutes later.
3. **Promotion to confirmed** — operator submits `$promote-to-confirmed`
   referencing both presumptive event IDs, technology codes
   `["IMS", "FPA"]`, operator URN. Boundary creates a `confirmed`
   record referencing both contributors and emits an AuditEvent.
4. **Sample take** — CBRN team takes a swab on a downwind surface
   under sealed-bag custody (PHASE 1 §5).
5. **Lab analysis** — sample submitted to the deployment's
   designated OPCW-accredited laboratory; analysis confirms GB
   identity at quantitative concentration.
6. **Promotion to validated** — operator submits `$promote-to-validated`
   with the lab report URI and the chain-of-custody log reference.
   Boundary creates a `validated` record citing both confirmed event
   and lab evidence; the `validated` record is the basis for STANAG
   2103 NBC-4 reporting and OPCW notification.

Each step in the sequence emits an AuditEvent so that the
promotion path is reconstructable post-incident.

## Annex D — Decontamination method codes (informative)

The closed list of method codes used in PHASE 1 §7:

| Code                          | Description                                                |
|-------------------------------|------------------------------------------------------------|
| `water-with-detergent`        | mechanical removal with surfactant solution                |
| `bleach-solution-1.5%`        | sodium hypochlorite solution at 1.5% available chlorine    |
| `hot-air`                     | thermal degradation by heated airflow                      |
| `mechanical-removal`          | physical scraping or mopping without chemical action       |
| `catalysed-decomposition`     | chemical neutralisation using a deployed catalyst          |
| `weathering`                  | ambient-condition decay (no active intervention)           |
| `ventilation`                 | enclosed-space air exchange                                 |
| `surface-coating`             | sequestration coating applied over contaminant              |
| `controlled-burn`             | thermal destruction of porous contaminated material         |

The list is appendable in future minor versions; existing codes
are stable across versions.
