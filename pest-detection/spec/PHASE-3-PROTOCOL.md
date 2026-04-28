# WIA-pest-detection PHASE 3 — Protocol Specification

**Standard:** WIA-pest-detection
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
data records and API surface into auditable lifecycles:
field-unit lifecycle, surveillance cadence (per ISPM 6),
trap-network operation, sample chain-of-custody,
diagnostic confirmation pathway (per ISPM 27 / EPPO PM
7), edge-AI inference governance, alert escalation,
intervention-decision IPM hierarchy, NPPO reporting
flow (per ISPM 17), and the audit-event chain. The
protocols are framed so an NPPO inspector or a supply-
chain auditor can reconstruct any pest event from the
event log.

References (CITATION-POLICY ALLOW only):
- IPPC ISPM 1, 5, 6, 8, 17, 23, 27, 31 — surveillance, reporting, diagnostic, sampling
- EPPO Diagnostic Standards (PM 7) and EPPO PRA (PM 5)
- FAO IPM Code of Conduct on Pesticide Management
- ISO 17025:2017 — testing-laboratory accreditation
- ISO 14971:2019 — risk-management lifecycle (where applied to advisory devices)
- ISO/IEC 27037 — digital evidence preservation
- WHO Pesticide Evaluation Scheme (WHOPES) — recommended classification
- Codex Alimentarius — MRL standards
- EPPO Code dictionary; Bayer Code crop dictionary
- IETF RFC 5424 (Syslog), RFC 8941, RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Field-unit lifecycle

```
registered → active → harvested → fallow / rotated
                  │
                  └→ retired (geometry change → new version)
```

Active field units accept observation, trap, sample,
imagery, and decision events. Harvested units retain
the event log; downstream lots inherit the pest-event
history through the agricultural-supply-chain binding.

## §2 Surveillance cadence (ISPM 6)

| Surveillance kind        | Cadence                                    |
|--------------------------|--------------------------------------------|
| General surveillance     | continuous (passive intake from scouts,    |
|                          | farmers, extension officers)               |
| Specific surveillance    | scheduled per pest, per crop phenology     |
| Delimiting survey        | triggered by suspect detection             |
| Detection survey         | NPPO-mandated (pre-export, area-freedom)   |
| Monitoring survey        | post-eradication / post-control            |

Specific surveillance schedules bind to BBCH phenology
codes (e.g. peach orchards: surveillance at petal-fall,
shuck-split, fruit-set; cereals: GS21, GS31, GS39, GS69,
GS83). The protocol records which BBCH stages were
surveilled and which were missed.

## §3 Trap-network operation

```
deployed → operating → serviced → retired
                │
                └→ replacement-pending (lure expiry / damage)
```

Trap service intervals are pest- and lure-specific
(e.g. *Lobesia botrana* pheromone trap: weekly during
flight, biweekly off-flight). The protocol enforces a
service-overdue gate: an alert with severity
`advisory` issues when service is overdue by 50 % of
the cadence; severity escalates to `action` at 100 %
overdue.

## §4 Sample chain of custody

```
collected → packaged → in-transit → received → processed → reported
                                          │
                                          └→ rejected (transport breach,
                                                       insufficient sample,
                                                       contamination)
```

Each transition emits an audit event with actor,
timestamp, container temperature, and a witness
signature (RFC 7515 JWS over the canonical event
payload). Rejected samples cannot bind to a diagnostic
result; a fresh sample is collected.

## §5 Diagnostic confirmation pathway

ISPM 27 protocols and EPPO PM 7 standards define the
authoritative pathway. The protocol records:

1. **screening** — first-line method (morphological,
   ELISA, real-time PCR with screening primers)
2. **confirmation** — orthogonal method (sequencing,
   second-target PCR, isolation + pathogenicity test)
3. **identity** — taxon assignment with EPPO Code,
   LSID, and where genomic, the INSDC accession

A diagnostic record is signed by the lab head and bears
the analyst credential. Diagnostic results that
confirm a quarantine pest trigger NPPO reporting
within the regulator-clock window (§10).

## §6 Edge-AI inference governance

Edge-AI models operating on field imagery follow:

- **dataset disclosure** — training-data composition,
  geographic representativeness, class balance
- **performance disclosure** — top-1, top-5, per-class
  recall on a held-out test set, with per-region slice
  metrics
- **model identity** — semantic version + container
  image digest pinned in every inference record
- **drift monitoring** — per-week class-distribution
  comparison; drift events emit an audit event
- **confidence threshold** — alert dispatch threshold
  per pest / crop combination, recorded in the model
  card

Inference records are not authoritative for NPPO
reporting; they trigger sample collection, not
quarantine action.

## §7 Alert escalation matrix

| Trigger                             | Severity      | Recipients              |
|-------------------------------------|---------------|-------------------------|
| Suspect detection (single observ.) | informational | farm operator           |
| Trap threshold met (single trap)   | advisory      | farm + agronomist       |
| Trap threshold met (network)       | action        | farm + agronomist + NPPO|
| Quarantine pest suspect             | critical      | NPPO + IPPC channel     |
| MRL-violation forecast              | action        | farm + buyer QA         |
| Drift / firmware-fault              | advisory      | device operator         |

Recipients acknowledge receipt with a signed
acknowledgement event; unacknowledged critical alerts
escalate after 2 hours.

## §8 Intervention-decision hierarchy (IPM)

The protocol enforces the IPM hierarchy on intervention
decisions:

```
preventive (cultural, host resistance, sanitation)
  → biological (predators, parasitoids, biocontrol agents)
  → mechanical / physical (traps, exclusion, hot-water)
  → chemical (pesticide, with MRL / PHI binding)
```

A chemical decision must justify the higher-tier
options as inadequate or non-applicable; the
justification is recorded on the decision record.

## §9 Pesticide application gate

A chemical intervention cannot be applied unless:

- the active ingredient is registered for the crop /
  pest combination in the regulator's catalogue
- the dose is within label range
- the applicator credential is active
- the previous application's PHI does not constrain
  the planned harvest date
- the buffer-zone rule (drift, neighbouring crops,
  bees, water bodies) is honoured

A decision that fails any gate returns `409` with the
applicable Problem Details body (PHASE 2 §11).

## §10 NPPO reporting flow

```
suspect → confirmed → official-record → IPPC-submitted → published / retired
                                              │
                                              └→ amended (post-event review)
```

ISPM 17 requires a contracting party's NPPO to report
new pests, outbreaks, and pest-status changes to the
IPPC. The protocol's NPPO-reporting flow:

1. diagnostic confirmation (§5)
2. official-record entry by NPPO authority
3. IPPC International Phytosanitary Portal submission
4. RPPO notification (e.g. EPPO, NAPPO)
5. trade-partner notification per WTO SPS
6. status updates as eradication / containment
   progresses

## §11 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (scout / agronomist / lab / NPPO / device)     |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / updated / confirmed / dispatched / filed      |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

The chain is per-field-unit and per-NPPO-report.

## §12 Reproducibility profile

Edge-AI inferences and remote-sensing analyses declare
a reproducibility tier:

- **reproducible-strong** — workflow URI published,
  container digest pinned, input manifest digest
  recorded, random seeds fixed
- **reproducible-weak** — one of the above missing

The reproducibility tier is recorded on the inference
record so downstream consumers see the level of
guarantee.

## Annex A — Worked outbreak example (informative)

A regional NPPO receives a suspect *Anoplophora
chinensis* (citrus longhorn beetle) detection in a
nursery import-inspection. The sample submits to an
ISO 17025 lab; PM 7/91 is run. The lab confirms the
pest and signs the diagnostic record. The NPPO
authority creates an official-record entry, files the
ISPM 17 report, and notifies EPPO. A delimiting
survey opens (§2) covering a 2 km buffer; alerts
dispatch to all nurseries in the buffer; a pesticide-
free containment plan implements (host-tree removal
under §8 mechanical tier). The audit chain records
each event with the sponsor's JWS key.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema version,
the JWS algorithm registry, the time-source authority,
and the EPPO / IPPC / WHOPES bindings supported.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS). For samples
spanning multiple time zones the local offset is
preserved on each event; UTC is reconstructed for
cross-border NPPO-to-NPPO exchange.

## Annex E — Operator-credential binding

| Credential                 | Source                                |
|----------------------------|---------------------------------------|
| ISO 17025 analyst          | accreditation body identifier         |
| Pesticide applicator       | national licensing authority          |
| NPPO inspector             | national appointment under ISPM 23    |
| Crop scout / agronomist    | extension service / professional body |

A diagnostic, NPPO report, or pesticide-application
event cannot be signed by an operator without an
active credential.

## Annex F — Surveillance-intensity targets (informative)

| Pest pressure context              | Minimum scout cadence |
|------------------------------------|-----------------------|
| Quarantine pest delimiting survey  | weekly until clear     |
| High-value perennial crop          | weekly through season  |
| Annual field crop, mid-pressure    | bi-weekly             |
| Stored product                     | monthly               |

Cadence under-runs are recorded as observation-gap
events on the field-unit record.

## Annex G — Inspector replay payload

For an NPPO inspection or supply-chain audit the
protocol exposes a replay payload covering:

- field-unit registration and version history
- observation, trap, sample, and diagnostic records
- inference and remote-sensing records
- alert dispatch and acknowledgement records
- intervention-decision and application records
- NPPO report records
- audit-chain export

The payload signs with the sponsor's audit-chain JWS
key so the inspector can verify integrity end-to-end
without trusting the API operator at runtime.

## Annex H — Pesticide drift mitigation evidence

Application records carry evidence of drift mitigation:

- nozzle type and droplet-size category (per ASABE
  S572 reference-spray classification)
- boom height
- forward-speed
- wind-speed and direction at application time
- buffer-zone observation
- reduced-drift adjuvant use (where required)

The evidence binds to the regulator's drift-management
expectations.

## Annex I — Outbreak escalation thresholds

| Pest                          | Action threshold (illustrative) |
|-------------------------------|---------------------------------|
| *Spodoptera frugiperda*       | 5 % infested whorls             |
| *Lobesia botrana*             | 5 captures per trap per week     |
| *Drosophila suzukii*          | 1 capture in pre-harvest window  |
| *Xylella fastidiosa* suspect  | any                              |
| *Anoplophora chinensis*       | any                              |
| Quarantine pest               | any                              |

Thresholds are pest- and region-specific; the field-
unit record cites the threshold table version under
which decisions were made.