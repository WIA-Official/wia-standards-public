# WIA-microbiome PHASE 3 — Protocol Specification

**Standard:** WIA-microbiome
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
the data records and API surface into auditable
sequences: specimen lifecycle, library and run lifecycle,
analysis workflow lifecycle, contamination control,
batch-effect mitigation, and the audit-event chain.
The protocols are designed so that an inspector
following ISO/IEC 17025 or biobanking ISO 20387 can
reconstruct the provenance of any analytical result
from the event log.

References (CITATION-POLICY ALLOW only):
- ISO 20387:2018 — General requirements for biobanking
- ISO/IEC 17025:2017 — Testing and calibration laboratories
- ISO 22174 / ISO 22118 / ISO 20837 — molecular methods for foodborne pathogens
- ISO/IEC 27037 — guidelines for digital evidence preservation
- INSDC submission policy — NCBI / ENA / DDBJ
- GSC MIxS / MIMARKS / MIMS — minimum information packages
- 21 CFR Part 11 — electronic records and signatures (US)
- IEEE 11073-10101 — laboratory device nomenclature
- IETF RFC 5424 (Syslog), RFC 8941 (Structured Field Values), RFC 7515 (JWS)
- ISO 8601 (date / time), BCP 47 (language)

---

## §1 Specimen lifecycle

```
collected  →  preserved  →  in-transit  →  received  →  extracted
                                              │
                                              └─→ excluded (preservation fail)
```

Each transition emits an audit event with actor,
timestamp, and method (the chain-of-custody event of
PHASE 1 §2). Excluded specimens cannot be bound to
new library records; the exclusion event records the
reason (temperature breach, container compromise,
pre-analytical TAT exceeded).

## §2 Library and run lifecycle

```
library:    drafted  →  prepped  →  qc-passed  →  loaded  →  released
                                       │
                                       └─→ qc-failed → repreparation | discarded

run:        opened   →  started  →  finished  →  qc-released
                                       │
                                       └─→ run-aborted → instrument-fault
                                                         flow-cell-fault
                                                         operator-error
```

QC thresholds for amplicon (read length post-trim,
median Q30 ≥ 90 %, demultiplex rate ≥ 80 %) and shotgun
(median Q30 ≥ 85 %, host-fraction declared) are recorded
on the run record. A run that fails QC blocks downstream
analysis dispatch.

## §3 Contamination control

Microbiome studies are uniquely sensitive to reagent
and environmental contamination. The standard requires:

- per extraction batch: at least one extraction-blank
  sequenced in parallel
- per library batch: at least one negative PCR control
  and one mock-community positive control
- per run: a per-run blank well or known-fingerprint
  control

The analysis workflow uses the negative-control reads
to subtract reagent-contaminant ASVs / OTUs (via
`decontam` or equivalent) before downstream statistics.
Studies that omit controls cannot reach `qc-released`
state.

## §4 Analysis workflow lifecycle

```
dispatched  →  scheduled  →  running  →  succeeded  →  archived
                                  │
                                  └─→ failed   →  retried  |  abandoned
```

Each workflow run records: the workflow URI, the
container image digest, the input manifest digest,
the parameter set, the resource budget consumed, and
the output artefact digests. Workflows that fail are
retried at most three times before requiring operator
review.

## §5 Batch-effect mitigation

Studies span sequencing batches. The protocol requires:

- randomised plate layout (subject × condition not
  confounded with plate position or extraction batch)
- bridge samples across batches (replicate aliquots of
  a reference specimen on each plate)
- batch-effect declaration in the analytical-result
  record (correction method, software version)

Bridge samples are flagged in the metadata table so the
analyst can include them in the batch-effect model
(e.g. ComBat-Seq, MMUPHin) or use them to validate
robustness of the result.

## §6 Audit event chain

Every state change emits one audit event:

| Field          | Meaning                                                   |
|----------------|-----------------------------------------------------------|
| `eventId`      | UUID                                                      |
| `eventTime`    | ISO 8601 with timezone                                    |
| `actor`        | identity (technician / operator / pipeline / submission)  |
| `resourceRef`  | URI of the resource that changed                          |
| `action`       | created / qc-passed / qc-failed / dispatched / released   |
| `priorHash`    | SHA-256 of the prior event payload                        |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)  |

The chain is per-study and exportable to a SIEM via
RFC 5424 syslog envelopes.

## §7 INSDC submission protocol

```
prepared  →  validated  →  submitted  →  suppressed  →  released
```

Submission state tracks the upstream archive's view.
`released` is irrevocable once the public archive has
honoured the embargo lift; corrections after release
go through a curated update channel rather than a
delete.

## §8 Reproducibility profile

A result is "reproducible-strong" when:

- workflow URI is published
- container image is content-addressed (digest pinned)
- input manifest digest is recorded
- random seeds are fixed in the parameter set

A result is "reproducible-weak" when one of the above
is absent. The analytical-result record discloses the
reproducibility tier so downstream consumers see the
level of guarantee.

## §9 Privacy posture for human microbiome

Human microbiome data may carry host genetic signal
(off-target reads). The standard requires:

- explicit host-read removal step (BWA / Bowtie2 against
  the consented host reference) before public release
- a fraction-removed metric reported per read-set
- residual host-fraction threshold below 0.01 % (or
  per-study lower) for public submission

Where residual host fraction exceeds the threshold a
re-filter step is mandatory; if technically infeasible
the dataset is declared controlled-access only.

## §10 Pre-analytical lifetime targets (informative)

| Matrix       | Time-to-preservation target | Storage             |
|--------------|-----------------------------|---------------------|
| Stool        | ≤ 30 min into stabiliser    | -80 °C archive      |
| Oral / saliva| ≤ 30 min into preservative  | -80 °C archive      |
| Skin swab    | ≤ 60 min                    | -80 °C archive      |
| Soil         | ≤ 4 h                       | -80 °C or lyophilised |
| Water        | ≤ 4 h, on ice               | -80 °C after filter |
| Food matrix  | per Codex CAC/RCP 1-1969    | per food-process SOP |

## §11 Reagent-lot traceability

All extraction kits, indexing primers, master mixes,
and flow cells carry the manufacturer lot in the library
record. Lot rotation events are bound to runs so a
recalled lot can be traced to all affected results.

## §12 Out-of-spec handling

A library or run that fails QC is retained for at least
one calendar quarter so root-cause analysis is possible.
Discarded material moves to a quarantine status before
deletion; deletion events are recorded in the audit
chain.

## Annex A — Worked SAE-equivalent example (informative)

An extraction batch processed for a clinical IBD cohort
shows extraction-blank reads dominated by *Burkholderia*,
matching a recent reagent-lot recall. The batch is
retroactively flagged; the analytical-result record
inherits a `qc-warning:reagent-recall:lot-2026-03-15`.
Affected libraries return to `qc-failed`; specimens
remain valid and re-extract on a new reagent lot.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema version,
the JWS algorithm registry, the time source (NTP
stratum-1, NIST, KASI), and the contamination-control
software (version of `decontam` or equivalent).

## Annex C — Versioning

This PHASE follows the standard's semantic-versioning
policy. Adding a new contamination-control method is a
minor revision; changing batch-effect declaration
semantics is major.

## Annex D — Decision matrix for workflow re-runs

| Trigger                                | Action                              |
|----------------------------------------|-------------------------------------|
| Reference-database release             | new analytical-result; prior kept   |
| Pipeline software update (minor)       | optional re-run; result tagged      |
| Pipeline software update (major)       | re-run recommended; tier downgraded |
| Reagent-lot recall                     | mandatory re-evaluation; affected   |
|                                        | results carry recall flag           |
| Batch-effect model update              | re-run analytical-result only       |
| Specimen-level QC re-evaluation        | re-run from library stage           |

The decision is recorded on the new analytical-result
record's `triggerEvent` field for audit purposes.

## Annex E — Operator-credential binding

The operator on a library-prep, sequencing-run, or
analysis-dispatch event carries:

| Credential      | Source                                          |
|-----------------|-------------------------------------------------|
| ISO/IEC 17025   | accreditation body identifier                   |
| Lab-internal    | training-record reference                       |
| ROIS (BCKL/ROIS)| Bioinformatics analyst credential               |

A run cannot release results signed by an operator
without an active credential. Credential events
(issued, expired, revoked) propagate to the audit
chain.

## Annex F — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, PTB). For samples spanning
multiple time zones the local offset is preserved on
each event; UTC is reconstructed for cross-cohort
analyses.

## Annex G — Sample-size sufficiency table (informative)

The protocol's sample-size sufficiency depends on the
intended outcome:

| Outcome target                     | Minimum specimens / arm |
|------------------------------------|------------------------:|
| Alpha-diversity comparison         | 30                      |
| Beta-diversity PERMANOVA           | 50                      |
| Differential-abundance discovery   | 80                      |
| Random-forest biomarker training   | 200                     |
| Strain-level shotgun resolution    | 30 (with deeper depth)  |

Studies declaring outcomes below these thresholds carry
an underpowered-flag on the analytical-result record
so downstream consumers see the limitation.

## Annex H — Per-step turnaround targets (informative)

| Step                          | Turnaround target          |
|-------------------------------|----------------------------|
| Receive → extracted           | ≤ 7 days                   |
| Extracted → library prepped   | ≤ 5 days                   |
| Library prepped → sequenced   | ≤ 14 days                  |
| Sequenced → primary analysis  | ≤ 7 days                   |
| Primary analysis → reviewed   | ≤ 14 days                  |

Targets are advisory; protocol-specific SOPs may
override. Deviations are logged on the audit chain.

## Annex I — Reagent-lot recall escalation

When a reagent-lot is recalled by the manufacturer the
implementation:

1. queries all libraries that bound the recalled lot
2. flags the affected runs with `recall-pending`
3. notifies the principal investigator and study
   coordinator
4. schedules a re-evaluation of the affected analytical
   results using a comparator lot or a re-extracted
   library aliquot
5. records the recall outcome on each affected
   analytical-result record (`recall-cleared` or
   `result-withdrawn`)
