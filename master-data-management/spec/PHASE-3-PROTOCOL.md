# WIA-master-data-management PHASE 3 — Protocol Specification

**Standard:** WIA-master-data-management
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
data records (PHASE 1) and API resources (PHASE 2)
into auditable lifecycles: record-lifecycle, golden-
record build cycle, match-cluster review, hierarchy
promotion, stewardship-task SLA, quality-rule
evaluation cadence, change-history retention, and the
audit-event chain. The protocols are framed so a data-
governance audit (per ISO 8000-150 / DAMA-DMBOK) can
reconstruct the full provenance of any master record
from the event log.

References (CITATION-POLICY ALLOW only):
- ISO 8000-1, 8000-110, 8000-115, 8000-120, 8000-150 — Data quality / master data
- ISO/IEC 11179-1, 11179-3, 11179-6 — Metadata registry
- ISO/IEC 19763 — Metamodel framework
- DAMA-DMBOK — Data Management Body of Knowledge (reference)
- ISO/IEC 27037 — digital evidence preservation
- ISO/IEC 27701 — privacy information management
- W3C SHACL — Shapes Constraint Language
- IETF RFC 5424 (Syslog), RFC 8941, RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Record lifecycle

```
ingested  →  validated  →  matched  →  survived  →  published
                  │            │
                  └→ rejected  └→ in-review (steward task)
```

Validation enforces the per-domain SHACL shapes (PHASE
1 §13). Match runs Fellegi-Sunter-style probabilistic
matching with optional ML rerankers; clusters above the
auto-merge threshold survive automatically; clusters
in the steward-review band open a stewardship task.

## §2 Golden-record build cycle

```
trigger → gather sources → score attributes → survive →
  publish snapshot → archive prior snapshot
```

Build triggers include: ingestion of a new source
record, steward action on a cluster, change to a
quality rule, or scheduled rebuild. Builds are
idempotent on the input set so re-runs produce the
same survivor under the same survivorship rules.

## §3 Match-cluster review

```
auto-merged → published
mid-band → steward-review → merged | split | hold
no-match → published as singleton
```

Steward actions are bounded by SLA tiers (PHASE 1
§12). A cluster `hold` action freezes downstream
golden-record consumption to prevent drift while the
review proceeds.

## §4 Hierarchy promotion

```
draft → reviewed → promoted (active) → superseded
            │
            └→ rejected → re-drafted
```

Promotion is single-version invariant per hierarchy
kind: only one version can be `active` at a time.
Effective-dating allows queries to retrieve any prior
version (`?at=<ISO-8601>`).

## §5 Stewardship-task SLA

| Priority   | Target acknowledgement | Target resolution |
|------------|------------------------|-------------------|
| critical   | 1 hour                 | 1 business day     |
| high       | 1 business day         | 5 business days    |
| medium     | 3 business days        | 15 business days   |
| low        | 5 business days        | 30 business days   |

SLAs are sponsor-tunable; the configured table is
recorded on the stewardship-policy reference record so
audits see the SLAs in force at any point in time.

## §6 Quality-rule evaluation cadence

| Trigger                          | Action                       |
|----------------------------------|------------------------------|
| New rule registered              | full re-evaluation queued     |
| Record write                     | rule subset matched at write |
| Daily / weekly / monthly         | scheduled batch evaluation    |
| Steward request                  | ad-hoc evaluation              |

Critical-rule violations on a record block the
golden-record from publishing until resolved; warning
violations annotate the record's quality-score.

## §7 Change-history retention

| Domain                      | Retention                              |
|-----------------------------|----------------------------------------|
| Party (regulated KYC / AML) | per regulator (typ. ≥ 5 years post-    |
|                             | relationship close)                    |
| Product                     | per regulator (lot trace) + sponsor    |
| Financial / asset           | per regulator (e.g. SEC 17a-4 — 6 y)   |
| Reference data              | indefinite (active + retired versions)  |
| Personal data               | per privacy basis + retention policy    |

Personal-data deletion under privacy law (GDPR Art. 17,
K-PIPA Art. 21, CCPA §1798.105) is supported by a
controlled erasure procedure that preserves the audit
chain hash but deletes the personally-identifiable
payload.

## §8 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (system / steward / source)                    |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / updated / merged / split / promoted / retired |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

The chain is per-domain and exportable to a SIEM via
RFC 5424 syslog envelopes.

## §9 Source-record ingest contract

| Concern                | Contract                                       |
|------------------------|------------------------------------------------|
| Idempotency            | `Idempotency-Key` header on POST               |
| Ordering               | event-sourced; out-of-order tolerated          |
| Late-arriving event    | bitemporal: `validFrom` distinct from `recordedAt` |
| Schema evolution       | `WIA-MDM-Schema-Version` header                 |
| Provenance             | `sourceSystemRef` mandatory                    |
| Quality                | rules evaluated at ingest                      |

## §10 Survivorship rules

Survivorship picks the surviving value per attribute
based on a precedence model:

| Strategy           | Description                                 |
|--------------------|---------------------------------------------|
| source-priority    | highest-trust source wins                    |
| recency            | most recent value wins                      |
| completeness       | longest non-null wins                       |
| reference-anchored | value bound to a regulator-issued anchor    |
|                    | (e.g. LEI for legalName) wins               |
| steward-override   | steward decision wins                       |

Strategy selection is per-attribute and recorded in
the survivorship-policy reference record.

## §11 Reproducibility

A golden-record snapshot is reproducible-strong if the
input source-record set, the rules applied, the
survivorship policy, and the algorithm version are all
content-addressed. Re-execution under the same inputs
produces a byte-equivalent snapshot.

## §12 Privacy posture

- Subject identifiers are opaque in cross-domain
  publication
- Personal data lives in domains under a lawful basis
  declaration (GDPR / K-PIPA / CCPA)
- Erasure preserves the audit-chain hash; the payload
  is removed and replaced with a tombstone

## Annex A — Worked steward review (informative)

A match cluster covering three "Acme Inc." records
falls in the steward-review band (probability 0.71).
The steward reviews the supporting evidence: two
records carry the same DUNS Number; the third lacks
DUNS but bears the same LEI as the first. The steward
applies a merge with a survivorship override that
prefers the LEI-bearing source for the address (post-
relocation). The merge emits an audit event; the
golden-record rebuild publishes the new snapshot.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the survivorship-
policy registry version, the SHACL shape catalogue
version, and the time source.

## Annex C — Versioning

Field additions are minor; semantic redefinition or
removal is major.

## Annex D — Bitemporal exposure

Records expose two time axes:

| Axis            | Meaning                                          |
|-----------------|--------------------------------------------------|
| `validFrom/To`  | when the modelled fact holds in the real world   |
| `recordedFrom/To` | when the record exists in the system            |

Queries support `?validAt=<ISO-8601>` and
`?recordedAt=<ISO-8601>` to reconstruct prior beliefs
or correct retroactively without losing history.

## Annex E — Quality-dimension reference

Per ISO 8000-150 the system records per-record scores
for at least:

- completeness
- accuracy
- consistency
- timeliness
- uniqueness
- validity
- referential integrity

Aggregate scores roll up to the domain and to the
implementation; quality-rule evaluations contribute
to the rolling score.

## Annex F — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB). Cross-region
implementations use UTC for the audit-chain canonical
timestamp; local offset is preserved on each event.

## Annex G — Steward credential binding

| Credential                | Source                                |
|---------------------------|---------------------------------------|
| Data-steward attestation  | sponsor-internal certification        |
| Privacy officer (DPO)     | per regulator (GDPR Art. 37 / K-PIPA) |
| Subject-rights handler    | sponsor-issued                        |
| Reference-data steward    | sponsor / regulator (e.g. for ISO     |
|                           | TC representatives)                   |

A merge / split / survivorship-override action cannot
be signed by a steward without an active credential.

## Annex H — Coexistence reconciliation cycle

In coexistence-pattern deployments the protocol runs
a reconciliation cycle:

```
fetch local extensions → diff vs. last sync →
  classify (golden-affecting / local-only) →
  apply golden updates → push back local-only →
  emit audit event
```

The cycle frequency is sponsor-configurable (typically
hourly to daily). Out-of-band changes that affect
golden attributes raise stewardship tasks rather than
silently overwriting the golden value.

## Annex I — Late-arriving event handling

Source systems may emit events with `validFrom` in the
past (e.g. a party's address change recorded weeks
later). The protocol:

1. accepts the event with its real-world `validFrom`
2. records the system-time `recordedAt` separately
3. re-runs survivorship for the affected attribute
4. re-builds the golden snapshot for the impacted date
   range
5. publishes a corrected change-event marked
   `correction=true`

Downstream consumers honouring the bitemporal exposure
update both their current view and their historical
view; consumers that only honour the current view see
the latest value plus a tag indicating the correction.

## Annex J — Golden-record snapshot publication

| Snapshot kind          | Cadence                                   |
|------------------------|-------------------------------------------|
| Per-record on change   | event-driven; CDC topic                   |
| Daily full snapshot    | end-of-day file + manifest digest         |
| Quarterly full archive | snapshot + retention-tier promotion       |

Daily snapshot manifests sign with the implementation's
JWS key so consumers can verify integrity end-to-end.

## Annex K — Operational metrics for protocol audit

The protocol records the following metrics on the
audit chain so an inspection can compute compliance:

- per-priority steward-task SLA compliance
- per-domain match-rate distribution
- coexistence reconciliation latency
- subject-rights handling latency
- late-arriving correction frequency
- snapshot publication completeness

These metrics export to the sponsor's data-governance
dashboard at the cadence configured in the
stewardship-policy reference record (typically daily for
operational metrics and quarterly for trend analyses).
Steward attestations (sign-off on quarterly metrics)
are themselves audit events on the chain.
