# WIA-next-gen-data-storage PHASE 3 — Protocol Specification

**Standard:** WIA-next-gen-data-storage
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
records and API resources into auditable lifecycles:
domain commissioning and decommissioning, namespace
lifecycle, replica-and-quorum operation, encryption-
envelope rotation cadence, durability monitoring with
scrubbing and fixity, fault recovery and rebuild,
firmware-update governance, capacity-planning trigger
chain, and the audit-event chain. The protocols
ensure that a vendor RMA, a security incident, or a
capacity-planning audit can reconstruct the storage
state from the event log.

References (CITATION-POLICY ALLOW only):
- SNIA SMI-S 1.8; SNIA Swordfish 1.2.6; DMTF Redfish 2024.x
- NVM Express Base Specification 2.0; NVMe-MI 1.2; NVMe-oF 1.1
- NIST SP 800-57 (key-management) Parts 1, 2, and 3
- NIST FIPS 140-3 — Cryptographic Module Validation
- NIST SP 800-88 Rev 1 — Media Sanitisation
- ISO/IEC 27037 — digital evidence preservation
- ISO/IEC 27040 — storage security
- IETF RFC 5424 (Syslog), RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Domain commissioning and decommissioning

```
ordered → installed → burn-in → certified → production →
  deprecated → decommissioned → media-sanitised → disposed
```

Burn-in covers manufacturer-recommended power-cycle,
performance, and endurance pre-checks. Certification
records the controller / firmware versions and the
performance baseline. Decommissioning runs media-
sanitisation per NIST SP 800-88 (clear, purge, or
destroy depending on data classification).

## §2 Namespace lifecycle

```
created → in-use → migrated → archived → released
                       │
                       └→ frozen (regulatory / forensic hold)
```

Frozen namespaces resist deletion until the hold is
released; the implementation enforces the hold via
the WORM-locked Problem Details type (PHASE 2 §12).

## §3 Replica-and-quorum operation

| Mode               | Operation                                       |
|--------------------|-------------------------------------------------|
| Synchronous        | every write replicated to all replicas before   |
|                    | acknowledgement; bounded by a tail-cut timer    |
| Asynchronous       | primary acknowledges; replica catches up        |
|                    | within RPO budget                               |
| Quorum             | write requires majority replica acknowledgement |
| Chain replication  | head receives writes; chain propagates           |
| Paxos / Raft       | consensus-bound replicated state machine         |

Split-brain detection invokes the configured policy
(majority-quorum / fence / pause) and emits a fault
event.

## §4 Encryption-envelope rotation cadence

| Layer       | Rotation cadence                                       |
|-------------|--------------------------------------------------------|
| KEK         | per sponsor policy (typ. annual)                        |
| DEK         | per object / per volume; on key-compromise emergency    |
| Tweak       | for XTS: per LBA; never reused across volumes           |
| KMS audit   | per access; per rotation; per envelope creation         |

Compromise events trigger immediate rotation; the
chain-of-custody records the compromise event and the
re-encryption progress.

## §5 Durability monitoring (scrubbing and fixity)

```
schedule → read-scrub → checksum-verify → repair-from-replica → record
```

Scrubbing cadence is media-aware:

| Media type        | Scrub cadence                                      |
|-------------------|----------------------------------------------------|
| NVMe SSD          | weekly                                             |
| HDD               | monthly                                            |
| Tape archive      | per LTO recommendations (typ. 3-5 year recall test) |
| DNA / polymer     | per substrate manufacturer guidance                |
| Optical disc      | annual representative-sample read                   |

Bit-rot detection triggers repair from replica; if no
replica passes verification the operator is notified
(fault: critical) and recovery falls back to the most
recent good replica.

## §6 Fault recovery and rebuild

```
fault-detected → degraded-mode → rebuild-initiated →
  rebuild-progressing → rebuild-completed → fault-cleared
```

Rebuild bandwidth is capacity-aware to limit
performance impact on production workloads; sustained
multi-hour rebuilds emit progress events to the
audit chain.

## §7 Firmware-update governance

```
update-staged → verified → quiesced → applied → re-validated
                                  │
                                  └→ rollback (on failure)
```

Updates honour vendor-recommended sequencing (e.g.
controller firmware before drive firmware); cluster-
wide rolling updates preserve quorum throughout. A
failed update triggers an automatic rollback to the
prior known-good firmware version.

## §8 Capacity-planning trigger chain

| Threshold           | Action                                          |
|---------------------|-------------------------------------------------|
| 70 % capacity       | informational; capacity-planning task            |
| 85 % capacity       | warning; provisioning request                    |
| 95 % capacity       | critical; new-write throttling option            |
| Quota-per-consumer  | per consumer's contractual cap                   |
| Replica-rebuild     | reserved capacity sized for rebuild + new write  |

Triggers emit audit events; sponsor-policy decides
whether new writes throttle or block at the critical
threshold.

## §9 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (operator / system / vendor / sponsor)         |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / migrated / repaired / rotated / decommissioned|
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

The chain is per-domain.

## §10 Media-sanitisation discipline

Per NIST SP 800-88 the protocol records:

| Method     | Description                                       |
|------------|---------------------------------------------------|
| `clear`    | logical wipe; sufficient for low-classification    |
| `purge`    | cryptographic erase + media-specific purge        |
|            | (overwrite / block-erase / firmware-purge)        |
| `destroy`  | physical destruction (shred, incinerate, melt)    |

Sanitisation events emit certificates that the storage
operator preserves per regulator retention.

## §11 Reproducibility

A storage configuration is `reproducible-strong` when
the firmware versions, the policies, the QoS
profiles, and the encryption envelopes are content-
addressed; `weak` when any is absent.

## §12 Privacy-aware storage

Where storage holds personal data, the protocol gates:

- DEK rotation on data-erasure (cryptographic erase)
- per-object retention policy for legal hold
- access-log retention
- regulator-aligned retention budget

Privacy events bind to the encryption-envelope record
(PHASE 1 §7) so cryptographic erasure is auditable.

## Annex A — Worked rebuild example (informative)

A 16-drive scale-out NVMe array detects a single-drive
failure. The scrub schedule recently confirmed the
remaining drives clean. Rebuild initiates from
parity-coded redundancy; rebuild bandwidth caps at
200 MB/s on the affected node to preserve front-end
performance. Rebuild completes in 11 hours; the
audit chain records the fault, the rebuild progress,
and the cleared-fault event.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the firmware-
update governance protocol, the NIST SP 800-88
sanitisation methods supported, and the QoS classes
offered.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB).

## Annex E — Operator-credential binding

| Credential                | Source                               |
|---------------------------|--------------------------------------|
| Storage admin             | sponsor + vendor certification        |
| Cryptographic-key custodian| sponsor + KMS-controller policy       |
| Firmware-update operator  | sponsor + vendor RMA/firmware policy  |
| Forensic-hold custodian   | sponsor + legal department             |

A signing event by an operator without an active
credential rejects.

## Annex F — Vendor RMA chain

Hardware returns to the vendor for warranty / RMA
handle:

```
fault-confirmed → media-sanitised → ship-prep →
  shipped → vendor-received → replacement-staged →
  installed → certified → fault-cleared
```

Sanitisation precedes shipment so customer data does
not leave the sponsor's environment.

## Annex G — Backup-and-DR protocol

```
schedule → snapshot-create → replicate-to-DR-site →
  verify-on-DR → confirm-RPO-met → record
```

Backup snapshots are immutable for the retention
window; restore-from-DR is a controlled procedure
recorded in the audit chain. DR drills exercise the
full chain at a sponsor-policy cadence (typically
quarterly for critical workloads).

## Annex H — Tiering decision protocol

```
access-pattern-observed → policy-evaluated →
  cost-impact-modelled → tier-transition-decided →
  data-moved → policy-completion-event
```

Per-object access counters feed the placement engine;
per-policy decisions consult the cost-policy record
(PHASE 1 Annex E) so tier-promotion / demotion
balances performance against cost.

## Annex I — Forensic-hold protocol

```
hold-requested → namespace-frozen → snapshot-taken →
  forensic-export-prepared → exported-to-investigator →
  hold-released-or-extended
```

Forensic holds preserve content via WORM-locking and
record the legal authority issuing the hold (court
order, regulator directive, internal investigation
mandate).

## Annex J — Performance-regression detection

The protocol records per-operation performance
distributions so a regression from baseline triggers
a stewardship task:

| Distribution               | Threshold                          |
|----------------------------|------------------------------------|
| p99 latency                | +20 % over baseline → warning;     |
|                            | +50 % → critical                    |
| Throughput                 | -15 % below baseline → warning;     |
|                            | -30 % → critical                    |
| Tail latency p99.99        | +50 % over baseline → critical      |
| QoS-class adherence rate   | < 99 % over rolling window →        |
|                            | warning                             |

Regression events cite the suspect software / firmware
version and the workload mix. Sponsor SREs review;
remediation may include rollback, capacity addition,
or workload-mix change.