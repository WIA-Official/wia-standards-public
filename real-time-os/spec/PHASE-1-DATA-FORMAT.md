# WIA-COMP-019 — Phase 1: Data Format

> RTOS canonical Phase 1: task-record + schedulability + IPC + time-base + memory-region + interrupt-record envelopes.

# WIA-COMP-019: Real-Time Operating System Specification v1.0

> **Standard ID:** WIA-COMP-019  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active


## 1. Introduction

This specification defines standards for real-time operating systems including task scheduling, IPC, resource management, and timing analysis.

**弘익인간 (Benefit All Humanity)** - Deterministic systems enable safety-critical applications.



## 2. Scheduling Algorithms

### 2.1 Fixed Priority Preemptive
- Priority-based task execution
- Preemption for higher priority tasks
- Deterministic behavior

### 2.2 Rate Monotonic (RM)
- Static priority by period
- Optimal for periodic tasks
- Schedulability test: U ≤ n(2^(1/n) - 1)

### 2.3 Earliest Deadline First (EDF)
- Dynamic priority scheduling
- Optimal for aperiodic tasks
- Schedulability test: U ≤ 1



## 3. Inter-Process Communication

### 3.1 Synchronization
- Semaphores (binary, counting)
- Mutexes with priority inheritance
- Events and flags

### 3.2 Message Passing
- Message queues
- Mailboxes
- Pipes




---

## A.1 Task-record envelope

The Phase 1 envelope groups RTOS tasks by scheduling class (periodic
— activated at fixed period T per the operator's RM/EDF schedule;
sporadic — minimum inter-arrival time MIT bounded; aperiodic —
event-driven with admission control via constant-bandwidth-server
per Abeni + Buttazzo 1998; idle — runs only when no schedulable
task is ready). Each task record carries: task identifier (UUID v7
per RFC 9562), task class, period T (ms or μs as the operator's
clock-resolution permits), worst-case execution time WCET (per the
operator's WCET-analysis tool — aiT, Bound-T, OTAWA, RapiTime,
Heptane, or measurement-based with the documented coverage envelope
per ISO 26262 §6 Table 2), deadline D (relative to release; default
D=T for implicit-deadline RM/EDF), priority (static for RM/DM;
dynamic for EDF), stack-size envelope (canary + guard-page
verification per per-RTOS conformance test), and the per-task
audit envelope tied to the host application.

## A.2 Schedulability-record envelope

A schedulability-record envelope MUST list: schedulability-test
identifier, set-of-tasks reference (linking to the §A.1 task-set
record), test-method envelope (Liu + Layland 1973 utilisation bound
for n-task RM `U ≤ n(2^{1/n}-1)` with the limit `ln 2 ≈ 0.693`;
hyperbolic bound per Bini + Buttazzo 2003 `∏(U_i+1) ≤ 2`; response-
time analysis per Joseph + Pandya 1986; processor-demand analysis
per Baruah + Mok + Rosier 1990 for EDF; mixed-criticality analysis
per Vestal 2007 for IEC 61508 + ISO 26262 + DO-178C compositions),
the per-task response-time histogram with its 99.99th-percentile
envelope, the schedulability-margin envelope (utilisation reserve
+ deadline-slack reserve per the operator's safety case), and the
audit envelope linking back to the operator's safety-case record.

## A.3 IPC-channel envelope

Inter-process-communication channel envelopes describe per-channel
properties: kind (mailbox per POSIX `mq_*`; binary semaphore per
POSIX `sem_*`; counting semaphore; condition variable per POSIX
`pthread_cond_*`; mutex with the per-mutex priority-inheritance /
priority-ceiling protocol per POSIX 1003.1c + Sha + Rajkumar +
Lehoczky 1990; event-flag group; ring-buffer per the operator's
lock-free queue specification; shared-memory region per POSIX
`shm_*` with the per-region locking protocol), channel-capacity
envelope, blocking-time envelope per the analysis above, the
deadlock-prevention envelope (resource-allocation graph per Coffman
1971; banker's algorithm per Dijkstra 1965), and the per-channel
priority-inversion bound proven via the chosen protocol.

## A.4 Time-base and clock-source envelope

Time-base and clock-source envelopes carry: monotonic clock source
(`CLOCK_MONOTONIC` per POSIX 1003.1b; high-resolution per
`CLOCK_MONOTONIC_RAW`; per-CPU TSC per Intel SDM Vol 3B; ARM
generic timer per ARM ARM § ChapterD7; RISC-V `mtime` per RISC-V
Privileged Spec); clock-resolution envelope (typical 10-ns to 1-μs
per the host platform); jitter envelope (kernel-tick jitter +
interrupt-latency jitter + scheduler-latency jitter per the
operator's per-platform jitter-test); time-source-synchronisation
envelope (PTPv2 per IEEE 1588-2019 with the per-domain BMCA
envelope; NTP per RFC 5905 for non-real-time wall-clock; chrony
per the operator's policy); and the leap-second handling envelope
per the operator's per-platform monotonic-clock immunity.

## A.5 Memory-region envelope

Memory-region envelopes describe per-region properties: kind (text
+ rodata + data + bss per ELF per System V ABI; per-task stack
with the canary + guard-page envelope; per-task heap with the
per-RTOS allocator envelope; DMA-coherent buffer with the IOMMU
mapping envelope per platform IOMMU); access-permission envelope
(R/W/X per the per-RTOS MPU/MMU policy); cache-coherence envelope
(per-region cache-coherent vs cache-incoherent envelope per the
host platform's cache coherency protocol — MESI per Goodman 1983
+ MOESI per AMD); memory-barrier envelope (acquire / release /
total-store-order per the per-platform memory model — TSO on x86;
weakly-ordered on ARMv8 + RISC-V); and the per-region scrub
envelope per ISO 26262 ASIL D requirements where applicable.

## A.6 Interrupt-record envelope

Interrupt-record envelopes describe per-interrupt-source properties:
interrupt identifier, source class (timer / serial / DMA / GPIO /
sensor-bus / network / inter-processor per the operator's hardware
manifest), per-source latency budget envelope (interrupt-disable
window per the kernel + interrupt-controller propagation latency
+ ISR entry latency + scheduler-restart latency), priority
envelope (per the operator's interrupt-priority policy with the
fixed vs configurable-priority envelope per the platform's
interrupt-controller — GIC v3/v4 per ARM; APIC per Intel; PLIC per
RISC-V), and the per-source de-bounce envelope (hardware filter +
software de-bounce envelope per the operator's SFR procedure).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/real-time-os/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-real-time-os-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/real-time-os-host:1.0.0` ships every real-time-os envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/real-time-os.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Real-time-os deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

## Z.6 Logging and observability hooks

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: ISO 8601 UTC timestamp per RFC 3339, host
identifier, tenant identifier, envelope class, envelope identifier,
operation outcome, and an opaque trace identifier propagated end-
to-end per W3C Trace Context (`traceparent` header) so a single
operation can be reconstructed across hosts. Phase 2 surfaces this
trace identifier as the `X-WIA-Trace-Id` response header. Phase 3
protocol exchanges propagate the trace identifier inside the
exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (e.g., Splunk, Elastic, Sumo
Logic, Wazuh, Microsoft Sentinel) per OpenTelemetry semantic
conventions, with `wia.standard.slug` and `wia.standard.phase` as
required attributes.

## Z.7 Versioning, deprecation, and capability discovery

Within the 1.x line, hosts MAY publish a capabilities document at
`/.well-known/wia-real-time-os-capabilities` that enumerates which
optional fields, optional endpoints, and optional protocol exchanges
the host implements. Clients MUST treat unsupported capabilities
as absent rather than as an error condition; a client that needs
a capability the host does not advertise MUST surface a clear
configuration error rather than silently degrade. Hosts moving
from one minor version to the next MUST publish the change in
the host's release notes with the per-capability migration window
per IETF RFC 8594 (Sunset header) + RFC 9745 (Deprecation header)
+ RFC 9651 (Structured Field Values) so machine consumers can
plan migration without waiting for human-channel notification.

## Z.8 Privacy and data-minimisation envelope

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California CPRA
per Cal. Civ. Code §1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Where the envelope ships across jurisdictional borders,
the operator's per-jurisdiction transfer envelope (SCC for EU, UK
IDTA, APEC CBPR, ASEAN MCC) MUST be referenced inside the audit
record. Subject-rights endpoints (access, rectification, erasure,
portability, restriction, objection) compose with WIA-OMNI-API per
its §5 subject-rights surface and need not be re-implemented
per-standard.

## Z.9 Disaster recovery and continuity-of-operations envelope

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-
exit envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity. The DR
envelope composes with WIA Secure Enclave for sealed-backup
envelopes and with WIA-AIR-SHIELD for runtime trust-list re-
hydration on the failover instance.

## Z.10 Supply-chain and software-bill-of-materials envelope

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per the operator's chosen specification: SPDX 2.3 / 3.0 per
ISO/IEC 5962 + Linux Foundation SPDX, or CycloneDX 1.6 per OWASP
Foundation. The SBOM enumerates every direct + transitive dependency
with the per-component name + version + licence + supplier + per-
component hash + per-component PURL (Package URL per package-url
spec) + per-component CPE (Common Platform Enumeration per NIST).
The host MUST publish per-release SBOM updates and MUST flag
breaking dependency-version migrations so downstream consumers
can plan ahead. Supply-chain attestation follows in-toto per
CNCF in-toto + SLSA (Supply-chain Levels for Software Artifacts)
per OpenSSF SLSA Framework — typically targeting SLSA Level 3 for
hosted production deployments.

弘益人間 — Benefit All Humanity.
