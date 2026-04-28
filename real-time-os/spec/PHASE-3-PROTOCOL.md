# WIA-COMP-019 — Phase 3: Protocol

> RTOS canonical Phase 3: protocols (RM/DM + EDF + mixed-criticality + priority-inheritance + interrupt-handling + time-keeping).

# WIA-COMP-019: Real-Time Operating System Specification v1.0

> **Standard ID:** WIA-COMP-019  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active



---

## A.1 RM / DM scheduling protocol

Rate-Monotonic protocols per Liu + Layland 1973 cover: per-task
priority assignment by inverse-period (shorter T → higher priority);
admission-control envelope via the n-task utilisation bound `U ≤
n(2^{1/n}-1)` with the asymptotic limit `ln 2 ≈ 0.693`; per-task
hyperbolic bound per Bini + Buttazzo 2003 `∏(U_i+1) ≤ 2` (necessary
+ sufficient for n-task RM); response-time analysis per Joseph +
Pandya 1986 `R_i = C_i + Σ_{j∈hp(i)} ⌈R_i / T_j⌉ C_j` with the
fixed-point iteration converging in O(n) for schedulable systems;
the deadline-monotonic extension per Leung + Whitehead 1982 (D ≤
T case); and the multi-processor RM extension per Andersson +
Baruah + Jonsson 2001.

## A.2 EDF scheduling protocol

Earliest-Deadline-First protocols per Liu + Layland 1973 cover:
per-task dynamic-priority assignment by absolute deadline (shortest
absolute deadline → highest priority); admission-control envelope
via `Σ U_i ≤ 1` (necessary + sufficient for implicit-deadline EDF
on uniprocessor); processor-demand analysis per Baruah + Mok +
Rosier 1990 covering arbitrary-deadline EDF; the constant-
bandwidth-server CBS per Abeni + Buttazzo 1998 for aperiodic
workload admission with bounded interference; the EDF-utilisation
upper-bound for sporadic tasks; the EDF-multiprocessor extension
per gEDF (global EDF) per Baruah + Goossens 2003 + EDF-FF (first-
fit) per Lee + Lee 1993 for partitioned EDF.

## A.3 Mixed-criticality scheduling protocol

Mixed-criticality protocols per Vestal 2007 cover: per-task multi-
criticality WCET envelope (typical bi-criticality LO + HI WCET per
Vestal 2007; tri-criticality LO + MED + HI per Burns + Davis 2017);
per-criticality budget envelope (LO-mode budget guarantees all-
criticality tasks; HI-mode budget guarantees only HI-criticality
tasks); the EDF-VD (Earliest-Deadline-First with Virtual Deadlines)
per Baruah + Bonifaci + D'Angelo 2012; the AMC (Adaptive Mixed-
Criticality) per Baruah + Burns + Davis 2011; the per-mode
transition envelope (LO → HI on first deadline-overshoot; HI → LO
after operator-defined cooldown); and the verification envelope
per the per-criticality-class certification bar (DO-178C DAL A/B
+ ISO 26262 ASIL D + IEC 62304 Class C as the typical HI-class).

## A.4 Priority-inheritance + ceiling-protocol protocol

Priority-inheritance + ceiling-protocol protocols per Sha +
Rajkumar + Lehoczky 1990 cover: priority-inheritance protocol
(PIP) — a low-priority task holding a mutex inherits the priority
of the highest-priority blocked task; priority-ceiling protocol
(PCP) — each mutex has a ceiling = priority of the highest-priority
task that may use the mutex; immediate ceiling priority protocol
(ICPP / Highest Locker's Priority) — a task acquiring a mutex
immediately runs at the mutex ceiling; the per-protocol blocking-
time bound (PIP `B_i = Σ over min(n_i, m_i) blocking critical
sections`; PCP `B_i ≤ max blocking critical-section length`); and
the deadlock-freedom guarantee under PCP / ICPP. POSIX 1003.1c
mandates PIP + ICPP availability on POSIX-RT-conformant systems.

## A.5 Interrupt-handling protocol

Interrupt-handling protocols cover: ISR entry envelope (interrupt-
disable window + interrupt-controller-acknowledge + register save
per the per-platform calling convention); top-half / bottom-half
split per Linux kernel softirq + tasklet + workqueue envelope;
threaded-IRQ envelope per Linux PREEMPT_RT for converting per-IRQ
work into a schedulable kernel thread; per-ISR latency budget
envelope (typical ≤10 μs entry latency on a tuned system; ≤100 μs
worst-case for a moderate platform; per-system goal documented in
the safety case); the nested-interrupt envelope per the platform's
interrupt-priority depth; and the per-IRQ-source rate-limit
envelope per the operator's per-IRQ-source policy.

## A.6 Time-keeping and clock-discipline protocol

Time-keeping and clock-discipline protocols cover: per-source
monotonic clock per `CLOCK_MONOTONIC` per POSIX 1003.1b; high-
resolution per `CLOCK_MONOTONIC_RAW` per Linux + per-platform
equivalent; PTP-disciplined wall-clock per IEEE 1588-2019 with the
per-domain BMCA envelope + per-port master-slave envelope + per-
domain hardware time-stamping envelope; per-platform clock-
adjustment envelope per `clock_adjtime` per POSIX-RT; per-platform
leap-second handling envelope (smear per Google + Facebook
practice; step per default `clock_gettime(CLOCK_REALTIME,...)`;
the per-system policy MUST be documented); the clock-source
hot-swap envelope per the operator's redundancy policy.


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
