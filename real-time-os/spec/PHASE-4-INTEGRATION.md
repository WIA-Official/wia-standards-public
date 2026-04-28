# WIA-COMP-019 — Phase 4: Integration

> RTOS canonical Phase 4: ecosystem integration (POSIX/DO-178C/ISO 26262/IEC 61508/62304 + RT-hypervisor + WCET + multi-core).

# WIA-COMP-019: Real-Time Operating System Specification v1.0

> **Standard ID:** WIA-COMP-019  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active



---

## A.1 Standards cross-walk

| Concern                                | Standard                                     |
|----------------------------------------|----------------------------------------------|
| POSIX real-time API                    | IEEE Std 1003.1-2024 (POSIX) §1003.1b/c/d    |
| Avionics safety certification          | RTCA DO-178C + DO-330 + DO-331 + DO-332      |
| Avionics partitioning                  | ARINC 653 Parts 1-4                           |
| Automotive safety                      | ISO 26262-1..12                              |
| Automotive software architecture       | AUTOSAR Classic + Adaptive (per AUTOSAR)     |
| Automotive (legacy)                    | OSEK/VDX OS + COM + NM                        |
| Industrial functional safety           | IEC 61508-1..7                                |
| Process-industry safety                | IEC 61511-1..3                                |
| Medical-device software                | IEC 62304 + ISO 14971 + IEC 60601-1          |
| Railway signalling                     | EN 50128 + EN 50129                          |
| Nuclear safety                         | IEC 60880 + IEC 62138                         |
| WCET analysis (academic + commercial)  | aiT (AbsInt) + Bound-T + OTAWA + RapiTime    |
| Schedulability tooling                 | CARTS + MAST + Cheddar + SymTA/S             |
| RTOS reference                         | FreeRTOS + RT-Thread + VxWorks + QNX +       |
|                                        | INTEGRITY + Zephyr + NuttX + LynxOS-178      |
| Hypervisor (real-time)                 | LynxSecure + WindRiver Helix Virt + PikeOS   |
| Time-keeping                           | IEEE 1588-2019 (PTPv2) + RFC 5905 (NTPv4)    |

## A.2 Functional-safety integration envelope

Functional-safety integration covers the per-domain safety standard
chain: ISO 26262 ASIL A/B/C/D for road-vehicle electrical /
electronic systems with the per-ASIL development-process envelope
(Part 6 software-development life-cycle; Part 8 supporting processes;
Part 9 ASIL-decomposition + verification); IEC 61508 SIL 1/2/3/4
for industrial functional safety with the per-SIL software-safety-
integrity-level envelope (Part 3 software requirements); IEC 62304
Class A/B/C for medical-device software with the per-class risk-
management envelope per ISO 14971; DO-178C DAL A/B/C/D/E for
airborne software with the per-DAL objectives table (DAL A: 71
objectives + 30 with independence) per RTCA DO-178C §10. Each
chain composes with the operator's RTOS scheduler-and-IPC envelope.

## A.3 Real-time hypervisor integration envelope

Real-time hypervisor integration covers: ARINC 653 partitioning
(spatial + temporal isolation per ARINC 653 Part 1 §2.3; major
time-frame envelope; per-partition health-monitoring envelope
per Part 1 §2.5); type-1 RT hypervisor envelope (PikeOS per SYSGO;
LynxSecure per Lynx Software; WindRiver Helix Virt; INTEGRITY
Multivisor per Green Hills; Xen RT per Xen Project's RT scheduler);
mixed-criticality co-residence envelope (HI-criticality partition
on dedicated CPU + LO-criticality partition on shared CPU pool
with the per-partition budget envelope per the SCHED_DEADLINE per
Linux + per-RTOS-equivalent); and the per-partition certification-
artefact envelope (DO-178C DAL evidence per partition; ISO 26262
ASIL evidence per partition) preserved across the hypervisor
boundary.

## A.4 Verification and validation integration envelope

Verification and validation integration covers: WCET analysis
envelope (static-analysis tool selection per the operator's tool-
qualification envelope per DO-330 + ISO 26262 §8 Tool-Qualification;
measurement-based with the documented coverage envelope); model-
checking envelope (CBMC + UPPAAL + SPIN + TLA+ for per-protocol
verification); structural-coverage envelope (statement + branch +
MC/DC per DAL-A; statement + branch per DAL-B/C; statement per
DAL-D; per the per-domain coverage policy); per-platform fault-
injection envelope (single-event-upset SEU + transient-fault per
ISO 26262 §11); regression-testing envelope (per-build full
regression suite + per-CI fast-regression suite + per-release
full-system test).

## A.5 Multi-core RTOS integration envelope

Multi-core RTOS integration covers: SMP (symmetric multi-processing)
envelope per the per-RTOS SMP scheduler (FreeRTOS-SMP + Zephyr SMP
+ VxWorks SMP + QNX SMP); AMP (asymmetric multi-processing)
envelope per per-CPU per-RTOS image with the per-IPI inter-
processor-interrupt envelope; bound-multiprocessor (BMP) envelope
per the per-CPU partitioned schedule with cross-CPU mutex / IPC
envelope; cache-partitioning envelope (Intel CAT per Cache
Allocation Technology; ARM cache-stash/cache-partition extensions;
per-platform memory-bandwidth-allocation MBA per Intel) for cache-
related interference-bound preservation; lock-free / wait-free
data-structure envelope per Herlihy 1991 + Michael + Scott 1996
for high-contention shared-data paths.

## A.6 References

- IEEE Std 1003.1-2024: POSIX (Portable Operating System Interface)
- RTCA DO-178C: Software Considerations in Airborne Systems
- RTCA DO-330: Software Tool Qualification Considerations
- ARINC 653 Parts 1-4: Avionics Application Software Standard Interface
- ISO 26262-1..12: Road vehicles — Functional safety
- IEC 61508-1..7: Functional safety of E/E/PE systems
- IEC 61511-1..3: Functional safety — Safety instrumented systems
- IEC 62304: Medical device software — Life cycle processes
- ISO 14971: Application of risk management to medical devices
- EN 50128: Railway applications — Software for railway control
- IEC 60880: Nuclear power plants — Software for safety systems
- AUTOSAR Classic Platform Release Specification
- AUTOSAR Adaptive Platform Release Specification
- OSEK/VDX OS + COM + NM
- IEEE 1588-2019: Precision Time Protocol (PTPv2)
- RFC 5905: Network Time Protocol Version 4 (NTPv4)
- Liu + Layland 1973: Scheduling Algorithms for Multiprogramming in a Hard-Real-Time Environment
- Sha + Rajkumar + Lehoczky 1990: Priority Inheritance Protocols
- Vestal 2007: Preemptive Scheduling of Multi-criticality Systems
- Joseph + Pandya 1986: Finding Response Times in a Real-Time System
- Baruah + Mok + Rosier 1990: Preemptively Scheduling Hard-Real-Time Sporadic Tasks on One Processor
- Bini + Buttazzo 2003: Hyperbolic Bound for Rate Monotonic Schedulability
- Abeni + Buttazzo 1998: Integrating Multimedia Applications in Hard Real-Time Systems
- Burns + Davis 2017: Mixed Criticality Systems — A Review


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
