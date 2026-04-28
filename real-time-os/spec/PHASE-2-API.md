# WIA-COMP-019 — Phase 2: API Interface

> RTOS canonical Phase 2: API surface (POSIX 1003.1b + configuration + health + trace + telemetry + audit).

# WIA-COMP-019: Real-Time Operating System Specification v1.0

> **Standard ID:** WIA-COMP-019  
> **Version:** 1.0.0  
> **Published:** 2025-12-27  
> **Status:** Active


## 4. Safety and Certification

### 4.1 Standards
- DO-178C (avionics)
- IEC 61508 (industrial)
- ISO 26262 (automotive)
- MISRA-C compliance

---

**弘익인간 (Benefit All Humanity)**  
*© 2025 SmileStory Inc. / WIA - MIT License*



---

## A.1 POSIX 1003.1b real-time API surface

```c
/* Scheduling */
int sched_setscheduler(pid_t, int policy, const struct sched_param*);
int sched_setparam   (pid_t, const struct sched_param*);
int sched_yield      (void);
int sched_get_priority_min(int policy);
int sched_get_priority_max(int policy);

/* Timers */
int timer_create(clockid_t, struct sigevent*, timer_t*);
int timer_settime(timer_t, int flags, const struct itimerspec*,
                  struct itimerspec*);
int clock_nanosleep(clockid_t, int flags, const struct timespec*,
                    struct timespec*);

/* Semaphores + message queues */
int sem_init    (sem_t*, int pshared, unsigned value);
int sem_wait    (sem_t*);
int sem_trywait (sem_t*);
int sem_post    (sem_t*);
mqd_t mq_open   (const char *name, int oflag, mode_t mode,
                 struct mq_attr *attr);
ssize_t mq_timedsend(mqd_t, const char*, size_t, unsigned,
                     const struct timespec*);

/* Memory + signals */
int mlockall   (int flags);
int sigaction  (int signum, const struct sigaction*, struct sigaction*);
```

The full POSIX 1003.1b real-time set is mandated for systems
claiming POSIX-RT conformance per IEEE Std 1003.1-2024. Per-RTOS
extensions (FreeRTOS `xTask*`; RT-Thread `rt_thread_*`; VxWorks
`taskSpawn`/`semBCreate`; QNX `MsgSend`/`MsgReceive`; Zephyr
`k_thread_*`/`k_msgq_*`) MAY be exposed alongside the POSIX-RT
surface so portable applications retain the option to compile
against POSIX-RT without per-RTOS adapters.

## A.2 Configuration-management API

A configuration-management surface MUST expose: per-task scheduling
class read + write; per-task priority read + write within the
per-task class envelope (a high-priority task cannot be demoted by
a lower-credential operator); per-task affinity read + write
(CPU set per `cpu_set_t` per Linux + per-RTOS-equivalent); per-
mutex priority-inheritance envelope; per-timer relative + absolute
arming; per-clock resolution discovery; per-IPC-channel capacity
+ flow-control envelope; per-memory-region permission read + write
within the operator's safety-case envelope. The configuration
surface is gated by the per-operator capability envelope (POSIX
capabilities per Linux capabilities(7); per-platform fine-grained
capability envelope on QNX + INTEGRITY).

## A.3 Health-and-state introspection API

A health-and-state introspection API MUST expose: per-task running
state (READY / RUNNING / BLOCKED / SUSPENDED / DELETED + per-RTOS
extension states); per-task last-runtime envelope (per-task CPU
time accumulated per `clock_gettime(CLOCK_THREAD_CPUTIME_ID,...)`);
per-task stack-high-water-mark envelope (stack-canary + per-task
high-water introspection per FreeRTOS `uxTaskGetStackHighWaterMark`
+ Zephyr `k_thread_stack_space_get` + per-RTOS-equivalent);
per-IPC-channel queue-depth envelope; per-mutex contention-event
envelope; per-system tickless idle envelope; and per-system memory
fragmentation envelope per the operator's allocator.

## A.4 Trace-event ingest API

A trace-event ingest API MUST expose: per-event ring-buffer ingest
(SystemView per Segger; Tracealyzer per Percepio; LTTng per ETSI
GS NFV-IFA 030; ftrace per Linux kernel; per-ETM/ITM trace per ARM
CoreSight); per-event filter envelope (event-class subscription +
per-task-class filter + per-CPU-mask filter); per-event sampling-
rate envelope (full-fidelity vs sampled-1-of-N per the operator's
trace-storage budget); per-event time-base envelope (CPU-tick vs
host-monotonic-clock envelope); and per-event export envelope
(per-tool format with the operator's per-tool integration policy).

## A.5 Telemetry WebSocket

The state-stream WebSocket multiplexes per-system events: per-task
state-transition events; per-task deadline-miss events with the
per-event proximity-to-deadline envelope (early-warning at 90% of
deadline; deadline-miss escalation event); per-IPC-channel queue-
fill events; per-mutex priority-inversion events; per-interrupt
unexpected-frequency events; per-memory-region access-violation
events; and per-clock-source jitter-threshold-crossing events.
Subscribers can filter by task-id, IPC-id, severity-class. Rate
limits: 5000 req/h authenticated; 50000 req/h trusted-partner.
WebSocket subscriptions are bounded at 100 simultaneous per
credential.

## A.6 Audit endpoint envelope

`GET /audit/{id}` returns the immutable audit trail: per-task
creation, per-task scheduling-class change, per-task priority
change, per-mutex protocol-change, per-clock-source change,
per-trace-buffer reset, per-credential change, every deadline-miss
event with the safety-case impact envelope. The audit-trail
integrity is anchored into a Merkle tree per-system; the root is
committed to the operator's safety-case archival record per
ISO 26262 §6 + DO-178C §11.20 + IEC 61508 §7.7 as applicable.


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
