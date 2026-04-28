# WIA-MED-026 — Phase 3: Protocol

> Rehabilitation-device canonical Phase 3: protocols (control + safety + telerehab + biofeedback + adverse-event + maintenance).


## A.1 Control-algorithm protocol

Control-algorithm protocols cover: impedance-control envelope per
Hogan 1985 (per-joint stiffness + damping + inertia parameter
envelope; per-joint adjustable-impedance per the rehabilitation
phase); admittance-control envelope per the inverse paradigm;
assist-as-needed (AAN) envelope per Reinkensmeyer + Akoner +
Ferris 2002 + Cai + Fong + Liang 2006 (per-patient adaptive
assistance based on per-trial performance); error-augmentation
envelope per Patton + Mussa-Ivaldi 2004 (per-trial deliberate
error magnification to elicit motor learning); per-protocol
randomised-versus-blocked-practice envelope per Shea + Morgan
1979; per-protocol task-specific repetitive-practice envelope
per Bayona + Bitensky + Salter + Teasell 2005 + Langhorne +
Bernhardt + Kwakkel 2011.

## A.2 Safety-monitoring protocol

Safety-monitoring protocols cover: per-device redundant-sensor
envelope (per-joint primary + secondary position sensor with the
operator's discrepancy-detection threshold per IEC 80601-2-78);
per-device watchdog envelope (per-loop deadline-monitoring per
the per-loop maximum-allowable-jitter envelope); per-device
patient-engagement detector envelope (per-channel EMG / per-
channel force / per-channel pressure detecting non-cooperative
condition); per-device fault-tolerance envelope per IEC 60601-1
§4.7 essential-performance preservation; per-device emergency-stop
chain per ISO 13850 with the per-platform tested-redundancy
envelope; per-device patient-restraint envelope per the operator's
positioning policy; per-device clinician-presence envelope (per-
session per-clinician supervision requirement per the operator's
clinical policy).

## A.3 Telerehabilitation protocol

Telerehabilitation protocols cover: per-session telehealth platform
envelope per the operator's HIPAA-compliant telehealth platform
+ per-jurisdiction-equivalent (per-platform end-to-end encryption
+ per-platform BAA / DPA); per-session video + audio envelope per
WebRTC per RFC 8825-8839 + SRTP per RFC 3711 with the per-session
mTLS envelope; per-session remote-actuation envelope (per-platform
clinician-side control of the per-patient device with the per-
session round-trip-latency budget envelope; per-session safety-
fallback envelope on per-session disconnect); per-session per-
patient pseudonymisation envelope per HIPAA + GDPR; per-session
ATA Telerehabilitation Special Interest Group + APTA Telehealth
Practice Guideline + per-jurisdiction-equivalent compliance.

## A.4 Biofeedback protocol

Biofeedback protocols cover: per-channel feedback modality envelope
(visual feedback per per-platform display; auditory feedback per
per-platform sonification per Pawluk + Adams + Kitada 2015;
haptic feedback per per-platform vibrotactile / kinesthetic actuator
per Sigrist + Rauter + Riener + Wolf 2013); per-channel per-task
mapping envelope (per-task target-state + per-task error-signal
per the per-protocol biofeedback-design envelope); per-channel
adaptive-difficulty envelope (per-trial per-patient performance
adjustment per Choi + Vining + Reinkensmeyer + Bachman 2011);
per-session motivational-engagement envelope (per-session
gamification per per-platform game-design per Lohse + Hilderman +
Cheung + Tatla + Van der Loos 2014; per-session per-patient
intrinsic-motivation envelope per Self-Determination Theory per
Deci + Ryan 1985).

## A.5 Adverse-event response protocol

Adverse-event response protocols cover: per-event detection envelope
(per-protocol adverse-event indicator catalogue per the per-protocol
IFU + per-event clinician-observation channel); per-event
classification envelope (CTCAE Grade 1-5 per US NCI + per-event
ANSI/AAMI/ISO 14971 risk severity); per-event immediate-response
envelope (per-event session-abort + per-event medical-officer
notification + per-event device-quarantine + per-event patient-
care escalation); per-event root-cause investigation envelope per
ISO 14971 + per-event CAPA per ISO 13485:2016 §8.5.2 + 21 CFR 820
Subpart J; per-event regulatory-reporting envelope per §A.5 of
Phase 2; per-event lessons-learned envelope feeding into the per-
device IFU + per-protocol revision policy.

## A.6 Maintenance-and-calibration protocol

Maintenance-and-calibration protocols cover: per-device PM
envelope per ASTM E2655 + per-device manufacturer-recommended
schedule with the operator's per-device adjustment per device-
event-log analysis; per-device calibration envelope per ISO/IEC
17025 traceable to NIST or PTB primary standards (per-channel
calibration-cert + per-channel calibration-due-date + per-channel
as-found / as-left envelope per ILAC-G24); per-device functional-
verification envelope per IEC 80601-2-78 + per-device manufacturer
acceptance-test procedure; per-device software-update envelope
per IEC 62304 software-life-cycle + per-update validation envelope
per the operator's change-control policy; per-device patient-
contact-surface cleaning + disinfection envelope per Spaulding
classification + per-jurisdiction infection-prevention envelope.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/rehabilitation-device/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-rehabilitation-device-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/rehabilitation-device-host:1.0.0` ships every rehabilitation-device envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/rehabilitation-device.sh` ships sample envelope generators with no
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
ecosystem. Rehabilitation-device deployments that follow this layering
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
`/.well-known/wia-rehabilitation-device-capabilities` that enumerates which
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
