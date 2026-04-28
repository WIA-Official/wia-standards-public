# WIA-MED-026 — Phase 1: Data Format

> Rehabilitation-device canonical Phase 1: device-record + therapy-session + sensor-stream + safety-envelope + clinical-evidence envelopes.


## A.1 Device-record envelope

The Phase 1 envelope groups rehabilitation devices by therapeutic
class: robotic upper-limb (per IEC 80601-2-78:2019 robotic
rehabilitation requirements; per the Lokomat / Armeo / InMotion /
Bi-Manu-Track / Amadeo platform reference); robotic lower-limb +
gait-training (per Lokomat per Hocoma; per ZeroG / Andago /
G-EO + Pediatric anti-gravity; per CARMINA / KineAssist gait
trainer); therapeutic exoskeleton (per ReWalk / Ekso Bionics /
Indego / Atalante / Phoenix per the per-platform IEC 80601-2-78
+ FDA 21 CFR 890.3480 powered-exoskeleton clearance envelope);
VR/AR rehabilitation (per ReHab Choice + Rehametrics + MIRA Rehab
+ Neuro Rehab VR + per-platform VR-cybersickness-protocol per
IEEE Std 3079-2020); FES (Functional Electrical Stimulation) per
Bioness + Saebo + Restorative Therapies + per-platform IEC 60601-
2-10 nerve-and-muscle stimulator + ISO 14708-3 implantable
neuromodulator envelope; BCI (Brain-Computer Interface) per
NeuroSky / OpenBCI / Blackrock + per-platform IEC 60601-2-26 EEG
envelope; sensor-based motion analysis per Vicon + OptiTrack +
Xsens + per-platform IMU + per-marker camera + per-platform
markerless ML envelope per ISO/IEEE 11073-10406. Each device
record carries: device identifier (UDI per FDA UDI rule per 21
CFR 801 Subpart B + per-EU MDR Article 27 UDI envelope), per-
device class envelope (FDA Class I/II/III per 21 CFR 860.3; EU
MDR Class I/IIa/IIb/III per Regulation 2017/745 Annex VIII;
Health Canada Class I/II/III/IV per Medical Devices Regulations
SOR/98-282; per-jurisdiction-equivalent), per-device firmware-
version envelope, per-device clinical-evidence envelope (linking
to §A.5), and the per-device audit envelope tied to the operating
clinical site.

## A.2 Therapy-session envelope

Therapy-session envelopes carry: session identifier (UUID v7 per
RFC 9562), per-patient pseudonymous reference per the operator's
de-identification envelope per HIPAA Safe Harbor + GDPR Article
26 pseudonymisation, per-session clinician reference, per-session
device reference (linking to §A.1), per-session prescription
envelope per the per-patient rehabilitation plan per ICF
(International Classification of Functioning, Disability and
Health) per WHO ICF + ICD-11 per WHO FIC, per-session protocol
envelope (per-platform protocol-id + per-platform parameter-set
per the device manufacturer's IFU), per-session duration envelope,
per-session outcome envelope (per-protocol primary outcome +
secondary outcomes per the per-condition outcome-measure catalogue
— Fugl-Meyer Assessment for stroke per Fugl-Meyer 1975; Berg
Balance Scale per Berg 1989; ARAT per Lyle 1981; Box-and-Block
Test per Mathiowetz 1985; 6-minute walk per Enright + Sherrill
1998), and the per-session audit envelope.

## A.3 Sensor-stream envelope

Sensor-stream envelopes catalogue per-channel real-time data:
EMG (electromyography) per ISEK SENIAM recommendations + ISO
12107 sampling + per-channel filter envelope (typical 20-500 Hz
band-pass + 50/60 Hz notch); kinematic per-marker position +
velocity + acceleration per ISB Standard for Reporting Joint
Motion + per-platform 3D-tracking envelope; ground-reaction
force per AMTI / Bertec / Kistler force-plate envelope per ISO
14945; pressure per insole-array per ISO 7250 + per-platform
in-shoe sensor envelope; electrical-stimulation per FES envelope
(per-channel pulse-width + amplitude + frequency per IEC 60601-
2-10); EEG per IEC 60601-2-26 + per-platform 10-20 / 10-10 / 10-5
electrode-placement system per Jasper 1958 + Klem 1999;
physiological per heart-rate + RR-interval per ANSI/AAMI EC57 +
SpO2 per ISO 80601-2-61 + per-platform respiratory + skin-
temperature envelope. Per-channel sampling-rate envelope (typical
1-10 kHz for EMG; 60-500 Hz for kinematics; 200-2000 Hz for force);
per-channel calibration envelope per ISO/IEC 17025 traceability.

## A.4 Safety-envelope record

Safety-envelope records describe per-device per-session safety
parameters: per-device range-of-motion limits (per-joint anatomical
limit per the per-device manufacturer's IFU + per-patient
prescription envelope); per-device maximum force / torque /
velocity envelope (per the per-device IEC 80601-2-78 verification
envelope); per-device emergency-stop envelope (per ISO 13850
+ per-device redundant-channel envelope); per-device fault-
detection envelope (per-device Type B / Type BF / Type CF
applied-part envelope per IEC 60601-1 §8); per-device patient-
weight + patient-height range envelope; per-device per-protocol
contraindication envelope (per the per-protocol exclusion-criteria
envelope per the manufacturer's IFU); per-session adverse-event
envelope (per-event severity per CTCAE per US NCI + per-event
relationship-to-device per FDA MedDRA + per-jurisdiction-equivalent).

## A.5 Clinical-evidence envelope

Clinical-evidence envelopes carry per-device evidence: per-device
pivotal-trial envelope (per-trial protocol per ISPE Good Clinical
Practice + ICH E6(R3) + ISO 14155:2020 clinical investigation of
medical devices for human subjects + FDA IDE per 21 CFR 812);
per-device safety + performance evidence per the per-device 510(k)
+ De Novo + PMA + per-EU CE-mark technical-file envelope per MDR
Annex II + Annex III + Annex IX/X/XI; per-device post-market
clinical follow-up (PMCF per MDR Article 83 + Annex XIV Part B);
per-device outcome-measure validation envelope (per-measure
psychometric-property envelope: reliability per ICC + Cronbach
alpha; validity per content + construct + criterion; responsiveness
per Effect Size + SRM per Cohen 1988 + Liang 1990; minimal-
clinically-important-difference per Jaeschke + Singer + Guyatt
1989); per-device guidelines envelope (AHA/ASA Guidelines for
Adult Stroke Rehabilitation + AAN Practice Parameters + ESO
Stroke Guidelines + per-jurisdiction-equivalent).

## A.6 ICF + ICD coding envelope

Per-patient outcome data carries per-domain ICF coding per WHO
International Classification of Functioning, Disability and Health
(per-domain b body function + s body structure + d activity-and-
participation + e environmental factors with the per-domain
qualifier scale 0-4 indicating extent of impairment / limitation
/ restriction / barrier-or-facilitator); per-condition ICD-11
coding per WHO Foundation per the per-patient diagnosis envelope;
per-procedure CPT coding per AMA per the per-billing-jurisdiction
envelope; per-procedure HCPCS coding per CMS where applicable;
per-procedure SNOMED CT coding per SNOMED International per the
operator's clinical-vocabulary envelope. The operator's per-
patient EHR envelope follows HL7 FHIR R5 per HL7 + per-jurisdiction
patient-data privacy envelope per HIPAA + GDPR + per-jurisdiction-
equivalent.


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
