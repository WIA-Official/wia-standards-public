# WIA-MED-026 — Phase 4: Integration

> Rehabilitation-device canonical Phase 4: ecosystem integration (FDA + IEC 62366 + IEC 60601 + ISO 14971 + DICOM + HL7 FHIR + ATA telerehab).


## A.1 Standards cross-walk

| Concern                                | Standard                                     |
|----------------------------------------|----------------------------------------------|
| Medical-device general safety          | IEC 60601-1 + IEC 60601-1-2 (EMC)             |
| Robotic rehabilitation devices         | IEC 80601-2-78:2019                           |
| Powered exoskeletons (FDA)             | 21 CFR 890.3480 + FDA Guidance                |
| Risk management for medical devices    | ISO 14971:2019                                |
| Usability engineering                  | IEC 62366-1:2015 + IEC TR 62366-2             |
| Medical-device software life cycle     | IEC 62304:2006/AMD1:2015                      |
| Quality management for med devices     | ISO 13485:2016                                |
| Clinical investigation                 | ISO 14155:2020                                |
| EEG (medical equipment)                | IEC 60601-2-26                                |
| Nerve and muscle stimulators           | IEC 60601-2-10                                |
| Implantable neuromodulators            | ISO 14708-3                                   |
| Force plates (biomechanics)            | ISO 14945                                     |
| EMG (clinical)                         | ISEK SENIAM Recommendations                    |
| Joint motion reporting                 | ISB Standard for Reporting Joint Motion       |
| Pulse oximetry                         | ISO 80601-2-61                                |
| FDA UDI                                | 21 CFR 801 Subpart B                          |
| EU MDR                                 | Regulation (EU) 2017/745                      |
| US Medical Device Regulation           | 21 CFR Part 820 + Part 803 + Part 812         |
| Clinical-data interchange              | HL7 FHIR R5 + DICOM PS3.x + IHE PCD profiles  |
| Clinical-vocabulary                    | SNOMED CT + ICD-11 + LOINC + ICF + CPT/HCPCS  |
| VR cybersickness                       | IEEE Std 3079-2020                            |

## A.2 FDA + EU MDR integration envelope

FDA integration covers: per-device classification per 21 CFR 860.3
(Class I / II / III); per-device pre-market pathway envelope (510(k)
per 21 CFR 807 Subpart E for Class I/II per-device with substantial
equivalence; De Novo per FDC §513(f)(2) for novel low-to-moderate
risk; PMA per 21 CFR 814 for Class III); per-device post-market
surveillance envelope per 21 CFR 822 (post-approval studies) +
21 CFR 803 (medical-device reporting MDR + MAUDE database); per-
device QSR per 21 CFR Part 820 (transitioning to harmonisation
with ISO 13485:2016 per the FDA QMSR final rule); per-device IDE
per 21 CFR 812 for clinical investigations.

EU MDR integration covers: per-device classification per Annex VIII
(rules 1-22); per-device pre-market technical-file envelope per
Annex II + Annex III; per-device clinical-evaluation per Annex
XIV Part A; per-device post-market clinical-follow-up per Annex
XIV Part B + Article 83; per-device post-market surveillance per
Article 83-86; per-device vigilance reporting per Article 87-90
+ EUDAMED incident reporting; per-device unique device
identification per Article 27 + EUDAMED UDI database; per-device
notified-body interaction per Article 52-54 + Annex VII.

## A.3 Clinical-data integration envelope

Clinical-data integration covers: per-EHR HL7 FHIR R5 envelope per
HL7 FHIR + per-EHR vendor-specific implementation envelope (Epic
+ Cerner Oracle Health + Allscripts + athenahealth + per-platform-
equivalent); per-imaging DICOM PS3.x envelope per the per-device
imaging-output envelope per IHE PCD (Patient Care Devices) profile
per the per-platform device-data integration envelope; per-clinical
ICD-11 + ICF + SNOMED CT + LOINC + CPT/HCPCS coding envelope per
the per-jurisdiction clinical-vocabulary policy; per-trial CDISC
SDTM per CDISC + ADaM per CDISC for clinical-trial data submission
per FDA + EMA + PMDA + Health Canada eSubmission gateway envelope.

## A.4 Reimbursement-and-coverage integration envelope

Reimbursement-and-coverage integration covers: per-device CMS
coverage envelope per Medicare + Medicaid + per-LCD per CMS Local
Coverage Determination + per-NCD per CMS National Coverage
Determination; per-device CPT-code envelope (per-AMA RUC valuation
+ per-jurisdiction commercial-payer policy); per-device HCPCS
envelope (per-DMEPOS Local Coverage Determination); per-EU per-
member-state HTA envelope (NICE in UK + IQWiG in Germany + HAS in
France + AIFA in Italy + per-member-state-equivalent); per-Asia-
Pacific PMDA + KNHIS + NHIA + Medicare-Australia + per-
jurisdiction-equivalent reimbursement envelope; per-OECD WHO
INAHTA Health Technology Assessment envelope.

## A.5 Telerehabilitation regulatory integration envelope

Telerehabilitation regulatory integration covers: per-jurisdiction
telehealth licensure envelope (per-US-state Interstate Medical
Licensure Compact + Physical Therapy Compact + per-discipline
compact + Federal-State telehealth-parity policy; per-EU cross-
border-care Directive 2011/24/EU; per-jurisdiction-equivalent);
per-platform privacy envelope per HIPAA Privacy + Security Rule
+ HITECH + per-jurisdiction-equivalent (BAA per HIPAA + DPA per
GDPR + per-jurisdiction-equivalent); per-platform encryption envelope
per NIST SP 800-66 Rev 2 + per-platform FIPS 140-3 + per-platform
HITRUST CSF; per-platform reimbursement envelope per CMS + per-
jurisdiction-equivalent telehealth-parity policy.

## A.6 References

- IEC 60601-1: Medical electrical equipment — Part 1: General requirements
- IEC 60601-1-2: Electromagnetic disturbances
- IEC 80601-2-78:2019: Particular requirements for medical robots for rehabilitation
- IEC 60601-2-10: Nerve and muscle stimulators
- IEC 60601-2-26: Electroencephalographs
- IEC 62304: Medical device software — Life cycle processes
- IEC 62366-1: Application of usability engineering to medical devices
- ISO 13485:2016: Medical devices — Quality management
- ISO 14155:2020: Clinical investigation of medical devices for human subjects
- ISO 14971:2019: Application of risk management to medical devices
- ISO 14708-3: Implantable neurological stimulators
- ISO 14945: Force plates (biomechanics)
- ISO 80601-2-61: Pulse oximeter equipment
- ISO 13850: Emergency stop function
- ISO/IEC 17025: Testing + calibration laboratory competence
- IEEE Std 3079-2020: Head-mounted display device for VR
- ISEK SENIAM: Recommendations for surface EMG
- ISB Standard for Reporting Joint Motion
- 21 CFR Part 820: Quality System Regulation
- 21 CFR Part 803: Medical Device Reporting (MDR)
- 21 CFR Part 812: Investigational Device Exemptions
- 21 CFR 890.3480: Powered exoskeleton
- Regulation (EU) 2017/745: Medical Devices Regulation (MDR)
- ICD-11 + ICF: WHO international classifications
- HL7 FHIR R5: Fast Healthcare Interoperability Resources
- DICOM PS3.x: Digital Imaging and Communications in Medicine
- IHE PCD: Integrating the Healthcare Enterprise — Patient Care Devices
- SNOMED CT + LOINC + CPT/HCPCS: clinical vocabularies
- CDISC SDTM + ADaM: Clinical Data Interchange Standards Consortium
- HIPAA Privacy Rule + Security Rule + HITECH Act
- ATA Telerehabilitation Special Interest Group: Practice Guidelines
- APTA Telehealth Practice Guideline
- AHA/ASA Guidelines for Adult Stroke Rehabilitation


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
