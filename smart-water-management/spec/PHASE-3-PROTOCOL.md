# WIA-CITY-020 — Phase 3: Protocol

> Smart-water canonical Phase 3: protocols (treatment + distribution + leak-detection + quality-management + wastewater + resilience).

# WIA-CITY-020: Smart Water Management Standard v1.0

## Executive Summary

The WIA-CITY-020 standard defines requirements for intelligent lighting systems in smart city environments. This standard enables 60-80% energy savings through LED technology, occupancy sensing, daylight harvesting, and AI-driven optimization.

**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity


## 6. Energy Savings

### 6.1 Required Reductions vs Baseline
- **Bronze**: 50-65%
- **Silver**: 65-75%
- **Gold**: 75-85%
- **Platinum**: 85%+



## 7. Certification

### 7.1 Process
1. Submit design documentation
2. Install and commission system
3. Operate for 90-day baseline
4. Third-party performance verification
5. Annual recertification

---

**Version**: 1.0
**Published**: 2025
**Maintained by**: WIA (World Certification Industry Association)
**Philosophy**: 弘益人間 · Benefit All Humanity

© 2025 SmileStory Inc. / WIA



---

## A.1 Treatment-process protocol

Treatment-process protocols cover: per-unit-process control envelope
per the per-utility multi-barrier treatment train per AWWA Manual
M37 + M53: coagulation-control envelope (per-jar-test optimisation
per Standard Methods 2310 + per-online streaming-current-detector
envelope); flocculation + sedimentation envelope (per-G-value
control per Camp + Stein 1943 + per-overflow-rate envelope per
AWWA M37); filtration envelope (per-bed turbidity-monitoring per
USEPA Filter Backwash Recycling Rule + per-filter-effluent
turbidity SCADA-alarm envelope); disinfection envelope (per-CT
(concentration-time) compliance envelope per USEPA SWTR + LT2ESWTR
+ Stage 1 + Stage 2 D/DBP per 40 CFR 141 Subpart H/U/V + per-
jurisdiction-equivalent; per-disinfectant residual-decay envelope
per ISO 7393); per-process control-loop envelope per ISA 95 +
ISA 88 (batch process control).

## A.2 Distribution-network operation protocol

Distribution-network operation protocols cover: per-DMA pressure-
management envelope (per-DMA Pressure-Reducing-Valve (PRV) setpoint
+ per-DMA inlet-meter envelope per IWA WLSG + AWWA M36); per-DMA
minimum-night-flow analysis envelope (per-DMA leak-detection per
the per-utility nightly minimum-night-flow profile); per-DMA
hydraulic-model calibration envelope (per-DMA pressure + flow
field-test campaign per AWWA M32 + EPANET calibration techniques
per Walski + Chase + Savic 2003); per-network water-age envelope
(per-node + per-pipe water-age tracking per EPANET water-age
analysis); per-network disinfectant-residual envelope (per-node
+ per-time disinfectant-residual prediction per EPANET-MSX
multi-species extension); per-network surge-protection envelope
per AWWA Manual M11 + ISO 4427 thrust-restraint design.

## A.3 Leak-detection-and-loss-management protocol

Leak-detection-and-loss-management protocols per IWA Water Loss
Specialist Group + AWWA M36 cover: water-balance computation per
the IWA standard water balance (system input volume = authorised
consumption + water losses + non-revenue water; ILI Infrastructure
Leakage Index = current annual real losses / unavoidable annual
real losses per IWA WLSG); per-DMA active-leak-detection envelope
(acoustic-correlator + leak-noise-logger + ground-microphone +
satellite-imagery + ML-pattern-recognition per the per-utility
detection-vendor); per-meter under-registration envelope (per-
meter accuracy degradation per AWWA M6 + per-meter age-replacement
policy per AWWA M22); per-meter unauthorised-consumption envelope
(per-meter tamper-detection envelope); per-utility apparent-loss
+ real-loss reduction envelope per the operator's per-year reduction
target.

## A.4 Water-quality-management protocol

Water-quality-management protocols cover: per-utility multi-barrier
contamination-prevention envelope per WHO Guidelines for Drinking-
water Quality (4th ed.) + per-jurisdiction Water Safety Plan per
WHO + IWA Bonn Charter; per-utility critical-control-point
identification per HACCP-adapted-for-water per Codex Alimentarius
+ ISO 22000; per-utility cross-connection-control envelope per
AWWA Manual M14 + USC FCCCHR Manual; per-utility lead + copper
control envelope per US EPA Lead and Copper Rule (LCR) + Lead
and Copper Rule Revisions (LCRR) + Lead and Copper Rule
Improvements (LCRI) + per-jurisdiction-equivalent; per-utility
PFAS monitoring per US EPA UCMR 5 + final PFAS NPDWR per 40 CFR
141 Subpart Z + per-jurisdiction-equivalent; per-utility per-
contaminant emerging-contaminant horizon envelope.

## A.5 Wastewater-and-stormwater protocol

Wastewater + stormwater protocols cover: per-utility wastewater
collection envelope per US EPA SSO + CMOM + per-jurisdiction-
equivalent (per-line condition assessment per NASSCO PACP per
NASSCO + ISO 11295 + ISO 16484 building-management overlap; per-
line CCTV inspection + per-line CIPP rehab per ASTM F1216 +
per-jurisdiction-equivalent); per-utility wastewater treatment
envelope per Standard Methods 2540 + 5210 + 4500-N + 4500-P +
NPDES per US EPA Clean Water Act + per-jurisdiction-equivalent
(BOD + COD + TSS + nitrogen + phosphorus + fecal-coliform per
the per-permit envelope); per-utility biosolids envelope per US
EPA 40 CFR Part 503 + ISO 19698 + per-jurisdiction-equivalent;
per-utility stormwater envelope per US EPA NPDES MS4 + per-
jurisdiction-equivalent (per-outfall sampling + per-event sampling
+ per-watershed BMP envelope per the per-utility stormwater plan);
per-utility CSO (combined-sewer-overflow) envelope per US EPA
CSO Control Policy + per-jurisdiction-equivalent.

## A.6 Resilience-and-emergency-response protocol

Resilience-and-emergency-response protocols cover: per-utility
emergency-response plan per America's Water Infrastructure Act
2018 (AWIA) §2013 ERP + per-jurisdiction-equivalent; per-utility
risk + resilience assessment per AWIA §2013 RRA + per-jurisdiction-
equivalent + AWWA J100 Risk Analysis and Management for Critical
Asset Protection (RAMCAP) for water utilities; per-utility
cybersecurity envelope per AWIA §2013 + per AWWA Manual M124 +
US EPA Cybersecurity for Drinking Water Utilities + IEC 62443
industrial automation cybersecurity (per-zone + per-conduit +
per-component cybersecurity); per-utility natural-disaster envelope
per ASCE-7 + ASCE-25 (earthquake / flood / hurricane / drought
per the per-utility hazard envelope); per-utility climate-
adaptation envelope per AWWA Manual M50 + ISO 14090 climate-change
adaptation; per-utility per-event mutual-aid envelope per
WARN (Water/Wastewater Agency Response Network) per AWWA + per-
jurisdiction-equivalent.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-water-management/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-water-management-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-water-management-host:1.0.0` ships every smart-water-management envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-water-management.sh` ships sample envelope generators with no
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
ecosystem. Smart-water-management deployments that follow this layering
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
`/.well-known/wia-smart-water-management-capabilities` that enumerates which
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
