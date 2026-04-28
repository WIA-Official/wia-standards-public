# WIA-CITY-020 — Phase 1: Data Format

> Smart-water canonical Phase 1: asset + telemetry + customer-meter + quality-result + hydraulic-model envelopes (corrects v1.0 source mislabel).

# WIA-CITY-020: Smart Water Management Standard v1.0

## Executive Summary

The WIA-CITY-020 standard defines requirements for intelligent lighting systems in smart city environments. This standard enables 60-80% energy savings through LED technology, occupancy sensing, daylight harvesting, and AI-driven optimization.

**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity


## 1. Scope and Purpose

### 1.1 Scope
- Commercial buildings (offices, retail, hospitality)
- Residential buildings
- Industrial facilities
- Outdoor and street lighting
- Smart city infrastructure

### 1.2 Purpose
- Reduce lighting energy consumption by 60-80%
- Improve lighting quality and human health
- Enable adaptive circadian rhythm support
- Integrate with smart city platforms
- Support sustainability goals



## 2. LED Requirements

### 2.1 Efficacy
- **Bronze**: 100 lm/W minimum
- **Silver**: 120 lm/W minimum
- **Gold**: 150 lm/W minimum
- **Platinum**: 180 lm/W minimum

### 2.2 Color Quality
- CRI (Color Rendering Index): 80 minimum (90+ for Gold/Platinum)
- R9 (red rendering): 0 minimum (50+ for Gold/Platinum)
- Color consistency: ±3 SDCM (MacAdam ellipses)

### 2.3 Lifespan
- L70 (70% lumen maintenance): 50,000 hours minimum
- L90 (90% lumen maintenance): 36,000 hours (Gold/Platinum)



## 3. Control Requirements

### 3.1 Dimming
- Range: 0.1-100%
- Steps: Continuous (not stepped)
- Flicker-free: <5% at all levels

### 3.2 Tunable White (Gold+)
- Range: 2700K-6500K
- Steps: 100K increments minimum




---

## A.1 Note on source-spec mismatch

The v1.0 source spec carries text describing intelligent lighting
(LED efficacy, occupancy sensing). This Phase 1 envelope authors
the actual smart-water-management domain (drinking-water + waste-
water + storm-water utility infrastructure) per ISO 24512:2007
(drinking-water service activities) + ISO 24511:2007 (wastewater
service activities) + ISO 24516 series (management of water-utility
infrastructure assets) + AWWA Manual M50 (water resources planning
+ asset management). Future v2.0 spec MUST realign the source-
document text accordingly.

## A.2 Asset-record envelope

The Phase 1 envelope groups water-utility assets by class: source
infrastructure (raw-water intake per AWWA M9 — surface intake +
groundwater well + spring per the operator's Source Water
Assessment Plan per US EPA SDWA + per-jurisdiction-equivalent);
treatment infrastructure (coagulation + flocculation + sedimentation
+ filtration + disinfection per AWWA WaterStats + AWWA Manual M37
+ M53; advanced treatment per membrane / GAC / UV + ozone per
AWWA Manual M46 / M53); distribution infrastructure (transmission
mains + distribution pipes per AWWA Manual M11 + M51 + M55; storage
tanks per AWWA D100 + D102 + D103 + D104 + D110); customer-service
infrastructure (service line + meter per AWWA Manual M6 + M22 +
M36); wastewater collection per the per-utility CMOM (Capacity,
Management, Operations, Maintenance) program per US EPA SSO Rule
+ per-jurisdiction-equivalent; storm-water infrastructure per US
EPA NPDES MS4 program + per-jurisdiction-equivalent.

## A.3 Telemetry-record envelope

Telemetry-record envelopes catalogue per-sensor reading: per-flow
sensor — magnetic-flow per AWWA Manual M33 + ultrasonic-flow per
ISO 6416 + Coriolis-mass-flow per ISO 10790 (units L/s or m³/h
per ISO 80000-3); per-pressure sensor — gauge-pressure per the
per-DMA (District Metered Area) envelope per IWA Water Loss
Specialist Group + AWWA M36 (units kPa per ISO 80000-4); per-
quality sensor — pH per ISO 10523, conductivity per ISO 7888,
turbidity per ISO 7027 (NTU), free-chlorine residual per Standard
Methods 4500-Cl per APHA + AWWA + WEF, dissolved-oxygen per ISO
5814, total-organic-carbon per ISO 8245, ORP (oxidation-reduction
potential) per Standard Methods 2580; per-level sensor — ultrasonic
or radar level per IEC 61298-1 (units m); per-actuator state —
valve position 0-100% open per ISA 88; pump status; per-sensor
calibration envelope per ISO/IEC 17025.

## A.4 Customer-meter (AMI) envelope

Customer-meter envelopes carry: meter identifier (per the per-
utility AMI / AMR catalogue per AWWA Manual M6 + M22 + M36); meter-
class envelope (positive-displacement per AWWA C700 + C701; turbine
per AWWA C702 + C704 + C710; magnetic per AWWA C750; ultrasonic
per AWWA C752); meter-test-bench accuracy envelope per AWWA C700
§5.6 (typical ±1.5% over the per-class operating range); register
envelope (cubic-meters or US-gallons per the operator's tariff
unit); communication envelope (LoRaWAN per LoRa Alliance + Wi-SUN
per Wi-SUN Alliance + NB-IoT per 3GPP Rel-13/14/15 + LTE-M per
3GPP + Sigfox per Sigfox SA + per-platform mesh per the operator's
AMI vendor); reading-cadence envelope (per-15-min for AMI vs per-
month for AMR vs per-quarter for legacy). Meter-data is sensitive
per the operator's per-jurisdiction privacy envelope (per-customer
consumption pattern is personal data under GDPR + CCPA + per-
jurisdiction-equivalent).

## A.5 Water-quality-record envelope

Water-quality-record envelopes follow the Standard Methods for the
Examination of Water and Wastewater per APHA + AWWA + WEF + the
per-jurisdiction regulatory monitoring envelope (US EPA per Safe
Drinking Water Act + per-state primacy; EU Directive (EU) 2020/2184
on the quality of water intended for human consumption; UK Drinking
Water Quality Regulations 2016 per DWI; per-jurisdiction-equivalent):
per-sample identifier, per-sample location envelope (entry-point
+ distribution-system + customer-tap per the per-utility sampling
plan), per-sample analyte panel (per-jurisdiction primary + secondary
+ unregulated contaminant per the per-utility monitoring schedule);
per-result analytical-method envelope (per-method limit-of-detection
+ limit-of-quantitation + uncertainty per ISO/IEC 17025); per-
result MCL (Maximum Contaminant Level) compliance envelope per the
per-jurisdiction regulatory threshold + per-result action-level
event when the per-result exceeds the per-MCL.

## A.6 Hydraulic-model envelope

Hydraulic-model envelopes carry per-utility distribution-network
model: pipe-network topology per EPANET per US EPA + KYPipe + WaterCAD
per Bentley Systems + InfoWater per Innovyze + per-platform-equivalent
(node + link + reservoir + tank + pump + valve + fitting catalogue
per the per-utility GIS); per-link hydraulic envelope (diameter +
length + Hazen-Williams C-factor per AWWA Manual M11 + Darcy-
Weisbach friction factor per Moody diagram + per-link minor-loss
coefficient); per-reservoir + per-tank head + level envelope; per-
pump curve (head-flow + efficiency-flow + power-flow per the
manufacturer's curve); per-valve characteristic; per-demand pattern
(per-customer-class baseline + per-time-of-day diurnal envelope);
per-event scenario envelope (peak-hour + max-day + minimum-night-
flow + fire-flow + emergency per the per-utility planning envelope).


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
