# WIA-CITY-020 — Phase 4: Integration

> Smart-water canonical Phase 4: ecosystem integration (ISO 24511/24512/24516 + AWWA + IWA + IEC 62443 + EPA SDWA + EU 2020/2184).

# WIA-CITY-020: Smart Water Management Standard v1.0

## Executive Summary

The WIA-CITY-020 standard defines requirements for intelligent lighting systems in smart city environments. This standard enables 60-80% energy savings through LED technology, occupancy sensing, daylight harvesting, and AI-driven optimization.

**Philosophy**: 弘益人間 (Hongik Ingan) - Benefit All Humanity



---

## A.1 Standards cross-walk

| Concern                                | Standard                                     |
|----------------------------------------|----------------------------------------------|
| Drinking-water service activities      | ISO 24512:2007                                |
| Wastewater service activities          | ISO 24511:2007                                |
| Water-utility asset management         | ISO 24516 series + ISO 55000 + 55001 + 55002  |
| Water-loss management                  | IWA Water Loss Specialist Group + AWWA M36    |
| Drinking-water quality                 | WHO Guidelines for Drinking-water Quality 4ed |
| US drinking-water regulation           | 40 CFR 141 + 40 CFR 142 + SDWA 1974/1986/1996 |
| EU drinking-water regulation           | Directive (EU) 2020/2184                      |
| UK drinking-water regulation           | Water Industry Act 1991 + WSR 2018 + DWI      |
| Lead + copper rule                     | US EPA LCR + LCRR + LCRI                      |
| US wastewater regulation               | NPDES + Clean Water Act + 40 CFR 122-136      |
| Industrial automation cybersecurity    | IEC 62443-1..4                                |
| Standard Methods (water + wastewater)  | APHA + AWWA + WEF Standard Methods 24th ed.   |
| Hydraulic modelling                    | EPANET (US EPA) + WaterCAD + InfoWater        |
| Water utility cybersecurity            | AWWA Manual M124 + US EPA Cyber for Water     |
| Water-meter accuracy                   | AWWA C700 + C701 + C702 + C704 + C710 + C752  |
| Water-storage tank standards           | AWWA D100 + D102 + D103 + D104 + D110         |
| Water-pipeline standards               | AWWA Manual M11 + M51 + M55 + ISO 2531 (DI)   |
| Water resilience + risk                | AWIA §2013 + AWWA J100 RAMCAP for water       |
| Water + sustainability                 | ISO 14046 (Water footprint) + ISO 46001 (WEM) |

## A.2 SCADA-and-control-system integration envelope

SCADA + control-system integration covers: per-utility SCADA
platform envelope (per-vendor: Wonderware (now AVEVA System
Platform) + GE Cimplicity / iFIX + Rockwell FactoryTalk + Siemens
WinCC + Schneider Electric Aquis + Honeywell Experion + Yokogawa
CENTUM); per-platform PLC + RTU envelope per IEC 61131-3 (LD + ST
+ FBD + IL + SFC); per-platform fieldbus envelope (Modbus per
Modicon + Profibus + EtherNet/IP per ODVA + Profinet per PI +
HART + WirelessHART + ISA-100.11a per the operator's network);
per-platform OT-IT integration envelope (per-platform DMZ envelope
per IEC 62443-3-2 + Purdue Reference Model per ISA-95 zones 0-5);
per-platform OT cyber-defence envelope (per-zone segmentation +
per-conduit firewall + per-asset whitelisting per IEC 62443-4-2 +
NIST SP 800-82 Rev 3 + AWWA Manual M124).

## A.3 Sustainability-and-climate integration envelope

Sustainability + climate integration covers: per-utility water-
footprint envelope per ISO 14046 + Water Footprint Network per
Hoekstra + Hung 2002 (blue + green + grey water footprint); per-
utility water-efficiency-management envelope per ISO 46001:2019;
per-utility carbon-footprint envelope per GHG Protocol Scope 1+2+3
+ ISO 14064-1 + ISO 14067 (per-product carbon footprint); per-
utility energy-recovery envelope (per-utility hydropower-from-water-
distribution per AWWA M51 + per-utility wastewater-biogas per WEF
biosolids guidance); per-utility climate-adaptation envelope per
AWWA Manual M50 + ISO 14090 + ISO 14091; per-utility nature-based-
solutions envelope per IUCN nature-based solutions standard for
stormwater + watershed protection.

## A.4 Customer-and-billing integration envelope

Customer + billing integration covers: per-utility customer-
information system (CIS) envelope per the per-vendor CIS catalogue
(Oracle CC&B + SAP IS-U + Itineris + Cayenta + per-platform-
equivalent); per-utility per-meter billing-determinant envelope
per the per-jurisdiction tariff envelope (per-customer-class +
per-tier-block + per-fixed-vs-variable component per the per-utility
rate-design per AWWA M1 Principles of Water Rates, Fees, and
Charges); per-utility per-customer affordability envelope per the
per-utility low-income assistance envelope; per-utility per-customer
arrears + shut-off envelope per the per-jurisdiction consumer-
protection envelope; per-customer privacy envelope per the per-
jurisdiction utility-privacy regulation (per-customer consumption
data is personal data per GDPR + CCPA + per-jurisdiction-equivalent).

## A.5 Smart-city + IoT integration envelope

Smart-city + IoT integration covers: per-utility per-city smart-city
platform envelope (FIWARE per FIWARE Foundation + Open Mobility
Foundation + per-platform smart-city envelope); per-platform IoT
device-management envelope per OMA LwM2M per Open Mobile Alliance
+ Eclipse IoT projects + AWS IoT + Azure IoT Hub + GCP IoT Core
legacy + per-platform-equivalent; per-platform digital-twin envelope
per ISO/IEC 30173 + ISO 23247 + DTC Digital Twin Consortium; per-
platform IPv6-low-power envelope per RFC 4944 (6LoWPAN) + RFC 6550
(RPL); per-platform smart-grid + smart-water cross-utility envelope
(per-utility integrated-resource planning per the per-jurisdiction
energy-water nexus envelope); per-platform open-data envelope per
Open Data Charter + per-jurisdiction open-data policy.

## A.6 References

- ISO 24511:2007: Wastewater service activities
- ISO 24512:2007: Drinking-water service activities
- ISO 24516 series: Asset management of water-utility infrastructure
- ISO 55000 + 55001 + 55002: Asset management
- ISO 14046: Water footprint
- ISO 46001:2019: Water efficiency management
- ISO 14064-1 + 14067: GHG inventory + product carbon footprint
- ISO 14090 + 14091: Climate-change adaptation
- ISO 22000: HACCP-aligned food/water safety management (cited via Bonn Charter)
- IEC 62443-1..4: Industrial automation + control system cybersecurity
- IEC 61131-3: Programming languages for PLCs
- WHO Guidelines for Drinking-water Quality, 4th edition
- WHO Sanitation Safety Planning + Bonn Charter for Safe Drinking-water
- US Safe Drinking Water Act (SDWA) 1974 / 1986 / 1996 amendments
- 40 CFR 141 + 142 + 143: National Primary + Secondary Drinking Water Regulations
- US Clean Water Act (CWA) + 40 CFR 122-136: NPDES + analytical methods
- US AWIA 2018 §2013: Risk-resilience assessment + emergency response
- EU Directive (EU) 2020/2184: Quality of water for human consumption
- UK Water Industry Act 1991 + Water Supply (Water Quality) Regulations 2018
- AWWA Manuals M1 + M6 + M9 + M11 + M14 + M22 + M32 + M33 + M36 + M37 + M46 + M50 + M51 + M53 + M55 + M124
- AWWA Standards C700 / C701 / C702 / C704 / C710 / C750 / C752 (meters); D100 / D102 / D103 / D104 / D110 (tanks)
- AWWA J100 RAMCAP for water + wastewater
- Standard Methods for the Examination of Water and Wastewater (APHA + AWWA + WEF) 24th ed.
- IWA Water Loss Specialist Group: water-balance + ILI methodology
- US EPA EPANET: hydraulic + water-quality network simulation
- NASSCO PACP: Pipeline Assessment Certification Program
- ASTM F1216: CIPP (cured-in-place pipe) rehabilitation
- 40 CFR Part 503: Standards for the Use or Disposal of Sewage Sludge
- ISA-95 + ISA-88: Enterprise-control system integration + batch control
- NIST SP 800-82 Rev 3: Guide to OT Security
- DTC Digital Twin Consortium: Capabilities Periodic Table
- FIWARE: NGSI-LD + IoT smart data models


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
