# WIA-nuclear-energy PHASE 3 — PROTOCOL Specification

**Standard:** WIA-nuclear-energy
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
nuclear-energy programme: regulator authorisation, safeguards
agreement compliance, defence-in-depth governance, fuel-cycle
custody, radiation-protection governance, environmental
surveillance, emergency preparedness, cybersecurity for safety-
related systems, decommissioning custody, and end-state release.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management)
- ISO 45001:2018 (occupational health and safety management)
- ISO 19443:2018 (quality management for nuclear-supplier organisations)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- IAEA Safety Standards Series (SSR-2/1, SSR-2/2, GSR Part 3, etc.;
  cited normatively for terminology)
- IAEA Safeguards Agreements and Additional Protocol
- ICRP Publications 60, 103, and successors

---

## §1 Regulator Authorisation

A nuclear-energy programme MAY claim conformance to WIA-nuclear-energy
only after the national nuclear regulator has issued a valid licence
or operating authorisation for each plant the programme operates.
Plant licensing follows the regulator's published process and is
recorded against the plant. The API enforces the authorisation
requirement at phase-transition endpoints (PHASE-2 §3): plants
without a current authorisation cannot transition to `operating`.

Operating licences are amended through the regulator's amendment
procedure; amendments are content-addressed and the prior
authorisation remains addressable as the historical regulatory
state.

## §2 Safeguards Agreement Compliance

Plants subject to safeguards agreements (typically all civil plants
in non-nuclear-weapon states under IAEA Comprehensive Safeguards
and the Additional Protocol) submit material balance reports and
host inspector visits according to the agreement's schedule. The
operating programme records the safeguards agreement reference,
the inspectorate point of contact, and the inspection cadence
against each plant.

Safeguards-relevant records (fuel assembly serial numbers,
spent-fuel inventory snapshots, material balance areas) are
exposed to the inspectorate via the dedicated endpoints in PHASE-2
§4 and §8 under the inspectorate's client certificate. Other
clients receive `403 Forbidden` with type
`urn:wia:nuclear-energy:safeguards-scope`.

## §3 Defence-in-Depth Governance

Plants operate under defence-in-depth: independent layers of
prevention, detection, and mitigation that protect public and
worker safety even when individual layers fail. The operating
organisation documents its defence-in-depth architecture in the
plant's quality dossier and references it from each safety-event
record (PHASE-1 §10). Defence-in-depth modifications (new safety
systems, modified credit for existing systems) require regulator
approval before implementation.

## §4 Fuel-Cycle Custody

Each fuel assembly carries a custody chain from fabrication through
final disposition. Custody events emit records that link to the
PHASE-1 §4 fuel-assembly record and to the operating programme's
shipment, receipt, and inventory records. The chain is auditable
end-to-end so that safeguards inspectors and regulators can
reconstruct the assembly's history without recourse to vendor or
operator-internal systems.

## §5 Radiation-Protection Governance

Worker dose limits follow the regulator's adoption of ICRP
Publication 103 (or its successor). The operating organisation
records its dose-management programme: the dose limits it applies,
the as-low-as-reasonably-achievable (ALARA) practices it has adopted,
the dosimetry system it operates, and the personnel dose-ledger
review cadence.

Programmes MUST refuse to dispatch a worker whose annual cumulative
dose has reached the regulator-set limit; the API's dose-policy
breach endpoint returns `403 Forbidden` with type
`urn:wia:nuclear-energy:dose-policy-breach`.

## §6 Environmental Surveillance

Environmental-release reports (PHASE-1 §9) are filed at the cadence
the regulator requires (typically quarterly or annually) and are
verified against environmental monitoring data from on-site and
off-site monitoring stations. The operating programme records its
monitoring network in its quality dossier and republishes annual
environmental-impact reports.

Releases above regulator-defined notification thresholds trigger an
expedited report to the regulator within the regulator's required
window (typically hours to days, depending on the radionuclide and
the release pathway).

## §7 Emergency Preparedness

Plants operate under an emergency-preparedness plan that names the
on-site emergency director, the off-site authority, the
notification chain, the public-protection actions (sheltering,
evacuation, prophylaxis), and the exercise schedule (table-top,
walk-through, full-scale). Emergency exercises are recorded against
the plant; regulators audit the exercises during routine
inspections.

## §8 Cybersecurity for Safety-Related Systems

Safety-related instrumentation and control systems are isolated
from external networks per the regulator's defence-in-depth
cybersecurity expectations. The operating programme records the
network architecture, the trust boundaries between safety-related
and balance-of-plant systems, and the access-control and audit-log
configuration. Cybersecurity-related safety events are classified
under PHASE-1 §10 with category `cybersecurity` and follow the
same notification timelines as physical events.

## §9 Records Retention

Plant records — every record defined in PHASE-1, the API audit
logs (PHASE-2), regulator submissions, safeguards reports,
emergency exercises, and safety-event investigations — retain
indefinitely. Decommissioning records retain through the post-
release period the regulator defines.

## §10 Time Synchronisation

Plant clocks synchronise per RFC 5905 (NTPv4) against a national-
metrological-laboratory stratum-1 service or an equivalent
recognised service. The synchronised time anchors all timestamped
records so that retrospective audits can correlate plant events
with off-site observations.

## §11 Decommissioning Custody

Decommissioning programmes maintain custody of plant systems,
spent fuel, radioactive waste, and the contaminated site itself
through the decommissioning timeline. Custody events emit records
that link to the decommissioning plan (PHASE-1 §11) and to the
relevant waste categorisation entries. Decommissioning is concluded
when the regulator releases the site to its agreed end state.

## §12 End-State Release

Site release follows the regulator's process: residual contamination
surveys, occupant exposure modelling for the agreed end use, and
the regulator's release decision. The API records the release
decision and the surveys that supported it; downstream consumers
(local authorities, prospective land users) consume the records
through the read-only profile.

## §13 Quality Dossier

The programme's quality dossier records the regulators it works
with, the safeguards inspectorate, the suppliers it has qualified
under ISO 19443, the emergency-preparedness exercises it has
conducted, and the operational events it has experienced. The
dossier is reviewed at least annually by the operating
organisation's quality manager.

## §14 Cross-Border Programme Operation

Multi-jurisdiction programmes (transboundary nuclear cooperatives,
international fuel-cycle initiatives) honour each participating
jurisdiction's regulator authorisations and safeguards agreements.
Cross-jurisdictional data transfers honour both source and
destination jurisdictions' export-control rules.

## §15 Configuration Management and Plant-State Documents

The Final Safety Analysis Report (FSAR), Technical Specifications,
Quality Assurance Manual, Emergency Plan, Security Plan, and
Decommissioning Plan together define the plant's licensed
configuration. Modifications to any of these documents follow a
configuration-management process under regulator oversight: proposed
amendment, safety review, regulator approval, document update,
training rollout, and effective-date publication.

The operating programme records every approved amendment as a
plant-state document (PHASE-1 §12) and references the in-force
document set from operating, outage, and safety-event records so
that retrospective audits resolve unambiguously to the licensed
configuration of the plant at the time.

## §16 Operating Experience Sharing

Operational events of broad interest are shared with the
international operating-experience community (the World Association
of Nuclear Operators, the IAEA Incident Reporting System, and
similar fora) through the channels the operating organisation
participates in. Event-sharing records are signed by the operating
organisation and reference the underlying safety-event record.

## §17 Periodic Safety Review

Operating plants undergo a periodic safety review (typically every
ten calendar years) that re-assesses the plant against current
regulatory expectations and operating experience. The review's
findings, the corrective actions identified, and the regulator's
acceptance are recorded as a chain of attestations against the
plant. Plants whose periodic review identifies open corrective
actions remain operational under interim measures recorded in the
quality dossier.

## §18 Lifetime Extension Governance

A plant whose operating organisation seeks to extend operation
beyond the original licence term submits a lifetime-extension case
to the regulator. The submission documents the plant's component
ageing assessments, the equipment-qualification programme, the
results of in-service inspection, the forecast environmental impact
through the extension period, and the proposed end-of-extension
disposition. Extension approvals are recorded as licence amendments
under §1.

## §19 Maintenance and Surveillance Programme

The plant's maintenance and surveillance programme implements the
Technical Specifications' surveillance requirements: routine
testing of safety-related components at the cadence the
specifications require, post-maintenance testing after corrective
work, and periodic equipment qualification re-assessment. The
programme records each surveillance against the relevant component
and against the operating cycle, so that surveillance compliance
is auditable end-to-end.

## §20 Conformance and Auditing

A programme conformant with WIA-nuclear-energy publishes its
operating authorisations, its safeguards agreement references, its
quality dossier, the catalogue of safety events at INES-1 and above,
and the catalogue of environmental releases, and answers an annual
self-assessment that maps each clause of this PHASE to the
programme's implementation. The self-assessment is reviewed during
routine regulator inspections.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-nuclear-energy
- **Last Updated:** 2026-04-27
