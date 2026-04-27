# WIA-infrastructure PHASE 3 — PROTOCOL Specification

**Standard:** WIA-infrastructure
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited civil-
infrastructure asset programme: asset-management-system accreditation
under ISO 55001, BIM information-management governance under ISO
19650, inspection-procedure governance, condition-rating governance,
maintenance-prioritisation governance, dam-safety and bridge-safety
inspection cycles, hydraulic structures' load-rating refresh, post-
event protocols (seismic, flood, wind, fire), data-stewardship
through asset transfer, and the chain of accountability across
designer-builder, owner, and inspector.

References (CITATION-POLICY ALLOW only):

- ISO 55001 (asset-management-system requirements)
- ISO 19650-2 (information delivery using BIM)
- ISO 16739 (IFC4 / IFC4.3)
- ISO 23386 / 23387 (data templates for built assets)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 31000 (risk management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)

---

## §1 Asset-Management-System Accreditation

A programme MAY claim conformance to WIA-infrastructure only after
a recognised certification body has issued a valid certificate
against ISO 55001:2014 covering the asset classes the programme
manages (bridges, hydraulic, water, transmission). The certificate
reference is published in the discovery document and is held in
the asset-management-system register that the API exposes as a
read-only resource.

Programmes that operate in jurisdictions whose national law adopts
ISO 55001-equivalent regulation (e.g. national asset-management
plans for transportation networks) MAY substitute the national
accreditation reference, provided the substitution is documented
in the operating programme's quality-management dossier.

## §2 BIM Information Management under ISO 19650

The federation file delivered at hand-over (PHASE-1 §8) is
governed by ISO 19650-2:2018 Information Delivery Cycle. The
operator's appointed BIM coordinator records:

- the Common Data Environment (CDE) reference;
- the levels of information need (LOIN) agreed for each
  delivery milestone;
- the appointed parties (designers, contractors, sub-
  contractors);
- the BIM execution plan governing the delivery; and
- the verification reports against the delivery's exchange
  information requirements.

CDE access is mutually authenticated; PII or commercially
sensitive content is redacted from the published federation file
before it enters the operating-phase record set.

## §3 Inspection-Procedure Governance

Inspection types in PHASE-1 §5 each map to a governing procedure
that the operator publishes in the programme's procedure register:

- Routine-visual inspections cite the operator's inspection manual
  and the qualifications of inspectors authorised to perform them
  (e.g. AASHTO Bridge Inspector qualification, EN 1090 inspector
  qualification, AWWA-certified water-utility inspector);
- In-depth and underwater inspections cite the diving or rope-
  access procedure and the safety-of-work plan;
- NDT inspections cite the ISO/IEC 17025-accredited laboratory's
  test methods (ISO 16828 for ultrasonic, ISO 17640 for weld
  testing, ASTM C876 for half-cell potential, ASTM D6432 for
  ground-penetrating radar);
- Load tests cite the design-code clause governing the test
  (AASHTO LRFR §8, EN 1990 Annex D).

Inspections submitted to the API whose `governingMethod` does not
resolve to an entry in the operator's procedure register return
`422` with type
`urn:wia:infrastructure:governing-method-unregistered`.

## §4 Condition-Rating Governance

Condition rating ladders cited in PHASE-1 §6 are versioned in the
programme's rating-scheme register. The register records the
ladder name (e.g. "AASHTO general appraisal 2024"), the categories
(0..9 in AASHTO), the descriptions, the period of applicability,
and the authority that maintains the ladder.

When an authority revises a ladder (e.g. an AASHTO update,
adoption of a new EN reliability category) the operator emits a
new register entry; prior entries remain addressable so that
historical assessments resolve to the ladder version current at
the time of assessment.

## §5 Maintenance-Prioritisation Governance

The operator maintains a prioritisation methodology that converts
condition assessments and remaining-service-life estimates into
ranked work-order recommendations. The methodology is documented
in the programme's quality-management dossier and is reviewed at
least annually by the asset-management-system steering committee.

Prioritisation factors include condition severity, public-safety
exposure (Annual Average Daily Traffic for roadway assets,
population downstream for hydraulic assets, customers served for
utility assets), financial cost-of-failure, environmental cost-of-
failure, and budget availability. Each work-order record records
the prioritisation score and the methodology version that
produced it.

## §6 Dam-Safety Inspection Cycle

Dams classified as significant-hazard or high-hazard under the
operator's national dam-safety regulator's classification follow
a regulator-mandated inspection cycle:

- annual visual inspection;
- five-yearly intermediate inspection by an independent
  qualified engineer;
- ten-yearly comprehensive inspection (statutory cycle in many
  jurisdictions, including the US National Dam Safety Program
  cadence and the UK Reservoirs Act 1975 inspection cadence).

Failure to complete a cycle within the regulator's deadline
triggers a regulatory-reportable event that the operator records
in the programme's incident register and in the regulator's
notification feed.

## §7 Bridge-Safety Inspection Cycle

Bridges follow inspection cycles published by the bridge owner's
regulator:

- US bridges: National Bridge Inspection Standards 23 CFR 650.305
  (typically 24-month routine cycle, with extensions or
  reductions per condition).
- EU bridges: cycles per the national-annex implementations of
  EN 1990 and the bridge owner's national bridge inspection
  manual.
- Korean bridges: cycles per 「시설물의 안전 및 유지관리에 관한
  특별법」 (Special Act on the Safety Control and Maintenance
  of Establishments) and the implementing regulations of the
  Ministry of Land, Infrastructure and Transport.

The operator records the regulator's reference cycle in the
programme's quality-management dossier and against each bridge
asset record.

## §8 Hydraulic-Asset Load-Rating Refresh

Hydraulic structures (dams, levees, lock gates, harbour walls)
refresh their load rating after each post-event inspection
(PHASE-1 §5 inspection-type "post-event") and at intervals
specified in the operating programme's procedure register. The
refresh emits a new condition-assessment record (PHASE-1 §6) with
`method` referencing the hydraulic load-rating method used
(e.g. CIRIA, USACE, KICT method).

## §9 Post-Event Protocols

Following a triggering event (earthquake, flood, wind, fire,
vehicle impact, vessel impact) the operator triggers a post-event
inspection workflow:

- a rapid visual screening within hours by the on-call duty
  inspector;
- a follow-on in-depth inspection by an independent qualified
  engineer;
- if warranted, a load test or NDT campaign;
- a revised condition assessment;
- a regulator notification within the deadline that the
  applicable jurisdiction requires.

Post-event inspection records are flagged with the triggering
event reference; downstream consumers can then filter the
asset's history by the event of interest.

## §10 Designer-Builder Liability and Defect Period

Hand-over records (PHASE-1 §8) include a defect-liability period
during which the designer-builder consortium is responsible for
remediating residual defects in the residual-defect register.
The operator records each defect-liability event (issue, response,
resolution) under the hand-over record. After the defect-
liability period elapses, the asset transitions to the operator's
ordinary maintenance regime.

## §11 Data Stewardship Through Asset Transfer

When an asset transfers from one owner to another (acquisition,
divestiture, transfer to successor jurisdiction) the originating
operator transfers the complete record set together with the
content-addresses; the receiving operator emits a transfer record
that binds the originating operator's records to the receiving
operator's identifier without reissuing the asset identifier.

The transfer record carries the date of effective transfer, the
contractual basis (sale, conveyance, statutory transfer), and any
warranties retained by the originating operator.

## §12 Cybersecurity for Asset-Management Platforms

Asset-management platforms operate under the operator's ISO/IEC
27001:2022 information-security-management system. Platforms
holding records that bear on public safety (dam-safety
inspections, bridge-safety inspections, water-quality monitoring)
have the highest IEC 62443-equivalent zoning the operator
declares; firmware updates to platform components are signed and
verified before install.

## §13 Quality-Management Dossier

The operator's quality-management dossier records the certified
asset-management-system scope, the procedure register, the
rating-scheme register, the prioritisation methodology, and the
deprecation history of inspection procedures. The dossier is
reviewed annually by the operator's asset-management-system
manager and is read during the operator's ISO 55001 surveillance
audits.

## §14 Cross-Jurisdiction Operation

Multi-jurisdiction operators (e.g. transport-corridor consortia,
trans-border water utilities) honour each participating
jurisdiction's regulatory regime and record the applicable
regulator references against each asset.

## §15 Programme Wind-Down and Successor Handover

A programme that ceases operations transfers asset records to a
recognised long-term archive, exports the record set to the
successor operator's records-management system, and notifies
regulators of the cessation. Records subject to indefinite-
retention regulations (dam-safety records, contaminated-land
records under the asset's footprint) transfer with content-
addresses preserved.

## §16 Climate-Adaptation Governance

Operators of long-life civil assets re-assess design assumptions
against the climate-adaptation guidance current in their
jurisdiction (national adaptation plans, regional climate-impact
assessments, sector-specific guidance such as the FHWA Adaptation
Decision-Making Assessment Process, the EU adaptation strategy,
or comparable national equivalents). Re-assessment outcomes feed
the prioritisation methodology in §5; assets whose climate-
adaptation review concludes that the design margin is depleted
are flagged for accelerated rehabilitation or re-design.

The operator records the climate-adaptation review reference,
the review's reviewer identifier, the asset classes covered, and
the date of the review. Reviews are repeated at intervals that
the operating programme's quality-management dossier sets.

## §17 Conformance and Auditing

A programme conformant with WIA-infrastructure publishes its
ISO 55001 certificate reference, its programme registration in
the discovery document, the catalogue of assets it operates, the
quality-management dossier reference, and the cyber-posture
summary, and answers an annual self-assessment that maps each
clause of this PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-infrastructure
- **Last Updated:** 2026-04-28
