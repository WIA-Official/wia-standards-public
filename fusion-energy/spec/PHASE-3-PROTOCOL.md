# WIA-fusion-energy PHASE 3 — PROTOCOL Specification

**Standard:** WIA-fusion-energy
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
fusion-energy facility: the IAEA fusion-safety
discipline (the IAEA Safety Standards GSR Part 1 to 7
applied as adopted by the operating jurisdiction's
regulator, augmented by the IAEA fusion-specific
Specific Safety Guides SSG-77 / SSG-78 / SSG-79); the
operating jurisdiction's regulatory-pathway discipline
(US NRC's risk-informed performance-based fusion
framework, UK ONR's fusion-safety adoption, EU national
basic-safety-standards transposition under Council
Directive 2009/71/Euratom and 2013/59/Euratom, JP NRA
discipline, KR NSSC discipline); the safety-case
discipline; the tritium-accountancy discipline (where
the fuel cycle includes tritium); the protection-
system discipline aligned to IEC 61508 / IEC 61511 /
IEC 60880; the ASME BPVC Section III + ASME NQA-1
discipline (where the operating jurisdiction adopts
ASME for fusion components); the operating-limit and
condition discipline; the plasma-operation conduct
discipline; the reportable-event discipline; and the
decommissioning discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- IAEA Safety Standards Series — General Safety
  Requirements (GSR Part 1 Governmental, Legal and
  Regulatory Framework for Safety; Part 2 Leadership
  and Management; Part 3 Radiation Protection and
  Safety of Radiation Sources; Part 4 Safety
  Assessment for Facilities and Activities; Part 5
  Predisposal Management of Radioactive Waste; Part
  6 Decommissioning of Facilities; Part 7 Emergency
  Preparedness and Response)
- IAEA fusion-specific Specific Safety Guides
  (SSG-77 / SSG-78 / SSG-79 in publication tracking,
  cited as the fusion-safety project's authoritative
  guidance on facility-and-activity safety
  considerations specific to fusion)
- IAEA INSAG and TECDOC documents on fusion safety
- ASME BPVC Section III (rules for construction of
  nuclear facility components)
- ASME B31.1 (power piping)
- ASME NQA-1 (quality assurance requirements for
  nuclear facility applications)
- ANS-58 series (American Nuclear Society safety
  standards for nuclear facilities)
- IEC 61508-1 to -7 (functional safety of E/E/PE
  safety-related systems)
- IEC 61511-1 to -3 (functional safety — safety
  instrumented systems for the process industry)
- IEC 60880 (computer-based systems performing
  category A functions)
- IEC 60709 (separation of redundant safety-classified
  channels)
- ICRP Publication 103 (radiation-protection
  recommendations) and ICRP Publication 119 (data
  for use in protection against external radiation)
- US 10 CFR Part 20 (standards for protection against
  radiation) where the operating jurisdiction is the
  US
- Council Directive 2013/59/Euratom (basic-safety-
  standards for protection against the dangers
  arising from exposure to ionising radiation) where
  the operating jurisdiction is an EU Member State
- US NRC Regulatory Guide framework for fusion (where
  the operating jurisdiction is the US and the NRC
  has issued risk-informed performance-based
  guidance)

---

## §1 IAEA Fusion-Safety Discipline

The IAEA Safety Standards Series provides the over-
arching framework that operating jurisdictions adapt
for fusion. The discipline:

- per-programme alignment to GSR Part 1 (governmental,
  legal and regulatory framework for safety) — the
  operating jurisdiction's regulator establishes the
  fusion regulatory pathway;
- per-programme alignment to GSR Part 2 (leadership
  and management for safety) — the facility operator
  establishes the safety-management system, safety
  committee, and safety culture;
- per-programme alignment to GSR Part 3 (radiation
  protection and safety of radiation sources) —
  worker dose limits, public dose limits, ALARA
  optimisation, emergency-exposure provisions;
- per-programme alignment to GSR Part 4 (safety
  assessment for facilities and activities) — the
  safety case (PHASE-1 §3) is the deliverable of the
  Part 4 assessment;
- per-programme alignment to GSR Part 5 (predisposal
  management of radioactive waste) — fusion-derived
  activated materials are managed under Part 5;
- per-programme alignment to GSR Part 6 (decommissioning
  of facilities) — the decommissioning record (PHASE-
  1 §10) tracks the Part 6 obligations;
- per-programme alignment to GSR Part 7 (emergency
  preparedness and response) — the emergency plan,
  on-site and off-site provisions, public-information
  arrangements.

Fusion-specific Specific Safety Guides SSG-77 /
SSG-78 / SSG-79 (in publication tracking) augment GSR
Parts 4, 5, and 6 with fusion-specific topics —
tritium fuel cycle, fusion-derived activation, plasma-
disruption events, magnet quench, vacuum-vessel-
breach scenarios.

## §2 Operating-Jurisdiction Regulatory-Pathway Discipline

Per-jurisdiction regulatory pathway:

- US NRC: risk-informed performance-based regulatory
  framework for fusion energy systems, formally
  adopted following the NRC's 10 CFR Part 53 / fusion
  rulemaking; the framework distinguishes fusion
  facilities from fission reactors and applies a
  graded approach proportional to the facility's
  hazard potential. Above-threshold facilities are
  licensed under the framework's pathway; below-
  threshold facilities operate under state byproduct-
  material licensing under Agreement-State arrangements.
- UK ONR: fusion-safety pathway under the Office for
  Nuclear Regulation framework, recognising fusion
  as a non-fission nuclear hazard with proportionate
  regulation;
- EU Member States: Council Directive 2009/71/Euratom
  (basic safety standards for nuclear safety of
  nuclear installations) and Council Directive
  2013/59/Euratom (basic safety standards for
  protection against ionising radiation) transposed
  into national law; the operating Member State's
  regulator (e.g. ASN-FR in France, BfS / BMUV in
  Germany, NRC-NL, etc.) adapts the framework for
  fusion;
- JP NRA: operating Japanese regulatory body;
- KR NSSC: Korea Nuclear Safety and Security
  Commission and the operating ministry's adaptation
  for fusion.

The facility's safety case cites the operating
jurisdiction's regulatory-pathway reference at the
programme level (PHASE-1 §3 `regulatoryPathway`).

## §3 Safety-Case Discipline

The safety case is the central deliverable. The
discipline:

- per-case enumeration of postulated initiating
  events (PHASE-1 §5) with frequency and consequence
  classification;
- per-case identification of safety functions the
  facility provides to prevent or mitigate the events;
- per-case allocation of safety functions to safety-
  classified components (PHASE-1 §6);
- per-case description of operating limits (PHASE-1
  §7) that maintain the design assumptions;
- per-case demonstration of compliance with the
  operating jurisdiction's dose targets and risk
  targets;
- per-case independent technical review (the operator's
  internal review plus, for higher-hazard programmes,
  external review by the regulator's technical-
  safety reviewer or the operating jurisdiction's
  designated review body).

## §4 Tritium-Accountancy Discipline

Where the fuel cycle includes tritium:

- per-location tritium-inventory measurement on the
  operator's measurement cadence (continuous
  monitoring at primary-confinement boundaries with
  periodic confirmation by independent methodology);
- per-period mass balance across the fuel cycle (the
  difference between the period's start-and-end
  inventories and the period's net additions / off-
  takes is reconciled within the per-method
  uncertainty);
- per-shipment chain of custody for tritium transfers
  between facilities (including transfers from the
  operating jurisdiction's tritium-extraction facility
  to the fusion facility, and shipments off-site for
  storage or disposition);
- per-period reporting to the operating jurisdiction's
  safeguards-and-accountancy regulator.

Tritium accountancy is the central record-keeping
under the operating jurisdiction's safeguards regime;
the operating jurisdiction may delegate accountancy to
the EURATOM safeguards inspectorate (EU Member States),
to the IAEA safeguards inspectorate (additional-
protocol arrangements), or to the operating
jurisdiction's domestic regulator.

## §5 Protection-System Discipline (IEC 61508 / IEC 61511 / IEC 60880)

Protection systems performing safety functions are
designed and qualified per:

- IEC 61508 for E/E/PE safety-related systems —
  systematic capability, hardware-fault-tolerance, and
  the SIL allocation per the per-function safety-
  requirements specification;
- IEC 61511 for the process-industry application of
  IEC 61508 (tritium fuel-cycle process-control
  applications often follow IEC 61511);
- IEC 60880 for computer-based instrumentation and
  control performing category A functions (the
  highest-importance functions) — the standard
  requires high-integrity software-development
  practices, V&V, independent verification, and
  pre-developed software qualification.

The ITER design choices for protection-system
functional safety provide a community-recognised
baseline for tokamak protection systems; the operating
jurisdiction's regulator adapts the baseline to the
facility's specific risk profile.

## §6 ASME BPVC Section III + ASME NQA-1 Discipline

Where the operating jurisdiction adopts ASME BPVC for
fusion components, the discipline:

- per-component classification under Section III
  Subsection NB / NC / ND (Class 1 / 2 / 3) or
  Section VIII Division 1 / 2 (non-nuclear pressure
  vessels) per the safety-classification of the
  component;
- per-component design, fabrication, examination,
  testing, and stamping per the applicable Subsection;
- per-facility ASME NQA-1 quality-assurance programme
  scoping the controls applied to safety-classified
  procurement, fabrication, and operation.

Where the operating jurisdiction does not adopt ASME
BPVC (e.g. EU jurisdictions adopting EN 13445 for
non-fusion-specific pressure equipment, or KR adopting
KEPIC for nuclear components), the discipline maps to
the equivalent national or community-recognised
standard.

## §7 Operating-Limit and Condition Discipline

Operating limits and conditions are the conditions
within which the facility operates safely:

- safety limits — quantitative bounds on critical
  parameters (e.g. plasma stored energy, magnet
  current, vacuum-vessel pressure) the protection
  system enforces;
- limiting conditions for operation — bounds on
  configurations and operations (e.g. the minimum
  operable channels of a redundant protection
  function);
- surveillance requirements — the cadence and method
  for confirming the limits are met;
- design-feature declarations — features that the
  safety case relies on (e.g. minimum-leak-tightness
  of the primary confinement);
- administrative controls — procedures, staffing, and
  training requirements.

## §8 Plasma-Operation Conduct Discipline

Plasma operations follow the conduct discipline:

- per-shot pre-operation review against the operating-
  limits register;
- per-shot in-progress monitoring with the protection
  system armed and the operations team monitoring
  protection-function status;
- per-shot post-operation data capture and review;
- per-campaign operational-experience feedback into
  the operating-limits register and safety-case
  refresh.

Off-normal plasma disruption events that invoke
protection functions are reportable per the operating
jurisdiction's reportable-event threshold (PHASE-1 §9).

## §9 Reportable-Event Discipline

Reportable events follow the operating jurisdiction's
reportable-threshold and notification-deadline rules.
The discipline:

- per-event detection through the facility's detection
  channels (radiation monitor, tritium monitor,
  protection-function activation, personnel-dose
  investigation alarm);
- per-event classification against the operating
  jurisdiction's classification table (the operator's
  pre-defined classification taxonomy);
- per-event regulator notification within the
  notification deadline;
- per-event root-cause analysis using the operating
  jurisdiction's investigation framework (e.g. the
  IAEA's INSAG-7 or operator's adopted framework);
- per-event lessons-learned narrative feeding the
  facility's preventive-maintenance and design-review
  cycles.

## §10 Records Retention

Programme records — every safety case revision,
postulated-event analysis, safety-classified-
component qualification, operating-limit register,
plasma operation, tritium-inventory measurement, and
reportable-event record — retain for the operating
life of the facility plus the operating jurisdiction's
records-retention horizon (typically the facility life
plus thirty years for nuclear safety records, with
some classes — tritium accountancy and reportable-
event root-cause records — retained longer).

## §11 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so
that protection-function timing analyses, reportable-
event regulator notification deadlines, and tritium-
mass-balance period closures are consistent across
the facility's runtime fleet.

## §12 Decommissioning Discipline

Per IAEA GSR Part 6:

- per-facility decommissioning planning prior to
  cessation of operations (planning starts during
  design and is updated through the operating life);
- per-facility decommissioning-phase declarations
  (planning, preparation, active decommissioning,
  post-decommissioning monitoring, site release);
- per-facility activated-component inventory derived
  from the fusion-derived neutron-fluence analysis
  through the operating life;
- per-facility waste-route declaration with the
  operating jurisdiction's radioactive-waste regulator;
- per-facility site-release acceptance per the
  operating jurisdiction's site-release criteria.

## §13 Emergency-Preparedness Discipline (GSR Part 7)

Per IAEA GSR Part 7 the facility's emergency plan
covers:

- on-site emergency response (the operator's safety
  organisation, the operator's emergency-control-
  centre, the facility's emergency-equipment cache);
- off-site emergency response (interface with the
  operating jurisdiction's off-site emergency
  agencies and the host community's emergency
  services);
- public-information arrangements (the operating
  jurisdiction's public-information protocol during
  the emergency);
- exercise programme (per-cycle drills of the on-site
  and off-site response);
- post-event review (the operator's post-emergency
  lessons-learned).

## §14 Per-Device-Class Operating Discipline

Device-class adaptations:

- magnetic-confinement (tokamak, stellarator): per-
  shot toroidal-field-and-plasma-current ramps,
  magnet-quench-protection arming, plasma-disruption-
  mitigation system arming;
- inertial-confinement (laser-direct-drive, laser-
  indirect-drive): per-shot target-and-laser
  alignment, target-bay radiation-and-debris
  containment;
- magnetised-target-fusion: per-shot pulse-power
  protection, lithium-handling protection where
  applicable;
- pilot-plant-net-energy-research: per-cycle
  operating-experience feedback into the design-basis
  refresh as the facility approaches commercial-
  scale operating envelopes.

## §15 Quality Dossier and Conformance

The facility's quality dossier records the governing
frameworks, the safety case revisions, the regulator-
correspondence history, the tritium-accountancy
history, the reportable-event history, the protection-
system functional-safety dossier, the ASME / NQA-1
quality-assurance programme (where adopted), and the
emergency-preparedness exercise history. The dossier
is reviewed at least annually by the facility's safety
manager and is provided to the regulator on request.

A programme conformant with WIA-fusion-energy
publishes its safety-case public summary (where the
operating jurisdiction publishes one), its regulator-
correspondence summary, and the aggregate reportable-
event count; and answers an annual self-assessment
that maps each clause of this PHASE to the facility's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-fusion-energy
- **Last Updated:** 2026-04-28
