# WIA-fuel-cell PHASE 3 — PROTOCOL Specification

**Standard:** WIA-fuel-cell
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
fuel-cell deployment: the IEC 62282 series safety,
performance, installation and test discipline; the ISO
14687 hydrogen-fuel-quality discipline; the IEEE 1547
grid-interconnection discipline (where the system is
grid-coupled); the IECEx hazardous-area discipline
(where the installation handles flammable hydrogen);
the UN GTR 13 / UN R134 vehicle-onboard discipline; the
operating-jurisdiction AHJ permitting and acceptance
discipline; the periodic-inspection cadence discipline;
and the incident-record discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 17021-1:2015 (management-system audit and
  certification)
- ISO/IEC 17025:2017 (testing and calibration
  laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- ISO 14687 (hydrogen fuel quality)
- ISO 19880-1 (gaseous hydrogen — fuelling stations)
- ISO 22734 (hydrogen generators using water
  electrolysis)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- IEC 62282-2 / -3-100 / -3-200 / -3-300 / -4-101 /
  -6-100 / -6-200 / -7-1 / -7-2 / -8
- IEEE 1547-2018 / IEEE 1547.1-2020
- IECEx system documents and the IEC 60079 series
  (60079-0 general, 60079-10-1 area classification,
  60079-14 design selection erection, 60079-17
  inspection and maintenance)
- UN GTR 13 (hydrogen and fuel-cell vehicles)
- UN ECE Regulation No. 134
- SAE J2601:2020 (gaseous hydrogen fuelling protocol);
  cited where the operating environment uses the SAE
  J2601 protocol in parallel with ISO 19880-1
- NFPA 2 (Hydrogen Technologies Code); cited where the
  operating jurisdiction is the United States and the
  AHJ adopts NFPA 2

---

## §1 IEC 62282 Discipline

The IEC 62282 series is the primary normative reference
for fuel-cell systems. The deployment's IEC 62282
discipline:

- per-stack IEC 62282-2 module test (or IEC 62282-7-1
  / 7-2 single-cell-and-stack test) before
  installation;
- per-stationary-system IEC 62282-3-100 safety
  attestation (the safety standard for stationary
  fuel-cell power systems);
- per-stationary-system IEC 62282-3-200 performance
  test methods (measurement of rated power, voltage,
  efficiency, response time at the manufacturer's
  rated operating point);
- per-stationary-system IEC 62282-3-300 installation
  inspection (covering electrical, fluid, gas,
  thermal, ventilation, and emergency-shutdown
  installation);
- per-industrial-truck system IEC 62282-4-101 (the
  electric-truck specific safety standard);
- per-micro-system IEC 62282-6-100 / 6-200 (safety and
  performance for portable systems);
- per-regenerative-energy-storage system IEC 62282-8
  (the standard for regenerative fuel-cell energy
  storage operating in reverse mode).

## §2 ISO 14687 Hydrogen-Fuel-Quality Discipline

Hydrogen fuel supplied to the stack inlet conforms to
the ISO 14687 grade the stack tolerates per the
manufacturer's data sheet:

- Grade A (general industrial use, lower purity);
- Grade B (intermediate);
- Grades C / D (PEMFC for stationary or transport
  applications, requiring tight control of CO,
  sulphur, ammonia, and total hydrocarbons);
- Grade E (PEMFC for road-vehicle applications, with
  the strictest contaminant limits to protect on-
  board catalysts).

The discipline:

- per-supply-handoff fuel-quality verification using
  the contaminant panel ISO 14687 specifies;
- per-laboratory ISO/IEC 17025 accreditation for the
  contaminant panel (without ISO/IEC 17025
  accreditation the verdict has no defensible
  provenance);
- per-non-conformance impact analysis — sulphur and
  CO contaminants typically cause reversible
  degradation, while halogenated compounds cause
  irreversible degradation, and the impact analysis
  drives the operations team's response (continued
  operation with elevated monitoring, supply
  rejection, or stack inspection).

## §3 IEEE 1547 Grid-Interconnection Discipline

Where the system is grid-coupled (PHASE-1 §2
`gridInterconnection` of `grid-paralleled-low-voltage`
or `grid-paralleled-medium-voltage`) the deployment's
discipline:

- per-system IEEE 1547.1-2020 conformance test report;
- per-system ride-through category declaration
  (category-i / ii / iii per IEEE 1547-2018 abnormal-
  condition performance);
- per-system interoperability protocol (SunSpec
  Modbus, IEEE 2030.5, DNP3 — IEEE 1547-2018 cites
  these as the recognised interoperability
  protocols);
- per-installation utility interconnection agreement
  with the grid system operator (the agreement records
  the point-of-common-coupling capacity, the
  voltage-and-frequency operating bands, and the
  abnormal-condition trip and ride-through settings).

The IEEE 1547-2018 abnormal-condition ride-through
requires the system to remain connected during voltage
and frequency excursions within the per-category
envelope rather than tripping immediately, supporting
grid stability under disturbance.

## §4 IECEx Hazardous-Area Discipline

Where the installation handles flammable hydrogen with
the potential for leak, the deployment's IECEx
discipline:

- per-area IEC 60079-10-1 area classification (Zone 0
  / 1 / 2 for gas atmospheres);
- per-equipment IECEx Certificate of Conformity for
  equipment intended to operate in the zone (the IECEx
  CoC certifies the equipment against the relevant IEC
  60079 part — flameproof Ex d, increased safety Ex e,
  intrinsic safety Ex i, encapsulation Ex m,
  pressurisation Ex p, etc.);
- per-installation IEC 60079-14 design verification
  (selection of equipment for the zone, cable entry,
  bonding, separation distances);
- per-installation IEC 60079-17 inspection cadence
  (initial detailed inspection followed by periodic
  close, visual, or sampling inspections per the
  cadence the standard prescribes).

The installation team's "Ex competent person"
qualification (per IEC 60079-14 / 60079-17
recommendations) is recorded against the inspector
identity in PHASE-1 §9.

## §5 UN GTR 13 / UN R134 Vehicle-Onboard Discipline

For vehicle-onboard installations the deployment's
discipline:

- per-vehicle UN GTR 13 test report (covering crash
  test, post-crash fuel leakage, hydrogen-storage
  container — environmental testing, expected service
  performance, baseline performance, durability — and
  fuel-system integrity in normal and post-crash
  conditions);
- per-vehicle UN ECE Regulation No. 134 type approval
  certificate where the operating jurisdiction
  recognises UN R134 type approval (the EU, JP, and
  KR, among the operating jurisdictions that have
  acceded to the UNECE 1958 Agreement);
- per-vehicle storage-container specification
  (typically 70 MPa for passenger cars and 35 to 70
  MPa for heavy duty, with Type 4 fully wrapped
  composite cylinders dominating the passenger-car
  market and Type 3 metal-lined composite cylinders
  also in use);
- per-vehicle fuelling-protocol declaration (SAE
  J2601:2020 for the SAE community, ISO 19880-1 for
  the ISO community).

## §6 AHJ Permitting and Acceptance Discipline

The operating jurisdiction's authority having
jurisdiction (AHJ) permits and accepts the
installation:

- per-permit installation-permit application with
  the submitted IEC 62282-3-300 installation design
  documentation;
- per-acceptance commissioning inspection by the AHJ
  or AHJ-recognised inspection body;
- per-acceptance certificate of occupancy or operating
  permit;
- per-modification re-acceptance for material
  modifications (capacity change, fuel-supply-rail
  change, hazardous-area-classification change).

In the United States, NFPA 2 (the Hydrogen Technologies
Code) is widely adopted by AHJs and applies in
parallel with the IEC 62282 series. In the European
Union, the operating Member State's transposition of
the ATEX Directives and the Pressure Equipment
Directive applies in parallel with the IEC 62282
series. In Korea, KGS Code AC112 (KGS Industries' fuel-
cell installation code) and the operating Ministry of
Trade, Industry and Energy's enforcement rules apply.

## §7 Periodic-Inspection Cadence Discipline

Periodic-inspection cadence is driven by:

- per-stack manufacturer-recommended-maintenance
  schedule;
- per-zone IEC 60079-17 inspection cadence;
- per-jurisdiction AHJ-mandated inspection cadence
  (some AHJs mandate annual inspection as a permit
  condition, some mandate inspection on operating-hour
  thresholds);
- per-event-trigger re-inspection (post-incident,
  post-modification, post-environmental event such
  as flood or seismic event affecting the
  installation).

## §8 Incident-Record Discipline

Per-incident discipline:

- per-incident detection — automated leak detection
  via hydrogen sensors with pre-defined alarm
  thresholds, automated flame detection where the
  application class warrants, and operator
  observation;
- per-incident containment — emergency shutdown,
  isolation of the affected hydrogen supply, ventilation
  activation, evacuation if personnel-affecting;
- per-incident root-cause analysis using the
  manufacturer's failure-mode library and the
  operating jurisdiction's incident-investigation
  framework;
- per-incident AHJ notification on the operating
  jurisdiction's reporting threshold (typical
  thresholds: any personnel injury, any environmental
  release, any incident triggering emergency-services
  response);
- per-incident lessons-learned narrative feeding the
  deployment's preventive-maintenance and design-
  review cycles.

## §9 Records Retention

Programme records — every stack registration, BoP
registration, fuel-quality verification, grid-
interconnection test, vehicle-onboard test,
commissioning record, periodic inspection, and
incident record — retain for the operating life of
the system plus the operating jurisdiction's records-
retention horizon (typically the system life plus five
to ten years for the AHJ's operational records, with
incident records retained longer per the operating
jurisdiction's investigation rules).

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so
that grid-disturbance event timestamps, IEEE 1547
ride-through performance audits, IEC 60079-17
inspection cadence, AHJ incident-notification
deadlines, and SAE J2601 fuelling-protocol exchanges
are consistent across the deployment's runtime fleet.

## §11 End-of-Life and Stack Replacement Discipline

End-of-life criteria are recorded at stack
registration (PHASE-1 §3 `endOfLifeCriteria`).
Replacement discipline:

- pre-replacement fuel-quality re-verification (a
  stack replacement provides an opportunity to
  recheck the supply rail);
- replacement-stack IEC 62282-2 or IEC 62282-7-1 / 7-2
  test record;
- post-replacement performance test (IEC 62282-3-200);
- post-replacement AHJ acceptance for material
  capacity-change replacements;
- replaced-stack disposition record (recycling,
  manufacturer take-back, or disposal per the
  operating jurisdiction's hazardous-waste rules).

## §12 Manufacturer-Operator Coordination Discipline

Manufacturer-operator coordination spans:

- per-system technical-data-package handoff at
  commissioning (manufacturer's IEC 62282 evidence,
  IEC 60079 evidence for Ex equipment, recommended-
  maintenance schedule, spare-parts inventory);
- per-system field-service incident reporting back to
  the manufacturer feeding the manufacturer's
  reliability database;
- per-system firmware / control-system update
  cadence;
- per-system warranty-period support handoff.

## §13 Per-Application Operating Discipline

Application-class adaptations:

- stationary-power: voltage-and-frequency operating
  bands declared in the IEEE 1547 utility agreement;
  CHP applications add thermal-recovery monitoring;
- industrial-electric-truck: per-shift refuelling
  events, per-cycle operating-hours tracking, on-site
  refuelling-station integration per ISO 19880-1
  where applicable;
- micro-portable: per-cartridge refuelling events,
  fuel-quality re-verification at cartridge swap;
- regenerative-energy-storage: per-cycle round-trip
  efficiency tracking, electrolysis-mode hydrogen-
  generation integration per ISO 22734 where the
  system is co-located with an electrolyser;
- vehicle-passenger / heavy-duty: per-fill SAE J2601
  / ISO 19880-1 protocol log, periodic UN R134 in-
  service inspection per the operating jurisdiction's
  vehicle-inspection regime.

## §14 Quality Dossier and Conformance

The deployment's quality dossier records the governing
frameworks, the manufacturer-of-record and the system-
integrator-of-record, the IEC 62282 evidence, the IEC
60079 / IECEx evidence (where Ex zones apply), the
IEEE 1547 / IEEE 1547.1 evidence (for grid-
interconnected installations), the UN GTR 13 / UN
R134 evidence (for vehicle-onboard installations), the
ISO 14687 fuel-quality verification history, the AHJ
permitting and acceptance correspondence, the periodic-
inspection history, and the incident history. The
dossier is reviewed at least annually by the
deployment's quality manager.

A programme conformant with WIA-fuel-cell publishes its
governing-framework declarations, its commissioning
acceptance, its periodic-inspection cadence, the
aggregate fuel-quality conformance rate, and the
aggregate incident rate; and answers an annual self-
assessment that maps each clause of this PHASE to the
deployment's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-fuel-cell
- **Last Updated:** 2026-04-28
