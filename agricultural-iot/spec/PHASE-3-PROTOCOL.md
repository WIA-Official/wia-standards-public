# WIA-agricultural-iot PHASE 3 — PROTOCOL Specification

**Standard:** WIA-agricultural-iot
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
agricultural-IoT operation: ISO 11783 (ISOBUS) task
controller compliance, AgGateway ADAPT envelope discipline,
OGC SensorThings ingest discipline, water-rights compliance,
pesticide application compliance, livestock RFID-traceability
discipline, animal-welfare governance, agronomic
recordkeeping, and end-of-season disposition.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 22000:2018 (food safety management)
- ISO/IEC 27001:2022 (information security management)
- ISO 11783 series (ISOBUS)
- ISO 11784 / 11785 (animal RFID)
- ISO 14064-1:2018 (greenhouse gas accounting)
- ISO 19156:2011 (Observations and Measurements)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- AgGateway ADAPT
- OGC SensorThings API 1.1
- W3C SSN / SOSA
- Codex Alimentarius CXG 65-1997 (HACCP)
- US EPA FIFRA / Worker Protection Standard
- EU Regulation (EC) 1107/2009 (plant protection products)
- KR Pesticide Control Act + Positive List System
- OIE Terrestrial Animal Health Code (cited as the
  international animal-welfare reference; the operating
  jurisdiction's national animal-welfare law is the
  governing framework)

---

## §1 ISOBUS Compliance Discipline

ISOBUS task controllers and implements communicate per ISO
11783 series (Parts 1, 7, 10, 11). The operator's per-OEM
ISOBUS conformance verification covers:

- per-implement compliance with ISO 11783-7 application-
  layer messages;
- per-task-controller compliance with ISO 11783-10 task
  data interchange;
- per-data-element coverage of ISO 11783-11 mobile data
  element dictionary;
- per-vehicle plug-and-play interoperability test against
  the operator's typical implement fleet.

Non-conformant implements are flagged in the operator's
equipment register and not used for task data round-trip
until the OEM resolves the conformance issue.

## §2 AgGateway ADAPT Envelope Discipline

ISOBUS task data, OEM telemetry, and FMIS-side ingest flow
through AgGateway ADAPT envelopes that wrap the ISOXML
representation in a vendor-neutral interchange shape. The
operator's per-vendor ADAPT plug-in mapping is recorded in
the quality dossier; ADAPT plug-in upgrades trigger
regression tests against the operator's historical task
archive to confirm interpretation continuity.

## §3 OGC SensorThings Ingest Discipline

Sensor observations follow the OGC SensorThings API 1.1
data model (Thing → Datastream → Observation). The
operator's ingest pipeline:

- per-device Datastream registration with ObservedProperty
  + UnitOfMeasurement bound to the W3C SSN/SOSA profile;
- per-observation FoI (Feature of Interest) reference for
  non-fixed-location sensors (e.g. rover-mounted soil
  sensors);
- per-observation timestamp tolerance (devices with
  imprecise clocks annotate observations with the
  device-side clock skew at upload).

## §4 Water-Rights Compliance

Irrigation operations honour the operating jurisdiction's
water-rights regime (US prior-appropriation in western
states, riparian rights in eastern states, KR Water Use
Permit Act, EU Water Framework Directive 2000/60/EC,
equivalent regimes elsewhere). Per-zone irrigation plans
(PHASE-1 §8) cite the water-rights allocation; over-
allocation triggers `urn:wia:agricultural-iot:water-rights-
exceeded` (PHASE-2 §9).

## §5 Pesticide Application Compliance

Spraying tasks (PHASE-1 §6 `taskKind=spraying`) honour the
operating jurisdiction's pesticide-application rules:

- US: EPA FIFRA + Worker Protection Standard + state
  pesticide control act;
- EU: Regulation (EC) 1107/2009 + national implementation;
- KR: Pesticide Control Act + Positive List System;
- equivalent rules elsewhere.

Per-task records cite the registered product, the rate
(per the product label), the buffer zones honoured (per
the product label), the applicator certification reference,
and the re-entry interval. Records flow into the operator's
pesticide-applicator log retained per the regulator's
retention rule (typically 2-3 years post-application).

## §6 Livestock RFID Traceability Discipline

Livestock RFID tags (PHASE-1 §7) follow ISO 11784 / 11785
code structure and air-interface. Per-jurisdiction
traceability requirements:

- US National Animal Identification System (where
  applicable) + state cattle-traceability programmes;
- EU IPAFFS / TRACES (traceability for animal movements);
- KR Livestock Traceability Act + KAHIS reporting;
- equivalent national rules elsewhere.

Per-animal birth, movement, and slaughter events are
recorded against the animal record and reported to the
jurisdiction's traceability authority at the cadence the
authority requires.

## §7 Animal Welfare Governance

Operators that handle animals follow the operating
jurisdiction's animal-welfare law (US AWA / state animal-
cruelty statutes; EU Council Directives on the protection
of farmed animals; KR Animal Protection Act; equivalent
national law). The operator's welfare programme covers:

- per-species housing standards (space, ventilation,
  lighting, social grouping);
- per-species health management (preventive veterinary
  care, vaccination, biosecurity);
- per-species transport conditions (max journey time,
  rest stops, loading density);
- per-species end-of-life handling (pre-slaughter
  stunning, on-farm euthanasia for non-marketable
  animals).

Welfare incidents trigger the operator's welfare-incident
workflow with the welfare officer of record; serious
incidents notify the jurisdiction's animal-welfare
authority within the required notification window.

## §8 Greenhouse Climate Control Discipline

Greenhouse climate controllers operate per the operator's
crop-specific climate-control SOP:

- per-crop temperature / humidity / CO2 setpoint envelope
  per growth stage;
- per-zone controller tuning to avoid integral-windup
  during sensor-fault recovery;
- per-house energy-use accounting per ISO 14064-1:2018
  for greenhouse-gas inventory.

Climate-control deviations from the SOP envelope emit
alerts through the streaming subscription and feed the
operator's quality-deviation workflow.

## §9 Records Retention

Operation records — every observation / ISOBUS task /
animal / irrigation plan / soil classification / API audit
log — retain per the operating jurisdiction's agricultural
records-retention rules (typically 5 years for EU CAP
cross-compliance, 3 years for US FSMA produce-safety, 7
years for tax-relevant records). Pesticide application
records and livestock traceability records retain per the
regulator's rule (typically longer than general operation
records).

## §10 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
observation timestamps and ISOBUS task timing are
consistent. Field devices with imprecise clocks annotate
their observations with clock-skew metadata so that the
ingest pipeline can correct or annotate accordingly.

## §11 Cross-Jurisdictional Operation

Multi-jurisdiction operators (cross-border co-operatives,
multinational farming companies) honour each jurisdiction's
water-rights, pesticide, livestock-traceability, and
animal-welfare rules. Per-operation governing-jurisdiction
tagging supports downstream regulator-specific reporting.

## §12 Quality Dossier

The operator's quality dossier records the ISOBUS
conformance verifications, the AgGateway ADAPT plug-in
mappings, the OGC SensorThings profile, the water-rights
allocations per zone, the pesticide-applicator
certifications, the livestock-traceability binding, the
animal-welfare officer of record, and the operator's
incident history. The dossier is reviewed at least
annually by the operator's quality manager.

## §13 Crop-Protection Drift and Buffer Discipline

Spraying tasks honour the operator's drift-and-buffer SOP:

- per-product buffer-zone distance per the product label
  (typically 5-30m depending on product class and adjacent
  receptor type — sensitive crops, water bodies, public
  areas);
- per-task wind-speed observation cited from the field's
  weather station (PHASE-1 §5 `wind-speed-ms` /
  `wind-direction-deg`); spraying halts when wind speed
  exceeds the product label's threshold;
- per-task drift-card placement at downwind receptor edges
  for high-drift-risk applications;
- per-task post-application drift assessment when drift
  cards or visual observation suggest off-target deposition.

Drift incidents trigger the operator's notification
workflow to the affected receptor's owner and (where the
jurisdiction requires) to the pesticide regulator.

## §14 Soil-Sampling and Laboratory Discipline

Soil classification (PHASE-1 §9) follows the operator's
soil-sampling SOP:

- per-sample geographic reference per ISO 19111 with sub-
  metre precision;
- per-sample depth interval (0-15cm topsoil, 15-30cm
  subsoil, deeper for tile-drainage management);
- per-sample laboratory analysis at an ISO/IEC 17025-
  accredited soil-testing laboratory;
- per-sample retention of the physical sample for the
  jurisdiction's required period (typically 12-24 months
  post-analysis).

## §15 Sustainability and Carbon Footprint Discipline

Operators that participate in agricultural carbon
markets (US Climate Action Reserve, EU CAP eco-schemes,
KR K-ETS agricultural protocol, voluntary registries
including Verra and Gold Standard) record per-field
practice changes (cover-cropping, reduced tillage,
nitrogen-management changes) that drive carbon credit
issuance. The operator's carbon-protocol binding cites
the registry's methodology version and the per-field
baseline year that the additionality claim depends on.

Carbon-credit verification follows the registry's
verification cadence (typically annual or biennial) by
an accredited third-party verifier; verification reports
are attached to the operator's quality dossier.

## §16 Greenhouse and Indoor Operation Discipline

Greenhouse and indoor (vertical farming) operations
operate per the operator's controlled-environment SOP:

- per-zone temperature / humidity / VPD setpoint envelope;
- per-zone CO2 enrichment rate per crop;
- per-zone supplemental lighting schedule (PPFD per
  growth stage);
- per-zone fertigation EC and pH targets;
- per-zone IPM monitoring (pest scouting, biological
  control agent releases).

Controlled-environment data flows through the same
observation channel (PHASE-1 §5) but with greenhouse-
specific ObservedProperty values (CO2-ppm, PPFD-
umol-per-m2-per-s, fertigation EC, fertigation pH, vapor
pressure deficit kPa).

## §17 Aquaculture Operation Discipline

Operators that operate aquaculture (pond, marine net-pen,
recirculating-aquaculture systems) honour the operating
jurisdiction's aquaculture rules: water-quality monitoring
(DO, ammonia, nitrite, pH), fish-health surveillance
(per-species pathogen-screening cadence), feed-conversion
recording, and (for marine net-pen) per-site environmental
monitoring including benthic impact and sea-lice or
parasite control.

## §18 Conformance and Auditing

An operation conformant with WIA-agricultural-iot publishes
its ISOBUS conformance verifications, its water-rights
allocations, its pesticide-application register summary,
its livestock-traceability binding, and its animal-welfare
incident summary, and answers an annual self-assessment
that maps each clause of this PHASE to the operator's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-agricultural-iot
- **Last Updated:** 2026-04-28
