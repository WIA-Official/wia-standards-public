# WIA-energy-cloud PHASE 3 — PROTOCOL Specification

**Standard:** WIA-energy-cloud
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
energy-cloud operator: the CIM-model lifecycle
discipline (IEC 61968-13 + IEC 61970-301);
the wholesale-market participation discipline (FERC
Order 2222 + RTO / ISO + KR KPX market rules); the
forecast-and-scheduling discipline; the customer-
engagement and program-enrolment discipline; the
third-party app-catalogue discipline; the cybersecurity
discipline (IEC 62351 + NERC CIP-002~014 + DOE C2M2);
the data-quality discipline applied to the AMI / SCADA
data substrate; the outage-management and reliability-
reporting discipline (NERC EOP / OE-417); the
operational-resilience discipline (DORA-equivalent
where applicable); and the supervisory and balancing-
authority cooperation discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 + 27019:2024
- ISO 50001:2018
- IEC 61968 series, IEC 61970 series, IEC 61850-7
  series, IEC 62325 series, IEC 62351 series
- IEEE 1547-2018 + IEEE 2030.5-2018 (cross-domain
  reference)
- NERC CIP-002 through CIP-014
- NERC BAL-001-2 + BAL-003-2 + BAL-005-1 (frequency
  response, primary frequency response, balancing
  authority control)
- NERC IRO-001 + IRO-008 + IRO-010 (interconnection
  reliability operations and coordination)
- NERC COM-001-3 + COM-002-4 (communications)
- NERC EOP-004-4 + EOP-005-3 + EOP-008-2 (emergency
  preparedness and operations, including OE-417
  electric-emergency reporting)
- US FERC Order 2222 + Order 2003-D
- US DOE C2M2 v2.1 (Cybersecurity Capability
  Maturity Model)
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- KR 전기사업법 + 전력시장운영규칙 + KPX 운영규정 +
  KEPIC

---

## §1 CIM Model Lifecycle Discipline

The CIM model lifecycle:

1. Model build — the operator's distribution-and-
   transmission planning function builds the network
   model in a CIM-conformant tool.
2. Validation — the model is validated against IEC
   61968-13 / IEC 61970-301 schema and against the
   operator's published validation rules
   (connectivity, data-completeness, electrical-
   feasibility).
3. Profile export — the appropriate profile (CGMES
   for ENTSO-E grids; per-RTO profiles in US; KEPCO
   profile in KR) is exported.
4. Publication — the model is published to the
   operator's federation registry; consumer
   subsystems (DERMS / ADMS / EMS / DMS) ingest the
   model.
5. Diff and rollback — model-update deltas are
   recorded; rollback to a prior version is
   exercised through the registry.

## §2 Wholesale-Market Participation Discipline

For FERC-jurisdictional markets:

- Order 2222 — DER aggregations participate through
  the operator's market-participation surface; the
  RTO / ISO's tariff-aligned product set (energy DA /
  RT, ancillary services, capacity, demand response)
  drives the operator's bid catalogue.
- Order 2003-D — generator interconnection.
- Per-RTO market rules — CAISO Tariff §29 + §30 ·
  PJM OATT · MISO Module C / E · ERCOT Nodal
  Protocols · NYISO Services Tariff · ISO-NE OATT ·
  SPP Open Access Transmission Tariff.

For KR-jurisdiction:

- KR 전력시장운영규칙 — the KPX market-operating
  rules cover the Korean wholesale-market
  participation discipline.
- KR 전기사업법 — the underlying statutory framework.

## §3 Forecasting and Scheduling Discipline

The forecasting discipline applies probabilistic
forecasts where appropriate:

- Solar-irradiance and wind-resource forecasts
  feed the renewable-DER fleet output forecast.
- Net-load forecasts (load minus distributed-
  generation) drive the wholesale-market bidding.
- EV-charging-load forecasts inform the
  distribution-feeder constraint analysis.
- Wholesale-price forecasts inform the dispatch
  optimisation for behind-the-meter storage.

Forecast-error metrics (MAE / RMSE / pinball-loss /
prediction-interval coverage) are reported on the
operator's published cadence; material drift
triggers the operator's model-revalidation
workflow.

## §4 Customer-Engagement and Program-Enrolment Discipline

The engagement discipline:

- Net-metering programs — the customer's net-export
  is metered and credited per the operator's
  published rate.
- Time-of-use tariff — the customer's hourly /
  seasonal price is communicated through the
  customer-portal.
- Demand-response programs — opt-in / opt-out
  enrolment, baseline-and-performance settlement,
  and customer dispatch.
- Virtual-power-plant programs — aggregation of
  behind-the-meter resources for wholesale-market
  participation.
- Consent capture — every program enrolment captures
  the customer's informed consent under the operating
  jurisdiction's data-protection regime (GDPR for
  EU; CCPA for CA; KR PIPA for KR).

## §5 Third-Party App-Catalogue Discipline

The app-catalogue discipline:

- App registration — the third-party developer
  registers the app with the operator's catalogue
  declaring the requested OAuth scopes and the
  customer-data-access patterns.
- App review — the operator's app-review committee
  reviews against security, customer-protection, and
  data-minimisation criteria.
- App approval — approved apps are published to
  the customer-facing app catalogue; customers grant
  the OAuth consent on the app's first use.
- App monitoring — anomalous data-access patterns
  trigger the operator's review or suspension.

## §6 Cybersecurity Discipline

The cybersecurity discipline:

- IEC 62351 series — TLS profile (62351-3), MMS
  secure profile (62351-4), DNP3 SAv6 (62351-5),
  GOOSE / SV (62351-6), NSM (62351-7), RBAC
  (62351-8), KMS (62351-9).
- NERC CIP-002 to CIP-014 — bulk-electric-system
  cyber asset categorisation, security-management
  controls, personnel and training, electronic
  security perimeters, physical security, system-
  security management, incident-reporting, recovery,
  configuration-change, information-protection,
  supply-chain risk management, physical security
  for transmission stations and substations.
- DOE C2M2 v2.1 — the operator's MIL maturity
  level is documented and exercised on a periodic
  re-assessment cadence.

## §7 Data-Quality Discipline (AMI / SCADA Substrate)

The data-quality discipline:

- AMI data — meter reads pass through the operator's
  validation, estimation, and editing (VEE) process
  before settlement use.
- SCADA data — analog and status-point quality
  flags drive the operator's state-estimator
  reconciliation.
- Topology consistency — the network model is
  reconciled against switch-status reports and field
  observations.

## §8 Outage-Management and Reliability-Reporting
       Discipline

Outage management and reliability:

- The operator's OMS records every interruption with
  the duration, the affected customer count, and
  the cause code.
- SAIDI / SAIFI / CAIDI / MAIFI reliability indices
  are computed under IEEE 1366-2022.
- Reportable events follow NERC EOP-004-4
  (electric-emergency report and disturbance
  reporting) — DOE Form OE-417 for US-jurisdiction;
  KR 전기사업법 reporting for KR-jurisdiction.

## §9 Operational-Resilience Discipline

For energy-cloud operators that are also financial
entities subject to DORA Regulation (EU) 2022/2554
the DORA discipline applies; otherwise the operator's
internal operational-resilience programme covers:

- ICT-third-party-risk mapping.
- ICT-incident reporting under the operating
  jurisdiction's regime (NERC CIP-008 / EU NIS2 /
  KR 정보통신망법).
- Threat-led penetration testing on the operator's
  cadence.
- Annual resilience-test programme.

## §10 Identity, Time, and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline; IEEE 1588 PTP is exercised where sub-
millisecond precision is required for synchrophasor
time-stamping. Audit-events are emitted for every
CIM publication, dispatch instruction, market bid,
program enrolment, app approval, customer-data
access, operations event, and cybersecurity-posture
update.

## §11 Volt-VAr / Volt-Watt Coordination Discipline

The volt-VAr and volt-watt coordination discipline
operationalises distributed-DER reactive-power
support at the distribution-feeder level:

- The ADMS computes feeder-level voltage profiles
  using the CIM topology and real-time measurements.
- Volt-VAr / Volt-Watt curves are dispatched to the
  fleet via IEEE 2030.5 DERControl messages or via
  the IEC 61850 GOOSE / MMS substation channel.
- Per-feeder hosting-capacity analysis identifies
  the maximum DER penetration the feeder can absorb
  without violating ANSI C84.1 voltage limits.
- Reactive-power dispatch follows the IEEE 1547-2018
  Category B reactive-capability envelope by default.

## §12 Demand-Response Settlement Discipline

The DR settlement discipline:

- Customer baseline computation per the relevant
  RTO / ISO methodology (CAISO 10-of-10 baseline,
  PJM CBL with day-of adjustment, ERCOT 4CP
  baseline, NYISO baseline, ISO-NE baseline, KR
  KPX 표준 베이스라인).
- Performance evaluation — the difference between
  baseline and actual is the DR performance.
- Settlement payment — per-product clearing prices
  apply to the dispatched DR quantity per the
  market's tariff.
- Performance penalties — under-performance
  penalties apply per the program's performance-
  band rules.

## §13 Wide-Area Monitoring (WAMS) Discipline

The wide-area monitoring discipline:

- Synchrophasor data ingestion via IEEE C37.118 +
  C37.118.2 message envelopes from PMUs deployed
  across the operator's transmission network.
- Time-synchronisation via IEEE 1588 PTP with sub-
  microsecond precision.
- Oscillation detection — sub-synchronous
  resonance, low-frequency inter-area oscillations,
  and forced-oscillation source identification.
- State-estimation enhancement — synchrophasor data
  improves the SCADA-only state-estimator
  observability.

## §14 Climate-and-Decarbonisation Tracking Discipline

For operators tracking the energy-cloud's
contribution to decarbonisation:

- Emission-intensity tracking — per-MWh CO2e
  computed against the operator's grid mix.
- Renewable-portfolio standard tracking — REC
  balance against jurisdictional RPS targets.
- Avoided-emissions accounting — DR program
  performance, behind-the-meter solar uplift, and
  EV-charging managed-load shifting are translated
  into avoided-emissions metrics.

The decarbonisation metrics feed the operator's
WIA-esg-finance disclosure record (ISSB IFRS S2 +
ESRS E1) where the operator is a CSRD or ISSB
reporting entity.

## §15 Renewable-Resource Integration Discipline

For high-renewable-penetration grids the
integration discipline:

- Solar-PV ramp-rate limits enforced at the inverter
  level via IEEE 2030.5 DERControl ramp-rate fields.
- Wind-resource frequency-response capability per
  IEEE 1547-2018 Category B + NERC BAL-003-2.
- Storage-coupled smoothing — co-located storage
  participates in inverter-level smoothing of solar
  / wind ramps.
- Synchronous-condenser retirement coordination —
  as conventional plants retire the operator
  coordinates inertia and short-circuit current
  contribution from grid-forming inverters.

## §16 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the CIM lifecycle and wholesale-market
participation, satisfy the IEC 62351 + NERC CIP +
DOE C2M2 cybersecurity discipline, and exercise the
operations-event and reliability-reporting discipline
under the operating jurisdiction's regulatory
regime.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-energy-cloud
- **Last Updated:** 2026-04-28
