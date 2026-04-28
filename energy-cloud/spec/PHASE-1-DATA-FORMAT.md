# WIA-energy-cloud PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-energy-cloud
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-energy-cloud. The standard covers
persistent record shapes for the lifecycle of an
energy-cloud operator — the utility-side cloud
platform that hosts ADMS (Advanced Distribution
Management System), DERMS (Distributed Energy
Resources Management System), EMS (Energy Management
System), MDMS (Meter Data Management System), OMS
(Outage Management System), and the analytical /
forecasting workloads that ride on top. Whereas the
WIA-distributed-energy standard governs the per-DER
asset interconnection and the device-to-cloud
protocols, this standard governs the cloud-side
substrate that aggregates fleet data, exposes utility-
operations APIs, integrates with the wholesale
market, and supports the customer-facing engagement
channels. Records are consumed by the utility's
distribution-operations team, the utility's customer-
service team, the wholesale-market settlement
function, the third-party application catalogue, the
supervisory authority for the operating jurisdiction,
external auditors, and the building-and-customer
energy-management systems that interact with the
utility through standardised APIs.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27019:2024 (information security in the
  energy industry)
- ISO 50001:2018 (energy management systems)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- IEC 61968 series (Application integration at
  electric utilities — system interfaces for
  distribution management) including IEC 61968-1
  (interface architecture), -3 (network operations),
  -4 (records and asset management), -8 (customer
  support), -9 (meter reading and control), -11
  (CIM extensions for distribution), -13 (CIM RDF
  model exchange), -100 (implementation profiles)
- IEC 61970 series (Energy management system
  application program interface — EMS-API) including
  IEC 61970-1 (guidelines and principles), -301
  (Common Information Model base), -452 (CIM static
  transmission network model profiles), -453 (CIM-
  based graphics exchange), -456 (solved power
  system state profiles)
- IEC 61850 series (Communication networks and
  systems for power utility automation), in
  particular IEC 61850-7-420:2021 (DER object
  models), IEC 61850-90-7 (object models for power
  converters in DER systems), IEC 61850-7-2 (ACSI)
- IEC 62325 series (Framework for energy market
  communications)
- IEC 62351 series (Power systems management and
  associated information exchange — Data and
  communications security)
- IEEE 1547-2018 + IEEE 2030.5-2018 (referenced
  cross-domain to WIA-distributed-energy)
- OpenADR Alliance OpenADR 2.0a / 2.0b
- Open Charge Alliance OCPP 2.0.1
- W3C Trace Context, W3C Verifiable Credentials
- OASIS Open Smart Grid Platform conventions
- Project Haystack semantic-tagging convention for
  building-and-equipment data
- NERC CIP-002 through CIP-014 (Critical
  Infrastructure Protection)
- NERC BAL (Resource and Demand Balancing), IRO
  (Interconnection Reliability Operations and
  Coordination), COM (Communications), EOP
  (Emergency Preparedness and Operations) standards
- US DOE OE Grid Modernization Initiative + DOE
  C2M2 (Cybersecurity Capability Maturity Model)
- KR 한국전력공사 + KEPCO Cloud + KEPIC + KR
  전기사업법 + KR 전력시장운영규칙 + KPX 전력거래소

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts an energy-cloud operator (a utility
operating its own private cloud, a third-party
cloud-vendor hosting utility workloads, a regional
DERMS / ADMS service provider, a wholesale-market
settlement-engine operator) maintains:

- The cloud-platform tenancy and inventory record.
- The CIM (Common Information Model) network model
  record under IEC 61970-301 + IEC 61968-11.
- The fleet aggregation and DER summary record.
- The forecasting-and-scheduling record.
- The wholesale-market participation record.
- The customer-engagement and program-enrolment
  record.
- The third-party application catalogue record.
- The operations-event and outage record.
- The cybersecurity posture record.
- The supervisory-correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("utility-private-
                       cloud" | "cloud-vendor-
                       utility-workload" | "regional-
                       derms-service" | "regional-
                       adms-service" | "wholesale-
                       market-engine" | "third-
                       party-aggregator-platform" |
                       "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("IEC-61968-1
                       -3-4-8-9-11-13-100" |
                       "IEC-61970-1-301-452-453-456"
                       | "IEC-61850-7-420" |
                       "IEC-62325" | "IEC-62351" |
                       "IEEE-1547-2018" |
                       "IEEE-2030-5-2018" |
                       "OPENADR-2-0B" |
                       "OCPP-2-0-1" |
                       "OASIS-OPEN-SMART-GRID" |
                       "PROJECT-HAYSTACK" |
                       "NERC-CIP-002-014" |
                       "NERC-BAL" | "NERC-IRO" |
                       "NERC-COM" | "NERC-EOP" |
                       "DOE-C2M2" | "ISO-50001" |
                       "ISO-27001" | "ISO-27019" |
                       "KR-한국전력공사" |
                       "KR-KEPIC" | "KR-전기사업법"
                       | "KR-전력시장운영규칙" |
                       "KR-KPX-전력거래소" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 CIM Network Model Record (IEC 61970-301 + IEC
       61968-11)

The CIM network model is the canonical electric-
network representation:

```
cimNetworkModel:
  modelId            : string (uuidv7)
  cimProfileRef      : enum ("iec-61970-452-cgmes"
                       | "iec-61968-13" |
                       "iec-61968-100" |
                       "user-defined")
  effectiveFrom      : string (ISO 8601)
  topologySnapshotRef : string (URI of the topology
                       RDF model — IEC 61970-501)
  ssiSnapshotRef     : string (the steady-state
                       solution profile per IEC
                       61970-456)
  cimDifferenceRef   : string (the model-update
                       diff against the previous
                       version)
  approvedBy         : string (the operator's network-
                       model committee reference)
```

## §4 Tenancy and Customer-Site Record

```
tenancyRecord:
  tenancyId          : string (uuidv7)
  customerKind       : enum ("residential" | "small-
                       commercial" | "large-
                       commercial" | "industrial" |
                       "municipal" | "agricultural"
                       | "user-defined")
  premiseRef         : object (the geographic
                       address + service-territory
                       feeder reference)
  meterIds           : array of string (ANSI C12.18
                       / IEC 62056 meter
                       identifiers)
  derAssetRefs       : array of string (the WIA-
                       distributed-energy asset
                       references behind this
                       customer-site)
  programEnrolments  : array of object (per-program
                       enrolment — net-metering,
                       time-of-use, demand-response,
                       virtual-power-plant)
```

## §5 DER Fleet Aggregation Record

```
derFleetAggregation:
  aggregationId      : string (uuidv7)
  cimNetworkModelRef : string (PHASE-1 §3)
  scope              : enum ("feeder-level" |
                       "substation-level" |
                       "balancing-area" |
                       "wholesale-market-area" |
                       "user-defined")
  effectiveFrom      : string (ISO 8601)
  aggregateNamePlate : object (sum-of-asset rated
                       capacity by DER kind)
  aggregateAvailability : object (real-time
                       availability summary; updated
                       on the operator's published
                       cadence)
  forecastRef        : array of string (PHASE-1 §6
                       forecast references)
```

## §6 Forecasting and Scheduling Record

```
forecastRecord:
  forecastId         : string (uuidv7)
  forecastKind       : enum ("solar-irradiance" |
                       "wind-resource" | "load-
                       net-of-distributed-energy" |
                       "ev-charging-load" |
                       "wholesale-price" |
                       "battery-arbitrage-
                       opportunity" | "user-defined")
  horizonHours       : integer (typical horizons —
                       1 hour intraday · 24 hours
                       day-ahead · 7 days week-ahead
                       · 1 month month-ahead)
  granularityMinutes : integer (typical 5 / 15 / 60)
  forecastMadeAt     : string (ISO 8601)
  forecastValuesRef  : string (URI of the forecast
                       time-series payload)
  forecastErrorMetricsRef : string (URI of the
                       per-horizon MAE / RMSE / pinball-
                       loss metrics report — for
                       probabilistic forecasts the
                       80% / 90% / 95% prediction
                       interval coverage is also
                       reported)
```

## §7 Wholesale-Market Participation Record

```
marketParticipation:
  participationId    : string (uuidv7)
  marketRef          : enum ("ferc-rto-caiso" |
                       "ferc-rto-pjm" | "ferc-rto-
                       miso" | "ferc-rto-ercot" |
                       "ferc-rto-nyiso" | "ferc-rto
                       -isone" | "ferc-rto-spp" |
                       "kr-kpx-전력거래소" |
                       "user-defined")
  registeredResourceRef : string (the RTO / ISO /
                       KPX-side registered-resource
                       identifier)
  productKinds       : array of enum ("energy-day-
                       ahead" | "energy-real-time" |
                       "regulation-up" | "regulation
                       -down" | "spinning-reserve" |
                       "non-spinning-reserve" |
                       "capacity" | "demand-response
                       -emergency" | "demand-response
                       -economic" | "user-defined")
  bidRef             : array of string (per-interval
                       bid reference)
  awardRef           : array of string (per-interval
                       award reference)
  settlementRef      : array of string (per-billing-
                       period settlement reference)
```

## §8 Customer-Engagement Record

```
engagementRecord:
  engagementId       : string (uuidv7)
  tenancyRef         : string
  channelKind        : enum ("billing-portal" |
                       "energy-savings-app" |
                       "demand-response-app" |
                       "outage-notification-sms" |
                       "smart-thermostat-bring-
                       your-own-device" | "rooftop-
                       solar-customer-portal" |
                       "ev-charging-app" |
                       "user-defined")
  consentRecordsRef  : array of string (the consent
                       captures driving program
                       enrolment)
  programEnrolmentRef : string (PHASE-1 §4)
  contactPreference  : object (per-channel contact
                       preferences — email / sms /
                       push / postal mail)
```

## §9 Third-Party Application Catalogue Record

```
appCatalogueRecord:
  appId              : string (uuidv7)
  developerLegalEntity : string
  appKind            : enum ("der-management-app" |
                       "demand-response-aggregation
                       -app" | "ev-fleet-management
                       -app" | "energy-analytics-app"
                       | "smart-thermostat-app" |
                       "building-energy-management"
                       | "user-defined")
  scopesRequested    : array of string (the OAuth
                       scopes the app exercises —
                       per-tenancy energy-data read,
                       per-tenancy DR enrolment
                       write, per-tenancy DER
                       dispatch write)
  approvedAt         : string (ISO 8601)
  approvedBy         : string (the operator's app-
                       review committee reference)
```

## §10 Operations-Event and Outage Record

```
operationsEvent:
  eventId            : string (uuidv7)
  eventKind          : enum ("planned-outage" |
                       "unplanned-outage" |
                       "automatic-restoration" |
                       "manual-restoration" |
                       "feeder-overload" |
                       "voltage-violation" |
                       "frequency-event-bal-001-2"
                       | "icp-rto-system-emergency"
                       | "user-defined")
  detectedAt         : string (ISO 8601)
  affectedFeederRef  : array of string
  affectedCustomerCount : integer (estimated)
  rootCauseRef       : string (URI of the root-cause
                       narrative)
  restoredAt         : string (ISO 8601; absent
                       until restored)
  nercReportRef      : string (URI of the NERC EOP
                       / OE-417 report; absent unless
                       reportable)
```

## §11 Cybersecurity Posture Record

```
cybersecurityPosture:
  postureId          : string (uuidv7)
  cipImpactClass     : enum ("low" | "medium" |
                       "high" | "n/a-non-bes")
  applicableNercCipStandards : array of string
  iec62351ProfilesApplied : array of string
  doeC2m2MaturityIndicatorLevel : enum ("mil-1" |
                       "mil-2" | "mil-3")
  postureReportedAt  : string (ISO 8601)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for the operator's
tenancy and DER fleet, exercise the CIM model-
update discipline on the operating cadence, satisfy
the wholesale-market participation reporting under
the relevant RTO / ISO / KPX rules, and preserve
the records under the operating jurisdiction's
recordkeeping discipline (NERC CIP retention; FERC
retention; KR 전기사업법 + KPX 보존).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-energy-cloud
- **Last Updated:** 2026-04-28
