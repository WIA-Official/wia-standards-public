# WIA-CITY-010 PHASE 3 — Protocol Specification

**Standard:** WIA-CITY-010
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 9. 에너지 효율

### 9.1 성능 지표

#### COP (Coefficient of Performance)
```yaml
cop_measurement:
  definition: "출력 / 입력"

  cooling_mode:
    calculation: "냉방능력_kw / 소비전력_kw"
    typical_range: [2.5, 4.5]
    excellent: ">4.0"

  heating_mode:
    calculation: "난방능력_kw / 소비전력_kw"
    typical_range: [3.0, 5.0]
    excellent: ">4.5"
```

#### SEER (Seasonal Energy Efficiency Ratio)
```yaml
seer_rating:
  definition: "계절별 냉방 총량 / 소비 전력 총량"
  unit: "BTU/Wh"

  ratings:
    minimum: 13
    standard: 14-16
    high_efficiency: 17-20
    ultra_high: ">20"

  calculation_conditions:
    outdoor_temp_range_f: [65, 104]
    test_conditions:
      - temp_f: 82
        weight_percent: 28
      - temp_f: 92
        weight_percent: 45
      - temp_f: 102
        weight_percent: 27
```

### 9.2 에너지 절감 전략

```yaml
energy_saving_strategies:
  - strategy: "Optimal Start/Stop"
    description: "건물 열 특성 학습하여 최적 시작/종료 시간 결정"
    potential_savings_percent: 10-20

  - strategy: "Demand Response"
    description: "전력 피크 시간대 부하 감소"
    potential_savings_percent: 15-25

  - strategy: "Free Cooling"
    description: "외기 온도 낮을 때 칠러 대신 외기 사용"
    potential_savings_percent: 20-40

  - strategy: "Variable Speed Control"
    description: "팬, 펌프 인버터 제어"
    potential_savings_percent: 30-50

  - strategy: "Heat Recovery"
    description: "배기 열 회수"
    potential_savings_percent: 15-30

  - strategy: "Occupancy-based Control"
    description: "재실 감지 기반 제어"
    potential_savings_percent: 20-35
```

### 9.3 에너지 모니터링

```yaml
energy_monitoring:
  metering:
    - meter_id: "EM-HVAC-01"
      type: "ELECTRIC"
      measurement:
        - "ACTIVE_POWER_KW"
        - "REACTIVE_POWER_KVAR"
        - "POWER_FACTOR"
        - "ENERGY_KWH"
      interval_s: 60

  kpi:
    - metric: "EUI"
      name: "Energy Use Intensity"
      unit: "kWh/m²/year"
      target: "<120"

    - metric: "Cooling_Specific_Power"
      unit: "kW/ton"
      target: "<0.6"

    - metric: "PUE"
      name: "Power Usage Effectiveness"
      calculation: "Total_Power / IT_Power"
      target: "<1.5"

  reporting:
    frequency: "DAILY"
    benchmarking: true
    alerts:
      - threshold_exceeded
      - anomaly_detected
      - target_missed
```

---

## 10. 프로토콜 통합

### 10.1 BACnet

```yaml
bacnet_integration:
  protocol_version: "1.24"
  device_profile: "BACnet Application Specific Controller (B-ASC)"

  objects:
    - object_type: "ANALOG_INPUT"
      instances:
        - id: 1
          name: "Zone Temperature"
          units: "DEGREES_CELSIUS"

    - object_type: "ANALOG_OUTPUT"
      instances:
        - id: 1
          name: "Cooling Valve Position"
          units: "PERCENT"

    - object_type: "ANALOG_VALUE"
      instances:
        - id: 1
          name: "Temperature Setpoint"
          units: "DEGREES_CELSIUS"

    - object_type: "BINARY_INPUT"
      instances:
        - id: 1
          name: "Occupancy Status"

    - object_type: "BINARY_OUTPUT"
      instances:
        - id: 1
          name: "Fan Enable"

  services:
    - "ReadProperty"
    - "WriteProperty"
    - "SubscribeCOV"
    - "GetAlarmSummary"
    - "GetEventInformation"

  network:
    transport: "BACnet/IP"
    port: 47808
    bbmd_enabled: true
```

### 10.2 Modbus

```yaml
modbus_integration:
  protocol: "Modbus TCP"
  port: 502

  register_map:
    holding_registers:
      - address: 40001
        name: "Zone_Temp_Setpoint"
        unit: "°C"
        scale: 0.1
        writable: true

      - address: 40002
        name: "Operating_Mode"
        values:
          0: "OFF"
          1: "COOLING"
          2: "HEATING"
          3: "AUTO"
        writable: true

    input_registers:
      - address: 30001
        name: "Zone_Temperature"
        unit: "°C"
        scale: 0.1

      - address: 30002
        name: "Zone_Humidity"
        unit: "%RH"
        scale: 0.1

      - address: 30003
        name: "Power_Consumption"
        unit: "kW"
        scale: 0.01

    coils:
      - address: 00001
        name: "System_Enable"
        writable: true

    discrete_inputs:
      - address: 10001
        name: "Alarm_Status"
```

### 10.3 KNX

```yaml
knx_integration:
  protocol_version: "KNX/IP"

  group_addresses:
    - ga: "1/1/1"
      dpt: "DPT_Switch"
      name: "HVAC_On_Off"

    - ga: "1/1/2"
      dpt: "DPT_Value_Temp"
      name: "Zone_Temperature"

    - ga: "1/1/3"
      dpt: "DPT_Scaling"
      name: "Fan_Speed_Percent"

  telegram_handling:
    read: true
    write: true
    respond: true
```

### 10.4 LonWorks

```yaml
lonworks_integration:
  protocol: "LonTalk"

  network_variables:
    - nv_name: "nviSpaceTemp"
      snvt: "SNVT_temp_p"
      direction: "INPUT"

    - nv_name: "nvoSpaceTemp"
      snvt: "SNVT_temp_p"
      direction: "OUTPUT"

    - nv_name: "nviSetpoint"
      snvt: "SNVT_temp_setpt"
      direction: "INPUT"
```

---

## 11. 예지 정비

### 11.1 상태 기반 모니터링

```yaml
condition_monitoring:
  parameters:
    - name: "Compressor Vibration"
      sensor_type: "ACCELEROMETER"
      threshold_mm_s: 7.1
      alert_level: "CRITICAL"

    - name: "Bearing Temperature"
      sensor_type: "TEMPERATURE"
      threshold_c: 75
      alert_level: "WARNING"

    - name: "Refrigerant Pressure"
      sensor_type: "PRESSURE"
      range_bar: [15, 25]
      alert_level: "CRITICAL"

    - name: "Oil Level"
      sensor_type: "LEVEL"
      min_percent: 30
      alert_level: "WARNING"

    - name: "Motor Current"
      sensor_type: "CURRENT"
      baseline_tolerance_percent: 20
      alert_level: "WARNING"
```

### 11.2 고장 예측

```yaml
fault_prediction:
  algorithms:
    - type: "MACHINE_LEARNING"
      model: "LSTM"
      training_data_months: 12
      prediction_horizon_days: 30

  failure_modes:
    - component: "Compressor"
      indicators:
        - "Increased vibration"
        - "Higher discharge temp"
        - "Lower efficiency"
      mtbf_hours: 50000

    - component: "Fan Motor"
      indicators:
        - "Bearing noise"
        - "Higher current"
        - "Reduced airflow"
      mtbf_hours: 40000

    - component: "Refrigerant Leak"
      indicators:
        - "Low pressure"
        - "Reduced cooling"
        - "Frost formation"
      detection_method: "PRESSURE_MONITORING"
```

### 11.3 정비 스케줄링

```yaml
maintenance_schedule:
  preventive:
    - task: "Filter Replacement"
      frequency_months: 3
      duration_hours: 1

    - task: "Coil Cleaning"
      frequency_months: 6
      duration_hours: 4


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
