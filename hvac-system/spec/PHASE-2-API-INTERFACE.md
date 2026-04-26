# WIA-CITY-010 PHASE 2 — API Interface Specification

**Standard:** WIA-CITY-010
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

supply_fan:
    type: "CENTRIFUGAL"
    motor_kw: 15
    vfd_enabled: true

  return_fan:
    type: "CENTRIFUGAL"
    motor_kw: 11
    vfd_enabled: true

control:
  type: "VAV"
  supply_temp_control: "RESET"
  supply_temp_setpoint_c: 14
  reset_schedule:
    outdoor_temp_c: [10, 15, 20, 25, 30]
    supply_temp_c: [18, 16, 14, 12, 10]

  ventilation:
    mode: "DEMAND_CONTROLLED"
    co2_setpoint_ppm: 800
    min_outdoor_air_percent: 20
    economizer_enabled: true
```

---

## 6. 존 관리

### 6.1 존 정의

존(Zone)은 독립적으로 제어되는 공간 단위입니다.

```yaml
zone:
  zone_id: "ZONE-301"
  name: "3층 회의실 A"
  floor: 3
  building: "본관"

  geometry:
    area_m2: 45.0
    volume_m3: 121.5
    ceiling_height_m: 2.7

  occupancy:
    max_occupancy: 12
    occupancy_sensor: true

  thermal:
    design_cooling_load_kw: 5.4
    design_heating_load_kw: 4.2
    thermal_mass: "MEDIUM"

  equipment:
    - type: "VRF_INDOOR_UNIT"
      unit_id: "IDU-301"
    - type: "THERMOSTAT"
      device_id: "TSTAT-301"
    - type: "CO2_SENSOR"
      device_id: "CO2-301"
```

### 6.2 멀티존 제어

#### 존 우선순위
```yaml
zone_priority:
  critical:
    - "서버실"
    - "수술실"
    - "클린룸"
  high:
    - "회의실"
    - "사무실"
  normal:
    - "복도"
    - "로비"
  low:
    - "창고"
    - "주차장"
```

#### 존간 상호작용
```yaml
zone_interaction:
  - source_zone: "ZONE-301"
    adjacent_zones:
      - zone_id: "ZONE-302"
        wall_type: "PARTITION"
        u_value: 0.5
      - zone_id: "CORRIDOR-3F"
        door_type: "GLASS"
        air_transfer: true
```

### 6.3 스케줄링

```yaml
zone_schedule:
  zone_id: "ZONE-301"
  schedules:
    - name: "평일 근무"
      days: [MON, TUE, WED, THU, FRI]
      periods:
        - start: "07:00"
          end: "09:00"
          mode: "PREHEAT/PRECOOL"
          temp_setpoint_c: 22

        - start: "09:00"
          end: "18:00"
          mode: "OCCUPIED"
          temp_setpoint_c: 22
          humidity_setpoint_percent: 50

        - start: "18:00"
          end: "22:00"
          mode: "SETBACK"
          temp_setpoint_c: 25

        - start: "22:00"
          end: "07:00"
          mode: "UNOCCUPIED"
          temp_setpoint_c: 28

    - name: "주말"
      days: [SAT, SUN]
      periods:
        - start: "00:00"
          end: "24:00"
          mode: "UNOCCUPIED"
          temp_setpoint_c: 28
```

---

## 7. 온도 및 습도 제어

### 7.1 온도 제어

#### 설정온도 범위
```yaml
temperature_control:
  cooling:
    min_setpoint_c: 18
    max_setpoint_c: 28
    default_setpoint_c: 24
    deadband_c: 2

  heating:
    min_setpoint_c: 16
    max_setpoint_c: 26
    default_setpoint_c: 22
    deadband_c: 2

  comfort_range:
    summer:
      optimal: [23, 26]  # °C
      acceptable: [20, 28]
    winter:
      optimal: [20, 23]
      acceptable: [18, 25]
```

#### 제어 알고리즘
```yaml
control_algorithm:
  type: "PID"
  parameters:
    proportional_gain: 0.5
    integral_time_s: 300
    derivative_time_s: 60

  anti_windup:
    enabled: true
    max_integral: 100

  output:
    type: "MODULATING"
    range: [0, 100]  # %
    update_interval_s: 10
```

### 7.2 습도 제어

```yaml
humidity_control:
  enabled: true

  setpoints:
    min_rh_percent: 40
    max_rh_percent: 60
    optimal_rh_percent: 50

  equipment:
    humidifier:
      type: "STEAM"
      capacity_kg_h: 50
      modulation: [0, 100]  # %

    dehumidifier:
      type: "COOLING_COIL_REHEAT"
      enabled: true

  control:
    priority: "SECONDARY"  # Temperature has priority
    deadband_percent: 5
    max_deviation_percent: 10
```

### 7.3 온습도 통합 제어

```yaml
integrated_control:
  psychrometric_control:
    enabled: true
    control_parameter: "ENTHALPY"  # or DEWPOINT

  sequential_control:
    - condition: "temp_high"
      action: "INCREASE_COOLING"

    - condition: "temp_low"
      action: "INCREASE_HEATING"

    - condition: "humidity_high"
      actions:
        - "INCREASE_COOLING"
        - "ENABLE_REHEAT"

    - condition: "humidity_low"
      action: "ACTIVATE_HUMIDIFIER"
```

---

## 8. 공기질 통합

### 8.1 공기질 파라미터

HVAC 시스템은 WIA-ENE-027 실내 공기질 표준과 통합됩니다.

```yaml
air_quality_integration:
  monitored_parameters:
    - parameter: "CO2"
      threshold_ppm: 1000
      action: "INCREASE_VENTILATION"

    - parameter: "PM2.5"
      threshold_ugm3: 35
      action: "INCREASE_FILTRATION"

    - parameter: "VOC"
      threshold_ppb: 500
      action: "INCREASE_OUTDOOR_AIR"

    - parameter: "HUMIDITY"
      threshold_percent: 60
      action: "ACTIVATE_DEHUMIDIFICATION"
```

### 8.2 환기 제어

```yaml
ventilation_control:
  mode: "DEMAND_CONTROLLED_VENTILATION"

  outdoor_air:
    min_flow_percent: 20
    max_flow_percent: 100

  co2_based:
    enabled: true
    setpoint_ppm: 800
    control_range_ppm: [400, 1200]

  economizer:
    enabled: true
    type: "DIFFERENTIAL_ENTHALPY"
    lockout_temp_c: 21

  strategies:
    - name: "Night Purge"
      enabled: true
      schedule: "02:00-06:00"
      outdoor_temp_range_c: [15, 25]

    - name: "Pre-Occupancy Flush"
      enabled: true
      duration_minutes: 30
      outdoor_air_percent: 100
```

### 8.3 필터 관리

```yaml
filter_management:
  filters:
    - filter_id: "PRE-FILTER-01"
      type: "G4"
      efficiency_percent: 90
      pressure_drop_initial_pa: 50
      pressure_drop_final_pa: 200

    - filter_id: "MAIN-FILTER-01"
      type: "F7"
      efficiency_percent: 85
      pressure_drop_initial_pa: 100
      pressure_drop_final_pa: 300

    - filter_id: "HEPA-FILTER-01"
      type: "H13"
      efficiency_percent: 99.95
      pressure_drop_initial_pa: 200
      pressure_drop_final_pa: 500

  replacement:
    criteria:
      - "PRESSURE_DROP"
      - "TIME_BASED"
      - "PARTICLE_COUNT"

    pressure_threshold_pa: 250
    time_threshold_hours: 2160  # 3 months

  monitoring:
    pressure_sensor: true
    particle_counter: true
    alert_threshold_days: 7
```

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
