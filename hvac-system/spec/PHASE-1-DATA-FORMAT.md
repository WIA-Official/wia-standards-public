# WIA-CITY-010 PHASE 1 — Data Format Specification

**Standard:** WIA-CITY-010
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-CITY-010: 냉난방 시스템 표준 v1.0 ❄️

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

## 문서 정보

- **표준 번호**: WIA-CITY-010
- **표준 명칭**: HVAC System Standard (냉난방 시스템 표준)
- **버전**: 1.0
- **상태**: Active
- **발행일**: 2025-12-25
- **발행처**: WIA (World Certification Industry Association)
- **카테고리**: CITY (도시 인프라)
- **라이선스**: MIT

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [참조 표준](#3-참조-표준)
4. [용어 정의](#4-용어-정의)
5. [HVAC 시스템 유형](#5-hvac-시스템-유형)
6. [존 관리](#6-존-관리)
7. [온도 및 습도 제어](#7-온도-및-습도-제어)
8. [공기질 통합](#8-공기질-통합)
9. [에너지 효율](#9-에너지-효율)
10. [프로토콜 통합](#10-프로토콜-통합)
11. [예지 정비](#11-예지-정비)
12. [데이터 모델](#12-데이터-모델)
13. [API 명세](#13-api-명세)
14. [보안 요구사항](#14-보안-요구사항)
15. [구현 가이드](#15-구현-가이드)

---

## 1. 개요

### 1.1 목적

WIA-CITY-010은 건물 냉난방 시스템(HVAC)의 모니터링, 제어, 최적화를 위한 포괄적 표준을 정의합니다. 이 표준은 다양한 HVAC 시스템 유형, 제어 프로토콜, 에너지 관리를 통합하여 효율적이고 쾌적한 실내 환경을 제공합니다.

### 1.2 배경

건물 에너지 소비의 40-60%가 HVAC 시스템에서 발생합니다. 스마트 빌딩 시대에 HVAC 시스템의 통합 관리는 다음을 가능하게 합니다:

- **에너지 절감**: 20-40% 에너지 소비 감소
- **쾌적성 향상**: 정밀한 온습도 제어
- **예방 정비**: 장비 수명 20-30% 연장
- **통합 관리**: 단일 플랫폼에서 다중 시스템 관리

### 1.3 핵심 가치

**弘益人間 (홍익인간)** 철학에 따라, 이 표준은:

- 모든 건물 거주자에게 쾌적한 환경 제공
- 에너지 효율을 통한 환경 보호
- 표준화를 통한 시스템 상호운용성 보장
- 지속가능한 건물 운영 지원

---

## 2. 적용 범위

### 2.1 대상 시스템

이 표준은 다음 HVAC 시스템에 적용됩니다:

- **분리형 에어컨** (Split AC)
- **멀티 시스템** (Multi-Split)
- **VRF/VRV** (Variable Refrigerant Flow)
- **칠러 시스템** (Chiller-based)
- **히트펌프** (Heat Pump)
- **공조기** (AHU - Air Handling Unit)
- **팬코일 유닛** (FCU - Fan Coil Unit)
- **복사 냉난방** (Radiant Heating/Cooling)
- **지열 시스템** (Geothermal)

### 2.2 대상 건물

- 주거용 건물 (아파트, 주택)
- 상업용 건물 (오피스, 쇼핑몰)
- 산업 시설
- 병원 및 의료 시설
- 교육 시설 (학교, 대학)
- 데이터센터
- 호텔 및 숙박 시설

### 2.3 제외 사항

다음은 이 표준의 범위에서 제외됩니다:

- HVAC 설비의 기계적 설계
- 전기 배선 및 설치
- 건축 구조 설계

---

## 3. 참조 표준

### 3.1 국제 표준

- **ASHRAE 90.1**: Energy Standard for Buildings
- **ASHRAE 55**: Thermal Environmental Conditions
- **ISO 16484**: Building Automation and Control Systems (BACS)
- **ISO 50001**: Energy Management Systems
- **BACnet (ISO 16484-5)**: Building Automation Protocol
- **Modbus**: Industrial Communication Protocol
- **EN 15232**: Energy Performance of Buildings (EPBD)

### 3.2 WIA 표준

- **WIA-ENE-027**: Indoor Air Quality (실내 공기질)
- **WIA-CITY-001**: Smart Building Platform (스마트 빌딩)
- **WIA-ENE-001**: Energy Monitoring (에너지 모니터링)
- **WIA-INTENT**: Intent Expression Standard

### 3.3 산업 표준

- AHRI 550/590: Performance Rating of Water-Chilling Packages
- SEER/EER: Seasonal Energy Efficiency Ratio
- COP: Coefficient of Performance

---

## 4. 용어 정의

### 4.1 기본 용어

| 용어 | 정의 |
|------|------|
| **HVAC** | Heating, Ventilation, and Air Conditioning (난방, 환기, 냉방) |
| **존 (Zone)** | 독립적으로 제어되는 공간 단위 |
| **설정온도 (Setpoint)** | 목표 온도 값 |
| **데드밴드 (Deadband)** | 냉난방이 작동하지 않는 온도 범위 |
| **VAV** | Variable Air Volume (가변 풍량) |
| **CAV** | Constant Air Volume (정풍량) |
| **BMS** | Building Management System |

### 4.2 시스템 용어

| 용어 | 정의 |
|------|------|
| **VRF** | Variable Refrigerant Flow (가변 냉매 유량) |
| **AHU** | Air Handling Unit (공조기) |
| **FCU** | Fan Coil Unit (팬코일 유닛) |
| **칠러 (Chiller)** | 냉각수 생성 장치 |
| **보일러 (Boiler)** | 온수/증기 생성 장치 |
| **냉각탑 (Cooling Tower)** | 냉각수 냉각 장치 |

### 4.3 성능 지표

| 용어 | 정의 |
|------|------|
| **COP** | Coefficient of Performance (성능계수) |
| **SEER** | Seasonal Energy Efficiency Ratio (계절 에너지 효율) |
| **EER** | Energy Efficiency Ratio (에너지 효율비) |
| **IPLV** | Integrated Part Load Value (부분부하 효율) |
| **PUE** | Power Usage Effectiveness (전력 사용 효율) |

---

## 5. HVAC 시스템 유형

### 5.1 분리형 에어컨 (Split AC)

#### 특징
- 실내기와 실외기로 분리
- 단일 공간 냉난방
- 주거 및 소규모 상업 공간

#### 제어 파라미터
```yaml
system_type: SPLIT_AC
components:
  - indoor_unit:
      model: "IU-2024"
      capacity_btu: 12000
      capacity_kw: 3.5
  - outdoor_unit:
      model: "OU-2024"
      compressor_type: "INVERTER"

control:
  mode: [COOLING, HEATING, AUTO, DRY, FAN_ONLY]
  temperature_range: [16, 30]  # °C
  fan_speed: [AUTO, LOW, MEDIUM, HIGH, TURBO]
  swing: [AUTO, FIXED, HORIZONTAL, VERTICAL]
```

### 5.2 VRF 시스템

#### 특징
- 하나의 실외기에 다수의 실내기
- 개별 존 제어
- 동시 냉난방 가능
- 높은 에너지 효율

#### 시스템 구성
```yaml
system_type: VRF
capacity_total_kw: 50.0
outdoor_units:
  - unit_id: "ODU-01"
    capacity_kw: 50.0
    compressor_type: "INVERTER_SCROLL"
    refrigerant: "R410A"

indoor_units:
  - unit_id: "IDU-01"
    type: "CEILING_CASSETTE"
    zone_id: "ZONE-101"
    capacity_kw: 5.6
  - unit_id: "IDU-02"
    type: "WALL_MOUNTED"
    zone_id: "ZONE-102"
    capacity_kw: 3.5
  - unit_id: "IDU-03"
    type: "DUCTED"
    zone_id: "ZONE-103"
    capacity_kw: 7.1

features:
  - simultaneous_heating_cooling: true
  - heat_recovery: true
  - individual_zone_control: true
```

### 5.3 칠러 시스템

#### 특징
- 중대형 건물용
- 냉각수/온수 순환 방식
- FCU, AHU와 연계

#### 시스템 구성
```yaml
system_type: CHILLER
chiller:
  chiller_id: "CH-01"
  type: "WATER_COOLED"
  compressor_type: "CENTRIFUGAL"
  capacity_ton: 500
  capacity_kw: 1758
  refrigerant: "R134a"
  cop_rated: 6.5
  iplv: 7.2

cooling_tower:
  tower_id: "CT-01"
  type: "OPEN_CIRCUIT"
  capacity_ton: 600
  fan_control: "VFD"

pumps:
  - pump_id: "CHWP-01"
    type: "CHILLED_WATER"
    flow_rate_lpm: 5000
    head_m: 40
    vfd_enabled: true
  - pump_id: "CWP-01"
    type: "CONDENSER_WATER"
    flow_rate_lpm: 6000
    head_m: 35
    vfd_enabled: true

distribution:
  primary_loop:
    supply_temp_c: 7
    return_temp_c: 12
    delta_t: 5
  secondary_loop:
    supply_temp_c: 8
    return_temp_c: 13
```

### 5.4 히트펌프

#### 특징
- 냉난방 겸용
- 높은 에너지 효율
- 지열/공기열/수열 방식

#### 타입
```yaml
heat_pump_types:
  air_source:
    cop_heating: 3.5-4.5
    cop_cooling: 3.0-4.0
    temp_range: [-15, 43]  # °C

  ground_source:
    cop_heating: 4.0-5.0
    cop_cooling: 4.5-5.5
    temp_range: [-5, 35]
    loop_type: [VERTICAL, HORIZONTAL, POND]

  water_source:
    cop_heating: 4.5-5.5
    cop_cooling: 5.0-6.0
    source: [LAKE, RIVER, WELL, SEA]
```

### 5.5 공조기 (AHU)

#### 특징
- 외기 처리 및 순환
- 필터링, 냉각, 가열, 가습
- VAV/CAV 제어

#### 구성
```yaml
system_type: AHU
ahu_id: "AHU-01"
capacity_cmh: 10000  # m³/h
capacity_cfm: 5886

components:
  filters:
    pre_filter: "G4"
    main_filter: "F7"
    final_filter: "H13_HEPA"

  cooling_coil:
    type: "CHILLED_WATER"
    rows: 6
    capacity_kw: 150

  heating_coil:
    type: "HOT_WATER"
    rows: 4
    capacity_kw: 100

  humidifier:
    type: "STEAM"
    capacity_kg_h: 50


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
