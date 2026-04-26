# WIA-ENE-032 PHASE 1 — Data Format Specification

**Standard:** WIA-ENE-032
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-ENE-032: 산불 감지 표준 v1.0 🔥

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-032
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [산불 위험도 분류](#4-산불-위험도-분류)
5. [데이터 모델](#5-데이터-모델)
6. [위성 영상 감지](#6-위성-영상-감지)
7. [열 감지 시스템](#7-열-감지-시스템)
8. [연기 감지 시스템](#8-연기-감지-시스템)
9. [기상 조건 모니터링](#9-기상-조건-모니터링)
10. [연료 수분 측정](#10-연료-수분-측정)
11. [화재 확산 예측](#11-화재-확산-예측)
12. [대피 구역 관리](#12-대피-구역-관리)
13. [소방 자원 관리](#13-소방-자원-관리)
14. [경보 시스템](#14-경보-시스템)
15. [통합 및 상호운용성](#15-통합-및-상호운용성)
16. [보안 및 개인정보보호](#16-보안-및-개인정보보호)
17. [인증 요구사항](#17-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-032 산불 감지 표준은 산림 화재의 조기 감지, 실시간 모니터링, 신속한 대응을 위한 통합 시스템 표준입니다. 본 표준은 위성 관측, 지상 센서, AI 분석을 결합하여 산불로 인한 인명 및 재산 피해를 최소화하고 산림 생태계를 보호하는 것을 목표로 합니다.

### 1.2 핵심 원칙

- **조기 감지 (Early Detection)**: 산불 발생 즉시 감지 (목표: 10분 이내)
- **정확성 (Accuracy)**: 오탐지율 5% 미만, 미탐지율 1% 미만
- **실시간성 (Real-time)**: 데이터 수집-분석-경보 지연시간 3분 이내
- **다중 센서 융합 (Multi-sensor Fusion)**: 위성, 지상, 드론 데이터 통합
- **AI 예측 (AI Prediction)**: 화재 확산 경로 및 위험도 자동 예측
- **공개 표준 (Open Standard)**: 국제적 상호운용성 보장
- **인명 우선 (Life First)**: 대피 및 구조 최우선

### 1.3 적용 대상

- 산림청 및 지방자치단체 산불 방지 부서
- 국가산불종합상황실 및 지역 상황실
- 위성 관측 기관 (MODIS, VIIRS, Sentinel)
- 기상청 및 기상 예보 기관
- 소방서 및 긴급구조대
- 드론 및 항공 소방 운영자
- 스마트시티 재난 관리 시스템
- 산림 보호 NGO 및 연구 기관

---

## 2. 적용 범위

### 2.1 감지 대상

본 표준은 다음 산불 유형에 적용됩니다:

- **지표화 (Surface Fire)**: 낙엽, 관목 등 지표면 연소
- **수관화 (Crown Fire)**: 나무 수관부 연소 (가장 위험)
- **지중화 (Ground Fire)**: 토탄층, 부식토 연소 (장기 지속)
- **잔불 (Smoldering Fire)**: 불씨 상태로 장시간 연소
- **비화 (Spot Fire)**: 바람에 날린 불씨로 인한 2차 발화

### 2.2 감지 단계

- **예방 단계**: 산불 위험도 예측 및 경보
- **발화 감지**: 최초 화재 지점 식별 (골든타임: 30분)
- **확산 모니터링**: 실시간 화재 진행 추적
- **진화 지원**: 소방 자원 배치 최적화
- **사후 모니터링**: 잔불 감시 및 재발화 방지

### 2.3 지리적 범위

- **산림 지역**: 국립공원, 도립공원, 사유림
- **도시 경계 (WUI)**: 산림과 인접한 주거지역
- **농촌 지역**: 논밭과 인접한 산림
- **자연보호구역**: 생태계 보전 지역

---

## 3. 용어 정의

### 3.1 기본 용어

- **산불 (Wildfire)**: 산림에서 발생하는 통제되지 않은 화재
- **화점 (Hot Spot)**: 위성 또는 센서가 감지한 고온 지점
- **화선 (Fire Line)**: 불이 확산되는 최전선
- **완전 진화 (Containment)**: 100% 화선 통제 완료
- **통제선 (Control Line)**: 화재 확산 방지를 위한 방화선
- **골든타임 (Golden Time)**: 초기 진화 가능한 시간 (통상 30분)

### 3.2 기술 용어

- **MODIS (Moderate Resolution Imaging Spectroradiometer)**: NASA의 중해상도 위성 센서 (Terra, Aqua 위성 탑재)
- **VIIRS (Visible Infrared Imaging Radiometer Suite)**: 가시광선-적외선 영상 관측 장비 (Suomi NPP, NOAA-20 위성)
- **Sentinel**: 유럽우주국(ESA)의 지구관측 위성 시리즈
- **FRP (Fire Radiative Power)**: 화재 복사 에너지 (MW 단위)
- **FWI (Fire Weather Index)**: 캐나다 산불 기상 지수
- **NFDRS (National Fire Danger Rating System)**: 미국 국가 화재위험도 평가 시스템
- **EFFIS (European Forest Fire Information System)**: 유럽 산불 정보 시스템
- **DFMC (Dead Fuel Moisture Content)**: 고사 연료 수분 함량
- **LFMC (Live Fuel Moisture Content)**: 생체 연료 수분 함량

### 3.3 센서 용어

- **열화상 카메라 (Thermal Camera)**: 적외선으로 온도 분포 촬영
- **연기 감지기 (Smoke Detector)**: 입자 또는 이온화 방식 연기 센서
- **기상 센서 (Weather Sensor)**: 온도, 습도, 풍속, 풍향 측정
- **수분 센서 (Moisture Sensor)**: 토양 및 연료 수분 측정

---

## 4. 산불 위험도 분류

### 4.1 위험도 등급

| 등급 | 명칭 | 색상 | FWI 범위 | 발화 확률 | 대응 조치 |
|------|------|------|----------|-----------|-----------|
| 1 | 낮음 (Low) | 🟢 녹색 | 0-5 | <5% | 정상 감시 |
| 2 | 보통 (Moderate) | 🟡 황색 | 6-12 | 5-15% | 주의 관찰 |
| 3 | 높음 (High) | 🟠 주황 | 13-22 | 15-35% | 경계 태세 |
| 4 | 매우 높음 (Very High) | 🔴 적색 | 23-38 | 35-60% | 통제 조치 (입산 통제) |
| 5 | 극도 (Extreme) | 🟣 보라 | 39-50+ | >60% | 비상 태세 (산림 폐쇄) |

### 4.2 위험도 계산 요소

```typescript
interface FireDangerCalculation {
  // 기상 조건 (가중치 40%)
  temperature: number;        // 온도 (°C)
  humidity: number;           // 상대습도 (%)
  windSpeed: number;          // 풍속 (m/s)
  precipitation: number;      // 강수량 (mm, 최근 7일)

  // 연료 상태 (가중치 30%)
  deadFuelMoisture: number;   // 고사 연료 수분 (%)
  liveFuelMoisture: number;   // 생체 연료 수분 (%)
  fuelLoad: number;           // 연료량 (ton/ha)

  // 지형 조건 (가중치 20%)
  slope: number;              // 경사도 (°)
  aspect: number;             // 사면 방향 (°)
  elevation: number;          // 고도 (m)

  // 인간 활동 (가중치 10%)
  humanActivity: number;      // 활동 빈도 (0-1)
  proximity: number;          // 인구 밀집지 거리 (km)
}
```

---


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
