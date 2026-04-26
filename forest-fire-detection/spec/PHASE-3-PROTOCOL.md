# WIA-ENE-032 PHASE 3 — Protocol Specification

**Standard:** WIA-ENE-032
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 8. 연기 감지 시스템

### 8.1 광학 연기 감지기

**원리:**
- 레이저 산란 방식 (Laser Scattering)
- 연기 입자에 의한 빛 산란 측정
- 입자 농도 임계값: 0.05 mg/m³

**설치 위치:**
- 산림 내 감시탑
- 등산로 주요 지점
- 휴게소 및 야영장

### 8.2 AI 기반 영상 연기 감지

**기술:**
- CNN (Convolutional Neural Network) 모델
- 연기 패턴 학습 (10만+ 이미지)
- 구름, 안개, 먼지 구분
- 탐지 정확도: 92%+

**입력 데이터:**
- CCTV 실시간 영상
- 드론 영상
- 위성 영상 (저해상도 보조)

---

## 9. 기상 조건 모니터링

### 9.1 필수 기상 요소

| 요소 | 측정 단위 | 업데이트 주기 | 영향도 |
|------|-----------|---------------|--------|
| 온도 (Temperature) | °C | 1분 | 높음 |
| 상대습도 (Humidity) | % | 1분 | 매우 높음 |
| 풍속 (Wind Speed) | m/s | 1분 | 매우 높음 |
| 풍향 (Wind Direction) | ° | 1분 | 높음 |
| 강수량 (Precipitation) | mm | 1시간 | 높음 |
| 기압 (Pressure) | hPa | 10분 | 보통 |
| 일사량 (Solar Radiation) | W/m² | 1분 | 보통 |

### 9.2 자동 기상 관측소 (AWS)

**배치 밀도:**
- 산림 내: 50km² 당 1개소
- 고위험 지역: 20km² 당 1개소

**데이터 전송:**
- 실시간: 1분 간격
- 백업: LoRaWAN, 위성통신

---

## 10. 연료 수분 측정

### 10.1 고사 연료 수분 (DFMC)

**측정 방법:**
- **직접 측정**: 10시간 연료봉 중량법
- **간접 추정**: 기상 데이터 기반 모델링

**위험 임계값:**
- DFMC < 10%: 극도 위험
- DFMC 10-15%: 높은 위험
- DFMC 15-25%: 보통 위험
- DFMC > 25%: 낮은 위험

### 10.2 생체 연료 수분 (LFMC)

**측정 방법:**
- **실험실 분석**: 잎 샘플 채취 후 건조 중량 비교
- **원격 탐사**: NDVI (Normalized Difference Vegetation Index)

**계절별 패턴:**
- 봄철 (3-5월): LFMC 최저 (60-80%) → 산불 다발
- 여름철 (6-8월): LFMC 상승 (80-120%)
- 가을철 (9-11월): LFMC 하강 (70-100%)
- 겨울철 (12-2월): LFMC 최저 (50-70%)

---

## 11. 화재 확산 예측

### 11.1 Rothermel 확산 모델

**기본 방정식:**

```
R = (IR × ξ × (1 + Φw + Φs)) / (ρb × ε × Qig)

여기서:
R = 확산 속도 (m/min)
IR = 반응 강도 (kJ/m²/min)
ξ = 전파 유동비
Φw = 풍속 인자
Φs = 경사도 인자
ρb = 연료 밀도 (kg/m³)
ε = 유효 발열량 비율
Qig = 발화 열량 (kJ/kg)
```

### 11.2 FARSITE 시뮬레이션

**입력 데이터:**
- 지형도 (DEM, Slope, Aspect)
- 연료 유형 맵
- 기상 예보 (72시간)
- 초기 화재 경계

**출력 결과:**
- 시간대별 화재 경계 (GeoJSON)
- 화선 강도 히트맵
- 위협 지역 우선순위

### 11.3 AI 기반 예측

**딥러닝 모델:**
- **아키텍처**: ConvLSTM (시공간 데이터)
- **학습 데이터**: 과거 10년 산불 1,000건
- **정확도**: 85% (24시간 예측)

---

## 12. 대피 구역 관리

### 12.1 대피 구역 등급

| 등급 | 명칭 | 도착 예상 시간 | 조치 |
|------|------|----------------|------|
| RED | 즉시 대피 | 0-2시간 | 강제 대피 명령 |
| ORANGE | 대피 준비 | 2-6시간 | 대피 권고 |
| YELLOW | 주의 | 6-12시간 | 상황 감시 |
| GREEN | 안전 | >12시간 | 정상 활동 |

### 12.2 대피 경로 최적화

**알고리즘:**
- Dijkstra 최단 경로
- 도로 용량 고려
- 실시간 교통 정보 반영
- 장애인/노약자 우선 차량 배정

---

## 13. 소방 자원 관리

### 13.1 자원 유형

| 유형 | 수량 | 배치 기준 |
|------|------|-----------|
| 소방 헬기 | 전국 50대 | 30분 내 도달 |
| 소방차 | 전국 500대 | 20분 내 도달 |
| 소방 인력 | 10,000명 | 권역별 배치 |
| 진화 장비 | - | 전진 기지 비축 |

### 13.2 실시간 추적

```json
{
  "resourceId": "HELI-02",
  "type": "helicopter",
  "status": "en_route",
  "location": {
    "latitude": 37.5700,
    "longitude": 126.9850,
    "altitude": 500
  },
  "capacity": {
    "waterTank": 2000,
    "currentLoad": 1800
  },
  "eta": "2025-04-15T15:10:00Z"
}
```

---


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
