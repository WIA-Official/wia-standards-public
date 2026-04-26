# WIA-ENE-032 PHASE 4 — Integration Specification

**Standard:** WIA-ENE-032
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 14. 경보 시스템

### 14.1 경보 등급

| 등급 | 수신 대상 | 발송 채널 | 응답 시간 |
|------|-----------|-----------|-----------|
| Level 1 - 관심 | 관련 기관 | 이메일 | 1시간 |
| Level 2 - 주의 | 지역 소방서 | SMS + 이메일 | 30분 |
| Level 3 - 경계 | 전체 소방 | 긴급 문자 + 전화 | 10분 |
| Level 4 - 심각 | 국가 재난망 | 재난 문자 (CBS) | 즉시 |

### 14.2 다국어 지원

- 한국어, 영어, 중국어, 일본어
- 자동 번역 API 연동
- 음성 안내 (TTS)

---

## 15. 통합 및 상호운용성

### 15.1 국제 표준 준수

- **WMO (World Meteorological Organization)**: 기상 데이터
- **OGC (Open Geospatial Consortium)**: 지리 데이터
- **EDXL (Emergency Data Exchange Language)**: 재난 정보 교환
- **CAP (Common Alerting Protocol)**: 경보 프로토콜

### 15.2 API 엔드포인트

- `POST /api/v1/fire/detect` - 화재 감지 이벤트 등록
- `GET /api/v1/fire/{id}` - 화재 정보 조회
- `GET /api/v1/fire/active` - 진행 중 화재 목록
- `POST /api/v1/prediction/spread` - 확산 예측 요청
- `GET /api/v1/evacuation/zones` - 대피 구역 조회
- `POST /api/v1/resources/dispatch` - 자원 출동 지시
- `POST /api/v1/alert/send` - 경보 발송

---

## 16. 보안 및 개인정보보호

### 16.1 보안 요구사항

- **전송 암호화**: TLS 1.3
- **인증**: OAuth 2.0 + JWT
- **접근 제어**: RBAC (Role-Based Access Control)
- **감사 로그**: 모든 API 호출 기록 (90일 보관)

### 16.2 개인정보 보호

- 위치 정보: 일반 대중에게는 행정구역 수준까지만 공개
- 대피자 명단: 암호화 저장, 권한자만 접근
- GDPR/개인정보보호법 준수

---

## 17. 인증 요구사항

### 17.1 시스템 인증

**Tier 1 (기본):**
- 위성 데이터 수신 및 처리
- 기본 경보 발송
- 데이터 정확도 90% 이상

**Tier 2 (고급):**
- 다중 센서 융합
- AI 확산 예측
- 실시간 자원 추적
- 데이터 정확도 95% 이상

**Tier 3 (전문):**
- 전국 규모 통합 운영
- 국제 협력 체계
- 연구 개발 기능
- 데이터 정확도 98% 이상

### 17.2 연간 인증 심사

- 시스템 가동률 99.5% 이상
- 평균 탐지 시간 10분 이내
- 오탐지율 5% 미만
- 미탐지율 1% 미만

---

## 부록 A: 화재 행동 연료 모델 (FBFM)

### 13가지 표준 연료 모델 (Anderson, 1982)

| 모델 | 명칭 | 특성 | 확산 속도 |
|------|------|------|-----------|
| 1 | 짧은 초본 | 건조 초지 | 빠름 |
| 2 | 낮은 관목 + 초본 | 사바나 | 중간 |
| 3 | 높은 초본 | 습지 건조 시 | 매우 빠름 |
| 4 | 높은 관목 (2m) | 차파랄 | 빠름 |
| 5 | 짧은 관목 (0.6m) | 어린 산림 | 중간 |
| 6 | 휴면 관목 | 침엽수림 하층 | 중간 |
| 7 | 남부 거친 관목 | 2.5m 관목 | 느림 |
| 8 | 밀집 침엽수 | 낙엽층 깊음 | 느림-중간 |
| 9 | 활엽수 낙엽 | 낙엽층 | 느림 |
| 10 | 목재 낙하 | 베기 후 잔재 (가벼움) | 중간 |
| 11 | 목재 낙하 | 베기 후 잔재 (중간) | 중간 |
| 12 | 목재 낙하 | 베기 후 잔재 (무거움) | 빠름 |
| 13 | 목재 낙하 | 베기 후 잔재 (매우 무거움) | 매우 빠름 |

---

## 부록 B: 위성 데이터 접근 정보

### MODIS

- **NASA FIRMS**: https://firms.modaps.eosdis.nasa.gov/
- **API**: https://firms.modaps.eosdis.nasa.gov/api/
- **갱신 주기**: 3시간
- **무료 제공**: 예 (등록 필요)

### VIIRS

- **NOAA**: https://www.star.nesdis.noaa.gov/jpss/VIIRS.php
- **API**: https://firms.modaps.eosdis.nasa.gov/api/
- **갱신 주기**: 3시간
- **무료 제공**: 예 (등록 필요)

### Sentinel

- **Copernicus Hub**: https://scihub.copernicus.eu/
- **API**: https://scihub.copernicus.eu/twiki/do/view/SciHubWebPortal/APIHubDescription
- **갱신 주기**: 5일 (Sentinel-2), 1일 (Sentinel-3)
- **무료 제공**: 예 (등록 필요)

---

## 부록 C: 참고 문헌

1. Anderson, H. E. (1982). "Aids to determining fuel models for estimating fire behavior." USDA Forest Service.
2. Rothermel, R. C. (1972). "A mathematical model for predicting fire spread in wildland fuels." USDA Forest Service Research Paper INT-115.
3. Finney, M. A. (1998). "FARSITE: Fire Area Simulator - Model development and evaluation." USDA Forest Service Research Paper RMRS-RP-4.
4. NASA FIRMS. (2024). "Fire Information for Resource Management System User Guide."
5. Van Wagner, C. E. (1987). "Development and structure of the Canadian Forest Fire Weather Index System." Canadian Forestry Service Forestry Technical Report 35.

---

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 기술 사양
- 위성 감지 프로토콜
- AI 예측 모델 정의
- 국제 표준 준수

---

## 弘益人間 (홍익인간) · 널리 인간을 이롭게 하라

WIA-ENE-032 산불 감지 표준은 弘益人間(홍익인간)의 정신을 구현합니다. 첨단 기술과 국제 협력을 통해 산림 화재로부터 인명과 재산을 보호하고, 소중한 자연 생태계를 지켜냅니다.

개방형 표준, 투명한 데이터, 협력적 대응을 통해 산불 감지 및 대응 기술이 인류 전체와 지구 환경의 공동선에 기여하도록 보장합니다.

**함께, 우리는 산불로부터 안전한 세상을 만들어갑니다.**

---

© 2025 SmileStory Inc. / WIA
**弘益人間 (홍익인간) · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
