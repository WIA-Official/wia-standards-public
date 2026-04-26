# WIA-HOME PHASE 4 — Integration Specification

**Standard:** WIA-HOME
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

// AIR-SHIELD 연동
  airShield: {
    enabled: boolean;
    scanUploads: boolean;
    protectForms: boolean;
  };
}
```

### 7.2 Privacy Protection

```typescript
interface PrivacyFeatures {
  // 방문자 추적 최소화
  tracking: {
    analytics: 'privacy-first' | 'full' | 'none';
    cookies: 'essential' | 'functional' | 'all';
    doNotTrack: boolean;
  };

  // 데이터 보호
  data: {
    encryption: boolean;
    localOnly: boolean; // 데이터 외부 전송 안 함
    autoDelete: Days;   // 로그 자동 삭제
  };

  // GDPR/개인정보보호법 준수
  compliance: {
    gdpr: boolean;
    ccpa: boolean;
    pipa: boolean; // 한국 개인정보보호법
  };
}
```

---

## 8. 사용 시나리오

### 8.1 동네 빵집

```typescript
// 1. 경쟁 업체 사이트 분석
const analysis = await WIAHome.analyze('https://famous-bakery.com');

// 2. 내 버전으로 클론
const myBakery = await WIAHome.clone('https://famous-bakery.com', {
  customize: {
    name: '행복한 빵집',
    logo: bakeryLogo,
    colors: { primary: '#E8B87D' }
  }
});

// 3. 내 상품 추가
await myBakery.shop.addProducts([
  { name: '소금빵', price: 3000 },
  { name: '크로아상', price: 4000 },
  { name: '식빵', price: 5000 }
]);

// 4. 결제 설정
await myBakery.shop.setPayment({
  kakaoPay: true,
  bankTransfer: { enabled: true }
});

// 5. 서버 시작
await myBakery.start();
// https://happy-bakery.wia.shop 오픈!
```

### 8.2 프리랜서 포트폴리오

```typescript
// 의도 기반 생성
const portfolio = await WIAHome.fromIntent(`
  나는 UI/UX 디자이너야.
  포트폴리오 10개 정도 보여주고 싶어.
  간단한 자기소개랑
  연락처, 견적 문의 폼 넣어줘.
  다크모드 지원하고
  애니메이션 좀 넣어줘.
`);

// 포트폴리오 추가
await portfolio.addWorks([
  { title: '앱 디자인', images: [...], description: '...' },
  { title: '웹 디자인', images: [...], description: '...' }
]);

await portfolio.start();
// https://designer-kim.wia.page 오픈!
```

### 8.3 기존 사이트 이전

```typescript
// WordPress에서 탈출
const mySite = await WIAHome.migrate({
  from: 'wordpress',
  url: 'https://my-expensive-site.com'
});

// 기존 도메인 연결 (선택)
await mySite.connectDomain('my-expensive-site.com');

// 또는 무료 WIA 도메인 사용
// https://mysite.wia.home

await mySite.start();
// 이제 호스팅비 0원!
```

---

## 9. 인증 레벨

### 9.1 WIA-HOME Starter

```
┌─────────────────────────────────────────┐
│                                         │
│   🏠  WIA-HOME STARTER  🏠              │
│                                         │
│   무료 기본 홈                           │
│                                         │
│   ✓ 기본 템플릿                         │
│   ✓ 5 페이지까지                        │
│   ✓ yourname.wia.home 도메인            │
│   ✓ 기본 보안                           │
│                                         │
└─────────────────────────────────────────┘
```

### 9.2 WIA-HOME Business

```
┌─────────────────────────────────────────┐
│                                         │
│   🏢  WIA-HOME BUSINESS  🏢             │
│                                         │
│   소상공인용                             │
│                                         │
│   ✓ 모든 Starter 기능                   │
│   ✓ 무제한 페이지                       │
│   ✓ 쇼핑몰 기능                         │
│   ✓ 예약 시스템                         │
│   ✓ 결제 연동                           │
│   ✓ 네이버/카카오 연동                  │
│                                         │
└─────────────────────────────────────────┘
```

### 9.3 WIA-HOME Enterprise

```
┌─────────────────────────────────────────┐
│                                         │
│   🏛️  WIA-HOME ENTERPRISE  🏛️          │
│                                         │
│   기업용                                 │
│                                         │
│   ✓ 모든 Business 기능                  │
│   ✓ 커스텀 도메인                       │
│   ✓ 다중 사이트                         │
│   ✓ 팀 협업                            │
│   ✓ 고급 분석                           │
│   ✓ 24/7 지원                          │
│                                         │
└─────────────────────────────────────────┘
```

---

## 10. 로드맵

### Phase 1: Foundation (기초)
- 개인 웹 서버 엔진
- P2P 네트워크 기본
- WIA DNS
- 기본 템플릿

### Phase 2: Clone Engine (클론 엔진)
- URL 분석기
- 자동 클론 생성
- 커스터마이징 도구
- 마이그레이션 도구

### Phase 3: Business (비즈니스)
- 쇼핑몰 기능
- 예약 시스템
- 결제 연동
- 플랫폼 연동 (네이버/카카오)

### Phase 4: Intelligence (지능화)
- INTENT 연동 (의도 기반 생성)
- AI 디자인 추천
- 자동 SEO
- 콘텐츠 생성 지원

---

## 부록 A: 집의 약속

```
WIA-HOME의 약속:

1. 건물주가 없어요
   - 내 PC가 서버, 내가 건물주

2. 영원히 내 것이에요
   - 플랫폼 망해도, 회사 망해도 내 집은 건재

3. 돈 안 들어요
   - 호스팅비 0원, 도메인 0원

4. 쉬워요
   - URL 넣으면 5분 만에 완성
   - 말로 해도 돼요

5. 안전해요
   - 이모(AIR-SHIELD)가 지켜줘요
   - 보안은 기본

WIA 대가족의 든든한 울타리,
비바람이 와도 흔들리지 않는 집.

- WIA-HOME -
```

---

## 부록 B: 가족 통합 예시

```typescript
import { Intent } from '@anthropic/wia-intent';        // 아버지
import { OmniApi } from '@anthropic/wia-omni-api';     // 어머니
import { AirPower } from '@anthropic/wia-air-power';   // 삼촌
import { AirShield } from '@anthropic/wia-air-shield'; // 이모
import { WIASocial } from '@anthropic/wia-social';     // 조카
import { WIAHome } from '@anthropic/wia-home';         // 집

// 가족의 집
const home = new WIAHome({
  family: {
    intent: new Intent(),      // 집 설계
    omniApi: new OmniApi(),    // 손님 맞이
    airPower: new AirPower(),  // 전기 공급
    airShield: new AirShield(), // 보안
    social: new WIASocial()    // 이웃 연결
  }
});

// 아버지가 집 설계
home.family.intent.express(`
  빵집 홈페이지 만들어줘
  메뉴판이랑 예약 기능 넣어줘
`);

// 어머니가 모든 API 요청 처리
home.family.omniApi.handle(allRequests);

// 삼촌이 전기 공급 (배터리 걱정 없이)
home.family.airPower.powerDevice(myLaptop);

// 이모가 집 지킴
home.family.airShield.protect(home);

// 조카가 SNS 연결
home.family.social.crossPost(newProduct);

// 모든 가족이 함께 사는 집
await home.start();
```

---

## 부록 C: 홍익인간 선언

```
弘益人間 (홍익인간)

집은 인권이다.

현실 세계에서 집이 필요하듯,
디지털 세계에서도 집이 필요하다.

플랫폼에 종속되지 않을 권리,
내 공간을 소유할 권리,
영원히 내 것으로 가질 권리.

WIA-HOME은 모든 인류에게
디지털 세상의 "내 집"을 선물한다.

소상공인도,
프리랜서도,
학생도,
누구나.

건물주 없는 세상,
월세 없는 세상,
영원히 내 것인 세상.

그것이 WIA가 꿈꾸는 세상이다.

- WIA (World Certification Industry Association) -
```

---

**Document Version**: 1.0.0
**Last Updated**: 2025
**Status**: Initial Release
**Author**: WIA Technical Committee
**Philosophy**: 홍익인간 (弘益人間) - Benefit All Humanity


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
