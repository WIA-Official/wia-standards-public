# WIA-HOME PHASE 1 — Data Format Specification

**Standard:** WIA-HOME
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-HOME v1.0

## 가족의 집 - "건물주 없는 내 집"

```
가족이 아무리 커져도
살 집 하나 없으면 뭐하나...

평생 건물주 눈치 보면서
월세 내면서 사는 게
디지털 세계도 마찬가지였어.

이제 그만.
내 PC가 서버야.
내 노트북이 집이야.
건물주 없이, 영원히 내 것.

WIA 대가족의 든든한 울타리,
비바람이 와도 흔들리지 않는 집.

- WIA-HOME -
```

**WIA-HOME**: World Interoperable Accessible Homestead

---

## 1. 개요

### 1.1 현실의 문제

```
현재 홈페이지를 만들려면:

1. 도메인 구매 ($10-50/년)
2. 호스팅 서버 ($5-100/월)
3. SSL 인증서 (무료~$200/년)
4. 개발자 고용 또는 학습
5. 유지보수 비용

= 최소 수십만원/년

소상공인?
그냥 네이버 스마트스토어, 인스타그램에 종속.
플랫폼이 정책 바꾸면? 끝.
플랫폼이 망하면? 모든 것 사라짐.

= 디지털 세상의 "전세/월세" 신세
```

### 1.2 WIA-HOME의 해결

```
WIA-HOME이 있으면:

1. 도메인? → yourname.wia.home (무료)
2. 호스팅? → 내 PC/노트북이 서버 (0원)
3. SSL? → WIA-TLS-LITE 내장 (0원)
4. 개발? → URL 넣으면 자동 생성
5. 유지보수? → 자동 업데이트

= 0원

내 집이니까.
건물주가 없으니까.
영원히 내 것이니까.
```

### 1.3 철학

```
홍익인간 (弘益人間) - Benefit All Humanity

집은 인권이다.
디지털 세상에서도 마찬가지다.

누구나 자신을 표현할 공간이 있어야 한다.
그 공간이 플랫폼에 종속되어선 안 된다.
그 공간은 영원히 자신의 것이어야 한다.

WIA-HOME은 모든 인류에게
디지털 세상의 "내 집"을 선물한다.
```

### 1.4 가족 관계

```
WIA Family의 집:

┌─────────────────────────────────────────────────────┐
│                    🏠 WIA-HOME                      │
│              가족 모두가 사는 집                      │
├─────────────────────────────────────────────────────┤
│                                                     │
│  👨 아버지 (INTENT)    - 집의 설계도를 그려         │
│  👩 어머니 (OMNI-API)  - 모든 손님을 맞이해         │
│  💪 삼촌 (AIR-POWER)   - 집에 전기를 공급해         │
│  🛡️ 이모 (AIR-SHIELD)  - 집을 안전하게 지켜         │
│  🌐 조카 (SOCIAL)      - 이웃과 연결해줘            │
│                                                     │
│  🏠 집 (HOME)          - 모두가 함께 사는 곳        │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

## 2. 핵심 개념

### 2.1 Personal Web Server (개인 웹 서버)

```
기존 방식:
┌──────────┐     ┌──────────┐     ┌──────────┐
│  사용자   │ ──→ │  AWS/    │ ──→ │ 웹사이트  │
│          │     │ 클라우드  │     │          │
└──────────┘     └──────────┘     └──────────┘
                    ↑
                 월세 지불
                 정책 종속

WIA-HOME 방식:
┌──────────┐     ┌──────────┐
│  방문자   │ ──→ │ 내 PC/   │ ← 내 집!
│          │     │ 노트북   │    건물주 없음
└──────────┘     └──────────┘
                    ↑
                 WIA-HOME
                 P2P Network
```

### 2.2 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                    WIA-HOME Stack                       │
├─────────────────────────────────────────────────────────┤
│  Layer 5: Content                                       │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐       │
│  │  Pages  │ │  Blog   │ │  Shop   │ │  Media  │       │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘       │
├─────────────────────────────────────────────────────────┤
│  Layer 4: Application                                   │
│  ┌─────────────────────────────────────────────────┐   │
│  │  WIA-INTENT Integration (의도 기반 페이지 생성)   │   │
│  └─────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────┤
│  Layer 3: API Gateway                                   │
│  ┌─────────────────────────────────────────────────┐   │
│  │  WIA-OMNI-API (모든 요청 처리)                    │   │
│  └─────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────┤
│  Layer 2: Security                                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │  WIA-AIR-SHIELD + WIA-TLS-LITE                   │   │
│  └─────────────────────────────────────────────────┘   │
├─────────────────────────────────────────────────────────┤
│  Layer 1: Network                                       │
│  ┌─────────────────────────────────────────────────┐   │
│  │  P2P Mesh + NAT Traversal + WIA DNS              │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

### 2.3 Site Generation Modes (사이트 생성 모드)

#### Mode 1: Clone & Customize (벤치마킹)
```typescript
// URL 넣으면 자동 분석 후 내 버전 생성
const mySite = await WIAHome.clone('https://example-shop.com', {
  customize: {
    name: '홍길동 가게',
    logo: myLogo,
    colors: { primary: '#FF6B6B' },
    content: myProducts
  }
});

// 분석 내용:
// - 레이아웃 구조
// - 컴포넌트 패턴
// - 기능 목록
// - 최적화 포인트
// → 내 콘텐츠로 재구성
```

#### Mode 2: Intent-Based (의도 기반)
```typescript
// 아버지(INTENT) 연동 - 말만 하면 사이트 생성
const mySite = await WIAHome.create(`
  나는 동네 빵집을 운영해.
  빵 종류랑 가격 보여주고
  예약 받을 수 있게 해줘.
  인스타그램이랑 연동해줘.
`);

// → 자동으로 빵집 홈페이지 생성
// → 메뉴판, 가격표, 예약 시스템, SNS 연동
```

#### Mode 3: Template (템플릿)
```typescript
// 검증된 템플릿에서 시작
const mySite = await WIAHome.fromTemplate('small-business', {
  industry: 'bakery',
  locale: 'ko-KR'
});
```

#### Mode 4: Import (가져오기)
```typescript
// 기존 사이트 마이그레이션
const mySite = await WIAHome.import({
  source: 'wordpress',
  url: 'https://my-old-site.com',
  credentials: wpCredentials
});
```

---

## 3. 기술 명세

### 3.1 Personal Server

```typescript
interface PersonalServer {
  // 서버 시작
  start(): Promise<void>;
  stop(): Promise<void>;

  // 상태
  status: ServerStatus;
  uptime: number;
  visitors: VisitorStats;

  // 설정
  config: ServerConfig;

  // 자원 사용
  resources: {
    cpu: Percentage;
    memory: Bytes;
    storage: Bytes;
    bandwidth: BandwidthStats;
  };
}

interface ServerConfig {
  // 포트 (기본: 자동)
  port: number | 'auto';

  // 최대 동시 연결
  maxConnections: number;

  // 대역폭 제한
  bandwidthLimit?: BytesPerSecond;

  // 자동 시작
  autoStart: boolean;

  // 절전 모드 (방문자 없을 때)
  sleepMode: boolean;
  sleepAfter: Minutes;

  // P2P 릴레이 허용
  allowRelay: boolean;
}
```

### 3.2 P2P Network

```typescript
interface P2PNetwork {
  // 네트워크 참여
  join(): Promise<PeerId>;
  leave(): Promise<void>;

  // 피어 관리
  peers: Peer[];
  connectedPeers: number;

  // NAT Traversal
  natType: NATType;
  publicAddress?: string;

  // 릴레이 (NAT 뒤에 있을 때)
  relayNodes: RelayNode[];

  // 콘텐츠 배포 (CDN 대체)
  distribute(content: Content): Promise<ContentHash>;
  fetch(hash: ContentHash): Promise<Content>;
}

type NATType =
  | 'open'           // 직접 연결 가능
  | 'full_cone'      // 포트 포워딩으로 가능
  | 'restricted'     // 릴레이 필요할 수 있음
  | 'symmetric'      // 릴레이 필요
  | 'unknown';
```

### 3.3 WIA DNS

```typescript
interface WIADNS {
  // 도메인 등록
  register(name: string): Promise<WIADomain>;

  // 형식: yourname.wia.home
  // 또는: yourname.wia.shop
  // 또는: yourname.wia.blog

  // 해석
  resolve(domain: WIADomain): Promise<PeerAddress>;

  // 기존 도메인 연결 (선택)


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
