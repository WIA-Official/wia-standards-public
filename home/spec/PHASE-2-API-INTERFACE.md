# WIA-HOME PHASE 2 — API Interface Specification

**Standard:** WIA-HOME
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

linkCustomDomain(domain: string): Promise<void>;
}

type WIADomain = `${string}.wia.${'home' | 'shop' | 'blog' | 'page'}`;

interface DomainRecord {
  name: string;
  peerId: PeerId;
  publicKey: PublicKey;
  created: Timestamp;
  updated: Timestamp;
  verified: boolean;
}
```

### 3.4 Site Cloner (벤치마킹 엔진)

```typescript
interface SiteCloner {
  // URL 분석
  analyze(url: string): Promise<SiteAnalysis>;

  // 클론 생성
  clone(url: string, options: CloneOptions): Promise<Site>;

  // 분석 결과
  interface SiteAnalysis {
    // 구조
    structure: {
      pages: PageInfo[];
      navigation: NavigationStructure;
      layout: LayoutType;
    };

    // 기능
    features: {
      hasShop: boolean;
      hasBlog: boolean;
      hasContact: boolean;
      hasBooking: boolean;
      hasSocialLinks: boolean;
      hasNewsletter: boolean;
    };

    // 기술 스택
    tech: {
      framework?: string;
      cms?: string;
      ecommerce?: string;
    };

    // 성능
    performance: {
      loadTime: Milliseconds;
      size: Bytes;
      score: number;
    };

    // 추천
    recommendations: Recommendation[];
  }
}

interface CloneOptions {
  // 커스터마이징
  customize: {
    name: string;
    logo?: MediaRef;
    colors?: ColorScheme;
    fonts?: FontScheme;
    content?: ContentMap;
  };

  // 기능 선택
  features?: {
    shop?: boolean;
    blog?: boolean;
    contact?: boolean;
    booking?: boolean;
  };

  // 최적화
  optimize?: {
    images: boolean;
    minify: boolean;
    lazyLoad: boolean;
  };
}
```

### 3.5 Content Management

```typescript
interface ContentManager {
  // 페이지 관리
  pages: {
    create(page: PageData): Promise<Page>;
    update(id: PageId, data: Partial<PageData>): Promise<Page>;
    delete(id: PageId): Promise<void>;
    list(): Promise<Page[]>;
    get(id: PageId): Promise<Page>;
  };

  // 미디어 관리
  media: {
    upload(file: File): Promise<MediaRef>;
    delete(id: MediaId): Promise<void>;
    list(): Promise<MediaRef[]>;
    optimize(id: MediaId): Promise<MediaRef>;
  };

  // 블로그
  blog: {
    createPost(post: BlogPost): Promise<Post>;
    updatePost(id: PostId, data: Partial<BlogPost>): Promise<Post>;
    deletePost(id: PostId): Promise<void>;
    listPosts(options?: ListOptions): Promise<Post[]>;
  };

  // 상점 (선택)
  shop?: {
    addProduct(product: Product): Promise<Product>;
    updateProduct(id: ProductId, data: Partial<Product>): Promise<Product>;
    removeProduct(id: ProductId): Promise<void>;
    listProducts(): Promise<Product[]>;
    processOrder(order: Order): Promise<OrderResult>;
  };
}
```

### 3.6 Theme Engine

```typescript
interface ThemeEngine {
  // 테마 적용
  apply(theme: Theme): Promise<void>;

  // AI 기반 테마 생성
  generate(description: string): Promise<Theme>;

  // 색상 팔레트
  palette: {
    fromImage(image: MediaRef): Promise<ColorPalette>;
    fromBrand(brandName: string): Promise<ColorPalette>;
    harmonize(baseColor: Color): Promise<ColorPalette>;
  };

  // 레이아웃
  layout: {
    templates: LayoutTemplate[];
    apply(template: LayoutTemplate): Promise<void>;
  };
}

interface Theme {
  name: string;
  colors: ColorScheme;
  fonts: FontScheme;
  spacing: SpacingScale;
  borderRadius: RadiusScale;
  shadows: ShadowScale;
  components: ComponentStyles;
}

interface ColorScheme {
  primary: Color;
  secondary: Color;
  accent: Color;
  background: Color;
  surface: Color;
  text: Color;
  textSecondary: Color;
  error: Color;
  success: Color;
  warning: Color;
}
```

---

## 4. API 명세

### 4.1 Core API

```typescript
import { WIAHome } from '@anthropic/wia-home';

// 새 홈 생성
const home = new WIAHome({
  name: '홍길동의 집',
  domain: 'gildong.wia.home'
});

// 서버 시작
await home.start();

// 상태 확인
console.log(`🏠 ${home.domain} 실행 중`);
console.log(`📊 방문자: ${home.stats.visitors.today}명`);
```

### 4.2 Quick Start API

```typescript
// 1. 벤치마킹으로 5분만에 사이트 만들기
const mySite = await WIAHome.quickStart({
  benchmark: 'https://nice-bakery-site.com',
  myInfo: {
    name: '행복한 빵집',
    phone: '010-1234-5678',
    address: '서울시 강남구...',
    hours: '09:00-21:00'
  }
});

await mySite.start();
// 끝! https://happy-bakery.wia.shop 오픈
```

### 4.3 Clone API

```typescript
// URL 분석
const analysis = await WIAHome.analyze('https://competitor.com');
console.log('기능:', analysis.features);
console.log('구조:', analysis.structure);
console.log('추천:', analysis.recommendations);

// 클론 + 커스터마이징
const mySite = await WIAHome.clone('https://competitor.com', {
  customize: {
    name: '내 가게',
    logo: await loadFile('logo.png'),
    colors: {
      primary: '#3498db',
      secondary: '#2ecc71'
    }
  },
  features: {
    shop: true,
    blog: true,
    contact: true
  }
});
```

### 4.4 Intent API (아버지 연동)

```typescript
// 말로 사이트 만들기
const mySite = await WIAHome.fromIntent(`
  나는 프리랜서 디자이너야.
  포트폴리오 보여주고 싶어.
  연락처랑 견적 문의 받을 수 있게 해줘.
  깔끔하고 모던한 느낌으로.
`);

// 또는
const mySite = await WIAHome.fromIntent(`
  동네 꽃집이야.
  꽃다발 종류 보여주고
  배달 주문 받고 싶어.
  인스타그램 피드도 보여줘.
`);
```

### 4.5 Migration API

```typescript
// WordPress에서 이전
const mySite = await WIAHome.migrate({
  from: 'wordpress',
  url: 'https://my-wp-site.com',
  credentials: {
    username: 'admin',
    password: '****'
  },
  options: {
    posts: true,
    pages: true,
    media: true,
    comments: true
  }
});

// Wix에서 이전
const mySite = await WIAHome.migrate({
  from: 'wix',
  exportFile: './wix-export.zip'
});

// 네이버 블로그에서 이전
const mySite = await WIAHome.migrate({
  from: 'naver-blog',
  blogId: 'my-blog-id'
});
```

### 4.6 Management API

```typescript
// 페이지 관리
await home.pages.create({
  title: '회사 소개',
  slug: 'about',
  content: '우리 회사는...'
});

// 블로그 포스트
await home.blog.createPost({
  title: '새로운 시작',
  content: '오늘부터...',
  tags: ['일상', '창업']
});

// 상품 추가 (쇼핑몰)
await home.shop.addProduct({
  name: '수제 초코 케이크',


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
