# WIA-HOME PHASE 3 — Protocol Specification

**Standard:** WIA-HOME
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

price: 35000,
  description: '신선한 재료로...',
  images: [cakeImage],
  inventory: 10
});
```

### 4.7 Analytics API

```typescript
// 방문자 통계
const stats = await home.analytics.get({
  period: 'last_30_days'
});

console.log(`총 방문자: ${stats.visitors.total}`);
console.log(`페이지뷰: ${stats.pageViews.total}`);
console.log(`인기 페이지:`, stats.topPages);
console.log(`유입 경로:`, stats.referrers);

// 실시간 모니터링
home.analytics.realtime((data) => {
  console.log(`현재 접속자: ${data.activeVisitors}`);
});
```

---

## 5. 소상공인 특화 기능

### 5.1 업종별 템플릿

```typescript
const templates = {
  // 음식점
  restaurant: {
    features: ['menu', 'reservation', 'delivery', 'reviews'],
    integrations: ['baemin', 'yogiyo', 'coupangeats']
  },

  // 카페
  cafe: {
    features: ['menu', 'location', 'instagram-feed', 'stamp-card'],
    integrations: ['instagram', 'naver-place']
  },

  // 미용실
  hairSalon: {
    features: ['portfolio', 'booking', 'price-list', 'staff'],
    integrations: ['naver-booking', 'kakao-hairshop']
  },

  // 학원
  academy: {
    features: ['courses', 'schedule', 'enrollment', 'reviews'],
    integrations: ['naver-academy']
  },

  // 병원/의원
  clinic: {
    features: ['departments', 'doctors', 'booking', 'location'],
    integrations: ['naver-place', 'kakao-map']
  },

  // 소매점
  retailShop: {
    features: ['products', 'inventory', 'order', 'delivery'],
    integrations: ['smartstore', 'coupang']
  }
};
```

### 5.2 간편 결제 연동

```typescript
interface PaymentIntegration {
  // 국내 결제
  korean: {
    kakaoPay: boolean;
    naverPay: boolean;
    tossPay: boolean;
    payco: boolean;
  };

  // 글로벌
  global: {
    stripe: boolean;
    paypal: boolean;
  };

  // 계좌이체
  bankTransfer: {
    enabled: boolean;
    accounts: BankAccount[];
  };
}

// 설정
home.shop.setPayment({
  kakaoPay: true,
  naverPay: true,
  bankTransfer: {
    enabled: true,
    accounts: [{
      bank: '신한은행',
      number: '110-xxx-xxx',
      holder: '홍길동'
    }]
  }
});
```

### 5.3 예약 시스템

```typescript
interface BookingSystem {
  // 예약 가능 시간
  availability: {
    days: DayOfWeek[];
    hours: TimeRange;
    slotDuration: Minutes;
    maxAdvance: Days;
  };

  // 예약 관리
  bookings: {
    pending: Booking[];
    confirmed: Booking[];
    completed: Booking[];
    cancelled: Booking[];
  };

  // 알림
  notifications: {
    newBooking: NotificationChannel[];
    reminder: NotificationChannel[];
    cancellation: NotificationChannel[];
  };
}

// 미용실 예약 시스템 예시
home.booking.setup({
  availability: {
    days: ['mon', 'tue', 'wed', 'thu', 'fri', 'sat'],
    hours: { start: '10:00', end: '20:00' },
    slotDuration: 60,
    maxAdvance: 30
  },
  services: [
    { name: '커트', duration: 30, price: 15000 },
    { name: '펌', duration: 120, price: 80000 },
    { name: '염색', duration: 90, price: 60000 }
  ],
  notifications: {
    newBooking: ['kakao', 'sms'],
    reminder: ['kakao']
  }
});
```

### 5.4 네이버/카카오 연동

```typescript
interface PlatformIntegration {
  // 네이버
  naver: {
    place: boolean;      // 네이버 플레이스
    smartstore: boolean; // 스마트스토어
    blog: boolean;       // 블로그 연동
    map: boolean;        // 지도 연동
  };

  // 카카오
  kakao: {
    channel: boolean;    // 카카오 채널
    map: boolean;        // 카카오맵
    pay: boolean;        // 카카오페이
    sync: boolean;       // 카카오싱크
  };

  // 배달
  delivery: {
    baemin: boolean;
    yogiyo: boolean;
    coupangeats: boolean;
  };
}

// 연동 설정
await home.integrate({
  naver: {
    place: true,
    placeId: 'xxxxx'
  },
  kakao: {
    channel: true,
    channelId: '@myhome'
  }
});
```

---

## 6. 오프라인 & 저사양 지원

### 6.1 Offline First

```typescript
interface OfflineSupport {
  // 오프라인 모드
  offline: {
    enabled: boolean;
    cachedPages: string[];
    syncOnReconnect: boolean;
  };

  // 서비스 워커
  serviceWorker: {
    cacheStrategy: 'network-first' | 'cache-first' | 'stale-while-revalidate';
    maxCacheSize: Bytes;
  };

  // P2P 캐싱
  p2pCache: {
    enabled: boolean;
    shareWithPeers: boolean;
  };
}
```

### 6.2 저사양 PC 지원

```typescript
interface LowSpecMode {
  // 리소스 제한
  limits: {
    maxMemory: Megabytes;    // 기본: 256MB
    maxCpu: Percentage;       // 기본: 20%
    maxStorage: Gigabytes;    // 기본: 1GB
    maxBandwidth: Mbps;       // 기본: 10Mbps
  };

  // 절전 모드
  powerSaving: {
    enabled: boolean;
    sleepAfterInactive: Minutes;
    wakeOnRequest: boolean;
  };

  // 최적화
  optimization: {
    imageCompression: boolean;
    lazyLoading: boolean;
    minimalJs: boolean;
  };
}

// 구형 노트북에서도 실행 가능
const home = new WIAHome({
  lowSpecMode: {
    limits: {
      maxMemory: 128,  // 128MB
      maxCpu: 10,      // 10%
    },
    powerSaving: {
      enabled: true,
      sleepAfterInactive: 5
    }
  }
});
```

### 6.3 모바일 호스팅

```typescript
// 스마트폰에서도 홈페이지 호스팅!
import { WIAHomeMobile } from '@anthropic/wia-home/mobile';

const home = new WIAHomeMobile({
  batteryAware: true,      // 배터리 고려
  wifiOnly: true,          // WiFi에서만 서빙
  backgroundMode: true     // 백그라운드 실행
});

await home.start();
// 내 스마트폰이 서버!
```

---

## 7. 보안 (이모 연동)

### 7.1 Built-in Security

```typescript
interface SecurityFeatures {
  // TLS-LITE 내장
  tls: {
    enabled: true;
    autoRenew: true;
    protocol: 'WIA-TLS-LITE';
  };

  // DDoS 방어
  ddos: {
    rateLimit: RequestsPerSecond;
    blockList: IP[];
    p2pDistribution: boolean; // 트래픽 분산
  };

  // WAF (Web Application Firewall)
  waf: {
    sqlInjection: boolean;
    xss: boolean;
    csrf: boolean;
  };


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
