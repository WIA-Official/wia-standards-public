# WIA-COMM-014 PHASE 2 — API Interface Specification

**Standard:** WIA-COMM-014
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 5. Cache Invalidation

### 5.1 Purge Methods

**1. URL Purge:**
```bash
# Single URL
curl -X POST "https://api.cdn.com/purge" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{"urls": ["https://example.com/style.css"]}'

# Wildcard
curl -X POST "https://api.cdn.com/purge" \
  -d '{"urls": ["https://example.com/assets/*"]}'
```

**2. Tag-based Purge:**
```bash
# Purge by cache tag
curl -X POST "https://api.cdn.com/purge" \
  -d '{"tags": ["homepage", "product-123"]}'

# Set cache tags
Cache-Tag: homepage, product-123, category-shoes
```

**3. Host Purge:**
```bash
# Purge entire hostname
curl -X POST "https://api.cdn.com/purge" \
  -d '{"hosts": ["www.example.com"]}'
```

**4. Full Zone Purge:**
```bash
# Purge everything (use sparingly!)
curl -X POST "https://api.cdn.com/purge" \
  -d '{"purge_everything": true}'
```

### 5.2 Purge Propagation

**Timing:**
```
Global purge propagation: 2-30 seconds

Breakdown:
- API processing: <1 second
- Edge notification: 1-5 seconds
- Edge cache clear: 1-20 seconds
- Verification: 5-10 seconds
```

**Verification:**
```bash
# Check purge status
curl -X GET "https://api.cdn.com/purge/$PURGE_ID" \
  -H "Authorization: Bearer $TOKEN"

Response:
{
  "id": "purge-123456",
  "status": "completed",
  "progress": 100,
  "created_at": "2025-12-26T10:00:00Z",
  "completed_at": "2025-12-26T10:00:15Z"
}
```

### 5.3 Soft Purge vs Hard Purge

**Hard Purge:**
```
- Immediately delete content from cache
- Next request must fetch from origin
- High origin load if traffic is high
```

**Soft Purge:**
```
- Mark content as stale
- Serve stale while revalidating
- Gradual cache refresh
- Lower origin impact
```

---

## 6. HTTP/2 and HTTP/3

### 6.1 HTTP/2 Features

**Multiplexing:**
```
Single TCP connection:
- Multiple parallel requests/responses
- No head-of-line blocking at HTTP layer
- Efficient connection reuse
```

**Server Push:**
```html
<!-- HTML response -->
<link rel="stylesheet" href="/style.css">

<!-- CDN automatically pushes style.css -->
Link: </style.css>; rel=preload; as=style
```

**Header Compression (HPACK):**
```
Before: 500 bytes of headers
After:  50 bytes (90% reduction)

Result: Faster page loads, reduced bandwidth
```

### 6.2 HTTP/3 and QUIC

**Key Improvements:**

**1. 0-RTT Connection Resumption:**
```
HTTP/2 (TLS 1.2): 2-3 RTT for connection
HTTP/3 (QUIC):    0 RTT for resumption

Time saved: 100-300ms on repeat visits
```

**2. No Head-of-Line Blocking:**
```
HTTP/2: Packet loss blocks entire connection
HTTP/3: Packet loss only affects one stream

Result: 30-50% faster on lossy networks
```

**3. Connection Migration:**
```
Scenario: User switches WiFi to 4G
HTTP/2: Connection drops, must reconnect
HTTP/3: Seamless migration, no interruption
```

### 6.3 Protocol Negotiation

**ALPN (Application-Layer Protocol Negotiation):**
```
Client Hello:
  ALPN: h2, http/1.1

Server Response:
  Selected: h2 (HTTP/2)

OR

Alt-Svc Header:
  Alt-Svc: h3=":443"; ma=86400
  (Advertise HTTP/3 availability)
```

---

## 7. TLS/SSL Termination

### 7.1 Edge TLS Termination

```
User ──[TLS]──► Edge Node ──[HTTP or TLS]──► Origin

Benefits:
- Offload crypto from origin
- Faster TLS handshake (edge is closer)
- Centralized certificate management
- Modern cipher support at edge
```

### 7.2 Certificate Management

**Automated Certificate Provisioning:**
```
1. User adds domain to CDN
2. CDN validates domain ownership (DNS TXT or HTTP file)
3. CDN issues Let's Encrypt certificate (free)
4. Auto-renewal every 60 days
```

**Custom Certificates:**
```bash
# Upload custom cert
curl -X POST "https://api.cdn.com/certificates" \
  -H "Authorization: Bearer $TOKEN" \
  -F "certificate=@cert.pem" \
  -F "private_key=@key.pem" \
  -F "bundle=@ca-bundle.pem"
```

### 7.3 TLS Versions and Ciphers

**Recommended Configuration:**
```
TLS Versions: TLS 1.2, TLS 1.3
Cipher Suites (TLS 1.3):
  - TLS_AES_128_GCM_SHA256
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256

Cipher Suites (TLS 1.2):
  - ECDHE-RSA-AES128-GCM-SHA256
  - ECDHE-RSA-AES256-GCM-SHA384

Disabled: SSLv3, TLS 1.0, TLS 1.1 (insecure)
```

### 7.4 HSTS and Security Headers

```http
Strict-Transport-Security: max-age=31536000; includeSubDomains; preload
X-Content-Type-Options: nosniff
X-Frame-Options: SAMEORIGIN
X-XSS-Protection: 1; mode=block
Content-Security-Policy: default-src 'self'; script-src 'self' 'unsafe-inline'
Referrer-Policy: strict-origin-when-cross-origin
Permissions-Policy: geolocation=(), microphone=(), camera=()
```

---

## 8. DDoS Protection

### 8.1 Attack Types and Mitigation

**1. Volumetric Attacks:**
```
Attack: UDP flood, ICMP flood, DNS amplification
Volume: 10-500 Gbps

Mitigation:
- Anycast distribution (spread traffic across PoPs)
- Scrubbing centers (filter malicious traffic)
- Rate limiting (per-IP throttling)
```

**2. Protocol Attacks:**
```
Attack: SYN flood, fragmented packets
Target: Exhaust connection table

Mitigation:
- SYN cookies
- Connection limits
- Protocol validation
```

**3. Application Layer Attacks:**
```
Attack: HTTP flood, slowloris, API abuse
Target: Exhaust server resources

Mitigation:
- Challenge-response (CAPTCHA, JavaScript)
- Rate limiting (requests/minute per IP)
- Bot detection (behavioral analysis)
```

### 8.2 Rate Limiting

**Configuration:**
```json
{
  "rateLimits": [
    {
      "rule": "global",
      "threshold": 10000,
      "period": 60,
      "action": "challenge"
    },
    {
      "rule": "per-ip",
      "threshold": 100,
      "period": 10,
      "action": "block"
    },
    {
      "rule": "api-endpoint",
      "path": "/api/*",
      "threshold": 1000,
      "period": 60,
      "action": "rate_limit"
    }
  ]
}
```

**Response:**
```http
HTTP/1.1 429 Too Many Requests
Retry-After: 60
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 0


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
