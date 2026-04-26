# WIA-EDU-019 Digital Content Standard v1.2

## Phase 3: Distribution Protocol & Content Delivery

**Status:** ✅ Complete
**Version:** 1.2.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 defines the protocols and mechanisms for distributing WIA-compliant digital content efficiently, securely, and globally. This specification ensures optimal content delivery, offline support, and synchronization across devices.

## 2. Scope

Phase 2 covers:
- Content Delivery Network (CDN) integration
- Adaptive streaming protocols
- Progressive download and caching
- Offline-first architecture
- Synchronization mechanisms
- Security and encryption

## 3. Content Delivery Networks

### 3.1 CDN Requirements

All content SHOULD be distributed via CDN with:
- Global edge server distribution
- HTTPS/TLS 1.3 encryption
- HTTP/2 or HTTP/3 support
- Intelligent caching strategies
- DDoS protection
- Geographic load balancing

### 3.2 CDN Configuration

```json
{
  "cdn": {
    "provider": "cloudflare",
    "distribution": "global",
    "origins": [
      {
        "url": "https://origin.example.com",
        "protocol": "https",
        "port": 443
      }
    ],
    "caching": {
      "defaultTTL": 86400,
      "maxTTL": 31536000,
      "minTTL": 0
    },
    "compression": {
      "gzip": true,
      "brotli": true
    },
    "edgeLocations": [
      "us-east", "us-west", "eu-west",
      "asia-northeast", "asia-southeast"
    ]
  }
}
```

## 4. Adaptive Streaming

### 4.1 HLS (HTTP Live Streaming)

**Master Playlist (playlist.m3u8):**
```m3u8
#EXTM3U
#EXT-X-VERSION:6

#EXT-X-STREAM-INF:BANDWIDTH=800000,RESOLUTION=640x360
360p.m3u8

#EXT-X-STREAM-INF:BANDWIDTH=1400000,RESOLUTION=842x480
480p.m3u8

#EXT-X-STREAM-INF:BANDWIDTH=2800000,RESOLUTION=1280x720
720p.m3u8

#EXT-X-STREAM-INF:BANDWIDTH=5000000,RESOLUTION=1920x1080
1080p.m3u8

#EXT-X-STREAM-INF:BANDWIDTH=8000000,RESOLUTION=3840x2160
4k.m3u8
```

**Media Playlist (1080p.m3u8):**
```m3u8
#EXTM3U
#EXT-X-VERSION:6
#EXT-X-TARGETDURATION:10
#EXT-X-MEDIA-SEQUENCE:0

#EXTINF:10.0,
segment-00.ts
#EXTINF:10.0,
segment-01.ts
#EXTINF:10.0,
segment-02.ts
#EXT-X-ENDLIST
```

### 4.2 DASH (MPEG-DASH)

**MPD Manifest:**
```xml
<?xml version="1.0"?>
<MPD xmlns="urn:mpeg:dash:schema:mpd:2011">
  <Period duration="PT900S">
    <AdaptationSet mimeType="video/mp4">
      <Representation bandwidth="5000000" width="1920" height="1080">
        <BaseURL>1080p/</BaseURL>
        <SegmentTemplate media="segment-$Number$.m4s" startNumber="1" duration="10"/>
      </Representation>
      <Representation bandwidth="2800000" width="1280" height="720">
        <BaseURL>720p/</BaseURL>
        <SegmentTemplate media="segment-$Number$.m4s" startNumber="1" duration="10"/>
      </Representation>
    </AdaptationSet>
  </Period>
</MPD>
```

## 5. Progressive Download

### 5.1 Byte-Range Requests

Support HTTP range requests for partial content delivery:

```http
GET /video.mp4 HTTP/1.1
Host: cdn.example.com
Range: bytes=0-1048575
```

**Response:**
```http
HTTP/1.1 206 Partial Content
Content-Range: bytes 0-1048575/245760000
Content-Length: 1048576
Content-Type: video/mp4

[Binary data]
```

### 5.2 Content Chunking

Large files SHOULD be split into manageable chunks:
- Chunk size: 1-5 MB
- Support resumable downloads
- Verify integrity with checksums

## 6. Caching Strategies

### 6.1 Cache-Control Headers

```http
# Static assets (long-lived)
Cache-Control: public, max-age=31536000, immutable

# Dynamic content (short-lived)
Cache-Control: public, max-age=3600, must-revalidate

# Private content
Cache-Control: private, max-age=0, no-cache
```

### 6.2 Service Workers for Client-Side Caching

```javascript
// service-worker.js
const CACHE_NAME = 'wia-content-v1';
const urlsToCache = [
  '/',
  '/styles/main.css',
  '/scripts/app.js',
  '/data/manifest.json'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(urlsToCache))
  );
});

self.addEventListener('fetch', (event) => {
  event.respondWith(
    caches.match(event.request)
      .then((response) => {
        // Cache hit - return response
        if (response) {
          return response;
        }

        // Clone request for fetch
        const fetchRequest = event.request.clone();

        return fetch(fetchRequest).then((response) => {
          // Check if valid response
          if (!response || response.status !== 200 || response.type !== 'basic') {
            return response;
          }

          // Clone response for cache
          const responseToCache = response.clone();

          caches.open(CACHE_NAME)
            .then((cache) => {
              cache.put(event.request, responseToCache);
            });

          return response;
        });
      })
  );
});
```

## 7. Offline Support

### 7.1 Offline-First Architecture

```javascript
// Offline detection
window.addEventListener('online', () => {
  console.log('Online - syncing content');
  syncContent();
});

window.addEventListener('offline', () => {
  console.log('Offline - using cached content');
  loadCachedContent();
});

// Check connection status
if (navigator.onLine) {
  fetchLatestContent();
} else {
  loadCachedContent();
}
```

### 7.2 IndexedDB for Offline Storage

```javascript
// Open database
const request = indexedDB.open('WIAContent', 1);

request.onupgradeneeded = (event) => {
  const db = event.target.result;
  const objectStore = db.createObjectStore('content', { keyPath: 'id' });
  objectStore.createIndex('type', 'type', { unique: false });
  objectStore.createIndex('language', 'language', { unique: false });
};

// Store content
function storeContent(content) {
  const transaction = db.transaction(['content'], 'readwrite');
  const objectStore = transaction.objectStore('content');
  objectStore.add(content);
}

// Retrieve content
function getContent(id) {
  return new Promise((resolve, reject) => {
    const transaction = db.transaction(['content']);
    const objectStore = transaction.objectStore('content');
    const request = objectStore.get(id);

    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
}
```

## 8. Synchronization

### 8.1 Content Sync Protocol

```http
POST /sync
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "deviceId": "device-123",
  "lastSyncTimestamp": "2025-01-20T10:00:00Z",
  "localContent": [
    {
      "id": "content-123",
      "version": "1.0.0",
      "checksum": "abc123..."
    }
  ]
}
```

**Response:**
```json
{
  "syncId": "sync-456",
  "timestamp": "2025-01-20T16:00:00Z",
  "updates": [
    {
      "id": "content-123",
      "version": "1.1.0",
      "action": "update",
      "url": "https://cdn.example.com/content-123-v1.1.0.zip",
      "checksum": "def456..."
    },
    {
      "id": "content-789",
      "action": "delete"
    }
  ]
}
```

### 8.2 Delta Sync for Efficiency

```javascript
// Only sync changed portions
{
  "deltaSync": {
    "baseVersion": "1.0.0",
    "targetVersion": "1.1.0",
    "patches": [
      {
        "file": "content/video.mp4",
        "operation": "binary_diff",
        "patchUrl": "https://cdn.example.com/patches/video-1.0-to-1.1.patch"
      },
      {
        "file": "metadata.json",
        "operation": "json_merge",
        "changes": {
          "title": "Updated Title",
          "updatedAt": "2025-01-20T15:00:00Z"
        }
      }
    ]
  }
}
```

## 9. Security

### 9.1 HTTPS/TLS Requirements

- TLS 1.3 minimum
- Strong cipher suites only
- HSTS (HTTP Strict Transport Security)
- Certificate pinning for mobile apps

### 9.2 Content Encryption

```javascript
// Encrypt content for secure storage
async function encryptContent(content, key) {
  const encoder = new TextEncoder();
  const data = encoder.encode(content);

  const encrypted = await crypto.subtle.encrypt(
    {
      name: "AES-GCM",
      iv: generateIV()
    },
    key,
    data
  );

  return encrypted;
}
```

### 9.3 Digital Signatures

```bash
# Sign content package
openssl dgst -sha256 -sign private.pem -out signature.bin content.zip

# Verify signature
openssl dgst -sha256 -verify public.pem -signature signature.bin content.zip
```

## 10. Performance Optimization

### 10.1 Compression

- Gzip compression for text assets
- Brotli compression for modern browsers
- Video: H.265/HEVC for better compression
- Images: WebP/AVIF for modern formats

### 10.2 Resource Hints

```html
<!-- Preconnect to CDN -->
<link rel="preconnect" href="https://cdn.example.com">

<!-- Prefetch next content -->
<link rel="prefetch" href="/content/next-video.mp4">

<!-- Preload critical resources -->
<link rel="preload" href="/styles/critical.css" as="style">
```

## 11. Monitoring and Analytics

### 11.1 Performance Metrics

Track:
- Time to First Byte (TTFB)
- Content load time
- Playback buffering events
- Error rates
- Geographic distribution

### 11.2 CDN Analytics

```json
{
  "analytics": {
    "period": "2025-01",
    "totalRequests": 1000000,
    "bandwidth": 5000000000000,
    "cacheHitRate": 0.95,
    "edgePerformance": {
      "us-east": {"avgLatency": 25, "requests": 400000},
      "eu-west": {"avgLatency": 30, "requests": 300000},
      "asia-northeast": {"avgLatency": 35, "requests": 300000}
    }
  }
}
```

---

**WIA-EDU-019 Phase 3 Complete**
弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association
MIT License


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
