# WIA-ART-001: Phase 4 - Integration Specification

**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers integration of WIA-ART-001 implementations with the broader creative-tooling, distribution, archival, and rights-management ecosystems. The Phase 4 design follows the same principle as Phases 1–3: composability over invention. The standard does not duplicate the work of established creative-industry standards; it specifies how WIA-ART-001 manifests, identifiers, and provenance graphs slot into those existing ecosystems without conflict.

---

## 2. Platform Integration

### 2.1 Web Applications

```html
<script type="module">
  import { WIAART001 } from 'https://cdn.wia.org/art-001/v1/wia-art-001.mjs';
  const sdk = new WIAART001({
    apiKey: window.WIA_API_KEY,
    acceptLanguage: navigator.language
  });
  await sdk.attach(document.querySelector('#canvas'));
</script>
```

Browser integrations conform to the W3C HTML5 Living Standard, the W3C Subresource Integrity specification (`integrity` attribute), and the W3C Content Security Policy Level 3 specification.

### 2.2 Mobile Applications

```swift
import WIAART001
let sdk = WIAART001(apiKey: ProcessInfo.processInfo.environment["WIA_API_KEY"]!)
```

```kotlin
import org.wia.art001.SDK
val sdk = SDK(apiKey = BuildConfig.WIA_API_KEY)
```

Mobile integrations conform to platform accessibility frameworks (iOS UIAccessibility, Android Accessibility) at WCAG 2.2 AA equivalent.

### 2.3 Server-Side Integration

```python
from wia_art_001 import SDK
sdk = SDK(api_key=os.environ["WIA_API_KEY"])
result = sdk.create(payload)
```

Server-side integrations support synchronous and asynchronous workflows; asynchronous workflows are recommended for any operation involving signing or large-file canonicalisation.

---

## 3. WIA Ecosystem Integration

| WIA standard | Role |
|--------------|------|
| WIA-INTENT | Intent-based queries against the artwork catalog |
| WIA-OMNI-API | Universal gateway for cross-standard composition |
| WIA-SOCIAL | Social sharing with verifiable creator attribution |
| WIA-AI-017 (AI content auth) | Co-located authentication and detection metadata |
| WIA-WBIN | Universal book/artwork identifier |
| WIA-CHILD-001 | Child-safety filtering for surfaces that include minors |

Cross-standard composition uses the WIA-OMNI-API gateway and follows the WIA composition profile documented separately.

---

## 4. Creative-Tool Integration

### 4.1 Plugin Patterns

Creative tools integrate via plugin frameworks. The reference programme provides plugins for:

- **Adobe Creative Cloud** (Photoshop, Illustrator, Premiere Pro) — UXP plugin format.
- **Procreate** — extension shortcuts.
- **Blender** — Python add-on.
- **Figma** — Plugin API.
- **DaVinci Resolve** — DCTL and Lua scripting.
- **Krita** — Python plugin.
- **Inkscape** — Inkscape extension.

Plugin packaging follows each tool's published guidelines. The packaged plugin is signed under the plugin developer's certificate and submitted to the tool vendor's marketplace where applicable.

### 4.2 Open File Formats

The reference programme prefers open file formats throughout:

- **Raster** — PNG (ISO 15948), JPEG (ISO/IEC 10918-1), HEIF (ISO/IEC 23008-12), AVIF (AOMedia AV1), WebP, OpenEXR for HDR.
- **Vector** — SVG (W3C), PDF/X (ISO 15930 series).
- **3D** — glTF 2.0 (Khronos), USDZ, COLLADA (ISO/PAS 17506:2012), FBX (proprietary, accepted with documented limitations).
- **Video** — Matroska / WebM containers with AV1 (AOMedia) or HEVC (ISO/IEC 23008-2).
- **Audio** — FLAC, Opus (RFC 6716), WAV per IETF/EBU conventions.

Closed or proprietary formats are accepted but flagged in the manifest as `format.openness = "proprietary"` so that downstream consumers can plan accordingly.

### 4.3 Colour Management

Colour management uses ICC profiles (ISO 15076-1:2010). The reference working colour space for raster artwork is sRGB IEC 61966-2-1:1999, with Display P3 and Rec. ITU-R BT.2020 supported for wide-gamut deployments. Colour appearance models follow CIECAM02 or CIECAM16 where rendering accuracy is critical.

---

## 5. Marketplace and Distribution Integration

### 5.1 Marketplace Connectors

Marketplace integrations follow the marketplace's published API. The reference programme provides connectors for:

- **OpenSea** — REST API.
- **Rarible** — REST API.
- **Foundation** — REST API.
- **SuperRare** — REST API.
- **Manifold** — REST API.

Cross-marketplace identifier resolution uses the artwork's WIA-ART-001 identifier as the canonical anchor; marketplace-specific identifiers map back to the WIA identifier in the manifest's external-id block.

### 5.2 Rights and Licensing

License identifiers follow the SPDX License List where applicable. Creative Commons licenses (CC0, CC-BY-4.0, CC-BY-SA-4.0, CC-BY-NC-4.0, CC-BY-ND-4.0, CC-BY-NC-SA-4.0, CC-BY-NC-ND-4.0) are recognised under their canonical identifiers.

For non-SPDX licenses, the manifest carries the licence URL and a SHA-256 hash of the licence text at the time of issuance.

### 5.3 Royalty and Resale Terms

Royalty terms follow the EIP-2981 (Ethereum Improvement Proposal) NFT Royalty Standard for blockchain marketplaces, and the marketplace's native conventions for non-blockchain channels. Terms travel with the artwork as a manifest assertion under the rights-claim block.

---

## 6. Storage Integration

### 6.1 Content Addressing

Persistent storage of artwork bytes uses content addressing where possible:

- **IPFS** — CIDv1 with SHA-256 hashing.
- **Arweave** — transaction IDs.
- **S3-compatible** — versioned objects with stored hash metadata.
- **Filecoin** — deal CIDs.
- **Self-hosted CDN** — URLs paired with explicit hash metadata in the manifest.

The manifest carries the storage URI and the canonical hash so that integrity is verifiable independently of the storage provider's availability.

### 6.2 Long-Term Archival

For long-term archival, the reference programme follows the OAIS Reference Model (ISO 14721:2012) for archive structure and metadata. Archival packages include the artwork bytes, the canonicalised manifest, and the verification chain.

---

## 7. Accessibility

User-facing surfaces conform to W3C WCAG 2.2 Level AA. Conformance is verified by automated checks (Axe Core or equivalent) and by manual review against the WCAG 2.2 success criteria.

For artworks themselves, the manifest carries optional accessibility metadata:

- `accessibility.alt_text` — short text alternative.
- `accessibility.long_description_uri` — extended description.
- `accessibility.audio_description_uri` — audio description for video and motion artworks.
- `accessibility.captions_uri` — captions for time-based media.
- `accessibility.transcript_uri` — transcript for time-based media.

Inclusion of these fields is encouraged for any artwork distributed at scale.

---

## 8. Internationalisation

Locale data follows BCP 47 (RFC 5646) and Unicode CLDR. Date and time storage follows ISO 8601:2019 in UTC; rendering follows the user's locale at presentation time.

Right-to-left scripts are supported through the W3C International Text Layout (CSS Logical Properties) and Unicode Bidirectional Algorithm (UAX #9).

---

## 9. Security and Privacy

Information security follows ISO/IEC 27001:2022. Privacy follows ISO/IEC 27701:2019, GDPR (Regulation (EU) 2016/679), CCPA/CPRA, Korea PIPA, and Brazil LGPD.

Privacy considerations specific to digital art:

- Personal data of named-author identities is treated as personal data under the deploying jurisdiction's law.
- Pseudonymous attribution (DID without offline identity binding) is supported.
- Right to withdraw attribution is honoured per the rule of the deploying jurisdiction; revocation produces a manifest amendment recorded in the provenance graph.

---

## 10. Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Web platform | W3C HTML5, W3C CSS3, W3C SVG, W3C SRI, W3C CSP Level 3 |
| Mobile accessibility | W3C WCAG 2.2 (mapped to native frameworks) |
| Raster formats | ISO/IEC 10918-1 (JPEG), ISO 15948 (PNG), ISO/IEC 23008-12 (HEIF), AOMedia AV1 (AVIF) |
| Vector formats | W3C SVG 2, ISO 15930 (PDF/X) |
| 3D formats | Khronos glTF 2.0, ISO/PAS 17506 (COLLADA) |
| Video | ISO/IEC 14496-10 (AVC), ISO/IEC 23008-2 (HEVC), AOMedia AV1 |
| Audio | ISO/IEC 14496-3 (AAC), RFC 6716 (Opus), ISO/IEC 11172-3 (MP3), FLAC |
| Colour | ISO 15076-1:2010, IEC 61966-2-1:1999 (sRGB), Rec. ITU-R BT.2020 |
| Royalty (blockchain) | EIP-2981 |
| Licenses | SPDX License List, Creative Commons 4.0 |
| Archival | ISO 14721:2012 (OAIS) |
| Storage hashing | FIPS 180-4 (SHA-2), FIPS 202 (SHA-3) |
| Information security | ISO/IEC 27001:2022, 27002:2022 |
| Privacy | ISO/IEC 27701:2019, GDPR, CCPA/CPRA, PIPA, LGPD |
| Locale | BCP 47 (RFC 5646), Unicode CLDR, UAX #9 |
| Time | ISO 8601:2019 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## 11. Conformance

A Phase 4 integration is conformant when:

1. Platform integrations follow the §2 patterns and platform-native accessibility expectations.
2. Creative-tool plugins follow each tool's signing and packaging conventions.
3. Marketplace connectors map external IDs back to WIA-ART-001 identifiers in the manifest.
4. Storage integrations preserve the canonical hash for integrity verification.
5. Accessibility metadata is included where applicable.
6. Privacy and security frameworks map to a published statement of applicability.

## 12. Operational Appendix

### 12.1 Provenance Graph in Practice

Each artwork's provenance graph captures the chain of activities that produced and modified it, conforming to W3C PROV-O. In practice the graph is visualised as a directed acyclic graph with three node types:

- **Entity** — the artwork or any derivative of it.
- **Activity** — an act of creation, modification, signing, or distribution.
- **Agent** — the human or software responsible for an activity.

Operating organisations publish their provenance-graph schema as a SHACL shape so that conformance can be machine-verified. Independent verifiers consume the same shape and produce auditable verification reports.

### 12.2 Migration from Single-Tool Workflows

Artists migrating from single-tool workflows to WIA-ART-001 typically adopt the standard incrementally:

1. **Manifest-only adoption** — Existing artworks are wrapped in WIA-ART-001 manifests without changing the underlying creative process.
2. **Signing adoption** — The artist or studio adopts an Ed25519 key pair and begins signing manifests at publication time.
3. **Provenance adoption** — The studio captures full provenance graphs from creation through publication, replacing or augmenting prior version-control systems.
4. **Collaborative adoption** — Multi-artist works adopt the Phase 3 collaborative protocol for real-time co-authorship.

Each adoption step is independently valuable and the WIA-ART-001 design does not require the full migration in order to derive value from the partial adoption.

### 12.3 Operating Organisation Responsibilities

Operating organisations that host WIA-ART-001 services for artists carry specific responsibilities:

- **Key custody** — When the operating organisation custodies signing keys on behalf of artists, custody follows ISO/IEC 11770-3 with HSM-backed storage and a published incident response procedure.
- **Privacy notice** — A privacy notice in plain language and the artist's locale is mandatory.
- **Dispute resolution** — A documented dispute-resolution procedure for attribution, rights, and royalty claims.
- **Termination of service** — A documented procedure for artist-initiated and operator-initiated termination of service, including data export in canonical WIA-ART-001 format.

### 12.4 Identifier Stability

Artwork identifiers are stable for the artwork's lifetime. If an artwork is amended (rights change, attribution change, errata), the amendment is recorded as a manifest revision and the identifier is preserved. If a previously-published artwork is withdrawn for legal reasons, the identifier is preserved as a tombstone with the withdrawal reason class (without revealing personal data).

### 12.5 Sustainability Reporting

Operating organisations may publish sustainability reports following ISO 14064-1:2018 (greenhouse-gas inventory) and ISO 50001:2018 (energy management). For digital art platforms, the dominant sustainability concern is typically the energy footprint of creative-tool compute and content delivery, both of which are addressable through hosting choices and efficient encoding.

### 12.6 Closing

WIA-ART-001 is built to make digital artwork verifiable, attributable, and durable across the full lifetime of the artwork — well beyond the lifetime of any specific platform or marketplace. The integration discipline in this Phase is the operational expression of that goal: it is not enough to produce a beautiful work; the work must remain identifiable as the artist's contribution, with the correct rights and attribution, through every platform it passes through and every form it is transformed into. That continuity is what the WIA-ART-001 stack provides.

---

**弘益人間 (Benefit All Humanity)**
*© 2025 WIA*
