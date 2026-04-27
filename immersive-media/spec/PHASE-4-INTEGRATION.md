# WIA-immersive-media PHASE 4 — INTEGRATION Specification

**Standard:** WIA-immersive-media
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited immersive-media programme
integrates with the systems that surround it: HMD platforms; WebXR
client runtimes; mixed-reality anchor services; comfort and
accessibility tooling; rights-clearance operators; provenance
verification services; long-term archives; and content discovery
and recommendation services that surface immersive content to end
users.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- W3C WebXR Device API
- W3C WCAG 2.2
- C2PA Content Authenticity (provenance manifest framework)
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance)

---

## §1 HMD Platform Integration

HMD platforms (consumer and enterprise headsets, AR glasses,
mobile-tethered displays) integrate via asset retrieval and the
platform-side gating endpoints. The platform's client certificate
is bound to the platform vendor's identifier and authorises the
platform to consume comfort, accessibility, and rights metadata in
the platform's gating decisions.

## §2 WebXR Runtime Integration

WebXR-capable browsers consume scenes and spatial audio through
the asset endpoints. The runtime relies on the comfort profile
(PHASE-1 §7) to render user-facing comfort prompts before playback;
the operator's well-known discovery document advertises the
playback prompts and the gating thresholds for each release.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  asset.json                   — asset record
  scenes/                      — scene metadata and content-
                                  addressed scene files
  volumetric/                  — volumetric records
  spatial-audio/               — spatial-audio records
  accessibility/               — accessibility records per locale
  comfort/                     — comfort profiles
  mr-anchors/                  — MR-anchor records
  provenance/                  — C2PA manifests
  rights/                      — rights-clearance records
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by the
operating programme and counter-signed by the rights-clearance
operator and the provenance signer when both are present.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:immersive-media:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-immersive-media` that links to the API root,
the public producer accreditation references, the published
quality dossier, the comfort and gating thresholds, the licence-
code catalogue (PHASE-3 §9), and the catalogue of released assets.

## §6 Comfort Tooling Integration

Comfort tooling (playtest harnesses, head-tracking analysis,
nausea-rating recorders) integrates via the comfort-profile
endpoint (PHASE-2 §6). Tools' client certificates are bound to
their accreditation reference; tools that lack the operator's
accreditation are accepted into the project but their classifications
require human override before publication.

## §7 Accessibility Tooling Integration

Auto-generated scene descriptions and caption tooling integrate
via the accessibility endpoint (PHASE-2 §6). Auto-tools' client
certificates are flagged so that records they emit are placed in
`auto-generated` review state until a human reviewer attests.

## §8 Rights-Clearance Operator Integration

Rights-clearance operators integrate via the rights endpoint
(PHASE-2 §8). The integration carries the operator's identifier,
the partnership scope, and the renewal-cadence schedule.

## §9 Provenance Verification Service Integration

Provenance verification services consume C2PA manifests through
the provenance endpoint. The service runs the signature chain
against the in-force C2PA trust list and emits verification
results back to the operator through the streaming subscription.

## §10 MR Anchor Service Integration

Mixed-reality anchor services consume MR-anchor records and
publish per-anchor refresh events. The integration is bidirectional:
the operator publishes anchor placement and the service publishes
the anchor's resolved transform after platform-side localisation.

## §11 Long-Term Archive Integration

Programmes designate a long-term archive that holds scene files,
volumetric sequences, and evidence packages beyond programme wind-
down. Quarterly deposits round-trip content-addresses; on
wind-down, remaining records transfer to the archive.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites an
immersive-media experience. The reader's tool resolves the citation
by:

1. Parsing the citation to extract the asset ID and manifest digest.
2. Fetching the discovery document for the issuing programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (scene file, volumetric sequence, comfort
   profile, accessibility description) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (web publishing,
museum-digital-archive, medical-imaging) emit cross-standard
linkage records that name the consuming standard and the version
under which the linkage is claimed.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (preview
thumbnails, timeline visualisers, comfort-profile dashboards)
alongside the canonical evidence package. Reader tools are non-
normative.

## §15 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of releasable assets
emit an Atom or JSON Feed listing assets with their evidence-
package manifest digests, the asset class, the comfort profile
summary, and the licence code. The feed is a discovery mechanism,
not a primary record.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full WebXR Device API specification revision cycle
so that downstream platforms have time to migrate.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-immersive-media reached
version 1.0 MAY migrate historical assets by emitting synthetic
asset records with a `legacyImport` flag. Synthetic assets are
accepted by the public catalogue but are not eligible for
evidence-package generation without contemporaneous re-validation
under PHASE-3 §3.

## §18 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (comfort review pass,
accessibility review pass, rights-clearance status) to consumers
of W3C Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0 specification.
Re-issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §19 Performer Engagement Integration

Programmes that engage live performers (motion-capture sessions,
voice talent for spatial-audio dialogue) integrate with performer
booking, payroll, and likeness-management systems through dedicated
adapters. The integration carries the performer's opaque release
token (PHASE-1 §6 of WIA-images analogous shape, adapted for
immersive-media) and the contract reference; performer identity
remains in the operator's HR system.

## §20 Live-Event Production Integration

Producers of live immersive events (concert capture, sports
broadcast in 360°, conference streaming with MR overlays) integrate
with broadcast-control rooms through latency-aware adapters that
relay live content into the asset pipeline. Live deliveries follow
a streamlined version of the gating workflow: comfort and
accessibility metadata are pre-registered for the recurring event
template, and per-event variations are appended just-in-time.

## §21 HMD Telemetry-Pipeline Integration

HMD platforms integrate with the telemetry endpoint (PHASE-2 §18)
via a one-way push from platform to operator. The platform's
client certificate authorises submissions; submissions exceeding
the platform's per-asset rate limit are throttled, with overflow
recorded as aggregate counts rather than dropped data.

## §22 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue emit an Atom or JSON
Feed listing released assets with their evidence-package manifest
digests, the asset class, the comfort-profile summary, the licence
code, and the supported streaming tiers. The feed is intended for
content-discovery services and does not carry telemetry detail.

## §23 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full WebXR Device API specification revision cycle.

## §24 Discovery and Recommendation Service Integration

Content-discovery and recommendation services consume the public
catalogue feed and the per-asset comfort profile to surface
appropriate content to end users. Discovery integrations are
read-only; the operating programme does not relinquish
authoritative status to the recommendation engine. Recommendation
results that surface mature-rated content to users below the
recommended age are governed by the discovery service's age-
verification chain, not by the producing operator.

## §25 Embargo and Coordinated-Launch Integration

Coordinated launches integrate with the operator's embargo
endpoint (PHASE-3 §22). The integration carries the launch
identifier, the release time, the pre-release consumer set, and
the post-release public-catalogue activation timestamp.

## §26 Reader Tooling for Comfort and Accessibility Reviewers

Comfort reviewers, accessibility reviewers, and platform-side
content moderators benefit from visualisation tools that surface
playback heatmaps (where users abandoned), comfort-warning trigger
points, and accessibility-coverage maps. Programmes MAY publish
such reader tools alongside the canonical evidence package; the
tools are non-normative and remain under the operator's editorial
control rather than being relinquished to the integrating
platform.

## §27 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one HMD platform, at least one WebXR runtime, at
least one comfort tooling integration, at least one accessibility-
reviewer workflow, at least one rights-clearance operator, and at
least one long-term archive, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-immersive-media
- **Last Updated:** 2026-04-27
