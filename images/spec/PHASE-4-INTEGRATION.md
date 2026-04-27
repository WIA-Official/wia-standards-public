# WIA-images PHASE 4 — INTEGRATION Specification

**Standard:** WIA-images
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited images programme integrates
with the systems that surround it: editorial content-management
systems; image-CDN providers; accessibility tooling and
accessibility-reviewer workflow systems; rights-clearance operators
and collective-management organisations; provenance verification
services; long-term archives; AI-training opt-out registries; and
the publication channels (web, mobile, print) that consume
delivered renditions.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 8601 (date and time)
- W3C WCAG 2.2 (accessibility)
- C2PA Content Authenticity (provenance manifest framework)
- IPTC Photo Metadata schema
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance
  of provenance attestations)

---

## §1 Editorial CMS Integration

Editorial CMS systems integrate via the asset / bitstream / rendition
endpoints. The CMS's client certificate is bound to the operating
publisher's identifier; the CMS publishes assets and consumes
delivery state, rights-clearance status, and accessibility-review
state to drive editorial UX (publish-blocking, review queue,
withdrawal alerts).

## §2 Image-CDN Integration

Image-CDN providers integrate via rendition retrieval, delivery
state reporting, and invalidation. The CDN's client certificate is
bound to the provider's identifier and authorised for the
edge-site identifier set the provider operates. Invalidations
propagated from the operator's withdrawal or rights-lapse events
honour the operator's invalidation-latency targets in §7 of
PHASE-3.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  asset.json                   — asset record
  bitstreams/                  — bitstream metadata + content-
                                  addressed binary URIs
  captures/                    — capture metadata (with redaction
                                  state preserved)
  renditions/                  — rendition metadata + delivery URIs
  accessibility/               — text alternatives and review state
                                  per locale
  provenance/                  — C2PA manifest content-addresses
                                  and verification history
  rights/                      — rights-clearance records and
                                  expiry / geography scopes
  delivery/                    — CDN delivery-state summaries
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed by the
operating programme and counter-signed by the rights-clearance
operator and the provenance signer when both are present.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:images:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-images` that links to the API root, the public
operator accreditation references, the published quality dossier,
the AI-training opt-out endpoint (PHASE-3 §8), the C2PA signing
key chain, and the catalogue of public-domain assets.

## §6 Accessibility Tooling Integration

Auto-alt-text generators and accessibility-reviewer workflow
systems integrate via the accessibility endpoint (PHASE-2 §6).
Auto-tools' client certificates are flagged so that records they
emit are placed in `auto-generated` review state until a human
reviewer attests; reviewer-side systems append the
`human-reviewed` outcome through their dedicated certificate.

## §7 Rights-Clearance Operator Integration

Rights-clearance operators integrate via the rights endpoint
(PHASE-2 §8). The integration carries the operator's identifier,
the partnership scope (which rights-holder portfolios the operator
represents), and the renewal-cadence schedule. Statutory
collective-management organisations integrate through dedicated
adapters that translate between the operator's licence terms and
the WIA-native clearance shape.

## §8 Provenance Verification Service Integration

Provenance verification services consume C2PA manifests through
the provenance endpoint (PHASE-2 §7). The integration is read-only;
the service returns verification outcomes through its own API and
publishes failure notifications back to the operator through the
streaming subscription. Services that re-publish verification
results to public dashboards carry the operator's authorisation.

## §9 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on wind-down, remaining records transfer to
the archive.

## §10 AI-Training Opt-Out Registry Integration

Programmes that publish `ai-training-prohibited` assets register
their opt-out at community-published opt-out registries. The
integration record carries the registry's identifier, the
operator's opt-out scope, and the refresh schedule. Opt-out
registry consumption is read-only on the registry's side; the
operator publishes its opt-out list and consumes the registry's
adoption status.

## §11 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose attestations (rights-clearance
status, accessibility-review pass, provenance manifest validity)
to consumers of W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials. Re-issuance is optional;
the canonical record remains the JSON evidence-package manifest.

## §12 Worked Example: Take-Down Propagation

1. Operator receives a take-down request (DMCA, GDPR erasure, or
   statutory).
2. Take-down workflow verifies the request, withdraws the asset,
   and emits invalidation events to every CDN edge that holds a
   rendition.
3. Invalidation latency is observed against the operator's target;
   exceptions trigger an alert through the streaming subscription.
4. Provenance manifests are amended with the withdrawal step; the
   updated manifest is signed.
5. Citation tools that hold a pinned manifest digest receive a
   superseded-manifest notification and may either retain the
   pinned reference (with the withdrawal context) or upgrade to
   the amended manifest.

A conformant tool completes the technical portion of this flow
without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (publishing
workflows, e-book illustration sets, museum-digital-archive
catalogues) emit cross-standard linkage records that name the
consuming standard and the version under which the linkage is
claimed.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
provenance graphs, accessibility coverage maps, rights-clearance
heat maps) alongside the canonical evidence package. Reader tools
are non-normative.

## §15 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of releasable assets
(a public-domain pool, an editorial gallery, a stock-photo
inventory) emit an Atom or JSON Feed listing assets with their
evidence-package manifest digests, the licence code, the
geographic scope, and the AI-training disposition. The feed is a
discovery mechanism, not a primary record.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full C2PA spec release cycle.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-images reached version 1.0
MAY migrate historical assets by emitting synthetic asset records
with a `legacyImport` flag. Synthetic assets are accepted by the
public catalogue but are not eligible for evidence-package
generation without contemporaneous re-validation under PHASE-3 §3.

## §18 IPTC Metadata Adapter Integration

IPTC metadata adapters bridge between editorial CMS systems'
native metadata fields and the IPTC PhotoMetadata schema. The
adapter is owned by the operating programme and is exercised
during CMS upgrades; submissions whose adapters fail to produce
IPTC-compliant fields trigger an alert through the streaming
subscription and halt publication of the affected asset.

## §19 Generative-Image Pipeline Integration

Programmes that use generative-image pipelines to create synthetic
assets integrate the pipeline through a dedicated synthesis
adapter that records the generator family, the prompt, the
seed, and the operator authorisation in the C2PA manifest's
edit chain. The adapter signs the synthesis step with the
operator's release key so that downstream verification can
confirm the synthetic origin.

## §20 Print-Production Pipeline Integration

Print-production pipelines consume CMYK-converted renditions and
the print-production record's process metadata (PHASE-1 §12).
The integration record carries the press operator's identifier,
the colour-management profile chain, and the proofing-cycle
acceptance threshold. Press-side approvals append to the
rendition's audit chain so that press deliverables are traceable
to the operator's source-of-truth bitstream.

## §21 CRM Mediation for Person-Pictured Subjects

The operating publisher's CRM mediates between depicted persons
and the API. Right-to-erasure or model-release requests submitted
to the CRM trigger the API's take-down or person-pictured update
flow under operator authorisation; the CRM strips clinical or
contact identifiers before submission so that the API never
receives subject PII directly.

## §22 C2PA Trust-List Refresh Integration

Programmes consume the C2PA trust list and signing key chain
through a refresh adapter that runs at the cadence the C2PA
working group publishes (typically weekly). Refresh events emit
stale-manifest alerts (PHASE-3 §20) when previously signed
manifests fall out of trust; the adapter re-verifies the
operator's own published manifests and emits cross-references when
re-signing is required.

## §23 Reduced-Motion Client Honour

Clients (web browsers, mobile apps) that honour user-level reduced-
motion preferences consume the animated asset's
`reduceMotionAlternativeRef` (PHASE-1 §13) and substitute the
still-image fallback transparently. The integration adapter on
the client side reads the operator-declared alternative URI and
caches it alongside the animated rendition so that the swap does
not require an additional network round-trip.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one editorial CMS, at least one image-CDN provider,
at least one accessibility-reviewer workflow, at least one
rights-clearance operator, at least one provenance verification
service, and at least one long-term archive, and has published at
least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-images
- **Last Updated:** 2026-04-27
