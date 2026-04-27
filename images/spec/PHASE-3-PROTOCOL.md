# WIA-images PHASE 3 — PROTOCOL Specification

**Standard:** WIA-images
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
images programme: operator accreditation, GPS redaction policy,
accessibility-review discipline, rights-clearance discipline,
provenance attestation policy, derivative recipe versioning,
delivery-state hygiene, AI-training opt-in / opt-out signalling,
records retention, take-down handling, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- W3C WCAG 2.2 (accessibility)
- C2PA Content Authenticity (provenance manifest framework)
- IPTC Photo Metadata (community-managed metadata schema; cited
  normatively as a metadata interchange envelope)

---

## §1 Operator Accreditation

An images programme MAY claim conformance to WIA-images only after
a recognised accreditation body has issued a valid certificate
against ISO 9001:2015 and against the operator's information
security management posture (ISO/IEC 27001:2022). Programmes that
operate accessibility review at scale SHOULD additionally maintain
an accessibility-review competency framework documented in their
quality dossier.

## §2 GPS Redaction Policy

Programmes that publish photographs of people, sensitive sites, or
private property MUST apply a GPS redaction policy that defaults
to redaction on ingestion. Operator-side override is permitted for
landscape and editorial-context assets where geolocation is
relevant; the override decision is recorded against the capture
record. GPS redaction is one-way at the bitstream level once a
derivative is published.

## §3 Accessibility Review Discipline

Auto-generated text alternatives MUST be reviewed by a qualified
accessibility reviewer before the asset transitions to
`ready-for-delivery`. The qualification framework is documented in
the operator's quality dossier and follows the WCAG 2.2
text-alternative conventions for non-text content (1.1.1).

Decorative-image classification (PHASE-1 §6 `isDecorative=true`)
is reviewed by a human at first publication; auto-classification
of decorative images is permitted only when the auto-tool's
precision against the operator's gold-set exceeds the operator's
declared threshold (typically 95% on the operator's calibration
sample).

## §4 Rights-Clearance Discipline

Rights-clearance operators verify that the asset's licence terms
are in force, that the operator is authorised to use the asset in
the intended use defined in §2 of PHASE-1, and that the licence
covers the geographic scope of intended publication. Renewal
events emit new clearance records; expired clearances retain at
the prior content-address but are flagged in the public catalogue.

Programmes operating in jurisdictions with statutory licensing
schemes (collective management organisations, statutory image
levies) record the scheme's identifier and the operator's
membership status in the rights-clearance record.

## §5 Provenance Attestation Policy

Programmes that publish externally cited photographs (news, legal
evidence, scholarly publication) attach a C2PA-aligned provenance
manifest at publication. The manifest signs the bitstream digest,
the editing chain, and the publishing operator's identity.
Verification re-runs the signature chain on every retrieval; failed
verification is exposed alongside the asset so that downstream
consumers can decide whether to trust an unverified manifest.

The C2PA framework is community-managed and evolves; programmes
pin the C2PA spec revision in the manifest record so that
verification re-runs against the same revision the manifest was
issued under.

## §6 Derivative Recipe Versioning

Rendition recipes (PHASE-1 §5) are content-addressed; recipe
revisions emit new identifiers rather than overwriting the prior
recipe. Re-running an old recipe against the same source produces
a byte-identical rendition, so historical builds are reproducible.

Recipe deprecation follows a published timeline; deprecated
recipes remain executable but the public catalogue marks
renditions produced under deprecated recipes for downstream
awareness.

## §7 Delivery-State Hygiene

CDN edges that serve renditions report delivery state through the
delivery endpoint (PHASE-2 §9). Invalidation events propagate
through the edge mesh to honour withdrawal notices and rights-
clearance lapses; invalidation latency targets are recorded in
the operator's quality dossier and audited at the annual
ISO 9001 surveillance.

## §8 AI-Training Opt-In / Opt-Out

Programmes record an explicit AI-training disposition for every
asset: `ai-training-permitted`, `ai-training-prohibited`, or
`silent` (operator declines to declare). Public-facing renditions
expose the disposition through the IPTC PLUS metadata when
present, the C2PA manifest's training disposition assertion, and
HTTP response headers conforming to community-published opt-out
conventions.

Programmes that ship `ai-training-prohibited` assets through
public CDNs MUST respect crawler robots-txt-style opt-outs at the
operator's discoverable URI; honouring the opt-out is the
operator's commitment, not a guarantee against non-compliant
crawlers.

## §9 Records Retention

Programme records — every record defined in PHASE-1, the API audit
logs, accessibility reviews, rights clearances, provenance
manifests, and delivery-state events — retain for a minimum of
seven calendar years from the last access of the asset. Externally
cited assets retain indefinitely; on programme wind-down
indefinite-retention assets transfer to a recognised long-term
archive.

## §10 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) so that capture
times, edit chain timestamps, and delivery events can be ordered
unambiguously across operator systems and CDN edges.

## §11 Take-Down Handling

Take-down requests (DMCA-style notifications, GDPR right-to-
erasure, statutory take-downs) follow a documented workflow:
intake, verification, action (withdraw asset and propagate
invalidation), notification of the original rights claimant or
data subject, and audit-log entry. The workflow is documented in
the operator's quality dossier and is exercised through the
asset withdrawal and delivery-invalidation endpoints.

## §12 Cybersecurity

Editorial CMS uploads, rights-clearance integrations, and CDN
edge connections operate over mutually-authenticated TLS 1.3
(RFC 8446). Provenance manifests are signed by the operator's
release key; verification failure halts the publication pipeline.

## §13 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers, and publishes a sunset timeline for in-flight assets. The
wind-down workflow honours pending rights-clearance obligations
through to expiry where the operator has remaining rights.

## §14 Quality Dossier and Cross-Border Operation

The programme's quality dossier records the rights-clearance
operators it works with, the provenance signing key chain, the
accessibility-reviewer competency framework, the take-down case
log, and the deprecation history of derivative recipes. Multi-
jurisdiction programmes maintain operating MoUs with partner
jurisdictions and honour each partner's data-protection law for
person-pictured assets.

## §15 IPTC Metadata Discipline

IPTC PhotoMetadata fields (PHASE-1 §10) are mandatory for editorial-
context assets. The operating programme records the IPTC fields at
ingestion through the editor's workflow and re-validates them at
publication; missing creator or copyright fields halt publication
under the workflow's enforcement.

Programmes that publish editorial photographs to wire services or
syndication networks include IPTC fan-out in the publication step
so that consuming systems do not need to fetch the WIA-native
sidecar separately.

## §16 Generative-Image Disclosure

Generative-image and synthetic-rendered assets (PHASE-1 §2
`sourceKind=synthetic-rendered`) carry an explicit synthesis
disclosure in the C2PA manifest's edit chain (PHASE-1 §7
`stepKind=synthesised`). The disclosure names the generator family,
the operator's authorisation to publish under the operator's
editorial standards, and the model-output content-address.

## §17 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary
jurisdiction of registration and operating MoUs with partner
jurisdictions. Cross-jurisdictional data transfers honour the
source-jurisdiction's data-protection and image-rights law for any
asset that pictures identifiable people or sensitive sites.

## §18 Person-Pictured Discipline

Editorial-context assets that depict identifiable people are
governed by the operator's image-rights workflow: model release
where applicable, public-figure / public-event exemption where
applicable, and `no-release-no-publication` for assets that lack
a defensible release. Minor-pictured assets follow stricter
governance (parental or guardian consent, jurisdiction-specific
broadcasting law) recorded in the person-pictured record.

Right-to-erasure or image-removal requests from depicted persons
flow through the operator's CRM and trigger the take-down workflow
(§11). The operator records the request reference and the
disposition.

## §19 Generative-Image Rights Disposition

Generative-image and synthetic-rendered assets carry their own
rights disposition: the operator's authorisation to use the
generator's outputs under the generator's licence, the operator's
authorisation to attribute the synthesis (or to omit attribution
when the generator's licence permits), and the operator's
declaration of the AI-training disposition for the synthetic
output. Programmes that publish synthetic editorial illustrations
treat the synthesis disclosure as a precondition for the publish
step, not as an optional metadata enrichment.

## §20 Stale-Manifest Handling

C2PA manifests reference an external trust list and a signing key
chain. When the trust list rotates or a signing key is revoked,
existing manifests become stale. The operating programme runs a
periodic re-verification sweep (typically weekly) that re-validates
manifests against the current trust list and emits stale-manifest
alerts; consumers of stale manifests receive the alert through the
streaming subscription so that they can re-fetch the freshest
manifest for the asset.

## §21 Reduced-Motion and Sensory Accessibility

Animated and sequence-image assets (PHASE-1 §13) are exposed
to consumers along with a still-image fallback honoured by clients
that respect user reduced-motion preferences. The operating
programme records the fallback URI for every animated asset and
verifies its presence at publication; assets that lack a fallback
cannot transition to `ready-for-delivery`.

Programmes that publish animated assets at sites with photosensitive-
seizure risk (children's media, public-display advertising)
additionally record the asset's flash and contrast profile against
WCAG 2.2 §2.3 expectations and exclude assets that fail the profile
from publication into those contexts.

## §22 Operator Quality Dossier and Annual Review

The operator's quality dossier (PHASE-3 §14) is reviewed at least
annually by the operator's quality manager, the operator's
accessibility advisor, and the operator's rights-clearance lead.
The review's outcomes are recorded as content-addressed minutes
that the public catalogue references; programmes that publish
externally cited assets attach the most recent review reference to
the evidence package's audit section.

## §23 Conformance and Auditing

A programme conformant with WIA-images publishes its operator
accreditation references, its rights-clearance partner list, its
quality dossier, and the catalogue of published assets, and
answers an annual self-assessment that maps each clause of this
PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-images
- **Last Updated:** 2026-04-27
