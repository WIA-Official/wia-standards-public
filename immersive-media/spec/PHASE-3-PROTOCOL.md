# WIA-immersive-media PHASE 3 — PROTOCOL Specification

**Standard:** WIA-immersive-media
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
immersive-media programme: producer accreditation, scene-format
versioning, comfort and safety governance, accessibility-review
discipline, photosensitive-content gating, age gating, MR-anchor
governance, provenance attestation policy, rights-clearance
discipline, records retention, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- W3C WCAG 2.2 (accessibility, including §2.3 photosensitive
  flash conventions)
- W3C WebXR Device API
- C2PA Content Authenticity (provenance manifest framework)

---

## §1 Producer Accreditation

A producer MAY claim conformance to WIA-immersive-media only after
a recognised accreditation body has issued a valid certificate
against ISO 9001:2015 and against the producer's information
security management posture (ISO/IEC 27001:2022). Producers that
publish to children's audiences additionally maintain a child-
safety competency framework.

## §2 Scene-Format Versioning

Scene formats (glTF 2.0, USD, MPEG-I families) evolve with
specification revisions. Producers pin the format revision per
asset; revisions emit new scene records and prior records remain
addressable as the historical state.

## §3 Comfort and Safety Governance

Comfort profiles (PHASE-1 §7) are reviewed by a comfort reviewer
qualified under the operator's competency framework. Reviewers
exercise the asset under representative HMD hardware and validate
the declared `motionIntensity`, `artificialLocomotion`, and
`flashPhotosensitiveRisk` against operator playtest conventions.

Assets whose declared profile does not match the playtest
observation are returned to the producer for reclassification
before publication.

## §4 Photosensitive Content Gating

Assets with `flashPhotosensitiveRisk = "above-WCAG-threshold"`
follow a published gating workflow: explicit user confirmation at
playback start, persistent reminder at scene transitions, and an
operator-side log of consent. The workflow is documented in the
operator's quality dossier and is verified during the annual
ISO 9001 surveillance audit.

## §5 Accessibility-Review Discipline

Auto-generated scene descriptions and captions MUST be reviewed by
a qualified accessibility reviewer before the asset transitions to
`ready-for-delivery`. The qualification framework follows WCAG 2.2
expectations adapted for immersive content (long-form scene
description that narrates spatial layout, captions for spatial-
audio dialogue, audio-described tracks for blind / low-vision
users).

## §6 Age Gating

Assets with `ageRecommendation` of `13+`, `16+`, or `18+` are
gated by the platform's age verification per the operating
jurisdiction's applicable rules. The operator records the
jurisdiction-specific gating mechanism in the platform integration
record.

## §7 Mixed-Reality Anchor Governance

MR-anchor records (PHASE-1 §8) bound to physical locations honour
operator-side rules that prevent anchor placement on protected
sites (heritage zones, places of worship, private property, sites
under emergency operational status). The anchor service maintains
an exclusion-zone registry that the API consults at anchor-
registration time.

## §8 Provenance Attestation Policy

Programmes that publish externally cited immersive content attach
a C2PA-aligned provenance manifest at publication. The manifest
signs the scene, volumetric, and spatial-audio digests as well as
the edit chain. Verification re-runs the signature chain on every
retrieval and on a scheduled sweep.

## §9 Rights-Clearance Discipline

Rights-clearance operators verify licence terms, geographic
scope, and duration. Renewal events emit new clearance records;
expired clearances retain at the prior content-address but are
flagged in the public catalogue.

Rights metadata for immersive content includes asset-class-
specific licence codes for derivative usage (MR-anchor placement,
VR-experience embedding). Programmes publish their licence-code
catalogue in the discovery document for downstream consumer
reference.

## §10 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, accessibility reviews, comfort profiles, rights
clearances, and provenance manifests — retain for a minimum of
seven calendar years from the last access of the asset.
Externally cited assets retain indefinitely.

## §11 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) so that
production, review, and publication events can be ordered
unambiguously across producer studios and platform services.

## §12 Cybersecurity

Producer-to-platform deliveries flow over mutually-authenticated
TLS 1.3 (RFC 8446). Provenance manifests are signed by the
operator's release key; verification failure halts the publication
pipeline.

## §13 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers via the well-known discovery document, and publishes a
sunset timeline for in-flight assets.

## §14 Quality Dossier

The programme's quality dossier records the platforms it integrates
with, the comfort-reviewer competency framework, the
photosensitive-content guard's playtest conventions, the
accessibility-reviewer competency framework, and the deprecation
history of scene formats it has supported. The dossier is reviewed
at least annually by the operator's quality manager.

## §15 Cross-Border Operation

Programmes that operate across borders honour each partner
jurisdiction's age-rating rules and content-classification
authorities; locale-specific age ratings are recorded against the
asset's comfort profile.

## §16 Avatar Likeness and Performer-Capture Governance

Avatar and character variants (PHASE-1 §12) that derive from a
real person's likeness or performance follow the operator's
likeness-rights workflow: signed performer release, model-release
where applicable, and the performer's ongoing right to revoke
post-publication. Likeness revocations follow the take-down
workflow described in §17.

Synthetic-performer variants generated by AI models carry the
synthesis-disclosure conventions documented in §8 and are gated
to prevent impersonation of identifiable people without their
release.

## §17 Take-Down and Likeness-Revocation Handling

Take-down requests (rights-holder claim, GDPR right-to-erasure,
performer likeness-revocation) follow a documented workflow that
the operator records in the quality dossier: intake, verification,
asset withdrawal, MR-anchor invalidation where the asset includes
anchors, performer notification, and audit-log entry. Withdrawn
assets remain addressable at their content-addressed URLs but
are flagged in the public catalogue and gated at the platform
level.

## §18 Streaming Tier Governance

Streaming-variant submissions (PHASE-1 §11) are reviewed for
target-tier appropriateness: scene budgets that exceed the tier
target return to the producer for re-decimation, and tiers that
lack at least one variant block publication of the asset to that
tier. Programmes that operate on a smaller number of supported
tiers declare the supported set in the discovery document so that
clients on unsupported tiers can fall back gracefully.

## §19 Inter-Platform Portability

Producers that target multiple HMD platforms in parallel verify
that scenes render comparably across platforms; cross-platform
divergences (lighting model differences, shader-capability
differences) are recorded as known issues against the asset and
documented for downstream consumers.

## §20 Telemetry Privacy

Quality-of-experience telemetry (PHASE-1 §13) is privacy-preserving
by construction: opaque session tokens, minimum aggregation
windows, and platform-side filtering of any user-identifying
metadata. The operator's privacy framework, recorded in the
quality dossier, is reviewed at least annually to confirm that
the telemetry pipeline does not enable re-identification by
intersection with platform-side data.

## §21 Manifest Re-Verification Cadence

Provenance manifest sweeps run at the cadence the operator
declares (typically weekly or per-C2PA-trust-list-update). Sweep
failures emit alerts and are recorded as append-only events in the
provenance verification history (PHASE-1 §14). Programmes with
externally cited assets re-publish the latest verification outcome
in the asset's evidence package on every sweep so that downstream
readers can verify the manifest's current trust state.

## §22 Embargoed Releases

Coordinated immersive-media launches (a console-launch tie-in, a
festival premiere) are held under embargo until the agreed
release time. The operating programme records the embargo holder,
the release time, and the authorised pre-release consumer set.
Pre-release consumer access is gated by their authorisation; the
API returns `403 Forbidden` with type
`urn:wia:immersive-media:embargo-active` to non-authorised
clients before the release time.

## §23 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive, notifies known external
citers, and publishes a sunset timeline for in-flight assets.
Rights-clearance obligations that survive the operating programme
(performer royalty schedules, ongoing licence terms) are
transferred to a successor or to a recognised licence-administration
body recorded in the operator's quality dossier.

## §24 Performer Health and Safety

Live-action volumetric capture and motion-capture sessions follow
the operator's performer health-and-safety framework: appropriate
session-duration limits, motion-capture-suit hygiene, rest cadence
during long shoots, and incident reporting for performer injury
during capture. The framework is documented in the quality dossier
and is reviewed in step with the operator's broader occupational-
safety governance under ISO 45001-aligned practice where
applicable.

## §25 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (medical-imaging
for clinical immersive training, museum-digital-archive for
curated cultural-heritage scenes, web publishing for WebXR
distribution) emit cross-standard linkage records that name the
consuming standard and the version under which the linkage is
claimed. Linkages are not transitive; consumers verify each
adjacent standard's evidence directly.

## §26 Streaming Tier Verification

Programmes verify that registered streaming variants (PHASE-1 §11)
render correctly on the target client tier before publication.
Verification is exercised on representative hardware (or on the
operator-approved emulator equivalent) and is recorded against the
streaming-variant record. Tiers that lack a passing verification
are gated at publication.

## §27 Conformance and Auditing

A programme conformant with WIA-immersive-media publishes its
producer accreditation references, its programme code registration,
its quality dossier, and the catalogue of published assets, and
answers an annual self-assessment that maps each clause of this
PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-immersive-media
- **Last Updated:** 2026-04-27
