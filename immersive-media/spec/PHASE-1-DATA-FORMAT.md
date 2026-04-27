# WIA-immersive-media PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-immersive-media
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-immersive-media. The standard covers exchange of immersive-
media artefacts — volumetric video, 3-D scenes, point clouds, 360°
spherical video, mixed-reality anchors, and spatial audio — among
producers, distributors, head-mounted-display platforms, browser-
based WebXR clients, and content-discovery services. The format
captures asset identity, scene-graph structure, encoding variants,
spatial-audio metadata, accessibility descriptions, comfort and
safety profiles, and rights-clearance state.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 11578 (UUID)
- ISO/IEC 23090 (MPEG-I Immersive Media — coded representations)
- ISO/IEC 23008 (MPEG-H part series — used for 3-D audio coding
  envelopes)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- W3C glTF 2.0 (community-managed 3-D scene format; cited
  normatively as the interchange envelope for scene records)
- W3C WebXR Device API and WebXR Layers (cited normatively for
  client-side rendering capability descriptions)
- W3C WCAG 2.2 (text-alternative and reduced-motion conventions)
- Khronos KTX 2.0 / Basis Universal (texture compression)
- C2PA Content Authenticity (provenance manifest framework)

---

## §1 Scope

This PHASE document defines persistent shapes for the records that
flow through immersive-media production, distribution, and client
playback. Implementations covered include:

- Volumetric-video producers that capture scenes with multi-camera
  arrays or depth sensors.
- 3-D scene authors that compose virtual environments for VR / MR
  consumption.
- Point-cloud producers (lidar surveys, photogrammetry pipelines).
- 360° spherical video producers and stitching pipelines.
- Mixed-reality anchor services that publish persistent spatial
  anchors keyed to physical locations.
- HMD platforms and WebXR client runtimes that consume the
  produced artefacts.

Live broadcast of immersive content, real-time multiplayer game
state, and avatar facial-performance capture pipelines are out of
scope.

## §2 Asset Identifier

```
assetId           : string (uuidv7)
assetCreatedAt    : string (ISO 8601 / RFC 3339)
operatorRef       : string (institutional identifier of the
                       producing operator)
assetClass        : enum  ("volumetric-video" | "scene-3d" |
                       "point-cloud" | "spherical-360-video" |
                       "spatial-audio-only" | "mr-anchor-set")
intendedExperience : enum ("seated" | "standing-stationary" |
                       "room-scale" | "passenger-vehicle" |
                       "open-public-space")
assetStatus       : enum ("draft" | "ready-for-delivery" |
                       "embargoed" | "withdrawn" | "deprecated")
```

## §3 Scene Record

```
scene:
  sceneId         : string (uuidv7)
  assetId         : string (uuidv7)
  encoding        : enum ("gltf-2" | "usd" | "vrm" |
                       "mpeg-i-v3c" | "mpeg-i-mesh" |
                       "user-defined")
  artefactRef     : string (content-addressed URI of the scene
                       file)
  contentDigest   : string (SHA-256)
  worldUnitsMeters : number (scale factor from scene units to
                       physical metres)
  axisConvention  : enum ("y-up-rh" | "z-up-rh" | "y-up-lh")
  textureCompression : enum ("none" | "ktx2-uastc" |
                       "ktx2-etc1s" | "basis-universal" |
                       "user-defined")
  lodLevels       : integer (level-of-detail tier count)
```

Scene files MUST conform to the cited encoding's specification;
ingestion that fails parsing returns a Problem-Details response of
type `urn:wia:immersive-media:scene-parse-failure`.

## §4 Volumetric Video / Point-Cloud Record

```
volumetric:
  recordId        : string (uuidv7)
  assetId         : string (uuidv7)
  modality        : enum ("multi-camera-mvs" |
                       "depth-sensor-fusion" |
                       "lidar-point-cloud" |
                       "photogrammetry-mesh" |
                       "neural-radiance-field" |
                       "gaussian-splatting")
  pointCount      : integer (per-frame for sequences; absent for
                       continuous mesh formats)
  frameRate       : number (frames per second; 0 for static
                       captures)
  durationSeconds : number
  artefactRef     : string (content-addressed URI of the
                       compressed sequence)
  compressionFamily : enum ("draco" | "edgebreaker" |
                       "mpeg-pcc" | "mpeg-vpcc" |
                       "user-defined")
```

## §5 Spatial Audio Record

```
spatialAudio:
  audioId         : string (uuidv7)
  assetId         : string (uuidv7)
  spatialModel    : enum ("ambisonics-1st-order" |
                       "ambisonics-3rd-order" |
                       "object-based-mha" |
                       "channel-based-7-1-4" |
                       "channel-based-9-1-6" |
                       "user-defined")
  artefactRef     : string (content-addressed URI of the encoded
                       audio)
  sampleRateHz    : integer
  bitDepth        : integer
  loudnessLufs    : number (integrated loudness; for mixing-
                       reference comparison across deliveries)
  hrtfPersonalisationHint : enum ("generic" |
                       "anthropometric-input" |
                       "individualised-recorded" |
                       "absent")
```

## §6 Accessibility Description Record

```
accessibility:
  accessibilityId : string (uuidv7)
  assetId         : string (uuidv7)
  locale          : string (BCP 47)
  shortDescription: string (text alternative analogous to alt-text
                       for static images; required for non-
                       decorative experiences)
  longDescriptionRef : string (URI of a long-form scene description
                       narrating salient elements for blind / low-
                       vision users)
  captionsRef     : string (URI of timed captions for any spoken
                       audio in spatial-audio tracks)
  audioDescriptionRef : string (URI of an audio-described
                       narration track)
  comfortNotes    : string (free text describing motion-sickness
                       triggers, photosensitive-flash content)
  reviewState     : enum ("auto-generated" | "human-reviewed" |
                       "human-authored" | "needs-review")
```

## §7 Comfort and Safety Profile

Immersive content carries a comfort and safety profile that
informs consenting consumption and platform-level gating.

```
comfortProfile:
  profileId       : string (uuidv7)
  assetId         : string (uuidv7)
  motionIntensity : enum ("stationary" | "low" | "medium" |
                       "high")
  artificialLocomotion : enum ("none" | "teleport" |
                       "smooth-vehicle" | "smooth-walk" |
                       "rollercoaster")
  flashPhotosensitiveRisk : enum ("none" | "below-WCAG-threshold" |
                       "above-WCAG-threshold")
  ageRecommendation : enum ("all" | "13+" | "16+" | "18+")
  contextWarnings : array of string (e.g. "loud-startle",
                       "sudden-darkness", "depicted-violence")
```

Assets with `flashPhotosensitiveRisk = "above-WCAG-threshold"` are
gated by the platform's photosensitive-content guard and require
explicit user confirmation before playback.

## §8 Mixed-Reality Anchor Record (Optional)

```
mrAnchor:
  anchorId        : string (uuidv7)
  assetId         : string (uuidv7)
  geoLatitude     : number (when the anchor is geospatially bound)
  geoLongitude    : number
  geoAltitude     : number (metres above the ellipsoid, when
                       applicable)
  anchorScopePolygonRef : string (content-addressed URI of the
                       geometric polygon defining the anchor's
                       valid placement)
  refreshIntervalS : integer (recommended client refresh interval
                       for the anchor's transform)
```

## §9 Provenance Attestation Record

C2PA-aligned provenance manifests describe the asset's chain of
edits and the signers responsible for each step. The record
follows the same shape as PHASE-1 §7 of the WIA-images standard
adapted to immersive-media steps (capture, scene-author, voxelise,
publish).

## §10 Rights and Clearance Record

Rights clearance follows the same shape as the analogous WIA-images
record, extended with additional licence codes appropriate to
immersive content (`derivative-mr-anchor-permitted`,
`derivative-vr-experience-only`).

## §11 LOD and Streaming Variants

Immersive scenes are delivered to a wide range of client capability
tiers, from high-end tethered HMDs to standalone mobile-class
devices. The level-of-detail (LOD) and streaming variant record
describes the per-tier renditions that clients select at playback
time.

```
streamingVariant:
  recordId        : string (uuidv7)
  assetId         : string (uuidv7)
  tier            : enum ("hmd-tethered-pc" | "hmd-standalone" |
                       "mobile-arcore-arkit" | "webxr-desktop" |
                       "webxr-mobile" | "user-defined")
  scenePolygonBudget : integer (target maximum polygon count for
                       the tier)
  textureMaxResolution : integer (max edge length in pixels)
  audioObjectBudget   : integer (max simultaneous audio objects)
  artefactRef        : string (content-addressed URI of the tier-
                       specific scene)
  authoringMethod    : enum ("manual-decimation" |
                       "automated-decimation" |
                       "alternative-asset" |
                       "user-defined")
```

Programmes that publish externally cited assets MUST register at
least one streaming variant per supported client tier so that
downstream platforms can serve appropriately scaled assets without
ad-hoc decimation.

## §12 Avatar and Character Variant (Optional)

Scenes that include user-controllable avatars or non-player
characters carry a character-variant record describing the rig,
the animation skeleton, and the licence terms specific to the
character.

```
characterVariant:
  recordId        : string (uuidv7)
  assetId         : string (uuidv7)
  rigConvention   : enum ("vrm-1.0" | "openxr-skeleton" |
                       "user-defined")
  blendshapeProfile : string (recipe identifier)
  expressionMappingRef : string (content-addressed URI mapping
                       facial-expression indices to blendshape
                       weights)
  authorisedDerivativeUse : enum ("avatar-public" |
                       "avatar-licensed-only" |
                       "npc-only" | "operator-internal")
```

Avatar variants intersect rights-clearance (PHASE-1 §10) for
likeness rights when the character is based on a real person; the
character variant cites the relevant clearance.

## §13 Telemetry and Quality-of-Experience Record

HMD platforms emit telemetry summarising the user's playback
experience: dropped frames, motion-to-photon latency, time spent
in each scene, abrupt comfort-related exits. The telemetry record
aggregates these summaries at the asset level so that producers
can refine future content. Individual user identity is never
carried in the telemetry; only opaque playback session tokens.

```
telemetry:
  telemetryId     : string (uuidv7)
  assetId         : string (uuidv7)
  capturedAt      : string (ISO 8601)
  intervalDurationS : integer
  perTier         : array of TierTelemetry

TierTelemetry:
  tier            : enum (matches PHASE-1 §11)
  playbackCount   : integer
  averageDurationS  : number
  abrupExitRate   : number (fraction of sessions ending within
                       the configured comfort-warning window)
  averageDroppedFrameRate : number
  motionToPhotonMsP95 : number
```

## §14 Provenance Verification History

Each asset's C2PA manifest is re-verified periodically; the
verification history is recorded as a content-addressed sequence
so that consumers can audit the trust state of the manifest over
time.

```
provenanceHistory:
  historyId       : string (uuidv7)
  provenanceId    : string (uuidv7)
  events          : array of VerificationEvent

VerificationEvent:
  occurredAt      : string (ISO 8601)
  trustListRevision : string (C2PA trust list revision identifier)
  outcome         : enum ("verified" | "stale-trust" |
                       "signature-invalid" | "manifest-malformed")
  notesRef        : string (URI of any operator-side notes about
                       the outcome)
```

## §15 Withdrawal and Supersession Record

Assets may be withdrawn (rights-clearance lapse, take-down notice,
performer likeness-revocation) or superseded (refined scene,
upgraded tier variant set, corrected accessibility description).
The withdrawal/supersession record links the affected asset to its
successor (when one exists) and records the public notice the
operator issued.

```
withdrawalSupersession:
  recordId        : string (uuidv7)
  affectedAssetId : string (uuidv7)
  kind            : enum ("withdrawn" | "superseded")
  successorAssetId : string (uuidv7; absent for withdrawn without
                       successor)
  reason          : enum ("rights-lapse" | "take-down" |
                       "likeness-revocation" |
                       "scene-revision" | "tier-uplift" |
                       "accessibility-correction" |
                       "user-defined")
  effectiveAt     : string (ISO 8601)
  publicNoticeRef : string (URI of the operator's public notice)
```

## §16 Reduced-Motion Fallback Reference

Comfort profiles with `motionIntensity` of `medium` or `high`
carry a reduced-motion fallback reference: a stationary or
seated-only variant of the experience that clients may switch to
when the user has expressed a reduced-motion preference. The
fallback is published as a sibling asset whose comfort profile
declares `motionIntensity = stationary`; the API enforces the
sibling-pair invariant before publication.

## §17 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every published asset and honour the
content-addressing rules in §3-§10.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-immersive-media
- **Last Updated:** 2026-04-27
