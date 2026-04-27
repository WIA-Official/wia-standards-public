# PHASE 2 — API Interface

> HTTP surface for layout publication, gesture stream, federation
> handshake, and accessibility binding query. Requests carry HTTP
> Message Signatures (RFC 9421); errors use Problem Details
> (RFC 9457).

## 2.1 Endpoint surface

```
POST /wihp/v1/layouts                         Publish a keyboard_layout
GET  /wihp/v1/layouts                         Query layouts
GET  /wihp/v1/layouts/{id}                    Fetch layout

POST /wihp/v1/gestures                        Publish a gesture (training)
GET  /wihp/v1/gestures/stream/{device_id}     SSE: live gesture stream

POST /wihp/v1/sign-mappings                   Publish a sign_mapping
GET  /wihp/v1/sign-mappings?lang=KSL          Query mappings

POST /wihp/v1/accessibility-bindings          Publish a binding
GET  /wihp/v1/accessibility-bindings?kind=... Query bindings

POST /wihp/v1/sessions                        Open input_session
POST /wihp/v1/sessions/{id}/end               Close input_session

GET  /.well-known/wihp                        Capability advertisement
```

## 2.2 Authentication

All POST requests carry HTTP Message Signatures (RFC 9421) over
the mandatory `(request-target)`, `@authority`, `content-digest`,
`@created`, `@expires` covered components. Device keys are
provisioned during initial pairing via the WIA-OMNI-API trust
fabric. The user's hardware-bound passkey co-signs sensitive
operations (federation handshake, layout publication for shared
team layouts).

## 2.3 Idempotency

POST endpoints accept `Idempotency-Key` headers (UUIDv7
recommended). A repeated request with the same key within 24
hours returns the original response without re-creating the
resource.

## 2.4 Pagination

List endpoints use cursor pagination with `Link` headers
(RFC 8288). Page size is capped at 200 envelopes per page.

## 2.5 Streaming semantics

The gesture stream emits Server-Sent Events for trajectory points
captured by the paired device. Trajectories are streamed in real
time during gesture training; in production use the stream is
local-only and is not exposed beyond the user's device.

## 2.6 Capability advertisement

```
GET /.well-known/wihp

200 OK
{
  "wihp_version": "1.0.0",
  "type": "capability_advertisement",
  "publisher_id": "did:wia:org:...",
  "endpoint_surface_version": "wihp/v1",
  "supported_envelope_types": [
    "keyboard_layout", "gesture", "sign_mapping",
    "accessibility_binding", "prediction_context",
    "input_session"
  ],
  "supported_scripts": ["Hang", "Latn", "Jpan", "Hans", "Hant",
                        "Arab", "Hebr", "Cyrl", "Beng", "Deva"],
  "supported_sign_languages": ["KSL", "ASL", "JSL", "BSL", "ISL"],
  "host_frameworks": ["wai-aria", "atk", "android-a11y",
                      "ax-ui-element", "windows-ui-automation"],
  "rate_limits": { "rpm": 600 },
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 2.7 Error envelope

```
{
  "type": "https://wiastandards.com/wihp/errors/unsupported-script",
  "title": "Layout script not supported by this host",
  "status": 415,
  "detail": "Layout uses script Cans; host supports Hang, Latn, Jpan",
  "layout_id": "01HXR..."
}
```

## 2.8 Versioning

The wire format version is in the envelope; the HTTP surface is
in the URL (`/wihp/v1`). Breaking changes get a new URL prefix.

## 2.9 Rate limits

Default: 600 RPM per device client identity, 100 burst per
second. Gesture stream is exempt from the per-second burst limit
during gesture-training sessions.

## 2.10 Content negotiation

JSON by default; CBOR (RFC 8949) negotiated via
`Accept: application/cbor`. Gesture trajectories in particular
benefit from CBOR encoding for the floating-point trajectory
points; the signature is over the canonical JSON form regardless.

## 2.11 Webhook callbacks

Layout publishers may register callbacks for downstream consumers
(IME platforms, accessibility integrators) to receive new layout
publications asynchronously. Callback delivery uses exponential
backoff (1, 2, 4, 8, 16, 32, 64s, dead-letter at the 8th
attempt).

## 2.12 Bulk export

For accessibility researchers and platform integrators, a bulk
export endpoint returns a signed manifest plus NDJSON envelope
content for a publisher's full layout / sign-mapping inventory.

## 2.13 Health and observability

```
GET /wihp/v1/health    → liveness probe
GET /wihp/v1/ready     → readiness probe
GET /wihp/v1/metrics   → Prometheus exposition
```

## 2.14 Backwards compatibility

Implementations migrating from CLDR keyboard XML or vendor-
specific IME formats MAY operate a translator that consumes the
legacy format and emits WIHP envelopes. The translator is a
publisher in its own right with its own DID-bound signing key.

## 2.15 Operational considerations

Layout-publication traffic is bursty: most days have zero
publications, post-conference days (after WIA Hangul Day, BCP-
adoption events) have hundreds. The standard's rate limits
accommodate the burst pattern.

Gesture-training traffic is high-volume but device-local; it
rarely traverses the public API surface in production. The
training data path is provided primarily for accessibility
researchers and layout designers who need to evaluate gesture
recognition performance across user populations.

## 2.16 Backwards compatibility (continued)

Vendor-specific IME stack integrations (iOS UIInputViewController,
Android InputMethodService, Linux IBus 1.5+, Fcitx 5+, Windows
TSF) consume WIHP layouts via platform-specific shim libraries.
The shim libraries are part of the implementation; the standard
itself is platform-neutral.

## 2.17 Test surface

A read-only test surface is exposed at
`https://sandbox.wiastandards.com/wihp/v1` returning realistic
synthetic layout and gesture traffic. Sandbox traffic is signed
by the WIA test key; signatures will not verify against
production trust roots, by design.

## 2.18 Schema registry

`GET /wihp/v1/schemas/{type}/{version}` returns the JSON Schema
for the named envelope type at the named version. Schemas are
immutable; new versions get new identifiers. Conforming clients
MUST validate against the schema version named in the envelope.

## 2.19 Worked example: layout publication

A layout designer publishes a new Hangul jamo layout:

```
1. POST /wihp/v1/layouts           (signed by designer's tenant key)
2. → 202 Accepted, Location: /wihp/v1/layouts/{id}
3. Layout enters review queue (community + accessibility audit)
4. Reviewers post conformance attestations referencing the layout
5. Once 3 attestations accumulate, the layout becomes
   available for public discovery via GET /wihp/v1/layouts
```

The review queue is operated per-tenant; the standard does not
mandate a single review process. Decentralised review is part of
the protocol's design — multiple downstream consumers can apply
their own review criteria without needing a central authority's
permission.

## 2.20 Operational coda

Adoption is incremental: a small accessibility-focused team can
begin by publishing a single layout and adding sign mappings or
gesture sets later. The standard is intentionally additive.

弘益人間 — Benefit All Humanity. The API exists so that the smallest
input-method team can author a layout that ships across iOS,
Android, Linux, and Windows without re-authoring once.

## Implementer's checklist

For organisations preparing a conforming WIHP implementation,
the following non-exhaustive checklist captures the operational
decisions every implementation has to make explicitly:

1. Identify the host platform (iOS, Android, Linux, Windows,
   macOS, web). Each has its own IME framework with its own
   accessibility surface.
2. Decide whether to author native WIHP layouts or to bridge
   from CLDR / vendor formats. Authoring native is more work
   up front but avoids round-tripping fidelity loss.
3. Select the accessibility framework integration depth.
   WAI-ARIA is the minimum for web; ATK/AT-SPI for Linux;
   AccessibilityService for Android; AXUIElement for macOS;
   UI Automation for Windows.
4. Decide the gesture-recognition pipeline. On-device (privacy-
   preserving, lower model size) versus server-assisted
   (higher model size, lower battery cost). The standard
   permits both but the federation rule prohibits sending raw
   trajectory data to a third party without explicit user
   consent.
5. Plan the federation onboarding. The user's first paired
   device is the canonical authoring device; subsequent
   devices receive federated bindings on first launch.
6. Decide the panic-switch implementation. Lost or stolen
   devices need a single-action revocation that propagates to
   every paired device within the 60-second SLA.
7. Build operational procedures for layout drift. A user who
   re-tunes their layout on one device should propagate the
   re-tune to every other paired device automatically.
8. Decide the localisation surface. WCAG 2.2 requires that the
   on-screen keyboard surface be operable in the user's
   preferred locale; a Korean user with switch input should
   not be required to navigate an English-language scanner.

弘益人間 — Benefit All Humanity. The checklist exists so that
implementers can make the cost of accessibility input
proportional to the value delivered to the users who depend on
it most.

## Operational coda

Adoption of this standard is incremental: an organisation can
begin by publishing a single layout envelope and incrementally
expand to gesture sets, sign mappings, accessibility bindings,
and federation. The standard is intentionally additive. There
is no minimum subset of envelope types that must be emitted to
participate; the only requirement is that the envelopes that
are emitted be properly signed and structurally correct.

The cumulative behaviour of all such organisations is an
input-method commons that no single platform owns and that
users can carry across device upgrades and platform
migrations. That commons is the public-good outcome the
standard exists to enable.

## Low-bandwidth and offline operation

WIHP is designed to operate fully offline once layouts and
gesture sets are provisioned. Layout and accessibility-binding
envelopes are small (typically under 16 KB) and tolerate
aggressive compression on the wire; gesture sets are larger
(tens of KB per trained user) but still well within the
synchronisation budget of even constrained mobile networks.

The federation protocol uses lazy propagation: envelopes are
fetched on demand when the user pairs a new device, not pushed
proactively from authoring devices. This keeps the standing
network footprint near zero in steady state and concentrates
network activity around the rare events (initial pair, layout
re-author) where bandwidth is genuinely needed.

For users in low-bandwidth contexts (rural mobile coverage,
satellite links, refugee camps, school computer labs in
developing regions), the offline-first design ensures that the
input method remains usable even when the network is
intermittent. The user's accessibility profile does not depend
on a live network connection.

弘益人間 — Benefit All Humanity. The user without reliable
connectivity should still own a portable input method.

## Cross-reference to companion standards

WIHP does not stand alone. The companion standards that complete
the user-facing accessibility surface are the WIA Sign Language
standard (which supplies the recognised-sign identifiers consumed
by the WIHP `sign_mapping` envelope), the WIA Braille standard
(which supplies the input/output binding for refreshable Braille
displays), and the WIA Sign-Language Recognition Conformance
attestation (which establishes that a sign-recognition model
meets the per-language accuracy thresholds appropriate for text
input). The standards together constitute the open accessibility
input layer for an open accessibility computing surface; using
WIHP without the companions is permissible but limits the
modalities the user can choose from.

弘益人間 — Benefit All Humanity. The full standard family exists so
that the user with the most specific input needs can compose
their own accessibility surface from interoperable parts.
