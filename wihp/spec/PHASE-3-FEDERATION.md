# PHASE 3 — Federation Protocol

> Cross-platform IME registry, replay defence, accessibility
> binding, and cross-device gesture-set sharing.

## 3.1 Why federate at all

A user with a custom layout — particularly an accessibility-
optimised layout for switch input or eye tracking — typically
spends weeks tuning that layout to their needs. Re-authoring that
layout when the user buys a new phone, switches from iOS to
Android, or pairs a new assistive device is a huge accessibility
penalty. The federation protocol exists so that the user's
layout, gesture set, and accessibility binding follow the user
across devices and platforms under the user's explicit control.

## 3.2 The federation handshake

```
{
  "wihp_version": "1.0.0",
  "type": "federation_handshake",
  "handshake_id": "ULID",
  "initiator": "did:wia:device:...",
  "counterparty": "did:wia:device:...",
  "user_id": "did:wia:user:...",
  "scope": ["layout:read", "sign_mapping:read",
            "accessibility_binding:read", "gesture_set:read"],
  "tlp": "AMBER+STRICT",
  "valid_until": "RFC 3339",
  "signature_by_user": "Ed25519"
}
```

The handshake is signed by the user (not the device) because the
user is the durable owner of the layout / gesture set; the
device is a transient tool.

## 3.3 Replay defence

Federated envelopes carry a 96-bit nonce and are accepted within
±300 seconds wall-clock skew. The receiver maintains a 600-
second replay cache.

## 3.4 Cross-device gesture set sharing

A user who has trained a custom gesture vocabulary on one device
publishes a `gesture_set` envelope referencing the trained
gestures by identifier. The federation handshake makes the
gesture set discoverable on the user's other paired devices; each
device retrains its local recognition engine on the user's
trajectory data without the trajectory data ever leaving the
user's tenant.

## 3.5 Accessibility binding inheritance

A user with switch input on their phone who pairs a new tablet
expects the new tablet to inherit the same `scan_rate_ms` and
`dwell_threshold_ms` settings without re-tuning. The federation
protocol propagates the user's current `accessibility_binding`
envelope to the new device on first pair; the new device may
override locally, but the user's federated binding is the
default.

## 3.6 Per-device revocation

The user revokes a device's federation grant by signing a
revocation envelope. The originating device stops publishing
within 60 seconds; the revoked device stops querying within 60
seconds. Stolen-device scenarios are the primary motivation for
the 60-second SLA — a stolen phone should lose access to the
user's gesture vocabulary as quickly as possible.

## 3.7 IME registry

A community-maintained IME registry catalogues conforming
implementations (iOS WIHP shim, Android WIHP shim, Linux IBus
WIHP module, Fcitx WIHP module, Windows TSF WIHP module). The
registry is itself an envelope stream signed by the maintainer;
consumers verify the maintainer's signature before trusting the
catalogue.

## 3.8 Cross-jurisdictional considerations

Layout and sign-mapping envelopes are typically not personal
data. They MAY include personal data when, for example, a custom
user-authored sign maps to a personal name; in that case the
publisher is responsible for the personal-data handling under
GDPR / KR PIPA / CN PIPL.

## 3.9 Federation telemetry

Daily `federation_telemetry` envelopes summarise envelope
volume, replay-cache hit rate, revocation latency, and device
inventory. The telemetry stream is queryable by the user (not by
device manufacturers) so that the user can audit which devices
have been receiving federated data.

## 3.10 Conformance test corpus

A reference test corpus is maintained at
`https://wiastandards.com/wihp/conformance/` covering valid
federation handshakes, replay defence, revocation latency, and
cross-device gesture-set propagation. Conforming implementations
MUST pass the corpus and publish a `conformance_attestation`
envelope.

## 3.11 Worked example: tablet pairing

A user with a switch-input phone pairs a new tablet. The flow:

```
1. User signs federation_handshake on the phone, scope:
   layout:read accessibility_binding:read gesture_set:read
2. Phone publishes federated envelopes to tablet via the
   user's tenant
3. Tablet receives layout (custom switch-scanning layout),
   accessibility_binding (scan_rate_ms: 800), gesture_set
4. Tablet's IME shim configures itself per the federated
   bindings on first launch
5. User can immediately use the tablet with the same
   accessibility profile they tuned on their phone
```

The user did not re-author anything; the tablet's accessibility
posture matches the phone's automatically.

## 3.12 Operational considerations

Federation introduces additional latency to first-launch on a
new device (the federated envelopes must arrive). Implementations
SHOULD pre-fetch the federated envelopes during pairing rather
than at first launch, so that the user's first IME interaction
on the new device is responsive.

## 3.13 Backwards compatibility

Implementations migrating from cloud-keyboard sync (Google
Gboard, Microsoft SwiftKey) MAY operate parallel envelope and
cloud-sync flows during a transitional window of 18 months.
WIHP envelopes are canonical for verification; cloud-sync
remains canonical for users who have not yet migrated.

## 3.14 Reference list

- W3C DID Core 1.0
- BCP 47 — language tags
- ISO 15924 — script codes
- Unicode CLDR — keyboard layout interchange
- WCAG 2.2 — accessibility baseline
- WAI-ARIA 1.3 — accessible rich internet applications
- ATK / AT-SPI — Linux accessibility
- iOS Accessibility Programming Guide (UIInputViewController)
- Android InputMethodService API
- IBus 1.5+ / Fcitx 5+ — Linux IME frameworks
- Windows TSF — Windows IME framework
- RFC 8785 — JSON Canonicalisation
- RFC 9421 — HTTP Message Signatures

## 3.15 Operational coda

Federation makes the most difference for users with
accessibility-driven custom layouts and for users who travel
across linguistic regions (a Korean speaker working in Japan, a
Deaf user crossing sign-language regions). The standard's
discipline — explicit consent, fast revocation, mutual audit —
makes federation safe to enable without compromising privacy.

弘益人間 — Benefit All Humanity. Federation lets the user own their
input method instead of renting it from a vendor.

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
