# PHASE 1 — Data Format

> WIHP canonical envelopes: keyboard layout, gesture, sign
> mapping, accessibility binding, prediction context. All
> envelopes are signed with Ed25519 over the canonical JSON form
> (RFC 8785 JCS).

## 1.1 Keyboard layout envelope

The `keyboard_layout` envelope describes a complete on-screen
keyboard layout in a vendor-neutral form so that the same layout
can be deployed across iOS, Android, Linux IBus/Fcitx, and
Windows TSF without per-platform re-authoring.

```
{
  "wihp_version": "1.0.0",
  "type": "keyboard_layout",
  "layout_id": "ULID",
  "name": "Cheonjiin (천지인) v3",
  "language_tag": "ko-KR",
  "script": "Hang",
  "input_method_engine": "wia-jamo-composer",
  "rows": [
    {
      "row_index": 0,
      "keys": [
        {
          "key_id": "k_jamo_aleph",
          "label_primary":   "ㅏ",
          "label_secondary": "ㅑ",
          "tap_emit": "U+1161",
          "long_press_emit": "U+1162",
          "swipe_left_emit":  "U+1163",
          "swipe_right_emit": "U+1164",
          "geometry": { "row_span": 1, "col_span": 1 },
          "accessibility_label": "Vowel a, long press for ya"
        }
      ]
    }
  ],
  "compose_rules_ref": "wia:compose:hangul-jamo-2024",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`label_primary` and `label_secondary` are decoupled from the emit
codepoints because some scripts (Hangul jamo, Indic scripts,
Arabic ligatures) compose multiple jamo/letters into a single
glyph that the key label visually represents but does not emit
directly.

## 1.2 Gesture envelope

```
{
  "wihp_version": "1.0.0",
  "type": "gesture",
  "gesture_id": "ULID",
  "code": "swipe-up-and-right",
  "captured_at": "RFC 3339",
  "device_id": "did:wia:device:...",
  "trajectory_points": [
    { "t_ms": 0,   "x": 120, "y": 480 },
    { "t_ms": 40,  "x": 145, "y": 460 },
    { "t_ms": 80,  "x": 170, "y": 440 },
    { "t_ms": 120, "x": 195, "y": 420 }
  ],
  "duration_ms": 120,
  "pressure_curve": [],
  "azimuth_deg": 35,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The gesture envelope is the lowest-level capture; downstream
recognition turns the trajectory into a higher-level
`recognised_gesture` event referenced by the layout's tap/swipe
emit codes.

## 1.3 Sign mapping envelope

The `sign_mapping` envelope binds a sign in a sign language to a
text emit in a target language. The standard does not redefine
sign languages; it provides the mapping format so that a
recognised KSL (Korean Sign Language) sign can drive Korean text
input without a vendor-specific glue layer.

```
{
  "wihp_version": "1.0.0",
  "type": "sign_mapping",
  "mapping_id": "ULID",
  "sign_language": "KSL" | "ASL" | "ISL" | "JSL" | "BSL"
                | "DGS" | "FSL" | "...",
  "target_text_language": "ko-KR" | "en-US" | "...",
  "entries": [
    {
      "sign_id": "ksl-hello-001",
      "sign_label": "안녕하세요",
      "emit_text": "안녕하세요",
      "emit_codepoints": ["U+C548", "U+B155", "..."],
      "context_hint": "greeting"
    }
  ],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## 1.4 Accessibility binding envelope

The `accessibility_binding` envelope ties a WIHP layout or gesture
set to an accessibility framework (WAI-ARIA on the web, ATK on
Linux, AccessibilityService on Android, AXUIElement on macOS).

```
{
  "wihp_version": "1.0.0",
  "type": "accessibility_binding",
  "binding_id": "ULID",
  "input_kind": "switch" | "eye_tracker" | "sip_and_puff"
              | "head_pointer" | "voice" | "touch"
              | "physical_keyboard" | "sign_language",
  "wcag_baseline": "2.2-AA" | "2.2-AAA",
  "host_framework": "wai-aria" | "atk" | "android-a11y"
                  | "ax-ui-element" | "windows-ui-automation",
  "scan_rate_ms": 600,
  "dwell_threshold_ms": 800,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`scan_rate_ms` and `dwell_threshold_ms` are mandatory for switch
and eye-tracker inputs respectively; misconfigured values are the
single largest source of usability complaints from users of these
input modalities.

## 1.5 Prediction context envelope

```
{
  "wihp_version": "1.0.0",
  "type": "prediction_context",
  "context_id": "ULID",
  "user_id": "did:wia:user:...",
  "language_tag": "ko-KR",
  "recent_text_window": "<last N words, locally>",
  "domain_hint": "general" | "medical" | "legal"
              | "code" | "messaging",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The prediction context is published only to the local on-device
prediction engine; it is never federated. The standard records
the structure so that on-device prediction implementations can
be validated for compliance with the no-federation rule.

## 1.6 Input session envelope

The `input_session` envelope brackets a contiguous user input
session and is signed by the device at session end. It carries
the layout used, the gesture/key event count, and the
prediction-context identifier.

```
{
  "wihp_version": "1.0.0",
  "type": "input_session",
  "session_id": "ULID",
  "user_id": "did:wia:user:...",
  "device_id": "did:wia:device:...",
  "started_at": "RFC 3339",
  "ended_at":   "RFC 3339",
  "layout_id":  "ULID",
  "event_count": 0,
  "characters_emitted": 0,
  "errors_corrected": 0,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`errors_corrected` is the count of backspace events plus
auto-correct rejections; it is the single most useful metric for
evaluating layout efficacy across users without exposing the
content of what was typed.

## 1.7 Identifiers

- `layout_id`, `gesture_id`, `mapping_id`, `binding_id`,
  `context_id`, `session_id` are ULIDs.
- `user_id`, `device_id` are DIDs (W3C DID Core 1.0).
- `language_tag` uses BCP 47.
- `script` uses ISO 15924 (e.g., Hang for Hangul, Latn for Latin).

## 1.8 References

- W3C DID Core 1.0
- BCP 47 — language tags
- ISO 15924 — script codes
- Unicode 15.1 + UAX #29 (text segmentation) + UAX #14 (line
  breaking)
- Unicode CLDR — keyboard layout interchange
- WCAG 2.2 — accessibility baseline
- WAI-ARIA 1.3 — accessible rich internet applications
- ATK / AT-SPI — Linux accessibility
- W3C Web Speech API — speech-to-text fallback

## 1.9 Privacy posture

The standard's privacy posture is deliberately conservative.
Prediction contexts are device-local; gesture and input-session
envelopes are signed by the device but NOT forwarded outside the
user's tenant by default. Users who opt in to cross-device
gesture recognition (for example, to share a custom gesture set
across their phone and desktop) explicitly grant the federation
scope; the default scope is the user's own tenant only.

弘益人間 — Benefit All Humanity. Input belongs to the user; the
envelopes record what the user typed, not who watched.

## 1.10 Hangul jamo composition rules

Hangul presents a particular challenge for text input: a single
on-screen syllable block represents the composition of two or
three jamo (initial consonant, medial vowel, optional final
consonant). The standard does not prescribe a single composition
algorithm; multiple composers have legitimate use cases (천지인
3-jamo composer, 나랏글 vowel-cluster composer, full
jamo-by-jamo composer for accessibility).

Composition rules are referenced by identifier in
`keyboard_layout.compose_rules_ref`; the rules themselves are
published at `https://wiastandards.com/wihp/compose-rules/`
under stable identifiers. Vendors may publish proprietary
composers using the same field with a vendor-specific
identifier; downstream tooling can detect proprietary composers
and warn users when no neutral fallback is available.

## 1.11 Worked example: switch-input layout

A user with motor impairment uses a single switch (a sip-and-puff
device or a head-button) to scan and select keys on an on-screen
keyboard. The corresponding `accessibility_binding` envelope sets
`input_kind: switch`, `scan_rate_ms: 800`, and references a
two-row layout optimised for switch scanning. The layout's
`keys[].accessibility_label` field provides the text-to-speech
announcement that drives the user's understanding of the
currently focused key.

## 1.12 Audit and tamper evidence

Input session envelopes are signed by the device; the device's
key is registered in WIA-OMNI-API. The session signature does
not cover the typed content (which never leaves the device); it
covers only the metadata (event count, error count, layout used).
This preserves user privacy while still allowing layout designers
to receive aggregate efficacy data from consenting users.

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
