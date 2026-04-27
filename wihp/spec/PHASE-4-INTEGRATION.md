# PHASE 4 — Integration

> Integration with iOS, Android, Linux IBus/Fcitx, Windows TSF,
> ARIA, ATK, eye-tracker SDKs, switch-input devices, and the
> Unicode CLDR keyboard layout repository.

## 4.1 iOS integration

iOS keyboards are implemented as UIInputViewController
extensions. The reference WIHP shim consumes a
`keyboard_layout` envelope and renders the on-screen keyboard via
UIKit, exposing each key as an accessible UIView with the
layout's `accessibility_label` mapped to the iOS Accessibility
Trait system. The shim emits jamo composition events through the
standard textDocumentProxy interface.

## 4.2 Android integration

Android keyboards are implemented as InputMethodService
subclasses. The reference WIHP shim consumes a
`keyboard_layout` envelope and renders the on-screen keyboard via
the Android UI toolkit, exposing each key with
android:contentDescription mapped to
AccessibilityService announcements. Switch-input integration uses
the AccessibilityService FOCUSED event sequence to drive
scan-and-select.

## 4.3 Linux integration (IBus / Fcitx)

IBus 1.5+ and Fcitx 5+ both expose a plugin API for IME
backends. The reference WIHP modules for each consume layout and
sign-mapping envelopes and expose the resulting input method to
GTK and Qt applications via the standard ibus-daemon /
fcitx-daemon protocols.

## 4.4 Windows integration (TSF)

Windows IMEs use the Text Services Framework (TSF). The
reference WIHP TSF module consumes WIHP layouts and integrates
with the Windows on-screen keyboard's accessibility surface.
Switch-input integration uses the Windows UI Automation
framework for scan-and-select.

## 4.5 macOS integration

macOS IMEs use the InputMethodKit framework. The reference WIHP
IMK module consumes WIHP layouts and exposes the resulting input
method via NSTextInputClient. AXUIElement integration provides
the accessibility surface.

## 4.6 Web integration

Web-based input methods consume WIHP layouts via the W3C
Keyboard Map API where available, falling back to keydown
intercept and beforeinput composition events. The reference web
shim is a small JavaScript module that targets WAI-ARIA 1.3
patterns for the on-screen keyboard surface.

## 4.7 Eye-tracker SDK integration

Eye trackers (Tobii Pro, EyeTech VT3, EyeTech Quick Glance) expose
gaze-point streams via vendor-specific SDKs. The reference WIHP
eye-tracker bridge consumes the gaze stream and emits dwell-based
key selection events using the `accessibility_binding`
envelope's `dwell_threshold_ms` field.

## 4.8 Switch-input device integration

Single-switch and dual-switch devices (sip-and-puff, head-button,
finger-button, foot-pedal) connect via Bluetooth HID or wired
USB HID. The reference WIHP switch-input bridge consumes HID
button events and drives the scan-and-select cursor at the
binding's `scan_rate_ms` cadence.

## 4.9 Sign-language recognition integration

Sign-language recognition models (community-trained KSL, ASL,
JSL, BSL recognisers) consume video frames and emit recognised
sign identifiers. The reference WIHP sign-bridge maps recognised
sign identifiers through the active `sign_mapping` envelope to
text emit codepoints, which then flow through the standard IME
text-emission path. The standard does not redistribute sign-
recognition models; it provides the binding so that any
conforming recogniser can drive any conforming IME.

## 4.10 Unicode CLDR integration

The Unicode CLDR keyboard layout repository
(https://cldr.unicode.org/index/keyboard-workgroup) is the
canonical source for many language layouts. The reference WIHP
CLDR bridge consumes CLDR LDML keyboard XML and emits WIHP
`keyboard_layout` envelopes. The bridge runs once per CLDR
release; the resulting envelopes are signed by the bridge
maintainer's tenant key.

## 4.11 WAI-ARIA integration

Web on-screen keyboards expose WAI-ARIA 1.3 patterns for the
on-screen keyboard surface (role="application", role="grid" for
the key matrix, aria-keyshortcuts for keyboard parity). The
reference WIHP web shim implements the patterns on top of the
WIHP layout envelope.

## 4.12 ATK / AT-SPI integration (Linux)

Linux assistive technology (Orca screen reader, Caribou on-
screen keyboard, GNOME accessibility services) consumes WIHP
layouts via the AT-SPI bus. The reference shim exposes each
key as an Atk.Object with role ATK_ROLE_PUSH_BUTTON and the
layout's `accessibility_label` as the ATK name property.

## 4.13 Korean integration notes

For Korean users, the standard's reference layouts include 천지인,
나랏글, and 두벌식 + a fully accessible jamo-by-jamo layout
optimised for switch input. The 의료재활 sector has a particular
need for switch-input optimised Korean keyboards; the standard's
decoupling of label_primary from emit_codepoints accommodates
the long-press-for-secondary-jamo patterns common in Korean
on-screen keyboards.

## 4.14 European integration notes

EU EAA (European Accessibility Act) requires that consumer
devices support assistive input. WIHP's `accessibility_binding`
envelope is the natural format for the EAA-required attestation
of supported assistive input modalities.

## 4.15 US integration notes

US ADA (Americans with Disabilities Act) Title II requires that
public-sector digital services be accessible. WIHP's switch and
eye-tracker bindings provide the cross-platform input layer that
ADA Title II compliance depends on.

## 4.16 WIA family integration

WIHP plugs into the broader WIA family at:

- **WIA-OMNI-API** — device pairing identity
- **WIA Sign Language** — sign-to-text companion standard
- **WIA Braille** — Braille input/output companion standard
- **WIA-AIR-SHIELD** — transport hardening for federation

## 4.17 Worked example: cross-platform switch user

A user with motor impairment uses a single sip-and-puff switch
across an iPhone, an Android tablet, a Linux desktop, and a
Windows work laptop. The flow:

```
1. User authors a custom switch-scanning layout on Linux
   desktop (the device with the most authoring tooling)
2. User signs federation_handshake on Linux to phone/tablet/work
   laptop via WIA-OMNI-API
3. Custom layout, accessibility_binding (scan_rate_ms: 1200,
   single-switch row-column scanning), and trained gesture set
   propagate to all three devices
4. Each device's WIHP shim configures itself per the federated
   bindings on first launch
5. User has a consistent input experience across four
   operating systems with one authoring effort
```

The federation eliminates the largest accessibility tax in current
practice — the cost of re-authoring a custom input method per
device — and makes the user's investment portable across device
upgrades and platform migrations.

## 4.18 Conformance maturity model

- **Bridge-only** — translates CLDR or vendor IME formats into
  WIHP envelopes
- **Native** — authors envelopes natively in the IME
- **Native + Federation** — additionally federates across
  user devices

## 4.19 Operational considerations

Implementations SHOULD provide a "panic switch" that lets the
user disable federation in a single action, in case a paired
device is lost or stolen. The panic switch publishes the
revocation envelopes for every paired device atomically.

Cross-platform shim libraries are versioned per platform release
schedule (annual for iOS/Android/macOS, ad-hoc for Linux/Windows
IME frameworks). Shim maintainers publish a `shim_compatibility`
envelope per release, naming which platform versions are tested
and which WIHP layout features are supported.

## 4.20 Backwards compatibility

Pre-standard IME stacks MAY operate proprietary layouts in
parallel with WIHP envelopes during a transitional window. The
WIHP envelope is canonical for cross-vendor verification; the
proprietary layout remains canonical inside its origin platform.

## 4.21 Reference list

- W3C DID Core 1.0
- W3C Keyboard Map API
- W3C Web Speech API
- WCAG 2.2
- WAI-ARIA 1.3
- ATK / AT-SPI specifications
- iOS UIInputViewController
- Android InputMethodService
- Linux IBus 1.5+, Fcitx 5+
- macOS InputMethodKit
- Windows TSF
- Unicode CLDR LDML keyboard
- ISO 15924 — script codes
- BCP 47 — language tags
- EU European Accessibility Act
- US ADA Title II
- KR 장애인차별금지법

## 4.22 Operational coda

WIHP succeeds when an accessibility-focused user authors a
layout once and uses it across every device they own. The
standard's envelope discipline contributes to that outcome by
making the layout a first-class signed artefact rather than a
device-local configuration setting that disappears on device
upgrade.

弘益人間 — Benefit All Humanity. The integration surface exists so
that the user with the most specific input needs has the most
portable input method.

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
