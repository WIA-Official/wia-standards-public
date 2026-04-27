# WIA Screen Reader — Phase 4: Integration

**Standard**: WIA Screen Reader
**Phase**: 4 of 4 — Integration
**Version**: 1.0.0
**Status**: Draft

---

## 1. Scope

Phase 4 specifies how WIA Screen Reader integrates with the four
existing screen-reader platforms (NVDA, VoiceOver, TalkBack, Orca),
the major browsers (Chrome, Firefox, Edge, Safari), with neighbouring
WIA standards, and with the existing accessibility ecosystem (WAI-ARIA,
WCAG, EPUB Accessibility).

---

## 2. NVDA Integration

NVDA (Windows) integrates via an addon distributed through the WIA
Screen Reader plugin channel. The addon:

* Hooks NVDA's speech and braille subsystems.
* Translates NVDA's internal accessibility tree into the WIA
  `a11y_node` shape (Phase 1 §4) for federated federation.
* Subscribes to pronunciation hint streams in the user's languages.
* Honours the user's WIA profile when NVDA's local profile permits.

Conformance test: the NVDA addon MUST round-trip a representative
accessibility tree through encode → emit → fetch → decode without
information loss.

---

## 3. VoiceOver Integration (macOS / iOS)

VoiceOver does not expose a public addon API; integration rides as a
separate accessibility daemon launched at user login. The daemon:

* Mirrors VoiceOver's AXAPI tree into the WIA `a11y_node` shape.
* Provides pronunciation hints via VoiceOver's pronunciation
  dictionary mechanism.
* Federates the user's WIA profile (rate, voice, braille grade) at
  login so that VoiceOver's local settings reflect the portable
  profile.

The daemon is open-sourced under the WIA standard so that Apple's
restrictions on third-party speech engines do not block community
hint sharing — the speech itself stays in VoiceOver, only the
pronunciation hints flow.

---

## 4. TalkBack Integration (Android)

TalkBack integration uses Android's `AccessibilityService` API. The
WIA Screen Reader app subscribes to accessibility events, translates
them into WIA envelopes, and applies hints from the user's federated
hint subscriptions.

For Android-specific contexts (notifications, in-app dialogs), the
app maps Android's `AccessibilityNodeInfo` fields onto the WIA
`a11y_node` shape so that the same pronunciation and braille rules
apply across desktop and mobile.

---

## 5. Orca Integration (Linux)

Orca (the GNOME screen reader) integrates via a Python plugin that
hooks Orca's `gi.repository.Atspi` accessibility bridge. The plugin:

* Translates AT-SPI events into WIA envelopes.
* Bridges the user's WIA profile into Orca's local settings.
* Provides braille translation through Orca's existing BRLTTY backend
  for the `g1` and `g2` grades, and through WIA's own translator for
  the `wia` 8-dot grade.

---

## 6. Browser Extension Integration

Browser extensions (Chrome, Firefox, Edge) implement the WIA Screen
Reader protocol through W3C WebExtensions message-passing. The
extension:

* Reads the page's accessibility tree via the browser's accessibility
  API or a content script that walks the DOM with ARIA awareness.
* Submits the tree to the user's host as a Phase 2 §5 POST.
* Receives pronunciation hints inline and overlays them on the
  page's text-to-speech rendering.
* Renders braille output through the user's connected braille display
  via the browser's `navigator.usb` or the platform's native braille
  driver.

Safari support is partial: Safari's WebExtensions API does not yet
expose the accessibility tree to extensions, so Safari users get the
hint-only flow without tree federation.

---

## 7. WIA Family Integration

### 7.1 WIHP (WIA Hangeul Phonetic)

The WIHP system is a sister WIA standard that defines how 211 source
languages are transcribed into Korean phonetic Hangeul. WIA Screen
Reader is the largest consumer of WIHP — every pronunciation hint
that targets a Korean reader rides WIHP's transcription rules. The
canonical mapping table at
`https://wiastandards.com/wihp/mapping-tables/` is the source of truth.

### 7.2 WIA Braille

The WIA Braille standard defines the 8-dot extension to Unicode
Braille Patterns. WIA Screen Reader implementations honour the
extension when the user's profile declares `braille_grade=wia`.

### 7.3 WIA-OMNI-API

User identity and credentials (preferred display reader, paired
braille hardware, language certifications) live in WIA-OMNI-API.
Hosts fetch them by DID rather than holding raw documents.

### 7.4 WIA-ACCESSIBILITY

The WIA-ACCESSIBILITY profile catalogue describes per-user
accommodations beyond just screen reading (motor, cognitive,
seizure-safety). WIA Screen Reader implementations honour the
linked accommodations profile when rendering page content.

### 7.5 WIA-SOCIAL

When a user shares a pronunciation hint they curated to friends,
the share rides via WIA-SOCIAL bridges with `audience=friends`.
The hint envelope is signed and stored at the WIA Screen Reader
hint host; the WIA-SOCIAL post carries only a pointer.

---

## 8. EPUB Accessibility Integration

EPUB 3 publications declare their accessibility metadata in
`metadata/`. WIA Screen Reader implementations MUST:

* Read the `schema:accessibilityFeature` and
  `schema:accessibilityHazard` fields and surface them to the user
  before opening the publication.
* Honour the publication's `epub:type` semantics (chapter, footnote,
  cover, glossary) when computing reading order.
* Use the publication's pronunciation lexicon (PLS file) as a
  high-priority hint source, ranked above community hints for the
  duration of that reading session.

---

## 9. Migration Paths

### 9.1 From Vendor-Locked Screen Reader

A user moving from a vendor-locked screen reader (the desktop NVDA
without the WIA addon, or an OS-bundled VoiceOver without the
daemon) follows:

1. Install the WIA addon / daemon for their platform.
2. The installer detects existing pronunciation lexicons and offers
   to import them as a private hint set under the user's identity.
3. The user's existing voice / rate / braille settings become the
   initial WIA profile.
4. The user enrols their device with their WIA-OMNI-API identity so
   that the profile becomes portable.

### 9.2 Between Hosts

A user moving from one WIA Screen Reader host to another follows
Phase 3 §5 with a `profile_move` envelope.

---

## 10. Observability

A conformant host SHOULD expose:

| Metric | Type | Description |
|--------|------|-------------|
| `wia_sr_pronunciation_hits_total{language}` | counter | hint-cache hits per language |
| `wia_sr_braille_translations_total{grade}` | counter | per-grade translations |
| `wia_sr_profile_moves_total{outcome}` | counter | profile portability events |
| `wia_sr_telemetry_consent_violations_total` | counter | refused submissions |

Labels MUST NOT include user identifiers.

---

## 11. Conformance Profiles

| Level | Required integrations |
|-------|-----------------------|
| **Minimal** | At least one platform addon (NVDA, VoiceOver, TalkBack, or Orca), pronunciation hint subscription |
| **Core**    | Plus browser-extension binding for ≥1 browser, WIHP for Korean readers, WIA-OMNI-API |
| **Full**    | Plus WIA Braille `wia` grade support, WIA-ACCESSIBILITY profile honouring, EPUB 3 accessibility metadata |

Hosts publish their level in `bridge_profile` of the discovery
document.

---

## 12. Worked Example — Cross-Device Reader

```
User      : did:wia:reader:01HXY
Devices   : (work) NVDA on Windows, (home) VoiceOver on iOS, (mobile) TalkBack
Hint host : did:wia:hint-host:nvda-cloud
Profile host : did:wia:profile-host:wia-omni
```

1. User installs the WIA addon / daemon on each device.
2. Each install fetches the user's profile from `wia-omni` via
   WIA-OMNI-API; speech rate, voice, braille grade are identical
   across devices.
3. User subscribes to the Korean medical terminology curator on the
   work NVDA; the subscription propagates to the iOS VoiceOver and
   Android TalkBack via the user's profile.
4. User reads a clinical PDF on the work NVDA at 9 am; a custom
   pronunciation hint for "Apixaban" applies. At 8 pm the user
   re-reads on iOS VoiceOver; the same hint applies.
5. User publishes a new hint they curated themselves; the publish
   rides via WIA-SOCIAL with `audience=friends` to their accessibility
   community.

---

## 13. Security Considerations

* Pronunciation hint streams could be misused to inject misleading
  speech; readers MUST verify each hint's curator signature and MUST
  fall back to rule-based pronunciation on verification failure.
* Browser extensions hold powerful page-content access; extensions
  MUST request only the minimum WebExtensions permissions needed for
  the WIA Screen Reader scope and MUST NOT use the access to harvest
  page content.
* Telemetry is anonymous by design but the aggregator could correlate
  IP addresses unless it strips them at ingestion. Hosts MUST strip
  IP addresses from telemetry payloads before storage.
* Braille displays connected over USB / Bluetooth carry sensitive
  reading content; drivers MUST honour the OS-level USB / Bluetooth
  permission models and MUST NOT log braille output to disk by default.

---

## 14. References

* W3C WAI-ARIA 1.2
* W3C WCAG 2.2
* EPUB 3 Accessibility 1.1
* W3C WebExtensions
* IETF RFC 8446 — TLS 1.3
* IETF RFC 9421 — HTTP Message Signatures
* WIA-WIHP standard
* WIA Braille standard
* WIA-OMNI-API standard
* WIA-ACCESSIBILITY standard
* WIA-SOCIAL standard

---

## Appendix A — Bridge Configuration File

A reference NVDA addon configuration uses INI (NVDA's native format):

```ini
[wia-screen-reader]
host_id = did:wia:reader-host:nvda-cloud
profile_endpoint = https://reader.example/sr/profile
hint_endpoints =
    https://reader.example/sr/pronunciation
    https://other-host.example/sr/pronunciation
braille_endpoint = https://reader.example/sr/braille
telemetry_endpoint = https://reader.example/sr/telemetry
telemetry_consent_required = true

[curators]
default_subscriptions =
    did:wia:curator:nvda-en-tech
    did:wia:curator:kma-medlex

[performance]
hint_cache_max_mb = 100
live_region_polite_budget_ms = 250
live_region_assertive_budget_ms = 100
live_region_gentle_budget_ms = 500
```

A browser-extension binding uses JSON (WebExtensions native):

```json
{
  "wia_screen_reader": {
    "version": "1.0.0",
    "host_id": "did:wia:reader-host:nvda-cloud",
    "permissions_required": ["storage", "activeTab", "wia-screen-reader"],
    "telemetry_consent_required": true,
    "default_languages": ["ko-KR", "en-US"]
  }
}
```

## Appendix B — Conformance Test Coverage by Profile

| Capability | Minimal | Core | Full |
|------------|---------|------|------|
| At least one platform addon | ✓ | ✓ | ✓ |
| Pronunciation hint subscription | ✓ | ✓ | ✓ |
| Browser extension binding (≥1 browser) | — | ✓ | ✓ |
| WIHP transcription for Korean | — | ✓ | ✓ |
| WIA-OMNI-API credential fetch | — | ✓ | ✓ |
| WIA Braille `wia` 8-dot grade | — | — | ✓ |
| WIA-ACCESSIBILITY profile honouring | — | — | ✓ |
| EPUB 3 Accessibility integration | — | — | ✓ |
| Telemetry aggregator with consent | optional | optional | ✓ |
| Curator publishing UI | optional | ✓ | ✓ |

A host publishing `bridge_profile=Full` MUST pass every line in the
Full column and SHOULD pass at least one optional line.

## Appendix C — Worked Migration Trace

A user moving from the bare NVDA (no WIA addon) to the WIA-enabled
NVDA experience:

```
T+0    User installs the WIA addon from the NVDA addon store.
T+30s  Addon detects existing NVDA pronunciation lexicon (12 entries).
T+45s  Addon prompts user to opt in to importing the lexicon as a
       private hint set. User accepts.
T+1m   Addon enrols user with WIA-OMNI-API identity (existing WIA SSO).
T+1m   Addon fetches existing WIA profile (none exists; uses NVDA
       defaults to seed: rate=220 wpm, voice=Eloquence, braille=g2).
T+2m   Addon prompts user to subscribe to default Korean curator.
       User accepts; hint cache populates with 5,000 entries over 30s.
T+5m   User opens a clinical PDF. The drug name "Apixaban" is hinted
       by the curator; NVDA renders "아픽사반" instead of guessing.
T+1d   User installs WIA daemon on home iOS VoiceOver.
T+1d   Daemon fetches profile from WIA-OMNI-API; identical settings
       apply on iOS without any manual configuration.
```

No data leaves the user's device unless an explicit telemetry consent
envelope is published. The 12 pronunciation lexicon entries imported
at T+45s become a private hint set associated with the user's identity;
the user can later choose to publish them as a public curator under
their own DID, but the standard never auto-publishes.

弘益人間 — Benefit All Humanity.
