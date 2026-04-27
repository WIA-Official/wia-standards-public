# WIA Screen Reader — Release Notes

## v1.0.0 (Initial publication)

* **4-Phase specification** complete: Data Format, API Interface,
  Federation Protocol, Integration with NVDA / VoiceOver / TalkBack /
  Orca / browsers.
* **211-language support** for pronunciation hints via WIHP (WIA Hangeul
  Phonetic) and IPA dual-encoding.
* **Three braille grades**: Grade 1, Grade 2, and the WIA 8-dot
  extension (private-use Unicode block U+E000..U+E07F).
* **Conformance test suite** open-sourced at
  `https://github.com/WIA-Official/wia-screen-reader-conformance`.
* **Reference implementations** for NVDA addon, VoiceOver daemon,
  Android TalkBack app, GNOME Orca plugin, and Chrome / Firefox / Edge
  browser extensions.

---

## Roadmap

### v1.1.x (planned)
* Additional curator workflows for community language packs.
* Tighter integration with EPUB 3 Accessibility 1.1 metadata.
* Optional WIA-AIR-SHIELD scoring for pronunciation hint federation.

### v1.2.x (planned)
* Federated identity for curators via WIA-OMNI-API.
* Optional public-registry export profile for community-curated hints.

### v2.0.0 (no earlier than 2028)
* Possible breaking change: post-quantum signature suite migration
  following the parent WIA family roadmap.

---

## Backwards-compatibility promise

Within the 1.x line every Phase 2 endpoint MUST remain reachable and
MUST continue to honour the documented status codes and content shapes.
Hosts MAY add optional query parameters, response fields, new
endpoints, or media types. Hosts MUST NOT remove or repurpose existing
ones. Breaking changes ride a major version bump and MUST be preceded
by a 12-month deprecation window per IETF RFC 8594 and RFC 9745.

---

## Governance

The standard is maintained by the WIA Standards Committee. Change
proposals follow the WIA RFC process: anyone may submit a proposal;
the Committee reviews quarterly; accepted proposals enter an open
comment period before being merged into a minor-version release.
Breaking changes require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.
