# WIA Screen Reader — Phase 1: Data Format

**Standard**: WIA Screen Reader (Universal Accessibility for 211 Languages)
**Phase**: 1 of 4 — Data Format
**Version**: 1.0.0
**Status**: Draft
**Philosophy**: 弘益人間 — Benefit All Humanity

---

## 1. Scope

Phase 1 fixes the JSON shapes that every WIA Screen Reader implementation
MUST emit and consume so that NVDA, VoiceOver, TalkBack, Orca, and browser
extensions can exchange pronunciation, navigation, and accessibility-tree
data interchangeably across 211 languages.

| Object family | Purpose |
|---------------|---------|
| **Pronunciation Hint** | Per-token IPA + WIHP override for one or more languages |
| **Accessibility Tree Node** | One node in the page accessibility tree (mirrors W3C ARIA roles) |
| **Reading Order Plan** | Ordered traversal across nodes for screen-reader playback |
| **Braille Mapping** | Grade 1 / Grade 2 / WIA Braille translation for a string |
| **User Profile** | Reader's preferred language, voice, speech rate, braille grade |
| **Telemetry Frame** | Anonymous engagement metric (skip rate, heading jump, search use) |

Out of scope: HTTP surface (Phase 2), federation across reader vendors
(Phase 3), platform bridges (Phase 4).

---

## 2. Encoding Rules

* UTF-8 JSON per IETF RFC 8259, `snake_case` keys.
* Language tags follow IETF RFC 5646 / BCP 47 (e.g. `ko-KR`, `en-US`,
  `zh-Hant-HK`).
* Phonetic transcription uses IPA characters from Unicode block
  U+0250..U+02AF; ambiguous glides MUST use the explicit IPA tie-bar
  (U+0361).
* Braille code points use Unicode block U+2800..U+28FF (Braille Patterns).
* WIA Braille extensions (the proprietary 8-dot extension defined by this
  standard) use private-use Unicode block U+E000..U+E07F to avoid
  conflicting with future Unicode allocations.
* Identifiers URI-shaped per IETF RFC 3986. User identifiers SHOULD use
  `did:wia:reader:…` to keep biometrics out of the wire.

### 2.1 Versioning

```json
"wia_screen_reader_version": "1.0.0"
```

A receiver MUST refuse a major version it does not implement and MUST
treat unknown minor-version optional fields as non-fatal.

---

## 3. Pronunciation Hint

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "pronunciation_hint",
  "hint_id": "ph_01HXY",
  "language": "ko-KR",
  "token": "WIA",
  "ipa": "ˈwiː.ɑ",
  "wihp": "위아",
  "stress_pattern": "PRIMARY",
  "applies_to_role": "abbr",
  "issued_at": "2026-04-27T10:00:00Z",
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

### 3.1 Required fields

`wia_screen_reader_version`, `type`, `language`, `token`, and either
`ipa` or `wihp` (at least one MUST be present).

### 3.2 WIHP (WIA Hangeul Phonetic) Notation

WIHP renders any of the 211 supported source languages as Korean
phonetic Hangeul. The transformation rules are deterministic and
reversible per language; the canonical mapping table is published at
`https://wiastandards.com/wihp/mapping-tables/`. Implementations
SHOULD prefer `wihp` for Korean readers and `ipa` for international
voice engines.

### 3.3 Stress Patterns

`PRIMARY`, `SECONDARY`, `NONE`. Multi-syllable hints MAY carry an
explicit stress vector indexed by syllable instead of a single pattern.

---

## 4. Accessibility Tree Node

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "a11y_node",
  "node_id": "n_01HXY",
  "role": "button",
  "name": "Submit form",
  "name_source": "aria-label",
  "value": null,
  "state": ["focusable", "enabled"],
  "children": ["n_01HXZ"],
  "live_region": null,
  "language": "en-US",
  "wia_pronunciation_hint_id": null
}
```

### 4.1 Roles

The `role` field MUST be one of the W3C ARIA 1.2 roles or one of the
WIA-Screen-Reader extension roles registered at
`https://wiastandards.com/screen-reader/roles/`. Implementations MUST
treat unknown roles as `generic` rather than fail the node.

### 4.2 Name Source

`aria-label`, `aria-labelledby`, `inner-text`, `title`, `placeholder`,
`alt`, `caption`, or `wia-name-source-extension`. The reader MUST log
the source so that the user can trace why a given name was chosen.

### 4.3 Live Region

If `live_region` is non-null it MUST be one of `polite`, `assertive`,
`off`. WIA Screen Reader extends ARIA's polite/assertive vocabulary
with a third value `gentle`, intended for chat/IM clients where a
"polite" interrupt is still too loud.

---

## 5. Reading Order Plan

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "reading_order",
  "plan_id": "plan_01HXY",
  "page_url": "https://example.com/article",
  "language": "en-US",
  "ordering": "dom-with-skips",
  "skip_regions": ["#site-navigation", "#footer"],
  "preferred_landmark_path": ["main", "article"],
  "nodes": ["n_01HXY", "n_01HXZ", "n_01HX0", "n_01HX1"],
  "issued_at": "2026-04-27T10:00:00Z",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

`ordering` is one of `dom-with-skips`, `landmark-first`, `heading-first`,
`focus-order`. Implementations MAY add new orderings via the WIA
Screen-Reader extensions process; future minor versions add orderings
but never remove them.

---

## 6. Braille Mapping

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "braille_mapping",
  "mapping_id": "br_01HXY",
  "source_language": "ko-KR",
  "source_text": "안녕하세요",
  "grade": "wia",
  "braille": "⠁⠉⠉⠝⠓⠁⠎⠑⠽⠕",
  "fallback_grade1": "⠁⠝⠝⠽⠑⠕⠝⠛⠓⠁⠎⠑⠽⠕"
}
```

`grade` is one of `g1` (Grade 1, uncontracted), `g2` (Grade 2,
contracted), `wia` (8-dot WIA extension). Readers MUST advertise their
supported grades in the user profile (§7).

---

## 7. User Profile

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "user_profile",
  "user_id": "did:wia:reader:01HXY",
  "preferred_language": "ko-KR",
  "fallback_languages": ["en-US"],
  "voice_id": "wia-jiyoung",
  "speech_rate_wpm": 220,
  "braille_grade": "g2",
  "verbosity": "minimal",
  "skip_punctuation": true,
  "indent_announcement": "level-only",
  "signature": { "alg": "Ed25519", "value": "Hd9w…" }
}
```

### 7.1 Verbosity

`minimal` (role + name only), `standard` (role + name + state),
`detailed` (above plus position-in-set / set-size / level), `verbose`
(everything including parent-chain). The reader MUST honour the user's
choice without surfacing extra fields.

---

## 8. Telemetry Frame

Anonymous engagement metrics. The frame MUST NOT contain the page URL,
the user identifier, or the page content; only the action class and
duration are recorded.

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "telemetry",
  "frame_id": "tm_01HXY",
  "captured_at": "2026-04-27T10:00:00Z",
  "action": "heading_jump",
  "context_role": "main",
  "duration_ms": 45,
  "interaction_count": 1
}
```

`action` is one of `read_aloud`, `heading_jump`, `landmark_jump`,
`form_jump`, `search`, `skip`, `bookmark`, `repeat`, `slow_down`,
`speed_up`, `mute`. Implementations MUST refuse telemetry with any
field outside this enum.

---

## 9. Schema Files

JSON Schema 2020-12 documents are served from
`https://wiastandards.com/screen-reader/schemas/`. Implementations
SHOULD bundle local copies for offline validation.

---

## 10. Conformance

A Phase 1 conformant implementation MUST:

1. Round-trip every object family byte-identically through encode/decode.
2. Reject objects missing required fields per the JSON Schemas.
3. Treat unknown optional fields as non-fatal.
4. Honour the user-profile verbosity setting strictly.
5. Refuse telemetry frames whose action token is outside the enum (§8).

---

## 11. References

* IETF RFC 8259 — JSON
* IETF RFC 5646 — BCP 47 language tags
* IETF RFC 3986 — URI Generic Syntax
* W3C WAI-ARIA 1.2 — accessibility roles
* W3C WCAG 2.2 — accessibility guidelines
* Unicode 15.1 — IPA Extensions block (U+0250..U+02AF), Braille
  Patterns block (U+2800..U+28FF)
* JSON Schema Draft 2020-12

---

## Appendix A — Reserved Tokens

| Field | Reserved tokens |
|-------|-----------------|
| `verbosity` | `minimal`, `standard`, `detailed`, `verbose` |
| `live_region` | `polite`, `assertive`, `off`, `gentle` |
| `ordering` | `dom-with-skips`, `landmark-first`, `heading-first`, `focus-order` |
| `grade` (braille) | `g1`, `g2`, `wia` |
| `name_source` | `aria-label`, `aria-labelledby`, `inner-text`, `title`, `placeholder`, `alt`, `caption`, `wia-name-source-extension` |
| `action` (telemetry) | see §8 |

Future minor versions add tokens but never remove them.

## Appendix B — WIHP Worked Example

The English token "WIA" pronounced for a Korean reader:

| Source | Output | Note |
|--------|--------|------|
| `WIA`  | `위아` | Two-syllable WIHP |
| IPA    | `ˈwiː.ɑ` | Primary stress on first syllable |

A Mandarin reader of the same token gets:

| Source | Output |
|--------|--------|
| `WIA`  | `维亚` (wéi yà) |

The standard does not require translation — it requires faithful
phonetic transcription. The mapping table at
`https://wiastandards.com/wihp/mapping-tables/` is the source of truth
for every (source language, target script) pair.

## Appendix C — Worked Pronunciation Hint with IPA + WIHP

A medical-terminology curator publishing a hint for the drug name
"Apixaban" for Korean readers:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "pronunciation_hint",
  "hint_id": "ph_kma_apixaban",
  "language": "ko-KR",
  "token": "Apixaban",
  "ipa": "ə.ˈpɪk.sə.bæn",
  "wihp": "아픽사반",
  "stress_pattern": ["NONE", "PRIMARY", "NONE", "NONE"],
  "applies_to_role": "term",
  "issued_at": "2026-04-27T10:00:00Z",
  "curator_id": "did:wia:curator:kma-medlex",
  "signature": { "alg": "Ed25519", "value": "AY3o…" }
}
```

When the reader encounters "Apixaban" in a clinical decision-support
page, the cached hint applies; the screen reader renders "아픽사반"
to a Korean voice engine instead of letting the rule-based engine
guess at the unfamiliar English token.

## Appendix D — Worked Accessibility Tree Node

A submit button inside a form, with WIA-extension state attributes:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "a11y_node",
  "node_id": "n_submit_form",
  "role": "button",
  "name": "Submit form",
  "name_source": "aria-label",
  "value": null,
  "state": ["focusable", "enabled", "default"],
  "children": [],
  "live_region": null,
  "language": "en-US",
  "wia_pronunciation_hint_id": null,
  "shortcut": "Alt+S",
  "position_in_set": 3,
  "set_size": 5,
  "level": null,
  "expanded": null
}
```

The reader announces this as "Submit form, button, default, three of
five, Alt+S" when the user's verbosity is `detailed`. With
`verbosity=minimal` the reader announces only "Submit form, button".

## Appendix E — Worked Reading Order Plan with Skips

A user has marked their site navigation and footer as
always-skip via the reader UI:

```json
{
  "wia_screen_reader_version": "1.0.0",
  "type": "reading_order",
  "plan_id": "plan_user_01HXY",
  "page_url": "https://example.com/article",
  "language": "en-US",
  "ordering": "landmark-first",
  "skip_regions": ["#site-navigation", "#footer", ".cookie-banner"],
  "preferred_landmark_path": ["main", "article"],
  "nodes": ["n_article_title", "n_article_byline", "n_article_body", "n_article_comments"],
  "issued_at": "2026-04-27T10:00:00Z",
  "signature": { "alg": "Ed25519", "value": "Bz9A…" }
}
```

The reader applies this plan automatically on subsequent visits to
the same page — the host stores the plan keyed by salted
`page_url_hash` (Phase 2 §5) and returns it on demand.

## Appendix F — Reading-Order Algorithm Reference

When a page has no user-published reading-order plan, the reader
synthesises one from the page's accessibility tree. The default
algorithm is `dom-with-skips` and proceeds as follows:

1. Build the tree from the platform accessibility API
   (NSAccessibility on macOS, MSAA / UIA on Windows, AT-SPI on Linux,
   AccessibilityNodeInfo on Android, the DOM with ARIA awareness
   in browsers).
2. For each top-level landmark (`banner`, `navigation`, `main`,
   `complementary`, `contentinfo`), record its starting node.
3. Apply user skip-region preferences from the user profile or
   reading-order plan if present.
4. Walk the tree in document order, emitting one node id per node
   encountered, depth-first.
5. Return the resulting `nodes` array.

Implementations MAY swap in `landmark-first` or `heading-first`
algorithms when the user's profile prefers them. The chosen
algorithm MUST be deterministic — the same tree input MUST always
produce the same `nodes` output for a given user profile.

弘益人間 — Benefit All Humanity.
