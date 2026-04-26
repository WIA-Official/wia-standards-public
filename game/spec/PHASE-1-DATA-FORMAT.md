# WIA Game Accessibility - Phase 1: Data Format Specification

**Version**: 1.0.0
**Status**: Draft
**Last Updated**: 2025-01

---

## 1. Overview

This document defines the standard data formats for gaming accessibility settings, enabling players to maintain consistent accessibility preferences across games and platforms.

### 1.1 Design Principles

- **Portability**: Settings transfer between games and platforms
- **Extensibility**: Support for future accessibility features
- **Compatibility**: Work with existing accessibility hardware
- **Privacy**: Player control over shared data
- **Simplicity**: Easy implementation for developers

---

## 2. Core Data Structures

### 2.1 Player Accessibility Profile

The central data structure containing a player's accessibility preferences.

```json
{
  "profile_id": "uuid",
  "version": "1.0.0",
  "created_at": "ISO8601",
  "updated_at": "ISO8601",
  "player_info": { ... },
  "disability_context": { ... },
  "visual_settings": { ... },
  "audio_settings": { ... },
  "motor_settings": { ... },
  "cognitive_settings": { ... },
  "presets": [ ... ]
}
```

### 2.2 Disability Context

```json
{
  "primary_disabilities": ["blind", "low_vision", "deaf", ...],
  "assistive_technologies": ["screen_reader", "eye_tracker", ...],
  "input_devices": ["xbox_adaptive_controller", "quadstick", ...],
  "notes": "Optional personal notes"
}
```

---

## 3. Visual Accessibility Settings

### 3.1 Structure

```json
{
  "visual_settings": {
    "screen_reader": {
      "enabled": true,
      "voice": "ko-KR-female",
      "speed": 1.2,
      "verbosity": "high"
    },
    "magnification": {
      "enabled": true,
      "level": 2.0,
      "follow_focus": true
    },
    "color_settings": {
      "colorblind_mode": "deuteranopia",
      "high_contrast": true,
      "contrast_level": 1.5
    },
    "text_settings": {
      "size_multiplier": 1.5,
      "font_weight": "bold",
      "dyslexia_friendly_font": true
    },
    "ui_settings": {
      "reduce_motion": true,
      "reduce_transparency": true,
      "highlight_interactive": true
    },
    "target_indicators": {
      "enemy_highlight": true,
      "highlight_color": "#FF0000",
      "item_highlight": true,
      "path_highlight": true
    }
  }
}
```

### 3.2 Colorblind Modes

| Mode | Description |
|------|-------------|
| `protanopia` | Red-blind |
| `deuteranopia` | Green-blind |
| `tritanopia` | Blue-blind |
| `achromatopsia` | Complete color blindness |
| `custom` | User-defined color mapping |

---

## 4. Audio Accessibility Settings

### 4.1 Structure

```json
{
  "audio_settings": {
    "subtitles": {
      "enabled": true,
      "style": "full_captions",
      "speaker_identification": true,
      "sound_effects": true,
      "music_indicators": true,
      "directional_indicators": true,
      "background_opacity": 0.8,
      "text_size": "large",
      "text_color": "#FFFFFF",
      "background_color": "#000000"
    },
    "visual_sound_cues": {
      "enabled": true,
      "style": "radar",
      "position": "screen_edge",
      "categories": {
        "footsteps": true,
        "gunshots": true,
        "explosions": true,
        "voices": true,
        "environmental": true
      }
    },
    "haptic_audio": {
      "enabled": true,
      "intensity": 0.8,
      "directional": true
    },
    "mono_audio": {
      "enabled": false,
      "balance": 0.0
    },
    "tts_for_chat": {
      "enabled": true,
      "voice": "ko-KR-male",
      "speed": 1.0
    }
  }
}
```

### 4.2 Subtitle Styles

| Style | Description |
|-------|-------------|
| `dialogue_only` | Spoken words only |
| `full_captions` | Dialogue + sound effects + music |
| `minimal` | Critical information only |
| `verbose` | Maximum detail |

### 4.3 Visual Sound Cue Styles

| Style | Description |
|-------|-------------|
| `radar` | 360° radar display |
| `screen_edge` | Indicators at screen edges |
| `compass` | Compass-style direction |
| `icons` | Icon-based near player |

---

## 5. Motor Accessibility Settings

### 5.1 Structure

```json
{
  "motor_settings": {
    "input_device": {
      "type": "xbox_adaptive_controller",
      "profile_name": "My Setup"
    },
    "button_behavior": {
      "hold_to_toggle": ["aim", "run", "crouch"],
      "tap_timing_ms": 500,
      "hold_timing_ms": 1000,
      "repeat_rate_ms": 100
    },
    "stick_settings": {
      "sensitivity": 0.8,
      "dead_zone": 0.15,
      "invert_y": false,
      "swap_sticks": false
    },
    "aim_assist": {
      "enabled": true,
      "strength": "strong",
      "auto_aim": true,
      "lock_on": true,
      "sticky_aim": true
    },
    "auto_actions": {
      "auto_run": true,
      "auto_climb": true,
      "auto_pickup": true,
      "auto_reload": true
    },
    "one_handed_mode": {
      "enabled": false,
      "hand": "right",
      "layout": "compact"
    },
    "sequential_inputs": {
      "enabled": true,
      "timeout_ms": 2000
    },
    "button_remapping": { ... }
  }
}
```

### 5.2 Input Device Types

| Type | Description |
|------|-------------|
| `standard_controller` | Default console controller |
| `xbox_adaptive_controller` | Xbox Adaptive Controller |
| `ps_access_controller` | PlayStation Access Controller |
| `hori_flex` | Hori Flex Controller |
| `quadstick` | QuadStick mouth controller |
| `eye_tracker` | Tobii or similar |
| `switch_array` | External switch setup |
| `keyboard_mouse` | Standard PC input |
| `custom` | Custom hardware |

### 5.3 Button Remapping Schema

```json
{
  "button_remapping": {
    "a": { "action": "jump", "modifiers": [] },
    "b": { "action": "crouch", "modifiers": [] },
    "x": { "action": "reload", "modifiers": [] },
    "y": { "action": "switch_weapon", "modifiers": [] },
    "lb": { "action": "aim", "modifiers": [] },
    "rb": { "action": "fire", "modifiers": [] },
    "lt": { "action": "grenade", "modifiers": [] },
    "rt": { "action": "melee", "modifiers": [] },
    "dpad_up": { "action": "item_1", "modifiers": [] },
    "dpad_down": { "action": "item_2", "modifiers": [] },
    "dpad_left": { "action": "item_3", "modifiers": [] },
    "dpad_right": { "action": "item_4", "modifiers": [] },
    "left_stick_click": { "action": "sprint", "modifiers": [] },
    "right_stick_click": { "action": "ping", "modifiers": [] },
    "start": { "action": "pause", "modifiers": [] },
    "select": { "action": "map", "modifiers": [] }
  }
}
```

---

## 6. Cognitive Accessibility Settings

### 6.1 Structure

```json
{
  "cognitive_settings": {
    "difficulty": {
      "combat_difficulty": "easy",
      "puzzle_difficulty": "normal",
      "time_pressure": "none",
      "auto_complete_qte": true
    },
    "guidance": {
      "objective_reminders": true,
      "reminder_frequency_sec": 60,
      "waypoint_guidance": true,
      "hint_system": "progressive",
      "tutorial_repeat": true
    },
    "simplification": {
      "simplified_controls": true,
      "reduced_hud": false,
      "reduced_visual_effects": true,
      "reduced_enemy_count": false
    },
    "content_warnings": {
      "enabled": true,
      "categories": ["violence", "flashing", "jumpscares"],
      "auto_skip": false
    },
    "reading_aids": {
      "text_to_speech_ui": true,
      "reading_speed_control": true,
      "pause_during_text": true
    },
    "memory_aids": {
      "story_recap": true,
      "quest_journal": true,
      "recent_actions_log": true
    }
  }
}
```

### 6.2 Difficulty Levels

| Level | Description |
|-------|-------------|
| `story` | Minimal challenge, focus on narrative |
| `easy` | Reduced difficulty |
| `normal` | Default experience |
| `hard` | Increased challenge |
| `custom` | User-defined parameters |

---

## 7. Accessibility Presets

### 7.1 Structure

```json
{
  "presets": [
    {
      "preset_id": "uuid",
      "name": "FPS Gaming",
      "description": "Settings for first-person shooters",
      "game_genre": "fps",
      "settings_override": {
        "motor_settings": {
          "aim_assist": { "strength": "maximum" }
        }
      }
    }
  ]
}
```

### 7.2 Standard Presets

| Preset | Description |
|--------|-------------|
| `blind_friendly` | Full audio/haptic feedback |
| `deaf_friendly` | Full visual cues/captions |
| `one_handed` | Single-hand operation |
| `low_mobility` | Minimal precision required |
| `cognitive_support` | Maximum guidance/simplification |

---

## 8. Game Accessibility Metadata

### 8.1 Structure

```json
{
  "game_id": "uuid",
  "title": "Game Title",
  "version": "1.0.0",
  "accessibility_version": "1.0.0",
  "platforms": ["pc", "xbox", "playstation", "switch"],
  "visual_features": { ... },
  "audio_features": { ... },
  "motor_features": { ... },
  "cognitive_features": { ... },
  "input_support": { ... },
  "accessibility_rating": { ... }
}
```

### 8.2 Feature Flags

```json
{
  "visual_features": {
    "screen_reader_support": true,
    "colorblind_modes": ["protanopia", "deuteranopia", "tritanopia"],
    "text_scaling": { "min": 0.5, "max": 2.0 },
    "high_contrast_mode": true,
    "reduce_motion_option": true,
    "target_lock_indicator": true,
    "customizable_crosshair": true
  },
  "audio_features": {
    "subtitle_support": true,
    "closed_captions": true,
    "visual_sound_indicators": true,
    "mono_audio_option": true,
    "separate_volume_controls": ["master", "music", "sfx", "voice", "ambient"],
    "tts_chat": true
  },
  "motor_features": {
    "full_button_remapping": true,
    "hold_to_toggle_options": true,
    "adjustable_sensitivity": true,
    "aim_assist_levels": ["off", "low", "medium", "high", "maximum"],
    "one_handed_modes": ["left", "right"],
    "adaptive_controller_support": true,
    "keyboard_mouse_on_console": false
  },
  "cognitive_features": {
    "multiple_difficulty_levels": true,
    "skip_puzzles_option": true,
    "objective_markers": true,
    "tutorial_replay": true,
    "content_warnings": true,
    "save_anywhere": true
  },
  "input_support": {
    "controllers": ["standard", "xbox_adaptive", "ps_access", "hori_flex"],
    "eye_tracking": true,
    "voice_commands": true,
    "switch_access": true
  }
}
```

---

## 9. Accessibility Rating

### 9.1 Structure

```json
{
  "accessibility_rating": {
    "overall_score": 85,
    "visual_score": 90,
    "audio_score": 85,
    "motor_score": 80,
    "cognitive_score": 85,
    "certifications": ["wia_certified", "able_gamers_approved"],
    "reviewed_by": ["can_i_play_that"],
    "last_review_date": "2025-01-15"
  }
}
```

---

## 10. Data Exchange Format

### 10.1 Profile Export

```json
{
  "wia_game_profile": {
    "format_version": "1.0.0",
    "export_date": "2025-01-15T10:30:00Z",
    "profile": { ... },
    "checksum": "sha256:..."
  }
}
```

### 10.2 Profile Import Validation

Games should validate imported profiles and gracefully handle:
- Unsupported features (ignore or suggest alternatives)
- Version mismatches (migrate or warn)
- Invalid values (use defaults)

---

## 11. File Formats

| Format | Extension | Use Case |
|--------|-----------|----------|
| JSON | `.wia.json` | Human-readable, development |
| Binary | `.wia` | Optimized, production |
| YAML | `.wia.yaml` | Configuration files |

---

## 12. Schema Files

The following JSON Schema files define the data structures:

| Schema | Description |
|--------|-------------|
| `player-profile.schema.json` | Complete player profile |
| `visual-settings.schema.json` | Visual accessibility |
| `audio-settings.schema.json` | Audio accessibility |
| `motor-settings.schema.json` | Motor accessibility |
| `cognitive-settings.schema.json` | Cognitive accessibility |
| `game-metadata.schema.json` | Game accessibility features |
| `controller-config.schema.json` | Button remapping |
| `preset.schema.json` | Accessibility presets |

---

## 13. Implementation Notes

### 13.1 Privacy Considerations

- Disability information is sensitive data
- Players control what data is shared
- Local-first storage by default
- Encrypted cloud sync optional

### 13.2 Platform Integration

- Console save data integration
- Steam Cloud compatibility
- Cross-platform profile sync

### 13.3 Backward Compatibility

- Versioned schemas
- Migration paths for updates
- Graceful degradation

---

**弘益人間** - Gaming for Everyone


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
