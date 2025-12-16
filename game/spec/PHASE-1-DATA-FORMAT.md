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
