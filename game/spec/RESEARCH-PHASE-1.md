# WIA Game Accessibility - Phase 1 Research

**Research Date**: 2025-01
**Version**: 1.0.0

---

## 1. Executive Summary

Gaming accessibility has reached a milestone in 2024-2025 with all three major console manufacturers (Xbox, PlayStation, Nintendo) now offering officially licensed accessibility controllers. The industry is moving toward standardization of accessibility features, though gaps remain in cross-platform compatibility and comprehensive standards.

---

## 2. Industry Landscape

### 2.1 Major Players

| Company | Products/Initiatives |
|---------|---------------------|
| **Microsoft/Xbox** | Xbox Adaptive Controller ($99), Adaptive Joystick ($30), Gaming for Everyone Framework |
| **Sony/PlayStation** | PlayStation Access Controller ($90), PS5 Accessibility Features |
| **Nintendo** | Hori Flex Controller, System-level accessibility settings |
| **Logitech** | Adaptive Gaming Kit |
| **Hori** | Flex Controller for Switch |
| **ByoWave** | Modular adaptive controller kit |
| **Tobii** | Eye Tracker 4C/5 for gaming |
| **QuadStick** | Mouth-operated controller ($499) |

### 2.2 Market Statistics

- All three major console platforms now support accessibility controllers
- Xbox Adaptive Controller: $99 + Adaptive Joystick: $30 = ~$130 total
- PlayStation Access Controller: $90 (all-in-one solution)
- Steam adding accessibility store tags in April 2025

---

## 3. Accessibility Categories

### 3.1 Visual Accessibility (Blind/Low Vision)

**Technologies:**
- Screen reader support
- Text-to-speech for menus and UI
- Audio descriptions
- High contrast modes
- Colorblind modes (Protanopia, Deuteranopia, Tritanopia)
- Magnification options
- Large text/UI scaling

**Best Practices - The Last of Us Part 2:**
- Full text-to-speech navigation
- Lock-on aim / auto-targeting
- Audio cues for traversal and combat
- Completely playable by blind gamers on hardest difficulty

### 3.2 Auditory Accessibility (Deaf/Hard of Hearing)

**Two Major Tools:**
1. **Subtitles/Captions**
   - Speaker identification
   - Sound effect descriptions [footsteps], [explosion]
   - Background music indicators
   - Directional indicators

2. **Visual Cues**
   - On-screen damage direction indicators
   - Visual sound radar
   - Enemy position indicators
   - Environmental audio visualization

**Subtitles vs. Closed Captions:**
- **Subtitles**: Dialogue only
- **Closed Captions [CC]**: Dialogue + sound effects + music indicators

**Best Practices - Fortnite:**
- "Visualize Sound Effects" toggle
- Symbols appear for in-game sounds
- Directional indicators for sound sources
- All content properly subtitled

**Best Practices - Minecraft:**
- Subtitles describe environmental sounds
- "Creeper hissing", "Water flowing", "Zombie groaning"
- Directional arrows indicate sound source

**Best Practices - Gears 5:**
- Music state changes described in subtitles
- Combat/safe state indicators

### 3.3 Motor Accessibility

**Input Devices:**

| Device | Type | Features |
|--------|------|----------|
| Xbox Adaptive Controller | Hub | 19 inputs (3.5mm jacks), 2 USB ports |
| PlayStation Access Controller | All-in-one | Programmable buttons, modular design |
| Hori Flex Controller | Switch-compatible | External switch/joystick support |
| QuadStick | Mouth-operated | Head movements, sip/puff |
| QuadJoy 3 | Mouth-operated | Sip/puff + joystick |
| Tobii Eye Tracker | Eye-gaze | Hands-free cursor control |
| Project Gameface (Google) | Face tracking | AI-based facial gesture recognition |

**Input Methods:**
- Button remapping
- One-handed controller configurations
- Eye tracking
- Head tracking
- Sip/puff switches
- Foot pedals
- Muscle twitch switches
- Voice commands

**Software Features:**
- Hold-to-toggle options
- Adjustable input timing
- Sequential button inputs
- Auto-aim/aim assist
- Motor accessibility presets

### 3.4 Cognitive Accessibility

**Features:**
- Simplified controls
- Tutorial replay
- Objective reminders
- Difficulty adjustment
- Reduced visual complexity
- Content warnings
- Progress saving anywhere
- Skip challenging sections

---

## 4. Legal Framework

### 4.1 CVAA (21st Century Communications and Video Accessibility Act)

- Mandates accessibility for "advanced communications services"
- Applies to in-game chat and communication features
- Requires accessible text/voice chat
- Transcripts for audio logs and story content

### 4.2 Regional Requirements

| Region | Legislation |
|--------|------------|
| USA | CVAA, ADA (indirect) |
| EU | European Accessibility Act (2025) |
| UK | Equality Act 2010 |
| Japan | JIS X 8341 |

---

## 5. Existing Standards and Guidelines

### 5.1 Xbox Gaming for Everyone Product Inclusion Framework (2024)

Microsoft's framework for accessible game development covering:
- Controller accessibility
- Visual accessibility
- Audio accessibility
- Input flexibility
- Difficulty options

### 5.2 Game Accessibility Guidelines (gameaccessibilityguidelines.com)

Categories:
- Basic (minimum)
- Intermediate (recommended)
- Advanced (best practice)

Areas:
- Motor
- Cognitive
- Vision
- Hearing
- Speech

### 5.3 IGDA Game Accessibility SIG

Industry group promoting accessibility standards and best practices.

### 5.4 Can I Play That?

Consumer-focused accessibility reviews and ratings.

---

## 6. Technology Trends

### 6.1 Eye Tracking Integration

- Tobii eye trackers becoming more affordable
- Integration with Xbox Adaptive Controller
- Game On 1 for AAC device integration

### 6.2 AI-Powered Accessibility

- Google Project Gameface: Facial gesture to input mapping
- AI-based auto-aim improvements
- Procedural audio description generation

### 6.3 Haptic Feedback Advancements

- PS5 DualSense haptic feedback
- Directional haptic cues for deaf players
- Spatial audio through haptics

### 6.4 Cloud Gaming Accessibility

- Xbox Cloud Gaming accessibility features
- Reduced hardware requirements
- Remote play for mobility-limited players

---

## 7. Cross-Platform Challenges

### 7.1 Controller Compatibility

| Controller | Xbox | PlayStation | Switch | PC |
|------------|------|-------------|--------|-----|
| Xbox Adaptive Controller | ✅ | ❌ | ❌ | ✅ |
| PS Access Controller | ❌ | ✅ | ❌ | ⚠️ |
| Hori Flex | ❌ | ❌ | ✅ | ✅ |

### 7.2 Feature Parity Gap

- No universal accessibility settings standard
- Per-game implementation varies widely
- Platform-specific accessibility features

---

## 8. Data Format Requirements

Based on research, the WIA Game accessibility data format should cover:

### 8.1 Player Profile
- Disability types
- Preferred input methods
- Visual preferences (contrast, color, size)
- Audio preferences (subtitles, visual cues)
- Motor preferences (hold-to-toggle, timing)
- Cognitive preferences (difficulty, complexity)

### 8.2 Game Accessibility Features
- Supported input devices
- Visual accessibility features
- Audio accessibility features
- Motor accessibility features
- Cognitive accessibility features
- Platform-specific features

### 8.3 Controller Configuration
- Button mappings
- Input device specifications
- Sensitivity settings
- Macro definitions

### 8.4 Accessibility Preset
- Pre-configured settings for common needs
- Quick-switch profiles
- Import/export capabilities

---

## 9. References

### Industry Resources
- [Xbox Accessibility Updates](https://news.xbox.com/en-us/2023/10/17/xbox-new-accessibility-updates/)
- [Gaming for Everyone Framework](https://caniplaythat.com/2024/03/22/gaming-for-everyone-product-inclusion-framework-released-by-xbox/)
- [2024 Video Game Accessibility Recap](https://access-ability.uk/2024/12/13/2024-video-game-accessibility-end-of-year-recap/)

### Deaf Accessibility
- [Deaf Accessibility in Video Games](https://leahybaker.com/deaf_access/)
- [Accessibility.com - Deaf Gaming](https://www.accessibility.com/blog/what-video-game-developers-should-know-about-deaf-accessibility)

### Motor Accessibility
- [AbleGamers Adaptive Equipment](https://ablegamers.org/adaptive-gaming-equipment/)
- [Pretorian Technologies](https://www.pretorianuk.com/flex-controller/)

### Eye Tracking
- [AbilityNet - Eye Tracking Gaming](https://abilitynet.org.uk/news-blogs/next-gen-eye-tracking-game-changer-disabled-people)

---

## 10. Conclusion

The gaming industry has made significant progress in accessibility, particularly with hardware solutions. However, the lack of standardized data formats for accessibility settings creates friction when players move between games and platforms. WIA Game aims to provide:

1. **Universal Player Profiles** - Portable accessibility preferences
2. **Standardized Feature Descriptions** - Consistent accessibility feature taxonomy
3. **Cross-Platform Configuration** - Shareable controller mappings
4. **Accessibility Metadata** - Machine-readable game accessibility information

---

**弘益人間** - Gaming for Everyone
