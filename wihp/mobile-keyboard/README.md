# WIHP Mobile Keyboards

Mobile keyboard applications for Universal Hangul Phonology conversion.

## Platforms

| Platform | Status | Min Version |
|----------|--------|-------------|
| Android | ✅ Ready | Android 7.0+ |
| iOS | ✅ Ready | iOS 14.0+ |

## Features

- **QWERTY Layout**: Standard keyboard with WIHP conversion
- **Real-time Candidates**: See Hangul + Braille while typing
- **One-tap Convert**: Press "한글" to convert and input
- **Haptic Feedback**: Tactile response on key press
- **188+ Languages**: Support for all WIHP mapped languages
- **IPA Support**: Direct IPA → Hangul conversion

## Quick Start

### Android

```bash
cd android

# Build debug APK
./gradlew assembleDebug

# Install to device
./gradlew installDebug
```

Then enable in Settings → Language & Input → Keyboard

### iOS

```bash
cd ios/WIHPKeyboard

# Open in Xcode
open WIHPKeyboard.xcodeproj
```

Build and run, then enable in Settings → General → Keyboard → Keyboards

## Architecture

```
mobile-keyboard/
├── android/                 # Android Kotlin project
│   ├── app/src/main/
│   │   ├── java/.../       # Kotlin source
│   │   │   ├── WIHPEngine.kt
│   │   │   ├── WIHPInputMethodService.kt
│   │   │   ├── MainActivity.kt
│   │   │   └── SettingsActivity.kt
│   │   └── res/            # Resources
│   └── build.gradle
│
├── ios/                     # iOS Swift project
│   └── WIHPKeyboard/
│       ├── WIHPKeyboard/    # Main app (SwiftUI)
│       │   ├── WIHPEngine.swift
│       │   └── ContentView.swift
│       └── WIHPKeyboardExtension/  # Keyboard extension
│           └── KeyboardViewController.swift
│
└── README.md
```

## Keyboard Layout

Both platforms share the same layout:

```
┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
│ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │
├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
│ A │ S │ D │ F │ G │ H │ J │ K │ L │   │
├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
│ ⇧ │ Z │ X │ C │ V │ B │ N │ M │ ⌫ │   │
├───┴───┼───┴───┴───┴───┴───┼───────┼───┤
│  123  │       WIHP        │ 한글  │ ↵ │
└───────┴───────────────────┴───────┴───┘
```

### Key Functions

| Key | Function |
|-----|----------|
| ⇧ | Shift (toggle uppercase) |
| ⌫ | Backspace |
| ↵ | Enter/Return |
| 123/ABC | Switch numeric/alpha |
| 한글 | Convert to Hangul |
| 🌐 | Switch keyboard (iOS) |

## Conversion Examples

| Input | Output |
|-------|--------|
| hello | 헬로 |
| konnichiwa | 곤니치와 |
| bonjour | 봉주르 |
| gracias | 그라시아스 |
| /həˈloʊ/ | 헐로우 |

## Philosophy

```
홍익인간 (弘益人間) - Benefit All Humanity

세종대왕 (1443): 한글 → 한국어를 누구나 읽게
WIHP (2025): 한글 → 모든 언어를 누구나 읽게

600년의 완성.
```

---

© 2025 SmileStory Inc. / WIA
