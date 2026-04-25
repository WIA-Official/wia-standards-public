# WIHP Android Keyboard

Android custom keyboard for Universal Hangul Phonology conversion.

## Features

- **QWERTY Layout**: Standard keyboard with WIHP conversion
- **Real-time Conversion**: See Hangul candidates while typing
- **One-tap Convert**: Press "한글" to convert and input
- **Braille Support**: View Braille representation in candidates
- **Haptic Feedback**: Tactile response on key press

## Installation

### From APK

1. Download `WIHP-Keyboard.apk`
2. Enable "Install from unknown sources"
3. Install the APK
4. Go to Settings → Language & Input → Keyboard
5. Enable "WIHP Keyboard"
6. Switch to WIHP when typing

### Build from Source

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-braille.git
cd wia-braille/wihp/mobile-keyboard/android

# Build debug APK
./gradlew assembleDebug

# Build release APK
./gradlew assembleRelease

# Install to device
./gradlew installDebug
```

## Usage

1. **Type normally** - Use QWERTY layout
2. **View candidates** - See original + Hangul + Braille
3. **Tap "한글"** - Convert buffer to Hangul and input
4. **Tap candidate** - Select directly from candidate bar

## Keyboard Layout

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

## Requirements

- Android 7.0 (API 24) or higher
- ~5MB storage

## Settings

- **Vibration**: Enable/disable haptic feedback
- **Auto-convert**: Convert on space key
- **Show Braille**: Display Braille in candidates

## File Structure

```
android/
├── app/
│   ├── src/main/
│   │   ├── java/com/wia/wihp/keyboard/
│   │   │   ├── WIHPEngine.kt          # Conversion engine
│   │   │   ├── WIHPInputMethodService.kt  # Keyboard service
│   │   │   ├── MainActivity.kt        # Setup activity
│   │   │   └── SettingsActivity.kt    # Settings
│   │   ├── res/
│   │   │   ├── values/                # Strings, colors, themes
│   │   │   └── xml/method.xml         # IME configuration
│   │   └── AndroidManifest.xml
│   └── build.gradle
├── build.gradle
└── settings.gradle
```

## Philosophy

```
홍익인간 (弘益人間) - Benefit All Humanity

세종대왕 (1443): 한글 → 한국어를 누구나 읽게
WIHP (2025): 한글 → 모든 언어를 누구나 읽게

600년의 완성.
```

---

© 2025 SmileStory Inc. / WIA
