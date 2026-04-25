# WIHP iOS Keyboard

iOS custom keyboard for Universal Hangul Phonology conversion.

## Features

- **QWERTY Layout**: Standard iOS keyboard with WIHP conversion
- **Real-time Candidates**: See Hangul conversion while typing
- **One-tap Convert**: Press "한글" to convert and input
- **Braille Support**: View Braille representation in candidates
- **Haptic Feedback**: Tactile response on key press
- **SwiftUI App**: Modern setup and demo interface

## Requirements

- iOS 14.0 or later
- Xcode 14.0 or later
- Swift 5.7 or later

## Installation

### From TestFlight (Coming Soon)

1. Join TestFlight beta program
2. Install WIHP Keyboard
3. Enable in Settings → General → Keyboard → Keyboards

### Build from Source

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-braille.git
cd wia-braille/wihp/mobile-keyboard/ios/WIHPKeyboard

# Open in Xcode
open WIHPKeyboard.xcodeproj

# Select target device/simulator
# Build and Run (⌘+R)
```

### Enable Keyboard

1. Open Settings app
2. General → Keyboard → Keyboards
3. Add New Keyboard...
4. Select "WIHP"
5. (Optional) Allow Full Access for haptic feedback

## Usage

1. **Switch to WIHP** - Tap 🌐 to switch keyboards
2. **Type normally** - Use QWERTY layout
3. **View candidates** - See original + Hangul + Braille
4. **Tap "한글"** - Convert buffer to Hangul and input
5. **Tap candidate** - Select directly from candidate bar

## Keyboard Layout

```
┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
│ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │
├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
│ A │ S │ D │ F │ G │ H │ J │ K │ L │   │
├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
│ ⇧ │ Z │ X │ C │ V │ B │ N │ M │ ⌫ │   │
├───┴───┼───┼───────────────┼───────┼───┤
│  123  │🌐│     WIHP      │ 한글  │ ↵ │
└───────┴───┴───────────────┴───────┴───┘
```

## File Structure

```
ios/WIHPKeyboard/
├── WIHPKeyboard/                    # Main App
│   ├── WIHPKeyboardApp.swift        # App entry
│   ├── ContentView.swift            # Setup UI
│   ├── WIHPEngine.swift             # Conversion engine
│   └── Info.plist
├── WIHPKeyboardExtension/           # Keyboard Extension
│   ├── KeyboardViewController.swift # Keyboard UI
│   └── Info.plist
└── WIHPKeyboard.xcodeproj/          # Xcode project
```

## Privacy

- **No network access required**
- **No data collection**
- **Full Access optional** (only for haptic feedback)
- All processing done locally on device

## Philosophy

```
홍익인간 (弘益人間) - Benefit All Humanity

세종대왕 (1443): 한글 → 한국어를 누구나 읽게
WIHP (2025): 한글 → 모든 언어를 누구나 읽게

600년의 완성.
```

---

© 2025 SmileStory Inc. / WIA
