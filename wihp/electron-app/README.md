# WIHP Desktop

Cross-platform desktop application for Universal Hangul Phonology conversion.

## Features

- **System Tray**: Runs in background, always accessible
- **Global Hotkey**: `Ctrl+Shift+W` to convert clipboard instantly
- **Real-time Conversion**: Type and see Hangul + Braille immediately
- **Copy to Clipboard**: One-click copy functionality
- **Cross-platform**: Windows, macOS, Linux

## Installation

### From Release (Recommended)

Download the appropriate installer for your platform:

- **Windows**: `WIHP-Desktop-Setup-1.0.0.exe`
- **macOS**: `WIHP-Desktop-1.0.0.dmg`
- **Linux**: `WIHP-Desktop-1.0.0.AppImage`

### From Source

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-braille.git
cd wia-braille/wihp/electron-app

# Install dependencies
npm install

# Run in development
npm start

# Build for your platform
npm run build
```

## Usage

### Basic Usage

1. Launch WIHP Desktop
2. Type any text (English, Japanese romaji, IPA, etc.)
3. See instant Hangul conversion
4. Click "Copy" to copy to clipboard

### Global Hotkey

1. Copy any text to clipboard
2. Press `Ctrl+Shift+W` (or `Cmd+Shift+W` on Mac)
3. Clipboard is automatically converted to Hangul
4. Paste anywhere!

### System Tray

- Click tray icon to show/hide window
- Right-click for menu:
  - Convert Clipboard
  - Show Window
  - Quit

## Build Commands

```bash
# Build for all platforms
npm run build:all

# Build for specific platform
npm run build:win    # Windows
npm run build:mac    # macOS
npm run build:linux  # Linux
```

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+Shift+W` | Convert clipboard (global) |
| `Ctrl+Shift+H` | Show window (global) |
| `Ctrl+Enter` | Copy result |
| `Escape` | Clear input |

## Technical Details

- **Framework**: Electron 28+
- **WIHP Engine**: JavaScript-based IPA→Hangul converter
- **Supported Input**:
  - IPA (International Phonetic Alphabet)
  - Romanization (English, Pinyin, Romaji, etc.)
  - 200+ phoneme mappings

## File Structure

```
electron-app/
├── package.json      # Dependencies & build config
├── main.js           # Electron main process
├── index.html        # UI
├── wihp-engine.js    # Core conversion engine
├── assets/           # Icons
│   ├── icon.png      # App icon
│   ├── icon.ico      # Windows icon
│   ├── icon.icns     # macOS icon
│   └── tray-icon.png # Tray icon
└── dist/             # Built applications
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
