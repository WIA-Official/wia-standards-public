# WIA Screen Reader - NVDA Plugin Guide

> 弘益人間 (홍익인간) - Benefit All Humanity

## Overview

The WIA Screen Reader NVDA Plugin provides WIHP pronunciation and WIA Braille support for NVDA screen reader on Windows.

## Requirements

- NVDA 2023.1 or later
- Windows 10/11
- Python 3.7+ (included with NVDA)

## Installation

### Method 1: Manual Installation

1. Download the `wia_screen_reader` folder
2. Copy to: `%APPDATA%\nvda\addons\`
3. Restart NVDA

### Method 2: Add-on Manager

1. Open NVDA menu (NVDA+N)
2. Go to Tools > Add-on Manager
3. Click "Install from external source"
4. Select the WIA Screen Reader add-on package
5. Restart NVDA

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| NVDA+Shift+W | Convert selected text to WIHP pronunciation |
| NVDA+Shift+B | Convert selected text to Braille |
| NVDA+Shift+S | Speak with optimized TTS |
| NVDA+Shift+L | Change language |
| NVDA+Shift+I | Show WIA Screen Reader info |

## Features

### WIHP Pronunciation

Converts any text to Korean Hangul pronunciation for consistent accessibility across 211 languages.

**Example:**
```
Original: "Hello World"
WIHP: "헬로우 월드"
```

### WIA Braille

Converts text to multiple braille formats:

- **Grade 1**: Uncontracted braille
- **Grade 2**: Contracted braille
- **WIA Format**: Korean Jamo-based

### Multi-language Support

Supports 211 languages including:
- English, Korean, Japanese, Chinese
- Spanish, French, German
- Arabic, Hindi, Russian
- And 200+ more

## Configuration

### Settings Location

NVDA Preferences > Settings > WIA Screen Reader

### Options

| Setting | Description | Default |
|---------|-------------|---------|
| Default Language | Primary language for conversion | English |
| Use WIHP for TTS | Apply WIHP when speaking | Enabled |
| Braille Grade | Default braille grade | Grade 1 |

## Troubleshooting

### Plugin Not Loading

1. Check NVDA version compatibility
2. Verify addon folder location
3. Check NVDA log for errors

### WIHP Not Working

1. Ensure text is selected
2. Try clipboard fallback (copy text first)
3. Check language setting

### Braille Display Issues

1. Verify braille display is connected
2. Check NVDA braille settings
3. Try different braille grade

## Development

### File Structure

```
wia_screen_reader/
├── __init__.py
├── globalPlugin.py
├── wihp_engine.py
├── braille_engine.py
└── config.py
```

### Adding Custom Pronunciations

```python
from .wihp_engine import WIHPEngine

engine = WIHPEngine()
engine.add_word("custom", "커스텀")
```

## Support

- GitHub Issues: https://github.com/WIA-Official/wia-standards/issues
- Documentation: https://wiastandards.com/screen-reader/

---

© 2025 WIA Standards - SmileStory Inc.
