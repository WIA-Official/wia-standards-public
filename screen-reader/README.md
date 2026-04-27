# WIA Screen Reader Standard

> Universal Screen Reader Accessibility Standard for 211 Languages

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity

## Overview

WIA Screen Reader Standard provides a unified accessibility framework that works across all major screen reader platforms (NVDA, VoiceOver, TalkBack, Orca) and web browsers.

```
WIA Screen Reader Standard
= NVDA + VoiceOver + TalkBack + Browser Extensions
= Universal Accessibility Experience Across All Platforms
```

## Features

- **WIHP Pronunciation Engine**: Converts text to Korean phonetic representation for 211 languages
- **WIA Braille System**: Universal braille conversion with Grade 1, Grade 2, and WIA format support
- **Cross-Platform Support**: NVDA (Windows), VoiceOver (Mac/iOS), TalkBack (Android), Orca (Linux)
- **Browser Extensions**: Chrome, Firefox, Edge
- **Multi-language TTS**: Optimized text-to-speech for all supported languages
- **WCAG 2.2 AAA Compliant**: Full accessibility standards compliance

## Installation

### NVDA Plugin (Windows)
```bash
# Copy to NVDA addons folder
cp -r plugins/nvda/wia_screen_reader %APPDATA%/nvda/addons/
# Restart NVDA
```

### Browser Extension (Chrome)
```bash
# Load unpacked extension
chrome://extensions → Developer mode → Load unpacked → browser-extensions/chrome/
```

### Python SDK
```bash
pip install wia-screen-reader
```

### TypeScript SDK
```bash
npm install @wia/screen-reader
```

## Quick Start

### Python
```python
from wia_screen_reader import WIAScreenReader

reader = WIAScreenReader()
result = reader.process("Hello World")
print(result.wihp)      # "헐로우 월드"
print(result.braille)   # "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"
```

### TypeScript
```typescript
import { WIAScreenReader } from '@wia/screen-reader';

const reader = new WIAScreenReader();
const result = await reader.process("Hello World");
console.log(result.wihp);     // "헐로우 월드"
console.log(result.braille);  // "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"
```

## Directory Structure

```
screen-reader/
├── index.html                    # Landing page (dark theme, EN/KO toggle)
├── simulator/
│   └── index.html               # 5-tab interactive simulator
├── ebook/
│   ├── en/                      # English (8 chapters)
│   └── ko/                      # Korean (8 chapters)
├── api/
│   ├── typescript/              # TypeScript SDK
│   ├── python/                  # Python SDK
│   └── rust/                    # Rust high-performance engine
├── plugins/
│   ├── nvda/                    # NVDA Plugin (Windows)
│   ├── voiceover/               # VoiceOver Extension (Mac/iOS)
│   ├── talkback/                # TalkBack Extension (Android)
│   └── orca/                    # Orca Plugin (Linux)
├── browser-extensions/
│   ├── chrome/                  # Chrome Extension
│   ├── firefox/                 # Firefox Extension
│   └── edge/                    # Edge Extension
├── docs/                        # Documentation
├── spec/                        # Specifications
└── examples/                    # Example code
```

## 4-Phase Architecture

### Phase 1: Data Format
Universal JSON schema for screen reader data exchange.

### Phase 2: API Interface
Cross-platform API with TypeScript, Python, and Rust implementations.

### Phase 3: Protocol
Standard communication protocol for screen reader integration.

### Phase 4: Integration
Platform-specific plugins and browser extensions.

## Keyboard Shortcuts (NVDA)

| Shortcut | Action |
|----------|--------|
| NVDA+Shift+W | Convert selection to WIHP pronunciation |
| NVDA+Shift+B | Convert selection to Braille |
| NVDA+Shift+S | Speak selection with optimized TTS |

## Supported Languages

211 languages including:
- English, Korean, Japanese, Chinese (Simplified/Traditional)
- Spanish, French, German, Italian, Portuguese
- Arabic, Hindi, Russian, Thai, Vietnamese
- And 200+ more languages

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## References

- [NVDA Developer Guide](https://www.nvaccess.org/files/nvda/documentation/developerGuide.html)
- [VoiceOver Accessibility](https://developer.apple.com/accessibility/)
- [TalkBack Guide](https://developer.android.com/guide/topics/ui/accessibility)
- [WCAG 2.2](https://www.w3.org/WAI/WCAG22/quickref/)
- [WIA Standards](https://wiastandards.com/)

## License

MIT License - see [LICENSE](LICENSE) for details.

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 WIA Standards - SmileStory Inc.
