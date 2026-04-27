# WIA Screen Reader - Python SDK

> Universal Screen Reader Accessibility Standard for 211 Languages

**Philosophy**: 弘益人間 (홍익인간) - Benefit All Humanity

## Installation

```bash
pip install wia-screen-reader
```

## Quick Start

```python
from wia_screen_reader import WIAScreenReader

reader = WIAScreenReader()

# Convert text
result = reader.process("Hello World")
print(result.wihp)      # "헬로우 월드"
print(result.braille)   # "⠓⠑⠇⠇⠕ ⠺⠕⠗⠇⠙"

# TTS
reader.set_tts(rate=1.2, pitch=1.0)
reader.speak("Hello World")

# Multi-language
reader.set_language("ja")
result = reader.process("こんにちは")
print(result.wihp)      # "곤니치와"
```

## Features

- WIHP (WIA International Hangul Pronunciation) for 211 languages
- Universal Braille conversion (Grade 1, Grade 2, WIA format)
- Cross-platform TTS support
- NVDA plugin integration
- WCAG 2.2 AAA compliant

## License

MIT License

© 2025 WIA Standards - SmileStory Inc.
