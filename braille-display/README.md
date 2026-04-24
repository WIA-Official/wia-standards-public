# ⠿ WIA Braille Display Standard

> **WIA-AAC-010** | Refreshable Braille Interface Standard v1.0.0

**홍익인간 (弘益人間)** - Benefit All Humanity

---

## Overview

The WIA Braille Display Standard provides a unified, open specification for refreshable Braille displays, enabling universal accessibility for blind and visually impaired users worldwide.

### Key Features

- **Universal Compatibility** - Works with all major screen readers (NVDA, JAWS, VoiceOver, TalkBack, Orca)
- **Multi-Protocol Support** - USB HID, Bluetooth LE, Serial, BRLTTY
- **Multi-Language** - 100+ language Braille tables
- **6-Dot & 8-Dot** - Support for standard and computer Braille
- **Open Standard** - Free to implement, MIT licensed

---

## Quick Stats

| Metric | Value |
|--------|-------|
| Braille Patterns | 256 (8-bit) |
| Dot Matrix | 6-dot / 8-dot |
| Cell Support | 20, 40, 80+ cells |
| Device Models | 100+ supported |

---

## Directory Structure

```
braille-display/
├── index.html                 # Landing page
├── simulator/
│   └── index.html             # Interactive simulator (5 tabs)
├── ebook/
│   ├── en/                    # English Ebook (8 chapters)
│   │   ├── index.html
│   │   └── chapter-01~08.html
│   └── ko/                    # Korean Ebook (8 chapters)
│       ├── index.html
│       └── chapter-01~08.html
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md # Data format specification
│   ├── PHASE-2-API.md         # API interface specification
│   ├── PHASE-3-PROTOCOL.md    # Communication protocol
│   └── PHASE-4-INTEGRATION.md # WIA integration
└── README.md
```

---

## Four-Phase Architecture

### Phase 1: Data Format
Unified Braille cell data model with JSON schema, Unicode mapping (U+2800-U+28FF), and multi-language support.

### Phase 2: API Interface
WiaBrailleDisplay class with device discovery, cell management, cursor routing, and key event handling.

### Phase 3: Communication Protocol
Support for USB HID, Bluetooth LE GATT, Serial, and BRLTTY integration.

### Phase 4: Ecosystem Integration
Screen reader integration patterns for NVDA, JAWS, VoiceOver, TalkBack, and Orca.

---

## Quick Start

### TypeScript/JavaScript

```typescript
import { WiaBrailleDisplay } from '@wia/braille-display';

// Discover and connect
const devices = await WiaBrailleDisplay.discover();
const display = await devices[0].connect();

// Write text
await display.writeText('Hello World');

// Handle key events
display.on('key', (event) => {
  console.log('Key pressed:', event.key);
});
```

### Python

```python
from wia_braille import BrailleDisplay

# Connect to display
display = BrailleDisplay.discover()[0]
display.connect()

# Write text
display.write_text("Hello World")

# Handle events
@display.on_key
def handle_key(event):
    print(f"Key pressed: {event.key}")
```

---

## Supported Devices

| Manufacturer | Models |
|--------------|--------|
| Freedom Scientific | Focus 40/80, PAC Mate |
| HumanWare | Brailliant, BrailleNote |
| HIMS | QBraille XL, Smart Beetle |
| Baum | VarioUltra, Pronto! |
| Orbit Research | Orbit Reader 20 |
| Help Tech | Actilino, Active Star |

---

## Certification

| Level | Description | Cost |
|-------|-------------|------|
| Level 1: Compliant | Basic interoperability | $500 |
| Level 2: Certified | Commercial products | $2,000 |
| Level 3: Certified Plus | Medical/professional | $5,000 |

Visit [cert.wiastandards.com](https://cert.wiastandards.com) to start certification.

---

## Resources

- **Landing Page**: [braille-display.wiastandards.com](https://braille-display.wiastandards.com)
- **Simulator**: [braille-display.wiastandards.com/simulator](https://braille-display.wiastandards.com/simulator)
- **Ebook**: [wiabook.com/braille-display](https://wiabook.com/braille-display)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)

---

## Related Standards

- **WIA-AAC-001** - AAC (Augmentative Communication)
- **WIA-AAC-011** - Sign Language Recognition
- **WIA-AAC-009** - BCI (Brain-Computer Interface)

---

## License

MIT License - Free to use, modify, and distribute.

---

## Contact

**World Certification Industry Association (WIA)**
SmileStory Inc.

- Website: [wiastandards.com](https://wiastandards.com)
- Email: contact@wia.family
- GitHub: [github.com/WIA-Official](https://github.com/WIA-Official)

---

**홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

*Universal Braille access for everyone, everywhere.*

© 2025 WIA - MIT License
