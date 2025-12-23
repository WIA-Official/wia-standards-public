# WIHP - WIA International Hangul Phonology

**IPA → Hangul Phonetic Mapping with 100% Coverage**

弘益人間 - Benefit All Humanity

---

## Overview

WIHP (WIA International Hangul Phonology) is the world's first complete IPA-to-native-script mapping system, providing 100% coverage of standard IPA symbols.

### Why Hangul?

Korean Hangul is the only prominent **featural writing system** in the world (Sampson, 1985). Its characters are designed based on the physical shape of speech organs:

```
ㄱ = Shape of tongue blocking throat (velar)
ㄴ = Shape of tongue touching alveolar ridge
ㅁ = Shape of lips (bilabial)
ㅅ = Shape of teeth (dental)
ㅇ = Shape of throat (glottal/nasal)
```

This scientific design makes Hangul uniquely suited for representing the sounds of **all human languages**.

---

## Coverage

| Category | Covered | Total | Percentage |
|----------|---------|-------|------------|
| **Consonants** | 69 | 69 | **100%** |
| **Vowels** | 28 | 28 | **100%** |
| **Diacritics** | 20 | 20 | **100%** |
| **Total** | 117 | 117 | **100%** |

**Grade: A+ (Perfect)**

---

## Quick Start

### Rust

```rust
use wia_wihp::{ipa_to_hangul, hangul_to_ipa};

// Convert IPA to Hangul
let hangul = ipa_to_hangul("/həˈloʊ/").unwrap();
assert_eq!(hangul, "헬로우");

// Convert back to IPA
let ipa = hangul_to_ipa("헬로우").unwrap();
assert_eq!(ipa, "həloʊ");
```

### TypeScript

```typescript
import { ipaToHangul } from 'wia-wihp';

const result = ipaToHangul('/həˈloʊ/');
console.log(result); // "헬로우"
```

### Python

```python
from wia_wihp import ipa_to_hangul

result = ipa_to_hangul("/həˈloʊ/")
print(result)  # "헬로우"
```

---

## Examples

| Language | Word | IPA | WIHP |
|----------|------|-----|------|
| English | hello | /həˈloʊ/ | 헬로우 |
| English | world | /wɝld/ | 월드 |
| Japanese | 東京 | /toːkʲoː/ | 토ː쿄ː |
| Mandarin | 你好 | /ni˨˩˦ xaʊ˨˩˦/ | 니˨˩˦ 하오˨˩˦ |
| Arabic | مرحبا | /marħaba/ | 마르ㅎˤ아바 |
| Swahili | jambo | /ˈdʒambo/ | '잠보 |
| Zulu | ngiyabonga | /ŋijaˈboŋɡa/ | 응이야'봉가 |
| Xhosa | ixhosa | /ikǁʰɔːsa/ | 이ㄷ͜ㄹ코ː사 |

---

## Use Cases

1. **Text-to-Speech (TTS)**: Use Korean TTS for any language
2. **Speech Recognition (ASR)**: Transcribe foreign audio with Korean ASR
3. **Accessibility**: Braille display for all pronunciations via WIA-Braille
4. **Language Learning**: Learn any pronunciation through Korean phonetics
5. **AI/ML**: Unified phonetic representation for multilingual models

---

## Directory Structure

```
wihp/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md     # Data format specification
│   ├── PHASE-2-API-INTERFACE.md   # API interface definition
│   ├── PHASE-3-PROTOCOL.md        # Conversion protocol
│   └── PHASE-4-INTEGRATION.md     # Integration guide
│
├── api/
│   ├── rust/                      # Rust SDK
│   ├── typescript/                # TypeScript SDK
│   └── python/                    # Python SDK
│
├── schemas/
│   ├── ipa-symbol.schema.json     # IPA symbol schema
│   └── wihp-mapping.schema.json   # Mapping file schema
│
├── docs/
│   ├── VALIDATION-REPORT-EN.md    # 100% coverage (English)
│   └── VALIDATION-REPORT-KO.md    # 100% coverage (Korean)
│
└── examples/
    └── basic-conversion/          # Basic usage examples
```

---

## Academic References

1. **Sampson, G. (1985)** - *Writing Systems*
   - Identified Hangul as the only prominent featural writing system

2. **International Phonetic Association (2015)** - *Handbook of the IPA*
   - Standard reference for phonetic symbols

3. **Lee, Y.S. (2012)** - *Hangulphabet* (arXiv)
   - Proposed Hangul as IPA alternative

---

## Related Standards

- [WIA-Braille](../wia-braille/) - Unified Braille system
- [Haptic](../haptic/) - Haptic feedback for accessibility
- [Voice](../voice/) - Voice-Sign conversion

---

## License

MIT License

Copyright (c) 2025 SmileStory Inc. / WIA

---

## Author

**Yeon Sam-Heum, Ph.D.**
Founder & Chief Standards Architect
SmileStory Inc. / WIA

---

**Built with ❤️ for 7,000+ languages**
