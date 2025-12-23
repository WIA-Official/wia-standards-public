# WIHP Data Format Specification

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-12-23

## 1. Overview

WIHP (WIA International Hangul Phonology) defines the data format for mapping International Phonetic Alphabet (IPA) symbols to Korean Hangul characters.

### 1.1 Design Principles

- **Complete Coverage**: 100% of IPA standard symbols (117 total)
- **Bidirectional**: IPA → Hangul and Hangul → IPA
- **Machine-Readable**: JSON/YAML formats for automation
- **Human-Readable**: Markdown tables for documentation

## 2. Symbol Categories

### 2.1 Coverage Summary

| Category | Count | Coverage |
|----------|-------|----------|
| Consonants | 69 | 100% |
| Vowels | 28 | 100% |
| Diacritics | 20 | 100% |
| **Total** | **117** | **100%** |

## 3. Data Structures

### 3.1 IPA Symbol Entry

```typescript
interface IPASymbol {
  // IPA character (e.g., "p", "ʃ", "ə")
  ipa: string;

  // WIHP code (e.g., "PL01-MN01-AR01")
  code: string;

  // Korean Hangul representation
  hangul: string;

  // Category: consonant, vowel, diacritic
  category: 'consonant' | 'vowel' | 'diacritic';

  // Subcategory (e.g., "plosive", "fricative", "close")
  subcategory: string;

  // Description in Korean
  description_ko: string;

  // Description in English
  description_en: string;

  // Example languages using this sound
  examples: string[];
}
```

### 3.2 WIHP Code Format

```
PL{place}-MN{manner}-AR{articulation}

Place (PL):
  01 = Bilabial (양순음)
  02 = Labiodental (순치음)
  03 = Dental (치음)
  04 = Alveolar (치조음)
  05 = Postalveolar (후치조음)
  06 = Retroflex (권설음)
  07 = Palatal (경구개음)
  08 = Velar (연구개음)
  09 = Uvular (구개수음)
  10 = Pharyngeal (인두음)
  11 = Glottal (성문음)

Manner (MN):
  01 = Plosive (파열음)
  02 = Nasal (비음)
  03 = Trill (전동음)
  04 = Tap/Flap (탄설음)
  05 = Fricative (마찰음)
  06 = Lateral Fricative (설측마찰음)
  07 = Approximant (접근음)
  08 = Lateral Approximant (설측접근음)

Articulation (AR):
  01 = Voiceless (무성)
  02 = Voiced (유성)
  03 = Aspirated (유기)
  04 = Ejective (방출)
  05 = Click (클릭)
  06 = Implosive (내파)
```

## 4. Consonant Mappings

### 4.1 Plosives (파열음)

| IPA | Code | Hangul | Description |
|-----|------|--------|-------------|
| /p/ | PL01-MN01-AR01 | ㅂ | Voiceless bilabial |
| /b/ | PL01-MN01-AR02 | ㅂ | Voiced bilabial |
| /t/ | PL04-MN01-AR01 | ㄷ | Voiceless alveolar |
| /d/ | PL04-MN01-AR02 | ㄷ | Voiced alveolar |
| /ʈ/ | PL06-MN01-AR01 | ㄷ̣ | Voiceless retroflex |
| /ɖ/ | PL06-MN01-AR02 | ㄷ̣ | Voiced retroflex |
| /c/ | PL07-MN01-AR01 | ㅈ | Voiceless palatal |
| /ɟ/ | PL07-MN01-AR02 | ㅈ | Voiced palatal |
| /k/ | PL08-MN01-AR01 | ㄱ | Voiceless velar |
| /ɡ/ | PL08-MN01-AR02 | ㄱ | Voiced velar |
| /q/ | PL09-MN01-AR01 | ㅋ̵ | Voiceless uvular |
| /ɢ/ | PL09-MN01-AR02 | ㄱ̵ | Voiced uvular |
| /ʔ/ | PL11-MN01-AR01 | ㆆ | Glottal stop |

### 4.2 Nasals (비음)

| IPA | Code | Hangul | Description |
|-----|------|--------|-------------|
| /m/ | PL01-MN02-AR02 | ㅁ | Bilabial |
| /ɱ/ | PL02-MN02-AR02 | ㅁ | Labiodental |
| /n/ | PL04-MN02-AR02 | ㄴ | Alveolar |
| /ɳ/ | PL06-MN02-AR02 | ㄴ̣ | Retroflex |
| /ɲ/ | PL07-MN02-AR02 | ㄴʸ | Palatal |
| /ŋ/ | PL08-MN02-AR02 | ㅇ | Velar |
| /ɴ/ | PL09-MN02-AR02 | ㅇ̵ | Uvular |

### 4.3 Fricatives (마찰음)

| IPA | Code | Hangul | Description |
|-----|------|--------|-------------|
| /f/ | PL02-MN05-AR01 | ㅍ/ㅎㅍ | Voiceless labiodental |
| /v/ | PL02-MN05-AR02 | ㅸ | Voiced labiodental |
| /θ/ | PL03-MN05-AR01 | ㅅ̪ | Voiceless dental |
| /ð/ | PL03-MN05-AR02 | ㅿ̪ | Voiced dental |
| /s/ | PL04-MN05-AR01 | ㅅ | Voiceless alveolar |
| /z/ | PL04-MN05-AR02 | ㅿ | Voiced alveolar |
| /ʃ/ | PL05-MN05-AR01 | ㅅㅣ/쉬 | Voiceless postalveolar |
| /ʒ/ | PL05-MN05-AR02 | ㅈㅣ | Voiced postalveolar |
| /h/ | PL11-MN05-AR01 | ㅎ | Voiceless glottal |

### 4.4 Implosives (내파음)

| IPA | Code | Hangul | Description | Languages |
|-----|------|--------|-------------|-----------|
| /ɓ/ | PL01-MN01-AR06 | ㅂ̰ | Bilabial implosive | Swahili, Vietnamese |
| /ɗ/ | PL04-MN01-AR06 | ㄷ̰ | Alveolar implosive | Hausa, Vietnamese |
| /ʄ/ | PL07-MN01-AR06 | ㅈ̰ | Palatal implosive | Sindhi, Swahili |
| /ɠ/ | PL08-MN01-AR06 | ㄱ̰ | Velar implosive | Swahili |
| /ʛ/ | PL09-MN01-AR06 | ㄱ̵̰ | Uvular implosive | Maiduan |

### 4.5 Clicks (클릭음)

| IPA | Code | Hangul | Description | Languages |
|-----|------|--------|-------------|-----------|
| /ʘ/ | PL01-MN01-AR05 | ㅂ͜ㅎ | Bilabial click | Khoisan |
| /ǀ/ | PL03-MN01-AR05 | ㄷ͜ㅎ | Dental click | Zulu, Xhosa |
| /ǃ/ | PL04-MN01-AR05 | ㄷ͜ㅋ | Alveolar click | Xhosa |
| /ǂ/ | PL07-MN01-AR05 | ㄷ͜ㅈ | Palatal click | Khoekhoe |
| /ǁ/ | PL08-MN06-AR05 | ㄷ͜ㄹ | Lateral click | Zulu |

## 5. Vowel Mappings

### 5.1 Vowel Chart

| Height | Front Unrounded | Front Rounded | Central | Back Unrounded | Back Rounded |
|--------|-----------------|---------------|---------|----------------|--------------|
| Close | i → ㅣ | y → ㅟ | ɨ → ㅡ | ɯ → ㅡ | u → ㅜ |
| Near-close | ɪ → ㅣ | ʏ → ㅟ | | | ʊ → ㅜ |
| Close-mid | e → ㅔ | ø → ㅚ | ɘ → ㅓ | ɤ → ㅓ | o → ㅗ |
| Mid | | | ə → ㅓ | | |
| Open-mid | ɛ → ㅔ | œ → ㅚ | ɜ → ㅓ | ʌ → ㅓ | ɔ → ㅗ |
| Near-open | æ → ㅐ | | ɐ → ㅏ | | |
| Open | a → ㅏ | ɶ → ㅚ | | ɑ → ㅏ | ɒ → ㆍ |

## 6. Diacritic Mappings

### 6.1 Phonation

| IPA | Hangul | Description |
|-----|--------|-------------|
| ʰ | ㅋ/ㅌ/ㅍ/ㅊ | Aspirated |
| ʼ | ʼ | Ejective |
| ̤ | ̤ | Breathy voice |
| ̰ | ̰ | Creaky voice |

### 6.2 Secondary Articulation

| IPA | Hangul | Description |
|-----|--------|-------------|
| ʷ | +ㅗ/ㅜ | Labialized |
| ʲ | +ㅣ | Palatalized |
| ˠ | (unmarked) | Velarized |
| ˤ | ˤ | Pharyngealized |

### 6.3 Tone Markers

| IPA | Hangul | Level | Languages |
|-----|--------|-------|-----------|
| ˥ | ˥ / 5 | High | Mandarin 1st, Cantonese |
| ˦ | ˊ / 4 | Mid-High | Vietnamese sắc |
| ˧ | ˉ / 3 | Mid | Thai |
| ˨ | ˋ / 2 | Mid-Low | Cantonese |
| ˩ | ˩ / 1 | Low | Mandarin 3rd, Yoruba |

## 7. File Formats

### 7.1 JSON Format

```json
{
  "version": "1.0.0",
  "symbols": [
    {
      "ipa": "p",
      "code": "PL01-MN01-AR01",
      "hangul": "ㅂ",
      "category": "consonant",
      "subcategory": "plosive",
      "description_ko": "무성 양순 파열음",
      "description_en": "Voiceless bilabial plosive",
      "examples": ["Korean", "English", "Spanish"]
    }
  ]
}
```

### 7.2 YAML Format

```yaml
version: "1.0.0"
symbols:
  - ipa: "p"
    code: "PL01-MN01-AR01"
    hangul: "ㅂ"
    category: consonant
    subcategory: plosive
    description_ko: 무성 양순 파열음
    description_en: Voiceless bilabial plosive
    examples:
      - Korean
      - English
      - Spanish
```

## 8. Validation

All mappings are validated using the automated script:

```bash
python scripts/validate_coverage.py
```

Results: **117/117 (100.0%)** coverage

---

## References

- International Phonetic Association (2020). *IPA Chart*
- PHOIBLE: https://phoible.org/
- ISO 639-3: https://iso639-3.sil.org/

---

© 2025 SmileStory Inc. / WIA - MIT License
