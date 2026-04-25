# WIHP-Braille Integration Mapping

## Overview

IPA → WIHP Hangul → WIA-Braille 점자 통합 매핑 체계

## Triple Mapping System

```
Source Language → IPA → WIHP Code → Hangul → Braille Dots → Unicode
```

## Consonant Chain Mapping

### Stops (파열음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| p | PL01-MN01-AR01 | ㅍ | ⠏ | U+280F |
| b | PL01-MN01-AR03 | ㅂ | ⠘ | U+2818 |
| t | PL04-MN01-AR01 | ㅌ | ⠞ | U+281E |
| d | PL04-MN01-AR03 | ㄷ | ⠙ | U+2819 |
| k | PL07-MN01-AR01 | ㅋ | ⠅ | U+2805 |
| g | PL07-MN01-AR03 | ㄱ | ⠛ | U+281B |
| ʔ | PL09-MN01-AR01 | ㅇ | ⠶ | U+2836 |

### Nasals (비음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| m | PL01-MN02-AR03 | ㅁ | ⠍ | U+280D |
| n | PL04-MN02-AR03 | ㄴ | ⠝ | U+281D |
| ŋ | PL07-MN02-AR03 | ㅇ | ⠶ | U+2836 |

### Fricatives (마찰음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| f | PL02-MN03-AR01 | ㅍ | ⠏ | U+280F |
| v | PL02-MN03-AR03 | ㅂ | ⠘ | U+2818 |
| s | PL04-MN03-AR01 | ㅅ | ⠎ | U+280E |
| z | PL04-MN03-AR03 | ㅈ | ⠨ | U+2828 |
| ʃ | PL06-MN03-AR01 | ㅅ | ⠱ | U+2831 |
| h | PL09-MN03-AR01 | ㅎ | ⠓ | U+2813 |

### Affricates (파찰음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| tʃ | PL06-MN04-AR01 | ㅊ | ⠾ | U+283E |
| dʒ | PL06-MN04-AR03 | ㅈ | ⠨ | U+2828 |

### Liquids (유음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| l | PL04-MN06-AR03 | ㄹ | ⠇ | U+2807 |
| r | PL04-MN08-AR03 | ㄹ | ⠗ | U+2817 |

## Vowel Chain Mapping

### Monophthongs (단모음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| i | VL01-VH01-VR01 | ㅣ | ⠊ | U+280A |
| e | VL01-VH02-VR01 | ㅔ | ⠢ | U+2822 |
| ɛ | VL01-VH03-VR01 | ㅐ | ⠪ | U+282A |
| a | VL02-VH04-VR01 | ㅏ | ⠁ | U+2801 |
| o | VL03-VH02-VR02 | ㅗ | ⠥ | U+2825 |
| u | VL03-VH01-VR02 | ㅜ | ⠥ | U+2825 |
| ə | VL02-VH02-VR01 | ㅓ | ⠣ | U+2823 |

### Diphthongs (이중모음)
| IPA | WIHP Code | Hangul | Braille | Unicode |
|-----|-----------|--------|---------|---------|
| ai | VL02-VH04-VR01+VL01-VH01-VR01 | ㅏㅣ | ⠁⠊ | U+2801 U+280A |
| au | VL02-VH04-VR01+VL03-VH01-VR02 | ㅏㅜ | ⠁⠥ | U+2801 U+2825 |
| ei | VL01-VH02-VR01+VL01-VH01-VR01 | ㅔㅣ | ⠢⠊ | U+2822 U+280A |
| ou | VL03-VH02-VR02+VL03-VH01-VR02 | ㅗㅜ | ⠥⠥ | U+2825 U+2825 |

## Complete Syllable Examples

### Basic Syllables
| Example | IPA | WIHP Hangul | Braille | Description |
|---------|-----|-------------|---------|-------------|
| "ka" | /ka/ | 카 | ⠅⠁ | CV syllable |
| "san" | /san/ | 산 | ⠎⠁⠝ | CVC syllable |
| "kim" | /kim/ | 킴 | ⠅⠊⠍ | CVC syllable |
| "hello" | /həˈloʊ/ | 헬로 | ⠓⠢⠇⠥ | English word |
| "konnichiwa" | /konnitɕiwa/ | 콘니치와 | ⠅⠥⠝⠝⠊⠾⠊⠺⠁ | Japanese word |

### Multi-language Examples
| Language | Word | IPA | Hangul | Braille |
|----------|------|-----|--------|---------|
| English | "thank" | /θæŋk/ | 땡크 | ⠮⠪⠶⠅ |
| French | "merci" | /mɛʁsi/ | 메르시 | ⠍⠢⠗⠎⠊ |
| Spanish | "gracias" | /ˈɡɾasjas/ | 그라시아스 | ⠛⠗⠁⠎⠊⠁⠎ |
| Japanese | "arigatou" | /aɾiɡatoː/ | 아리가토 | ⠁⠗⠊⠛⠁⠞⠥ |
| Chinese | "nǐ hǎo" | /ni˧˥ xau˧˩˦/ | 니하오 | ⠝⠊⠓⠁⠥ |
| Korean | "annyeong" | /annjʌŋ/ | 안녕 | ⠁⠝⠝⠣⠶ |
| Arabic | "shukran" | /ʃukran/ | 슈크란 | ⠱⠥⠅⠗⠁⠝ |
| Hindi | "namaste" | /nəməsteː/ | 나마스테 | ⠝⠁⠍⠁⠎⠞⠢ |

## Korean Hangul to Braille Reference

### 자음 (Consonants)
| 초성 | Braille | 종성 | Braille |
|------|---------|------|---------|
| ㄱ | ⠈ | ㄱ | ⠁ |
| ㄴ | ⠉ | ㄴ | ⠒ |
| ㄷ | ⠊ | ㄷ | ⠔ |
| ㄹ | ⠐ | ㄹ | ⠂ |
| ㅁ | ⠑ | ㅁ | ⠢ |
| ㅂ | ⠘ | ㅂ | ⠃ |
| ㅅ | ⠠ | ㅅ | ⠄ |
| ㅇ | ⠛ | ㅇ | ⠶ |
| ㅈ | ⠨ | ㅈ | ⠅ |
| ㅊ | ⠰ | ㅊ | ⠆ |
| ㅋ | ⠋ | ㅋ | ⠖ |
| ㅌ | ⠓ | ㅌ | ⠦ |
| ㅍ | ⠙ | ㅍ | ⠲ |
| ㅎ | ⠚ | ㅎ | ⠴ |

### 모음 (Vowels)
| 모음 | Braille | 모음 | Braille |
|------|---------|------|---------|
| ㅏ | ⠣ | ㅑ | ⠜ |
| ㅓ | ⠎ | ㅕ | ⠱ |
| ㅗ | ⠥ | ㅛ | ⠬ |
| ㅜ | ⠍ | ㅠ | ⠹ |
| ㅡ | ⠪ | ㅣ | ⠕ |
| ㅐ | ⠗ | ㅔ | ⠝ |
| ㅚ | ⠽ | ㅟ | ⠍⠗ |
| ㅘ | ⠧ | ㅙ | ⠧⠗ |
| ㅝ | ⠻ | ㅞ | ⠻⠗ |
| ㅢ | ⠺ | | |

## Usage Applications

### 1. Multi-language Learning for Blind Users
```
Teacher: "How do you say 'hello' in Japanese?"
Student reads: ⠅⠥⠝⠝⠊⠾⠊⠺⠁ → 콘니치와 → /konnitɕiwa/
```

### 2. IPA Learning Through Touch
```
IPA: /θ/ (dental fricative)
Braille: ⠮ (dot pattern for 'th')
Description: "Tongue between teeth, blow air"
```

### 3. Cross-script Dictionary
```
Query: "water"
Results:
  English: water /ˈwɔːtər/ → 워터 → ⠺⠣⠞⠣
  Korean: 물 /mul/ → 물 → ⠑⠍⠂
  Japanese: 水 mizu /mizɯ/ → 미즈 → ⠍⠕⠨⠪
```

## Technical Integration

### API Endpoint (Planned)
```json
POST /api/convert
{
  "source": "hello",
  "source_lang": "en",
  "output": ["ipa", "hangul", "braille"]
}

Response:
{
  "ipa": "/həˈloʊ/",
  "hangul": "헬로",
  "braille": "⠓⠢⠇⠥",
  "braille_unicode": ["U+2813", "U+2822", "U+2807", "U+2825"]
}
```

### WIA-Braille Project Link
```
/home/user/wia-braille/
├── src/
│   ├── hangul-to-braille.js
│   └── braille-to-hangul.js
└── (WIHP integration pending)
```

## Future Development

1. **Real-time Converter**: Web interface for IPA→Hangul→Braille
2. **Audio Integration**: Add TTS for complete accessibility
3. **Braille Display Support**: Hardware integration
4. **Mobile App**: Touch-friendly braille learning

## References

- WIA-Braille Project: https://github.com/WIA-Official/wia-braille
- Korean Braille Standard: 한글 점자 규정 (문화체육관광부)
- Unicode Braille Patterns: U+2800 - U+28FF
