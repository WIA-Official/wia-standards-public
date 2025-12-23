# WIHP Protocol Specification

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-12-23

## 1. Overview

This document defines the conversion protocol for WIHP (WIA International Hangul Phonology), specifying how IPA strings are parsed, converted, and rendered into Hangul.

## 2. Conversion Pipeline

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Input     │ → │   Parse     │ → │   Convert   │ → │   Render    │
│   (IPA)     │    │   (Tokens)  │    │   (Map)     │    │   (Hangul)  │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

### 2.1 Stage 1: Input Normalization

```rust
fn normalize_input(ipa: &str) -> String {
    // 1. Unicode NFC normalization
    // 2. Remove IPA delimiters: / [ ]
    // 3. Trim whitespace
}
```

### 2.2 Stage 2: Tokenization

```rust
fn tokenize(normalized: &str) -> Vec<IpaToken> {
    // Parse into tokens:
    // - Base symbols (consonants, vowels)
    // - Diacritics (attached to preceding symbol)
    // - Suprasegmentals (stress, length, tone)
}

enum IpaToken {
    Consonant { base: char, diacritics: Vec<char> },
    Vowel { base: char, diacritics: Vec<char> },
    Stress(StressLevel),
    Length(LengthMark),
    Tone(ToneMark),
    Boundary(BoundaryType),
}
```

### 2.3 Stage 3: Mapping

```rust
fn map_token(token: IpaToken) -> HangulSegment {
    match token {
        Consonant { base, diacritics } => {
            let hangul = CONSONANT_MAP.get(&base);
            apply_diacritics(hangul, diacritics)
        }
        Vowel { base, diacritics } => {
            let hangul = VOWEL_MAP.get(&base);
            apply_diacritics(hangul, diacritics)
        }
        // ...
    }
}
```

### 2.4 Stage 4: Rendering

```rust
fn render(segments: Vec<HangulSegment>) -> String {
    // 1. Combine jamo into syllable blocks
    // 2. Handle tone markers
    // 3. Apply stress notation
}
```

## 3. Syllable Block Formation

### 3.1 Korean Syllable Structure

```
       ┌─────────────────┐
       │    Syllable     │
       │   (음절 블록)     │
       └─────────────────┘
              │
    ┌─────────┼─────────┐
    │         │         │
┌───┴───┐ ┌───┴───┐ ┌───┴───┐
│ 초성   │ │ 중성   │ │ 종성   │
│(Initial)│ │(Medial)│ │(Final)│
│  ㄱ    │ │  ㅏ    │ │  ㄴ   │
└───────┘ └───────┘ └───────┘
            ↓
          "간"
```

### 3.2 Formation Rules

```rust
fn form_syllable_block(
    initial: Option<Jamo>,  // 초성
    medial: Jamo,           // 중성 (required)
    final_: Option<Jamo>    // 종성
) -> char {
    // Unicode Hangul syllable formula:
    // S = 0xAC00 + (initial * 21 + medial) * 28 + final
}
```

### 3.3 Example: "hello" /həˈloʊ/

```
/h/  → ㅎ (initial)
/ə/  → ㅓ (medial)      → 허
/l/  → ㄹ (initial)
/oʊ/ → ㅗ+ㅜ (medial)   → 로우
                        ─────
                        헬로우
```

## 4. Diacritic Processing

### 4.1 Attachment Rules

Diacritics attach to the immediately preceding base symbol:

```
/pʰ/  → ㅍ (aspirated p)
/tʲ/  → 티 (palatalized t)
/aː/  → 아ː (long a)
```

### 4.2 Stacking Order

When multiple diacritics appear:

```
Order: Phonation → Articulation → Length → Tone

/t̬ʲː˧/ → process as:
  1. /t/ → ㄷ
  2. /̬/ → (voiced, apply)
  3. /ʲ/ → +ㅣ
  4. /ː/ → (length marker)
  5. /˧/ → (mid tone)
```

## 5. Tone Handling

### 5.1 Tone Notation

| Method | Example | Use Case |
|--------|---------|----------|
| Numeric | ma1, ma2, ma3, ma4 | ASCII-only |
| Chao Letters | ma˥, ma˧˥ | Full Unicode |
| Diacritics | mā, má, mǎ, mà | Pinyin style |

### 5.2 WIHP Tone Rendering

```rust
fn render_tone(syllable: &str, tone: ToneMark) -> String {
    match tone {
        Tone::High => format!("{}˥", syllable),
        Tone::Rising => format!("{}ˊ", syllable),
        Tone::Low => format!("{}˩", syllable),
        // ...
    }
}
```

## 6. Stress Handling

### 6.1 Stress Markers

| IPA | WIHP | Position |
|-----|------|----------|
| ˈ | ' | Before stressed syllable |
| ˌ | , | Before secondary stress |

### 6.2 Example

```
/ˈfoʊtəˌɡræf/ → '포토,그래프
  ↑       ↑
  primary secondary
```

## 7. Special Cases

### 7.1 Affricates

Affricates are treated as single units:

```
/t͡ʃ/ → ㅊ (not ㄷ+ㅅ)
/d͡ʒ/ → ㅈ (not ㄷ+ㅈ)
```

### 7.2 Diphthongs

Diphthongs map to compound vowels:

```
/aɪ/ → ㅏㅣ or ㅐ (context-dependent)
/aʊ/ → ㅏㅜ
/oɪ/ → ㅗㅣ
```

### 7.3 Gemination

Double consonants:

```
/pː/ or /pp/ → ㅃ
/tː/ or /tt/ → ㄸ
/kː/ or /kk/ → ㄲ
```

## 8. Round-Trip Fidelity

### 8.1 Lossless Conversion

For standard IPA input:

```
IPA → WIHP → IPA = Original IPA
```

### 8.2 Information Preservation

Some fine phonetic distinctions may be neutralized:

| IPA | WIHP | Notes |
|-----|------|-------|
| /t/ vs /d/ | ㄷ | Voicing neutralized word-finally |
| /ɾ/ vs /r/ | ㄹ | Tap/trill merged |

## 9. Error Recovery

### 9.1 Unknown Symbols

```rust
enum UnknownSymbolStrategy {
    Fail,           // Return error
    Skip,           // Omit unknown symbols
    PassThrough,    // Include as-is
    Placeholder,    // Replace with □
}
```

### 9.2 Malformed Input

```rust
fn handle_malformed(input: &str) -> Result<String, Vec<Warning>> {
    // Attempt best-effort conversion
    // Collect warnings for issues
}
```

## 10. Streaming Protocol

For real-time applications:

```rust
trait WihpStream {
    fn push(&mut self, ipa_chunk: &str) -> Option<String>;
    fn flush(&mut self) -> String;
}

// Usage for TTS/ASR
let mut stream = WihpStream::new();
stream.push("/hə");     // Returns None (incomplete)
stream.push("ˈloʊ/");   // Returns Some("헬로우")
```

---

## References

- IPA Handbook (2015) - Transcription conventions
- Unicode Standard Annex #15 - Normalization

---

© 2025 SmileStory Inc. / WIA - MIT License
