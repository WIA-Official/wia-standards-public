# WIHP Integration Guide

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-12-23

## 1. Overview

This document provides integration guidelines for implementing WIHP (WIA International Hangul Phonology) in various systems including TTS, ASR, accessibility tools, and educational applications.

## 2. Use Cases

### 2.1 Text-to-Speech (TTS)

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Text      │ → │  Phonemizer  │ → │    WIHP     │
│  "Hello"    │     │   /həˈloʊ/  │     │   헬로우    │
└─────────────┘     └─────────────┘     └─────────────┘
                                              │
                                              ↓
                                    ┌─────────────────┐
                                    │   TTS Engine    │
                                    │   (Korean)      │
                                    └─────────────────┘
```

**Benefit**: Use Korean TTS engines for any language pronunciation.

### 2.2 Automatic Speech Recognition (ASR)

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Audio     │ → │  Korean ASR  │ → │    WIHP     │
│  (Speech)   │     │   헬로우     │     │  /həˈloʊ/  │
└─────────────┘     └─────────────┘     └─────────────┘
```

**Benefit**: Transcribe foreign language audio using Korean ASR.

### 2.3 Braille Display (WIA-Braille)

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│    IPA      │ → │    WIHP     │ → │  WIA-Braille │
│  /həˈloʊ/   │     │   헬로우     │     │   ⠓⠦⠇⠕⠥    │
└─────────────┘     └─────────────┘     └─────────────┘
```

**Benefit**: Unified Braille for 7,000+ language pronunciations.

### 2.4 Language Learning

```
English: Hello
IPA: /həˈloʊ/
WIHP: 헬로우
Audio: [Korean TTS pronunciation]
```

**Benefit**: Learn any language pronunciation through Korean phonetics.

## 3. Integration Patterns

### 3.1 Direct Library Integration

```rust
// Rust
use wia_wihp::ipa_to_hangul;

fn process_text(text: &str) -> String {
    let ipa = phonemize(text);  // Your phonemizer
    ipa_to_hangul(&ipa).unwrap()
}
```

```typescript
// TypeScript
import { ipaToHangul } from 'wia-wihp';

function processText(text: string): string {
    const ipa = phonemize(text);
    return ipaToHangul(ipa);
}
```

```python
# Python
from wia_wihp import ipa_to_hangul

def process_text(text: str) -> str:
    ipa = phonemize(text)
    return ipa_to_hangul(ipa)
```

### 3.2 REST API Integration

```yaml
# OpenAPI 3.0 specification
openapi: 3.0.0
info:
  title: WIHP API
  version: 1.0.0

paths:
  /convert:
    post:
      summary: Convert IPA to WIHP Hangul
      requestBody:
        content:
          application/json:
            schema:
              type: object
              properties:
                ipa:
                  type: string
                  example: "/həˈloʊ/"
      responses:
        200:
          content:
            application/json:
              schema:
                type: object
                properties:
                  hangul:
                    type: string
                    example: "헬로우"
```

### 3.3 gRPC Integration

```protobuf
syntax = "proto3";

service WihpService {
  rpc ConvertIpaToHangul(IpaRequest) returns (HangulResponse);
  rpc ConvertHangulToIpa(HangulRequest) returns (IpaResponse);
  rpc LookupSymbol(SymbolRequest) returns (SymbolResponse);
}

message IpaRequest {
  string ipa = 1;
}

message HangulResponse {
  string hangul = 1;
}
```

## 4. Platform Integrations

### 4.1 Chrome Extension

```javascript
// content.js
chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
    if (request.action === 'convertIpa') {
        const hangul = wihp.ipaToHangul(request.ipa);
        sendResponse({ hangul });
    }
});
```

### 4.2 iOS/macOS

```swift
import WiaWihp

let converter = WihpConverter()
let hangul = try converter.ipaToHangul("/həˈloʊ/")
// hangul = "헬로우"
```

### 4.3 Android

```kotlin
import com.wia.wihp.WihpConverter

val converter = WihpConverter()
val hangul = converter.ipaToHangul("/həˈloʊ/")
// hangul = "헬로우"
```

### 4.4 Web Browser (WASM)

```javascript
import init, { ipa_to_hangul } from 'wia-wihp-wasm';

async function convert(ipa) {
    await init();
    return ipa_to_hangul(ipa);
}

convert('/həˈloʊ/').then(console.log);  // "헬로우"
```

## 5. Accessibility Integration

### 5.1 Screen Readers

```html
<!-- Provide pronunciation hint -->
<span lang="en" data-wihp="헬로우">Hello</span>

<script>
// Screen reader speaks Korean pronunciation
document.querySelectorAll('[data-wihp]').forEach(el => {
    el.setAttribute('aria-label', el.dataset.wihp);
});
</script>
```

### 5.2 Braille Displays

```rust
use wia_wihp::ipa_to_hangul;
use wia_braille::hangul_to_braille;

fn ipa_to_braille(ipa: &str) -> String {
    let hangul = ipa_to_hangul(ipa)?;
    hangul_to_braille(&hangul)
}

// /həˈloʊ/ → 헬로우 → ⠓⠦⠇⠕⠥
```

### 5.3 Haptic Feedback

```rust
use wia_wihp::ipa_to_hangul;
use wia_haptic::HapticPattern;

fn ipa_to_haptic(ipa: &str) -> Vec<HapticPattern> {
    let hangul = ipa_to_hangul(ipa)?;
    hangul.chars()
        .map(|c| HapticPattern::for_hangul(c))
        .collect()
}
```

## 6. Educational Integration

### 6.1 Language Learning Apps

```typescript
interface PronunciationGuide {
    original: string;      // "Hello"
    ipa: string;           // "/həˈloʊ/"
    wihp: string;          // "헬로우"
    audioUrl: string;      // Korean TTS audio
    breakdown: Syllable[]; // Syllable-by-syllable
}

function generateGuide(word: string, language: string): PronunciationGuide {
    const ipa = phonemize(word, language);
    const wihp = ipaToHangul(ipa);
    return {
        original: word,
        ipa,
        wihp,
        audioUrl: generateKoreanTts(wihp),
        breakdown: breakIntoSyllables(wihp)
    };
}
```

### 6.2 Dictionary Integration

```json
{
    "word": "hello",
    "language": "en",
    "ipa": "/həˈloʊ/",
    "wihp": "헬로우",
    "audio": {
        "original": "https://audio.example.com/en/hello.mp3",
        "wihp": "https://audio.example.com/ko/헬로우.mp3"
    }
}
```

## 7. AI/ML Integration

### 7.1 LLM Prompt Engineering

```
System: When providing pronunciations, use WIHP format:
- Word: Hello
- IPA: /həˈloʊ/
- WIHP: 헬로우

This helps users who can read Korean but not IPA.
```

### 7.2 Speech Model Training

```python
# Training data format
training_data = [
    {"audio": "hello.wav", "ipa": "/həˈloʊ/", "wihp": "헬로우"},
    {"audio": "world.wav", "ipa": "/wɝld/", "wihp": "월드"},
    # ...
]
```

## 8. Performance Optimization

### 8.1 Caching

```rust
use std::collections::HashMap;
use lazy_static::lazy_static;

lazy_static! {
    static ref CACHE: Mutex<HashMap<String, String>> = Mutex::new(HashMap::new());
}

fn cached_ipa_to_hangul(ipa: &str) -> String {
    let mut cache = CACHE.lock().unwrap();
    if let Some(result) = cache.get(ipa) {
        return result.clone();
    }
    let result = ipa_to_hangul(ipa).unwrap();
    cache.insert(ipa.to_string(), result.clone());
    result
}
```

### 8.2 Batch Processing

```rust
fn batch_convert(ipa_list: Vec<&str>) -> Vec<String> {
    ipa_list.par_iter()  // Parallel iterator
        .map(|ipa| ipa_to_hangul(ipa).unwrap())
        .collect()
}
```

## 9. Testing

### 9.1 Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hello() {
        assert_eq!(ipa_to_hangul("/həˈloʊ/").unwrap(), "헬로우");
    }

    #[test]
    fn test_world() {
        assert_eq!(ipa_to_hangul("/wɝld/").unwrap(), "월드");
    }

    #[test]
    fn test_all_consonants() {
        // Test all 69 consonants
        for (ipa, expected) in CONSONANT_TEST_CASES {
            assert_eq!(ipa_to_hangul(ipa).unwrap(), expected);
        }
    }
}
```

### 9.2 Integration Tests

```rust
#[test]
fn test_roundtrip() {
    let original = "/həˈloʊ/";
    let hangul = ipa_to_hangul(original).unwrap();
    let back = hangul_to_ipa(&hangul).unwrap();
    assert_eq!(normalize_ipa(original), normalize_ipa(&back));
}
```

## 10. Versioning & Updates

### 10.1 Semantic Versioning

```
MAJOR.MINOR.PATCH

1.0.0 - Initial release (100% IPA coverage)
1.1.0 - Added streaming API
1.1.1 - Bug fixes
2.0.0 - Breaking API changes
```

### 10.2 Update Notifications

```rust
fn check_for_updates() -> Option<Version> {
    // Check latest version from registry
}
```

---

## References

- WIA-Braille Standard
- WIA-Haptic Standard
- Korean Language TTS Best Practices

---

© 2025 SmileStory Inc. / WIA - MIT License
