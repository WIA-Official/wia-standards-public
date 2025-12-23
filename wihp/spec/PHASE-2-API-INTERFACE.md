# WIHP API Interface Specification

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-12-23

## 1. Overview

This document defines the API interface for WIHP (WIA International Hangul Phonology) implementations across multiple programming languages.

## 2. Core Functions

### 2.1 IPA to Hangul Conversion

```rust
/// Convert IPA string to WIHP Hangul representation
fn ipa_to_hangul(ipa: &str) -> Result<String, WihpError>;

/// Example:
/// ipa_to_hangul("/həˈloʊ/") => Ok("헬로우")
/// ipa_to_hangul("/ˈwɝld/") => Ok("월드")
```

### 2.2 Hangul to IPA Conversion

```rust
/// Convert WIHP Hangul back to IPA
fn hangul_to_ipa(hangul: &str) -> Result<String, WihpError>;

/// Example:
/// hangul_to_ipa("헬로우") => Ok("/həloʊ/")
```

### 2.3 Symbol Lookup

```rust
/// Look up a single IPA symbol
fn lookup_symbol(ipa_char: char) -> Option<WihpSymbol>;

/// Look up by WIHP code
fn lookup_by_code(code: &str) -> Option<WihpSymbol>;
```

## 3. Data Types

### 3.1 WihpSymbol

```rust
pub struct WihpSymbol {
    /// IPA character
    pub ipa: char,

    /// WIHP systematic code
    pub code: String,

    /// Korean Hangul representation
    pub hangul: String,

    /// Symbol category
    pub category: SymbolCategory,

    /// Detailed subcategory
    pub subcategory: String,

    /// Description in Korean
    pub description_ko: String,

    /// Description in English
    pub description_en: String,
}

pub enum SymbolCategory {
    Consonant,
    Vowel,
    Diacritic,
    Suprasegmental,
}
```

### 3.2 WihpError

```rust
pub enum WihpError {
    /// Unknown IPA symbol encountered
    UnknownSymbol(char),

    /// Invalid IPA syntax
    InvalidSyntax(String),

    /// Encoding error
    EncodingError(String),
}
```

## 4. Rust API

### 4.1 Crate Structure

```
wia-wihp/
├── Cargo.toml
└── src/
    ├── lib.rs          # Public API
    ├── consonants.rs   # 69 consonant mappings
    ├── vowels.rs       # 28 vowel mappings
    ├── diacritics.rs   # 20 diacritic mappings
    ├── convert.rs      # Conversion logic
    ├── types.rs        # Data structures
    └── error.rs        # Error types
```

### 4.2 Usage Example

```rust
use wia_wihp::{ipa_to_hangul, hangul_to_ipa, lookup_symbol};

fn main() {
    // Basic conversion
    let hangul = ipa_to_hangul("/həˈloʊ/").unwrap();
    assert_eq!(hangul, "헬로우");

    // Reverse conversion
    let ipa = hangul_to_ipa("헬로우").unwrap();
    assert_eq!(ipa, "/həloʊ/");

    // Symbol lookup
    let symbol = lookup_symbol('ʃ').unwrap();
    println!("ʃ → {}", symbol.hangul);  // "쉬"
}
```

### 4.3 Cargo.toml

```toml
[package]
name = "wia-wihp"
version = "1.0.0"
edition = "2021"
authors = ["WIA Standards"]
description = "WIHP - IPA to Hangul phonetic mapping (100% coverage)"
license = "MIT OR Apache-2.0"
repository = "https://github.com/WIA-Official/wia-standards-public"
keywords = ["ipa", "hangul", "phonetics", "accessibility", "i18n"]
categories = ["internationalization", "accessibility", "text-processing"]

[lib]
name = "wia_wihp"
path = "src/lib.rs"

[features]
default = ["std"]
std = []
no_std = []

[dependencies]
thiserror = "1.0"
lazy_static = "1.4"
unicode-normalization = "0.1"
```

## 5. TypeScript API

### 5.1 Interface

```typescript
// Types
interface WihpSymbol {
  ipa: string;
  code: string;
  hangul: string;
  category: 'consonant' | 'vowel' | 'diacritic';
  subcategory: string;
  descriptionKo: string;
  descriptionEn: string;
}

// Functions
function ipaToHangul(ipa: string): string;
function hangulToIpa(hangul: string): string;
function lookupSymbol(ipaChar: string): WihpSymbol | null;

// Usage
import { ipaToHangul } from 'wia-wihp';

const result = ipaToHangul('/həˈloʊ/');
console.log(result); // "헬로우"
```

### 5.2 Package.json

```json
{
  "name": "wia-wihp",
  "version": "1.0.0",
  "description": "WIHP - IPA to Hangul phonetic mapping",
  "main": "dist/index.js",
  "types": "dist/index.d.ts",
  "keywords": ["ipa", "hangul", "phonetics", "accessibility"],
  "license": "MIT"
}
```

## 6. Python API

### 6.1 Interface

```python
from wia_wihp import ipa_to_hangul, hangul_to_ipa, lookup_symbol

# Basic conversion
result = ipa_to_hangul("/həˈloʊ/")
assert result == "헬로우"

# Reverse conversion
ipa = hangul_to_ipa("헬로우")
assert ipa == "/həloʊ/"

# Symbol lookup
symbol = lookup_symbol("ʃ")
print(f"ʃ → {symbol.hangul}")  # "쉬"
```

### 6.2 Pyproject.toml

```toml
[project]
name = "wia-wihp"
version = "1.0.0"
description = "WIHP - IPA to Hangul phonetic mapping"
authors = [{name = "WIA Standards"}]
license = {text = "MIT"}
keywords = ["ipa", "hangul", "phonetics", "accessibility"]
classifiers = [
    "Development Status :: 5 - Production/Stable",
    "Intended Audience :: Developers",
    "Topic :: Text Processing :: Linguistic",
]
```

## 7. Error Handling

### 7.1 Unknown Symbol

```rust
// When encountering unknown IPA symbol
let result = ipa_to_hangul("/xyz/");
match result {
    Ok(hangul) => println!("{}", hangul),
    Err(WihpError::UnknownSymbol(c)) => {
        eprintln!("Unknown IPA symbol: {}", c);
    }
}
```

### 7.2 Graceful Degradation

```rust
// Option to pass through unknown symbols
let hangul = ipa_to_hangul_lenient("/həˈloʊ§/");
// Returns: "헬로우§" (unknown § passed through)
```

## 8. Performance

### 8.1 Lookup Complexity

| Operation | Time Complexity | Space Complexity |
|-----------|-----------------|------------------|
| `lookup_symbol` | O(1) | O(1) |
| `ipa_to_hangul` | O(n) | O(n) |
| `hangul_to_ipa` | O(n) | O(n) |

### 8.2 Memory Usage

- Symbol table: ~15 KB (117 symbols)
- Per-conversion: O(n) where n = input length

## 9. Thread Safety

All functions are thread-safe and can be called concurrently.

```rust
use std::thread;
use wia_wihp::ipa_to_hangul;

let handles: Vec<_> = (0..10).map(|_| {
    thread::spawn(|| {
        ipa_to_hangul("/həˈloʊ/").unwrap()
    })
}).collect();

for handle in handles {
    assert_eq!(handle.join().unwrap(), "헬로우");
}
```

---

## References

- Rust API Guidelines: https://rust-lang.github.io/api-guidelines/
- IPA Handbook (2015)

---

© 2025 SmileStory Inc. / WIA - MIT License
