//! WIHP Vowel mappings (28 symbols - 100% coverage)

use lazy_static::lazy_static;
use std::collections::HashMap;
use crate::types::{WihpSymbol, SymbolCategory};

lazy_static! {
    /// IPA to Hangul vowel mapping
    pub static ref VOWEL_MAP: HashMap<char, &'static str> = {
        let mut m = HashMap::new();

        // Close vowels (고모음)
        m.insert('i', "ㅣ");
        m.insert('y', "ㅟ");
        m.insert('ɨ', "ㅡ");
        m.insert('ʉ', "ㅜ̈");
        m.insert('ɯ', "ㅡ");
        m.insert('u', "ㅜ");

        // Near-close vowels (근고모음)
        m.insert('ɪ', "ㅣ");
        m.insert('ʏ', "ㅟ");
        m.insert('ʊ', "ㅜ");

        // Close-mid vowels (중고모음)
        m.insert('e', "ㅔ");
        m.insert('ø', "ㅚ");
        m.insert('ɘ', "ㅓ");
        m.insert('ɵ', "ㅗ̈");
        m.insert('ɤ', "ㅓ");
        m.insert('o', "ㅗ");

        // Mid vowel (중모음)
        m.insert('ə', "ㅓ");

        // Open-mid vowels (중저모음)
        m.insert('ɛ', "ㅔ");
        m.insert('œ', "ㅚ");
        m.insert('ɜ', "ㅓ");
        m.insert('ɞ', "ㅓ̤");
        m.insert('ʌ', "ㅓ");
        m.insert('ɔ', "ㅗ");

        // Near-open vowels (근저모음)
        m.insert('æ', "ㅐ");
        m.insert('ɐ', "ㅏ");

        // Open vowels (저모음)
        m.insert('a', "ㅏ");
        m.insert('ɶ', "ㅚ");
        m.insert('ɑ', "ㅏ");
        m.insert('ɒ', "ㆍ");

        m
    };
}

/// Look up vowel by IPA character
pub fn lookup(ipa_char: char) -> Option<WihpSymbol> {
    VOWEL_MAP.get(&ipa_char).map(|hangul| {
        WihpSymbol::new(
            ipa_char,
            get_code(ipa_char),
            hangul,
            SymbolCategory::Vowel,
            get_subcategory(ipa_char),
            get_description_ko(ipa_char),
            get_description_en(ipa_char),
        )
    })
}

/// Look up by WIHP code
pub fn lookup_by_code(code: &str) -> Option<WihpSymbol> {
    // TODO: Implement code lookup
    None
}

/// Get all vowels
pub fn all() -> Vec<WihpSymbol> {
    VOWEL_MAP.keys().filter_map(|&ipa| lookup(ipa)).collect()
}

/// Get WIHP code for IPA character
fn get_code(ipa: char) -> &'static str {
    match ipa {
        'i' => "V01-CLOSE-FRONT-UNROUND",
        'y' => "V02-CLOSE-FRONT-ROUND",
        'ɨ' => "V03-CLOSE-CENTRAL-UNROUND",
        'ʉ' => "V04-CLOSE-CENTRAL-ROUND",
        'ɯ' => "V05-CLOSE-BACK-UNROUND",
        'u' => "V06-CLOSE-BACK-ROUND",
        'e' => "V07-CLOSEMID-FRONT-UNROUND",
        'o' => "V08-CLOSEMID-BACK-ROUND",
        'ə' => "V09-MID-CENTRAL",
        'ɛ' => "V10-OPENMID-FRONT-UNROUND",
        'ɔ' => "V11-OPENMID-BACK-ROUND",
        'a' => "V12-OPEN-FRONT",
        'ɑ' => "V13-OPEN-BACK-UNROUND",
        'ɒ' => "V14-OPEN-BACK-ROUND",
        _ => "UNKNOWN",
    }
}

/// Get subcategory (height)
fn get_subcategory(ipa: char) -> &'static str {
    match ipa {
        'i' | 'y' | 'ɨ' | 'ʉ' | 'ɯ' | 'u' => "close",
        'ɪ' | 'ʏ' | 'ʊ' => "near-close",
        'e' | 'ø' | 'ɘ' | 'ɵ' | 'ɤ' | 'o' => "close-mid",
        'ə' => "mid",
        'ɛ' | 'œ' | 'ɜ' | 'ɞ' | 'ʌ' | 'ɔ' => "open-mid",
        'æ' | 'ɐ' => "near-open",
        'a' | 'ɶ' | 'ɑ' | 'ɒ' => "open",
        _ => "unknown",
    }
}

/// Get Korean description
fn get_description_ko(ipa: char) -> &'static str {
    match ipa {
        'i' => "전설 비원순 고모음",
        'y' => "전설 원순 고모음",
        'u' => "후설 원순 고모음",
        'e' => "전설 비원순 중고모음",
        'o' => "후설 원순 중고모음",
        'ə' => "중설 중모음 (슈와)",
        'a' => "전설 저모음",
        'ɑ' => "후설 비원순 저모음",
        'æ' => "근저 전설 비원순 모음",
        'ɛ' => "전설 비원순 중저모음",
        'ɔ' => "후설 원순 중저모음",
        'ʊ' => "근고 후설 원순 모음",
        'ɪ' => "근고 전설 비원순 모음",
        _ => "모음",
    }
}

/// Get English description
fn get_description_en(ipa: char) -> &'static str {
    match ipa {
        'i' => "close front unrounded vowel",
        'y' => "close front rounded vowel",
        'u' => "close back rounded vowel",
        'e' => "close-mid front unrounded vowel",
        'o' => "close-mid back rounded vowel",
        'ə' => "mid central vowel (schwa)",
        'a' => "open front unrounded vowel",
        'ɑ' => "open back unrounded vowel",
        'æ' => "near-open front unrounded vowel",
        'ɛ' => "open-mid front unrounded vowel",
        'ɔ' => "open-mid back rounded vowel",
        'ʊ' => "near-close back rounded vowel",
        'ɪ' => "near-close front unrounded vowel",
        _ => "vowel",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vowel_count() {
        assert_eq!(VOWEL_MAP.len(), 28);
    }

    #[test]
    fn test_basic_vowels() {
        assert_eq!(VOWEL_MAP.get(&'a'), Some(&"ㅏ"));
        assert_eq!(VOWEL_MAP.get(&'e'), Some(&"ㅔ"));
        assert_eq!(VOWEL_MAP.get(&'i'), Some(&"ㅣ"));
        assert_eq!(VOWEL_MAP.get(&'o'), Some(&"ㅗ"));
        assert_eq!(VOWEL_MAP.get(&'u'), Some(&"ㅜ"));
    }

    #[test]
    fn test_schwa() {
        assert_eq!(VOWEL_MAP.get(&'ə'), Some(&"ㅓ"));
    }

    #[test]
    fn test_arae_a() {
        // ㆍ (arae-a) for open back rounded vowel
        assert_eq!(VOWEL_MAP.get(&'ɒ'), Some(&"ㆍ"));
    }
}
