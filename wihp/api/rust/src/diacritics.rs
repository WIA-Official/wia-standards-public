//! WIHP Diacritic mappings (20 symbols - 100% coverage)

use lazy_static::lazy_static;
use std::collections::HashMap;
use crate::types::{WihpSymbol, SymbolCategory};

lazy_static! {
    /// IPA to Hangul diacritic mapping
    pub static ref DIACRITIC_MAP: HashMap<char, &'static str> = {
        let mut m = HashMap::new();

        // Phonation (발성)
        m.insert('\u{02b0}', "ㅋ/ㅌ/ㅍ/ㅊ");  // ʰ aspirated
        m.insert('\u{02bc}', "ʼ");              // ʼ ejective
        m.insert('\u{0324}', "̤");               // ̤ breathy voice
        m.insert('\u{0330}', "̰");               // ̰ creaky voice
        m.insert('\u{032c}', "");               // ̬ voiced (unmarked)

        // Secondary articulation (이차 조음)
        m.insert('\u{02b7}', "+ㅗ/ㅜ");         // ʷ labialized
        m.insert('\u{02b2}', "+ㅣ");            // ʲ palatalized
        m.insert('\u{02e0}', "");               // ˠ velarized (unmarked)
        m.insert('\u{02e4}', "ˤ");              // ˤ pharyngealized
        m.insert('\u{031a}', "받침");           // ̚ unreleased

        // Voicing (유무성)
        m.insert('\u{030a}', "̊");               // ̊ voiceless

        // Syllabicity (음절성)
        m.insert('\u{0329}', "+ㅡ");            // ̩ syllabic
        m.insert('\u{032f}', "");               // ̯ non-syllabic

        // Nasalization (비음화)
        m.insert('\u{0303}', "ㄴ/ㅇ");          // ̃ nasalized

        // Length (장단)
        m.insert('\u{02d0}', "ː");              // ː long

        // Tone (성조)
        m.insert('\u{02e5}', "˥");              // ˥ high tone
        m.insert('\u{02e6}', "ˊ");              // ˦ mid-high tone
        m.insert('\u{02e7}', "ˉ");              // ˧ mid tone
        m.insert('\u{02e8}', "ˋ");              // ˨ mid-low tone
        m.insert('\u{02e9}', "˩");              // ˩ low tone

        m
    };
}

/// Look up diacritic by IPA character
pub fn lookup(ipa_char: char) -> Option<WihpSymbol> {
    DIACRITIC_MAP.get(&ipa_char).map(|hangul| {
        WihpSymbol::new(
            ipa_char,
            get_code(ipa_char),
            hangul,
            SymbolCategory::Diacritic,
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

/// Get all diacritics
pub fn all() -> Vec<WihpSymbol> {
    DIACRITIC_MAP.keys().filter_map(|&ipa| lookup(ipa)).collect()
}

/// Get WIHP code for IPA character
fn get_code(ipa: char) -> &'static str {
    match ipa {
        '\u{02b0}' => "DIA-ASPIRATION",
        '\u{02bc}' => "DIA-EJECTIVE",
        '\u{02b7}' => "DIA-LABIALIZED",
        '\u{02b2}' => "DIA-PALATALIZED",
        '\u{02e4}' => "DIA-PHARYNGEALIZED",
        '\u{031a}' => "DIA-UNRELEASED",
        '\u{0303}' => "DIA-NASALIZED",
        '\u{02d0}' => "DIA-LONG",
        '\u{02e5}' => "TONE-HIGH",
        '\u{02e6}' => "TONE-MIDHIGH",
        '\u{02e7}' => "TONE-MID",
        '\u{02e8}' => "TONE-MIDLOW",
        '\u{02e9}' => "TONE-LOW",
        _ => "UNKNOWN",
    }
}

/// Get subcategory
fn get_subcategory(ipa: char) -> &'static str {
    match ipa {
        '\u{02b0}' | '\u{02bc}' | '\u{0324}' | '\u{0330}' | '\u{032c}' => "phonation",
        '\u{02b7}' | '\u{02b2}' | '\u{02e0}' | '\u{02e4}' | '\u{031a}' => "articulation",
        '\u{030a}' => "voicing",
        '\u{0329}' | '\u{032f}' => "syllabicity",
        '\u{0303}' => "nasalization",
        '\u{02d0}' => "length",
        '\u{02e5}' | '\u{02e6}' | '\u{02e7}' | '\u{02e8}' | '\u{02e9}' => "tone",
        _ => "unknown",
    }
}

/// Get Korean description
fn get_description_ko(ipa: char) -> &'static str {
    match ipa {
        '\u{02b0}' => "유기음 (기식음)",
        '\u{02bc}' => "방출음",
        '\u{0324}' => "기식성",
        '\u{0330}' => "삐걱성",
        '\u{032c}' => "유성화",
        '\u{02b7}' => "원순화",
        '\u{02b2}' => "구개음화",
        '\u{02e0}' => "연구개화",
        '\u{02e4}' => "인두화",
        '\u{031a}' => "불파음",
        '\u{030a}' => "무성화",
        '\u{0329}' => "음절성",
        '\u{032f}' => "비음절성",
        '\u{0303}' => "비음화",
        '\u{02d0}' => "장음",
        '\u{02e5}' => "고조 (성조 5)",
        '\u{02e6}' => "중고조 (성조 4)",
        '\u{02e7}' => "중조 (성조 3)",
        '\u{02e8}' => "중저조 (성조 2)",
        '\u{02e9}' => "저조 (성조 1)",
        _ => "다이아크리틱",
    }
}

/// Get English description
fn get_description_en(ipa: char) -> &'static str {
    match ipa {
        '\u{02b0}' => "aspirated",
        '\u{02bc}' => "ejective",
        '\u{0324}' => "breathy voice",
        '\u{0330}' => "creaky voice",
        '\u{032c}' => "voiced",
        '\u{02b7}' => "labialized",
        '\u{02b2}' => "palatalized",
        '\u{02e0}' => "velarized",
        '\u{02e4}' => "pharyngealized",
        '\u{031a}' => "unreleased",
        '\u{030a}' => "voiceless",
        '\u{0329}' => "syllabic",
        '\u{032f}' => "non-syllabic",
        '\u{0303}' => "nasalized",
        '\u{02d0}' => "long",
        '\u{02e5}' => "high tone",
        '\u{02e6}' => "mid-high tone",
        '\u{02e7}' => "mid tone",
        '\u{02e8}' => "mid-low tone",
        '\u{02e9}' => "low tone",
        _ => "diacritic",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_diacritic_count() {
        assert_eq!(DIACRITIC_MAP.len(), 20);
    }

    #[test]
    fn test_aspiration() {
        assert_eq!(DIACRITIC_MAP.get(&'\u{02b0}'), Some(&"ㅋ/ㅌ/ㅍ/ㅊ"));
    }

    #[test]
    fn test_tones() {
        assert_eq!(DIACRITIC_MAP.get(&'\u{02e5}'), Some(&"˥"));
        assert_eq!(DIACRITIC_MAP.get(&'\u{02e9}'), Some(&"˩"));
    }

    #[test]
    fn test_long() {
        assert_eq!(DIACRITIC_MAP.get(&'\u{02d0}'), Some(&"ː"));
    }
}
