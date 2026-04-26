//! Braille conversion module
//!
//! **Philosophy: Braille is NOT an "additional feature"**
//! **Braille is a first-class citizen, EQUAL to text**
//!
//! This module provides braille conversion through multiple systems:
//! - **liblouis**: 56 language-specific braille tables
//! - **WIA Braille**: Universal IPA-based system for 7,000+ languages
//!
//! ## Language Support
//!
//! ### liblouis (56 languages)
//! - Korean (한국어), Japanese (日本語), Chinese (中文)
//! - English, French, German, Spanish, Italian, Portuguese, Dutch
//! - Russian, Polish, Czech, Hungarian, Romanian, Bulgarian
//! - Swedish, Norwegian, Danish, Finnish, Icelandic
//! - Arabic, Hebrew, Persian, Hindi, Thai, Vietnamese, Bengali, Tamil
//! - And 38+ more languages!
//!
//! ### WIA Braille (7,000+ languages)
//! - Universal IPA-based system
//! - Philosophy: 홍익인간 (Benefit All Humanity)
//! - Coverage: ALL human languages through phonetic representation

pub mod japanese;
pub mod korean;
pub mod liblouis_wrapper;
pub mod wia_braille;

pub use liblouis_wrapper::{Language, LibLouisTranslator};
pub use wia_braille::{ipa_to_wia_braille, WiaBrailleConverter};

/// Convert text to braille with explicit language selection
///
/// Uses liblouis for 56-language support via system command (lou_translate).
/// Requires liblouis-bin to be installed on the system.
pub fn text_to_braille_with_language(text: &str, language: Language) -> Result<String, String> {
    let translator = LibLouisTranslator::new();
    translator.translate(text, language)
}

/// Convert text to braille (auto-detect language)
///
/// Automatically detects Korean, Japanese, or falls back to English.
/// For explicit language selection, use `text_to_braille_with_language`.
pub fn text_to_braille(text: &str) -> String {
    // Detect language and use appropriate converter
    if text.chars().any(|c| is_korean_char(c)) {
        korean::korean_to_braille(text)
    } else if text.chars().any(|c| is_japanese_char(c)) {
        japanese::text_to_japanese_braille(text)
    } else {
        // Use ASCII braille for other text
        ascii_to_braille(text)
    }
}

/// Check if a character is Korean
fn is_korean_char(c: char) -> bool {
    matches!(c as u32, 0xAC00..=0xD7A3 | 0x1100..=0x11FF | 0x3130..=0x318F)
}

/// Check if a character is Japanese
fn is_japanese_char(c: char) -> bool {
    matches!(
        c as u32,
        0x3040..=0x309F |  // Hiragana
        0x30A0..=0x30FF |  // Katakana
        0x4E00..=0x9FAF    // Kanji (CJK Unified Ideographs)
    )
}

/// Simple ASCII to braille conversion (English Grade 1)
fn ascii_to_braille(text: &str) -> String {
    text.chars()
        .flat_map(|c| {
            let braille: &str = match c.to_ascii_lowercase() {
                'a' => "⠁",
                'b' => "⠃",
                'c' => "⠉",
                'd' => "⠙",
                'e' => "⠑",
                'f' => "⠋",
                'g' => "⠛",
                'h' => "⠓",
                'i' => "⠊",
                'j' => "⠚",
                'k' => "⠅",
                'l' => "⠇",
                'm' => "⠍",
                'n' => "⠝",
                'o' => "⠕",
                'p' => "⠏",
                'q' => "⠟",
                'r' => "⠗",
                's' => "⠎",
                't' => "⠞",
                'u' => "⠥",
                'v' => "⠧",
                'w' => "⠺",
                'x' => "⠭",
                'y' => "⠽",
                'z' => "⠵",
                '0' => "⠼⠚",
                '1' => "⠼⠁",
                '2' => "⠼⠃",
                '3' => "⠼⠉",
                '4' => "⠼⠙",
                '5' => "⠼⠑",
                '6' => "⠼⠋",
                '7' => "⠼⠛",
                '8' => "⠼⠓",
                '9' => "⠼⠊",
                ' ' => " ",
                ',' => "⠂",
                '.' => "⠲",
                '!' => "⠖",
                '?' => "⠦",
                _ => return c.to_string().chars().collect::<Vec<_>>(),
            };
            braille.chars().collect::<Vec<_>>()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_english_braille() {
        assert_eq!(text_to_braille("hello"), "⠓⠑⠇⠇⠕");
        assert_eq!(text_to_braille("world"), "⠺⠕⠗⠇⠙");
    }

    #[test]
    fn test_korean_detection() {
        assert!(is_korean_char('가'));
        assert!(is_korean_char('한'));
        assert!(!is_korean_char('a'));
    }

    #[test]
    fn test_japanese_detection() {
        assert!(is_japanese_char('あ')); // Hiragana
        assert!(is_japanese_char('ア')); // Katakana
        assert!(is_japanese_char('日')); // Kanji
        assert!(!is_japanese_char('a'));
    }

    #[test]
    fn test_japanese_braille_conversion() {
        let result = text_to_braille("こんにちは");
        assert!(!result.is_empty());
        println!("こんにちは → {}", result);
    }
}
