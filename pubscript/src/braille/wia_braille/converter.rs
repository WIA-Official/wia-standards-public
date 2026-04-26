//! IPA to WIA Braille Converter
//!
//! Converts International Phonetic Alphabet (IPA) transcriptions
//! to WIA Braille patterns, enabling universal language support.

use std::collections::HashMap;

/// WIA Braille Converter
///
/// Converts IPA symbols to braille patterns based on the
/// WIA Braille Universal Phonetic Mapping.
pub struct WiaBrailleConverter {
    /// Mapping from IPA symbols to braille patterns
    mapping: HashMap<String, String>,
}

impl Default for WiaBrailleConverter {
    fn default() -> Self {
        Self::new()
    }
}

impl WiaBrailleConverter {
    /// Create a new WIA Braille converter with built-in mappings
    pub fn new() -> Self {
        let mut mapping = HashMap::new();

        // VOWELS
        mapping.insert("i".to_string(), "⠊".to_string());
        mapping.insert("y".to_string(), "⠽".to_string());
        mapping.insert("ɨ".to_string(), "⠌".to_string());
        mapping.insert("ʉ".to_string(), "⠳".to_string());
        mapping.insert("ɯ".to_string(), "⠍⠥".to_string());
        mapping.insert("u".to_string(), "⠥".to_string());
        mapping.insert("ɪ".to_string(), "⠊⠄".to_string());
        mapping.insert("ʏ".to_string(), "⠽⠄".to_string());
        mapping.insert("ʊ".to_string(), "⠥⠄".to_string());
        mapping.insert("e".to_string(), "⠑".to_string());
        mapping.insert("ø".to_string(), "⠪".to_string());
        mapping.insert("ɘ".to_string(), "⠢".to_string());
        mapping.insert("ɵ".to_string(), "⠹".to_string());
        mapping.insert("ɤ".to_string(), "⠔".to_string());
        mapping.insert("o".to_string(), "⠕".to_string());
        mapping.insert("ə".to_string(), "⠜".to_string());
        mapping.insert("ɛ".to_string(), "⠣".to_string());
        mapping.insert("œ".to_string(), "⠪⠑".to_string());
        mapping.insert("ɜ".to_string(), "⠻".to_string());
        mapping.insert("ɞ".to_string(), "⠣⠕".to_string());
        mapping.insert("ʌ".to_string(), "⠡".to_string());
        mapping.insert("ɔ".to_string(), "⠪⠕".to_string());
        mapping.insert("æ".to_string(), "⠁⠑".to_string());
        mapping.insert("ɐ".to_string(), "⠁⠄".to_string());
        mapping.insert("a".to_string(), "⠁".to_string());
        mapping.insert("ɶ".to_string(), "⠪⠁".to_string());
        mapping.insert("ɑ".to_string(), "⠡⠁".to_string());
        mapping.insert("ɒ".to_string(), "⠪⠁⠕".to_string());

        // CONSONANTS
        mapping.insert("p".to_string(), "⠏".to_string());
        mapping.insert("b".to_string(), "⠃".to_string());
        mapping.insert("t".to_string(), "⠞".to_string());
        mapping.insert("d".to_string(), "⠙".to_string());
        mapping.insert("ʈ".to_string(), "⠞⠄".to_string());
        mapping.insert("ɖ".to_string(), "⠙⠄".to_string());
        mapping.insert("c".to_string(), "⠉".to_string());
        mapping.insert("ɟ".to_string(), "⠚⠄".to_string());
        mapping.insert("k".to_string(), "⠅".to_string());
        mapping.insert("g".to_string(), "⠛".to_string());
        mapping.insert("q".to_string(), "⠟".to_string());
        mapping.insert("ɢ".to_string(), "⠛⠄".to_string());
        mapping.insert("ʔ".to_string(), "⠦".to_string());
        mapping.insert("m".to_string(), "⠍".to_string());
        mapping.insert("ɱ".to_string(), "⠍⠄".to_string());
        mapping.insert("n".to_string(), "⠝".to_string());
        mapping.insert("ɳ".to_string(), "⠝⠄".to_string());
        mapping.insert("ɲ".to_string(), "⠝⠽".to_string());
        mapping.insert("ŋ".to_string(), "⠝⠛".to_string());
        mapping.insert("ɴ".to_string(), "⠝⠶".to_string());
        mapping.insert("ʙ".to_string(), "⠃⠗".to_string());
        mapping.insert("r".to_string(), "⠗".to_string());
        mapping.insert("ʀ".to_string(), "⠗⠶".to_string());
        mapping.insert("ⱱ".to_string(), "⠧⠄".to_string());
        mapping.insert("ɾ".to_string(), "⠗⠄".to_string());
        mapping.insert("ɽ".to_string(), "⠗⠲".to_string());
        mapping.insert("ɸ".to_string(), "⠋⠄".to_string());
        mapping.insert("β".to_string(), "⠃⠧".to_string());
        mapping.insert("f".to_string(), "⠋".to_string());
        mapping.insert("v".to_string(), "⠧".to_string());
        mapping.insert("θ".to_string(), "⠹⠄".to_string());
        mapping.insert("ð".to_string(), "⠙⠓".to_string());
        mapping.insert("s".to_string(), "⠎".to_string());
        mapping.insert("z".to_string(), "⠵".to_string());
        mapping.insert("ʃ".to_string(), "⠩".to_string());
        mapping.insert("ʒ".to_string(), "⠵⠓".to_string());
        mapping.insert("ʂ".to_string(), "⠎⠄".to_string());
        mapping.insert("ʐ".to_string(), "⠵⠄".to_string());
        mapping.insert("ç".to_string(), "⠉⠄".to_string());
        mapping.insert("ʝ".to_string(), "⠚⠧".to_string());
        mapping.insert("x".to_string(), "⠭".to_string());
        mapping.insert("ɣ".to_string(), "⠛⠓".to_string());
        mapping.insert("χ".to_string(), "⠭⠄".to_string());
        mapping.insert("ʁ".to_string(), "⠗⠓".to_string());
        mapping.insert("ħ".to_string(), "⠓⠄".to_string());
        mapping.insert("ʕ".to_string(), "⠦⠄".to_string());
        mapping.insert("h".to_string(), "⠓".to_string());
        mapping.insert("ɦ".to_string(), "⠓⠦".to_string());
        mapping.insert("ɬ".to_string(), "⠇⠄".to_string());
        mapping.insert("ɮ".to_string(), "⠇⠵".to_string());
        mapping.insert("ʋ".to_string(), "⠧⠥".to_string());
        mapping.insert("ɹ".to_string(), "⠗⠺".to_string());
        mapping.insert("ɻ".to_string(), "⠗⠻".to_string());
        mapping.insert("j".to_string(), "⠚".to_string());
        mapping.insert("ɰ".to_string(), "⠍⠺".to_string());
        mapping.insert("l".to_string(), "⠇".to_string());
        mapping.insert("ɭ".to_string(), "⠇⠄".to_string());
        mapping.insert("ʎ".to_string(), "⠇⠽".to_string());
        mapping.insert("ʟ".to_string(), "⠇⠶".to_string());
        mapping.insert("w".to_string(), "⠺".to_string());
        mapping.insert("ʍ".to_string(), "⠺⠓".to_string());
        mapping.insert("ɥ".to_string(), "⠓⠽".to_string());

        // DIACRITICS
        mapping.insert("ʰ".to_string(), "⠈⠓".to_string());
        mapping.insert("ʷ".to_string(), "⠈⠺".to_string());
        mapping.insert("ʲ".to_string(), "⠈⠚".to_string());
        mapping.insert("ˠ".to_string(), "⠈⠛".to_string());
        mapping.insert("ˤ".to_string(), "⠈⠦".to_string());
        mapping.insert("ⁿ".to_string(), "⠈⠝".to_string());
        mapping.insert("ˡ".to_string(), "⠈⠇".to_string());

        // SUPRASEGMENTALS
        mapping.insert("ˈ".to_string(), "⠄".to_string());
        mapping.insert("ˌ".to_string(), "⠠⠄".to_string());
        mapping.insert("ː".to_string(), "⠒".to_string());
        mapping.insert("ˑ".to_string(), "⠐".to_string());
        mapping.insert("|".to_string(), "⠳⠄".to_string());
        mapping.insert("‖".to_string(), "⠳⠳".to_string());
        mapping.insert(".".to_string(), "⠲".to_string());
        mapping.insert("‿".to_string(), "⠤".to_string());

        // TONES
        mapping.insert("˥".to_string(), "⠁⠂".to_string());
        mapping.insert("˦".to_string(), "⠁⠆".to_string());
        mapping.insert("˧".to_string(), "⠁⠒".to_string());
        mapping.insert("˨".to_string(), "⠁⠢".to_string());
        mapping.insert("˩".to_string(), "⠁⠖".to_string());
        mapping.insert("ꜛ".to_string(), "⠘⠁".to_string());
        mapping.insert("ꜜ".to_string(), "⠘⠂".to_string());

        // SPECIAL CHARACTERS
        mapping.insert(" ".to_string(), " ".to_string());
        mapping.insert("/".to_string(), "".to_string()); // IPA delimiters are removed
        mapping.insert("[".to_string(), "".to_string());
        mapping.insert("]".to_string(), "".to_string());

        WiaBrailleConverter { mapping }
    }

    /// Convert IPA text to WIA Braille
    ///
    /// # Arguments
    ///
    /// * `ipa` - IPA transcription (with or without slashes)
    ///
    /// # Returns
    ///
    /// WIA Braille representation
    ///
    /// # Example
    ///
    /// ```
    /// use wia_pubscript::braille::wia_braille::WiaBrailleConverter;
    ///
    /// let converter = WiaBrailleConverter::new();
    /// let braille = converter.convert("/hello/");
    /// assert!(!braille.is_empty());
    /// ```
    pub fn convert(&self, ipa: &str) -> String {
        let mut result = String::new();
        let mut chars = ipa.chars().peekable();

        while let Some(ch) = chars.next() {
            // Try to match multi-character IPA symbols first
            let mut matched = false;

            // Check for 3-character sequences
            if let Some(&next1) = chars.peek() {
                let mut temp_chars = chars.clone();
                temp_chars.next();
                if let Some(&next2) = temp_chars.peek() {
                    let three_char = format!("{}{}{}", ch, next1, next2);
                    if let Some(braille) = self.mapping.get(&three_char) {
                        result.push_str(braille);
                        chars.next();
                        chars.next();
                        matched = true;
                    }
                }
            }

            // Check for 2-character sequences
            if !matched {
                if let Some(&next) = chars.peek() {
                    let two_char = format!("{}{}", ch, next);
                    if let Some(braille) = self.mapping.get(&two_char) {
                        result.push_str(braille);
                        chars.next();
                        matched = true;
                    }
                }
            }

            // Check for single character
            if !matched {
                let single_char = ch.to_string();
                if let Some(braille) = self.mapping.get(&single_char) {
                    result.push_str(braille);
                } else if ch.is_alphabetic() {
                    // Fallback: if not in mapping, keep as-is
                    result.push(ch);
                }
                // Skip non-alphabetic characters not in mapping
            }
        }

        result
    }

    /// Get the number of supported IPA symbols
    pub fn symbol_count(&self) -> usize {
        self.mapping.len()
    }

    /// Check if an IPA symbol is supported
    pub fn supports(&self, symbol: &str) -> bool {
        self.mapping.contains_key(symbol)
    }
}

/// Convert IPA text to WIA Braille (convenience function)
///
/// # Arguments
///
/// * `ipa` - IPA transcription
///
/// # Returns
///
/// WIA Braille representation
///
/// # Example
///
/// ```
/// use wia_pubscript::braille::wia_braille::ipa_to_wia_braille;
///
/// // Korean: 안녕하세요
/// let korean = ipa_to_wia_braille("/annjʌŋhasʰejo/");
///
/// // Arabic: السلام عليكم
/// let arabic = ipa_to_wia_braille("/asːalaːmu ʕalajkum/");
///
/// // Any language works!
/// ```
pub fn ipa_to_wia_braille(ipa: &str) -> String {
    let converter = WiaBrailleConverter::new();
    converter.convert(ipa)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_converter_creation() {
        let converter = WiaBrailleConverter::new();
        assert!(converter.symbol_count() > 0);
    }

    #[test]
    fn test_basic_vowels() {
        let converter = WiaBrailleConverter::new();

        assert_eq!(converter.convert("a"), "⠁");
        assert_eq!(converter.convert("e"), "⠑");
        assert_eq!(converter.convert("i"), "⠊");
        assert_eq!(converter.convert("o"), "⠕");
        assert_eq!(converter.convert("u"), "⠥");
    }

    #[test]
    fn test_basic_consonants() {
        let converter = WiaBrailleConverter::new();

        assert_eq!(converter.convert("p"), "⠏");
        assert_eq!(converter.convert("t"), "⠞");
        assert_eq!(converter.convert("k"), "⠅");
        assert_eq!(converter.convert("m"), "⠍");
        assert_eq!(converter.convert("n"), "⠝");
    }

    #[test]
    fn test_korean() {
        let converter = WiaBrailleConverter::new();

        // 안녕하세요 (annyeonghaseyo)
        let result = converter.convert("/annjʌŋhasʰejo/");
        assert!(!result.is_empty());
        assert!(result.contains("⠁")); // 'a'
        assert!(result.contains("⠝")); // 'n'
    }

    #[test]
    fn test_arabic() {
        let converter = WiaBrailleConverter::new();

        // السلام (salam)
        let result = converter.convert("/salaːm/");
        assert!(!result.is_empty());
        assert!(result.contains("⠎")); // 's'
        assert!(result.contains("⠁")); // 'a'
        assert!(result.contains("⠇")); // 'l'
    }

    #[test]
    fn test_swahili() {
        let converter = WiaBrailleConverter::new();

        // Habari (habari)
        let result = converter.convert("/habaɾi/");
        assert!(!result.is_empty());
        assert!(result.contains("⠓")); // 'h'
        assert!(result.contains("⠁")); // 'a'
    }

    #[test]
    fn test_multilingual() {
        // Test multiple languages in sequence
        let languages = vec![
            ("/hello/", "English"),
            ("/annjʌŋ/", "Korean"),
            ("/salaːm/", "Arabic"),
            ("/habaɾi/", "Swahili"),
            ("/nihau˨˩˦/", "Mandarin"),
        ];

        for (ipa, lang) in languages {
            let result = ipa_to_wia_braille(ipa);
            assert!(!result.is_empty(), "Failed for {}", lang);
        }
    }

    #[test]
    fn test_special_ipa_symbols() {
        let converter = WiaBrailleConverter::new();

        // Test stress markers
        assert!(converter.supports("ˈ")); // primary stress
        assert!(converter.supports("ˌ")); // secondary stress

        // Test length markers
        assert!(converter.supports("ː")); // long

        // Test tones
        assert!(converter.supports("˥")); // high tone
        assert!(converter.supports("˩")); // low tone
    }

    #[test]
    fn test_philosophy() {
        // Verify we can handle diverse languages
        let test_cases = vec![
            "/annjʌŋ/",      // Korean
            "/nihau/",        // Mandarin
            "/salaːm/",       // Arabic
            "/jambo/",        // Swahili
            "/aloha/",        // Hawaiian
            "/ɡamarʤoba/",    // Georgian
        ];

        for ipa in test_cases {
            let result = ipa_to_wia_braille(ipa);
            assert!(!result.is_empty(), "Failed to convert: {}", ipa);
        }
    }
}
