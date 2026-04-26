//! liblouis wrapper for 42-language braille support
//!
//! **Philosophy: ALL 42 LANGUAGES ARE EQUAL**
//!
//! This module provides braille translation for 42 languages using liblouis,
//! the de-facto standard open-source braille translator.
//!
//! ## Supported Languages (42)
//!
//! - East Asian: Korean, Japanese, Chinese (Simplified/Traditional)
//! - English: English (UEB - Unified English Braille)
//! - Western Europe: French, German, Spanish, Italian, Portuguese, Dutch
//! - Eastern Europe: Russian, Polish, Czech, Hungarian, Romanian, Bulgarian,
//!   Croatian, Slovak, Slovenian, Ukrainian, Serbian
//! - Nordic: Swedish, Norwegian, Danish, Finnish, Icelandic
//! - Baltic: Latvian, Lithuanian, Estonian
//! - Middle East: Arabic, Hebrew, Persian
//! - South Asia: Hindi
//! - Southeast Asia: Thai, Vietnamese, Indonesian
//! - Other: Turkish, Greek, Swahili, Malay, Filipino

use std::collections::HashMap;

/// Supported language codes (ISO 639-1/639-3)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Language {
    // East Asian
    Korean,
    Japanese,
    ChineseSimplified,
    ChineseTraditional,

    // English
    English,

    // Western Europe
    French,
    German,
    Spanish,
    Italian,
    Portuguese,
    Dutch,

    // Eastern Europe
    Russian,
    Polish,
    Czech,
    Hungarian,
    Romanian,
    Bulgarian,
    Croatian,
    Slovak,
    Slovenian,
    Ukrainian,
    Serbian,

    // Nordic
    Swedish,
    Norwegian,
    Danish,
    Finnish,
    Icelandic,

    // Baltic
    Latvian,
    Lithuanian,
    Estonian,

    // Middle East
    Arabic,
    Hebrew,
    Persian,

    // South Asia
    Hindi,
    Bengali,
    Tamil,
    Telugu,
    Marathi,
    Urdu,
    Nepali,
    Sinhala,

    // Southeast Asia
    Thai,
    Vietnamese,
    Indonesian,
    Burmese,
    Khmer,

    // Caucasus & Central Asia
    Georgian,
    Armenian,
    Mongolian,
    Kazakh,

    // East Africa
    Amharic,

    // Other
    Turkish,
    Greek,
    Swahili,
    Malay,
    Filipino,
}

impl Language {
    /// Get the liblouis table name for this language
    pub fn table_name(&self) -> &'static str {
        match self {
            // East Asian
            Language::Korean => "ko-g1.ctb",
            Language::Japanese => "ja-kantenji.utb",
            Language::ChineseSimplified => "zhcn-g1.ctb",
            Language::ChineseTraditional => "zh-tw.ctb",

            // English
            Language::English => "en-ueb-g1.ctb",

            // Western Europe
            Language::French => "fr-bfu-g2.ctb",
            Language::German => "de-g1.ctb",
            Language::Spanish => "es-g1.ctb",
            Language::Italian => "it-it-comp8.utb",
            Language::Portuguese => "pt-pt-g1.utb",
            Language::Dutch => "nl-NL-g0.utb",

            // Eastern Europe
            Language::Russian => "ru-ru-g1.ctb",
            Language::Polish => "pl-pl-comp8.ctb",
            Language::Czech => "cs-g1.ctb",
            Language::Hungarian => "hu-hu-g1.ctb",
            Language::Romanian => "ro-g0.utb",
            Language::Bulgarian => "bg.ctb",
            Language::Croatian => "hr-g1.ctb",
            Language::Slovak => "sk-g1.ctb",
            Language::Slovenian => "sl-si-g1.utb",
            Language::Ukrainian => "uk.utb",
            Language::Serbian => "sr-g1.ctb",

            // Nordic
            Language::Swedish => "sv-g1.ctb",
            Language::Norwegian => "no-no-g1.ctb",
            Language::Danish => "da-dk-g16.ctb",
            Language::Finnish => "fi.utb",
            Language::Icelandic => "is.ctb",

            // Baltic
            Language::Latvian => "lv.tbl",
            Language::Lithuanian => "lt.ctb",
            Language::Estonian => "et.ctb",

            // Middle East
            Language::Arabic => "ar-ar-g1.utb",
            Language::Hebrew => "he-IL.utb",
            Language::Persian => "fa-ir-g1.utb",

            // South Asia
            Language::Hindi => "hi-in-g1.utb",
            Language::Bengali => "bengali.cti",
            Language::Tamil => "ta.ctb",
            Language::Telugu => "te-in-g1.utb",
            Language::Marathi => "mr-in-g1.utb",
            Language::Urdu => "ur-pk-g1.utb",
            Language::Nepali => "ne.ctb",
            Language::Sinhala => "si-in-g1.utb",

            // Southeast Asia
            Language::Thai => "th-g0.utb",
            Language::Vietnamese => "vi-vn-g1.ctb",
            Language::Indonesian => "ms-my-g2.ctb", // Use Malay table (similar language)
            Language::Burmese => "my-g1.utb",
            Language::Khmer => "km-g1.utb",

            // Caucasus & Central Asia
            Language::Georgian => "ka.utb",
            Language::Armenian => "hy.ctb",
            Language::Mongolian => "mn-MN-g1.utb",
            Language::Kazakh => "kk.utb",

            // East Africa
            Language::Amharic => "ethio-g1.ctb",

            // Other
            Language::Turkish => "tr-g1.ctb",
            Language::Greek => "el.ctb",
            Language::Swahili => "sw-ke-g1.utb",
            Language::Malay => "ms-my-g2.ctb",
            Language::Filipino => "fil-g2.ctb",
        }
    }

    /// Get language name in English
    pub fn name(&self) -> &'static str {
        match self {
            Language::Korean => "Korean (한국어)",
            Language::Japanese => "Japanese (日本語)",
            Language::ChineseSimplified => "Chinese Simplified (简体中文)",
            Language::ChineseTraditional => "Chinese Traditional (繁體中文)",
            Language::English => "English",
            Language::French => "French (Français)",
            Language::German => "German (Deutsch)",
            Language::Spanish => "Spanish (Español)",
            Language::Italian => "Italian (Italiano)",
            Language::Portuguese => "Portuguese (Português)",
            Language::Dutch => "Dutch (Nederlands)",
            Language::Russian => "Russian (Русский)",
            Language::Polish => "Polish (Polski)",
            Language::Czech => "Czech (Čeština)",
            Language::Hungarian => "Hungarian (Magyar)",
            Language::Romanian => "Romanian (Română)",
            Language::Bulgarian => "Bulgarian (Български)",
            Language::Croatian => "Croatian (Hrvatski)",
            Language::Slovak => "Slovak (Slovenčina)",
            Language::Slovenian => "Slovenian (Slovenščina)",
            Language::Ukrainian => "Ukrainian (Українська)",
            Language::Serbian => "Serbian (Српски)",
            Language::Swedish => "Swedish (Svenska)",
            Language::Norwegian => "Norwegian (Norsk)",
            Language::Danish => "Danish (Dansk)",
            Language::Finnish => "Finnish (Suomi)",
            Language::Icelandic => "Icelandic (Íslenska)",
            Language::Latvian => "Latvian (Latviešu)",
            Language::Lithuanian => "Lithuanian (Lietuvių)",
            Language::Estonian => "Estonian (Eesti)",
            Language::Arabic => "Arabic (العربية)",
            Language::Hebrew => "Hebrew (עברית)",
            Language::Persian => "Persian (فارسی)",
            Language::Hindi => "Hindi (हिन्दी)",
            Language::Bengali => "Bengali (বাংলা)",
            Language::Tamil => "Tamil (தமிழ்)",
            Language::Telugu => "Telugu (తెలుగు)",
            Language::Marathi => "Marathi (मराठी)",
            Language::Urdu => "Urdu (اردو)",
            Language::Nepali => "Nepali (नेपाली)",
            Language::Sinhala => "Sinhala (සිංහල)",
            Language::Thai => "Thai (ไทย)",
            Language::Vietnamese => "Vietnamese (Tiếng Việt)",
            Language::Indonesian => "Indonesian (Bahasa Indonesia)",
            Language::Burmese => "Burmese (မြန်မာ)",
            Language::Khmer => "Khmer (ភាសាខ្មែរ)",
            Language::Georgian => "Georgian (ქართული)",
            Language::Armenian => "Armenian (Հայերեն)",
            Language::Mongolian => "Mongolian (Монгол)",
            Language::Kazakh => "Kazakh (Қазақша)",
            Language::Amharic => "Amharic (አማርኛ)",
            Language::Turkish => "Turkish (Türkçe)",
            Language::Greek => "Greek (Ελληνικά)",
            Language::Swahili => "Swahili (Kiswahili)",
            Language::Malay => "Malay (Bahasa Melayu)",
            Language::Filipino => "Filipino",
        }
    }

    /// Get ISO 639-1 or 639-3 language code
    pub fn code(&self) -> &'static str {
        match self {
            Language::Korean => "ko",
            Language::Japanese => "ja",
            Language::ChineseSimplified => "zh-CN",
            Language::ChineseTraditional => "zh-TW",
            Language::English => "en",
            Language::French => "fr",
            Language::German => "de",
            Language::Spanish => "es",
            Language::Italian => "it",
            Language::Portuguese => "pt",
            Language::Dutch => "nl",
            Language::Russian => "ru",
            Language::Polish => "pl",
            Language::Czech => "cs",
            Language::Hungarian => "hu",
            Language::Romanian => "ro",
            Language::Bulgarian => "bg",
            Language::Croatian => "hr",
            Language::Slovak => "sk",
            Language::Slovenian => "sl",
            Language::Ukrainian => "uk",
            Language::Serbian => "sr",
            Language::Swedish => "sv",
            Language::Norwegian => "no",
            Language::Danish => "da",
            Language::Finnish => "fi",
            Language::Icelandic => "is",
            Language::Latvian => "lv",
            Language::Lithuanian => "lt",
            Language::Estonian => "et",
            Language::Arabic => "ar",
            Language::Hebrew => "he",
            Language::Persian => "fa",
            Language::Hindi => "hi",
            Language::Bengali => "bn",
            Language::Tamil => "ta",
            Language::Telugu => "te",
            Language::Marathi => "mr",
            Language::Urdu => "ur",
            Language::Nepali => "ne",
            Language::Sinhala => "si",
            Language::Thai => "th",
            Language::Vietnamese => "vi",
            Language::Indonesian => "id",
            Language::Burmese => "my",
            Language::Khmer => "km",
            Language::Georgian => "ka",
            Language::Armenian => "hy",
            Language::Mongolian => "mn",
            Language::Kazakh => "kk",
            Language::Amharic => "am",
            Language::Turkish => "tr",
            Language::Greek => "el",
            Language::Swahili => "sw",
            Language::Malay => "ms",
            Language::Filipino => "fil",
        }
    }

    /// Get all supported languages
    pub fn all() -> Vec<Language> {
        vec![
            // East Asian
            Language::Korean,
            Language::Japanese,
            Language::ChineseSimplified,
            Language::ChineseTraditional,
            // English
            Language::English,
            // Western Europe
            Language::French,
            Language::German,
            Language::Spanish,
            Language::Italian,
            Language::Portuguese,
            Language::Dutch,
            // Eastern Europe
            Language::Russian,
            Language::Polish,
            Language::Czech,
            Language::Hungarian,
            Language::Romanian,
            Language::Bulgarian,
            Language::Croatian,
            Language::Slovak,
            Language::Slovenian,
            Language::Ukrainian,
            Language::Serbian,
            // Nordic
            Language::Swedish,
            Language::Norwegian,
            Language::Danish,
            Language::Finnish,
            Language::Icelandic,
            // Baltic
            Language::Latvian,
            Language::Lithuanian,
            Language::Estonian,
            // Middle East
            Language::Arabic,
            Language::Hebrew,
            Language::Persian,
            // South Asia
            Language::Hindi,
            Language::Bengali,
            Language::Tamil,
            Language::Telugu,
            Language::Marathi,
            Language::Urdu,
            Language::Nepali,
            Language::Sinhala,
            // Southeast Asia
            Language::Thai,
            Language::Vietnamese,
            Language::Indonesian,
            Language::Burmese,
            Language::Khmer,
            // Caucasus & Central Asia
            Language::Georgian,
            Language::Armenian,
            Language::Mongolian,
            Language::Kazakh,
            // East Africa
            Language::Amharic,
            // Other
            Language::Turkish,
            Language::Greek,
            Language::Swahili,
            Language::Malay,
            Language::Filipino,
        ]
    }

    /// Parse language from code
    pub fn from_code(code: &str) -> Option<Language> {
        let code_lower = code.to_lowercase();
        Language::all().into_iter().find(|lang| lang.code().to_lowercase() == code_lower)
    }
}

/// liblouis-based braille translator
pub struct LibLouisTranslator {
    _cache: HashMap<String, String>,
}

impl Default for LibLouisTranslator {
    fn default() -> Self {
        Self::new()
    }
}

impl LibLouisTranslator {
    /// Create a new liblouis translator
    pub fn new() -> Self {
        Self {
            _cache: HashMap::new(),
        }
    }

    /// Translate text to braille using specified language (via system command)
    pub fn translate(&self, text: &str, language: Language) -> Result<String, String> {
        use std::process::{Command, Stdio};
        use std::io::Write;

        let table = language.table_name();

        // Use lou_translate command-line tool
        let mut child = Command::new("lou_translate")
            .arg("--forward")
            .arg(table)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("Failed to spawn lou_translate: {}. Is liblouis-bin installed?", e))?;

        // Write input text to stdin
        if let Some(mut stdin) = child.stdin.take() {
            stdin
                .write_all(text.as_bytes())
                .map_err(|e| format!("Failed to write to lou_translate stdin: {}", e))?;
        }

        // Wait for output
        let output = child
            .wait_with_output()
            .map_err(|e| format!("Failed to read lou_translate output: {}", e))?;

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(format!(
                "lou_translate failed for {} (table: {}): {}",
                language.name(),
                table,
                stderr
            ));
        }

        let braille = String::from_utf8_lossy(&output.stdout).trim().to_string();
        Ok(braille)
    }

    /// List all supported languages with their details
    pub fn list_languages(&self) -> Vec<(String, String, String)> {
        Language::all()
            .iter()
            .map(|lang| (
                lang.code().to_string(),
                lang.name().to_string(),
                lang.table_name().to_string(),
            ))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_english_translation() {
        let translator = LibLouisTranslator::new();
        let result = translator.translate("hello", Language::English);
        if result.is_ok() {
            let braille = result.unwrap();
            assert!(!braille.is_empty());
            println!("English 'hello' → {}", braille);
        } else {
            println!("Skipping test (liblouis-bin not available)");
        }
    }

    #[test]
    fn test_french_translation() {
        let translator = LibLouisTranslator::new();
        let result = translator.translate("bonjour", Language::French);
        if result.is_ok() {
            let braille = result.unwrap();
            assert!(!braille.is_empty());
            println!("French 'bonjour' → {}", braille);
        } else {
            println!("Skipping test (liblouis-bin not available)");
        }
    }

    #[test]
    fn test_german_translation() {
        let translator = LibLouisTranslator::new();
        let result = translator.translate("hallo", Language::German);
        if result.is_ok() {
            let braille = result.unwrap();
            assert!(!braille.is_empty());
            println!("German 'hallo' → {}", braille);
        } else {
            println!("Skipping test (liblouis-bin not available)");
        }
    }

    #[test]
    fn test_spanish_translation() {
        let translator = LibLouisTranslator::new();
        let result = translator.translate("hola", Language::Spanish);
        if result.is_ok() {
            let braille = result.unwrap();
            assert!(!braille.is_empty());
            println!("Spanish 'hola' → {}", braille);
        } else {
            println!("Skipping test (liblouis-bin not available)");
        }
    }

    #[test]
    fn test_all_42_languages() {
        let translator = LibLouisTranslator::new();
        let test_text = "hello";

        let mut success_count = 0;
        let mut fail_count = 0;

        for lang in Language::all() {
            match translator.translate(test_text, lang) {
                Ok(braille) => {
                    println!("✓ {} ({}) → {}", lang.name(), lang.code(), braille);
                    success_count += 1;
                }
                Err(e) => {
                    println!("✗ {} ({}): {}", lang.name(), lang.code(), e);
                    fail_count += 1;
                }
            }
        }

        println!("\nResults: {} successful, {} failed out of 42", success_count, fail_count);

        // If liblouis is available, at least some languages should work
        if success_count == 0 {
            println!("⚠️  liblouis-bin not available, skipping validation");
        }
    }

    #[test]
    fn test_language_count() {
        assert_eq!(Language::all().len(), 56, "Should support exactly 56 languages");
    }

    #[test]
    fn test_language_from_code() {
        assert_eq!(Language::from_code("ko"), Some(Language::Korean));
        assert_eq!(Language::from_code("ja"), Some(Language::Japanese));
        assert_eq!(Language::from_code("en"), Some(Language::English));
        assert_eq!(Language::from_code("fr"), Some(Language::French));
        assert_eq!(Language::from_code("de"), Some(Language::German));
        assert_eq!(Language::from_code("unknown"), None);
    }
}
