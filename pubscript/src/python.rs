//! Python bindings for WIA PubScript
//!
//! This module provides Python bindings using PyO3.
//!
//! ## Usage
//!
//! ```python
//! from wia_pubscript import convert, convert_with_language
//!
//! # Auto-detect language (Korean, Japanese, English)
//! braille = convert("안녕하세요")
//! print(braille)  # "⠅⠣⠉⠉⠱⠎⠅⠟⠣⠎⠝⠅⠍"
//!
//! # Explicit language selection (42 languages)
//! braille = convert_with_language("bonjour", "fr")
//! print(braille)  # French braille
//!
//! braille = convert_with_language("hola", "es")
//! print(braille)  # Spanish braille
//! ```

use pyo3::prelude::*;
use crate::braille::{text_to_braille, text_to_braille_with_language, Language, ipa_to_wia_braille};

/// Convert text to braille with auto-detection
///
/// Automatically detects Korean, Japanese, or English and converts to braille.
///
/// Args:
///     text (str): Input text to convert
///
/// Returns:
///     str: Braille representation
///
/// Example:
///     >>> convert("안녕하세요")
///     "⠅⠣⠉⠉⠱⠎⠅⠟⠣⠎⠝⠅⠍"
///     >>> convert("hello")
///     "⠓⠑⠇⠇⠕"
#[pyfunction]
fn convert(text: &str) -> String {
    text_to_braille(text)
}

/// Convert text to braille with explicit language selection
///
/// Supports 56 languages via liblouis.
///
/// Args:
///     text (str): Input text to convert
///     language (str): ISO language code (e.g., "ko", "ja", "fr", "de", "es")
///
/// Returns:
///     str: Braille representation
///
/// Raises:
///     ValueError: If language code is invalid or conversion fails
///
/// Supported languages (56 total):
///     East Asian:
///     - ko: Korean (한국어)
///     - ja: Japanese (日本語)
///     - zh-CN: Chinese Simplified (简体中文)
///     - zh-TW: Chinese Traditional (繁體中文)
///
///     European:
///     - en: English
///     - fr: French (Français)
///     - de: German (Deutsch)
///     - es: Spanish (Español)
///     - it: Italian (Italiano)
///     - pt: Portuguese (Português)
///     - nl: Dutch (Nederlands)
///     - ru: Russian (Русский)
///     - pl: Polish (Polski)
///     - cs: Czech (Čeština)
///     - hu: Hungarian (Magyar)
///     - ro: Romanian (Română)
///     - bg: Bulgarian (Български)
///     - hr: Croatian (Hrvatski)
///     - sk: Slovak (Slovenčina)
///     - sl: Slovenian (Slovenščina)
///     - uk: Ukrainian (Українська)
///     - sr: Serbian (Српски)
///     - sv: Swedish (Svenska)
///     - no: Norwegian (Norsk)
///     - da: Danish (Dansk)
///     - fi: Finnish (Suomi)
///     - is: Icelandic (Íslenska)
///     - lv: Latvian (Latviešu)
///     - lt: Lithuanian (Lietuvių)
///     - et: Estonian (Eesti)
///     - el: Greek (Ελληνικά)
///
///     Middle East:
///     - ar: Arabic (العربية)
///     - he: Hebrew (עברית)
///     - fa: Persian (فارسی)
///     - tr: Turkish (Türkçe)
///
///     South Asia:
///     - hi: Hindi (हिन्दी)
///     - bn: Bengali (বাংলা)
///     - ta: Tamil (தமிழ்)
///     - te: Telugu (తెలుగు)
///     - mr: Marathi (मराठी)
///     - ur: Urdu (اردو)
///     - ne: Nepali (नेपाली)
///     - si: Sinhala (සිංහල)
///
///     Southeast Asia:
///     - th: Thai (ไทย)
///     - vi: Vietnamese (Tiếng Việt)
///     - id: Indonesian (Bahasa Indonesia)
///     - my: Burmese (မြန်မာ)
///     - km: Khmer (ភាសាខ្មែរ)
///     - ms: Malay (Bahasa Melayu)
///     - fil: Filipino
///
///     Caucasus & Central Asia:
///     - ka: Georgian (ქართული)
///     - hy: Armenian (Հայերեն)
///     - mn: Mongolian (Монгол)
///     - kk: Kazakh (Қазақша)
///
///     East Africa:
///     - am: Amharic (አማርኛ)
///
///     Other:
///     - sw: Swahili (Kiswahili)
///
/// Example:
///     >>> convert_with_language("bonjour", "fr")
///     "he|o"
///     >>> convert_with_language("hola", "es")
///     "hello"
#[pyfunction]
fn convert_with_language(text: &str, language: &str) -> PyResult<String> {
    let lang = Language::from_code(language)
        .ok_or_else(|| PyErr::new::<pyo3::exceptions::PyValueError, _>(
            format!("Unknown language code: '{}'. See list_languages() for all 56 supported language codes.", language)
        ))?;

    text_to_braille_with_language(text, lang)
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e))
}

/// List all supported languages
///
/// Returns:
///     list: List of (code, name, table) tuples for all 56 languages
///
/// Example:
///     >>> languages = list_languages()
///     >>> print(languages[0])
///     ('ko', 'Korean (한국어)', 'ko-g1.ctb')
#[pyfunction]
fn list_languages() -> Vec<(String, String, String)> {
    use crate::braille::LibLouisTranslator;
    let translator = LibLouisTranslator::new();
    translator.list_languages()
}

/// Convert IPA to WIA Braille
///
/// Universal braille system for ALL languages (7,000+) through IPA.
/// Philosophy: 홍익인간 (Benefit All Humanity)
///
/// Args:
///     ipa (str): IPA transcription (with or without slashes)
///
/// Returns:
///     str: WIA Braille representation
///
/// Example:
///     >>> # Korean: 안녕하세요
///     >>> ipa_to_braille("/annjʌŋhasʰejo/")
///     "⠁⠝⠝⠚⠜⠝⠛⠓⠁⠎⠈⠓⠑⠚⠕"
///
///     >>> # Arabic: السلام عليكم
///     >>> ipa_to_braille("/asːalaːmu ʕalajkum/")
///     "⠁⠎⠒⠁⠇⠁⠒⠍⠥ ⠦⠄⠁⠇⠁⠚⠅⠥⠍"
///
///     >>> # Navajo: Yá'át'ééh
///     >>> ipa_to_braille("/jaːteːh/")
///     "⠚⠁⠒⠞⠑⠒⠓"
#[pyfunction]
fn ipa_to_braille(ipa: &str) -> String {
    ipa_to_wia_braille(ipa)
}

/// WIA PubScript Python Module
///
/// Multi-sensory publishing engine with comprehensive braille support:
/// - liblouis: 56 language-specific braille tables
/// - WIA Braille: 7,000+ languages through IPA
///
/// Philosophy: "ALL LANGUAGES ARE EQUAL - NO DEFAULT EXISTS"
#[pymodule]
fn wia_pubscript(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(convert, m)?)?;
    m.add_function(wrap_pyfunction!(convert_with_language, m)?)?;
    m.add_function(wrap_pyfunction!(list_languages, m)?)?;
    m.add_function(wrap_pyfunction!(ipa_to_braille, m)?)?;
    Ok(())
}
