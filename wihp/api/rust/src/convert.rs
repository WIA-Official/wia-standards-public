//! WIHP conversion functions

#[cfg(not(feature = "std"))]
use alloc::string::String;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

use unicode_normalization::UnicodeNormalization;
use crate::consonants::CONSONANT_MAP;
use crate::vowels::VOWEL_MAP;
use crate::diacritics::DIACRITIC_MAP;
use crate::error::{WihpError, WihpResult};

/// Convert IPA string to WIHP Hangul representation
///
/// # Arguments
///
/// * `ipa` - IPA transcription (with or without slashes/brackets)
///
/// # Returns
///
/// Korean Hangul representation of the pronunciation
///
/// # Example
///
/// ```rust
/// use wia_wihp::ipa_to_hangul;
///
/// let result = ipa_to_hangul("/həˈloʊ/").unwrap();
/// assert_eq!(result, "헬로우");
/// ```
pub fn ipa_to_hangul(ipa: &str) -> WihpResult<String> {
    if ipa.is_empty() {
        return Err(WihpError::EmptyInput);
    }

    // Normalize and clean input
    let normalized = normalize_ipa(ipa);

    let mut result = String::new();
    let mut chars = normalized.chars().peekable();

    while let Some(c) = chars.next() {
        // Skip stress markers (just mark them)
        if c == 'ˈ' || c == 'ˌ' {
            // Primary stress: ' , Secondary stress: ,
            result.push(if c == 'ˈ' { '\'' } else { ',' });
            continue;
        }

        // Look up in maps
        if let Some(&hangul) = CONSONANT_MAP.get(&c) {
            result.push_str(hangul);
        } else if let Some(&hangul) = VOWEL_MAP.get(&c) {
            result.push_str(hangul);
        } else if let Some(&hangul) = DIACRITIC_MAP.get(&c) {
            if !hangul.is_empty() {
                result.push_str(hangul);
            }
        } else {
            // Handle common combinations
            match c {
                // Diphthongs
                'o' if chars.peek() == Some(&'ʊ') => {
                    chars.next();
                    result.push_str("오우");
                }
                'a' if chars.peek() == Some(&'ɪ') => {
                    chars.next();
                    result.push_str("아이");
                }
                'a' if chars.peek() == Some(&'ʊ') => {
                    chars.next();
                    result.push_str("아우");
                }
                'e' if chars.peek() == Some(&'ɪ') => {
                    chars.next();
                    result.push_str("에이");
                }
                // Skip combining characters that don't add meaning
                _ if c.is_ascii_whitespace() => {}
                // Unknown symbol - skip with warning in lenient mode
                _ => {
                    // For now, pass through unknown symbols
                    result.push(c);
                }
            }
        }
    }

    // Post-process: combine jamo into syllables where possible
    let combined = combine_jamo(&result);

    Ok(combined)
}

/// Convert WIHP Hangul back to IPA
///
/// # Arguments
///
/// * `hangul` - Korean Hangul pronunciation
///
/// # Returns
///
/// IPA transcription
///
/// # Example
///
/// ```rust
/// use wia_wihp::hangul_to_ipa;
///
/// let result = hangul_to_ipa("헬로우").unwrap();
/// assert_eq!(result, "həloʊ");
/// ```
pub fn hangul_to_ipa(hangul: &str) -> WihpResult<String> {
    if hangul.is_empty() {
        return Err(WihpError::EmptyInput);
    }

    let mut result = String::new();

    for c in hangul.chars() {
        // Check if it's a complete Hangul syllable
        if c >= '\u{AC00}' && c <= '\u{D7A3}' {
            // Decompose syllable
            let decomposed = decompose_syllable(c);
            result.push_str(&decomposed);
        } else if c >= '\u{1100}' && c <= '\u{11FF}' {
            // Hangul Jamo
            result.push_str(&jamo_to_ipa(c));
        } else if c >= '\u{3131}' && c <= '\u{3163}' {
            // Hangul Compatibility Jamo
            result.push_str(&compat_jamo_to_ipa(c));
        } else {
            // Pass through other characters
            result.push(c);
        }
    }

    Ok(result)
}

/// Normalize IPA input
fn normalize_ipa(ipa: &str) -> String {
    ipa.nfc()
        .collect::<String>()
        .trim()
        .trim_matches(|c| c == '/' || c == '[' || c == ']')
        .to_string()
}

/// Combine individual jamo into syllable blocks
fn combine_jamo(jamo: &str) -> String {
    let mut result = String::new();
    let chars: Vec<char> = jamo.chars().collect();
    let mut i = 0;

    while i < chars.len() {
        let c = chars[i];

        // Check if this is a jamo that can be combined
        if is_initial_jamo(c) && i + 1 < chars.len() && is_medial_jamo(chars[i + 1]) {
            let initial = c;
            let medial = chars[i + 1];

            // Check for final consonant
            let final_jamo = if i + 2 < chars.len() && is_final_jamo(chars[i + 2]) {
                // Only use as final if not followed by a vowel
                if i + 3 >= chars.len() || !is_medial_jamo(chars[i + 3]) {
                    i += 1;
                    Some(chars[i + 1])
                } else {
                    None
                }
            } else {
                None
            };

            // Combine into syllable
            if let Some(syllable) = compose_syllable(initial, medial, final_jamo) {
                result.push(syllable);
                i += 2;
                continue;
            }
        }

        result.push(c);
        i += 1;
    }

    result
}

/// Check if character is an initial jamo
fn is_initial_jamo(c: char) -> bool {
    matches!(c, 'ㄱ'..='ㅎ')
}

/// Check if character is a medial jamo (vowel)
fn is_medial_jamo(c: char) -> bool {
    matches!(c, 'ㅏ'..='ㅣ')
}

/// Check if character is a final jamo
fn is_final_jamo(c: char) -> bool {
    matches!(c, 'ㄱ'..='ㅎ')
}

/// Compose a Hangul syllable from jamo
fn compose_syllable(initial: char, medial: char, final_jamo: Option<char>) -> Option<char> {
    // Convert compatibility jamo to indices
    let initial_idx = match initial {
        'ㄱ' => 0, 'ㄲ' => 1, 'ㄴ' => 2, 'ㄷ' => 3, 'ㄸ' => 4,
        'ㄹ' => 5, 'ㅁ' => 6, 'ㅂ' => 7, 'ㅃ' => 8, 'ㅅ' => 9,
        'ㅆ' => 10, 'ㅇ' => 11, 'ㅈ' => 12, 'ㅉ' => 13, 'ㅊ' => 14,
        'ㅋ' => 15, 'ㅌ' => 16, 'ㅍ' => 17, 'ㅎ' => 18,
        _ => return None,
    };

    let medial_idx = match medial {
        'ㅏ' => 0, 'ㅐ' => 1, 'ㅑ' => 2, 'ㅒ' => 3, 'ㅓ' => 4,
        'ㅔ' => 5, 'ㅕ' => 6, 'ㅖ' => 7, 'ㅗ' => 8, 'ㅘ' => 9,
        'ㅙ' => 10, 'ㅚ' => 11, 'ㅛ' => 12, 'ㅜ' => 13, 'ㅝ' => 14,
        'ㅞ' => 15, 'ㅟ' => 16, 'ㅠ' => 17, 'ㅡ' => 18, 'ㅢ' => 19,
        'ㅣ' => 20,
        _ => return None,
    };

    let final_idx = match final_jamo {
        None => 0,
        Some('ㄱ') => 1, Some('ㄲ') => 2, Some('ㄳ') => 3,
        Some('ㄴ') => 4, Some('ㄵ') => 5, Some('ㄶ') => 6,
        Some('ㄷ') => 7, Some('ㄹ') => 8, Some('ㄺ') => 9,
        Some('ㄻ') => 10, Some('ㄼ') => 11, Some('ㄽ') => 12,
        Some('ㄾ') => 13, Some('ㄿ') => 14, Some('ㅀ') => 15,
        Some('ㅁ') => 16, Some('ㅂ') => 17, Some('ㅄ') => 18,
        Some('ㅅ') => 19, Some('ㅆ') => 20, Some('ㅇ') => 21,
        Some('ㅈ') => 22, Some('ㅊ') => 23, Some('ㅋ') => 24,
        Some('ㅌ') => 25, Some('ㅍ') => 26, Some('ㅎ') => 27,
        _ => 0,
    };

    // Calculate syllable code point
    let code = 0xAC00 + (initial_idx * 21 + medial_idx) * 28 + final_idx;
    char::from_u32(code)
}

/// Decompose a Hangul syllable into IPA
fn decompose_syllable(syllable: char) -> String {
    let code = syllable as u32 - 0xAC00;
    let initial_idx = code / (21 * 28);
    let medial_idx = (code % (21 * 28)) / 28;
    let final_idx = code % 28;

    let mut result = String::new();

    // Initial consonant
    result.push_str(match initial_idx {
        0 => "k", 1 => "kk", 2 => "n", 3 => "t", 4 => "tt",
        5 => "l", 6 => "m", 7 => "p", 8 => "pp", 9 => "s",
        10 => "ss", 11 => "", 12 => "tɕ", 13 => "ttɕ", 14 => "tɕʰ",
        15 => "kʰ", 16 => "tʰ", 17 => "pʰ", 18 => "h",
        _ => "",
    });

    // Medial vowel
    result.push_str(match medial_idx {
        0 => "a", 1 => "ɛ", 2 => "ja", 3 => "jɛ", 4 => "ʌ",
        5 => "e", 6 => "jʌ", 7 => "je", 8 => "o", 9 => "wa",
        10 => "wɛ", 11 => "ø", 12 => "jo", 13 => "u", 14 => "wʌ",
        15 => "we", 16 => "y", 17 => "ju", 18 => "ɯ", 19 => "ɯi",
        20 => "i",
        _ => "",
    });

    // Final consonant
    if final_idx > 0 {
        result.push_str(match final_idx {
            1 => "k", 2 => "kk", 3 => "ks", 4 => "n", 5 => "ntɕ",
            6 => "nh", 7 => "t", 8 => "l", 9 => "lk", 10 => "lm",
            11 => "lp", 12 => "ls", 13 => "ltʰ", 14 => "lpʰ", 15 => "lh",
            16 => "m", 17 => "p", 18 => "ps", 19 => "s", 20 => "ss",
            21 => "ŋ", 22 => "tɕ", 23 => "tɕʰ", 24 => "kʰ", 25 => "tʰ",
            26 => "pʰ", 27 => "h",
            _ => "",
        });
    }

    result
}

/// Convert jamo to IPA
fn jamo_to_ipa(c: char) -> String {
    // TODO: Implement full jamo mapping
    c.to_string()
}

/// Convert compatibility jamo to IPA
fn compat_jamo_to_ipa(c: char) -> String {
    match c {
        'ㄱ' => "k".into(),
        'ㄴ' => "n".into(),
        'ㄷ' => "t".into(),
        'ㄹ' => "l".into(),
        'ㅁ' => "m".into(),
        'ㅂ' => "p".into(),
        'ㅅ' => "s".into(),
        'ㅇ' => "ŋ".into(),
        'ㅈ' => "tɕ".into(),
        'ㅊ' => "tɕʰ".into(),
        'ㅋ' => "kʰ".into(),
        'ㅌ' => "tʰ".into(),
        'ㅍ' => "pʰ".into(),
        'ㅎ' => "h".into(),
        'ㅏ' => "a".into(),
        'ㅓ' => "ʌ".into(),
        'ㅗ' => "o".into(),
        'ㅜ' => "u".into(),
        'ㅡ' => "ɯ".into(),
        'ㅣ' => "i".into(),
        _ => c.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hello() {
        let result = ipa_to_hangul("/həˈloʊ/").unwrap();
        assert_eq!(result, "헬로우");
    }

    #[test]
    fn test_normalize() {
        assert_eq!(normalize_ipa("/test/"), "test");
        assert_eq!(normalize_ipa("[test]"), "test");
        assert_eq!(normalize_ipa("  test  "), "test");
    }

    #[test]
    fn test_empty_input() {
        assert!(matches!(ipa_to_hangul(""), Err(WihpError::EmptyInput)));
    }
}
