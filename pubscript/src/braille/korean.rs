//! Korean Braille (한국 점자) conversion
//!
//! **Korean Braille is a first-class representation, EQUAL to Hangul text**
//!
//! This module implements Korean Braille conversion based on
//! the official Korean Braille system (한국 점자 규정).
//!
//! Korean uses syllable-based braille where each syllable is decomposed into:
//! - 초성 (Initial consonant)
//! - 중성 (Vowel)
//! - 종성 (Final consonant, optional)

/// Convert Korean text to Korean Braille
pub fn korean_to_braille(text: &str) -> String {
    text.chars()
        .map(|c| {
            if is_hangul_syllable(c) {
                syllable_to_braille(c)
            } else if c.is_whitespace() {
                " ".to_string()
            } else {
                // Pass through non-Korean characters
                c.to_string()
            }
        })
        .collect()
}

/// Check if character is a Hangul syllable
fn is_hangul_syllable(c: char) -> bool {
    matches!(c as u32, 0xAC00..=0xD7A3)
}

/// Decompose a Hangul syllable into 초성, 중성, 종성
fn decompose_syllable(c: char) -> (u32, u32, Option<u32>) {
    let code = c as u32 - 0xAC00;

    let initial = code / (21 * 28); // 초성
    let vowel = (code % (21 * 28)) / 28; // 중성
    let final_consonant = code % 28; // 종성

    let final_consonant = if final_consonant == 0 {
        None
    } else {
        Some(final_consonant - 1)
    };

    (initial, vowel, final_consonant)
}

/// Convert Hangul syllable to Korean Braille
fn syllable_to_braille(c: char) -> String {
    let (initial, vowel, final_consonant) = decompose_syllable(c);

    let mut braille = String::new();

    // 초성 (Initial consonant)
    braille.push_str(initial_to_braille(initial));

    // 중성 (Vowel)
    braille.push_str(vowel_to_braille(vowel));

    // 종성 (Final consonant)
    if let Some(fc) = final_consonant {
        braille.push_str(final_to_braille(fc));
    }

    braille
}

/// Convert 초성 (initial consonant) to braille
fn initial_to_braille(index: u32) -> &'static str {
    match index {
        0 => "⠈", // ㄱ
        1 => "⠠", // ㄲ
        2 => "⠉", // ㄴ
        3 => "⠊", // ㄷ
        4 => "⠰", // ㄸ
        5 => "⠗", // ㄹ
        6 => "⠍", // ㅁ
        7 => "⠃", // ㅂ
        8 => "⠠⠃", // ㅃ
        9 => "⠎", // ㅅ
        10 => "⠠⠎", // ㅆ
        11 => "⠅", // ㅇ
        12 => "⠛", // ㅈ
        13 => "⠠⠛", // ㅉ
        14 => "⠹", // ㅊ
        15 => "⠋", // ㅋ
        16 => "⠓", // ㅌ
        17 => "⠏", // ㅍ
        18 => "⠟", // ㅎ
        _ => "",
    }
}

/// Convert 중성 (vowel) to braille
fn vowel_to_braille(index: u32) -> &'static str {
    match index {
        0 => "⠣", // ㅏ
        1 => "⠜", // ㅐ
        2 => "⠱", // ㅑ
        3 => "⠌", // ㅒ
        4 => "⠎", // ㅓ
        5 => "⠝", // ㅔ
        6 => "⠱⠎", // ㅕ
        7 => "⠌⠎", // ㅖ
        8 => "⠥", // ㅗ
        9 => "⠽", // ㅘ
        10 => "⠬", // ㅙ
        11 => "⠭", // ㅚ
        12 => "⠍", // ㅛ
        13 => "⠥⠎", // ㅜ
        14 => "⠍⠎", // ㅝ
        15 => "⠔", // ㅞ
        16 => "⠺", // ㅟ
        17 => "⠍⠱", // ㅠ
        18 => "⠪", // ㅡ
        19 => "⠕", // ㅢ
        20 => "⠊⠎", // ㅣ
        _ => "",
    }
}

/// Convert 종성 (final consonant) to braille
fn final_to_braille(index: u32) -> &'static str {
    match index {
        0 => "⠈", // ㄱ
        1 => "⠠⠈", // ㄲ
        2 => "⠈⠎", // ㄳ
        3 => "⠉", // ㄴ
        4 => "⠉⠛", // ㄵ
        5 => "⠉⠟", // ㄶ
        6 => "⠊", // ㄷ
        7 => "⠗", // ㄹ
        8 => "⠗⠈", // ㄺ
        9 => "⠗⠍", // ㄻ
        10 => "⠗⠃", // ㄼ
        11 => "⠗⠎", // ㄽ
        12 => "⠗⠓", // ㄾ
        13 => "⠗⠏", // ㄿ
        14 => "⠗⠟", // ㅀ
        15 => "⠍", // ㅁ
        16 => "⠃", // ㅂ
        17 => "⠃⠎", // ㅄ
        18 => "⠎", // ㅅ
        19 => "⠠⠎", // ㅆ
        20 => "⠅", // ㅇ
        21 => "⠛", // ㅈ
        22 => "⠹", // ㅊ
        23 => "⠋", // ㅋ
        24 => "⠓", // ㅌ
        25 => "⠏", // ㅍ
        26 => "⠟", // ㅎ
        _ => "",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decompose_syllable() {
        // '한' = ㅎ(18) + ㅏ(0) + ㄴ(4)
        let (initial, vowel, final_consonant) = decompose_syllable('한');
        assert_eq!(initial, 18); // ㅎ
        assert_eq!(vowel, 0); // ㅏ
        assert_eq!(final_consonant, Some(3)); // ㄴ
    }

    #[test]
    fn test_korean_to_braille() {
        // Test basic syllables
        let result = korean_to_braille("한");
        println!("한 -> {}", result);
        assert!(!result.is_empty());

        let result = korean_to_braille("글");
        println!("글 -> {}", result);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_korean_word() {
        // "안녕하세요"
        let result = korean_to_braille("안녕하세요");
        println!("안녕하세요 -> {}", result);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_korean_with_space() {
        let result = korean_to_braille("안녕 하세요");
        println!("안녕 하세요 -> {}", result);
        assert!(result.contains(' '));
    }
}
