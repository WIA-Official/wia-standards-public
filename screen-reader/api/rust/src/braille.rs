//! Braille Engine for WIA Screen Reader

use crate::types::{BrailleCell, BrailleGrade, BrailleOutput};
use lazy_static::lazy_static;
use std::collections::HashMap;

lazy_static! {
    /// Unicode Braille mapping (Grade 1)
    static ref BRAILLE_MAP: HashMap<char, char> = {
        let mut m = HashMap::new();
        m.insert('a', '⠁'); m.insert('b', '⠃'); m.insert('c', '⠉');
        m.insert('d', '⠙'); m.insert('e', '⠑'); m.insert('f', '⠋');
        m.insert('g', '⠛'); m.insert('h', '⠓'); m.insert('i', '⠊');
        m.insert('j', '⠚'); m.insert('k', '⠅'); m.insert('l', '⠇');
        m.insert('m', '⠍'); m.insert('n', '⠝'); m.insert('o', '⠕');
        m.insert('p', '⠏'); m.insert('q', '⠟'); m.insert('r', '⠗');
        m.insert('s', '⠎'); m.insert('t', '⠞'); m.insert('u', '⠥');
        m.insert('v', '⠧'); m.insert('w', '⠺'); m.insert('x', '⠭');
        m.insert('y', '⠽'); m.insert('z', '⠵');
        // Numbers
        m.insert('0', '⠚'); m.insert('1', '⠁'); m.insert('2', '⠃');
        m.insert('3', '⠉'); m.insert('4', '⠙'); m.insert('5', '⠑');
        m.insert('6', '⠋'); m.insert('7', '⠛'); m.insert('8', '⠓');
        m.insert('9', '⠊');
        // Punctuation
        m.insert(' ', ' '); m.insert('.', '⠲'); m.insert(',', '⠂');
        m.insert('!', '⠖'); m.insert('?', '⠦'); m.insert('\'', '⠄');
        m.insert('-', '⠤'); m.insert(':', '⠒'); m.insert(';', '⠆');
        m
    };

    /// WIA Braille mapping (Korean Jamo based)
    static ref WIA_MAP: HashMap<char, &'static str> = {
        let mut m = HashMap::new();
        m.insert('a', "ㅏ"); m.insert('b', "ㅂ"); m.insert('c', "ㅋ");
        m.insert('d', "ㄷ"); m.insert('e', "ㅔ"); m.insert('f', "ㅍ");
        m.insert('g', "ㄱ"); m.insert('h', "ㅎ"); m.insert('i', "ㅣ");
        m.insert('j', "ㅈ"); m.insert('k', "ㅋ"); m.insert('l', "ㄹ");
        m.insert('m', "ㅁ"); m.insert('n', "ㄴ"); m.insert('o', "ㅗ");
        m.insert('p', "ㅍ"); m.insert('q', "ㅋ"); m.insert('r', "ㄹ");
        m.insert('s', "ㅅ"); m.insert('t', "ㅌ"); m.insert('u', "ㅜ");
        m.insert('v', "ㅂ"); m.insert('w', "ㅇ"); m.insert('x', "ㅋㅅ");
        m.insert('y', "ㅇ"); m.insert('z', "ㅈ"); m.insert(' ', " ");
        m
    };
}

/// Braille Engine for converting text to various braille formats
pub struct BrailleEngine {
    grade: BrailleGrade,
}

impl BrailleEngine {
    /// Create a new Braille engine
    pub fn new(grade: BrailleGrade) -> Self {
        Self { grade }
    }

    /// Convert text to braille
    pub fn convert(&self, text: &str) -> BrailleOutput {
        let lower = text.to_lowercase();

        let grade1 = self.convert_grade1(&lower);
        let grade2 = self.convert_grade2(&lower);
        let wia = self.convert_wia(&lower);
        let cells = self.generate_cells(&lower);
        let cell_count = cells.len();

        BrailleOutput {
            grade1,
            grade2,
            wia,
            dots: Some(cells),
            cells: cell_count,
        }
    }

    /// Convert to Grade 1 braille
    fn convert_grade1(&self, text: &str) -> String {
        text.chars()
            .map(|c| BRAILLE_MAP.get(&c).copied().unwrap_or(c))
            .collect()
    }

    /// Convert to Grade 2 braille (with contractions)
    fn convert_grade2(&self, text: &str) -> String {
        // For now, same as Grade 1
        // Full Grade 2 implementation would include contractions
        self.convert_grade1(text)
    }

    /// Convert to WIA braille format
    fn convert_wia(&self, text: &str) -> String {
        text.chars()
            .map(|c| {
                WIA_MAP
                    .get(&c)
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| c.to_string())
            })
            .collect()
    }

    /// Generate braille cell data
    fn generate_cells(&self, text: &str) -> Vec<BrailleCell> {
        text.chars()
            .filter(|c| *c != ' ' && BRAILLE_MAP.contains_key(c))
            .map(|c| {
                let braille = BRAILLE_MAP.get(&c).copied().unwrap_or(c);
                BrailleCell {
                    char: c,
                    dots: unicode_to_dots(braille),
                    unicode: braille,
                }
            })
            .collect()
    }

    /// Set the braille grade
    pub fn set_grade(&mut self, grade: BrailleGrade) {
        self.grade = grade;
    }
}

/// Convert Unicode braille character to dot pattern
fn unicode_to_dots(c: char) -> Vec<u8> {
    let code = (c as u32).saturating_sub(0x2800);
    (0..8)
        .filter(|i| code & (1 << i) != 0)
        .map(|i| (i + 1) as u8)
        .collect()
}

/// Convert dot pattern to Unicode braille character
pub fn dots_to_unicode(dots: &[u8]) -> char {
    let code: u32 = dots
        .iter()
        .filter(|&&d| d >= 1 && d <= 8)
        .map(|&d| 1u32 << (d - 1))
        .sum();
    char::from_u32(0x2800 + code).unwrap_or('⠀')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grade1_conversion() {
        let engine = BrailleEngine::new(BrailleGrade::Grade1);
        let result = engine.convert("abc");
        assert_eq!(result.grade1, "⠁⠃⠉");
    }

    #[test]
    fn test_wia_conversion() {
        let engine = BrailleEngine::new(BrailleGrade::Grade1);
        let result = engine.convert("abc");
        assert_eq!(result.wia, "ㅏㅂㅋ");
    }

    #[test]
    fn test_unicode_to_dots() {
        assert_eq!(unicode_to_dots('⠁'), vec![1]);
        assert_eq!(unicode_to_dots('⠃'), vec![1, 2]);
    }

    #[test]
    fn test_dots_to_unicode() {
        assert_eq!(dots_to_unicode(&[1]), '⠁');
        assert_eq!(dots_to_unicode(&[1, 2]), '⠃');
    }
}
