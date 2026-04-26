//! Japanese Braille Converter
//!
//! **Philosophy: Japanese Braille is a FIRST-CLASS representation**
//!
//! Converts Japanese text (Hiragana, Katakana, Kanji) to Japanese Braille (点字).
//!
//! Japanese braille is based on the 50-sound table (五十音図) with special
//! prefixes for katakana and numerical expressions.

use std::collections::HashMap;

/// Japanese Braille converter
pub struct JapaneseBrailleConverter {
    /// Hiragana to braille mapping
    hiragana_map: HashMap<char, &'static str>,

    /// Katakana to braille mapping (uses same patterns as hiragana + prefix)
    katakana_map: HashMap<char, &'static str>,

    /// Voiced sound marks
    dakuten_map: HashMap<char, &'static str>,
}

impl Default for JapaneseBrailleConverter {
    fn default() -> Self {
        Self::new()
    }
}

impl JapaneseBrailleConverter {
    /// Create a new Japanese braille converter
    pub fn new() -> Self {
        let mut hiragana_map = HashMap::new();
        let mut katakana_map = HashMap::new();
        let mut dakuten_map = HashMap::new();

        // Vowels (母音)
        hiragana_map.insert('あ', "⠁"); // a
        hiragana_map.insert('い', "⠃"); // i
        hiragana_map.insert('う', "⠉"); // u
        hiragana_map.insert('え', "⠋"); // e
        hiragana_map.insert('お', "⠊"); // o

        // K-row (か行)
        hiragana_map.insert('か', "⠡"); // ka
        hiragana_map.insert('き', "⠣"); // ki
        hiragana_map.insert('く', "⠩"); // ku
        hiragana_map.insert('け', "⠫"); // ke
        hiragana_map.insert('こ', "⠪"); // ko

        // S-row (さ行)
        hiragana_map.insert('さ', "⠱"); // sa
        hiragana_map.insert('し', "⠳"); // shi
        hiragana_map.insert('す', "⠹"); // su
        hiragana_map.insert('せ', "⠻"); // se
        hiragana_map.insert('そ', "⠺"); // so

        // T-row (た行)
        hiragana_map.insert('た', "⠕"); // ta
        hiragana_map.insert('ち', "⠗"); // chi
        hiragana_map.insert('つ', "⠝"); // tsu
        hiragana_map.insert('て', "⠟"); // te
        hiragana_map.insert('と', "⠞"); // to

        // N-row (な行)
        hiragana_map.insert('な', "⠅"); // na
        hiragana_map.insert('に', "⠇"); // ni
        hiragana_map.insert('ぬ', "⠍"); // nu
        hiragana_map.insert('ね', "⠏"); // ne
        hiragana_map.insert('の', "⠎"); // no

        // H-row (は行)
        hiragana_map.insert('は', "⠥"); // ha
        hiragana_map.insert('ひ', "⠧"); // hi
        hiragana_map.insert('ふ', "⠭"); // fu
        hiragana_map.insert('へ', "⠯"); // he
        hiragana_map.insert('ほ', "⠮"); // ho

        // M-row (ま行)
        hiragana_map.insert('ま', "⠵"); // ma
        hiragana_map.insert('み', "⠷"); // mi
        hiragana_map.insert('む', "⠽"); // mu
        hiragana_map.insert('め', "⠿"); // me
        hiragana_map.insert('も', "⠾"); // mo

        // Y-row (や行)
        hiragana_map.insert('や', "⠌"); // ya
        hiragana_map.insert('ゆ', "⠬"); // yu
        hiragana_map.insert('よ', "⠜"); // yo

        // R-row (ら行)
        hiragana_map.insert('ら', "⠑"); // ra
        hiragana_map.insert('り', "⠓"); // ri
        hiragana_map.insert('る', "⠙"); // ru
        hiragana_map.insert('れ', "⠛"); // re
        hiragana_map.insert('ろ', "⠚"); // ro

        // W-row (わ行)
        hiragana_map.insert('わ', "⠄"); // wa
        hiragana_map.insert('を', "⠔"); // wo
        hiragana_map.insert('ん', "⠴"); // n

        // Voiced sounds (濁音) - using dakuten marker
        dakuten_map.insert('が', "⠐⠡"); // ga
        dakuten_map.insert('ぎ', "⠐⠣"); // gi
        dakuten_map.insert('ぐ', "⠐⠩"); // gu
        dakuten_map.insert('げ', "⠐⠫"); // ge
        dakuten_map.insert('ご', "⠐⠪"); // go

        dakuten_map.insert('ざ', "⠐⠱"); // za
        dakuten_map.insert('じ', "⠐⠳"); // ji
        dakuten_map.insert('ず', "⠐⠹"); // zu
        dakuten_map.insert('ぜ', "⠐⠻"); // ze
        dakuten_map.insert('ぞ', "⠐⠺"); // zo

        dakuten_map.insert('だ', "⠐⠕"); // da
        dakuten_map.insert('ぢ', "⠐⠗"); // di
        dakuten_map.insert('づ', "⠐⠝"); // du
        dakuten_map.insert('で', "⠐⠟"); // de
        dakuten_map.insert('ど', "⠐⠞"); // do

        dakuten_map.insert('ば', "⠐⠥"); // ba
        dakuten_map.insert('び', "⠐⠧"); // bi
        dakuten_map.insert('ぶ', "⠐⠭"); // bu
        dakuten_map.insert('べ', "⠐⠯"); // be
        dakuten_map.insert('ぼ', "⠐⠮"); // bo

        // Semi-voiced sounds (半濁音) - using handakuten marker
        dakuten_map.insert('ぱ', "⠠⠥"); // pa
        dakuten_map.insert('ぴ', "⠠⠧"); // pi
        dakuten_map.insert('ぷ', "⠠⠭"); // pu
        dakuten_map.insert('ぺ', "⠠⠯"); // pe
        dakuten_map.insert('ぽ', "⠠⠮"); // po

        // Small characters (小書き仮名)
        hiragana_map.insert('ゃ', "⠈⠌"); // small ya
        hiragana_map.insert('ゅ', "⠈⠬"); // small yu
        hiragana_map.insert('ょ', "⠈⠜"); // small yo
        hiragana_map.insert('っ', "⠈⠝"); // small tsu

        // Katakana (uses same patterns with katakana prefix ⠨)
        // Basic vowels
        katakana_map.insert('ア', "⠁");
        katakana_map.insert('イ', "⠃");
        katakana_map.insert('ウ', "⠉");
        katakana_map.insert('エ', "⠋");
        katakana_map.insert('オ', "⠊");

        // K-row
        katakana_map.insert('カ', "⠡");
        katakana_map.insert('キ', "⠣");
        katakana_map.insert('ク', "⠩");
        katakana_map.insert('ケ', "⠫");
        katakana_map.insert('コ', "⠪");

        // S-row
        katakana_map.insert('サ', "⠱");
        katakana_map.insert('シ', "⠳");
        katakana_map.insert('ス', "⠹");
        katakana_map.insert('セ', "⠻");
        katakana_map.insert('ソ', "⠺");

        // T-row
        katakana_map.insert('タ', "⠕");
        katakana_map.insert('チ', "⠗");
        katakana_map.insert('ツ', "⠝");
        katakana_map.insert('テ', "⠟");
        katakana_map.insert('ト', "⠞");

        // N-row
        katakana_map.insert('ナ', "⠅");
        katakana_map.insert('ニ', "⠇");
        katakana_map.insert('ヌ', "⠍");
        katakana_map.insert('ネ', "⠏");
        katakana_map.insert('ノ', "⠎");

        // H-row
        katakana_map.insert('ハ', "⠥");
        katakana_map.insert('ヒ', "⠧");
        katakana_map.insert('フ', "⠭");
        katakana_map.insert('ヘ', "⠯");
        katakana_map.insert('ホ', "⠮");

        // M-row
        katakana_map.insert('マ', "⠵");
        katakana_map.insert('ミ', "⠷");
        katakana_map.insert('ム', "⠽");
        katakana_map.insert('メ', "⠿");
        katakana_map.insert('モ', "⠾");

        // Y-row
        katakana_map.insert('ヤ', "⠌");
        katakana_map.insert('ユ', "⠬");
        katakana_map.insert('ヨ', "⠜");

        // R-row
        katakana_map.insert('ラ', "⠑");
        katakana_map.insert('リ', "⠓");
        katakana_map.insert('ル', "⠙");
        katakana_map.insert('レ', "⠛");
        katakana_map.insert('ロ', "⠚");

        // W-row
        katakana_map.insert('ワ', "⠄");
        katakana_map.insert('ヲ', "⠔");
        katakana_map.insert('ン', "⠴");

        Self {
            hiragana_map,
            katakana_map,
            dakuten_map,
        }
    }

    /// Convert Japanese text to braille
    pub fn convert(&self, text: &str) -> String {
        let mut result = String::new();
        let mut in_katakana = false;

        for c in text.chars() {
            // Check if it's a voiced/semi-voiced character first
            if let Some(braille) = self.dakuten_map.get(&c) {
                result.push_str(braille);
                continue;
            }

            // Check hiragana
            if let Some(braille) = self.hiragana_map.get(&c) {
                if in_katakana {
                    in_katakana = false;
                }
                result.push_str(braille);
                continue;
            }

            // Check katakana
            if let Some(braille) = self.katakana_map.get(&c) {
                if !in_katakana {
                    result.push_str("⠨"); // Katakana prefix
                    in_katakana = true;
                }
                result.push_str(braille);
                continue;
            }

            // Handle spaces and punctuation
            match c {
                ' ' => {
                    result.push(' ');
                    in_katakana = false;
                }
                '。' => result.push_str("⠲"), // Period
                '、' => result.push_str("⠂"), // Comma
                '「' => result.push_str("⠶"), // Opening quote
                '」' => result.push_str("⠶"), // Closing quote
                '？' => result.push_str("⠢"), // Question mark
                '！' => result.push_str("⠖"), // Exclamation mark
                _ => {
                    // For kanji and other characters, try to preserve them
                    // In a real implementation, we'd need kanji->kana conversion
                    if c.is_ascii() {
                        // Convert ASCII using basic braille
                        result.push_str(&crate::braille::text_to_braille(&c.to_string()));
                    } else {
                        // Keep unknown characters as-is (or skip)
                        result.push(c);
                    }
                    in_katakana = false;
                }
            }
        }

        result
    }
}

/// Convert Japanese text to braille
pub fn text_to_japanese_braille(text: &str) -> String {
    let converter = JapaneseBrailleConverter::new();
    converter.convert(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_japanese_hiragana() {
        let converter = JapaneseBrailleConverter::new();
        let result = converter.convert("あいうえお");
        assert_eq!(result, "⠁⠃⠉⠋⠊");
    }

    #[test]
    fn test_japanese_katakana() {
        let converter = JapaneseBrailleConverter::new();
        let result = converter.convert("アイウエオ");
        // Katakana uses prefix ⠨ before the sequence
        assert!(result.starts_with("⠨"));
        assert!(result.contains("⠁"));
    }

    #[test]
    fn test_japanese_word() {
        let converter = JapaneseBrailleConverter::new();
        let result = converter.convert("こんにちは");
        assert!(!result.is_empty());
        println!("こんにちは → {}", result);
    }

    #[test]
    fn test_japanese_dakuten() {
        let converter = JapaneseBrailleConverter::new();
        let result = converter.convert("が");
        assert_eq!(result, "⠐⠡");
    }

    #[test]
    fn test_japanese_mixed() {
        let converter = JapaneseBrailleConverter::new();
        let result = converter.convert("ひらがな カタカナ");
        assert!(!result.is_empty());
        assert!(result.contains("⠨")); // Should have katakana prefix
        println!("ひらがな カタカナ → {}", result);
    }
}
