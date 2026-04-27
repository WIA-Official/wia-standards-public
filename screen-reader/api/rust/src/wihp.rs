//! WIHP (WIA International Hangul Pronunciation) Engine

use lazy_static::lazy_static;
use std::collections::HashMap;

lazy_static! {
    /// Phoneme mapping for basic characters
    static ref PHONEME_MAP: HashMap<char, &'static str> = {
        let mut m = HashMap::new();
        // Consonants
        m.insert('b', "ㅂ"); m.insert('c', "ㅋ"); m.insert('d', "ㄷ");
        m.insert('f', "ㅍ"); m.insert('g', "ㄱ"); m.insert('h', "ㅎ");
        m.insert('j', "ㅈ"); m.insert('k', "ㅋ"); m.insert('l', "ㄹ");
        m.insert('m', "ㅁ"); m.insert('n', "ㄴ"); m.insert('p', "ㅍ");
        m.insert('q', "ㅋ"); m.insert('r', "ㄹ"); m.insert('s', "ㅅ");
        m.insert('t', "ㅌ"); m.insert('v', "ㅂ"); m.insert('w', "ㅇ");
        m.insert('x', "ㅋㅅ"); m.insert('y', "ㅇ"); m.insert('z', "ㅈ");
        // Vowels
        m.insert('a', "ㅏ"); m.insert('e', "ㅔ"); m.insert('i', "ㅣ");
        m.insert('o', "ㅗ"); m.insert('u', "ㅜ");
        m
    };

    /// Common word pronunciations
    static ref WORD_MAP: HashMap<&'static str, &'static str> = {
        let mut m = HashMap::new();
        m.insert("hello", "헬로우");
        m.insert("world", "월드");
        m.insert("the", "더");
        m.insert("is", "이즈");
        m.insert("a", "어");
        m.insert("an", "앤");
        m.insert("and", "앤드");
        m.insert("or", "오어");
        m.insert("but", "벗");
        m.insert("for", "포");
        m.insert("with", "위드");
        m.insert("this", "디스");
        m.insert("that", "댓");
        m.insert("what", "왓");
        m.insert("when", "웬");
        m.insert("where", "웨어");
        m.insert("why", "와이");
        m.insert("how", "하우");
        m.insert("you", "유");
        m.insert("we", "위");
        m.insert("they", "데이");
        m.insert("it", "잇");
        m.insert("he", "히");
        m.insert("she", "쉬");
        m.insert("good", "굿");
        m.insert("bad", "배드");
        m.insert("yes", "예스");
        m.insert("no", "노");
        m.insert("thank", "땡크");
        m.insert("please", "플리즈");
        m.insert("welcome", "웰컴");
        m.insert("computer", "컴퓨터");
        m.insert("internet", "인터넷");
        m.insert("screen", "스크린");
        m.insert("reader", "리더");
        m
    };
}

/// WIHP Engine for converting text to Korean Hangul pronunciation
pub struct WIHPEngine {
    language: String,
}

impl WIHPEngine {
    /// Create a new WIHP engine
    pub fn new(language: &str) -> Self {
        Self {
            language: language.to_string(),
        }
    }

    /// Convert text to WIHP
    pub fn convert(&self, text: &str, language: &str) -> String {
        match language {
            "ja" => self.convert_japanese(text),
            "zh" => self.convert_chinese(text),
            _ => self.convert_english(text),
        }
    }

    /// Convert English text to WIHP
    fn convert_english(&self, text: &str) -> String {
        text.to_lowercase()
            .split_whitespace()
            .map(|word| {
                let clean_word: String = word.chars().filter(|c| c.is_alphanumeric()).collect();
                WORD_MAP
                    .get(clean_word.as_str())
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| self.phonetic_convert(&clean_word))
            })
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Convert Japanese text to WIHP
    fn convert_japanese(&self, text: &str) -> String {
        let mut result = text.to_string();
        let jp_map = [
            ("こんにちは", "곤니치와"),
            ("ありがとう", "아리가토우"),
            ("さようなら", "사요나라"),
        ];
        for (jp, kr) in jp_map {
            result = result.replace(jp, kr);
        }
        result
    }

    /// Convert Chinese text to WIHP
    fn convert_chinese(&self, text: &str) -> String {
        let mut result = text.to_string();
        let cn_map = [
            ("你好", "니하오"),
            ("谢谢", "씨에씨에"),
            ("再见", "짜이젠"),
        ];
        for (cn, kr) in cn_map {
            result = result.replace(cn, kr);
        }
        result
    }

    /// Phonetic conversion using character mapping
    fn phonetic_convert(&self, word: &str) -> String {
        word.chars()
            .map(|c| {
                PHONEME_MAP
                    .get(&c)
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| c.to_string())
            })
            .collect()
    }

    /// Generate IPA representation (simplified)
    pub fn get_ipa(&self, text: &str) -> String {
        format!("/{}/", text.to_lowercase())
    }

    /// Set the default language
    pub fn set_language(&mut self, language: &str) {
        self.language = language.to_string();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_english_conversion() {
        let engine = WIHPEngine::new("en");
        assert_eq!(engine.convert("hello", "en"), "헬로우");
        assert_eq!(engine.convert("world", "en"), "월드");
        assert_eq!(engine.convert("hello world", "en"), "헬로우 월드");
    }

    #[test]
    fn test_phonetic_fallback() {
        let engine = WIHPEngine::new("en");
        let result = engine.convert("xyz", "en");
        assert!(!result.is_empty());
    }
}
