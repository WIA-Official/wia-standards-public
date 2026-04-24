//! Braille Output Adapter

use async_trait::async_trait;
use chrono::Utc;
use std::collections::HashMap;

use crate::error::WiaAiError;
use crate::integration::{
    AiOutput, AiOutputType, WiaMessage, WiaMessageType, WiaPayload, WiaStandardType,
    connector::AiOutputAdapter,
};

/// Braille grade/level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BrailleGrade {
    /// Grade 1: Letter-by-letter (no contractions)
    Grade1,
    /// Grade 2: With contractions
    Grade2,
}

impl Default for BrailleGrade {
    fn default() -> Self {
        Self::Grade2
    }
}

/// Braille Output Adapter
///
/// Converts AI text output into braille-ready messages.
pub struct BrailleOutputAdapter {
    /// Braille grade
    grade: BrailleGrade,
    /// Language code
    language: String,
}

impl BrailleOutputAdapter {
    /// Create a new braille output adapter
    pub fn new() -> Self {
        Self {
            grade: BrailleGrade::default(),
            language: "en".into(),
        }
    }

    /// Create with specific grade
    pub fn with_grade(grade: BrailleGrade) -> Self {
        Self {
            grade,
            language: "en".into(),
        }
    }

    /// Create with specific language
    pub fn with_language(language: impl Into<String>) -> Self {
        Self {
            grade: BrailleGrade::default(),
            language: language.into(),
        }
    }

    /// Simple ASCII to Braille Unicode conversion (basic demo)
    /// Real implementation would use liblouis or similar
    fn text_to_braille(&self, text: &str) -> String {
        // Basic ASCII to Braille Unicode mapping (Grade 1)
        let braille_map: HashMap<char, char> = [
            ('a', '⠁'), ('b', '⠃'), ('c', '⠉'), ('d', '⠙'), ('e', '⠑'),
            ('f', '⠋'), ('g', '⠛'), ('h', '⠓'), ('i', '⠊'), ('j', '⠚'),
            ('k', '⠅'), ('l', '⠇'), ('m', '⠍'), ('n', '⠝'), ('o', '⠕'),
            ('p', '⠏'), ('q', '⠟'), ('r', '⠗'), ('s', '⠎'), ('t', '⠞'),
            ('u', '⠥'), ('v', '⠧'), ('w', '⠺'), ('x', '⠭'), ('y', '⠽'),
            ('z', '⠵'), (' ', '⠀'), ('.', '⠲'), (',', '⠂'), ('?', '⠦'),
            ('!', '⠖'), ('\'', '⠄'), ('-', '⠤'),
            ('0', '⠴'), ('1', '⠂'), ('2', '⠆'), ('3', '⠒'), ('4', '⠲'),
            ('5', '⠢'), ('6', '⠖'), ('7', '⠶'), ('8', '⠦'), ('9', '⠔'),
        ].into_iter().collect();

        text.to_lowercase()
            .chars()
            .map(|c| *braille_map.get(&c).unwrap_or(&c))
            .collect()
    }

    /// Get braille Unicode code points
    fn get_unicode_points(&self, braille: &str) -> Vec<String> {
        braille
            .chars()
            .map(|c| format!("U+{:04X}", c as u32))
            .collect()
    }
}

impl Default for BrailleOutputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AiOutputAdapter for BrailleOutputAdapter {
    fn name(&self) -> &str {
        "braille_output"
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::Braille
    }

    fn target_standard(&self) -> WiaStandardType {
        WiaStandardType::Braille
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        let braille = self.text_to_braille(&output.text);
        let unicode_points = self.get_unicode_points(&braille);

        let grade_str = match self.grade {
            BrailleGrade::Grade1 => "grade1",
            BrailleGrade::Grade2 => "grade2",
        };

        let mut metadata = HashMap::new();
        metadata.insert("grade".into(), serde_json::json!(grade_str));
        metadata.insert("language".into(), serde_json::json!(self.language));
        metadata.insert("char_count".into(), serde_json::json!(output.text.len()));
        metadata.insert("braille_count".into(), serde_json::json!(braille.len()));

        // Merge with output metadata
        for (key, value) in output.metadata {
            metadata.insert(key, value);
        }

        Ok(WiaMessage {
            id: uuid::Uuid::new_v4().to_string(),
            source: WiaStandardType::Ai,
            target: WiaStandardType::Braille,
            message_type: WiaMessageType::Response,
            payload: WiaPayload::Json(serde_json::json!({
                "original_text": output.text,
                "braille": braille,
                "unicode": unicode_points,
                "grade": grade_str,
                "language": self.language,
            })),
            metadata,
            timestamp: Utc::now(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_braille_output() {
        let adapter = BrailleOutputAdapter::new();

        let output = AiOutput::text("hello");
        let message = adapter.from_ai_output(output).await.unwrap();

        assert_eq!(message.target, WiaStandardType::Braille);

        if let WiaPayload::Json(json) = &message.payload {
            assert_eq!(json["original_text"], "hello");
            assert_eq!(json["braille"], "⠓⠑⠇⠇⠕");
        } else {
            panic!("Expected JSON payload");
        }
    }

    #[test]
    fn test_text_to_braille() {
        let adapter = BrailleOutputAdapter::new();

        assert_eq!(adapter.text_to_braille("abc"), "⠁⠃⠉");
        assert_eq!(adapter.text_to_braille("hello"), "⠓⠑⠇⠇⠕");
        assert_eq!(adapter.text_to_braille("hi"), "⠓⠊");
    }

    #[test]
    fn test_unicode_points() {
        let adapter = BrailleOutputAdapter::new();

        let braille = "⠁⠃⠉";
        let points = adapter.get_unicode_points(braille);

        assert_eq!(points.len(), 3);
        assert_eq!(points[0], "U+2801");
        assert_eq!(points[1], "U+2803");
        assert_eq!(points[2], "U+2809");
    }
}
