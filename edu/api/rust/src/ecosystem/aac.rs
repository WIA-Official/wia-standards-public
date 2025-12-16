//! AAC (Augmentative and Alternative Communication) Integration
//! 弘益人間 - Enable AAC users to participate in learning activities

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;

use crate::error::{EduError, Result};
use crate::types::{LearnerProfile, Assessment, QuestionType};

/// AAC Education trait for learning activity participation
pub trait AACEducation: Send + Sync {
    /// Convert AAC symbol to assessment answer
    fn symbol_to_answer(&self, symbol: &AACSymbol, question_type: &QuestionType) -> Option<AssessmentAnswer>;

    /// Generate voice response for text
    fn voice_response(&self, text: &str, language: &str) -> Result<VoiceOutput>;

    /// Generate feedback message using AAC symbols
    fn generate_feedback(&self, result: &AnswerResult) -> AACMessage;

    /// Get available symbol sets
    fn available_symbol_sets(&self) -> Vec<SymbolSet>;

    /// Translate text to symbols
    fn text_to_symbols(&self, text: &str) -> Vec<AACSymbol>;

    /// Check if AAC device is connected
    fn is_connected(&self) -> bool;
}

/// AAC Symbol representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACSymbol {
    /// Symbol ID
    pub id: String,
    /// Symbol set (PCS, Blissymbols, etc.)
    pub symbol_set: SymbolSet,
    /// Symbol meaning/label
    pub meaning: String,
    /// Symbol category
    pub category: SymbolCategory,
    /// Unicode representation if available
    pub unicode: Option<String>,
    /// Image URL for the symbol
    pub image_url: Option<String>,
    /// Spoken text for TTS
    pub spoken_text: Option<String>,
}

/// Symbol set types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SymbolSet {
    /// Picture Communication Symbols
    PCS,
    /// Blissymbolics
    Blissymbols,
    /// SymbolStix
    SymbolStix,
    /// Widgit Symbols
    Widgit,
    /// ARASAAC
    ARASAAC,
    /// Mulberry Symbols
    Mulberry,
    /// Custom symbols
    Custom,
}

/// Symbol category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SymbolCategory {
    /// People
    People,
    /// Actions/Verbs
    Actions,
    /// Objects/Nouns
    Objects,
    /// Descriptors/Adjectives
    Descriptors,
    /// Social words
    Social,
    /// Questions
    Questions,
    /// Numbers
    Numbers,
    /// Letters
    Letters,
    /// Yes/No responses
    YesNo,
    /// Academic content
    Academic,
}

/// Assessment answer from AAC input
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentAnswer {
    /// Answer ID
    pub answer_id: Uuid,
    /// Original symbol(s) used
    pub symbols: Vec<AACSymbol>,
    /// Interpreted answer text
    pub answer_text: String,
    /// Confidence level (0.0-1.0)
    pub confidence: f32,
    /// Input method used
    pub input_method: AACInputMethod,
}

/// AAC input method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AACInputMethod {
    /// Direct touch selection
    Touch,
    /// Switch scanning
    Switch,
    /// Eye gaze selection
    EyeGaze,
    /// Head tracking
    HeadTracking,
    /// Voice input
    Voice,
}

/// Result of an answer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnswerResult {
    /// Whether the answer was correct
    pub correct: bool,
    /// Score achieved (0.0-1.0)
    pub score: f32,
    /// Feedback text
    pub feedback: String,
    /// Correct answer if wrong
    pub correct_answer: Option<String>,
}

/// AAC message for feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACMessage {
    /// Message ID
    pub id: Uuid,
    /// Symbols in the message
    pub symbols: Vec<AACSymbol>,
    /// Text representation
    pub text: String,
    /// TTS output text
    pub spoken_text: String,
    /// Message type
    pub message_type: MessageType,
}

/// Message type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MessageType {
    /// Positive feedback (correct answer)
    Positive,
    /// Encouragement (wrong but close)
    Encouragement,
    /// Correction (wrong answer)
    Correction,
    /// Instruction
    Instruction,
    /// Prompt to continue
    Prompt,
}

/// Voice output configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceOutput {
    /// Text to speak
    pub text: String,
    /// Language code
    pub language: String,
    /// Speech rate (0.5-2.0)
    pub rate: f32,
    /// Voice pitch (0.5-2.0)
    pub pitch: f32,
    /// Voice name/ID
    pub voice: Option<String>,
}

/// AAC Adapter implementation
pub struct AACAdapter {
    /// Symbol dictionary
    symbol_dict: HashMap<String, AACSymbol>,
    /// Supported symbol sets
    symbol_sets: Vec<SymbolSet>,
    /// Default language
    language: String,
    /// Connection status
    connected: bool,
    /// Feedback symbol mappings
    feedback_symbols: HashMap<MessageType, Vec<AACSymbol>>,
}

impl AACAdapter {
    /// Create a new AAC adapter
    pub fn new() -> Self {
        let mut adapter = Self {
            symbol_dict: HashMap::new(),
            symbol_sets: vec![SymbolSet::PCS, SymbolSet::ARASAAC],
            language: "en-US".to_string(),
            connected: true,
            feedback_symbols: HashMap::new(),
        };
        adapter.load_default_symbols();
        adapter
    }

    /// Create with specific language
    pub fn with_language(language: &str) -> Self {
        let mut adapter = Self::new();
        adapter.language = language.to_string();
        adapter
    }

    /// Load default symbol mappings
    fn load_default_symbols(&mut self) {
        // Yes/No symbols
        self.add_symbol(AACSymbol {
            id: "yes".to_string(),
            symbol_set: SymbolSet::PCS,
            meaning: "Yes".to_string(),
            category: SymbolCategory::YesNo,
            unicode: Some("✓".to_string()),
            image_url: None,
            spoken_text: Some("Yes".to_string()),
        });

        self.add_symbol(AACSymbol {
            id: "no".to_string(),
            symbol_set: SymbolSet::PCS,
            meaning: "No".to_string(),
            category: SymbolCategory::YesNo,
            unicode: Some("✗".to_string()),
            image_url: None,
            spoken_text: Some("No".to_string()),
        });

        // Number symbols
        for i in 0..=10 {
            self.add_symbol(AACSymbol {
                id: format!("number_{}", i),
                symbol_set: SymbolSet::PCS,
                meaning: i.to_string(),
                category: SymbolCategory::Numbers,
                unicode: Some(i.to_string()),
                image_url: None,
                spoken_text: Some(i.to_string()),
            });
        }

        // Letter symbols
        for c in 'A'..='Z' {
            self.add_symbol(AACSymbol {
                id: format!("letter_{}", c.to_lowercase()),
                symbol_set: SymbolSet::PCS,
                meaning: c.to_string(),
                category: SymbolCategory::Letters,
                unicode: Some(c.to_string()),
                image_url: None,
                spoken_text: Some(c.to_string()),
            });
        }

        // Feedback symbols
        self.feedback_symbols.insert(MessageType::Positive, vec![
            AACSymbol {
                id: "great".to_string(),
                symbol_set: SymbolSet::PCS,
                meaning: "Great job!".to_string(),
                category: SymbolCategory::Social,
                unicode: Some("🎉".to_string()),
                image_url: None,
                spoken_text: Some("Great job!".to_string()),
            },
        ]);

        self.feedback_symbols.insert(MessageType::Encouragement, vec![
            AACSymbol {
                id: "try_again".to_string(),
                symbol_set: SymbolSet::PCS,
                meaning: "Try again".to_string(),
                category: SymbolCategory::Social,
                unicode: Some("💪".to_string()),
                image_url: None,
                spoken_text: Some("Try again, you can do it!".to_string()),
            },
        ]);

        self.feedback_symbols.insert(MessageType::Correction, vec![
            AACSymbol {
                id: "not_quite".to_string(),
                symbol_set: SymbolSet::PCS,
                meaning: "Not quite".to_string(),
                category: SymbolCategory::Social,
                unicode: Some("🤔".to_string()),
                image_url: None,
                spoken_text: Some("Not quite right. Let me show you.".to_string()),
            },
        ]);
    }

    /// Add a symbol to the dictionary
    pub fn add_symbol(&mut self, symbol: AACSymbol) {
        self.symbol_dict.insert(symbol.id.clone(), symbol);
    }

    /// Get symbol by ID
    pub fn get_symbol(&self, id: &str) -> Option<&AACSymbol> {
        self.symbol_dict.get(id)
    }

    /// Set connection status
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }
}

impl Default for AACAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl AACEducation for AACAdapter {
    fn symbol_to_answer(&self, symbol: &AACSymbol, question_type: &QuestionType) -> Option<AssessmentAnswer> {
        let answer_text = match question_type {
            QuestionType::MultipleChoice => {
                // Map letter symbols to answer choices
                if symbol.category == SymbolCategory::Letters {
                    symbol.meaning.clone()
                } else {
                    symbol.meaning.clone()
                }
            }
            QuestionType::TrueFalse => {
                // Map yes/no to true/false
                if symbol.id == "yes" {
                    "true".to_string()
                } else if symbol.id == "no" {
                    "false".to_string()
                } else {
                    return None;
                }
            }
            QuestionType::ShortAnswer | QuestionType::Essay => {
                // Use symbol meaning directly
                symbol.meaning.clone()
            }
            QuestionType::Matching => {
                symbol.meaning.clone()
            }
            QuestionType::FillBlank => {
                symbol.meaning.clone()
            }
            // Handle other question types
            _ => symbol.meaning.clone(),
        };

        Some(AssessmentAnswer {
            answer_id: Uuid::new_v4(),
            symbols: vec![symbol.clone()],
            answer_text,
            confidence: 1.0,
            input_method: AACInputMethod::Touch,
        })
    }

    fn voice_response(&self, text: &str, language: &str) -> Result<VoiceOutput> {
        Ok(VoiceOutput {
            text: text.to_string(),
            language: language.to_string(),
            rate: 1.0,
            pitch: 1.0,
            voice: None,
        })
    }

    fn generate_feedback(&self, result: &AnswerResult) -> AACMessage {
        let message_type = if result.correct {
            MessageType::Positive
        } else if result.score > 0.5 {
            MessageType::Encouragement
        } else {
            MessageType::Correction
        };

        let symbols = self.feedback_symbols
            .get(&message_type)
            .cloned()
            .unwrap_or_default();

        let text = if result.correct {
            format!("Correct! {}", result.feedback)
        } else if let Some(correct) = &result.correct_answer {
            format!("{}. The correct answer is: {}", result.feedback, correct)
        } else {
            result.feedback.clone()
        };

        let spoken_text = symbols.first()
            .and_then(|s| s.spoken_text.clone())
            .unwrap_or_else(|| text.clone());

        AACMessage {
            id: Uuid::new_v4(),
            symbols,
            text,
            spoken_text,
            message_type,
        }
    }

    fn available_symbol_sets(&self) -> Vec<SymbolSet> {
        self.symbol_sets.clone()
    }

    fn text_to_symbols(&self, text: &str) -> Vec<AACSymbol> {
        let words: Vec<&str> = text.split_whitespace().collect();
        let mut symbols = Vec::new();

        for word in words {
            // Try to find a matching symbol
            let lower_word = word.to_lowercase();
            if let Some(symbol) = self.symbol_dict.get(&lower_word) {
                symbols.push(symbol.clone());
            } else {
                // Check number symbols
                if let Ok(num) = word.parse::<i32>() {
                    if let Some(symbol) = self.symbol_dict.get(&format!("number_{}", num)) {
                        symbols.push(symbol.clone());
                    }
                }
                // Check letter symbols
                else if word.len() == 1 {
                    if let Some(symbol) = self.symbol_dict.get(&format!("letter_{}", lower_word)) {
                        symbols.push(symbol.clone());
                    }
                }
            }
        }

        symbols
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_aac_adapter() {
        let adapter = AACAdapter::new();
        assert!(adapter.is_connected());
        assert!(!adapter.available_symbol_sets().is_empty());
    }

    #[test]
    fn test_symbol_to_answer_true_false() {
        let adapter = AACAdapter::new();
        let yes_symbol = adapter.get_symbol("yes").unwrap();

        let answer = adapter.symbol_to_answer(yes_symbol, &QuestionType::TrueFalse);
        assert!(answer.is_some());
        assert_eq!(answer.unwrap().answer_text, "true");
    }

    #[test]
    fn test_symbol_to_answer_multiple_choice() {
        let adapter = AACAdapter::new();
        let letter_a = adapter.get_symbol("letter_a").unwrap();

        let answer = adapter.symbol_to_answer(letter_a, &QuestionType::MultipleChoice);
        assert!(answer.is_some());
        assert_eq!(answer.unwrap().answer_text, "A");
    }

    #[test]
    fn test_generate_feedback_correct() {
        let adapter = AACAdapter::new();
        let result = AnswerResult {
            correct: true,
            score: 1.0,
            feedback: "Well done!".to_string(),
            correct_answer: None,
        };

        let message = adapter.generate_feedback(&result);
        assert_eq!(message.message_type, MessageType::Positive);
        assert!(message.text.contains("Correct!"));
    }

    #[test]
    fn test_generate_feedback_incorrect() {
        let adapter = AACAdapter::new();
        let result = AnswerResult {
            correct: false,
            score: 0.0,
            feedback: "Not quite right.".to_string(),
            correct_answer: Some("B".to_string()),
        };

        let message = adapter.generate_feedback(&result);
        assert_eq!(message.message_type, MessageType::Correction);
        assert!(message.text.contains("correct answer is: B"));
    }

    #[test]
    fn test_text_to_symbols() {
        let adapter = AACAdapter::new();
        let symbols = adapter.text_to_symbols("yes 1 A");

        assert_eq!(symbols.len(), 3);
    }

    #[test]
    fn test_voice_response() {
        let adapter = AACAdapter::new();
        let output = adapter.voice_response("Hello", "en-US");

        assert!(output.is_ok());
        let voice = output.unwrap();
        assert_eq!(voice.text, "Hello");
        assert_eq!(voice.language, "en-US");
    }
}
