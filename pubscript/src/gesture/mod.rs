//! Gesture input module with WIA Talk integration
//!
//! WIA Talk is a gesture-based Korean Sign Language system with 94 gestures
//! representing Korean phonemes (consonants and vowels).
//!
//! ## Philosophy
//!
//! Input methods are equal:
//! - Keyboard → Text
//! - Voice → Speech Recognition → Text
//! - Gestures (WIA Talk) → Text
//!
//! All three input methods produce the same IR, ensuring equal access.
//!
//! ## Example
//!
//! ```rust
//! use wia_pubscript::gesture::{GestureInput, Hand, gesture_to_text};
//! use std::time::Duration;
//!
//! let gestures = vec![
//!     GestureInput::new(1, Hand::Right, Duration::from_secs(0)),  // ㄱ
//!     GestureInput::new(19, Hand::Right, Duration::from_secs(1)), // ㅏ
//! ];
//!
//! let text = gesture_to_text(&gestures);
//! // Produces Korean text from gestures
//! ```

use crate::ir::{ContentNode, Metadata, NodeMetadata, PubScriptDocument, Representations};
use crate::SemanticType;
use crate::ir::representations::visual::{VisualRep, Layout, VisualStyle};
use std::collections::HashMap;
use std::time::Duration;

/// Hand used for gesture
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Hand {
    /// Left hand
    Left,
    /// Right hand
    Right,
    /// Both hands
    Both,
}

impl Hand {
    /// Convert to string
    pub fn as_str(&self) -> &'static str {
        match self {
            Hand::Left => "left",
            Hand::Right => "right",
            Hand::Both => "both",
        }
    }
}

/// Gesture input from WIA Talk
#[derive(Debug, Clone)]
pub struct GestureInput {
    /// WIA Talk gesture ID (1-94)
    pub gesture_id: u32,
    /// Hand used for gesture
    pub hand: Hand,
    /// Timestamp of gesture
    pub timestamp: Duration,
    /// Optional confidence score (0.0-1.0)
    pub confidence: Option<f32>,
}

impl GestureInput {
    /// Create new gesture input
    pub fn new(gesture_id: u32, hand: Hand, timestamp: Duration) -> Self {
        Self {
            gesture_id,
            hand,
            timestamp,
            confidence: None,
        }
    }

    /// Create gesture with confidence
    pub fn with_confidence(mut self, confidence: f32) -> Self {
        self.confidence = Some(confidence);
        self
    }

    /// Get phoneme for this gesture
    pub fn to_phoneme(&self) -> Option<&'static str> {
        WIA_TALK_GESTURES
            .iter()
            .find(|(id, _)| *id == self.gesture_id)
            .map(|(_, phoneme)| *phoneme)
    }
}

/// WIA Talk gesture mappings (94 gestures)
///
/// Consonants (ㄱ-ㅎ): IDs 1-19
/// Vowels (ㅏ-ㅣ): IDs 20-40
/// Double consonants: IDs 41-45
/// Complex vowels: IDs 46-94
pub const WIA_TALK_GESTURES: [(u32, &str); 94] = [
    // Basic consonants (19)
    (1, "ㄱ"), (2, "ㄴ"), (3, "ㄷ"), (4, "ㄹ"), (5, "ㅁ"),
    (6, "ㅂ"), (7, "ㅅ"), (8, "ㅇ"), (9, "ㅈ"), (10, "ㅊ"),
    (11, "ㅋ"), (12, "ㅌ"), (13, "ㅍ"), (14, "ㅎ"),
    // Additional consonants
    (15, "ㄲ"), (16, "ㄸ"), (17, "ㅃ"), (18, "ㅆ"), (19, "ㅉ"),

    // Basic vowels (21)
    (20, "ㅏ"), (21, "ㅑ"), (22, "ㅓ"), (23, "ㅕ"), (24, "ㅗ"),
    (25, "ㅛ"), (26, "ㅜ"), (27, "ㅠ"), (28, "ㅡ"), (29, "ㅣ"),
    (30, "ㅐ"), (31, "ㅒ"), (32, "ㅔ"), (33, "ㅖ"),

    // Complex vowels (14)
    (34, "ㅘ"), (35, "ㅙ"), (36, "ㅚ"), (37, "ㅝ"), (38, "ㅞ"),
    (39, "ㅟ"), (40, "ㅢ"),

    // Final consonants (batchim) (27)
    (41, "ㄱ_"), (42, "ㄴ_"), (43, "ㄷ_"), (44, "ㄹ_"), (45, "ㅁ_"),
    (46, "ㅂ_"), (47, "ㅅ_"), (48, "ㅇ_"), (49, "ㅈ_"), (50, "ㅊ_"),
    (51, "ㅋ_"), (52, "ㅌ_"), (53, "ㅍ_"), (54, "ㅎ_"),
    (55, "ㄲ_"), (56, "ㄳ_"), (57, "ㄵ_"), (58, "ㄶ_"), (59, "ㄺ_"),
    (60, "ㄻ_"), (61, "ㄼ_"), (62, "ㄽ_"), (63, "ㄾ_"), (64, "ㄿ_"),
    (65, "ㅀ_"), (66, "ㅄ_"), (67, "ㅆ_"),

    // Special gestures (27)
    (68, "SPACE"), (69, "BACKSPACE"), (70, "ENTER"),
    (71, "SHIFT"), (72, "CAPS"), (73, "NUM"),
    (74, "PUNCT"), (75, "EMOJI"), (76, "SYMBOL"),
    (77, "0"), (78, "1"), (79, "2"), (80, "3"), (81, "4"),
    (82, "5"), (83, "6"), (84, "7"), (85, "8"), (86, "9"),
    (87, "."), (88, ","), (89, "?"), (90, "!"),
    (91, "UNDO"), (92, "REDO"), (93, "SELECT"), (94, "CONFIRM"),
];

/// Convert gesture sequence to Korean text
pub fn gesture_to_text(gestures: &[GestureInput]) -> String {
    let mut result = String::new();

    for gesture in gestures {
        if let Some(phoneme) = gesture.to_phoneme() {
            // Special handling for control gestures
            match phoneme {
                "SPACE" => result.push(' '),
                "BACKSPACE" => {
                    result.pop();
                }
                "ENTER" => result.push('\n'),
                "." => result.push('.'),
                "," => result.push(','),
                "?" => result.push('?'),
                "!" => result.push('!'),
                // Numeric gestures
                "0" => result.push('0'),
                "1" => result.push('1'),
                "2" => result.push('2'),
                "3" => result.push('3'),
                "4" => result.push('4'),
                "5" => result.push('5'),
                "6" => result.push('6'),
                "7" => result.push('7'),
                "8" => result.push('8'),
                "9" => result.push('9'),
                // Korean phonemes (simplified - real implementation would compose syllables)
                _ => result.push_str(phoneme.trim_end_matches('_')),
            }
        }
    }

    result
}

/// Convert gesture sequence to IR document
pub fn gesture_to_ir(gestures: Vec<GestureInput>) -> PubScriptDocument {
    let text = gesture_to_text(&gestures);

    // Create content node with gestural input metadata
    let mut node = ContentNode {
        id: "gesture-input-1".to_string(),
        representations: Representations {
            visual: Some(VisualRep {
                text: Some(text.clone()),
                image: None,
                layout: Layout::Block,
                style: VisualStyle::default(),
            }),
            auditory: None,
            tactile: None,
            spatial: None,
            gestural: None,
        },
        children: Vec::new(),
        metadata: NodeMetadata {
            semantic_type: Some(SemanticType::Paragraph),
            language: Some("ko".to_string()),
            aria_label: None,
            custom: {
                let mut custom = HashMap::new();
                custom.insert("input_method".to_string(), "wia_talk".to_string());
                custom.insert("gesture_count".to_string(), gestures.len().to_string());
                custom
            },
        },
    };

    // Store gesture timing information
    if !gestures.is_empty() {
        if let Some(first) = gestures.first() {
            node.metadata.custom.insert(
                "start_time".to_string(),
                format!("{:?}", first.timestamp),
            );
        }
        if let Some(last) = gestures.last() {
            node.metadata.custom.insert(
                "end_time".to_string(),
                format!("{:?}", last.timestamp),
            );
        }
    }

    PubScriptDocument {
        metadata: Metadata {
            title: Some("WIA Talk Gesture Input".to_string()),
            author: None,
            version: "3.0".to_string(),
            language: Some("ko".to_string()),
            created_at: None,
            updated_at: None,
        },
        content: vec![node],
        timeline: None,
    }
}

/// Get gesture name by ID
pub fn get_gesture_name(gesture_id: u32) -> Option<&'static str> {
    WIA_TALK_GESTURES
        .iter()
        .find(|(id, _)| *id == gesture_id)
        .map(|(_, name)| *name)
}

/// Get gesture ID by name
pub fn get_gesture_id(name: &str) -> Option<u32> {
    WIA_TALK_GESTURES
        .iter()
        .find(|(_, n)| *n == name)
        .map(|(id, _)| *id)
}

/// List all available gestures
pub fn list_gestures() -> Vec<(u32, &'static str)> {
    WIA_TALK_GESTURES.to_vec()
}

/// Get gesture category
pub fn get_gesture_category(gesture_id: u32) -> Option<&'static str> {
    match gesture_id {
        1..=14 => Some("basic_consonants"),
        15..=19 => Some("double_consonants"),
        20..=33 => Some("basic_vowels"),
        34..=40 => Some("complex_vowels"),
        41..=67 => Some("final_consonants"),
        68..=76 => Some("control"),
        77..=86 => Some("numbers"),
        87..=90 => Some("punctuation"),
        91..=94 => Some("editing"),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gesture_to_phoneme() {
        let gesture = GestureInput::new(1, Hand::Right, Duration::from_secs(0));
        assert_eq!(gesture.to_phoneme(), Some("ㄱ"));

        let gesture = GestureInput::new(20, Hand::Right, Duration::from_secs(0));
        assert_eq!(gesture.to_phoneme(), Some("ㅏ"));
    }

    #[test]
    fn test_gesture_to_text() {
        let gestures = vec![
            GestureInput::new(1, Hand::Right, Duration::from_secs(0)),  // ㄱ
            GestureInput::new(20, Hand::Right, Duration::from_secs(1)), // ㅏ
            GestureInput::new(68, Hand::Both, Duration::from_secs(2)),  // SPACE
            GestureInput::new(2, Hand::Right, Duration::from_secs(3)),  // ㄴ
            GestureInput::new(22, Hand::Right, Duration::from_secs(4)), // ㅓ
        ];

        let text = gesture_to_text(&gestures);
        assert!(text.contains("ㄱ"));
        assert!(text.contains("ㅏ"));
        assert!(text.contains(" ")); // space
    }

    #[test]
    fn test_gesture_to_ir() {
        let gestures = vec![
            GestureInput::new(1, Hand::Right, Duration::from_secs(0)),
            GestureInput::new(20, Hand::Right, Duration::from_secs(1)),
        ];

        let doc = gesture_to_ir(gestures);
        assert_eq!(doc.metadata.title, Some("WIA Talk Gesture Input".to_string()));
        assert_eq!(doc.content.len(), 1);
        assert_eq!(
            doc.content[0].metadata.custom.get("input_method"),
            Some(&"wia_talk".to_string())
        );
    }

    #[test]
    fn test_wia_talk_gestures_count() {
        assert_eq!(WIA_TALK_GESTURES.len(), 94);
    }

    #[test]
    fn test_get_gesture_name() {
        assert_eq!(get_gesture_name(1), Some("ㄱ"));
        assert_eq!(get_gesture_name(20), Some("ㅏ"));
        assert_eq!(get_gesture_name(68), Some("SPACE"));
        assert_eq!(get_gesture_name(999), None);
    }

    #[test]
    fn test_get_gesture_id() {
        assert_eq!(get_gesture_id("ㄱ"), Some(1));
        assert_eq!(get_gesture_id("ㅏ"), Some(20));
        assert_eq!(get_gesture_id("SPACE"), Some(68));
        assert_eq!(get_gesture_id("nonexistent"), None);
    }

    #[test]
    fn test_gesture_categories() {
        assert_eq!(get_gesture_category(1), Some("basic_consonants"));
        assert_eq!(get_gesture_category(15), Some("double_consonants"));
        assert_eq!(get_gesture_category(20), Some("basic_vowels"));
        assert_eq!(get_gesture_category(34), Some("complex_vowels"));
        assert_eq!(get_gesture_category(41), Some("final_consonants"));
        assert_eq!(get_gesture_category(68), Some("control"));
        assert_eq!(get_gesture_category(77), Some("numbers"));
        assert_eq!(get_gesture_category(87), Some("punctuation"));
        assert_eq!(get_gesture_category(91), Some("editing"));
    }

    #[test]
    fn test_list_gestures() {
        let gestures = list_gestures();
        assert_eq!(gestures.len(), 94);
        assert_eq!(gestures[0], (1, "ㄱ"));
        assert_eq!(gestures[93], (94, "CONFIRM"));
    }

    #[test]
    fn test_control_gestures() {
        let gestures = vec![
            GestureInput::new(1, Hand::Right, Duration::from_secs(0)),  // ㄱ
            GestureInput::new(68, Hand::Both, Duration::from_secs(1)),  // SPACE
            GestureInput::new(2, Hand::Right, Duration::from_secs(2)),  // ㄴ
            GestureInput::new(87, Hand::Right, Duration::from_secs(3)), // .
        ];

        let text = gesture_to_text(&gestures);
        assert!(text.contains(" "));
        assert!(text.contains("."));
    }
}
