//! Emergency communication detection and handling

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Emergency detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyDetectionResult {
    /// Whether emergency was detected
    pub is_emergency: bool,

    /// Urgency level
    pub urgency_level: UrgencyLevel,

    /// Detected keywords
    pub keywords: Vec<String>,

    /// Confidence score
    pub confidence: f32,

    /// Recommended action
    pub action: EmergencyAction,
}

/// Urgency levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UrgencyLevel {
    /// Normal processing
    Normal = 1,
    /// Low urgency
    Low = 2,
    /// Medium urgency
    Medium = 3,
    /// High urgency - priority processing
    High = 4,
    /// Critical - immediate action required
    Critical = 5,
}

/// Emergency action recommendations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmergencyAction {
    /// Normal processing
    Normal,
    /// Priority queue processing
    PriorityProcess,
    /// Fast track mode
    FastTrack,
    /// Enable offline fallback
    OfflineFallback,
    /// Connect to emergency services
    ConnectEmergencyServices,
}

/// Emergency keywords by language
#[derive(Debug, Clone)]
pub struct EmergencyKeywords {
    keywords: HashMap<String, Vec<EmergencyKeyword>>,
}

#[derive(Debug, Clone)]
struct EmergencyKeyword {
    word: String,
    urgency: UrgencyLevel,
    category: EmergencyCategory,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EmergencyCategory {
    LifeThreatening,
    Urgent,
    Standard,
}

impl EmergencyKeywords {
    fn new() -> Self {
        let mut keywords = HashMap::new();

        // English keywords
        keywords.insert(
            "en".to_string(),
            vec![
                EmergencyKeyword {
                    word: "help".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "emergency".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "911".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "fire".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "ambulance".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "police".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "danger".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "hurt".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "accident".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::Urgent,
                },
            ],
        );

        // Korean keywords
        keywords.insert(
            "ko".to_string(),
            vec![
                EmergencyKeyword {
                    word: "도와주세요".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "긴급".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "119".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "불이야".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "구급차".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
                EmergencyKeyword {
                    word: "경찰".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "위험".to_string(),
                    urgency: UrgencyLevel::High,
                    category: EmergencyCategory::Urgent,
                },
                EmergencyKeyword {
                    word: "살려주세요".to_string(),
                    urgency: UrgencyLevel::Critical,
                    category: EmergencyCategory::LifeThreatening,
                },
            ],
        );

        Self { keywords }
    }

    fn get(&self, language: &str) -> Option<&Vec<EmergencyKeyword>> {
        self.keywords.get(language)
    }
}

/// Emergency detector
pub struct EmergencyDetector {
    keywords: EmergencyKeywords,
    /// Minimum confidence for emergency detection
    min_confidence: f32,
}

impl EmergencyDetector {
    /// Create a new emergency detector
    pub fn new() -> Self {
        Self {
            keywords: EmergencyKeywords::new(),
            min_confidence: 0.8,
        }
    }

    /// Detect emergency in text
    pub fn detect(&self, text: &str, language: &str) -> EmergencyDetectionResult {
        let text_lower = text.to_lowercase();
        let mut detected_keywords = Vec::new();
        let mut max_urgency = UrgencyLevel::Normal;
        let mut has_life_threatening = false;

        // Get language-specific keywords, fallback to English
        let lang_keywords = self
            .keywords
            .get(language)
            .or_else(|| self.keywords.get("en"));

        if let Some(keywords) = lang_keywords {
            for kw in keywords {
                if text_lower.contains(&kw.word.to_lowercase()) {
                    detected_keywords.push(kw.word.clone());

                    if kw.urgency as u8 > max_urgency as u8 {
                        max_urgency = kw.urgency;
                    }

                    if kw.category == EmergencyCategory::LifeThreatening {
                        has_life_threatening = true;
                    }
                }
            }
        }

        let is_emergency = !detected_keywords.is_empty();
        let confidence = if is_emergency {
            // Higher confidence for multiple keywords or life-threatening
            let base_confidence = 0.85;
            let multi_keyword_bonus = (detected_keywords.len() as f32 - 1.0) * 0.05;
            let life_threat_bonus = if has_life_threatening { 0.05 } else { 0.0 };
            (base_confidence + multi_keyword_bonus + life_threat_bonus).min(1.0)
        } else {
            0.0
        };

        let action = if is_emergency {
            match max_urgency {
                UrgencyLevel::Critical => EmergencyAction::ConnectEmergencyServices,
                UrgencyLevel::High => EmergencyAction::FastTrack,
                UrgencyLevel::Medium => EmergencyAction::PriorityProcess,
                _ => EmergencyAction::Normal,
            }
        } else {
            EmergencyAction::Normal
        };

        EmergencyDetectionResult {
            is_emergency,
            urgency_level: max_urgency,
            keywords: detected_keywords,
            confidence,
            action,
        }
    }

    /// Check if text should be fast-tracked
    pub fn should_fast_track(&self, text: &str, language: &str) -> bool {
        let result = self.detect(text, language);
        matches!(
            result.urgency_level,
            UrgencyLevel::Critical | UrgencyLevel::High
        )
    }
}

impl Default for EmergencyDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// Pre-loaded emergency signs
pub struct EmergencySigns {
    signs: HashMap<String, Vec<EmergencySign>>,
}

#[derive(Debug, Clone)]
pub struct EmergencySign {
    pub gloss: String,
    pub category: String,
    pub priority: UrgencyLevel,
}

impl EmergencySigns {
    /// Create with pre-loaded emergency signs
    pub fn new() -> Self {
        let mut signs = HashMap::new();

        signs.insert(
            "ASL".to_string(),
            vec![
                EmergencySign {
                    gloss: "HELP".to_string(),
                    category: "request".to_string(),
                    priority: UrgencyLevel::Critical,
                },
                EmergencySign {
                    gloss: "EMERGENCY".to_string(),
                    category: "alert".to_string(),
                    priority: UrgencyLevel::Critical,
                },
                EmergencySign {
                    gloss: "FIRE".to_string(),
                    category: "danger".to_string(),
                    priority: UrgencyLevel::Critical,
                },
                EmergencySign {
                    gloss: "POLICE".to_string(),
                    category: "service".to_string(),
                    priority: UrgencyLevel::High,
                },
                EmergencySign {
                    gloss: "AMBULANCE".to_string(),
                    category: "service".to_string(),
                    priority: UrgencyLevel::Critical,
                },
                EmergencySign {
                    gloss: "HOSPITAL".to_string(),
                    category: "location".to_string(),
                    priority: UrgencyLevel::High,
                },
                EmergencySign {
                    gloss: "HURT".to_string(),
                    category: "condition".to_string(),
                    priority: UrgencyLevel::High,
                },
                EmergencySign {
                    gloss: "DANGER".to_string(),
                    category: "alert".to_string(),
                    priority: UrgencyLevel::High,
                },
                EmergencySign {
                    gloss: "STOP".to_string(),
                    category: "command".to_string(),
                    priority: UrgencyLevel::High,
                },
                EmergencySign {
                    gloss: "CALL".to_string(),
                    category: "action".to_string(),
                    priority: UrgencyLevel::High,
                },
            ],
        );

        Self { signs }
    }

    /// Get emergency signs for a language
    pub fn get(&self, sign_language: &str) -> Option<&Vec<EmergencySign>> {
        self.signs.get(sign_language)
    }

    /// Check if a gloss is an emergency sign
    pub fn is_emergency_sign(&self, gloss: &str, sign_language: &str) -> bool {
        self.signs
            .get(sign_language)
            .map(|signs| signs.iter().any(|s| s.gloss == gloss))
            .unwrap_or(false)
    }
}

impl Default for EmergencySigns {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_emergency_detection_english() {
        let detector = EmergencyDetector::new();

        let result = detector.detect("Help! I need an ambulance!", "en");
        assert!(result.is_emergency);
        assert_eq!(result.urgency_level, UrgencyLevel::Critical);
        assert!(result.keywords.contains(&"help".to_string()));
        assert!(result.keywords.contains(&"ambulance".to_string()));
    }

    #[test]
    fn test_emergency_detection_korean() {
        let detector = EmergencyDetector::new();

        let result = detector.detect("도와주세요! 119 불러주세요!", "ko");
        assert!(result.is_emergency);
        assert_eq!(result.urgency_level, UrgencyLevel::Critical);
    }

    #[test]
    fn test_no_emergency() {
        let detector = EmergencyDetector::new();

        let result = detector.detect("Hello, how are you today?", "en");
        assert!(!result.is_emergency);
        assert_eq!(result.urgency_level, UrgencyLevel::Normal);
    }

    #[test]
    fn test_fast_track() {
        let detector = EmergencyDetector::new();

        assert!(detector.should_fast_track("Fire! Call 911!", "en"));
        assert!(!detector.should_fast_track("Nice weather today", "en"));
    }
}
