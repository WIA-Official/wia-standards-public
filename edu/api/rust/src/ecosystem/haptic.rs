//! Haptic Integration
//! 弘益人間 - Enable tactile learning feedback and braille output

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;

use crate::error::{EduError, Result};

/// Haptic Education trait for tactile learning
pub trait HapticEducation: Send + Sync {
    /// Play a haptic pattern for feedback
    fn play_pattern(&self, pattern: &HapticPattern) -> Result<()>;

    /// Send text to braille display
    fn send_to_braille(&self, text: &str, options: &BrailleOptions) -> Result<BrailleOutput>;

    /// Get notification pattern for event type
    fn notification_pattern(&self, event: NotificationEvent) -> HapticPattern;

    /// Create learning feedback pattern
    fn learning_feedback(&self, result: LearningResult) -> HapticPattern;

    /// Check if haptic device is connected
    fn is_connected(&self) -> bool;

    /// Get braille display info
    fn braille_display_info(&self) -> Option<BrailleDisplayInfo>;
}

/// Haptic pattern for tactile feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticPattern {
    /// Pattern ID
    pub id: String,
    /// Pattern name
    pub name: String,
    /// Vibration segments
    pub segments: Vec<HapticSegment>,
    /// Total duration (ms)
    pub duration_ms: u32,
    /// Pattern type
    pub pattern_type: PatternType,
}

/// Single haptic segment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HapticSegment {
    /// Duration of this segment (ms)
    pub duration_ms: u32,
    /// Intensity (0.0-1.0)
    pub intensity: f32,
    /// Frequency (Hz) if applicable
    pub frequency: Option<u32>,
    /// Segment type
    pub segment_type: SegmentType,
}

/// Segment type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SegmentType {
    /// Vibration
    Vibration,
    /// Pause
    Pause,
    /// Pulse
    Pulse,
    /// Ramp up
    RampUp,
    /// Ramp down
    RampDown,
}

/// Pattern type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PatternType {
    /// Success/correct answer
    Success,
    /// Error/incorrect answer
    Error,
    /// Warning
    Warning,
    /// Information
    Info,
    /// Navigation feedback
    Navigation,
    /// Timer alert
    Timer,
    /// Custom pattern
    Custom,
}

/// Notification event types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NotificationEvent {
    /// New content available
    NewContent,
    /// Question ready
    QuestionReady,
    /// Timer warning (time running low)
    TimerWarning,
    /// Timer expired
    TimerExpired,
    /// Answer submitted
    AnswerSubmitted,
    /// Course completed
    CourseCompleted,
    /// Achievement unlocked
    Achievement,
    /// Message received
    MessageReceived,
}

/// Learning result for feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LearningResult {
    /// Whether correct
    pub correct: bool,
    /// Score (0.0-1.0)
    pub score: f32,
    /// Streak count
    pub streak: u32,
    /// First attempt
    pub first_attempt: bool,
}

/// Braille output options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrailleOptions {
    /// Braille grade (1 for uncontracted, 2 for contracted)
    pub grade: u8,
    /// Language code
    pub language: String,
    /// Use computer braille for code/symbols
    pub computer_braille: bool,
    /// Flash new text
    pub flash_new: bool,
    /// Scroll speed (characters per second)
    pub scroll_speed: Option<f32>,
}

impl Default for BrailleOptions {
    fn default() -> Self {
        Self {
            grade: 2, // Contracted braille
            language: "en-US".to_string(),
            computer_braille: false,
            flash_new: true,
            scroll_speed: None,
        }
    }
}

/// Braille output result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrailleOutput {
    /// Original text
    pub text: String,
    /// Braille representation (Unicode)
    pub braille: String,
    /// Number of cells used
    pub cells_used: usize,
    /// Total cells available
    pub cells_available: usize,
    /// Pages if text exceeds display
    pub pages: usize,
}

/// Braille display information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrailleDisplayInfo {
    /// Display name
    pub name: String,
    /// Number of cells
    pub cells: usize,
    /// Number of rows
    pub rows: usize,
    /// Has input keys
    pub has_input: bool,
    /// Connection type
    pub connection: String,
}

/// Haptic Adapter implementation
pub struct HapticAdapter {
    /// Predefined patterns
    patterns: HashMap<String, HapticPattern>,
    /// Braille translation table (simplified)
    braille_table: HashMap<char, char>,
    /// Braille display info
    display_info: Option<BrailleDisplayInfo>,
    /// Connection status
    connected: bool,
}

impl HapticAdapter {
    /// Create a new haptic adapter
    pub fn new() -> Self {
        let mut adapter = Self {
            patterns: HashMap::new(),
            braille_table: HashMap::new(),
            display_info: Some(BrailleDisplayInfo {
                name: "Virtual Braille Display".to_string(),
                cells: 40,
                rows: 1,
                has_input: true,
                connection: "USB".to_string(),
            }),
            connected: true,
        };
        adapter.load_default_patterns();
        adapter.load_braille_table();
        adapter
    }

    /// Load default haptic patterns
    fn load_default_patterns(&mut self) {
        // Success pattern
        self.patterns.insert("success".to_string(), HapticPattern {
            id: "success".to_string(),
            name: "Success".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.8,
                    frequency: Some(200),
                    segment_type: SegmentType::Vibration,
                },
                HapticSegment {
                    duration_ms: 50,
                    intensity: 0.0,
                    frequency: None,
                    segment_type: SegmentType::Pause,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 1.0,
                    frequency: Some(250),
                    segment_type: SegmentType::Vibration,
                },
            ],
            duration_ms: 250,
            pattern_type: PatternType::Success,
        });

        // Error pattern
        self.patterns.insert("error".to_string(), HapticPattern {
            id: "error".to_string(),
            name: "Error".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 300,
                    intensity: 0.6,
                    frequency: Some(100),
                    segment_type: SegmentType::Vibration,
                },
            ],
            duration_ms: 300,
            pattern_type: PatternType::Error,
        });

        // Warning pattern
        self.patterns.insert("warning".to_string(), HapticPattern {
            id: "warning".to_string(),
            name: "Warning".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.7,
                    frequency: Some(150),
                    segment_type: SegmentType::Pulse,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.0,
                    frequency: None,
                    segment_type: SegmentType::Pause,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.7,
                    frequency: Some(150),
                    segment_type: SegmentType::Pulse,
                },
            ],
            duration_ms: 300,
            pattern_type: PatternType::Warning,
        });

        // Timer pattern
        self.patterns.insert("timer".to_string(), HapticPattern {
            id: "timer".to_string(),
            name: "Timer Alert".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 50,
                    intensity: 0.5,
                    frequency: Some(200),
                    segment_type: SegmentType::Pulse,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.0,
                    frequency: None,
                    segment_type: SegmentType::Pause,
                },
                HapticSegment {
                    duration_ms: 50,
                    intensity: 0.5,
                    frequency: Some(200),
                    segment_type: SegmentType::Pulse,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.0,
                    frequency: None,
                    segment_type: SegmentType::Pause,
                },
                HapticSegment {
                    duration_ms: 50,
                    intensity: 0.5,
                    frequency: Some(200),
                    segment_type: SegmentType::Pulse,
                },
            ],
            duration_ms: 350,
            pattern_type: PatternType::Timer,
        });

        // Navigation pattern
        self.patterns.insert("nav".to_string(), HapticPattern {
            id: "nav".to_string(),
            name: "Navigation".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 30,
                    intensity: 0.4,
                    frequency: Some(180),
                    segment_type: SegmentType::Vibration,
                },
            ],
            duration_ms: 30,
            pattern_type: PatternType::Navigation,
        });

        // Achievement pattern
        self.patterns.insert("achievement".to_string(), HapticPattern {
            id: "achievement".to_string(),
            name: "Achievement".to_string(),
            segments: vec![
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.3,
                    frequency: Some(150),
                    segment_type: SegmentType::RampUp,
                },
                HapticSegment {
                    duration_ms: 200,
                    intensity: 1.0,
                    frequency: Some(300),
                    segment_type: SegmentType::Vibration,
                },
                HapticSegment {
                    duration_ms: 100,
                    intensity: 0.3,
                    frequency: Some(150),
                    segment_type: SegmentType::RampDown,
                },
            ],
            duration_ms: 400,
            pattern_type: PatternType::Success,
        });
    }

    /// Load basic braille translation table (English letters)
    fn load_braille_table(&mut self) {
        // Basic English Grade 1 Braille
        let mappings = [
            ('a', '⠁'), ('b', '⠃'), ('c', '⠉'), ('d', '⠙'), ('e', '⠑'),
            ('f', '⠋'), ('g', '⠛'), ('h', '⠓'), ('i', '⠊'), ('j', '⠚'),
            ('k', '⠅'), ('l', '⠇'), ('m', '⠍'), ('n', '⠝'), ('o', '⠕'),
            ('p', '⠏'), ('q', '⠟'), ('r', '⠗'), ('s', '⠎'), ('t', '⠞'),
            ('u', '⠥'), ('v', '⠧'), ('w', '⠺'), ('x', '⠭'), ('y', '⠽'),
            ('z', '⠵'),
            ('1', '⠁'), ('2', '⠃'), ('3', '⠉'), ('4', '⠙'), ('5', '⠑'),
            ('6', '⠋'), ('7', '⠛'), ('8', '⠓'), ('9', '⠊'), ('0', '⠚'),
            (' ', '⠀'), ('.', '⠲'), (',', '⠂'), ('?', '⠦'), ('!', '⠖'),
        ];

        for (c, b) in mappings {
            self.braille_table.insert(c, b);
        }
    }

    /// Set connection status
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }

    /// Set braille display info
    pub fn set_display_info(&mut self, info: BrailleDisplayInfo) {
        self.display_info = Some(info);
    }

    /// Translate text to braille
    fn translate_to_braille(&self, text: &str) -> String {
        text.to_lowercase()
            .chars()
            .map(|c| *self.braille_table.get(&c).unwrap_or(&c))
            .collect()
    }
}

impl Default for HapticAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl HapticEducation for HapticAdapter {
    fn play_pattern(&self, pattern: &HapticPattern) -> Result<()> {
        if !self.connected {
            return Err(EduError::NotFound("Haptic device not connected".to_string()));
        }
        // In real implementation, would send to hardware
        Ok(())
    }

    fn send_to_braille(&self, text: &str, options: &BrailleOptions) -> Result<BrailleOutput> {
        let display = self.display_info.as_ref()
            .ok_or_else(|| EduError::NotFound("No braille display".to_string()))?;

        let braille = self.translate_to_braille(text);
        let cells_used = braille.chars().count();
        let cells_available = display.cells;
        let pages = (cells_used + cells_available - 1) / cells_available;

        Ok(BrailleOutput {
            text: text.to_string(),
            braille,
            cells_used,
            cells_available,
            pages,
        })
    }

    fn notification_pattern(&self, event: NotificationEvent) -> HapticPattern {
        let pattern_id = match event {
            NotificationEvent::NewContent => "nav",
            NotificationEvent::QuestionReady => "nav",
            NotificationEvent::TimerWarning => "warning",
            NotificationEvent::TimerExpired => "timer",
            NotificationEvent::AnswerSubmitted => "nav",
            NotificationEvent::CourseCompleted => "achievement",
            NotificationEvent::Achievement => "achievement",
            NotificationEvent::MessageReceived => "nav",
        };

        self.patterns.get(pattern_id)
            .cloned()
            .unwrap_or_else(|| self.patterns.get("nav").cloned().unwrap())
    }

    fn learning_feedback(&self, result: LearningResult) -> HapticPattern {
        if result.correct {
            if result.streak >= 3 {
                // Extended success for streaks
                self.patterns.get("achievement").cloned()
                    .unwrap_or_else(|| self.patterns.get("success").cloned().unwrap())
            } else {
                self.patterns.get("success").cloned().unwrap()
            }
        } else {
            self.patterns.get("error").cloned().unwrap()
        }
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn braille_display_info(&self) -> Option<BrailleDisplayInfo> {
        self.display_info.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_haptic_adapter() {
        let adapter = HapticAdapter::new();
        assert!(adapter.is_connected());
        assert!(adapter.braille_display_info().is_some());
    }

    #[test]
    fn test_play_pattern() {
        let adapter = HapticAdapter::new();

        let pattern = adapter.notification_pattern(NotificationEvent::Achievement);
        let result = adapter.play_pattern(&pattern);
        assert!(result.is_ok());
    }

    #[test]
    fn test_braille_output() {
        let adapter = HapticAdapter::new();
        let options = BrailleOptions::default();

        let output = adapter.send_to_braille("hello", &options).unwrap();
        assert_eq!(output.text, "hello");
        assert_eq!(output.braille, "⠓⠑⠇⠇⠕");
        assert_eq!(output.cells_used, 5);
    }

    #[test]
    fn test_braille_with_spaces() {
        let adapter = HapticAdapter::new();
        let options = BrailleOptions::default();

        let output = adapter.send_to_braille("a b c", &options).unwrap();
        assert_eq!(output.braille, "⠁⠀⠃⠀⠉");
    }

    #[test]
    fn test_learning_feedback_correct() {
        let adapter = HapticAdapter::new();

        let result = LearningResult {
            correct: true,
            score: 1.0,
            streak: 1,
            first_attempt: true,
        };

        let pattern = adapter.learning_feedback(result);
        assert_eq!(pattern.pattern_type, PatternType::Success);
    }

    #[test]
    fn test_learning_feedback_streak() {
        let adapter = HapticAdapter::new();

        let result = LearningResult {
            correct: true,
            score: 1.0,
            streak: 5, // Streak triggers achievement
            first_attempt: true,
        };

        let pattern = adapter.learning_feedback(result);
        // Achievement pattern should be used for streaks
        assert!(pattern.duration_ms > 300);
    }

    #[test]
    fn test_learning_feedback_incorrect() {
        let adapter = HapticAdapter::new();

        let result = LearningResult {
            correct: false,
            score: 0.0,
            streak: 0,
            first_attempt: true,
        };

        let pattern = adapter.learning_feedback(result);
        assert_eq!(pattern.pattern_type, PatternType::Error);
    }

    #[test]
    fn test_notification_patterns() {
        let adapter = HapticAdapter::new();

        let timer_pattern = adapter.notification_pattern(NotificationEvent::TimerWarning);
        assert_eq!(timer_pattern.pattern_type, PatternType::Warning);

        let achievement_pattern = adapter.notification_pattern(NotificationEvent::Achievement);
        assert_eq!(achievement_pattern.name, "Achievement");
    }
}
