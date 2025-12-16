//! Eye Gaze Integration
//! 弘益人間 - Enable eye gaze users to interact with learning content

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use std::collections::HashMap;

use crate::error::{EduError, Result};

/// Eye Gaze Education trait for learning interaction
pub trait EyeGazeEducation: Send + Sync {
    /// Select content based on gaze point
    fn gaze_select_content(&self, gaze: &GazePoint, content_regions: &[ContentRegion]) -> Option<ContentSelection>;

    /// Activate content using dwell time
    fn dwell_activate(&self, content_id: &str, dwell_ms: u32) -> Result<ActivationResult>;

    /// Track reading pattern from gaze path
    fn track_reading(&self, gaze_path: &[GazePoint]) -> ReadingAnalysis;

    /// Get current gaze position
    fn current_gaze(&self) -> Option<GazePoint>;

    /// Set dwell time threshold
    fn set_dwell_threshold(&mut self, threshold_ms: u32);

    /// Check if eye tracker is connected
    fn is_connected(&self) -> bool;
}

/// Gaze point representing eye position
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GazePoint {
    /// X coordinate (0.0-1.0, normalized screen position)
    pub x: f32,
    /// Y coordinate (0.0-1.0, normalized screen position)
    pub y: f32,
    /// Timestamp in milliseconds
    pub timestamp: u64,
    /// Confidence/validity (0.0-1.0)
    pub confidence: f32,
    /// Pupil diameter (mm)
    pub pupil_diameter: Option<f32>,
}

impl GazePoint {
    /// Create a new gaze point
    pub fn new(x: f32, y: f32, timestamp: u64) -> Self {
        Self {
            x,
            y,
            timestamp,
            confidence: 1.0,
            pupil_diameter: None,
        }
    }

    /// Calculate distance to another point
    pub fn distance_to(&self, other: &GazePoint) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Check if point is within a region
    pub fn is_in_region(&self, region: &ContentRegion) -> bool {
        self.x >= region.x
            && self.x <= region.x + region.width
            && self.y >= region.y
            && self.y <= region.y + region.height
    }
}

/// Content region on screen
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentRegion {
    /// Region ID/content ID
    pub id: String,
    /// X position (0.0-1.0)
    pub x: f32,
    /// Y position (0.0-1.0)
    pub y: f32,
    /// Width (0.0-1.0)
    pub width: f32,
    /// Height (0.0-1.0)
    pub height: f32,
    /// Content type
    pub content_type: ContentRegionType,
    /// Selectable flag
    pub selectable: bool,
}

/// Content region type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContentRegionType {
    /// Text content
    Text,
    /// Image content
    Image,
    /// Video content
    Video,
    /// Button/interactive element
    Button,
    /// Answer choice
    AnswerChoice,
    /// Navigation element
    Navigation,
    /// Menu item
    MenuItem,
}

/// Content selection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentSelection {
    /// Selected content ID
    pub content_id: String,
    /// Selection type
    pub selection_type: SelectionType,
    /// Gaze point at selection
    pub gaze_point: GazePoint,
    /// Dwell time (ms) if dwell-based
    pub dwell_time_ms: Option<u32>,
}

/// Selection type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelectionType {
    /// Fixation-based (looking at)
    Fixation,
    /// Dwell-based (looking for threshold time)
    Dwell,
    /// Blink-based (blink to select)
    Blink,
    /// Gaze gesture (pattern recognition)
    Gesture,
}

/// Activation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivationResult {
    /// Whether activation was successful
    pub success: bool,
    /// Content ID activated
    pub content_id: String,
    /// Activation type
    pub activation_type: SelectionType,
    /// Feedback message
    pub feedback: String,
}

/// Reading analysis from gaze tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReadingAnalysis {
    /// Total reading time (ms)
    pub total_time_ms: u64,
    /// Number of fixations
    pub fixation_count: usize,
    /// Average fixation duration (ms)
    pub avg_fixation_duration_ms: f32,
    /// Number of regressions (backward movements)
    pub regression_count: usize,
    /// Reading speed estimate (words per minute)
    pub estimated_wpm: Option<f32>,
    /// Areas of difficulty (high fixation time)
    pub difficulty_areas: Vec<DifficultyArea>,
    /// Overall reading pattern
    pub pattern: ReadingPattern,
    /// Attention metrics
    pub attention: AttentionMetrics,
}

/// Area where user had difficulty
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DifficultyArea {
    /// Region of difficulty
    pub region: ContentRegion,
    /// Time spent (ms)
    pub time_spent_ms: u64,
    /// Number of revisits
    pub revisit_count: usize,
}

/// Reading pattern classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReadingPattern {
    /// Normal left-to-right reading
    Normal,
    /// Scanning/skimming
    Scanning,
    /// Careful/detailed reading
    Careful,
    /// Struggling/difficulty
    Struggling,
    /// Disengaged/distracted
    Disengaged,
}

/// Attention metrics from gaze
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AttentionMetrics {
    /// Percentage of time on task
    pub on_task_percentage: f32,
    /// Longest continuous focus (ms)
    pub longest_focus_ms: u64,
    /// Number of off-screen looks
    pub off_screen_count: usize,
    /// Attention score (0.0-1.0)
    pub attention_score: f32,
}

/// Eye Gaze Adapter implementation
pub struct EyeGazeAdapter {
    /// Dwell time threshold (ms)
    dwell_threshold_ms: u32,
    /// Current dwell timers
    dwell_timers: HashMap<String, u32>,
    /// Current gaze position
    current_gaze: Option<GazePoint>,
    /// Gaze history for analysis
    gaze_history: Vec<GazePoint>,
    /// Connection status
    connected: bool,
    /// Fixation detection threshold (pixels/distance)
    fixation_threshold: f32,
    /// Minimum fixation duration (ms)
    min_fixation_ms: u32,
}

impl EyeGazeAdapter {
    /// Create a new Eye Gaze adapter
    pub fn new() -> Self {
        Self {
            dwell_threshold_ms: 800, // 800ms default dwell time
            dwell_timers: HashMap::new(),
            current_gaze: None,
            gaze_history: Vec::new(),
            connected: true,
            fixation_threshold: 0.02, // 2% of screen
            min_fixation_ms: 100,
        }
    }

    /// Set connection status
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }

    /// Update current gaze position
    pub fn update_gaze(&mut self, gaze: GazePoint) {
        self.current_gaze = Some(gaze);
        self.gaze_history.push(gaze);

        // Keep only last 1000 samples for memory efficiency
        if self.gaze_history.len() > 1000 {
            self.gaze_history.remove(0);
        }
    }

    /// Clear gaze history
    pub fn clear_history(&mut self) {
        self.gaze_history.clear();
    }

    /// Detect fixations from gaze data
    fn detect_fixations(&self, gaze_path: &[GazePoint]) -> Vec<Fixation> {
        let mut fixations = Vec::new();
        if gaze_path.len() < 2 {
            return fixations;
        }

        let mut fixation_start = 0;
        let mut centroid = gaze_path[0];

        for i in 1..gaze_path.len() {
            let point = &gaze_path[i];
            if point.distance_to(&centroid) < self.fixation_threshold {
                // Still in fixation, update centroid
                let n = (i - fixation_start + 1) as f32;
                centroid.x = (centroid.x * (n - 1.0) + point.x) / n;
                centroid.y = (centroid.y * (n - 1.0) + point.y) / n;
            } else {
                // Fixation ended
                let duration = gaze_path[i - 1].timestamp - gaze_path[fixation_start].timestamp;
                if duration >= self.min_fixation_ms as u64 {
                    fixations.push(Fixation {
                        x: centroid.x,
                        y: centroid.y,
                        start_time: gaze_path[fixation_start].timestamp,
                        duration_ms: duration,
                    });
                }
                fixation_start = i;
                centroid = *point;
            }
        }

        // Check last fixation
        let last = gaze_path.last().unwrap();
        let duration = last.timestamp - gaze_path[fixation_start].timestamp;
        if duration >= self.min_fixation_ms as u64 {
            fixations.push(Fixation {
                x: centroid.x,
                y: centroid.y,
                start_time: gaze_path[fixation_start].timestamp,
                duration_ms: duration,
            });
        }

        fixations
    }

    /// Count regressions (backward eye movements in reading)
    fn count_regressions(&self, fixations: &[Fixation]) -> usize {
        let mut count = 0;
        for i in 1..fixations.len() {
            // Regression is when x moves leftward significantly
            if fixations[i].x < fixations[i - 1].x - 0.05 {
                count += 1;
            }
        }
        count
    }

    /// Classify reading pattern
    fn classify_pattern(&self, analysis: &ReadingAnalysis) -> ReadingPattern {
        if analysis.attention.on_task_percentage < 0.5 {
            return ReadingPattern::Disengaged;
        }

        let regression_ratio = analysis.regression_count as f32 / analysis.fixation_count.max(1) as f32;

        if regression_ratio > 0.4 {
            ReadingPattern::Struggling
        } else if analysis.avg_fixation_duration_ms > 400.0 {
            ReadingPattern::Careful
        } else if analysis.avg_fixation_duration_ms < 150.0 {
            ReadingPattern::Scanning
        } else {
            ReadingPattern::Normal
        }
    }
}

/// Fixation data point
#[derive(Debug, Clone)]
struct Fixation {
    x: f32,
    y: f32,
    start_time: u64,
    duration_ms: u64,
}

impl Default for EyeGazeAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl EyeGazeEducation for EyeGazeAdapter {
    fn gaze_select_content(&self, gaze: &GazePoint, content_regions: &[ContentRegion]) -> Option<ContentSelection> {
        for region in content_regions {
            if region.selectable && gaze.is_in_region(region) {
                return Some(ContentSelection {
                    content_id: region.id.clone(),
                    selection_type: SelectionType::Fixation,
                    gaze_point: *gaze,
                    dwell_time_ms: None,
                });
            }
        }
        None
    }

    fn dwell_activate(&self, content_id: &str, dwell_ms: u32) -> Result<ActivationResult> {
        if dwell_ms >= self.dwell_threshold_ms {
            Ok(ActivationResult {
                success: true,
                content_id: content_id.to_string(),
                activation_type: SelectionType::Dwell,
                feedback: format!("Activated after {}ms dwell", dwell_ms),
            })
        } else {
            Ok(ActivationResult {
                success: false,
                content_id: content_id.to_string(),
                activation_type: SelectionType::Dwell,
                feedback: format!("Keep looking ({}/{}ms)", dwell_ms, self.dwell_threshold_ms),
            })
        }
    }

    fn track_reading(&self, gaze_path: &[GazePoint]) -> ReadingAnalysis {
        if gaze_path.is_empty() {
            return ReadingAnalysis {
                total_time_ms: 0,
                fixation_count: 0,
                avg_fixation_duration_ms: 0.0,
                regression_count: 0,
                estimated_wpm: None,
                difficulty_areas: vec![],
                pattern: ReadingPattern::Disengaged,
                attention: AttentionMetrics {
                    on_task_percentage: 0.0,
                    longest_focus_ms: 0,
                    off_screen_count: 0,
                    attention_score: 0.0,
                },
            };
        }

        let fixations = self.detect_fixations(gaze_path);
        let total_time = gaze_path.last().unwrap().timestamp - gaze_path.first().unwrap().timestamp;
        let avg_duration = if !fixations.is_empty() {
            fixations.iter().map(|f| f.duration_ms as f32).sum::<f32>() / fixations.len() as f32
        } else {
            0.0
        };

        let regression_count = self.count_regressions(&fixations);

        // Calculate attention metrics
        let on_task = gaze_path.iter().filter(|p| p.confidence > 0.5).count() as f32
            / gaze_path.len() as f32;
        let off_screen = gaze_path.iter().filter(|p| p.confidence < 0.3).count();

        let attention = AttentionMetrics {
            on_task_percentage: on_task,
            longest_focus_ms: fixations.iter().map(|f| f.duration_ms).max().unwrap_or(0),
            off_screen_count: off_screen,
            attention_score: on_task * 0.8 + (1.0 - (off_screen as f32 / gaze_path.len() as f32).min(1.0)) * 0.2,
        };

        let mut analysis = ReadingAnalysis {
            total_time_ms: total_time,
            fixation_count: fixations.len(),
            avg_fixation_duration_ms: avg_duration,
            regression_count,
            estimated_wpm: None, // Would need word count to estimate
            difficulty_areas: vec![],
            pattern: ReadingPattern::Normal,
            attention,
        };

        analysis.pattern = self.classify_pattern(&analysis);
        analysis
    }

    fn current_gaze(&self) -> Option<GazePoint> {
        self.current_gaze
    }

    fn set_dwell_threshold(&mut self, threshold_ms: u32) {
        self.dwell_threshold_ms = threshold_ms;
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_eye_gaze_adapter() {
        let adapter = EyeGazeAdapter::new();
        assert!(adapter.is_connected());
        assert!(adapter.current_gaze().is_none());
    }

    #[test]
    fn test_gaze_point_distance() {
        let p1 = GazePoint::new(0.0, 0.0, 0);
        let p2 = GazePoint::new(0.3, 0.4, 100);

        let distance = p1.distance_to(&p2);
        assert!((distance - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_gaze_in_region() {
        let gaze = GazePoint::new(0.5, 0.5, 0);
        let region = ContentRegion {
            id: "test".to_string(),
            x: 0.4,
            y: 0.4,
            width: 0.2,
            height: 0.2,
            content_type: ContentRegionType::Button,
            selectable: true,
        };

        assert!(gaze.is_in_region(&region));
    }

    #[test]
    fn test_gaze_select_content() {
        let adapter = EyeGazeAdapter::new();
        let gaze = GazePoint::new(0.5, 0.5, 0);

        let regions = vec![
            ContentRegion {
                id: "button1".to_string(),
                x: 0.4,
                y: 0.4,
                width: 0.2,
                height: 0.2,
                content_type: ContentRegionType::Button,
                selectable: true,
            },
        ];

        let selection = adapter.gaze_select_content(&gaze, &regions);
        assert!(selection.is_some());
        assert_eq!(selection.unwrap().content_id, "button1");
    }

    #[test]
    fn test_dwell_activate_success() {
        let adapter = EyeGazeAdapter::new();

        let result = adapter.dwell_activate("content1", 1000).unwrap();
        assert!(result.success);
    }

    #[test]
    fn test_dwell_activate_not_enough() {
        let adapter = EyeGazeAdapter::new();

        let result = adapter.dwell_activate("content1", 500).unwrap();
        assert!(!result.success);
    }

    #[test]
    fn test_track_reading() {
        let adapter = EyeGazeAdapter::new();

        // Simulate reading with fixations (points close together for each fixation)
        // Create 3 fixation clusters
        let mut gaze_path: Vec<GazePoint> = Vec::new();
        let mut timestamp = 0u64;

        // First fixation at (0.2, 0.5)
        for i in 0..5 {
            gaze_path.push(GazePoint::new(0.2 + i as f32 * 0.005, 0.5, timestamp));
            timestamp += 50;
        }
        // Jump to next word
        // Second fixation at (0.4, 0.5)
        for i in 0..5 {
            gaze_path.push(GazePoint::new(0.4 + i as f32 * 0.005, 0.5, timestamp));
            timestamp += 50;
        }

        let analysis = adapter.track_reading(&gaze_path);
        assert!(analysis.total_time_ms > 0);
        // At least one fixation should be detected
        assert!(analysis.fixation_count >= 1);
    }

    #[test]
    fn test_update_gaze() {
        let mut adapter = EyeGazeAdapter::new();

        adapter.update_gaze(GazePoint::new(0.5, 0.5, 100));
        assert!(adapter.current_gaze().is_some());
        assert_eq!(adapter.current_gaze().unwrap().x, 0.5);
    }

    #[test]
    fn test_empty_gaze_path() {
        let adapter = EyeGazeAdapter::new();
        let analysis = adapter.track_reading(&[]);

        assert_eq!(analysis.total_time_ms, 0);
        assert_eq!(analysis.pattern, ReadingPattern::Disengaged);
    }
}
