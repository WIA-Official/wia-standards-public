//! Eye Gaze Integration
//!
//! Provides integration with eye gaze tracking systems for
//! in-vehicle control and passenger monitoring.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::error::Result;

/// Gaze point in normalized coordinates (0-1)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GazePoint {
    /// X coordinate (0=left, 1=right)
    pub x: f32,
    /// Y coordinate (0=top, 1=bottom)
    pub y: f32,
    /// Confidence score (0-1)
    pub confidence: f32,
    /// Timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

impl GazePoint {
    /// Create a new gaze point
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            x: x.clamp(0.0, 1.0),
            y: y.clamp(0.0, 1.0),
            confidence: 1.0,
            timestamp: chrono::Utc::now(),
        }
    }

    /// Check if point is in deadzone (center of screen)
    pub fn is_in_deadzone(&self, radius: f32) -> bool {
        let dx = self.x - 0.5;
        let dy = self.y - 0.5;
        (dx * dx + dy * dy).sqrt() < radius
    }
}

/// Display zone for gaze interactions
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisplayZone {
    /// Emergency actions (stop, help)
    Emergency,
    /// Main content (map, destination)
    MainContent,
    /// Quick actions (temperature, window)
    QuickActions,
    /// Status bar
    StatusBar,
}

/// Zone bounds
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZoneBounds {
    /// Zone identifier
    pub zone: DisplayZone,
    /// Left bound (0-1)
    pub left: f32,
    /// Right bound (0-1)
    pub right: f32,
    /// Top bound (0-1)
    pub top: f32,
    /// Bottom bound (0-1)
    pub bottom: f32,
}

impl ZoneBounds {
    /// Check if point is within zone
    pub fn contains(&self, point: &GazePoint) -> bool {
        point.x >= self.left && point.x <= self.right &&
        point.y >= self.top && point.y <= self.bottom
    }
}

/// Default zone layout
pub fn default_zones() -> Vec<ZoneBounds> {
    vec![
        ZoneBounds {
            zone: DisplayZone::Emergency,
            left: 0.0,
            right: 0.2,
            top: 0.0,
            bottom: 0.8,
        },
        ZoneBounds {
            zone: DisplayZone::MainContent,
            left: 0.2,
            right: 0.8,
            top: 0.0,
            bottom: 0.8,
        },
        ZoneBounds {
            zone: DisplayZone::QuickActions,
            left: 0.8,
            right: 1.0,
            top: 0.0,
            bottom: 0.8,
        },
        ZoneBounds {
            zone: DisplayZone::StatusBar,
            left: 0.0,
            right: 1.0,
            top: 0.8,
            bottom: 1.0,
        },
    ]
}

/// Dwell selection configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellConfig {
    /// Dwell time for emergency buttons (seconds)
    pub emergency_dwell: f32,
    /// Dwell time for standard buttons (seconds)
    pub standard_dwell: f32,
    /// Dwell time for critical decisions (seconds)
    pub critical_dwell: f32,
    /// Dwell radius tolerance
    pub dwell_radius: f32,
    /// Visual feedback enabled
    pub visual_feedback: bool,
    /// Audio feedback enabled
    pub audio_feedback: bool,
}

impl Default for DwellConfig {
    fn default() -> Self {
        Self {
            emergency_dwell: 0.5,
            standard_dwell: 1.0,
            critical_dwell: 1.5,
            dwell_radius: 0.1,
            visual_feedback: true,
            audio_feedback: true,
        }
    }
}

/// Dwell selection state
#[derive(Debug, Clone)]
pub struct DwellState {
    /// Current gaze target
    pub target: Option<String>,
    /// Dwell start time
    pub start_time: Option<chrono::DateTime<chrono::Utc>>,
    /// Required dwell time
    pub required_time: f32,
    /// Current progress (0-1)
    pub progress: f32,
}

impl DwellState {
    /// Create new dwell state
    pub fn new() -> Self {
        Self {
            target: None,
            start_time: None,
            required_time: 1.0,
            progress: 0.0,
        }
    }

    /// Update dwell state with new gaze point
    pub fn update(&mut self, target: Option<String>, required_time: f32) {
        if self.target != target {
            // Reset on target change
            self.target = target;
            self.start_time = Some(chrono::Utc::now());
            self.required_time = required_time;
            self.progress = 0.0;
        } else if let Some(start) = self.start_time {
            // Update progress
            let elapsed = (chrono::Utc::now() - start).num_milliseconds() as f32 / 1000.0;
            self.progress = (elapsed / self.required_time).min(1.0);
        }
    }

    /// Check if dwell is complete
    pub fn is_complete(&self) -> bool {
        self.progress >= 1.0
    }

    /// Reset dwell state
    pub fn reset(&mut self) {
        self.target = None;
        self.start_time = None;
        self.progress = 0.0;
    }
}

impl Default for DwellState {
    fn default() -> Self {
        Self::new()
    }
}

/// Passenger monitoring data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassengerMonitoring {
    /// Alertness score (0-100)
    pub alertness_score: u8,
    /// Distress score (0-100)
    pub distress_score: u8,
    /// Eyes open
    pub eyes_open: bool,
    /// Blink rate per minute
    pub blink_rate: f32,
    /// Gaze stability (0-1)
    pub gaze_stability: f32,
    /// Possible distress indicators
    pub distress_indicators: Vec<DistressIndicator>,
}

/// Distress indicator types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DistressIndicator {
    /// Rapid eye movement
    RapidEyeMovement,
    /// Prolonged eye closure
    ProlongedEyeClosure,
    /// Erratic gaze pattern
    ErraticGazePattern,
    /// Looking at emergency zone
    EmergencyZoneFocus,
    /// Looking at door
    DoorFocus,
}

/// Eye gaze integration handler
#[async_trait]
pub trait EyeGazeIntegration: Send + Sync {
    /// Initialize eye tracking
    async fn initialize(&mut self) -> Result<()>;

    /// Calibrate eye tracker
    async fn calibrate(&mut self) -> Result<()>;

    /// Get current gaze point
    async fn gaze_point(&self) -> Result<Option<GazePoint>>;

    /// Get current zone
    async fn current_zone(&self) -> Result<Option<DisplayZone>>;

    /// Get dwell state
    async fn dwell_state(&self) -> Result<DwellState>;

    /// Get passenger monitoring data
    async fn monitoring(&self) -> Result<PassengerMonitoring>;

    /// Check if tracking is active
    fn is_tracking(&self) -> bool;
}

/// Mock eye gaze integration for testing
pub struct MockEyeGazeIntegration {
    tracking: bool,
    gaze: std::sync::Arc<tokio::sync::RwLock<Option<GazePoint>>>,
    dwell: std::sync::Arc<tokio::sync::RwLock<DwellState>>,
    zones: Vec<ZoneBounds>,
}

impl MockEyeGazeIntegration {
    /// Create new mock integration
    pub fn new() -> Self {
        Self {
            tracking: false,
            gaze: std::sync::Arc::new(tokio::sync::RwLock::new(None)),
            dwell: std::sync::Arc::new(tokio::sync::RwLock::new(DwellState::new())),
            zones: default_zones(),
        }
    }

    /// Simulate gaze at point
    pub async fn simulate_gaze(&self, x: f32, y: f32) {
        *self.gaze.write().await = Some(GazePoint::new(x, y));
    }

    /// Simulate gaze lost
    pub async fn simulate_gaze_lost(&self) {
        *self.gaze.write().await = None;
    }
}

impl Default for MockEyeGazeIntegration {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl EyeGazeIntegration for MockEyeGazeIntegration {
    async fn initialize(&mut self) -> Result<()> {
        self.tracking = true;
        Ok(())
    }

    async fn calibrate(&mut self) -> Result<()> {
        Ok(())
    }

    async fn gaze_point(&self) -> Result<Option<GazePoint>> {
        Ok(self.gaze.read().await.clone())
    }

    async fn current_zone(&self) -> Result<Option<DisplayZone>> {
        if let Some(gaze) = self.gaze.read().await.as_ref() {
            for zone in &self.zones {
                if zone.contains(gaze) {
                    return Ok(Some(zone.zone));
                }
            }
        }
        Ok(None)
    }

    async fn dwell_state(&self) -> Result<DwellState> {
        Ok(self.dwell.read().await.clone())
    }

    async fn monitoring(&self) -> Result<PassengerMonitoring> {
        Ok(PassengerMonitoring {
            alertness_score: 85,
            distress_score: 5,
            eyes_open: true,
            blink_rate: 15.0,
            gaze_stability: 0.9,
            distress_indicators: vec![],
        })
    }

    fn is_tracking(&self) -> bool {
        self.tracking
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gaze_point_deadzone() {
        let center = GazePoint::new(0.5, 0.5);
        assert!(center.is_in_deadzone(0.1));

        let edge = GazePoint::new(0.8, 0.8);
        assert!(!edge.is_in_deadzone(0.1));
    }

    #[test]
    fn test_zone_detection() {
        let zones = default_zones();
        let emergency_zone = &zones[0];

        let in_zone = GazePoint::new(0.1, 0.5);
        assert!(emergency_zone.contains(&in_zone));

        let out_zone = GazePoint::new(0.5, 0.5);
        assert!(!emergency_zone.contains(&out_zone));
    }

    #[test]
    fn test_dwell_state() {
        let mut dwell = DwellState::new();
        dwell.update(Some("button1".to_string()), 1.0);

        assert_eq!(dwell.target, Some("button1".to_string()));
        assert!(!dwell.is_complete());

        // Simulate time passing
        dwell.progress = 1.0;
        assert!(dwell.is_complete());
    }

    #[tokio::test]
    async fn test_mock_integration() {
        let mut integration = MockEyeGazeIntegration::new();
        integration.initialize().await.unwrap();

        assert!(integration.is_tracking());

        integration.simulate_gaze(0.1, 0.5).await;
        let zone = integration.current_zone().await.unwrap();
        assert_eq!(zone, Some(DisplayZone::Emergency));
    }
}
