//! Smart Wheelchair Integration
//!
//! Provides integration with WIA Smart Wheelchair for docking,
//! securement, and coordinated mobility.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::error::{Error, Result};
use crate::types::{EntryType, SecurementPosition, SecurementStatus, SecurementType};

/// Wheelchair category for integration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WheelchairCategory {
    /// Manual wheelchair
    Manual,
    /// Power wheelchair
    Power,
    /// Mobility scooter
    Scooter,
}

/// Wheelchair dimensions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairDimensions {
    /// Width in cm
    pub width_cm: f32,
    /// Length in cm
    pub length_cm: f32,
    /// Height in cm
    pub height_cm: f32,
    /// Weight in kg
    pub weight_kg: f32,
    /// Turning radius in cm
    pub turning_radius_cm: f32,
}

impl Default for WheelchairDimensions {
    fn default() -> Self {
        Self {
            width_cm: 65.0,
            length_cm: 105.0,
            height_cm: 95.0,
            weight_kg: 25.0,
            turning_radius_cm: 80.0,
        }
    }
}

/// Wheelchair information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairInfo {
    /// Wheelchair ID
    pub id: Uuid,
    /// Category of wheelchair
    pub wheelchair_category: WheelchairCategory,
    /// Dimensions
    pub dimensions: WheelchairDimensions,
    /// Entry preference
    pub entry_preference: EntryType,
    /// Securement preference
    pub securement_preference: SecurementType,
    /// Current battery level (for power wheelchairs)
    pub battery_level: Option<u8>,
}

impl WheelchairInfo {
    /// Create new wheelchair info
    pub fn new(wheelchair_category: WheelchairCategory) -> Self {
        Self {
            id: Uuid::new_v4(),
            wheelchair_category,
            dimensions: WheelchairDimensions::default(),
            entry_preference: EntryType::Ramp,
            securement_preference: SecurementType::FullAuto,
            battery_level: None,
        }
    }

    /// Set dimensions
    pub fn with_dimensions(mut self, dimensions: WheelchairDimensions) -> Self {
        self.dimensions = dimensions;
        self
    }

    /// Set entry preference
    pub fn with_entry_preference(mut self, entry: EntryType) -> Self {
        self.entry_preference = entry;
        self
    }
}

/// Docking phase
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DockingPhase {
    /// Not docking
    None,
    /// Approaching vehicle
    Approach,
    /// Entering vehicle
    Entry,
    /// Positioning in space
    Positioning,
    /// Being secured
    Securement,
    /// Verified and complete
    Verified,
    /// Exiting vehicle
    Exiting,
}

/// Docking guidance direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GuidanceDirection {
    Forward,
    Back,
    Left,
    Right,
    Stop,
}

/// Docking guidance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockingGuidance {
    /// Direction to move
    pub direction: GuidanceDirection,
    /// Distance in cm
    pub distance_cm: f32,
    /// Audio cue
    pub audio_cue: String,
    /// Visual indicator type
    pub visual_indicator: Option<String>,
}

/// Docking securement point detail
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockingSecurementPoint {
    /// Point ID
    pub point_id: String,
    /// Position
    pub position: SecurementPosition,
    /// Is engaged
    pub engaged: bool,
    /// Tension in lbs
    pub tension_lbs: Option<f32>,
}

/// Docking status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockingStatus {
    /// Current phase
    pub phase: DockingPhase,
    /// Progress percentage
    pub progress_percent: u8,
    /// Estimated time remaining in seconds
    pub eta_seconds: u32,
    /// Current guidance
    pub guidance: Option<DockingGuidance>,
    /// Securement points status
    pub securement_points: Vec<DockingSecurementPoint>,
}

impl Default for DockingStatus {
    fn default() -> Self {
        Self {
            phase: DockingPhase::None,
            progress_percent: 0,
            eta_seconds: 0,
            guidance: None,
            securement_points: Vec::new(),
        }
    }
}

/// Docking result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockingResult {
    /// Success status
    pub success: bool,
    /// Wheelchair ID
    pub wheelchair_id: Uuid,
    /// Final securement status
    pub securement: SecurementStatus,
    /// Duration in seconds
    pub duration_seconds: u32,
    /// Any issues encountered
    pub issues: Vec<String>,
}

/// Wheelchair detection methods
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DetectionMethod {
    /// Bluetooth beacon
    BluetoothBeacon,
    /// Ultra-wideband positioning
    UwbPositioning,
    /// Visual marker
    VisualMarker,
    /// NFC tag
    NfcTag,
}

/// Wheelchair detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairDetection {
    /// Detected wheelchair
    pub wheelchair: WheelchairInfo,
    /// Detection method used
    pub method: DetectionMethod,
    /// Distance in meters
    pub distance_m: f32,
    /// Signal strength (0-100)
    pub signal_strength: u8,
    /// Timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// Wheelchair integration handler
#[async_trait]
pub trait WheelchairIntegration: Send + Sync {
    /// Scan for nearby wheelchairs
    async fn scan(&self) -> Result<Vec<WheelchairDetection>>;

    /// Start docking sequence
    async fn start_docking(&self, wheelchair: &WheelchairInfo) -> Result<()>;

    /// Get docking status
    async fn docking_status(&self) -> Result<DockingStatus>;

    /// Cancel docking
    async fn cancel_docking(&self) -> Result<()>;

    /// Start exit sequence
    async fn start_exit(&self) -> Result<()>;

    /// Send guidance to wheelchair
    async fn send_guidance(&self, guidance: &DockingGuidance) -> Result<()>;
}

/// Mock wheelchair integration for testing
pub struct MockWheelchairIntegration {
    status: std::sync::Arc<tokio::sync::RwLock<DockingStatus>>,
    wheelchair: std::sync::Arc<tokio::sync::RwLock<Option<WheelchairInfo>>>,
}

impl MockWheelchairIntegration {
    /// Create new mock integration
    pub fn new() -> Self {
        Self {
            status: std::sync::Arc::new(tokio::sync::RwLock::new(DockingStatus::default())),
            wheelchair: std::sync::Arc::new(tokio::sync::RwLock::new(None)),
        }
    }

    /// Simulate docking progress
    pub async fn simulate_progress(&self, phase: DockingPhase, progress: u8) {
        let mut status = self.status.write().await;
        status.phase = phase;
        status.progress_percent = progress;
    }
}

impl Default for MockWheelchairIntegration {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl WheelchairIntegration for MockWheelchairIntegration {
    async fn scan(&self) -> Result<Vec<WheelchairDetection>> {
        // Return a mock wheelchair detection
        Ok(vec![WheelchairDetection {
            wheelchair: WheelchairInfo::new(WheelchairCategory::Power),
            method: DetectionMethod::BluetoothBeacon,
            distance_m: 5.0,
            signal_strength: 85,
            timestamp: chrono::Utc::now(),
        }])
    }

    async fn start_docking(&self, wheelchair: &WheelchairInfo) -> Result<()> {
        *self.wheelchair.write().await = Some(wheelchair.clone());
        let mut status = self.status.write().await;
        status.phase = DockingPhase::Approach;
        status.progress_percent = 0;
        status.eta_seconds = 120;
        Ok(())
    }

    async fn docking_status(&self) -> Result<DockingStatus> {
        Ok(self.status.read().await.clone())
    }

    async fn cancel_docking(&self) -> Result<()> {
        *self.status.write().await = DockingStatus::default();
        *self.wheelchair.write().await = None;
        Ok(())
    }

    async fn start_exit(&self) -> Result<()> {
        let mut status = self.status.write().await;
        if status.phase == DockingPhase::Verified {
            status.phase = DockingPhase::Exiting;
            Ok(())
        } else {
            Err(Error::validation("Not in docked state"))
        }
    }

    async fn send_guidance(&self, guidance: &DockingGuidance) -> Result<()> {
        let mut status = self.status.write().await;
        status.guidance = Some(guidance.clone());
        Ok(())
    }
}

/// Last mile handoff information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LastMileHandoff {
    /// Building name
    pub building_name: String,
    /// Entrance type
    pub entrance_type: EntranceType,
    /// Indoor navigation available
    pub indoor_navigation: bool,
    /// Contact information
    pub contact_info: Option<String>,
    /// Environment information
    pub environment: EnvironmentInfo,
}

/// Entrance type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EntranceType {
    Accessible,
    Main,
    Side,
    Rear,
}

/// Environment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentInfo {
    /// Surface type
    pub surface_type: SurfaceType,
    /// Slope in degrees
    pub slope_degrees: f32,
    /// Weather advisory
    pub weather_advisory: Option<String>,
}

/// Surface type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfaceType {
    Paved,
    Gravel,
    Grass,
    Tile,
    Carpet,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wheelchair_info() {
        let wheelchair = WheelchairInfo::new(WheelchairCategory::Power)
            .with_entry_preference(EntryType::Lift);

        assert_eq!(wheelchair.wheelchair_category, WheelchairCategory::Power);
        assert_eq!(wheelchair.entry_preference, EntryType::Lift);
    }

    #[tokio::test]
    async fn test_mock_integration() {
        let integration = MockWheelchairIntegration::new();

        let detections = integration.scan().await.unwrap();
        assert!(!detections.is_empty());

        let wheelchair = &detections[0].wheelchair;
        integration.start_docking(wheelchair).await.unwrap();

        let status = integration.docking_status().await.unwrap();
        assert_eq!(status.phase, DockingPhase::Approach);
    }
}
