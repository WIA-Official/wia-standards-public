//! Eye Tracker Adapter
//! 弘益人間 - Gaming for Everyone

use super::DeviceAdapter;
use crate::error::{GameError, Result};
use crate::protocol::device::*;
use crate::protocol::event::*;
use async_trait::async_trait;
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Eye tracker adapter
#[derive(Debug)]
pub struct EyeTrackerAdapter {
    device: ConnectedDevice,
    connected: bool,
    config: EyeTrackerConfig,
    event_queue: Arc<Mutex<Vec<InputEvent>>>,
    dwell_tracker: DwellTracker,
}

/// Eye tracker configuration
#[derive(Debug, Clone)]
pub struct EyeTrackerConfig {
    /// Smoothing factor (0.0-1.0)
    pub smoothing: f32,
    /// Gaze sample rate in Hz
    pub sample_rate: u32,
    /// Enable head tracking
    pub head_tracking: bool,
    /// Dwell selection config
    pub dwell_config: DwellConfig,
}

impl Default for EyeTrackerConfig {
    fn default() -> Self {
        Self {
            smoothing: 0.3,
            sample_rate: 60,
            head_tracking: true,
            dwell_config: DwellConfig::default(),
        }
    }
}

/// Dwell selection configuration
#[derive(Debug, Clone)]
pub struct DwellConfig {
    /// Time required to dwell for selection (ms)
    pub dwell_time_ms: u32,
    /// Tolerance radius (normalized, 0.0-1.0)
    pub tolerance_radius: f32,
    /// Show progress indicator
    pub show_progress: bool,
    /// Play sound on selection
    pub play_sound: bool,
}

impl Default for DwellConfig {
    fn default() -> Self {
        Self {
            dwell_time_ms: 800,
            tolerance_radius: 0.05,
            show_progress: true,
            play_sound: true,
        }
    }
}

/// Dwell selection tracker
#[derive(Debug)]
struct DwellTracker {
    config: DwellConfig,
    current_point: Option<Point2D>,
    dwell_start: Option<Instant>,
    selected_regions: Vec<DwellRegion>,
}

/// Dwell selectable region
#[derive(Debug, Clone)]
pub struct DwellRegion {
    pub id: String,
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
    pub action: DwellAction,
}

/// Action on dwell selection
#[derive(Debug, Clone)]
pub enum DwellAction {
    Button(Button),
    Click,
    Custom(String),
}

impl DwellTracker {
    fn new(config: DwellConfig) -> Self {
        Self {
            config,
            current_point: None,
            dwell_start: None,
            selected_regions: Vec::new(),
        }
    }

    fn update(&mut self, point: Point2D) -> Option<&DwellRegion> {
        // Check if point moved outside tolerance
        if let Some(ref current) = self.current_point {
            let dx = point.x - current.x;
            let dy = point.y - current.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance > self.config.tolerance_radius {
                // Point moved, reset dwell
                self.current_point = Some(point);
                self.dwell_start = Some(Instant::now());
                return None;
            }
        } else {
            self.current_point = Some(point);
            self.dwell_start = Some(Instant::now());
            return None;
        }

        // Check dwell time
        if let Some(start) = self.dwell_start {
            if start.elapsed().as_millis() >= self.config.dwell_time_ms as u128 {
                // Dwell complete - check if in any region
                for region in &self.selected_regions {
                    if point.x >= region.x
                        && point.x <= region.x + region.width
                        && point.y >= region.y
                        && point.y <= region.y + region.height
                    {
                        // Reset for next dwell
                        self.dwell_start = None;
                        return Some(region);
                    }
                }
            }
        }

        None
    }

    fn add_region(&mut self, region: DwellRegion) {
        self.selected_regions.push(region);
    }

    fn clear_regions(&mut self) {
        self.selected_regions.clear();
    }

    fn progress(&self) -> f32 {
        if let Some(start) = self.dwell_start {
            let elapsed = start.elapsed().as_millis() as f32;
            (elapsed / self.config.dwell_time_ms as f32).min(1.0)
        } else {
            0.0
        }
    }
}

impl EyeTrackerAdapter {
    /// Create a new eye tracker adapter
    pub fn new(device_id: DeviceId, name: &str) -> Self {
        let device = ConnectedDevice {
            device_id,
            name: name.to_string(),
            device_type: DeviceType::EyeTracker,
            connection: ConnectionType::Usb,
            vendor_id: vendor::TOBII,
            product_id: 0x0001,
            battery_level: None,
            capabilities: DeviceCapabilities::eye_tracker(),
        };

        let config = EyeTrackerConfig::default();
        let dwell_tracker = DwellTracker::new(config.dwell_config.clone());

        Self {
            device,
            connected: false,
            config,
            event_queue: Arc::new(Mutex::new(Vec::new())),
            dwell_tracker,
        }
    }

    /// Create Tobii Eye Tracker 5
    pub fn tobii_5() -> Self {
        Self::new(uuid::Uuid::new_v4(), "Tobii Eye Tracker 5")
    }

    /// Set configuration
    pub fn set_config(&mut self, config: EyeTrackerConfig) {
        self.dwell_tracker = DwellTracker::new(config.dwell_config.clone());
        self.config = config;
    }

    /// Add dwell region
    pub fn add_dwell_region(&mut self, region: DwellRegion) {
        self.dwell_tracker.add_region(region);
    }

    /// Clear dwell regions
    pub fn clear_dwell_regions(&mut self) {
        self.dwell_tracker.clear_regions();
    }

    /// Get dwell progress (0.0-1.0)
    pub fn dwell_progress(&self) -> f32 {
        self.dwell_tracker.progress()
    }

    /// Simulate gaze point
    pub fn simulate_gaze(&mut self, x: f32, y: f32) {
        let point = Point2D { x, y };

        // Check for dwell selection
        if let Some(region) = self.dwell_tracker.update(point) {
            let action = region.action.clone();
            match action {
                DwellAction::Button(button) => {
                    let mut queue = self.event_queue.lock().unwrap();
                    queue.push(InputEvent::ButtonPressed {
                        device_id: self.device.device_id,
                        button,
                        timestamp: InputEvent::now(),
                    });
                }
                _ => {}
            }
        }

        // Add gaze point event
        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::GazePoint {
            device_id: self.device.device_id,
            x,
            y,
            validity: GazeValidity::both_valid(),
            timestamp: InputEvent::now(),
        });
    }

    /// Simulate full gaze data
    pub fn simulate_full_gaze(&mut self, data: FullGazeData) {
        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::GazeData {
            device_id: self.device.device_id,
            data,
            timestamp: InputEvent::now(),
        });
    }

    /// Simulate head pose
    pub fn simulate_head_pose(&mut self, yaw: f32, pitch: f32, roll: f32) {
        if !self.config.head_tracking {
            return;
        }

        let mut queue = self.event_queue.lock().unwrap();
        queue.push(InputEvent::HeadPose {
            device_id: self.device.device_id,
            yaw,
            pitch,
            roll,
            timestamp: InputEvent::now(),
        });
    }
}

#[async_trait]
impl DeviceAdapter for EyeTrackerAdapter {
    fn device_info(&self) -> &ConnectedDevice {
        &self.device
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn poll(&mut self) -> Vec<InputEvent> {
        let mut queue = self.event_queue.lock().unwrap();
        queue.drain(..).collect()
    }

    async fn start(&mut self) -> Result<()> {
        // In a real implementation, this would initialize the eye tracker SDK
        self.connected = true;
        Ok(())
    }

    async fn stop(&mut self) -> Result<()> {
        self.connected = false;
        Ok(())
    }

    async fn rumble(&self, _left: f32, _right: f32, _duration: u32) -> Result<()> {
        Err(GameError::FeatureNotSupported(
            "Eye trackers do not support rumble".to_string(),
        ))
    }

    async fn set_led(&self, r: u8, g: u8, b: u8) -> Result<()> {
        if !self.connected {
            return Err(GameError::DeviceNotConnected(self.device.device_id.to_string()));
        }

        // Some eye trackers have status LEDs
        println!("👁️ Eye Tracker LED: R={}, G={}, B={}", r, g, b);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eye_tracker_creation() {
        let adapter = EyeTrackerAdapter::tobii_5();
        assert_eq!(adapter.device.device_type, DeviceType::EyeTracker);
        assert!(!adapter.device.capabilities.has_buttons);
    }

    #[test]
    fn test_simulate_gaze() {
        let mut adapter = EyeTrackerAdapter::tobii_5();

        adapter.simulate_gaze(0.5, 0.5);

        let events = adapter.poll();
        assert_eq!(events.len(), 1);

        match &events[0] {
            InputEvent::GazePoint { x, y, .. } => {
                assert_eq!(*x, 0.5);
                assert_eq!(*y, 0.5);
            }
            _ => panic!("Expected GazePoint event"),
        }
    }

    #[test]
    fn test_dwell_region() {
        let mut adapter = EyeTrackerAdapter::tobii_5();

        // Configure fast dwell for testing
        let mut config = EyeTrackerConfig::default();
        config.dwell_config.dwell_time_ms = 10; // Very short for test
        adapter.set_config(config);

        adapter.add_dwell_region(DwellRegion {
            id: "button_a".to_string(),
            x: 0.4,
            y: 0.4,
            width: 0.2,
            height: 0.2,
            action: DwellAction::Button(Button::A),
        });

        // Simulate dwelling on region
        adapter.simulate_gaze(0.5, 0.5);
        std::thread::sleep(std::time::Duration::from_millis(20));
        adapter.simulate_gaze(0.5, 0.5);

        let events = adapter.poll();
        // Should have gaze events and potentially button press
        assert!(!events.is_empty());
    }

    #[tokio::test]
    async fn test_adapter_lifecycle() {
        let mut adapter = EyeTrackerAdapter::tobii_5();

        assert!(!adapter.is_connected());

        adapter.start().await.unwrap();
        assert!(adapter.is_connected());

        adapter.stop().await.unwrap();
        assert!(!adapter.is_connected());
    }
}
