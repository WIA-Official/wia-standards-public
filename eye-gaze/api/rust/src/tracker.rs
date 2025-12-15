//! WIA Eye Gaze Standard - Eye Tracker Implementation
//!
//! 弘益人間 - 널리 인간을 이롭게

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use uuid::Uuid;

use crate::types::*;

/// Callback type for gaze data
pub type GazeCallback = Box<dyn Fn(&GazePoint) + Send + Sync>;

/// Callback type for events
pub type EventCallback = Box<dyn Fn(&GazeEvent) + Send + Sync>;

/// Subscription handle
pub struct Subscription {
    pub id: String,
}

/// Eye tracker adapter trait
#[async_trait]
pub trait EyeTrackerAdapter: Send + Sync {
    /// Adapter name
    fn name(&self) -> &str;

    /// Connect to native device
    async fn connect_native(&mut self) -> Result<()>;

    /// Disconnect from native device
    async fn disconnect_native(&mut self) -> Result<()>;

    /// Start native data streaming
    fn start_native_stream(&mut self, callback: GazeCallback);

    /// Stop native data streaming
    fn stop_native_stream(&mut self);

    /// Get device capabilities
    fn get_capabilities(&self) -> EyeTrackerCapabilities;

    /// Perform calibration
    async fn calibrate(&mut self, points: &[CalibrationPoint]) -> Result<CalibrationResult>;
}

/// WIA Eye Tracker
pub struct EyeTracker<A: EyeTrackerAdapter> {
    adapter: A,
    status: TrackerStatus,
    subscriptions: Arc<Mutex<HashMap<String, GazeCallback>>>,
    event_handlers: Arc<Mutex<HashMap<GazeEventType, Vec<EventCallback>>>>,
    latest_gaze: Arc<Mutex<Option<GazePoint>>>,
    calibration_result: Option<CalibrationResult>,
    smoothing: bool,
    smoothing_factor: f64,
}

impl<A: EyeTrackerAdapter> EyeTracker<A> {
    /// Create new eye tracker
    pub fn new(adapter: A) -> Self {
        Self {
            adapter,
            status: TrackerStatus::default(),
            subscriptions: Arc::new(Mutex::new(HashMap::new())),
            event_handlers: Arc::new(Mutex::new(HashMap::new())),
            latest_gaze: Arc::new(Mutex::new(None)),
            calibration_result: None,
            smoothing: false,
            smoothing_factor: 0.3,
        }
    }

    /// Create with smoothing enabled
    pub fn with_smoothing(mut self, factor: f64) -> Self {
        self.smoothing = true;
        self.smoothing_factor = factor;
        self
    }

    /// Connect to device
    pub async fn connect(&mut self) -> Result<()> {
        if self.status.connected {
            return Ok(());
        }

        self.status.state = TrackerState::Connecting;
        self.adapter.connect_native().await?;

        self.status.state = TrackerState::Connected;
        self.status.connected = true;

        self.emit_event(GazeEventType::DeviceConnected);
        Ok(())
    }

    /// Disconnect from device
    pub async fn disconnect(&mut self) -> Result<()> {
        if !self.status.connected {
            return Ok(());
        }

        self.stop_tracking();
        self.adapter.disconnect_native().await?;

        self.status.state = TrackerState::Disconnected;
        self.status.connected = false;
        self.status.tracking = false;

        self.emit_event(GazeEventType::DeviceDisconnected);
        Ok(())
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.status.connected
    }

    /// Start calibration
    pub async fn start_calibration(
        &mut self,
        points: Option<Vec<CalibrationPoint>>,
    ) -> Result<CalibrationResult> {
        if !self.status.connected {
            return Err(EyeTrackerError::NotConnected);
        }

        self.status.state = TrackerState::Calibrating;
        self.emit_event(GazeEventType::CalibrationStart);

        let calibration_points = points.unwrap_or_else(|| self.default_calibration_points());
        let result = self.adapter.calibrate(&calibration_points).await?;

        self.status.state = TrackerState::Connected;
        self.status.calibrated = result.success;
        self.calibration_result = Some(result.clone());

        self.emit_event(GazeEventType::CalibrationEnd);
        Ok(result)
    }

    /// Get calibration quality
    pub fn get_calibration_quality(&self) -> Option<CalibrationQuality> {
        self.calibration_result.as_ref().and_then(|result| {
            if !result.success {
                return None;
            }

            let accuracy = result.average_accuracy.unwrap_or(1.0);
            let overall = if accuracy < 0.5 {
                "excellent"
            } else if accuracy < 1.0 {
                "good"
            } else if accuracy < 2.0 {
                "fair"
            } else {
                "poor"
            };

            Some(CalibrationQuality {
                overall: overall.to_string(),
                accuracy,
                precision: result.average_precision.unwrap_or(0.1),
                coverage: 1.0,
            })
        })
    }

    /// Subscribe to gaze data
    pub fn subscribe<F>(&self, callback: F) -> Subscription
    where
        F: Fn(&GazePoint) + Send + Sync + 'static,
    {
        let id = Uuid::new_v4().to_string();
        let mut subs = self.subscriptions.lock().unwrap();
        subs.insert(id.clone(), Box::new(callback));
        Subscription { id }
    }

    /// Unsubscribe
    pub fn unsubscribe(&self, subscription: Subscription) {
        let mut subs = self.subscriptions.lock().unwrap();
        subs.remove(&subscription.id);
    }

    /// Start tracking
    pub fn start_tracking(&mut self) {
        if !self.status.connected || self.status.tracking {
            return;
        }

        let subscriptions = Arc::clone(&self.subscriptions);
        let latest_gaze = Arc::clone(&self.latest_gaze);
        let smoothing = self.smoothing;
        let smoothing_factor = self.smoothing_factor;

        self.adapter.start_native_stream(Box::new(move |gaze| {
            // Apply smoothing
            let processed = if smoothing {
                let mut latest = latest_gaze.lock().unwrap();
                let smoothed = if let Some(prev) = latest.as_ref() {
                    GazePoint {
                        x: prev.x * (1.0 - smoothing_factor) + gaze.x * smoothing_factor,
                        y: prev.y * (1.0 - smoothing_factor) + gaze.y * smoothing_factor,
                        ..gaze.clone()
                    }
                } else {
                    gaze.clone()
                };
                *latest = Some(smoothed.clone());
                smoothed
            } else {
                let mut latest = latest_gaze.lock().unwrap();
                *latest = Some(gaze.clone());
                gaze.clone()
            };

            // Notify subscribers
            let subs = subscriptions.lock().unwrap();
            for callback in subs.values() {
                callback(&processed);
            }
        }));

        self.status.state = TrackerState::Tracking;
        self.status.tracking = true;
    }

    /// Stop tracking
    pub fn stop_tracking(&mut self) {
        if !self.status.tracking {
            return;
        }

        self.adapter.stop_native_stream();
        self.status.state = TrackerState::Connected;
        self.status.tracking = false;
    }

    /// Check if tracking
    pub fn is_tracking(&self) -> bool {
        self.status.tracking
    }

    /// Register event handler
    pub fn on<F>(&self, event_type: GazeEventType, handler: F)
    where
        F: Fn(&GazeEvent) + Send + Sync + 'static,
    {
        let mut handlers = self.event_handlers.lock().unwrap();
        handlers
            .entry(event_type)
            .or_insert_with(Vec::new)
            .push(Box::new(handler));
    }

    /// Get capabilities
    pub fn get_capabilities(&self) -> EyeTrackerCapabilities {
        self.adapter.get_capabilities()
    }

    /// Get status
    pub fn get_status(&self) -> TrackerStatus {
        self.status.clone()
    }

    /// Get latest gaze
    pub fn get_latest_gaze(&self) -> Option<GazePoint> {
        self.latest_gaze.lock().unwrap().clone()
    }

    fn emit_event(&self, event_type: GazeEventType) {
        let event = GazeEvent {
            event_type,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            event_id: Uuid::new_v4().to_string(),
            duration: None,
            position: None,
            target: None,
            gaze_data: None,
        };

        let handlers = self.event_handlers.lock().unwrap();
        if let Some(handlers) = handlers.get(&event_type) {
            for handler in handlers {
                handler(&event);
            }
        }
    }

    fn default_calibration_points(&self) -> Vec<CalibrationPoint> {
        vec![
            CalibrationPoint::new(0.5, 0.5, 0),
            CalibrationPoint::new(0.1, 0.1, 1),
            CalibrationPoint::new(0.9, 0.1, 2),
            CalibrationPoint::new(0.1, 0.9, 3),
            CalibrationPoint::new(0.9, 0.9, 4),
        ]
    }
}
