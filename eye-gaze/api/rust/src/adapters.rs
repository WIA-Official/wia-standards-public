//! WIA Eye Gaze Standard - Adapters
//!
//! 弘益人間 - 널리 인간을 이롭게

use async_trait::async_trait;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::tracker::{EyeTrackerAdapter, GazeCallback};
use crate::types::*;

/// Mock adapter for testing
pub struct MockAdapter {
    sampling_rate: u32,
    add_noise: bool,
    noise_level: f64,
    simulate_blinks: bool,
    blink_interval_ms: u64,
    connected: bool,
    streaming: Arc<Mutex<bool>>,
    current_x: Arc<Mutex<f64>>,
    current_y: Arc<Mutex<f64>>,
    last_blink: Arc<Mutex<Instant>>,
}

impl MockAdapter {
    /// Create new mock adapter
    pub fn new(sampling_rate: u32) -> Self {
        Self {
            sampling_rate,
            add_noise: true,
            noise_level: 0.01,
            simulate_blinks: true,
            blink_interval_ms: 4000,
            connected: false,
            streaming: Arc::new(Mutex::new(false)),
            current_x: Arc::new(Mutex::new(0.5)),
            current_y: Arc::new(Mutex::new(0.5)),
            last_blink: Arc::new(Mutex::new(Instant::now())),
        }
    }

    /// Set noise level
    pub fn with_noise(mut self, level: f64) -> Self {
        self.noise_level = level;
        self
    }

    /// Disable noise
    pub fn without_noise(mut self) -> Self {
        self.add_noise = false;
        self
    }

    fn generate_gaze_point(&self) -> GazePoint {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let mut x = *self.current_x.lock().unwrap();
        let mut y = *self.current_y.lock().unwrap();

        // Check blink
        let should_blink = {
            let last_blink = self.last_blink.lock().unwrap();
            last_blink.elapsed().as_millis() as u64 > self.blink_interval_ms
        };

        if self.simulate_blinks && should_blink {
            *self.last_blink.lock().unwrap() = Instant::now();
            return GazePoint {
                timestamp: now,
                x,
                y,
                confidence: 0.0,
                valid: false,
                left_eye: None,
                right_eye: None,
                fixation: None,
                saccade: None,
                fixation_id: None,
                device_timestamp: Some(now),
            };
        }

        // Add noise
        if self.add_noise {
            x += (rand_float() - 0.5) * self.noise_level * 2.0;
            y += (rand_float() - 0.5) * self.noise_level * 2.0;
        }

        // Simulate movement
        {
            let mut cx = self.current_x.lock().unwrap();
            let mut cy = self.current_y.lock().unwrap();
            *cx += (rand_float() - 0.5) * 0.02;
            *cy += (rand_float() - 0.5) * 0.02;
            *cx = cx.clamp(0.1, 0.9);
            *cy = cy.clamp(0.1, 0.9);
        }

        let pupil_diameter = 3.5 + rand_float() * 1.0;
        let eye_openness = 0.85 + rand_float() * 0.1;

        GazePoint {
            timestamp: now,
            x: x.clamp(0.0, 1.0),
            y: y.clamp(0.0, 1.0),
            confidence: 0.9 + rand_float() * 0.1,
            valid: true,
            left_eye: Some(EyeData {
                gaze: Vector2D::new(x - 0.005, y),
                valid: true,
                pupil_diameter: Some(pupil_diameter),
                pupil_center: None,
                gaze_origin: None,
                gaze_direction: None,
                eye_openness: Some(eye_openness),
            }),
            right_eye: Some(EyeData {
                gaze: Vector2D::new(x + 0.005, y),
                valid: true,
                pupil_diameter: Some(pupil_diameter + 0.1),
                pupil_center: None,
                gaze_origin: None,
                gaze_direction: None,
                eye_openness: Some(eye_openness),
            }),
            fixation: None,
            saccade: None,
            fixation_id: None,
            device_timestamp: Some(now),
        }
    }
}

#[async_trait]
impl EyeTrackerAdapter for MockAdapter {
    fn name(&self) -> &str {
        "mock"
    }

    async fn connect_native(&mut self) -> Result<()> {
        tokio::time::sleep(Duration::from_millis(100)).await;
        self.connected = true;
        Ok(())
    }

    async fn disconnect_native(&mut self) -> Result<()> {
        self.stop_native_stream();
        self.connected = false;
        Ok(())
    }

    fn start_native_stream(&mut self, callback: GazeCallback) {
        let streaming = Arc::clone(&self.streaming);
        {
            let mut s = streaming.lock().unwrap();
            if *s {
                return;
            }
            *s = true;
        }

        let interval = Duration::from_micros(1_000_000 / self.sampling_rate as u64);
        let current_x = Arc::clone(&self.current_x);
        let current_y = Arc::clone(&self.current_y);
        let last_blink = Arc::clone(&self.last_blink);
        let add_noise = self.add_noise;
        let noise_level = self.noise_level;
        let simulate_blinks = self.simulate_blinks;
        let blink_interval_ms = self.blink_interval_ms;

        thread::spawn(move || {
            while *streaming.lock().unwrap() {
                let now = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                let mut x = *current_x.lock().unwrap();
                let mut y = *current_y.lock().unwrap();

                // Check blink
                let should_blink = {
                    let last = last_blink.lock().unwrap();
                    last.elapsed().as_millis() as u64 > blink_interval_ms
                };

                let gaze = if simulate_blinks && should_blink {
                    *last_blink.lock().unwrap() = Instant::now();
                    GazePoint::new(now, x, y, 0.0, false)
                } else {
                    if add_noise {
                        x += (rand_float() - 0.5) * noise_level * 2.0;
                        y += (rand_float() - 0.5) * noise_level * 2.0;
                    }

                    {
                        let mut cx = current_x.lock().unwrap();
                        let mut cy = current_y.lock().unwrap();
                        *cx += (rand_float() - 0.5) * 0.02;
                        *cy += (rand_float() - 0.5) * 0.02;
                        *cx = cx.clamp(0.1, 0.9);
                        *cy = cy.clamp(0.1, 0.9);
                    }

                    GazePoint::new(now, x.clamp(0.0, 1.0), y.clamp(0.0, 1.0), 0.95, true)
                };

                callback(&gaze);
                thread::sleep(interval);
            }
        });
    }

    fn stop_native_stream(&mut self) {
        let mut streaming = self.streaming.lock().unwrap();
        *streaming = false;
    }

    fn get_capabilities(&self) -> EyeTrackerCapabilities {
        EyeTrackerCapabilities {
            device: EyeTrackerInfo {
                device_id: "mock-tracker-001".to_string(),
                vendor: "WIA".to_string(),
                model: "Mock Eye Tracker".to_string(),
                firmware_version: "1.0.0".to_string(),
                protocol_version: "1.0.0".to_string(),
                device_type: Some(DeviceType::WebcamBased),
            },
            tracking: TrackingCapabilities {
                binocular: true,
                head_tracking: false,
                gaze_3d: false,
                sampling_rate: SamplingRateSpec {
                    supported: vec![30, 60, 120],
                    default: 60,
                    current: Some(self.sampling_rate),
                },
                accuracy_typical: 1.0,
                precision_typical: 0.5,
                latency_average: 20.0,
            },
            supported_features: vec![
                "GAZE_POINT".to_string(),
                "BINOCULAR".to_string(),
                "PUPIL_DIAMETER".to_string(),
                "EYE_OPENNESS".to_string(),
                "CALIBRATION".to_string(),
            ],
        }
    }

    async fn calibrate(&mut self, points: &[CalibrationPoint]) -> Result<CalibrationResult> {
        tokio::time::sleep(Duration::from_millis(500)).await;

        let point_results: Vec<PointCalibrationResult> = points
            .iter()
            .enumerate()
            .map(|(i, p)| PointCalibrationResult {
                point_index: i as u32,
                position: Vector2D::new(p.x, p.y),
                accuracy: 0.5 + rand_float() * 0.5,
                precision: 0.05 + rand_float() * 0.1,
                valid: true,
            })
            .collect();

        Ok(CalibrationResult {
            success: true,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            average_accuracy: Some(0.8 + rand_float() * 0.4),
            average_precision: Some(0.1 + rand_float() * 0.1),
            point_results: Some(point_results),
        })
    }
}

// Simple random float (0.0 - 1.0)
fn rand_float() -> f64 {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    let state = RandomState::new();
    let mut hasher = state.build_hasher();
    hasher.write_u64(
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64,
    );
    (hasher.finish() % 10000) as f64 / 10000.0
}
