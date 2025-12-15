//! WIA Eye Gaze Standard - Dwell Controller
//!
//! 弘益人間 - 널리 인간을 이롭게

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use crate::types::*;

/// Dwell event handler type
pub type DwellEventHandler = Box<dyn Fn(&GazeTarget, Option<f64>) + Send + Sync>;

/// Dwell Controller for gaze-based selection
pub struct DwellController {
    config: DwellConfig,
    targets: Arc<Mutex<HashMap<String, GazeTarget>>>,
    state: Arc<Mutex<DwellState>>,
    cooldown_until: Arc<Mutex<u64>>,
    enabled: Arc<Mutex<bool>>,

    on_start_handlers: Arc<Mutex<Vec<DwellEventHandler>>>,
    on_progress_handlers: Arc<Mutex<Vec<DwellEventHandler>>>,
    on_complete_handlers: Arc<Mutex<Vec<DwellEventHandler>>>,
    on_cancel_handlers: Arc<Mutex<Vec<DwellEventHandler>>>,
}

impl DwellController {
    /// Create new dwell controller
    pub fn new(config: DwellConfig) -> Self {
        Self {
            config,
            targets: Arc::new(Mutex::new(HashMap::new())),
            state: Arc::new(Mutex::new(DwellState::default())),
            cooldown_until: Arc::new(Mutex::new(0)),
            enabled: Arc::new(Mutex::new(false)),
            on_start_handlers: Arc::new(Mutex::new(Vec::new())),
            on_progress_handlers: Arc::new(Mutex::new(Vec::new())),
            on_complete_handlers: Arc::new(Mutex::new(Vec::new())),
            on_cancel_handlers: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Create with default config
    pub fn default() -> Self {
        Self::new(DwellConfig::default())
    }

    /// Set dwell threshold
    pub fn set_dwell_time(&mut self, ms: u64) {
        self.config.threshold_ms = ms;
    }

    /// Get dwell time
    pub fn get_dwell_time(&self) -> u64 {
        self.config.threshold_ms
    }

    /// Register target
    pub fn register_target(&self, target: GazeTarget) {
        let mut targets = self.targets.lock().unwrap();
        targets.insert(target.element_id.clone(), target);
    }

    /// Unregister target
    pub fn unregister_target(&self, target_id: &str) {
        let mut targets = self.targets.lock().unwrap();
        targets.remove(target_id);
    }

    /// Clear all targets
    pub fn clear_targets(&self) {
        let mut targets = self.targets.lock().unwrap();
        targets.clear();
        self.cancel_dwell();
    }

    /// Get targets
    pub fn get_targets(&self) -> Vec<GazeTarget> {
        let targets = self.targets.lock().unwrap();
        targets.values().cloned().collect()
    }

    /// Register dwell start handler
    pub fn on_dwell_start<F>(&self, handler: F)
    where
        F: Fn(&GazeTarget, Option<f64>) + Send + Sync + 'static,
    {
        let mut handlers = self.on_start_handlers.lock().unwrap();
        handlers.push(Box::new(handler));
    }

    /// Register dwell progress handler
    pub fn on_dwell_progress<F>(&self, handler: F)
    where
        F: Fn(&GazeTarget, Option<f64>) + Send + Sync + 'static,
    {
        let mut handlers = self.on_progress_handlers.lock().unwrap();
        handlers.push(Box::new(handler));
    }

    /// Register dwell complete handler
    pub fn on_dwell_complete<F>(&self, handler: F)
    where
        F: Fn(&GazeTarget, Option<f64>) + Send + Sync + 'static,
    {
        let mut handlers = self.on_complete_handlers.lock().unwrap();
        handlers.push(Box::new(handler));
    }

    /// Register dwell cancel handler
    pub fn on_dwell_cancel<F>(&self, handler: F)
    where
        F: Fn(&GazeTarget, Option<f64>) + Send + Sync + 'static,
    {
        let mut handlers = self.on_cancel_handlers.lock().unwrap();
        handlers.push(Box::new(handler));
    }

    /// Start dwell detection
    pub fn start(&self) {
        let mut enabled = self.enabled.lock().unwrap();
        *enabled = true;
    }

    /// Stop dwell detection
    pub fn stop(&self) {
        let mut enabled = self.enabled.lock().unwrap();
        *enabled = false;
        self.cancel_dwell();
    }

    /// Pause dwell detection
    pub fn pause(&self) {
        let mut enabled = self.enabled.lock().unwrap();
        *enabled = false;
        self.cancel_dwell();
    }

    /// Resume dwell detection
    pub fn resume(&self) {
        let mut enabled = self.enabled.lock().unwrap();
        *enabled = true;
    }

    /// Get current state
    pub fn get_state(&self) -> DwellState {
        self.state.lock().unwrap().clone()
    }

    /// Process gaze data
    pub fn process_gaze(&self, gaze: &GazePoint) {
        if !*self.enabled.lock().unwrap() || !gaze.valid {
            if self.state.lock().unwrap().active {
                self.cancel_dwell();
            }
            return;
        }

        // Check cooldown
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        if now < *self.cooldown_until.lock().unwrap() {
            return;
        }

        // Find hit target
        let hit_target = self.find_target_at(gaze.x, gaze.y);

        match hit_target {
            None => {
                if self.state.lock().unwrap().active {
                    self.cancel_dwell();
                }
            }
            Some(target) => {
                let current_state = self.state.lock().unwrap().clone();

                if !current_state.active {
                    self.start_dwell(target);
                } else if let Some(ref current_target) = current_state.target {
                    if current_target.element_id != target.element_id {
                        self.cancel_dwell();
                        self.start_dwell(target);
                    } else {
                        self.update_progress();
                    }
                }
            }
        }
    }

    fn find_target_at(&self, x: f64, y: f64) -> Option<GazeTarget> {
        let targets = self.targets.lock().unwrap();
        targets
            .values()
            .find(|t| t.bounding_box.contains(x, y))
            .cloned()
    }

    fn start_dwell(&self, target: GazeTarget) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        {
            let mut state = self.state.lock().unwrap();
            state.active = true;
            state.target = Some(target.clone());
            state.start_time = Some(now);
            state.progress = 0.0;
        }

        self.emit_event(&self.on_start_handlers, &target, None);
    }

    fn update_progress(&self) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let (target, progress, complete) = {
            let mut state = self.state.lock().unwrap();

            if !state.active || state.target.is_none() || state.start_time.is_none() {
                return;
            }

            let elapsed = now - state.start_time.unwrap();
            let progress = (elapsed as f64 / self.config.threshold_ms as f64).min(1.0);
            state.progress = progress;

            (state.target.clone().unwrap(), progress, progress >= 1.0)
        };

        self.emit_event(&self.on_progress_handlers, &target, Some(progress));

        if complete {
            self.complete_dwell();
        }
    }

    fn complete_dwell(&self) {
        let target = {
            let mut state = self.state.lock().unwrap();
            let target = state.target.clone();
            state.active = false;
            state.progress = 0.0;
            state.target = None;
            state.start_time = None;
            target
        };

        if let Some(target) = target {
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            *self.cooldown_until.lock().unwrap() = now + self.config.cooldown_period_ms;
            self.emit_event(&self.on_complete_handlers, &target, Some(1.0));
        }
    }

    fn cancel_dwell(&self) {
        let target = {
            let mut state = self.state.lock().unwrap();
            if !state.active {
                return;
            }
            let target = state.target.clone();
            state.active = false;
            state.progress = 0.0;
            state.target = None;
            state.start_time = None;
            target
        };

        if let Some(target) = target {
            self.emit_event(&self.on_cancel_handlers, &target, None);
        }
    }

    fn emit_event(
        &self,
        handlers: &Arc<Mutex<Vec<DwellEventHandler>>>,
        target: &GazeTarget,
        progress: Option<f64>,
    ) {
        let handlers = handlers.lock().unwrap();
        for handler in handlers.iter() {
            handler(target, progress);
        }
    }
}
