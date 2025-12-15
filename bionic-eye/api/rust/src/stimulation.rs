//! Stimulation control for WIA Bionic Eye

use crate::types::*;
use crate::electrode::*;
use serde::{Deserialize, Serialize};

/// Stimulation controller
pub struct StimulationController {
    array: ElectrodeArray,
    params: StimulationParams,
    strategy: MappingStrategy,
    state: ControllerState,
}

/// Controller state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerState {
    Idle,
    Running,
    Paused,
    EmergencyStop,
}

/// Stimulation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StimulationResult {
    pub frame_id: String,
    pub timestamp: i64,
    pub electrodes_stimulated: Vec<u32>,
    pub amplitudes: Vec<f32>,
    pub total_charge_nc: f32,
    pub success: bool,
    pub warnings: Vec<String>,
}

impl StimulationController {
    /// Create a new stimulation controller
    pub fn new(array: ElectrodeArray) -> Self {
        Self {
            array,
            params: StimulationParams::default(),
            strategy: MappingStrategy::Scoreboard,
            state: ControllerState::Idle,
        }
    }

    /// Start stimulation
    pub fn start(&mut self) -> Result<(), StimulationError> {
        if self.state == ControllerState::EmergencyStop {
            return Err(StimulationError::EmergencyStopActive);
        }
        self.state = ControllerState::Running;
        Ok(())
    }

    /// Stop stimulation
    pub fn stop(&mut self) {
        self.state = ControllerState::Idle;
    }

    /// Pause stimulation
    pub fn pause(&mut self) {
        if self.state == ControllerState::Running {
            self.state = ControllerState::Paused;
        }
    }

    /// Resume stimulation
    pub fn resume(&mut self) {
        if self.state == ControllerState::Paused {
            self.state = ControllerState::Running;
        }
    }

    /// Emergency stop
    pub fn emergency_stop(&mut self) {
        self.state = ControllerState::EmergencyStop;
    }

    /// Set mapping strategy
    pub fn set_strategy(&mut self, strategy: MappingStrategy) {
        self.strategy = strategy;
    }

    /// Set stimulation parameters
    pub fn set_params(&mut self, params: StimulationParams) {
        self.params = params;
    }

    /// Get current state
    pub fn state(&self) -> ControllerState {
        self.state
    }

    /// Stimulate based on processed visual
    pub fn stimulate_frame(&self, visual: &ProcessedVisual) -> Result<StimulationResult, StimulationError> {
        if self.state != ControllerState::Running {
            return Err(StimulationError::NotRunning);
        }

        let map = self.generate_stimulation_map(visual);
        let mut result = StimulationResult {
            frame_id: visual.frame_id.clone(),
            timestamp: chrono::Utc::now().timestamp_millis(),
            electrodes_stimulated: Vec::new(),
            amplitudes: Vec::new(),
            total_charge_nc: 0.0,
            success: true,
            warnings: Vec::new(),
        };

        // Apply stimulation based on map
        for (idx, &intensity) in map.active_electrodes.iter().zip(map.intensities.iter()) {
            if let Some(electrode) = self.array.electrodes.get(idx as usize) {
                if !electrode.functional || !electrode.enabled {
                    continue;
                }

                let amplitude = self.intensity_to_amplitude(intensity, electrode);

                // Safety check
                if amplitude > electrode.max_safe_current_ua {
                    result.warnings.push(format!(
                        "Electrode {} amplitude limited: {} -> {}",
                        idx, amplitude, electrode.max_safe_current_ua
                    ));
                    continue;
                }

                result.electrodes_stimulated.push(idx);
                result.amplitudes.push(amplitude);
                result.total_charge_nc += self.calculate_charge(amplitude);
            }
        }

        Ok(result)
    }

    /// Generate stimulation map from processed visual
    fn generate_stimulation_map(&self, visual: &ProcessedVisual) -> StimulationMap {
        let rows = self.array.physical.rows;
        let cols = self.array.physical.columns;
        let total = (rows * cols) as usize;

        let mut active = Vec::new();
        let mut intensities = Vec::new();

        match self.strategy {
            MappingStrategy::Scoreboard => {
                // Simple scoreboard: map object centers to nearest electrodes
                for obj in &visual.objects {
                    let electrode_x = (obj.bounding_box.x * cols as f32) as u32;
                    let electrode_y = (obj.bounding_box.y * rows as f32) as u32;
                    let idx = electrode_y * cols + electrode_x;

                    if (idx as usize) < total {
                        let intensity = (obj.priority as u8) * 50 + 50;
                        active.push(idx);
                        intensities.push(intensity.min(255));
                    }
                }
            }
            MappingStrategy::Direct => {
                // Direct mapping would require actual image data
                // Placeholder implementation
            }
            _ => {
                // Other strategies
            }
        }

        StimulationMap {
            frame_id: visual.frame_id.clone(),
            timestamp: chrono::Utc::now().timestamp_millis(),
            active_electrodes: active,
            intensities,
            strategy: self.strategy,
        }
    }

    /// Convert intensity (0-255) to amplitude in μA
    fn intensity_to_amplitude(&self, intensity: u8, electrode: &ElectrodeInfo) -> f32 {
        let normalized = intensity as f32 / 255.0;
        let range = electrode.max_safe_current_ua - electrode.threshold_current_ua;
        electrode.threshold_current_ua + (normalized * range)
    }

    /// Calculate charge for given amplitude
    fn calculate_charge(&self, amplitude_ua: f32) -> f32 {
        // Q = I * t (charge = current * pulse width)
        amplitude_ua * self.params.pulse_width_us / 1000.0  // Result in nC
    }
}

impl Default for StimulationParams {
    fn default() -> Self {
        Self {
            waveform: WaveformType::BiphasicSymmetric,
            amplitude_ua: 200.0,
            pulse_width_us: 200.0,
            frequency_hz: 20.0,
            interphase_gap_us: 50.0,
            pulses_per_burst: 1,
        }
    }
}

/// Stimulation errors
#[derive(Debug, Clone, thiserror::Error)]
pub enum StimulationError {
    #[error("Controller not running")]
    NotRunning,

    #[error("Emergency stop active")]
    EmergencyStopActive,

    #[error("Safety limit exceeded: {0}")]
    SafetyLimitExceeded(String),

    #[error("Invalid electrode: {0}")]
    InvalidElectrode(u32),
}
