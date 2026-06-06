//! A mock sensor producing synthetic PCI readings.
//!
//! [`SimulatedSensor`] models a single sensor whose contamination accumulates
//! over time and resets on a cleaning cycle. Each tick advances the damage axes,
//! recomputes the PCI from the class default weights, and emits a [`Sensor`]
//! snapshot. The progression is deterministic so tests can assert on it.

use chrono::{DateTime, Duration, Utc};

use crate::core::{compute_pci, SensorBuilder};
use crate::error::Result;
use crate::types::{
    ClarityAxes, ClarityState, Contaminant, ContaminantType, PciWeights, Sensor, SensorClass,
};

/// A deterministic synthetic sensor.
#[derive(Debug, Clone)]
pub struct SimulatedSensor {
    sensor_id: String,
    sensor_class: SensorClass,
    weights: PciWeights,
    /// Per-tick damage added to every axis (clamped to 1.0).
    soiling_rate: f64,
    /// Current damage axes.
    axes: ClarityAxes,
    /// Dominant contaminant being simulated.
    contaminant: ContaminantType,
    /// Seconds per tick.
    tick_seconds: f64,
    /// Accumulated dwell in the current non-CLEAR state.
    dwell_seconds: f64,
    /// Simulation clock.
    now: DateTime<Utc>,
    last_cleaned_at: Option<DateTime<Utc>>,
}

impl SimulatedSensor {
    /// Create a clean simulated sensor.
    pub fn new(sensor_id: impl Into<String>, sensor_class: SensorClass) -> Self {
        Self {
            sensor_id: sensor_id.into(),
            sensor_class,
            weights: PciWeights::default_for(sensor_class),
            soiling_rate: 0.05,
            axes: ClarityAxes::clean(),
            contaminant: ContaminantType::MudDust,
            tick_seconds: 1.0,
            dwell_seconds: 0.0,
            now: Utc::now(),
            last_cleaned_at: None,
        }
    }

    /// Set the per-tick soiling rate (damage added per axis each tick).
    pub fn soiling_rate(mut self, rate: f64) -> Self {
        self.soiling_rate = rate;
        self
    }

    /// Set the dominant contaminant being simulated.
    pub fn contaminant(mut self, contaminant: ContaminantType) -> Self {
        self.contaminant = contaminant;
        self
    }

    /// Set the seconds represented by one tick.
    pub fn tick_seconds(mut self, secs: f64) -> Self {
        self.tick_seconds = secs;
        self
    }

    /// The current synthetic PCI state, without advancing the clock.
    pub fn current_state(&self) -> ClarityState {
        compute_pci(&self.axes, &self.weights).state()
    }

    /// Run a cleaning cycle: reset all damage axes and the dwell counter.
    pub fn clean(&mut self) {
        self.axes = ClarityAxes::clean();
        self.dwell_seconds = 0.0;
        self.last_cleaned_at = Some(self.now);
    }

    /// Advance the simulation by one tick and emit a [`Sensor`] snapshot.
    pub fn tick(&mut self) -> Result<Sensor> {
        // Accumulate damage on every axis (occlusion grows fastest).
        let bump = |v: f64, k: f64| (v + self.soiling_rate * k).clamp(0.0, 1.0);
        self.axes = ClarityAxes::new(
            bump(self.axes.occlusion, 1.0),
            bump(self.axes.distance_degradation, 0.9),
            bump(self.axes.mtf_reduction, 0.7),
        );
        self.now += Duration::milliseconds((self.tick_seconds * 1000.0) as i64);

        let pci = compute_pci(&self.axes, &self.weights);
        if pci.state() == ClarityState::Clear {
            self.dwell_seconds = 0.0;
        } else {
            self.dwell_seconds += self.tick_seconds;
        }

        let coverage = self.axes.occlusion;
        let mut builder = SensorBuilder::from_axes(self.sensor_id.clone(), self.sensor_class, self.axes)
            .dwell_seconds(self.dwell_seconds)
            .confidence(0.9);
        if let Some(ts) = self.last_cleaned_at {
            builder = builder.last_cleaned_at(ts);
        }
        if coverage > 0.0 {
            builder = builder.add_contaminant(Contaminant::new(self.contaminant, coverage));
        }
        builder.build()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soiling_progresses_toward_blind() {
        let mut sim = SimulatedSensor::new("nav-lidar", SensorClass::LidarWindow)
            .soiling_rate(0.1)
            .contaminant(ContaminantType::MudDust);

        let mut last = ClarityState::Clear;
        for _ in 0..30 {
            let snap = sim.tick().unwrap();
            last = snap.state;
            if last == ClarityState::Blind {
                break;
            }
        }
        assert_eq!(last, ClarityState::Blind);
        assert!(sim.tick().unwrap().dwell_seconds > 0.0);

        // Cleaning restores clarity.
        sim.clean();
        assert_eq!(sim.current_state(), ClarityState::Clear);
    }
}
