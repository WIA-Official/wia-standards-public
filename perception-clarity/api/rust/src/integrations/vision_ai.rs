//! Vision-AI integration: gate inference on a minimum PCI.
//!
//! A perception model's outputs are only as trustworthy as the imagery feeding
//! it. This gate decides whether to run inference for a given sensor and how
//! much to discount the resulting detections.

use crate::types::{ClarityState, PciIndex, Sensor};

/// A decision about whether — and how — to run inference on a sensor's feed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InferenceGate {
    /// Whether inference should run at all.
    pub run_inference: bool,
    /// Multiplier applied to model confidence scores (`0.0–1.0`).
    pub confidence_scale: f64,
    /// Recommended human / secondary review of the outputs.
    pub flag_for_review: bool,
}

/// Decide an [`InferenceGate`] from a sensor, given a minimum acceptable PCI.
///
/// Below `min_pci`, or when the sensor is blind, inference is skipped entirely.
/// Otherwise inference runs with a confidence scale tied to the PCI, and
/// non-clear states are flagged for review.
pub fn gate_inference(sensor: &Sensor, min_pci: PciIndex) -> InferenceGate {
    let blind = sensor.state == ClarityState::Blind;
    let below_floor = sensor.pci < min_pci;

    if blind || below_floor {
        return InferenceGate {
            run_inference: false,
            confidence_scale: 0.0,
            flag_for_review: true,
        };
    }

    let scale = (sensor.pci.value() as f64 / 100.0).clamp(0.0, 1.0);
    InferenceGate {
        run_inference: true,
        confidence_scale: scale,
        flag_for_review: sensor.state != ClarityState::Clear,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::SensorBuilder;
    use crate::types::{ClarityAxes, SensorClass};

    #[test]
    fn blind_sensor_skips_inference() {
        let blind =
            SensorBuilder::from_axes("cam", SensorClass::LidarWindow, ClarityAxes::new(0.81, 0.88, 0.30))
                .dwell_seconds(312.0)
                .build()
                .unwrap();
        let gate = gate_inference(&blind, PciIndex::new(50));
        assert!(!gate.run_inference);
    }

    #[test]
    fn degraded_sensor_runs_but_flags_review() {
        let degraded =
            SensorBuilder::from_axes("cam", SensorClass::RgbCamera, ClarityAxes::new(0.18, 0.22, 0.40))
                .dwell_seconds(47.0)
                .build()
                .unwrap();
        let gate = gate_inference(&degraded, PciIndex::new(50));
        assert!(gate.run_inference);
        assert!(gate.flag_for_review);
        assert!(gate.confidence_scale < 1.0);
    }
}
