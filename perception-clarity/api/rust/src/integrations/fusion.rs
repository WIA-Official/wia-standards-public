//! Sensor-fusion integration: down-weight a degraded sensor.
//!
//! A fusion stack typically combines sensor outputs with per-sensor weights. As
//! a sensor's clarity drops, its trust should drop with it. This maps a
//! [`Sensor`]'s PCI/state to a fusion weight in `[0.0, 1.0]`.

use crate::types::{ClarityState, Sensor};

/// A fusion weight for one sensor — `1.0` = full trust, `0.0` = excluded.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FusionWeight(pub f64);

impl FusionWeight {
    /// The weight value.
    pub fn value(self) -> f64 {
        self.0
    }
}

/// Map a sensor's clarity to a fusion weight.
///
/// `clear` keeps full weight; `degraded` is scaled by PCI (so a 60-PCI sensor
/// counts less than an 89-PCI one); `obstructed` is heavily attenuated; `blind`
/// is excluded entirely. Confidence in the PCI computation further scales the
/// result, so a low-confidence reading never dominates the fused estimate.
pub fn fusion_weight(sensor: &Sensor) -> FusionWeight {
    let pci = sensor.pci.value() as f64 / 100.0;
    let base = match sensor.state {
        ClarityState::Clear => 1.0,
        ClarityState::Degraded => pci,        // 0.60–0.89
        ClarityState::Obstructed => pci * 0.5, // 0.15–0.295
        ClarityState::Blind => 0.0,
    };
    let conf = sensor.confidence.clamp(0.0, 1.0);
    FusionWeight((base * conf).clamp(0.0, 1.0))
}

/// Compute normalized fusion weights over a set of sensors (weights sum to 1.0
/// unless every sensor is blind, in which case all are 0.0).
pub fn normalized_fusion_weights(sensors: &[Sensor]) -> Vec<f64> {
    let raw: Vec<f64> = sensors.iter().map(|s| fusion_weight(s).value()).collect();
    let total: f64 = raw.iter().sum();
    if total <= f64::EPSILON {
        return vec![0.0; sensors.len()];
    }
    raw.into_iter().map(|w| w / total).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::SensorBuilder;
    use crate::types::{ClarityAxes, SensorClass};

    #[test]
    fn degraded_sensor_is_down_weighted_below_clear() {
        let clear = SensorBuilder::from_axes("a", SensorClass::RgbCamera, ClarityAxes::clean())
            .build()
            .unwrap();
        let degraded =
            SensorBuilder::from_axes("b", SensorClass::RgbCamera, ClarityAxes::new(0.18, 0.22, 0.40))
                .dwell_seconds(47.0)
                .build()
                .unwrap();

        assert!(fusion_weight(&degraded).value() < fusion_weight(&clear).value());
    }

    #[test]
    fn blind_sensor_is_excluded() {
        let blind =
            SensorBuilder::from_axes("c", SensorClass::LidarWindow, ClarityAxes::new(0.81, 0.88, 0.30))
                .dwell_seconds(312.0)
                .build()
                .unwrap();
        assert_eq!(fusion_weight(&blind).value(), 0.0);
    }
}
