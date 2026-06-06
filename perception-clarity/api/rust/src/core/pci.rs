//! PCI computation and the state / safe-action mappings — Phase 1 §3.

use serde::{Deserialize, Serialize};

use crate::types::{ClarityAxes, ClarityState, PciIndex, PciWeights};

/// Compute the PCI from the three damage axes and a weight set — Phase 1 §3.1.
///
/// ```text
/// PCI = round( 100 - ( w_occ·OCC + w_dist·DIST + w_mtf·MTF ) × 100 )
/// ```
///
/// Each axis damage value (`0.0–1.0`) is weighted, summed into a `0.0–1.0`
/// damage fraction, scaled to a 0–100 damage scale, subtracted from 100, and
/// clamped to `[0, 100]`.
///
/// # Example
/// ```
/// use wia_perception_clarity::prelude::*;
/// let axes = ClarityAxes::new(0.18, 0.22, 0.40);
/// let weights = PciWeights::default_for(SensorClass::RgbCamera);
/// let pci = compute_pci(&axes, &weights);
/// assert_eq!(pci.state(), ClarityState::Degraded);
/// ```
pub fn compute_pci(axes: &ClarityAxes, weights: &PciWeights) -> PciIndex {
    debug_assert!(weights.is_valid(), "weight set must sum to 1.00 (±0.001)");

    let damage = weights.occlusion * axes.occlusion
        + weights.distance * axes.distance_degradation
        + weights.mtf * axes.mtf_reduction; // 0.0–1.0

    let pci = 100.0 - damage * 100.0;
    PciIndex::new(pci.round() as i32)
}

/// Map a PCI to its default clarity state — Phase 1 §3.3.
///
/// Thin free-function wrapper over [`PciIndex::state`] for call sites that
/// prefer a function over a method.
pub fn pci_to_state(pci: PciIndex) -> ClarityState {
    pci.state()
}

/// A safe action a consuming system may take in response to a clarity state —
/// Phase 1 §5.1 `safeActionTriggers`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafeAction {
    /// Reduce speed.
    SlowDown,
    /// Reroute / take an alternate path.
    Reroute,
    /// Trigger a cleaning cycle.
    TriggerCleaning,
    /// Enter a minimal-risk safe state.
    SafeState,
    /// Disable the affected sensor and exclude it from fusion.
    DisableSensor,
}

/// The safe actions required for a given clarity state.
///
/// `Clear` requires none. `Degraded` triggers monitoring/cleaning. `Obstructed`
/// adds slow-down and reroute. `Blind` enters safe-state and disables the sensor.
pub fn required_safe_actions(state: ClarityState) -> Vec<SafeAction> {
    match state {
        ClarityState::Clear => vec![],
        ClarityState::Degraded => vec![SafeAction::TriggerCleaning],
        ClarityState::Obstructed => vec![
            SafeAction::SlowDown,
            SafeAction::Reroute,
            SafeAction::TriggerCleaning,
        ],
        ClarityState::Blind => vec![SafeAction::DisableSensor, SafeAction::SafeState],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::SensorClass;

    #[test]
    fn clean_sensor_is_clear_100() {
        let pci = compute_pci(&ClarityAxes::clean(), &PciWeights::default_for(SensorClass::RgbCamera));
        assert_eq!(pci.value(), 100);
        assert_eq!(pci.state(), ClarityState::Clear);
    }

    #[test]
    fn rain_camera_is_degraded() {
        let axes = ClarityAxes::new(0.18, 0.22, 0.40);
        let weights = PciWeights::default_for(SensorClass::RgbCamera);
        let pci = compute_pci(&axes, &weights);
        // 100 - (0.072 + 0.055 + 0.140) * 100 = 73.3 -> 73, within the degraded band.
        assert_eq!(pci.value(), 73);
        assert_eq!(pci.state(), ClarityState::Degraded);
    }

    #[test]
    fn mud_lidar_is_blind() {
        let axes = ClarityAxes::new(0.81, 0.88, 0.30);
        let weights = PciWeights::default_for(SensorClass::LidarWindow);
        let pci = compute_pci(&axes, &weights);
        assert_eq!(pci.state(), ClarityState::Blind);
    }

    #[test]
    fn blind_requires_safe_state() {
        let actions = required_safe_actions(ClarityState::Blind);
        assert!(actions.contains(&SafeAction::SafeState));
        assert!(actions.contains(&SafeAction::DisableSensor));
    }
}
