//! Autonomous-vehicle integration: SAE J3016 fallback on a blind sensor.
//!
//! When a perception sensor goes blind, the driving automation system can no
//! longer rely on it within its operational design domain (ODD). This maps the
//! agent-wide worst clarity state to an SAE J3016 minimal-risk response.

use serde::{Deserialize, Serialize};

use crate::types::{ClarityState, SensorClarityReport};

/// An SAE J3016 fallback decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SaeFallback {
    /// Within ODD — continue automated driving.
    Nominal,
    /// Heightened monitoring; remain within ODD with reduced performance.
    DegradedOperation,
    /// Request the fallback-ready user / remote operator to take over.
    RequestTakeover,
    /// Execute a minimal-risk maneuver (controlled stop in a safe location).
    MinimalRiskManeuver,
}

/// Decide the SAE fallback from an agent's report.
///
/// A `blind` sensor forces a minimal-risk maneuver; `obstructed` requests a
/// takeover; `degraded` continues in a degraded mode; `clear` is nominal.
pub fn sae_fallback(report: &SensorClarityReport) -> SaeFallback {
    match report.worst_state() {
        ClarityState::Blind => SaeFallback::MinimalRiskManeuver,
        ClarityState::Obstructed => SaeFallback::RequestTakeover,
        ClarityState::Degraded => SaeFallback::DegradedOperation,
        ClarityState::Clear => SaeFallback::Nominal,
    }
}

/// True when the report requires the vehicle to leave fully automated operation.
pub fn requires_human_fallback(report: &SensorClarityReport) -> bool {
    matches!(
        sae_fallback(report),
        SaeFallback::RequestTakeover | SaeFallback::MinimalRiskManeuver
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{ReportBuilder, SensorBuilder};
    use crate::types::{AgentType, ClarityAxes, SensorClass};

    #[test]
    fn blind_lidar_triggers_minimal_risk_maneuver() {
        let lidar =
            SensorBuilder::from_axes("nav-lidar", SensorClass::LidarWindow, ClarityAxes::new(0.81, 0.88, 0.30))
                .dwell_seconds(312.0)
                .build()
                .unwrap();
        let report = ReportBuilder::new("veh-1", AgentType::Vehicle)
            .add_sensor(lidar)
            .build()
            .unwrap();

        assert_eq!(sae_fallback(&report), SaeFallback::MinimalRiskManeuver);
        assert!(requires_human_fallback(&report));
    }
}
