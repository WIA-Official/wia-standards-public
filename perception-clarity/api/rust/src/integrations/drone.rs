//! Drone integration: return-to-home when a sensor is obstructed.
//!
//! For an aerial agent, an obstructed obstacle-avoidance sensor is sufficient to
//! abort the mission and recover, while a blind sensor demands an immediate
//! landing. This maps the agent-wide worst clarity state to a flight action.

use serde::{Deserialize, Serialize};

use crate::types::{ClarityState, SensorClarityReport};

/// A drone flight-control action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DroneAction {
    /// Continue the planned mission.
    ContinueMission,
    /// Reduce speed and hold position briefly.
    HoldAndAssess,
    /// Abort and return to the home / launch point.
    ReturnToHome,
    /// Land immediately at the safest reachable point.
    LandImmediately,
}

/// Decide the flight action from a drone's report.
///
/// `obstructed` triggers return-to-home (the requested behavior); `blind`
/// escalates to an immediate landing; `degraded` holds to reassess; `clear`
/// continues.
pub fn drone_action(report: &SensorClarityReport) -> DroneAction {
    match report.worst_state() {
        ClarityState::Blind => DroneAction::LandImmediately,
        ClarityState::Obstructed => DroneAction::ReturnToHome,
        ClarityState::Degraded => DroneAction::HoldAndAssess,
        ClarityState::Clear => DroneAction::ContinueMission,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{ReportBuilder, SensorBuilder};
    use crate::types::{AgentType, ClarityAxes, SensorClass};

    #[test]
    fn obstructed_camera_returns_to_home() {
        // Axes chosen to land squarely in the obstructed band (PCI 30–59).
        let cam =
            SensorBuilder::from_axes("obstacle-cam", SensorClass::RgbCamera, ClarityAxes::new(0.55, 0.50, 0.50))
                .dwell_seconds(20.0)
                .build()
                .unwrap();
        assert_eq!(cam.state, ClarityState::Obstructed);

        let report = ReportBuilder::new("drone-7", AgentType::Drone)
            .add_sensor(cam)
            .build()
            .unwrap();
        assert_eq!(drone_action(&report), DroneAction::ReturnToHome);
    }
}
