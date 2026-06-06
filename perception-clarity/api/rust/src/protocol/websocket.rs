//! Real-time clarity event stream — Phase 2 §9.
//!
//! Server → client messages are a `#[serde(tag = "type")]` enum whose wire tags
//! match Phase 2 §9.2 (`clarity.snapshot`, `sensor.state_change`, …). The payload
//! types reuse the Phase 1 model so the stream is schema-faithful with the REST path.

use serde::{Deserialize, Serialize};

use crate::types::{ClarityState, Contaminant, PciIndex, SensorClarityReport};

/// The WebSocket subprotocol identifier used in the handshake (Phase 2 §9.1).
pub const WS_SUBPROTOCOL: &str = "wia-perception-clarity-v1";

/// A server → client clarity stream event — Phase 2 §9.2.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ClarityEvent {
    /// Full snapshot sent once right after connect.
    #[serde(rename = "clarity.snapshot")]
    Snapshot { report: SensorClarityReport },

    /// A sensor crossed a state threshold (sent immediately).
    #[serde(rename = "sensor.state_change")]
    StateChange {
        #[serde(rename = "sensorId")]
        sensor_id: String,
        from: ClarityState,
        to: ClarityState,
        pci: PciIndex,
        #[serde(rename = "dwellSeconds")]
        dwell_seconds: f64,
    },

    /// A within-band PCI update (may be debounced ≥ 1 s).
    #[serde(rename = "sensor.pci_update")]
    PciUpdate {
        #[serde(rename = "sensorId")]
        sensor_id: String,
        pci: PciIndex,
        state: ClarityState,
    },

    /// A contaminant was detected on a sensor.
    #[serde(rename = "contaminant.detected")]
    ContaminantDetected {
        #[serde(rename = "sensorId")]
        sensor_id: String,
        contaminant: Contaminant,
    },

    /// The agent-wide worst state changed (for safety gating).
    #[serde(rename = "agent.worst_state")]
    WorstState { state: ClarityState },
}

impl ClarityEvent {
    /// Decode an event from a JSON text frame.
    pub fn from_json(text: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(text)
    }

    /// Encode the event to a JSON text frame.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// True when this event represents a worsening transition (`to < from`).
    pub fn is_worsening(&self) -> bool {
        matches!(self, ClarityEvent::StateChange { from, to, .. } if to < from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_change_tag_and_worsening() {
        let ev = ClarityEvent::StateChange {
            sensor_id: "front-cam-main".into(),
            from: ClarityState::Clear,
            to: ClarityState::Degraded,
            pci: PciIndex::new(73),
            dwell_seconds: 1.0,
        };
        let json = ev.to_json().unwrap();
        assert!(json.contains("\"sensor.state_change\""));
        assert!(ev.is_worsening());

        let back = ClarityEvent::from_json(&json).unwrap();
        assert!(back.is_worsening());
    }
}
