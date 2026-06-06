//! REST reporting / query client contract — Phase 2 §3, §8.
//!
//! The crate keeps the trait transport-agnostic: an HTTP backend (e.g. `reqwest`)
//! or the in-crate simulator may implement it. Request/response envelopes here
//! (de)serialize exactly to the Phase 2 wire format.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::error::Result;
use crate::types::{Sensor, SensorClarityReport};

/// Reporting + query client — Phase 2 §3 API surface.
///
/// - `submit_report` → `POST /reports` (reporting side, scope `clarity:report`).
/// - `get_agent_clarity` / `get_sensor_clarity` → `GET ...` (query side, scope
///   `clarity:read`).
pub trait ClarityClient {
    /// `POST /reports` → `202 Accepted`. Submit a sensor clarity report.
    fn submit_report(&self, report: &SensorClarityReport) -> Result<ReportAccepted>;

    /// `GET /agents/{agentId}/clarity` → the agent's latest full report.
    fn get_agent_clarity(&self, agent_id: &str) -> Result<SensorClarityReport>;

    /// `GET /agents/{agentId}/sensors/{sensorId}` → a single sensor snapshot.
    fn get_sensor_clarity(&self, agent_id: &str, sensor_id: &str) -> Result<Sensor>;
}

/// `202 Accepted` body for `POST /reports` (Phase 2 §8.1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportAccepted {
    #[serde(rename = "messageId")]
    pub message_id: String,
    #[serde(rename = "receivedAt")]
    pub received_at: DateTime<Utc>,
}

/// Freshness / staleness hints from the query response headers (Phase 2 §8.2).
#[derive(Debug, Clone, Default)]
pub struct ClarityFreshness {
    /// Seconds elapsed since the latest report was received (`Age`).
    pub age_seconds: Option<u64>,
    /// The original report's `header.timestamp` (`X-Report-Timestamp`).
    pub report_timestamp: Option<DateTime<Utc>>,
    /// Whether the report exceeds the freshness threshold (`X-Report-Stale`).
    pub stale: bool,
}

/// A query result paired with its freshness hints.
#[derive(Debug, Clone)]
pub struct ClarityQueryResult {
    pub report: SensorClarityReport,
    pub freshness: ClarityFreshness,
}

/// Serialize a report to its canonical JSON wire form (Phase 2 §3 schema-faithful).
pub fn encode_report(report: &SensorClarityReport) -> Result<String> {
    Ok(serde_json::to_string(report)?)
}

/// Deserialize a report from its JSON wire form.
pub fn decode_report(json: &str) -> Result<SensorClarityReport> {
    Ok(serde_json::from_str(json)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{ReportBuilder, SensorBuilder};
    use crate::types::{AgentType, ClarityAxes, SensorClass};

    #[test]
    fn report_round_trips_through_json() {
        let cam = SensorBuilder::from_axes(
            "front-cam-main",
            SensorClass::RgbCamera,
            ClarityAxes::new(0.18, 0.22, 0.40),
        )
        .dwell_seconds(47.0)
        .confidence(0.92)
        .build()
        .unwrap();

        let report = ReportBuilder::new("veh-seoul-0421", AgentType::Vehicle)
            .add_sensor(cam)
            .build()
            .unwrap();

        let json = encode_report(&report).unwrap();
        let back = decode_report(&json).unwrap();
        assert_eq!(back.sensors.len(), 1);
        assert_eq!(back.sensors[0].sensor_id, "front-cam-main");
        // snake_case wire value present for the enum.
        assert!(json.contains("\"rgb_camera\""));
        // camelCase field names present.
        assert!(json.contains("\"sensorId\""));
    }
}
