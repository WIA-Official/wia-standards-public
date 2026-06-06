//! Ergonomic builders for [`Sensor`] and [`SensorClarityReport`].

use chrono::{DateTime, Utc};
use uuid::Uuid;

use crate::core::compute_pci;
use crate::error::{Error, Result};
use crate::types::{
    AgentType, ClarityAxes, ClarityState, ConformanceLevel, Contaminant, Header, PciIndex,
    PciWeights, Sensor, SensorClarityReport, SensorClass,
};

/// Builder for a [`Sensor`] snapshot.
///
/// The most common path is [`SensorBuilder::from_axes`], which computes the PCI
/// from the axes + class default weights and derives the state, keeping `pci`,
/// `state`, `axes`, and `pciWeights` mutually consistent.
#[derive(Debug, Clone)]
pub struct SensorBuilder {
    sensor_id: String,
    sensor_class: SensorClass,
    pci: PciIndex,
    state: ClarityState,
    contaminants: Vec<Contaminant>,
    axes: Option<ClarityAxes>,
    pci_weights: Option<PciWeights>,
    last_cleaned_at: Option<DateTime<Utc>>,
    dwell_seconds: f64,
    confidence: f64,
}

impl SensorBuilder {
    /// Start from an explicit PCI value (state is derived from the band).
    pub fn new(sensor_id: impl Into<String>, sensor_class: SensorClass, pci: PciIndex) -> Self {
        Self {
            sensor_id: sensor_id.into(),
            sensor_class,
            pci,
            state: pci.state(),
            contaminants: Vec::new(),
            axes: None,
            pci_weights: None,
            last_cleaned_at: None,
            dwell_seconds: 0.0,
            confidence: 1.0,
        }
    }

    /// Start from damage axes: PCI is computed with the class default weights,
    /// the state is derived, and both `axes` and `pciWeights` are recorded for
    /// audit transparency.
    pub fn from_axes(
        sensor_id: impl Into<String>,
        sensor_class: SensorClass,
        axes: ClarityAxes,
    ) -> Self {
        let weights = PciWeights::default_for(sensor_class);
        let pci = compute_pci(&axes, &weights);
        Self {
            sensor_id: sensor_id.into(),
            sensor_class,
            pci,
            state: pci.state(),
            contaminants: Vec::new(),
            axes: Some(axes),
            pci_weights: Some(weights),
            last_cleaned_at: None,
            dwell_seconds: 0.0,
            confidence: 1.0,
        }
    }

    /// Add a contaminant entry.
    pub fn add_contaminant(mut self, contaminant: Contaminant) -> Self {
        self.contaminants.push(contaminant);
        self
    }

    /// Override the recorded axes.
    pub fn axes(mut self, axes: ClarityAxes) -> Self {
        self.axes = Some(axes);
        self
    }

    /// Override the recorded weight set.
    pub fn pci_weights(mut self, weights: PciWeights) -> Self {
        self.pci_weights = Some(weights);
        self
    }

    /// Set the last-cleaned timestamp.
    pub fn last_cleaned_at(mut self, ts: DateTime<Utc>) -> Self {
        self.last_cleaned_at = Some(ts);
        self
    }

    /// Set the dwell time (seconds in the current non-CLEAR state).
    pub fn dwell_seconds(mut self, secs: f64) -> Self {
        self.dwell_seconds = secs;
        self
    }

    /// Set the self-diagnosis confidence.
    pub fn confidence(mut self, confidence: f64) -> Self {
        self.confidence = confidence;
        self
    }

    /// Finish, validating the Phase 1 §6 consistency rules.
    pub fn build(self) -> Result<Sensor> {
        if !(0.0..=1.0).contains(&self.confidence) {
            return Err(Error::OutOfRange {
                field: "confidence".into(),
                min: 0.0,
                max: 1.0,
                value: self.confidence,
            });
        }
        if self.dwell_seconds < 0.0 {
            return Err(Error::validation("dwellSeconds must be >= 0"));
        }
        // Phase 1 §6.3: dwell > 0 implies a non-CLEAR state.
        if self.dwell_seconds > 0.0 && self.state == ClarityState::Clear {
            return Err(Error::validation(
                "dwellSeconds > 0 requires state != clear",
            ));
        }
        // Phase 1 §6.3: state must agree with the PCI band.
        let expected = self.pci.state();
        if expected != self.state {
            return Err(Error::StatePciConflict {
                pci: self.pci.value(),
                expected: format!("{:?}", expected),
                actual: format!("{:?}", self.state),
            });
        }
        if let Some(w) = self.pci_weights {
            if !w.is_valid() {
                return Err(Error::WeightsSumInvalid(w.sum()));
            }
        }
        Ok(Sensor {
            sensor_id: self.sensor_id,
            sensor_class: self.sensor_class,
            pci: self.pci,
            state: self.state,
            contaminants: self.contaminants,
            axes: self.axes,
            pci_weights: self.pci_weights,
            last_cleaned_at: self.last_cleaned_at,
            dwell_seconds: self.dwell_seconds,
            confidence: self.confidence,
        })
    }
}

/// Builder for a [`SensorClarityReport`].
#[derive(Debug, Clone)]
pub struct ReportBuilder {
    version: String,
    message_id: String,
    timestamp: DateTime<Utc>,
    agent_id: String,
    agent_type: AgentType,
    conformance_level: Option<ConformanceLevel>,
    sensors: Vec<Sensor>,
}

impl ReportBuilder {
    /// Start a report for an agent. A fresh `messageId` (UUID v4) and the
    /// current timestamp are filled in; `version` defaults to `"1.0.0"`.
    pub fn new(agent_id: impl Into<String>, agent_type: AgentType) -> Self {
        Self {
            version: "1.0.0".to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now(),
            agent_id: agent_id.into(),
            agent_type,
            conformance_level: None,
            sensors: Vec::new(),
        }
    }

    /// Override the payload schema version.
    pub fn version(mut self, version: impl Into<String>) -> Self {
        self.version = version.into();
        self
    }

    /// Override the message id.
    pub fn message_id(mut self, id: impl Into<String>) -> Self {
        self.message_id = id.into();
        self
    }

    /// Override the timestamp.
    pub fn timestamp(mut self, ts: DateTime<Utc>) -> Self {
        self.timestamp = ts;
        self
    }

    /// Declare the conformance level.
    pub fn conformance_level(mut self, level: ConformanceLevel) -> Self {
        self.conformance_level = Some(level);
        self
    }

    /// Append a sensor snapshot.
    pub fn add_sensor(mut self, sensor: Sensor) -> Self {
        self.sensors.push(sensor);
        self
    }

    /// Finish, enforcing the `minItems: 1` rule for `sensors`.
    pub fn build(self) -> Result<SensorClarityReport> {
        if self.sensors.is_empty() {
            return Err(Error::validation("a report requires at least one sensor"));
        }
        Ok(SensorClarityReport {
            header: Header {
                version: self.version,
                message_id: self.message_id,
                timestamp: self.timestamp,
                agent_id: self.agent_id,
                agent_type: self.agent_type,
                conformance_level: self.conformance_level,
            },
            sensors: self.sensors,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ContaminantType;

    #[test]
    fn builds_consistent_rain_camera_report() {
        let cam = SensorBuilder::from_axes(
            "front-cam-main",
            SensorClass::RgbCamera,
            ClarityAxes::new(0.18, 0.22, 0.40),
        )
        .add_contaminant(Contaminant::new(ContaminantType::RainFilm, 0.35))
        .dwell_seconds(47.0)
        .confidence(0.92)
        .build()
        .unwrap();

        assert_eq!(cam.state, ClarityState::Degraded);

        let report = ReportBuilder::new("veh-seoul-0421", AgentType::Vehicle)
            .conformance_level(ConformanceLevel::Lc)
            .add_sensor(cam)
            .build()
            .unwrap();

        assert_eq!(report.sensors.len(), 1);
    }

    #[test]
    fn rejects_dwell_on_clear() {
        let err = SensorBuilder::new("c", SensorClass::Ultrasonic, PciIndex::new(100))
            .dwell_seconds(5.0)
            .build();
        assert!(err.is_err());
    }
}
