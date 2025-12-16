//! HL7 FHIR (Fast Healthcare Interoperability Resources) exporter

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// FHIR Coding element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCoding {
    pub system: String,
    pub code: String,
    pub display: String,
}

impl FhirCoding {
    pub fn new(system: &str, code: &str, display: &str) -> Self {
        Self {
            system: system.to_string(),
            code: code.to_string(),
            display: display.to_string(),
        }
    }

    /// Create LOINC coding
    pub fn loinc(code: &str, display: &str) -> Self {
        Self::new("http://loinc.org", code, display)
    }

    /// Create SNOMED CT coding
    pub fn snomed(code: &str, display: &str) -> Self {
        Self::new("http://snomed.info/sct", code, display)
    }

    /// Create IEEE 11073 coding
    pub fn ieee_11073(code: &str, display: &str) -> Self {
        Self::new("urn:iso:std:iso:11073:10101", code, display)
    }
}

/// FHIR CodeableConcept element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCodeableConcept {
    pub coding: Vec<FhirCoding>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

impl FhirCodeableConcept {
    pub fn new(coding: FhirCoding) -> Self {
        Self {
            coding: vec![coding],
            text: None,
        }
    }

    pub fn with_text(mut self, text: &str) -> Self {
        self.text = Some(text.to_string());
        self
    }
}

/// FHIR Reference element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirReference {
    pub reference: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<String>,
}

impl FhirReference {
    pub fn new(reference: &str) -> Self {
        Self {
            reference: reference.to_string(),
            display: None,
        }
    }

    pub fn patient(id: &str) -> Self {
        Self::new(&format!("Patient/{}", id))
    }

    pub fn device(id: &str) -> Self {
        Self::new(&format!("Device/{}", id))
    }

    pub fn practitioner(id: &str) -> Self {
        Self::new(&format!("Practitioner/{}", id))
    }
}

/// FHIR Quantity element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirQuantity {
    pub value: f64,
    pub unit: String,
    pub system: String,
    pub code: String,
}

impl FhirQuantity {
    pub fn new(value: f64, unit: &str, code: &str) -> Self {
        Self {
            value,
            unit: unit.to_string(),
            system: "http://unitsofmeasure.org".to_string(),
            code: code.to_string(),
        }
    }

    pub fn degrees(value: f64) -> Self {
        Self::new(value, "degrees", "deg")
    }

    pub fn millimeters(value: f64) -> Self {
        Self::new(value, "mm", "mm")
    }

    pub fn newtons(value: f64) -> Self {
        Self::new(value, "N", "N")
    }

    pub fn millivolts(value: f64) -> Self {
        Self::new(value, "mV", "mV")
    }

    pub fn percentage(value: f64) -> Self {
        Self::new(value, "%", "%")
    }

    pub fn bpm(value: f64) -> Self {
        Self::new(value, "beats/min", "/min")
    }

    pub fn celsius(value: f64) -> Self {
        Self::new(value, "Cel", "Cel")
    }
}

/// FHIR Observation resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirObservation {
    pub resource_type: String,
    pub id: String,
    pub status: String,
    pub category: Vec<FhirCodeableConcept>,
    pub code: FhirCodeableConcept,
    pub subject: FhirReference,
    pub effective_date_time: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value_quantity: Option<FhirQuantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device: Option<FhirReference>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub performer: Option<Vec<FhirReference>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<Vec<FhirAnnotation>>,
}

impl FhirObservation {
    pub fn new(id: &str, code: FhirCodeableConcept, subject: FhirReference) -> Self {
        Self {
            resource_type: "Observation".to_string(),
            id: id.to_string(),
            status: "final".to_string(),
            category: Vec::new(),
            code,
            subject,
            effective_date_time: Utc::now().to_rfc3339(),
            value_quantity: None,
            device: None,
            performer: None,
            note: None,
        }
    }

    pub fn with_category(mut self, category: FhirCodeableConcept) -> Self {
        self.category.push(category);
        self
    }

    pub fn with_value(mut self, quantity: FhirQuantity) -> Self {
        self.value_quantity = Some(quantity);
        self
    }

    pub fn with_device(mut self, device: FhirReference) -> Self {
        self.device = Some(device);
        self
    }

    pub fn with_timestamp(mut self, timestamp: DateTime<Utc>) -> Self {
        self.effective_date_time = timestamp.to_rfc3339();
        self
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// FHIR Annotation element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirAnnotation {
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time: Option<String>,
}

/// FHIR DeviceMetric resource
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirDeviceMetric {
    pub resource_type: String,
    pub id: String,
    #[serde(rename = "type")]
    pub metric_type: FhirCodeableConcept,
    pub unit: FhirCodeableConcept,
    pub source: FhirReference,
    pub operational_status: String,
    pub category: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub measurement_period: Option<FhirTiming>,
}

impl FhirDeviceMetric {
    pub fn new(id: &str, metric_type: FhirCodeableConcept, source: FhirReference) -> Self {
        Self {
            resource_type: "DeviceMetric".to_string(),
            id: id.to_string(),
            metric_type,
            unit: FhirCodeableConcept::new(FhirCoding::new(
                "http://unitsofmeasure.org",
                "1",
                "unity",
            )),
            source,
            operational_status: "on".to_string(),
            category: "measurement".to_string(),
            measurement_period: None,
        }
    }

    pub fn with_unit(mut self, unit: FhirCodeableConcept) -> Self {
        self.unit = unit;
        self
    }

    pub fn with_measurement_period(mut self, frequency: u32) -> Self {
        self.measurement_period = Some(FhirTiming {
            repeat: FhirTimingRepeat {
                frequency,
                period: 1,
                period_unit: "s".to_string(),
            },
        });
        self
    }

    /// Convert to JSON
    pub fn to_json(&self) -> RobotResult<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// FHIR Timing element
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirTiming {
    pub repeat: FhirTimingRepeat,
}

/// FHIR Timing Repeat element
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FhirTimingRepeat {
    pub frequency: u32,
    pub period: u32,
    pub period_unit: String,
}

/// FHIR exporter for medical system integration
pub struct FhirExporter {
    base: BaseAdapter,
    server_url: Option<String>,
    patient_id: Option<String>,
}

impl Default for FhirExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl FhirExporter {
    /// Create a new FHIR exporter
    pub fn new() -> Self {
        Self {
            base: BaseAdapter::new("fhir", OutputType::Medical),
            server_url: None,
            patient_id: None,
        }
    }

    /// Set FHIR server URL
    pub fn with_server(mut self, url: &str) -> Self {
        self.server_url = Some(url.to_string());
        self
    }

    /// Set default patient ID
    pub fn with_patient(mut self, patient_id: &str) -> Self {
        self.patient_id = Some(patient_id.to_string());
        self
    }

    /// Create ROM (Range of Motion) observation
    pub fn create_rom_observation(
        &self,
        id: &str,
        patient_id: &str,
        device_id: &str,
        rom_value: f64,
        joint_name: &str,
    ) -> FhirObservation {
        let code = FhirCodeableConcept::new(FhirCoding::loinc("89255-4", "Range of motion"))
            .with_text(&format!("{} ROM", joint_name));

        let category = FhirCodeableConcept::new(FhirCoding::new(
            "http://terminology.hl7.org/CodeSystem/observation-category",
            "therapy",
            "Therapy",
        ));

        FhirObservation::new(id, code, FhirReference::patient(patient_id))
            .with_category(category)
            .with_value(FhirQuantity::degrees(rom_value))
            .with_device(FhirReference::device(device_id))
    }

    /// Create force measurement observation
    pub fn create_force_observation(
        &self,
        id: &str,
        patient_id: &str,
        device_id: &str,
        force_value: f64,
    ) -> FhirObservation {
        let code = FhirCodeableConcept::new(FhirCoding::loinc("89258-8", "Grip strength"))
            .with_text("Force measurement");

        FhirObservation::new(id, code, FhirReference::patient(patient_id))
            .with_value(FhirQuantity::newtons(force_value))
            .with_device(FhirReference::device(device_id))
    }

    /// Create EMG signal metric
    pub fn create_emg_metric(
        &self,
        id: &str,
        device_id: &str,
        sample_rate: u32,
    ) -> FhirDeviceMetric {
        let metric_type = FhirCodeableConcept::new(FhirCoding::ieee_11073("150456", "EMG signal"));
        let unit = FhirCodeableConcept::new(FhirCoding::new(
            "http://unitsofmeasure.org",
            "mV",
            "millivolt",
        ));

        FhirDeviceMetric::new(id, metric_type, FhirReference::device(device_id))
            .with_unit(unit)
            .with_measurement_period(sample_rate)
    }

    /// Create vital signs observation (heart rate)
    pub fn create_heart_rate_observation(
        &self,
        id: &str,
        patient_id: &str,
        device_id: &str,
        heart_rate: f64,
    ) -> FhirObservation {
        let code = FhirCodeableConcept::new(FhirCoding::loinc("8867-4", "Heart rate"))
            .with_text("Heart rate");

        let category = FhirCodeableConcept::new(FhirCoding::new(
            "http://terminology.hl7.org/CodeSystem/observation-category",
            "vital-signs",
            "Vital Signs",
        ));

        FhirObservation::new(id, code, FhirReference::patient(patient_id))
            .with_category(category)
            .with_value(FhirQuantity::bpm(heart_rate))
            .with_device(FhirReference::device(device_id))
    }

    /// Create temperature observation
    pub fn create_temperature_observation(
        &self,
        id: &str,
        patient_id: &str,
        device_id: &str,
        temperature: f64,
    ) -> FhirObservation {
        let code = FhirCodeableConcept::new(FhirCoding::loinc("8310-5", "Body temperature"))
            .with_text("Body temperature");

        let category = FhirCodeableConcept::new(FhirCoding::new(
            "http://terminology.hl7.org/CodeSystem/observation-category",
            "vital-signs",
            "Vital Signs",
        ));

        FhirObservation::new(id, code, FhirReference::patient(patient_id))
            .with_category(category)
            .with_value(FhirQuantity::celsius(temperature))
            .with_device(FhirReference::device(device_id))
    }

    /// Convert robot data to FHIR observations
    pub fn robot_to_fhir(&self, data: &OutputData) -> RobotResult<Vec<serde_json::Value>> {
        let patient_id = self.patient_id.as_deref().unwrap_or("unknown");
        let device_id = &data.device_id;
        let mut observations = Vec::new();

        // Extract ROM data
        if let Some(rom) = data.data.get("rom_deg").and_then(|v| v.as_f64()) {
            let joint = data.data.get("joint")
                .and_then(|j| j.as_str())
                .unwrap_or("unknown");
            let obs = self.create_rom_observation(
                &uuid::Uuid::new_v4().to_string(),
                patient_id,
                device_id,
                rom,
                joint,
            );
            observations.push(serde_json::to_value(&obs)?);
        }

        // Extract vital signs
        if let Some(heart_rate) = data.data.get("heart_rate_bpm").and_then(|v| v.as_f64()) {
            let obs = self.create_heart_rate_observation(
                &uuid::Uuid::new_v4().to_string(),
                patient_id,
                device_id,
                heart_rate,
            );
            observations.push(serde_json::to_value(&obs)?);
        }

        if let Some(temp) = data.data.get("body_temp_c").and_then(|v| v.as_f64()) {
            let obs = self.create_temperature_observation(
                &uuid::Uuid::new_v4().to_string(),
                patient_id,
                device_id,
                temp,
            );
            observations.push(serde_json::to_value(&obs)?);
        }

        // Extract force data
        if let Some(force) = data.data.get("force_n").and_then(|v| v.as_f64()) {
            let obs = self.create_force_observation(
                &uuid::Uuid::new_v4().to_string(),
                patient_id,
                device_id,
                force,
            );
            observations.push(serde_json::to_value(&obs)?);
        }

        Ok(observations)
    }
}

impl OutputAdapter for FhirExporter {
    fn output_type(&self) -> OutputType {
        OutputType::Medical
    }

    fn name(&self) -> &str {
        &self.base.name
    }

    fn initialize(&mut self, config: &OutputConfig) -> RobotResult<()> {
        if let Some(url) = config.get_string_option("server_url") {
            self.server_url = Some(url.to_string());
        }
        if let Some(patient) = config.get_string_option("patient_id") {
            self.patient_id = Some(patient.to_string());
        }
        self.base.set_config(config.clone());
        Ok(())
    }

    fn output(&self, data: &OutputData) -> RobotResult<OutputResult> {
        let observations = self.robot_to_fhir(data)?;

        Ok(OutputResult::success(&format!(
            "Generated {} FHIR observations",
            observations.len()
        ))
        .with_metadata(serde_json::json!({
            "format": "fhir_r4",
            "observation_count": observations.len(),
            "observations": observations
        })))
    }

    fn is_available(&self) -> bool {
        self.base.available
    }

    fn dispose(&mut self) -> RobotResult<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fhir_coding() {
        let loinc = FhirCoding::loinc("89255-4", "Range of motion");
        assert_eq!(loinc.system, "http://loinc.org");
        assert_eq!(loinc.code, "89255-4");
    }

    #[test]
    fn test_fhir_quantity() {
        let degrees = FhirQuantity::degrees(45.5);
        assert_eq!(degrees.value, 45.5);
        assert_eq!(degrees.unit, "degrees");
        assert_eq!(degrees.code, "deg");
    }

    #[test]
    fn test_create_rom_observation() {
        let exporter = FhirExporter::new();
        let obs = exporter.create_rom_observation(
            "obs-001",
            "patient-001",
            "device-001",
            85.5,
            "knee",
        );

        assert_eq!(obs.resource_type, "Observation");
        assert_eq!(obs.status, "final");
        assert!(obs.value_quantity.is_some());
        assert_eq!(obs.value_quantity.as_ref().unwrap().value, 85.5);
    }

    #[test]
    fn test_create_emg_metric() {
        let exporter = FhirExporter::new();
        let metric = exporter.create_emg_metric("metric-001", "device-001", 1000);

        assert_eq!(metric.resource_type, "DeviceMetric");
        assert_eq!(metric.operational_status, "on");
        assert!(metric.measurement_period.is_some());
    }

    #[test]
    fn test_fhir_exporter_output() {
        let exporter = FhirExporter::new().with_patient("patient-test");

        let data = OutputData::new("rehab-001", "rehabilitation")
            .with_data(serde_json::json!({
                "rom_deg": 85.5,
                "joint": "knee",
                "heart_rate_bpm": 72.0,
                "body_temp_c": 36.5
            }));

        let result = exporter.output(&data).unwrap();
        assert!(result.success);

        let metadata = result.metadata.unwrap();
        assert_eq!(metadata["observation_count"], 3);
    }

    #[test]
    fn test_observation_to_json() {
        let exporter = FhirExporter::new();
        let obs = exporter.create_heart_rate_observation(
            "obs-001",
            "patient-001",
            "care-robot-001",
            72.0,
        );

        let json = obs.to_json().unwrap();
        assert!(json.contains("\"resourceType\": \"Observation\""));
        assert!(json.contains("8867-4")); // LOINC code for heart rate
    }
}
