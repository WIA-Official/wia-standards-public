//! FHIR Integration Adapter
//!
//! Healthcare interoperability via HL7 FHIR R4

use async_trait::async_trait;
use chrono::Utc;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use super::adapter::*;
use crate::error::{HealthError, Result};
use crate::types::{
    BiologicalSex, HealthProfile, Measurement, MeasurementFlag, Subject,
};

/// FHIR resource types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub enum FhirResourceType {
    Patient,
    Observation,
    Condition,
    MedicationRequest,
    Procedure,
    Bundle,
}

/// FHIR Bundle
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirBundle {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    #[serde(rename = "type")]
    pub bundle_type: String,
    pub entry: Vec<FhirBundleEntry>,
}

impl Default for FhirBundle {
    fn default() -> Self {
        Self {
            resource_type: "Bundle".to_string(),
            bundle_type: "collection".to_string(),
            entry: Vec::new(),
        }
    }
}

/// FHIR Bundle entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirBundleEntry {
    #[serde(rename = "fullUrl", skip_serializing_if = "Option::is_none")]
    pub full_url: Option<String>,
    pub resource: serde_json::Value,
}

/// FHIR Patient resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirPatient {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub identifier: Option<Vec<FhirIdentifier>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<Vec<FhirHumanName>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gender: Option<String>,
    #[serde(rename = "birthDate", skip_serializing_if = "Option::is_none")]
    pub birth_date: Option<String>,
}

/// FHIR Identifier
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirIdentifier {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system: Option<String>,
    pub value: String,
}

/// FHIR HumanName
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirHumanName {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub given: Option<Vec<String>>,
}

/// FHIR Observation resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirObservation {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<Vec<FhirCodeableConcept>>,
    pub code: FhirCodeableConcept,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subject: Option<FhirReference>,
    #[serde(rename = "effectiveDateTime", skip_serializing_if = "Option::is_none")]
    pub effective_date_time: Option<String>,
    #[serde(rename = "valueQuantity", skip_serializing_if = "Option::is_none")]
    pub value_quantity: Option<FhirQuantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub interpretation: Option<Vec<FhirCodeableConcept>>,
}

/// FHIR CodeableConcept
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCodeableConcept {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coding: Option<Vec<FhirCoding>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

/// FHIR Coding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirCoding {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<String>,
}

/// FHIR Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirReference {
    pub reference: String,
}

/// FHIR Quantity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirQuantity {
    pub value: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<String>,
}

/// LOINC code mappings for common biomarkers
pub struct LoincCodes;

impl LoincCodes {
    pub const HEART_RATE: &'static str = "8867-4";
    pub const BLOOD_PRESSURE_SYSTOLIC: &'static str = "8480-6";
    pub const BLOOD_PRESSURE_DIASTOLIC: &'static str = "8462-4";
    pub const BODY_TEMPERATURE: &'static str = "8310-5";
    pub const RESPIRATORY_RATE: &'static str = "9279-1";
    pub const OXYGEN_SATURATION: &'static str = "2708-6";
    pub const BLOOD_GLUCOSE: &'static str = "2339-0";
    pub const HBA1C: &'static str = "4548-4";
    pub const CRP: &'static str = "1988-5";
    pub const IL6: &'static str = "26881-3";
    pub const TNF_ALPHA: &'static str = "3167-2";
    pub const TELOMERE_LENGTH: &'static str = "94500-0";
}

/// FHIR R4 integration adapter
pub struct FhirAdapter {
    name: String,
    config: AdapterConfig,
    initialized: bool,
}

impl FhirAdapter {
    /// Create new FHIR adapter
    pub fn new() -> Self {
        Self {
            name: "FHIR R4 Adapter".to_string(),
            config: AdapterConfig::default(),
            initialized: false,
        }
    }

    /// Create with base URL
    pub fn with_url(url: &str) -> Self {
        let mut adapter = Self::new();
        adapter.config.base_url = Some(url.to_string());
        adapter
    }

    /// Set authentication
    pub fn with_auth(mut self, token: &str) -> Self {
        self.config.auth_token = Some(token.to_string());
        self
    }

    /// Convert WIA Subject to FHIR Patient
    pub fn subject_to_patient(&self, subject: &Subject) -> FhirPatient {
        let gender = subject.biological_sex.as_ref().map(|sex| match sex {
            BiologicalSex::Male => "male".to_string(),
            BiologicalSex::Female => "female".to_string(),
            BiologicalSex::Other => "other".to_string(),
            BiologicalSex::Unknown => "unknown".to_string(),
        });

        let birth_date = subject.birth_year.map(|year| format!("{}", year));

        FhirPatient {
            resource_type: "Patient".to_string(),
            id: subject.id.to_string(),
            identifier: subject.anonymized_id.as_ref().map(|id| {
                vec![FhirIdentifier {
                    system: Some("http://wia.live/fhir/identifier/anonymized".to_string()),
                    value: id.clone(),
                }]
            }),
            name: None,
            gender,
            birth_date,
        }
    }

    /// Convert FHIR Patient to WIA Subject
    pub fn patient_to_subject(&self, patient: &FhirPatient) -> Result<Subject> {
        let id = Uuid::parse_str(&patient.id)
            .unwrap_or_else(|_| Uuid::new_v4());

        let biological_sex = patient.gender.as_ref().map(|g| match g.as_str() {
            "male" => BiologicalSex::Male,
            "female" => BiologicalSex::Female,
            "other" => BiologicalSex::Other,
            _ => BiologicalSex::Unknown,
        });

        let birth_year = patient.birth_date.as_ref().and_then(|d| {
            d.split('-').next().and_then(|y| y.parse().ok())
        });

        let anonymized_id = patient.identifier.as_ref().and_then(|ids| {
            ids.first().map(|id| id.value.clone())
        });

        Ok(Subject {
            id,
            anonymized_id,
            birth_year,
            biological_sex,
            ethnicity: None,
            consent: None,
        })
    }

    /// Convert WIA Measurement to FHIR Observation
    pub fn measurement_to_observation(
        &self,
        measurement: &Measurement,
        marker_name: &str,
        loinc_code: &str,
        subject_ref: &str,
    ) -> FhirObservation {
        let interpretation = measurement.flags.as_ref().map(|flags| {
            flags
                .iter()
                .map(|flag| {
                    let (code, display) = match flag {
                        MeasurementFlag::Normal => ("N", "Normal"),
                        MeasurementFlag::Low => ("L", "Low"),
                        MeasurementFlag::High => ("H", "High"),
                        MeasurementFlag::Critical => ("AA", "Critical abnormal"),
                        MeasurementFlag::Abnormal => ("A", "Abnormal"),
                    };
                    FhirCodeableConcept {
                        coding: Some(vec![FhirCoding {
                            system: Some(
                                "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation"
                                    .to_string(),
                            ),
                            code: Some(code.to_string()),
                            display: Some(display.to_string()),
                        }]),
                        text: None,
                    }
                })
                .collect()
        });

        FhirObservation {
            resource_type: "Observation".to_string(),
            id: Uuid::new_v4().to_string(),
            status: "final".to_string(),
            category: Some(vec![FhirCodeableConcept {
                coding: Some(vec![FhirCoding {
                    system: Some(
                        "http://terminology.hl7.org/CodeSystem/observation-category".to_string(),
                    ),
                    code: Some("vital-signs".to_string()),
                    display: Some("Vital Signs".to_string()),
                }]),
                text: None,
            }]),
            code: FhirCodeableConcept {
                coding: Some(vec![FhirCoding {
                    system: Some("http://loinc.org".to_string()),
                    code: Some(loinc_code.to_string()),
                    display: Some(marker_name.to_string()),
                }]),
                text: Some(marker_name.to_string()),
            },
            subject: Some(FhirReference {
                reference: format!("Patient/{}", subject_ref),
            }),
            effective_date_time: measurement.timestamp.map(|t| t.to_rfc3339()),
            value_quantity: Some(FhirQuantity {
                value: measurement.value,
                unit: Some(measurement.unit.clone()),
                system: Some("http://unitsofmeasure.org".to_string()),
                code: None,
            }),
            interpretation,
        }
    }

    /// Convert HealthProfile to FHIR Bundle
    pub fn profile_to_bundle(&self, profile: &HealthProfile) -> FhirBundle {
        let mut bundle = FhirBundle::default();

        // Add Patient resource
        let patient = self.subject_to_patient(&profile.subject);
        bundle.entry.push(FhirBundleEntry {
            full_url: Some(format!("urn:uuid:{}", patient.id)),
            resource: serde_json::to_value(&patient).unwrap(),
        });

        let subject_id = profile.subject.id.to_string();

        // Add biomarker observations
        if let Some(ref biomarkers) = profile.biomarkers {
            // Inflammatory markers
            if let Some(ref inflammatory) = biomarkers.inflammatory_markers {
                if let Some(ref crp) = inflammatory.crp {
                    let obs = self.measurement_to_observation(
                        crp,
                        "C-Reactive Protein",
                        LoincCodes::CRP,
                        &subject_id,
                    );
                    bundle.entry.push(FhirBundleEntry {
                        full_url: Some(format!("urn:uuid:{}", obs.id)),
                        resource: serde_json::to_value(&obs).unwrap(),
                    });
                }
            }

            // Metabolic markers
            if let Some(ref metabolic) = biomarkers.metabolic_markers {
                if let Some(ref glucose) = metabolic.glucose {
                    let obs = self.measurement_to_observation(
                        glucose,
                        "Blood Glucose",
                        LoincCodes::BLOOD_GLUCOSE,
                        &subject_id,
                    );
                    bundle.entry.push(FhirBundleEntry {
                        full_url: Some(format!("urn:uuid:{}", obs.id)),
                        resource: serde_json::to_value(&obs).unwrap(),
                    });
                }
                if let Some(ref hba1c) = metabolic.hba1c {
                    let obs = self.measurement_to_observation(
                        hba1c,
                        "Hemoglobin A1c",
                        LoincCodes::HBA1C,
                        &subject_id,
                    );
                    bundle.entry.push(FhirBundleEntry {
                        full_url: Some(format!("urn:uuid:{}", obs.id)),
                        resource: serde_json::to_value(&obs).unwrap(),
                    });
                }
            }
        }

        bundle
    }
}

impl Default for FhirAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IntegrationAdapter for FhirAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Fhir
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn is_available(&self) -> bool {
        self.initialized && self.config.base_url.is_some()
    }

    async fn initialize(&mut self, config: AdapterConfig) -> Result<()> {
        self.config = config;
        self.initialized = true;
        Ok(())
    }

    async fn shutdown(&mut self) -> Result<()> {
        self.initialized = false;
        Ok(())
    }

    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            can_import: true,
            can_export: true,
            can_stream: false,
            supports_auth: true,
            data_types: vec![
                "Patient".to_string(),
                "Observation".to_string(),
                "Condition".to_string(),
                "MedicationRequest".to_string(),
            ],
        }
    }
}

#[async_trait]
impl ExportAdapter for FhirAdapter {
    async fn export(&self, profile: &HealthProfile, options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("FHIR adapter not initialized"));
        }

        let bundle = self.profile_to_bundle(profile);
        let records_exported = bundle.entry.len() as u64;

        // In a real implementation, this would POST to the FHIR server
        // For now, we just return success

        Ok(ExportResult {
            success: true,
            records_exported,
            destination: self
                .config
                .base_url
                .clone()
                .unwrap_or_else(|| "local".to_string()),
            resource_ids: bundle
                .entry
                .iter()
                .filter_map(|e| e.full_url.clone())
                .collect(),
            exported_at: Utc::now(),
        })
    }

    async fn export_data(&self, data: ExportData, _options: ExportOptions) -> Result<ExportResult> {
        if !self.initialized {
            return Err(HealthError::adapter("FHIR adapter not initialized"));
        }

        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: self
                .config
                .base_url
                .clone()
                .unwrap_or_else(|| "local".to_string()),
            resource_ids: vec![data.data_type],
            exported_at: Utc::now(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;

    #[test]
    fn test_subject_to_patient() {
        let adapter = FhirAdapter::new();

        let subject = Subject {
            id: Uuid::new_v4(),
            anonymized_id: Some("anon-001".to_string()),
            birth_year: Some(1985),
            biological_sex: Some(BiologicalSex::Male),
            ethnicity: None,
            consent: None,
        };

        let patient = adapter.subject_to_patient(&subject);

        assert_eq!(patient.resource_type, "Patient");
        assert_eq!(patient.gender, Some("male".to_string()));
        assert_eq!(patient.birth_date, Some("1985".to_string()));
    }

    #[test]
    fn test_measurement_to_observation() {
        let adapter = FhirAdapter::new();

        let measurement = Measurement {
            value: 120.0,
            unit: "mg/dL".to_string(),
            reference_range: None,
            timestamp: Some(Utc::now()),
            method: None,
            laboratory: None,
            flags: Some(vec![MeasurementFlag::High]),
            notes: None,
        };

        let obs = adapter.measurement_to_observation(
            &measurement,
            "Blood Glucose",
            LoincCodes::BLOOD_GLUCOSE,
            "patient-001",
        );

        assert_eq!(obs.resource_type, "Observation");
        assert_eq!(obs.status, "final");
        assert!(obs.value_quantity.is_some());
        assert_eq!(obs.value_quantity.unwrap().value, 120.0);
    }
}
