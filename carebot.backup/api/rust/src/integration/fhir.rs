//! HL7 FHIR integration for healthcare interoperability

use serde::{Deserialize, Serialize};
use crate::health::VitalSigns;

/// FHIR Bundle for batch submissions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirBundle {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    #[serde(rename = "type")]
    pub bundle_type: String,
    pub entry: Vec<BundleEntry>,
}

impl FhirBundle {
    /// Create a new batch bundle
    pub fn new_batch() -> Self {
        Self {
            resource_type: "Bundle".to_string(),
            bundle_type: "batch".to_string(),
            entry: Vec::new(),
        }
    }

    /// Add an entry to the bundle
    pub fn add_entry(&mut self, resource: FhirResource) {
        self.entry.push(BundleEntry { resource });
    }

    /// Create a vitals bundle from CareBot health data
    pub fn from_vitals(recipient_id: &str, device_id: &str, vitals: &VitalSigns) -> Self {
        let mut bundle = Self::new_batch();

        if let Some(hr) = &vitals.heart_rate {
            bundle.add_entry(FhirResource::Observation(
                FhirObservation::heart_rate(recipient_id, device_id, hr.bpm),
            ));
        }

        if let Some(bp) = &vitals.blood_pressure {
            bundle.add_entry(FhirResource::Observation(
                FhirObservation::blood_pressure(recipient_id, device_id, bp.systolic, bp.diastolic),
            ));
        }

        if let Some(spo2) = &vitals.spo2 {
            bundle.add_entry(FhirResource::Observation(
                FhirObservation::spo2(recipient_id, device_id, spo2.percentage),
            ));
        }

        if let Some(temp) = &vitals.temperature {
            bundle.add_entry(FhirResource::Observation(
                FhirObservation::temperature(recipient_id, device_id, temp.celsius),
            ));
        }

        bundle
    }
}

/// Bundle entry wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BundleEntry {
    pub resource: FhirResource,
}

/// FHIR Resource types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FhirResource {
    Observation(FhirObservation),
    MedicationAdministration(FhirMedicationAdministration),
    AdverseEvent(FhirAdverseEvent),
}

/// FHIR Observation resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirObservation {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    pub status: String,
    pub category: Vec<CodeableConcept>,
    pub code: CodeableConcept,
    pub subject: Reference,
    #[serde(rename = "effectiveDateTime")]
    pub effective_date_time: String,
    #[serde(rename = "valueQuantity", skip_serializing_if = "Option::is_none")]
    pub value_quantity: Option<Quantity>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub component: Option<Vec<ObservationComponent>>,
    pub device: Reference,
}

impl FhirObservation {
    /// Create heart rate observation
    pub fn heart_rate(patient_id: &str, device_id: &str, bpm: u16) -> Self {
        Self {
            resource_type: "Observation".to_string(),
            id: format!("carebot-hr-{}", uuid::Uuid::new_v4()),
            status: "final".to_string(),
            category: vec![CodeableConcept::vital_signs_category()],
            code: CodeableConcept::loinc("8867-4", "Heart rate"),
            subject: Reference::patient(patient_id),
            effective_date_time: chrono::Utc::now().to_rfc3339(),
            value_quantity: Some(Quantity {
                value: bpm as f64,
                unit: "beats/minute".to_string(),
                system: "http://unitsofmeasure.org".to_string(),
                code: "/min".to_string(),
            }),
            component: None,
            device: Reference::device(device_id),
        }
    }

    /// Create blood pressure observation
    pub fn blood_pressure(patient_id: &str, device_id: &str, systolic: u16, diastolic: u16) -> Self {
        Self {
            resource_type: "Observation".to_string(),
            id: format!("carebot-bp-{}", uuid::Uuid::new_v4()),
            status: "final".to_string(),
            category: vec![CodeableConcept::vital_signs_category()],
            code: CodeableConcept::loinc("85354-9", "Blood pressure panel"),
            subject: Reference::patient(patient_id),
            effective_date_time: chrono::Utc::now().to_rfc3339(),
            value_quantity: None,
            component: Some(vec![
                ObservationComponent {
                    code: CodeableConcept::loinc("8480-6", "Systolic blood pressure"),
                    value_quantity: Quantity {
                        value: systolic as f64,
                        unit: "mmHg".to_string(),
                        system: "http://unitsofmeasure.org".to_string(),
                        code: "mm[Hg]".to_string(),
                    },
                },
                ObservationComponent {
                    code: CodeableConcept::loinc("8462-4", "Diastolic blood pressure"),
                    value_quantity: Quantity {
                        value: diastolic as f64,
                        unit: "mmHg".to_string(),
                        system: "http://unitsofmeasure.org".to_string(),
                        code: "mm[Hg]".to_string(),
                    },
                },
            ]),
            device: Reference::device(device_id),
        }
    }

    /// Create SpO2 observation
    pub fn spo2(patient_id: &str, device_id: &str, percentage: u8) -> Self {
        Self {
            resource_type: "Observation".to_string(),
            id: format!("carebot-spo2-{}", uuid::Uuid::new_v4()),
            status: "final".to_string(),
            category: vec![CodeableConcept::vital_signs_category()],
            code: CodeableConcept::loinc("2708-6", "Oxygen saturation"),
            subject: Reference::patient(patient_id),
            effective_date_time: chrono::Utc::now().to_rfc3339(),
            value_quantity: Some(Quantity {
                value: percentage as f64,
                unit: "%".to_string(),
                system: "http://unitsofmeasure.org".to_string(),
                code: "%".to_string(),
            }),
            component: None,
            device: Reference::device(device_id),
        }
    }

    /// Create temperature observation
    pub fn temperature(patient_id: &str, device_id: &str, celsius: f32) -> Self {
        Self {
            resource_type: "Observation".to_string(),
            id: format!("carebot-temp-{}", uuid::Uuid::new_v4()),
            status: "final".to_string(),
            category: vec![CodeableConcept::vital_signs_category()],
            code: CodeableConcept::loinc("8310-5", "Body temperature"),
            subject: Reference::patient(patient_id),
            effective_date_time: chrono::Utc::now().to_rfc3339(),
            value_quantity: Some(Quantity {
                value: celsius as f64,
                unit: "Cel".to_string(),
                system: "http://unitsofmeasure.org".to_string(),
                code: "Cel".to_string(),
            }),
            component: None,
            device: Reference::device(device_id),
        }
    }
}

/// Observation component for multi-value observations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservationComponent {
    pub code: CodeableConcept,
    #[serde(rename = "valueQuantity")]
    pub value_quantity: Quantity,
}

/// FHIR MedicationAdministration resource
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirMedicationAdministration {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    pub status: String,
    #[serde(rename = "medicationCodeableConcept")]
    pub medication: CodeableConcept,
    pub subject: Reference,
    #[serde(rename = "effectiveDateTime")]
    pub effective_date_time: String,
    pub performer: Vec<Performer>,
    pub note: Vec<Annotation>,
}

impl FhirMedicationAdministration {
    /// Create a medication administration record
    pub fn new(
        patient_id: &str,
        device_id: &str,
        medication_name: &str,
        rxnorm_code: Option<&str>,
    ) -> Self {
        Self {
            resource_type: "MedicationAdministration".to_string(),
            id: format!("carebot-med-{}", uuid::Uuid::new_v4()),
            status: "completed".to_string(),
            medication: if let Some(code) = rxnorm_code {
                CodeableConcept::rxnorm(code, medication_name)
            } else {
                CodeableConcept::text_only(medication_name)
            },
            subject: Reference::patient(patient_id),
            effective_date_time: chrono::Utc::now().to_rfc3339(),
            performer: vec![Performer {
                actor: Reference::device(device_id),
            }],
            note: vec![Annotation {
                text: "약 복용 확인됨 (카메라 감지)".to_string(),
            }],
        }
    }
}

/// FHIR AdverseEvent resource for falls
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FhirAdverseEvent {
    #[serde(rename = "resourceType")]
    pub resource_type: String,
    pub id: String,
    pub status: String,
    pub actuality: String,
    pub category: Vec<CodeableConcept>,
    pub event: CodeableConcept,
    pub subject: Reference,
    pub date: String,
    pub location: Option<Reference>,
    pub outcome: Option<CodeableConcept>,
}

impl FhirAdverseEvent {
    /// Create a fall event
    pub fn fall(patient_id: &str, location: &str) -> Self {
        Self {
            resource_type: "AdverseEvent".to_string(),
            id: format!("carebot-fall-{}", uuid::Uuid::new_v4()),
            status: "completed".to_string(),
            actuality: "actual".to_string(),
            category: vec![CodeableConcept {
                coding: vec![Coding {
                    system: "http://terminology.hl7.org/CodeSystem/adverse-event-category".to_string(),
                    code: "product-use-error".to_string(),
                    display: "Product Use Error".to_string(),
                }],
                text: Some("낙상".to_string()),
            }],
            event: CodeableConcept {
                coding: vec![Coding {
                    system: "http://snomed.info/sct".to_string(),
                    code: "217082002".to_string(),
                    display: "Fall".to_string(),
                }],
                text: Some("낙상 감지".to_string()),
            },
            subject: Reference::patient(patient_id),
            date: chrono::Utc::now().to_rfc3339(),
            location: Some(Reference {
                reference: format!("Location/{}", location),
                display: Some(location.to_string()),
            }),
            outcome: None,
        }
    }
}

/// FHIR CodeableConcept
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CodeableConcept {
    pub coding: Vec<Coding>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

impl CodeableConcept {
    /// Create vital signs category
    pub fn vital_signs_category() -> Self {
        Self {
            coding: vec![Coding {
                system: "http://terminology.hl7.org/CodeSystem/observation-category".to_string(),
                code: "vital-signs".to_string(),
                display: "Vital Signs".to_string(),
            }],
            text: None,
        }
    }

    /// Create LOINC code
    pub fn loinc(code: &str, display: &str) -> Self {
        Self {
            coding: vec![Coding {
                system: "http://loinc.org".to_string(),
                code: code.to_string(),
                display: display.to_string(),
            }],
            text: None,
        }
    }

    /// Create RxNorm code
    pub fn rxnorm(code: &str, display: &str) -> Self {
        Self {
            coding: vec![Coding {
                system: "http://www.nlm.nih.gov/research/umls/rxnorm".to_string(),
                code: code.to_string(),
                display: display.to_string(),
            }],
            text: Some(display.to_string()),
        }
    }

    /// Create text-only concept
    pub fn text_only(text: &str) -> Self {
        Self {
            coding: Vec::new(),
            text: Some(text.to_string()),
        }
    }
}

/// FHIR Coding
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coding {
    pub system: String,
    pub code: String,
    pub display: String,
}

/// FHIR Quantity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quantity {
    pub value: f64,
    pub unit: String,
    pub system: String,
    pub code: String,
}

/// FHIR Reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Reference {
    pub reference: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<String>,
}

impl Reference {
    pub fn patient(id: &str) -> Self {
        Self {
            reference: format!("Patient/{}", id),
            display: None,
        }
    }

    pub fn device(id: &str) -> Self {
        Self {
            reference: format!("Device/{}", id),
            display: Some("WIA CareBot".to_string()),
        }
    }
}

/// FHIR Performer
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Performer {
    pub actor: Reference,
}

/// FHIR Annotation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Annotation {
    pub text: String,
}

/// FHIR client for submitting data
pub struct FhirClient {
    base_url: String,
    auth_token: Option<String>,
}

impl FhirClient {
    /// Create a new FHIR client
    pub fn new(base_url: &str) -> Self {
        Self {
            base_url: base_url.to_string(),
            auth_token: None,
        }
    }

    /// Set authentication token
    pub fn with_auth(mut self, token: &str) -> Self {
        self.auth_token = Some(token.to_string());
        self
    }

    /// Get the base URL
    pub fn base_url(&self) -> &str {
        &self.base_url
    }

    /// Build bundle endpoint URL
    pub fn bundle_url(&self) -> String {
        format!("{}/Bundle", self.base_url)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heart_rate_observation() {
        let obs = FhirObservation::heart_rate("patient-001", "carebot-001", 72);

        assert_eq!(obs.resource_type, "Observation");
        assert_eq!(obs.value_quantity.as_ref().unwrap().value, 72.0);
    }

    #[test]
    fn test_blood_pressure_observation() {
        let obs = FhirObservation::blood_pressure("patient-001", "carebot-001", 128, 82);

        assert!(obs.component.is_some());
        let components = obs.component.unwrap();
        assert_eq!(components.len(), 2);
        assert_eq!(components[0].value_quantity.value, 128.0);
        assert_eq!(components[1].value_quantity.value, 82.0);
    }

    #[test]
    fn test_fhir_bundle() {
        let mut bundle = FhirBundle::new_batch();
        bundle.add_entry(FhirResource::Observation(
            FhirObservation::heart_rate("patient-001", "carebot-001", 72),
        ));

        assert_eq!(bundle.bundle_type, "batch");
        assert_eq!(bundle.entry.len(), 1);
    }
}
