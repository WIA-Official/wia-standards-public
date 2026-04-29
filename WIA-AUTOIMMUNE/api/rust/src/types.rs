//! Type definitions for Autoimmune Disease Management

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Patient {
    pub id: Uuid,
    pub diagnosis: AutoimmuneDiseaseType,
    pub symptom_severity: f64,
    pub treatment_plan: TreatmentPlan,
    pub last_assessment: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum AutoimmuneDiseaseType {
    #[serde(rename = "lupus")]
    Lupus,
    #[serde(rename = "rheumatoid_arthritis")]
    RheumatoidArthritis,
    #[serde(rename = "multiple_sclerosis")]
    MultipleSclerosis,
    #[serde(rename = "type1_diabetes")]
    Type1Diabetes,
    #[serde(rename = "celiac")]
    Celiac,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TreatmentPlan {
    pub id: Uuid,
    pub medications: Vec<Medication>,
    pub lifestyle_modifications: Vec<String>,
    pub monitoring_schedule: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Medication {
    pub name: String,
    pub dosage: String,
    pub frequency: String,
    pub side_effects: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SymptomLog {
    pub id: Uuid,
    pub patient_id: Uuid,
    pub symptoms: Vec<String>,
    pub severity: f64,
    pub timestamp: DateTime<Utc>,
    pub triggers: Vec<String>,
}
