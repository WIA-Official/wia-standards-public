//! Type definitions for Chronic Pain Management

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PainProfile {
    pub id: Uuid,
    pub patient_id: Uuid,
    pub pain_type: PainType,
    pub severity: f64,
    pub location: String,
    pub duration_months: u32,
    pub assessment_date: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum PainType {
    #[serde(rename = "neuropathic")]
    Neuropathic,
    #[serde(rename = "nociceptive")]
    Nociceptive,
    #[serde(rename = "mixed")]
    Mixed,
    #[serde(rename = "psychogenic")]
    Psychogenic,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TreatmentPlan {
    pub id: Uuid,
    pub medications: Vec<Medication>,
    pub therapies: Vec<Therapy>,
    pub lifestyle_interventions: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Medication {
    pub name: String,
    pub dosage: String,
    pub frequency: String,
    pub effectiveness: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Therapy {
    pub therapy_type: TherapyType,
    pub frequency: String,
    pub duration_weeks: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum TherapyType {
    #[serde(rename = "physical")]
    Physical,
    #[serde(rename = "cognitive_behavioral")]
    CognitiveBehavioral,
    #[serde(rename = "acupuncture")]
    Acupuncture,
    #[serde(rename = "mindfulness")]
    Mindfulness,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PainLog {
    pub id: Uuid,
    pub patient_id: Uuid,
    pub pain_level: f64,
    pub timestamp: DateTime<Utc>,
    pub triggers: Vec<String>,
    pub relief_methods: Vec<String>,
}
