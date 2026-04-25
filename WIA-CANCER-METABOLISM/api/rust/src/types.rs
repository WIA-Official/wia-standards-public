//! Type definitions for Cancer Metabolism Research

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetabolicProfile {
    pub id: Uuid,
    pub patient_id: Uuid,
    pub cancer_type: CancerType,
    pub metabolic_markers: Vec<MetabolicMarker>,
    pub analysis_date: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum CancerType {
    #[serde(rename = "lung")]
    Lung,
    #[serde(rename = "breast")]
    Breast,
    #[serde(rename = "colon")]
    Colon,
    #[serde(rename = "pancreatic")]
    Pancreatic,
    #[serde(rename = "leukemia")]
    Leukemia,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetabolicMarker {
    pub name: String,
    pub value: f64,
    pub unit: String,
    pub normal_range_min: f64,
    pub normal_range_max: f64,
    pub significance: MarkerSignificance,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum MarkerSignificance {
    #[serde(rename = "elevated")]
    Elevated,
    #[serde(rename = "normal")]
    Normal,
    #[serde(rename = "decreased")]
    Decreased,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetabolicPathway {
    pub name: String,
    pub enzymes: Vec<String>,
    pub activity_level: f64,
    pub therapeutic_targets: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TherapeuticStrategy {
    pub id: Uuid,
    pub target_pathway: String,
    pub intervention_type: InterventionType,
    pub expected_efficacy: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum InterventionType {
    #[serde(rename = "metabolic_inhibitor")]
    MetabolicInhibitor,
    #[serde(rename = "dietary")]
    Dietary,
    #[serde(rename = "combined")]
    Combined,
}
