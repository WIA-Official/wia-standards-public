//! Utility functions for autoimmune disease management

use crate::types::*;

pub fn calculate_disease_severity_index(patient: &Patient) -> f64 {
    patient.symptom_severity
}

pub fn recommend_treatment_adjustments(patient: &Patient) -> Vec<String> {
    let mut recommendations = Vec::new();

    if patient.symptom_severity > 7.0 {
        recommendations.push("Consider increasing medication dosage".to_string());
        recommendations.push("Schedule urgent specialist consultation".to_string());
    }

    recommendations
}

pub fn analyze_symptom_patterns(logs: &[SymptomLog]) -> String {
    format!("Analyzed {} symptom logs", logs.len())
}
