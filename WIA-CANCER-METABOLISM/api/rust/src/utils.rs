//! Utility functions for cancer metabolism research

use crate::types::*;

pub fn calculate_metabolic_dysregulation_score(profile: &MetabolicProfile) -> f64 {
    let abnormal_markers = profile.metabolic_markers.iter()
        .filter(|m| m.significance != MarkerSignificance::Normal)
        .count();

    (abnormal_markers as f64 / profile.metabolic_markers.len() as f64) * 100.0
}

pub fn identify_therapeutic_targets(profile: &MetabolicProfile) -> Vec<String> {
    let mut targets = Vec::new();

    for marker in &profile.metabolic_markers {
        if marker.significance == MarkerSignificance::Elevated {
            targets.push(format!("Target: {}", marker.name));
        }
    }

    targets
}

pub fn generate_research_report(profile: &MetabolicProfile) -> String {
    format!(
        "Cancer Metabolism Analysis Report\nCancer Type: {:?}\nMarkers Analyzed: {}\nDate: {}",
        profile.cancer_type,
        profile.metabolic_markers.len(),
        profile.analysis_date
    )
}
