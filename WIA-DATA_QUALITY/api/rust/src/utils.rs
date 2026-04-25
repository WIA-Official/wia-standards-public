//! Utility functions for data quality

use crate::types::*;

pub fn calculate_overall_score(
    completeness: f64,
    accuracy: f64,
    consistency: f64,
    timeliness: f64,
    validity: f64,
) -> f64 {
    (completeness + accuracy + consistency + timeliness + validity) / 5.0
}

pub fn calculate_completeness(total: u64, missing: u64) -> f64 {
    if total == 0 {
        return 0.0;
    }
    ((total - missing) as f64 / total as f64) * 100.0
}

pub fn calculate_accuracy(total: u64, valid: u64) -> f64 {
    if total == 0 {
        return 0.0;
    }
    (valid as f64 / total as f64) * 100.0
}

pub fn categorize_score(score: f64) -> &'static str {
    match score {
        s if s >= 90.0 => "Excellent",
        s if s >= 75.0 => "Good",
        s if s >= 50.0 => "Fair",
        s if s >= 25.0 => "Poor",
        _ => "Critical",
    }
}
