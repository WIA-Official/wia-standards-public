//! Validation functions for data quality

use crate::{error::{Error, Result}, types::*};

pub fn validate_report(report: &DataQualityReport) -> Result<()> {
    if report.report_id.is_empty() {
        return Err(Error::ValidationError("Report ID cannot be empty".to_string()));
    }
    if report.overall_score < 0.0 || report.overall_score > 100.0 {
        return Err(Error::ValidationError("Overall score must be 0-100".to_string()));
    }
    Ok(())
}

pub fn validate_metrics(metrics: &QualityMetrics) -> Result<()> {
    if metrics.valid_records + metrics.invalid_records > metrics.total_records {
        return Err(Error::ValidationError("Invalid metrics: sum exceeds total".to_string()));
    }
    Ok(())
}
