//! Core types for data quality

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataQualityReport {
    pub report_id: String,
    pub dataset_id: String,
    pub completeness: f64,
    pub accuracy: f64,
    pub consistency: f64,
    pub timeliness: f64,
    pub validity: f64,
    pub overall_score: f64,
    pub issues: Vec<QualityIssue>,
    pub created_at: DateTime<Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityIssue {
    pub issue_id: String,
    pub severity: IssueSeverity,
    pub category: IssueCategory,
    pub description: String,
    pub affected_records: u64,
    pub recommendation: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum IssueSeverity {
    Critical,
    High,
    Medium,
    Low,
    Info,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum IssueCategory {
    MissingData,
    InvalidFormat,
    DuplicateRecords,
    Inconsistency,
    Outliers,
    Obsolete,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityMetrics {
    pub total_records: u64,
    pub valid_records: u64,
    pub invalid_records: u64,
    pub duplicate_records: u64,
    pub missing_values: u64,
}
