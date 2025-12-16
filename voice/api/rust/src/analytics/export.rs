//! Analytics data export functionality

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::aggregator::{AggregatedMetrics, AggregationWindow};

/// Export format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExportFormat {
    Json,
    Csv,
    Parquet,
}

/// Export request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportRequest {
    /// Metrics to export
    pub metrics: Vec<String>,

    /// Start time
    pub start_time: DateTime<Utc>,

    /// End time
    pub end_time: DateTime<Utc>,

    /// Aggregation granularity
    pub granularity: AggregationWindow,

    /// Export format
    pub format: ExportFormat,

    /// Optional filters
    pub filters: Option<ExportFilters>,
}

/// Export filters
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ExportFilters {
    /// Filter by source language
    pub source_language: Option<String>,

    /// Filter by target language
    pub target_language: Option<String>,

    /// Filter by endpoint
    pub endpoint: Option<String>,

    /// Filter by status
    pub status: Option<u16>,
}

/// Export response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportResponse {
    /// Export metadata
    pub metadata: ExportMetadata,

    /// Exported data
    pub data: Vec<ExportRow>,
}

/// Export metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportMetadata {
    /// Export ID
    pub export_id: String,

    /// Generated at
    pub generated_at: DateTime<Utc>,

    /// Query parameters
    pub query: ExportQuery,

    /// Row count
    pub row_count: usize,

    /// Format
    pub format: ExportFormat,
}

/// Export query
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportQuery {
    pub metrics: Vec<String>,
    pub start_time: DateTime<Utc>,
    pub end_time: DateTime<Utc>,
    pub granularity: AggregationWindow,
}

/// Export row
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportRow {
    /// Timestamp
    pub timestamp: DateTime<Utc>,

    /// Metric values
    #[serde(flatten)]
    pub values: serde_json::Value,
}

/// Analytics exporter
pub struct AnalyticsExporter;

impl AnalyticsExporter {
    /// Create a new exporter
    pub fn new() -> Self {
        Self
    }

    /// Export aggregated metrics to JSON
    pub fn export_json(&self, metrics: &[AggregatedMetrics]) -> Result<String, ExportError> {
        let rows: Vec<ExportRow> = metrics
            .iter()
            .map(|m| ExportRow {
                timestamp: m.window_start,
                values: serde_json::to_value(m).unwrap_or(serde_json::Value::Null),
            })
            .collect();

        let response = ExportResponse {
            metadata: ExportMetadata {
                export_id: uuid::Uuid::new_v4().to_string(),
                generated_at: Utc::now(),
                query: ExportQuery {
                    metrics: vec!["all".to_string()],
                    start_time: metrics.first().map(|m| m.window_start).unwrap_or_else(Utc::now),
                    end_time: metrics.last().map(|m| m.window_end).unwrap_or_else(Utc::now),
                    granularity: metrics
                        .first()
                        .map(|m| m.window)
                        .unwrap_or(AggregationWindow::Hour),
                },
                row_count: rows.len(),
                format: ExportFormat::Json,
            },
            data: rows,
        };

        serde_json::to_string_pretty(&response)
            .map_err(|e| ExportError::SerializationError(e.to_string()))
    }

    /// Export aggregated metrics to CSV
    pub fn export_csv(&self, metrics: &[AggregatedMetrics]) -> Result<String, ExportError> {
        let mut csv = String::new();

        // Header
        csv.push_str("timestamp,requests_total,error_rate,latency_p50_ms,latency_p95_ms,latency_p99_ms,translations_total,avg_quality_score\n");

        // Data rows
        for m in metrics {
            csv.push_str(&format!(
                "{},{},{:.4},{:.2},{:.2},{:.2},{},{:.4}\n",
                m.window_start.to_rfc3339(),
                m.requests.total,
                m.errors.error_rate,
                m.performance.latency_p50_ms,
                m.performance.latency_p95_ms,
                m.performance.latency_p99_ms,
                m.translations.total,
                m.translations.avg_quality_score,
            ));
        }

        Ok(csv)
    }

    /// Export specific metrics
    pub fn export_metrics(
        &self,
        metrics: &[AggregatedMetrics],
        selected_metrics: &[&str],
        format: ExportFormat,
    ) -> Result<String, ExportError> {
        match format {
            ExportFormat::Json => self.export_selected_json(metrics, selected_metrics),
            ExportFormat::Csv => self.export_selected_csv(metrics, selected_metrics),
            ExportFormat::Parquet => Err(ExportError::UnsupportedFormat(
                "Parquet export not implemented".to_string(),
            )),
        }
    }

    fn export_selected_json(
        &self,
        metrics: &[AggregatedMetrics],
        selected_metrics: &[&str],
    ) -> Result<String, ExportError> {
        let rows: Vec<serde_json::Value> = metrics
            .iter()
            .map(|m| {
                let mut row = serde_json::Map::new();
                row.insert(
                    "timestamp".to_string(),
                    serde_json::Value::String(m.window_start.to_rfc3339()),
                );

                for metric in selected_metrics {
                    match *metric {
                        "requests_total" => {
                            row.insert(
                                "requests_total".to_string(),
                                serde_json::Value::Number(m.requests.total.into()),
                            );
                        }
                        "error_rate" => {
                            row.insert(
                                "error_rate".to_string(),
                                serde_json::json!(m.errors.error_rate),
                            );
                        }
                        "latency_p50" => {
                            row.insert(
                                "latency_p50_ms".to_string(),
                                serde_json::json!(m.performance.latency_p50_ms),
                            );
                        }
                        "latency_p95" => {
                            row.insert(
                                "latency_p95_ms".to_string(),
                                serde_json::json!(m.performance.latency_p95_ms),
                            );
                        }
                        "latency_p99" => {
                            row.insert(
                                "latency_p99_ms".to_string(),
                                serde_json::json!(m.performance.latency_p99_ms),
                            );
                        }
                        "translations_total" => {
                            row.insert(
                                "translations_total".to_string(),
                                serde_json::Value::Number(m.translations.total.into()),
                            );
                        }
                        "quality_score" => {
                            row.insert(
                                "avg_quality_score".to_string(),
                                serde_json::json!(m.translations.avg_quality_score),
                            );
                        }
                        _ => {}
                    }
                }

                serde_json::Value::Object(row)
            })
            .collect();

        serde_json::to_string_pretty(&rows)
            .map_err(|e| ExportError::SerializationError(e.to_string()))
    }

    fn export_selected_csv(
        &self,
        metrics: &[AggregatedMetrics],
        selected_metrics: &[&str],
    ) -> Result<String, ExportError> {
        let mut csv = String::new();

        // Build header
        let mut headers = vec!["timestamp"];
        headers.extend(selected_metrics.iter().copied());
        csv.push_str(&headers.join(","));
        csv.push('\n');

        // Build data rows
        for m in metrics {
            let mut row = vec![m.window_start.to_rfc3339()];

            for metric in selected_metrics {
                let value = match *metric {
                    "requests_total" => m.requests.total.to_string(),
                    "error_rate" => format!("{:.4}", m.errors.error_rate),
                    "latency_p50" => format!("{:.2}", m.performance.latency_p50_ms),
                    "latency_p95" => format!("{:.2}", m.performance.latency_p95_ms),
                    "latency_p99" => format!("{:.2}", m.performance.latency_p99_ms),
                    "translations_total" => m.translations.total.to_string(),
                    "quality_score" => format!("{:.4}", m.translations.avg_quality_score),
                    _ => "".to_string(),
                };
                row.push(value);
            }

            csv.push_str(&row.join(","));
            csv.push('\n');
        }

        Ok(csv)
    }
}

impl Default for AnalyticsExporter {
    fn default() -> Self {
        Self::new()
    }
}

/// Export error
#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    #[error("Invalid request: {0}")]
    InvalidRequest(String),

    #[error("IO error: {0}")]
    IoError(String),
}

/// Report generator
pub struct ReportGenerator;

impl ReportGenerator {
    /// Generate a daily summary report
    pub fn daily_summary(metrics: &AggregatedMetrics) -> DailySummaryReport {
        DailySummaryReport {
            date: metrics.window_start.date_naive(),
            total_requests: metrics.requests.total,
            total_translations: metrics.translations.total,
            successful_translations: metrics.translations.successful,
            failed_translations: metrics.translations.failed,
            average_quality_score: metrics.translations.avg_quality_score,
            error_rate: metrics.errors.error_rate,
            latency_p50_ms: metrics.performance.latency_p50_ms,
            latency_p99_ms: metrics.performance.latency_p99_ms,
            top_source_languages: Self::top_n(&metrics.requests.by_source_language, 5),
            top_target_languages: Self::top_n(&metrics.requests.by_target_language, 5),
            safety_events: metrics.safety.content_filtered,
            emergency_detections: metrics.safety.emergency_detections,
        }
    }

    fn top_n(map: &std::collections::HashMap<String, u64>, n: usize) -> Vec<(String, u64)> {
        let mut items: Vec<_> = map.iter().map(|(k, v)| (k.clone(), *v)).collect();
        items.sort_by(|a, b| b.1.cmp(&a.1));
        items.truncate(n);
        items
    }
}

/// Daily summary report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DailySummaryReport {
    pub date: chrono::NaiveDate,
    pub total_requests: u64,
    pub total_translations: u64,
    pub successful_translations: u64,
    pub failed_translations: u64,
    pub average_quality_score: f64,
    pub error_rate: f64,
    pub latency_p50_ms: f64,
    pub latency_p99_ms: f64,
    pub top_source_languages: Vec<(String, u64)>,
    pub top_target_languages: Vec<(String, u64)>,
    pub safety_events: u64,
    pub emergency_detections: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytics::aggregator::{
        ErrorMetrics, PerformanceMetrics, RequestMetrics, SafetyMetrics, TranslationMetrics,
        QualityDistribution,
    };

    fn sample_metrics() -> AggregatedMetrics {
        AggregatedMetrics {
            window_start: Utc::now(),
            window_end: Utc::now(),
            window: AggregationWindow::Hour,
            requests: RequestMetrics {
                total: 1000,
                ..Default::default()
            },
            performance: PerformanceMetrics {
                latency_p50_ms: 150.0,
                latency_p95_ms: 300.0,
                latency_p99_ms: 500.0,
                throughput_per_second: 10.0,
            },
            errors: ErrorMetrics {
                total: 10,
                error_rate: 0.01,
                ..Default::default()
            },
            translations: TranslationMetrics {
                total: 900,
                successful: 880,
                failed: 20,
                avg_quality_score: 0.92,
                ..Default::default()
            },
            safety: SafetyMetrics::default(),
        }
    }

    #[test]
    fn test_export_json() {
        let exporter = AnalyticsExporter::new();
        let metrics = vec![sample_metrics()];

        let json = exporter.export_json(&metrics).unwrap();
        assert!(json.contains("requests_total"));
        assert!(json.contains("1000"));
    }

    #[test]
    fn test_export_csv() {
        let exporter = AnalyticsExporter::new();
        let metrics = vec![sample_metrics()];

        let csv = exporter.export_csv(&metrics).unwrap();
        assert!(csv.contains("timestamp"));
        assert!(csv.contains("1000"));
    }

    #[test]
    fn test_export_selected_metrics() {
        let exporter = AnalyticsExporter::new();
        let metrics = vec![sample_metrics()];

        let json = exporter
            .export_metrics(&metrics, &["requests_total", "error_rate"], ExportFormat::Json)
            .unwrap();

        assert!(json.contains("requests_total"));
        assert!(json.contains("error_rate"));
    }
}
