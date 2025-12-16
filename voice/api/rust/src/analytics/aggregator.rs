//! Analytics data aggregation

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Aggregation window
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AggregationWindow {
    Minute,
    Hour,
    Day,
    Week,
    Month,
}

impl AggregationWindow {
    /// Get duration for this window
    pub fn duration(&self) -> Duration {
        match self {
            Self::Minute => Duration::minutes(1),
            Self::Hour => Duration::hours(1),
            Self::Day => Duration::days(1),
            Self::Week => Duration::weeks(1),
            Self::Month => Duration::days(30),
        }
    }
}

/// Aggregated metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AggregatedMetrics {
    /// Start of aggregation window
    pub window_start: DateTime<Utc>,

    /// End of aggregation window
    pub window_end: DateTime<Utc>,

    /// Window type
    pub window: AggregationWindow,

    /// Request metrics
    pub requests: RequestMetrics,

    /// Performance metrics
    pub performance: PerformanceMetrics,

    /// Error metrics
    pub errors: ErrorMetrics,

    /// Translation metrics
    pub translations: TranslationMetrics,

    /// Safety metrics
    pub safety: SafetyMetrics,
}

/// Request metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RequestMetrics {
    /// Total requests
    pub total: u64,

    /// By endpoint
    pub by_endpoint: HashMap<String, u64>,

    /// By source language
    pub by_source_language: HashMap<String, u64>,

    /// By target language
    pub by_target_language: HashMap<String, u64>,

    /// By status code
    pub by_status: HashMap<u16, u64>,
}

/// Performance metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    /// P50 latency in ms
    pub latency_p50_ms: f64,

    /// P95 latency in ms
    pub latency_p95_ms: f64,

    /// P99 latency in ms
    pub latency_p99_ms: f64,

    /// Throughput per second
    pub throughput_per_second: f64,
}

/// Error metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ErrorMetrics {
    /// Total errors
    pub total: u64,

    /// By error type
    pub by_type: HashMap<String, u64>,

    /// By endpoint
    pub by_endpoint: HashMap<String, u64>,

    /// Error rate
    pub error_rate: f64,
}

/// Translation metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TranslationMetrics {
    /// Total translations
    pub total: u64,

    /// Successful translations
    pub successful: u64,

    /// Failed translations
    pub failed: u64,

    /// Average quality score
    pub avg_quality_score: f64,

    /// Quality score distribution
    pub quality_distribution: QualityDistribution,

    /// By language pair
    pub by_language_pair: HashMap<String, u64>,

    /// Fallback count
    pub fallback_count: u64,
}

/// Quality score distribution
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct QualityDistribution {
    /// High quality (>0.9)
    pub high: u64,

    /// Medium quality (0.7-0.9)
    pub medium: u64,

    /// Low quality (<0.7)
    pub low: u64,
}

/// Safety metrics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SafetyMetrics {
    /// Content filtered count
    pub content_filtered: u64,

    /// By category
    pub by_category: HashMap<String, u64>,

    /// Emergency detections
    pub emergency_detections: u64,

    /// By urgency level
    pub by_urgency: HashMap<String, u64>,

    /// PII detections
    pub pii_detections: u64,

    /// PII redactions
    pub pii_redactions: u64,
}

/// Metrics aggregator
pub struct MetricsAggregator {
    /// Current window data
    current_window: AggregatedMetrics,

    /// Latency samples for percentile calculation
    latency_samples: Vec<f64>,

    /// Quality score samples
    quality_samples: Vec<f64>,
}

impl MetricsAggregator {
    /// Create a new aggregator
    pub fn new(window: AggregationWindow) -> Self {
        let now = Utc::now();
        Self {
            current_window: AggregatedMetrics {
                window_start: now,
                window_end: now + window.duration(),
                window,
                requests: RequestMetrics::default(),
                performance: PerformanceMetrics::default(),
                errors: ErrorMetrics::default(),
                translations: TranslationMetrics::default(),
                safety: SafetyMetrics::default(),
            },
            latency_samples: Vec::new(),
            quality_samples: Vec::new(),
        }
    }

    /// Record a request
    pub fn record_request(
        &mut self,
        endpoint: &str,
        source_lang: &str,
        target_lang: &str,
        status: u16,
        latency_ms: f64,
    ) {
        self.current_window.requests.total += 1;

        *self
            .current_window
            .requests
            .by_endpoint
            .entry(endpoint.to_string())
            .or_insert(0) += 1;

        *self
            .current_window
            .requests
            .by_source_language
            .entry(source_lang.to_string())
            .or_insert(0) += 1;

        *self
            .current_window
            .requests
            .by_target_language
            .entry(target_lang.to_string())
            .or_insert(0) += 1;

        *self
            .current_window
            .requests
            .by_status
            .entry(status)
            .or_insert(0) += 1;

        self.latency_samples.push(latency_ms);

        // Update error metrics if applicable
        if status >= 400 {
            self.current_window.errors.total += 1;
            *self
                .current_window
                .errors
                .by_endpoint
                .entry(endpoint.to_string())
                .or_insert(0) += 1;
        }
    }

    /// Record a translation
    pub fn record_translation(
        &mut self,
        source_lang: &str,
        target_lang: &str,
        success: bool,
        quality_score: Option<f64>,
        used_fallback: bool,
    ) {
        self.current_window.translations.total += 1;

        if success {
            self.current_window.translations.successful += 1;
        } else {
            self.current_window.translations.failed += 1;
        }

        let lang_pair = format!("{}->{}", source_lang, target_lang);
        *self
            .current_window
            .translations
            .by_language_pair
            .entry(lang_pair)
            .or_insert(0) += 1;

        if let Some(score) = quality_score {
            self.quality_samples.push(score);

            if score > 0.9 {
                self.current_window.translations.quality_distribution.high += 1;
            } else if score > 0.7 {
                self.current_window.translations.quality_distribution.medium += 1;
            } else {
                self.current_window.translations.quality_distribution.low += 1;
            }
        }

        if used_fallback {
            self.current_window.translations.fallback_count += 1;
        }
    }

    /// Record a safety event
    pub fn record_safety_event(&mut self, category: &str, is_emergency: bool, urgency: Option<&str>) {
        self.current_window.safety.content_filtered += 1;
        *self
            .current_window
            .safety
            .by_category
            .entry(category.to_string())
            .or_insert(0) += 1;

        if is_emergency {
            self.current_window.safety.emergency_detections += 1;
            if let Some(urgency_level) = urgency {
                *self
                    .current_window
                    .safety
                    .by_urgency
                    .entry(urgency_level.to_string())
                    .or_insert(0) += 1;
            }
        }
    }

    /// Record PII detection
    pub fn record_pii_detection(&mut self, redacted: bool) {
        self.current_window.safety.pii_detections += 1;
        if redacted {
            self.current_window.safety.pii_redactions += 1;
        }
    }

    /// Finalize the current window and return aggregated metrics
    pub fn finalize(&mut self) -> AggregatedMetrics {
        // Calculate percentiles
        self.latency_samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = self.latency_samples.len();

        if n > 0 {
            self.current_window.performance.latency_p50_ms =
                percentile(&self.latency_samples, 0.50);
            self.current_window.performance.latency_p95_ms =
                percentile(&self.latency_samples, 0.95);
            self.current_window.performance.latency_p99_ms =
                percentile(&self.latency_samples, 0.99);
        }

        // Calculate throughput
        let window_seconds = self.current_window.window.duration().num_seconds() as f64;
        self.current_window.performance.throughput_per_second =
            self.current_window.requests.total as f64 / window_seconds;

        // Calculate error rate
        if self.current_window.requests.total > 0 {
            self.current_window.errors.error_rate =
                self.current_window.errors.total as f64 / self.current_window.requests.total as f64;
        }

        // Calculate average quality score
        if !self.quality_samples.is_empty() {
            self.current_window.translations.avg_quality_score =
                self.quality_samples.iter().sum::<f64>() / self.quality_samples.len() as f64;
        }

        // Reset and return
        let result = self.current_window.clone();
        self.reset();
        result
    }

    /// Reset the aggregator for a new window
    fn reset(&mut self) {
        let now = Utc::now();
        self.current_window.window_start = now;
        self.current_window.window_end = now + self.current_window.window.duration();
        self.current_window.requests = RequestMetrics::default();
        self.current_window.performance = PerformanceMetrics::default();
        self.current_window.errors = ErrorMetrics::default();
        self.current_window.translations = TranslationMetrics::default();
        self.current_window.safety = SafetyMetrics::default();
        self.latency_samples.clear();
        self.quality_samples.clear();
    }

    /// Get current metrics snapshot (without finalizing)
    pub fn snapshot(&self) -> AggregatedMetrics {
        let mut snapshot = self.current_window.clone();

        // Calculate percentiles from current samples
        let mut sorted_latencies = self.latency_samples.clone();
        sorted_latencies.sort_by(|a, b| a.partial_cmp(b).unwrap());

        if !sorted_latencies.is_empty() {
            snapshot.performance.latency_p50_ms = percentile(&sorted_latencies, 0.50);
            snapshot.performance.latency_p95_ms = percentile(&sorted_latencies, 0.95);
            snapshot.performance.latency_p99_ms = percentile(&sorted_latencies, 0.99);
        }

        // Calculate average quality
        if !self.quality_samples.is_empty() {
            snapshot.translations.avg_quality_score =
                self.quality_samples.iter().sum::<f64>() / self.quality_samples.len() as f64;
        }

        snapshot
    }
}

/// Calculate percentile from sorted samples
fn percentile(sorted_samples: &[f64], p: f64) -> f64 {
    if sorted_samples.is_empty() {
        return 0.0;
    }

    let n = sorted_samples.len();
    let index = (p * (n - 1) as f64).round() as usize;
    sorted_samples[index.min(n - 1)]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metrics_aggregator() {
        let mut aggregator = MetricsAggregator::new(AggregationWindow::Minute);

        // Record some requests
        aggregator.record_request("/translate", "en", "ASL", 200, 150.0);
        aggregator.record_request("/translate", "en", "ASL", 200, 200.0);
        aggregator.record_request("/translate", "ko", "KSL", 500, 100.0);

        // Record translations
        aggregator.record_translation("en", "ASL", true, Some(0.95), false);
        aggregator.record_translation("ko", "KSL", true, Some(0.85), false);

        let metrics = aggregator.finalize();

        assert_eq!(metrics.requests.total, 3);
        assert_eq!(metrics.translations.total, 2);
        assert!(metrics.performance.latency_p50_ms > 0.0);
    }

    #[test]
    fn test_percentile() {
        let samples = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];

        assert!((percentile(&samples, 0.50) - 5.0).abs() < 1.0);
        assert!((percentile(&samples, 0.99) - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_safety_metrics() {
        let mut aggregator = MetricsAggregator::new(AggregationWindow::Hour);

        aggregator.record_safety_event("profanity", false, None);
        aggregator.record_safety_event("emergency", true, Some("critical"));
        aggregator.record_pii_detection(true);

        let metrics = aggregator.snapshot();

        assert_eq!(metrics.safety.content_filtered, 2);
        assert_eq!(metrics.safety.emergency_detections, 1);
        assert_eq!(metrics.safety.pii_detections, 1);
        assert_eq!(metrics.safety.pii_redactions, 1);
    }
}
