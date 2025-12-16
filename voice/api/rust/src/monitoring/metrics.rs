//! Prometheus-compatible metrics collection

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Instant;

/// Metrics registry
pub struct MetricsRegistry {
    counters: RwLock<HashMap<String, Arc<Counter>>>,
    gauges: RwLock<HashMap<String, Arc<Gauge>>>,
    histograms: RwLock<HashMap<String, Arc<Histogram>>>,
}

impl MetricsRegistry {
    /// Create a new metrics registry
    pub fn new() -> Self {
        Self {
            counters: RwLock::new(HashMap::new()),
            gauges: RwLock::new(HashMap::new()),
            histograms: RwLock::new(HashMap::new()),
        }
    }

    /// Register a counter
    pub fn counter(&self, name: &str, help: &str) -> Arc<Counter> {
        let mut counters = self.counters.write().unwrap();
        counters
            .entry(name.to_string())
            .or_insert_with(|| {
                Arc::new(Counter {
                    name: name.to_string(),
                    help: help.to_string(),
                    values: RwLock::new(HashMap::new()),
                })
            })
            .clone()
    }

    /// Register a gauge
    pub fn gauge(&self, name: &str, help: &str) -> Arc<Gauge> {
        let mut gauges = self.gauges.write().unwrap();
        gauges
            .entry(name.to_string())
            .or_insert_with(|| {
                Arc::new(Gauge {
                    name: name.to_string(),
                    help: help.to_string(),
                    values: RwLock::new(HashMap::new()),
                })
            })
            .clone()
    }

    /// Register a histogram
    pub fn histogram(&self, name: &str, help: &str, buckets: Vec<f64>) -> Arc<Histogram> {
        let mut histograms = self.histograms.write().unwrap();
        histograms
            .entry(name.to_string())
            .or_insert_with(|| {
                Arc::new(Histogram {
                    name: name.to_string(),
                    help: help.to_string(),
                    buckets,
                    values: RwLock::new(HashMap::new()),
                })
            })
            .clone()
    }

    /// Export metrics in Prometheus format
    pub fn export_prometheus(&self) -> String {
        let mut output = String::new();

        // Export counters
        let counters = self.counters.read().unwrap();
        for counter in counters.values() {
            output.push_str(&format!("# HELP {} {}\n", counter.name, counter.help));
            output.push_str(&format!("# TYPE {} counter\n", counter.name));

            let values = counter.values.read().unwrap();
            for (labels, value) in values.iter() {
                if labels.is_empty() {
                    output.push_str(&format!("{} {}\n", counter.name, value.load(Ordering::Relaxed)));
                } else {
                    output.push_str(&format!(
                        "{}{{{}}} {}\n",
                        counter.name,
                        labels,
                        value.load(Ordering::Relaxed)
                    ));
                }
            }
        }

        // Export gauges
        let gauges = self.gauges.read().unwrap();
        for gauge in gauges.values() {
            output.push_str(&format!("# HELP {} {}\n", gauge.name, gauge.help));
            output.push_str(&format!("# TYPE {} gauge\n", gauge.name));

            let values = gauge.values.read().unwrap();
            for (labels, value) in values.iter() {
                if labels.is_empty() {
                    output.push_str(&format!("{} {}\n", gauge.name, value.load(Ordering::Relaxed)));
                } else {
                    output.push_str(&format!(
                        "{}{{{}}} {}\n",
                        gauge.name,
                        labels,
                        value.load(Ordering::Relaxed)
                    ));
                }
            }
        }

        // Export histograms
        let histograms = self.histograms.read().unwrap();
        for histogram in histograms.values() {
            output.push_str(&format!("# HELP {} {}\n", histogram.name, histogram.help));
            output.push_str(&format!("# TYPE {} histogram\n", histogram.name));

            let values = histogram.values.read().unwrap();
            for (labels, data) in values.iter() {
                let label_str = if labels.is_empty() {
                    String::new()
                } else {
                    format!("{{{}}}", labels)
                };

                // Export buckets
                let buckets = data.buckets.read().unwrap();
                for (i, &bucket) in histogram.buckets.iter().enumerate() {
                    output.push_str(&format!(
                        "{}_bucket{{le=\"{}\",{}}} {}\n",
                        histogram.name,
                        bucket,
                        labels,
                        buckets.get(i).copied().unwrap_or(0)
                    ));
                }
                output.push_str(&format!(
                    "{}_bucket{{le=\"+Inf\",{}}} {}\n",
                    histogram.name,
                    labels,
                    data.count.load(Ordering::Relaxed)
                ));

                // Export sum and count
                output.push_str(&format!(
                    "{}_sum{} {}\n",
                    histogram.name,
                    label_str,
                    f64::from_bits(data.sum.load(Ordering::Relaxed))
                ));
                output.push_str(&format!(
                    "{}_count{} {}\n",
                    histogram.name,
                    label_str,
                    data.count.load(Ordering::Relaxed)
                ));
            }
        }

        output
    }
}

impl Default for MetricsRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Counter metric
pub struct Counter {
    name: String,
    help: String,
    values: RwLock<HashMap<String, Arc<AtomicU64>>>,
}

impl Counter {
    /// Increment counter
    pub fn inc(&self) {
        self.inc_by(1);
    }

    /// Increment counter by value
    pub fn inc_by(&self, value: u64) {
        self.with_labels("").inc_by(value);
    }

    /// Get counter with labels
    pub fn with_labels(&self, labels: &str) -> CounterWithLabels {
        let mut values = self.values.write().unwrap();
        let counter = values
            .entry(labels.to_string())
            .or_insert_with(|| Arc::new(AtomicU64::new(0)))
            .clone();
        CounterWithLabels { counter }
    }
}

/// Counter with labels
pub struct CounterWithLabels {
    counter: Arc<AtomicU64>,
}

impl CounterWithLabels {
    /// Increment
    pub fn inc(&self) {
        self.inc_by(1);
    }

    /// Increment by value
    pub fn inc_by(&self, value: u64) {
        self.counter.fetch_add(value, Ordering::Relaxed);
    }
}

/// Gauge metric
pub struct Gauge {
    name: String,
    help: String,
    values: RwLock<HashMap<String, Arc<AtomicU64>>>,
}

impl Gauge {
    /// Set gauge value
    pub fn set(&self, value: f64) {
        self.with_labels("").set(value);
    }

    /// Increment gauge
    pub fn inc(&self) {
        self.with_labels("").inc();
    }

    /// Decrement gauge
    pub fn dec(&self) {
        self.with_labels("").dec();
    }

    /// Get gauge with labels
    pub fn with_labels(&self, labels: &str) -> GaugeWithLabels {
        let mut values = self.values.write().unwrap();
        let gauge = values
            .entry(labels.to_string())
            .or_insert_with(|| Arc::new(AtomicU64::new(0)))
            .clone();
        GaugeWithLabels { gauge }
    }
}

/// Gauge with labels
pub struct GaugeWithLabels {
    gauge: Arc<AtomicU64>,
}

impl GaugeWithLabels {
    /// Set value
    pub fn set(&self, value: f64) {
        self.gauge.store(value.to_bits(), Ordering::Relaxed);
    }

    /// Increment
    pub fn inc(&self) {
        let current = f64::from_bits(self.gauge.load(Ordering::Relaxed));
        self.set(current + 1.0);
    }

    /// Decrement
    pub fn dec(&self) {
        let current = f64::from_bits(self.gauge.load(Ordering::Relaxed));
        self.set(current - 1.0);
    }
}

/// Histogram metric
pub struct Histogram {
    name: String,
    help: String,
    buckets: Vec<f64>,
    values: RwLock<HashMap<String, Arc<HistogramData>>>,
}

struct HistogramData {
    buckets: RwLock<Vec<u64>>,
    sum: AtomicU64,
    count: AtomicU64,
}

impl Histogram {
    /// Observe a value
    pub fn observe(&self, value: f64) {
        self.with_labels("").observe(value);
    }

    /// Get histogram with labels
    pub fn with_labels(&self, labels: &str) -> HistogramWithLabels {
        let mut values = self.values.write().unwrap();
        let data = values
            .entry(labels.to_string())
            .or_insert_with(|| {
                Arc::new(HistogramData {
                    buckets: RwLock::new(vec![0; self.buckets.len()]),
                    sum: AtomicU64::new(0),
                    count: AtomicU64::new(0),
                })
            })
            .clone();
        HistogramWithLabels {
            buckets: self.buckets.clone(),
            data,
        }
    }

    /// Start a timer
    pub fn start_timer(&self) -> HistogramTimer {
        HistogramTimer {
            histogram: self.with_labels(""),
            start: Instant::now(),
        }
    }
}

/// Histogram with labels
pub struct HistogramWithLabels {
    buckets: Vec<f64>,
    data: Arc<HistogramData>,
}

impl HistogramWithLabels {
    /// Observe a value
    pub fn observe(&self, value: f64) {
        // Update buckets
        {
            let mut buckets = self.data.buckets.write().unwrap();
            for (i, &bucket) in self.buckets.iter().enumerate() {
                if value <= bucket {
                    buckets[i] += 1;
                }
            }
        }

        // Update sum
        loop {
            let current = self.data.sum.load(Ordering::Relaxed);
            let new = f64::from_bits(current) + value;
            if self
                .data
                .sum
                .compare_exchange(current, new.to_bits(), Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                break;
            }
        }

        // Update count
        self.data.count.fetch_add(1, Ordering::Relaxed);
    }
}

/// Histogram timer
pub struct HistogramTimer {
    histogram: HistogramWithLabels,
    start: Instant,
}

impl Drop for HistogramTimer {
    fn drop(&mut self) {
        let duration = self.start.elapsed().as_secs_f64();
        self.histogram.observe(duration);
    }
}

/// Voice-Sign specific metrics
pub struct VoiceSignMetrics {
    registry: Arc<MetricsRegistry>,

    // HTTP metrics
    pub http_requests_total: Arc<Counter>,
    pub http_request_duration: Arc<Histogram>,
    pub http_active_connections: Arc<Gauge>,

    // Translation metrics
    pub translations_total: Arc<Counter>,
    pub translation_duration: Arc<Histogram>,
    pub translation_quality_score: Arc<Histogram>,
    pub translation_queue_depth: Arc<Gauge>,

    // Cache metrics
    pub cache_hits_total: Arc<Counter>,
    pub cache_misses_total: Arc<Counter>,

    // Safety metrics
    pub content_filter_total: Arc<Counter>,
    pub emergency_detected_total: Arc<Counter>,
    pub pii_detected_total: Arc<Counter>,
}

impl VoiceSignMetrics {
    /// Create new metrics instance
    pub fn new() -> Self {
        let registry = Arc::new(MetricsRegistry::new());

        // HTTP metrics
        let http_requests_total =
            registry.counter("voice_sign_http_requests_total", "Total HTTP requests");
        let http_request_duration = registry.histogram(
            "voice_sign_http_request_duration_seconds",
            "HTTP request duration in seconds",
            vec![0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0],
        );
        let http_active_connections =
            registry.gauge("voice_sign_http_active_connections", "Active HTTP connections");

        // Translation metrics
        let translations_total =
            registry.counter("voice_sign_translations_total", "Total translations");
        let translation_duration = registry.histogram(
            "voice_sign_translation_duration_seconds",
            "Translation duration in seconds",
            vec![0.1, 0.25, 0.5, 1.0, 2.0, 5.0],
        );
        let translation_quality_score = registry.histogram(
            "voice_sign_translation_quality_score",
            "Translation quality scores",
            vec![0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1.0],
        );
        let translation_queue_depth =
            registry.gauge("voice_sign_translation_queue_depth", "Translation queue depth");

        // Cache metrics
        let cache_hits_total = registry.counter("voice_sign_cache_hits_total", "Cache hits");
        let cache_misses_total = registry.counter("voice_sign_cache_misses_total", "Cache misses");

        // Safety metrics
        let content_filter_total = registry.counter(
            "voice_sign_content_filter_total",
            "Content filter activations",
        );
        let emergency_detected_total =
            registry.counter("voice_sign_emergency_detected_total", "Emergencies detected");
        let pii_detected_total = registry.counter("voice_sign_pii_detected_total", "PII detections");

        Self {
            registry,
            http_requests_total,
            http_request_duration,
            http_active_connections,
            translations_total,
            translation_duration,
            translation_quality_score,
            translation_queue_depth,
            cache_hits_total,
            cache_misses_total,
            content_filter_total,
            emergency_detected_total,
            pii_detected_total,
        }
    }

    /// Export metrics in Prometheus format
    pub fn export(&self) -> String {
        self.registry.export_prometheus()
    }

    /// Record HTTP request
    pub fn record_http_request(&self, method: &str, endpoint: &str, status: u16, duration: f64) {
        let labels = format!("method=\"{}\",endpoint=\"{}\",status=\"{}\"", method, endpoint, status);
        self.http_requests_total.with_labels(&labels).inc();
        self.http_request_duration.with_labels(&labels).observe(duration);
    }

    /// Record translation
    pub fn record_translation(
        &self,
        source_lang: &str,
        target_lang: &str,
        status: &str,
        duration: f64,
        quality_score: Option<f64>,
    ) {
        let labels = format!(
            "source_lang=\"{}\",target_lang=\"{}\",status=\"{}\"",
            source_lang, target_lang, status
        );
        self.translations_total.with_labels(&labels).inc();
        self.translation_duration.with_labels(&labels).observe(duration);

        if let Some(score) = quality_score {
            let score_labels = format!("target_lang=\"{}\"", target_lang);
            self.translation_quality_score.with_labels(&score_labels).observe(score);
        }
    }
}

impl Default for VoiceSignMetrics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_counter() {
        let registry = MetricsRegistry::new();
        let counter = registry.counter("test_counter", "A test counter");

        counter.inc();
        counter.inc_by(5);
        counter.with_labels("method=\"GET\"").inc();

        let output = registry.export_prometheus();
        assert!(output.contains("test_counter"));
    }

    #[test]
    fn test_gauge() {
        let registry = MetricsRegistry::new();
        let gauge = registry.gauge("test_gauge", "A test gauge");

        gauge.set(42.0);
        gauge.inc();
        gauge.dec();

        let output = registry.export_prometheus();
        assert!(output.contains("test_gauge"));
    }

    #[test]
    fn test_histogram() {
        let registry = MetricsRegistry::new();
        let histogram = registry.histogram(
            "test_histogram",
            "A test histogram",
            vec![0.1, 0.5, 1.0, 5.0],
        );

        histogram.observe(0.25);
        histogram.observe(0.75);
        histogram.observe(2.0);

        let output = registry.export_prometheus();
        assert!(output.contains("test_histogram"));
        assert!(output.contains("test_histogram_bucket"));
        assert!(output.contains("test_histogram_sum"));
        assert!(output.contains("test_histogram_count"));
    }

    #[test]
    fn test_voice_sign_metrics() {
        let metrics = VoiceSignMetrics::new();

        metrics.record_http_request("POST", "/translate", 200, 0.5);
        metrics.record_translation("en", "ASL", "success", 0.3, Some(0.95));

        let output = metrics.export();
        assert!(output.contains("voice_sign_http_requests_total"));
        assert!(output.contains("voice_sign_translations_total"));
    }
}
