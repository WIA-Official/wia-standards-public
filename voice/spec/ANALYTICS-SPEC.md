# WIA Voice-Sign Analytics Specification

Version: 1.0.0
Status: Draft
Last Updated: 2025-01-15

## 1. Overview

This specification defines the analytics collection, processing, and reporting standards for WIA Voice-Sign translation systems, with a strong emphasis on privacy protection.

## 2. Privacy-First Design Principles

### 2.1 Core Principles

1. **Data Minimization** - Collect only necessary metrics
2. **Anonymization** - No PII in analytics data
3. **Aggregation** - Individual events aggregated before storage
4. **Transparency** - Users know what is collected
5. **Control** - Users can opt-out of analytics

### 2.2 Data Classification

| Classification | Description | Retention | Examples |
|----------------|-------------|-----------|----------|
| Operational | System metrics | 30 days | Latency, errors, throughput |
| Aggregated | Usage patterns | 1 year | Daily translation counts |
| Anonymized | User behavior | 90 days | Session patterns (no IDs) |
| Sensitive | Never collected | N/A | Audio content, translations |

### 2.3 Privacy Controls

```yaml
privacy_settings:
  pii_collection: never
  audio_retention: none
  translation_content_retention: none

  anonymization:
    user_id: hash_with_daily_rotation
    session_id: random_uuid
    ip_address: truncate_to_country

  aggregation:
    minimum_sample_size: 100
    time_window: 1h

  opt_out:
    enabled: true
    granular: true
    categories:
      - usage_analytics
      - quality_feedback
      - error_reporting
```

## 3. Metrics Collection

### 3.1 Usage Metrics

```typescript
interface UsageMetrics {
  // Request metrics (no PII)
  requests: {
    total: number;
    by_endpoint: Record<string, number>;
    by_source_language: Record<string, number>;
    by_target_language: Record<string, number>;
    by_status: Record<string, number>;
  };

  // Performance metrics
  performance: {
    latency_p50_ms: number;
    latency_p95_ms: number;
    latency_p99_ms: number;
    throughput_per_second: number;
  };

  // Error metrics
  errors: {
    total: number;
    by_type: Record<string, number>;
    by_endpoint: Record<string, number>;
  };

  // Time window
  window: {
    start: string;  // ISO8601
    end: string;    // ISO8601
    duration_seconds: number;
  };
}
```

### 3.2 Translation Quality Metrics

```typescript
interface QualityMetrics {
  // Quality scores (aggregated)
  scores: {
    average: number;
    p50: number;
    p95: number;
    distribution: Record<string, number>;  // Buckets
  };

  // By language pair
  by_language_pair: {
    source: string;
    target: string;
    sample_count: number;
    average_score: number;
  }[];

  // Confidence levels
  confidence: {
    high: number;      // > 0.9
    medium: number;    // 0.7 - 0.9
    low: number;       // < 0.7
  };

  // Fallback usage
  fallbacks: {
    total: number;
    by_reason: Record<string, number>;
  };
}
```

### 3.3 Feature Usage Metrics

```typescript
interface FeatureUsageMetrics {
  // Translation features
  translation: {
    text_input_count: number;
    audio_input_count: number;
    streaming_count: number;
    batch_count: number;
  };

  // Output preferences
  output: {
    gloss_requested: number;
    notation_requested: number;
    pose_requested: number;
    render_requested: number;
  };

  // Sign languages used
  sign_languages: Record<string, number>;

  // Playback features
  playback: {
    speed_adjustments: number;
    repeat_requests: number;
    pause_events: number;
  };
}
```

### 3.4 Safety Metrics

```typescript
interface SafetyMetrics {
  // Content filtering
  content_filter: {
    total_filtered: number;
    by_category: Record<string, number>;
    by_action: Record<string, number>;
  };

  // Emergency detection
  emergency: {
    total_detected: number;
    by_urgency: Record<string, number>;
    average_response_time_ms: number;
  };

  // PII handling
  pii: {
    detections: number;
    redactions: number;
    by_type: Record<string, number>;
  };
}
```

## 4. Data Pipeline

### 4.1 Collection Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   API       │────▶│  Collector  │────▶│   Buffer    │
│  Services   │     │   Agent     │     │   (Kafka)   │
└─────────────┘     └─────────────┘     └─────────────┘
                                               │
                    ┌──────────────────────────┤
                    ▼                          ▼
             ┌─────────────┐           ┌─────────────┐
             │  Real-time  │           │   Batch     │
             │  Processor  │           │  Processor  │
             └─────────────┘           └─────────────┘
                    │                          │
                    ▼                          ▼
             ┌─────────────┐           ┌─────────────┐
             │   Redis     │           │  Data Lake  │
             │  (Hot Data) │           │  (S3/GCS)   │
             └─────────────┘           └─────────────┘
                    │                          │
                    └──────────┬───────────────┘
                               ▼
                        ┌─────────────┐
                        │  Dashboard  │
                        │    API      │
                        └─────────────┘
```

### 4.2 Event Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "event_id": {
      "type": "string",
      "format": "uuid"
    },
    "event_type": {
      "type": "string",
      "enum": ["translation_request", "translation_complete", "error", "safety_event"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "service": {
      "type": "string"
    },
    "version": {
      "type": "string"
    },
    "session_hash": {
      "type": "string",
      "description": "Anonymized session identifier"
    },
    "metrics": {
      "type": "object"
    }
  },
  "required": ["event_id", "event_type", "timestamp", "service"]
}
```

### 4.3 Aggregation Windows

```yaml
aggregation:
  real_time:
    window: 1m
    metrics:
      - request_count
      - error_count
      - latency_percentiles

  short_term:
    window: 1h
    metrics:
      - all_real_time
      - quality_scores
      - feature_usage

  daily:
    window: 1d
    metrics:
      - all_short_term
      - unique_sessions (HyperLogLog)
      - language_distribution

  monthly:
    window: 30d
    metrics:
      - all_daily
      - trend_analysis
      - capacity_planning
```

## 5. Real-time Processing

### 5.1 Streaming Pipeline

```rust
// Real-time analytics processor
pub struct RealtimeAnalytics {
    window_size: Duration,
    aggregators: HashMap<String, Box<dyn Aggregator>>,
}

impl RealtimeAnalytics {
    pub async fn process_event(&mut self, event: AnalyticsEvent) {
        // Update real-time counters
        self.increment_counter(&event);

        // Update sliding window aggregations
        self.update_window_aggregation(&event);

        // Check for anomalies
        if let Some(anomaly) = self.detect_anomaly(&event) {
            self.emit_alert(anomaly).await;
        }

        // Emit to real-time dashboard
        self.emit_to_dashboard(&event).await;
    }
}

// Aggregator trait
pub trait Aggregator: Send + Sync {
    fn add(&mut self, value: f64);
    fn result(&self) -> AggregationResult;
    fn reset(&mut self);
}

// Percentile aggregator
pub struct PercentileAggregator {
    samples: Vec<f64>,
    percentiles: Vec<f64>,
}

// Count aggregator with HyperLogLog for unique counts
pub struct UniqueCountAggregator {
    hll: HyperLogLog,
}
```

### 5.2 Anomaly Detection

```yaml
anomaly_detection:
  latency:
    method: z_score
    threshold: 3.0
    window: 5m

  error_rate:
    method: percentage_change
    threshold: 200%  # 2x normal
    baseline_window: 1h

  request_rate:
    method: forecasting
    model: holt_winters
    sensitivity: medium

  quality_score:
    method: moving_average
    deviation_threshold: 0.1
    window: 1h
```

## 6. Batch Processing

### 6.1 Daily Aggregation Job

```python
# Daily analytics aggregation (pseudo-code)
def daily_aggregation():
    # Read raw events from data lake
    events = read_events(
        source="s3://analytics/raw/",
        date=yesterday()
    )

    # Aggregate metrics
    aggregated = (
        events
        .filter(lambda e: e.event_type in METRIC_EVENTS)
        .group_by("service", "endpoint", "language_pair")
        .aggregate(
            request_count=count("*"),
            error_count=count_where("status >= 400"),
            latency_p50=percentile("latency_ms", 0.50),
            latency_p95=percentile("latency_ms", 0.95),
            latency_p99=percentile("latency_ms", 0.99),
            quality_avg=avg("quality_score"),
            unique_sessions=approx_distinct("session_hash")
        )
    )

    # Write to analytics database
    write_to_db(aggregated, table="daily_metrics")

    # Generate reports
    generate_daily_report(aggregated)
```

### 6.2 Retention Policies

```yaml
retention:
  raw_events:
    hot_storage: 7d
    cold_storage: 30d
    archive: 90d (then delete)

  aggregated_metrics:
    minutely: 7d
    hourly: 30d
    daily: 365d
    monthly: forever

  reports:
    daily: 90d
    weekly: 1y
    monthly: forever
```

## 7. A/B Testing Framework

### 7.1 Experiment Configuration

```yaml
experiment:
  id: "exp-2025-001"
  name: "New ASR Model Comparison"
  description: "Compare Whisper v3 vs custom ASR model"

  variants:
    - id: "control"
      name: "Whisper v3"
      traffic_percentage: 50

    - id: "treatment"
      name: "Custom ASR v1"
      traffic_percentage: 50

  assignment:
    method: "session_hash"
    sticky: true

  metrics:
    primary:
      - name: "translation_quality"
        type: "mean"
        min_sample_size: 1000

    secondary:
      - name: "latency_p50"
        type: "percentile"
      - name: "error_rate"
        type: "proportion"

  duration:
    start: "2025-01-15T00:00:00Z"
    end: "2025-01-29T00:00:00Z"

  guardrails:
    max_error_rate_increase: 0.01
    max_latency_increase_percent: 20
```

### 7.2 Statistical Analysis

```rust
pub struct ExperimentAnalysis {
    experiment_id: String,
    variants: Vec<VariantMetrics>,
    statistical_results: StatisticalResults,
}

pub struct StatisticalResults {
    // Primary metric
    primary_metric: MetricResult,

    // Secondary metrics
    secondary_metrics: Vec<MetricResult>,

    // Overall conclusion
    conclusion: ExperimentConclusion,
}

pub struct MetricResult {
    metric_name: String,
    control_value: f64,
    treatment_value: f64,
    relative_difference: f64,
    confidence_interval: (f64, f64),
    p_value: f64,
    is_significant: bool,
}

pub enum ExperimentConclusion {
    TreatmentWins,
    ControlWins,
    NoSignificantDifference,
    InsufficientData,
}
```

## 8. Dashboard Visualizations

### 8.1 Executive Dashboard

```yaml
executive_dashboard:
  refresh_interval: 5m

  sections:
    - title: "Key Metrics"
      panels:
        - type: "single_stat"
          metric: "total_translations_today"
          comparison: "vs_yesterday"

        - type: "single_stat"
          metric: "average_quality_score"
          comparison: "vs_last_week"

        - type: "single_stat"
          metric: "error_rate"
          comparison: "vs_sla"

    - title: "Usage Trends"
      panels:
        - type: "time_series"
          metric: "translations_per_hour"
          period: "7d"

        - type: "pie_chart"
          metric: "translations_by_language"

    - title: "Quality Overview"
      panels:
        - type: "heatmap"
          metric: "quality_by_language_pair"

        - type: "histogram"
          metric: "quality_score_distribution"
```

### 8.2 Operations Dashboard

```yaml
operations_dashboard:
  refresh_interval: 30s

  sections:
    - title: "Real-time Metrics"
      panels:
        - type: "gauge"
          metric: "requests_per_second"
          thresholds: [100, 500, 1000]

        - type: "gauge"
          metric: "current_latency_p99"
          thresholds: [200, 500, 1000]

    - title: "Service Health"
      panels:
        - type: "status_map"
          services: ["api", "asr", "translation", "renderer"]

        - type: "time_series"
          metric: "error_rate_by_service"
          period: "1h"

    - title: "Resource Utilization"
      panels:
        - type: "multi_gauge"
          metrics: ["cpu", "memory", "gpu"]

        - type: "table"
          metric: "instance_health"
```

### 8.3 Quality Dashboard

```yaml
quality_dashboard:
  refresh_interval: 1h

  sections:
    - title: "Translation Quality"
      panels:
        - type: "time_series"
          metric: "quality_score_trend"
          breakdown: "by_language"

        - type: "bar_chart"
          metric: "low_confidence_translations"
          breakdown: "by_reason"

    - title: "ASR Performance"
      panels:
        - type: "time_series"
          metric: "word_error_rate"
          breakdown: "by_language"

        - type: "histogram"
          metric: "audio_duration_distribution"

    - title: "User Experience"
      panels:
        - type: "funnel"
          stages: ["request", "asr", "translation", "render", "complete"]

        - type: "bar_chart"
          metric: "playback_speed_usage"
```

## 9. Export Formats

### 9.1 Supported Formats

| Format | Use Case | Structure |
|--------|----------|-----------|
| JSON | API responses, integrations | Hierarchical |
| CSV | Spreadsheets, simple analysis | Flat, tabular |
| Parquet | Big data, columnar storage | Columnar |
| Arrow | In-memory analytics | Columnar |

### 9.2 Export API

```yaml
export_api:
  endpoint: /api/v1/analytics/export

  parameters:
    metrics:
      type: array
      required: true
      description: "Metrics to export"

    start_time:
      type: datetime
      required: true

    end_time:
      type: datetime
      required: true

    granularity:
      type: enum
      values: [minute, hour, day, week, month]
      default: hour

    format:
      type: enum
      values: [json, csv, parquet]
      default: json

    filters:
      type: object
      description: "Optional filters"

  response:
    sync:
      max_rows: 10000
      timeout: 30s

    async:
      notification_webhook: optional
      download_expiry: 24h
```

### 9.3 Export Schema

```json
{
  "metadata": {
    "export_id": "exp-123456",
    "generated_at": "2025-01-15T10:00:00Z",
    "query": {
      "metrics": ["translations", "quality_scores"],
      "start_time": "2025-01-01T00:00:00Z",
      "end_time": "2025-01-15T00:00:00Z",
      "granularity": "day"
    },
    "row_count": 15,
    "format": "json"
  },
  "data": [
    {
      "timestamp": "2025-01-01T00:00:00Z",
      "translations_total": 10000,
      "translations_success": 9950,
      "quality_score_avg": 0.92,
      "quality_score_p50": 0.94,
      "quality_score_p95": 0.78
    }
  ]
}
```

## 10. Compliance

### 10.1 GDPR Compliance

```yaml
gdpr:
  data_subject_rights:
    access:
      endpoint: /api/v1/privacy/data-export
      response_time: 30d

    erasure:
      endpoint: /api/v1/privacy/delete
      response_time: 30d
      note: "Analytics data is anonymized, erasure applies to any linked identifiers"

    portability:
      format: json
      endpoint: /api/v1/privacy/data-export

  lawful_basis: legitimate_interest
  purpose: service_improvement

  dpia:
    required: true
    last_review: "2025-01-01"
```

### 10.2 PIPA (Korea) Compliance

```yaml
pipa:
  data_localization:
    korean_user_data: stored_in_korea

  consent:
    analytics_consent: optional
    withdrawal_mechanism: provided

  data_retention:
    personal_data: "Not collected in analytics"
    aggregated_data: "As per retention policy"
```

### 10.3 Audit Trail

```yaml
audit_trail:
  events:
    - analytics_query
    - data_export
    - configuration_change
    - consent_update

  fields:
    - timestamp
    - actor (anonymized)
    - action
    - resource
    - result

  retention: 7y
  immutable: true
```
